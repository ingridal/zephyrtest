#define DT_DRV_COMPAT te_ms8607h

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ms8607h, CONFIG_MS8607_LOG_LEVEL);

#include <stdbool.h>
#include "ms8607.h"

struct ms8607h_data {
	enum ms8607_humidity_i2c_master_mode mode;
	uint32_t conversion_time;
	uint16_t humid;
	bool heater_on;
	bool ready;
};

struct ms8607h_config {
	struct i2c_dt_spec i2c;
};

static int read_data(const struct device *dev, uint8_t *val, uint8_t num_bytes)
{
	const struct ms8607h_config *cfg = dev->config;
	int rc = i2c_read_dt(&cfg->i2c, val, num_bytes);
	return rc;
}

static int read_register(const struct device *dev, uint8_t reg, uint8_t *buf)
{
	const struct ms8607h_config *cfg = dev->config;
	uint8_t tx_buf[1] = {reg};
	int rc = i2c_write_read_dt(&cfg->i2c, tx_buf, sizeof(tx_buf), buf, 1);
	return rc;
}

static int write_register(const struct device *dev, uint8_t value)
{
	const struct ms8607h_config *cfg = dev->config;
	int rc = 0;
	uint8_t rx_buf[1] = {0x0};
	uint8_t tx_buf[2] = {0x00};
	rc = read_register(dev, HSENSOR_READ_USER_REG_COMMAND, rx_buf);
	if (rc != 0) {
		return rc;
	}

	// Clear bits of reg that are not reserved
	rx_buf[0] &= HSENSOR_USER_REG_RESERVED_MASK;
	// Set bits from value that are not reserved
	rx_buf[0] |= (value & ~HSENSOR_USER_REG_RESERVED_MASK);
	
	tx_buf[0] = HSENSOR_WRITE_USER_REG_COMMAND;
	tx_buf[1] = rx_buf[0];
	rc = i2c_write_dt(&cfg->i2c, tx_buf, sizeof(tx_buf));
	return rc;
}

static int write_command(const struct device *dev, uint8_t command)
{
	const struct ms8607h_config *cfg = dev->config;
	int rc = 0;
	uint8_t tx_buf[1] = {command};
	rc = i2c_write_dt(&cfg->i2c, tx_buf, sizeof(tx_buf));
	return rc;
}

static int is_connected(const struct device* dev) {
	const struct ms8607h_config *cfg = dev->config;
	int rc = i2c_write_dt(&cfg->i2c, NULL, 0);
	return rc;
}

static uint8_t ms8607h_crc_check(uint16_t value) 
{
	uint32_t polynom = 0x988000; // x^8 + x^5 + x^4 + 1
	uint32_t msb     = 0x800000;
	uint32_t mask    = 0xFF8000;
	uint32_t result  = (uint32_t)value<<8; // Pad with zeros as specified in spec
	
	while( msb != 0x80 ) {
		
		// Check if msb of current value is 1 and apply XOR mask
		if( result & msb )
			result = ((result ^ polynom) & mask) | ( result & ~mask);
			
		// Shift by one
		msb >>= 1;
		mask >>= 1;
		polynom >>=1;
	}
	
	return (uint8_t)result;
}

static int ms8607h_sample_fetch(const struct device *dev,
				      enum sensor_channel chan)
{
	struct ms8607h_data *data = dev->data;

	if (chan != SENSOR_CHAN_ALL) {
		return -ENOTSUP;
	}

	int rc = 0;
	uint8_t buffer[3];
	uint8_t crc;
	uint16_t _adc;

	buffer[0] = 0;
	buffer[1] = 0;
	buffer[2] = 0;

	if( data->mode == ms8607_i2c_hold) 
	{
		rc = write_command(dev, HSENSOR_READ_HUMIDITY_W_HOLD_COMMAND);
	}
	else 
	{
		rc = write_command(dev, HSENSOR_READ_HUMIDITY_WO_HOLD_COMMAND);
		// delay depending on resolution
		k_msleep(data->conversion_time / 1000);
	}

	if (rc != 0) 
	{
		LOG_ERR("Failed to call READ HUMIDITY command %d", rc);
		return rc;
	}

	rc = read_data(dev, buffer, sizeof(buffer));
	if (rc != 0) {
		LOG_ERR("Failed to read buffer from READ HUMIDITY %d", rc);
		return rc;
	}

	_adc = (buffer[0] << 8) | buffer[1];
	crc = buffer[2];

	uint8_t cal_crc = ms8607h_crc_check(_adc);

	if (cal_crc != crc) {
		data->humid = _adc;
		data->ready= true;
		return 0;
	}

	LOG_ERR("CRC invalid 0x%02x - 0x%02x", cal_crc, crc);
	return -EINVAL;
}

static int ms8607h_channel_get(const struct device *dev,
				     enum sensor_channel chan,
				     struct sensor_value *val)
{
	struct ms8607h_data *data = dev->data;

	if (chan != SENSOR_CHAN_HUMIDITY) {
		return -ENOTSUP;
	}

	if (data->ready) {
		float humid = (float)(data->humid * HUMIDITY_COEFF_MUL) / (float)(1UL<<16) + (float)HUMIDITY_COEFF_ADD;
		val->val1 = (int32_t)humid;
		val->val2 = (humid - (float)val->val1) * 1000000;
		return 0;
	}

	return -EIO;
}


static int ms8607h_attr_set(const struct device *dev, enum sensor_channel chan,
			   enum sensor_attribute attr,
			   const struct sensor_value *val)
{
	uint8_t reg_value, tmp=0;
	uint32_t conversion_time = HSENSOR_CONVERSION_TIME_12b;
	struct ms8607h_data *data = dev->data;

	if (attr == SENSOR_ATTR_FULL_SCALE) {
		enum ms8607_humidity_resolution res = (enum ms8607_humidity_resolution)val->val1;
		if( res == ms8607_humidity_resolution_12b) {
			tmp = HSENSOR_USER_REG_RESOLUTION_12b;
			conversion_time = HSENSOR_CONVERSION_TIME_12b;
		}
		else if( res == ms8607_humidity_resolution_10b) {
			tmp = HSENSOR_USER_REG_RESOLUTION_10b;
			conversion_time = HSENSOR_CONVERSION_TIME_10b;
		}
		else if( res == ms8607_humidity_resolution_8b) {
			tmp = HSENSOR_USER_REG_RESOLUTION_8b;
			conversion_time = HSENSOR_CONVERSION_TIME_8b;
		}
		else if( res == ms8607_humidity_resolution_11b) {
			tmp = HSENSOR_USER_REG_RESOLUTION_11b;
			conversion_time = HSENSOR_CONVERSION_TIME_11b;
		}

		uint8_t rx_buf[1] = {0x00};
		int rc = read_register(dev, HSENSOR_READ_USER_REG_COMMAND, rx_buf);
		if (rc != 0) {
			LOG_ERR("Failed to read user data %d", rc);
			return -EIO;
		}

		reg_value = rx_buf[0];
		// Clear the resolution bits
		reg_value &= ~HSENSOR_USER_REG_RESOLUTION_MASK;
		reg_value |= tmp & HSENSOR_USER_REG_RESOLUTION_MASK;
		
		data->conversion_time = conversion_time;
		rc = write_register(dev, reg_value);
		if (rc != 0) {
			LOG_ERR("Failed to set attribute %d", rc);
			return -EIO;
		}
	}
	return 0;
}

static const struct sensor_driver_api ms8607h_api = {
	.sample_fetch = ms8607h_sample_fetch,
	.channel_get = ms8607h_channel_get,
	.attr_set = ms8607h_attr_set,
};

static int ms8607h_init(const struct device *dev)
{
	struct ms8607h_data *data = dev->data;
	const struct ms8607h_config *cfg = dev->config;
	int rc = 0;
	if (!i2c_is_ready_dt(&cfg->i2c)) {
		LOG_ERR("I2C bus device not ready");
		return -ENODEV;
	}

	rc = is_connected(dev);
	if (rc != 0) {
		LOG_ERR("The humidity is not ready");
		return rc;
	}

	rc = write_command(dev, HSENSOR_RESET_COMMAND);
	if (rc != 0) {
		LOG_ERR("Failed to call RESET command %d", rc);
		return rc;
	}
	k_msleep(HSENSOR_RESET_TIME);
	data->conversion_time = HSENSOR_CONVERSION_TIME_12b;
	data->mode = ms8607_i2c_no_hold;
	data->ready = false;
	data->heater_on = false;
	return 0;
}

int ms8607_enable_heater(const struct device *dev) {
	struct ms8607h_data *data = dev->data;
	uint8_t rx_buf[1] = {0x00};
	int rc = read_register(dev, HSENSOR_READ_USER_REG_COMMAND, rx_buf);
	if (rc != 0) {
		LOG_ERR("Failed to read user data %d", rc);
		return -EIO;
	}

	// Clear the resolution bits
	rx_buf[0] |= HSENSOR_USER_REG_ONCHIP_HEATER_ENABLE;
	data->heater_on = true;
	
	rc = write_register(dev, rx_buf[0]);
	return rc;
}

int ms8607_disable_heater(const struct device *dev) {
	struct ms8607h_data *data = dev->data;
	uint8_t rx_buf[1] = {0x00};
	int rc = read_register(dev, HSENSOR_READ_USER_REG_COMMAND, rx_buf);
	if (rc != 0) {
		LOG_ERR("Failed to read user data %d", rc);
		return -EIO;
	}

	// Clear the resolution bits
	rx_buf[0] &= ~HSENSOR_USER_REG_ONCHIP_HEATER_ENABLE;
	data->heater_on = true;
	
	rc = write_register(dev, rx_buf[0]);
	return rc;
}

int ms8607_get_battery_status(const struct device *dev, enum ms8607_battery_status *status) {
	uint8_t rx_buf[1] = {0x00};
	int rc = read_register(dev, HSENSOR_READ_USER_REG_COMMAND, rx_buf);
	if (rc != 0) {
		LOG_ERR("Failed to read user data %d", rc);
		return -EIO;
	}

	if( rx_buf[0] & HSENSOR_USER_REG_END_OF_BATTERY_VDD_BELOW_2_25V )
	{
		*status = ms8607_battery_low;
	}	
	else
	{
		*status = ms8607_battery_ok;
	}
	
	return 0;
}

static struct ms8607h_data ms8607h_data;
static const struct ms8607h_config ms8607h_config = { 
	.i2c = I2C_DT_SPEC_INST_GET(0),	
};

// Device definition
DEVICE_DT_INST_DEFINE(0, ms8607h_init, NULL, &ms8607h_data,
		&ms8607h_config, POST_KERNEL,
		CONFIG_SENSOR_INIT_PRIORITY, &ms8607h_api);