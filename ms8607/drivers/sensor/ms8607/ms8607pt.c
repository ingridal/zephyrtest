#define DT_DRV_COMPAT te_ms8607pt

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ms8607pt, CONFIG_MS8607_LOG_LEVEL);

#include "ms8607.h"

struct ms8607pt_data {
	const struct device *dev;
	uint16_t eeprom_coeff[COEFFICIENT_NUMBERS+1];
	bool coeff_is_ready;
	bool ready;
	enum ms8607_pressure_resolution resolution;
	float temperature;
	float pressure;
};

struct ms8607pt_config {
	struct i2c_dt_spec i2c;
};

static uint32_t psensor_conversion_time[6] = {	PSENSOR_CONVERSION_TIME_OSR_256,
												PSENSOR_CONVERSION_TIME_OSR_512,
												PSENSOR_CONVERSION_TIME_OSR_1024,
												PSENSOR_CONVERSION_TIME_OSR_2048,
												PSENSOR_CONVERSION_TIME_OSR_4096,
												PSENSOR_CONVERSION_TIME_OSR_8192
												};

static int read_register(const struct device *dev, uint8_t *val, uint8_t num_bytes)
{
	const struct ms8607pt_config *cfg = dev->config;
	int rc = i2c_read_dt(&cfg->i2c, val, num_bytes);
	return rc;
}

static int write_command(const struct device *dev, uint8_t command)
{
	const struct ms8607pt_config *cfg = dev->config;
	int rc = 0;
	uint8_t tx_buf[1] = {command};
	rc = i2c_write_dt(&cfg->i2c, tx_buf, sizeof(tx_buf));
	k_msleep(1);
	return rc;
}

static uint8_t ms8607pt_crc_check (uint16_t *n_prom)
{
	uint8_t cnt, n_bit;
	uint16_t n_rem, crc_read;
	
	n_rem = 0x00;
	crc_read = n_prom[0];
	n_prom[COEFFICIENT_NUMBERS] = 0;
	n_prom[0] = (0x0FFF & (n_prom[0]));    // Clear the CRC byte

	for( cnt = 0 ; cnt < (COEFFICIENT_NUMBERS+1)*2 ; cnt++ ) {

		// Get next byte
		if (cnt%2 == 1)
			n_rem ^=  n_prom[cnt>>1] & 0x00FF ;
		else
			n_rem ^=  n_prom[cnt>>1]>>8 ;

		for( n_bit = 8; n_bit > 0 ; n_bit-- ) {

			if( n_rem & 0x8000 )
				n_rem = (n_rem << 1) ^ 0x3000;
			else
				n_rem <<= 1;
		}
	}
	n_rem >>= 12;
	n_prom[0] = crc_read;
	
	return  (uint8_t)(n_rem);
}

static int ms8607pt_read_eeprom_coeff(const struct device *dev, uint8_t command, uint16_t *coeff)
{
	uint8_t buffer[2];
	int rc = 0;
	buffer[0] = 0;
	buffer[1] = 0;

	rc = write_command(dev, command);
	if (rc != 0) {
		LOG_ERR("Failed to call command 0x%02x - error %d", command, rc);
		return rc;
	}
	
	rc = read_register(dev, buffer, sizeof(buffer));
	if (rc != 0) {
		LOG_ERR("Failed to read buffer - error %d", rc);
		return rc;
	}

	*coeff = (buffer[0] << 8) | buffer[1];
	if (*coeff == 0) {
		return -EINVAL;
	}

	return 0;
}

static int ms8607pt_read_eeprom(const struct device *dev) 
{
	struct ms8607pt_data *data = dev->data;
	int rc = 0;
	for(int i = 0 ; i < COEFFICIENT_NUMBERS ;i++)
	{
		rc = ms8607pt_read_eeprom_coeff(dev, PROM_ADDRESS_READ_ADDRESS_0 + i * 2, data->eeprom_coeff + i);
		if(rc != 0)
		{
			return rc;
		}
	}

	uint8_t cal_crc = ms8607pt_crc_check(data->eeprom_coeff);
	uint8_t crc = (data->eeprom_coeff[CRC_INDEX] & 0xF000) >> 12;

	if (cal_crc != crc) {
		LOG_ERR("Invalid CRC 0x%2x - 0x%02x", cal_crc, crc);
		return -EINVAL;
	}
	data->coeff_is_ready = true;
	return 0;
}

static int ms8607pt_conversion_and_read_adc(const struct device *dev, uint8_t cmd, uint32_t *adc) 
{
	int rc = 0;
	uint8_t buffer[3];

	buffer[0] = 0;
	buffer[1] = 0;
	buffer[2] = 0;

	rc = write_command(dev, cmd);
	if (rc != 0) {
		LOG_ERR("Failed to call COMMAND 0x%02x - error %d", cmd, rc);
		return rc;
	}
	// 20ms wait for conversion
	k_msleep(psensor_conversion_time[(cmd & PSENSOR_CONVERSION_OSR_MASK)/2 ]/1000 );
	
	rc = write_command(dev, PSENSOR_READ_ADC);
	if (rc != 0) {
		LOG_ERR("Failed to call COMMAND 0x%02x - error %d", cmd, rc);
		return rc;
	}

	rc = read_register(dev, buffer, sizeof(buffer));
	if (rc != 0) {
		LOG_ERR("Failed to read buffer from READ PT %d", rc);
		return rc;
	}

	*adc = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];
	return 0;
}

static int ms8607pt_sample_fetch(const struct device *dev,
				      enum sensor_channel chan)
{
	struct ms8607pt_data *data = dev->data;

	if (chan != SENSOR_CHAN_ALL) {
		return -ENOTSUP;
	}

	uint32_t adc_temperature = 0, adc_pressure = 0;
	int32_t dT, TEMP;
	int64_t OFF, SENS, P, T2, OFF2, SENS2;
	uint8_t cmd;

	int rc = 0;
	if (data->coeff_is_ready == false) {
		rc = ms8607pt_read_eeprom(dev);
		if (rc != 0) {
			LOG_ERR("Failed to read eeprom");
			return rc;
		}
	}

	// First read temperature
	cmd = data->resolution * 2;
	cmd |= PSENSOR_START_TEMPERATURE_ADC_CONVERSION;
	rc = ms8607pt_conversion_and_read_adc(dev, cmd, &adc_temperature);
	if (rc != 0) {
		LOG_ERR("Failed to read temperature error %d", rc);
		return rc;
	}

	// Now read pressure
	cmd = data->resolution * 2;
	cmd |= PSENSOR_START_PRESSURE_ADC_CONVERSION;
	rc = ms8607pt_conversion_and_read_adc(dev, cmd, &adc_temperature);
	if (rc != 0) {
		LOG_ERR("Failed to read pressure error %d", rc);
		return rc;
	}

	if (adc_temperature == 0 || adc_pressure == 0) {
		LOG_ERR("Invalid value from ADC of temperature and pressure");
		return -EINVAL;
	}

	// Difference between actual and reference temperature = D2 - Tref
	dT = (int32_t)adc_temperature - ( (int32_t)data->eeprom_coeff[REFERENCE_TEMPERATURE_INDEX] <<8 );
	
	// Actual temperature = 2000 + dT * TEMPSENS
	TEMP = 2000 + ((int64_t)dT * (int64_t)data->eeprom_coeff[TEMP_COEFF_OF_TEMPERATURE_INDEX] >> 23) ;

	// Second order temperature compensation
	if( TEMP < 2000 )
	{
		T2 = ( 3 * ( (int64_t)dT  * (int64_t)dT  ) ) >> 33;
		OFF2 = 61 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000) / 16 ;
		SENS2 = 29 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000) / 16 ;
		
		if( TEMP < -1500 )
		{
			OFF2 += 17 * ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500) ;
			SENS2 += 9 * ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500) ;
		}
	}
	else
	{
		T2 = ( 5 * ( (int64_t)dT  * (int64_t)dT  ) ) >> 38;
		OFF2 = 0 ;
		SENS2 = 0 ;
	}
	
	// OFF = OFF_T1 + TCO * dT
	OFF = ( (int64_t)(data->eeprom_coeff[PRESSURE_OFFSET_INDEX]) << 17 ) + ( ( (int64_t)(data->eeprom_coeff[TEMP_COEFF_OF_PRESSURE_OFFSET_INDEX]) * dT ) >> 6 ) ;
	OFF -= OFF2 ;
	
	// Sensitivity at actual temperature = SENS_T1 + TCS * dT
	SENS = ( (int64_t)data->eeprom_coeff[PRESSURE_SENSITIVITY_INDEX] << 16 ) + ( ((int64_t)data->eeprom_coeff[TEMP_COEFF_OF_PRESSURE_SENSITIVITY_INDEX] * dT) >> 7 ) ;
	SENS -= SENS2 ;
	
	// Temperature compensated pressure = D1 * SENS - OFF
	P = ( ( (adc_pressure * SENS) >> 21 ) - OFF ) >> 15 ;
	data->temperature = ( (float)TEMP - T2 ) / 100;
	data->pressure  = (float)P / 100;
	data->ready = true;
	return 0;
}

static int ms8607pt_channel_get(const struct device *dev,
				     enum sensor_channel chan,
				     struct sensor_value *val)
{
	struct ms8607pt_data *data = dev->data;
	if (chan != SENSOR_CHAN_PRESS || chan != SENSOR_CHAN_AMBIENT_TEMP) {
		return -ENOTSUP;
	}

	switch (chan) {
		case SENSOR_CHAN_AMBIENT_TEMP:
			val->val1 = (int32_t)data->temperature;
			val->val2 = (int32_t)((data->temperature - (float)val->val1) * 1000000.0);
			break;
		case SENSOR_CHAN_PRESS:
			val->val1 = (int32_t)data->pressure;
			val->val2 = (int32_t)((data->pressure - (float)val->val1) * 1000000.0);
			break;
		default:
			return -EINVAL;
	}
	
	return 0;
}

static int ms8607pt_attr_set(const struct device *dev, enum sensor_channel chan,
			   enum sensor_attribute attr,
			   const struct sensor_value *val)
{
	struct ms8607pt_data *data = dev->data;
	if (attr == SENSOR_ATTR_FULL_SCALE && chan == SENSOR_CHAN_PRESS) {
		data->resolution = (enum ms8607_pressure_resolution)val->val1;
		return 0;
	}

	return -EINVAL;
}

static const struct sensor_driver_api ms8607pt_api = {
	.sample_fetch = ms8607pt_sample_fetch,
	.channel_get = ms8607pt_channel_get,
	.attr_set = ms8607pt_attr_set,
};

static int ms8607pt_init(const struct device *dev)
{
	struct ms8607pt_data *data = dev->data;
	const struct ms8607pt_config *cfg = dev->config;
	int rc = 0;
	if (!i2c_is_ready_dt(&cfg->i2c)) {
		LOG_ERR("I2C bus device not ready");
		return -ENODEV;
	}
	k_msleep(5);
	rc = write_command(dev, PSENSOR_RESET_COMMAND);
	if (rc != 0) {
		LOG_ERR("Failed to call RESET command %d", rc);
		return rc;
	}
	k_msleep(5);
	data->coeff_is_ready = false;
	data->pressure = 0;
	data->temperature = 0;
	data->ready = false;
	data->resolution = ms8607_pressure_resolution_osr_8192;

	rc = ms8607pt_read_eeprom(dev);
	if (rc != 0) {
		LOG_ERR("Failed to erad EEPROM %d", rc);
		return rc;
	}

	return 0;
}


static struct ms8607pt_data ms8607pt_data;
static const struct ms8607pt_config ms8607pt_config = { 
	.i2c = I2C_DT_SPEC_INST_GET(0),	
};

// Device definition
DEVICE_DT_INST_DEFINE(0, ms8607pt_init, NULL, &ms8607pt_data,
		&ms8607pt_config, POST_KERNEL,
		CONFIG_SENSOR_INIT_PRIORITY, &ms8607pt_api);