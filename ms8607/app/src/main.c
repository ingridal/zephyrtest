#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>

#include "app_version.h"
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

#include <sensor/ms8607.h>

void main(void)
{
	int ret;
	const struct device *ms8607_pt_dev;
	const struct device *ms8607_h_dev;
	struct sensor_value humidity, pressure, temperature;
	enum ms8607_battery_status bat_status;
	LOG_INF("MS8607 PHT Sensor Example Application - Version %s", APP_VERSION_STR);

	ms8607_h_dev = DEVICE_DT_GET_ONE(te_ms8607h);
	if (!device_is_ready(ms8607_h_dev)) {
		LOG_ERR("Sensor %s not ready", ms8607_h_dev->name);
		return;
	}

	ms8607_pt_dev = DEVICE_DT_GET_ONE(te_ms8607pt);
	if (!device_is_ready(ms8607_pt_dev)) {
		LOG_ERR("Sensor %s not ready", ms8607_pt_dev->name);
		return;
	}

	ret = ms8607_enable_heater(ms8607_h_dev);
	if (ret != 0) {
		LOG_ERR("Failed to enable heater");
		return;
	}

	k_sleep(K_SECONDS(1));

	ret = ms8607_disable_heater(ms8607_h_dev);
	if (ret != 0) {
		LOG_ERR("Failed to disable heater");
		return;
	}

	ret = ms8607_get_battery_status(ms8607_h_dev, &bat_status);
	if (ret != 0) {
		LOG_ERR("Failed to get battery status");
		return;
	}

	LOG_INF("Battery status %s", bat_status == ms8607_battery_ok ? "Okay" : "Not Okay");

	while (1) {
		ret = sensor_sample_fetch(ms8607_pt_dev);
		if (ret < 0) {
			LOG_ERR("Could not fetch sample (%d)", ret);
			return;
		}

		ret = sensor_sample_fetch(ms8607_h_dev);
		if (ret < 0) {
			LOG_ERR("Could not fetch sample (%d)", ret);
			return;
		}

		sensor_channel_get(ms8607_pt_dev, SENSOR_CHAN_AMBIENT_TEMP, &temperature);
		sensor_channel_get(ms8607_pt_dev, SENSOR_CHAN_PRESS, &pressure);
		sensor_channel_get(ms8607_h_dev, SENSOR_CHAN_HUMIDITY, &humidity);
		printf("MS8607: %.2f Temp ; %0.2f RH ; %0.2f Pressure \n",
		       sensor_value_to_double(&temperature),
		       sensor_value_to_double(&humidity),
		       sensor_value_to_double(&pressure));
		k_sleep(K_MSEC(1000));
	}
}

