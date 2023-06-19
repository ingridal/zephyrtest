#ifndef MS8607_SENSOR_H_
#define MS8607_SENSOR_H_

#include <zephyr/device.h>

enum ms8607_battery_status {
	ms8607_battery_ok,
	ms8607_battery_low
};

/** @brief Enable heater
 *
 * @retval return 0 on success, or a negative error code from an I2C
 * transaction or invalid parameter.
 */
int ms8607_enable_heater(const struct device *dev);

/** @brief Disable heater
 *
 * @retval return 0 on success, or a negative error code from an I2C
 * transaction or invalid parameter.
 */
int ms8607_disable_heater(const struct device *dev);

/** @brief Get the battery status
 *
 * @retval return 0 on success, or a negative error code from an I2C
 * transaction or invalid parameter.
 */
int ms8607_get_battery_status(const struct device *dev, enum ms8607_battery_status *status);

#endif /* MS8607_SENSOR_H_ */
