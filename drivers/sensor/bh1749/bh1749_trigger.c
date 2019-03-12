/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * This driver is heavily inspired by the apds9960_trigger.c driver.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <gpio.h>
#include <i2c.h>
#include <misc/util.h>
#include <kernel.h>
#include <sensor.h>
#include "apds9960.h"

extern struct bh1749_data bh1749_driver;

#include <logging/log.h>
LOG_MODULE_REGISTER(BH1749, CONFIG_SENSOR_LOG_LEVEL);

void bh1749_work_fn(struct k_work *work)
{
	struct bh1749_data *data = CONTAINER_OF(work,
						struct bh1749_data,
						work);
	struct device *dev = data->dev;

	if (data->p_th_handler != NULL) {
		data->p_th_handler(dev, &data->p_th_trigger);
	}

	gpio_pin_enable_callback(data->gpio, DT_ROHM_BH1749_0_INT_GPIOS_PIN);
}

int bh1749_attr_set(struct device *dev,
		      enum sensor_channel chan,
		      enum sensor_attribute attr,
		      const struct sensor_value *val)
{
	int err;
	struct bh1749_data *data = dev->driver_data;

	if (chan != SENSOR_CHAN_ALL) {
		return -ENOTSUP;
	}

	if (attr == SENSOR_ATTR_UPPER_THRESH) {
		err = i2c_reg_write_byte(data->i2c,
					BH1749_I2C_ADDRESS,
					BH1749_TH_HIGH_LSB,
					(u8_t)val->val1);
		if (err) {
			LOG_ERR("Could not set threshold, error: %d", err);
			return err;
		}

		err = i2c_reg_write_byte(data->i2c,
					BH1749_I2C_ADDRESS,
					BH1749_TH_HIGH_MSB,
					(u8_t)(val->val1 >> 8));
		if (err) {
			LOG_ERR("Could not set threshold, error: %d", err);
			return err;
		}

		return 0;
	}

	if (attr == SENSOR_ATTR_LOWER_THRESH) {
		err = i2c_reg_write_byte(data->i2c,
					BH1749_I2C_ADDRESS,
					BH1749_TH_LOW_LSB,
					(u8_t)val->val1);
		if (err) {
			LOG_ERR("Could not set threshold, error: %d", err);
			return err;
		}

		err = i2c_reg_write_byte(data->i2c,
					BH1749_I2C_ADDRESS,
					BH1749_TH_LOW_MSB,
					(u8_t)(val->val1 >> 8));
		if (err) {
			LOG_ERR("Could not set threshold, error: %d", err);
			return err;
		}

		return 0;
	}
}

int bh1749_trigger_set(struct device *dev,
			const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler)
{
	struct bh1749_data *data = dev->driver_data;

	gpio_pin_disable_callback(data->gpio, DT_ROHM_BH1749_0_INT_GPIOS_PIN);

	switch (trig->type) {
	case SENSOR_TRIG_THRESHOLD:
		if (trig->chan == SENSOR_CHAN_RED) {
			data->p_th_handler = handler;
			if (i2c_reg_update_byte(data->i2c,
					BH1749_I2C_ADDRESS,
					BH1749_INTERRUPT,
					BH1749_INTERRUPT_INT_SOURCE_Msk,
					BH1749_INTERRUPT_INT_SOURCE_RED)) {
				return -EIO;
			}
		} else if (trig->chan == SENSOR_CHAN_GREEN) {
			data->p_th_handler = handler;
			if (i2c_reg_update_byte(data->i2c,
					BH1749_I2C_ADDRESS,
					BH1749_INTERRUPT,
					BH1749_INTERRUPT_INT_SOURCE_Msk,
					BH1749_INTERRUPT_INT_SOURCE_GREEN)) {
				return -EIO;
			}
		} else if (trig->chan == SENSOR_CHAN_BLUE) {
			data->p_th_handler = handler;
			if (i2c_reg_update_byte(data->i2c,
					BH1749_I2C_ADDRESS,
					BH1749_INTERRUPT,
					BH1749_INTERRUPT_INT_SOURCE_Msk,
					BH1749_INTERRUPT_INT_SOURCE_BLUE)) {
				return -EIO;
			}
		} else {
			return -ENOTSUP;
		}

		if (i2c_reg_update_byte(data->i2c,
				BH1749_I2C_ADDRESS,
				BH1749_PERSISTENCE,
				BH1749_PERSISTENCE_PERSISTENCE_Msk,
				BH1749_PERSISTENCE_PERSISTENCE_UPDATE_END)) {
			return -EIO;
		}
		break;
	case SENSOR_TRIG_DATA_READY:
		if (i2c_reg_update_byte(data->i2c,
				BH1749_I2C_ADDRESS,
				BH1749_PERSISTENCE,
				BH1749_PERSISTENCE_PERSISTENCE_Msk,
				BH1749_PERSISTENCE_PERSISTENCE_ACTIVE_END)) {
			return -EIO;
		}

		break;
	default:
		LOG_ERR("Unsupported sensor trigger");
		return -ENOTSUP;
	}

	gpio_pin_enable_callback(data->gpio, DT_ROHM_BH1749_0_INT_GPIOS_PIN);

	return 0;
}
