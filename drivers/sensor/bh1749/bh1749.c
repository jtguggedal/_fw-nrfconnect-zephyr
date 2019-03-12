/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * This driver is heavily inspired by the apds9960.c driver.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <sensor.h>
#include <i2c.h>
#include <misc/__assert.h>
#include <misc/byteorder.h>
#include <init.h>
#include <kernel.h>
#include <string.h>
#include <logging/log.h>

#include "bh1749.h"

LOG_MODULE_REGISTER(BH1749, CONFIG_SENSOR_LOG_LEVEL);

static void bh1749_gpio_callback(struct device *dev,
				  struct gpio_callback *cb, u32_t pins)
{
	struct bh1749_data *drv_data =
		CONTAINER_OF(cb, struct bh1749_data, gpio_cb);
	printk("Interrupt\n");

	gpio_pin_disable_callback(dev, DT_ROHM_BH1749_0_INT_GPIOS_PIN);

#ifdef CONFIG_BH1749_TRIGGER
	k_work_submit(&drv_data->work);
#else
	k_sem_give(&drv_data->data_sem);
#endif
}

static int bh1749_sample_fetch(struct device *dev, enum sensor_channel chan)
{
	struct bh1749_data *data = dev->driver_data;
	u8_t status;

	if (chan != SENSOR_CHAN_ALL) {
		LOG_ERR("Unsupported sensor channel");
		return -ENOTSUP;
	}

	printk("Fetching...\n");

#ifndef CONFIG_BH1749_TRIGGER
	gpio_pin_enable_callback(data->gpio,
				 DT_ROHM_BH1749_0_INT_GPIOS_PIN);

	if (i2c_reg_update_byte(data->i2c, DT_ROHM_BH1749_0_BASE_ADDRESS,
				BH1749_INTERRUPT,
				BH1749_INTERRUPT_ENABLE_Msk,
				BH1749_INTERRUPT_ENABLE_ENABLE)) {
		LOG_ERR("Could not enable RGB sampling.");
		return -EIO;
	}

	u8_t buf;
	if (i2c_reg_read_byte(data->i2c, DT_ROHM_BH1749_0_BASE_ADDRESS,
				BH1749_INTERRUPT,
				&buf)) {
		LOG_ERR("Could not enable RGB sampling.");
		return -EIO;
	}

	printk("BH1749_INTERRUPT: 0x%02x\n", buf);

	//k_sem_take(&data->data_sem, K_FOREVER);
#endif

	if (i2c_reg_read_byte(data->i2c, DT_ROHM_BH1749_0_BASE_ADDRESS,
			      BH1749_MODE_CONTROL2, &status)) {
		return -EIO;
	}

	printk("\n\nBH1749_MODE_CONTROL2: 0x%02x\n", status);
	// if ((status & BH1749_MODE_CONTROL2_VALID_Msk) == 0) {
	// 	LOG_ERR("No valid data to fetch.");
	// 	return -EIO;
	// }

	i2c_burst_read(data->i2c, DT_ROHM_BH1749_0_BASE_ADDRESS,
		BH1749_RED_DATA_LSB,
		(u8_t *)data->sample_rgb_ir, 8);

#ifndef CONFIG_BH1749_TRIGGER
	if (i2c_reg_update_byte(data->i2c, DT_ROHM_BH1749_0_BASE_ADDRESS,
				BH1749_INTERRUPT,
				BH1749_INTERRUPT_ENABLE_Msk,
				BH1749_INTERRUPT_ENABLE_ENABLE)) {
		return -EIO;
	}
#endif

	return 0;
}

static int bh1749_channel_get(struct device *dev,
				enum sensor_channel chan,
				struct sensor_value *val)
{
	struct bh1749_data *data = dev->driver_data;

	switch (chan) {
	case SENSOR_CHAN_RED:
		val->val1 = sys_le16_to_cpu(data->sample_rgb_ir[0]);
		val->val2 = 0;
		break;
	case SENSOR_CHAN_GREEN:
		val->val1 = sys_le16_to_cpu(data->sample_rgb_ir[1]);
		val->val2 = 0;
		break;
	case SENSOR_CHAN_BLUE:
		val->val1 = sys_le16_to_cpu(data->sample_rgb_ir[2]);
		val->val2 = 0;
		break;
	case SENSOR_CHAN_IR:
		val->val1 = sys_le16_to_cpu(data->sample_rgb_ir[3]);
		val->val2 = 0;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int bh1749_check(struct device *dev)
{
	int err;
	struct bh1749_data *data = dev->driver_data;
	u8_t buf;

	err = i2c_reg_read_byte(data->i2c, DT_ROHM_BH1749_0_BASE_ADDRESS,
				BH1749_MANUFACTURER_ID, &buf);
	if (err) {
		LOG_ERR("Failed when reading manufacturer ID");
		return -EIO;
	}

	LOG_DBG("Manufacturer ID: 0x%02x", buf);

	if (buf != BH1749_MANUFACTURER_ID_DEFAULT) {
		LOG_ERR("Invalid manufacturer ID: 0x%02x", buf);
		return -EIO;
	}

	err = i2c_reg_read_byte(data->i2c, DT_ROHM_BH1749_0_BASE_ADDRESS,
				BH1749_SYSTEM_CONTROL, &buf);
	if (err) {
		LOG_ERR("Failed when reading manufacturer ID");
		return -EIO;
	}

	if ((buf & BH1749_SYSTEM_CONTROL_PART_ID_Msk) !=
	     BH1749_SYSTEM_CONTROL_PART_ID) {
		LOG_ERR("Invalid part ID: 0x%02x", buf);
		return -EIO;
	}

	LOG_DBG("Part ID: 0x%02x", buf);
	k_sleep(100);

	return 0;
}

static int bh1749_interrupt_init(struct device *dev)
{
	int err;
	struct bh1749_data *drv_data = dev->driver_data;

	/* Setup gpio interrupt */
	drv_data->gpio =
		device_get_binding(DT_ROHM_BH1749_0_INT_GPIOS_CONTROLLER);
	if (drv_data->gpio == NULL) {
		LOG_ERR("Failed to get binding to %s device!",
			    DT_ROHM_BH1749_0_INT_GPIOS_CONTROLLER);
		return -EINVAL;
	}

	err = gpio_pin_configure(drv_data->gpio, DT_ROHM_BH1749_0_INT_GPIOS_PIN,
			   GPIO_DIR_IN | GPIO_INT | GPIO_INT_LEVEL |
			   GPIO_INT_ACTIVE_LOW  | GPIO_PUD_PULL_UP);
	if (err) {
		LOG_DBG("Failed to configure interrupt GPIO");
		return -EIO;
	}

	gpio_init_callback(&drv_data->gpio_cb,
			   bh1749_gpio_callback,
			   BIT(DT_ROHM_BH1749_0_INT_GPIOS_PIN));

	err = gpio_add_callback(drv_data->gpio, &drv_data->gpio_cb);
	if (err) {
		LOG_DBG("Failed to set GPIO callback");
		return err;
	}

	err = i2c_reg_update_byte(drv_data->i2c, DT_ROHM_BH1749_0_BASE_ADDRESS,
				  BH1749_INTERRUPT,
				  BH1749_INTERRUPT_LATCH,
				  BH1749_INTERRUPT_LATCH);
	if (err) {
		LOG_ERR("Interrupts could not be enabled.");
		return -EIO;
	}

#ifdef CONFIG_BH1749_TRIGGER
	drv_data->work.handler = bh1749_work_fn;
	drv_data->dev = dev;

	err = i2c_reg_update_byte(drv_data->i2c, DT_ROHM_BH1749_0_BASE_ADDRESS,
				BH1749_INTERRUPT,
				BH1749_INTERRUPT_ENABLE_Msk |
				BH1749_INTERRUPT_INT_SOURCE_Msk,
				BH1749_INTERRUPT_ENABLE_ENABLE |
				BH1749_INTERRUPT_INT_SOURCE_RED);
	if (err) {
		LOG_ERR("Interrupts could not be enabled.");
		return -EIO;
	}
#else
	k_sem_init(&drv_data->data_sem, 0, UINT_MAX);
#endif
	return 0;
}

#ifdef CONFIG_DEVICE_POWER_MANAGEMENT
static int bh1749_power_control(struct device *dev, u32_t ctrl_command,
				void *context)
{
	struct bh1749_data *data = dev->driver_data;

	if (ctrl_command == DEVICE_PM_SET_POWER_STATE) {
		u32_t device_pm_state = *(u32_t *)context;

		if (device_pm_state == DEVICE_PM_ACTIVE_STATE) {
			if (i2c_reg_update_byte(data->i2c, DT_ROHM_BH1749_0_BASE_ADDRESS,
					BH1749_MODE_CONTROL2,
					BH1749_MODE_CONTROL2_RGB_EN_Msk,
					BH1749_MODE_CONTROL2_RGB_EN_ENABLE)) {
				return -EIO;
			}

			return 0;
		} else if(device_pm_state == DEVICE_PM_LOW_POWER_STATE) {
			if (i2c_reg_update_byte(data->i2c, DT_ROHM_BH1749_0_BASE_ADDRESS,
					BH1749_MODE_CONTROL2,
					BH1749_MODE_CONTROL2_RGB_EN_Msk,
					BH1749_MODE_CONTROL2_RGB_EN_DISABLE)) {
				return -EIO;
			}

			return 0;
		}
	} else if (ctrl_command == DEVICE_PM_GET_POWER_STATE) {
		*((u32_t *)context) = DEVICE_PM_ACTIVE_STATE;
	}

	return 0;
}
#endif

static int bh1749_sw_reset(struct device *dev)
{
	return i2c_reg_update_byte(dev, DT_ROHM_BH1749_0_BASE_ADDRESS,
				   BH1749_SYSTEM_CONTROL,
				   BH1749_SYSTEM_CONTROL_SW_RESET_Msk,
				   BH1749_SYSTEM_CONTROL_SW_RESET);
}

static int bh1749_rgb_gain_set(struct device *dev, u8_t value)
{
	return i2c_reg_update_byte(dev, DT_ROHM_BH1749_0_BASE_ADDRESS,
				   BH1749_MODE_CONTROL1,
				   BH1749_MODE_CONTROL1_RGB_GAIN_Msk,
				   value);
}

static int bh1749_rgb_measurement_enable(struct device *dev) {
	return i2c_reg_update_byte(dev, DT_ROHM_BH1749_0_BASE_ADDRESS,
				BH1749_MODE_CONTROL2,
				BH1749_MODE_CONTROL2_RGB_EN_Msk,
				BH1749_MODE_CONTROL2_RGB_EN_ENABLE);
}

static int bh1749_init(struct device *dev)
{
	int err;
	struct bh1749_data *data = dev->driver_data;

	/* Activation time 2ms */
	k_sleep(2);

	data->i2c = device_get_binding(DT_ROHM_BH1749_0_BUS_NAME);

	if (data->i2c == NULL) {
		LOG_ERR("Failed to get pointer to %s device!",
			DT_ROHM_BH1749_0_BUS_NAME);
		return -EINVAL;
	}

	(void)memset(data->sample_rgb_ir, 0, sizeof(data->sample_rgb_ir));
	data->pdata = 0U;

	err = bh1749_sw_reset(data->i2c);
	if (err) {
		LOG_ERR("Could not apply software reset.");
		return -EIO;
	}

	err = bh1749_check(dev);
	if (err) {
		LOG_ERR("Communication with BH1749 failed with error %d", err);
		return -EIO;
	}

	err = bh1749_interrupt_init(dev);
	if (err) {
		LOG_ERR("Failed to initialize interrupt with error %d", err);
		return -EIO;
	}

	LOG_DBG("Interrupts initialized");

	if (i2c_reg_update_byte(data->i2c, DT_ROHM_BH1749_0_BASE_ADDRESS,
				BH1749_PERSISTENCE,
				BH1749_PERSISTENCE_PERSISTENCE_Msk,
				BH1749_PERSISTENCE_PERSISTENCE_ACTIVE_END)) {
		LOG_ERR("Could not set persistence configuration.");
		return -EIO;
	}

	if (i2c_reg_update_byte(data->i2c, DT_ROHM_BH1749_0_BASE_ADDRESS,
				BH1749_MODE_CONTROL1,
				BH1749_MODE_CONTROL1_MEAS_MODE_Msk,
				0x00)) {
		LOG_ERR("Could not set persistence configuration.");
		return -EIO;
	}

	err = bh1749_rgb_measurement_enable(data->i2c);
	if (err) {
		LOG_ERR("Could not set measurement mode.");
		return -EIO;
	}

	LOG_DBG("Persistence set");

	return 0;
}

static const struct sensor_driver_api bh1749_driver_api = {
	.sample_fetch = &bh1749_sample_fetch,
	.channel_get = &bh1749_channel_get,
#ifdef CONFIG_BH1749_TRIGGER
	.attr_set = bh1749_attr_set,
	.trigger_set = bh1749_trigger_set,
#endif
};

static struct bh1749_data bh1749_data;

#ifndef CONFIG_DEVICE_POWER_MANAGEMENT
DEVICE_AND_API_INIT(bh1749, DT_ROHM_BH1749_0_LABEL, &bh1749_init,
		    &bh1749_data, NULL, POST_KERNEL,
		    CONFIG_SENSOR_INIT_PRIORITY, &bh1749_driver_api);
#else
DEVICE_DEFINE(bh1749, DT_ROHM_BH1749_0_LABEL, bh1749_init,
	      bh1749_power_control, &bh1749_data, NULL,
	      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &bh1749_driver_api);
#endif
