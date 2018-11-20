/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>
#include <device.h>
#include <i2c.h>
#include <misc/util.h>

#define ADP5360_I2C_ADDR				0x46

/* Register addresses */
#define ADP5360_BUCK_OUTPUT				0x2A
#define ADP5360_BUCKBST_CFG				0x2B
#define ADP5360_BUCKBST_OUTPUT				0x2C
#define ADP5360_DEFAULT_SET_REG				0x37

/* Buck output voltage setting register. */
#define ADP5360_BUCK_OUTPUT_VOUT_BUCK_MSK		GENMASK(5, 0)
#define ADP5360_BUCK_OUTPUT_VOUT_BUCK(x)		(((x) & 0x3F) << 0)
#define ADP5360_BUCK_OUTPUT_BUCK_DLY_MSK		GENMASK(7, 6)
#define ADP5360_BUCK_OUTPUT_BUCK_DLY(x)			(((x) & 0x03) << 6)

/* Buck/boost output voltage setting register. */
#define ADP5360_BUCKBST_OUTPUT_VOUT_BUCKBST_MSK		GENMASK(5, 0)
#define ADP5360_BUCKBST_OUTPUT_VOUT_BUCKBST(x)		(((x) & 0x3F) << 0)
#define ADP5360_BUCKBST_OUT_BUCK_DLY_MSK		GENMASK(7, 6)
#define ADP5360_BUCKBST_OUT_BUCK_DLY(x)			(((x) & 0x03) << 6)

/* Buck/boost configure register. */
#define ADP5360_BUCKBST_CFG_EN_BUCKBST_MSK		BIT(0)
#define ADP5360_BUCKBST_CFG_EN_BUCKBST(x)		(((x) & 0x01) << 0)

/* DEFAULT_SET register. */
#define ADP5360_DEFAULT_SET_MSK				GENMASK(7, 0)
#define ADP5360_DEFAULT_SET(x)				(((x) & 0xFF) << 0)


static struct device *i2c_dev;

static int adp5360_reg_read(u8_t reg, u8_t *buff)
{
	return i2c_reg_read_byte(i2c_dev, ADP5360_I2C_ADDR, reg, buff);
}

static int adp5360_reg_write(u8_t reg, u8_t val)
{
	return i2c_reg_write_byte(i2c_dev, ADP5360_I2C_ADDR, reg, val);
}

static int adp5360_reg_write_mask(u8_t reg_addr,
			       u32_t mask,
			       u8_t data)
{
	int err;
	u8_t tmp;

	err = adp5360_reg_read(reg_addr, &tmp);
	if (err) {
		return err;
	}

	tmp &= ~mask;
	tmp |= data;

	return adp5360_reg_write(reg_addr, tmp);
}

int adp5360_buck_1v8_set(void)
{
	/* 1.8V equals to 0b11000 = 0x18 according to ADP5360 datasheet. */
	u8_t value = 0x18;

	return adp5360_reg_write_mask(ADP5360_BUCK_OUTPUT,
					ADP5360_BUCK_OUTPUT_VOUT_BUCK_MSK,
					ADP5360_BUCK_OUTPUT_VOUT_BUCK(value));
}

int adp5360_buckbst_3v3_set(void)
{
	/* 3.3V equals to 0b10011 = 0x13, according to ADP5360 datasheet. */
	u8_t value = 0x13;

	return adp5360_reg_write_mask(ADP5360_BUCKBST_OUTPUT,
				ADP5360_BUCKBST_OUTPUT_VOUT_BUCKBST_MSK,
				ADP5360_BUCKBST_OUTPUT_VOUT_BUCKBST(value));
}

int adp5360_buckbst_enable(bool enable)
{
	return adp5360_reg_write_mask(ADP5360_BUCKBST_CFG,
					ADP5360_BUCKBST_CFG_EN_BUCKBST_MSK,
					ADP5360_BUCKBST_CFG_EN_BUCKBST(enable));
}

static int adp5360_default_set(void)
{
	/* The value 0x7F has to be written to this register to accomplish
	 * factory reset of the I2C registers, according to ADP5360 datasheet.
	 */
	return adp5360_reg_write_mask(ADP5360_DEFAULT_SET_REG,
					ADP5360_DEFAULT_SET_MSK,
					ADP5360_DEFAULT_SET(0x7F));
}

int adp5360_factory_reset(void)
{
	int err;

	err = adp5360_default_set();
	if (err) {
		printk("adp5360_default_set failed: %d\n", err);
		return err;
	}

	return 0;
}

int adp5360_init(const char *dev_name)
{
	int err = 0;

	i2c_dev = device_get_binding(dev_name);
	if (err) {
		err = -ENODEV;
	}

	return err;
}
