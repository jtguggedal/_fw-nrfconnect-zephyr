/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <hal/nrf_clock.h>
#include <nrfx.h>
#include <gpio.h>
#include <net/socket.h>

#define POWER_CTRL_1V8 3
#define POWER_CTRL_3V3 28

#define LC_MAX_READ_LENGTH 128

static struct device *gpio_dev;

int pca20035_power_1v8_set(bool enable)
{
	u32_t value;

	if (!gpio_dev) {
		return -ENODEV;
	}

	if (enable) {
		value = 1;
	} else {
		value = 0;
	}

	return gpio_pin_write(gpio_dev, POWER_CTRL_1V8, value);
}

int pca20035_power_3v3_set(bool enable)
{
	u32_t value;

	if (!gpio_dev) {
		return -ENODEV;
	}

	/* TODO: Add needed ADP5360 control */

	if (enable) {
		value = 1;
	} else {
		value = 0;
	}

	return gpio_pin_write(gpio_dev, POWER_CTRL_3V3, value);
}

static int pca20035_switch_to_xtal(void)
{
	nrf_clock_task_trigger(NRF_CLOCK_TASK_HFCLKSTART);
	return 0;
}

static int pca20035_magpio_configure(void)
{
	static const char magpio[] = "AT%XMAGPIO=1,1,1,450,451,698,747,747,803,803,849,849,894,894,960,1710,2200,1574,1577";
	int at_socket_fd;
	int buffer;
	u8_t read_buffer[LC_MAX_READ_LENGTH];

	at_socket_fd = socket(AF_LTE, 0, NPROTO_AT);
	if (at_socket_fd == -1) {
		return -EFAULT;
	}
	buffer = send(at_socket_fd, magpio, strlen(magpio), 0);
	if (buffer != strlen(magpio)) {
		close(at_socket_fd);
		return -EIO;
	}

	buffer = recv(at_socket_fd, read_buffer, LC_MAX_READ_LENGTH, 0);
	if ((buffer < 2) ||
	    (memcmp("OK", read_buffer, 2 != 0))) {
		close(at_socket_fd);
		return -EIO;
	}

	close(at_socket_fd);
}

static int pca20035_pins_init(void)
{
	int err;

	gpio_dev = device_get_binding(CONFIG_GPIO_P0_DEV_NAME);
	if (!gpio_dev) {
		return -ENODEV;
	}

	err = gpio_pin_configure(gpio_dev, POWER_CTRL_1V8, GPIO_DIR_OUT);
	if (err) {
		return err;
	}

	err = gpio_pin_configure(gpio_dev, POWER_CTRL_3V3, GPIO_DIR_OUT);
	if (err) {
		return err;
	}

	return 0;
}

static int pca20035_board_init(struct device *dev)
{
	int err;

	err = pca20035_switch_to_xtal();
	if (err != 0) {
		return err;
	}

	err = pca20035_pins_init();
	if (err != 0) {
		return err;
	}

	err = pca20035_power_3v3_set(false);
	if (err != 0) {
		return err;
	}

	err = pca20035_power_1v8_set(true);
	if (err != 0) {
		return err;
	}

	err = pca20035_magpio_configure();
	if (err != 0) {
		return err;
	}

	return 0;
}

SYS_INIT(pca20035_board_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

