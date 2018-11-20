/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>

int adp5360_init(const char *dev_name);
int adp5360_buckbst_enable(bool enable);
int adp5360_buck_1v8_set(void);
int adp5360_buckbst_3v3_set(void);
int adp5360_factory_reset(void);
