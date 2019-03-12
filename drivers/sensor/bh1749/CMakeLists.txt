#
#  Copyright (c) 2019 Nordic Semiconductor ASA
#
#  SPDX-License-Identifier: Apache-2.0
#

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_BH1749 bh1749.c)
