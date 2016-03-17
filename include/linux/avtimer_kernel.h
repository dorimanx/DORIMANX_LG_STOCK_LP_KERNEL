/*
 * Copyright (c) 2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _AVTIMER_H
#define _AVTIMER_H

#include <linux/ioctl.h>

#define MAJOR_NUM 100

#define IOCTL_GET_AVTIMER_TICK _IOR(MAJOR_NUM, 0, char *)
/*
 * This IOCTL is used read the avtimer tick value.
 * Avtimer is a 64 bit timer tick, hence the expected
 * argument is of type uint64_t
 */
struct dev_avtimer_data {
	uint32_t avtimer_msw_phy_addr;
	uint32_t avtimer_lsw_phy_addr;
};

int avcs_core_open(void);
int avcs_core_disable_power_collapse(int disable);/* true or flase */
int avcs_core_query_timer(uint64_t *avtimer_tick);

#endif
