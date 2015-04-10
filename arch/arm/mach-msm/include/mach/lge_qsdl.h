/* Copyright (c) 2014 LG Electronics, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __MACH_LGE_QSDL_H
#define __MACH_LGE_QSDL_H

struct lge_qsdl_platform_data {
	const bool oneshot_read;
	const bool using_uevent;
};

extern int lge_qsdl_trigger_modem_uevent(void);
extern int lge_qsdl_increase_modem_ssr(void);

#define LGE_QSDL_DEV_NAME "lge_qsdl"

#endif
