/*
 * LION DSV MFD Driver
 *
 * Copyright 2014 LG Electronics Inc,
 *
 * Author:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __MFD_LION_DSV_H__
#define __MFD_LION_DSV_H__

#include <linux/gpio.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/regulator/machine.h>

#define DW8768_ENABLE_REG 0x05

#define SM5107_CTRL_SET_REG 0xFF

#define LION_DSV_MAX_REGISTERS 0x04

struct lion_dsv_platform_data {
	const char *name;
};

struct lion_dsv {
	struct device *dev;
	struct regmap *regmap;
	struct lion_dsv_platform_data *pdata;
};

#endif
