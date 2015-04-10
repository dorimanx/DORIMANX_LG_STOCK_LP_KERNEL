/*
 * TPS65132 MFD Driver
 *
 * Copyright 2014 LG Electronics Inc,
 *
 * Author: Baryun Hwang <baryun.hwang@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __MFD_TPS65132_H__
#define __MFD_TPS65132_H__

#include <linux/gpio.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/regulator/machine.h>

#define TPS65132_REG_1 0x00
#define TPS65132_REG_2 0x01
#define TPS65132_REG_3 0x02
#define TPS65132_REG_4 0x03

#define TPS65132_VREG_LV_40 0x00
#define TPS65132_VREG_LV_50 0x0A
#define TPS65132_VREG_LV_55 0x0F
#define TPS65132_VREG_LV_60 0x14

#define TPS65132_MAX_REGISTERS 0x04

struct tps65132_platform_data {
	const char *name;
};

struct tps65132 {
	struct device *dev;
	struct regmap *regmap;
	struct tps65132_platform_data *pdata;
};

extern int tps65132_read_byte(struct tps65132 *tps65132, u8 reg, u8 *read);
extern int tps65132_write_byte(struct tps65132 *tps65132, u8 reg, u8 data);
extern int tps65132_update_bits(struct tps65132 *tps65132, u8 reg, u8 mask, u8 data);
extern int tps65132_regulate_voltage(int on, int mode);
#endif
