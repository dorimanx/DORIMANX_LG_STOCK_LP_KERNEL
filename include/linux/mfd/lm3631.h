/*
 * TI LM3631 MFD Driver
 *
 * Copyright 2013 Texas Instruments
 *
 * Author: Milo Kim <milo.kim@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __MFD_LM3631_H__
#define __MFD_LM3631_H__

#include <linux/gpio.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/regulator/machine.h>

/* Registers */
#define LM3631_REG_DEVCTRL		0x00
#define LM3631_LCD_EN_MASK		BIT(1)
#define LM3631_LCD_EN_SHIFT		1
#define LM3631_BL_EN_MASK		BIT(0)
#define LM3631_BL_EN_SHIFT		0

#define LM3631_REG_BRT_LSB		0x01
#define LM3631_BRT_LSB_MASK		(BIT(0) | BIT(1) | BIT(2))
#define LM3631_REG_BRT_MSB		0x02
#define LM3631_BRT_MSB_SHIFT		3

#define LM3631_REG_BL_CFG		0x06
#define LM3631_BL_STRING_MASK		BIT(3)
#define LM3631_BL_TWO_STRINGS		0
#define LM3631_BL_ONE_STRING		BIT(3)

#define LM3631_REG_BL_BOOST 0x07
#define LM3631_BOOST_OVP_MASK (BIT(1) | BIT(2))
#define LM3631_BOOST_OVP_25V 0x04

#define LM3631_REG_BRT_MODE		0x08
#define LM3631_BRT_MASK			(BIT(2) | BIT(3))

#define LM3631_REG_LDO_CTRL1		0x0A
#define LM3631_EN_OREF_MASK		BIT(0)
#define LM3631_EN_VNEG_MASK		BIT(1)
#define LM3631_EN_VPOS_MASK		BIT(2)

#define LM3631_REG_LDO_CTRL2		0x0B
#define LM3631_EN_CONT_MASK		BIT(0)

#define LM3631_REG_VOUT_CONT		0x0C
#define LM3631_VOUT_CONT_MASK		(BIT(6) | BIT(7))

#define LM3631_REG_VOUT_BOOST		0x0C
#define LM3631_REG_VOUT_POS		0x0D
#define LM3631_REG_VOUT_NEG		0x0E
#define LM3631_REG_VOUT_OREF		0x0F
#define LM3631_VOUT_MASK		0x3F

#define LM3631_REG_ENTIME_VCONT		0x0B
#define LM3631_ENTIME_CONT_MASK		0x70

#define LM3631_REG_ENTIME_VOREF		0x0F
#define LM3631_REG_ENTIME_VPOS		0x10
#define LM3631_REG_ENTIME_VNEG		0x11
#define LM3631_ENTIME_MASK		0xF0
#define LM3631_ENTIME_SHIFT		4

#define LM3631_MAX_REGISTERS		0x16

#define LM3631_NUM_REGULATORS		5

enum lm3631_brightness_mode {
	LM3631_I2C_ONLY	= 0 << 2,
	LM3631_PWM_ONLY	= 1 << 2,
	LM3631_COMB1	= 2 << 2,	/* I2C x PWM befoer sloping */
	LM3631_COMB2	= 3 << 2,	/* Sloped I2C x PWM */
};

/*
 * struct lm3633_bl_platform_data
 * @name: Backlight driver name
 * @is_full_strings: set true if two strings are used
 * @init_brightness: Initial brightness value
 * @mode: Backlight control mode
 * @pwm_period: Platform specific PWM period value. unit is nano
 */
struct lm3631_backlight_platform_data {
	const char *name;
	bool is_full_strings;
#ifdef CONFIG_MACH_LGE
	u32 init_brightness;
#else
	u8 init_brightness;
#endif
	enum lm3631_brightness_mode mode;

	/* Only valid in case of PWM mode */
	unsigned int pwm_period;

#ifdef CONFIG_MACH_LGE
	int blmap_size;
	u16 *blmap;
#endif
};

/*
 * struct lmu_platform_data
 * @en_gpio: GPIO for nRST pin
 * @regulator_data: Regulator initial data for LCD bias
 * @bl_pdata: Backlight platform data
 */
struct lm3631_platform_data {
	int en_gpio;
	int bl_en_gpio;
	struct regulator_init_data *regulator_data[LM3631_NUM_REGULATORS];
	struct lm3631_backlight_platform_data *bl_pdata;
};

struct lm3631_info {
	const char *name;
	const int *volt_table;
	unsigned int n_voltages;

	unsigned int vsel_reg;
	unsigned int vsel_mask;
	unsigned int apply_reg;
	unsigned int apply_bit;
	unsigned int enable_reg;
	unsigned int enable_mask;
	bool enable_is_inverted;
};
/*
 * struct lm3631
 * @dev: Parent device pointer
 * @regmap: Used for i2c communcation on accessing registers
 * @pdata: LMU platform specific data
 */
struct lm3631 {
	struct device *dev;
	struct regmap *regmap;
	struct lm3631_info *info[LM3631_NUM_REGULATORS];
	struct lm3631_platform_data *pdata;
};

static inline struct lm3631 *dev_to_lm3631(struct device *dev)
{
	    return dev_get_drvdata(dev);
}
int lm3631_read_byte(struct lm3631 *lm3631, u8 reg, u8 *read);
int lm3631_write_byte(struct lm3631 *lm3631, u8 reg, u8 data);
int lm3631_update_bits(struct lm3631 *lm3631, u8 reg, u8 mask, u8 data);
#endif
