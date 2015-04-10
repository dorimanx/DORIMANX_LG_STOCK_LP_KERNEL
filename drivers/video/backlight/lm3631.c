/*
 * TI LM3631 Backlight Driver
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

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/mfd/lm3631.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/slab.h>
#include <mach/board_lge.h>


#define LM3631_MAX_BRIGHTNESS	1511
#define DEFAULT_BL_NAME			"lcd-backlight"

enum lm3631_bl_ctrl_mode {
	LMU_BL_I2C,
	LMU_BL_PWM,
};

struct lm3631_bl {
	struct device *dev;
	struct backlight_device *bl_dev;

	struct lm3631 *lm3631;
	struct lm3631_backlight_platform_data *pdata;
	enum lm3631_bl_ctrl_mode mode;

	struct pwm_device *pwm;
};

#define BL_OFF 0
#define BL_ON 1
static int backlight_status = BL_OFF;

struct backlight_device *lm3631_device;
static int cur_main_lcd_level = LM3631_MAX_BRIGHTNESS;

static int lm3631_bl_enable(struct lm3631_bl *lm3631_bl, int enable)
{
	backlight_status = enable;
	pr_info("%s:enable:%d\n", __func__, enable);
	return lm3631_update_bits(lm3631_bl->lm3631, LM3631_REG_DEVCTRL,
				  LM3631_BL_EN_MASK,
				  enable << LM3631_BL_EN_SHIFT);
}

#ifndef CONFIG_MACH_LGE_BACKLIGHT_SUPPORT
static void lm3631_bl_pwm_ctrl(struct lm3631_bl *lm3631_bl, int br, int max_br)
{
	unsigned int period;
	unsigned int duty;
	struct pwm_device *pwm;
	if (!lm3631_bl->pdata)
		return;

	period = lm3631_bl->pdata->pwm_period;
	duty = br * period / max_br;
	/* Request a PWM device with the consumer name */
	if (!lm3631_bl->pwm) {
#ifdef CONFIG_MACH_LGE
		pwm = pwm_request(1, "lm3631-backlight");
#else
		pwm = devm_pwm_get(m3631_bl->dev, "lm3631-backlight");
#endif
		if (IS_ERR(pwm)) {
			dev_err(lm3631_bl->dev, "can not get PWM device\n");
			return;
		}
		lm3631_bl->pwm = pwm;
	}

	pwm_config(lm3631_bl->pwm, duty, period);
	if (duty)
		pwm_enable(lm3631_bl->pwm);
	else
		pwm_disable(lm3631_bl->pwm);
}
#endif

static inline int lm3631_bl_set_brightness(struct lm3631_bl *lm3631_bl, int val)
{
	u8 data;
	int ret;

	cur_main_lcd_level = val;
#ifndef CONFIG_MACH_LGE_BACKLIGHT_SUPPORT
	lm3631_bl->bl_dev->props.brightness = cur_main_lcd_level;
#endif

	data = val & LM3631_BRT_LSB_MASK;
	ret = lm3631_update_bits(lm3631_bl->lm3631, LM3631_REG_BRT_LSB,
				 LM3631_BRT_LSB_MASK, data);
	if (ret)
		return ret;

	data = (val >> LM3631_BRT_MSB_SHIFT) & 0xFF;
	return lm3631_write_byte(lm3631_bl->lm3631, LM3631_REG_BRT_MSB,
				 data);
}

void lm3631_lcd_backlight_set_level(int level)
{
	struct lm3631_bl *lm3631_bl = bl_get_data(lm3631_device);
	int ret = 0;
#ifdef CONFIG_MACH_LGE
	struct lm3631_backlight_platform_data *pdata = lm3631_bl->pdata;
	int cal_level;

	pr_info("%s : level is %d\n", __func__, level);

	if (lge_get_boot_mode() == LGE_BOOT_MODE_FACTORY) {
		pr_info("%s : factory mode! Will not turn on backligh.\n", __func__);
		level = 0;
	}
#ifdef CONFIG_MACH_LGE_BACKLIGHT_SUPPORT
	lm3631_bl->bl_dev->props.brightness = level;
#endif
#endif

	if (level == 0) {
		if (backlight_status == BL_ON)
			ret = lm3631_bl_enable(lm3631_bl, 0);
		return;
	} else{
		if (backlight_status == BL_OFF)
			ret = lm3631_bl_enable(lm3631_bl, 1);
	}

	if (ret) {
		pr_err("%s DEBUG error enable or disable backlight\n", __func__);
	}

#ifdef CONFIG_MACH_LGE
	if (level >= pdata->blmap_size)
		level = pdata->blmap_size - 1;

	if (pdata->blmap)
		cal_level = pdata->blmap[level];
	else
		cal_level = level;

	if (cal_level == cur_main_lcd_level)
		return;

	ret = lm3631_bl_set_brightness(lm3631_bl, cal_level);
#else
	ret = lm3631_bl_set_brightness(lm3631_bl, level);
#endif

	if (ret) {
		pr_err("%s DEBUG error set backlight\n", __func__);
	}
};
EXPORT_SYMBOL(lm3631_lcd_backlight_set_level);

static int lm3631_bl_update_status(struct backlight_device *bl_dev)
{
	int ret = 0;

#if defined(CONFIG_MACH_LGE_BACKLIGHT_SUPPORT)
	lm3631_lcd_backlight_set_level(bl_dev->props.brightness);
#else

	struct lm3631_bl *lm3631_bl = bl_get_data(bl_dev);
	int brt;

	if (bl_dev->props.state & BL_CORE_SUSPENDED)
		bl_dev->props.brightness = 0;

	brt = bl_dev->props.brightness;
	if (brt == cur_main_lcd_level)
		return 0;

	if (brt > 0) {
		if (backlight_status == BL_OFF)
			ret = lm3631_bl_enable(lm3631_bl, 1);
	} else {
		if (backlight_status == BL_ON)
			ret = lm3631_bl_enable(lm3631_bl, 0);
	}

	if (ret)
		return ret;

	if (lm3631_bl->mode == LMU_BL_PWM)
		lm3631_bl_pwm_ctrl(lm3631_bl, brt,
				   bl_dev->props.max_brightness);
	else
		ret = lm3631_bl_set_brightness(lm3631_bl, brt);
#endif
	return ret;
}

static int lm3631_bl_get_brightness(struct backlight_device *bl_dev)
{
	return bl_dev->props.brightness;
}


static void lm3631_lcd_backlight_set_level_nomapping(int level)
{
	struct lm3631_bl *lm3631_bl = bl_get_data(lm3631_device);
	int ret;
	pr_err("%s : level is %d\n", __func__, level);

	cur_main_lcd_level = level;

	if (level > 2047)
		level = 2047;

	ret = lm3631_bl_set_brightness(lm3631_bl, level);
	if (ret) {
		pr_err("%s DEBUG error set backlight\n", __func__);
	}
};

static const struct backlight_ops lm3631_bl_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = lm3631_bl_update_status,
	.get_brightness = lm3631_bl_get_brightness,
};

static int lm3631_bl_register(struct lm3631_bl *lm3631_bl)
{
	struct backlight_device *bl_dev;
	struct backlight_properties props;
	struct lm3631_backlight_platform_data *pdata = lm3631_bl->pdata;
	char name[20];

	props.type = BACKLIGHT_PLATFORM;
	props.brightness = pdata ? pdata->init_brightness : LM3631_MAX_BRIGHTNESS;
	props.max_brightness = LM3631_MAX_BRIGHTNESS;

	if (!pdata || !pdata->name)
		snprintf(name, sizeof(name), "%s", DEFAULT_BL_NAME);
	else
		snprintf(name, sizeof(name), "%s", pdata->name);

	bl_dev = backlight_device_register(name, lm3631_bl->dev, lm3631_bl,
					   &lm3631_bl_ops, &props);
	lm3631_device = bl_dev;
	if (IS_ERR(bl_dev))
		return PTR_ERR(bl_dev);

	lm3631_bl->bl_dev = bl_dev;

	return 0;
}

static void lm3631_bl_unregister(struct lm3631_bl *lm3631_bl)
{
	if (lm3631_bl->bl_dev)
		backlight_device_unregister(lm3631_bl->bl_dev);
}

static int lm3631_bl_set_ovp(struct lm3631_bl *lm3631_bl)
{
    /* Set OVP to 25V by default */
    return lm3631_update_bits(lm3631_bl->lm3631, LM3631_REG_BL_BOOST,
								LM3631_BOOST_OVP_MASK, LM3631_BOOST_OVP_25V);
}

static int lm3631_bl_set_ctrl_mode(struct lm3631_bl *lm3631_bl)
{
#ifndef CONFIG_MACH_LGE
	struct lm3631_backlight_platform_data *pdata = lm3631_bl->pdata;
#endif
	/* Brightness control mode is I2C only by default */
#ifdef CONFIG_MACH_LGE
		lm3631_bl->mode = LMU_BL_I2C;
		return lm3631_update_bits(lm3631_bl->lm3631,
					  LM3631_REG_BRT_MODE, LM3631_BRT_MASK,
					  LM3631_I2C_ONLY);
#else
	if (!pdata) {
		lm3631_bl->mode = LMU_BL_I2C;
		return lm3631_update_bits(lm3631_bl->lm3631,
					  LM3631_REG_BRT_MODE, LM3631_BRT_MASK,
					  LM3631_I2C_ONLY);
	}

	if (pdata->pwm_period > 0)
		lm3631_bl->mode = LMU_BL_PWM;

	return lm3631_update_bits(lm3631_bl->lm3631, LM3631_REG_BRT_MODE,
				  LM3631_BRT_MASK, pdata->mode);
#endif
}

static int lm3631_bl_string_configure(struct lm3631_bl *lm3631_bl)
{
	u8 val;

	if (lm3631_bl->pdata->is_full_strings)
		val = LM3631_BL_TWO_STRINGS;
	else
		val = LM3631_BL_ONE_STRING;

	return lm3631_update_bits(lm3631_bl->lm3631, LM3631_REG_BL_CFG,
				  LM3631_BL_STRING_MASK, val);
}

static int lm3631_bl_configure(struct lm3631_bl *lm3631_bl)
{
	int ret;

    ret = lm3631_bl_set_ovp(lm3631_bl);
    if (ret)
		return ret;

	ret = lm3631_bl_set_ctrl_mode(lm3631_bl);
	if (ret)
		return ret;

	return lm3631_bl_string_configure(lm3631_bl);
}

static ssize_t lcd_backlight_show_level(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int r = 0;

	r = snprintf(buf, PAGE_SIZE, "LCD Backlight Level is : %d\n",
			cur_main_lcd_level);

	return r;
}

static ssize_t lcd_backlight_store_level(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int level;

	if (!count)
		return -EINVAL;

	level = simple_strtoul(buf, NULL, 10);
	lm3631_lcd_backlight_set_level_nomapping(level);

	return count;
}

DEVICE_ATTR(bl_level, 0644, lcd_backlight_show_level,
		lcd_backlight_store_level);

static int lm3631_bl_parse_dt(struct device *dev, struct lm3631_bl *lm3631_bl)
{
	struct device_node *node = dev->of_node;
	struct lm3631_backlight_platform_data *pdata;
	int i;
	u32 *array;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	of_property_read_string(node, "bl-name", &pdata->name);

	if (of_find_property(node, "full-strings-used", NULL))
		pdata->is_full_strings = true;

	if (of_find_property(node, "mode-pwm-only", NULL))
		pdata->mode = LM3631_PWM_ONLY;
	else if (of_find_property(node, "mode-comb1", NULL))
		pdata->mode = LM3631_COMB1;
	else if (of_find_property(node, "mode-comb2", NULL))
		pdata->mode = LM3631_COMB2;
#ifdef CONFIG_MACH_LGE
	of_property_read_u32(node, "initial-brightness",
			    (u32 *)&pdata->init_brightness);

	of_property_read_u32(node, "blmap_size",
			&pdata->blmap_size);

	if (pdata->blmap_size) {
		array = kzalloc(sizeof(u32) * pdata->blmap_size, GFP_KERNEL);
		if (!array) {
			pr_err("no more mem for array\n");
			return -ENOMEM;
		}
		of_property_read_u32_array(node, "blmap", array, pdata->blmap_size);
		pdata->blmap = kzalloc(sizeof(u16) * pdata->blmap_size, GFP_KERNEL);
		if (!pdata->blmap) {
			pr_err("no more mem for blmap\n");
			kfree(array);
			return -ENOMEM;
		}

		for (i = 0; i < pdata->blmap_size; i++)
			pdata->blmap[i] = (u16)array[i];

		kfree(array);
	} else {
		pr_err("not defined blmap_size");
	}

#else
	of_property_read_u8(node, "initial-brightness",
			    (u8 *)&pdata->init_brightness);
#endif
	of_property_read_u32(node, "pwm-period", &pdata->pwm_period);
	lm3631_bl->pdata = pdata;

	return 0;
}

static int lm3631_bl_probe(struct platform_device *pdev)
{
	struct lm3631 *lm3631 = dev_get_drvdata(pdev->dev.parent);
	struct lm3631_backlight_platform_data *pdata = lm3631->pdata->bl_pdata;
	struct lm3631_bl *lm3631_bl;
	int ret;

	lm3631_bl = devm_kzalloc(&pdev->dev, sizeof(*lm3631_bl), GFP_KERNEL);
	if (!lm3631_bl)
		return -ENOMEM;

	lm3631_bl->pdata = pdata;
	if (!lm3631_bl->pdata) {
		if (IS_ENABLED(CONFIG_OF))
			ret = lm3631_bl_parse_dt(&pdev->dev, lm3631_bl);
		else
			return -ENODEV;

		if (ret)
			return ret;
	}

	lm3631_bl->dev = &pdev->dev;
	lm3631_bl->lm3631 = lm3631;
	platform_set_drvdata(pdev, lm3631_bl);

	ret = lm3631_bl_configure(lm3631_bl);
	if (ret) {
		dev_err(&pdev->dev, "backlight config err: %d\n", ret);
		return ret;
	}

	ret = lm3631_bl_register(lm3631_bl);
	if (ret) {
		dev_err(&pdev->dev, "register backlight err: %d\n", ret);
		return ret;
	}

	device_create_file(&pdev->dev, &dev_attr_bl_level);

#ifdef CONFIG_LGE_LCD_OFF_DIMMING
	if ((lge_get_bootreason() == 0x77665560) || (lge_get_bootreason() == 0x77665561)) {
		lm3631_bl->bl_dev->props.brightness = 50;
		pr_info("%s : fota reboot - backlight set 50\n", __func__);
	}
#endif
	if (lge_get_boot_mode() == LGE_BOOT_MODE_FACTORY) {
		lm3631_bl->bl_dev->props.brightness = 0;
		pr_info("%s : 130K is connected\n", __func__);
	} else {
		backlight_status = BL_ON;
		backlight_update_status(lm3631_bl->bl_dev);
	}

	return 0;
}

static int lm3631_bl_remove(struct platform_device *pdev)
{
	struct lm3631_bl *lm3631_bl = platform_get_drvdata(pdev);
	struct backlight_device *bl_dev = lm3631_bl->bl_dev;

	bl_dev->props.brightness = 0;
	backlight_update_status(bl_dev);
	lm3631_bl_unregister(lm3631_bl);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id lm3631_bl_of_match[] = {
	{ .compatible = "ti,lm3631-backlight", },
	{ }
};
MODULE_DEVICE_TABLE(of, lm3631_bl_of_match);
#endif

static struct platform_driver lm3631_bl_driver = {
	.probe = lm3631_bl_probe,
	.remove = lm3631_bl_remove,
	.driver = {
		.name = "lm3631-backlight",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(lm3631_bl_of_match),
	},
};
module_platform_driver(lm3631_bl_driver);

MODULE_DESCRIPTION("TI LM3631 Backlight Driver");
MODULE_AUTHOR("Milo Kim");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:lm3631-backlight");
