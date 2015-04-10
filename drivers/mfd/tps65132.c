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

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>

#include <linux/mfd/tps65132.h>

static struct tps65132 *tps65132_base;
static struct mfd_cell tps65132_devs[] = {
	{ .name = "tps65132_dev" },
};

int tps65132_read_byte(struct tps65132 *tps65132, u8 reg, u8 *read)
{
	int ret;
	unsigned int val;

	ret = regmap_read(tps65132->regmap, reg, &val);
	if (ret < 0)
		return ret;

	*read = (u8)val;
	return 0;
}
EXPORT_SYMBOL_GPL(tps65132_read_byte);

int tps65132_write_byte(struct tps65132 *tps65132, u8 reg, u8 data)
{
	return regmap_write(tps65132->regmap, reg, data);
}
EXPORT_SYMBOL_GPL(tps65132_write_byte);

int tps65132_update_bits(struct tps65132 *tps65132, u8 reg, u8 mask, u8 data)
{
	int ret;
	ret = regmap_update_bits(tps65132->regmap, reg, mask, data);
	return ret;
}
EXPORT_SYMBOL_GPL(tps65132_update_bits);

int tps65132_regulate_voltage(int on, int mode)
{
	int ret;

	if (tps65132_base == NULL)
		return -EINVAL;

	if (on) {
		ret = tps65132_write_byte(tps65132_base, mode, TPS65132_VREG_LV_40);
		if (ret < 0)
			return ret;
	} else {
		ret = tps65132_write_byte(tps65132_base, mode, TPS65132_VREG_LV_50);
		if (ret < 0)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(tps65132_regulate_voltage);

static struct regmap_config tps65132_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = TPS65132_MAX_REGISTERS,
};

static int tps65132_probe(struct i2c_client *cl, const struct i2c_device_id *id)
{
	struct tps65132 *tps65132;
	struct device *dev = &cl->dev;
	struct tps65132_platform_data *pdata = dev_get_platdata(dev);
	int rc = 0;

	pr_err("%s start\n",__func__);

	tps65132 = devm_kzalloc(dev, sizeof(*tps65132), GFP_KERNEL);
	if (!tps65132)
		return -ENOMEM;

	tps65132->pdata = pdata;

	tps65132->regmap = devm_regmap_init_i2c(cl, &tps65132_regmap_config);
	if (IS_ERR(tps65132->regmap)){
		pr_err("Failed to allocate register map\n");
		devm_kfree(dev, tps65132);
		return PTR_ERR(tps65132->regmap);
	}

	tps65132->dev = &cl->dev;
	i2c_set_clientdata(cl,tps65132);
	tps65132_base = tps65132;

	rc = mfd_add_devices(dev, -1, tps65132_devs, ARRAY_SIZE(tps65132_devs),
			       NULL, 0);
	if (rc)
		pr_err("Failed to add tps65132 subdevice ret=%d\n", rc);

	return rc;
}

static int tps65132_remove(struct i2c_client *cl)
{
	struct tps65132 *tps65132 = i2c_get_clientdata(cl);

	mfd_remove_devices(tps65132->dev);

	return 0;
}

static const struct i2c_device_id tps65132_ids[] = {
	{ "tps65132", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tps65132_ids);

#ifdef CONFIG_OF
static const struct of_device_id tps65132_of_match[] = {
	{ .compatible = "ti,tps65132", },
	{ }
};
MODULE_DEVICE_TABLE(of, tps65132_of_match);
#endif

static struct i2c_driver tps65132_driver = {
	.probe = tps65132_probe,
	.remove = tps65132_remove,
	.driver = {
		.name = "tps65132",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(tps65132_of_match),
	},
	.id_table = tps65132_ids,
};
module_i2c_driver(tps65132_driver);

MODULE_DESCRIPTION("tps65132 MFD Core");
