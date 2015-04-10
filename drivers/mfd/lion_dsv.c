/*
 * LION DSV  MFD Driver
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

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>

#include <linux/mfd/lion_dsv.h>

static struct lion_dsv *lion_dsv_base;
static struct mfd_cell lion_dsv_devs[] = {
	{ .name = "lion_dsv_dev" },
};

int lion_dsv_read_byte(struct lion_dsv *lion_dsv, u8 reg, u8 *read)
{
	int ret;
	unsigned int val;

	ret = regmap_read(lion_dsv->regmap, reg, &val);
	if (ret < 0)
		return ret;

	*read = (u8)val;
	return 0;
}
EXPORT_SYMBOL_GPL(lion_dsv_read_byte);

int lion_dsv_write_byte(struct lion_dsv *lion_dsv, u8 reg, u8 data)
{
	return regmap_write(lion_dsv->regmap, reg, data);
}
EXPORT_SYMBOL_GPL(lion_dsv_write_byte);

int lion_dsv_update_bits(struct lion_dsv *lion_dsv, u8 reg, u8 mask, u8 data)
{
	int ret;
	ret = regmap_update_bits(lion_dsv->regmap, reg, mask, data);
	return ret;
}
EXPORT_SYMBOL_GPL(lion_dsv_update_bits);

int sm5107_mode_change(int mode)
{
	int ret = 0;

	printk("[%s]mode = %d \n", __func__, mode);
	if (lion_dsv_base == NULL)
		return -EINVAL;

	if(mode == 0) {
		ret = lion_dsv_write_byte(lion_dsv_base, SM5107_CTRL_SET_REG, 0); // + 5.5V
	} else if(mode == 1) {
		ret = lion_dsv_write_byte(lion_dsv_base, SM5107_CTRL_SET_REG, 0x40); // + 5.5V
	}

	return ret;
}
EXPORT_SYMBOL_GPL(sm5107_mode_change);

int dw8768_mode_change(int mode)
{
	int ret = 0;

	printk("[%s]mode = %d \n", __func__, mode);
	if (lion_dsv_base == NULL)
		return -EINVAL;

	if(mode == 0) {
//		ret = lion_dsv_update_bits(lion_dsv_base, DW8768_ENABLE_REG, 0x08, 0);
		ret = lion_dsv_write_byte(lion_dsv_base, DW8768_ENABLE_REG, 0x07);

	} else if(mode == 1) {
//		ret = lion_dsv_update_bits(lion_dsv_base, DW8768_ENABLE_REG, 0x08, 1);
		ret = lion_dsv_write_byte(lion_dsv_base, DW8768_ENABLE_REG, 0x0F);

	}
    printk("[%s]mode = %d \n", __func__, mode);

    pr_err("ret=%d [dw8768_mode_change]\n",ret);
    
	return ret;
}
EXPORT_SYMBOL_GPL(dw8768_mode_change);


static struct regmap_config lion_dsv_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = LION_DSV_MAX_REGISTERS,
};

static int lion_dsv_probe(struct i2c_client *cl, const struct i2c_device_id *id)
{
	struct lion_dsv *lion_dsv;
	struct device *dev = &cl->dev;
	struct lion_dsv_platform_data *pdata = dev_get_platdata(dev);
	int rc = 0;

	pr_err("%s start\n",__func__);

	lion_dsv = devm_kzalloc(dev, sizeof(*lion_dsv), GFP_KERNEL);
	if (!lion_dsv)
		return -ENOMEM;

	lion_dsv->pdata = pdata;

	lion_dsv->regmap = devm_regmap_init_i2c(cl, &lion_dsv_regmap_config);
	if (IS_ERR(lion_dsv->regmap)){
		pr_err("Failed to allocate register map\n");
		devm_kfree(dev, lion_dsv);
		return PTR_ERR(lion_dsv->regmap);
	}

	lion_dsv->dev = &cl->dev;
	i2c_set_clientdata(cl,lion_dsv);
	lion_dsv_base = lion_dsv;

	rc = mfd_add_devices(dev, -1, lion_dsv_devs, ARRAY_SIZE(lion_dsv_devs),
			       NULL, 0);
	if (rc) {
		pr_err("Failed to add lion_dsv subdevice ret=%d\n", rc);
        return -ENODEV;
    }

	return rc;
}

static int lion_dsv_remove(struct i2c_client *cl)
{
	struct lion_dsv *lion_dsv = i2c_get_clientdata(cl);

	mfd_remove_devices(lion_dsv->dev);

	return 0;
}

static const struct i2c_device_id lion_dsv_ids[] = {
	{ "lion_dsv", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lion_dsv_ids);

#ifdef CONFIG_OF
static const struct of_device_id lion_dsv_of_match[] = {
	{ .compatible = "sm_dw,lion_dsv", },
	{ }
};
MODULE_DEVICE_TABLE(of, lion_dsv_of_match);
#endif

static struct i2c_driver lion_dsv_driver = {
    .probe = lion_dsv_probe,
    .remove = lion_dsv_remove,
    .driver = {
		.name = "lion_dsv",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(lion_dsv_of_match),
	},
	.id_table = lion_dsv_ids,
	
};
module_i2c_driver(lion_dsv_driver);

MODULE_DESCRIPTION("lion_dsv MFD Core");
MODULE_LICENSE("GPL");

