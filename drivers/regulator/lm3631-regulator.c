/*
 * TI LM3631 LDO Regulator Driver
 *
 * Copyright 2013 Texas Instruments
 *
 * Author: Milo(Woogyom) Kim <milo.kim@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/err.h>
#include <linux/mfd/lm3631.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/slab.h>

#define ENABLE_TIME_USEC		1000

enum lm3631_regulator_id {
	LM3631_REGULATOR_BOOST,
	LM3631_LDO_CONT,
	LM3631_LDO_OREF,
	LM3631_LDO_POS,
	LM3631_LDO_NEG,
};

enum lm3631_regulator_control {
	LM3631_SET_ENABLE,
	LM3631_SET_DISABLE,
	LM3631_GET_STATUS,
};

struct of_regulator_match {
	const char *name;
	void *driver_data;
	struct regulator_init_data *init_data;
	struct device_node *of_node;
};
struct lm3631_regulator {
	struct lm3631 *lm3631;
	struct regulator_desc *desc;
	struct regulator_dev *regulator;
	struct regulator_init_data *init_data;
};

static const int lm3631_boost_vtbl[] = {
	4500000, 4550000, 4600000, 4650000, 4700000, 4750000, 4800000, 4850000,
	4900000, 4950000, 5000000, 5050000, 5100000, 5150000, 5200000, 5250000,
	5300000, 5350000, 5400000, 5450000, 5500000, 5550000, 5600000, 5650000,
	5700000, 5750000, 5800000, 5850000, 5900000, 5950000, 6000000, 6050000,
	6100000, 6150000, 6200000, 6250000, 6300000, 6350000,
};

static const int lm3631_ldo_cont_vtbl[] = {
	1800000, 2300000, 2800000, 3300000,
};

static const int lm3631_ldo_target_vtbl[] = {
	4000000, 4050000, 4100000, 4150000, 4200000, 4250000, 4300000, 4350000,
	4400000, 4450000, 4500000, 4550000, 4600000, 4650000, 4700000, 4750000,
	4800000, 4850000, 4900000, 4950000, 5000000, 5050000, 5100000, 5150000,
	5200000, 5250000, 5300000, 5350000, 5400000, 5450000, 5500000, 5550000,
	5600000, 5650000, 5700000, 5750000, 5800000, 5850000, 5900000, 5950000,
	6000000,
};

const int ldo_cont_enable_time[] = {
	0, 2000, 5000, 10000, 20000, 50000, 100000, 200000,
};

static int lm3631_list_voltage_table(struct regulator_dev *rdev,
				     unsigned selector)
{
	enum lm3631_regulator_id id = rdev_get_id(rdev);
	const unsigned int *volt_table;

	if (selector >= rdev->desc->n_voltages)
		return -EINVAL;

	switch (id) {
	case LM3631_REGULATOR_BOOST:
		volt_table = lm3631_boost_vtbl;
		break;
	case LM3631_LDO_CONT:
		volt_table = lm3631_ldo_cont_vtbl;
		break;
	case LM3631_LDO_OREF:
	case LM3631_LDO_POS:
	case LM3631_LDO_NEG:
		volt_table = lm3631_ldo_target_vtbl;
		break;
	default:
		return -EINVAL;
	}

	return volt_table[selector];
}

static int lm3631_set_voltage_sel(struct regulator_dev *rdev,
				  unsigned selector)
{
	struct lm3631_regulator *lm3631_regulator = rdev_get_drvdata(rdev);
	enum lm3631_regulator_id id = rdev_get_id(rdev);
	u8 addr, mask;

	switch (id) {
	case LM3631_REGULATOR_BOOST:
		addr = LM3631_REG_VOUT_BOOST;
		mask = LM3631_VOUT_MASK;
		break;
	case LM3631_LDO_CONT:
		addr = LM3631_REG_VOUT_CONT;
		mask = LM3631_VOUT_CONT_MASK;
		break;
	case LM3631_LDO_OREF:
		addr = LM3631_REG_VOUT_OREF;
		mask = LM3631_VOUT_MASK;
		break;
	case LM3631_LDO_POS:
		addr = LM3631_REG_VOUT_POS;
		mask = LM3631_VOUT_MASK;
		break;
	case LM3631_LDO_NEG:
		addr = LM3631_REG_VOUT_NEG;
		mask = LM3631_VOUT_MASK;
		break;
	default:
		return -EINVAL;
	}

	selector <<= ffs(mask) - 1;

	return lm3631_update_bits(lm3631_regulator->lm3631, addr, mask,
				  selector);
}

static int lm3631_get_voltage_sel(struct regulator_dev *rdev)
{
	struct lm3631_regulator *lm3631_regulator = rdev_get_drvdata(rdev);
	enum lm3631_regulator_id id = rdev_get_id(rdev);
	u8 addr, mask, val;
	int ret;

	switch (id) {
	case LM3631_REGULATOR_BOOST:
		addr = LM3631_REG_VOUT_BOOST;
		mask = LM3631_VOUT_MASK;
		break;
	case LM3631_LDO_CONT:
		addr = LM3631_REG_VOUT_CONT;
		mask = LM3631_VOUT_CONT_MASK;
		break;
	case LM3631_LDO_OREF:
		addr = LM3631_REG_VOUT_OREF;
		mask = LM3631_VOUT_MASK;
		break;
	case LM3631_LDO_POS:
		addr = LM3631_REG_VOUT_POS;
		mask = LM3631_VOUT_MASK;
		break;
	case LM3631_LDO_NEG:
		addr = LM3631_REG_VOUT_NEG;
		mask = LM3631_VOUT_MASK;
		break;
	default:
		return -EINVAL;
	}

	ret = lm3631_read_byte(lm3631_regulator->lm3631, addr, &val);
	if (ret)
		return ret;

	val &= mask;
	val >>= ffs(mask) - 1;

	return val;
}

static int lm3631_regulator_ctrl(struct regulator_dev *rdev,
				 enum lm3631_regulator_control ctrl)
{
	struct lm3631_regulator *lm3631_regulator = rdev_get_drvdata(rdev);
	enum lm3631_regulator_id id = rdev_get_id(rdev);
	u8 addr, mask, val;
	int ret;

	switch (id) {
	case LM3631_LDO_CONT:
		addr = LM3631_REG_LDO_CTRL2;
		mask = LM3631_EN_CONT_MASK;
		break;
	case LM3631_LDO_OREF:
		addr = LM3631_REG_LDO_CTRL1;
		mask = LM3631_EN_OREF_MASK;
		break;
	case LM3631_LDO_POS:
		addr = LM3631_REG_LDO_CTRL1;
		mask = LM3631_EN_VPOS_MASK;
		break;
	case LM3631_LDO_NEG:
		addr = LM3631_REG_LDO_CTRL1;
		mask = LM3631_EN_VNEG_MASK;
		break;
	default:
		return -EINVAL;
	}

	switch (ctrl) {
	case LM3631_SET_ENABLE:
		return lm3631_update_bits(lm3631_regulator->lm3631, addr, mask,
					  mask);
	case LM3631_SET_DISABLE:
		return lm3631_update_bits(lm3631_regulator->lm3631, addr, mask,
					  0);
	case LM3631_GET_STATUS:
		ret = lm3631_read_byte(lm3631_regulator->lm3631, addr, &val);
		if (ret)
			return ret;
		return (val & mask) != 0;
	default:
		return -EINVAL;
	}
}

static int lm3631_regulator_enable(struct regulator_dev *rdev)
{
	return lm3631_regulator_ctrl(rdev, LM3631_SET_ENABLE);
}

static int lm3631_regulator_disable(struct regulator_dev *rdev)
{
	return lm3631_regulator_ctrl(rdev, LM3631_SET_DISABLE);
}

static int lm3631_regulator_is_enabled(struct regulator_dev *rdev)
{
	return lm3631_regulator_ctrl(rdev, LM3631_GET_STATUS);
}

static int lm3631_regulator_enable_time(struct regulator_dev *rdev)
{
	struct lm3631_regulator *lm3631_regulator = rdev_get_drvdata(rdev);
	enum lm3631_regulator_id id = rdev_get_id(rdev);
	u8 val, addr, mask;

	switch (id) {
	case LM3631_LDO_CONT:
		addr = LM3631_REG_ENTIME_VCONT;
		mask = LM3631_ENTIME_CONT_MASK;
		break;
	case LM3631_LDO_OREF:
		addr = LM3631_REG_ENTIME_VOREF;
		mask = LM3631_ENTIME_MASK;
		break;
	case LM3631_LDO_POS:
		addr = LM3631_REG_ENTIME_VPOS;
		mask = LM3631_ENTIME_MASK;
		break;
	case LM3631_LDO_NEG:
		addr = LM3631_REG_ENTIME_VNEG;
		mask = LM3631_ENTIME_MASK;
		break;
	default:
		return -EINVAL;
	}

	if (lm3631_read_byte(lm3631_regulator->lm3631, addr, &val))
		return -EINVAL;

	val = (val & mask) >> LM3631_ENTIME_SHIFT;

	if (id == LM3631_LDO_CONT)
		return ldo_cont_enable_time[val];
	else
		return ENABLE_TIME_USEC * val;
}

static struct regulator_ops lm3631_boost_voltage_table_ops = {
	.list_voltage     = lm3631_list_voltage_table,
	.set_voltage_sel  = lm3631_set_voltage_sel,
	.get_voltage_sel  = lm3631_get_voltage_sel,
};

static struct regulator_ops lm3631_regulator_voltage_table_ops = {
	.list_voltage     = lm3631_list_voltage_table,
	.set_voltage_sel  = lm3631_set_voltage_sel,
	.get_voltage_sel  = lm3631_get_voltage_sel,
	.enable           = lm3631_regulator_enable,
	.disable          = lm3631_regulator_disable,
	.is_enabled       = lm3631_regulator_is_enabled,
	.enable_time      = lm3631_regulator_enable_time,
};

static struct regulator_desc lm3631_regulator_desc[] = {
	{
		.name           = "vboost",
		.id             = LM3631_REGULATOR_BOOST,
		.ops            = &lm3631_boost_voltage_table_ops,
		.n_voltages     = ARRAY_SIZE(lm3631_boost_vtbl),
		.type           = REGULATOR_VOLTAGE,
		.owner          = THIS_MODULE,
	},
	{
		.name           = "ldo_cont",
		.id             = LM3631_LDO_CONT,
		.ops            = &lm3631_regulator_voltage_table_ops,
		.n_voltages     = ARRAY_SIZE(lm3631_ldo_cont_vtbl),
		.type           = REGULATOR_VOLTAGE,
		.owner          = THIS_MODULE,
	},
	{
		.name           = "ldo_oref",
		.id             = LM3631_LDO_OREF,
		.ops            = &lm3631_regulator_voltage_table_ops,
		.n_voltages     = ARRAY_SIZE(lm3631_ldo_target_vtbl),
		.type           = REGULATOR_VOLTAGE,
		.owner          = THIS_MODULE,
	},
	{
		.name           = "ldo_vpos",
		.id             = LM3631_LDO_POS,
		.ops            = &lm3631_regulator_voltage_table_ops,
		.n_voltages     = ARRAY_SIZE(lm3631_ldo_target_vtbl),
		.type           = REGULATOR_VOLTAGE,
		.owner          = THIS_MODULE,
	},
	{
		.name           = "ldo_vneg",
		.id             = LM3631_LDO_NEG,
		.ops            = &lm3631_regulator_voltage_table_ops,
		.n_voltages     = ARRAY_SIZE(lm3631_ldo_target_vtbl),
		.type           = REGULATOR_VOLTAGE,
		.owner          = THIS_MODULE,
	},
};

static struct of_regulator_match lm3631_regulator_matches[] = {
	{ .name = "vboost", .driver_data = (void *)LM3631_REGULATOR_BOOST, },
	{ .name = "vcont",  .driver_data = (void *)LM3631_LDO_CONT, },
	{ .name = "voref",  .driver_data = (void *)LM3631_LDO_OREF, },
	{ .name = "vpos",   .driver_data = (void *)LM3631_LDO_POS,  },
	{ .name = "vneg",   .driver_data = (void *)LM3631_LDO_NEG,  },
};

/**
 * of_regulator_match - extract multiple regulator init data from device tree.
 * @dev: device requesting the data
 * @node: parent device node of the regulators
 * @matches: match table for the regulators
 * @num_matches: number of entries in match table
 *
 * This function uses a match table specified by the regulator driver to
 * parse regulator init data from the device tree. @node is expected to
 * contain a set of child nodes, each providing the init data for one
 * regulator. The data parsed from a child node will be matched to a regulator
 * based on either the deprecated property regulator-compatible if present,
 * or otherwise the child node's name. Note that the match table is modified
 * in place.
 *
 * Returns the number of matches found or a negative error code on failure.
 */
static int of_regulator_match(struct device *dev, struct device_node *node,
			      struct of_regulator_match *matches,
			      unsigned int num_matches)
{
	unsigned int count = 0;
	unsigned int i;
	const char *name;
	struct device_node *child;

	if (!dev || !node)
		return -EINVAL;

	for (i = 0; i < num_matches; i++) {
		struct of_regulator_match *match = &matches[i];
		match->init_data = NULL;
		match->of_node = NULL;
	}

	for_each_child_of_node(node, child) {
		name = of_get_property(child,
					"regulator-compatible", NULL);
		if (!name)
			name = child->name;
		for (i = 0; i < num_matches; i++) {
			struct of_regulator_match *match = &matches[i];
			if (match->of_node)
				continue;

			if (strcmp(match->name, name))
				continue;

			match->init_data =
				of_get_regulator_init_data(dev, child);
			if (!match->init_data) {
				dev_err(dev,
					"failed to parse DT for regulator %s\n",
					child->name);
				return -EINVAL;
			}
			match->of_node = child;
			count++;
			break;
		}
	}

	return count;
}

static int lm3631_regulator_parse_dt(struct device *dev,
				     struct lm3631_regulator *lm3631_regulator,
				     int id)
{
	struct device_node *node = dev->of_node;
	int count;

	count = of_regulator_match(dev, node, &lm3631_regulator_matches[id], 1);
	if (count <= 0){
		pr_err("%s DEBUG count is %d \n",__func__, count);
		return -ENODEV;
	}

	lm3631_regulator->init_data = lm3631_regulator_matches[id].init_data;

	return 0;
}

#define LCD_BOOST_VOUT		5800000
#define LCD_BIAS_VOUT		5500000

enum lcd_ldo_id {
	VBOOST,
	VPOS,
	VNEG,
	NUM_REGULATORS,
};
static int lm3631_regulator_probe(struct platform_device *pdev)
{
	struct lm3631 *lm3631 = dev_get_drvdata(pdev->dev.parent);
	struct lm3631_regulator *lm3631_regulator;
	struct regulator_dev *rdev;
	struct device *dev = pdev->dev.parent;
	int id = pdev->id;
	int ret;

	lm3631_regulator = devm_kzalloc(&pdev->dev, sizeof(*lm3631_regulator),
					GFP_KERNEL);

	if (!lm3631_regulator){
	pr_err("%s DEBUG no lm3631_regulator \n" ,__func__);
		return -ENOMEM;
	}
	lm3631_regulator->lm3631 = lm3631;
	lm3631_regulator->init_data = lm3631->pdata->regulator_data[id];
	if (!lm3631_regulator->init_data) {
		if (IS_ENABLED(CONFIG_OF))
			ret = lm3631_regulator_parse_dt(&pdev->dev,
							lm3631_regulator, id);
		else
			ret = -ENODEV;

		if (ret)
			return ret;
	}

	rdev = regulator_register(&lm3631_regulator_desc[id], dev,
				  lm3631_regulator->init_data,
				  lm3631_regulator, dev->of_node);
	if (IS_ERR(rdev)) {
		ret = PTR_ERR(rdev);
		dev_err(&pdev->dev, "[%d] regulator register err: %d\n",
			id + 1, ret);
		return ret;
	}

	lm3631_regulator->regulator = rdev;
	platform_set_drvdata(pdev, lm3631_regulator);

	return 0;
}

static int lm3631_regulator_remove(struct platform_device *pdev)
{
	struct lm3631_regulator *lm3631_regulator = platform_get_drvdata(pdev);

	regulator_unregister(lm3631_regulator->regulator);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id lm3631_regulator_of_match[] = {
	{ .compatible = "ti,lm3631-regulator", },
	{ }
};
MODULE_DEVICE_TABLE(of, lm3631_regulator_of_match);
#endif

static struct platform_driver lm3631_regulator_driver = {
	.probe = lm3631_regulator_probe,
	.remove = lm3631_regulator_remove,
	.driver = {
		.name = "lm3631-regulator",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(lm3631_regulator_of_match),
	},
};

module_platform_driver(lm3631_regulator_driver);

MODULE_DESCRIPTION("TI LM3631 Regulator Driver");
MODULE_AUTHOR("Milo Kim");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:lm3631-regulator");
