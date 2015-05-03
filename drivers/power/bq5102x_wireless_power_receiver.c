/*
 * BQ5102x Wireless Power Receiver driver
 *
 * Copyright (C) 2014 LG Electronics, Inc
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/regulator/machine.h>
#include <linux/string.h>
#include <linux/of_gpio.h>



#ifdef CONFIG_CHARGER_UNIFIED_WLC_BQ5102X
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/power_supply.h>
#include <linux/power/unified_wireless_charger.h>
#include <linux/power/unified_wireless_charger_alignment.h>


#define DEVICE_NAME		"bq5102x"
#define _SMBUS_SUPPORT_

/* <OUTPUT VOLTAGE VALUE> */
/* 0x0				4.5 V */
/* 0x1				5.0 V */
/* 0x2				5.5 V */
/* 0x3				6.0 V */
/* 0x4				6.5 V */
/* 0x5				7.0 V */
/* 0x6				7.5 V */
/* 0x7				8.0 V */
#define BQ5102X_CURRENT_REGISTER1_REG (0x01)
#define BQ5102X_OUTPUT_VOLTAGE_VALUE (0x2 << 0)
#define BQ5102X_OUTPUT_VOLTAGE_MASK (BIT(2) | BIT(1) | BIT(0))

/* <OUTPUT CURRENT VALUE> */
/* 0x0				10 %  */
/* 0x1				20 %  */
/* 0x2				30 %  */
/* 0x3				40 %  */
/* 0x4				50 %  */
/* 0x5				60 %  */
/* 0x6				90 %  */
/* 0x7				100 % */
#define BQ5102X_CURRENT_REGISTER2_REG (0x02)
#define BQ5102X_OUTPUT_CURRENT_VALUE (0x6 << 0)
#define BQ5102X_OUTPUT_CURRENT_MASK (BIT(2) | BIT(1) | BIT(0))

#define BQ5102X_MAILBOX (0xE0)
#define BQ5102X_MAILBOX_ALIGN_AID_MODE (0x08)

/* <FOD OFFSET VALUE>  */
/* 0x0			0 mW   */
/* 0x1			39 mW  */
/* 0x2			78 mW  */
/* 0x3			117 mW */
/* 0x4			156 mW */
/* 0x5			195 mW */
/* 0x6			234 mW */
/* 0x7			273 mW */
#define BQ5102X_FOD_RAM_REG (0xE1)
#define BQ5102X_FOD_OFFSET_ENABLE (0x1 << 6)
#define BQ5102X_FOD_OFFSET_VALUE (0x4 << 3)
#define BQ5102X_FOD_OFFSET_VALUE_MASK (BIT(5) | BIT(4) | BIT(3))

#define BQ5102X_VRECT (0xE3)
#define BQ5102X_VOLTAGE (0xE4)
#define BQ5102X_MAILBOX (0xE0)

#define CONV_VOLTS(x) 	(x * 46)

#define VRECT_TH	3500
#define VRECT_HYSTERESIS	500

#define VRECT_MIN	2000


static struct i2c_client *bq5102x_client = NULL;

#ifdef _SMBUS_SUPPORT_
static int bq5102x_read_byte(struct i2c_client *client, int reg, u8 *val)
{
	s32 ret;

	if (unlikely(!client))
		return -1;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev, "i2c read fail: can't read from %02x: %d\n",reg, ret);
		return ret;
	} else {
		*val = ret;
	}
	return 0;
}

static int bq5102x_write_byte(struct i2c_client *client, int reg, u8 val)
{
	s32 ret;

	if (unlikely(!client))
		return -1;
	
	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		dev_err(&client->dev, "i2c write fail: can't write from %02x: %d\n",reg, ret);
		return ret;
	}

	return 0;
}
#else
static int bq5102x_read_byte(struct i2c_client *client, u8 reg, u8 *value)
{
	u8 buf[2];
	u8 val[2];
	struct i2c_msg xfer[2];

	int retry_cnt;

	if (unlikely(!client))
		return -1;

	for(retry_cnt = 2; retry_cnt > 0; retry_cnt--)
	{
		buf[0] = reg & 0xff;

		xfer[0].addr = 	client->addr;
		xfer[0].flags = 0;
		xfer[0].len = 1;
		xfer[0].buf = buf;

		xfer[1].addr = client->addr;
		xfer[1].flags = I2C_M_RD;
		xfer[1].len = 1;
		xfer[1].buf = val;

		if(i2c_transfer(client->adapter, xfer, 2) == 2) {
			*value = val[0];
			return 0;
		}
	}

	return -1;
}

static int bq5102x_write_byte(u8 reg, u8 value)
{
	u8 buf[2];
	struct i2c_msg xfer;
	int retry_cnt;

	if (unlikely(!client))
		return -1;

	for(retry_cnt = 2; retry_cnt > 0; retry_cnt--)
	{
		buf[0] = reg & 0xff;
		buf[1] = value;

		xfer.addr = 	client->addr;
		xfer.flags = 0;
		xfer.len = 2;
		xfer.buf = buf;

		if(i2c_transfer(client->adapter, &xfer, 1) == 1) {
			return 0;
		}
	}

	return -1;
}
#endif

#ifdef CONFIG_CHARGER_UNIFIED_WLC_ALIGNMENT_BQ5102X
static int bq5102x_get_vrect(void)
{
	u8 val;

	if(bq5102x_read_byte(bq5102x_client,
					BQ5102X_VRECT, &val) == 0) {
		return CONV_VOLTS(val);
	}

	return -1;
}

static int last_vrect = 0;
static int last_alignment = 0;

static int bq5102x_calc_alignment(int vrect)
{
	switch (last_alignment) {
		case 0:
		{
			if(vrect > VRECT_TH) {
				last_vrect = vrect;
				last_alignment = 2;
			} else if (vrect > VRECT_MIN) {
				last_vrect = vrect;
				last_alignment = 1;
			} else {
				last_vrect = vrect;
				last_alignment = 0;
			}
		}
		break;
		case 1:
		{
			if(vrect > (VRECT_TH+VRECT_HYSTERESIS)) {
				last_vrect = vrect;
				last_alignment = 2;
			} else if (vrect < VRECT_MIN) {
				last_vrect = vrect;
				last_alignment = 0;
			}
		}
		break;
		case 2:
		{
			if(vrect < (VRECT_TH-VRECT_HYSTERESIS)) {
				last_vrect = vrect;
				last_alignment = 1;
			}
		}
		break;
		default:
			break;
	}

	return last_alignment;
}

static int bq5102x_get_alignment(void)
{
	int vrect = 0;

	vrect = bq5102x_get_vrect();
	if(vrect > 0) {
		return bq5102x_calc_alignment(vrect);
	}

	return -1;
}

int bq5102x_align_start(void)
{
	last_vrect = 0;
	last_alignment = 0;
#if 0 // don't use ALIGN Mailer (I2C Mailbox Regisger B3 BIT)
	bq5102x_write_byte(bq5102x_client, BQ5102X_MAILBOX, BQ5102X_MAILBOX_ALIGN_AID_MODE);
#endif

	if (unlikely(!bq5102x_client)) {
		return -EINVAL;
	}
	
	return bq5102x_get_alignment();
}

int bq5102x_align_stop(void)
{
	last_vrect = 0;
	last_alignment = 0;

	return 0;
}

int bq5102x_align_get_value(void)
{
	if (unlikely(!bq5102x_client)) {
		return -EINVAL;
	}

	return bq5102x_get_alignment();
}
#endif

int bq5102x_get_output_voltage(void)
{
	int ret;
	u8 output_voltage;

	if(bq5102x_client == NULL) {
		pr_err("bq5102x_get_output_voltage is failed : bq5102x_client is NULL\n");
		return -EINVAL;
	}

	ret = bq5102x_read_byte(bq5102x_client, BQ5102X_CURRENT_REGISTER1_REG,
		&output_voltage);
	if(ret < 0) {
		pr_err("bq5102x_get_output_voltage is failed : bq5102x_read_byte is failed\n");
		return ret;
	}
	output_voltage = output_voltage & BQ5102X_OUTPUT_VOLTAGE_MASK;

	pr_info("%s : get output_voltage to 0x%x\n",
		__func__, output_voltage);

	return output_voltage;
}

int bq5102x_set_output_voltage(int output_voltage)
{
	int ret;

	if(bq5102x_client == NULL) {
		pr_err("bq5102x_set_output_voltage is failed : bq5102x_client is NULL\n");
		return -EINVAL;
	}

	ret = bq5102x_write_byte(bq5102x_client, BQ5102X_CURRENT_REGISTER1_REG,
		output_voltage);
	if(ret < 0) {
		pr_err("bq5102x_set_output_voltage is failed : bq5102x_write_byte is failed\n");
		return ret;
	}

	pr_info("%s : set output_voltage to 0x%x\n",
		__func__, output_voltage);

	return 0;
}

int bq5102x_get_output_current(void)
{
	int ret;
	u8 output_current;

	if(bq5102x_client == NULL) {
		pr_err("bq5102x_get_output_current is failed : bq5102x_client is NULL\n");
		return -EINVAL;
	}

	ret = bq5102x_read_byte(bq5102x_client, BQ5102X_CURRENT_REGISTER2_REG,
		&output_current);
	if(ret < 0) {
		pr_err("bq5102x_get_output_current is failed : bq5102x_read_byte is failed\n");
		return ret;
	}
	output_current = output_current & BQ5102X_OUTPUT_CURRENT_MASK;

	pr_info("%s : get output_current to 0x%x\n",
		__func__, output_current);

	return output_current;
}

int bq5102x_set_output_current(int output_current)
{
	int ret;

	if(bq5102x_client == NULL) {
		pr_err("bq5102x_set_output_voltage is failed : bq5102x_client is NULL\n");
		return -EINVAL;
	}

	ret = bq5102x_write_byte(bq5102x_client, BQ5102X_CURRENT_REGISTER2_REG,
		output_current);
	if(ret < 0) {
		pr_err("bq5102x_set_output_current is failed : bq5102x_write_byte is failed\n");
		return ret;
	}

	pr_info("%s : set output_current to 0x%x\n",
		__func__, output_current);

	return 0;
}

int bq5102x_get_fod_calibration(void)
{
	int ret;
	u8 fod_offset;

	if(bq5102x_client == NULL) {
		pr_err("bq5102x_get_fod_calibration is failed : bq5102x_client is NULL\n");
		return -EINVAL;
	}

	ret = bq5102x_read_byte(bq5102x_client, BQ5102X_FOD_RAM_REG,
		&fod_offset);
	if(ret < 0) {
		pr_err("bq5102x_get_fod_calibration is failed : bq5102x_read_byte is failed\n");
		return ret;
	}
	fod_offset = (fod_offset & BQ5102X_FOD_OFFSET_VALUE_MASK) >> 3;

	pr_info("%s : get FOD calibration offset to 0x%x\n",
		__func__, fod_offset);

	return fod_offset;
}

int bq5102x_set_fod_calibration(int fod_offset)
{
	int ret;

	if(bq5102x_client == NULL) {
		pr_err("bq5102x_set_fod_calibration is failed : bq5102x_client is NULL\n");
		return -EINVAL;
	}

	ret = bq5102x_write_byte(bq5102x_client, BQ5102X_FOD_RAM_REG,
		(BQ5102X_FOD_OFFSET_ENABLE | (fod_offset << 3)));
	if(ret < 0) {
		pr_err("bq5102x_set_fod_calibration is failed : bq5102x_write_byte is failed\n");
		return ret;
	}

	pr_info("%s : set FOD calibration offset to 0x%x\n",
		__func__, fod_offset);

	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id ti_bq5102x_table[] = {
		{ .compatible = "ti,bq5102x", },
		{},
};
#else
#define ti_bq5102x_table NULL
#endif

static const struct i2c_device_id ti_id[] = {
	{"bq5102x", 0},
	{}
};
static int bq5102x_parse_dt(struct device_node *dev_node)
{
	int ret = 0;

	return ret;
}
static int __devinit bq5102x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device_node *dev_node = client->dev.of_node;
	int ret;

	pr_err("%s:\n", __func__);

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		pr_err("i2c func fail.\n");
		return -EIO;
	}
	bq5102x_client = client;
	/* need dts parser */
	if (dev_node) {
		ret = bq5102x_parse_dt(dev_node);
	}
	i2c_set_clientdata(client, NULL);
	return 0;
}

static struct i2c_driver bq5102x_driver = {
	.driver = {
		.name = "bq5102x",
		.owner = THIS_MODULE,
		.of_match_table = ti_bq5102x_table,
	},
	.probe = bq5102x_probe,
	.id_table = ti_id,
};


module_i2c_driver(bq5102x_driver);

MODULE_DESCRIPTION("BQ5102X");
MODULE_LICENSE("GPL V2");
#endif

