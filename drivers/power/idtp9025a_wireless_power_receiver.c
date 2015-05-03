/*
 * IDPT9025A Wireless Power Receiver driver
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



#ifdef CONFIG_CHARGER_UNIFIED_WLC_ALIGNMENT_IDT9025A 
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/power_supply.h>
#include <linux/power/unified_wireless_charger.h>
#include <linux/power/unified_wireless_charger_alignment.h>


#define DEVICE_NAME		"idtp9025a"
#define _SMBUS_SUPPORT_

#define IDTP9025A_VOLTAGE (0x40)
#define IDTP9025A_CURRENT (0x42)
#define IDTP9025A_FREQ (0x44)

#define CONV_VOLTS(x) 	((x * 5 * 1800) / 256)
#define CONV_CURRENT(x) 	((x * 1800) / 256)
#define CONV_FREQ(x) 		((16384 / x))

#define OUTPUT_CURRENT_TH	512

#define FREQ_TH_1	157
#define FREQ_HYSTERESIS_UP_1	3
#define FREQ_HYSTERESIS_DOWN_1	3

#define FREQ_TH_2	147
#define FREQ_HYSTERESIS_UP_2	3
#define FREQ_HYSTERESIS_DOWN_2	3


#define FREQ_MIN	100

static struct i2c_client *idtp9025_client = NULL;

enum {
	WLC_ALIGNMENT_LEVEL_NC = 0,
	WLC_ALIGNMENT_LEVEL_1,
	WLC_ALIGNMENT_LEVEL_2,
};

static int last_freq = 0;
static int last_current = 0;
static int last_alignment = WLC_ALIGNMENT_LEVEL_NC;

#ifdef _SMBUS_SUPPORT_
static int idtp9025_read_byte(struct i2c_client *client, int reg, u8 *val)
{
	s32 ret;

	if (unlikely(!idtp9025_client))
		return -1;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev, "i2c read fail: can't read from %02x: %d\n",
			reg, ret);
		return ret;
	} else {
		*val = ret;
	}
	return 0;
}
#else
static int idtp9025_read_byte(struct i2c_client *client, u8 reg, u8 *value)
{
	u8 buf[2];
	u8 val[2];
	struct i2c_msg xfer[2];
	
	int retry_cnt;

	if (unlikely(!idtp9025_client))
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
#endif

static int idtp9025_get_current(void)
{
	u8 val;

	if(idtp9025_read_byte(idtp9025_client,
					IDTP9025A_CURRENT, &val) == 0) {
		return CONV_CURRENT(val);
	}

	return -1;
}

static int idtp9025_get_frequency(void)
{
	u8 val;

	if(idtp9025_read_byte(idtp9025_client,
					IDTP9025A_FREQ, &val) == 0) {
		return CONV_FREQ(val);
	}

	return -1;
}



static int idtp9025_calc_alignment(int freq, int curr)
{
	int freq_th = 0;
	int freq_hystersis_up = 0;
	int freq_hystersis_down = 0;	

	if(curr < OUTPUT_CURRENT_TH) {
		freq_th = FREQ_TH_1;
		freq_hystersis_up = FREQ_HYSTERESIS_UP_1;
		freq_hystersis_down = FREQ_HYSTERESIS_DOWN_1;			
	} else {
		freq_th = FREQ_TH_2;
		freq_hystersis_up = FREQ_HYSTERESIS_UP_2;
		freq_hystersis_down = FREQ_HYSTERESIS_DOWN_2;		
	}

	switch(last_alignment) {
		case WLC_ALIGNMENT_LEVEL_NC:
			{
				if(freq > freq_th) {
					last_freq = freq;
					last_current = curr;
					last_alignment = WLC_ALIGNMENT_LEVEL_2;
				} else if(freq > FREQ_MIN) {
					last_freq = freq;
					last_current = curr;
					last_alignment = WLC_ALIGNMENT_LEVEL_1;
				} else {
					last_freq = freq;
					last_current = curr;
					last_alignment = WLC_ALIGNMENT_LEVEL_NC;
				}
			}
			break;
		case WLC_ALIGNMENT_LEVEL_1:
			{
				if(freq > (freq_th+freq_hystersis_up)) {
					last_freq = freq;
					last_current = curr;
					last_alignment = WLC_ALIGNMENT_LEVEL_2;
				} else if(freq < FREQ_MIN) {
					last_freq = freq;
					last_current = curr;
					last_alignment = WLC_ALIGNMENT_LEVEL_NC;
				}
			}
			break;
		case WLC_ALIGNMENT_LEVEL_2:
			{
				if(freq < FREQ_MIN) {
					last_freq = freq;
					last_current = curr;
					last_alignment = WLC_ALIGNMENT_LEVEL_NC;
				} else if(freq <= (freq_th-freq_hystersis_down)) {
					last_freq = freq;
					last_current = curr;
					last_alignment = WLC_ALIGNMENT_LEVEL_1;
				}
			}
			break;
		default:
			break;
	}

	return last_alignment;
}

static int idtp9025_get_alignment(void)
{
	int freq = 0;
	int curr = 0;
	int align = 0;

	freq = idtp9025_get_frequency();
	if(freq > 0) {
		curr = idtp9025_get_current();
		if(curr > 0) {
			align = idtp9025_calc_alignment(freq, curr);
#ifdef WLC_ALIGN_DEBUG			
			pr_err("freq: %d curr: %d align: %d\n", freq, curr, align);
#endif
			return align;
		}
	}
#ifdef WLC_ALIGN_DEBUG			
	pr_err("freq: %d curr: %d align: %d\n", freq, curr, align);
#endif

	return -1;
}

int idtp9025_align_start(void)
{
	last_freq = 0;
	last_current = 0;
	last_alignment = WLC_ALIGNMENT_LEVEL_NC;

	if (unlikely(!idtp9025_client)) {
		return -EINVAL;
	}

	return idtp9025_get_alignment();
}

int idtp9025_align_stop(void)
{
	last_freq = 0;
	last_current = 0;
	last_alignment = WLC_ALIGNMENT_LEVEL_NC;

	return 0;
}

int idtp9025_align_get_value(void)
{
	if (unlikely(!idtp9025_client)) {
		return -EINVAL;
	}

	return idtp9025_get_alignment();
}


#ifdef CONFIG_OF
static struct of_device_id idt_idtp9025_table[] = {
		{ .compatible = "idt,idtp9025a", },
		{},
};
#else
#define idt_idtp9025_table NULL
#endif

static const struct i2c_device_id idt_id[] = {
	{"idtp9025a", 0},
	{}
};
static int idtp9025a_parse_dt(struct device_node *dev_node)
{
	int ret = 0;

	return ret;
}
static int __devinit idtp9025_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device_node *dev_node = client->dev.of_node;
	int ret;
	
	pr_err("%s:\n", __func__);
	
	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		pr_err("i2c func fail.\n");
		return -EIO;
	}
	idtp9025_client = client;
	/* need dts parser */	
	if (dev_node) {
		ret = idtp9025a_parse_dt(dev_node);
	}
	i2c_set_clientdata(client, NULL);
	return 0;
}

static struct i2c_driver idtp9025_driver = {
	.driver = {
		.name = "idtp9025a",
		.owner = THIS_MODULE,
		.of_match_table = idt_idtp9025_table,
	},
	.probe = idtp9025_probe,
	.id_table = idt_id,
};


module_i2c_driver(idtp9025_driver);

MODULE_DESCRIPTION("IDTP9025");
MODULE_LICENSE("GPL V2");
#endif
