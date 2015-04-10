/* Touch_synaptics_proximity.c
 *
 * Copyright (C) 2014 LGE.
 *
 * Author: daehyun.gil@lge.com
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>

#include "lge_touch_core.h"
#include "touch_synaptics.h"
//#include <linux/input/lge_touch_core.h>
//#include <linux/input/touch_synaptics.h>

#define PAGES_TO_SERVICE	(10)
#define PDT_START			(0x00E9)
#define PDT_END				(0x000A)
#define PDT_ENTRY_SIZE		(0x0006)

#define SYNAPTICS_TS_F01	(0x01)
#define SYNAPTICS_TS_F11	(0x11)
#define SYNAPTICS_TS_F12	(0x12)
#define SYNAPTICS_TS_F1A	(0x1A)
#define SYNAPTICS_TS_F34	(0x34)
#define SYNAPTICS_TS_F51	(0x51)
#define SYNAPTICS_TS_F54	(0x54)

#define MASK_16BIT			(0xFFFF)
#define MASK_8BIT			(0xFF)
#define MASK_7BIT			(0x7F)
#define MASK_6BIT			(0x3F)
#define MASK_5BIT			(0x1F)
#define MASK_4BIT			(0x0F)
#define MASK_3BIT			(0x07)
#define MASK_2BIT			(0x03)
#define MASK_1BIT			(0x01)

#define HOVERING_FINGER_EN	(1 << 4)
#define HOVER_Z_MAX			(255)

#define TOUCH_PROX_MSG(fmt, args...) \
	printk(KERN_INFO "[Touch Proximity] %s: " fmt, __func__, ##args);


struct synaptics_ts_fn_desc {
	union {
		struct {
			unsigned char query_base_addr;
			unsigned char cmd_base_addr;
			unsigned char ctrl_base_addr;
			unsigned char data_base_addr;
			unsigned char intr_src_count;
			unsigned char fn_number;
		};
		unsigned char data[6];
	};
};

struct prox_finger_data {
	union {
		struct {
			unsigned char object_type_and_status;
			unsigned char x_lsb;
			unsigned char x_msb;
			unsigned char y_lsb;
			unsigned char y_msb;
			unsigned char z;
		} __packed;
		unsigned char proximity_data[6];
	};
};

struct synaptics_ts_prox_handle {
	bool hover_finger_present;
	bool hover_finger_en;
	unsigned char intr_mask;
	unsigned short query_base_addr;
	unsigned short control_base_addr;
	unsigned short data_base_addr;
	unsigned short command_base_addr;
	unsigned short hover_finger_en_addr;
	unsigned short hover_finger_data_addr;
	unsigned short interrupt_enable_0_addr;
	struct input_dev *prox_dev;
	struct prox_finger_data *finger_data;
	struct synaptics_ts_data *ts_data;
};

static struct synaptics_ts_prox_handle *prox;

DECLARE_COMPLETION(prox_remove_complete);

static int prox_i2c_read(struct synaptics_ts_data *ts, unsigned short addr, unsigned char *data, unsigned short length)
{
	int retval;
	unsigned char page_old = 0;
	unsigned char page_new = 0;
	bool page_changed;

/* page read */
	retval = touch_i2c_read(ts->client, PAGE_SELECT_REG, sizeof(page_old), &page_old);

	if (retval < 0) {
		TOUCH_PROX_MSG("Failed to read from Page_Select register\n");
		return retval;
	}

	page_new = (addr >> 8);

/* page compare & change */
	if (page_old == page_new)
		page_changed = false;
	else
	{
		retval = touch_i2c_write_byte(ts->client, PAGE_SELECT_REG, page_new);

		if (retval < 0) {
			TOUCH_PROX_MSG("Failed to change Page_Select register\n");
			return retval;
		}

		page_changed = true;
	}

/* read register */
	retval = touch_i2c_read(ts->client, (unsigned char)(addr & ~(MASK_8BIT << 8)), length, data);

	if (retval < 0) {
		TOUCH_PROX_MSG("Failed to read from the register(addr=0x%04x)\n", addr);
		return retval;
	}

/* page restore */
	if (page_changed) {
		retval = touch_i2c_write_byte(ts->client, PAGE_SELECT_REG, page_old);

		if (retval < 0) {
			TOUCH_PROX_MSG("Failed to restore Page_Select register\n");
			return retval;
		}
	}

	return 0;
}

static int prox_i2c_write(struct synaptics_ts_data *ts, unsigned short addr, unsigned char *data, unsigned short length)
{
	int retval;
	unsigned char page_old = 0;
	unsigned char page_new = 0;
	bool page_changed;

/* page read */
	retval = touch_i2c_read(ts->client, PAGE_SELECT_REG, sizeof(page_old), &page_old);

	if (retval < 0) {
		TOUCH_PROX_MSG("Failed to read from Page_Select register\n");
		return retval;
	}

	page_new = (addr >> 8);

/* page compare & change */
	if (page_old == page_new)
		page_changed = false;
	else
	{
		retval = touch_i2c_write_byte(ts->client, PAGE_SELECT_REG, page_new);

		if (retval < 0) {
			TOUCH_PROX_MSG("Failed to change Page_Select register\n");
			return retval;
		}

		page_changed = true;
	}

/* write register */
	retval = touch_i2c_write(ts->client, (unsigned char)(addr & ~(MASK_8BIT << 8)), length, data);

	if (retval < 0) {
		TOUCH_PROX_MSG("Failed to write to the register(addr=0x%04x)\n", addr);
		return retval;
	}

/* page restore */
	if (page_changed) {
		retval = touch_i2c_write_byte(ts->client, PAGE_SELECT_REG, page_old);

		if (retval < 0) {
			TOUCH_PROX_MSG("Failed to restore Page_Select register\n");
			return retval;
		}
	}

	return 0;
}

static int prox_reg_init(void)
{
	int retval;
	unsigned char ctrl_23_offset;
	unsigned char data_01_offset;
	struct synaptics_ts_f12_query_5 query_5;
	struct synaptics_ts_f12_query_8 query_8;
	struct synaptics_ts_data *ts = prox->ts_data;

	TOUCH_PROX_MSG("\n");

/* hover_finger_en_addr setting */
	retval = prox_i2c_read(ts, (prox->query_base_addr + 5), query_5.data, sizeof(query_5.data));

	if (retval < 0) {
		TOUCH_PROX_MSG("Failed to read from F12_2D_QUERY_05_Control_Presence register\n");
		return retval;
	}

	ctrl_23_offset = query_5.ctrl_00_is_present + query_5.ctrl_01_is_present + query_5.ctrl_02_is_present +
					query_5.ctrl_03_is_present + query_5.ctrl_04_is_present + query_5.ctrl_05_is_present +
					query_5.ctrl_06_is_present + query_5.ctrl_07_is_present + query_5.ctrl_08_is_present +
					query_5.ctrl_09_is_present + query_5.ctrl_10_is_present + query_5.ctrl_11_is_present +
					query_5.ctrl_12_is_present + query_5.ctrl_13_is_present + query_5.ctrl_14_is_present +
					query_5.ctrl_15_is_present + query_5.ctrl_16_is_present + query_5.ctrl_17_is_present +
					query_5.ctrl_18_is_present + query_5.ctrl_19_is_present + query_5.ctrl_20_is_present +
					query_5.ctrl_21_is_present + query_5.ctrl_22_is_present;

	prox->hover_finger_en_addr = prox->control_base_addr + ctrl_23_offset;

	TOUCH_PROX_MSG("prox->hover_finger_en_addr=0x%04X\n", prox->hover_finger_en_addr);

/* hover_finger_data_addr setting */
	retval = prox_i2c_read(ts, (prox->query_base_addr + 8), query_8.data, sizeof(query_8.data));

	if (retval < 0) {
		TOUCH_PROX_MSG("Failed to read from F12_2D_QUERY_08_Data_Presence register\n");
		return retval;
	}

	data_01_offset = query_8.data_00_is_present;

	prox->hover_finger_data_addr = prox->data_base_addr + data_01_offset;

	TOUCH_PROX_MSG("prox->hover_finger_data_addr=0x%04X\n", prox->hover_finger_data_addr);

	return 0;
}

static int prox_scan_pdt(void)
{
	int retval;
	unsigned char ii;
	unsigned char page;
	unsigned char intr_count = 0;
	unsigned char intr_off;
	unsigned char intr_src;
	unsigned char interrupt_enable_0 = 0;
	unsigned short addr;
	struct synaptics_ts_fn_desc fd;
	struct synaptics_ts_data *ts = prox->ts_data;

	TOUCH_PROX_MSG("\n");

	for (page = 0; page < PAGES_TO_SERVICE; page++) {
		for (addr = PDT_START; addr > PDT_END; addr -= PDT_ENTRY_SIZE) {
			addr |= (page << 8);

			retval = prox_i2c_read(ts, addr, fd.data, sizeof(fd.data));

			if (retval < 0) {
				TOUCH_PROX_MSG("Failed to read from register\n");
				return retval;
			}

			addr &= ~(MASK_8BIT << 8);

			if (fd.fn_number) {
				TOUCH_PROX_MSG("Found F%02X\n", fd.fn_number);

				switch (fd.fn_number) {
					case SYNAPTICS_TS_F01:
						prox->interrupt_enable_0_addr = (fd.ctrl_base_addr + 1) | (page << 8);
						break;
					case SYNAPTICS_TS_F12:
						goto f12_found;
						break;
				}
			} else {
				break;
			}

			intr_count += (fd.intr_src_count & MASK_3BIT);
		}
	}

	TOUCH_PROX_MSG("Failed to find F12\n");

	return -EINVAL;

f12_found:
	prox->query_base_addr = fd.query_base_addr | (page << 8);
	prox->control_base_addr = fd.ctrl_base_addr | (page << 8);
	prox->data_base_addr = fd.data_base_addr | (page << 8);
	prox->command_base_addr = fd.cmd_base_addr | (page << 8);

	TOUCH_PROX_MSG("prox->query_base_addr=0x%04X\n", prox->query_base_addr);
	TOUCH_PROX_MSG("prox->control_base_addr=0x%04X\n", prox->control_base_addr);
	TOUCH_PROX_MSG("prox->data_base_addr=0x%04X\n", prox->data_base_addr);
	TOUCH_PROX_MSG("prox->command_base_addr=0x%04X\n", prox->command_base_addr);
	TOUCH_PROX_MSG("prox->interrupt_enable_0_addr=0x%04X\n", prox->interrupt_enable_0_addr);

	retval = prox_reg_init();

	if (retval < 0) {
		TOUCH_PROX_MSG("Failed to initialize proximity registers\n");
		return retval;
	}

	prox->intr_mask = 0;

	intr_src = fd.intr_src_count;
	intr_off = intr_count % 8;

	for (ii = intr_off; ii < ((intr_src & MASK_3BIT) + intr_off); ii++) {
		prox->intr_mask |= 1 << ii;
	}

	TOUCH_PROX_MSG("prox->intr_mask=0x%02X\n", prox->intr_mask);

	retval = prox_i2c_read(ts, prox->interrupt_enable_0_addr, &interrupt_enable_0, sizeof(interrupt_enable_0));

	if (retval < 0) {
		TOUCH_PROX_MSG("Failed to read from interrupt_enable_0 register\n");
		return retval;
	}

	interrupt_enable_0 |= prox->intr_mask;

	retval = prox_i2c_write(ts, prox->interrupt_enable_0_addr, &interrupt_enable_0, sizeof(interrupt_enable_0));

	if (retval < 0) {
		TOUCH_PROX_MSG("Failed to write to interrupt_enable_0 register\n");
		return retval;
	}

	retval = prox_i2c_read(ts, prox->interrupt_enable_0_addr, &interrupt_enable_0, sizeof(interrupt_enable_0));

	if (retval < 0) {
		TOUCH_PROX_MSG("Failed to read from interrupt_enable_0 register\n");
		return retval;
	}

	return 0;
}

static int prox_set_hover_finger_en(bool enable)
{
	int retval;
	unsigned char object_report_enable;
	struct synaptics_ts_data *ts = prox->ts_data;

	TOUCH_PROX_MSG("\n");

	retval = prox_i2c_read(ts, prox->hover_finger_en_addr, &object_report_enable, sizeof(object_report_enable));

	if (retval < 0) {
		TOUCH_PROX_MSG("Failed to read from object_report_enable register\n");
		return retval;
	}

	if (enable == prox->hover_finger_en) {
		TOUCH_PROX_MSG("enable(%d) and prox->hover_finger_en(%d) values are equal.\n", enable, prox->hover_finger_en);
		return retval;
	}

	if (enable)
		object_report_enable |= HOVERING_FINGER_EN;
	else
		object_report_enable &= ~HOVERING_FINGER_EN;

	retval = prox_i2c_write(ts, prox->hover_finger_en_addr, &object_report_enable, sizeof(object_report_enable));

	if (retval < 0) {
		TOUCH_PROX_MSG("Failed to write to object_report_enable register\n");
		return retval;
	}

	retval = prox_i2c_read(ts, prox->hover_finger_en_addr, &object_report_enable, sizeof(object_report_enable));

	if (retval < 0) {
		TOUCH_PROX_MSG("Failed to read from object_report_enable register\n");
		return retval;
	}

	prox->hover_finger_en = object_report_enable & HOVERING_FINGER_EN;

	return 0;
}

static void prox_set_params(void)
{
	TOUCH_PROX_MSG("\n");

	input_set_abs_params(prox->prox_dev, ABS_X, 0, (int)prox->ts_data->pdata->caps->max_x, 0, 0);
	input_set_abs_params(prox->prox_dev, ABS_Y, 0, (int)prox->ts_data->pdata->caps->max_y, 0, 0);
	input_set_abs_params(prox->prox_dev, ABS_DISTANCE, 0, (int)HOVER_Z_MAX, 0, 0);

	return;
}

static void prox_hover_finger_lift(void)
{
	TOUCH_PROX_MSG("\n");

	input_report_key(prox->prox_dev, BTN_TOUCH, 0);
	input_report_key(prox->prox_dev, BTN_TOOL_FINGER, 0);

	input_sync(prox->prox_dev);

	prox->hover_finger_present = false;

	return;
}

static void prox_hover_finger_report(void)
{
	int retval;
	int x;
	int y;
	int z;
	struct prox_finger_data *data;
	struct synaptics_ts_data *ts = prox->ts_data;

	data = prox->finger_data;

	retval = prox_i2c_read(ts, prox->hover_finger_data_addr, data->proximity_data, sizeof(data->proximity_data));

	if (retval < 0) {
		TOUCH_PROX_MSG("Failed to read hovering finger data\n");
		return;
	}

	if (data->object_type_and_status != F12_HOVERING_FINGER_STATUS) {
		if (prox->hover_finger_present)
			prox_hover_finger_lift();

		return;
	}

	x = (data->x_msb << 8) | (data->x_lsb);
	y = (data->y_msb << 8) | (data->y_lsb);
	z = HOVER_Z_MAX - data->z;

	input_report_key(prox->prox_dev, BTN_TOUCH, 0);
	input_report_key(prox->prox_dev, BTN_TOOL_FINGER, 1);
	input_report_abs(prox->prox_dev, ABS_X, x);
	input_report_abs(prox->prox_dev, ABS_Y, y);
	input_report_abs(prox->prox_dev, ABS_DISTANCE, z);

	input_sync(prox->prox_dev);

	prox->hover_finger_present = true;

	return;
}

static int synaptics_ts_prox_init(struct synaptics_ts_data *ts)
{
	int retval;

	TOUCH_PROX_MSG("\n");

	if (!ts) {
		TOUCH_PROX_MSG("ts points to NULL\n");
		retval = -EACCES;
		goto exit;
	}

	prox = kzalloc(sizeof(*prox), GFP_KERNEL);

	if (!prox) {
		TOUCH_PROX_MSG("Failed to alloc mem for prox\n");
		retval = -ENOMEM;
		goto exit;
	}

	prox->ts_data = ts;

	prox->finger_data = kzalloc(sizeof(*(prox->finger_data)), GFP_KERNEL);

	if(!prox->finger_data) {
		TOUCH_PROX_MSG("Failed to alloc mem for finger_data\n");
		retval = -ENOMEM;
		goto exit_free_prox;
	}

	retval = prox_scan_pdt();

	if (retval < 0) {
		TOUCH_PROX_MSG("Failed to scan pdt\n");
		goto exit_free_finger_data;
	}

	retval = prox_set_hover_finger_en(true);

	if (retval < 0) {
		TOUCH_PROX_MSG("Failed to set hover_finger_enable\n");
		goto exit_free_finger_data;
	}

	prox->prox_dev = input_allocate_device();

	if (!prox->prox_dev) {
		TOUCH_PROX_MSG("Failed to alloc proximity device\n");
		retval = -ENOMEM;
		goto exit_free_finger_data;
	}

	prox->prox_dev->name = "touch_prox_dev";

	set_bit(EV_KEY, prox->prox_dev->evbit);
	set_bit(EV_ABS, prox->prox_dev->evbit);
	set_bit(BTN_TOUCH, prox->prox_dev->keybit);
	set_bit(BTN_TOOL_FINGER, prox->prox_dev->keybit);
	set_bit(INPUT_PROP_DIRECT, prox->prox_dev->propbit);

	prox_set_params();

	retval = input_register_device(prox->prox_dev);

	if (retval) {
		TOUCH_PROX_MSG("Failed to register proximity device\n");
		goto exit_free_input_device;
	}

	return 0;

exit_free_input_device:
	if (prox->prox_dev)
		input_free_device(prox->prox_dev);

exit_free_finger_data:
	    kfree(prox->finger_data);

exit_free_prox:
	kfree(prox);
	prox = NULL;

exit:
	return retval;
}

static void synaptics_ts_prox_remove(struct synaptics_ts_data *ts)
{
	TOUCH_PROX_MSG("\n");

	if (!prox)
		goto exit;

	input_unregister_device(prox->prox_dev);
	kfree(prox->finger_data);
	kfree(prox);
	prox = NULL;

exit:
	complete(&prox_remove_complete);

	return;
}

static void synaptics_ts_prox_reset(struct synaptics_ts_data *ts)
{
	TOUCH_PROX_MSG("\n");

	if (!prox) {
		synaptics_ts_prox_init(ts);
		return;
	}

	prox_hover_finger_lift();

	prox_scan_pdt();

	prox_set_hover_finger_en(true);

	prox_set_params();

	return;
}

static void synaptics_ts_prox_reinit(struct synaptics_ts_data *ts)
{
	TOUCH_PROX_MSG("\n");

	if (!prox)
		return;

	prox_hover_finger_lift();

	prox_set_hover_finger_en(true);

	return;
}

static void synaptics_ts_prox_early_suspend(struct synaptics_ts_data *ts)
{
	TOUCH_PROX_MSG("\n");

	if (!prox)
		return;

	prox_set_hover_finger_en(false);

	prox_hover_finger_lift();

	return;
}

static void synaptics_ts_prox_suspend(struct synaptics_ts_data *ts)
{
	TOUCH_PROX_MSG("\n");

	if (!prox)
		return;

	prox_set_hover_finger_en(false);

	prox_hover_finger_lift();

	return;
}

static void synaptics_ts_prox_resume(struct synaptics_ts_data *ts)
{
	TOUCH_PROX_MSG("\n");

	if (!prox)
		return;

	prox_hover_finger_lift();

	prox_set_hover_finger_en(true);

	return;
}

static void synaptics_ts_prox_late_resume(struct synaptics_ts_data *ts)
{
	TOUCH_PROX_MSG("\n");

	if (!prox)
		return;

	prox_hover_finger_lift();

	prox_set_hover_finger_en(true);

	return;
}

static void synaptics_ts_prox_attn(unsigned char intr_status_reg)
{
	if (!prox)
		return;

	if (prox->intr_mask & intr_status_reg)
		prox_hover_finger_report();

	return;
}

static struct synaptics_ts_exp_fn proximity_module = {
	.init = synaptics_ts_prox_init,
	.remove = synaptics_ts_prox_remove,
	.reset = synaptics_ts_prox_reset,
	.reinit = synaptics_ts_prox_reinit,
	.early_suspend = synaptics_ts_prox_early_suspend,
	.suspend = synaptics_ts_prox_suspend,
	.resume = synaptics_ts_prox_resume,
	.late_resume = synaptics_ts_prox_late_resume,
	.attn = synaptics_ts_prox_attn,
};


static int touch_proximity_module_init(void)
{
	TOUCH_PROX_MSG("\n");

	synaptics_ts_prox_function(&proximity_module, true);

	return 0;
}

static void touch_proximity_module_exit(void)
{
	TOUCH_PROX_MSG("\n");

	synaptics_ts_prox_function(&proximity_module, false);

	wait_for_completion(&prox_remove_complete);

	return;
}

module_init(touch_proximity_module_init);
module_exit(touch_proximity_module_exit);

MODULE_AUTHOR("daehyun.gil@lge.com");
MODULE_DESCRIPTION("LGE Touch Proximity Module");
MODULE_LICENSE("GPL");
