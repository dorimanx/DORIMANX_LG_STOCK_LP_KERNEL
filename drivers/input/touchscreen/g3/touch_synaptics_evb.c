/* Touch_synaptics.c
 *
 * Copyright (C) 2012 LGE.
 *
 * Author: yehan.ahn@lge.com, hyesung.shin@lge.com
 *
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

#include <linux/err.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/regulator/machine.h>
#include <linux/async.h>

#include "lge_touch_core.h"
#include "touch_synaptics_evb.h"

/* RMI4 spec from 511-000405-01 Rev.D
 * Function	Purpose				See page
 * $01		RMI Device Control			45
 * $1A		0-D capacitive button sensors	61
 * $05		Image Reporting			68
 * $07		Image Reporting			75
 * $08		BIST				82
 * $09		BIST				87
 * $11		2-D TouchPad sensors		93
 * $19		0-D capacitive button sensors	141
 * $30		GPIO/LEDs			148
 * $31		LEDs				162
 * $34		Flash Memory Management		163
 * $36		Auxiliary ADC			174
 * $54		Test Reporting			176
 */
#define RMI_DEVICE_CONTROL			0x01
#define TOUCHPAD_SENSORS			0x11
#define CAPACITIVE_BUTTON_SENSORS		0x1A
#define GPIO_LEDS				0x30
#define LEDS					0x31
#define ANALOG_CONTROL				0x54
#define TIMER					0x32
#define FLASH_MEMORY_MANAGEMENT			0x34
#define AUXILIARY_ADC				0x36

/* Register Map & Register bit mask
 * - Please check "One time" this map before using this device driver
 */
/* RMI_DEVICE_CONTROL */
#define MANUFACTURER_ID_REG			(ts->common_fc.dsc.query_base)			/* Manufacturer ID */
#define FW_REVISION_REG				(ts->common_fc.dsc.query_base+3)		/* FW revision */
#define PRODUCT_ID_REG				(ts->common_fc.dsc.query_base+11)		/* Product ID */

#define DEVICE_COMMAND_REG			(ts->common_fc.dsc.command_base)

#define DEVICE_CONTROL_REG 			(ts->common_fc.dsc.control_base)		/* Device Control */
#define DEVICE_CONTROL_NORMAL_OP		0x00	/* sleep mode : go to doze mode after 500 ms */
#define DEVICE_CONTROL_SLEEP 			0x01	/* sleep mode : go to sleep */
#define DEVICE_CONTROL_SPECIFIC			0x02	/* sleep mode : go to doze mode after 5 sec */
#define DEVICE_CONTROL_NOSLEEP			0x04
#define DEVICE_CONTROL_CONFIGURED		0x80
#define DEVICE_CHARGER_CONNECTED		0x20

#define INTERRUPT_ENABLE_REG			(ts->common_fc.dsc.control_base+1)		/* Interrupt Enable 0 */

#define DEVICE_STATUS_REG			(ts->common_fc.dsc.data_base)			/* Device Status */
#define DEVICE_FAILURE_MASK			0x03
#define DEVICE_CRC_ERROR_MASK			0x04
#define DEVICE_STATUS_FLASH_PROG		0x40
#define DEVICE_STATUS_UNCONFIGURED		0x80

#define INTERRUPT_STATUS_REG			(ts->common_fc.dsc.data_base+1)		/* Interrupt Status */
#define INTERRUPT_MASK_FLASH			0x01
#define INTERRUPT_MASK_STATUS			0x02
#define INTERRUPT_MASK_ABS0			0x04
#define INTERRUPT_MASK_BUTTON			0x10


/* TOUCHPAD_SENSORS */
#define FINGER_COMMAND_REG			(ts->finger_fc.dsc.command_base)

#define FINGER_STATE_REG			(ts->finger_fc.dsc.data_base)			/* Finger State */
#define FINGER_DATA_REG_START			(ts->finger_fc.dsc.data_base+3)		/* Finger Data Register */
#define FINGER_STATE_MASK			0x03
#define REG_X_POSITION				0
#define REG_Y_POSITION				1
#define REG_YX_POSITION				2
#define REG_WY_WX				3
#define REG_Z					4
#define TWO_D_EXTEND_STATUS			(ts->finger_fc.dsc.data_base+53)

#define TWO_D_REPORTING_MODE			(ts->finger_fc.dsc.control_base+0)		/* 2D Reporting Mode */

#define REPORT_BEYOND_CLIP			0x80
#define REPORT_MODE_CONTINUOUS			0x00
#define REPORT_MODE_REDUCED			0x01
#define ABS_FILTER				0x08
#define PALM_DETECT_REG 			(ts->finger_fc.dsc.control_base+1)		/* Palm Detect */
#define DELTA_X_THRESH_REG 			(ts->finger_fc.dsc.control_base+2)		/* Delta-X Thresh */
#define DELTA_Y_THRESH_REG 			(ts->finger_fc.dsc.control_base+3)		/* Delta-Y Thresh */
#define SENSOR_MAX_X_POS			(ts->finger_fc.dsc.control_base+6)		/* SensorMaxXPos */
#define SENSOR_MAX_Y_POS			(ts->finger_fc.dsc.control_base+8)		/* SensorMaxYPos */

/* CAPACITIVE_BUTTON_SENSORS */
#define BUTTON_COMMAND_REG			(ts->button_fc.dsc.command_base)
#define BUTTON_DATA_REG				(ts->button_fc.dsc.data_base)			/* Button Data */

#define MAX_NUM_OF_BUTTON			4

/* ANALOG_CONTROL */
#define ANALOG_COMMAND_REG			(ts->analog_fc.dsc.command_base)
#define FORCE_UPDATE				0x04

#define ANALOG_CONTROL_REG			(ts->analog_fc.dsc.control_base)
#define FORCE_FAST_RELAXATION			0x04

#define FAST_RELAXATION_RATE			(ts->analog_fc.dsc.control_base+16)

/* FLASH_MEMORY_MANAGEMENT */
#define FLASH_CONFIG_ID_REG			(ts->flash_fc.dsc.control_base)		/* Flash Control */
#define FLASH_CONTROL_REG			(ts->flash_fc.dsc.data_base+18)
#define FLASH_STATUS_MASK			0xF0

/* Page number */
#define COMMON_PAGE				(ts->common_fc.function_page)
#define FINGER_PAGE				(ts->finger_fc.function_page)
#define BUTTON_PAGE				(ts->button_fc.function_page)
#define ANALOG_PAGE				(ts->analog_fc.function_page)
#define FLASH_PAGE				(ts->flash_fc.function_page)
#define DEFAULT_PAGE				0x00

#define SMALL_OBJECT_DETECTION_TUNNING_REG	(ts->finger_fc.dsc.control_base+45) //0x0083
#define SMALL_OBJECT_DETECTION			0x04

/* Get user-finger-data from register.
 */
#define TS_SNTS_GET_X_POSITION(_high_reg, _low_reg) \
		( ((u16)((_high_reg << 4) & 0x0FF0) | (u16)(_low_reg&0x0F)))
#define TS_SNTS_GET_Y_POSITION(_high_reg, _low_reg) \
		( ((u16)((_high_reg << 4) & 0x0FF0) | (u16)((_low_reg >> 4) & 0x0F)))
#define TS_SNTS_GET_WIDTH_MAJOR(_width) \
		((((_width & 0xF0) >> 4) - (_width & 0x0F)) > 0) ? (_width & 0xF0) >> 4 : _width & 0x0F
#define TS_SNTS_GET_WIDTH_MINOR(_width) \
		((((_width & 0xF0) >> 4) - (_width & 0x0F)) > 0) ? _width & 0x0F : (_width & 0xF0) >> 4
#define TS_SNTS_GET_ORIENTATION(_width) \
		((((_width & 0xF0) >> 4) - (_width & 0x0F)) > 0) ? 0 : 1
#define TS_SNTS_GET_PRESSURE(_pressure) \
		_pressure

/* GET_BIT_MASK & GET_INDEX_FROM_MASK
 *
 * For easily checking the user input.
 * Usually, User use only one or two fingers.
 * However, we should always check all finger-status-register
 * because we can't know the total number of fingers.
 * These Macro will prevent it.
 */
#define GET_BIT_MASK(_finger_status_reg)	\
		(_finger_status_reg[2] & 0x04)<<7 | (_finger_status_reg[2] & 0x01)<<8 |	\
		(_finger_status_reg[1] & 0x40)<<1 | (_finger_status_reg[1] & 0x10)<<2 | \
		(_finger_status_reg[1] & 0x04)<<3 | (_finger_status_reg[1] & 0x01)<<4 |	\
		(_finger_status_reg[0] & 0x40)>>3 | (_finger_status_reg[0] & 0x10)>>2 | \
		(_finger_status_reg[0] & 0x04)>>1 | (_finger_status_reg[0] & 0x01)

#define GET_INDEX_FROM_MASK(_index, _bit_mask, _max_finger)	\
		for(; !((_bit_mask>>_index)&0x01) && _index <= _max_finger; _index++);	\
		if (_index <= _max_finger) _bit_mask &= ~(_bit_mask & (1<<(_index)));


/* wrapper function for i2c communication - except defalut page
 * if you have to select page for reading or writing, then using this wrapper function */
int synaptics_ts_page_data_read(struct i2c_client *client, u8 page, u8 reg, int size, u8 *data)
{
	DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, page), error);
	DO_SAFE(touch_i2c_read(client, reg, size, data), error);
	DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, DEFAULT_PAGE), error);
	return 0;
error:
	return -1;
}
/*
int synaptics_ts_page_data_write(struct i2c_client *client, u8 page, u8 reg, int size, u8 *data)
{
	DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, page), error);
	DO_SAFE(touch_i2c_write(client, reg, size, data), error);
	DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, DEFAULT_PAGE), error);
	return 0;
error:
	return -1;
}


int synaptics_ts_page_data_write_byte(struct i2c_client *client, u8 page, u8 reg, u8 data)
{
	DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, page), error);
	DO_SAFE(touch_i2c_write_byte(client, reg, data), error);
	DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, DEFAULT_PAGE), error);
	return 0;
error:
	return -1;
}
*/

static ssize_t show_test(struct i2c_client *client, char *buf)
{
	struct synaptics_ts_data *ts =
			(struct synaptics_ts_data *)get_touch_handle(client);
	int ret = 0;

	ret = sprintf(buf, "Success!! test[%d]\n", ts->is_probed);

	return ret;
}

static LGE_TOUCH_ATTR(test, S_IRUGO | S_IWUSR, show_test, NULL);

static struct attribute *lge_specific_touch_attribute_list[] = {
	&lge_touch_attr_test.attr,
};


static int read_page_description_table(struct i2c_client *client)
{
	struct synaptics_ts_data *ts =
			(struct synaptics_ts_data *)get_touch_handle(client);

	struct function_descriptor buffer;

	unsigned short u_address = 0;
	unsigned short page_num = 0;

	TOUCH_TRACE();

	memset(&buffer, 0x0, sizeof(struct function_descriptor));
	memset(&ts->common_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->finger_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->button_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->analog_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->flash_fc, 0x0, sizeof(struct ts_ic_function));

	for (page_num = 0; page_num < PAGE_MAX_NUM; page_num++) {
		DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, page_num), error);

		for (u_address = DESCRIPTION_TABLE_START; u_address > 10; u_address -= sizeof(struct function_descriptor)) {
			DO_SAFE(touch_i2c_read(client, u_address, sizeof(buffer), (unsigned char *)&buffer) < 0, error);

			if (buffer.id == 0)
				break;

			switch (buffer.id) {
			case RMI_DEVICE_CONTROL:
				ts->common_fc.dsc = buffer;
				ts->common_fc.function_page = page_num;
				break;
			case TOUCHPAD_SENSORS:
				ts->finger_fc.dsc = buffer;
				ts->finger_fc.function_page = page_num;
				break;
			case CAPACITIVE_BUTTON_SENSORS:
				ts->button_fc.dsc = buffer;
				ts->button_fc.function_page = page_num;
				break;
			case ANALOG_CONTROL:
				ts->analog_fc.dsc = buffer;
				ts->analog_fc.function_page = page_num;
				break;
			case FLASH_MEMORY_MANAGEMENT:
				ts->flash_fc.dsc = buffer;
				ts->flash_fc.function_page = page_num;
			default:
				break;
			}
		}
	}

	DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, 0x00), error);
	ERROR_IF(ts->common_fc.dsc.id == 0 || ts->finger_fc.dsc.id == 0
			|| ts->analog_fc.dsc.id == 0 || ts->flash_fc.dsc.id == 0, "page_init_error", error);

	TOUCH_DEBUG(DEBUG_BASE_INFO,"common[%dP:0x%02x] finger[%dP:0x%02x] button[%dP:0x%02x] analog[%dP:0x%02x] flash[%dP:0x%02x]\n",
				ts->common_fc.function_page, ts->common_fc.dsc.id, ts->finger_fc.function_page, ts->finger_fc.dsc.id,
				ts->button_fc.function_page, ts->button_fc.dsc.id, ts->analog_fc.function_page, ts->analog_fc.dsc.id,
				ts->flash_fc.function_page, ts->flash_fc.dsc.id);
	return 0;
error:
	return -EIO;
}

err_t synaptics_ts_probe(struct i2c_client *client, const struct touch_platform_data* lge_ts_data,
				const struct state_info* state, struct attribute ***attribute_list)
{
	struct synaptics_ts_data *ts;

	TOUCH_TRACE();

	ASSIGN(ts = devm_kzalloc(&client->dev, sizeof(struct synaptics_ts_data), GFP_KERNEL), error);
	set_touch_handle(client, ts);

	ts->client = client;
	ts->pdata = lge_ts_data;

	if(ts->pdata->pwr->use_regulator){
		DO_IF(IS_ERR(ts->regulator_vdd = regulator_get(&client->dev, ts->pdata->pwr->vdd)), error);
		DO_IF(IS_ERR(ts->regulator_vio = regulator_get(&client->dev, ts->pdata->pwr->vio)), error);
		if(ts->pdata->pwr->vdd_voltage > 0)
			DO_SAFE(regulator_set_voltage(ts->regulator_vdd, ts->pdata->pwr->vdd_voltage, ts->pdata->pwr->vdd_voltage), error);
		if(ts->pdata->pwr->vio_voltage > 0)
			DO_SAFE(regulator_set_voltage(ts->regulator_vio, ts->pdata->pwr->vio_voltage, ts->pdata->pwr->vio_voltage), error);
	}

	*attribute_list = lge_specific_touch_attribute_list;
	ts->is_probed = 0;

	return NO_ERROR;
error:
	return ERROR;
}

err_t synaptics_ts_remove(struct i2c_client* client)
{
	struct synaptics_ts_data* ts = (struct synaptics_ts_data*)get_touch_handle(client);

	TOUCH_TRACE();

	if (ts->pdata->pwr->use_regulator) {
		regulator_put(ts->regulator_vio);
		regulator_put(ts->regulator_vdd);
	}

	kfree(ts);
	return NO_ERROR;
}

err_t synaptics_ts_init(struct i2c_client* client)
{
	struct synaptics_ts_data *ts = (struct synaptics_ts_data *)get_touch_handle(client);
	u8 buf = 0;

	TOUCH_TRACE();

	if(ts->is_probed == 0){
		ts->is_probed = 1;
		read_page_description_table(ts->client);
	}

	DO_SAFE(touch_i2c_write_byte(client, DEVICE_CONTROL_REG,
			DEVICE_CONTROL_NORMAL_OP | DEVICE_CONTROL_CONFIGURED), error);

	DO_SAFE(touch_i2c_read(client, INTERRUPT_ENABLE_REG, 1, &buf), error);
	DO_SAFE(touch_i2c_write_byte(client, INTERRUPT_ENABLE_REG, buf | INTERRUPT_MASK_ABS0), error);

	if(ts->pdata->role->report_mode == CONTINUOUS_REPORT_MODE)
		DO_SAFE(touch_i2c_write_byte(client, TWO_D_REPORTING_MODE, REPORT_MODE_CONTINUOUS), error);
	else{
		DO_SAFE(touch_i2c_write_byte(client, TWO_D_REPORTING_MODE,
					REPORT_BEYOND_CLIP | ABS_FILTER | REPORT_MODE_REDUCED), error);
		DO_SAFE(touch_i2c_write_byte(client, DELTA_X_THRESH_REG,
					ts->pdata->role->delta_pos_threshold), error);
		DO_SAFE(touch_i2c_write_byte(client, DELTA_Y_THRESH_REG,
					ts->pdata->role->delta_pos_threshold), error);
	}

	DO_SAFE(touch_i2c_read(client, INTERRUPT_STATUS_REG, 1, &buf), error);

	return NO_ERROR;
error:
	return ERROR;
}

err_t synaptics_ts_get_data(struct i2c_client *client, struct touch_data* curr_data, const struct touch_data* prev_data)
{
	struct synaptics_ts_data *ts = (struct synaptics_ts_data *)get_touch_handle(client);

	u8  i = 0;
	u8  finger_index = 0;
	u16 touch_finger_bit_mask = 0;

	TOUCH_TRACE();

	curr_data->total_num = 0;
	curr_data->id_mask = 0;

	DO_SAFE(touch_i2c_read(client, INTERRUPT_STATUS_REG,
		sizeof(ts->ts_data.interrupt_status_reg), &ts->ts_data.interrupt_status_reg), error);

	// ABS
	if(ts->ts_data.interrupt_status_reg & INTERRUPT_MASK_ABS0){
		DO_SAFE(touch_i2c_read(ts->client, FINGER_STATE_REG,
					sizeof(ts->ts_data.finger.finger_status_reg),
					ts->ts_data.finger.finger_status_reg), error);

		touch_finger_bit_mask = GET_BIT_MASK(ts->ts_data.finger.finger_status_reg);

		for(i = 0; i < MAX_NUM_OF_FINGERS; i++){
			if(touch_finger_bit_mask & (1 << i)){
				DO_SAFE(touch_i2c_read(ts->client,
						FINGER_DATA_REG_START + (NUM_OF_EACH_FINGER_DATA_REG * i),
						NUM_OF_EACH_FINGER_DATA_REG, ts->ts_data.finger.finger_reg[i]), error);

				curr_data->abs_data[finger_index].id = i;
				curr_data->abs_data[finger_index].x =
				curr_data->abs_data[finger_index].raw_x =
					TS_SNTS_GET_X_POSITION(ts->ts_data.finger.finger_reg[i][REG_X_POSITION],
							       ts->ts_data.finger.finger_reg[i][REG_YX_POSITION]);
				curr_data->abs_data[finger_index].y =
				curr_data->abs_data[finger_index].raw_y =
					TS_SNTS_GET_Y_POSITION(ts->ts_data.finger.finger_reg[i][REG_Y_POSITION],
							       ts->ts_data.finger.finger_reg[i][REG_YX_POSITION]);
				curr_data->abs_data[finger_index].width_major =
					TS_SNTS_GET_WIDTH_MAJOR(ts->ts_data.finger.finger_reg[i][REG_WY_WX]);
				curr_data->abs_data[finger_index].width_minor =
					TS_SNTS_GET_WIDTH_MINOR(ts->ts_data.finger.finger_reg[i][REG_WY_WX]);
				curr_data->abs_data[finger_index].orientation =
					TS_SNTS_GET_ORIENTATION(ts->ts_data.finger.finger_reg[i][REG_WY_WX]);
				curr_data->abs_data[finger_index].pressure =
					TS_SNTS_GET_PRESSURE(ts->ts_data.finger.finger_reg[i][REG_Z]);

				curr_data->id_mask |= (0x1 << i);
				curr_data->total_num++;

				TOUCH_DEBUG(DEBUG_GET_DATA, "<%d> pos(%4d,%4d) w_m[%2d] w_n[%2d] o[%2d] p[%2d]\n",
						i, curr_data->abs_data[finger_index].x, curr_data->abs_data[finger_index].y,
						curr_data->abs_data[finger_index].width_major, curr_data->abs_data[finger_index].width_minor,
						curr_data->abs_data[finger_index].orientation, curr_data->abs_data[finger_index].pressure);

				finger_index++;
			}
		}
		TOUCH_DEBUG(DEBUG_GET_DATA, "ID[0x%x] Total_num[%d]\n", curr_data->id_mask, curr_data->total_num);
	}

	// BUTTON
	if (likely(ts->ts_data.interrupt_status_reg & INTERRUPT_MASK_BUTTON)) {
		DO_SAFE(synaptics_ts_page_data_read(ts->client, BUTTON_PAGE, BUTTON_DATA_REG,
				sizeof(ts->ts_data.button_data_reg), &ts->ts_data.button_data_reg), error);

		if (ts->ts_data.button_data_reg) {
			/* pressed - find first one */
			int cnt = 0;
			for (cnt = 0; cnt < ts->pdata->caps->number_of_button; cnt++)
			{
				if ((ts->ts_data.button_data_reg >> cnt) & 0x1) {
					ts->ts_data.button.key_code = ts->pdata->caps->button_name[cnt];
					curr_data->button_data.key_code = ts->ts_data.button.key_code;
					curr_data->button_data.state = 1;
					break;
				}
			}
		}else {
			/* release */
			curr_data->button_data.key_code = ts->ts_data.button.key_code;
			curr_data->button_data.state = 0;
		}

		TOUCH_DEBUG(DEBUG_GET_DATA, "button_register[0x%x] key_code[%d] state[%d]\n",
				ts->ts_data.button_data_reg, curr_data->button_data.key_code, curr_data->button_data.state);
	}

	return NO_ERROR;
error:
	return ERROR;
}

err_t synaptics_ts_filter(struct i2c_client *client, struct touch_data* curr_data, const struct touch_data* prev_data)
{
//	struct synaptics_ts_data *ts =
//			(struct synaptics_ts_data *)get_touch_handle(client);

	return NO_ERROR;
}

err_t synaptics_ts_power(struct i2c_client *client, int power_ctrl)
{
	struct synaptics_ts_data *ts = (struct synaptics_ts_data *)get_touch_handle(client);

	TOUCH_TRACE();

	switch (power_ctrl) {
		case POWER_OFF:
			if (ts->pdata->pwr->use_regulator) {
				if(regulator_is_enabled(ts->regulator_vio))
					regulator_disable(ts->regulator_vio);
				if(regulator_is_enabled(ts->regulator_vdd))
					regulator_disable(ts->regulator_vdd);
			}
			else
				ts->pdata->pwr->power(0);
			break;
		case POWER_ON:
			if (ts->pdata->pwr->use_regulator) {
				if(!regulator_is_enabled(ts->regulator_vdd))
					regulator_enable(ts->regulator_vdd);
				if(!regulator_is_enabled(ts->regulator_vio))
					regulator_enable(ts->regulator_vio);
			}
			else
				ts->pdata->pwr->power(1);
			break;
		case POWER_SLEEP:
		case POWER_WAKE:
			break;
		default:
			return -EIO;
			break;
		}

	return NO_ERROR;
}

err_t synaptics_ts_ic_ctrl(struct i2c_client *client, u8 code, u32 value, u32* ret)
{
//	struct synaptics_ts_data *ts =
//			(struct synaptics_ts_data *)get_touch_handle(client);

	return NO_ERROR;
}

err_t synaptics_ts_fw_upgrade(struct i2c_client* client, struct touch_fw_info* info, struct touch_firmware_module* fw)
{
//	struct synaptics_ts_data *ts =
//			(struct synaptics_ts_data*)get_touch_handle(client);

	return NO_ERROR;
}

err_t synaptics_ts_suspend(struct i2c_client* client)
{
//	struct synaptics_ts_data *ts =
//			(struct synaptics_ts_data*)get_touch_handle(client);

	return NO_ERROR;
}

err_t synaptics_ts_resume(struct i2c_client* client)
{
//	struct synaptics_ts_data *ts =
//			(struct synaptics_ts_data*)get_touch_handle(client);

	return NO_ERROR;
}

err_t synaptics_ts_notify(struct i2c_client *client, u8 code, u32 value)
{
//	struct synaptics_ts_data *ts =
//			(struct synaptics_ts_data*)get_touch_handle(client);

	return NO_ERROR;
}

struct touch_device_driver synaptics_ts_driver = {
	.probe		= synaptics_ts_probe,
	.remove		= synaptics_ts_remove,
	.init  		= synaptics_ts_init,
	.data  		= synaptics_ts_get_data,
	.filter		= synaptics_ts_filter,
	.power 		= synaptics_ts_power,
	.ic_ctrl	= synaptics_ts_ic_ctrl,
	.fw_upgrade 	= synaptics_ts_fw_upgrade,
	.suspend	= synaptics_ts_suspend,
	.resume		= synaptics_ts_resume,
	.notify     = synaptics_ts_notify,
};

static void async_touch_init(void *data, async_cookie_t cookie)
{
	touch_driver_register(&synaptics_ts_driver);
	return;
}


static int __devinit touch_init(void)
{
	TOUCH_TRACE();
	async_schedule(async_touch_init, NULL);

	return 0;
}

static void __exit touch_exit(void)
{
	TOUCH_TRACE();
	touch_driver_unregister();
}

module_init(touch_init);
module_exit(touch_exit);

MODULE_AUTHOR("yehan.ahn@lge.com, hyesung.shin@lge.com");
MODULE_DESCRIPTION("LGE Touch Driver");
MODULE_LICENSE("GPL");


