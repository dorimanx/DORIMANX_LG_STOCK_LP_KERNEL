/* Touch_synaptics.c
 *
 * Copyright (C) 2013 LGE.
 *
 * Author: yehan.ahn@lge.com, hyesung.shin@lge.com
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
#include <asm/atomic.h>
#include <linux/file.h>     /* for file access */
#include <linux/syscalls.h> /* for file access */
#include <linux/uaccess.h>  /* for file access */
#include <linux/workqueue.h>
#include <linux/wakelock.h>

#include "lge_touch_core.h"
#include "touch_synaptics.h"
#include <linux/firmware.h>
#include "./DS5/RefCode_F54.h"
#include <mach/board_lge.h>

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

static int get_ic_info(struct synaptics_ts_data *ts);
static int read_page_description_table(struct i2c_client *client);

extern char f54_wlog_buf[6000];
static unsigned long im_sum, cns_sum, cid_im_sum, freq_scan_im_sum;
static u16 im_aver, cns_aver, cid_im_aver, freq_scan_im_aver;
static unsigned int cnt;
u8 int_mask_cust;
int is_Sensing;
int boot_mode = NORMAL_BOOT_MODE;
int mfts_mode;
int mfts_enable;
static int fw_upgrade_state;

/*static int ts_suspend = 0;
int thermal_status = 0;
extern int touch_thermal_mode; */
u8 default_finger_amplitude[2] = {0};
extern int touch_ta_status;
extern int touch_hdmi_status;
struct synaptics_ts_f12_info {
	bool ctrl_reg_is_present[32];
	bool data_reg_is_present[16];
	u8 ctrl_reg_addr[32];
	u8 data_reg_addr[16];
};

static struct synaptics_ts_f12_info f12_info;

#define RMI_DEVICE_CONTROL			0x01
#define TOUCHPAD_SENSORS			0x12
#define FLASH_MEMORY_MANAGEMENT			0x34
#define ANALOG_CONTROL				0x54
#define SENSOR_CONTROL				0x55
#define F1A_EXIST					0x1A

/* Register Map & Register bit mask
 * - Please check "One time" this map before using this device driver
 */
/* RMI_DEVICE_CONTROL */
#define MANUFACTURER_ID_REG			(ts->common_fc.dsc.query_base)		/* Manufacturer ID */
#define CUSTOMER_FAMILY_REG			(ts->common_fc.dsc.query_base+2)	/* CUSTOMER_FAMILY QUERY */
#define FW_REVISION_REG				(ts->common_fc.dsc.query_base+3)	/* FW revision */
#define PRODUCT_ID_REG				(ts->common_fc.dsc.query_base+11)	/* Product ID */

#define DEVICE_COMMAND_REG			(ts->common_fc.dsc.command_base)

#define DEVICE_CONTROL_REG 			(ts->common_fc.dsc.control_base)	/* Device Control */
#define DEVICE_CONTROL_NORMAL_OP		0x00	/* sleep mode : go to doze mode after 500 ms */
#define DEVICE_CONTROL_SLEEP 			0x01	/* sleep mode : go to sleep */
#define DEVICE_CONTROL_SLEEP_NO_RECAL		0x02	/* sleep mode : go to sleep. no-recalibration */
#define DEVICE_CONTROL_NOSLEEP			0x04
#define DEVICE_CHARGER_CONNECTED		0x20
#define DEVICE_CONTROL_CONFIGURED		0x80

#define INTERRUPT_ENABLE_REG			(ts->common_fc.dsc.control_base+1)	/* Interrupt Enable 0 */
#define DOZE_INTERVAL_REG				(ts->common_fc.dsc.control_base+2)	/* Doze Interval : unit 10ms */
#define DOZE_WAKEUP_THRESHOLD_REG		(ts->common_fc.dsc.control_base+3)


#define DEVICE_STATUS_REG			(ts->common_fc.dsc.data_base)		/* Device Status */
#define DEVICE_FAILURE_MASK			0x03
#define DEVICE_CRC_ERROR_MASK			0x04
#define DEVICE_STATUS_FLASH_PROG		0x40
#define DEVICE_STATUS_UNCONFIGURED		0x80

#define INTERRUPT_STATUS_REG			(ts->common_fc.dsc.data_base+1)		/* Interrupt Status */
#define INTERRUPT_MASK_FLASH			0x01
#define INTERRUPT_MASK_ABS0			0x04
#define INTERRUPT_MASK_BUTTON			0x10
#define INTERRUPT_MASK_CUSTOM			0x40

/* TOUCHPAD_SENSORS */
#define FINGER_COMMAND_REG			(ts->finger_fc.dsc.command_base)
#define MOTION_SUPPRESSION			(ts->finger_fc.dsc.control_base+5)		/* f12_info.ctrl_reg_addr[20] */
#define GLOVED_FINGER_MASK			0x20

#define OBJECT_TYPE_AND_STATUS_REG		(ts->finger_fc.dsc.data_base)		/* Finger State */
#define OBJECT_ATTENTION_REG			(ts->finger_fc.dsc.data_base+2)
#define FINGER_DATA_REG_START			(ts->finger_fc.dsc.data_base)		/* Finger Data Register */
#define REG_OBJECT_TYPE_AND_STATUS		0
#define REG_X_LSB				1
#define REG_X_MSB				2
#define REG_Y_LSB				3
#define REG_Y_MSB				4
#define REG_Z					5
#define REG_WX					6
#define REG_WY					7

#define MAXIMUM_XY_COORDINATE_REG		(ts->finger_fc.dsc.control_base)

/* ANALOG_CONTROL */
#define ANALOG_COMMAND_REG			(ts->analog_fc.dsc.command_base)
#define ANALOG_CONTROL_REG			(ts->analog_fc.dsc.control_base)

/* FLASH_MEMORY_MANAGEMENT */
#define FLASH_CONFIG_ID_REG			(ts->flash_fc.dsc.control_base)		/* Flash Control */
#define FLASH_CONTROL_REG			(ts->flash_fc.dsc.data_base+2)
#define FLASH_STATUS_REG			(ts->flash_fc.dsc.data_base+3)
#define FLASH_STATUS_MASK			0xFF

/* Page number */
#define COMMON_PAGE				(ts->common_fc.function_page)
#define FINGER_PAGE				(ts->finger_fc.function_page)
#define ANALOG_PAGE				(ts->analog_fc.function_page)
#define FLASH_PAGE				(ts->flash_fc.function_page)
#define SENSOR_PAGE				(ts->sensor_fc.function_page)
#define DEFAULT_PAGE				0x00
#define LPWG_PAGE				0x04

/* Others */
#define LPWG_STATUS_REG				0x00 /* 4-page */
#define LPWG_DATA_REG				0x01 /* 4-page */
#define LPWG_TAPCOUNT_REG			0x31 /* 4-page */
#define LPWG_MIN_INTERTAP_REG			0x32 /* 4-page */
#define LPWG_MAX_INTERTAP_REG			0x33 /* 4-page */
#define LPWG_TOUCH_SLOP_REG			0x34 /* 4-page */
#define LPWG_TAP_DISTANCE_REG			0x35 /* 4-page */
#define LPWG_INTERRUPT_DELAY_REG		0x37 /* 4-page */

#define LPWG_TAPCOUNT_REG2			0x38 /* 4-page */
#define LPWG_MIN_INTERTAP_REG2			0x39 /* 4-page */
#define LPWG_MAX_INTERTAP_REG2			0x3A /* 4-page */
#define LPWG_TOUCH_SLOP_REG2			0x3B /* 4-page */
#define LPWG_TAP_DISTANCE_REG2			0x3C /* 4-page */
#define LPWG_INTERRUPT_DELAY_REG2		0x3E /* 4-page */
#define WAKEUP_GESTURE_ENABLE_REG		0x20		/* f12_info.ctrl_reg_addr[27] */
#define MISC_HOST_CONTROL_REG			0x3F
#define THERMAL_HIGH_FINGER_AMPLITUDE		0x60	/* finger_amplitude(0x80) = 0.5 */

/* LPWG Control Value */
#define REPORT_MODE_CTRL	1
#define TCI_ENABLE_CTRL		2
#define TAP_COUNT_CTRL		3
#define MIN_INTERTAP_CTRL	4
#define MAX_INTERTAP_CTRL	5
#define TOUCH_SLOP_CTRL		6
#define TAP_DISTANCE_CTRL	7
#define INTERRUPT_DELAY_CTRL   8

#define TCI_ENABLE_CTRL2	22
#define TAP_COUNT_CTRL2		23
#define MIN_INTERTAP_CTRL2	24
#define MAX_INTERTAP_CTRL2	25
#define TOUCH_SLOP_CTRL2	26
#define TAP_DISTANCE_CTRL2	27
#define INTERRUPT_DELAY_CTRL2   28

/* Palm / Hover */
#define PALM_TYPE	3
#define HOVER_TYPE	5
#define MAX_PRESSURE	255

#define I2C_DELAY			50
#define UEVENT_DELAY			200
#define REBASE_DELAY			100
#define FILE_MOUNT_DELAY             20000
#define CAP_DIFF_MAX             500
#define CAP_MIN_MAX_DIFF             1000
#define KNOCKON_DELAY			68  /* 700ms */


/* Get user-finger-data from register.
 */
#define TS_SNTS_GET_X_POSITION(_msb_reg, _lsb_reg) \
		(((u16)((_msb_reg << 8)  & 0xFF00)  | (u16)((_lsb_reg) & 0xFF)))
#define TS_SNTS_GET_Y_POSITION(_msb_reg, _lsb_reg) \
		(((u16)((_msb_reg << 8)  & 0xFF00)  | (u16)((_lsb_reg) & 0xFF)))
#define TS_SNTS_GET_WIDTH_MAJOR(_width_x, _width_y) \
			((_width_x - _width_y) > 0) ? _width_x : _width_y
#define TS_SNTS_GET_WIDTH_MINOR(_width_x, _width_y) \
			((_width_x - _width_y) > 0) ? _width_y : _width_x

#define TS_SNTS_GET_ORIENTATION(_width_y, _width_x) \
		((_width_y - _width_x) > 0) ? 0 : 1
#define TS_SNTS_GET_PRESSURE(_pressure) \
		_pressure
#define jitter_abs(x)	(x > 0 ? x : -x)
#define jitter_sub(x, y)	(x > y ? x - y : y - x)
#define get_time_interval(a, b) (a >= b ? a - b : 1000000 + a - b)
struct timeval t_debug[EX_PROFILE_MAX];

#define GET_OBJECT_REPORT_INFO(_reg, _type) \
	(((_reg) & ((u8)(1 << (_type)))) >> (_type))

static u8 half_err_cnt;
/* static u8 resume_flag = 0; */
static int rebase_retry_cnt;
static int ref_chk_enable;
static int ref_chk_rebase_stop;
static int wq_rebase_lock = 1;
static int raw_cap_file_exist;
struct i2c_client *ds4_i2c_client;
static char power_state;
int compare_fw_version(struct i2c_client *client, struct touch_fw_info *fw_info);
static char *productcode_parse(unsigned char *product);
extern struct workqueue_struct *touch_wq;
int f54_window_crack_check_mode;
int f54_window_crack;
int after_crack_check;
bool touch_irq_mask = 1;
bool touch_wake_test;
unsigned int  touch_wake_count;
char touch_wake_log_buf[256] = {0};
#define TOUCH_WAKE_COUNTER_LOG_PATH		"/mnt/sdcard/wake_cnt.txt"
enum error_type synaptics_ts_init(struct i2c_client *client);
enum error_type synaptics_ts_ic_ctrl(struct i2c_client *client, u8 code, u32 value, u32 *ret);
static int set_doze_param(struct synaptics_ts_data *ts);
static bool need_scan_pdt = true;

void touch_enable_irq(unsigned int irq)
{
	if (!touch_irq_mask) {
		touch_irq_mask = 1;
		enable_irq(irq);
	}
}

void touch_disable_irq(unsigned int irq)
{
	if (touch_irq_mask) {
		touch_irq_mask = 0;
		disable_irq_nosync(irq);
	}
}

void write_time_log(char *filename, char *data, int data_include)
{
	int fd = 0;
	char *fname = NULL;
	char time_string[64] = {0};
	struct timespec my_time;
	struct tm my_date;

	mm_segment_t old_fs = get_fs();

	my_time = __current_kernel_time();
	time_to_tm(my_time.tv_sec, sys_tz.tz_minuteswest * 60 * (-1), &my_date);
	snprintf(time_string, 64, "%02d-%02d %02d:%02d:%02d.%03lu \n\n",
		my_date.tm_mon + 1, my_date.tm_mday,
		my_date.tm_hour, my_date.tm_min, my_date.tm_sec,
		(unsigned long) my_time.tv_nsec / 1000000);

	set_fs(KERNEL_DS);

	if (filename == NULL)
		fname = "/mnt/sdcard/touch_self_test.txt";
	else
		fname = filename;
	fd = sys_open(fname, O_WRONLY|O_CREAT|O_APPEND, 0666);

    TOUCH_INFO_MSG("write open %s, fd : %d \n", (fd >= 0) ? "success" : "fail", fd);

	if (fd >= 0) {
		if (data_include && data != NULL)
			sys_write(fd, data, strlen(data));
		sys_write(fd, time_string, strlen(time_string));
		sys_close(fd);
	}
	set_fs(old_fs);
}

void write_firmware_version_log(struct synaptics_ts_data *ts)
{
    char *version_string = NULL;
    int ver_outbuf = 0;

    version_string = kzalloc(448 * sizeof(char), GFP_KERNEL);

    TOUCH_INFO_MSG("%s: first: version_string %p(%d)\n", __FUNCTION__, version_string, strlen(version_string));

	ver_outbuf += sprintf(version_string+ver_outbuf, "===== Firmware Info =====\n");

    if (ts->fw_info.fw_version[0] > 0x40)
		ver_outbuf += sprintf(version_string+ver_outbuf, "ic_fw_version[%s]\n", ts->fw_info.fw_version);
	else {
		ver_outbuf += sprintf(version_string+ver_outbuf, "version : v%d.%02d\n", ((ts->fw_info.fw_version[3] & 0x80) >> 7), (ts->fw_info.fw_version[3] & 0x7F));
	}

	ver_outbuf += sprintf(version_string+ver_outbuf, "IC_product_id[%s]\n", ts->fw_info.fw_product_id);

    if (!(strncmp(ts->fw_info.fw_product_id, "PLG313", 6)) || !(strncmp(ts->fw_info.fw_product_id, "PLG352", 6)) || !(strncmp(ts->fw_info.fw_product_id, "PLG391", 6))) {
		if (ts->fw_info.family && ts->fw_info.fw_revision) {
			ver_outbuf += sprintf(version_string+ver_outbuf, "Touch IC : s3528(family_id = %d, fw_rev = %d)\n",
					ts->fw_info.family, ts->fw_info.fw_revision);
		} else {
			ver_outbuf += sprintf(version_string+ver_outbuf, "Touch IC : s3528(family_id = %d, fw_rev = %d)\n",
					ts->fw_info.family, ts->fw_info.fw_revision);
		}
    } else if (!(strncmp(ts->fw_info.fw_product_id, "PLG298", 6))) {
		ver_outbuf += sprintf(version_string+ver_outbuf, "Touch IC : s3621\n");
	} else {
		ver_outbuf += sprintf(version_string+ver_outbuf, "Touch product id read error\n\n");
	}

    ver_outbuf += sprintf(version_string+ver_outbuf, "=========================\n\n");

    write_log(NULL, version_string);
    msleep(30);

    TOUCH_INFO_MSG("%s: after: version_string %p(%d)\n", __FUNCTION__, version_string, strlen(version_string));
    kfree(version_string);
}


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
/**
 * Knock on
 *
 * Type		Value
 *
 * 1		WakeUp_gesture_only=1 / Normal=0
 * 2		TCI enable=1 / disable=0
 * 3		Tap Count
 * 4		Min InterTap
 * 5		Max InterTap
 * 6		Touch Slop
 * 7		Tap Distance
 * 8		Interrupt Delay
 */
static int tci_control(struct synaptics_ts_data *ts, int type, u8 value)
{
	struct i2c_client *client = ts->client;
	u8 buffer[3] = {0};


	switch (type) {
	case REPORT_MODE_CTRL:
		DO_SAFE(touch_i2c_read(ts->client, INTERRUPT_ENABLE_REG, 1, buffer), error);
		DO_SAFE(touch_i2c_write_byte(ts->client, INTERRUPT_ENABLE_REG,
				value ? buffer[0] & ~INTERRUPT_MASK_ABS0 : buffer[0] | INTERRUPT_MASK_ABS0), error);
		if (value) {
			buffer[0] = 0x29;
			buffer[1] = 0x29;
			DO_SAFE(touch_i2c_write(client, f12_info.ctrl_reg_addr[15], 2, buffer), error);
		}
		DO_SAFE(touch_i2c_read(client, f12_info.ctrl_reg_addr[20], 3, buffer), error);
		buffer[2] = (buffer[2] & 0xfc) | (value ? 0x2 : 0x0);
		DO_SAFE(touch_i2c_write(client, f12_info.ctrl_reg_addr[20], 3, buffer), error);
		TOUCH_DEBUG(DEBUG_BASE_INFO, "report mode: %d\n", value);
		break;
	case TCI_ENABLE_CTRL:
		DO_SAFE(synaptics_ts_page_data_read(client,
				LPWG_PAGE, LPWG_TAPCOUNT_REG, 1, buffer), error);
		buffer[0] = (buffer[0] & 0xfe) | (value & 0x1);
		DO_SAFE(synaptics_ts_page_data_write(client,
				LPWG_PAGE, LPWG_TAPCOUNT_REG, 1, buffer), error);
		break;
	case TCI_ENABLE_CTRL2:
		DO_SAFE(synaptics_ts_page_data_read(client,
				LPWG_PAGE, LPWG_TAPCOUNT_REG2, 1, buffer), error);
		buffer[0] = (buffer[0] & 0xfe) | (value & 0x1);
		DO_SAFE(synaptics_ts_page_data_write(client,
				LPWG_PAGE, LPWG_TAPCOUNT_REG2, 1, buffer), error);
		break;
	case TAP_COUNT_CTRL:
		DO_SAFE(synaptics_ts_page_data_read(client,
				LPWG_PAGE, LPWG_TAPCOUNT_REG, 1, buffer), error);
		buffer[0] = ((value << 3) & 0xf8) | (buffer[0] & 0x7);
		DO_SAFE(synaptics_ts_page_data_write(client,
				LPWG_PAGE, LPWG_TAPCOUNT_REG, 1, buffer), error);
		break;
	case TAP_COUNT_CTRL2:
		DO_SAFE(synaptics_ts_page_data_read(client,
				LPWG_PAGE, LPWG_TAPCOUNT_REG2, 1, buffer), error);
		buffer[0] = ((value << 3) & 0xf8) | (buffer[0] & 0x7);
		DO_SAFE(synaptics_ts_page_data_write(client,
				LPWG_PAGE, LPWG_TAPCOUNT_REG2, 1, buffer), error);
		break;
	case MIN_INTERTAP_CTRL:
		DO_SAFE(synaptics_ts_page_data_write_byte(client,
				LPWG_PAGE, LPWG_MIN_INTERTAP_REG, value), error);
		break;
	case MIN_INTERTAP_CTRL2:
		DO_SAFE(synaptics_ts_page_data_write_byte(client,
				LPWG_PAGE, LPWG_MIN_INTERTAP_REG2, value), error);
		break;
	case MAX_INTERTAP_CTRL:
		DO_SAFE(synaptics_ts_page_data_write_byte(client,
				LPWG_PAGE, LPWG_MAX_INTERTAP_REG, value), error);
		break;
	case MAX_INTERTAP_CTRL2:
		DO_SAFE(synaptics_ts_page_data_write_byte(client,
				LPWG_PAGE, LPWG_MAX_INTERTAP_REG2, value), error);
		break;
	case TOUCH_SLOP_CTRL:
		DO_SAFE(synaptics_ts_page_data_write_byte(client,
				LPWG_PAGE, LPWG_TOUCH_SLOP_REG, value), error);
		break;
	case TOUCH_SLOP_CTRL2:
		DO_SAFE(synaptics_ts_page_data_write_byte(client,
				LPWG_PAGE, LPWG_TOUCH_SLOP_REG2, value), error);
		break;
	case TAP_DISTANCE_CTRL:
		DO_SAFE(synaptics_ts_page_data_write_byte(client,
				LPWG_PAGE, LPWG_TAP_DISTANCE_REG, value), error);
		break;
	case TAP_DISTANCE_CTRL2:
		DO_SAFE(synaptics_ts_page_data_write_byte(client,
				LPWG_PAGE, LPWG_TAP_DISTANCE_REG2, value), error);
		break;
	case INTERRUPT_DELAY_CTRL:
		DO_SAFE(synaptics_ts_page_data_write_byte(client,
				LPWG_PAGE, LPWG_INTERRUPT_DELAY_REG,
				value ? (buffer[0] = (KNOCKON_DELAY << 1) | 0x1) : (buffer[0] = 0)), error);
		break;
	case INTERRUPT_DELAY_CTRL2:
		DO_SAFE(synaptics_ts_page_data_write_byte(client,
				LPWG_PAGE, LPWG_INTERRUPT_DELAY_REG2,
				value ? (buffer[0] = (KNOCKON_DELAY << 1) | 0x1) : (buffer[0] = 0)), error);
		break;
	default:
		break;
	}

	return 0;
error:
	return -1;

}

static int get_tci_data(struct synaptics_ts_data *ts, int count)
{
	struct i2c_client *client = ts->client;
	u8 i = 0;
	u8 buffer[12][4] = {{0}};

	ts->pw_data.data_num = count;

	if (!count)
		return 0;

	DO_SAFE(synaptics_ts_page_data_read(client, LPWG_PAGE, LPWG_DATA_REG, 4*count, &buffer[0][0]), error);
	TOUCH_DEBUG(DEBUG_BASE_INFO || DEBUG_LPWG, "%s : knock code coordinates\n", __func__);

	for (i = 0; i < count; i++) {
		ts->pw_data.data[i].x = TS_SNTS_GET_X_POSITION(buffer[i][1], buffer[i][0]);
		ts->pw_data.data[i].y = TS_SNTS_GET_Y_POSITION(buffer[i][3], buffer[i][2]);
		TOUCH_DEBUG(DEBUG_BASE_INFO || DEBUG_LPWG, "LPWG data\n");
	}

	return 0;
error:
	return -1;
}

static void set_lpwg_mode(struct lpwg_control *ctrl, int mode)
{
	if (mfts_enable) {
		return;
	}
	ctrl->double_tap_enable = (mode & (LPWG_DOUBLE_TAP | LPWG_PASSWORD)) ? 1 : 0;
	ctrl->password_enable = (mode & LPWG_PASSWORD) ? 1 : 0;
	ctrl->signature_enable = (mode & LPWG_SIGNATURE) ? 1 : 0;
	ctrl->lpwg_is_enabled = ctrl->double_tap_enable
			|| ctrl->password_enable || ctrl->signature_enable;
}

static int sleep_control(struct synaptics_ts_data *ts, int mode, int recal)
{
	u8 curr = 0;
	u8 next = 0;

	DO_SAFE(touch_i2c_read(ts->client, DEVICE_CONTROL_REG, 1, &curr), error);

	next = (curr & 0xFC) | (mode ? DEVICE_CONTROL_NORMAL_OP : DEVICE_CONTROL_SLEEP);

	TOUCH_DEBUG(DEBUG_BASE_INFO, "%s : curr = [%x] next[%x]\n", __func__, curr, next);

	DO_SAFE(touch_i2c_write_byte(ts->client, DEVICE_CONTROL_REG, next), error);

	return 0;
error:
	return -1;
}

static int lpwg_control(struct synaptics_ts_data *ts, int mode)
{
	if (mfts_enable) {
		return 0;
	}

	set_lpwg_mode(&ts->lpwg_ctrl, mode);

	switch (mode) {
	case LPWG_SIGNATURE:
		break;
	case LPWG_DOUBLE_TAP:					/* Only TCI-1 */
		tci_control(ts, TCI_ENABLE_CTRL, 1);		/* tci enable */
		tci_control(ts, TAP_COUNT_CTRL, 2); 		/* tap count = 2 */
		tci_control(ts, MIN_INTERTAP_CTRL, 0);		/* min inter_tap = 0ms */
		tci_control(ts, MAX_INTERTAP_CTRL, 70);		/* max inter_tap = 700ms */
		tci_control(ts, TOUCH_SLOP_CTRL, 100);		/* touch_slop = 10mm */
		tci_control(ts, TAP_DISTANCE_CTRL, 10);		/* tap distance = 10mm */
		tci_control(ts, INTERRUPT_DELAY_CTRL, 0);	/* interrupt delay = 0ms */
		tci_control(ts, TCI_ENABLE_CTRL2, 0);		/* tci-2 disable */
		tci_control(ts, REPORT_MODE_CTRL, 1);		/* wakeup_gesture_only */
		break;
	case LPWG_PASSWORD:					/* TCI-1 and TCI-2 */
		tci_control(ts, TCI_ENABLE_CTRL, 1);		/* tci-1 enable */
		tci_control(ts, TAP_COUNT_CTRL, 2); 		/* tap count = 2 */
		tci_control(ts, MIN_INTERTAP_CTRL, 0);		/* min inter_tap = 0ms */
		tci_control(ts, MAX_INTERTAP_CTRL, 70);		/* max inter_tap = 700ms */
		tci_control(ts, TOUCH_SLOP_CTRL, 100);		/* touch_slop = 10mm */
		tci_control(ts, TAP_DISTANCE_CTRL, 7);		/* tap distance = 7mm */
		tci_control(ts, INTERRUPT_DELAY_CTRL, (u8)ts->pw_data.double_tap_check);	/* interrupt delay = 0ms */

		tci_control(ts, TCI_ENABLE_CTRL2, 1);		/* tci-2 enable */
		tci_control(ts, TAP_COUNT_CTRL2, (u8)ts->pw_data.tap_count); /* tap count = "user_setting" */
		tci_control(ts, MIN_INTERTAP_CTRL2, 0);		/* min inter_tap = 0ms */
		tci_control(ts, MAX_INTERTAP_CTRL2, 70);	/* max inter_tap = 700ms */
		tci_control(ts, TOUCH_SLOP_CTRL2, 100);		/* touch_slop = 10mm */
		tci_control(ts, TAP_DISTANCE_CTRL2, 255);	/* tap distance = MAX */
		tci_control(ts, INTERRUPT_DELAY_CTRL2, 0);	/* interrupt delay = 0ms */

		tci_control(ts, REPORT_MODE_CTRL, 1);		/* wakeup_gesture_only */
		break;
	default:
		tci_control(ts, TCI_ENABLE_CTRL, 0);		/* tci-1 disable */
		tci_control(ts, TCI_ENABLE_CTRL2, 0);		/* tci-2 disable */
		tci_control(ts, REPORT_MODE_CTRL, 0);		/* normal */
		sleep_control(ts, 1, 0);
		break;
	}

	TOUCH_DEBUG(DEBUG_BASE_INFO | DEBUG_LPWG, "lpwg_mode[%d]\n", mode);
	return 0;
}

struct synaptics_ts_proximity_fhandler {
	struct synaptics_ts_proximity_fn *prox_fn;
	bool prox_inserted;
	bool prox_initialized;
};

static struct synaptics_ts_proximity_fhandler prox_fhandler;

void synaptics_ts_new_function(struct synaptics_ts_proximity_fn *prox_fn, bool insert)
{
	prox_fhandler.prox_inserted = insert;

	if (insert)
		prox_fhandler.prox_fn = prox_fn;
	else
		prox_fhandler.prox_fn = NULL;

	return;
}

void get_f12_info(struct synaptics_ts_data *ts)
{
	int retval;
	struct synaptics_ts_f12_query_5 query_5;
	struct synaptics_ts_f12_query_8 query_8;
	struct synaptics_ts_f12_ctrl_23 ctrl_23;
	int i;
	u8 offset;

	if (!ts) {
		TOUCH_ERR_MSG("ts is null\n");
		return;
	}

/* ctrl_reg_info setting */
	retval = touch_i2c_read(ts->client, (ts->finger_fc.dsc.query_base + 5), sizeof(query_5.data), query_5.data);

	if (retval < 0) {
		TOUCH_ERR_MSG("Failed to read from F12_2D_QUERY_05_Control_Presence register\n");
		return;
	}

	f12_info.ctrl_reg_is_present[0] = query_5.ctrl_00_is_present;
	f12_info.ctrl_reg_is_present[1] = query_5.ctrl_01_is_present;
	f12_info.ctrl_reg_is_present[2] = query_5.ctrl_02_is_present;
	f12_info.ctrl_reg_is_present[3] = query_5.ctrl_03_is_present;
	f12_info.ctrl_reg_is_present[4] = query_5.ctrl_04_is_present;
	f12_info.ctrl_reg_is_present[5] = query_5.ctrl_05_is_present;
	f12_info.ctrl_reg_is_present[6] = query_5.ctrl_06_is_present;
	f12_info.ctrl_reg_is_present[7] = query_5.ctrl_07_is_present;
	f12_info.ctrl_reg_is_present[8] = query_5.ctrl_08_is_present;
	f12_info.ctrl_reg_is_present[9] = query_5.ctrl_09_is_present;
	f12_info.ctrl_reg_is_present[10] = query_5.ctrl_10_is_present;
	f12_info.ctrl_reg_is_present[11] = query_5.ctrl_11_is_present;
	f12_info.ctrl_reg_is_present[12] = query_5.ctrl_12_is_present;
	f12_info.ctrl_reg_is_present[13] = query_5.ctrl_13_is_present;
	f12_info.ctrl_reg_is_present[14] = query_5.ctrl_14_is_present;
	f12_info.ctrl_reg_is_present[15] = query_5.ctrl_15_is_present;
	f12_info.ctrl_reg_is_present[16] = query_5.ctrl_16_is_present;
	f12_info.ctrl_reg_is_present[17] = query_5.ctrl_17_is_present;
	f12_info.ctrl_reg_is_present[18] = query_5.ctrl_18_is_present;
	f12_info.ctrl_reg_is_present[19] = query_5.ctrl_19_is_present;
	f12_info.ctrl_reg_is_present[20] = query_5.ctrl_20_is_present;
	f12_info.ctrl_reg_is_present[21] = query_5.ctrl_21_is_present;
	f12_info.ctrl_reg_is_present[22] = query_5.ctrl_22_is_present;
	f12_info.ctrl_reg_is_present[23] = query_5.ctrl_23_is_present;
	f12_info.ctrl_reg_is_present[24] = query_5.ctrl_24_is_present;
	f12_info.ctrl_reg_is_present[25] = query_5.ctrl_25_is_present;
	f12_info.ctrl_reg_is_present[26] = query_5.ctrl_26_is_present;
	f12_info.ctrl_reg_is_present[27] = query_5.ctrl_27_is_present;
	f12_info.ctrl_reg_is_present[28] = query_5.ctrl_28_is_present;
	f12_info.ctrl_reg_is_present[29] = query_5.ctrl_29_is_present;
	f12_info.ctrl_reg_is_present[30] = query_5.ctrl_30_is_present;
	f12_info.ctrl_reg_is_present[31] = query_5.ctrl_31_is_present;

	offset = 0;

	for (i = 0; i < 32; i++) {
		f12_info.ctrl_reg_addr[i] = ts->finger_fc.dsc.control_base + offset;

		if (f12_info.ctrl_reg_is_present[i])
			offset++;
	}

/* data_reg_info setting */
	retval = touch_i2c_read(ts->client, (ts->finger_fc.dsc.query_base + 8), sizeof(query_8.data), query_8.data);

	if (retval < 0) {
		TOUCH_ERR_MSG("Failed to read from F12_2D_QUERY_08_Data_Presence register\n");
		return;
	}

	f12_info.data_reg_is_present[0] = query_8.data_00_is_present;
	f12_info.data_reg_is_present[1] = query_8.data_01_is_present;
	f12_info.data_reg_is_present[2] = query_8.data_02_is_present;
	f12_info.data_reg_is_present[3] = query_8.data_03_is_present;
	f12_info.data_reg_is_present[4] = query_8.data_04_is_present;
	f12_info.data_reg_is_present[5] = query_8.data_05_is_present;
	f12_info.data_reg_is_present[6] = query_8.data_06_is_present;
	f12_info.data_reg_is_present[7] = query_8.data_07_is_present;
	f12_info.data_reg_is_present[8] = query_8.data_08_is_present;
	f12_info.data_reg_is_present[9] = query_8.data_09_is_present;
	f12_info.data_reg_is_present[10] = query_8.data_10_is_present;
	f12_info.data_reg_is_present[11] = query_8.data_11_is_present;
	f12_info.data_reg_is_present[12] = query_8.data_12_is_present;
	f12_info.data_reg_is_present[13] = query_8.data_13_is_present;
	f12_info.data_reg_is_present[14] = query_8.data_14_is_present;
	f12_info.data_reg_is_present[15] = query_8.data_15_is_present;

	offset = 0;

	for (i = 0; i < 16; i++) {
		f12_info.data_reg_addr[i] = ts->finger_fc.dsc.data_base + offset;

		if (f12_info.data_reg_is_present[i])
			offset++;
	}

/* print info */
	for (i = 0; i < 32; i++) {
		if (f12_info.ctrl_reg_is_present[i])
			TOUCH_INFO_MSG("f12_info.ctrl_reg_addr[%d]=0x%02X\n", i, f12_info.ctrl_reg_addr[i]);
	}

	for (i = 0; i < 16; i++) {
		if (f12_info.data_reg_is_present[i])
			TOUCH_INFO_MSG("f12_info.data_reg_addr[%d]=0x%02X\n", i, f12_info.data_reg_addr[i]);
	}

	retval = touch_i2c_read(ts->client, f12_info.ctrl_reg_addr[23],
			sizeof(ctrl_23.data), ctrl_23.data);
	if (retval < 0) {
		TOUCH_ERR_MSG("Failed to read from F12_2D_CTRL_23 register\n");
		return;
	}

	ts->object_report = ctrl_23.obj_type_enable;

	TOUCH_INFO_MSG("ts->object_report[0x%x]\n", ts->object_report);

	return;
}


int synaptics_get_reference_chk(struct synaptics_ts_data *ts)
{
	int diff_v = 0;
	u16 pre_v = 0;
	int err_cnt_horizon = 0;
	static u8 *err_tot_x = NULL;
	static u8 *err_tot_y = NULL;
	u16 diff_max = 0;

	/* vertical values */

	int diff_vertical = 0;
	u16 prev_vertical = 0;
	int err_cnt_vertical = 0;
	u16 vertical_diff_max = 0;

	/* common values */

	int i = 0;
	static unsigned short *ImagepTest=NULL;
	static int rebase = 1;
	u16 curx = 0, cury = 0, ref_min = 5000, ref_max = 0;
	u8 rx_err_weight = 150;
	u8 tx_err_weight = 150;
	/*	u8 ref_err_cnt = 10; */
	u8 line_err_chk = 0;
	u8 err_line_num = 0;
	/*	static u8 err_chk = 0; */

	if (power_state == POWER_ON || power_state == POWER_WAKE) {
	        if (need_scan_pdt) {
	                touch_disable_irq(ts->client->irq);
	                SCAN_PDT();
	                need_scan_pdt = false;
	                touch_enable_irq(ts->client->irq);
	                TOUCH_INFO_MSG("SCAN_PDT\n");
	        }

	        if (rebase) {
	                TOUCH_INFO_MSG("ASSIGN MEMORY\n");
	                ASSIGN(err_tot_x = kzalloc(2*TxChannelCount * sizeof(u8), GFP_KERNEL), error);
	                ASSIGN(err_tot_y = kzalloc(2*RxChannelCount * sizeof(u8), GFP_KERNEL), error);
	                ASSIGN(ImagepTest = (unsigned short *)kmalloc(TxChannelCount * RxChannelCount * sizeof(unsigned short), GFP_KERNEL), error);

	        }


	        if (!half_err_cnt)
	                half_err_cnt = 10;

	        TOUCH_INFO_MSG("START check_diff_node\n");

	        if (ref_chk_rebase_stop) {
	                touch_disable_irq(ts->client->irq);
	        }

	        if (diffnode(ImagepTest) < 0) {
	                TOUCH_INFO_MSG("FAIL check_diff_node\n");
	                goto error;
	        }

	        TOUCH_INFO_MSG("END check_diff_node\n");

#ifdef REFERENCE_CHECK_LOG

	        int k = 0;

	        for (k = 0; k < TxChannelCount * RxChannelCount; k++) {
	                TOUCH_INFO_MSG("%5d", ImagepTest[k]);
	                if (k%RxChannelCount == RxChannelCount - 1)
	                        TOUCH_INFO_MSG("\n");
	        }
	        for (k = 0; k < TRX_MAX; k++)
	                TOUCH_INFO_MSG("pdata->rx_cap[%d] = %3d\n", k, ts->pdata->rx_cap[k]);
	        TOUCH_INFO_MSG("\n\n");
	        for (k = 0; k < TRX_MAX; k++)
	                TOUCH_INFO_MSG("pdata->tx_cap[%d] = %3d\n", k, ts->pdata->tx_cap[k]);

#endif

	   for (i = 0; i < TxChannelCount * RxChannelCount; i++) {

	        if (ref_min > ImagepTest[i]) {
	            ref_min = ImagepTest[i];
	        }
	        if (ref_max < ImagepTest[i]) {
	            ref_max = ImagepTest[i];
	        }

	        /* RX check MAX check */
			if (ts->pdata->ref_chk_option[3]) {
	                if ((cury == 0) || (cury == 13)) {
	                        pre_v = ImagepTest[i];
	                } else {
	                        diff_v = abs(ImagepTest[i] - pre_v);

	                        if (diff_v > abs(ts->pdata->rx_cap[cury-1]) + rx_err_weight) {
	                                err_cnt_horizon++;
	                                if ((err_tot_y[RxChannelCount+cury] == 0) || (err_tot_x[curx] == 0)) {
	                                        ++err_tot_x[curx];
	                                        ++err_tot_y[RxChannelCount+cury];
	                                }
	                                TOUCH_INFO_MSG("Rx_Err Node(%d, %d) raw<%d> prev<%d> diff_v<%d> err_tot_x<%d>\n",
	                                                curx, cury, ImagepTest[i], pre_v, diff_v, err_tot_x[curx]);
	                        }
	                }
	                pre_v = ImagepTest[i];

	                if (diff_max < diff_v) {
	                        diff_max = diff_v;
	                        if (diff_max > CAP_DIFF_MAX) {
	                                TOUCH_INFO_MSG("Err Rx_Cnt is %d, Tx_Cnt is %d, limit Cnt is %d\n", err_cnt_horizon, err_cnt_vertical, half_err_cnt);
	                                TOUCH_INFO_MSG("Diff max exceed H(set : %d) (diff_max : %d)  rebase_retry_cnt(%d)",
	                                                CAP_DIFF_MAX, diff_max, rebase_retry_cnt);
	                                if (rebase_retry_cnt >= 100) {
	                                        TOUCH_ERR_MSG("retry_chk\n");
	                                        rebase_retry_cnt = 0;
	                                        TOUCH_INFO_MSG("diff_horizon : over retry_chk : %d \n", rebase_retry_cnt);
	                                        goto no_cal;
	                                } else {
	                                        /* we need to recalibration.. */
	                                        TOUCH_INFO_MSG("WE NEED RECALIBRATION HORIZON\n");
	                                        rebase = 0;
	                                        rebase_retry_cnt++;
	                                        /* to avoid scheduling while atomic bug */
	                                        /*	ts->h_err_cnt = err_cnt_horizon; */
	                                        /*	ts->v_err_cnt = err_cnt_vertical; */
	                                        if (wq_rebase_lock) {
	                                                queue_delayed_work(touch_wq, &ts->diff_node_timer, msecs_to_jiffies(REBASE_DELAY));
	                                                return 0;
	                                        }
	                                }
	                        }
	                }
			} /* RX check MAX check END */

	        /* TX DIFF MAX check */
			if (ts->pdata->ref_chk_option[2]) {
	                if (curx == 0) {
	                        prev_vertical = ImagepTest[i];
	                } else {
	                        diff_vertical = abs(ImagepTest[i] - prev_vertical);
	                        if (diff_vertical > abs(ts->pdata->tx_cap[curx-1]) + tx_err_weight) {

	                                err_cnt_vertical++;
	                                if ((err_tot_x[TxChannelCount+curx] == 0) || (err_tot_y[cury] == 0)) {
	                                        ++err_tot_y[cury];
	                                        ++err_tot_x[TxChannelCount+curx];
	                                }
	                                TOUCH_INFO_MSG("Tx_Err Node(%d, %d) raw<%d> prev<%d> diff<%d> err_tot_y<%d> \n",
	                                                curx, cury, ImagepTest[i], prev_vertical, diff_vertical, err_tot_y[cury]);
	                        }
	                }

	                prev_vertical = ImagepTest[i - (RxChannelCount-1)];

	                if (vertical_diff_max < diff_vertical) {
	                        vertical_diff_max = diff_vertical;
	                        if (vertical_diff_max > CAP_DIFF_MAX) {
	                                TOUCH_INFO_MSG("Err Rx_Cnt is %d, Tx_Cnt is %d, limit Cnt is %d\n", err_cnt_horizon, err_cnt_vertical, half_err_cnt);
	                                TOUCH_INFO_MSG("Diff max exceed V(set : %d) (diff_max : %d)  rebase_retry_cnt(%d)",
	                                                CAP_DIFF_MAX, vertical_diff_max, rebase_retry_cnt);
	                                if (rebase_retry_cnt >= 100) {
	                                        rebase_retry_cnt = 0;
	                                        TOUCH_INFO_MSG("diff_vertical : over retry_chk_temp : %d \n", rebase_retry_cnt);
	                                        goto no_cal;
	                                } else {
	                                        /* we need to recalibration.. */
	                                        TOUCH_INFO_MSG("WE NEED RECALIBRATION VERTICAL\n");
	                                        rebase = 0;
	                                        rebase_retry_cnt++;
	                                        /* to avoid scheduling while atomic bug */
	                                        /*		ts->h_err_cnt = err_cnt_horizon; */
	                                        /*		ts->v_err_cnt = err_cnt_vertical; */
	                                        if (wq_rebase_lock) {
	                                                queue_delayed_work(touch_wq, &ts->diff_node_timer, msecs_to_jiffies(REBASE_DELAY));
	                                                return 0;
	                                        }
	                                }
	                        }
	                }
			}  /* TX DIFF MAX check END */

	                cury++;
	                if (cury >= RxChannelCount) {
	                        curx++;
	                        cury = 0;
	                }

	                if (i >= RxChannelCount * TxChannelCount)
						break;
		 }

	        /* reference max-min check (difference set to 400) */
	        if ((ref_max - ref_min) > CAP_MIN_MAX_DIFF) {
	                TOUCH_INFO_MSG("Reference Max(%d) - Min(%d) exceed (set : %d) (Max - Min : %d) rebase_retry_cnt(%d)"
	                                , ref_max, ref_min, CAP_MIN_MAX_DIFF, (ref_max - ref_min), rebase_retry_cnt);
	                if (rebase_retry_cnt >= 100) {
	                        rebase_retry_cnt = 0;
	                        TOUCH_INFO_MSG("reference max-min : over retry_chk_temp : %d \n", rebase_retry_cnt);
	                        goto no_cal;
	                } else {
	                        /*we need to recalibration.. */
	                        TOUCH_INFO_MSG("WE NEED RECALIBRATION Min Max\n");
	                        rebase = 0;
	                        rebase_retry_cnt++;
	                        /* to avoid scheduling while atomic bug */
	                        /*		ts->h_err_cnt = err_cnt_horizon; */
	                        /*		ts->v_err_cnt = err_cnt_vertical; */
	                        if (wq_rebase_lock) {
	                                queue_delayed_work(touch_wq, &ts->diff_node_timer, msecs_to_jiffies(REBASE_DELAY));
	                                return 0;
	                        }
	                }
	        }

	        TOUCH_INFO_MSG("Err Rx_Cnt is %d, Tx_Cnt is %d, limit Cnt is %d\n", err_cnt_horizon, err_cnt_vertical, half_err_cnt);
	        /*data->patch.src_item[MXT_PATCH_ITEM_USER6] = err_cnt; */
	        /*check for sd test*/
	        ts->h_err_cnt = err_cnt_horizon;
	        ts->v_err_cnt = err_cnt_vertical;
	        /*check for sd test*/

	        /* Half line of TX Line is fail */
	        for (i = 0; i < TxChannelCount; i++) {
	                if (err_tot_x[i] != 0) {
	                        line_err_chk++;
	                        err_line_num = i;
	                }

	                if (err_tot_x[i] >= RxChannelCount/2) {
	                        TOUCH_INFO_MSG("X%d Line all Fail, calibration skip error count(%d)\n", i, err_tot_x[i]);
	                        goto no_cal;
	                }
	        }
	        /* Half line of RX Line is fail */
	        for (i = 0; i < RxChannelCount ; i++) {
	                if (err_tot_y[i] != 0) {
	                        line_err_chk++;
	                        err_line_num = i;
	                }
	                if (err_tot_y[i] >= TxChannelCount/2) {
	                        TOUCH_INFO_MSG("Y%d Line all Fail, calibration skip error count(%d)\n", i, err_tot_y[i]);
	                        goto no_cal;
	                }
	        }
	        line_err_chk = 0; /* not use */
	        err_line_num = 0; /* not use */


	        /* check err node cnt over > 10 */
	        if ((err_cnt_horizon && (err_cnt_horizon >= half_err_cnt)) || (err_cnt_vertical && (err_cnt_vertical >= half_err_cnt))) {
	                TOUCH_INFO_MSG("Reference Err Calibration: H[%d] V[%d]\n", err_cnt_horizon, err_cnt_vertical);
	                if (rebase_retry_cnt >= 100) {
	                        TOUCH_INFO_MSG("over rebase_retry_cnt : %d \n", rebase_retry_cnt);
	                        rebase_retry_cnt = 0; /* rebase cnt reset */
	                        goto no_cal;
	                } else {
	                        err_cnt_horizon = 0; /* value reset */
	                        err_cnt_vertical = 0;
	                        rebase_retry_cnt++;
	                        rebase = 0;
	                        if (wq_rebase_lock) {
	                                queue_delayed_work(touch_wq, &ts->diff_node_timer, msecs_to_jiffies(REBASE_DELAY));
	                                return 0;
	                        }
	                }

	        }

	} else {

	        TOUCH_INFO_MSG("======= check_diff_node not power on/power wake =======\n");

	}

	TOUCH_INFO_MSG("END TEST\n");
	goto no_cal;
	return 0;

	error:
	TOUCH_INFO_MSG("error\n");
	touch_enable_irq(ts->client->irq);
	rebase = 1;
	kfree(err_tot_x);
	kfree(err_tot_y);
	kfree(ImagepTest);
	return -1;


	no_cal:
	TOUCH_INFO_MSG("no_cal\n");
	if (ref_chk_rebase_stop) {
	        touch_enable_irq(ts->client->irq);
	        ref_chk_rebase_stop = 0;
	}
	synaptics_ts_init(ts->client);
	rebase = 1;
	/* parameter initialize*/
	rebase_retry_cnt = 0;
	if (!wq_rebase_lock) {
	        wq_rebase_lock = 1;
	}
	/*	time_reset_chk_cnt = 0; */
	touch_enable_irq(ts->client->irq);
	kfree(err_tot_x);
	kfree(err_tot_y);
	kfree(ImagepTest);
	return 0;

	}

/* dajung Y */
static int synaptics_get_cap_diff(struct synaptics_ts_data *ts)
{
	int t_diff = 0;
        int r_diff = 0;
	int x = 0;
	int y = 0;
	int ret = 0;
	s8 *rx_cap_diff = NULL;
	s8 *tx_cap_diff = NULL;
	unsigned short *raw_cap = NULL;
/*	u8 ref_new_chk = 0; */
	char *f54_cap_wlog_buf = NULL;
	static int cap_outbuf;
	/* allocation of cap_diff */
	rx_cap_diff = NULL;
	tx_cap_diff = NULL;
	raw_cap = NULL;
	cap_outbuf = 0;

	ASSIGN(rx_cap_diff = kzalloc(RxChannelCount * sizeof(u8), GFP_KERNEL), error_mem);
	ASSIGN(tx_cap_diff = kzalloc(TxChannelCount * sizeof(u8), GFP_KERNEL), error_mem);
	ASSIGN(raw_cap = kzalloc(TxChannelCount*RxChannelCount * sizeof(unsigned char) * 2, GFP_KERNEL), error_mem);
	ASSIGN(f54_cap_wlog_buf = kzalloc(DS5_BUFFER_SIZE, GFP_KERNEL), error_mem);

	memset(f54_cap_wlog_buf, 0, DS5_BUFFER_SIZE);

	if (diffnode(raw_cap) < 0) {
		TOUCH_INFO_MSG("check_diff_node fail!!\n");
		kfree(rx_cap_diff);
		kfree(tx_cap_diff);
		kfree(raw_cap);
		kfree(f54_cap_wlog_buf);
	return -1;
	}

	ts->bad_sample = 0;
	for (y = 0; y < (int)RxChannelCount - 1; y++) {
		t_diff = 0;

		for (x = 0; x < (int)TxChannelCount; x++) {
			t_diff += (raw_cap[x * RxChannelCount + y + 1] - raw_cap[x * RxChannelCount + y]) ;
		}

        /* TOUCH_INFO_MSG("[Rx:%2d]rx_cap_diff[%d] %d ",y ,y + 1 ,t_diff); */
		t_diff = t_diff / (int)TxChannelCount;

		if (jitter_abs(t_diff) > 1000) {
			ts->bad_sample = 1;
		}

		if (t_diff < -0x7F) /* limit diff max */
			rx_cap_diff[y + 1] = -0x7F;
		else if (t_diff > 0x7F)
			rx_cap_diff[y + 1] = 0x7F;
		else
			rx_cap_diff[y + 1] = (s8)t_diff;

        /* TOUCH_INFO_MSG("rx_cap_diff[%d] %d\n",y + 1 ,rx_cap_diff[y + 1]); */
/* need to modify */
        cap_outbuf += sprintf(f54_cap_wlog_buf+cap_outbuf, "%5d \n", rx_cap_diff[y + 1]);
	}

	if (tx_cap_diff != NULL && ts->bad_sample == 0) {

		for (x = 0; x < (int)TxChannelCount - 1; x++) {
			r_diff = 0;
			for (y = 0; y < (int)RxChannelCount; y++) {
				r_diff += (raw_cap[(x + 1) * RxChannelCount + y] - raw_cap[x * RxChannelCount + y]) ;
			}

            /* TOUCH_INFO_MSG("[Tx:%2d]tx_cap_diff[%d] %d ", x, x + 1 ,r_diff); */
			r_diff = r_diff / (int)RxChannelCount;

			if (jitter_abs(r_diff) > 1000/*need to tunning limit_value*/) {
				ts->bad_sample = 1;
			}

			if (r_diff < -0x7F) /* limit diff max */
				tx_cap_diff[x + 1] = -0x7F;
			else if (r_diff > 0x7F)
				tx_cap_diff[x + 1] = 0x7F;
			else
				tx_cap_diff[x + 1] = (s8)r_diff;

			/* TOUCH_INFO_MSG("tx_cap_diff[%d] %d\n",x + 1 ,tx_cap_diff[x + 1]); */
/* need to modify */
            cap_outbuf += sprintf(f54_cap_wlog_buf+cap_outbuf, "%5d \n", tx_cap_diff[x + 1]);
		}
	}
/* need to modify */
     if (write_log(CAP_FILE_PATH, f54_cap_wlog_buf) == 1)
             raw_cap_file_exist = 1;
     read_log(CAP_FILE_PATH, ts->pdata);

/* average of Rx_line Cap Value */
	kfree(rx_cap_diff);
	kfree(tx_cap_diff);
	kfree(raw_cap);
	kfree(f54_cap_wlog_buf);
	return ret;

error_mem:
	TOUCH_INFO_MSG("error_mem\n");
	return -1;

}

static char *productcode_parse(unsigned char *product)
{
	static char str[128] = {0};
	int len = 0;
	char inch[2] = {0};
	char paneltype = 0;
	char version[2] = {0};

	switch ((product[0] & 0xF0) >> 4) {
	case 0:
		len += sprintf(str+len, "ELK ");
		break;
	case 1:
		len += sprintf(str+len, "Suntel ");
		break;
	case 2:
		len += sprintf(str+len, "Tovis ");
		break;
	case 3:
		len += sprintf(str+len, "Innotek ");
		break;
	default:
		len += sprintf(str+len, "Unknown ");
		break;
	}

	len += sprintf(str+len, "\n");

	switch (product[0] & 0x0F) {
	case 0:
		len += sprintf(str+len, "0key ");
		break;
	case 2:
		len += sprintf(str+len, "2Key ");
		break;
	case 3:
		len += sprintf(str+len, "3Key ");
		break;
	case 4:
		len += sprintf(str+len, "4Key ");
		break;
	default:
		len += sprintf(str+len, "Unknown ");
		break;
	}

	len += sprintf(str+len, "\n");

	switch ((product[1] & 0xF0) >> 4) {
	case 0:
		len += sprintf(str+len, "Synaptics ");
		break;
	default:
		len += sprintf(str+len, "Unknown ");
		break;
	}

	len += sprintf(str+len, "\n");

	inch[0] = (product[1] & 0x0F);
	inch[1] = ((product[2] & 0xF0) >> 4);
	len += sprintf(str+len, "%d.%d ", inch[0], inch[1]);

	len += sprintf(str+len, "\n");

	paneltype = (product[2] & 0x0F);
	len += sprintf(str+len, "PanelType %d ", paneltype);

	len += sprintf(str+len, "\n");

	version[0] = ((product[3] & 0x80) >> 7);
	version[1] = (product[3] & 0x7F);
	len += sprintf(str+len, "version : v%d.%02d\n", version[0], version[1]);

	return str;
 }
static void lpwg_timer_func(struct work_struct *work_timer)
{
	struct synaptics_ts_data *ts = container_of(to_delayed_work(work_timer),
		struct synaptics_ts_data, work_timer);

	if (mfts_enable) {
		return;
	}

	sleep_control(ts, 0, 1); /* sleep until receive the reply. */
	send_uevent_lpwg(ts->client, LPWG_PASSWORD);
	wake_unlock(&ts->timer_wake_lock);

	TOUCH_DEBUG(DEBUG_LPWG, "u-event timer occur!\n");
	return;
}

static void diff_node_timer_func(struct work_struct *diff_node_timer)
{
	struct synaptics_ts_data *ts = container_of(to_delayed_work(diff_node_timer),
		struct synaptics_ts_data, diff_node_timer);

	int ret = 0;
	TOUCH_INFO_MSG("diff_node_timer_func\n");

	if ((power_state == POWER_ON || power_state == POWER_WAKE) && (ref_chk_rebase_stop)) {
		if (synaptics_ts_ic_ctrl(ts->client, IC_CTRL_BASELINE_REBASE, FORCE_CAL, &ret) < 0) {
			TOUCH_ERR_MSG("IC_CTRL_REBASE handling fail\n");
		}
	} else {
	/* to avoid interrupt hw key suspend when doing rebase */
			synaptics_ts_init(ts->client);
		TOUCH_INFO_MSG("DO NOTHING\n");
	}

	return;

}
static void read_cap_diff_file(struct work_struct *cap_diff)
{
	struct synaptics_ts_data *ts = container_of(to_delayed_work(cap_diff),
		struct synaptics_ts_data, cap_diff);

	TOUCH_INFO_MSG("[%s] queue_delayed_work ->cap_diff success!!\n", __FUNCTION__);

	/* if file(sns_cap_diff.txt) exist, read_log */
	if (sys_access(CAP_FILE_PATH, 0) == 0) {
	    TOUCH_INFO_MSG("[%s] %s file exist!\n", __FUNCTION__, CAP_FILE_PATH);
	    /*SCAN_PDT();*/
	    read_log(CAP_FILE_PATH, ts->pdata);
		raw_cap_file_exist = 1; /*if file exist, do ref chk func*/
	} else {
	    TOUCH_INFO_MSG("[%s] %s file doesn't exist!\n", __FUNCTION__, CAP_FILE_PATH);
	}

	return;

}

static void all_palm_released_func(struct work_struct *work_palm)
{
	struct synaptics_ts_data *ts = container_of(to_delayed_work(work_palm),
			struct synaptics_ts_data, work_palm);

	ts->ts_palm_data.all_palm_released = false;
	TOUCH_INFO_MSG("%s: ABS0 event disable time is expired.\n", __func__);

	return;
}

static ssize_t show_firmware(struct i2c_client *client, char *buf)
{
	struct synaptics_ts_data *ts =
			(struct synaptics_ts_data *)get_touch_handle(client);
	int ret = 0;
	int rc = 0;
	mfts_mode = 0;

	read_page_description_table(ts->client);
	rc = get_ic_info(ts);
	if (rc < 0) {
		ret += sprintf(buf+ret, "-1\n");
		ret += sprintf(buf+ret, "Read Fail Touch IC Info.\n");
		return ret;
	}

	ret = sprintf(buf, "\n======== Firmware Info ========\n");
	if (ts->fw_info.fw_version[0] > 0x40)
		ret += sprintf(buf+ret, "ic_fw_version[%s]\n", ts->fw_info.fw_version);
	else {
		ret += sprintf(buf+ret, "ic_fw_version RAW = %02X %02X %02X %02X\n",
			ts->fw_info.fw_version[0], ts->fw_info.fw_version[1],
			ts->fw_info.fw_version[2], ts->fw_info.fw_version[3]);
		ret += sprintf(buf+ret, "=== ic_fw_version info === \n%s", productcode_parse(ts->fw_info.fw_version));
	}
	ret += sprintf(buf+ret, "IC_product_id[%s]\n", ts->fw_info.fw_product_id);
	if (!(strncmp(ts->fw_info.fw_product_id, "PLG313", 6)) || !(strncmp(ts->fw_info.fw_product_id, "PLG352", 6)) || !(strncmp(ts->fw_info.fw_product_id, "PLG391", 6))) {
		if (ts->fw_info.family && ts->fw_info.fw_revision) {
			ret += sprintf(buf+ret, "Touch IC : s3528(family_id = %d, fw_rev = %d)\n\n",
							ts->fw_info.family, ts->fw_info.fw_revision);
		} else {
			ret += sprintf(buf+ret, "Touch IC : s3528(family_id = %d, fw_rev = %d)\n\n",
							ts->fw_info.family, ts->fw_info.fw_revision);
		}
	} else if (!(strncmp(ts->fw_info.fw_product_id, "PLG298", 6))) {
		ret += sprintf(buf+ret, "Touch IC : s3621\n\n");
	} else {
		ret += sprintf(buf+ret, "Touch product id read error\n\n");
	}

	if (ts->fw_info.fw_image_version[0] > 0x40)
		ret += sprintf(buf+ret, "img_fw_version[%s]\n", ts->fw_info.fw_image_version);
	else {
		ret += sprintf(buf+ret, "img_fw_version RAW = %02X %02X %02X %02X\n",
			ts->fw_info.fw_image_version[0], ts->fw_info.fw_image_version[1],
			ts->fw_info.fw_image_version[2], ts->fw_info.fw_image_version[3]);
		ret += sprintf(buf+ret, "=== img_fw_version info === \n%s", productcode_parse(ts->fw_info.fw_image_version));
	}
	ret += sprintf(buf+ret, "Img_product_id[%s]\n", ts->fw_info.fw_image_product_id);
	if (!(strncmp(ts->fw_info.fw_image_product_id, "PLG313", 6)) || !(strncmp(ts->fw_info.fw_image_product_id, "PLG352", 6)) || !(strncmp(ts->fw_info.fw_image_product_id, "PLG391", 6))) {
		if (ts->fw_info.family && ts->fw_info.fw_revision) {
			ret += sprintf(buf+ret, "Touch IC : s3528(family_id = %d, fw_rev = %d)\n",
							ts->fw_info.family, ts->fw_info.fw_revision);
		} else {
			ret += sprintf(buf+ret, "Touch IC : s3528(family_id = %d, fw_rev = %d)\n",
							ts->fw_info.family, ts->fw_info.fw_revision);
		}
	} else if (!(strncmp(ts->fw_info.fw_product_id, "PLG298", 6))) {
		ret += sprintf(buf+ret, "Touch IC : s3621\n\n");
	} else {
		ret += sprintf(buf+ret, "Touch product id read error\n\n");
	}

	return ret;
}

static ssize_t show_synaptics_fw_version(struct i2c_client *client, char *buf)
{
	struct synaptics_ts_data *ts =
			(struct synaptics_ts_data *)get_touch_handle(client);
	int ret = 0;
	int rc = 0;

	read_page_description_table(ts->client);
	rc = get_ic_info(ts);
	if (rc < 0) {
		ret += sprintf(buf+ret, "-1\n");
		ret += sprintf(buf+ret, "Read Fail Touch IC Info.\n");
		return ret;
	}

	ret = sprintf(buf, "\n======== Auto Touch Test ========\n");
	if (ts->fw_info.fw_version[0] > 0x40)
		ret += sprintf(buf+ret, "ic_fw_version[%s]\n", ts->fw_info.fw_version);
	else {
		ret += sprintf(buf+ret, "version : v%d.%02d\n", ((ts->fw_info.fw_version[3] & 0x80) >> 7), (ts->fw_info.fw_version[3] & 0x7F));
	}
	ret += sprintf(buf+ret, "IC_product_id[%s]\n", ts->fw_info.fw_product_id);
	if (!(strncmp(ts->fw_info.fw_product_id, "PLG313", 6)) || !(strncmp(ts->fw_info.fw_product_id, "PLG352", 6)) || !(strncmp(ts->fw_info.fw_product_id, "PLG391", 6))) {
		if (ts->fw_info.family && ts->fw_info.fw_revision) {
			ret += sprintf(buf+ret, "Touch IC : s3528(family_id = %d, fw_rev = %d)\n\n",
							ts->fw_info.family, ts->fw_info.fw_revision);
		} else {
			ret += sprintf(buf+ret, "Touch IC : s3528(family_id = %d, fw_rev = %d)\n\n",
							ts->fw_info.family, ts->fw_info.fw_revision);
		}
	} else if (!(strncmp(ts->fw_info.fw_product_id, "PLG298", 6))) {
		ret += sprintf(buf+ret, "Touch IC : s3621\n\n");
	} else {
		ret += sprintf(buf+ret, "Touch product id read error\n\n");
	}

	return ret;
}

static ssize_t show_sd(struct i2c_client *client, char *buf)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);

	int ret = 0;
	int full_raw_cap = 0;
	int trx_to_trx = 0;
	int high_resistance = 0;
	int adc_range = 0;
	int sensor_speed = 0;
	int lower_img = 0;
	int upper_img = 0;
	int lower_sensor = 0;
	int upper_sensor = 0;
	int lower_adc = 0;
	int upper_adc = 0;
	char *temp_buf = NULL;
	int len = 0;

	if (power_state == POWER_ON || power_state == POWER_WAKE) {

		temp_buf = kzalloc(100, GFP_KERNEL);
		if (!temp_buf) {
			TOUCH_INFO_MSG("%s Failed to allocate memory\n", __func__);
			return 0;
		}
		write_time_log(NULL, NULL, 0);
		msleep(10);
		write_firmware_version_log(ts);

		touch_disable_irq(ts->client->irq);

		SCAN_PDT();

		if (!(strncmp(ts->fw_info.fw_product_id, "PLG391", 6))) {
			lower_img = get_limit(TxChannelCount, RxChannelCount, *ts->client, ts->pdata, "LowerImageLimitSuntel", LowerImage);
			upper_img = get_limit(TxChannelCount, RxChannelCount, *ts->client, ts->pdata, "UpperImageLimitSuntel", UpperImage);
		} else {
			lower_img = get_limit(TxChannelCount, RxChannelCount, *ts->client, ts->pdata, "LowerImageLimit", LowerImage);
			upper_img = get_limit(TxChannelCount, RxChannelCount, *ts->client, ts->pdata, "UpperImageLimit", UpperImage);
		}

		lower_sensor = get_limit(TxChannelCount, RxChannelCount, *ts->client, ts->pdata, "SensorSpeedLowerImageLimit", SensorSpeedLowerImage);
		upper_sensor = get_limit(TxChannelCount, RxChannelCount, *ts->client, ts->pdata, "SensorSpeedUpperImageLimit", SensorSpeedUpperImage);
		lower_adc = get_limit(TxChannelCount, RxChannelCount, *ts->client, ts->pdata, "ADCLowerImageLimit", ADCLowerImage);
		upper_adc = get_limit(TxChannelCount, RxChannelCount, *ts->client, ts->pdata, "ADCUpperImageLimit", ADCUpperImage);

		if (lower_img < 0 || upper_img < 0 ) {
			TOUCH_INFO_MSG("[%s] lower return = %d upper return = %d\n", __FUNCTION__, lower_img, upper_img);
			TOUCH_INFO_MSG("[%s][FAIL] Can not check the limit of raw cap\n", __FUNCTION__);
			ret = sprintf(buf+ret, "Can not check the limit of raw cap\n");
		} else {
			TOUCH_INFO_MSG("[%s] lower return = %d upper return = %d\n", __FUNCTION__, lower_img, upper_img);
			TOUCH_INFO_MSG("[%s][SUCCESS] Can check the limit of raw cap\n", __FUNCTION__);
			full_raw_cap = F54Test('a', 0, buf);
			if (ts->pdata->ref_chk_option[0]) {
				msleep(30);
				synaptics_get_cap_diff(ts);
			}
			msleep(30);
		}

		trx_to_trx = F54Test('f', 0, buf);
		high_resistance = F54Test('g', 0, buf);
		msleep(50);
		if (lower_sensor < 0 || upper_sensor < 0 ) {
			TOUCH_INFO_MSG("[%s] lower return = %d upper return = %d\n", __FUNCTION__, lower_sensor, upper_sensor);
			TOUCH_INFO_MSG("[%s][FAIL] Can not check the limit of sensor speed image\n", __FUNCTION__);
			ret = sprintf(buf+ret, "Can not check the limit of sensor speed image limit\n");
		} else {
			TOUCH_INFO_MSG("[%s] lower return = %d upper return = %d\n", __FUNCTION__, lower_adc, upper_adc);
			TOUCH_INFO_MSG("[%s][SUCCESS] Can check the limit of sensor speed image limit\n", __FUNCTION__);
			sensor_speed = F54Test('c', 0, buf);
		}
		msleep(50);

		if (lower_adc < 0 || upper_adc < 0 ) {
			TOUCH_INFO_MSG("[%s] lower return = %d upper return = %d\n", __FUNCTION__, lower_adc, upper_adc);
			TOUCH_INFO_MSG("[%s][FAIL] Can not check the limit of ADC image\n", __FUNCTION__);
			ret = sprintf(buf+ret, "Can not check the limit of ADC image limit\n");
		} else {
			TOUCH_INFO_MSG("[%s] lower return = %d upper return = %d\n", __FUNCTION__, lower_sensor, upper_sensor);
			TOUCH_INFO_MSG("[%s][SUCCESS] Can check the limit of ADC image limit\n", __FUNCTION__);
			adc_range = F54Test('b', 0, buf);
		}


		synaptics_ts_init(ts->client);

		if (raw_cap_file_exist && ts->pdata->ref_chk_option[0]) {
			/*status change to operate ref_chk*/
			if ((ts->fw_info.family && ts->fw_info.fw_revision) || !(strncmp(ts->fw_info.fw_product_id, "PLG352", 6))) {
				ref_chk_rebase_stop = 1;
				wq_rebase_lock = 0;
				if (synaptics_get_reference_chk(ts) < 0) {
					TOUCH_INFO_MSG("ref_chk fail!!\n");
				}
			} else {
				touch_enable_irq(ts->client->irq);
			}
		} else {
			touch_enable_irq(ts->client->irq);
		}

		msleep(30);


		if (ts->h_err_cnt || ts->v_err_cnt || ts->bad_sample) {

			full_raw_cap = 0;

		}

		len += snprintf(temp_buf + len, PAGE_SIZE - len, "Cap Diff : %s\n", ts->bad_sample == 0 ? "PASS" : "FAIL");
		if ((ts->fw_info.family && ts->fw_info.fw_revision) || !(strncmp(ts->fw_info.fw_product_id, "PLG352", 6))) {
			len += snprintf(temp_buf + len, PAGE_SIZE - len, "Error node Check h_err_cnt: %s(err count:%d)\n", (ts->h_err_cnt == 0 ? "PASS" : "FAIL"), ts->h_err_cnt);
			len += snprintf(temp_buf + len, PAGE_SIZE - len, "Error node Check v_err_cnt: %s(err count:%d)\n\n", (ts->v_err_cnt == 0 ? "PASS" : "FAIL"), ts->v_err_cnt);
		}
		write_log(NULL, temp_buf);
		msleep(30);
		write_time_log(NULL, NULL, 0);
		msleep(10);


		ret = sprintf(buf, "========RESULT=======\n");

		ret += sprintf(buf+ret, "Channel Status : %s", (trx_to_trx &&  high_resistance) ? "Pass\n" : "Fail");

		if (!(trx_to_trx && high_resistance)) {
			ret += sprintf(buf+ret, " (");
			ret += sprintf(buf+ret, "%s /%s", (trx_to_trx == 0 ? " 0" : " 1"), (high_resistance == 0 ? " 0" : " 1"));
			ret += sprintf(buf+ret, " )\n");
		}

		ret += sprintf(buf+ret, "Raw Data : %s", (full_raw_cap > 0) ? "Pass\n" : "Fail\n");


	} else {
		write_time_log(NULL, NULL, 0);
		ret += sprintf(buf+ret, "state=[suspend]. we cannot use I2C, now. Test Result: Fail\n");
	}
	if (temp_buf)
		kfree(temp_buf);

	return ret;
}

static ssize_t show_rawdata(struct i2c_client *client, char *buf)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);

	int ret = 0;
	int lower_ret = 0;
	int upper_ret = 0;

	if (power_state == POWER_ON || power_state == POWER_WAKE) {

		if (need_scan_pdt) {
			SCAN_PDT();
			need_scan_pdt = false;
		}
		touch_disable_irq(ts->client->irq);

		lower_ret = get_limit(TxChannelCount, RxChannelCount, *ts->client, ts->pdata, "LowerImageLimit", LowerImage);
		upper_ret = get_limit(TxChannelCount, RxChannelCount, *ts->client, ts->pdata, "UpperImageLimit", UpperImage);

		if (lower_ret < 0 || upper_ret < 0) {
			TOUCH_INFO_MSG("[%s] lower return = %d upper return = %d\n", __FUNCTION__, lower_ret, upper_ret);
			TOUCH_INFO_MSG("[%s] [FAIL] Can not check the limit of raw cap\n", __FUNCTION__);
			ret = sprintf(buf+ret, "Can not check the limit of raw cap\n");
		} else {
			TOUCH_INFO_MSG("[%s] lower return = %d upper return = %d\n", __FUNCTION__, lower_ret, upper_ret);
			TOUCH_INFO_MSG("[%s] [SUCCESS] Can check the limit of raw cap\n", __FUNCTION__);
			ret = F54Test('a', 1, buf);
		}

		touch_enable_irq(ts->client->irq);

		synaptics_ts_init(ts->client);
	} else {
		ret += sprintf(buf+ret, "state=[suspend]. we cannot use I2C, now. Test Result: Fail\n");
	}

	if (ret == 0)
		ret += sprintf(buf+ret, "ERROR: full_raw_cap failed.\n" );

	return ret;
}

static ssize_t show_delta(struct i2c_client *client, char *buf)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);

	int ret = 0;

	if (power_state == POWER_ON || power_state == POWER_WAKE) {

		if (need_scan_pdt) {
			SCAN_PDT();
			need_scan_pdt = false;
		}
		touch_disable_irq(ts->client->irq);

		ret = F54Test('m', 0, buf);

		touch_enable_irq(ts->client->irq);
		synaptics_ts_init(ts->client);

	} else {
		ret += sprintf(buf+ret, "state=[suspend]. we cannot use I2C, now. Test Result: Fail\n");
	}

	if (ret == 0)
		ret += sprintf(buf+ret, "ERROR: full_raw_cap failed.\n" );

	return ret;
}

static ssize_t show_chstatus(struct i2c_client *client, char *buf)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);

	int ret = 0;
	int high_resistance = 0;
	int trx_to_trx = 0;

	if (power_state == POWER_ON || power_state == POWER_WAKE) {
		touch_disable_irq(ts->client->irq);

		if (need_scan_pdt) {
			SCAN_PDT();
			need_scan_pdt = false;
		}
		high_resistance = F54Test('g', 0, buf);
		trx_to_trx = F54Test('f', 0, buf);

		ret = sprintf(buf, "========RESULT=======\n");

		ret += sprintf(buf+ret, "TRex Short Report : RESULT: %s", (trx_to_trx > 0) ? "Pass\n" : "Fail\n");


		/*High Resistance always return fail, you should see raw data.*/
		ret += sprintf(buf+ret, "High Resistance   : RESULT: %s", (high_resistance > 0) ? "Pass\n" : "Fail\n");

		synaptics_ts_init(ts->client);

		touch_enable_irq(ts->client->irq);

	} else {
		ret += sprintf(buf+ret, "state=[suspend]. we cannot use I2C, now. Test Result: Fail\n");
	}

	return ret;
}

static ssize_t show_abs_test(struct i2c_client *client, char *buf)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);

	int ret = 0;
	int abs_raw_short = 0;
	int abs_raw_open = 0;

	if (power_state == POWER_ON || power_state == POWER_WAKE) {
		touch_disable_irq(ts->client->irq);

		if (need_scan_pdt) {
			SCAN_PDT();
			need_scan_pdt = false;
		}
		abs_raw_short = F54Test('n', 1, buf);
		abs_raw_open = F54Test('o', 2, buf);

		ret = sprintf(buf, "========RESULT=======\n");

		ret += sprintf(buf+ret, "Absolute Sensing Short Test : RESULT: %s", (abs_raw_short > 0) ? "Pass\n" : "Fail\n");

		ret += sprintf(buf+ret, "Absolute Sensing Open Test  : RESULT: %s", (abs_raw_open > 0) ? "Pass\n" : "Fail\n");

		synaptics_ts_init(ts->client);

		touch_enable_irq(ts->client->irq);

	} else {
		ret += sprintf(buf+ret, "state=[suspend]. we cannot use I2C, now. Test Result: Fail\n");
	}

	return ret;
}
static ssize_t show_sensor_speed_test(struct i2c_client *client, char *buf)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);

	int ret = 0;
	int sensor_speed = 0;
	int lower_ret = 0;
	int upper_ret = 0;

	if (power_state == POWER_ON || power_state == POWER_WAKE) {
		touch_disable_irq(ts->client->irq);

		if (need_scan_pdt) {
			SCAN_PDT();
			need_scan_pdt = false;
		}

		lower_ret = get_limit(TxChannelCount, RxChannelCount, *ts->client, ts->pdata, "SensorSpeedLowerImageLimit", SensorSpeedLowerImage);
		upper_ret = get_limit(TxChannelCount, RxChannelCount, *ts->client, ts->pdata, "SensorSpeedUpperImageLimit", SensorSpeedUpperImage);

		if(lower_ret < 0 || upper_ret < 0) {
			TOUCH_INFO_MSG("[%s] lower return = %d upper return = %d\n", __FUNCTION__, lower_ret, upper_ret);
			TOUCH_INFO_MSG("[%s][FAIL] Can not check the limit of sensor speed image\n", __FUNCTION__);
			ret = sprintf(buf+ret, "Can not check the limit of sensor speed image limit\n");
		} else {
			TOUCH_INFO_MSG("[%s] lower return = %d upper return = %d\n", __FUNCTION__, lower_ret, upper_ret);
			TOUCH_INFO_MSG("[%s][SUCCESS] Can check the limit of sensor speed image limit\n", __FUNCTION__);
			sensor_speed = F54Test('c', 0, buf);
		}

		ret = sprintf(buf, "========RESULT=======\n");

		ret += sprintf(buf+ret, "Sensor Speed Test : RESULT: %s", ( sensor_speed > 0) ? "Pass\n" : "Fail\n");

		synaptics_ts_init(ts->client);

		touch_enable_irq(ts->client->irq);

	} else {
		ret += sprintf(buf+ret, "state=[suspend]. we cannot use I2C, now. Test Result: Fail\n");
	}

	return ret;
}
static ssize_t show_adc_range_test(struct i2c_client *client, char *buf)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);

	int ret = 0;
	int adc_range = 0;
	int lower_ret = 0;
	int upper_ret = 0;

	if (power_state == POWER_ON || power_state == POWER_WAKE) {
		touch_disable_irq(ts->client->irq);


		if (need_scan_pdt) {
			SCAN_PDT();
			need_scan_pdt = false;
		}
		lower_ret = get_limit(TxChannelCount, RxChannelCount, *ts->client, ts->pdata, "ADCLowerImageLimit", ADCLowerImage);
		upper_ret = get_limit(TxChannelCount, RxChannelCount, *ts->client, ts->pdata, "ADCUpperImageLimit", ADCUpperImage);

		if(lower_ret < 0 || upper_ret < 0) {
			TOUCH_INFO_MSG("[%s] lower return = %d upper return = %d\n", __FUNCTION__, lower_ret, upper_ret);
			TOUCH_INFO_MSG("[%s][FAIL] Can not check the limit of ADC image\n", __FUNCTION__);
			ret = sprintf(buf+ret, "Can not check the limit of ADC image limit\n");
		} else {
			TOUCH_INFO_MSG("[%s] lower return = %d upper return = %d\n", __FUNCTION__, lower_ret, upper_ret);
			TOUCH_INFO_MSG("[%s][SUCCESS] Can check the limit of ADC image limit\n", __FUNCTION__);
			adc_range = F54Test('b', 0, buf);
		}

		ret = sprintf(buf, "========RESULT=======\n");

		ret += sprintf(buf+ret, "ADC Range Test : RESULT: %s", (adc_range > 0) ? "Pass\n" : "Fail\n");

		synaptics_ts_init(ts->client);

		touch_enable_irq(ts->client->irq);

	} else {
		ret += sprintf(buf+ret, "state=[suspend]. we cannot use I2C, now. Test Result: Fail\n");
	}

	return ret;
}
/*
 * show_atcmd_fw_ver
 *
 * show only firmware version.
 * It will be used for AT-COMMAND
 */
static ssize_t show_atcmd_fw_ver(struct i2c_client *client, char *buf)
{
	struct synaptics_ts_data *ts =
			(struct synaptics_ts_data *)get_touch_handle(client);

    int ret = 0;

    if (ts->fw_info.fw_version[0] > 0x40) {
		ret += sprintf(buf+ret, "%s\n", ts->fw_info.fw_version);
	} else {
		ret = sprintf(buf, "V%d.%02d (0x%X/0x%X/0x%X/0x%X)\n",
			(ts->fw_info.fw_version[3]&0x80 ? 1 : 0), ts->fw_info.fw_version[3]&0x7F,
			ts->fw_info.fw_version[0], ts->fw_info.fw_version[1],
			ts->fw_info.fw_version[2], ts->fw_info.fw_version[3]);
	}

	return ret;
}

static ssize_t store_tci(struct i2c_client *client, const char *buf, size_t count)
{
	struct synaptics_ts_data *ts = (struct synaptics_ts_data *)get_touch_handle(client);
	u32 type = 0, value = 0;

	sscanf(buf, "%d %d", &type, &value);

	tci_control(ts, type, (u8)value);

	return count;
}

static ssize_t show_tci(struct i2c_client *client, char *buf)
{
	int ret = 0;
	u8 buffer[5] = {0};

	touch_i2c_read(client, f12_info.ctrl_reg_addr[20], 3, buffer);
	ret += sprintf(buf+ret, "report_mode [%s]\n", (buffer[2] & 0x3) == 0x2 ? "WAKEUP_ONLY" : "NORMAL");
	touch_i2c_read(client, f12_info.ctrl_reg_addr[27], 1, buffer);
	ret += sprintf(buf+ret, "wakeup_gesture [%d]\n", buffer[0]);


	synaptics_ts_page_data_read(client, LPWG_PAGE, LPWG_TAPCOUNT_REG, 5, buffer);
	ret += sprintf(buf+ret, "TCI [%s]\n", (buffer[0] & 0x1) == 1 ? "enabled" : "disabled");
	ret += sprintf(buf+ret, "Tap Count [%d]\n", (buffer[0] & 0xf8) >> 3);
	ret += sprintf(buf+ret, "Min InterTap [%d]\n", buffer[1]);
	ret += sprintf(buf+ret, "Max InterTap [%d]\n", buffer[2]);
	ret += sprintf(buf+ret, "Touch Slop [%d]\n", buffer[3]);
	ret += sprintf(buf+ret, "Tap Distance [%d]\n", buffer[4]);

	return ret;
}

static ssize_t store_reg_ctrl(struct i2c_client *client, const char *buf, size_t count)
{
	u8 buffer[50] = {0};
	char command[6] = {0};
	int page = 0;
	int reg = 0;
	int offset = 0;
	int value = 0;

	sscanf(buf, "%s %d %d %d %d ", command, &page, &reg, &offset, &value);

	if (!strcmp(command, "write")) {
		synaptics_ts_page_data_read(client, page, reg, offset+1, buffer);
		buffer[offset] = (u8)value;
		synaptics_ts_page_data_write(client, page, reg, offset+1, buffer);
	} else if (!strcmp(command, "read")) {
		synaptics_ts_page_data_read(client, page, reg, offset+1, buffer);
		TOUCH_DEBUG(DEBUG_BASE_INFO, "page[%d] reg[%d] offset[%d] = 0x%x\n", page, reg, offset, buffer[offset]);
	} else {
		TOUCH_DEBUG(DEBUG_BASE_INFO, "Usage\n");
		TOUCH_DEBUG(DEBUG_BASE_INFO, "Write page reg offset value\n");
		TOUCH_DEBUG(DEBUG_BASE_INFO, "Read page reg offset\n");
	}
	return count;
}

static ssize_t show_object_report(struct i2c_client *client, char *buf)
{
	int ret = 0;
	u8 object_report_enable_reg_addr;
	u8 object_report_enable_reg;

	object_report_enable_reg_addr = f12_info.ctrl_reg_addr[23];

	ret = touch_i2c_read(client, object_report_enable_reg_addr, sizeof(object_report_enable_reg), &object_report_enable_reg);

	if (ret < 0)
		ret = sprintf(buf, "%s: Failed to read object_report_enable register\n", __func__);
	else {
		u8 temp[8];
		int i;

		for (i = 0; i < 8; i++) {
			temp[i] = (object_report_enable_reg >> i) & 0x01;
		}

		ret = sprintf(buf, "\n============ read object_report_enable register ============\n");
		ret += sprintf(buf+ret, " Addr  Bit7  Bit6  Bit5  Bit4  Bit3  Bit2  Bit1  Bit0  HEX\n");
		ret += sprintf(buf+ret, "------------------------------------------------------------\n");
		ret += sprintf(buf+ret, " 0x%02X   %d     %d     %d     %d     %d     %d     %d     %d    0x%02X\n",
				object_report_enable_reg_addr, temp[7], temp[6], temp[5], temp[4], temp[3], temp[2], temp[1], temp[0], object_report_enable_reg);
		ret += sprintf(buf+ret, "------------------------------------------------------------\n");
		ret += sprintf(buf+ret, " Bit0     :     [F]inger                  ->     %s\n", temp[0] ? "Enable" : "Disable");
		ret += sprintf(buf+ret, " Bit1     :     [S]tylus                  ->     %s\n", temp[1] ? "Enable" : "Disable");
		ret += sprintf(buf+ret, " Bit2     :     [P]alm                    ->     %s\n", temp[2] ? "Enable" : "Disable");
		ret += sprintf(buf+ret, " Bit3     :     [U]nclassified Object     ->     %s\n", temp[3] ? "Enable" : "Disable");
		ret += sprintf(buf+ret, " Bit4     :     [H]overing Finger         ->     %s\n", temp[4] ? "Enable" : "Disable");
		ret += sprintf(buf+ret, " Bit5     :     [G]loved Finger           ->     %s\n", temp[5] ? "Enable" : "Disable");
		ret += sprintf(buf+ret, " Bit6     :     [N]arrow Object Swipe     ->     %s\n", temp[6] ? "Enable" : "Disable");
		ret += sprintf(buf+ret, " Bit7     :     Hand[E]dge                ->     %s\n", temp[7] ? "Enable" : "Disable");
		ret += sprintf(buf+ret, "============================================================\n\n");
	}

	return ret;
}

static ssize_t store_object_report(struct i2c_client *client, const char *buf, size_t count)
{
	int ret;
	char select[16];
	u8 value = 2;
	int select_cnt;
	int i;
	u8 bit_select = 0;
	u8 object_report_enable_reg_addr;
	u8 object_report_enable_reg_old;
	u8 object_report_enable_reg_new;
	u8 old[8];
	u8 new[8];

	sscanf(buf, "%s %hhu", select, &value);

	if ((strlen(select) > 8) || (value > 1)) {
		TOUCH_INFO_MSG("<writing object_report guide>\n");
		TOUCH_INFO_MSG("echo [select] [value] > object_report\n");
		TOUCH_INFO_MSG("select: [F]inger, [S]tylus, [P]alm, [U]nclassified Object, [H]overing Finger, [G]loved Finger, [N]arrow Object Swipe, Hand[E]dge\n");
		TOUCH_INFO_MSG("select length: 1~8, value: 0~1\n");
		TOUCH_INFO_MSG("ex) echo F 1 > object_report         (enable [F]inger)\n");
		TOUCH_INFO_MSG("ex) echo s 1 > object_report         (enable [S]tylus)\n");
		TOUCH_INFO_MSG("ex) echo P 0 > object_report         (disable [P]alm)\n");
		TOUCH_INFO_MSG("ex) echo u 0 > object_report         (disable [U]nclassified Object)\n");
		TOUCH_INFO_MSG("ex) echo HgNe 1 > object_report      (enable [H]overing Finger, [G]loved Finger, [N]arrow Object Swipe, Hand[E]dge)\n");
		TOUCH_INFO_MSG("ex) echo eNGh 1 > object_report      (enable Hand[E]dge, [N]arrow Object Swipe, [G]loved Finger, [H]overing Finger)\n");
		TOUCH_INFO_MSG("ex) echo uPsF 0 > object_report      (disable [U]nclassified Object, [P]alm, [S]tylus, [F]inger)\n");
		TOUCH_INFO_MSG("ex) echo HguP 0 > object_report      (disable [H]overing Finger, [G]loved Finger, [U]nclassified Object, [P]alm)\n");
		TOUCH_INFO_MSG("ex) echo HFnuPSfe 1 > object_report  (enable all object)\n");
		TOUCH_INFO_MSG("ex) echo enghupsf 0 > object_report  (disbale all object)\n");
	} else {
		select_cnt = strlen(select);

		for (i = 0; i < select_cnt; i++) {
			switch ((char)(*(select + i))) {
				case 'F': case 'f':
					bit_select |= (0x01 << 0); break;   /* Bit0 : (F)inger*/
				case 'S': case 's':
					bit_select |= (0x01 << 1); break;   /* Bit1 : (S)tylus*/
				case 'P': case 'p':
					bit_select |= (0x01 << 2); break;   /* Bit2 : (P)alm*/
				case 'U': case 'u':
					bit_select |= (0x01 << 3); break;   /* Bit3 : (U)nclassified Object*/
				case 'H': case 'h':
					bit_select |= (0x01 << 4); break;   /* Bit4 : (H)overing Finger*/
				case 'G': case 'g':
					bit_select |= (0x01 << 5); break;   /* Bit5 : (G)loved Finger*/
				case 'N': case 'n':
					bit_select |= (0x01 << 6); break;   /* Bit6 : (N)arrow Object Swipe*/
				case 'E': case 'e':
					bit_select |= (0x01 << 7); break;   /* Bit7 : Hand(E)dge*/
				default:
					break;
			}
		}

		object_report_enable_reg_addr = f12_info.ctrl_reg_addr[23];

		ret = touch_i2c_read(client, object_report_enable_reg_addr, sizeof(object_report_enable_reg_old), &object_report_enable_reg_old);

		if (ret < 0) {
			TOUCH_ERR_MSG("Failed to read object_report_enable_reg old value\n");
			return count;
		}

		object_report_enable_reg_new = object_report_enable_reg_old;

		if (value > 0)
			object_report_enable_reg_new |= bit_select;
		else
			object_report_enable_reg_new &= ~(bit_select);

		ret = touch_i2c_write_byte(client, object_report_enable_reg_addr, object_report_enable_reg_new);

		if (ret < 0) {
			TOUCH_ERR_MSG("Failed to write object_report_enable_reg new value\n");
			return count;
		}

		ret = touch_i2c_read(client, object_report_enable_reg_addr, sizeof(object_report_enable_reg_new), &object_report_enable_reg_new);

		if (ret < 0) {
			TOUCH_ERR_MSG("Failed to read object_report_enable_reg new value\n");
			return count;
		}

		for (i = 0; i < 8; i++) {
			old[i] = (object_report_enable_reg_old >> i) & 0x01;
			new[i] = (object_report_enable_reg_new >> i) & 0x01;
		}

		TOUCH_INFO_MSG("======= write object_report_enable register (before) =======\n");
		TOUCH_INFO_MSG(" Addr  Bit7  Bit6  Bit5  Bit4  Bit3  Bit2  Bit1  Bit0  HEX\n");
		TOUCH_INFO_MSG("------------------------------------------------------------\n");
		TOUCH_INFO_MSG(" 0x%02X   %d     %d     %d     %d     %d     %d     %d     %d    0x%02X\n",
				object_report_enable_reg_addr, old[7], old[6], old[5], old[4], old[3], old[2], old[1], old[0], object_report_enable_reg_old);
		TOUCH_INFO_MSG("============================================================\n");

		TOUCH_INFO_MSG("======= write object_report_enable register (after) ========\n");
		TOUCH_INFO_MSG(" Addr  Bit7  Bit6  Bit5  Bit4  Bit3  Bit2  Bit1  Bit0  HEX\n");
		TOUCH_INFO_MSG("------------------------------------------------------------\n");
		TOUCH_INFO_MSG(" 0x%02X   %d     %d     %d     %d     %d     %d     %d     %d    0x%02X\n",
				object_report_enable_reg_addr, new[7], new[6], new[5], new[4], new[3], new[2], new[1], new[0], object_report_enable_reg_new);
		TOUCH_INFO_MSG("------------------------------------------------------------\n");
		TOUCH_INFO_MSG(" Bit0     :     [F]inger                  ->     %s\n", new[0] ? "Enable" : "Disable");
		TOUCH_INFO_MSG(" Bit1     :     [S]tylus                  ->     %s\n", new[1] ? "Enable" : "Disable");
		TOUCH_INFO_MSG(" Bit2     :     [P]alm                    ->     %s\n", new[2] ? "Enable" : "Disable");
		TOUCH_INFO_MSG(" Bit3     :     [U]nclassified Object     ->     %s\n", new[3] ? "Enable" : "Disable");
		TOUCH_INFO_MSG(" Bit4     :     [H]overing Finger         ->     %s\n", new[4] ? "Enable" : "Disable");
		TOUCH_INFO_MSG(" Bit5     :     [G]loved Finger           ->     %s\n", new[5] ? "Enable" : "Disable");
		TOUCH_INFO_MSG(" Bit6     :     [N]arrow Object Swipe     ->     %s\n", new[6] ? "Enable" : "Disable");
		TOUCH_INFO_MSG(" Bit7     :     Hand[E]dge                ->     %s\n", new[7] ? "Enable" : "Disable");
		TOUCH_INFO_MSG("============================================================\n");
	}

	return count;
}

static ssize_t store_boot_mode(struct i2c_client *client, const char *buf, size_t count)
{
	struct synaptics_ts_data *ts =
			(struct synaptics_ts_data *)get_touch_handle(client);

	sscanf(buf, "%d", &boot_mode);

	switch (boot_mode) {
		case CHARGERLOGO_MODE:
			TOUCH_INFO_MSG("%s: Charger mode!!! Disable irq\n", __FUNCTION__);
			sleep_control(ts, 0, 1);
			disable_irq_wake(ts->client->irq);
			break;
		case NORMAL_BOOT_MODE:
			TOUCH_INFO_MSG("%s: Normal boot mode!!! Enable irq\n", __FUNCTION__);
			enable_irq_wake(ts->client->irq);
			sleep_control(ts, 1, 1);
			break;
		default:
			break;
	}

	return count;
}

static ssize_t store_sensing_test(struct i2c_client *client, const char *buf, size_t count)
{
	int value = 0;

	sscanf(buf, "%d", &value);

	is_Sensing = value;

	return count;
}

static ssize_t show_noise_delta_test(struct i2c_client *client, char *buf)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);

	int ret = 0;
	int noise_delta = 0;

	if (power_state == POWER_ON || power_state == POWER_WAKE) {

		if (need_scan_pdt) {
			SCAN_PDT();
			need_scan_pdt = false;
		}

		touch_disable_irq(ts->client->irq);

		noise_delta = F54Test('x', 0, buf);

		touch_enable_irq(ts->client->irq);
		synaptics_ts_init(ts->client);

		ret += sprintf(buf+ret, "Noise Delta Test : RESULT: %s", (noise_delta > 0) ? "Pass\n" : "Fail\n");
	} else {
		ret += sprintf(buf+ret, "state=[suspend]. we cannot use I2C, now. Test Result: Fail\n");
	}

	return ret;

}


static ssize_t show_ts_noise(struct i2c_client *client, char *buf)
{
	int ret = 0;

	ret += sprintf(buf+ret, "Test Count : %u\n", cnt);
	ret += sprintf(buf+ret, "Current Noise State : %d\n", cns_aver);
	ret += sprintf(buf+ret, "Interference Metric : %d\n", im_aver);
	ret += sprintf(buf+ret, "CID IM : %d\n", cid_im_aver);
	ret += sprintf(buf+ret, "Freq Scan IM : %d\n", freq_scan_im_aver);

	TOUCH_INFO_MSG("Aver: CNS[%5d]   IM[%5d]	 CID_IM[%5d]	FREQ_SCAN_IM[%5d] (cnt:%u)\n",
					cns_aver, im_aver, cid_im_aver, freq_scan_im_aver, cnt);

	return ret;
}

static ssize_t store_ts_noise(struct i2c_client *client, const char *buf, size_t count)
{
	struct synaptics_ts_data *ts =
			(struct synaptics_ts_data *)get_touch_handle(client);
	int value;

	sscanf(buf, "%d", &value);

	if ((ts->ts_state_flag.check_noise_menu == MENU_OUT) && (value == MENU_ENTER)) {
		ts->ts_state_flag.check_noise_menu = MENU_ENTER;
	} else if ((ts->ts_state_flag.check_noise_menu == MENU_ENTER) && (value == MENU_OUT)) {
		ts->ts_state_flag.check_noise_menu = MENU_OUT;
	} else {
		TOUCH_INFO_MSG("Already entered Check Noise menu .\n");
		TOUCH_INFO_MSG("check_noise_menu:%d, value:%d \n",
			ts->ts_state_flag.check_noise_menu, value);
		return count;
	}

	TOUCH_INFO_MSG("Check Noise = %s\n",
		(ts->ts_state_flag.check_noise_menu == MENU_OUT) ? "MENU_OUT" : "MENU_ENTER");
	TOUCH_INFO_MSG("TA state = %s\n",
		(touch_ta_status) ? "TA_CONNECTED" : "TA_DISCONNECTED");

	return count;
}

static ssize_t show_ts_noise_log_enable(struct i2c_client *client, char *buf)
{
	struct synaptics_ts_data *ts =
		(struct synaptics_ts_data *)get_touch_handle(client);
	int ret = 0;

	ret += sprintf(buf+ret, "%d\n", ts->ts_state_flag.ts_noise_log_flag);
	TOUCH_INFO_MSG("ts noise log flag = %s\n",
			(ts->ts_state_flag.ts_noise_log_flag == TS_NOISE_LOG_DISABLE) ? "TS_NOISE_LOG_DISABLE" : "TS_NOISE_LOG_ENABLE");

	return ret;
}

static ssize_t store_ts_noise_log_enable(struct i2c_client *client, const char *buf, size_t count)
{
	struct synaptics_ts_data *ts =
			(struct synaptics_ts_data *)get_touch_handle(client);
	int value;

	sscanf(buf, "%d", &value);

	if ((ts->ts_state_flag.ts_noise_log_flag == TS_NOISE_LOG_DISABLE) && (value == TS_NOISE_LOG_ENABLE)) {
		ts->ts_state_flag.ts_noise_log_flag = TS_NOISE_LOG_ENABLE;
	} else if ((ts->ts_state_flag.ts_noise_log_flag == TS_NOISE_LOG_ENABLE) && (value == TS_NOISE_LOG_DISABLE)) {
		ts->ts_state_flag.ts_noise_log_flag = TS_NOISE_LOG_DISABLE;
	} else {
		TOUCH_INFO_MSG("Already Enable TS Noise Log.\n");
		TOUCH_INFO_MSG("ts_noise_log_flag:%d, value:%d \n",
			ts->ts_state_flag.ts_noise_log_flag, value);
		return count;
	}

	TOUCH_INFO_MSG("ts noise log flag = %s\n",
		(ts->ts_state_flag.ts_noise_log_flag == TS_NOISE_LOG_DISABLE) ? "TS_NOISE_LOG_DISABLE" : "TS_NOISE_LOG_ENABLE");
	TOUCH_INFO_MSG("TA state = %s\n",
		(touch_ta_status) ? "TA_CONNECTED" : "TA_DISCONNECTED");

	return count;
}
static ssize_t show_diff_node(struct i2c_client *client, char *buf)
{

    int ret = 0;

	ret = sprintf(buf, "show_diff_node: %d \n",  ref_chk_enable);

	return ret;
}


/* test code for operating ref chk code */
static ssize_t store_diff_node(struct i2c_client *client, const char *buf, size_t count)
{

	struct synaptics_ts_data *ts =
			(struct synaptics_ts_data *)get_touch_handle(client);

	int ret = 0;

	sscanf(buf, "%d", &ref_chk_enable);

    if (synaptics_ts_ic_ctrl(ts->client, IC_CTRL_BASELINE_REBASE, FORCE_CAL, &ret) < 0) {
		TOUCH_ERR_MSG("IC_CTRL_REBASE handling fail\n");
    }

	return count;
}

static ssize_t show_lpwg_test_info(struct i2c_client *client, char *buf)
{
	struct synaptics_ts_data *ts =
			(struct synaptics_ts_data *)get_touch_handle(client);
	int ret = 0;

	ret = sprintf(buf+ret, "%d\n", atomic_read(&ts->lpwg_ctrl.is_suspend));

	return ret;
}

static ssize_t show_touch_wake_up_test(struct i2c_client *client, char *buf)
{
	int ret = 0;

	ret += sprintf(buf+ret, "%d\n", touch_wake_count);
	ret += sprintf(buf+ret, "%d\n", touch_wake_test);

	return ret;
}

static ssize_t store_touch_wake_up_test(struct i2c_client *client, const char *buf, size_t count)
{
	int cmd = 0;

	if (sscanf(buf, "%d", &cmd) != 1)
		return -EINVAL;

	switch (cmd) {
	case 0:
		if (touch_wake_test) {
			TOUCH_INFO_MSG("Stop touch wake test !\n");
			write_time_log(TOUCH_WAKE_COUNTER_LOG_PATH, "Stop touch wake test !\n", 1);
			touch_wake_test = false;
			touch_wake_count = 0;
		}
		break;
	case 1:
		if (!touch_wake_test) {
			TOUCH_INFO_MSG("Start touch wake test !\n");
			write_time_log(TOUCH_WAKE_COUNTER_LOG_PATH, "Start touch wake test !\n", 1);
			touch_wake_test = true;
		}
		break;
	case 2:
		TOUCH_INFO_MSG("Reset touch wake count !\n");
		touch_wake_count = 0;
		break;
	default:
		TOUCH_INFO_MSG("else case.\n");
	}

	return count;
}

static ssize_t show_pen_support(struct i2c_client *client, char *buf)
{
	struct synaptics_ts_data *ts = (struct synaptics_ts_data *)get_touch_handle(client);
	int ret = 0;
	int pen_support = 0;   /* 1: Support , 0: Not support */

	pen_support = GET_OBJECT_REPORT_INFO(ts->object_report, OBJECT_STYLUS_BIT);

	ret = sprintf(buf, "%d\n", pen_support);

	return ret;
}

static ssize_t show_palm_ctrl_mode(struct i2c_client *client, char *buf)
{
	struct synaptics_ts_data *ts = (struct synaptics_ts_data *)get_touch_handle(client);
	int ret = 0;

	ret = sprintf(buf, "%u\n", ts->pdata->role->palm_ctrl_mode);

	return ret;
}

static ssize_t store_palm_ctrl_mode(struct i2c_client *client, const char *buf, size_t count)
{
	struct synaptics_ts_data *ts = (struct synaptics_ts_data *)get_touch_handle(client);
	int value;

	sscanf(buf, "%d", &value);

	if (value < PALM_REJECT_FW || value > PALM_REPORT) {
		TOUCH_INFO_MSG("Invalid palm_ctrl_mode:%d (palm_ctrl_mode -> PALM_REJECT_FW)\n", value);
		value = PALM_REJECT_FW;
	}

	ts->pdata->role->palm_ctrl_mode = value;
	TOUCH_INFO_MSG("palm_ctrl_mode:%u\n", ts->pdata->role->palm_ctrl_mode);

	return count;
}

static ssize_t show_fw_upgrade_status(struct i2c_client *client, char *buf)
{
	int ret = 0;

	ret = sprintf(buf, "%d\n", fw_upgrade_state);

	return ret;
}

static ssize_t show_factory_version(struct i2c_client *client, char *buf)
{
	struct synaptics_ts_data *ts = (struct synaptics_ts_data *)get_touch_handle(client);
	int ret = 0;
	int rc = 0;
	mfts_mode = 1;

	read_page_description_table(ts->client);
	rc = get_ic_info(ts);
	if (rc < 0) {
		ret += sprintf(buf+ret, "-1\n");
		ret += sprintf(buf+ret, "Read Fail Touch IC Info.\n");
		mfts_mode = 0;
		return ret;
	}
	mfts_mode = 0;

	ret = sprintf(buf, "\n======== Factory FW version ========\n");
	if (ts->fw_info.fw_version[0] > 0x40)
		ret += sprintf(buf+ret, "factory_fw_version[%s]\n", ts->fw_info.fw_image_version);
	else {
		ret += sprintf(buf+ret, "version : v%d.%02d\n",
			((ts->fw_info.fw_image_version[3] & 0x80) >> 7), (ts->fw_info.fw_image_version[3] & 0x7F));
	}

	return ret;
}

static ssize_t show_mfts_enable(struct i2c_client *client, char *buf)
{
	int ret = 0;

	ret = sprintf(buf, "%d\n", mfts_enable);

	return ret;
}

static ssize_t store_mfts_enable(struct i2c_client *client, const char *buf, size_t count)
{
	int value;

	sscanf(buf, "%d", &value);

	mfts_enable = value;
	TOUCH_INFO_MSG("mfts_enable:%d\n", mfts_enable);

	return count;
}

static LGE_TOUCH_ATTR(firmware, S_IRUGO | S_IWUSR, show_firmware, NULL);
static LGE_TOUCH_ATTR(sd, S_IRUGO | S_IWUSR, show_sd, NULL);
static LGE_TOUCH_ATTR(rawdata, S_IRUGO | S_IWUSR, show_rawdata, NULL);
static LGE_TOUCH_ATTR(delta, S_IRUGO | S_IWUSR, show_delta, NULL);
static LGE_TOUCH_ATTR(chstatus, S_IRUGO | S_IWUSR, show_chstatus, NULL);
static LGE_TOUCH_ATTR(testmode_ver, S_IRUGO | S_IWUSR, show_atcmd_fw_ver, NULL);
static LGE_TOUCH_ATTR(tci, S_IRUGO | S_IWUSR, show_tci, store_tci);
static LGE_TOUCH_ATTR(reg_ctrl, S_IRUGO | S_IWUSR, NULL, store_reg_ctrl);
static LGE_TOUCH_ATTR(object_report, S_IRUGO | S_IWUSR, show_object_report, store_object_report);
static LGE_TOUCH_ATTR(version, S_IRUGO | S_IWUSR, show_synaptics_fw_version, NULL);
static LGE_TOUCH_ATTR(bootmode, S_IRUGO | S_IWUSR, NULL, store_boot_mode);
static LGE_TOUCH_ATTR(ts_noise, S_IRUGO | S_IWUSR, show_ts_noise, store_ts_noise);
static LGE_TOUCH_ATTR(ts_noise_log_enable, S_IRUGO | S_IWUSR, show_ts_noise_log_enable, store_ts_noise_log_enable);
static LGE_TOUCH_ATTR(diff_node, S_IRUGO | S_IWUSR, show_diff_node, store_diff_node);
static LGE_TOUCH_ATTR(lpwg_test_info, S_IRUGO | S_IWUSR, show_lpwg_test_info, NULL);
static LGE_TOUCH_ATTR(touch_wake_up_test, S_IRUGO | S_IWUSR, show_touch_wake_up_test, store_touch_wake_up_test);
static LGE_TOUCH_ATTR(pen_support, S_IRUGO | S_IWUSR, show_pen_support, NULL);
static LGE_TOUCH_ATTR(palm_ctrl_mode, S_IRUGO | S_IWUSR, show_palm_ctrl_mode, store_palm_ctrl_mode);
static LGE_TOUCH_ATTR(sensing_test, S_IRUGO | S_IWUSR, NULL, store_sensing_test);
static LGE_TOUCH_ATTR(abs_test, S_IRUGO | S_IWUSR, show_abs_test, NULL);
static LGE_TOUCH_ATTR(sensor_speed_test, S_IRUGO | S_IWUSR, show_sensor_speed_test, NULL);
static LGE_TOUCH_ATTR(adc_range_test, S_IRUGO | S_IWUSR, show_adc_range_test, NULL);
static LGE_TOUCH_ATTR(noise_delta_test, S_IRUGO | S_IWUSR, show_noise_delta_test, NULL);
static LGE_TOUCH_ATTR(fw_upgrade_status, S_IRUGO | S_IWUSR, show_fw_upgrade_status, NULL);
static LGE_TOUCH_ATTR(factory_version, S_IRUGO | S_IWUSR, show_factory_version, NULL);
static LGE_TOUCH_ATTR(mfts, S_IRUGO | S_IWUSR, show_mfts_enable, store_mfts_enable);

static struct attribute *lge_specific_touch_attribute_list[] = {
	&lge_touch_attr_firmware.attr,
	&lge_touch_attr_sd.attr,
	&lge_touch_attr_rawdata.attr,
	&lge_touch_attr_delta.attr,
	&lge_touch_attr_chstatus.attr,
	&lge_touch_attr_testmode_ver.attr,
	&lge_touch_attr_tci.attr,
	&lge_touch_attr_reg_ctrl.attr,
	&lge_touch_attr_object_report.attr,
	&lge_touch_attr_version.attr,
	&lge_touch_attr_bootmode.attr,
	&lge_touch_attr_ts_noise.attr,
	&lge_touch_attr_ts_noise_log_enable.attr,
	&lge_touch_attr_diff_node.attr,
	&lge_touch_attr_lpwg_test_info.attr,
	&lge_touch_attr_touch_wake_up_test.attr,
	&lge_touch_attr_pen_support.attr,
	&lge_touch_attr_palm_ctrl_mode.attr,
	&lge_touch_attr_sensing_test.attr,
	&lge_touch_attr_abs_test.attr,
	&lge_touch_attr_sensor_speed_test.attr,
	&lge_touch_attr_adc_range_test.attr,
	&lge_touch_attr_noise_delta_test.attr,
	&lge_touch_attr_fw_upgrade_status.attr,
	&lge_touch_attr_factory_version.attr,
	&lge_touch_attr_mfts.attr,
	NULL,
};

static int read_page_description_table(struct i2c_client *client)
{
	struct synaptics_ts_data *ts = (struct synaptics_ts_data *)get_touch_handle(client);
	struct function_descriptor buffer;

	unsigned short u_address = 0;
	unsigned short page_num = 0;
	u8 intterrupt_cust = 0;

	TOUCH_TRACE();

	memset(&buffer, 0x0, sizeof(struct function_descriptor));
	memset(&ts->common_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->finger_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->sensor_fc, 0x0, sizeof(struct ts_ic_function));
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
			case SENSOR_CONTROL:
				ts->sensor_fc.dsc = buffer;
				ts->sensor_fc.function_page = page_num;
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

	/* INTERRUPT_MASK_CUSTOM has been changed 0x40 to 0x20 from fw v1.12 */
	   DO_SAFE(synaptics_ts_page_data_read(client, 0x02, EXIST_OFFSET , 1, &intterrupt_cust), error);
	/*means fw version before v1.12*/
	   if (intterrupt_cust == F1A_EXIST)
		   int_mask_cust = 0x40;
	   else
		   int_mask_cust = 0x20;

	DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, 0x00), error);
	ERROR_IF(ts->common_fc.dsc.id == 0 || ts->finger_fc.dsc.id == 0
			|| ts->analog_fc.dsc.id == 0 || ts->flash_fc.dsc.id == 0, "page_init_error", init_error);

	TOUCH_DEBUG(DEBUG_BASE_INFO, "common[%dP:0x%02x] finger[%dP:0x%02x] sensor[%dP:0x%02x] analog[%dP:0x%02x] flash[%dP:0x%02x]\n",
				ts->common_fc.function_page, ts->common_fc.dsc.id, ts->finger_fc.function_page, ts->finger_fc.dsc.id,
				ts->sensor_fc.function_page, ts->sensor_fc.dsc.id, ts->analog_fc.function_page, ts->analog_fc.dsc.id,
				ts->flash_fc.function_page, ts->flash_fc.dsc.id);

	get_f12_info(ts);

	return 0;

init_error:
	return EINVAL;

error:
	return -EIO;
}

static int get_ic_info(struct synaptics_ts_data *ts)
{
	const struct firmware *fw_entry = NULL;
	const u8 *fw = NULL;
	int rc = 0;

	memset(&ts->fw_info, 0, sizeof(struct synaptics_ts_fw_info));

	DO_SAFE(touch_i2c_read(ts->client, PRODUCT_ID_REG, sizeof(ts->fw_info.fw_product_id) - 1, ts->fw_info.fw_product_id), error);
	DO_SAFE(touch_i2c_read(ts->client, FLASH_CONFIG_ID_REG, sizeof(ts->fw_info.fw_version) - 1, ts->fw_info.fw_version), error);
	DO_SAFE(touch_i2c_read(ts->client, CUSTOMER_FAMILY_REG, 1, &(ts->fw_info.family)), error);
	DO_SAFE(touch_i2c_read(ts->client, FW_REVISION_REG, 1, &(ts->fw_info.fw_revision)), error);
	TOUCH_DEBUG(DEBUG_BASE_INFO, "CUSTOMER_FAMILY_REG = %d\n", ts->fw_info.family);
	TOUCH_DEBUG(DEBUG_BASE_INFO, "FW_REVISION_REG = %d\n", ts->fw_info.fw_revision);

	if (ts->fw_info.fw_product_id == NULL) {
		goto error;
	}

	if (!(strncmp(ts->fw_info.fw_product_id, "PLG313", 6))
		|| !(strncmp(ts->fw_info.fw_product_id, "PLG352", 6))
		|| !(strncmp(ts->fw_info.fw_product_id, "PLG391", 6))) {
		if (ts->fw_info.family == 0 && ts->fw_info.fw_revision == 0) {
			rc = request_firmware(&fw_entry, ts->pdata->inbuilt_fw_name, &ts->client->dev);
			ts->fw_flag = S3528_A0;
		} else {
			if (mfts_mode) {
				rc = request_firmware(&fw_entry, ts->pdata->inbuilt_fw_name_s3528_a1_factory, &ts->client->dev);
				ts->fw_flag = S3528_A1;
			} else if (!(strncmp(ts->fw_info.fw_product_id, "PLG391", 6))) {
				rc = request_firmware(&fw_entry, ts->pdata->inbuilt_fw_name_s3528_a1_suntel, &ts->client->dev);
				ts->fw_flag = S3528_A1_SUN;
			} else {
				rc = request_firmware(&fw_entry, ts->pdata->inbuilt_fw_name_s3528_a1, &ts->client->dev);
				ts->fw_flag = S3528_A1;
			}
		}
		if (rc != 0) {
			TOUCH_ERR_MSG("request_firmware() failed %d\n", rc);
			goto error;
		}
	} else if (!(strncmp(ts->fw_info.fw_product_id, "PLG298", 6))) {
		rc = request_firmware(&fw_entry, ts->pdata->inbuilt_fw_name_s3621, &ts->client->dev);
		ts->fw_flag = S3621;
		if (rc != 0) {
			TOUCH_ERR_MSG("request_firmware() failed %d\n", rc);
			goto error;
		}
	} else {
		goto error;
	}

	TOUCH_DEBUG(DEBUG_BASE_INFO, "ts->fw_flag = %d\n", ts->fw_flag);
	fw = fw_entry->data;

	memcpy(ts->fw_info.fw_image_product_id, &fw[0x0040], 6);
	memcpy(ts->fw_info.fw_image_version, &fw[0x16d00], 4);
	ts->fw_info.fw_start = (unsigned char *)&fw[0];
	ts->fw_info.fw_size = sizeof(fw);

	/*because s3621 doesn't support knock-on*/
	if (!strncmp(ts->fw_info.fw_product_id, "PLG298" , 6))
		ts->pdata->role->use_sleep_mode = 0;
	return 0;
error:
	memset(&fw_entry, 0, sizeof(fw_entry));
	return -1;
}

static int check_firmware_status(struct synaptics_ts_data *ts)
{
	u8 device_status = 0;
	u8 flash_status = 0;

	DO_SAFE(touch_i2c_read(ts->client, FLASH_STATUS_REG, sizeof(flash_status), &flash_status), error);
	DO_SAFE(touch_i2c_read(ts->client, DEVICE_STATUS_REG, sizeof(device_status), &device_status), error);

	ts->fw_info.need_rewrite_firmware = 0;

	if ((device_status & DEVICE_STATUS_FLASH_PROG) || (device_status & DEVICE_CRC_ERROR_MASK)
			|| (flash_status & FLASH_STATUS_MASK)) {
		TOUCH_ERR_MSG("FLASH_STATUS[0x%x] DEVICE_STATUS[0x%x]\n", (u32)flash_status, (u32)device_status);
		ts->fw_info.need_rewrite_firmware = 1;
	}

	return 0;
error:
	return -1;
}

enum error_type synaptics_ts_probe(struct i2c_client *client, const struct touch_platform_data* lge_ts_data,
				const struct state_info *state, struct attribute ***attribute_list)
{
	struct synaptics_ts_data *ts;

	TOUCH_TRACE();

	ASSIGN(ts = devm_kzalloc(&client->dev, sizeof(struct synaptics_ts_data), GFP_KERNEL), error);
	set_touch_handle(client, ts);

	ts->client = client;
	ds4_i2c_client = client;
	ts->pdata = lge_ts_data;
	ts->state = state;

	if (ts->pdata->pwr->use_regulator) {
		DO_IF(IS_ERR(ts->regulator_vdd = regulator_get(&client->dev, ts->pdata->pwr->vdd)), error);
		DO_IF(IS_ERR(ts->regulator_vio = regulator_get(&client->dev, ts->pdata->pwr->vio)), error);
		if (ts->pdata->pwr->vdd_voltage > 0)
			DO_SAFE(regulator_set_voltage(ts->regulator_vdd, ts->pdata->pwr->vdd_voltage, ts->pdata->pwr->vdd_voltage), error);
		if (ts->pdata->pwr->vio_voltage > 0)
			DO_SAFE(regulator_set_voltage(ts->regulator_vio, ts->pdata->pwr->vio_voltage, ts->pdata->pwr->vio_voltage), error);
	}

	*attribute_list = lge_specific_touch_attribute_list;
	ts->is_probed = 0;
	ts->is_init = 0;

	ts->lpwg_ctrl.screen = 1;
	ts->lpwg_ctrl.sensor = 1;

	atomic_set(&ts->lpwg_ctrl.is_suspend, 0);
	INIT_DELAYED_WORK(&ts->work_timer, lpwg_timer_func);
	INIT_DELAYED_WORK(&ts->diff_node_timer, diff_node_timer_func);
	INIT_DELAYED_WORK(&ts->cap_diff, read_cap_diff_file);
	INIT_DELAYED_WORK(&ts->work_palm, all_palm_released_func);
	wake_lock_init(&ts->timer_wake_lock, WAKE_LOCK_SUSPEND, "touch_timer");
    queue_delayed_work(touch_wq, &ts->cap_diff, msecs_to_jiffies(FILE_MOUNT_DELAY));
	return NO_ERROR;
error:
	return ERROR;
}

enum error_type synaptics_ts_remove(struct i2c_client *client)
{
	struct synaptics_ts_data *ts = (struct synaptics_ts_data *)get_touch_handle(client);

	TOUCH_TRACE();

	if (prox_fhandler.prox_inserted && prox_fhandler.prox_initialized) {
		prox_fhandler.prox_fn->remove();
		prox_fhandler.prox_initialized = false;
	}

	if (ts->pdata->pwr->use_regulator) {
		regulator_put(ts->regulator_vio);
		regulator_put(ts->regulator_vdd);
	}
	cancel_delayed_work(&ts->cap_diff);
	wake_lock_destroy(&ts->timer_wake_lock);
	return NO_ERROR;
}

static int lpwg_update_all(struct synaptics_ts_data *ts, bool irqctrl)
{
	int sleep_status = 0;
	int lpwg_status = 0;
	bool req_lpwg_param = false;

	if (mfts_enable) {
		goto error;
	}

	TOUCH_TRACE();

	if (!irqctrl && !atomic_read(&ts->lpwg_ctrl.is_suspend) && !ts->lpwg_ctrl.screen)
		ts->lpwg_ctrl.screen = 1;

	if (ts->lpwg_ctrl.screen) {
		if (atomic_read(&ts->lpwg_ctrl.is_suspend) == 1) {
			if (power_state == POWER_OFF || power_state == POWER_SLEEP)
				ts->is_init = 0;
			if (irqctrl)
				disable_irq_wake(ts->client->irq);
		}
		atomic_set(&ts->lpwg_ctrl.is_suspend, 0);
		TOUCH_INFO_MSG("%s : disable, irqctrl=%d\n", __func__, irqctrl);
	} else {
		if (atomic_read(&ts->lpwg_ctrl.is_suspend) == 0) {
			atomic_set(&ts->lpwg_ctrl.is_suspend, 1);
			if (irqctrl)
				enable_irq_wake(ts->client->irq);
			TOUCH_INFO_MSG("%s : enable, irqctrl=%d\n", __func__, irqctrl);
			set_doze_param(ts);
		}
	}

	if (ts->lpwg_ctrl.screen) {		/*  on(1) */
		sleep_status = 1;
		lpwg_status = 0;
	} else if (!ts->lpwg_ctrl.screen && ts->lpwg_ctrl.qcover) {	/* off(0), closed(0),   -- */
		sleep_status = 1;
		lpwg_status = 1;
	} else if (!ts->lpwg_ctrl.screen && !ts->lpwg_ctrl.qcover && ts->lpwg_ctrl.sensor) {	/* off(0),   open(1),  far(1) */
		sleep_status = 1;
		lpwg_status = ts->lpwg_ctrl.lpwg_mode;
	} else if (!ts->lpwg_ctrl.screen && !ts->lpwg_ctrl.qcover && !ts->lpwg_ctrl.sensor) {	/* off(0),   open(1), near(0) */
		if (!after_crack_check) {
			TOUCH_DEBUG(DEBUG_BASE_INFO, "%s : Crack check not done...use nonsleep mode to check Crack!!\n", __FUNCTION__);
			sleep_status = 1;
			lpwg_status = ts->lpwg_ctrl.lpwg_mode;
		} else {
			sleep_status = 0;
			req_lpwg_param = true;
		}
	}

	DO_SAFE(sleep_control(ts, sleep_status, 0), error);
	if (req_lpwg_param == false)
		DO_SAFE(lpwg_control(ts, lpwg_status), error);

	return NO_ERROR;
error:
	return ERROR;
}

enum error_type synaptics_ts_init(struct i2c_client *client)
{
	struct synaptics_ts_data *ts = (struct synaptics_ts_data *)get_touch_handle(client);
	u8 buf = 0;
	u8 buf_array[2] = {0};
	int prox_retval;
	u8 motion_suppression_reg_addr;
	int rc = 0;
	u8 lpwg_mode = ts->lpwg_ctrl.lpwg_mode;
	int is_suspend = atomic_read(&ts->lpwg_ctrl.is_suspend);

	TOUCH_TRACE();

	if (ts->is_probed == 0) {
		rc = read_page_description_table(ts->client);
		if (rc < 0) {
			goto error;
		}
		get_ic_info(ts);
		DO_SAFE(check_firmware_status(ts), error);
		if (rc == EINVAL) {
			TOUCH_INFO_MSG("%s : need to rewrite firmware !!",__func__);
			ts->fw_info.need_rewrite_firmware = 1;
		}
		ts->is_probed = 1;
	}

	if (prox_fhandler.prox_inserted)
	{
		if (!prox_fhandler.prox_initialized) {
			prox_retval = prox_fhandler.prox_fn->init(ts);

			if (prox_retval < 0) {
				TOUCH_INFO_MSG("[Touch Proximity] %s: Failed to init proximity settings\n", __func__);
			} else {
				prox_fhandler.prox_initialized = true;
			}
		} else {
			prox_fhandler.prox_fn->reinit(ts);
		}
	}

	DO_SAFE(touch_i2c_write_byte(client, DEVICE_CONTROL_REG,
			DEVICE_CONTROL_NORMAL_OP | DEVICE_CONTROL_CONFIGURED), error);

	DO_SAFE(touch_i2c_read(client, INTERRUPT_ENABLE_REG, 1, &buf), error);
	if (!(strncmp(ts->fw_info.fw_product_id, "PLG313", 6)) || !(strncmp(ts->fw_info.fw_product_id, "PLG352", 6)) || !(strncmp(ts->fw_info.fw_product_id, "PLG391", 6))) {
		DO_SAFE(touch_i2c_write_byte(client, INTERRUPT_ENABLE_REG,
			buf | INTERRUPT_MASK_ABS0 | int_mask_cust), error);
	} else {
		DO_SAFE(touch_i2c_write_byte(client, INTERRUPT_ENABLE_REG, buf | INTERRUPT_MASK_ABS0), error);
	}

	if (ts->pdata->role->report_mode == REDUCED_REPORT_MODE
			&& !ts->pdata->role->ghost_detection->check_enable.long_press_chk) {
		buf_array[0] = buf_array[1] = ts->pdata->role->delta_pos_threshold;
	} else {
		buf_array[0] = buf_array[1] = 0;
		ts->pdata->role->ghost_detection->force_continuous_mode = true;
	}

	motion_suppression_reg_addr = f12_info.ctrl_reg_addr[20];
	DO_SAFE(touch_i2c_write(client, motion_suppression_reg_addr, 2, buf_array), error);

	DO_SAFE(touch_i2c_read(client, f12_info.ctrl_reg_addr[15], 2, default_finger_amplitude), error);

	if (ts->pdata->role->palm_ctrl_mode > PALM_REPORT) {
		TOUCH_INFO_MSG("Invalid palm_ctrl_mode:%u (palm_ctrl_mode -> PALM_REJECT_FW)\n", ts->pdata->role->palm_ctrl_mode);
		ts->pdata->role->palm_ctrl_mode = PALM_REJECT_FW;
	}

	TOUCH_INFO_MSG("palm_ctrl_mode:%u\n", ts->pdata->role->palm_ctrl_mode);

	DO_SAFE(touch_i2c_read(client, f12_info.ctrl_reg_addr[22], 1, &buf), error);
	buf_array[0] = buf & 0x03;

	if (ts->pdata->role->palm_ctrl_mode == PALM_REJECT_DRIVER || ts->pdata->role->palm_ctrl_mode == PALM_REPORT) {
		if (buf_array[0] != 0x00) {
			buf &= ~(0x03);		/* PalmFilterMode bits[1:0] (00:Disable palm filter) */
			DO_SAFE(touch_i2c_write_byte(client, f12_info.ctrl_reg_addr[22], buf), error);
		}
		memset(&ts->ts_palm_data, 0, sizeof(struct palm_data));
	} else {
		if (buf_array[0] != 0x01) {
			buf &= ~(0x02);		/* PalmFilterMode bits[1:0] (01:Enable palm filter) */
			buf |= 0x01;
			DO_SAFE(touch_i2c_write_byte(client, f12_info.ctrl_reg_addr[22], buf), error);
		}
	}

	if (lge_get_boot_mode() == LGE_BOOT_MODE_FACTORY2) {
		TOUCH_INFO_MSG("mini_os_finger_amplitude = 0x%02X\n", ts->pdata->role->mini_os_finger_amplitude);
		buf_array[0] = ts->pdata->role->mini_os_finger_amplitude;
		buf_array[1] = ts->pdata->role->mini_os_finger_amplitude;
		DO_SAFE(touch_i2c_write(client, f12_info.ctrl_reg_addr[15], 2, buf_array), error);
	}

	if (!(strncmp(ts->fw_info.fw_product_id, "PLG313", 6)) || !(strncmp(ts->fw_info.fw_product_id, "PLG352", 6)) || !(strncmp(ts->fw_info.fw_product_id, "PLG391", 6))) {
		if (ts->pdata->role->use_lpwg_all) {
			DO_SAFE(lpwg_update_all(ts, 0), error);
		} else {
			DO_SAFE(lpwg_control(ts, (is_suspend) ? lpwg_mode : 0), error);
		}
	}

	DO_SAFE(touch_i2c_read(client, INTERRUPT_STATUS_REG, 1, &buf), error);	/* It always should be done last. */
	ts->is_init = 1;

	return NO_ERROR;
error:
	return ERROR;
}

static int synaptics_ts_noise_log(struct i2c_client *client, struct touch_data* curr_data, const struct touch_data* prev_data)
{
	struct synaptics_ts_data *ts = (struct synaptics_ts_data *)get_touch_handle(client);
	u8 buffer[3] = {0};
	u8 buf1 = 0, buf2 = 0, cns = 0;
	u16 im = 0, cid_im = 0, freq_scan_im = 0;
	int i = 0;

	DO_SAFE((synaptics_ts_page_data_read(client, ANALOG_PAGE, 0x04, 1, &buf1) < 0), error);
	DO_SAFE((synaptics_ts_page_data_read(client, ANALOG_PAGE, 0x05, 1, &buf2) < 0), error);
	im = (buf2<<8)|buf1;
	im_sum += im;

	DO_SAFE((synaptics_ts_page_data_read(client, ANALOG_PAGE, 0x08, 1, &cns) < 0), error);
	cns_sum += cns;

	DO_SAFE((synaptics_ts_page_data_read(client, ANALOG_PAGE, 0x09, 2, buffer) < 0), error);
	cid_im = (buffer[1]<<8)|buffer[0];
	cid_im_sum += cid_im;

	DO_SAFE((synaptics_ts_page_data_read(client, ANALOG_PAGE, 0x0A, 2, buffer) < 0), error);
	freq_scan_im = (buffer[1]<<8)|buffer[0];
	freq_scan_im_sum += freq_scan_im;

	cnt++;

	if ((ts->ts_state_flag.ts_noise_log_flag == TS_NOISE_LOG_ENABLE)
		|| (touch_debug_mask & DEBUG_NOISE)) {
		if (prev_data->total_num != curr_data->total_num)
			TOUCH_INFO_MSG("Curr: CNS[%5d]   IM[%5d]   CID_IM[%5d]   FREQ_SCAN_IM[%5d]\n", cns, im, cid_im, freq_scan_im);
	}

	for (i = 0; i < MAX_FINGER; i++) {
		if ((prev_data->report_id_mask & (1 << i)) && !(curr_data->id_mask & (1 << i))) {
			break;
		}
	}
	if (((i < MAX_FINGER) && curr_data->total_num == 0)
		|| (im_sum >= ULONG_MAX || cns_sum >= ULONG_MAX || cid_im_sum >= ULONG_MAX || freq_scan_im_sum >= ULONG_MAX || cnt >= UINT_MAX)) {
		if ((ts->ts_state_flag.ts_noise_log_flag == TS_NOISE_LOG_ENABLE)
			|| touch_ta_status
			|| touch_hdmi_status
			|| (touch_debug_mask & DEBUG_NOISE))
			TOUCH_INFO_MSG("Aver: CNS[%5lu]   IM[%5lu]   CID_IM[%5lu]   FREQ_SCAN_IM[%5lu] (cnt:%u)\n",
				cns_sum/cnt, im_sum/cnt, cid_im_sum/cnt, freq_scan_im_sum/cnt, cnt);

		im_aver = im_sum/cnt;
		cns_aver = cns_sum/cnt;
		cid_im_aver = cid_im_sum/cnt;
		freq_scan_im_aver = freq_scan_im_sum/cnt;
	}

	if (prev_data->total_num == 0 && curr_data->total_num != 0) {
		cnt = im_sum = cns_sum = cid_im_sum = freq_scan_im_sum = 0;
		im_aver = cns_aver = cid_im_aver = freq_scan_im_aver = 0;
	}
	return 0;

error:
	return -1;
}

enum error_type synaptics_ts_get_data(struct i2c_client *client, struct touch_data* curr_data, const struct touch_data* prev_data)
{
	struct synaptics_ts_data *ts = (struct synaptics_ts_data *)get_touch_handle(client);

	u8  i = 0;
	u8  finger_index = 0;

	TOUCH_TRACE();

	if (!ts->is_init)
		return IGNORE_EVENT;

	curr_data->total_num = 0;
	curr_data->id_mask = 0;

	switch (ts->pdata->role->palm_ctrl_mode) {
		default:
		case PALM_REJECT_FW:
			break;
		case PALM_REJECT_DRIVER:
		case PALM_REPORT:
			memset(ts->ts_palm_data.curr_palm_mask, 0, sizeof(ts->ts_palm_data.curr_palm_mask));
			break;
	}

	DO_SAFE(touch_i2c_read(client, DEVICE_STATUS_REG,
		sizeof(ts->ts_data.device_status_reg), &ts->ts_data.device_status_reg), error);

	DO_IF((ts->ts_data.device_status_reg & DEVICE_FAILURE_MASK) == DEVICE_FAILURE_MASK, error);

	DO_SAFE(touch_i2c_read(client, INTERRUPT_STATUS_REG,
		sizeof(ts->ts_data.interrupt_status_reg), &ts->ts_data.interrupt_status_reg), error);

	if (prox_fhandler.prox_inserted)
		prox_fhandler.prox_fn->attn(ts->ts_data.interrupt_status_reg);

	if (ts->ts_data.interrupt_status_reg & int_mask_cust) {
		u8 status = 0;
		DO_SAFE(synaptics_ts_page_data_read(client, LPWG_PAGE, LPWG_STATUS_REG, 1, &status), error);
		if ((status & 0x1)) {   /* TCI-1 Double-Tap */
			TOUCH_DEBUG(DEBUG_BASE_INFO || DEBUG_LPWG, "LPWG Double-Tap mode\n");
		    if (ts->lpwg_ctrl.double_tap_enable) {
				get_tci_data(ts, 2);
				send_uevent_lpwg(ts->client, LPWG_DOUBLE_TAP);
				if (touch_wake_test) {
					if (touch_wake_count < 0xFFFFFFFF)
						touch_wake_count++;
					memset(touch_wake_log_buf, 0x0, 256);
					sprintf(touch_wake_log_buf, "Cnt : %d \r\n", touch_wake_count);
					TOUCH_INFO_MSG("%s\n", touch_wake_log_buf);
					write_time_log(TOUCH_WAKE_COUNTER_LOG_PATH, touch_wake_log_buf, 1);
				}
		    }
		} else if ((status & 0x2)) { /* TCI-2 Multi-Tap */
		    if (ts->lpwg_ctrl.password_enable) {
				TOUCH_DEBUG(DEBUG_BASE_INFO || DEBUG_LPWG, "LPWG Multi-Tap mode\n");
				get_tci_data(ts, ts->pw_data.tap_count);
				wake_lock(&ts->timer_wake_lock);
				tci_control(ts, REPORT_MODE_CTRL, 0);
				queue_delayed_work(touch_wq, &ts->work_timer, msecs_to_jiffies(UEVENT_DELAY-I2C_DELAY));
		    }
		} else {
		    TOUCH_DEBUG(DEBUG_BASE_INFO || DEBUG_LPWG, "LPWG status has problem\n");
		}
		return IGNORE_EVENT;
	} else if (ts->ts_data.interrupt_status_reg & INTERRUPT_MASK_ABS0) {
		DO_SAFE(touch_i2c_read(ts->client, FINGER_DATA_REG_START,
				(NUM_OF_EACH_FINGER_DATA_REG * MAX_NUM_OF_FINGERS),
				ts->ts_data.finger.finger_reg[0]), error);

		for (i = 0; i < MAX_NUM_OF_FINGERS; i++) {
			if (ts->ts_data.finger.finger_reg[i][0] == F12_FINGER_STATUS
				|| ts->ts_data.finger.finger_reg[i][0] == F12_STYLUS_STATUS
				|| ts->ts_data.finger.finger_reg[i][0] == F12_PALM_STATUS
				|| ts->ts_data.finger.finger_reg[i][0] == F12_GLOVED_FINGER_STATUS) {
				curr_data->abs_data[finger_index].id = i;
				curr_data->abs_data[finger_index].type =
					ts->ts_data.finger.finger_reg[i][REG_OBJECT_TYPE_AND_STATUS];
				curr_data->abs_data[finger_index].x =
				curr_data->abs_data[finger_index].raw_x =
					TS_SNTS_GET_X_POSITION(ts->ts_data.finger.finger_reg[i][REG_X_MSB],
							       ts->ts_data.finger.finger_reg[i][REG_X_LSB]);
				curr_data->abs_data[finger_index].y =
				curr_data->abs_data[finger_index].raw_y =
					TS_SNTS_GET_Y_POSITION(ts->ts_data.finger.finger_reg[i][REG_Y_MSB],
							       ts->ts_data.finger.finger_reg[i][REG_Y_LSB]);
				curr_data->abs_data[finger_index].width_major =
					TS_SNTS_GET_WIDTH_MAJOR(ts->ts_data.finger.finger_reg[i][REG_WX],
								ts->ts_data.finger.finger_reg[i][REG_WY]);
				curr_data->abs_data[finger_index].width_minor =
					TS_SNTS_GET_WIDTH_MINOR(ts->ts_data.finger.finger_reg[i][REG_WX],
								ts->ts_data.finger.finger_reg[i][REG_WY]);
				curr_data->abs_data[finger_index].orientation =
					TS_SNTS_GET_ORIENTATION(ts->ts_data.finger.finger_reg[i][REG_WY],
								ts->ts_data.finger.finger_reg[i][REG_WX]);
				curr_data->abs_data[finger_index].pressure =
					TS_SNTS_GET_PRESSURE(ts->ts_data.finger.finger_reg[i][REG_Z]);

				if (curr_data->abs_data[finger_index].type == F12_PALM_STATUS) {
					switch (ts->pdata->role->palm_ctrl_mode) {
						default:
						case PALM_REJECT_FW:
							break;
						case PALM_REJECT_DRIVER:
						case PALM_REPORT:
							curr_data->abs_data[finger_index].pressure = MAX_PRESSURE;
							ts->ts_palm_data.curr_palm_mask[i] = 1;
							ts->ts_palm_data.palm_coordinate[i].x = curr_data->abs_data[finger_index].x;
							ts->ts_palm_data.palm_coordinate[i].y = curr_data->abs_data[finger_index].y;
							break;
					}
				}

				if (ts->pdata->role->ghost_detection->check_enable.pressure_zero_chk
						&& curr_data->abs_data[finger_index].pressure == 0)
					ts->pdata->role->ghost_detection->pressure_zero = true;

				curr_data->id_mask |= (0x1 << i);
				curr_data->total_num++;

				TOUCH_DEBUG(DEBUG_GET_DATA, "<%d> type[%d] pos(%4d,%4d) w_m[%2d] w_n[%2d] o[%2d] p[%2d]\n",
						i, curr_data->abs_data[finger_index].type,
						curr_data->abs_data[finger_index].x, curr_data->abs_data[finger_index].y,
						curr_data->abs_data[finger_index].width_major, curr_data->abs_data[finger_index].width_minor,
						curr_data->abs_data[finger_index].orientation, curr_data->abs_data[finger_index].pressure);

				finger_index++;
			}
		}

		switch (ts->pdata->role->palm_ctrl_mode) {
			default:
			case PALM_REJECT_FW:
				break;
			case PALM_REJECT_DRIVER:
			case PALM_REPORT:
				for (i = 0; i < MAX_NUM_OF_FINGERS; i++) {
					if (ts->ts_palm_data.curr_palm_mask[i] != ts->ts_palm_data.prev_palm_mask[i]) {
						if (ts->ts_palm_data.curr_palm_mask[i]) {
							ts->ts_palm_data.curr_palm_num++;
							TOUCH_INFO_MSG("Palm is detected : id[%d] pos[%4d,%4d] total palm:%u\n",
								i, ts->ts_palm_data.palm_coordinate[i].x,
								ts->ts_palm_data.palm_coordinate[i].y,
								ts->ts_palm_data.curr_palm_num);
						} else {
							ts->ts_palm_data.curr_palm_num--;
							TOUCH_INFO_MSG("Palm is released : id[%d] pos[%4d,%4d] total palm:%u\n",
								i, ts->ts_palm_data.palm_coordinate[i].x,
								ts->ts_palm_data.palm_coordinate[i].y,
								ts->ts_palm_data.curr_palm_num);
						}
					}
				}

				memcpy(ts->ts_palm_data.prev_palm_mask, ts->ts_palm_data.curr_palm_mask,
						sizeof(ts->ts_palm_data.prev_palm_mask));

				if (ts->pdata->role->palm_ctrl_mode == PALM_REJECT_DRIVER) {
					if (ts->ts_palm_data.curr_palm_num) {
						ts->ts_palm_data.prev_palm_num = ts->ts_palm_data.curr_palm_num;
						memset(curr_data, 0, sizeof(struct touch_data));
						return NO_FILTER;
					} else {
						if (ts->ts_palm_data.prev_palm_num) {
							ts->ts_palm_data.all_palm_released = true;
							queue_delayed_work(touch_wq, &ts->work_palm, msecs_to_jiffies(50));
							TOUCH_INFO_MSG("All palm is released.\n");
							ts->ts_palm_data.prev_palm_num = ts->ts_palm_data.curr_palm_num;
							memset(curr_data, 0, sizeof(struct touch_data));
							return NO_FILTER;
						} else {
							if (ts->ts_palm_data.all_palm_released) {
								ts->ts_palm_data.all_palm_released = true;
								cancel_delayed_work(&ts->work_palm);
								queue_delayed_work(touch_wq, &ts->work_palm, msecs_to_jiffies(50));
								memset(curr_data, 0, sizeof(struct touch_data));
								return NO_FILTER;
							}
						}
					}
				}

				ts->ts_palm_data.prev_palm_num = ts->ts_palm_data.curr_palm_num;
				break;
		}

		TOUCH_DEBUG(DEBUG_GET_DATA, "ID[0x%x] Total_num[%d]\n", curr_data->id_mask, curr_data->total_num);
		if (ts->lpwg_ctrl.password_enable) {
			if(wake_lock_active(&ts->timer_wake_lock)) {
				if (curr_data->id_mask & ~(prev_data->id_mask)) {
					if (cancel_delayed_work(&ts->work_timer)) {
						// password-matching will be failed
						ts->pw_data.data_num = 1;
						queue_delayed_work(touch_wq,
							&ts->work_timer,
							msecs_to_jiffies(UEVENT_DELAY));
					}
				}
				return IGNORE_EVENT_BUT_SAVE_IT;
			}
			if (atomic_read(&ts->lpwg_ctrl.is_suspend) == 1) {
				TOUCH_DEBUG(DEBUG_BASE_INFO, "%s : ignore abs interrupt in suspend\n", __func__);
				return IGNORE_EVENT;
			}
		}
	} else if (ts->ts_data.interrupt_status_reg & INTERRUPT_MASK_FLASH) {
		return ERROR;
	} else {
		return IGNORE_EVENT;
	}

	if ((ts->ts_state_flag.ts_noise_log_flag == TS_NOISE_LOG_ENABLE)
		|| (ts->ts_state_flag.check_noise_menu == MENU_ENTER)
		|| touch_ta_status
		|| touch_hdmi_status)
		DO_SAFE(synaptics_ts_noise_log(client, curr_data, prev_data), error);

	return NO_ERROR;
error:
	return ERROR;
}

enum error_type synaptics_ts_filter(struct i2c_client *client, struct touch_data* curr_data, const struct touch_data* prev_data)
{
/*	struct synaptics_ts_data *ts = (struct synaptics_ts_data *)get_touch_handle(client);*/
	int i = 0;

	for (i = 0; i < curr_data->total_num; i++) {
		if (curr_data->abs_data[i].type == HOVER_TYPE) {
			curr_data->abs_data[i].pressure = 0;
		} else if (curr_data->abs_data[i].type == PALM_TYPE) {
			curr_data->abs_data[i].pressure = MAX_PRESSURE;
		} else if (curr_data->abs_data[i].pressure == MAX_PRESSURE) {
			curr_data->abs_data[i].pressure = MAX_PRESSURE - 1;
		}
	}

	return NO_ERROR;
}

enum error_type synaptics_ts_power(struct i2c_client *client, int power_ctrl)
{
	struct synaptics_ts_data *ts = (struct synaptics_ts_data *)get_touch_handle(client);

	TOUCH_TRACE();

	switch (power_ctrl) {
	case POWER_OFF:
			ts->is_init = 0;

			if ((ts->pdata->int_pin > 0) && (ts->pdata->reset_pin > 0)) {
			gpio_tlmm_config(GPIO_CFG(ts->pdata->int_pin, 0, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
			gpio_tlmm_config(GPIO_CFG(ts->pdata->reset_pin, 0, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_DOWN, GPIO_CFG_6MA), GPIO_CFG_ENABLE);
			}

			if (ts->pdata->pwr->use_regulator) {
				if (regulator_is_enabled(ts->regulator_vio))
					regulator_disable(ts->regulator_vio);
				if (regulator_is_enabled(ts->regulator_vdd))
					regulator_disable(ts->regulator_vdd);
			} else
				ts->pdata->pwr->power(0);
			break;
	case POWER_ON:
			ts->is_init = 0;
			if (ts->pdata->pwr->use_regulator) {
				if (!regulator_is_enabled(ts->regulator_vdd))
					regulator_enable(ts->regulator_vdd);
				if (!regulator_is_enabled(ts->regulator_vio))
					regulator_enable(ts->regulator_vio);
			} else
				ts->pdata->pwr->power(1);

			gpio_tlmm_config(GPIO_CFG(ts->pdata->reset_pin, 0, GPIO_CFG_OUTPUT,
			    GPIO_CFG_PULL_UP, GPIO_CFG_6MA), GPIO_CFG_ENABLE);
			gpio_direction_output(ts->pdata->reset_pin, 1);
			break;
	case POWER_SLEEP:
			if (mfts_enable) {
				break;
			}
			if ((ts->pdata->reset_pin > 0))  {
				   gpio_tlmm_config(GPIO_CFG(ts->pdata->reset_pin, 0, GPIO_CFG_INPUT,
					   GPIO_CFG_PULL_DOWN, GPIO_CFG_6MA), GPIO_CFG_ENABLE);
				   gpio_direction_input(ts->pdata->reset_pin);
				   }
			if (!ts->lpwg_ctrl.lpwg_is_enabled)
				sleep_control(ts, 0, 1);
			break;
	case POWER_WAKE:
			if (mfts_enable) {
				break;
			}
			sleep_control(ts, 1, 1);
			gpio_tlmm_config(GPIO_CFG(ts->pdata->reset_pin, 0, GPIO_CFG_OUTPUT,
				GPIO_CFG_PULL_UP, GPIO_CFG_6MA), GPIO_CFG_ENABLE);
			gpio_direction_output(ts->pdata->reset_pin, 1);
			break;
	default:
			break;
	}
    power_state = power_ctrl;

	return NO_ERROR;
}

enum error_type synaptics_ts_ic_ctrl(struct i2c_client *client, u8 code, u32 value, u32* ret)
{
	struct synaptics_ts_data *ts =
				(struct synaptics_ts_data *)get_touch_handle(client);
	u8 buf = 0;
	u8 buf_array[2] = {0};

	switch (code) {
	case IC_CTRL_READ:
		DO_SAFE(touch_i2c_read(client, value, 1, &buf), error);
		*ret = (u32)buf;
		break;
	case IC_CTRL_WRITE:
		DO_SAFE(touch_i2c_write_byte(client, ((value & 0xFFF0) >> 8), (value & 0xFF)), error);
		break;
	case IC_CTRL_BASELINE_REBASE:
		DO_SAFE(synaptics_ts_page_data_write_byte(client,
				ANALOG_PAGE, ANALOG_COMMAND_REG, value), error);
	    /* raw cap data file exist check*/
		if (raw_cap_file_exist && (ts->pdata->ref_chk_option[1] || ref_chk_enable)) {
		/*	resume_flag = 1; */
			ref_chk_rebase_stop = 1;
			if ((ts->fw_info.family && ts->fw_info.fw_revision) || !(strncmp(ts->fw_info.fw_product_id, "PLG352", 6))) {

				if (synaptics_get_reference_chk(ts) < 0) {
					TOUCH_INFO_MSG("reference checking...\n");
				}
			}
        }
		break;
	case IC_CTRL_REPORT_MODE:
		if (value == REDUCED_REPORT_MODE)
			 buf_array[0] = buf_array[1] = ts->pdata->role->delta_pos_threshold;
		DO_SAFE(touch_i2c_write(client, f12_info.ctrl_reg_addr[20], 2, buf_array), error);
		break;
	case IC_CTRL_THERMAL:
		switch (value) {
			case THERMAL_LOW:
				buf = 0x00;
				buf_array[0] = default_finger_amplitude[0];
				buf_array[1] = default_finger_amplitude[1];
				break;
			case THERMAL_HIGH:
				buf = 0x04;
				buf_array[0] = buf_array[1] = THERMAL_HIGH_FINGER_AMPLITUDE;
				break;
			default:
				TOUCH_ERR_MSG("Invalid current_thermal_mode (%u)\n", value);
				goto error;
		}

		TOUCH_INFO_MSG("High Temp Control(0x%02X), Finger Amplitude Threshold(0x%02X), Small Finger Amplitude Threshold(0x%02X)\n",
				buf, buf_array[0], buf_array[1]);

		DO_SAFE(synaptics_ts_page_data_write_byte(client, LPWG_PAGE, MISC_HOST_CONTROL_REG, buf), error);
		DO_SAFE(touch_i2c_write(client, f12_info.ctrl_reg_addr[15], 2, buf_array), error);
		break;
	default:
		break;
	}

	return NO_ERROR;
error:
	return ERROR;
}

int compare_fw_version(struct i2c_client *client, struct touch_fw_info *fw_info)
{
	struct synaptics_ts_data *ts =
			(struct synaptics_ts_data *)get_touch_handle(client);
	int i = 0;

	if (ts->fw_info.fw_version[0] > 0x40) {
		if (ts->fw_info.fw_image_version[0] > 0x40) {
			TOUCH_DEBUG(DEBUG_BASE_INFO, "product_id[%s(ic):%s(fw)] fw_version[%s(ic):%s(fw)]\n",
				ts->fw_info.fw_product_id, ts->fw_info.fw_image_product_id, ts->fw_info.fw_version, ts->fw_info.fw_image_version);
			if (strncmp(ts->fw_info.fw_version, ts->fw_info.fw_image_version, 4)) {
				TOUCH_DEBUG(DEBUG_BASE_INFO, "fw_version mismatch.\n");
				return 1;
			} else {
				goto matched;
			}
		} else {
			TOUCH_DEBUG(DEBUG_BASE_INFO, "product_id[%s(ic):%s(fw)] fw_version[%s(ic):V%d.%02d(fw)]\n",
					ts->fw_info.fw_product_id, ts->fw_info.fw_image_product_id, ts->fw_info.fw_version,
					(ts->fw_info.fw_image_version[3] & 0x80 ? 1:0), ts->fw_info.fw_image_version[3] & 0x7F);
			if (strncmp(ts->fw_info.fw_version, ts->fw_info.fw_image_version, 4)) {
				TOUCH_DEBUG(DEBUG_BASE_INFO, "fw_version mismatch.\n");
				return 1;
			} else {
				goto matched;
			}
		}
	} else {
		if(!(ts->fw_info.fw_version[3] & 0x80)) {/*temporary code to support HW team if FW in IC is Test Ver., no update FW.*/
			if((ts->fw_info.fw_version[3] & 0x7F) > 7) {
				TOUCH_DEBUG(DEBUG_BASE_INFO, "%s : FW version is Test Version.\n", __func__);
				goto matched;
			}
			else { /*Possible to upgrade FW below v0.07 */
				TOUCH_DEBUG(DEBUG_BASE_INFO, "%s : FW version is Test Version which is below v0.07\n", __func__);
				return 1;
			}
		}
		if (ts->fw_info.fw_image_version[0] > 0x40) {
			TOUCH_DEBUG(DEBUG_BASE_INFO, "product_id[%s(ic):%s(fw)] fw_version[V%d.%02d(ic):%s(fw)]\n",
					ts->fw_info.fw_product_id, ts->fw_info.fw_image_product_id,
					(ts->fw_info.fw_version[3] & 0x80 ? 1 : 0), ts->fw_info.fw_version[3] & 0x7F, ts->fw_info.fw_image_version);
			if(strncmp(ts->fw_info.fw_version, ts->fw_info.fw_image_version, 4)) {
				TOUCH_DEBUG(DEBUG_BASE_INFO, "fw_version mismatch.\n");
				return 1;
			} else {
				goto matched;
			}
		} else {
			TOUCH_DEBUG(DEBUG_BASE_INFO, "product_id[%s(ic):%s(fw)] ",
					ts->fw_info.fw_product_id, ts->fw_info.fw_image_product_id);
			TOUCH_DEBUG(DEBUG_BASE_INFO, "ic_fw_version[V%d.%02d(0x%02X 0x%02X 0x%02X 0x%02X)] ",
					(ts->fw_info.fw_version[3] & 0x80 ? 1 : 0), ts->fw_info.fw_version[3] & 0x7F,
					ts->fw_info.fw_version[0], ts->fw_info.fw_version[1], ts->fw_info.fw_version[2], ts->fw_info.fw_version[3]);
			TOUCH_DEBUG(DEBUG_BASE_INFO, "fw_version[V%d.%02d(0x%02X 0x%02X 0x%02X 0x%02X)]\n",
					(ts->fw_info.fw_image_version[3] & 0x80 ? 1:0), ts->fw_info.fw_image_version[3] & 0x7F,
					ts->fw_info.fw_image_version[0], ts->fw_info.fw_image_version[1], ts->fw_info.fw_image_version[2], ts->fw_info.fw_image_version[3]);
			if (mfts_mode) {
				if ((ts->fw_info.fw_version[3] & 0x7F) < (ts->fw_info.fw_image_version[3] & 0x7F)) {
					TOUCH_DEBUG(DEBUG_BASE_INFO, "fw_version mismatch(mfts_mode).\n");
					return 1;
				} else {
					mfts_mode = 0;
					goto matched;
				}
			} else {
				for (i = 0 ; i < FW_VER_INFO_NUM ; i++) {
					if (ts->fw_info.fw_version[i] != ts->fw_info.fw_image_version[i]) {
						TOUCH_DEBUG(DEBUG_BASE_INFO, "fw_version mismatch. ic_fw_version[%d]:0x%02X != fw_version[%d]:0x%02X\n",
							i, ts->fw_info.fw_version[i], i, ts->fw_info.fw_image_version[i]);
						return 1;
					}
				}
				goto matched;
			}
		}
	}
matched:
	return 0;
}
enum error_type synaptics_ts_fw_upgrade(struct i2c_client *client, struct touch_fw_info *info, struct touch_firmware_module *fw)
{
	struct synaptics_ts_data *ts = (struct synaptics_ts_data *)get_touch_handle(client);
	int need_upgrade = 0;
	int rc = 0;
	char path[256];

	if (info->fw_force_upgrade) {
		memcpy(path, info->fw_path, sizeof(path));
		TOUCH_DEBUG(DEBUG_BASE_INFO | DEBUG_FW_UPGRADE, "FW: need_upgrade[%d] force[%d] file[%s]\n",
				fw->need_upgrade, info->fw_force_upgrade, path);
		goto firmware;
	}

	if (info->fw_force_upgrade_cat) {
		if (mfts_mode) {
			get_ic_info(ts);
			goto compare;
		} else {
			if (ts->fw_flag == S3528_A0) {
				memcpy(path, info->fw_path, sizeof(path));
				TOUCH_DEBUG(DEBUG_BASE_INFO | DEBUG_FW_UPGRADE, "FW: need_upgrade[%d] force_cat[%d] file[%s]\n",
					fw->need_upgrade, info->fw_force_upgrade_cat, path);
				goto firmware;
			} else if (ts->fw_flag == S3528_A1) {
				memcpy(path, info->fw_path_s3528_a1, sizeof(path));
				TOUCH_DEBUG(DEBUG_BASE_INFO | DEBUG_FW_UPGRADE, "FW: need_upgrade[%d] force_cat[%d] file[%s]\n",
					fw->need_upgrade, info->fw_force_upgrade_cat, path);
				goto firmware;
			} else if (ts->fw_flag == S3528_A1_SUN) {
				memcpy(path, info->fw_path_s3528_a1_suntel, sizeof(path));
				TOUCH_DEBUG(DEBUG_BASE_INFO | DEBUG_FW_UPGRADE, "FW: need_upgrade[%d] force_cat[%d] file[%s]\n",
					fw->need_upgrade, info->fw_force_upgrade_cat, path);
				goto firmware;
			} else {
				memcpy(path, info->fw_path_s3621, sizeof(path));
				TOUCH_DEBUG(DEBUG_BASE_INFO | DEBUG_FW_UPGRADE, "FW: need_upgrade[%d] force_cat[%d] file[%s]\n",
					fw->need_upgrade, info->fw_force_upgrade_cat, path);
				goto firmware;
			}
		}
	}

compare:
	need_upgrade = !strncmp(ts->fw_info.fw_product_id,
				ts->fw_info.fw_image_product_id, sizeof(ts->fw_info.fw_product_id));

	rc = compare_fw_version(client, info);
	if (fw->need_upgrade) {
		need_upgrade = need_upgrade && rc;
	} else {
		need_upgrade = need_upgrade && rc;
	}

	if (need_upgrade || ts->fw_info.need_rewrite_firmware) {
		TOUCH_DEBUG(DEBUG_BASE_INFO | DEBUG_FW_UPGRADE, "FW: start-upgrade - need[%d] rewrite[%d]\n",
				need_upgrade, ts->fw_info.need_rewrite_firmware);

		if (ts->fw_flag == S3528_A0) {
			memcpy(path, info->fw_path, sizeof(path));
			TOUCH_DEBUG(DEBUG_BASE_INFO | DEBUG_FW_UPGRADE, "FW: need_upgrade[%d] force[%d] force_cat[%d] file[%s]\n",
				fw->need_upgrade, info->fw_force_upgrade, info->fw_force_upgrade_cat, path);
			goto firmware;
		} else if (ts->fw_flag == S3528_A1) {
			if (mfts_mode) {
				memcpy(path, info->fw_path_s3528_a1_factory, sizeof(path));
				mfts_mode = 0;
			} else {
				memcpy(path, info->fw_path_s3528_a1, sizeof(path));
			}
			TOUCH_DEBUG(DEBUG_BASE_INFO | DEBUG_FW_UPGRADE, "FW: need_upgrade[%d] force[%d] force_cat[%d] file[%s]\n",
				fw->need_upgrade, info->fw_force_upgrade, info->fw_force_upgrade_cat, path);
			goto firmware;
		} else if (ts->fw_flag == S3528_A1_SUN) {
			memcpy(path, info->fw_path_s3528_a1_suntel, sizeof(path));
			TOUCH_DEBUG(DEBUG_BASE_INFO | DEBUG_FW_UPGRADE, "FW: need_upgrade[%d] force[%d] force_cat[%d] file[%s]\n",
				fw->need_upgrade, info->fw_force_upgrade, info->fw_force_upgrade_cat, path);
			goto firmware;
		} else {
			memcpy(path, info->fw_path_s3621, sizeof(path));
			TOUCH_DEBUG(DEBUG_BASE_INFO | DEBUG_FW_UPGRADE, "FW: need_upgrade[%d] force[%d] force_cat[%d] file[%s]\n",
				fw->need_upgrade, info->fw_force_upgrade, info->fw_force_upgrade_cat, path);
			goto firmware;
		}
		/* it will be reset and initialized automatically by lge_touch_core. */
	}
	return NO_ERROR;

firmware:
	ts->fw_flag = 0;
	ts->is_probed = 0;
	ts->is_init = 0; /* During upgrading, interrupt will be ignored. */
	info->fw_force_upgrade = 0;
	info->fw_force_upgrade_cat = 0;
	need_scan_pdt = true;
	fw_upgrade_state = 1;
	DO_SAFE(FirmwareUpgrade(ts, path), error);
	fw_upgrade_state = 0;
	return NO_ERROR;
error:
	fw_upgrade_state = 0;
	return ERROR;
}

enum error_type synaptics_ts_notify(struct i2c_client *client, u8 code, u32 value)
{
	/* struct synaptics_ts_data *ts = (struct synaptics_ts_data *)get_touch_handle(client); */

	switch (code) {
	case NOTIFY_TA_CONNECTION:
		break;
	case NOTIFY_TEMPERATURE_CHANGE:
		break;
	case NOTIFY_PROXIMITY:
		break;
	default:
		break;
	}

	return NO_ERROR;
}

enum error_type synaptics_ts_suspend(struct i2c_client *client)
{
	struct synaptics_ts_data *ts = (struct synaptics_ts_data *)get_touch_handle(client);

	/*because s3621 doesn't support knock-on*/
	if(!(strncmp(ts->fw_info.fw_product_id, "PLG298", 6)))
		return NO_ERROR;

	if (!atomic_read(&ts->lpwg_ctrl.is_suspend)) {
		DO_SAFE(lpwg_control(ts, ts->lpwg_ctrl.lpwg_mode), error);
		atomic_set(&ts->lpwg_ctrl.is_suspend, 1);
	}
	return NO_ERROR;
error:
	return ERROR;
}

enum error_type synaptics_ts_resume(struct i2c_client *client)
{
	struct synaptics_ts_data *ts = (struct synaptics_ts_data *)get_touch_handle(client);

	cancel_delayed_work_sync(&ts->work_timer);

	if (wake_lock_active(&ts->timer_wake_lock))
		wake_unlock(&ts->timer_wake_lock);

	atomic_set(&ts->lpwg_ctrl.is_suspend, 0);

	return NO_ERROR;
}
enum error_type synaptics_ts_lpwg(struct i2c_client *client, u32 code, u32 value, struct point *data)
{
	int i;
	u8 buffer[50] = {0};
	struct synaptics_ts_data *ts = (struct synaptics_ts_data *)get_touch_handle(client);
	u8 mode = ts->lpwg_ctrl.lpwg_mode;
	u8 doubleTap_area_reg_addr = f12_info.ctrl_reg_addr[18];

	if (mfts_enable) {
		goto error;
	}

	/*because s3621 doesn't support knock-on */
	if (!(strncmp(ts->fw_info.fw_product_id, "PLG298",6)))
		return NO_ERROR;

	switch (code) {
	case LPWG_READ:
		memcpy(data, ts->pw_data.data, sizeof(struct point)*ts->pw_data.data_num);

		data[ts->pw_data.data_num].x = -1;
		data[ts->pw_data.data_num].y = -1;
		/* '-1' should be assigned to the last data. */
		/* Each data should be converted to LCD-resolution.*/
		memset(ts->pw_data.data, -1, sizeof(struct point)*ts->pw_data.data_num);
		break;
	case LPWG_ENABLE:
		if (!atomic_read(&ts->lpwg_ctrl.is_suspend)) {
			ts->lpwg_ctrl.lpwg_mode = value;
		}
		break;
	case LPWG_LCD_X:
	case LPWG_LCD_Y:
		/* If touch-resolution is not same with LCD-resolution,*/
		/* position-data should be converted to LCD-resolution.*/
		break;
	case LPWG_ACTIVE_AREA_X1:
		for (i = 0; i < 2; i++) {
			synaptics_ts_page_data_read(client, COMMON_PAGE, doubleTap_area_reg_addr, i+1, buffer);
			if(i==0)	buffer[i] = value;
			else	buffer[i] = value >> 8;
			synaptics_ts_page_data_write(client, COMMON_PAGE, doubleTap_area_reg_addr, i+1, buffer);
		}
		break;
	case LPWG_ACTIVE_AREA_X2:
		for (i = 4; i < 6; i++) {
			synaptics_ts_page_data_read(client, COMMON_PAGE, doubleTap_area_reg_addr, i+1, buffer);
			if(i == 4)
				buffer[i] = value;
			else
				buffer[i] = value >> 8;
			synaptics_ts_page_data_write(client, COMMON_PAGE, doubleTap_area_reg_addr, i+1, buffer);
		}
		break;
	case LPWG_ACTIVE_AREA_Y1:
		for(i = 2; i < 4; i++) {
			synaptics_ts_page_data_read(client, COMMON_PAGE, doubleTap_area_reg_addr, i+1, buffer);
			if(i == 2)
				buffer[i] = value;
			else
				buffer[i] = value >> 8;
			synaptics_ts_page_data_write(client, COMMON_PAGE, doubleTap_area_reg_addr, i+1, buffer);
		}
		break;
	case LPWG_ACTIVE_AREA_Y2:
		/* Quick Cover Area */
		for (i = 6; i < 8; i++){
			synaptics_ts_page_data_read(client, COMMON_PAGE, doubleTap_area_reg_addr, i+1, buffer);
			if(i == 6)
				buffer[i] = value;
			else
				buffer[i] = value >> 8;
			synaptics_ts_page_data_write(client, COMMON_PAGE, doubleTap_area_reg_addr, i+1, buffer);
		}
		sleep_control(ts, 1, 0);
		break;
	case LPWG_TAP_COUNT:
		ts->pw_data.tap_count = value;
		if (ts->lpwg_ctrl.password_enable) {
			tci_control(ts, TAP_COUNT_CTRL, (u8)ts->pw_data.tap_count);
		}
		break;
	case LPWG_LENGTH_BETWEEN_TAP:
		if (ts->lpwg_ctrl.double_tap_enable || ts->lpwg_ctrl.password_enable) {
			tci_control(ts, TAP_DISTANCE_CTRL, value);
		}
		break;
	case LPWG_EARLY_SUSPEND:
		/*to avoid bug*/
		/*if interrupt occured so power state changed when doing rebase => touch irq bug*/
		if ((ts->pdata->ref_chk_option[1] || ref_chk_enable) && (value == 0) && ref_chk_rebase_stop){
			touch_enable_irq(ts->client->irq);
			ref_chk_rebase_stop = 0;
			}
		if (mode) { /*wakeup gesture enable */
			if (value) {
				if (atomic_read(&ts->lpwg_ctrl.is_suspend) == 1
					&& (power_state == POWER_OFF || power_state == POWER_SLEEP))
					ts->is_init = 0;
				sleep_control(ts, 1, 0);
				DO_SAFE(lpwg_control(ts, 0), error);
				atomic_set(&ts->lpwg_ctrl.is_suspend, 0);
			} else {
				set_doze_param(ts);
				sleep_control(ts, 1, 0);
				DO_SAFE(lpwg_control(ts, ts->lpwg_ctrl.lpwg_mode), error);
				atomic_set(&ts->lpwg_ctrl.is_suspend, 1);
			}
		}
		else { /*wakeup gesture disable */
			if (value) /*lcd on*/
				enable_irq_wake(ts->client->irq);
			else /*lcd off*/
				disable_irq_wake(ts->client->irq);
		}
		break;
	case LPWG_SENSOR_STATUS:
		if (mode) {
			/* IC should sleep when proximity is 'NEAR'.*/
			DO_SAFE(sleep_control(ts, value, 0), error);

			if (value) {	/* Far*/
				DO_SAFE(lpwg_control(ts, mode), error);
			} else {	/* Near*/
				if (ts->lpwg_ctrl.password_enable && wake_lock_active(&ts->timer_wake_lock)) {
					cancel_delayed_work_sync(&ts->work_timer);
					tci_control(ts, REPORT_MODE_CTRL, 1);		/* wakeup_gesture_only*/
					wake_unlock(&ts->timer_wake_lock);
				}
			}
		}
		break;
	case LPWG_DOUBLE_TAP_CHECK:
		ts->pw_data.double_tap_check = value;
		if (ts->lpwg_ctrl.password_enable) {
			tci_control(ts, INTERRUPT_DELAY_CTRL, value);
		}
		break;
	case LPWG_REPLY:
		TOUCH_DEBUG(DEBUG_BASE_INFO || DEBUG_LPWG, "%s : reply = %d\n", __func__, value);

		if (ts->pdata->role->use_lpwg_all) {
			if (atomic_read(&ts->lpwg_ctrl.is_suspend) == 0) {
				TOUCH_DEBUG(DEBUG_BASE_INFO || DEBUG_LPWG, "%s : screen on\n", __func__);
				break;
			}
			ts->lpwg_ctrl.screen = value;
			DO_SAFE(lpwg_update_all(ts, 1), error);
		} else {
			DO_SAFE(sleep_control(ts, 1, 0), error);
			if (ts->lpwg_ctrl.password_enable && !value)
				DO_SAFE(lpwg_control(ts, mode), error);
		}
		break;
	case LPWG_UPDATE_ALL:
	{
		int *v = (int *) value;
		int mode = *(v + 0);
		int screen = *(v + 1);
		int sensor = *(v + 2);
		int qcover = *(v + 3);

		ts->lpwg_ctrl.lpwg_mode = mode;
		ts->lpwg_ctrl.screen = screen;
		ts->lpwg_ctrl.sensor = sensor;
		ts->lpwg_ctrl.qcover = qcover;

		TOUCH_DEBUG(DEBUG_BASE_INFO || DEBUG_LPWG, "mode[%d], screen[%d], sensor[%d], qcover[%d]\n",
			ts->lpwg_ctrl.lpwg_mode, ts->lpwg_ctrl.screen, ts->lpwg_ctrl.sensor, ts->lpwg_ctrl.qcover);

		DO_SAFE(lpwg_update_all(ts, 1), error);
		break;
	}
	default:
		break;
	}

	return NO_ERROR;
error:
	return ERROR;
}

static void synapitcs_change_ime_status(struct i2c_client *client, int ime_status)
{
	struct synaptics_ts_data *ts = (struct synaptics_ts_data *)get_touch_handle(client);

	u8 udata[5] = {0};
	u8 drumming_address = f12_info.ctrl_reg_addr[10];

	touch_i2c_read(ts->client, drumming_address, 5, udata);

	if (ime_status) {
		TOUCH_INFO_MSG("%s : IME ON \n",__func__);
		udata[3] = 0x08;/*Drumming Acceleration Threshold*/
		udata[4] = 0x05;/*Minimum Drumming Separation*/
		if(touch_i2c_write(ts->client, drumming_address, 5, udata) < 0) {
			TOUCH_ERR_MSG("%s : Touch i2c write fail \n", __func__);
		}
	 } else {
	 	udata[3] = 0x0f;/*Drumming Acceleration Threshold*/
		udata[4] = 0x0a;/*Minimum Drumming Separation*/
 		if (touch_i2c_write(ts->client, drumming_address, 5, udata) < 0) {
			TOUCH_ERR_MSG("%s : Touch i2c write fail \n", __func__);
 		}
		TOUCH_INFO_MSG("%s : IME OFF\n",__func__);
	 }
	 return;
}

static int set_doze_param(struct synaptics_ts_data *ts)
{
	int interval = 3;
	int wakeup = 30;
	u8 buf[6] = {0};

	if (mfts_enable) {
		goto error;
	}

	DO_SAFE(touch_i2c_read(ts->client, f12_info.ctrl_reg_addr[27], 6, buf), error);

	/* max active duration */
	if (ts->pw_data.tap_count < 3)
		buf[3] = 3;
	else
		buf[3] = 3 + ts->pw_data.tap_count;

	buf[2] = 0x0C;	/* False Activation Threshold */
	buf[4] = 0x01;	/* Timer 1 */
	buf[5] = 0x01;	/* Max Active Duration Timeout */

	DO_SAFE(touch_i2c_write(ts->client, f12_info.ctrl_reg_addr[27], 6, buf), error);
	DO_SAFE(touch_i2c_write_byte(ts->client, DOZE_INTERVAL_REG, interval), error);
	DO_SAFE(touch_i2c_write_byte(ts->client, DOZE_WAKEUP_THRESHOLD_REG, wakeup), error);

	TOUCH_DEBUG(DEBUG_BASE_INFO, "%s: [%d] [%d] [%d] [%d] [%d]\n",
					__func__, interval, wakeup, (int)buf[3], (int)buf[4], (int)buf[5]);
	TOUCH_INFO_MSG("Set doze parameters\n");

	return 0;
error:
	return -1;
}

enum window_status synapitcs_check_crack(struct i2c_client *client)
{
	struct synaptics_ts_data *ts = (struct synaptics_ts_data *)get_touch_handle(client);
	char result[2] = {0x00, };

	if (need_scan_pdt) {
		SCAN_PDT();
		need_scan_pdt = false;
	}

	touch_disable_irq(ts->client->irq);
	F54Test('l', (int)ts->pdata->role->crack_detection->min_cap_value, result);
	touch_enable_irq(ts->client->irq);

	TOUCH_DEBUG(DEBUG_BASE_INFO, "check window crack = %s.\n", result);

	after_crack_check = 1; // set crack check flag

	if (strncmp(result, "1", 1) == 0)
		return CRACK;
	else
		return NO_CRACK;
}

struct touch_device_driver synaptics_ts_driver = {
	.probe		= synaptics_ts_probe,
	.remove		= synaptics_ts_remove,
	.suspend	= synaptics_ts_suspend,
	.resume		= synaptics_ts_resume,
	.init  		= synaptics_ts_init,
	.data  		= synaptics_ts_get_data,
	.filter		= synaptics_ts_filter,
	.power 		= synaptics_ts_power,
	.ic_ctrl	= synaptics_ts_ic_ctrl,
	.fw_upgrade 	= synaptics_ts_fw_upgrade,
	.notify		= synaptics_ts_notify,
	.lpwg		= synaptics_ts_lpwg,
	.ime_drumming = synapitcs_change_ime_status,
	.inspection_crack = synapitcs_check_crack
};

static struct of_device_id match_table[] = {
	{ .compatible = "synaptics,s3528",},
	{ },
};
static void async_touch_init(void *data, async_cookie_t cookie)
{
	touch_driver_register(&synaptics_ts_driver, match_table);
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

