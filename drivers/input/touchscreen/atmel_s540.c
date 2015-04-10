/*
 * Atmel maXTouch Touchscreen driver
 *
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/uaccess.h>
#include <asm/unaligned.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/i2c/atmel_s540.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <mach/board_lge.h>
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif

#include <linux/time.h>

#include <linux/wakelock.h>
#include <linux/mutex.h>
#include <linux/mfd/pm8xxx/cradle.h>
#include <linux/sysdev.h>
#include "atmel_s540_mfts_config.h"
#ifdef TSP_PATCH
static u8 patch_bin[] = {
	#include "mxts_patch_bin.h"
};

#define MXT_PATCH_BAT_MODE_EVENT	0	/* 0 : BAT mode */
#define MXT_PATCH_TA_MODE_EVENT	1	/* 1: TA mode */
#define MXT_PATCH_KNOCKON_BAT_MODE_EVENT	2	/* 2: Knock on BAT mode */
#define MXT_PATCH_WAKEUP_BAT_MODE_EVENT	3	/* 3: Wake Up  BAT mode */
#define MXT_PATCH_KNOCKON_TA_MODE_EVENT	4	/* 4: Knock On TA mode */
#define MXT_PATCH_WAKEUP_TA_MODE_EVENT	5	/* 5: Wakeup  TA mode */
#define MXT_PATCH_PASSWORD_BAT_MODE_EVENT	6
#define MXT_PATCH_PASSWORD_TA_MODE_EVENT	7
#define MXT_PATCH_WIRELESS_TA_MODE_EVENT	8
#define MXT_PATCH_WAKEUP_WIRELESS_TA_MODE_EVENT	9
#define MXT_PATCH_FTM_BAT_MODE_EVENT	10
#define MXT_PATCH_FTM_TA_MODE_EVENT	11
#define MXT_PATCH_QUICKCOVER_CLOSED_MODE_EVENT	12
#define MXT_DEEPSLEEP_MODE		13

#include "mxts_patch.c"
#else
#include "atmel_s540_config.h"
#endif

#define DEBUG_ABS	1
#define FIRMUP_ON_PROBE

/* Configuration file */
#define MXT_CFG_MAGIC		"OBP_RAW V1"
#ifdef FIRMUP_ON_PROBE
#ifdef MXT_GESTURE_RECOGNIZE
#ifdef MXT_LPWG
#define MXT_LATEST_CONFIG_CRC	0xC2D8E2
//#define UDF_CONTROL_CLEAR_T37_DATA
u8 latest_firmware[] = {
#ifdef ALPHA_FW
	#include "mXT540SC02_V5.0.F6_.h"
#else
	#include "mxt540s_F1.enc.h" // pre Alpha - based on area, has T84
#endif
};
#else
#define MXT_LATEST_CONFIG_CRC	0xA8EA22
u8 latest_firmware[] = {
	#include "mXT540S_V1.0.E2_.h"
};
#endif
#else
#define MXT_LATEST_CONFIG_CRC	0x629BB2
u8 latest_firmware[] = {
	#include "mxt540s_V3.0.AA_.h"
};
#endif
#endif
#define MXT_LATEST_CONFIG_CRC_MFTS	0x5F4D1E
#define MFTS_T6_ADDRESS			355 /* 3.0 ver */
#define ORIGINAL_T6_ADDRESS		367	/* 5.0 ver */

#ifdef CUST_B_TOUCH
#define get_time_interval(a,b) a>=b ? a-b : 1000000+a-b
struct timeval t_ex_debug[TIME_EX_PROFILE_MAX];
bool touch_probe_ok = 0;
#endif

#define MXT_ANTI_ENABLE_MASK	 0x0F
#define MXT_ANTI_DISABLE_MASK	 0xF0
static bool is_probing;
static bool selftest_enable;
static bool selftest_show;
static bool update_cfg_force = true;
static bool update_fw_force;
static bool chargerlogo;
int quick_cover_status = 0;
int use_quick_window = 0;
int outside_touch = 0;
#ifdef MXT_WIRELESS
bool wireless = 0;
#endif
#ifdef MXT_FACTORY
static bool factorymode = false;
#endif
bool mxt_mfts = false;
bool mxt_mfts_30 = false;
u32 config_crc_mfts = 0;
bool mxt_mfts_for_mxt_start = false;
/* check TA status */
static bool wait_change_cfg = false;
static int ime_drumming_status = 0;

#ifdef MXT_GESTURE_RECOGNIZE
#define MXT_T100_REPORT_EN_MASK	0x02
#define MXT_T100_REPORT_DIS_MASK	0xFD
#define MXT_ENABLE_MASK	0x01
#define MXT_DISABLE_MASK 0xFE
/* Setting Gesture mode (Sleep Current) */
static u8 t47_ctrl_cfg = 0;
static u8 t65_ctrl_cfg = 0;
static u8 t72_ctrl_cfg = 0;
static u8 t100_ctrl_cfg = 0;
#endif

static void safety_reset(struct mxt_data *data);
#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
/* Early-suspend level */
#define MXT_SUSPEND_LEVEL 1
static void mxt_early_suspend(struct early_suspend *h);
static void mxt_late_resume(struct early_suspend *h);
#endif
struct workqueue_struct*	touch_wq;
extern int cradle_smart_cover_status(void);

struct lge_touch_attribute {
	struct attribute	attr;
	ssize_t (*show)(struct mxt_data *ts, char *buf);
	ssize_t (*store)(struct mxt_data *ts, const char *buf, size_t count);
};

#define LGE_TOUCH_ATTR(_name, _mode, _show, _store)	\
struct lge_touch_attribute lge_touch_attr_##_name = __ATTR(_name, _mode, _show, _store)

/* Auto Test interface for some model */
struct mxt_data *touch_test_dev = NULL;
EXPORT_SYMBOL(touch_test_dev);

#ifdef MXT_GESTURE_RECOGNIZE
static struct wake_lock touch_wake_lock;
static struct mutex i2c_suspend_lock;
static struct mutex mxt_early_mutex;
static bool touch_irq_wake = 0;

static int touch_enable_irq_wake(unsigned int irq){
	int ret = 0;

	TOUCH_INFO_MSG("Enable Touch IRQ Wake [%d]\n", touch_irq_wake);
	if(!touch_irq_wake){
		touch_irq_wake = 1;
		ret= enable_irq_wake(irq);
	}
	return ret;
}
static int touch_disable_irq_wake(unsigned int irq){
	int ret = 0;

	TOUCH_INFO_MSG("Disable Touch IRQ Wake [%d]\n", touch_irq_wake);
	if(touch_irq_wake){
		touch_irq_wake = 0;
		ret = disable_irq_wake(irq);
	}
	return ret;
}
#endif

static bool touch_enable = 1;
static void touch_enable_irq(unsigned int irq){
	TOUCH_INFO_MSG("Enable Touch IRQ [%d]\n", touch_enable);

	if(!touch_enable){
		touch_enable = 1;
		enable_irq(irq);
	}
}
static void touch_disable_irq(unsigned int irq){
	TOUCH_INFO_MSG("Disable Touch IRQ [%d]\n", touch_enable);

	if(touch_enable){
		touch_enable = 0;
		disable_irq(irq);
	}
}

inline size_t mxt_obj_size(const struct mxt_object *obj)
{
	return obj->size_minus_one + 1;
}

static inline size_t mxt_obj_instances(const struct mxt_object *obj)
{
	return obj->instances_minus_one + 1;
}

static bool mxt_object_readable(unsigned int type)
{
	switch (type) {
	case MXT_GEN_COMMAND_T6:
	case MXT_SPT_USERDATA_T38:
	case MXT_SPT_DYNAMICCONFIGURATIONCONTAINER_T71:
	case MXT_GEN_POWER_T7:
	case MXT_GEN_ACQUIRE_T8:
	case MXT_TOUCH_KEYARRAY_T15:
	case MXT_SPT_COMMSCONFIG_T18:
	case MXT_SPT_GPIOPWM_T19:
	case MXT_TOUCH_PROXIMITY_T23:
#ifdef MXT_GESTURE_RECOGNIZE
	case MXT_PROCI_ONETOUCH_T24:
#endif
	case MXT_SPT_SELFTEST_T25:
	case MXT_PROCI_GRIPSUPPRESSION_T40:
	case MXT_PROCI_TOUCHSUPPRESSION_T42:
	case MXT_SPT_CTECONFIG_T46:
	case MXT_PROCI_STYLUS_T47:
	case MXT_PROCI_ADAPTIVETHRESHOLD_T55:
	case MXT_PROCI_SHIELDLESS_T56:
	/*case MXT_SPT_TIMER_T61:
	case MXT_PROCI_LENSBENDING_T65:
	case MXT_SPT_GOLDENREFERENCES_T66:
	case MXT_PROCI_PALMGESTUREPROCESSOR_T69:
	case MXT_SPT_DYNAMICCONFIGURATIONCONTROLLER_T70:
	case MXT_PROCG_NOISESUPPRESSION_T72:
	case MXT_GLOVEDETECTION_T78:
	case MXT_RETRANSMISSIONCOMPENSATION_T80:
	case MXT_PROCI_GESTUREPROCESSOR_T84:
	case MXT_TOUCH_MULTITOUCHSCREEN_T100:
	case MXT_SPT_TOUCHSCREENHOVER_T101:
	case MXT_SPT_SELFCAPCBCRCONFIG_T102:
	case MXT_PROCI_SCHNOISESUPPRESSION_T103:
	case MXT_SPT_AUXTOUCHCONFIG_T104:
	case MXT_SPT_DRIVENPLATEHOVERCONFIG_T105:*/
		return true;
	default:
		return false;
	}
}

static void mxt_dump_message(struct mxt_data *data, u8 *message)
{
	print_hex_dump(KERN_DEBUG, "MXT MSG:", DUMP_PREFIX_NONE, 16, 1,
		       message, data->T5_msg_size, false);
}

static int mxt_wait_for_completion(struct mxt_data *data,
			struct completion *comp, unsigned int timeout_ms)
{
	struct device *dev = &data->client->dev;
	unsigned long timeout = msecs_to_jiffies(timeout_ms);
	long ret;

	ret = wait_for_completion_interruptible_timeout(comp, timeout);
	if (ret < 0) {
		dev_err(dev, "Wait for completion interrupted.\n");
		return -EINTR;
	} else if (ret == 0) {
		dev_err(dev, "Wait for completion timed out.\n");
		return -ETIMEDOUT;
	}
	return 0;
}

static int mxt_bootloader_read(struct mxt_data *data,
			       u8 *val, unsigned int count)
{
	int ret;
	struct i2c_msg msg;

	msg.addr = data->bootloader_addr;
	msg.flags = data->client->flags & I2C_M_TEN;
	msg.flags |= I2C_M_RD;
	msg.len = count;
	msg.buf = val;

	ret = i2c_transfer(data->client->adapter, &msg, 1);

	if (ret == 1) {
		ret = 0;
	} else {
		ret = (ret < 0) ? ret : -EIO;
		dev_err(&data->client->dev, "i2c recv failed (%d)\n", ret);
	}

	return ret;
}

static int mxt_bootloader_write(struct mxt_data *data,
				const u8 * const val, unsigned int count)
{
	int ret;
	struct i2c_msg msg;

	msg.addr = data->bootloader_addr;
	msg.flags = data->client->flags & I2C_M_TEN;
	msg.len = count;
	msg.buf = (u8 *)val;

	ret = i2c_transfer(data->client->adapter, &msg, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		ret = (ret < 0) ? ret : -EIO;
		dev_err(&data->client->dev, "i2c send failed (%d)\n", ret);
	}

	return ret;
}

static int mxt_lookup_bootloader_address(struct mxt_data *data, u8 retry)
{
	u8 appmode = data->client->addr;
	u8 bootloader;
	u8 family_id = 0;

	if (data->info)
		family_id = data->info->family_id;

	switch (appmode) {
	case 0x4a:
	case 0x4b:
		/* Chips after 1664S use different scheme */
		if ((retry % 2) || family_id >= 0xa2) {
			bootloader = appmode - 0x24;
			break;
		}
		/* Fall through for normal case */
	case 0x4c:
	case 0x4d:
	case 0x5a:
	case 0x5b:
		bootloader = appmode - 0x26;
		break;
	default:
		dev_err(&data->client->dev,
			"Appmode i2c address 0x%02x not found\n",
			appmode);
		return -EINVAL;
	}

	data->bootloader_addr = bootloader;
	return 0;
}

static int mxt_probe_bootloader(struct mxt_data *data, u8 retry)
{
	struct device *dev = &data->client->dev;
	int ret;
	u8 val;
	bool crc_failure;
	dev_info(dev, "mxt probe bootloader\n");
	ret = mxt_lookup_bootloader_address(data, retry);
	if (ret)
		return ret;

	ret = mxt_bootloader_read(data, &val, 1);
	if (ret)
		return ret;

	/* Check app crc fail mode */
	crc_failure = (val & ~MXT_BOOT_STATUS_MASK) == MXT_APP_CRC_FAIL;

	dev_err(dev, "Detected bootloader, status:%02X%s\n",
			val, crc_failure ? ", APP_CRC_FAIL" : "");

	return 0;
}

static u8 mxt_get_bootloader_version(struct mxt_data *data, u8 val)
{
	struct device *dev = &data->client->dev;
	u8 buf[3];

	if (val & MXT_BOOT_EXTENDED_ID) {
		if (mxt_bootloader_read(data, &buf[0], 3) != 0) {
			dev_err(dev, "%s: i2c failure\n", __func__);
			return -EIO;
		}

		dev_info(dev, "Bootloader ID:%d Version:%d\n", buf[1], buf[2]);

		return buf[0];
	} else {
		dev_info(dev, "Bootloader ID:%d\n", val & MXT_BOOT_ID_MASK);

		return val;
	}
}

static int mxt_check_bootloader(struct mxt_data *data, unsigned int state)
{
	struct device *dev = &data->client->dev;
	u8 val;
	int ret;

recheck:
	if (state != MXT_WAITING_BOOTLOAD_CMD) {
		/*
		 * In application update mode, the interrupt
		 * line signals state transitions. We must wait for the
		 * CHG assertion before reading the status byte.
		 * Once the status byte has been read, the line is deasserted.
		 */
		ret = mxt_wait_for_completion(data, &data->bl_completion,
					      MXT_FW_CHG_TIMEOUT);
		if (ret) {
			/*
			 * TODO: handle -EINTR better by terminating fw update
			 * process before returning to userspace by writing
			 * length 0x000 to device (iff we are in
			 * WAITING_FRAME_DATA state).
			 */
			dev_err(dev, "Update wait error %d\n", ret);
			return ret;
		}
	}

	ret = mxt_bootloader_read(data, &val, 1);
	if (ret)
		return ret;

	if (state == MXT_WAITING_BOOTLOAD_CMD)
		val = mxt_get_bootloader_version(data, val);

	switch (state) {
	case MXT_WAITING_BOOTLOAD_CMD:
	case MXT_WAITING_FRAME_DATA:
	case MXT_APP_CRC_FAIL:
		val &= ~MXT_BOOT_STATUS_MASK;
		break;
	case MXT_FRAME_CRC_PASS:
		if (val == MXT_FRAME_CRC_CHECK) {
			goto recheck;
		} else if (val == MXT_FRAME_CRC_FAIL) {
			dev_err(dev, "Bootloader CRC fail\n");
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	if (val != state) {
		dev_err(dev, "Invalid bootloader state %02X != %02X\n",
			val, state);
		return -EINVAL;
	}

	return 0;
}

static int mxt_send_bootloader_cmd(struct mxt_data *data, bool unlock)
{
	int ret;
	u8 buf[2];

	if (unlock) {
		buf[0] = MXT_UNLOCK_CMD_LSB;
		buf[1] = MXT_UNLOCK_CMD_MSB;
	} else {
		buf[0] = 0x01;
		buf[1] = 0x01;
	}

	ret = mxt_bootloader_write(data, buf, 2);
	if (ret)
		return ret;

	return 0;
}

static int __mxt_read_reg(struct i2c_client *client,
			       u16 reg, u16 len, void *val)
{
	struct mxt_data *data = i2c_get_clientdata(client);
	struct i2c_msg xfer[2];
	u8 buf[2];
#ifdef MXT_GESTURE_RECOGNIZE
	int i=0;
#else
	int ret;
	bool retry = false;
#endif


	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;

	/* Write register */
	xfer[0].addr = client->addr;
	xfer[0].flags = 0;
	xfer[0].len = 2;
	xfer[0].buf = buf;

	/* Read data */
	xfer[1].addr = client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = len;
	xfer[1].buf = val;
#ifdef MXT_GESTURE_RECOGNIZE
	do {
		if (i2c_transfer(client->adapter, xfer, ARRAY_SIZE(xfer))==2)
			return 0;
		dev_dbg(&client->dev, "%s: i2c retry %d\n", __func__, i+1);
		msleep(MXT_WAKEUP_TIME);
	} while (++i < 3);

	dev_err(&client->dev, "%s: i2c transfer failed\n", __func__);
	if (unlikely(mxt_mfts_for_mxt_start))
		return -EIO;
	else
		goto io_error;
#else
retry_read:
	ret = i2c_transfer(client->adapter, xfer, ARRAY_SIZE(xfer));
	if (ret != ARRAY_SIZE(xfer)) {
		if (!retry) {
			dev_err(&client->dev, "%s: i2c retry\n", __func__);
			msleep(MXT_WAKEUP_TIME);
			retry = true;
			goto retry_read;
		} else {
			dev_err(&client->dev, "%s: i2c transfer failed (%d)\n",
				__func__, ret);
			return -EIO;
		}
	}

	return 0;
#endif
io_error:
	WARN(1, "I/O error occured. safety_reset will be invoked\n");
	safety_reset(data);
	return 0;
}

static int __mxt_write_reg(struct i2c_client *client, u16 reg, u16 len,
			   const void *val)
{
	struct mxt_data *data = i2c_get_clientdata(client);
	u8 *buf;
	size_t count;
#ifdef MXT_GESTURE_RECOGNIZE
	int i = 0;
#else
	int ret;
	bool retry = false;
#endif

	count = len + 2;
	buf = kmalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;
	memcpy(&buf[2], val, len);
#ifdef MXT_GESTURE_RECOGNIZE
	do {
		if (i2c_master_send(client, buf, count)==count){
			kfree(buf);
			return 0;
		}
		dev_info(&client->dev, "%s: i2c retry %d\n", __func__, i+1);
		msleep(MXT_WAKEUP_TIME);
	} while (++i < 3);
		dev_err(&client->dev, "%s: i2c transfer failed\n", __func__);
		kfree(buf);
		if (unlikely(mxt_mfts_for_mxt_start))
			return -EIO;
		else
			goto io_error;
#else
retry_write:
	ret = i2c_master_send(client, buf, count);
	if (ret == count) {
		ret = 0;
	} else {
		if (!retry) {
			dev_err(&client->dev, "%s: i2c retry\n", __func__);
			msleep(MXT_WAKEUP_TIME);
			retry = true;
			goto retry_write;
		} else {
			dev_err(&client->dev, "%s: i2c send failed (%d)\n",
				__func__, ret);
			ret = -EIO;
		}
	}

	kfree(buf);
	return ret;
#endif
io_error:
	WARN(1, "I/O error occured. safety_reset will be invoked\n");
	safety_reset(data);
	return 0;
}

int mxt_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	return __mxt_write_reg(client, reg, 1, &val);
}

#ifdef TSP_PATCH
int mxt_write_mem(struct mxt_data *data,
		u16 reg, u8 len, const u8 *buf)
{
	int ret = 0;
	ret = __mxt_write_reg(data->client, reg, len, buf);
	return ret;
}

int mxt_read_mem(struct mxt_data *data, u16 reg, u8 len, void *buf)
{
	int ret = 0;
	ret = __mxt_read_reg(data->client, reg, len, buf);
	return ret;
}
#endif

static void mxt_make_reportid_table(struct mxt_data *data)
{
	struct mxt_object *object = data->object_table;
	struct mxt_reportid *reportids = data->reportids;		
	int i, j;
	int id = 0;

	for (i = 0; i < data->info->object_num; i++) {
		for (j = 0; j < object[i].num_report_ids * (object[i].instances_minus_one+1); j++) {
			id++;

			reportids[id].type = object[i].type;
			reportids[id].index = j;

			dev_dbg(&data->client->dev, "Report_id[%d]:\tType=%d\tIndex[%d]\n",
					id, reportids[id].type, reportids[id].index);
		}
	}
}

struct mxt_object *mxt_get_object(struct mxt_data *data, u8 type)
{
	struct mxt_object *object;
	int i;

	for (i = 0; i < data->info->object_num; i++) {
		object = data->object_table + i;
		if (object->type == type)
			return object;
	}

	dev_err(&data->client->dev, "Invalid object type T%u\n", type);
	return NULL;
}

#ifdef FIRMUP_ON_PROBE
static int mxt_init_write_config(struct mxt_data *data,
		u8 type, const u8 *cfg)
{
	struct mxt_object *object;
	int ret;
	u8 *val;

	object = mxt_get_object(data, type);
	if (!object) {
        dev_err(&data->client->dev,
			"%s error Cannot get object_type T%d\n", __func__, type);
        return -EINVAL;
    }

	if ((mxt_obj_size(object) == 0) || (object->start_address == 0)) {
		dev_err(&data->client->dev,
			"%s error object_type T%d\n", __func__, type);
		return -ENODEV;
	}

	dev_dbg(&data->client->dev, "Write config T%d: start_addr=%d, size=%d * instance=%d\n",
		type, object->start_address, mxt_obj_size(object), mxt_obj_instances(object));

	val = kmalloc(mxt_obj_size(object)*mxt_obj_instances(object), GFP_KERNEL);
	memset(val, 0, sizeof(val));
	ret = __mxt_write_reg(data->client, object->start_address,
					mxt_obj_size(object)*mxt_obj_instances(object), val);
	kfree(val);

	ret = __mxt_write_reg(data->client, object->start_address,
				mxt_obj_size(object)*mxt_obj_instances(object), cfg);

	if (ret) {
		dev_err(&data->client->dev,
			"%s write error T%d address[0x%x]\n",
			__func__, type, object->start_address);
		return ret;
	}

	return ret;
}

static int mxt_write_configuration(struct mxt_data *data)
{
	int i = 0;
	int ret = 0;
	int max = 0;
	u8 ** tsp_config = (u8 **)data->pdata->config_array->config_t;

	max = MXT_TMAX;

	if(mxt_mfts && mxt_mfts_30){
		dev_info(&data->client->dev, "MFTS 3.0 version\n");
		tsp_config = (u8 **)config_mfts;
		max = 31; /* 3.0 ver object num */
	}

	dev_info(&data->client->dev, "Write configuration data\n");

	for (i = 0; i < max; i++) {
		ret = mxt_init_write_config(data, tsp_config[i][0],
							tsp_config[i] + 1);
		if (ret) {
			dev_err(&data->client->dev, "Failed to write configuration\n");
			goto out;
		}
	}

out:
	return ret;
}
#endif
#ifdef MXT_TA_TIME
void trigger_early_baseline_state_machine(int plug_in)
{
	if(!touch_probe_ok)
		return;
	do_gettimeofday(&t_ex_debug[TIME_TA_DETECT]);
	printk("[lge_touch] EARLY DETECT ! TA/USB CONNECTED.\n");
}
#endif

void trigger_baseline_state_machine(int plug_in, int type)
{
	if(!touch_test_dev){
		return;
	}

	if(plug_in == 0 || plug_in == 1){
		if(plug_in ==0){
			dev_info(&touch_test_dev->client->dev, " TA/USB NOT CONNECTED.\n");
			touch_test_dev->charging_mode = 0;
#ifdef TSP_PATCH
			if (!touch_test_dev->suspended) {
				if(touch_test_dev->power_status == MXT_POWER_OFF || touch_test_dev->power_status == MXT_POWER_CFG_DEEPSLEEP){
#ifdef MXT_FACTORY
					if(factorymode){
						dev_info(&touch_test_dev->client->dev, " WAIT FTM BAT_MODE %d\n", MXT_PATCH_FTM_BAT_MODE_EVENT);
						wait_change_cfg = true;
						touch_test_dev->ta_status = MXT_PATCH_FTM_BAT_MODE_EVENT;
					} else
#endif
					{
						dev_info(&touch_test_dev->client->dev, " WAIT BAT_MODE %d\n", MXT_PATCH_BAT_MODE_EVENT);
						wait_change_cfg = true;
						touch_test_dev->ta_status = MXT_PATCH_BAT_MODE_EVENT;
					}
				}else{
#ifdef MXT_FACTORY
					if(factorymode){
						dev_info(&touch_test_dev->client->dev, " FTM BAT_MODE %d\n", MXT_PATCH_FTM_BAT_MODE_EVENT);
						touch_test_dev->ta_status = MXT_PATCH_FTM_BAT_MODE_EVENT;
						mxt_patch_test_event(touch_test_dev, MXT_PATCH_FTM_BAT_MODE_EVENT);
					} else
#endif
					{
						dev_info(&touch_test_dev->client->dev, " BAT_MODE %d\n", MXT_PATCH_BAT_MODE_EVENT);
						touch_test_dev->ta_status = MXT_PATCH_BAT_MODE_EVENT;
						mxt_patch_test_event(touch_test_dev, MXT_PATCH_BAT_MODE_EVENT);
					}
				}
			} else {
				if(touch_test_dev->power_status == MXT_POWER_OFF || touch_test_dev->power_status == MXT_POWER_CFG_DEEPSLEEP){
#ifdef MXT_FACTORY
					if(factorymode){
						dev_info(&touch_test_dev->client->dev, " WAIT FTM_BAT_MODE %d\n", MXT_PATCH_FTM_BAT_MODE_EVENT);
						wait_change_cfg = true;
						touch_test_dev->ta_status = MXT_PATCH_FTM_BAT_MODE_EVENT;
					} else
#endif
					{
						dev_info(&touch_test_dev->client->dev, " WAIT WAKEUP_BAT_MODE %d\n", MXT_PATCH_WAKEUP_BAT_MODE_EVENT);
						wait_change_cfg = true;
						touch_test_dev->ta_status = MXT_PATCH_WAKEUP_BAT_MODE_EVENT;
					}
				}else{
#ifdef MXT_FACTORY
					if(factorymode){
						dev_info(&touch_test_dev->client->dev, " FTM_BAT_MODE %d\n", MXT_PATCH_FTM_BAT_MODE_EVENT);
						touch_test_dev->ta_status = MXT_PATCH_FTM_BAT_MODE_EVENT;
						mxt_patch_test_event(touch_test_dev, MXT_PATCH_FTM_BAT_MODE_EVENT);

					} else
#endif
					{
						dev_info(&touch_test_dev->client->dev, " WAKEUP_BAT_MODE %d\n", MXT_PATCH_WAKEUP_BAT_MODE_EVENT);
						touch_test_dev->ta_status = MXT_PATCH_WAKEUP_BAT_MODE_EVENT;
						mxt_patch_test_event(touch_test_dev, MXT_PATCH_WAKEUP_BAT_MODE_EVENT);
					}
				}
			}
#endif

		}else if(plug_in ==1){
			dev_info(&touch_test_dev->client->dev, " %s CONNECTED.\n", type ? "TA" : "USB");
			touch_test_dev->charging_mode = 1;
#ifdef TSP_PATCH
			if (!touch_test_dev->suspended) {
				if(touch_test_dev->power_status == MXT_POWER_OFF || touch_test_dev->power_status == MXT_POWER_CFG_DEEPSLEEP){
#ifdef MXT_FACTORY
					if(factorymode){
						dev_info(&touch_test_dev->client->dev, " WAIT FTM TA_MODE %d\n", MXT_PATCH_FTM_TA_MODE_EVENT);
						wait_change_cfg = true;
						touch_test_dev->ta_status = MXT_PATCH_FTM_TA_MODE_EVENT;
					} else
#endif
					{
						dev_info(&touch_test_dev->client->dev, " WAIT TA_MODE %d\n", MXT_PATCH_TA_MODE_EVENT);
						wait_change_cfg = true;
						touch_test_dev->ta_status = MXT_PATCH_TA_MODE_EVENT;
					}
				}else{
#ifdef MXT_FACTORY
					if(factorymode){
						dev_info(&touch_test_dev->client->dev, " FTM TA_MODE %d\n", MXT_PATCH_FTM_TA_MODE_EVENT);
						touch_test_dev->ta_status = MXT_PATCH_FTM_TA_MODE_EVENT;
						mxt_patch_test_event(touch_test_dev, MXT_PATCH_FTM_TA_MODE_EVENT);
					} else
#endif
					{
						dev_info(&touch_test_dev->client->dev, " TA_MODE %d\n", MXT_PATCH_TA_MODE_EVENT);
						touch_test_dev->ta_status = MXT_PATCH_TA_MODE_EVENT;
						mxt_patch_test_event(touch_test_dev, MXT_PATCH_TA_MODE_EVENT);
					}
				}
			} else {
				if(touch_test_dev->power_status == MXT_POWER_OFF || touch_test_dev->power_status == MXT_POWER_CFG_DEEPSLEEP){
#ifdef MXT_FACTORY
					if(factorymode){
						dev_info(&touch_test_dev->client->dev, " WAIT FTM TA_MODE %d\n", MXT_PATCH_FTM_TA_MODE_EVENT);
						wait_change_cfg = true;
						touch_test_dev->ta_status = MXT_PATCH_FTM_TA_MODE_EVENT;
					} else
#endif
					{
						dev_info(&touch_test_dev->client->dev, " WAIT WAKEUP_TA_MODE %d\n", MXT_PATCH_WAKEUP_TA_MODE_EVENT);
						wait_change_cfg = true;
						touch_test_dev->ta_status = MXT_PATCH_WAKEUP_TA_MODE_EVENT;
					}
				}else{
#ifdef MXT_FACTORY
					if(factorymode){
						dev_info(&touch_test_dev->client->dev, " WAKEUP FTM TA_MODE %d\n", MXT_PATCH_FTM_TA_MODE_EVENT);
						touch_test_dev->ta_status = MXT_PATCH_FTM_TA_MODE_EVENT;
						mxt_patch_test_event(touch_test_dev, MXT_PATCH_FTM_TA_MODE_EVENT);
					} else
#endif
					{
						dev_info(&touch_test_dev->client->dev, " WAKEUP_TA_MODE %d\n", MXT_PATCH_WAKEUP_TA_MODE_EVENT);
						touch_test_dev->ta_status = MXT_PATCH_WAKEUP_TA_MODE_EVENT;
						mxt_patch_test_event(touch_test_dev, MXT_PATCH_WAKEUP_TA_MODE_EVENT);
					}
				}
			}
#endif
		}
	}
#ifdef MXT_WIRELESS
	else if(plug_in == 3 || plug_in == 4){
		if(plug_in == 3){
			dev_info(&touch_test_dev->client->dev, " WIRELESS TA NOT CONNECTED.\n");
			touch_test_dev->charging_mode = 0;
			wireless = 0;

#ifdef TSP_PATCH
			if (!touch_test_dev->suspended) {
				if(touch_test_dev->power_status == MXT_POWER_OFF || touch_test_dev->power_status == MXT_POWER_CFG_DEEPSLEEP){
					dev_info(&touch_test_dev->client->dev, " WAIT BAT_MODE %d\n", MXT_PATCH_BAT_MODE_EVENT);
					wait_change_cfg = true;
					touch_test_dev->ta_status = MXT_PATCH_BAT_MODE_EVENT;
				}else{
					dev_info(&touch_test_dev->client->dev, " BAT_MODE %d\n", MXT_PATCH_BAT_MODE_EVENT);
					touch_test_dev->ta_status = MXT_PATCH_BAT_MODE_EVENT;
					mxt_patch_test_event(touch_test_dev, MXT_PATCH_BAT_MODE_EVENT);
				}
			} else {
				if(touch_test_dev->power_status == MXT_POWER_OFF || touch_test_dev->power_status == MXT_POWER_CFG_DEEPSLEEP){
					dev_info(&touch_test_dev->client->dev, " WAIT WAKEUP_BAT_MODE %d\n", MXT_PATCH_WAKEUP_BAT_MODE_EVENT);
					wait_change_cfg = true;
					touch_test_dev->ta_status = MXT_PATCH_WAKEUP_BAT_MODE_EVENT;
				}else{
					dev_info(&touch_test_dev->client->dev, " wireless: lpwg_mode: %d, ta_status: %d\n", touch_test_dev->lpwg_mode, touch_test_dev->ta_status);
					if(touch_test_dev->lpwg_mode == LPWG_DOUBLE_TAP){
						dev_info(&touch_test_dev->client->dev, " ~KNOCKON_BAT_MODE %d\n", MXT_PATCH_KNOCKON_BAT_MODE_EVENT);
						touch_test_dev->ta_status = MXT_PATCH_KNOCKON_BAT_MODE_EVENT;
						mxt_patch_test_event(touch_test_dev, MXT_PATCH_KNOCKON_BAT_MODE_EVENT);
					}
					else if(touch_test_dev->lpwg_mode == LPWG_PASSWORD){
						dev_info(&touch_test_dev->client->dev, " ~PASSWORD_BAT_MODE %d\n", MXT_PATCH_PASSWORD_BAT_MODE_EVENT);
						touch_test_dev->ta_status = MXT_PATCH_PASSWORD_BAT_MODE_EVENT;
						mxt_patch_test_event(touch_test_dev, MXT_PATCH_PASSWORD_BAT_MODE_EVENT);
					}
					else{
						dev_info(&touch_test_dev->client->dev, " WAKEUP_BAT_MODE %d\n", MXT_PATCH_WAKEUP_BAT_MODE_EVENT);
						touch_test_dev->ta_status = MXT_PATCH_WAKEUP_BAT_MODE_EVENT;
						mxt_patch_test_event(touch_test_dev, MXT_PATCH_WAKEUP_BAT_MODE_EVENT);
					}
				}
			}
#endif
		} else if(plug_in == 4){
			dev_info(&touch_test_dev->client->dev, " WIRELESS TA CONNECTED.\n");
			touch_test_dev->charging_mode = 1;
			wireless = 1;
#ifdef TSP_PATCH
			if (!touch_test_dev->suspended) {
				if(touch_test_dev->power_status == MXT_POWER_OFF || touch_test_dev->power_status == MXT_POWER_CFG_DEEPSLEEP){
					dev_info(&touch_test_dev->client->dev, " WAIT WIRELESS_TA_MODE %d\n", MXT_PATCH_WIRELESS_TA_MODE_EVENT);
					wait_change_cfg = true;
					touch_test_dev->ta_status = MXT_PATCH_WIRELESS_TA_MODE_EVENT;
				}else{
					dev_info(&touch_test_dev->client->dev, " WIRELESS_TA_MODE %d\n", MXT_PATCH_WIRELESS_TA_MODE_EVENT);
					touch_test_dev->ta_status = MXT_PATCH_WIRELESS_TA_MODE_EVENT;
					mxt_patch_test_event(touch_test_dev, MXT_PATCH_WIRELESS_TA_MODE_EVENT);
				}
			} else {
				if(touch_test_dev->power_status == MXT_POWER_OFF || touch_test_dev->power_status == MXT_POWER_CFG_DEEPSLEEP){
					dev_info(&touch_test_dev->client->dev, " WAIT WAKEUP_WIRELESS_TA_MODE %d\n", MXT_PATCH_WAKEUP_WIRELESS_TA_MODE_EVENT);
					wait_change_cfg = true;
					touch_test_dev->ta_status = MXT_PATCH_WAKEUP_WIRELESS_TA_MODE_EVENT;
				}else{
					dev_info(&touch_test_dev->client->dev, " wireless: lpwg_mode: %d, ta_status: %d\n", touch_test_dev->lpwg_mode, touch_test_dev->ta_status);
					if(touch_test_dev->lpwg_mode == LPWG_DOUBLE_TAP){
						dev_info(&touch_test_dev->client->dev, " ~KNOCKON_TA_MODE %d\n", MXT_PATCH_KNOCKON_TA_MODE_EVENT);
						touch_test_dev->ta_status = MXT_PATCH_KNOCKON_TA_MODE_EVENT;
						mxt_patch_test_event(touch_test_dev, MXT_PATCH_KNOCKON_TA_MODE_EVENT);
					}
					else if(touch_test_dev->lpwg_mode == LPWG_PASSWORD){
						dev_info(&touch_test_dev->client->dev, " ~PASSWORD_TA_MODE %d\n", MXT_PATCH_PASSWORD_TA_MODE_EVENT);
						touch_test_dev->ta_status = MXT_PATCH_PASSWORD_TA_MODE_EVENT;
						mxt_patch_test_event(touch_test_dev, MXT_PATCH_PASSWORD_TA_MODE_EVENT);
					}
					else{
						dev_info(&touch_test_dev->client->dev, " WAKEUP_WIRELESS_TA_MODE %d\n", MXT_PATCH_WAKEUP_WIRELESS_TA_MODE_EVENT);
						touch_test_dev->ta_status = MXT_PATCH_WAKEUP_WIRELESS_TA_MODE_EVENT;
						mxt_patch_test_event(touch_test_dev, MXT_PATCH_WAKEUP_WIRELESS_TA_MODE_EVENT);
					}
				}
			}
#endif
		}
	}
#endif
}

static void mxt_reset_slots(struct mxt_data *data);
static void mxt_proc_t6_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	u8 status = msg[1];
	u32 crc = msg[2] | (msg[3] << 8) | (msg[4] << 16);

	if (crc != data->config_crc) {
		data->config_crc = crc;
		dev_info(dev, "T6 Config Checksum: 0x%06X\n", crc);
		complete(&data->crc_completion);
	}

	/* Detect transition out of reset */
	if ((data->t6_status & MXT_T6_STATUS_RESET) &&
	    !(status & MXT_T6_STATUS_RESET))
		complete(&data->reset_completion);

	/* Output debug if status has changed */
	if (status != data->t6_status 
		|| (status & MXT_T6_STATUS_SIGERR)
		|| (status & MXT_T6_STATUS_CFGERR)
		|| (status & MXT_T6_STATUS_COMSERR)
		)
		dev_info(dev, "T6 Status 0x%02X%s%s%s%s%s%s%s\n",
			status,
			(status == 0) ? " OK" : "",
			(status & MXT_T6_STATUS_RESET) ? " RESET" : "",
			(status & MXT_T6_STATUS_OFL) ? " OFL" : "",
			(status & MXT_T6_STATUS_SIGERR) ? " SIGERR" : "",
			(status & MXT_T6_STATUS_CAL) ? " CAL" : "",
			(status & MXT_T6_STATUS_CFGERR) ? " CFGERR" : "",
			(status & MXT_T6_STATUS_COMSERR) ? " COMSERR" : "");

	/* Save current status */
	data->t6_status = status;

	if(status & MXT_T6_STATUS_CAL){
		dev_dbg(dev, "Calibration Time Stmap!\n");
		do_gettimeofday(&t_ex_debug[TIME_CAL_START]);
	}
	data->anti->fcnt0_msg_cnt = 0;
}

static void mxt_input_button(struct mxt_data *data, u8 *message)
{
	struct input_dev *input = data->input_dev;
	const struct mxt_platform_data *pdata = data->pdata;
	bool button;
	int i;

	/* do not report events if input device not yet registered */
	if (!data->enable_reporting)
		return;

	/* Active-low switch */
	for (i = 0; i < pdata->t19_num_keys; i++) {
		if (pdata->t19_keymap[i] == KEY_RESERVED)
			continue;
		button = !(message[1] & (1 << i));
		input_report_key(input, pdata->t19_keymap[i], button);
	}
}

static void mxt_input_sync(struct input_dev *input_dev)
{
	input_mt_report_pointer_emulation(input_dev, false);
	input_sync(input_dev);
}

static bool chk_time_interval(struct timeval t_aft, struct timeval t_bef, int t_val);
static void mxt_proc_t9_message(struct mxt_data *data, u8 *message)
{
	struct device *dev = &data->client->dev;
	struct input_dev *input_dev = data->input_dev;
	int id;
	u8 status;
	int x;
	int y;
	int area;
	int amplitude;
	u8 vector;
	int tool;

	/* do not report events if input device not yet registered */
	if (!data->enable_reporting)
		return;

	id = message[0] - data->T9_reportid_min;
	status = message[1];
	x = (message[2] << 4) | ((message[4] >> 4) & 0xf);
	y = (message[3] << 4) | ((message[4] & 0xf));

	/* Handle 10/12 bit switching */
	if (data->max_x < 1024)
		x >>= 2;
	if (data->max_y < 1024)
		y >>= 2;

	area = message[5];

	amplitude = message[6];
	vector = message[7];

	dev_dbg(dev,
		"[%u] %c%c%c%c%c%c%c%c x: %5u y: %5u area: %3u amp: %3u vector: %02X\n",
		id,
		(status & MXT_T9_DETECT) ? 'D' : '.',
		(status & MXT_T9_PRESS) ? 'P' : '.',
		(status & MXT_T9_RELEASE) ? 'R' : '.',
		(status & MXT_T9_MOVE) ? 'M' : '.',
		(status & MXT_T9_VECTOR) ? 'V' : '.',
		(status & MXT_T9_AMP) ? 'A' : '.',
		(status & MXT_T9_SUPPRESS) ? 'S' : '.',
		(status & MXT_T9_UNGRIP) ? 'U' : '.',
		x, y, area, amplitude, vector);

	input_mt_slot(input_dev, id);

	if (status & MXT_T9_DETECT) {
		/* Multiple bits may be set if the host is slow to read the
		 * status messages, indicating all the events that have
		 * happened */
		if (status & MXT_T9_RELEASE) {
			input_mt_report_slot_state(input_dev,
						   MT_TOOL_FINGER, 0);
			mxt_input_sync(input_dev);
		}

		/* A reported size of zero indicates that the reported touch
		 * is a stylus from a linked Stylus T47 object. */
		if (area == 0) {
			area = MXT_TOUCH_MAJOR_T47_STYLUS;
			tool = MT_TOOL_PEN;
		} else {
			tool = MT_TOOL_FINGER;
		}

		/* Touch active */
		input_mt_report_slot_state(input_dev, tool, 1);
		input_report_abs(input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
		input_report_abs(input_dev, ABS_MT_PRESSURE, amplitude);
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, area);
		input_report_abs(input_dev, ABS_MT_ORIENTATION, vector);
	} else {
		/* Touch no longer active, close out slot */
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
	}

	data->update_input = true;
}

static struct sys_device lge_touch_sys_device;
#if defined(MXT_GESTURE_RECOGNIZE) || defined(MXT_LPWG)
struct tci_abs {
	int x;
	int y;
	struct timeval time;
};

int g_tap_cnt = 4;
struct tci_abs g_tci_press[MAX_POINT_SIZE_FOR_LPWG];
struct tci_abs g_tci_report[MAX_POINT_SIZE_FOR_LPWG];

char *knockon_event[2] = { "TOUCH_GESTURE_WAKEUP=WAKEUP", NULL };
char *lpwg_event[2] = { "TOUCH_GESTURE_WAKEUP=PASSWORD", NULL };
void send_uevent(char* string[2])
{
	kobject_uevent_env(&lge_touch_sys_device.kobj, KOBJ_CHANGE, string);
	TOUCH_INFO_MSG("uevent[%s]\n", string[0]);
}
#endif	//MXT_GESTURE_RECOGNIZE) || defined(MXT_LPWG)

#ifdef WAITED_UDF

#ifdef TSP_PATCH
static u8 end_config_s[] = { MXT_RESERVED_T255 };

static u8 t93_UDF_off_config[] = {	MXT_T93_NEW, 1,
	0, 0x01,
};

static u8 t100_report_on[] = {	MXT_TOUCH_MULTITOUCHSCREEN_T100, 1,
	0, 0x87,
};

static u8 t100_report_on_resume[] = {	MXT_TOUCH_MULTITOUCHSCREEN_T100, 1,
	0, 0x83,
};

static const u8 *UDF_off_configs_[] = {
	t93_UDF_off_config,
	t100_report_on,
	end_config_s
};

static const u8 *UDF_off_configs_resume[] = {
	t93_UDF_off_config,
	t100_report_on_resume,
	end_config_s
};

static u8 t93_UDF_on_config[] = {	MXT_T93_NEW, 1,
	0, 0x0F,
};

static u8 t100_report_off[] = {	MXT_TOUCH_MULTITOUCHSCREEN_T100, 1,
	0, 0x85,
};

static const u8 *UDF_on_configs_[] = {
	t93_UDF_on_config,
	t100_report_off,
	end_config_s
};
#endif	//TSP_PATCH

#else	//WAITED_UDF
#ifdef TSP_PATCH
static u8 end_config_s[] = { MXT_RESERVED_T255 };

static u8 t93_UDF_off_config[] = {	MXT_T93_NEW, 1,
	0, 0x01,
};

static const u8 *UDF_off_configs_[] = {
	t93_UDF_off_config,
	end_config_s
};

static u8 t93_UDF_on_config[] = {	MXT_T93_NEW, 1,
	0, 0x0F,
};

static const u8 *UDF_on_configs_[] = {
	t93_UDF_on_config,
	end_config_s
};
#endif	//TSP_PATCH
#endif	//WAITED_UDF

#ifdef WAITED_UDF
#define WWAITED_UDF_TIME 200
#define MS_TO_NS(x)	(x * 1E6L)
int write_partial_configs(struct mxt_data *ts, const u8** configs);
static int mxt_set_t7_power_cfg(struct mxt_data *data, u8 sleep);

err_t atmel_ts_lpwg(struct i2c_client* client, u32 code, u32 value, struct point *tci_point);

struct workqueue_struct*    touch_multi_tap_wq;
static enum hrtimer_restart tci_timer_func(struct hrtimer *multi_tap_timer)
{
	struct mxt_data *ts = container_of(multi_tap_timer, struct mxt_data, multi_tap_timer);

	queue_work(touch_multi_tap_wq, &ts->multi_tap_work);

	TOUCH_INFO_MSG("TCI TIMER in \n");
	return HRTIMER_NORESTART;
}

static void touch_multi_tap_work(struct work_struct *multi_tap_work)
{
	struct mxt_data *data = container_of(multi_tap_work, struct mxt_data, multi_tap_work);

	TOUCH_INFO_MSG("TCI WORK in \n");
	data->is_lpwg_report_enable = 1;
	wake_unlock(&touch_wake_lock);
	wake_lock_timeout(&touch_wake_lock, msecs_to_jiffies(3000));
	write_partial_configs(data, UDF_on_configs_);
	TOUCH_INFO_MSG("T93 ENABLE LPWG \n");
	if (data->suspended) {
		send_uevent(lpwg_event);
		mxt_set_t7_power_cfg(data, MXT_POWER_CFG_DEEPSLEEP);
	}
}

static void waited_udf(struct mxt_data *data, u8 *message)
{
	u8 status = message[1];
	if((status & MXT_T100_STATUS_MASK) != MXT_T100_PRESS)
		return ;

	 hrtimer_try_to_cancel(&data->multi_tap_timer);

	//multi tap enable
	TOUCH_INFO_MSG("TCI over tap in \n");

	g_tci_press[0].x = -1;	//for Error state
	g_tci_press[0].y = -1;
	//TOUCH_INFO_MSG("%d tap release x: %d, y: %d \n", i, g_tci_press[0].x, g_tci_press[0].y);

	wake_unlock(&touch_wake_lock);
	wake_lock(&touch_wake_lock);

	if (!hrtimer_callback_running(&data->multi_tap_timer))
		hrtimer_start(&data->multi_tap_timer, ktime_set(0, MS_TO_NS(WWAITED_UDF_TIME)), HRTIMER_MODE_REL);
}
#endif	//WAITED_UDF

static int mxt_t6_command(struct mxt_data *data, u16 cmd_offset, u8 value, bool wait);
static void mxt_reset_slots(struct mxt_data *data);
static void mxt_proc_t100_anti_message(struct mxt_data *data, u8 *message)
{
	struct device *dev = &data->client->dev;
	u8 scr_status;
	u8 num_rpt_touch;

	scr_status = message[1];
	num_rpt_touch = message[2];
	data->anti->inter_area = (message[8] << 8) | message[7];
	data->anti->anti_area = (message[6] << 8) | message[5];
	data->anti->touch_area = (message[4] << 8) | message[3];

	/* release all fingers after touch suppression */
	if(scr_status & MXT_T100_FRIST_ID_SUPPRESSION) {
		dev_info(dev, "T100_message First Report id has SUP!!! %02X Release all, Otherwise Ghost!\n", scr_status);
		mxt_reset_slots(data);
		data->anti->fcnt0_msg_cnt = 0;
		return;
	}

	if(chk_time_interval(t_ex_debug[TIME_CURR_TIME], t_ex_debug[TIME_CAL_START], 5000000)){
		if (scr_status == 0 && num_rpt_touch == 0 && (data->anti->touch_area == 0 || data->anti->touch_area >= 0x30)) {
				if(data->anti->anti_area > 0 || data->anti->inter_area > 0){
					data->anti->fcnt0_msg_cnt++;
					dev_dbg(dev, "Anti #%d scr_status(%d) num_rpt_touch(%d) anti touch area(%d), touch area(%d), internal area(%d)\n",
							data->anti->fcnt0_msg_cnt, scr_status, num_rpt_touch, data->anti->anti_area, data->anti->touch_area, data->anti->inter_area);
					if (data->anti->fcnt0_msg_cnt >= 10) {

						dev_info(dev, "Anti touch fcnt=0 : run Calibration*****************\n");
						mxt_t6_command(data, MXT_COMMAND_CALIBRATE, 1, false);
						mxt_reset_slots(data);

						data->anti->fcnt0_msg_cnt = 0;
						return;
					}
				} else {
					if (data->anti->fcnt0_msg_cnt)
						dev_dbg(dev, "Reset Condition 1. Anti scr_status(%d) num_rpt_touch(%d) anti touch area(%d), touch area(%d), internal area(%d)\n",
							scr_status, num_rpt_touch, data->anti->anti_area, data->anti->touch_area, data->anti->inter_area);
					data->anti->fcnt0_msg_cnt = 0;
				}
		} else {
			if (data->anti->fcnt0_msg_cnt)
				dev_dbg(dev, "Reset Condition 2. Anti scr_status(%d) num_rpt_touch(%d) anti touch area(%d), touch area(%d), internal area(%d)\n",
					scr_status, num_rpt_touch, data->anti->anti_area, data->anti->touch_area, data->anti->inter_area);
			data->anti->fcnt0_msg_cnt = 0;
		}
	}
}

static bool chk_time_interval(struct timeval t_aft, struct timeval t_bef, int t_val)
{
	if( t_aft.tv_sec - t_bef.tv_sec == 0 ) {
		if((get_time_interval(t_aft.tv_usec, t_bef.tv_usec)) <= t_val)
			return true;
	} else if( t_aft.tv_sec - t_bef.tv_sec == 1 ) {
		if( t_aft.tv_usec + 1000000 - t_bef.tv_usec <= t_val)
			return true;
	} else if( t_aft.tv_sec - t_bef.tv_sec == 2 ) {
		if( t_aft.tv_usec + 2000000 - t_bef.tv_usec <= t_val)
			return true;
	} else if( t_aft.tv_sec - t_bef.tv_sec == 3 ) {
		if( t_aft.tv_usec + 3000000 - t_bef.tv_usec <= t_val)
			return true;
	} else if( t_aft.tv_sec - t_bef.tv_sec == 4 ) {
		if( t_aft.tv_usec + 4000000 - t_bef.tv_usec <= t_val)
			return true;
	} else if( t_aft.tv_sec - t_bef.tv_sec == 5 ) {
		if( t_aft.tv_usec + 5000000 - t_bef.tv_usec <= t_val)
			return true;
	}
	return false;
}

static void mxt_proc_t100_message(struct mxt_data *data, u8 *message)
{
	struct device *dev = &data->client->dev;
#ifndef CUST_B_TOUCH
	struct input_dev *input_dev = data->input_dev;
#endif
	int id;
	u8 status;
	int x;
	int y;
	int area;
	int amplitude;
	u8 vector;
	u8 height;
	u8 width;

	/* do not report events if input device not yet registered */
	if (!data->enable_reporting){
		dev_dbg(dev,"return event\n");
		return;
	}

	data->anti->fcnt0_msg_cnt = 0;

	id = message[0] - data->T100_reportid_min - 2;
#ifndef CUST_B_TOUCH
	/* ignore SCRSTATUS events */
	if (id < 0 || id >= data->pdata->numtouch) {
		dev_err(dev, "limited number of finger is %d\n", data->pdata->numtouch);
		return;
	}
	input_mt_slot(input_dev, id);
#endif

	status = message[1];
	x = (message[3] << 8) | message[2];
	y = (message[5] << 8) | message[4];

	vector =  message[data->t100_aux_vect];
	amplitude = message[data->t100_aux_ampl];	/* message[6] */
	area = message[data->t100_aux_area];
	height = message[7];
	width = message[8];

#ifdef CUST_B_TOUCH
	do_gettimeofday(&t_ex_debug[TIME_CURR_TIME]);
	if(chk_time_interval(t_ex_debug[TIME_CURR_TIME], t_ex_debug[TIME_RESUME_END], 42000)){
		dev_info(dev,"Ghost Release  [%6lu sec], [%6lu sec], [%6lu sec]\n", t_ex_debug[TIME_CURR_TIME].tv_sec, t_ex_debug[TIME_RESUME_END].tv_sec, get_time_interval(t_ex_debug[TIME_CURR_TIME].tv_sec, t_ex_debug[TIME_RESUME_END].tv_sec));
		dev_info(dev,"Ghost Release [%6luus], [%6luus], [%6luus]\n", t_ex_debug[TIME_CURR_TIME].tv_usec, t_ex_debug[TIME_RESUME_END].tv_usec, get_time_interval(t_ex_debug[TIME_CURR_TIME].tv_usec, t_ex_debug[TIME_RESUME_END].tv_usec));
		mxt_reset_slots(data);
		return;
	}
#endif

#ifdef T100_AREA_W_H_IN_2BYTES
	if(data->t100_aux_ampl && data->t100_aux_area && data->t100_aux_resv){
		area  = message[7] & MXT_T100_AREA_MASK;
		height= message[8] & MXT_T100_HEIGHT_MASK;
		width = ((message[7] & MXT_T100_WIDTH_MSB_MASK)>>3) | (message[8]>>5);
	}
#endif

#ifndef T100_AREA_REPLACE_AMPLITUDE
	//if(status & (1 << 2))
		dev_dbg(dev, "T100_message[%u] %s%s%s%s%s%s%s%s%s %s%s%s (%02X) x:%u y:%u amp:%u area:%02X vec:%02X\n",
			id,
			((status & MXT_T100_STATUS_MASK) == MXT_T100_MOVE) ? "MOVE" : "",
			((status & MXT_T100_STATUS_MASK) == 2) ? "UNSUP" : "",
			((status & MXT_T100_STATUS_MASK) == 3) ? "SUP" : "",
			((status & MXT_T100_STATUS_MASK) == MXT_T100_PRESS) ? "PRESS" : "",
			((status & MXT_T100_STATUS_MASK) == MXT_T100_RELEASE) ? "RELEASE" : "",
			((status & MXT_T100_STATUS_MASK) == 6) ? "UNSUPSUP" : "",
			((status & MXT_T100_STATUS_MASK) == 7) ? "UNSUPUP" : "",
			((status & MXT_T100_STATUS_MASK) == 8) ? "DOWNSUP" : "",
			((status & MXT_T100_STATUS_MASK) == 9) ? "DOWNUP" : "",
			((status & MXT_T100_TYPE_MASK) == MXT_T100_TYPE_FINGER) ? "FIN" : ".",
			((status & MXT_T100_TYPE_MASK) == MXT_T100_TYPE_STYLUS) ? "PEN" : ".",
			((status & MXT_T100_TYPE_MASK) == MXT_T100_TYPE_PALM) ? "PALM" : ".",
			status, x, y, amplitude, area, vector);
#endif

#ifdef CUST_B_TOUCH
	if (status & MXT_T100_DETECT) {
		/* Multiple bits may be set if the host is slow to read the
		* status messages, indicating all the events that have
	 	* happened */

		if ((status & MXT_T100_STATUS_MASK) == MXT_T100_RELEASE || (status & MXT_T100_STATUS_MASK) == MXT_T100_SUPPRESSION) {
			data->ts_data.curr_data[id].id = id;
			data->ts_data.curr_data[id].status = FINGER_RELEASED;
			if((status & MXT_T100_STATUS_MASK) == MXT_T100_SUPPRESSION)
				dev_info(dev, "T100_message[%u] ###DETECT && SUPPRESSION (%02X)\n", id, status);
		}

#ifdef T100_AREA_REPLACE_AMPLITUDE
		if(data->t100_aux_area){
			/* Z-value is based on area */
			data->ts_data.curr_data[id].id = id;
			data->ts_data.curr_data[id].x_position = x;
			data->ts_data.curr_data[id].y_position = y;
			data->ts_data.curr_data[id].pressure = area;
			data->ts_data.curr_data[id].orientation = vector;
			data->ts_data.curr_data[id].tool = MT_TOOL_FINGER;

			if (height >= width) {
				data->ts_data.curr_data[id].touch_major = height;
				data->ts_data.curr_data[id].touch_minor = width;
			} else {
				data->ts_data.curr_data[id].touch_major = width;
				data->ts_data.curr_data[id].touch_minor = height;
			}

			if ((status & MXT_T100_STATUS_MASK) == MXT_T100_PRESS) {
				data->ts_data.curr_data[id].status = FINGER_PRESSED;
			}else if((status & MXT_T100_STATUS_MASK) == MXT_T100_MOVE){
				data->ts_data.curr_data[id].status = FINGER_MOVED;
			}

			dev_dbg(dev, "%s : curr_data[%d] x(%d), y(%d), area(%d), amplitude(%d)\n",
					__func__, id, x, y, area, amplitude);
			if((status & MXT_T100_TYPE_MASK) == MXT_T100_TYPE_STYLUS){ /* PEN */
				data->ts_data.curr_data[id].is_pen= true;
				data->ts_data.curr_data[id].is_palm= false;
				if(area == 0)
					data->ts_data.curr_data[id].pressure = 4;
				else if(area * 4 > 34)
					data->ts_data.curr_data[id].pressure = 34;
				else
					data->ts_data.curr_data[id].pressure = area * 4;
				if(data->ts_data.curr_data[id].touch_minor > 3)
					data->ts_data.curr_data[id].touch_minor = 3;
			}else if(area >= data->T100_palm_threshold){ /* PALM */
				data->ts_data.curr_data[id].is_pen= false;
				data->ts_data.curr_data[id].is_palm= true;
				data->ts_data.curr_data[id].pressure = 255;
			}else{ /* FINGER */
				data->ts_data.curr_data[id].is_pen= false;
				data->ts_data.curr_data[id].is_palm= false;
				if(area == 0)
					data->ts_data.curr_data[id].pressure = 6;
				else if(area * 6 < 40)
					data->ts_data.curr_data[id].pressure = 40;
				else if(area * 6 > 249)
					data->ts_data.curr_data[id].pressure = 249;
				else
					data->ts_data.curr_data[id].pressure = area * 6;
			}
		} else {
#endif /* T100_AREA_REPLACE_AMPLITUDE */
			/* Z-value is based on amplitude */
			data->ts_data.curr_data[id].id = id;
			data->ts_data.curr_data[id].x_position = x;
			data->ts_data.curr_data[id].y_position = y;
			data->ts_data.curr_data[id].pressure = amplitude;
			data->ts_data.curr_data[id].orientation = vector;
			data->ts_data.curr_data[id].tool = MT_TOOL_FINGER;

			if (height >= width) {
				data->ts_data.curr_data[id].touch_major = height;
				data->ts_data.curr_data[id].touch_minor = width;
			} else {
				data->ts_data.curr_data[id].touch_major = width;
				data->ts_data.curr_data[id].touch_minor = height;
			}

			if ((status & MXT_T100_STATUS_MASK) == MXT_T100_PRESS) {
				data->ts_data.curr_data[id].status = FINGER_PRESSED;
			}else if((status & MXT_T100_STATUS_MASK) == MXT_T100_MOVE){
				data->ts_data.curr_data[id].status = FINGER_MOVED;
			}

			dev_dbg(dev, "%s : curr_data[%d] x(%d), y(%d), area(%d), amplitude(%d)\n",
					__func__, id, x, y, area, amplitude);
			if((status & MXT_T100_TYPE_MASK) == MXT_T100_TYPE_STYLUS){
				data->ts_data.curr_data[id].is_pen= true;
				data->ts_data.curr_data[id].is_palm= false;
			}else if((status & MXT_T100_TYPE_MASK) == MXT_T100_TYPE_PALM){
				data->ts_data.curr_data[id].is_pen= false;
				data->ts_data.curr_data[id].is_palm= true;
			}else{
				data->ts_data.curr_data[id].is_pen= false;
				data->ts_data.curr_data[id].is_palm= false;
			}
#ifdef T100_AREA_REPLACE_AMPLITUDE
		}
#endif
	} else {
		/* Touch Release */
		data->ts_data.curr_data[id].id = id;
		data->ts_data.curr_data[id].status = FINGER_RELEASED;
	}

	dev_dbg(dev, "T100_message[%u] %s%s%s%s%s%s%s%s%s %s%s%s%s%s (0x%02X) x:%u y:%u z:%u area:%u amp:%u vec:%u h:%u w:%u minor:%d\n",
		id,
		((status & MXT_T100_STATUS_MASK) == MXT_T100_MOVE) ? "MOVE" : "",
		((status & MXT_T100_STATUS_MASK) == 2) ? "UNSUP" : "",
		((status & MXT_T100_STATUS_MASK) == 3) ? "SUP" : "",
		((status & MXT_T100_STATUS_MASK) == MXT_T100_PRESS) ? "PRESS" : "",
		((status & MXT_T100_STATUS_MASK) == MXT_T100_RELEASE) ? "RELEASE" : "",
		((status & MXT_T100_STATUS_MASK) == 6) ? "UNSUPSUP" : "",
		((status & MXT_T100_STATUS_MASK) == 7) ? "UNSUPUP" : "",
		((status & MXT_T100_STATUS_MASK) == 8) ? "DOWNSUP" : "",
		((status & MXT_T100_STATUS_MASK) == 9) ? "DOWNUP" : "",
		((status & MXT_T100_TYPE_MASK) == MXT_T100_TYPE_FINGER) ? "FIN" : ".",
		((status & MXT_T100_TYPE_MASK) == MXT_T100_TYPE_STYLUS) ? "PEN" : ".",
		((status & MXT_T100_TYPE_MASK) == MXT_T100_TYPE_PALM) ? "PALM" : ".",
		((status & MXT_T100_TYPE_MASK) == 0x40) ? "HOVER" : ".",
		((status & MXT_T100_TYPE_MASK) == 0x30) ? "ACTSTY" : ".",
		status, x, y, data->ts_data.curr_data[id].pressure, area, amplitude, vector,
		height, width, data->ts_data.curr_data[id].touch_minor);

#else /* CUST_B_TOUCH */
	if (status & MXT_T100_DETECT) {
		/* A reported size of zero indicates that the reported touch
		 * is a stylus from a linked Stylus T47 object. */
		if ((status & MXT_T100_TYPE_MASK) == MXT_T100_TYPE_STYLUS)
			tool = MT_TOOL_PEN;
		else
			tool = MT_TOOL_FINGER;

		/* Touch active */
		input_mt_report_slot_state(input_dev, tool, 1);
		input_report_abs(input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, y);

		if (data->t100_aux_ampl)
			input_report_abs(input_dev, ABS_MT_PRESSURE,
					 message[data->t100_aux_ampl]);

		if (data->t100_aux_area) {
			if (tool == MT_TOOL_PEN)
				input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR,
						 MXT_TOUCH_MAJOR_T47_STYLUS);
			else
				input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR,
						 message[data->t100_aux_area]);
		}

		if (data->t100_aux_vect)
			input_report_abs(input_dev, ABS_MT_ORIENTATION,
					 message[data->t100_aux_vect]);
	} else {
		/* Touch no longer active, close out slot */
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
	}
#endif /* CUST_B_TOUCH */
	data->update_input = true;
}


static void mxt_proc_t15_messages(struct mxt_data *data, u8 *msg)
{
	struct input_dev *input_dev = data->input_dev;
	struct device *dev = &data->client->dev;
	int key;
	bool curr_state, new_state;
	bool sync = false;
	unsigned long keystates = le32_to_cpu(msg[2]);

	/* do not report events if input device not yet registered */
	if (!data->enable_reporting)
		return;

	for (key = 0; key < data->pdata->t15_num_keys; key++) {
		curr_state = test_bit(key, &data->t15_keystatus);
		new_state = test_bit(key, &keystates);

		if (!curr_state && new_state) {
			dev_dbg(dev, "T15 key press: %u\n", key);
			__set_bit(key, &data->t15_keystatus);
			input_event(input_dev, EV_KEY,
				    data->pdata->t15_keymap[key], 1);
			sync = true;
		} else if (curr_state && !new_state) {
			dev_dbg(dev, "T15 key release: %u\n", key);
			__clear_bit(key, &data->t15_keystatus);
			input_event(input_dev, EV_KEY,
				    data->pdata->t15_keymap[key], 0);
			sync = true;
		}
	}

	if (sync)
		input_sync(input_dev);
}

static void mxt_proc_t42_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	u8 status = msg[1];

	if (status & MXT_T42_MSG_TCHSUP)
		dev_info(dev, "T42 suppress\n");
	else
		dev_info(dev, "T42 normal\n");
}

static int mxt_proc_t48_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	u8 status, state;

	status = msg[1];
	state  = msg[4];

	dev_dbg(dev, "T48 state %d status %02X %s%s%s%s%s\n",
			state,
			status,
			(status & 0x01) ? "FREQCHG " : "",
			(status & 0x02) ? "APXCHG " : "",
			(status & 0x04) ? "ALGOERR " : "",
			(status & 0x10) ? "STATCHG " : "",
			(status & 0x20) ? "NLVLCHG " : "");

	return 0;
}

static void mxt_proc_t63_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	struct input_dev *input_dev = data->input_dev;
	u8 id;
	u16 x, y;
	u8 pressure;

	/* do not report events if input device not yet registered */
	if (!data->enable_reporting)
		return;

	/* stylus slots come after touch slots */
	id = data->num_touchids + (msg[0] - data->T63_reportid_min);

	if (id < 0 || id > (data->num_touchids + data->num_stylusids)) {
		dev_err(dev, "invalid stylus id %d, max slot is %d\n",
			id, data->num_stylusids);
		return;
	}

	x = msg[3] | (msg[4] << 8);
	y = msg[5] | (msg[6] << 8);
	pressure = msg[7] & MXT_STYLUS_PRESSURE_MASK;

	dev_dbg(dev,
		"[%d] %c%c%c%c x: %d y: %d pressure: %d stylus:%c%c%c%c\n",
		id,
		(msg[1] & MXT_STYLUS_SUPPRESS) ? 'S' : '.',
		(msg[1] & MXT_STYLUS_MOVE)     ? 'M' : '.',
		(msg[1] & MXT_STYLUS_RELEASE)  ? 'R' : '.',
		(msg[1] & MXT_STYLUS_PRESS)    ? 'P' : '.',
		x, y, pressure,
		(msg[2] & MXT_STYLUS_BARREL) ? 'B' : '.',
		(msg[2] & MXT_STYLUS_ERASER) ? 'E' : '.',
		(msg[2] & MXT_STYLUS_TIP)    ? 'T' : '.',
		(msg[2] & MXT_STYLUS_DETECT) ? 'D' : '.');

	input_mt_slot(input_dev, id);

	if (msg[2] & MXT_STYLUS_DETECT) {
		input_mt_report_slot_state(input_dev, MT_TOOL_PEN, 1);
		input_report_abs(input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
		input_report_abs(input_dev, ABS_MT_PRESSURE, pressure);
	} else {
		input_mt_report_slot_state(input_dev, MT_TOOL_PEN, 0);
	}

	input_report_key(input_dev, BTN_STYLUS, (msg[2] & MXT_STYLUS_ERASER));
	input_report_key(input_dev, BTN_STYLUS2, (msg[2] & MXT_STYLUS_BARREL));

	mxt_input_sync(input_dev);
}

static int mxt_proc_t25_message(struct mxt_data *data, u8 *message)
{
	struct device *dev = &data->client->dev;
	u8 status = message[1];

	if(!selftest_enable)
		return 0;

	dev_info(dev, "T25 Self Test completed %u\n",status);

	if(selftest_show)
		data->self_test_status[0] = status;

	if ( status == 0xFE ) {
		dev_info(dev, "[SUCCESS] All tests passed\n");
		data->self_test_result = true;
	} else {
		if (status == 0xFD) {
			dev_err(dev, "[FAIL] Invalid test code\n");
		} else if (status == 0xFC)  {
			dev_err(dev, "[FAIL] Unrelated fault\n");
		} else if (status == 0x01) {
			dev_err(dev, "[FAIL] AVdd or XVdd is not present\n");
		} else if (status == 0x12) {
			dev_err(dev, "[FAIL] Pin fault (SEQ_NUM %u, X_PIN %u, Y_PIN %u)\n", message[2], message[3], message[4]);
			if(selftest_show){
				data->self_test_status[1] = message[2];
				data->self_test_status[2] = message[3];
				data->self_test_status[3] = message[4];
			}
		} else if (status == 0x17) {
			dev_err(dev, "[FAIL] Signal limit fault (TYPE_NUM %u, TYPE_INSTANCE %u)\n", message[2], message[3]);
			if(selftest_show){
				data->self_test_status[1] = message[2];
				data->self_test_status[2] = message[3];
			}
		} else;
		data->self_test_result = false;
	}

	selftest_enable = false;
	complete(&data->t25_completion);
	return 0;
}

#ifdef MXT_LPWG
#if 1
err_t mxt_proc_t37_message(struct mxt_data *data, u8 *msg_buf)
{
	struct mxt_object *object;
	u8 *buf;
	int i = 0;
	int cnt = 0;
	int tap_num = 0;
	int msg_size = 0;
	int x = 0;
	int y = 0;

/*
	TOUCH_INFO_MSG("t37: %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
		msg_buf[0], msg_buf[1], msg_buf[2], msg_buf[3],
		msg_buf[4], msg_buf[5], msg_buf[6], msg_buf[7], msg_buf[8]);
*/
	object = mxt_get_object(data, MXT_T37);

	if (!object) {
		TOUCH_ERR_MSG("error Cannot get object_type T%d\n", MXT_T37);
		return -EINVAL;
	}
	if ( (mxt_obj_size(object) == 0) || (object->start_address == 0)) {
		TOUCH_ERR_MSG("error object_type T%d\n", object->type);
		return -ENODEV;
	}

	buf = kmalloc(1, GFP_KERNEL);

retry:
	msleep(50);		// to need time to write new data
	DO_IF(__mxt_read_reg(data->client, object->start_address, 1, buf) != 0, error);

	if(buf[0] != 50){
		if(cnt == 5)
			{
			TOUCH_ERR_MSG("cnt = 5, buf[0]= %d\n", buf[0]);
			goto error;
			}
		msleep(20);
		cnt++;
		goto retry;
	}

	DO_IF(__mxt_read_reg(data->client, object->start_address + 2, 1, buf) != 0, error);

	tap_num = buf[0];
	if (g_tap_cnt != tap_num) {
		DO_IF(__mxt_read_reg(data->client, data->T93_address + 17, 1, buf) != 0, error);
		TOUCH_ERR_MSG("ERROR tap number T93[17]=%d T37[2]=%d\n", buf[0], tap_num);
		if(tap_num == 0)
			tap_num = g_tap_cnt;
	} else {
		g_tap_cnt = tap_num;
	}
	TOUCH_INFO_MSG("t37[2] tap number is %d\n",	tap_num);

	kfree(buf);
#ifdef ALPHA_FW
	msg_size = tap_num * 9 ; 	// 1tap has 10 byte data : press 4byte, release 4byte, reserve 1byte, dummy 1byte (EB 9byte, EE 10byte)
	buf = kmalloc(msg_size, GFP_KERNEL);
	DO_IF(__mxt_read_reg(data->client, object->start_address + 3, msg_size, buf) != 0, error);
#else
	msg_size = tap_num * 10 ; 	// 1tap has 10 byte data : press 4byte, release 4byte, reserve 1byte, dummy 1byte (EB 9byte, EE 10byte)
	buf = kmalloc(msg_size, GFP_KERNEL);
	DO_IF(__mxt_read_reg(data->client, object->start_address + 4, msg_size, buf) != 0, error);
#endif

	for (i = 0; i < tap_num ; i++)
	{
#ifdef ALPHA_FW
		cnt = i * 9;
#else
		cnt = i * 10;
#endif
		x = (buf[cnt + 1] << 8) | buf[cnt];
		y = (buf[cnt + 3] << 8) | buf[cnt + 2];
		g_tci_press[i].x = x;
		g_tci_press[i].y = y;
		TOUCH_INFO_MSG("%d tap press x: %5u, y: %5u (%d)\n", i, x, y, cnt);

		x = (buf[cnt + 5] << 8) | buf[cnt + 4];
		y = (buf[cnt + 7] << 8) | buf[cnt + 6];
		g_tci_report[i].x = x;
		g_tci_report[i].y = y;
		TOUCH_INFO_MSG("%d tap release x: %5u, y: %5u (%d)\n", i, x, y, cnt);
	}

	kfree(buf);
	return IGNORE_EVENT;
error:
	TOUCH_ERR_MSG("T37 error\n");
	kfree(buf);
	return IGNORE_EVENT;
}
#else
err_t mxt_proc_t37_message(struct mxt_data *data, u8 *msg_buf)
{
	struct mxt_object *object;
	u8 *buf;
	int i = 0;
	int cnt = 0;
	int tap_num = 0;
	int msg_size = 0;
	int x = 0;
	int y = 0;

	TOUCH_INFO_MSG("t37: %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
		msg_buf[0], msg_buf[1], msg_buf[2], msg_buf[3],
		msg_buf[4], msg_buf[5], msg_buf[6], msg_buf[7], msg_buf[8]);

	object = mxt_get_object(data, MXT_T37);

	if (!object) {
		TOUCH_ERR_MSG("error Cannot get object_type T%d\n", MXT_T37);
		return -EINVAL;
	}
	if ( (mxt_obj_size(object) == 0) || (object->start_address == 0)) {
		TOUCH_ERR_MSG("error object_type T%d\n", object->type);
		return -ENODEV;
	}

	buf = kmalloc(1, GFP_KERNEL);

retry:
	msleep(50);		// to need time to write new data
	DO_IF(__mxt_read_reg(data->client, object->start_address, 1, buf) != 0, error);

	if(buf[0] != 50){
		if(cnt == 5)
			{
			TOUCH_ERR_MSG("cnt = 5\n");
			goto error;
			}
		msleep(20);
		cnt++;
		goto retry;
	}

	DO_IF(__mxt_read_reg(data->client, object->start_address + 2, 1, buf) != 0, error);

	TOUCH_INFO_MSG("t37[2] tap number is %d\n",	buf[0]);
	tap_num = buf[0];
	g_tap_cnt = tap_num;

	kfree(buf);
	msg_size = tap_num * 9 ; 	// 1tap has 9 byte data : press 4byte, release 4byte, reserve 1byte
	buf = kmalloc(msg_size, GFP_KERNEL);
	DO_IF(__mxt_read_reg(data->client, object->start_address + 3, msg_size, buf) != 0, error);

	for (i = 0; i < tap_num ; i++)
	{
		cnt = i * 9;
		x = (buf[cnt + 1] << 8) | buf[cnt];
		y = (buf[cnt + 3] << 8) | buf[cnt + 2];
		g_tci_press[i].x = x;
		g_tci_press[i].y = y;
		TOUCH_INFO_MSG("%d tap press x: %5u, y: %5u (%d)\n", i, x, y, cnt);

		x = (buf[cnt + 5] << 8) | buf[cnt + 4];
		y = (buf[cnt + 7] << 8) | buf[cnt + 6];
		g_tci_report[i].x = x;
		g_tci_report[i].y = y;
		TOUCH_INFO_MSG("%d tap release x: %5u, y: %5u (%d)\n", i, x, y, cnt);
	}

	kfree(buf);
	return IGNORE_EVENT;
error:
	TOUCH_ERR_MSG("T37 error\n");
	kfree(buf);
	return IGNORE_EVENT;
}
#endif

#endif
int write_partial_configs(struct mxt_data *ts, const u8** configs);
static void mxt_proc_t35_messages(struct mxt_data *data, u8 *message)
{
	struct device *dev = &data->client->dev;
	u8 msg;

	if (data->in_bootloader)
		return;

	msg = message[1];

	if(data->mxt_character_enable && msg == 0x43){	/*C*/
		dev_info(dev, "character C recognize.\n");
	}else if(data->mxt_character_enable && msg == 0x4D){	/*M*/
		dev_info(dev, "character M recognize.\n");
	}else if(data->mxt_character_enable && msg == 0x57){	/*W*/
		dev_info(dev, "character W recognize.\n");
	}else if(data->mxt_volume_enable && msg == 0x83){	/*Vol Up*/
		dev_info(dev, "Volume Up recognize.\n");
	}else if(data->mxt_volume_enable && msg == 0x84){	/*Vol Down*/
		dev_info(dev, "Volume Down recognize.\n");
	}else{
		dev_info(dev, "Unknown pattern recognize %d 0x%x\n", msg, msg);
#ifdef MXT_LPWG
		if(!data->is_lpwg_report_enable){
			dev_err(dev, "lpwg reply is not coming\n");
		}
		mxt_t6_command(data, MXT_COMMAND_DIAGNOSTIC, 50, false);
		mxt_proc_t37_message(data, message);
		data->is_lpwg_report_enable = 0;
		wake_lock_timeout(&touch_wake_lock, msecs_to_jiffies(3000));
		send_uevent(lpwg_event);
		write_partial_configs(data, UDF_off_configs_);
		TOUCH_INFO_MSG("T35 DISABLE\n");
#endif
	}
}

#ifdef MXT_LPWG
#ifdef UDF_CONTROL_CLEAR_T37_DATA
#define T93_CTRL_ARMCMD		(1 << 7)
#define T93_CTRL_AUTOARMEN		0xF7
#define T93_CTRL_DISABLE_AUTOARMEN	(1 << 3)
int set_t93_sequence_arm(struct mxt_data *data, u8 value)
{
	struct mxt_object *object;
	u8 buf;

	object = mxt_get_object(data, MXT_T93_NEW);
	if (!object) {
		TOUCH_ERR_MSG("error Cannot get object_type T%d\n", MXT_T93_NEW);
		return -EINVAL;
	}
	if ( (mxt_obj_size(object) == 0) || (object->start_address == 0)) {
		TOUCH_ERR_MSG("error object_type T%d\n", object->type);
		return -ENODEV;
	}

	DO_IF(__mxt_read_reg(data->client, object->start_address, 1, &buf), error);
	TOUCH_INFO_MSG("set_t93_sequence_arm read T%d %d %x\n", MXT_T93_NEW, 0, buf);

	if (value) //new protocol
		buf = (buf & T93_CTRL_AUTOARMEN) | T93_CTRL_ARMCMD;
	else //previous protocol
		buf = buf | T93_CTRL_DISABLE_AUTOARMEN;

	DO_IF(__mxt_write_reg(data->client, object->start_address, 1, &buf), error);
	TOUCH_INFO_MSG("set_t93_sequence_arm write T%d %d %x\n", MXT_T93_NEW, 0, buf);

	return NO_ERROR;

error:
	return ERROR;
}
#endif

static void mxt_proc_t93_messages(struct mxt_data *data, u8 *message)
{
	struct device *dev = &data->client->dev;
	u8 msg;

	if (data->in_bootloader)
		return;

	msg = message[1];

	dev_info(dev, "T93 %u \n",msg);

	if( msg & 0x01){
		mutex_lock(&data->input_dev->mutex);
		mxt_t6_command(data, MXT_COMMAND_DIAGNOSTIC, 50, false);
		mxt_proc_t37_message(data, message);
#ifdef UDF_CONTROL_CLEAR_T37_DATA
		set_t93_sequence_arm(data, 0);
#endif
		data->is_lpwg_report_enable = 0;
		wake_lock_timeout(&touch_wake_lock, msecs_to_jiffies(3000));
		write_partial_configs(data, UDF_off_configs_);
		//mxt_t6_command(data, MXT_COMMAND_CALIBRATE, 1, false);
		TOUCH_INFO_MSG("T93 DISABLE\n");
#ifdef WAITED_UDF
		//timer on
		hrtimer_try_to_cancel(&data->multi_tap_timer);
		if (!hrtimer_callback_running(&data->multi_tap_timer))
			hrtimer_start(&data->multi_tap_timer, ktime_set(0, MS_TO_NS(WWAITED_UDF_TIME)), HRTIMER_MODE_REL);
#else
		send_uevent(lpwg_event);
#endif
		mutex_unlock(&data->input_dev->mutex);
	}
}
#endif
static void mxt_proc_t24_messages(struct mxt_data *data, u8 *message)
{
	struct device *dev = &data->client->dev;
	u8 msg;
	int x;
	int y;

	if (data->in_bootloader)
		return;

	msg = message[1];


	x = (message[2] << 4) | ((message[4] >> 4) & 0xf);
	y = (message[3] << 4) | ((message[4] & 0xf));

	if(msg == 0x04) {
		if(quick_cover_status == 1){
#ifdef MXT_LPWG
			dev_dbg(dev, "QuickCoverSize X %d-%d  Y %d-%d \n",
				data->qwindow_size->x_min, data->qwindow_size->x_max,
				data->qwindow_size->y_min, data->qwindow_size->y_max);	/* QuickCoverSize X 216-863  Y 17-922 */

			if(!((x > data->qwindow_size->x_min ) && (x < data->qwindow_size->x_max)
				&& (y > data->qwindow_size->y_min) && (y < data->qwindow_size->y_max)))	// resolution : 1080 * 1920
#else
			if(!((x > 440 ) && (x < 1760) && (y > 80) && (y < 1780)))
#endif
			{
				dev_info(dev, "Out Of Quick Window Boundary Double_Tap!!     %d     %d \n",x,y);
				wake_unlock(&touch_wake_lock);
				return;
			}
		}else{
#ifdef MXT_LPWG
			if(!((x > 110 ) && (x < 970) && (y > 135) && (y < 1785)))	// resolution : 1080 * 1920
#else
			if(!((x > 220 ) && (x < 1940) && (y > 270) && (y < 3570)))
#endif
			{
				dev_info(dev, "Out Of Boundary Double_Tap!!     %d     %d \n",x,y);
				wake_unlock(&touch_wake_lock);
				return;
			}
		}
		dev_info(dev, "Double_Tap!!     %d     %d \n",x,y);
		send_uevent(knockon_event);
	}
}

static int mxt_proc_message(struct mxt_data *data, u8 *message)
{
	u8 object;
	u8 report_id = message[0];
	bool dump = data->debug_enabled;

	if (report_id == MXT_RPTID_NOMSG)
		return 0;

	object = REPORT_ID_TO_OBJECT(report_id, data);
	dev_dbg(&data->client->dev, "object %d\n", object);

	do_gettimeofday(&t_ex_debug[TIME_CURR_TIME]);

	switch(object){
		case MXT_GEN_COMMAND_T6:
			mxt_proc_t6_messages(data, message);
			break;
		case MXT_TOUCH_MULTI_T9:
			mxt_proc_t9_message(data, message);
			break;
		case MXT_TOUCH_MULTITOUCHSCREEN_T100:
			if(report_id == data->T100_reportid_min){
				mxt_proc_t100_anti_message(data, message);
			} else {
#ifdef WAITED_UDF
				if (data->mxt_password_enable && data->suspended && !data->is_lpwg_report_enable){
					waited_udf(data, message);
					return 1;
				}
				else
#endif
				mxt_proc_t100_message(data, message);
			}
			break;
		case MXT_SPT_GPIOPWM_T19:
			mxt_input_button(data, message);
			data->update_input = true;
			break;
		case MXT_PROCI_ACTIVE_STYLUS_T63:
			mxt_proc_t63_messages(data, message);
			break;
		case MXT_PROCI_TOUCHSUPPRESSION_T42:
			mxt_proc_t42_messages(data, message);
			break;
		case MXT_PROCG_NOISESUPPRESSION_T48:
			mxt_proc_t48_messages(data, message);
			break;
		case MXT_TOUCH_KEYARRAY_T15:
			mxt_proc_t15_messages(data, message);
			break;
#ifdef MXT_GESTURE_RECOGNIZE
		case MXT_PROCI_ONETOUCH_T24:
#ifdef MXT_LPWG
			if(!data->mxt_password_enable && data->mxt_knock_on_enable)
#else
			if (data->mxt_knock_on_enable && data->suspended)
#endif
			{
				wake_lock_timeout(&touch_wake_lock, msecs_to_jiffies(1000));
				mxt_proc_t24_messages(data, message);
			}
			break;
		case MXT_SPT_PROTOTYPE_T35:
#ifdef MXT_LPWG
			if (data->mxt_password_enable)
#else
			if (data->suspended)
#endif
			{
#ifdef MXT_LPWG
				if(!data->mxt_password_enable)
					return 1;
#endif
			mxt_proc_t35_messages(data, message);
			}
			break;
#endif
		case MXT_SPT_SELFTEST_T25:
			mxt_proc_t25_message(data, message);
			break;
#ifdef MXT_LPWG
		case MXT_T93_NEW:
			if(!mxt_mfts)
				mxt_proc_t93_messages(data, message);
			break;
#endif
		default:
			/* dump = true; */
			break;
	}

	if (dump)
		mxt_dump_message(data, message);

#ifdef TSP_PATCH
	{
		struct mxt_message stMsg;
		stMsg.reportid = report_id;
		memcpy(stMsg.message, &message[1], 8);	
		mxt_patch_message(data, &stMsg);
	}	
#endif
	return 1;
}

static int mxt_read_and_process_messages(struct mxt_data *data, u8 count)
{
	struct device *dev = &data->client->dev;
	int ret;
	int i;
	u8 num_valid = 0;

	/* Safety check for msg_buf */
	if (count > data->max_reportid)
		return -EINVAL;

	/* Process remaining messages if necessary */
	ret = __mxt_read_reg(data->client, data->T5_address,
				data->T5_msg_size * count, data->msg_buf);
	if (ret) {
		dev_err(dev, "Failed to read %u messages (%d)\n", count, ret);
		return ret;
	}

	for (i = 0;  i < count; i++) {
		ret = mxt_proc_message(data,
			data->msg_buf + data->T5_msg_size * i);

		if (ret == 1)
			num_valid++;
	}

	/* return number of messages read */
	return num_valid;
}

#ifdef CUST_B_TOUCH
// LGE_CHANGE_S [naomi.kim@lge.com] 13.06.18, make width minor data
#if TOUCHEVENTFILTER
int set_minor_data(struct mxt_data *data, int area, u8 vector)
{
	struct device *dev = &data->client->dev;

	u8 tmp;
	int component1;
	int component2;
	int magnitude = 0;
	int minor = 0;

	int i;

	/* 1. get componet data */
	// component1
	tmp = ( (vector >> 4) & 0xf);
	if( tmp & 0x8 )
		component1 = (int)(((~(tmp) ) & 0xf) | 0x1 );
	else
		component1 = tmp;

	// component2
	tmp = (vector & 0xf);
	if( tmp & 0x8 )
		component2 = (int)(((~(tmp) ) & 0xf) | 0x1 );
	else
		component2 = tmp;

	/* 2. magnitude = SQRT(component1^2 + component2^2) */
	tmp = (component1 * component1) + (component2 * component2);

	/* 3. make minor date
	// when the magnitude is same, the larger area has the longer minor value.
	// that means, when the area is same, the longer magnitude has the shorter minor value.
	// 3-1. if shape is circle, minor = major.
	// when the shape is circle, vector = 0x00 / 0x01 / 0x10 / 0x11
	// then magnitude^2 = 0 /1 / 1 / 2
	*/
	if ( tmp < 3 )
	{
		minor = area;
		magnitude = 1;
	}
	else {
	/* 3-2. if shape is elipse, minor = area / magnitude.	*/
	// find SQRT(magnitude^2)
		for( i = 9 ; i > 1 ; i--) {
			if ( tmp > ((i*i) - 1) ){
				magnitude = i;
				break;
			}
		}
		minor = area / magnitude;
	}

	dev_dbg(dev,
		"%5u area: %5u minor: %5u magnitude: %5u vector: %5u component1: %5u component2: %5u \n",
			tmp, area, minor, magnitude, vector, component1, component2);

	return minor;
}
#endif
// LGE_CHANGE_E [naomi.kim@lge.com] 13.06.18, make width minor data

static char* get_tool_type(struct mxt_data *data, struct t_data touch_data) {
	if (touch_data.tool == MT_TOOL_FINGER) {
		if (touch_data.is_pen) {
			return "PEN";
		} else if(touch_data.is_palm){
			return "PALM";
		} else {
			return "FINGER";
		}
	} else {
		dev_err(&data->client->dev, "Invalid tool type : %d", touch_data.tool);
	}
	return "Unknown";
}
#endif

static u16 gap_of_position(u16 pos_aft, u16 pos_bef)
{
	u16 result;

	result = (pos_aft - pos_bef < 0) ? (pos_bef - pos_aft) : (pos_aft - pos_bef);
	return result;
}

static void mxt_process_messages_t44(struct work_struct *work)
{
	struct mxt_data *data =
			container_of(work, struct mxt_data, work);
	struct device *dev = &data->client->dev;
	int ret;
	u8 count, num_left;
#ifdef CUST_B_TOUCH
	int report_num = 0;
	char *tool_type;
	int i;
#endif

	/* Read T44 and T5 together */
	ret = __mxt_read_reg(data->client, data->T44_address,
		data->T5_msg_size + 1, data->msg_buf);

	if (ret) {
		dev_err(dev, "Failed to read T44 and T5 (%d)\n", ret);
		goto err_out_critical;
	}

	count = data->msg_buf[0];

	if (count == 0) {
		dev_dbg(dev, "Interrupt triggered but zero messages\n");
		goto out_ignore_interrupt;
	} else if (count > data->max_reportid) {
		dev_err(dev, "T44 count %d exceeded max report id\n", count);
		count = data->max_reportid;
	}
#ifdef CUST_B_TOUCH
	data->ts_data.total_num = 0;
#endif

  mutex_lock(&mxt_early_mutex);
	/* Process first message */
	ret = mxt_proc_message(data, data->msg_buf + 1);
	if (ret < 0) {
		dev_warn(dev, "Unexpected invalid message\n");
		mutex_unlock(&mxt_early_mutex);
		goto out_ignore_interrupt;
	}

	num_left = count - 1;

	/* Process remaining messages if necessary */
	if (num_left) {
		ret = mxt_read_and_process_messages(data, num_left);
		if (ret < 0)
			goto end;
		else if (ret != num_left)
			dev_warn(dev, "Unexpected invalid message\n");
	}
#ifdef CUST_B_TOUCH
	for (i = 0; i < data->pdata->numtouch; i++) {

		if (data->ts_data.curr_data[i].status == FINGER_INACTIVE &&
			data->ts_data.prev_data[i].status != FINGER_INACTIVE &&
			data->ts_data.prev_data[i].status != FINGER_RELEASED) {
			memcpy(&data->ts_data.curr_data[i], &data->ts_data.prev_data[i], sizeof(data->ts_data.prev_data[i]));
			data->ts_data.curr_data[i].skip_report = true;
		}else if (data->ts_data.curr_data[i].status == FINGER_INACTIVE) {
			continue;
		}

		if (data->ts_data.curr_data[i].status == FINGER_PRESSED ||
			data->ts_data.curr_data[i].status == FINGER_MOVED) {
			data->ts_data.total_num++;
		}
		report_num++;
	}

#if defined(CONFIG_TOUCHSCREEN_ATMEL_S540)
	if(data->ts_data.total_num) {
		static struct t_data pen_data[MXT_MAX_NUM_TOUCHES];
		static struct timeval t_pen_start[MXT_MAX_NUM_TOUCHES];
		static struct timeval t_pen_prev[MXT_MAX_NUM_TOUCHES];
		for (i = 0; i < MXT_MAX_NUM_TOUCHES ; i++) {
			if (data->ts_data.curr_data[i].is_pen) {
				if (data->ts_data.curr_data[i].status == FINGER_PRESSED || data->ts_data.curr_data[i].status == FINGER_MOVED) {
					if(!pen_data[i].is_pen) {
						//first detection
						do_gettimeofday(&t_pen_start[i]);
						memcpy(&t_pen_prev[i], &t_pen_start[i], sizeof(t_pen_prev[i]));
						memcpy(&pen_data[i], &data->ts_data.curr_data[i], sizeof(pen_data[i]));
						dev_dbg(&data->client->dev, "[Ghost]pen_ghost start <%d> x:%u y:%u s:%d\n", i,
							data->ts_data.curr_data[i].x_position, data->ts_data.curr_data[i].y_position, data->ts_data.curr_data[i].status);
					} else {
						//in 50ms //x,y in 50
						do_gettimeofday(&t_ex_debug[TIME_CURR_TIME]);
						if( chk_time_interval(t_ex_debug[TIME_CURR_TIME], t_pen_prev[i], 50000)
							&& gap_of_position(pen_data[i].x_position, data->ts_data.curr_data[i].x_position) <=50
							&& gap_of_position(pen_data[i].y_position, data->ts_data.curr_data[i].y_position) <=50
						) {
							dev_dbg(&data->client->dev, "[Ghost]pen_ghost stuck? <%d> x:%u y:%u s:%d\n", i,
								data->ts_data.curr_data[i].x_position, data->ts_data.curr_data[i].y_position, data->ts_data.curr_data[i].status);
							memcpy(&t_pen_prev[i], &t_ex_debug[TIME_CURR_TIME], sizeof(t_pen_prev[i]));
							//for 5 secs
							if( t_ex_debug[TIME_CURR_TIME].tv_sec - t_pen_start[i].tv_sec >= 5){
								dev_info(&data->client->dev, "[Ghost]pen_ghost run calibration! <%d> x:%u y:%u s:%d\n", i,
									data->ts_data.curr_data[i].x_position, data->ts_data.curr_data[i].y_position, data->ts_data.curr_data[i].status);
								mxt_t6_command(data, MXT_COMMAND_CALIBRATE, 1, false);
								mxt_reset_slots(data);
								//init
								pen_data[i].is_pen = false;
								goto out;
							}
						}
						else {
							//init
							pen_data[i].is_pen = false;
							dev_dbg(&data->client->dev,"[Ghost]pen_ghost init, not in Condition <%d> x:%u y:%u\n", i,
									data->ts_data.curr_data[i].x_position, data->ts_data.curr_data[i].y_position);
						}
					}
				} else {
					if(pen_data[i].is_pen){
						//init
						pen_data[i].is_pen = false;
						dev_dbg(&data->client->dev,"[Ghost]pen_ghost init, Release <%d>\n", i);
					}
				}
			} else {
				if(pen_data[i].is_pen){
					//init
					pen_data[i].is_pen = false;
					dev_dbg(&data->client->dev,"[Ghost]pen_ghost init, Finger <%d>\n", i);
				}
			}
		}
	}
#endif

	if (!data->enable_reporting || !report_num)
		goto out;

	for (i = 0; i < data->pdata->numtouch; i++) {
		if (data->ts_data.curr_data[i].status == FINGER_INACTIVE || data->ts_data.curr_data[i].skip_report) {
			continue;
		}
		if (data->ts_data.curr_data[i].status == FINGER_RELEASED && data->ts_data.prev_data[i].status != FINGER_RELEASED) {
			input_mt_slot(data->input_dev, data->ts_data.curr_data[i].id);
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
		}  else {
#ifdef MXT_LPWG
		dev_dbg(dev, "quick_cover_status %d  use_quick_window %d hall ic %d\n",
							quick_cover_status, use_quick_window, cradle_smart_cover_status());
			if (quick_cover_status || (use_quick_window && (cradle_smart_cover_status() == 1))){	/* drop touch event (out of quick window) */
				if(!((data->ts_data.curr_data[i].x_position > 432)
					&& (data->ts_data.curr_data[i].x_position < 1726)
					&& (data->ts_data.curr_data[i].y_position > 34)
					&& (data->ts_data.curr_data[i].y_position < 1844))){
						dev_dbg(dev, "out of quick window!! %d		%d\n",
							data->ts_data.curr_data[i].x_position, data->ts_data.curr_data[i].y_position);
						if (data->ts_data.curr_data[i].status == FINGER_PRESSED ||
							data->ts_data.curr_data[i].status == FINGER_MOVED) {
							data->ts_data.total_num--;
						}
						data->ts_data.curr_data[i].status = FINGER_RELEASED;
						outside_touch = 1;
				}
			}
#endif
#ifdef MXT_TA_TIME
			if(touch_probe_ok)
			{
				do_gettimeofday(&t_ex_debug[TIME_CURR_TIME]);
				if(data->ts_data.curr_data[i].status == FINGER_PRESSED || data->ts_data.curr_data[i].status == FINGER_MOVED){
					if(chk_time_interval(t_ex_debug[TIME_CURR_TIME], t_ex_debug[TIME_TA_DETECT], 1000000)){
						dev_warn(dev, "In 1 sec TA Timing Condition. Change Finger id %d Stautus to Release\n",i);
						data->ts_data.curr_data[i].status = FINGER_RELEASED;
						data->ts_data.total_num--;
					}
				}
			}
#endif
			if(data->ts_data.curr_data[i].status == FINGER_RELEASED){
				input_mt_slot(data->input_dev, data->ts_data.curr_data[i].id);
				input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
			} else {
				input_mt_slot(data->input_dev, data->ts_data.curr_data[i].id);
				input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 1);
				input_report_abs(data->input_dev, ABS_MT_TRACKING_ID,
					data->ts_data.curr_data[i].id);
				input_report_abs(data->input_dev, ABS_MT_POSITION_X,
					data->ts_data.curr_data[i].x_position);
				input_report_abs(data->input_dev, ABS_MT_POSITION_Y,
					data->ts_data.curr_data[i].y_position);

				/* Report Palm event : if touch event is palm, report pressure 255 to framework */
				if(data->ts_data.curr_data[i].is_palm)
					input_report_abs(data->input_dev, ABS_MT_PRESSURE, 255);
				else if(data->ts_data.curr_data[i].pressure == 255)
					input_report_abs(data->input_dev, ABS_MT_PRESSURE, 254);
				else
					input_report_abs(data->input_dev, ABS_MT_PRESSURE,
						data->ts_data.curr_data[i].pressure);
				/* Report Palm event end */

				input_report_abs(data->input_dev, ABS_MT_ORIENTATION,
								data->ts_data.curr_data[i].orientation);
				input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR,
					data->ts_data.curr_data[i].touch_major);
				input_report_abs(data->input_dev, ABS_MT_WIDTH_MINOR,
					data->ts_data.curr_data[i].touch_minor);

			// LGE_CHANGE_S [naomi.kim@lge.com] 13.06.18, add more debugging data
				#if TOUCHEVENTFILTER
				dev_dbg(dev,
					"report_data[%d] : x: %d y: %d, z: %d, M: %d, m: %d, orient: %d)\n",
						data->ts_data.curr_data[i].id,
						data->ts_data.curr_data[i].x_position,
						data->ts_data.curr_data[i].y_position,
						data->ts_data.curr_data[i].pressure,
						data->ts_data.curr_data[i].touch_major,
						data->ts_data.curr_data[i].touch_minor,
						data->ts_data.curr_data[i].orientation
				);
				#else
				dev_dbg(dev, "report_data[%d] : (x %d, y %d, presure %d, touch_major %d, orient %d)\n",
						i,
						data->ts_data.curr_data[i].x_position,
						data->ts_data.curr_data[i].y_position,
						data->ts_data.curr_data[i].pressure,
						data->ts_data.curr_data[i].touch_major,
						data->ts_data.curr_data[i].orientation
				);
				#endif
				// LGE_CHANGE_E [naomi.kim@lge.com] 13.06.18, add more debugging data
			}
		}
#if DEBUG_ABS
		if (data->ts_data.curr_data[i].status == FINGER_PRESSED) {
			tool_type = get_tool_type(data, data->ts_data.curr_data[i]);
			dev_info(dev, "%d %s Pressed <%d> : x[%4d] y[%4d], z[%3d]\n",
					data->ts_data.total_num, tool_type,
					data->ts_data.curr_data[i].id,
					data->ts_data.curr_data[i].x_position,
					data->ts_data.curr_data[i].y_position,
					data->ts_data.curr_data[i].pressure);
		} else if (data->ts_data.curr_data[i].status == FINGER_RELEASED
						&& data->ts_data.prev_data[i].status != FINGER_RELEASED) {
			if(likely(!outside_touch)){
				tool_type = get_tool_type(data, data->ts_data.prev_data[i]);
				dev_info(dev, "%s Released <%d> <%d P>\n",
						tool_type,
						data->ts_data.curr_data[i].id, data->ts_data.total_num);
			}
			outside_touch = 0;
		}
#endif
	}

	if(data->ts_data.total_num < data->ts_data.prev_total_num)
		dev_dbg(dev, "Total_num(move+press)= %d\n",data->ts_data.total_num);
	if (data->ts_data.total_num) {
		data->ts_data.prev_total_num = data->ts_data.total_num;
		memcpy(data->ts_data.prev_data, data->ts_data.curr_data, sizeof(data->ts_data.curr_data));
	} else{
		data->ts_data.prev_total_num = 0;
		memset(data->ts_data.prev_data, 0, sizeof(data->ts_data.prev_data));
	}
	memset(data->ts_data.curr_data, 0, sizeof(data->ts_data.curr_data));
#endif


end:
	if (data->update_input) {
		mxt_input_sync(data->input_dev);
		data->update_input = false;
	}

#ifdef CUST_B_TOUCH
out:
    mutex_unlock(&mxt_early_mutex);
	return;
#endif
out_ignore_interrupt:
	return;
err_out_critical:
	safety_reset(data);
	return;
}

static int mxt_process_messages_until_invalid(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int count, read;
	u8 tries = 2;

	count = data->max_reportid;

	/* Read messages until we force an invalid */
	do {
		read = mxt_read_and_process_messages(data, count);
		if (read < count)
			return 0;
	} while (--tries);

	if (data->update_input) {
		mxt_input_sync(data->input_dev);
		data->update_input = false;
	}

	dev_err(dev, "CHG pin isn't cleared\n");
	return -EBUSY;
}

static irqreturn_t mxt_process_messages(struct mxt_data *data)
{
	int total_handled, num_handled;
	u8 count = data->last_message_count;

	if (count < 1 || count > data->max_reportid)
		count = 1;

	/* include final invalid message */
	total_handled = mxt_read_and_process_messages(data, count + 1);
	if (total_handled < 0)
		return IRQ_NONE;
	/* if there were invalid messages, then we are done */
	else if (total_handled <= count)
		goto update_count;

	/* read two at a time until an invalid message or else we reach
	 * reportid limit */
	do {
		num_handled = mxt_read_and_process_messages(data, 2);
		if (num_handled < 0)
			return IRQ_NONE;

		total_handled += num_handled;

		if (num_handled < 2)
			break;
	} while (total_handled < data->num_touchids);

update_count:
	data->last_message_count = total_handled;

	if (data->enable_reporting && data->update_input) {
		mxt_input_sync(data->input_dev);
		data->update_input = false;
	}

	return IRQ_HANDLED;
}

static irqreturn_t mxt_interrupt(int irq, void *dev_id)
{
	struct mxt_data *data = dev_id;
	if (data->in_bootloader) {
		/* bootloader state transition completion */
		complete(&data->bl_completion);
		return IRQ_HANDLED;
	}

	if (!data->object_table)
		return IRQ_NONE;

	if (data->T44_address) {
#ifdef I2C_SUSPEND_WORKAROUND
		queue_delayed_work(touch_wq, &data->check_suspended_work, 0);
#else
		queue_work(touch_wq, &data->work);
#endif
		return IRQ_HANDLED;
	} else {
		return mxt_process_messages(data);
	}
}

#ifdef I2C_SUSPEND_WORKAROUND
static void touch_check_suspended_worker(struct work_struct *check_suspended_work)
{
	struct mxt_data *data =
		container_of(to_delayed_work(check_suspended_work), struct mxt_data, check_suspended_work);

	if (atmel_touch_i2c_suspended){
		dev_err(&data->client->dev, "lge_touch touch suspended. try i2c operation after 10ms.\n");
		queue_delayed_work(touch_wq, &data->check_suspended_work, msecs_to_jiffies(10));
		return;
	}else{
		dev_dbg(&data->client->dev, "lge_touch touch resume. do touch work.\n");
		queue_work(touch_wq, &data->work);
		return;
	}
}
#endif
static int mxt_t6_command(struct mxt_data *data, u16 cmd_offset,
			  u8 value, bool wait)
{
	u16 reg;
	u8 command_register = 0;
	int timeout_counter = 0;
	int ret;

	reg = data->T6_address + cmd_offset;

	ret = mxt_write_reg(data->client, reg, value);
	if (ret)
		return ret;

	if (!wait)
		return 0;

	do {
		msleep(20);
		ret = __mxt_read_reg(data->client, reg, 1, &command_register);
		if (ret)
			return ret;
	} while ((command_register != 0) && (timeout_counter++ <= 100));

	if (timeout_counter > 100) {
		dev_err(&data->client->dev, "Command failed!\n");
		return -EIO;
	}

	return 0;
}

static int mxt_t25_command(struct mxt_data *data, u8 value, bool wait)
{
	u16 reg;
	u8 command_register = 0;
	int timeout_counter = 0;
	int ret;

	if(!selftest_enable)
		return 0;

	reg = data->T25_address + 1 ;

	ret = mxt_write_reg(data->client, reg, value);
	if (ret){
		dev_err(&data->client->dev, "Write Self Test Command fail!\n");
		return ret;
	}

	if (!wait)
		return 0;

	do {
		msleep(20);
		ret = __mxt_read_reg(data->client, reg, 1, &command_register);
		if (ret)
			return ret;
	} while ((command_register != 0) && (timeout_counter++ <= 100));

	if (timeout_counter > 100) {
		dev_err(&data->client->dev, "Command failed!\n");
		return -EIO;
	}

	return 0;
}

static int mxt_soft_reset(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int ret = 0;

	dev_info(dev, "Resetting chip\n");

	INIT_COMPLETION(data->reset_completion);

	ret = mxt_t6_command(data, MXT_COMMAND_RESET, MXT_RESET_VALUE, false);
	if (ret)
		return ret;

#ifdef CUST_B_TOUCH
	msleep(MXT_RESET_TIME);
	cancel_work_sync(&data->work);
#ifdef I2C_SUSPEND_WORKAROUND
	cancel_delayed_work_sync(&data->check_suspended_work);
#endif
	cancel_delayed_work_sync(&data->work_ime_drumming);
#else
	ret = mxt_wait_for_completion(data, &data->reset_completion,
				      MXT_RESET_TIMEOUT);
	if (ret)
		return ret;
#endif
	return 0;
}
#ifdef FIRMUP_ON_PROBE
static int mxt_backup(struct mxt_data *data)
{
    struct device *dev = &data->client->dev;
	int ret = 0;

	dev_info(dev, "Backup configuration data\n");

	ret = mxt_t6_command(data, MXT_COMMAND_BACKUPNV, MXT_BACKUP_VALUE, false);
	if (ret)
		return ret;

	msleep(MXT_BACKUP_TIME);
	return 0;
}

static void mxt_update_crc(struct mxt_data *data, u8 cmd, u8 value)
{
	/* on failure, CRC is set to 0 and config will always be downloaded */
	data->config_crc = 0;
	INIT_COMPLETION(data->crc_completion);

	mxt_t6_command(data, cmd, value, true);
	/* Wait for crc message. On failure, CRC is set to 0 and config will
	 * always be downloaded */
#ifdef CUST_B_TOUCH
	msleep(MXT_CRC_TIMEOUT);
#else
	mxt_wait_for_completion(data, &data->crc_completion, MXT_CRC_TIMEOUT);
#endif
}
#endif

static void mxt_calc_crc24(u32 *crc, u8 firstbyte, u8 secondbyte)
{
	static const unsigned int crcpoly = 0x80001B;
	u32 result;
	u32 data_word;

	data_word = (secondbyte << 8) | firstbyte;
	result = ((*crc << 1) ^ data_word);

	if (result & 0x1000000)
		result ^= crcpoly;

	*crc = result;
}

static u32 mxt_calculate_crc(u8 *base, off_t start_off, off_t end_off)
{
	u32 crc = 0;
	u8 *ptr = base + start_off;
	u8 *last_val = base + end_off - 1;

	if (end_off < start_off)
		return -EINVAL;

	while (ptr < last_val) {
		mxt_calc_crc24(&crc, *ptr, *(ptr + 1));
		ptr += 2;
	}

	/* if len is odd, fill the last byte with 0 */
	if (ptr == last_val)
		mxt_calc_crc24(&crc, *ptr, 0);

	/* Mask to 24-bit */
	crc &= 0x00FFFFFF;

	return crc;
}

static int mxt_init_t7_power_cfg(struct mxt_data *data);

static int mxt_check_retrigen(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error = 0;
	int val = 0;

	if (data->pdata->irqflags & IRQF_TRIGGER_LOW)
		return 0;

	if (data->T18_address) {
		error = __mxt_read_reg(client,
				       data->T18_address + MXT_COMMS_CTRL,
				       1, &val);
		if (error)
			return error;

		if (val & MXT_COMMS_RETRIGEN)
			return 0;
	}

	dev_warn(&client->dev, "Enabling RETRIGEN workaround\n");
	data->use_retrigen_workaround = true;
	return 0;
}

/*
 * mxt_check_reg_init - download configuration to chip
 *
 * Atmel Raw Config File Format
 *
 * The first four lines of the raw config file contain:
 *  1) Version
 *  2) Chip ID Information (first 7 bytes of device memory)
 *  3) Chip Information Block 24-bit CRC Checksum
 *  4) Chip Configuration 24-bit CRC Checksum
 *
 * The rest of the file consists of one line per object instance:
 *   <TYPE> <INSTANCE> <SIZE> <CONTENTS>
 *
 *   <TYPE> - 2-byte object type as hex
 *   <INSTANCE> - 2-byte object instance number as hex
 *   <SIZE> - 2-byte object size as hex
 *   <CONTENTS> - array of <SIZE> 1-byte hex values
 */

static int mxt_set_t7_power_cfg(struct mxt_data *data, u8 sleep)
{
	struct device *dev = &data->client->dev;
	int error;
	struct t7_config *new_config;
	struct t7_config deepsleep = { .idle = 0, .active = 0 };
	struct t7_config knockon_ta = { .idle = 255, .active = 255 };
	struct t7_config knockon = { .idle = 32, .active = 15 };	/* need to sync patch bin */

	if (sleep == MXT_POWER_CFG_DEEPSLEEP){
		data->power_status = MXT_POWER_CFG_DEEPSLEEP;
		new_config = &deepsleep;
	} else if (sleep == MXT_POWER_CFG_KNOCKON) {
		data->power_status = MXT_POWER_CFG_KNOCKON;
		if(data->charging_mode)
			new_config = &knockon_ta;
		else
			new_config = &knockon;
	} else {
		data->power_status = MXT_POWER_CFG_RUN;
		new_config = &data->t7_cfg;
	}

	error = __mxt_write_reg(data->client, data->T7_address,
			sizeof(data->t7_cfg),
			new_config);
	if (error)
		return error;

	dev_info(dev, "Set T7 ACTV:%d IDLE:%d\n",
		new_config->active, new_config->idle);

	return 0;
}

static int mxt_init_t7_power_cfg(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int error;
	bool retry = false;

recheck:
	error = __mxt_read_reg(data->client, data->T7_address,
				sizeof(data->t7_cfg), &data->t7_cfg);
	if (error)
		return error;

	if (data->t7_cfg.active == 0 || data->t7_cfg.idle == 0) {
		if (!retry) {
			dev_info(dev, "T7 cfg zero, resetting\n");
			mxt_soft_reset(data);
			retry = true;
			goto recheck;
		} else {
		    dev_dbg(dev, "T7 cfg zero after reset, overriding\n");
		    data->t7_cfg.active = 20;
		    data->t7_cfg.idle = 100;
		    return mxt_set_t7_power_cfg(data, MXT_POWER_CFG_RUN);
		}
	} else {
		dev_info(dev, "Initialised power cfg: ACTV %d, IDLE %d\n",
				data->t7_cfg.active, data->t7_cfg.idle);
		return 0;
	}
}

static int mxt_acquire_irq(struct mxt_data *data)
{
	int error;

	touch_enable_irq(data->irq);

	if (data->use_retrigen_workaround) {
		error = mxt_process_messages_until_invalid(data);
		if (error)
			return error;
	}

	return 0;
}

static void mxt_free_input_device(struct mxt_data *data)
{
	if (data->input_dev) {
		dev_info(&data->client->dev, "mxt_free_input_device\n");
		input_unregister_device(data->input_dev);
		data->input_dev = NULL;
	}
}

static void mxt_free_object_table(struct mxt_data *data)
{
	dev_info(&data->client->dev, "mxt_free_object_table\n");

	if(!update_cfg_force && !update_fw_force && !mxt_mfts){
	kfree(data->raw_info_block);
	data->object_table = NULL;
	data->info = NULL;
	data->raw_info_block = NULL;
	kfree(data->msg_buf);
	data->msg_buf = NULL;

	mxt_free_input_device(data);
	data->enable_reporting = false;
	}

	data->T5_address = 0;
	data->T5_msg_size = 0;
	data->T6_address = 0;
	data->T7_address = 0;
	data->T9_reportid_min = 0;
	data->T9_reportid_max = 0;
	data->T15_reportid_min = 0;
	data->T15_reportid_max = 0;
	data->T18_address = 0;
	data->T42_address = 0;
	data->T44_address = 0;
	data->T47_address = 0;
	data->T63_reportid_min = 0;
	data->T63_reportid_max = 0;
#ifdef MXT_LPWG
	data->T92_address = 0;
	data->T93_address = 0;
#endif
	data->T100_reportid_min = 0;
	data->T100_address = 0;
	data->max_reportid = 0;
}

static int mxt_parse_object_table(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int i;
	u8 reportid;
	u16 end_address;

	/* Valid Report IDs start counting from 1 */
	reportid = 1;
	data->mem_size = 0;
	for (i = 0; i < data->info->object_num; i++) {
		struct mxt_object *object = data->object_table + i;
		u8 min_id, max_id;

		le16_to_cpus(&object->start_address);

		if (object->num_report_ids) {
			min_id = reportid;
			reportid += object->num_report_ids *
					mxt_obj_instances(object);
			max_id = reportid - 1;
		} else {
			min_id = 0;
			max_id = 0;
		}

		dev_dbg(&data->client->dev,
			"T%u Start:%u Size:%u Instances:%u Report IDs:%u-%u\n",
			object->type, object->start_address,
			mxt_obj_size(object), mxt_obj_instances(object),
			min_id, max_id);

		switch (object->type) {
		case MXT_GEN_MESSAGE_T5:
			if (data->info->family_id == 0x80) {
				/* On mXT224 read and discard unused CRC byte
				 * otherwise DMA reads are misaligned */
				data->T5_msg_size = mxt_obj_size(object);
			} else {
				/* CRC not enabled, so skip last byte */
				data->T5_msg_size = mxt_obj_size(object) - 1;
			}
			data->T5_address = object->start_address;
		case MXT_GEN_COMMAND_T6:
			data->T6_address = object->start_address;
			break;
		case MXT_GEN_POWER_T7:
			data->T7_address = object->start_address;
			break;
		case MXT_TOUCH_MULTI_T9:
			/* Only handle messages from first T9 instance */
			data->T9_reportid_min = min_id;
			data->T9_reportid_max = min_id +
						object->num_report_ids - 1;
			data->num_touchids = object->num_report_ids;
			break;
		case MXT_TOUCH_KEYARRAY_T15:
			data->T15_reportid_min = min_id;
			data->T15_reportid_max = max_id;
			break;
		case MXT_SPT_COMMSCONFIG_T18:
			data->T18_address = object->start_address;
			break;
		case MXT_SPT_SELFTEST_T25:
			data->T25_address = object->start_address;
			break;
		case MXT_PROCI_TOUCHSUPPRESSION_T42:
			data->T42_address = object->start_address;
			break;
		case MXT_SPT_MESSAGECOUNT_T44:
			data->T44_address = object->start_address;
			break;
		case MXT_PROCI_STYLUS_T47:
			data->T47_address = object->start_address;
			break;
		case MXT_PROCI_ACTIVE_STYLUS_T63:
			/* Only handle messages from first T63 instance */
			data->T63_reportid_min = min_id;
			data->T63_reportid_max = min_id;
			data->num_stylusids = 1;
			break;
#ifdef MXT_LPWG
		case MXT_T92_NEW:
			data->T92_address = object->start_address;
			break;
		case MXT_T93_NEW:
			data->T93_address = object->start_address;
			data->T93_reportid_min = min_id;
			break;
#endif
		case MXT_TOUCH_MULTITOUCHSCREEN_T100:
			data->T100_reportid_min = min_id;
			/* first two report IDs reserved */
			data->num_touchids = object->num_report_ids - 2;
			data->T100_address = object->start_address;
			break;
		}

		end_address = object->start_address
			+ mxt_obj_size(object) * mxt_obj_instances(object) - 1;

		if (end_address >= data->mem_size)
			data->mem_size = end_address + 1;
	}

	/* Store maximum reportid */
	data->max_reportid = reportid;

	/* If T44 exists, T5 position has to be directly after */
	if (data->T44_address && (data->T5_address != data->T44_address + 1)) {
		dev_err(&client->dev, "Invalid T44 position\n");
		return -EINVAL;
	}

	data->msg_buf = kcalloc(data->max_reportid,
				data->T5_msg_size, GFP_KERNEL);
	if (!data->msg_buf) {
		dev_err(&client->dev, "Failed to allocate message buffer\n");
		return -ENOMEM;
	}

	return 0;
}

static int mxt_read_info_block(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	size_t size;
	void *buf;
	struct mxt_info *info;
	u32 calculated_crc;
	u8 *crc_ptr;
	dev_info(&data->client->dev, "mxt_read_info_block\n");

	/* If info block already allocated, free it */
	if (data->raw_info_block != NULL)
		mxt_free_object_table(data);

	/* Read 7-byte ID information block starting at address 0 */
	size = sizeof(struct mxt_info);
	buf = kzalloc(size, GFP_KERNEL);
	if (!buf) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	error = __mxt_read_reg(client, 0, size, buf);
	if (error)
		goto err_free_mem;

	/* Resize buffer to give space for rest of info block */
	info = (struct mxt_info *)buf;
	size = size + (info->object_num * sizeof(struct mxt_object))
		+ MXT_INFO_CHECKSUM_SIZE;

	buf = krealloc(buf, size, GFP_KERNEL);
	if (!buf) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		error = -ENOMEM;
		goto err_free_mem;
	}

	/* Read rest of info block */
	error = __mxt_read_reg(client, MXT_OBJECT_START,
			       size - MXT_OBJECT_START,
			       buf + MXT_OBJECT_START);
	if (error)
		goto err_free_mem;

	/* Extract & calculate checksum */
	crc_ptr = buf + size - MXT_INFO_CHECKSUM_SIZE;
	data->info_crc = crc_ptr[0] | (crc_ptr[1] << 8) | (crc_ptr[2] << 16);

	calculated_crc = mxt_calculate_crc(buf, 0,
					   size - MXT_INFO_CHECKSUM_SIZE);

	/* CRC mismatch can be caused by data corruption due to I2C comms
	 * issue or else device is not using Object Based Protocol */
	if (data->info_crc != calculated_crc) {
		dev_err(&client->dev,
			"Info Block CRC error calculated=0x%06X read=0x%06X\n",
			data->info_crc, calculated_crc);
		return -EIO;
	}

	/* Save pointers in device data structure */
	data->raw_info_block = buf;
	data->info = (struct mxt_info *)buf;
	data->object_table = (struct mxt_object *)(buf + MXT_OBJECT_START);

	dev_info(&client->dev,
		 "Family: %02X Variant: %02X Firmware V%u.%u.%02X Objects: %u\n",
		 data->info->family_id, data->info->variant_id, data->info->version >> 4,
		 data->info->version & 0xf, data->info->build, data->info->object_num);

	/* Parse object table information */
	error = mxt_parse_object_table(data);
	if (error) {
		dev_err(&client->dev, "Error %d reading object table\n", error);
		mxt_free_object_table(data);
		return error;
	}

	data->reportids = kcalloc(data->max_reportid + 1,
			sizeof(struct mxt_reportid),
			GFP_KERNEL);
	if (!data->reportids) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		error = -ENOMEM;
		goto err_free_mem;
	}

	/* Make report id table */
	mxt_make_reportid_table(data);

	return 0;

err_free_mem:
	kfree(buf);
	return error;
}

static int mxt_read_t9_resolution(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	struct t9_range range;
	unsigned char orient = 0;
	struct mxt_object *object;
	memset(&range, 0, sizeof(range));

	object = mxt_get_object(data, MXT_TOUCH_MULTI_T9);
	if (!object)
		return -EINVAL;

	error = __mxt_read_reg(client,
			       object->start_address + MXT_T9_RANGE,
			       sizeof(range), &range);
	if (error)
		return error;

	le16_to_cpus(range.x);
	le16_to_cpus(range.y);

	error =  __mxt_read_reg(client,
				object->start_address + MXT_T9_ORIENT,
				1, &orient);
	if (error)
		return error;

	/* Handle default values */
	if (range.x == 0)
		range.x = 2159;

	if (range.y == 0)
		range.y = 3839;

	if (orient & MXT_T9_ORIENT_SWITCH) {
		data->max_x = range.y;
		data->max_y = range.x;
	} else {
		data->max_x = range.x;
		data->max_y = range.y;
	}

	dev_info(&client->dev,
		 "Touchscreen size X%uY%u\n", data->max_x, data->max_y);

	return 0;
}

static void mxt_regulator_enable(struct mxt_data *data)
{
	int error = 0;
	dev_info(&data->client->dev, "regulator enable\n");

	data->power_status = MXT_POWER_ON;

	gpio_set_value(data->pdata->gpio_reset, 0);

	error = regulator_enable(data->vdd_ana);
	if (error < 0) {
		dev_err(&data->client->dev, "vdd_ana regulator enable fail\n");
		return;
	}
	error = regulator_enable(data->vcc_i2c);
	if (error < 0) {
		dev_err(&data->client->dev, "vcc_i2c regulator enable fail\n");
		return;
	}
	error = regulator_enable(data->vcc_dig);
	if (error < 0) {
		dev_err(&data->client->dev, "vcc_dig regulator enable fail\n");
		return;
	}
	msleep(MXT_REGULATOR_DELAY);

	INIT_COMPLETION(data->bl_completion);
	gpio_set_value(data->pdata->gpio_reset, 1);
#ifdef CUST_B_TOUCH
	msleep(MXT_POWERON_DELAY);
#else
	mxt_wait_for_completion(data, &data->bl_completion, MXT_POWERON_DELAY);
#endif
}

static void mxt_regulator_disable(struct mxt_data *data)
{
	int error = 0;
	dev_info(&data->client->dev, "regulator disable\n");

	data->power_status = MXT_POWER_OFF;

	error = regulator_disable(data->vdd_ana);
	if (error < 0) {
		dev_err(&data->client->dev, "vdd_ana regulator disable fail\n");
		return;
	}
	error = regulator_disable(data->vcc_i2c);
	if (error < 0) {
		dev_err(&data->client->dev, "vcc_i2c regulator disable fail\n");
		return;
	}
	error = regulator_disable(data->vcc_dig);
	if (error < 0) {
		dev_err(&data->client->dev, "vcc_dig regulator disable fail\n");
		return;
	}
}

static void mxt_probe_regulators(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int error;

	/* According to maXTouch power sequencing specification, RESET line
	 * must be kept low until some time after regulators come up to
	 * voltage */
	if (!data->pdata->gpio_reset) {
		dev_warn(dev, "Must have reset GPIO to use regulator support\n");
		goto fail;
	}

	data->vdd_ana = regulator_get(dev, "vdd_ana");
	if (IS_ERR(data->vdd_ana)) {
		error = PTR_ERR(data->vdd_ana);
		dev_err(dev, "Error %d getting ana regulator\n", error);
		goto fail;
	}

	data->vcc_i2c = regulator_get(dev, "vcc_i2c");
	if (IS_ERR(data->vcc_i2c)) {
		error = PTR_ERR(data->vcc_i2c);
		dev_err(dev, "Error %d getting i2c regulator\n", error);
		goto fail_release;
	}

	data->vcc_dig= regulator_get(dev, "vcc_dig");
	if (IS_ERR(data->vcc_dig)) {
		error = PTR_ERR(data->vcc_dig);
		dev_err(dev, "Error %d getting dig regulator\n", error);
		goto fail_release2;
	}

	error = regulator_set_voltage(data->vdd_ana, 3300000, 3300000);
	if (error < 0) {
		dev_err(dev, "Error %d cannot control ana regulator\n", error);
		goto fail;
	}

	error = regulator_set_voltage(data->vcc_dig, 2850000, 2850000);
	if (error < 0) {
		dev_err(dev, "Error %d cannot control dig regulator\n", error);
		goto fail_release2;
	}

	data->use_regulator = true;
	mxt_regulator_enable(data);

	dev_info(dev, "Initialized regulators\n");
	return;

fail_release2:
	regulator_put(data->vcc_i2c);
fail_release:
	regulator_put(data->vdd_ana);
fail:
	data->vdd_ana = NULL;
	data->vcc_i2c = NULL;
	data->vcc_dig = NULL;
	data->use_regulator = false;
}

static int mxt_read_t100_config(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	struct mxt_object *object;
	u16 range_x = 0, range_y = 0;
	u8 cfg = 0, tchaux = 0;
	u8 aux = 0;
#ifdef T100_AREA_REPLACE_AMPLITUDE
	u8 palm_threshold = 0;
#endif

	object = mxt_get_object(data, MXT_TOUCH_MULTITOUCHSCREEN_T100);
	if (!object)
		return -EINVAL;

	error = __mxt_read_reg(client,
			       object->start_address + MXT_T100_XRANGE,
			       sizeof(range_x), &range_x);
	if (error)
		return error;

	le16_to_cpus(range_x);

	error = __mxt_read_reg(client,
			       object->start_address + MXT_T100_YRANGE,
			       sizeof(range_y), &range_y);
	if (error)
		return error;

	le16_to_cpus(range_y);

	error =  __mxt_read_reg(client,
				object->start_address + MXT_T100_CFG1,
				1, &cfg);
	if (error)
		return error;

	error =  __mxt_read_reg(client,
				object->start_address + MXT_T100_TCHAUX,
				1, &tchaux);
	if (error)
		return error;

	/* Handle default values */
	if (range_x == 0)
		range_x = 2159;

	/* Handle default values */
	if (range_x == 0)
		range_x = 2159;

	if (range_y == 0)
		range_y = 3839;

	if (cfg & MXT_T100_CFG_SWITCHXY) {
		data->max_x = range_y;
		data->max_y = range_x;
	} else {
		data->max_x = range_x;
		data->max_y = range_y;
	}

	/* allocate aux bytes */
	aux = 6;

	if (tchaux & MXT_T100_TCHAUX_VECT)
		data->t100_aux_vect = aux++;

	if (tchaux & MXT_T100_TCHAUX_AMPL)
		data->t100_aux_ampl = aux++;

	if (tchaux & MXT_T100_TCHAUX_AREA)
		data->t100_aux_area = aux++;

	if (tchaux & MXT_T100_TCHAUX_RESV)
		data->t100_aux_resv = aux++;

#ifdef T100_AREA_REPLACE_AMPLITUDE
	if(data->t100_aux_area){
		object = mxt_get_object(data, MXT_SPT_USERDATA_T38);
		if (!object)
			return -EINVAL;

		error = __mxt_read_reg(client, object->start_address + 7, 1, &palm_threshold);
		if (error)
			return error;

		if(palm_threshold == 0){
			dev_info(&client->dev,"T100 Palm detecting threshold is 0!!!!");
			palm_threshold = 45;
		}
		data->T100_palm_threshold = palm_threshold;
	} else {
		data->T100_palm_threshold = 255;
	}

	dev_info(&client->dev,
		 "T100 Touchscreen size X%u Y%u amp%u area%u vec%u resv%u palm_threshold%d\n",
			data->max_x, data->max_y, data->t100_aux_ampl,
			data->t100_aux_area, data->t100_aux_vect, data->t100_aux_resv, data->T100_palm_threshold);
#else
	dev_info(&client->dev,
		 "T100 Touchscreen size X%u Y%u amp%u area%u vec%u\n",
	 		data->max_x, data->max_y, data->t100_aux_ampl,
	 		data->t100_aux_area, data->t100_aux_vect);
#endif

	return 0;
}

static int mxt_input_open(struct input_dev *dev);
static void mxt_input_close(struct input_dev *dev);

static int mxt_initialize_t100_input_device(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	struct input_dev *input_dev;
	int error;
	unsigned int num_mt_slots;

	error = mxt_read_t100_config(data);
	if (error)
		dev_warn(dev, "Failed to initialize T100 configuration\n");

	input_dev = input_allocate_device();
	if (!data || !input_dev) {
		dev_err(dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}
	/* Auto Test interface */
	touch_test_dev = data;

	input_dev->name = "touch_dev"; /*must sync to idc file name*/
	input_dev->phys = data->phys;
	input_dev->id.bustype = BUS_I2C;
	/* input_dev->dev.parent = &data->client->dev;	remove this line for sysfs path (virtual) */
	input_dev->open = mxt_input_open;
	input_dev->close = mxt_input_close;

	set_bit(EV_ABS, input_dev->evbit);
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit); 
	input_set_capability(input_dev, EV_KEY, BTN_TOUCH);
	/* For multi touch */
	num_mt_slots = data->num_touchids;
	error = input_mt_init_slots(input_dev, num_mt_slots);
	if (error) {
		dev_err(dev, "Error %d initialising slots\n", error);
		goto err_free_mem;
	}

	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID,
			     0, data->pdata->numtouch, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR,
			     0, MXT_MAX_AREA, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MINOR,
			     0, MXT_MAX_AREA, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			     0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			     0, data->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE,
			     0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_ORIENTATION,
			     0, 255, 0, 0);

	input_set_drvdata(input_dev, data);

	error = input_register_device(input_dev);
	if (error) {
		dev_err(dev, "Error %d registering input device\n", error);
		goto err_free_mem;
	}

	data->input_dev = input_dev;

	return 0;

err_free_mem:
	input_free_device(input_dev);
	return error;
}

static int mxt_initialize_t9_input_device(struct mxt_data *data);
static int mxt_configure_objects(struct mxt_data *data);
static ssize_t mxt_firmware_update(struct mxt_data *data);

static int mxt_initialize(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	u8 retry_count = 0;
	dev_info(&data->client->dev, "mxt_initialize\n");

retry_probe:
	error = mxt_read_info_block(data);
	if (error) {
		error = mxt_probe_bootloader(data, retry_count);
		if (error) {
			if (++retry_count > 11)
				/* Chip is not in appmode or bootloader mode */
				return error;

			goto retry_probe;
		} else {
#ifdef FIRMUP_ON_PROBE
			dev_err(&client->dev, "IC stay in bootloader mode\n");
			data->in_bootloader = true;
			return 0;
#else
			if (++retry_count > 10) {
				dev_err(&client->dev,
						"Could not recover device from "
						"bootloader mode\n");
				/* this is not an error state, we can reflash
				 * from here */
				data->in_bootloader = true;
				return 0;
			}

			/* Attempt to exit bootloader into app mode */
			mxt_send_bootloader_cmd(data, false);
			msleep(MXT_FW_RESET_TIME);
			goto retry_probe;
#endif
		}
	}

	error = mxt_check_retrigen(data);
	if (error)
		return error;

	return 0;
}

static int mxt_rest_init(struct mxt_data *data)
{
	int error;

	error = mxt_acquire_irq(data);
	if (error)
		return error;

	error = mxt_configure_objects(data);
	if (error)
		return error;

#ifdef TSP_PATCH
	if(sizeof(patch_bin))
		data->patch.patch = patch_bin;
	else
		dev_info(&data->client->dev, "No patch file %p\n", data->patch.patch);

	if (data->patch.patch){
		dev_info(&data->client->dev, "mxt_patch_init on probe, size:%d patch:%p\n", sizeof(patch_bin), data->patch.patch);
		error = mxt_patch_init(data, data->patch.patch);
		if (error) {
			dev_err(&data->client->dev, "Failed to mxt_patch_init\n");
		}
		dev_info(&data->client->dev, " patch date = %d\n", data->patch.date);
	}
	else
		dev_info(&data->client->dev, "No patch on probe, size:%d\n", sizeof(patch_bin));
#endif

	return 0;
}

static int mxt_configure_objects(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;

	error = mxt_init_t7_power_cfg(data);
	if (error) {
		dev_err(&client->dev, "Failed to initialize power cfg\n");
		return error;
	}

#ifdef FIRMUP_ON_PROBE
	mxt_update_crc(data, MXT_COMMAND_REPORTALL, 1);

	dev_dbg(&data->client->dev, "current config checksum is 0x%06X\n",data->config_crc);
	if (data->config_crc == 0 || data->config_crc != MXT_LATEST_CONFIG_CRC || update_cfg_force) {
		/* Disable T70 Dynamic configuration */
		mxt_t6_command(data, MXT_COMMAND_BACKUPNV, MXT_STOP_DYNAMIC_CONFIG, false);

		error = mxt_write_configuration(data);
		if (error) {
			dev_err(&data->client->dev, "Failed to write config\n");
			return error;
		}

		error = mxt_backup(data);
		if (error) {
			dev_err(&data->client->dev, "Failed to backup NV data\n");
			return error;
		}

		error = mxt_soft_reset(data);
		if (error) {
			dev_err(&data->client->dev, "Failed to reset IC\n");
			return error;
		}

		error = mxt_init_t7_power_cfg(data);
		if (error) {
			dev_err(&client->dev, "Failed to initialize power cfg after write configuration\n");
			return error;
		}
	}
	error = mxt_check_retrigen(data);
	if (error)
		return error;
#endif

	if (data->T9_reportid_min) {
		error = mxt_initialize_t9_input_device(data);
		if (error)
			return error;
	} else if (data->T100_reportid_min) {
		error = mxt_initialize_t100_input_device(data);
		if (error)
			return error;
	} else {
		dev_warn(&client->dev, "No touch object detected\n");
	}
	return 0;
}

/* Firmware Version is returned as Major.Minor.Build */
static ssize_t mxt_fw_ver_show(struct mxt_data *data, char *buf)
{
	int ret = 0;
	u8 build = 0;

	if (!mxt_mfts_for_mxt_start){
		ret = sprintf(buf, "0x%06X\n",MXT_LATEST_CONFIG_CRC);	/* test mode */
		return ret;
	}
	ret = __mxt_read_reg(data->client, 0x03, 1, &build);
	if(ret){
		sprintf(buf, "Fail read Buid info\n");
	}

	if (build == 0xAA) { /* mfts */
		ret = sprintf(buf, "0x%06X\n", MXT_LATEST_CONFIG_CRC_MFTS);	/* MFTS for 3.0 panel */
	} else {
	#if 0	//kyutae
		ret = sprintf(buf, "0x%06X\n",MXT_LATEST_CONFIG_CRC);
	#else
		ret = sprintf(buf, "No need to upgrade\n");
	#endif
	}
	return ret;
	/*return scnprintf(buf, PAGE_SIZE, "%u.%u.%02X\n",
			 data->info->version >> 4, data->info->version & 0xf,
			 data->info->build); */
}

/* Hardware Version is returned as FamilyID.VariantID */
static ssize_t mxt_hw_version_show(struct mxt_data *data, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u.%u\n",
			data->info->family_id, data->info->variant_id);
}

/* Configuration Checksum */
static ssize_t mxt_mxt_info_show(struct mxt_data *data, char *buf)
{
	int ret = 0;

	ret = sprintf(buf, "\n====== Touch IC Info ======\n");
	ret += sprintf(buf+ret, "FW version           = %u.%u.%02X\n",
					 data->info->version >> 4, data->info->version & 0xf,
					 data->info->build);
	ret += sprintf(buf+ret, "config checksum      = 0x%06X\n", data->config_crc);
	ret += sprintf(buf+ret, "Object Num           = %d\n", data->info->object_num);
	ret += sprintf(buf+ret, "Faily Id             = %d\n", data->info->family_id);
	ret += sprintf(buf+ret, "Variant              = %d\n", data->info->variant_id);
	ret += sprintf(buf+ret, "Version              = %d\n", data->info->version);
	ret += sprintf(buf+ret, "Build                = %d\n", data->info->build);
#ifdef TSP_PATCH
	if (data->patch.patch)
		ret += sprintf(buf+ret, "Patch Date           = %d\n", data->patch.date);
#endif

	return ret;
}

static ssize_t mxt_selftest_store(struct mxt_data *data, const char *buf, size_t count)
{
	int command = 0;
	int ret = 0;

	ret = sscanf(buf, "%u", &command);
	printk(KERN_INFO "\n");

	selftest_enable = true;
	mxt_t25_command(data, command, false);

	return count;
}

static ssize_t mxt_selftest_show(struct mxt_data *data, char *buf)
{
	int ret = 0;
	int test_all_cmd = 254;

	selftest_enable = true;
	selftest_show = true;

	mxt_t25_command(data, test_all_cmd, false);
	msleep(MXT_SELFTEST_TIME);

	ret = sprintf(buf, "====== MXT Self Test Info ======\n");
	if(data->self_test_status[0] == 0){
		ret += sprintf(buf+ret, "Need more time. Try Again.\n");
		return ret;
	}

	if(data->self_test_status[0] == 0xFD){
		ret += sprintf(buf+ret, "Invalid Test Code. Try Again.");
	}else if(data->self_test_status[0] == 0xFC){
		ret += sprintf(buf+ret, "The test could not be completed due to an unrelated fault. Try again.");
	}else{
		ret += sprintf(buf+ret, "All Test Result: %s", (data->self_test_status[0] == 0xFE) ? "Pass\n" : "Fail\n");
		ret += sprintf(buf+ret, "AVdd power Test Result: %s", (data->self_test_status[0] != 0x01) ? "Pass\n" : "Fail\n");

		ret += sprintf(buf+ret, "Pin Falut Test Result: %s", (data->self_test_status[0] != 0x12) ? "Pass\n" : "Fail\n");
		if(data->self_test_status[0] == 0x12)
			ret += sprintf(buf+ret, "# Fail # seq_num(%u) x_pin(%u) y_pin(%u)",
										data->self_test_status[1], data->self_test_status[2], data->self_test_status[3]);

		ret += sprintf(buf+ret, "Signal Limit Test: %s", (data->self_test_status[0] != 0x17) ? "Pass\n" : "Fail\n");
		if(data->self_test_status[0] == 0x17)
			ret += sprintf(buf+ret, "# Fail # type_num(%u) type_instance(%u)", data->self_test_status[1], data->self_test_status[2]);
	}

	memset(&data->self_test_status, 0, sizeof(data->self_test_status));
	selftest_show = false;
	return ret;
}

static ssize_t mxt_show_instance(char *buf, int count,
				 struct mxt_object *object, int instance,
				 const u8 *val)
{
	int i;

	if (mxt_obj_instances(object) > 1)
		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "Instance %u\n", instance);

	for (i = 0; i < mxt_obj_size(object); i++)
		count += scnprintf(buf + count, PAGE_SIZE - count,
				"\t[%2u]: %02x (%d)\n", i, val[i], val[i]);
	count += scnprintf(buf + count, PAGE_SIZE - count, "\n");

	return count;
}

static ssize_t mxt_object_show(struct mxt_data *data, char *buf)
{
	struct mxt_object *object;
	int count = 0;
	int i, j;
	int error;
	u8 *obuf;

	/* Pre-allocate buffer large enough to hold max sized object. */
	obuf = kmalloc(256, GFP_KERNEL);
	if (!obuf)
		return -ENOMEM;

	error = 0;
	for (i = 0; i < data->info->object_num; i++) {
		object = data->object_table + i;

		if (!mxt_object_readable(object->type))
			continue;

		count += scnprintf(buf + count, PAGE_SIZE - count,
				"T%u:\n", object->type);

		for (j = 0; j < mxt_obj_instances(object); j++) {
			u16 size = mxt_obj_size(object);
			u16 addr = object->start_address + j * size;

			error = __mxt_read_reg(data->client, addr, size, obuf);
			if (error)
				goto done;

			count = mxt_show_instance(buf, count, object, j, obuf);
		}
	}

done:
	kfree(obuf);
	return error ?: count;
}

/* mxt_object_control
 * Usage
 * - read : echo read object 0 0 > object_ctrl
 * - wirte : echo write object address_offset value > object_ctrl
 */
static ssize_t mxt_object_control(struct mxt_data *data, const char *buf, size_t count)
{
	struct mxt_object *object;
	unsigned char command[6];
	int type = 0;
	int addr_offset = 0;
	int value = 0;
	int error = 0;
	int i = 0,j = 0;
	u8 *obuf;

	sscanf(buf, "%s %d %d %d", command, &type, &addr_offset, &value);

	obuf = kmalloc(256, GFP_KERNEL);
	if (!obuf)
		return -ENOMEM;

	if(type == 25)
		selftest_enable = true;

	object = mxt_get_object(data, type);
	if (!object) {
        dev_err(&data->client->dev,
			"error Cannot get object_type T%d\n", type);
        return -EINVAL;
    }

	if ((mxt_obj_size(object) == 0) || (object->start_address == 0)) {
		dev_err(&data->client->dev,
			"error object_type T%d\n", type);
		return -ENODEV;
	}

	if (!strncmp(command, "read", 4)){	/*read*/
		dev_info(&data->client->dev, "Object Read T%d: start_addr=%d, size=%d * instance=%d\n",
		type, object->start_address, mxt_obj_size(object), mxt_obj_instances(object));

		for (j = 0; j < mxt_obj_instances(object); j++) {
			u16 size = mxt_obj_size(object);
			u16 addr = object->start_address + j * size;

			error = __mxt_read_reg(data->client, addr, size, obuf);
			if (error)
				dev_err(&data->client->dev, "Object Read Fail\n");
		}

		for (i = 0; i < mxt_obj_size(object)*mxt_obj_instances(object); i++)
			dev_info(&data->client->dev, "T%d [%d] %d[0x%x]\n", type, i, obuf[i], obuf[i]);

	}else if (!strncmp(command, "write", 4)){	/*write*/
		dev_info(&data->client->dev, "Object Write T%d: start_addr=%d, size=%d * instance=%d\n",
		type, object->start_address, mxt_obj_size(object), mxt_obj_instances(object));

		error = mxt_write_reg(data->client, object->start_address+addr_offset, value);
		if (error)
			dev_err(&data->client->dev, "Object Write Fail\n");

		dev_info(&data->client->dev, "Object Write Success. Execute Read Object and Check Value.\n");
#ifdef T100_AREA_REPLACE_AMPLITUDE
		if(type == 38 && !error){
			u8 palm_threshold = 0;

			object = mxt_get_object(data, MXT_SPT_USERDATA_T38);
			__mxt_read_reg(data->client, object->start_address + 7, 1, &palm_threshold);
			if(palm_threshold == 0){
				dev_info(&data->client->dev,"Object Palm detecting threshold is 0!!!!");
				palm_threshold = 255;
			}
			data->T100_palm_threshold = palm_threshold;

			dev_info(&data->client->dev, "Object write palm_threshold T38[7]=%d\n",	palm_threshold);
		}
		else if(type == 100 && addr_offset == 3 && !error){
			u8 tchaux = 0;
			u8 aux = 0;
			u8 palm_threshold = 0;

			tchaux = value;
			aux = 6; /* allocate aux bytes */

			if (tchaux & MXT_T100_TCHAUX_VECT)
				data->t100_aux_vect = aux++;

			if (tchaux & MXT_T100_TCHAUX_AMPL){
				data->t100_aux_ampl = aux++;
			}

			if (tchaux & MXT_T100_TCHAUX_AREA){
				data->t100_aux_area = aux++;
			}

			if (tchaux & MXT_T100_TCHAUX_RESV){
				data->t100_aux_resv = aux++;
			}

			if(data->t100_aux_area){
				object = mxt_get_object(data, MXT_SPT_USERDATA_T38);
				__mxt_read_reg(data->client, object->start_address + 7, 1, &palm_threshold);
				if(palm_threshold == 0){
					dev_info(&data->client->dev,"Object Palm detecting threshold is 0!!!!");
					palm_threshold = 255;
				}
				data->T100_palm_threshold = palm_threshold;
			} else {
				data->T100_palm_threshold = 255;
			}

			dev_info(&data->client->dev,
				 "T100 Touchscreen size X%u Y%u amp%u area%u vec%u resv%u palm_threshold%d\n",
						data->max_x, data->max_y, data->t100_aux_ampl,
						data->t100_aux_area, data->t100_aux_vect, data->t100_aux_resv, data->T100_palm_threshold);
		}
#endif
	}else{
		dev_err(&data->client->dev, "Command Fail. Usage: echo [read | write] object cmd_field value > object_ctrl\n");
	}
	return count;
}

int mxt_read_object(struct mxt_data *data, u8 type, u8 offset, u8 *value)
{
	struct mxt_object *object;
	int error = 0;

	object = mxt_get_object(data, type);
	if (!object) {
		dev_err(&data->client->dev,
			"error Cannot get object_type T%d\n", type);
		return -EINVAL;
	}

	if ((mxt_obj_size(object) == 0) || (object->start_address == 0)) {
		dev_err(&data->client->dev,
			"error object_type T%d\n", type);
		return -ENODEV;
	}
	error = __mxt_read_reg(data->client, object->start_address+offset, 1, value);
	if (error)
		dev_err(&data->client->dev, "Object Read Fail\n");

	return error;
}

int mxt_write_object(struct mxt_data *data, u8 type, u8 offset, int value)
{
	struct mxt_object *object;
	int error = 0;

	object = mxt_get_object(data, type);
	if (!object) {
		dev_err(&data->client->dev,
			"error Cannot get object_type T%d\n", type);
		return -EINVAL;
	}

	if ((mxt_obj_size(object) == 0) || (object->start_address == 0)) {
		dev_err(&data->client->dev,
			"error object_type T%d\n", type);
		return -ENODEV;
	}

	error = mxt_write_reg(data->client, object->start_address+offset, value);
	if (error)
		dev_err(&data->client->dev, "Object Write Fail\n");

	return 0;
}

static int mxt_check_firmware_format(struct device *dev,
				     const struct firmware *fw)
{
	unsigned int pos = 0;
	char c;

	while (pos < fw->size) {
		c = *(fw->data + pos);

		if (c < '0' || (c > '9' && c < 'A') || c > 'F')
			return 0;

		pos++;
	}

	/* To convert file try
	 * xxd -r -p mXTXXX__APP_VX-X-XX.enc > maxtouch.fw */
	dev_err(dev, "Aborting: firmware file must be in binary format\n");

	return -1;
}

static int mxt_load_fw(struct device *dev, bool from_header)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	const struct firmware *fw = NULL;
#ifdef FIRMUP_ON_PROBE
	struct firmware *fw_from_header = NULL;
#endif
	unsigned int frame_size;
	unsigned int pos = 0;
	unsigned int retry = 0;
	unsigned int frame = 0;
	int ret;

#ifdef FIRMUP_ON_PROBE
	if (from_header) {
		fw_from_header = kzalloc(sizeof(struct firmware), GFP_KERNEL);

		fw_from_header->data = latest_firmware;
		fw_from_header->size = sizeof(latest_firmware);
		fw = fw_from_header;
	} else {
		ret = request_firmware(&fw, data->fw_name, dev);
		if (ret) {
			dev_err(dev, "Unable to open firmware %s  ret %d\n", data->fw_name, ret);
			return ret;
		}
	}
#else
	ret = request_firmware(&fw, data->fw_name, dev);
	if (ret) {
		dev_err(dev, "Unable to open firmware %s  ret %d\n", data->fw_name, ret);
		return ret;
	}
#endif

	/* Check for incorrect enc file */
	ret = mxt_check_firmware_format(dev, fw);
	if (ret)
		goto release_firmware;

	if (data->suspended) {
		if (data->use_regulator)
			mxt_regulator_enable(data);

		touch_enable_irq(data->irq);
		data->suspended = false;
	}

	if (!data->in_bootloader) {
		/* Change to the bootloader mode */
		data->in_bootloader = true;

		ret = mxt_t6_command(data, MXT_COMMAND_RESET,
				     MXT_BOOT_VALUE, false);
		if (ret)
			goto release_firmware;

		msleep(MXT_RESET_TIME);

		/* At this stage, do not need to scan since we know
		 * family ID */
		ret = mxt_lookup_bootloader_address(data, 0);
		if (ret)
			goto release_firmware;
	}

	mxt_free_object_table(data);
	INIT_COMPLETION(data->bl_completion);

	ret = mxt_check_bootloader(data, MXT_WAITING_BOOTLOAD_CMD);
	if (ret) {
		/* Bootloader may still be unlocked from previous update
		 * attempt */
		ret = mxt_check_bootloader(data, MXT_WAITING_FRAME_DATA);
		if (ret)
			goto disable_irq;
	} else {
		dev_info(dev, "Unlocking bootloader\n");

		/* Unlock bootloader */
		ret = mxt_send_bootloader_cmd(data, true);
		if (ret)
			goto disable_irq;
	}

	while (pos < fw->size) {
		ret = mxt_check_bootloader(data, MXT_WAITING_FRAME_DATA);
		if (ret)
			goto disable_irq;

		frame_size = ((*(fw->data + pos) << 8) | *(fw->data + pos + 1));

		/* Take account of CRC bytes */
		frame_size += 2;

		/* Write one frame to device */
		ret = mxt_bootloader_write(data, fw->data + pos, frame_size);
		if (ret)
			goto disable_irq;

		ret = mxt_check_bootloader(data, MXT_FRAME_CRC_PASS);
		if (ret) {
			retry++;

			/* Back off by 20ms per retry */
			msleep(retry * 20);

			if (retry > 20) {
				dev_err(dev, "Retry count exceeded\n");
				goto disable_irq;
			}
		} else {
			retry = 0;
			pos += frame_size;
			frame++;
		}

		if (frame % 50 == 0)
			dev_info(dev, "Sent %d frames, %d/%zd bytes\n",
				 frame, pos, fw->size);
	}

	/* Wait for flash */
	ret = mxt_wait_for_completion(data, &data->bl_completion,
				      MXT_FW_RESET_TIME);
	if (ret)
		goto disable_irq;
	dev_info(dev, "Sent %d frames, %zd bytes\n", frame, pos);

#if 0	/*To avoid reset timeout*/
	/* Wait for device to reset */
	mxt_wait_for_completion(data, &data->bl_completion, MXT_RESET_TIMEOUT);
#endif
	data->in_bootloader = false;

disable_irq:
	touch_disable_irq(data->irq);
release_firmware:
#ifdef FIRMUP_ON_PROBE
	if (from_header)
		kfree(fw_from_header);
	else
#endif
	release_firmware(fw);
	return ret;
}

static int mxt_update_file_name(struct device *dev, char **file_name,
				const char *buf, size_t count)
{
	char *file_name_tmp;

	/* Simple sanity check */
	if (count > 64) {
		dev_warn(dev, "File name too long\n");
		return -EINVAL;
	}

	file_name_tmp = krealloc(*file_name, count + 1, GFP_KERNEL);
	if (!file_name_tmp) {
		dev_warn(dev, "no memory\n");
		return -ENOMEM;
	}

	*file_name = file_name_tmp;
	memcpy(*file_name, buf, count);

	/* Echo into the sysfs entry may append newline at the end of buf */
	if (buf[count - 1] == '\n')
		(*file_name)[count - 1] = '\0';
	else
		(*file_name)[count] = '\0';

	return 0;
}

#ifdef FIRMUP_ON_PROBE
/*
*   Update firmware from header file
*/
static ssize_t mxt_firmware_update(struct mxt_data *data)
{
    struct device *dev = &data->client->dev;
    int error;

    error = mxt_load_fw(dev, true);
    if (error) {
		dev_err(dev, "The firmware update failed(%d)\n", error);
    } else {
		dev_info(dev, "The firmware update succeeded\n");
		if(!update_fw_force && !mxt_mfts)
			data->suspended = false;
		error = mxt_read_info_block(data);
    }

    return error;
}
#endif

static ssize_t mxt_update_fw_store(struct mxt_data *data, const char *buf, size_t count)
{
	int error = 0;
	int value = 0;

	sscanf(buf, "%d", &value);
	update_fw_force = true;

	if(value && !(data->suspended)){
		dev_info(&data->client->dev, "Execute firmware update func\n");
		error = mxt_initialize(data);
		if (error) {
		dev_err(&data->client->dev, "Failed to initialize mxt\n");
		data->in_bootloader = false;
			goto out;
		}

		error = mxt_firmware_update(data);
		if (error) {
		dev_err(&data->client->dev, "Failed to update firmware\n");
		data->in_bootloader = false;
			goto out;
		}

		error = mxt_rest_init(data);
		if (error) {
		dev_err(&data->client->dev, "Failed to rest init\n");
			goto out;
		}
	}else
		dev_info(&data->client->dev, "Can't Execute firmware update func\n");

out:
	update_fw_force = false;
	return count;
}

static ssize_t mxt_update_cfg_store(struct mxt_data *data, const char *buf, size_t count)
{
	int ret = 0;
	int value = 0;

	sscanf(buf, "%d", &value);
	dev_info(&data->client->dev, "Update mxt Configuration.\n");

	if (data->in_bootloader) {
		dev_err(&data->client->dev, "Not in appmode\n");
		return -EINVAL;
	}

	if(value && !(data->suspended)){
		dev_info(&data->client->dev, "Update mxt Configuration Start.\n");
		data->enable_reporting = false;
		update_cfg_force = true;
		mxt_free_object_table(data);
		mxt_read_info_block(data);
		ret = mxt_configure_objects(data);
		if (ret){
			dev_err(&data->client->dev, "Update mxt Configuration Fail!\n");
			goto out;
		}
	}else{
		dev_err(&data->client->dev, "Update isn't excuted!\n");
		ret = count;
		goto out;
	}

	ret = count;
	dev_info(&data->client->dev, "Update mxt Configuration Success.\n");
out:
	data->enable_reporting = true;
	update_cfg_force = false;
	return ret;
}

static ssize_t mxt_debug_enable_show(struct mxt_data *data, char *buf)
{
	int count;
	char c;

	c = data->debug_enabled ? '1' : '0';
	count = sprintf(buf, "%c\n", c);

	return count;
}

static ssize_t mxt_debug_enable_store(struct mxt_data *data, const char *buf, size_t count)
{
	int i;

	if (sscanf(buf, "%u", &i) == 1 && i < 2) {
		data->debug_enabled = (i == 1);

		dev_dbg(&data->client->dev, "%s\n", i ? "debug enabled" : "debug disabled");
		return count;
	} else {
		dev_dbg(&data->client->dev, "debug_enabled write error\n");
		return -EINVAL;
	}
}

static int mxt_check_mem_access_params(struct mxt_data *data, loff_t off,
				       size_t *count)
{
	data->mem_size = 32768;

	if (off >= data->mem_size)
		return -EIO;

	if (off + *count > data->mem_size)
		*count = data->mem_size - off;

	if (*count > MXT_MAX_BLOCK_WRITE)
		*count = MXT_MAX_BLOCK_WRITE;

	return 0;
}

static ssize_t mxt_mem_access_read(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct mxt_data *data = dev_get_drvdata(dev);
	int ret = 0;

	ret = mxt_check_mem_access_params(data, off, &count);
	if (ret < 0)
		return ret;

	if (count > 0)
		ret = __mxt_read_reg(data->client, off, count, buf);

	return ret == 0 ? count : ret;
}

static ssize_t mxt_mem_access_write(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off,
	size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct mxt_data *data = dev_get_drvdata(dev);
	int ret = 0;

	ret = mxt_check_mem_access_params(data, off, &count);
	if (ret < 0)
		return ret;

	if (count > 0)
		ret = __mxt_write_reg(data->client, off, count, buf);

	return ret == 0 ? count : 0;
}
#ifdef MXT_GESTURE_RECOGNIZE
static ssize_t mxt_knock_on_store(struct mxt_data *data, const char *buf, size_t size)
{
	/* gesture mode control only lpwg_notify */
	return size;
}
#endif

static ssize_t store_quick_cover_status(struct mxt_data *data, const char *buf, size_t size)
{
	int value;
	int mode = 0;
	int ret = 0;
	dev_info(&data->client->dev, "%s\n", __func__);

	sscanf(buf, "%d", &value);

	if( (value == 1) && (quick_cover_status == 0) ){
		quick_cover_status = 1;
		mode = 0;
	} else if( (value == 0) && (quick_cover_status == 1) ){
		quick_cover_status = 0;
		mode = 25;
	} else
		return size;

	ret = mxt_write_reg(data->client, data->T47_address, mode);
	if (ret) {
		dev_err(&data->client->dev, "quick cover pen mode change error. status(T47[0]))\n");
	}
	dev_info(&data->client->dev, "quick cover status = %s, pen mode %d\n", (quick_cover_status == 1) ? "QUICK_COVER_ON(CLOSED)" : "QUICK_COVER_OFF(OPEN)", mode);
#ifdef TSP_PATCH
	if(quick_cover_status == 0)
		mxt_patch_goto_stage(data, value);
#endif
	return size;
}

#ifdef TSP_PATCH
static ssize_t mxt_load_patch_from_ums(struct mxt_data *data, const char *buf, size_t size)
{
	struct device *dev = &data->client->dev;
	struct file *filp = NULL;
	struct firmware fw;
	mm_segment_t old_fs = {0};
	u8 *patch_data;
	const char *firmware_name = "patch.bin";
	char *fw_path;
	int ret = 0;
	int value;

	sscanf(buf, "%d", &value);
	if(!value){
		dev_err(dev, "Invaild command.\n");
		return size;
	}
	memset(&fw, 0, sizeof(struct firmware));

	fw_path = kzalloc(MXT_MAX_FW_PATH, GFP_KERNEL);
	if (fw_path == NULL) {
		dev_err(dev, "Failed to allocate firmware path.\n");
		return -ENOMEM;
	}

	snprintf(fw_path, MXT_MAX_FW_PATH, "/sdcard/%s", firmware_name);

	old_fs = get_fs();
	set_fs(get_ds());

	filp = filp_open(fw_path, O_RDONLY, 0);
	if (IS_ERR(filp)) {
		dev_err(dev, "Could not open firmware: %s,%d\n",
			fw_path, (s32)filp);
		ret = -ENOENT;
		goto err_open;
	}

	fw.size = filp->f_path.dentry->d_inode->i_size;

	patch_data = kzalloc(fw.size, GFP_KERNEL);
	if (!patch_data) {
		dev_err(dev, "Failed to alloc buffer for fw\n");
		ret = -ENOMEM;
		goto err_alloc;
	}
	ret = vfs_read(filp, (char __user *)patch_data, fw.size, &filp->f_pos);
	if (ret != fw.size) {
		dev_err(dev, "Failed to read file %s (ret = %d)\n",
			fw_path, ret);
		ret = -EINVAL;
		goto err_alloc;
	}
	fw.data = patch_data;
	data->patch.patch = patch_data;

	dev_info(dev, "%s patch file size:%d\n", __func__, fw.size);
	
	dev_info(dev, "%s ppatch:%p %p\n", __func__, patch_data, data->patch.patch);
	ret = mxt_patch_init(data, data->patch.patch);
	if(ret)
		dev_err(dev, "mxt patch init error.\n");
	ret = 0;

err_alloc:
	filp_close(filp, current->files);
err_open:
	set_fs(old_fs);
	kfree(fw_path);

	return size;
}
#endif

static ssize_t show_mfts_fw_ver(struct mxt_data *data, char *buf)
{
	int ret = 0;
	u8 build = 0;	/* 3.0 - 170(AA) */
	mxt_mfts = true;
	mxt_mfts_for_mxt_start = true;
	dev_info(&data->client->dev, "Show MFTS ver!\n");
	#if 1	//kyutae
	mxt_t6_command(data, MXT_COMMAND_RESET, MXT_RESET_VALUE, false);
	printk("lge_touch mxt_t6_command!! soft reset !!!\n");
	#endif

	ret = __mxt_read_reg(data->client, 0x03, 1, &build);
	if(ret)
		sprintf(buf, "Fail read Buid info\n");

	if (build == 0xAA) {
		data->T6_address = MFTS_T6_ADDRESS;
	} else {
		data->T6_address = ORIGINAL_T6_ADDRESS;
	}
	if (config_crc_mfts == 0) {
		printk("lge_touch config_crc_mfts == 0 at %s\n", __func__);
		mxt_update_crc(data, MXT_COMMAND_REPORTALL, 1);
	}
	dev_info(&data->client->dev, "Show MFTS ver! Config 0x%06X\n", data->config_crc);
	config_crc_mfts = data->config_crc;
	ret = sprintf(buf, "0x%06X\n", data->config_crc);
	mxt_mfts = false;
	return ret;
}

static ssize_t show_mfts_fw(struct mxt_data *data, char *buf)
{
	int ret = 0;
	u8 build = 0;	/* 3.0 - 170(AA) */
	mxt_mfts = true;
	dev_info(&data->client->dev, "Config Update - MFTS\n");

	ret = __mxt_read_reg(data->client, 0x03, 1, &build);
	if(ret){
		sprintf(buf, "Fail read Buid info\n");
		goto out;
	}

	if(!data->suspended){
		dev_info(&data->client->dev, "Update mxt Configuration Start. build - 0x%x \n", build);
		data->enable_reporting = false;
		update_cfg_force = true;
		mxt_free_object_table(data);
		mxt_read_info_block(data);
		ret = mxt_configure_objects(data);
		if (ret){
			dev_err(&data->client->dev, "Update mxt Configuration Fail!\n");
			goto out;
		}
	}else{
		dev_err(&data->client->dev, "Update isn't excuted! build - 0x%x \n", build);
		goto out;
	}

	dev_info(&data->client->dev, "Update mxt Configuration Success.\n");
out:
	data->enable_reporting = true;
	update_cfg_force = false;
	mxt_mfts = false;
	mxt_mfts_30 = false;
	return ret;
}

static ssize_t show_mfts(struct mxt_data *data, char *buf)
{
	int ret = 0;
	u8 build = 0;
	mxt_mfts = true;
	dev_info(&data->client->dev, "MFTS!\n");

	ret = __mxt_read_reg(data->client, 0x03, 1, &build);
	if(ret)
		sprintf(buf, "Fail read Buid info\n");

	if (build == 0xAA) {
		data->T6_address = MFTS_T6_ADDRESS;
		mxt_mfts_30 = true;
	} else {
		data->T6_address = ORIGINAL_T6_ADDRESS;
		mxt_mfts_30 = false;
	}

	if (config_crc_mfts == 0) {
		mxt_update_crc(data, MXT_COMMAND_REPORTALL, 1);
		printk("lge_touch config_crc_mfts == 0 at %s\n", __func__);
	}
	config_crc_mfts = data->config_crc;
	printk("lge_touch config_crc_mfts == 0x%x at %s\n", config_crc_mfts, __func__);

	if (build == 0xAA) {
		if(data->config_crc == MXT_LATEST_CONFIG_CRC_MFTS) {
			ret = sprintf(buf, "0");
		}else{
			ret = sprintf(buf, "1");
		}
	} else {
	#if 0	//kyutae
		ret = sprintf(buf, "0");
	#else
		/*if(data->config_crc == MXT_LATEST_CONFIG_CRC || data->config_crc == 0xB16618) {*/
		if(data->config_crc == 0) {
			ret = sprintf(buf, "1");
		}else{
			ret = sprintf(buf, "0");
		}
	#endif
	}
	dev_info(&data->client->dev, "Show MFTS! Config 0x%06X\n", data->config_crc);
	mxt_mfts = false;
	return ret;
}

static void change_ime_drumming_func(struct work_struct *work_ime_drumming)
{
	struct mxt_data *data = container_of(to_delayed_work(work_ime_drumming), struct mxt_data, work_ime_drumming);
	int ret = 0;
	u8 enable;
	u8 jump_limit = 15;

	if(data->power_status == MXT_POWER_OFF)
		return;

	if (ime_drumming_status && !data->suspended){
		enable = 0;
		jump_limit = 15;
	} else {
		enable = 35;
		if(touch_test_dev->charging_mode){ //TA mode
			jump_limit = 15;
		} else {
			jump_limit = 45;
		}
	}

	ret = mxt_write_reg(data->client, data->T100_address + 43, jump_limit);
	if (ret) {
		dev_err(&data->client->dev, "change_ime_drumming_func error. jump_limit(T100[43]))\n");
	}
	dev_dbg(&data->client->dev, "change_ime_drumming_func. Jump Limit %d\n", jump_limit);

	ret = mxt_write_reg(data->client, data->T42_address, enable);
	if (ret) {
		dev_err(&data->client->dev, "change_ime_drumming_func error. Palm Detect(T42))\n");
	}
	dev_dbg(&data->client->dev, "change_ime_drumming_func. Palm detect %d\n", enable);
}

static ssize_t show_ime_drumming_status(struct mxt_data *data, char *buf)
{
	int ret = 0;

	ret += sprintf(buf+ret, "%s\n", (ime_drumming_status == 1) ? "IME_ON" : "IME_OFF");
	return ret;
}

static ssize_t store_ime_drumming_status(struct mxt_data *data, const char *buf, size_t count)
{
	int value;
	sscanf(buf, "%d", &value);

	if( (value == 1) && (ime_drumming_status == 0) ) {
		ime_drumming_status = 1;
		queue_delayed_work(touch_wq, &data->work_ime_drumming, msecs_to_jiffies(10));
	}
	else if( (value == 0) && (ime_drumming_status == 1) ) {
		ime_drumming_status = 0;
		queue_delayed_work(touch_wq, &data->work_ime_drumming, msecs_to_jiffies(10));
	}
	else {
		return count;
	}

	dev_info(&data->client->dev, "%s ime status = %s\n", __func__, (ime_drumming_status == 1) ? "IME_ON" : "IME_OFF");
	return count;
}

static ssize_t store_keyguard_info(struct mxt_data *data, const char *buf, size_t count)
{
	int value;
	sscanf(buf, "%d", &value);

	if(value == KEYGUARD_RESERVED) {
		if(ime_drumming_status) {
			queue_delayed_work(touch_wq, &data->work_ime_drumming, msecs_to_jiffies(10));
		}
	}

	dev_info(&data->client->dev, "%s KEYGUARD = %d\n", __func__, value);
	return count;
}

#ifdef MXT_LPWG
static int gesture_control(struct mxt_data *data, int on);
static void mxt_lpwg_enable(struct mxt_data *data, u32 value)
{
	if(value == LPWG_DOUBLE_TAP){
		data->mxt_knock_on_enable= 1;
		data->mxt_password_enable= 0;
		dev_info(&data->client->dev, "Knock On Enable\n");
	}else if(value == LPWG_PASSWORD){
		data->mxt_knock_on_enable= 0;
		data->mxt_password_enable= 1;
		dev_info(&data->client->dev, "PassWord Enable\n");
	}else{
		dev_err(&data->client->dev, "Unknown Value. Not Setting\n");
		return;
	}

	if(data->power_status == MXT_POWER_CFG_DEEPSLEEP)
		mxt_set_t7_power_cfg(data, MXT_POWER_CFG_KNOCKON);
	else if(data->power_status == MXT_POWER_OFF)
		mxt_regulator_enable(data);

	if (touch_enable_irq_wake(data->irq) != 0){
		dev_err(&data->client->dev, "Gesture wakeup irq enable fail.\n");
	}
	touch_enable_irq(data->irq);

	gesture_control(data, 1); //suspend status
}

static void mxt_lpwg_disable(struct mxt_data *data, u32 value)
{
	if(value == LPWG_NONE){
		data->mxt_knock_on_enable= 0;
		data->mxt_password_enable= 0;
		dev_info(&data->client->dev, "KnockOn/PassWord Gesture Disable\n");
	}else{
		dev_err(&data->client->dev, "Unknown Value. Not Setting\n");
		return;
	}

	if (touch_disable_irq_wake(data->irq) != 0){
		dev_err(&data->client->dev, "Gesture wakeup irq disable fail.\n");
	}

	if(data->power_status == MXT_POWER_ON || data->power_status == MXT_POWER_CFG_RUN || data->power_status == MXT_POWER_CFG_KNOCKON){
		touch_disable_irq(data->irq);
		cancel_work_sync(&data->work);
#ifdef I2C_SUSPEND_WORKAROUND
		cancel_delayed_work_sync(&data->check_suspended_work);
#endif
		cancel_delayed_work_sync(&data->work_ime_drumming);
#ifdef TSP_PATCH
		mxt_set_t7_power_cfg(data, MXT_POWER_CFG_DEEPSLEEP);
#else
		;
#endif
	}else{
		dev_err(&data->client->dev, "Power off already.\n");
	}
}

static void mxt_lpwg_control(struct mxt_data *data, u32 value, bool onoff)
{
	struct input_dev *input_dev = data->input_dev;

	dev_info(&data->client->dev, "%s [%s]\n", __func__, data->suspended ? "SLEEP" : "WAKEUP");

	if (data->in_bootloader){
		dev_info(&data->client->dev, "%s : Fw upgrade mode.\n", __func__);
		return;
	}

	mutex_lock(&input_dev->mutex);

	if(onoff == 1){
		mxt_lpwg_enable(data, value);
	}else{
		mxt_lpwg_disable(data, value);
	}

	mutex_unlock(&input_dev->mutex);
}

int tci_write_tap_cnt(struct i2c_client *client, int value)
{
	struct mxt_data *data = i2c_get_clientdata(client);
	struct mxt_object *object;

	int offset = 17;

	object = mxt_get_object(data, MXT_T93_NEW);
	if (!object) {
		TOUCH_ERR_MSG("error Cannot get object_type T%d\n", MXT_T93_NEW);
		return -EINVAL;
	}
	if ( (mxt_obj_size(object) == 0) || (object->start_address == 0)) {
		TOUCH_ERR_MSG("error object_type T%d\n", object->type);
		return -ENODEV;
	}

	DO_IF(__mxt_write_reg(data->client, object->start_address + offset, 1, &value), error);
	TOUCH_INFO_MSG("Write TCI tap count T%d %d %d\n", MXT_T93_NEW, offset, value);

	return NO_ERROR;

error:
	return ERROR;
}

#ifdef MXT_LPWG
static void lpwg_early_suspend(struct mxt_data *data)
{
	dev_info(&data->client->dev, "%s Start\n", __func__);
    mutex_lock(&mxt_early_mutex);
	mxt_reset_slots(data);
    mutex_unlock(&mxt_early_mutex);
	switch (data->lpwg_mode) {
			case LPWG_DOUBLE_TAP:
				data->mxt_knock_on_enable= 1;
				data->is_lpwg_report_enable = 1;
				gesture_control(data, 1);
				break;
			case LPWG_PASSWORD:
				data->mxt_password_enable = 1;
				data->is_lpwg_report_enable = 1;
				gesture_control(data, 1);
				break;
			default:
				data->ta_status = MXT_DEEPSLEEP_MODE;
				break;
	}
	data->suspended = true;
	dev_info(&data->client->dev, "%s End\n", __func__);
}
static void lpwg_late_resume(struct mxt_data *data)
{
	dev_info(&data->client->dev, "%s Start\n", __func__);

	data->suspended = false;
	data->is_lpwg_report_enable = 0;
	data->mxt_knock_on_enable= 0;
	data->mxt_password_enable = 0;
	gesture_control(data, 0);

	if(!data->mxt_suspended){
		dev_info(&data->client->dev, "Recovery IRQ status!!!\n");
		touch_enable_irq(data->irq);
	}

	memset(g_tci_press, 0, sizeof(g_tci_press));
	memset(g_tci_report, 0, sizeof(g_tci_report));

	dev_info(&data->client->dev, "%s End\n", __func__);
}
#endif

err_t atmel_ts_lpwg(struct i2c_client* client, u32 code, u32 value, struct point *tci_point)
{
	struct mxt_data *data = i2c_get_clientdata(client);
	int i;
	if(code == 2 || code == 9 || code == 10 || code == 12)
		TOUCH_INFO_MSG("Lpwg Code: %d Value: %d\n", code, value);

	switch (code) {
	case LPWG_READ:
		if (data->mxt_password_enable) {
			if( (g_tci_press[0].x == -1) && (g_tci_press[0].x == -1))	//Error state : Tap count over
			{
				TOUCH_INFO_MSG("Tap count error \n");
				tci_point[0].x = 1;
				tci_point[0].y = 1;

				tci_point[1].x = -1;
				tci_point[1].y = -1;
			}
			else
			{
				for(i = 0; i < g_tap_cnt ; i++) {
					tci_point[i].x = g_tci_report[i].x;
					tci_point[i].y = g_tci_report[i].y;
				}
				// '-1' should be assinged to the last data.
				tci_point[g_tap_cnt].x = -1;
				tci_point[g_tap_cnt].y = -1;
			}

			// Each data should be converted to LCD-resolution.
			// B1 already set LCD-resolution when suspend mode
		}
		break;
	case LPWG_ENABLE:
		data->lpwg_mode = value;
		// The 'lpwg_mode' is changed to 'value' but it is applied in suspend-state.
		if(!data->suspended)
			break;

		if(value)
			mxt_lpwg_control(data, value, true);
		else
			mxt_lpwg_control(data, value, false);

		data->is_lpwg_report_enable = 1;
		break;
	case LPWG_LCD_X:
	case LPWG_LCD_Y:
		// If touch-resolution is not same with LCD-resolution,
		// position-data should be converted to LCD-resolution.
		break;
	case LPWG_ACTIVE_AREA_X1:
		data->qwindow_size->x_min = value;
		break;
	case LPWG_ACTIVE_AREA_X2:
		data->qwindow_size->x_max = value;
		break;
	case LPWG_ACTIVE_AREA_Y1:
		data->qwindow_size->y_min = value;
		break;
	case LPWG_ACTIVE_AREA_Y2:
		data->qwindow_size->y_max = value;
		break;
		// Quick Cover Area
	case LPWG_TAP_COUNT:
		// Tap Count Control . get from framework write to IC
		g_tap_cnt = value;
		DO_IF(tci_write_tap_cnt(client, value), error);
		break;
	case LPWG_REPLY:
		// Do something, if you need.
		if(value == 0 && data->lpwg_mode != 1){	/* password fail */
			data->is_lpwg_report_enable = 1;
#ifdef WAITED_UDF
			mxt_set_t7_power_cfg(data, MXT_POWER_CFG_KNOCKON);
#else
			write_partial_configs(data, UDF_on_configs_);
			TOUCH_INFO_MSG("T93 ENABLE LPWG \n");
#endif
		}
		//mxt_process_messages_until_invalid(data);

		//wake_unlock(&touch_wake_lock);	/* knock on, password wake unlock */
		break;
	case LPWG_LENGTH_BETWEEN_TAP:
		break;
	case LPWG_EARLY_MODE:
		if(value == 0)
			lpwg_early_suspend(data);
		else if(value == 1)
			lpwg_late_resume(data);
		break;
	default:
		break;
	}

	return NO_ERROR;
error:
	return ERROR;
}

/* Sysfs - lpwg_data (Low Power Wake-up Gesture)
 *
 * read : "x1 y1\n x2 y2\n ..."
 * write
 * 1 : ENABLE/DISABLE
 * 2 : LCD SIZE
 * 3 : ACTIVE AREA
 * 4 : TAP COUNT
 */
static struct point lpwg_data[MAX_POINT_SIZE_FOR_LPWG];
static ssize_t show_lpwg_data(struct mxt_data *data, char *buf)
{
	int i = 0, ret = 0;

	memset(lpwg_data, 0, sizeof(struct point)*MAX_POINT_SIZE_FOR_LPWG);
	atmel_ts_lpwg(data->client, LPWG_READ, 0, lpwg_data);
	for (i = 0; i < MAX_POINT_SIZE_FOR_LPWG; i++) {
		if (lpwg_data[i].x == -1 && lpwg_data[i].y == -1)
			break;
		ret += sprintf(buf+ret, "%d %d\n", lpwg_data[i].x, lpwg_data[i].y);
	}
	return ret;
}

static ssize_t store_lpwg_data(struct mxt_data *data, const char *buf, size_t count)
{
	//struct mxt_data *data = i2c_get_clientdata(client);
	int reply = 0;

	sscanf(buf, "%d", &reply);

	atmel_ts_lpwg(data->client, LPWG_REPLY, reply, NULL);

	//atomic_set(&data->state.uevent_state, UEVENT_IDLE);
	wake_unlock(&touch_wake_lock);

	return count;
}

/* Sysfs - lpwg_notify (Low Power Wake-up Gesture)
 *
 */
static ssize_t store_lpwg_notify(struct mxt_data *data, const char *buf, size_t count)
{
	int type = 0;
	int value[4] = {0};

	sscanf(buf, "%d %d %d %d %d", &type, &value[0], &value[1], &value[2], &value[3]);

	switch(type){
	case 1 :
		atmel_ts_lpwg(data->client, LPWG_ENABLE, value[0], NULL);
		break;
	case 2 :
		atmel_ts_lpwg(data->client, LPWG_LCD_X, value[0], NULL);
		atmel_ts_lpwg(data->client, LPWG_LCD_Y, value[1], NULL);
		break;
	case 3 :
		atmel_ts_lpwg(data->client, LPWG_ACTIVE_AREA_X1, value[0], NULL);
		atmel_ts_lpwg(data->client, LPWG_ACTIVE_AREA_X2, value[1], NULL);
		atmel_ts_lpwg(data->client, LPWG_ACTIVE_AREA_Y1, value[2], NULL);
		atmel_ts_lpwg(data->client, LPWG_ACTIVE_AREA_Y2, value[3], NULL);
		break;
	case 4 :
		atmel_ts_lpwg(data->client, LPWG_TAP_COUNT, value[0], NULL);
		break;
	case 5:
		atmel_ts_lpwg(data->client, LPWG_LENGTH_BETWEEN_TAP, value[0], NULL);
		break;
	case 6 :
		atmel_ts_lpwg(data->client, LPWG_EARLY_MODE, value[0], NULL);
		break;
	default:
		break;
		}
	return count;
}
#endif

static ssize_t store_use_quick_window(struct mxt_data *data, const char *buf, size_t size)
{
	int value;

	dev_info(&data->client->dev, "%s\n", __func__);

	sscanf(buf, "%d", &value);

	if( (value == 1) && (use_quick_window == 0) )
		use_quick_window = 1;
	else if( (value == 0) && (use_quick_window == 1) )
		use_quick_window = 0;
	else
		return size;

	dev_info(&data->client->dev, "use quick window = %s\n", (use_quick_window == 1) ? "USE_QUICK_WIN" : "UNUSE_QUICK_WIN");

	return size;
}

static LGE_TOUCH_ATTR(fw_ver, S_IRUGO, mxt_fw_ver_show, NULL);
static LGE_TOUCH_ATTR(hw_version, S_IRUGO, mxt_hw_version_show, NULL);
static LGE_TOUCH_ATTR(mxt_info, S_IRUGO, mxt_mxt_info_show, NULL);
static LGE_TOUCH_ATTR(self_test, S_IRUGO | S_IWUSR, mxt_selftest_show, mxt_selftest_store);
static LGE_TOUCH_ATTR(object, S_IRUGO, mxt_object_show, NULL);
static LGE_TOUCH_ATTR(object_ctrl, S_IWUSR, NULL, mxt_object_control);
static LGE_TOUCH_ATTR(update_fw, S_IWUSR, NULL, mxt_update_fw_store);
static LGE_TOUCH_ATTR(update_cfg, S_IWUSR, NULL, mxt_update_cfg_store);
static LGE_TOUCH_ATTR(debug_enable, S_IWUSR | S_IRUSR, mxt_debug_enable_show,
		   mxt_debug_enable_store);
#ifdef MXT_GESTURE_RECOGNIZE
static LGE_TOUCH_ATTR(touch_gesture,S_IRUGO | S_IWUSR, NULL, mxt_knock_on_store);
#endif
static LGE_TOUCH_ATTR(quick_cover_status,S_IRUGO | S_IWUSR, NULL, store_quick_cover_status);
#ifdef TSP_PATCH
static LGE_TOUCH_ATTR(update_patch, S_IWUSR, NULL, mxt_load_patch_from_ums);
#endif
static LGE_TOUCH_ATTR(mfts_fw_ver, S_IRUGO, show_mfts_fw_ver, NULL);
static LGE_TOUCH_ATTR(mfts_fw, S_IRUGO, show_mfts_fw, NULL);
static LGE_TOUCH_ATTR(mfts, S_IRUGO, show_mfts, NULL);
static LGE_TOUCH_ATTR(ime_status, S_IRUGO | S_IWUSR, show_ime_drumming_status, store_ime_drumming_status);
static LGE_TOUCH_ATTR(keyguard, S_IRUGO | S_IWUSR, NULL, store_keyguard_info);
#ifdef	MXT_LPWG
static LGE_TOUCH_ATTR(lpwg_data, S_IRUGO | S_IWUSR, show_lpwg_data, store_lpwg_data);
static LGE_TOUCH_ATTR(lpwg_notify, S_IRUGO | S_IWUSR, NULL, store_lpwg_notify);
#endif
static LGE_TOUCH_ATTR(use_quick_window, S_IRUGO | S_IWUSR, NULL, store_use_quick_window);

static struct attribute *lge_touch_attribute_list[] = {
	&lge_touch_attr_fw_ver.attr,
	&lge_touch_attr_hw_version.attr,
	&lge_touch_attr_mxt_info.attr,
	&lge_touch_attr_self_test.attr,
	&lge_touch_attr_object.attr,
	&lge_touch_attr_object_ctrl.attr,
	&lge_touch_attr_update_fw.attr,
	&lge_touch_attr_update_cfg.attr,
	&lge_touch_attr_debug_enable.attr,
#ifdef MXT_GESTURE_RECOGNIZE
	&lge_touch_attr_touch_gesture.attr,
#endif
	&lge_touch_attr_quick_cover_status.attr,
#ifdef TSP_PATCH
	&lge_touch_attr_update_patch.attr,
#endif
	&lge_touch_attr_mfts_fw_ver.attr,
	&lge_touch_attr_mfts_fw.attr,
	&lge_touch_attr_mfts.attr,
	&lge_touch_attr_ime_status.attr,
	&lge_touch_attr_keyguard.attr,
#ifdef	MXT_LPWG
	&lge_touch_attr_lpwg_data.attr,
	&lge_touch_attr_lpwg_notify.attr,
#endif
	&lge_touch_attr_use_quick_window.attr,
	NULL
};

/* lge_touch_attr_show / lge_touch_attr_store
 *
 * sysfs bindings for lge_touch
 */
static ssize_t lge_touch_attr_show(struct kobject *lge_touch_kobj, struct attribute *attr,
			     char *buf)
{
	struct mxt_data *ts =
			container_of(lge_touch_kobj, struct mxt_data, lge_touch_kobj);
	struct lge_touch_attribute *lge_touch_priv =
		container_of(attr, struct lge_touch_attribute, attr);
	ssize_t ret = 0;

	if (lge_touch_priv->show)
		ret = lge_touch_priv->show(ts, buf);

	return ret;
}

static ssize_t lge_touch_attr_store(struct kobject *lge_touch_kobj, struct attribute *attr,
			      const char *buf, size_t count)
{
	struct mxt_data *ts =
			container_of(lge_touch_kobj, struct mxt_data, lge_touch_kobj);
	struct lge_touch_attribute *lge_touch_priv =
		container_of(attr, struct lge_touch_attribute, attr);
	ssize_t ret = 0;

	if (lge_touch_priv->store)
		ret = lge_touch_priv->store(ts, buf, count);

	return ret;
}

static const struct sysfs_ops lge_touch_sysfs_ops = {
	.show	= lge_touch_attr_show,
	.store	= lge_touch_attr_store,
};

static struct kobj_type lge_touch_kobj_type = {
	.sysfs_ops		= &lge_touch_sysfs_ops,
	.default_attrs 	= lge_touch_attribute_list,
};

static struct sysdev_class lge_touch_sys_class = {
	.name = MXT_DEVICE_NAME,
};

static struct sys_device lge_touch_sys_device = {
	.id		= 0,
	.cls	= &lge_touch_sys_class,
};

static void mxt_reset_slots(struct mxt_data *data)
{
	struct input_dev *input_dev = data->input_dev;
	int id;

	if (data->ts_data.prev_total_num) {
		for (id = 0; id < data->pdata->numtouch; id++) {
			if (data->ts_data.prev_data[id].status != FINGER_RELEASED) {
				input_mt_slot(input_dev, id);
				input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
				dev_info(&data->client->dev, "Release id %d finger release! prev status : %d\n", id, data->ts_data.prev_data[id].status);
			}
		}

		mxt_input_sync(input_dev);
		memset(&data->ts_data, 0, sizeof(data->ts_data));

		dev_info(&data->client->dev, "Release all touch event!\n");
	} else {
		dev_info(&data->client->dev, "Not Release any touch event!\n");
	}
}

static void safety_reset(struct mxt_data *data)
{
	touch_disable_irq(data->irq);
	mxt_reset_slots(data);
	mxt_regulator_disable(data);
	mxt_regulator_enable(data);
	if(touch_probe_ok){
		msleep(100);
		dev_info(&data->client->dev, "Safety Reset Recovery Value! Tap Count : %d, Stage : %d\n", g_tap_cnt, data->ta_status);
		__mxt_write_reg(data->client, data->T93_address + 17, 1, &g_tap_cnt);
		mxt_patch_test_event(data, data->ta_status);
	}
	touch_enable_irq(data->irq);
	return;
}

#ifdef MXT_GESTURE_RECOGNIZE
int write_partial_configs(struct mxt_data *ts, const u8** configs)
{
	struct mxt_object *object;
	int i, j;
	int ret = 0;

	for (i = 0; configs[i][0] != MXT_RESERVED_T255; i++) {
		object = mxt_get_object(ts, configs[i][0]);
		if (!object) {
			dev_err(&ts->client->dev,"error Cannot get object_type T%d\n", configs[i][0]);
			return -EINVAL;
		}
		if ( (mxt_obj_size(object) == 0) || (object->start_address == 0)) {
			dev_err(&ts->client->dev, "error object_type T%d\n", object->type);
			return -ENODEV;
		}

		for (j = 1; j <= configs[i][1]; j++) {
			dev_dbg(&ts->client->dev, "write config data T%d offset(%d) value(%d)\n",
					configs[i][0],
					configs[i][j * 2],
					configs[i][j * 2 + 1] );
			ret = __mxt_write_reg(ts->client,
				object->start_address+ configs[i][j * 2],
				1, &configs[i][j * 2 + 1]);

			if (ret) {
				dev_err(&ts->client->dev, "Failed to write configuration\n");
				return -EIO;
			}
		}
	}
	return ret;
}
static int gesture_control(struct mxt_data *data, int on)
{
#ifdef TSP_PATCH
	if(is_probing)
		return NO_ERROR;

	if(on) { 	//DoubleTap_ON_When_suspend
#ifdef MXT_FACTORY
		if(factorymode)
			goto mode_keep;
#endif
		if (data->charging_mode) {
			if(data->lpwg_mode == LPWG_DOUBLE_TAP){
				if(data->ta_status != MXT_PATCH_KNOCKON_TA_MODE_EVENT) {
					dev_info(&data->client->dev, " KNOCKON_TA_MODE %d\n", MXT_PATCH_KNOCKON_TA_MODE_EVENT);
					data->ta_status = MXT_PATCH_KNOCKON_TA_MODE_EVENT;
					mxt_patch_test_event(data, MXT_PATCH_KNOCKON_TA_MODE_EVENT);
					goto mode_change;
				}
			}else if(data->lpwg_mode == LPWG_PASSWORD){
				if(data->ta_status != MXT_PATCH_PASSWORD_TA_MODE_EVENT) {
					dev_info(&data->client->dev, " PASSWORD_TA_MODE %d\n", MXT_PATCH_PASSWORD_TA_MODE_EVENT);
					data->ta_status = MXT_PATCH_PASSWORD_TA_MODE_EVENT;
					mxt_patch_test_event(data, MXT_PATCH_PASSWORD_TA_MODE_EVENT);
					goto mode_change;
				}
			}
		} else {
			if(data->lpwg_mode == LPWG_DOUBLE_TAP){
				if(data->ta_status != MXT_PATCH_KNOCKON_BAT_MODE_EVENT) {
					dev_info(&data->client->dev, " KNOCKON_BAT_MODE %d\n", MXT_PATCH_KNOCKON_BAT_MODE_EVENT);
					data->ta_status = MXT_PATCH_KNOCKON_BAT_MODE_EVENT;
					mxt_patch_test_event(data, MXT_PATCH_KNOCKON_BAT_MODE_EVENT);
					goto mode_change;
				}
			}else if(data->lpwg_mode == LPWG_PASSWORD){
				if(data->ta_status != MXT_PATCH_PASSWORD_BAT_MODE_EVENT) {
					dev_info(&data->client->dev, " PASSWORD_BAT_MODE %d\n", MXT_PATCH_PASSWORD_BAT_MODE_EVENT);
					data->ta_status = MXT_PATCH_PASSWORD_BAT_MODE_EVENT;
					mxt_patch_test_event(data, MXT_PATCH_PASSWORD_BAT_MODE_EVENT);
					goto mode_change;
				}
			}
		}
	}
	else {
		if (data->charging_mode) {
#ifdef MXT_FACTORY
			if(factorymode){
				if(data->ta_status == MXT_PATCH_FTM_TA_MODE_EVENT)
					goto mode_keep;
				dev_info(&data->client->dev," wake up! factroy charging mode!!!!!!!!!!!\n");
				dev_info(&data->client->dev, "  FTM_TA_MODE %d\n", MXT_PATCH_FTM_TA_MODE_EVENT);
				data->ta_status = MXT_PATCH_FTM_TA_MODE_EVENT;
				mxt_patch_test_event(data, MXT_PATCH_FTM_TA_MODE_EVENT);
				goto mode_change;
			} else
#endif
#ifdef MXT_WIRELESS
			if(!wireless){
				if(data->ta_status != MXT_PATCH_WAKEUP_TA_MODE_EVENT){
					dev_info(&data->client->dev, " WAKEUP_TA_MODE %d\n", MXT_PATCH_WAKEUP_TA_MODE_EVENT);
					data->ta_status = MXT_PATCH_WAKEUP_TA_MODE_EVENT;
					mxt_patch_test_event(data, MXT_PATCH_WAKEUP_TA_MODE_EVENT);
					goto mode_change;
				}
			} else
#endif
			{
				if(data->ta_status != MXT_PATCH_WAKEUP_WIRELESS_TA_MODE_EVENT){
					dev_info(&data->client->dev, " WAKEUP_WIRELESS_TA_MODE %d\n", MXT_PATCH_WAKEUP_WIRELESS_TA_MODE_EVENT);
					data->ta_status = MXT_PATCH_WAKEUP_WIRELESS_TA_MODE_EVENT;
					mxt_patch_test_event(data, MXT_PATCH_WAKEUP_WIRELESS_TA_MODE_EVENT);
					goto mode_change;
				}
			}
		} else {
#ifdef MXT_FACTORY
			if(factorymode){
				if(data->ta_status == MXT_PATCH_FTM_BAT_MODE_EVENT)
					goto mode_keep;
				dev_info(&data->client->dev," wake up! factroy battery  mode!!!!!!!!!!!\n");
				dev_info(&data->client->dev, "  FTM_BAT_MODE %d\n", MXT_PATCH_FTM_BAT_MODE_EVENT);
				data->ta_status = MXT_PATCH_FTM_BAT_MODE_EVENT;
				mxt_patch_test_event(data, MXT_PATCH_FTM_BAT_MODE_EVENT);
				goto mode_change;
			} else
#endif
			if(data->ta_status != MXT_PATCH_WAKEUP_BAT_MODE_EVENT){
				dev_info(&data->client->dev, " WAKEUP_BAT_MODE %d\n", MXT_PATCH_WAKEUP_BAT_MODE_EVENT);
				data->ta_status = MXT_PATCH_WAKEUP_BAT_MODE_EVENT;
				mxt_patch_test_event(data, MXT_PATCH_WAKEUP_BAT_MODE_EVENT);
				goto mode_change;
			}
		}
	}
#else
	if(on) { 	//DoubleTap_ON_When_suspend
		// Gesture ON-1. Setting LowPower Configurations
		if(0)
			DO_SAFE(write_partial_configs(data, lowpower_on_configs), error);

		// Gesture ON-2.
		if(data->mxt_knock_on_enable){
			DO_SAFE(write_partial_configs(data, knockon_on_configs_), error);
		}
#ifdef MXT_LPWG
		else if (data->mxt_password_enable){
			DO_SAFE(write_partial_configs(data, UDF_on_configs_), error);
			TOUCH_INFO_MSG("T93 ENABLE gesture \n");
		}
#endif
		msleep(MXT_WAKEUP_TIME);
	}
	else {	//DoubleTap_OFF_When_resume
		// Gesture OFF-1. Setting Normal Configurations
		if(0)
			DO_SAFE(write_partial_configs(data, lowpower_off_configs), error);

		// Gesture OFF-2. Setting Gesture Type
		DO_SAFE(write_partial_configs(data, knockon_off_configs_), error);
#ifdef MXT_LPWG
		DO_SAFE(write_partial_configs(data, UDF_off_configs_), error);
#endif
		msleep(MXT_WAKEUP_TIME);
	}
	if( 0 ){
		DO_SAFE(write_partial_configs(data, ta_configs), error);
		DO_SAFE(write_partial_configs(data, bat_configs), error);
	}

#endif
#ifndef MXT_LPWG
	dev_info(&data->client->dev,"Gesture_control Mode Change (%s)  KnockOn: %d\n",
				on ? "SUSPEND":"RESUME", data->mxt_knock_on_enable);
#endif
	return NO_ERROR;
mode_change:
	if(data->mxt_password_enable == 0){
		hrtimer_try_to_cancel(&data->multi_tap_timer);
	}
#ifdef MXT_FACTORY
	if(factorymode){
		dev_info(&data->client->dev,"[FACTORY MODE] Gesture_control Mode Change (%s)  KnockOn: %d / PASSWD : %d\n",
				on ? "SUSPEND":"RESUME", data->mxt_knock_on_enable, data->mxt_password_enable);
		return NO_ERROR;
	}

#endif
	if(!on && quick_cover_status){
		mxt_write_reg(data->client, data->T47_address, 0);
		dev_info(&data->client->dev, "Gesture_control Pen Disable!!\n");
	}
#ifdef MXT_LPWG
	dev_info(&data->client->dev,"Gesture_control Mode Change (%s)  KnockOn: %d / PASSWD : %d\n",
				on ? "SUSPEND":"RESUME", data->mxt_knock_on_enable, data->mxt_password_enable);
#endif
	return NO_ERROR;
#ifdef MXT_FACTORY
mode_keep:
#endif
	dev_info(&data->client->dev,"Gesture_control not Mode Change\n");
	return NO_ERROR;
#ifndef TSP_PATCH
error:
	return -ERROR;
#endif
}
#endif
static void mxt_start(struct mxt_data *data)
{
	if(chargerlogo){
		chargerlogo = false;
		return;
	}

	if(data->in_bootloader)
		return;

	if(data->power_status == MXT_POWER_OFF){
		mxt_regulator_enable(data);
	}

	touch_disable_irq(data->irq);
#if 1 //kyutae
	if (unlikely(mxt_mfts_for_mxt_start))
		mxt_process_messages_until_invalid(data);
#endif
	if((data->power_status != MXT_POWER_ON || data->power_status != MXT_POWER_CFG_RUN)
#ifdef TSP_PATCH
		&& (data->ta_status != MXT_PATCH_WAKEUP_TA_MODE_EVENT) && (data->ta_status != MXT_PATCH_WAKEUP_WIRELESS_TA_MODE_EVENT)
#endif
		)
#ifdef TSP_PATCH
		mxt_set_t7_power_cfg(data, MXT_POWER_CFG_RUN);
#else
	;
#endif
	else
		data->power_status = MXT_POWER_CFG_RUN;

	if(wait_change_cfg){
		dev_info(&data->client->dev, " %s : CHANGE TA MODE %d\n", __func__, data->ta_status);
#ifdef TSP_PATCH
		mxt_patch_test_event(data, data->ta_status);
#endif
		wait_change_cfg = false;
	} else {
#ifdef MXT_GESTURE_RECOGNIZE
#ifdef MXT_LPWG
		data->is_lpwg_report_enable = 0;
		data->mxt_knock_on_enable= 0;
		data->mxt_password_enable = 0;
		gesture_control(data, 0);

		memset(g_tci_press, 0, sizeof(g_tci_press));
		memset(g_tci_report, 0, sizeof(g_tci_report));
#else
		gesture_control(data, 0);
#endif
#endif
	}
	if(ime_drumming_status) {
		queue_delayed_work(touch_wq, &data->work_ime_drumming, msecs_to_jiffies(10));
	}

	data->enable_reporting = true;
#ifdef WAITED_UDF
#ifdef MXT_FACTORY
	if(!factorymode){ /* remain Disable Screen Status bit (T100[0] bit2 = 1) for disable patch in FTM mode */
		write_partial_configs(data, UDF_off_configs_resume);
	}
#else
	write_partial_configs(data, UDF_off_configs_resume);
#endif
	wake_unlock(&touch_wake_lock);
#endif
	config_crc_mfts = 0;
	//printk("lge_touch config_crc_mfts=0 at %s\n", __func__);
	/* Recalibrate since touch doesn't power off when lcd on */
	mxt_t6_command(data, MXT_COMMAND_CALIBRATE, 1, false);
	/* disabled report touch irq until probing is finished */
	if(!is_probing){
		touch_enable_irq(data->irq);
	}
	else
		TOUCH_INFO_MSG("probing isn't finished yet\n");
	do_gettimeofday(&t_ex_debug[TIME_START_TIME]);
}

static void mxt_stop(struct mxt_data *data)
{
	if(data->in_bootloader)
		return;

	dev_dbg(&data->client->dev, "%s\n", __func__);
	data->enable_reporting = false;
	touch_disable_irq(data->irq);
	cancel_work_sync(&data->work);
#ifdef I2C_SUSPEND_WORKAROUND
	cancel_delayed_work_sync(&data->check_suspended_work);
#endif
	cancel_delayed_work_sync(&data->work_ime_drumming);

#ifdef MXT_GESTURE_RECOGNIZE
#ifdef MXT_LPWG
	switch (data->lpwg_mode) {
		case LPWG_DOUBLE_TAP:
			data->mxt_knock_on_enable= 1;
			data->mxt_password_enable = 0;
			data->is_lpwg_report_enable = 1;
			gesture_control(data, 1);
			break;
		case LPWG_PASSWORD:
			data->mxt_knock_on_enable= 0;
			data->mxt_password_enable = 1;
			data->is_lpwg_report_enable = 1;
			gesture_control(data, 1);
			break;
		default:
			data->ta_status = MXT_DEEPSLEEP_MODE;
			break;
		}
#else
	gesture_control(data, 1);
#endif
#ifdef MXT_LPWG
	if (data->lpwg_mode)
#else
	if (data->mxt_knock_on_enable)
#endif
	{
#if 0
		/* Recalibrate since touch doesn't power off when lcd off */
		mxt_t6_command(data, MXT_COMMAND_CALIBRATE, 1, false);
#endif
	} else
#else
	if (data->use_regulator)
#endif
#ifdef TSP_PATCH
		mxt_set_t7_power_cfg(data, MXT_POWER_CFG_DEEPSLEEP);
#else
		;
#endif
#ifdef UDF_CONTROL_CLEAR_T37_DATA
	if(data->lpwg_mode == LPWG_PASSWORD)
		set_t93_sequence_arm(data, 0);
#endif

	mxt_reset_slots(data);
	if(ime_drumming_status) {
		queue_delayed_work(touch_wq, &data->work_ime_drumming, msecs_to_jiffies(10));
	}
#ifdef MXT_GESTURE_RECOGNIZE
#ifdef MXT_LPWG
	if (data->lpwg_mode)
#else
	if (data->mxt_knock_on_enable)
#endif
	{
		touch_enable_irq(data->irq);
	}
#endif
	mxt_process_messages_until_invalid(data);	/*i2c transaction error*/
}

static int mxt_input_open(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);
	mxt_start(data);
	return 0;
}

static void mxt_input_close(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);
	mxt_stop(data);
}

static int mxt_parse_config(struct device *dev, struct device_node *np,
				struct mxt_config_info *info)
{
	struct property *prop;
	u8 *temp_cfg;
	u32 temp_length;
	int i = 0;

	const char *config_node_name[MXT_TMAX] = {
		"atmel,config_t6"
		,"atmel,config_t38"
		,"atmel,config_t71"
		,"atmel,config_t7"
		,"atmel,config_t8"
		,"atmel,config_t15"
		,"atmel,config_t18"
		,"atmel,config_t19"
		,"atmel,config_t23"
#ifdef MXT_GESTURE_RECOGNIZE
		,"atmel,config_t24"
#endif
		,"atmel,config_t25"
		,"atmel,config_t40"
		,"atmel,config_t42"
		,"atmel,config_t46"
		,"atmel,config_t47"
		,"atmel,config_t55"
		,"atmel,config_t56"
		,"atmel,config_t61"
		,"atmel,config_t65"
		,"atmel,config_t66"
		,"atmel,config_t69"
		,"atmel,config_t70"
		,"atmel,config_t72"
		,"atmel,config_t78"
		,"atmel,config_t80"
#ifndef ALPHA_FW
		,"atmel,config_t84"
#endif
#ifdef MXT_LPWG
		,"atmel,config_t92"
		,"atmel,config_t93"
#endif
		,"atmel,config_t100"
		,"atmel,config_t101"
		,"atmel,config_t102"
		,"atmel,config_t103"
		,"atmel,config_t104"
		,"atmel,config_t105"
	};

	memset(&info->config_t, 0, sizeof(info->config_t));

	while(i < MXT_TMAX) {
		prop = of_find_property(np, config_node_name[i], &temp_length);
		if (!prop) {
			dev_err(dev, "Looking up %s property in node %s failed",
				config_node_name[i], np->full_name);
			return -ENODEV;
		} else if (!temp_length) {
			dev_err(dev, "Invalid length of %s configuration data\n", config_node_name[i]+13);
			return -EINVAL;
		}

		temp_cfg = devm_kzalloc(dev,
				temp_length * sizeof(u8), GFP_KERNEL);
		if (!temp_cfg) {
			dev_err(dev, "Unable to allocate memory to store %s cfg\n", config_node_name[i]+13);
			return -ENOMEM;
		}

		memcpy(temp_cfg, prop->value, temp_length);
		info->config_t[i] = temp_cfg;

#ifdef MXT_GESTURE_RECOGNIZE
		if(info->config_t[i][0] == 47){
			dev_dbg(dev, "%s's ctrl value is 0x%x",config_node_name[i],info->config_t[i][1]);
			t47_ctrl_cfg = info->config_t[i][1];
		}else if(info->config_t[i][0] == 65){
			dev_dbg(dev, "%s's ctrl value is 0x%x",config_node_name[i],info->config_t[i][1]);
			t65_ctrl_cfg = info->config_t[i][1];
		}else if(info->config_t[i][0] == 72){
			dev_dbg(dev, "%s's ctrl value is 0x%x",config_node_name[i],info->config_t[i][1]);
			t72_ctrl_cfg = info->config_t[i][1];
		}else if(info->config_t[i][0] == 100){
			dev_dbg(dev, "%s's ctrl value is 0x%x, scraux value is 0x%x",config_node_name[i],info->config_t[i][1],info->config_t[i][3]);
			t100_ctrl_cfg = info->config_t[i][1];
		}
#endif
		dev_dbg(dev, "%s config :%d\n", config_node_name[i]+13, info->config_t[i][0]);
		i++;
	}
	return 0;
}



static int mxt_parse_dt(struct device *dev, struct mxt_platform_data *pdata)
{
	struct mxt_config_info *info;
	struct device_node *temp;
	struct device_node *node = dev->of_node;
	int rc = 0;
	u32 temp_val;

	/* reset, irq gpio info */
	if (node == NULL)
		return -ENODEV;
	
	pdata->gpio_reset= of_get_named_gpio_flags(node, "atmel,reset-gpio", 0, NULL);
	pdata->gpio_int = of_get_named_gpio_flags(node, "atmel,irq-gpio", 0, NULL);
	
	rc = of_property_read_u32(node, "atmel,numtouch", &temp_val);
	if(rc){
		dev_err(dev, "Unable to read numtouch\n");
		return rc;
	}else
		pdata->numtouch = temp_val;
	
	rc = of_property_read_u32(node, "atmel,max_x", &temp_val);
	if(rc){
		dev_err(dev, "Unable to read max_x\n");
		return rc;
	}else
		pdata->max_x = temp_val;

	rc = of_property_read_u32(node, "atmel,max_y", &temp_val);
	if(rc){
		dev_err(dev, "Unable to read max_y\n");
		return rc;
	}else
		pdata->max_y = temp_val;
	
	rc = of_property_read_u32(node, "atmel,irqflags", &temp_val);
	if(rc){
		dev_err(dev, "Unable to read irqflags\n");
		return rc;
	}else
		pdata->irqflags = temp_val;

	rc = of_property_read_u32(node, "atmel,t19_num_keys", &temp_val);
	if(rc){
		dev_err(dev, "Unable to read t19_num_keys\n");
		return rc;
	}else
		pdata->t19_num_keys = temp_val;

	/* config array size */
	pdata->config_array_size = 0;
	temp = NULL;
	while ((temp = of_get_next_child(node, temp)))
		pdata->config_array_size++;

	dev_dbg(dev, "config array size is '%d'\n",pdata->config_array_size);

	if (!pdata->config_array_size){
		dev_err(dev, "config array size is '0'\n");
		return 0;
	}

	info = devm_kzalloc(dev, pdata->config_array_size *
				sizeof(struct mxt_config_info), GFP_KERNEL);
	if (!info) {
		dev_err(dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}
	
	pdata->config_array  = info;

	for_each_child_of_node(node, temp) {
		rc = mxt_parse_config(dev, temp, info);
		if (rc) {
			dev_err(dev, "Unable to parse config data\n");
			return rc;
		}
		info++;
	}
	return 0;

}


static int mxt_handle_pdata(struct mxt_data *data)
{
	data->pdata = dev_get_platdata(&data->client->dev);

	/* Use provided platform data if present */
	if (data->pdata) {
		if (data->pdata->cfg_name)
			mxt_update_file_name(&data->client->dev,
					     &data->cfg_name,
					     data->pdata->cfg_name,
					     strlen(data->pdata->cfg_name));

		return 0;
	}

	data->pdata = kzalloc(sizeof(*data->pdata), GFP_KERNEL);
	if (!data->pdata) {
		dev_err(&data->client->dev, "Failed to allocate pdata\n");
		return -ENOMEM;
	}

	/* Set default parameters */
	data->pdata->irqflags = IRQF_TRIGGER_FALLING;

	return 0;
}

static int mxt_initialize_t9_input_device(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	const struct mxt_platform_data *pdata = data->pdata;
	struct input_dev *input_dev;
	int error;
	unsigned int num_mt_slots;
	int i;

	error = mxt_read_t9_resolution(data);
	if (error)
		dev_warn(dev, "Failed to initialize T9 resolution\n");

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	input_dev->name = MXT_DEVICE_NAME;
	input_dev->phys = data->phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = dev;
	input_dev->open = mxt_input_open;
	input_dev->close = mxt_input_close;

	__set_bit(EV_ABS, input_dev->evbit);
	input_set_capability(input_dev, EV_KEY, BTN_TOUCH);

	if (pdata->t19_num_keys) {
		__set_bit(INPUT_PROP_BUTTONPAD, input_dev->propbit);

		for (i = 0; i < pdata->t19_num_keys; i++)
			if (pdata->t19_keymap[i] != KEY_RESERVED)
				input_set_capability(input_dev, EV_KEY,
						     pdata->t19_keymap[i]);

		__set_bit(BTN_TOOL_FINGER, input_dev->keybit);
		__set_bit(BTN_TOOL_DOUBLETAP, input_dev->keybit);
		__set_bit(BTN_TOOL_TRIPLETAP, input_dev->keybit);
		__set_bit(BTN_TOOL_QUADTAP, input_dev->keybit);

		input_abs_set_res(input_dev, ABS_X, MXT_PIXELS_PER_MM);
		input_abs_set_res(input_dev, ABS_Y, MXT_PIXELS_PER_MM);
		input_abs_set_res(input_dev, ABS_MT_POSITION_X,
				  MXT_PIXELS_PER_MM);
		input_abs_set_res(input_dev, ABS_MT_POSITION_Y,
				  MXT_PIXELS_PER_MM);

		input_dev->name = "Atmel maXTouch Touchpad";
	}

	/* For single touch */
	input_set_abs_params(input_dev, ABS_X,
			     0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_Y,
			     0, data->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE,
			     0, 255, 0, 0);

	/* For multi touch */
	num_mt_slots = data->num_touchids + data->num_stylusids;
	error = input_mt_init_slots(input_dev, num_mt_slots);
	if (error) {
		dev_err(dev, "Error %d initialising slots\n", error);
		goto err_free_mem;
	}

	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
			     0, MXT_MAX_AREA, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			     0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			     0, data->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE,
			     0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_ORIENTATION,
			     0, 255, 0, 0);

	/* For T63 active stylus */
	if (data->T63_reportid_min) {
		input_set_capability(input_dev, EV_KEY, BTN_STYLUS);
		input_set_capability(input_dev, EV_KEY, BTN_STYLUS2);
		input_set_abs_params(input_dev, ABS_MT_TOOL_TYPE,
			0, MT_TOOL_MAX, 0, 0);
	}

	/* For T15 key array */
	if (data->T15_reportid_min) {
		data->t15_keystatus = 0;

		for (i = 0; i < data->pdata->t15_num_keys; i++)
			input_set_capability(input_dev, EV_KEY,
					     data->pdata->t15_keymap[i]);
	}

	input_set_drvdata(input_dev, data);

	error = input_register_device(input_dev);
	if (error) {
		dev_err(dev, "Error %d registering input device\n", error);
		goto err_free_mem;
	}

	data->input_dev = input_dev;

	return 0;

err_free_mem:
	input_free_device(input_dev);
	return error;
}

static int mxt_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct mxt_data *data;
	int error;
	u8 fw_version = 0;
	u8 fw_build = 0;
	touch_probe_ok = 0;

	is_probing = true;
	dev_info(&client->dev, "%s\n", __func__);

#ifdef	MXT_GESTURE_RECOGNIZE
	wake_lock_init(&touch_wake_lock, WAKE_LOCK_SUSPEND, "touch_irq");
	mutex_init(&i2c_suspend_lock);
	mutex_init(&mxt_early_mutex);

#endif

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c functionality check error\n");
		return -ENOMEM;
	}
	
	data = kzalloc(sizeof(struct mxt_data), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	snprintf(data->phys, sizeof(data->phys), "i2c-%u-%04x/input0",
		 client->adapter->nr, client->addr);
	dev_info(&client->dev, "i2c-%u-%04x/input0\n", client->adapter->nr, client->addr);
	data->client = client;
	data->irq = client->irq;
	i2c_set_clientdata(client, data);

	/*read dtsi data*/
	if (client->dev.of_node) {
		data->pdata = devm_kzalloc(&client->dev,
			sizeof(struct mxt_platform_data), GFP_KERNEL);
		if (!data->pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			error = -ENOMEM;
			goto err_free_mem;
		}

		error = mxt_parse_dt(&client->dev, data->pdata);
		if (error)
			goto err_free_mem;

	} else{
		error = mxt_handle_pdata(data);
		if (error)
			goto err_free_mem;
	}
	/*read dtsi data*/

	data->anti = devm_kzalloc(&client->dev,
		sizeof(struct mxt_anti_info), GFP_KERNEL);
	if (!data->anti) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		error = -ENOMEM;
		goto err_free_mem;
	}
#ifdef MXT_LPWG
	data->qwindow_size = devm_kzalloc(&client->dev,
		sizeof(struct quickcover_size), GFP_KERNEL);
	if (!data->qwindow_size) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		error = -ENOMEM;
		goto err_free_mem;
	}
#endif
	init_completion(&data->bl_completion);
	init_completion(&data->reset_completion);
	init_completion(&data->crc_completion);
	/* Self Test */
	init_completion(&data->t25_completion);
	/* workqueue */
	INIT_WORK(&data->work, mxt_process_messages_t44);
#ifdef WAITED_UDF
	INIT_WORK(&data->multi_tap_work, touch_multi_tap_work);
#endif
#ifdef I2C_SUSPEND_WORKAROUND
	INIT_DELAYED_WORK(&data->check_suspended_work, touch_check_suspended_worker);
#endif
	INIT_DELAYED_WORK(&data->work_ime_drumming, change_ime_drumming_func);
	/* request reset pin */
	if(data->pdata->gpio_reset> 0){
		error = gpio_request(data->pdata->gpio_reset, "touch_reset");
		if (error < 0) {
			dev_err(&client->dev, "FAIL: touch_reset gpio_request\n");
			goto err_interrupt_failed;
		}
		gpio_direction_output(data->pdata->gpio_reset, 1);
	}

	/* request interrupt pin */
	if(data->pdata->gpio_int > 0){
		error = gpio_request(data->pdata->gpio_int, "touch_int");
		if (error < 0) {
			dev_err(&client->dev, "FAIL: touch_int gpio_request\n");
			goto err_interrupt_failed;
		}
		gpio_direction_input(data->pdata->gpio_int);
	}
	
	error = request_threaded_irq(data->irq, NULL, mxt_interrupt,
				     data->pdata->irqflags | IRQF_ONESHOT,
				     client->name, data);
	if (error) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		goto err_free_pdata;
	}

	mxt_probe_regulators(data);

	touch_disable_irq(data->irq);

	error = mxt_initialize(data);
	if (error)
		goto err_free_irq;

#ifdef FIRMUP_ON_PROBE
	if(data->info != NULL)
	{
		fw_version = data->info->version;
		fw_build = data->info->build;
	} else {
		dev_info(&client->dev,"%s data->info = %s \n", __func__, data->info?"Exist":"NULL");
	}

	dev_dbg(&client->dev, "Need to firmware update? %d, %x!=%x, %x!=%x\n",
				data->in_bootloader, data->info->version, MXT_LATEST_FW_VERSION,
				data->info->build, MXT_LATEST_FW_BUILD);
	if (data->in_bootloader ||
			  fw_version != MXT_LATEST_FW_VERSION ||
			  fw_build != MXT_LATEST_FW_BUILD) {

		touch_enable_irq(data->irq);

		dev_info(&client->dev, "Execute firmware update func\n");
		error = mxt_firmware_update(data);
		 if (error) {
			dev_err(&client->dev, "Failed to update firmware\n");
			return error;
		}
	}
#endif
	error = mxt_rest_init(data);
	if (error)
		goto err_free_irq;

#if defined(CONFIG_FB)
		data->fb_notif.notifier_call = fb_notifier_callback;

		error = fb_register_client(&data->fb_notif);
		if (error)
			dev_err(&client->dev, "Unable to register fb_notifier: %d\n",
				error);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
		data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN +
							MXT_SUSPEND_LEVEL;
		data->early_suspend.suspend = mxt_early_suspend;
		data->early_suspend.resume = mxt_late_resume;
		register_early_suspend(&data->early_suspend);
#endif

	if(lge_get_boot_mode() == LGE_BOOT_MODE_CHARGERLOGO) {
		dev_info(&client->dev, "Boot chargerlogo mode\n");
		chargerlogo = true;
	}else{
		dev_info(&client->dev, "Boot normal mode\n");
	}
#ifdef MXT_FACTORY
	if ((lge_get_boot_mode() == LGE_BOOT_MODE_FACTORY) || (lge_get_boot_mode() == LGE_BOOT_MODE_FACTORY2)) {
		factorymode = true;
		printk("[lge_touch] yes-factory factory = %d\n", factorymode);
	}else{
		factorymode = false;
		printk("[lge_touch] no-factory factory = %d\n", factorymode);
	}
#endif
	/* disabled report touch event to prevent unnecessary event.
	* it will be enabled in open function
	*/
	mxt_stop(data);

	/* Register sysfs for making fixed communication path to framework layer */
	error = sysdev_class_register(&lge_touch_sys_class);
	if (error < 0) {
		dev_err(&client->dev, "sysdev_class_register is failed\n");
		goto err_lge_touch_sys_class_register;
	}

	error = sysdev_register(&lge_touch_sys_device);
	if (error < 0) {
		dev_err(&client->dev, "sysdev_register is failed\n");
		goto err_lge_touch_sys_dev_register;
	}

	error = kobject_init_and_add(&data->lge_touch_kobj, &lge_touch_kobj_type,
			data->input_dev->dev.kobj.parent,
			"%s", MXT_DEVICE_NAME);
	if (error < 0) {
		dev_err(&client->dev, "kobject_init_and_add is failed\n");
		goto err_lge_touch_sysfs_init_and_add;
	}

	sysfs_bin_attr_init(&data->mem_access_attr);
	data->mem_access_attr.attr.name = "mem_access";
	data->mem_access_attr.attr.mode = S_IRUGO | S_IWUSR;
	data->mem_access_attr.read = mxt_mem_access_read;
	data->mem_access_attr.write = mxt_mem_access_write;
	data->mem_access_attr.size = data->mem_size;

	if (sysfs_create_bin_file(&client->dev.kobj,
				  &data->mem_access_attr) < 0) {
		dev_err(&client->dev, "Failed to create %s\n",
			data->mem_access_attr.attr.name);
		goto err_lge_touch_sysfs_init_and_add;
	}

	is_probing = false;
	data->anti->pen_id = 255;
	data->anti->curr_ths = 23;	/* don't setting 23 on first 1 finger release event after booting*/
#ifdef WAITED_UDF
	hrtimer_init(&data->multi_tap_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	data->multi_tap_timer.function = &tci_timer_func;
#endif
	dev_info(&client->dev, "probe success\n");
	touch_probe_ok = 1;
	return 0;

err_lge_touch_sysfs_init_and_add:
	kobject_del(&data->lge_touch_kobj);
err_lge_touch_sys_dev_register:
	sysdev_unregister(&lge_touch_sys_device);
err_lge_touch_sys_class_register:
	sysdev_class_unregister(&lge_touch_sys_class);
	mxt_free_object_table(data);
err_free_irq:
	free_irq(data->irq, data);
err_interrupt_failed:
err_free_pdata:
err_free_mem:
	mutex_destroy(&i2c_suspend_lock);
	mutex_destroy(&mxt_early_mutex);
	if(data)
		kfree(data);
	return error;
}

static int __devexit mxt_remove(struct i2c_client *client)
{
	struct mxt_data *data = i2c_get_clientdata(client);

#if defined(CONFIG_FB)
	if (fb_unregister_client(&data->fb_notif))
		dev_err(&client->dev, "Error occurred while unregistering fb_notifier.\n");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&data->early_suspend);
#endif

	kobject_del(&data->lge_touch_kobj);
	sysdev_unregister(&lge_touch_sys_device);
	sysdev_class_unregister(&lge_touch_sys_class);

	if (data->pdata->gpio_int > 0)
		gpio_free(data->pdata->gpio_int);
	free_irq(data->irq, data);
	regulator_put(data->vdd_ana);
	regulator_put(data->vcc_i2c);
	regulator_put(data->vcc_dig);
	mutex_destroy(&i2c_suspend_lock);
	mutex_destroy(&mxt_early_mutex);
	mxt_free_object_table(data);
	if(data)
		kfree(data);
#ifdef MXT_GESTURE_RECOGNIZE
	wake_lock_destroy(&touch_wake_lock);
#endif

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int mxt_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;

	dev_info(&client->dev, "%s\n", __func__);

	if (data->in_bootloader){
		dev_info(&data->client->dev, "%s : Fw upgrade mode.\n", __func__);
		return 0;
	}

	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
		mxt_stop(data);

#ifdef MXT_GESTURE_RECOGNIZE
#ifdef MXT_LPWG
	if (data->lpwg_mode)
#else
	if (data->mxt_knock_on_enable)
#endif
	{
		touch_enable_irq_wake(data->irq);
		dev_dbg(&client->dev, "touch enable irq wake");
	}
	gpio_tlmm_config(GPIO_CFG(data->pdata->gpio_reset, 0, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
#endif
	data->mxt_suspended = 1;
	mutex_unlock(&input_dev->mutex);
	return 0;
}

static int mxt_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;
#ifdef MXT_RECOVERY_RESOLUTION
	struct mxt_object *object;
	int error;
#endif

	dev_info(&client->dev, "%s\n", __func__);

	mutex_lock(&input_dev->mutex);
	if((lge_get_boot_mode() == LGE_BOOT_MODE_CHARGERLOGO) || (lge_get_laf_mode() == LGE_LAF_MODE_LAF)){
		dev_warn(&data->client->dev, "%s : Ignore resume in Chargerlogo or Laf mode\n", __func__);
		mutex_unlock(&input_dev->mutex);
		return 0;
	}
	else
		dev_warn(&data->client->dev, "%s : normal mode\n", __func__);

#ifdef MXT_GESTURE_RECOGNIZE
	if(!data->pdata->gpio_reset)
		gpio_direction_output(data->pdata->gpio_reset, 1);
#ifdef MXT_LPWG
	if (data->lpwg_mode)
#else
	if (data->mxt_knock_on_enable)
#endif
	{

#ifdef MXT_RECOVERY_RESOLUTION
		object = mxt_get_object(data, 100);
		error = mxt_write_reg(data->client, object->start_address+13, 255);
		if (error)
			dev_err(&data->client->dev, "Object Write Fail\n");
		error = mxt_write_reg(data->client, object->start_address+14, 14);
		if (error)
			dev_err(&data->client->dev, "Object Write Fail\n");
		error = mxt_write_reg(data->client, object->start_address+24, 111);
		if (error)
			dev_err(&data->client->dev, "Object Write Fail\n");
		error = mxt_write_reg(data->client, object->start_address+25, 8);
		if (error)
			dev_err(&data->client->dev, "Object Write Fail\n");
#endif
		touch_disable_irq_wake(data->irq);
		dev_dbg(&client->dev, "touch disable irq wake");
	}
#endif
	if (input_dev->users){
		mxt_start(data);
	}
	data->mxt_suspended = 0;
	mutex_unlock(&input_dev->mutex);
	do_gettimeofday(&t_ex_debug[TIME_RESUME_END]);
	return 0;
}
#endif

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct mxt_data *mxt_dev_data =
		container_of(self, struct mxt_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK && mxt_dev_data &&
			mxt_dev_data->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			mxt_resume(&mxt_dev_data->client->dev);
		else if (*blank == FB_BLANK_POWERDOWN)
			mxt_suspend(&mxt_dev_data->client->dev);
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void mxt_early_suspend(struct early_suspend *h)
{
	struct mxt_data *data = container_of(h, struct mxt_data,
						early_suspend);
	mxt_suspend(&data->client->dev);
}

static void mxt_late_resume(struct early_suspend *h)
{
	struct mxt_data *data = container_of(h, struct mxt_data,
						early_suspend);
	mxt_resume(&data->client->dev);
}
#endif

#if defined(CONFIG_PM)
#if (!defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND))
static int touch_suspend(struct device *device)
{
	return 0;
}

static int touch_resume(struct device *device)
{
	return 0;
}
#elif defined(MXT_GESTURE_RECOGNIZE)
static int touch_suspend(struct device *device)
{
#ifdef I2C_SUSPEND_WORKAROUND
	struct mxt_data *data = dev_get_drvdata(device);
	if (!data) {
		TOUCH_ERR_MSG("Called before init\n");
		return 0;
	}
#endif
	mutex_lock(&i2c_suspend_lock);
#ifdef I2C_SUSPEND_WORKAROUND
	dev_dbg(&data->client->dev, " %s\n", __func__);
#endif
	return 0;
}

static int touch_resume(struct device *device)
{
#ifdef I2C_SUSPEND_WORKAROUND
	struct mxt_data *data = dev_get_drvdata(device);
	if (!data) {
		TOUCH_ERR_MSG("Called before init\n");
		return 0;
	}
#endif

	mutex_unlock(&i2c_suspend_lock);
#ifdef I2C_SUSPEND_WORKAROUND
	dev_dbg(&data->client->dev, " %s\n", __func__);
#endif

	return 0;
}
#endif
#endif

#if defined(CONFIG_PM)
static const struct dev_pm_ops mxt_pm_ops = {
#if (!defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND))||defined(MXT_GESTURE_RECOGNIZE)
	.suspend	= touch_suspend,
	.resume		= touch_resume,
#endif
};
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static const struct dev_pm_ops mxt_pm_ops = {
	.suspend	= NULL,
	.resume		= NULL,
};
#else
static SIMPLE_DEV_PM_OPS(mxt_pm_ops, mxt_suspend, mxt_resume);
#endif

static void mxt_shutdown(struct i2c_client *client)
{
	struct mxt_data *data = i2c_get_clientdata(client);

	touch_disable_irq(data->irq);
}

static struct of_device_id mxt_match_table[] = {
	{ .compatible = "atmel,s540",},
	{ },
};

static const struct i2c_device_id mxt_id[] = {
	{ "qt602240_ts", 0 },
	{ "atmel_mxt_ts", 0 },
	{ "atmel_mxt_tp", 0 },
	{ "mXT224", 0 },
	{ MXT_DEVICE_NAME, 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, mxt_id);

static struct i2c_driver mxt_driver = {
	.driver = {
		.name	= MXT_DEVICE_NAME,
		.of_match_table = mxt_match_table,
		.owner	= THIS_MODULE,
		.pm	= &mxt_pm_ops,
	},
	.probe		= mxt_probe,
	.remove		= __devexit_p(mxt_remove),
	.shutdown	= mxt_shutdown,
	.id_table	= mxt_id,
};

static int __devinit mxt_init(void)
{
	int ret = 0;

	touch_wq = create_singlethread_workqueue("touch_wq");
	if (!touch_wq) {
		TOUCH_ERR_MSG("CANNOT create new workqueue\n");
		ret = -EMLINK;
		goto err_work_queue;
	}
#ifdef WAITED_UDF
    touch_multi_tap_wq = create_singlethread_workqueue("touch_multi_tap_wq");
#endif
	ret = i2c_add_driver(&mxt_driver);
	if (ret < 0) {
		TOUCH_ERR_MSG("FAIL: i2c_add_driver\n");
		goto err_i2c_add_driver;
	}

	return 0;

err_i2c_add_driver:
	destroy_workqueue(touch_wq);
#ifdef WAITED_UDF
	destroy_workqueue(touch_multi_tap_wq);
#endif
err_work_queue:
	return ret;
}

static void __exit mxt_exit(void)
{
	i2c_del_driver(&mxt_driver);

	if (touch_wq)
		destroy_workqueue(touch_wq);
#ifdef WAITED_UDF
	if (touch_multi_tap_wq)
		destroy_workqueue(touch_multi_tap_wq);
#endif
}

module_init(mxt_init);
module_exit(mxt_exit);

/* Module information */
MODULE_AUTHOR("LGE");
MODULE_DESCRIPTION("Atmel maXTouch Touchscreen driver");
MODULE_LICENSE("GPL");

