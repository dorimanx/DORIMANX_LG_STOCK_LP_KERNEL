/*
 * Atmel maXTouch Touchscreen driver
 *

 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __LINUX_ATMEL_MXT_TS_H
#define __LINUX_ATMEL_MXT_TS_H

#ifndef CUST_B_TOUCH
#define CUST_B_TOUCH
#endif

#include <linux/types.h>

#define MXT_DEVICE_NAME			"lge_touch"
#define MXT_MAX_NUM_TOUCHES		10
#define MXT_GESTURE_RECOGNIZE
#define TSP_PATCH
#define I2C_SUSPEND_WORKAROUND 1
#define MXT_LPWG
#define ALPHA_FW
#define WAITED_UDF
#define T100_AREA_REPLACE_AMPLITUDE
#ifdef T100_AREA_REPLACE_AMPLITUDE
#define T100_AREA_W_H_IN_2BYTES
#endif
#define MXT_TA_TIME
#define MXT_FACTORY
#define MXT_WIRELESS
#define MXT_RECOVERY_RESOLUTION

#ifdef MXT_GESTURE_RECOGNIZE
#ifdef MXT_LPWG
#ifdef ALPHA_FW
#define MXT_LATEST_FW_VERSION       0x50
#define MXT_LATEST_FW_BUILD         0xF6
#else
#define MXT_LATEST_FW_VERSION       0x10
#define MXT_LATEST_FW_BUILD         0xEE
#endif
#else
#define MXT_LATEST_FW_VERSION       0x10
#define MXT_LATEST_FW_BUILD         0xE2
#endif
#else
#define MXT_LATEST_FW_VERSION       0x30
#define MXT_LATEST_FW_BUILD         0xAA
#endif

/* Registers */
#define MXT_OBJECT_START	0x07
#define MXT_OBJECT_SIZE		6
#define MXT_INFO_CHECKSUM_SIZE	3
#define MXT_MAX_BLOCK_WRITE	256

/* Object types */
#define MXT_GEN_MESSAGE_T5		5
#define MXT_GEN_MESSAGEPROCESSOR_T5	5
#define MXT_GEN_COMMAND_T6		6
#define MXT_GEN_COMMANDPROCESSOR_T6	6
#define MXT_GEN_POWER_T7		7
#define MXT_GEN_ACQUIRE_T8		8
#define MXT_TOUCH_KEYARRAY_T15		15
#define MXT_SPT_COMMSCONFIG_T18		18
#define MXT_SPT_GPIOPWM_T19		19
#define MXT_TOUCH_PROXIMITY_T23		23
#define MXT_PROCI_ONETOUCH_T24		24
#define MXT_SPT_SELFTEST_T25		25
#define MXT_SPT_PROTOTYPE_T35		35
#define MXT_DEBUG_DIAGNOSTIC_T37	37
#define MXT_T37		37
#define MXT_SPT_USERDATA_T38		38
#define MXT_PROCI_GRIPSUPPRESSION_T40		40
#define MXT_PROCI_TOUCHSUPPRESSION_T42	42
#define MXT_SPT_MESSAGECOUNT_T44	44
#define MXT_SPT_CTECONFIG_T46		46
#define MXT_PROCI_STYLUS_T47		47
#define MXT_PROCI_ADAPTIVETHRESHOLD_T55			55
#define MXT_PROCI_SHIELDLESS_T56				56
#define MXT_PROCI_EXTRATOUCHSCREENDATA_T57		57
#define MXT_SPT_TIMER_T61			61
#define MXT_PROCI_LENSBENDING_T65				65
#define MXT_SPT_GOLDENREFERENCES_T66	66
#define MXT_PROCI_PALMGESTUREPROCESSOR_T69		69
#define MXT_SPT_DYNAMICCONFIGURATIONCONTROLLER_T70	70
#define MXT_SPT_DYNAMICCONFIGURATIONCONTAINER_T71	71
#define MXT_PROCG_NOISESUPPRESSION_T72	72
#define MXT_GLOVEDETECTION_T78				78
#define MXT_RETRANSMISSIONCOMPENSATION_T80		80
#define MXT_PROCI_GESTUREPROCESSOR_T84			84
#define MXT_T92_NEW	92
#define MXT_T93_NEW	93
#define MXT_TOUCH_MULTITOUCHSCREEN_T100 100
#define MXT_SPT_TOUCHSCREENHOVER_T101		101
#define MXT_SPT_SELFCAPCBCRCONFIG_T102		102
#define MXT_PROCI_SCHNOISESUPPRESSION_T103	103
#define MXT_SPT_AUXTOUCHCONFIG_T104			104
#define MXT_SPT_DRIVENPLATEHOVERCONFIG_T105	105
#define MXT_RESERVED_T255 255

/* for 1188s */
#define MXT_GEN_DATASOURCE_T53		53
#define MXT_TOUCH_MULTI_T9		9
#define MXT_TOUCH_MULTITOUCHSCREEN_T9	9
#define MXT_TOUCH_PROXKEY_T52		52
#define MXT_PROCI_GRIPFACE_T20		20
#define MXT_PROCG_NOISE_T22		22
#define MXT_PROCI_TWOTOUCH_T27		27
#define MXT_PROCI_PALM_T41		41
#define MXT_PROCG_NOISESUPPRESSION_T48	48
#define MXT_SPT_CTECONFIG_T28		28
#define MXT_SPT_DIGITIZER_T43		43
#define MXT_SPT_NOISESUPPRESSION_T48	48
#define MXT_PROCI_ACTIVE_STYLUS_T63	63

/* MXT_GEN_MESSAGE_T5 object */
#define MXT_RPTID_NOMSG		0xff

/* MXT_GEN_COMMAND_T6 field */
#define MXT_COMMAND_RESET	0
#define MXT_COMMAND_BACKUPNV	1
#define MXT_COMMAND_CALIBRATE	2
#define MXT_COMMAND_REPORTALL	3
#define MXT_COMMAND_DIAGNOSTIC	5

/* Define for T6 status byte */
#define MXT_T6_STATUS_RESET	(1 << 7)
#define MXT_T6_STATUS_OFL	(1 << 6)
#define MXT_T6_STATUS_SIGERR	(1 << 5)
#define MXT_T6_STATUS_CAL	(1 << 4)
#define MXT_T6_STATUS_CFGERR	(1 << 3)
#define MXT_T6_STATUS_COMSERR	(1 << 2)

/* MXT POWER */
#define MXT_POWER_CFG_RUN		0
#define MXT_POWER_CFG_DEEPSLEEP		1
#define MXT_POWER_CFG_KNOCKON	2
#define MXT_POWER_OFF		3
#define MXT_POWER_ON		4

/* MXT_TOUCH_MULTI_T9 field */
#define MXT_T9_ORIENT		9
#define MXT_T9_RANGE		18

/* MXT_TOUCH_MULTI_T9 status */
#define MXT_T9_UNGRIP		(1 << 0)
#define MXT_T9_SUPPRESS		(1 << 1)
#define MXT_T9_AMP		(1 << 2)
#define MXT_T9_VECTOR		(1 << 3)
#define MXT_T9_MOVE		(1 << 4)
#define MXT_T9_RELEASE		(1 << 5)
#define MXT_T9_PRESS		(1 << 6)
#define MXT_T9_DETECT		(1 << 7)

/* MXT_TOUCH_MULTI_T9 orient */
#define MXT_T9_ORIENT_SWITCH	(1 << 0)

/* MXT_SPT_COMMSCONFIG_T18 */
#define MXT_COMMS_CTRL		0
#define MXT_COMMS_CMD		1
#define MXT_COMMS_RETRIGEN      (1 << 6)

/* Define for MXT_GEN_COMMAND_T6 */
#define MXT_BOOT_VALUE		0xa5
#define MXT_RESET_VALUE		0x01
#define MXT_BACKUP_VALUE	0x55
#define MXT_STOP_DYNAMIC_CONFIG	0x33

/* Define for MXT_PROCI_TOUCHSUPPRESSION_T42 */
#define MXT_T42_MSG_TCHSUP	(1 << 0)

/* T47 Stylus */
#define MXT_TOUCH_MAJOR_T47_STYLUS	1

/* T63 Stylus */
#define MXT_STYLUS_PRESS	(1 << 0)
#define MXT_STYLUS_RELEASE	(1 << 1)
#define MXT_STYLUS_MOVE		(1 << 2)
#define MXT_STYLUS_SUPPRESS	(1 << 3)

#define MXT_STYLUS_DETECT	(1 << 4)
#define MXT_STYLUS_TIP		(1 << 5)
#define MXT_STYLUS_ERASER	(1 << 6)
#define MXT_STYLUS_BARREL	(1 << 7)

#define MXT_STYLUS_PRESSURE_MASK	0x3F

/* T100 Multiple Touch Touchscreen */
#define MXT_T100_CTRL		0
#define MXT_T100_CFG1		1
#define MXT_T100_TCHAUX		3
#define MXT_T100_XRANGE		13
#define MXT_T100_YRANGE		24

#define MXT_T100_CFG_SWITCHXY	(1 << 5)

#define MXT_T100_TCHAUX_VECT	(1 << 0)
#define MXT_T100_TCHAUX_AMPL	(1 << 1)
#define MXT_T100_TCHAUX_AREA	(1 << 2)
#define MXT_T100_TCHAUX_RESV	(1 << 3)

#define MXT_T100_DETECT		(1 << 7)
#define MXT_T100_FRIST_ID_SUPPRESSION	(1 << 6)
#define MXT_T100_TYPE_MASK	0x70
#define MXT_T100_TYPE_FINGER	0x10
#define MXT_T100_TYPE_STYLUS	0x20
#define MXT_T100_TYPE_GLOVE		0x50
#define MXT_T100_TYPE_PALM		0x60
#define MXT_T100_STATUS_MASK	0x0F
#define MXT_T100_PRESS		0x04
#define MXT_T100_RELEASE	0x05
#define MXT_T100_MOVE		0x01
#define MXT_T100_SUPPRESSION		0x03

#ifdef T100_AREA_W_H_IN_2BYTES
#define MXT_T100_AREA_MASK			0x3F
#define MXT_T100_WIDTH_MSB_MASK		0xC0
#define MXT_T100_WIDTH_LSB_MASK		0xE0
#define MXT_T100_HEIGHT_MASK		0x1F
#endif

/* Delay times */
#define MXT_BACKUP_TIME		50	/* msec */
#define MXT_RESET_TIME		200	/* msec */
#define MXT_RESET_TIMEOUT	3000	/* msec */
#define MXT_CRC_TIMEOUT		1000	/* msec */
#define MXT_FW_RESET_TIME	1000	/* msec */
#define MXT_FW_CHG_TIMEOUT	300	/* msec */
#define MXT_WAKEUP_TIME		25	/* msec */
#define MXT_REGULATOR_DELAY	150	/* msec */
#define MXT_POWERON_DELAY	150	/* msec */
#define MXT_SELFTEST_TIME	3000	/* msec */

/* Command to unlock bootloader */
#define MXT_UNLOCK_CMD_MSB	0xaa
#define MXT_UNLOCK_CMD_LSB	0xdc

/* Bootloader mode status */
#define MXT_WAITING_BOOTLOAD_CMD	0xc0	/* valid 7 6 bit only */
#define MXT_WAITING_FRAME_DATA	0x80	/* valid 7 6 bit only */
#define MXT_FRAME_CRC_CHECK	0x02
#define MXT_FRAME_CRC_FAIL	0x03
#define MXT_FRAME_CRC_PASS	0x04
#define MXT_APP_CRC_FAIL	0x40	/* valid 7 8 bit only */
#define MXT_BOOT_STATUS_MASK	0x3f
#define MXT_BOOT_EXTENDED_ID	(1 << 5)
#define MXT_BOOT_ID_MASK	0x1f

/* Touchscreen absolute values */
#define MXT_MAX_AREA		0xff

#define MXT_PIXELS_PER_MM	20

#ifdef CUST_B_TOUCH
// LGE_CHANGE_S [naomi.kim@lge.com] 13.06.17, set data format
#define TOUCHEVENTFILTER	1
// LGE_CHANGE_E [naomi.kim@lge.com] 13.06.17, set data format
#endif

#define MXT_MAX_FW_PATH				64
#ifdef MXT_LPWG
#define MAX_POINT_SIZE_FOR_LPWG	12

struct point
{
	int x;
	int y;
};

struct quickcover_size
{
	int x_max;
	int y_max;
	int x_min;
	int y_min;
};

enum{
	LPWG_READ = 1,
	LPWG_ENABLE,
	LPWG_LCD_X,
	LPWG_LCD_Y,
	LPWG_ACTIVE_AREA_X1,
	LPWG_ACTIVE_AREA_X2,
	LPWG_ACTIVE_AREA_Y1,
	LPWG_ACTIVE_AREA_Y2,
	LPWG_TAP_COUNT,
	LPWG_REPLY,
	LPWG_LENGTH_BETWEEN_TAP,
	LPWG_EARLY_MODE,
};

enum{
	LPWG_NONE = 0,
	LPWG_DOUBLE_TAP,
	LPWG_PASSWORD,
};
#endif

typedef enum error_type {
	NO_ERROR = 0,
	ERROR,
	IGNORE_EVENT,
	IGNORE_EVENT_BUT_SAVE_IT,
} err_t;

enum{
	KEYGUARD_RESERVED,
	KEYGUARD_ENABLE,
};

#ifdef CUST_B_TOUCH
enum{
	TIME_START_TIME,
	TIME_CURR_TIME,
	TIME_RESUME_END,
	TIME_TA_DETECT,
	TIME_CAL_START,
	TIME_EX_PROFILE_MAX
};

enum{
	FINGER_INACTIVE,
	FINGER_RELEASED,
	FINGER_PRESSED,
	FINGER_MOVED
};
#endif

enum { MXT_T6 = 0,
	MXT_T38,
	MXT_T71,
	MXT_T7,
	MXT_T8,
	MXT_T15,
	MXT_T18,
	MXT_T19,
	MXT_T23,
#ifdef MXT_GESTURE_RECOGNIZE
	MXT_T24,
#endif
	MXT_T25,
	MXT_T40,
	MXT_T42,
	MXT_T46,
	MXT_T47,
	MXT_T55,
	MXT_T56,
	MXT_T61,
	MXT_T65,
	MXT_T66,
	MXT_T69,
	MXT_T70,
	MXT_T72,
	MXT_T78,
	MXT_T80,
#ifndef ALPHA_FW
	MXT_T84,
#endif
#ifdef MXT_LPWG
	MXT_T92,
	MXT_T93,
#endif
	MXT_T100,
	MXT_T101,
	MXT_T102,
	MXT_T103,
	MXT_T104,
	MXT_T105,
	MXT_TMAX,
};

/* Config data for a given maXTouch controller with a specific firmware */
struct mxt_config_info {
	u8 *config_t[MXT_TMAX];
};

/* The platform data for the Atmel maXTouch touchscreen driver */
struct mxt_platform_data {
	const struct mxt_config_info *config_array;
	size_t config_array_size;
	u8    numtouch;	/* Number of touches to report	*/
	int   max_x;    /* The default reported X range   */
	int   max_y;    /* The default reported Y range   */
	bool i2c_pull_up;
	unsigned long irqflags;
	u8 t19_num_keys;
	const unsigned int *t19_keymap;
	int t15_num_keys;
	const unsigned int *t15_keymap;
	unsigned long gpio_reset;
	unsigned long gpio_int;
	const char *cfg_name;
};

/* MXT_GEN_POWER_T7 field */
struct t7_config {
	u8 idle;
	u8 active;
} __packed;

struct t9_range {
	u16 x;
	u16 y;
} __packed;

struct mxt_info {
	u8 family_id;
	u8 variant_id;
	u8 version;
	u8 build;
	u8 matrix_xsize;
	u8 matrix_ysize;
	u8 object_num;
};

struct mxt_object {
	u8 type;
	u16 start_address;
	u8 size_minus_one;
	u8 instances_minus_one;
	u8 num_report_ids;
} __packed;

#ifdef CUST_B_TOUCH
struct t_data {
	u16	id;
	u8	status;
	u16	x_position;
	u16	y_position;
	u8	touch_major;
	#if TOUCHEVENTFILTER
	u8	touch_minor;
	#endif
	u8	pressure;
	u8	orientation;
	int tool;
	bool is_pen;
	bool is_palm;
	bool skip_report;
};

struct touch_data {
	u8 total_num;
	u8 prev_total_num;
	u8 state;
	u8 palm;
	struct t_data curr_data[MXT_MAX_NUM_TOUCHES];
	struct t_data prev_data[MXT_MAX_NUM_TOUCHES];
};
#endif

struct mxt_fw_info {
	u8 fw_ver;
	u8 build_ver;
	u32 hdr_len;
	u32 cfg_len;
	u32 fw_len;
	u32 cfg_crc;
	const u8 *cfg_raw_data; /* start address of configuration data */
	const u8 *fw_raw_data;	/* start address of firmware data */
	struct mxt_data *data;
};

#ifdef TSP_PATCH
struct mxt_patch {
	u8 *patch;
	u16 *stage_addr;
	u16 *tline_addr;
	u16 *trigger_addr;
	u16 *event_addr;
	u16 *src_item;
	u16 *check_cnt;
	u16 period;
	u8 stage_cnt;
	u8 tline_cnt;
	u8 trigger_cnt;
	u8 event_cnt;
	u8 option;
	u8 debug;
	u8 timer_id;
	u8 cur_stage;
	u8 cur_stage_opt;
	u8 run_stage;
	u8 start;
	u8 finger_cnt;
	u8 start_stage; //0904
	u8 skip_test;	//0908
	u8 cal_flag; //1107
	u32 date;
	u32 stage_timestamp;
};

struct mxt_message {
    u8 reportid;
    u8 message[8];
};
#endif

struct mxt_reportid {
	u8 type;
	u8 index;
};

struct mxt_anti_info{
	bool insensitive_th;
	bool sensitive_th;
	bool anti_report;
	bool autocal;
	bool pen;
	int pen_id;
	int curr_ths;
	u16 anti_area;
	u16 touch_area;
	u16 inter_area;
	int fcnt0_msg_cnt;
};


/* Each client has this additional data */
struct mxt_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	char phys[64];		/* device physical location */
	struct mxt_platform_data *pdata;
	struct mxt_object *object_table;
	struct mxt_info *info;
	struct work_struct		work;
	struct mxt_anti_info *anti;
#ifdef TSP_PATCH
	struct mxt_patch patch;
#endif
	struct mxt_reportid *reportids;
#ifdef MXT_LPWG
	u8 is_lpwg_report_enable;
	u8 mxt_password_enable;
	u8 lpwg_mode;
	struct quickcover_size *qwindow_size;
#endif
	void *raw_info_block;
	unsigned int irq;
	unsigned int max_x;
	unsigned int max_y;
	bool in_bootloader;
	u16 mem_size;
	u8 t100_aux_ampl;
	u8 t100_aux_area;
	u8 t100_aux_vect;
	u8 t100_aux_resv;
	struct bin_attribute mem_access_attr;
	bool debug_enabled;
	u8 max_reportid;
	u32 config_crc;
	u32 info_crc;
	u8 bootloader_addr;
	struct t7_config t7_cfg;
	u8 *msg_buf;
	u8 t6_status;
	bool update_input;
	u8 last_message_count;
	u8 num_touchids;
	u8 num_stylusids;
	unsigned long t15_keystatus;
	bool use_retrigen_workaround;
	bool use_regulator;
	struct regulator *vdd_ana;
	struct regulator *vcc_i2c;
	struct regulator *vcc_dig;
	int power_status;
	int ta_status;
	bool charging_mode;
	bool charging_wireless;
	char *fw_name;
	char *cfg_name;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#endif
	struct kobject 				lge_touch_kobj;
	/* Cached parameters from object table */
	u16 T5_address;
	u8 T5_msg_size;
	u16 T6_address;
	u16 T7_address;
	u16 T8_address;
	u8 T9_reportid_min;
	u8 T9_reportid_max;
	u8 T15_reportid_min;
	u8 T15_reportid_max;
	u16 T18_address;
	u16 T25_address;
	u16 T42_address;
	u16 T44_address;
	u16 T47_address;
	u8 T63_reportid_min;
	u8 T63_reportid_max;
#ifdef MXT_LPWG
	u16 T92_address;
	u16 T93_address;
	u8 T93_reportid_min;
#endif
	u8 T100_reportid_min;
	u16 T100_address;

	/* for fw update in bootloader */
	struct completion bl_completion;

	/* for reset handling */
	struct completion reset_completion;

	/* for reset handling */
	struct completion crc_completion;

	/* Auto touch test */
	struct completion t25_completion;

	/* Enable reporting of input events */
	bool enable_reporting;

	/* Indicates whether device is in suspend */
	bool suspended;
	bool mxt_suspended;
#ifdef I2C_SUSPEND_WORKAROUND
	struct delayed_work check_suspended_work;
#endif
	struct delayed_work work_ime_drumming;
#ifdef CUST_B_TOUCH
	struct touch_data ts_data;
#endif
#ifdef MXT_GESTURE_RECOGNIZE
	bool mxt_knock_on_enable;
	bool mxt_character_enable;
	bool mxt_volume_enable;
	bool run_wakeup_enable;
	bool run_wakeup_disable;
#endif
	bool self_test_result;
	u8 self_test_status[4];
#ifdef T100_AREA_REPLACE_AMPLITUDE
	u8 T100_palm_threshold;
#endif
#ifdef WAITED_UDF
	struct hrtimer multi_tap_timer;
	struct work_struct  multi_tap_work;
#endif
};
#ifdef TSP_PATCH
int mxt_write_object(struct mxt_data *data, u8 type, u8 offset, int value);
int mxt_write_mem(struct mxt_data *data, u16 reg, u8 len, const u8 *buf);
int mxt_read_object(struct mxt_data *data, u8 type, u8 offset, u8 *value);
struct mxt_object *mxt_get_object(struct mxt_data *data, u8 type);
extern int mxt_patch_test_event(struct mxt_data *data, u8 event_id);
extern void mxt_patch_message(struct mxt_data *data, struct mxt_message *message);
extern int mxt_patch_init(struct mxt_data *data, u8* ppatch);
#endif
#ifdef I2C_SUSPEND_WORKAROUND
extern bool atmel_touch_i2c_suspended;
#endif

#define	REPORT_ID_TO_OBJECT(rid, data)			\
	(((rid) == 0xff) ? 0 : data->reportids[rid].type)

#define DO_IF(do_work, goto_error) 								\
do {												\
	if(do_work){ 										\
		printk(KERN_INFO "[lge_touch E] Action Failed [%s %d] \n", __FUNCTION__, __LINE__); \
		goto goto_error; 								\
	}											\
} while(0)

#define DO_SAFE(do_work, goto_error) 								\
	DO_IF(unlikely((do_work) < 0), goto_error)

#define TOUCH_INFO_MSG(fmt, args...) \
		printk(KERN_INFO "[lge_touch] " fmt, ##args);

#define TOUCH_ERR_MSG(fmt, args...) \
	printk(KERN_ERR "[lge_touch E] [%s %d] " fmt, __FUNCTION__, __LINE__, ##args);

#define TOUCH_DEBUG_MSG(fmt, args...) \
	printk(KERN_INFO "[lge_touch D] [%s %d] " fmt, __FUNCTION__, __LINE__, ##args);

#define TOUCH_PATCH_MSG(fmt, args...) \
		;//printk(KERN_INFO "[mxts_patch] " fmt, ##args);		//print too many patch logs
#define TOUCH_PATCH_ERRMSG(fmt, args...) \
		;//printk(KERN_ERR "[mxts_patch] " fmt, ##args);

#endif /* __LINUX_ATMEL_MXT_TS_H */
