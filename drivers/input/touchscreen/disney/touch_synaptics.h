/* include/linux/lge_touch_core.h
 *
 * Copyright (C) 2012 LGE.
 *
 * Writer: yehan.ahn@lge.com, 	hyesung.shin@lge.com
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

#ifndef LGE_TOUCH_SYNAPTICS_H
#define LGE_TOUCH_SYNAPTICS_H

#define NUM_OF_EACH_FINGER_DATA_REG		8
#define MAX_NUM_OF_FINGERS			10

#define DESCRIPTION_TABLE_START			0xe9
#define EXIST_OFFSET                            0xEE

#define PAGE_SELECT_REG				0xFF		/* Button exists Page 02 */
#define PAGE_MAX_NUM				5		/* number of page register */

#define F12_NO_OBJECT_STATUS		(0x00)
#define F12_FINGER_STATUS			(0x01)
#define F12_STYLUS_STATUS			(0x02)
#define F12_PALM_STATUS				(0x03)
#define F12_HOVERING_FINGER_STATUS	(0x05)
#define F12_GLOVED_FINGER_STATUS	(0x06)

#define OBJECT_FINGER_BIT		0
#define OBJECT_STYLUS_BIT		1
#define OBJECT_PALM_BIT			2
#define OBJECT_UNCLASSIFIED_OBJECT_BIT	3
#define OBJECT_HOVERING_FINGER_BIT	4
#define OBJECT_GLOVEED_FINGER_BIT	5
#define OBJECT_NARROW_OBJECT_SWIPE_BIT	6
#define OBJECT_HAND_EDGE_BUT		7

struct synaptics_ts_f12_query_5 {
	union {
		struct {
			unsigned char size_of_query_6;
			struct {
				unsigned char ctrl_00_is_present:1;
				unsigned char ctrl_01_is_present:1;
				unsigned char ctrl_02_is_present:1;
				unsigned char ctrl_03_is_present:1;
				unsigned char ctrl_04_is_present:1;
				unsigned char ctrl_05_is_present:1;
				unsigned char ctrl_06_is_present:1;
				unsigned char ctrl_07_is_present:1;
			} __packed;
			struct {
				unsigned char ctrl_08_is_present:1;
				unsigned char ctrl_09_is_present:1;
				unsigned char ctrl_10_is_present:1;
				unsigned char ctrl_11_is_present:1;
				unsigned char ctrl_12_is_present:1;
				unsigned char ctrl_13_is_present:1;
				unsigned char ctrl_14_is_present:1;
				unsigned char ctrl_15_is_present:1;
			} __packed;
			struct {
				unsigned char ctrl_16_is_present:1;
				unsigned char ctrl_17_is_present:1;
				unsigned char ctrl_18_is_present:1;
				unsigned char ctrl_19_is_present:1;
				unsigned char ctrl_20_is_present:1;
				unsigned char ctrl_21_is_present:1;
				unsigned char ctrl_22_is_present:1;
				unsigned char ctrl_23_is_present:1;
			} __packed;
			struct {
				unsigned char ctrl_24_is_present:1;
				unsigned char ctrl_25_is_present:1;
				unsigned char ctrl_26_is_present:1;
				unsigned char ctrl_27_is_present:1;
				unsigned char ctrl_28_is_present:1;
				unsigned char ctrl_29_is_present:1;
				unsigned char ctrl_30_is_present:1;
				unsigned char ctrl_31_is_present:1;
			} __packed;
		};
		unsigned char data[5];
	};
};

struct synaptics_ts_f12_query_8 {
	union {
		struct {
			unsigned char size_of_query_9;
			struct {
				unsigned char data_00_is_present:1;
				unsigned char data_01_is_present:1;
				unsigned char data_02_is_present:1;
				unsigned char data_03_is_present:1;
				unsigned char data_04_is_present:1;
				unsigned char data_05_is_present:1;
				unsigned char data_06_is_present:1;
				unsigned char data_07_is_present:1;
			} __packed;
			struct {
				unsigned char data_08_is_present:1;
				unsigned char data_09_is_present:1;
				unsigned char data_10_is_present:1;
				unsigned char data_11_is_present:1;
				unsigned char data_12_is_present:1;
				unsigned char data_13_is_present:1;
				unsigned char data_14_is_present:1;
				unsigned char data_15_is_present:1;
			} __packed;
		};
		unsigned char data[3];
	};
};

struct synaptics_ts_f12_reg {
	u8 ctrl[32];
	u8 data[16];
};

struct synaptics_ts_f12_ctrl_8 {
	union {
		struct {
			unsigned char max_x_coord_lsb;
			unsigned char max_x_coord_msb;
			unsigned char max_y_coord_lsb;
			unsigned char max_y_coord_msb;
			unsigned char rx_pitch_lsb;
			unsigned char rx_pitch_msb;
			unsigned char tx_pitch_lsb;
			unsigned char tx_pitch_msb;
			unsigned char low_rx_clip;
			unsigned char high_rx_clip;
			unsigned char low_tx_clip;
			unsigned char high_tx_clip;
			unsigned char num_of_rx;
			unsigned char num_of_tx;
		};
		unsigned char data[14];
	};
};

struct synaptics_ts_f12_ctrl_23 {
	union {
		struct {
			unsigned char obj_type_enable;
			unsigned char max_reported_objects;
		};
		unsigned char data[2];
	};
};

struct synaptics_ts_f12_finger_data {
	unsigned char object_type_and_status;
	unsigned char x_lsb;
	unsigned char x_msb;
	unsigned char y_lsb;
	unsigned char y_msb;
	unsigned char z;
	unsigned char wx;
	unsigned char wy;
};

struct synaptics_ts_f1a_query {
	union {
		struct {
			unsigned char max_button_count:3;
			unsigned char reserved:5;
			unsigned char has_general_control:1;
			unsigned char has_interrupt_enable:1;
			unsigned char has_multibutton_select:1;
			unsigned char has_tx_rx_map:1;
			unsigned char has_perbutton_threshold:1;
			unsigned char has_release_threshold:1;
			unsigned char has_strongestbtn_hysteresis:1;
			unsigned char has_filter_strength:1;
		} __packed;
		unsigned char data[2];
	};
};

struct synaptics_ts_f1a_control_0 {
	union {
		struct {
			unsigned char multibutton_report:2;
			unsigned char filter_mode:2;
			unsigned char reserved:4;
		} __packed;
		unsigned char data[1];
	};
};

struct synaptics_ts_f1a_control_3_4 {
	unsigned char transmitterbutton;
	unsigned char receiverbutton;
};

struct synaptics_ts_f1a_control {
	struct synaptics_ts_f1a_control_0 general_control;
	unsigned char *button_int_enable;
	unsigned char *multi_button;
	struct synaptics_ts_f1a_control_3_4 *electrode_map;
	unsigned char *button_threshold;
	unsigned char button_release_threshold;
	unsigned char strongest_button_hysteresis;
	unsigned char filter_strength;
};

struct synaptics_ts_f1a_handle {
	int button_bitmask_size;
	unsigned char button_count;
	unsigned char valid_button_count;
	unsigned char *button_data_buffer;
	unsigned char *button_map;
	struct synaptics_ts_f1a_query button_query;
	struct synaptics_ts_f1a_control button_control;
};

struct synaptics_ts_f12_extra_data {
	unsigned char data1_offset;
	unsigned char data15_offset;
	unsigned char data15_size;
	unsigned char data15_data[(MAX_NUM_OF_FINGERS + 7) / 8];
};

struct synaptics_ts_f51_reg {
	u8 lpwg_status_reg;
	u8 lpwg_data_reg;
	u8 lpwg_tapcount_reg;
	u8 lpwg_min_intertap_reg;
	u8 lpwg_max_intertap_reg;
	u8 lpwg_touch_slop_reg;
	u8 lpwg_tap_distance_reg;
	u8 lpwg_interrupt_delay_reg;
	u8 lpwg_tapcount_reg2;
	u8 lpwg_min_intertap_reg2;
	u8 lpwg_max_intertap_reg2;
	u8 lpwg_touch_slop_reg2;
	u8 lpwg_tap_distance_reg2;
	u8 lpwg_interrupt_delay_reg2;
};

struct synaptics_ts_f54_reg {
	u8 interference__metric_LSB;
	u8 interference__metric_MSB;
	u8 current_noise_status;
	u8 cid_im;
	u8 freq_scan_im;
};

struct function_descriptor {
	u8 	query_base;
	u8 	command_base;
	u8 	control_base;
	u8 	data_base;
	u8 	int_source_count;
	u8 	id;
};

struct ts_ic_function {
	struct function_descriptor dsc;
	u8 page;
};

struct finger_data {
	u8 type;
	u8 x_lsb;
	u8 x_msb;
	u8 y_lsb;
	u8 y_msb;
	u8 z;
	u8 wx;
	u8 wy;
};

struct button_data {
	u16	key_code;
};

struct cur_touch_data {
	u8	device_status_reg;		/* DEVICE_STATUS_REG */
	u8	interrupt_status_reg;
	u8	button_data_reg;
	struct finger_data	finger[MAX_NUM_OF_FINGERS];
	struct button_data	button;
};

struct synaptics_ts_fw_info {
	u8		fw_version[5];
	u8		fw_product_id[11];
	u8		fw_image_version[5];
	u8		fw_image_product_id[11];
	unsigned char	*fw_start;
	unsigned char   family;
	unsigned char   fw_revision;
	unsigned long	fw_size;
	u8		need_rewrite_firmware;
};

struct lpwg_control {
	u8		lpwg_mode;
	u8              screen;
	u8		prev_screen;
	u8              sensor;
	u8              qcover;
	u8		double_tap_enable;
	u8 		password_enable;
	u8		signature_enable;
	u8		lpwg_is_enabled;
	u8		has_debug_module;
	bool 		protocol9_sleep_flag;
	atomic_t	is_suspend;
};

struct lpwg_password_data {
	u8		tap_count;
	u8		data_num;
	u8              double_tap_check;
	struct point 	data[MAX_POINT_SIZE_FOR_LPWG];
};

struct state_flag {
	u8		ts_noise_log_flag;
	u8		check_noise_menu;
};

struct palm_data {
	bool curr_palm_mask[MAX_NUM_OF_FINGERS];
	bool prev_palm_mask[MAX_NUM_OF_FINGERS];
	struct point palm_coordinate[MAX_NUM_OF_FINGERS];
	u8 curr_palm_num;
	u8 prev_palm_num;
	bool all_palm_released;
};

struct swipe_data {
	bool	support_swipe;
	bool	support_swipe_fail_reason;
	bool	support_swipe_time;
	u8	wakeup_by_swipe;
	u8	swipe_ratio_threshold;
	u8	swipe_ratio_check_period;
	u16	min_swipe_time_threshold;
	u16	max_swipe_time_threshold;
	u8	swipe_min_distance;
};

enum {
	CHARGERLOGO_MODE = 0,
	NORMAL_BOOT_MODE,
};

enum{
	TS_NOISE_LOG_DISABLE = 0,
	TS_NOISE_LOG_ENABLE,
};

enum{
	MENU_OUT = 0,
	MENU_ENTER,
};

enum {
	THERMAL_LOW = 0,
	THERMAL_HIGH,
};

struct synaptics_ts_data {
	u8	is_probed;
	u8	is_init;
	u8	object_report;
	u8	num_of_fingers;
	u8 	curr_page;
	u8 	default_finger_amplitude;
	u8	default_small_finger_amplitude;
	u8	min_finger_amplitude;
	struct lpwg_control		lpwg_ctrl;
	struct lpwg_password_data	pw_data;
	struct regulator		*regulator_vdd;
	struct regulator		*regulator_vio;
	struct i2c_client		*client;
	struct ts_ic_function		f01;
	struct ts_ic_function		f11;
	struct ts_ic_function		f12;
	struct ts_ic_function		f1a;
	struct ts_ic_function		f34;
	struct ts_ic_function		f51; /* only lge */
	struct ts_ic_function		f54;
	struct ts_ic_function		f55;

	struct synaptics_ts_f12_reg	f12_reg;
	struct synaptics_ts_f51_reg f51_reg;
	struct synaptics_ts_f54_reg f54_reg;

	struct cur_touch_data		ts_data;
	struct synaptics_ts_fw_info	fw_info;
	struct delayed_work		work_timer;
	struct delayed_work		diff_node_timer;
	struct delayed_work     	work_palm;
	struct wake_lock		timer_wake_lock;
	struct touch_platform_data	*pdata;
	const struct state_info		*state;
	struct state_flag		ts_state_flag;
	unsigned int bad_sample;
	int h_err_cnt;
	int v_err_cnt;
	struct palm_data		ts_palm_data;
	struct swipe_data		ts_swipe_data;

};

struct synaptics_ts_exp_fn {
	int (*init)(struct synaptics_ts_data *ts);
	void (*remove)(struct synaptics_ts_data *ts);
	void (*reset)(struct synaptics_ts_data *ts);
	void (*reinit)(struct synaptics_ts_data *ts);
	void (*early_suspend)(struct synaptics_ts_data *ts);
	void (*suspend)(struct synaptics_ts_data *ts);
	void (*resume)(struct synaptics_ts_data *ts);
	void (*late_resume)(struct synaptics_ts_data *ts);
	void (*attn)(unsigned char intr_status_reg);
};

extern struct workqueue_struct *touch_wq;
extern char f54_wlog_buf[6000];
enum error_type synaptics_ts_init(struct i2c_client *client);
extern void SCAN_PDT(void);
int compare_fw_version(struct i2c_client *client, struct touch_fw_info *fw_info);
//mode:0 => write_log, mode:1 && buf => cat, mode:2 && buf => delta
extern int F54Test(int input, int mode, char *buf);
extern int GetImageReport(char *buf);

void synaptics_ts_prox_function(struct synaptics_ts_exp_fn *prox_fn, bool insert);
void synaptics_ts_rmidev_function(struct synaptics_ts_exp_fn *rmidev_fn, bool insert);

/* extern function */
extern int FirmwareUpgrade(struct synaptics_ts_data *ts, const char* fw_path);
int synaptics_ts_set_page(struct i2c_client *client, u8 page);
int synaptics_ts_page_data_read(struct i2c_client *client, u8 page, u8 reg, int size, u8 *data);
int synaptics_ts_page_data_write(struct i2c_client *client, u8 page, u8 reg, int size, u8 *data);
int synaptics_ts_page_data_write_byte(struct i2c_client *client, u8 page, u8 reg, u8 data);

#endif
