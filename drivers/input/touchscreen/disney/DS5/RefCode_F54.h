/* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 Copyright ?2012 Synaptics Incorporated. All rights reserved.

 The information in this file is confidential under the terms
 of a non-disclosure agreement with Synaptics and is provided
 AS IS.

 The information in this file shall remain the exclusive property
 of Synaptics and may be the subject of Synaptics?patents, in
 whole or part. Synaptics?intellectual property rights in the
 information in this file are not expressly or implicitly licensed
 or otherwise transferred to you as a result of such information
 being made available to you.
 +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <linux/kernel.h>/* printk */
#include <linux/delay.h>/* msleep */
#include <linux/time.h>	/* struct timeval t_interval[TIME_PROFILE_MAX];*/
#include <linux/math64.h> /*for abs func*/
#include <linux/string.h>/* memset */
#include <linux/i2c.h>
#include "../lge_touch_core.h"
//#include <linux/input/lge_touch_core.h>

#define TRX_max 32
#define CAP_FILE_PATH "/sns/touch/cap_diff_test.txt"
#define DS5_BUFFER_SIZE 6000

extern int UpperImage[32][32];
extern int LowerImage[32][32];
extern int SensorSpeedUpperImage[32][32];
extern int SensorSpeedLowerImage[32][32];
extern int ADCUpperImage[32][32];
extern int ADCLowerImage[32][32];
extern int RspLowerSlope[32][32];
extern int RspUpperSlope[32][32];
extern int RspNoise[32][32];
extern unsigned char RxChannelCount;
extern unsigned char TxChannelCount;
extern int f54_window_crack;
extern int f54_window_crack_check_mode;
extern void SCAN_PDT(void);
extern int f54_window_crack;
extern int f54_window_crack_check_mode;
extern struct i2c_client *ds4_i2c_client;
extern int touch_i2c_read(struct i2c_client *client, u8 reg, int len, u8 *buf);
extern int touch_i2c_write(struct i2c_client *client, u8 reg, int len, u8 *buf);
/*mode:0 => write_log, mode:1 && buf => cat, mode:2 && buf => delta*/
extern int F54Test(int input, int mode, char *buf);
extern int GetImageReport(char *buf);
extern int diffnode(unsigned short *ImagepTest);
extern int write_file(char *filename, char *data);
extern int write_log(char *filename, char *data);
extern void read_log(char *filename, const struct touch_platform_data *pdata);
extern int Read8BitRegisters(unsigned short regAddr,
		unsigned char *data, int length);
extern int Write8BitRegisters(unsigned short regAddr,
		unsigned char *data, int length);
extern int get_limit(unsigned char Tx,
		unsigned char Rx, struct i2c_client client,
		const struct touch_platform_data *pdata,
		char *breakpoint, int limit_data[32][32]);
extern int mfts_enable;
