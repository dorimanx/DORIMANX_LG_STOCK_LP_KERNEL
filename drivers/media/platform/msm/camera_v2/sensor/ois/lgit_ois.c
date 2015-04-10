//==============================================================================
// FujiFlim OIS firmware
//==============================================================================
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <mach/gpio.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/types.h>
#include <linux/syscalls.h>
#include <linux/fcntl.h>
#include <linux/file.h>
#include <mach/camera2.h>
#include <linux/poll.h>
#include "msm_ois.h"
#include "msm_ois_i2c.h"

#define LAST_UPDATE "13-12-18, 13M LGIT OIS bu24205"

#define FIRMWARE_VER_BIN_1 "bu24205_LGIT_rev16_1_2030_data1.bin"
#define FIRMWARE_VER_BIN_2 "bu24205_LGIT_rev16_1_2030_data2.bin"
#define FIRMWARE_VER_BIN_3 "bu24205_LGIT_rev16_1_2030_data3.bin"
#define FIRMWARE_VER_CHECKSUM 0x00060A29

#define E2P_FIRST_ADDR 			(0x0900)
#define E2P_DATA_BYTE			(28)
#define CTL_END_ADDR_FOR_E2P_DL	(0x13A8)

#define OIS_START_DL_ADDR		(0xF010)
#define OIS_COMPLETE_DL_ADDR	(0xF006)
#define OIS_READ_STATUS_ADDR	(0x6024)
#define OIS_CHECK_SUM_ADDR		(0xF008)

#define LIMIT_STATUS_POLLING	(10)
#define LIMIT_OIS_ON_RETRY		(5)

#define GYRO_SCALE_FACTOR 262
#define HALL_SCALE_FACTOR 187
#define NUM_GYRO_SAMLPING (10)

static int16_t g_gyro_offset_value_x, g_gyro_offset_value_y = 0;

static struct ois_i2c_bin_list LGIT_VER14_REL_BIN_DATA =
{
	.files = 3,
	.entries =
	{
		{
			.filename = FIRMWARE_VER_BIN_1,
			.filesize = 0x0D7C,
			.blocks = 1,
			.addrs = {
				{0x0000,0x0D7B,0x0000},
				}
		},
		{
			.filename = FIRMWARE_VER_BIN_2,
			.filesize = 0x00D4,
			.blocks = 1,
			.addrs = {
				{0x0000,0x00D3,0x5400},
				}
		},
		{
			.filename = FIRMWARE_VER_BIN_3,
			.filesize = 0x0954,
			.blocks = 1,
			.addrs = {
				{0x0000,0x0953,0x1188},
				}
		}
	},
	.checksum = FIRMWARE_VER_CHECKSUM
};

static int lgit2_ois_poll_ready(int limit)
{
	uint8_t ois_status;
	int read_byte = 0;

	RegReadA(OIS_READ_STATUS_ADDR, &ois_status); //polling status ready
	read_byte++;

	while((ois_status != 0x01) && (read_byte < limit)) {
		RegReadA(OIS_READ_STATUS_ADDR, &ois_status); //polling status ready
		usleep(1000); //wait 1ms
		read_byte++;
	}

	return ois_status;
}

int lgit2_bin_download(struct ois_i2c_bin_list bin_list)
{
	int rc = 0;
	int cnt = 0;
	int32_t read_value_32t;

	CDBG("%s\n", __func__);

	/* check OIS ic is alive */
	if (!lgit2_ois_poll_ready(LIMIT_STATUS_POLLING)) {
		printk("%s: no reply 1\n",__func__);
		rc = OIS_INIT_I2C_ERROR;
		goto END;
	}

	/* Send command ois start dl */
	rc = RegWriteA(OIS_START_DL_ADDR, 0x00);

	while (rc < 0 && cnt < LIMIT_STATUS_POLLING) {
		usleep(2000);
		rc = RegWriteA(OIS_START_DL_ADDR, 0x00);
		cnt ++;
	}

	if (rc < 0) {
		printk("%s: no reply \n",__func__);
		rc = OIS_INIT_I2C_ERROR;
		goto END;
	}

	/* OIS program downloading */
	rc = ois_i2c_load_and_write_bin_list(bin_list);
	if (rc < 0)
	{
		goto END;
	}

	/* Check sum value!*/
	RamRead32A( OIS_CHECK_SUM_ADDR , &read_value_32t );
	if(read_value_32t != bin_list.checksum)
	{
		printk("%s: sum = 0x%x\n",__func__, read_value_32t);
		rc = OIS_INIT_CHECKSUM_ERROR;
		goto END;
	}

	rc = ois_i2c_load_and_write_e2prom_data(E2P_FIRST_ADDR, E2P_DATA_BYTE, CTL_END_ADDR_FOR_E2P_DL);
	if (rc < 0)
	{
		goto END;
	}

	/* Send command ois complete dl */
	RegWriteA(OIS_COMPLETE_DL_ADDR, 0x00) ;

	/* Read ois status */
	if (!lgit2_ois_poll_ready(LIMIT_STATUS_POLLING))
	{
			printk("%s: no reply 3\n",__func__);
			rc = OIS_INIT_TIMEOUT;
			goto END;
	}

	printk("%s, complete dl FINISHED! \n",__func__);

END:
	return rc;
}

int lgit2_ois_init_cmd(int limit)
{
	int trial = 0;

	do{
		RegWriteA(0x6020, 0x01);
		trial++;
	} while(trial<limit && !lgit2_ois_poll_ready(LIMIT_STATUS_POLLING));

	if (trial == limit) { return OIS_INIT_TIMEOUT; }

	RegWriteA(0x6023, 0x04);// gyro on
	if (!lgit2_ois_poll_ready(LIMIT_STATUS_POLLING)) { return OIS_INIT_TIMEOUT; }

	return OIS_SUCCESS;
}

static struct msm_ois_fn_t lgit2_ois_func_tbl;

int lgit2_ois_mode(enum ois_mode_t data)
{
	int cur_mode = lgit2_ois_func_tbl.ois_cur_mode;
	printk("%s:%d\n", __func__,data);

	if (cur_mode == data)
		return OIS_SUCCESS;

	if (cur_mode != OIS_MODE_CENTERING_ONLY) {
		RegWriteA(0x6020, 0x01);
		if (!lgit2_ois_poll_ready(LIMIT_STATUS_POLLING)) { return OIS_INIT_TIMEOUT; }
	}

	switch (data) {
	case OIS_MODE_PREVIEW_CAPTURE:
	case OIS_MODE_CAPTURE:
		CDBG("%s:%d, %d preview capture \n", __func__,data, cur_mode);
		RegWriteA(0x6021, 0x10); // zero shutter mode 2
		RegWriteA(0x6020, 0x02);
		break;
	case OIS_MODE_VIDEO:
		CDBG("%s:%d, %d capture \n", __func__,data, cur_mode);
		RegWriteA(0x6021, 0x11);
		RegWriteA(0x6020, 0x02);
		break;
	case OIS_MODE_CENTERING_ONLY:
		CDBG("%s:%d, %d centering_only \n", __func__,data, cur_mode);
		break;
	case OIS_MODE_CENTERING_OFF:
		CDBG("%s:%d, %d centering_off \n", __func__,data, cur_mode);
		RegWriteA(0x6020, 0x00); //lens centering off
		if (!lgit2_ois_poll_ready(LIMIT_STATUS_POLLING)) { return OIS_INIT_TIMEOUT; }
		break;
	}

	lgit2_ois_func_tbl.ois_cur_mode = data;

	return OIS_SUCCESS;
}

int lgit_ois_calibration(int ver)
{
	int16_t gyro_offset_value_x, gyro_offset_value_y = 0;

	CDBG("%s: lgit_ois_calibration start \n", __func__);
	//Gyro Zero Calibration Starts.
	RegWriteA(0x6020,0x01);//update mode & servo on & ois off mode
	if (!lgit2_ois_poll_ready(LIMIT_STATUS_POLLING)) {
		printk("%s 0x6024 result fail 1 @01\n", __func__);
		return OIS_INIT_TIMEOUT;
	}
	//Gyro On
	RegWriteA(0x6023,0x00);
	if (!lgit2_ois_poll_ready(LIMIT_STATUS_POLLING)) {
		printk("%s 0x6024 result fail 3 @02\n", __func__);
		return OIS_INIT_TIMEOUT;
	}
	//X
	RegWriteA(0x6088,0);
	//usleep(50000);
	if (!lgit2_ois_poll_ready(LIMIT_STATUS_POLLING + 30)) {
		printk("%s 0x6024 result fail 3 @03\n", __func__);
		return OIS_INIT_TIMEOUT;
	}
	RegReadB(0x608A, &gyro_offset_value_x);
	if (!lgit2_ois_poll_ready(LIMIT_STATUS_POLLING)) {
		printk("%s 0x6024 result fail 3 @04\n", __func__);
		return OIS_INIT_TIMEOUT;
	}
	//Y
	RegWriteA(0x6088,1);
	//usleep(50000);
	if (!lgit2_ois_poll_ready(LIMIT_STATUS_POLLING + 30)) {
		CDBG("%s 0x6024 result fail 3 @05\n", __func__);
		return OIS_INIT_TIMEOUT;
	}
	RegReadB(0x608A, &gyro_offset_value_y);
	if (!lgit2_ois_poll_ready(LIMIT_STATUS_POLLING)) {
		printk("%s 0x6024 result fail 3 @06A\n", __func__);
		return OIS_INIT_TIMEOUT;
	}

	//Cal. Dato to eeprom
	ois_i2c_e2p_write(0x0908, (uint16_t)(0xFFFF & gyro_offset_value_x), 2);
	ois_i2c_e2p_write(0x090A, (uint16_t)(0xFFFF & gyro_offset_value_y), 2); //gyro_offset_value_x -> gyro_offset_value_y로 수정함.(김형관)

	//Cal. Data to OIS Driver
	RegWriteA(0x609C, 0x00);
	RamWriteA(0x609D, gyro_offset_value_x); //16
	RegWriteA(0x609C, 0x01);
	RamWriteA(0x609D, gyro_offset_value_y); //16

	//	Gyro Zero Calibration Ends.
	CDBG("%s gyro_offset_value_x %d gyro_offset_value_y %d \n", __func__, gyro_offset_value_x, gyro_offset_value_y);
	g_gyro_offset_value_x = gyro_offset_value_x;
	g_gyro_offset_value_y = gyro_offset_value_y;

	CDBG("%s: lgit_ois_calibration end\n", __func__);
	return OIS_SUCCESS;
}
int32_t	lgit2_ois_on ( enum ois_ver_t ver  )
{
	int32_t rc = OIS_SUCCESS;
	uint16_t cal_ver = 0;

	printk("%s, %s\n", __func__,LAST_UPDATE);

	ois_i2c_e2p_read(E2P_FIRST_ADDR+0x1C, &cal_ver, 2);
	printk("%s ver %x\n", __func__, cal_ver);

	switch (cal_ver) {
	case 0xE:
		rc = lgit2_bin_download(LGIT_VER14_REL_BIN_DATA);
		break;
	default:
		rc = OIS_INIT_NOT_SUPPORTED;
		break;
	}

	if (rc < 0)	{
		printk("%s: init fail \n", __func__);
		return rc;
	}

	switch (ver) {
	case OIS_VER_RELEASE:
		lgit2_ois_init_cmd(LIMIT_OIS_ON_RETRY);

		/* OIS ON */
		RegWriteA(0x6021, 0x12);//LGIT STILL & PAN ON MODE
		RegWriteA(0x6020, 0x02);//OIS ON

		if (!lgit2_ois_poll_ready(5))
			return OIS_INIT_TIMEOUT;

		break;
	case OIS_VER_CALIBRATION:
	case OIS_VER_DEBUG:
		lgit_ois_calibration(ver);

		/* OIS ON */
		RegWriteA(0x6021, 0x12);//LGIT STILL & PAN ON MODE
		RegWriteA(0x6020, 0x02);//OIS ON

		if (!lgit2_ois_poll_ready(5))
			return OIS_INIT_TIMEOUT;

		break;
	}

	lgit2_ois_func_tbl.ois_cur_mode = OIS_MODE_CENTERING_ONLY;

	CDBG("%s : complete!\n", __func__);
	return rc;
}

int32_t	lgit2_ois_off(void)
{
	printk("%s enter\n", __func__);

	RegWriteA(0x6020, 0x01);
	if (!lgit2_ois_poll_ready(LIMIT_STATUS_POLLING))  {
		printk("%s poll time out\n", __func__);
		return OIS_INIT_TIMEOUT;
	}

	printk("%s exit\n", __func__);
	return OIS_SUCCESS;
}

int lgit2_ois_stat(struct msm_sensor_ois_info_t *ois_stat)
{
	int16_t val_hall_x;
	int16_t val_hall_y;

	//float gyro_scale_factor_idg2020 = (1.0)/(262.0);
	short int val_gyro_x;
	short int val_gyro_y;
	//Hall Fail Spec.
	short int spec_hall_x_lower = 1467;//+-0.65deg확보하기 위하여 45.0um 필요함.(실제 Spec.은 +-0.5deg이므로 충분한 마진 포함됨)
	short int spec_hall_x_upper = 2629;
	short int spec_hall_y_lower = 1467;
	short int spec_hall_y_upper = 2629;

	snprintf(ois_stat->ois_provider, ARRAY_SIZE(ois_stat->ois_provider), "LGIT_ROHM");

	//Gyro Read by reg
	RegWriteA(0x609C, 0x02);
	RamReadA(0x609D, &val_gyro_x);
	RegWriteA(0x609C, 0x03);
	RamReadA(0x609D, &val_gyro_y);

	ois_stat->gyro[0] = (int16_t)val_gyro_x;
	ois_stat->gyro[1] = (int16_t)val_gyro_y;

	//Hall Fail
	//Read Hall X
	RegWriteA(0x6060, 0x00);
	if (!lgit2_ois_poll_ready(LIMIT_STATUS_POLLING + 30)) {
		printk("%s 0x6024 result fail 3 @09\n", __func__);
		return OIS_INIT_TIMEOUT;
	}
	RegReadB(0x6062, &val_hall_x);

	//Read Hall Y
	RegWriteA(0x6060, 0x01);
	if (!lgit2_ois_poll_ready(LIMIT_STATUS_POLLING + 30)) {
		printk("%s 0x6024 result fail 3 @10\n", __func__);
		return OIS_INIT_TIMEOUT;
	}

	RegReadB(0x6062, &val_hall_y);

	ois_stat->hall[0] = val_hall_x;
	ois_stat->hall[1] = val_hall_y;

	ois_stat->is_stable = 1;

#if 0
	CDBG("%s val_hall_x(%d) -> 0x%x g_gyro_offset_value_x (%d)\n", __func__,
		val_hall_x, ois_stat->gyro[0], g_gyro_offset_value_x );
	CDBG("%s val_hall_y(%d) -> 0x%x g_gyro_offset_value_y (%d)\n", __func__,
		val_hall_y, ois_stat->gyro[1], g_gyro_offset_value_y);
#endif

	if (abs(val_gyro_x) > (25 * 262) ||
		abs(val_gyro_y) > (25 * 262)) {
		printk("Gyro Offset X is FAIL!!! (%d) \n", val_gyro_x);
		printk("Gyro Offset Y is FAIL!!! (%d) \n", val_gyro_y);
		ois_stat->is_stable = 0;
	}

	//Hall Spec. Out?
	if(val_hall_x > spec_hall_x_upper || val_hall_x < spec_hall_x_lower) {
		printk("val_hall_x is FAIL!!! (%d) 0x%x\n", val_hall_x,
			val_hall_x);
		ois_stat->is_stable = 0;
	}

	if(val_hall_y > spec_hall_y_upper || val_hall_y < spec_hall_y_lower) {
		printk("val_hall_y is FAIL!!! (%d) 0x%x\n", val_hall_y,
			val_hall_y);
		ois_stat->is_stable = 0;
	}
	ois_stat->target[0] = 0; /* not supported */
	ois_stat->target[1] = 0; /* not supported */

	return 0;
}

int32_t lgit2_ois_move_lens(int16_t target_x, int16_t target_y)
{
	int8_t hallx =  target_x / HALL_SCALE_FACTOR;
	int8_t hally =  target_y / HALL_SCALE_FACTOR;
	uint8_t result = 0;

	/* check ois mode & change to suitable mode */
	RegReadA(0x6020, &result);
	if (result != 0x01) {
		RegWriteA(0x6020, 0x01);
		if (!lgit2_ois_poll_ready(LIMIT_STATUS_POLLING))
			return OIS_INIT_TIMEOUT;
	}

	printk("%s target : %d(0x%x), %d(0x%x)\n", __func__,
		hallx, hallx, hally, hally);

	/* hallx range -> D2 to 2E (-46, 46) */
	RegWriteA(0x6099, 0xFF & hallx); /* target x position input */
	RegWriteA(0x609A, 0xFF & hally); /* target y position input */
	/* wait 100ms */
	usleep(100000);
	RegWriteA(0x6098, 0x01); /* order to move. */

	if (!lgit2_ois_poll_ready(LIMIT_STATUS_POLLING * 12))
		return OIS_INIT_TIMEOUT;

	RegReadA(0x609B, &result);

	RegWriteA(0x6023, 0x00);//Gyro On
	RegWriteA(0x6021, 0x12);//LGIT STILL & PAN ON MODE
	RegWriteA(0x6020, 0x02);//OIS ON

	if (result == 0x03)
		return  OIS_SUCCESS;

	printk("%s move fail : 0x%x \n", __func__, result);
	return OIS_FAIL;
}

void lgit2_ois_init(struct msm_ois_ctrl_t *msm_ois_t)
{
	lgit2_ois_func_tbl.ois_on = lgit2_ois_on;
    lgit2_ois_func_tbl.ois_off = lgit2_ois_off;
    lgit2_ois_func_tbl.ois_mode = lgit2_ois_mode;
    lgit2_ois_func_tbl.ois_stat = lgit2_ois_stat;
	lgit2_ois_func_tbl.ois_move_lens = lgit2_ois_move_lens;
 	lgit2_ois_func_tbl.ois_cur_mode = OIS_MODE_CENTERING_ONLY;

	msm_ois_t->sid_ois = 0x7C >> 1;
	msm_ois_t->ois_func_tbl = &lgit2_ois_func_tbl;
}


