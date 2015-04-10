/*
   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   Copyright (c) 2011 Synaptics, Inc.

   Permission is hereby granted, free of charge, to any person obtaining a copy of
   this software and associated documentation files (the "Software"), to deal in
   the Software without restriction, including without limitation the rights to use,
   copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
   Software, and to permit persons to whom the Software is furnished to do so,
   subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   SOFTWARE.

   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*/
#include "RefCode_F54.h"
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>	//msleep
#include <linux/file.h>		//for file access
#include <linux/syscalls.h> //for file access
#include <linux/uaccess.h>  //for file access
#include <linux/firmware.h>

static char line[49152]={0};
int UpperImage[32][32];
int LowerImage[32][32];
int SensorSpeedUpperImage[32][32];
int SensorSpeedLowerImage[32][32];
int ADCUpperImage[32][32];
int ADCLowerImage[32][32];

extern struct i2c_client* ds4_i2c_client;
extern int touch_i2c_read(struct i2c_client *client, u8 reg, int len, u8 *buf);
extern int touch_i2c_write(struct i2c_client *client, u8 reg, int len, u8 * buf);

int Read8BitRegisters(unsigned short regAddr, unsigned char *data, int length)
{
	// I2C read
	int rst = 0;
	
	rst = touch_i2c_read(ds4_i2c_client, regAddr, length, data);
	
	return rst;
}

int Write8BitRegisters(unsigned short regAddr, unsigned char *data, int length)
{
	// I2C write
	int rst = 0;
	
	rst = touch_i2c_write(ds4_i2c_client, regAddr, length, data);
	
	return rst;
}

void delayMS(int val)
{
	// Wait for val MS
	msleep(val);
}

int write_file(char *filename, char *data)
{
	int fd = 0;

	fd = sys_open(filename, O_WRONLY|O_CREAT|O_APPEND, 0666);
	if (fd < 0) {
		TOUCH_INFO_MSG("%s :  Open file error [ %d ]\n",__func__, fd);
		return fd;
	} else {
		sys_write(fd, data, strlen(data));
		sys_close(fd);
	}
	return 0;
}


int write_log(char *filename, char *data)
{
	extern int f54_window_crack;
	extern int f54_window_crack_check_mode;

	int fd;
	char *fname = "/mnt/sdcard/touch_self_test.txt";
	int cap_file_exist = 0;

	if(f54_window_crack||f54_window_crack_check_mode==0) {
		mm_segment_t old_fs = get_fs();
		set_fs(KERNEL_DS);
		if(filename == NULL){
			fd = sys_open(fname, O_WRONLY|O_CREAT|O_APPEND, 0666);
			TOUCH_INFO_MSG("write log in /mnt/sdcard/touch_self_test.txt\n");
		} else{
			fd = sys_open(filename, O_WRONLY|O_CREAT, 0666);
			TOUCH_INFO_MSG("write log in /sns/touch/cap_diff_test.txt\n");
		}

		TOUCH_INFO_MSG("[%s]write file open %s, fd : %d\n", __FUNCTION__, (fd >= 0)? "success": "fail", fd);

		if(fd >= 0) {
			/*because of erro, this code is blocked.*/
			/*if(sys_newstat((char __user *) fname, (struct stat *)&fstat) < 0) {
			  printk("[Touch] cannot read %s stat info\n", fname);
			  } else {
			  if(fstat.st_size > 5 * 1024 * 1024) {
			  printk("[Touch] delete %s\n", fname);
			  sys_unlink(fname);
			  sys_close(fd);

			  fd = sys_open(fname, O_WRONLY|O_CREAT|O_APPEND, 0644);
			  if(fd >= 0) {
			  sys_write(fd, data, strlen(data));
			  }
			  } else {
			  sys_write(fd, data, strlen(data));
			  }
			  sys_close(fd);
			  }*/
			sys_write(fd, data, strlen(data));
			sys_close(fd);

			if(filename != NULL)
				cap_file_exist = 1;
		}
		set_fs(old_fs);
	}
	return cap_file_exist;
}

void read_log(char *filename, const struct touch_platform_data *pdata)
{
	int fd;
	char* buf = NULL;
	int rx_num = 0;
	int tx_num = 0;
	int data_pos = 0;
	int offset = 0;

	struct touch_platform_data *ppdata = (struct touch_platform_data*)pdata;

	mm_segment_t old_fs = get_fs();
	set_fs(KERNEL_DS);

	fd = sys_open(filename, O_RDONLY, 0);
	buf = kzalloc(1024, GFP_KERNEL);

	TOUCH_INFO_MSG("[%s]read file open %s, fd : %d\n", __FUNCTION__, (fd >= 0)? "success": "fail", fd);

	if (fd >= 0) {
		TOUCH_INFO_MSG("[%s]open read_log funcion in /sns/touch/cap_diff_test.txt\n",__FUNCTION__);
		while(sys_read(fd, buf, 1024)) {

			TOUCH_INFO_MSG("[%s]sys_read success\n",__FUNCTION__);

			for(rx_num = 0; rx_num < (ppdata->rx_ch_count) - 1; rx_num++)
			{
				sscanf(buf + data_pos, "%d%n", &ppdata->rx_cap[rx_num], &offset);
				data_pos += offset;
			}

			for(tx_num = 0; tx_num < (ppdata->tx_ch_count) - 1; tx_num++)
			{
				sscanf(buf + data_pos, "%d%n", &ppdata->tx_cap[tx_num], &offset);
				data_pos += offset;
			}

			TOUCH_INFO_MSG("[%s]rx_num = %d, tx_num = %d\n", __FUNCTION__, rx_num, tx_num);
			TOUCH_INFO_MSG("[%s]rx_ch_count = %d, tx_ch_count = %d\n", __FUNCTION__, ppdata->rx_ch_count, ppdata->tx_ch_count);

			if((rx_num == (ppdata->rx_ch_count) -1) && (tx_num == (ppdata->tx_ch_count) -1))
				break;
		}

		sys_close(fd);
	}

	if(buf)
		kfree(buf);

	set_fs(old_fs);

}

int get_limit(unsigned char Tx, unsigned char Rx, struct i2c_client client, const struct touch_platform_data *pdata, char *breakpoint, int limit_data[32][32])
{
	int p = 0;
	int q = 0;
	int r = 0;
	int cipher = 1;
	int ret = 0;
	int rx_num = 0;
	int tx_num = 0;
	const struct firmware *fwlimit = NULL;

	TOUCH_INFO_MSG("[%s] Tx=[%d], Rx=[%d]\n", __FUNCTION__, (int)Tx, (int)Rx);
	TOUCH_INFO_MSG("[%s] breakpoint = [%s]\n", __FUNCTION__, breakpoint);

	if (pdata->panel_spec == NULL) {
		TOUCH_INFO_MSG("panel_spec_file name is null\n");
		ret =  -1;
		goto exit;
	}

	if(request_firmware(&fwlimit, pdata->panel_spec, &client.dev) < 0) {
		TOUCH_INFO_MSG(" request ihex is failed\n");
		ret =  -1;
		goto exit;
	}

	strcpy(line, fwlimit->data);

	q = strstr(line, breakpoint) - line;

	if (q < 0) {
		TOUCH_INFO_MSG("failed to find breakpoint. The panel_spec_file is wrong");
		ret =  -1;
		goto exit;
	}

	memset(limit_data, 0, (TRX_max * TRX_max) * 4);

	while(1) {
		if (line[q] == ',') {
			cipher = 1;
			for (p = 1; (line[q - p] >= '0') && (line[q - p] <= '9'); p++) {
				limit_data[tx_num][rx_num] += ((line[q - p] - '0') * cipher);
				//printk("[r = %d]limit_data[%d][%d] = %d\n", r, tx_num, rx_num, limit_data[tx_num][rx_num]);
				cipher *= 10;
			}
			if(line[q - p] == '-') {
				limit_data[tx_num][rx_num] = (-1) * (limit_data[tx_num][rx_num]);
				//printk("[r = %d]limit_data[%d][%d] = %d\n", r, tx_num, rx_num, limit_data[tx_num][rx_num]);
			}
			r++;

			if(r % (int)Rx == 0){
				rx_num = 0;
				tx_num++;
			} else {
				rx_num++;
			}
		}
		q++;

		if (r == (int)Tx * (int)Rx) {
			TOUCH_INFO_MSG("panel_spec_file scanning is success\n");
			break;
		}
	}

	if (fwlimit)
		release_firmware(fwlimit);

	return ret;

exit :
	if (fwlimit)
		release_firmware(fwlimit);

	return ret;
}
#if 0
void main(void)
	/* Please be informed this main() function & related functions are an example for host implementation */
{
	PowerOnSensor();
	delayMS(400);

	F54Test();
	PowerOffSensor();
}
#endif
