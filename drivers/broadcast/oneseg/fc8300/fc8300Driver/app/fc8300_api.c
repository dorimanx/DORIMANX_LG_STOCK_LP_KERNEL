#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/kernel.h>

//#include "oneseg_driver_if.h"
#include "fc8300_regs.h"
#include "fci_dal.h"
#include "fci_oal.h"
#include "fci_types.h"

int mtv_drv_open(HANDLE *hDevice)
{
	int handle;

	handle = open("/dev/isdbt", O_RDWR | O_NDELAY);

	if(handle < 0)
	{
		printf("Cannot open handle : %d\n", handle);
		return BBM_NOK;
	}

	*hDevice = handle;

	return BBM_OK;
}

int mtv_drv_close(HANDLE hDevice)
{
	int res = BBM_NOK;

	res = close(hDevice);

	return BBM_OK;
}

void mtv_power_on(HANDLE hDevice)
{
	BBM_POWER_ON(hDevice);
}

void mtv_power_off(HANDLE hDevice)
{
	BBM_POWER_OFF(hDevice);
}

int mtv_init(HANDLE hDevice)
{
	int res = BBM_NOK;

	res = BBM_INIT(hDevice);

	return res;
}

int mtv_set_channel(HANDLE hDevice, u32 freq)
{
	int res = BBM_NOK;

	res = BBM_TUNER_SET_FREQ(hDevice, freq);

	return  res;
}

int mtv_lock_check(HANDLE hDevice)
{
	int res;

	res = BBM_SCAN_STATUS(hDevice);

	return res;
}

int mtv_data_read(HANDLE hDevice, u8 *buf, u32 bufSize)
{
	s32 outSize;

	outSize = read(hDevice, buf, bufSize);

	return outSize;
}

void mtv_ts_start(HANDLE hDevice)
{
	BBM_TS_START(hDevice);
}

void mtv_ts_stop(HANDLE hDevice)
{
	BBM_TS_STOP(hDevice);
}

int mtv_signal_quality_info(HANDLE hDevice, u8 *Lock, u8 *CN, u32 *ui32BER_A, u32 *ui32PER_A, u32 *ui32BER_B, u32 *ui32PER_B, u32 *ui32BER_C, u32 *ui32PER_C, s32 *i32RSSI, u32 *ui32Quality, u32 *ui32Antlvl)
{
	u8  mod_info, fd;
	s32 res;

	struct dm_st {
		u8  start;
		s8  rssi;
		u8  sync_0;
		u8  sync_1;

		u8  fec_on;
		u8  fec_layer;
		u8  wscn;
		u8  reserved;

		u16 vit_a_ber_rxd_rsps;
		u16 vit_a_ber_err_rsps;
		u32 vit_a_ber_err_bits;

		u16 vit_b_ber_rxd_rsps;
		u16 vit_b_ber_err_rsps;
		u32 vit_b_ber_err_bits;

		u16 vit_c_ber_rxd_rsps;
		u16 vit_c_ber_err_rsps;
		u32 vit_c_ber_err_bits;

		u16 reserved0;
		u16 reserved1;
		u32 reserved2;

		u32 dmp_a_ber_rxd_bits;
		u32 dmp_a_ber_err_bits;

		u32 dmp_b_ber_rxd_bits;
		u32 dmp_b_ber_err_bits;

		u32 dmp_c_ber_rxd_bits;
		u32 dmp_c_ber_err_bits;

		u32 reserved3;
		u32 reserved4;
	} dm;

	res = BBM_BULK_READ(hDevice, BBM_DM_DATA, (u8*) &dm, sizeof(dm));

	if(res)
		print_log("mtv_signal_measure Error res : %d\n");

	if(dm.sync_1 & 0x02)
		*Lock = 1;
	else
		*Lock =0;

	if (dm.vit_a_ber_rxd_rsps)
		*ui32PER_A = ((double) dm.vit_a_ber_err_rsps / (double) dm.vit_a_ber_rxd_rsps) * 10000;
	else
		*ui32PER_A = 10000;

	if (dm.vit_b_ber_rxd_rsps)
		*ui32PER_B = ((double) dm.vit_b_ber_err_rsps / (double) dm.vit_b_ber_rxd_rsps) * 10000;
	else
		*ui32PER_B = 10000;

	if (dm.vit_c_ber_rxd_rsps)
		*ui32PER_C = ((double) dm.vit_c_ber_err_rsps / (double) dm.vit_c_ber_rxd_rsps) * 10000;
	else
		*ui32PER_C = 10000;

	if (dm.dmp_a_ber_rxd_bits)
		*ui32BER_A = ((double) dm.dmp_a_ber_err_bits / (double) dm.dmp_a_ber_rxd_bits) * 10000;
	else
		*ui32BER_A = 10000;

	if (dm.dmp_b_ber_rxd_bits)
		*ui32BER_B = ((double) dm.dmp_b_ber_err_bits / (double) dm.dmp_b_ber_rxd_bits) * 10000;
	else
		*ui32BER_B = 10000;

	if (dm.dmp_c_ber_rxd_bits)
		*ui32BER_C = ((double) dm.dmp_c_ber_err_bits / (double) dm.dmp_c_ber_rxd_bits) * 10000;
	else
		*ui32BER_C = 10000;

	*i32RSSI = (signed char) dm.rssi;

	/* WSCN			 */
	BBM_READ(hDevice, 0x4113, &mod_info);

	mod_info = mod_info & 0x70;

	BBM_READ(hDevice, 0x4066, &fd);

	if (fd < 50) {
		if (mod_info == 0x40) { /* QPSK */
			if (dm.wscn <= 2)
				dm.wscn = 0;
			else if (dm.wscn == 3)
				dm.wscn = dm.wscn - 2;
			else if (dm.wscn == 4)
				dm.wscn = dm.wscn - 1;
		}
		else if (mod_info == 0x20) { /* 16QAM */
			if (dm.wscn >= 0 && dm.wscn <= 4)
				dm.wscn = 0;
			else if (dm.wscn >= 5 && dm.wscn <= 8)
				dm.wscn = dm.wscn - 5;
			else if (dm.wscn == 9)
				dm.wscn = dm.wscn - 4;
			else if (dm.wscn >= 10 && dm.wscn <=11)
				dm.wscn = dm.wscn - 3;
			else if (dm.wscn == 12)
				dm.wscn = dm.wscn - 2;
			else if (dm.wscn == 13)
				dm.wscn = dm.wscn - 1;
		}
	}
	else if (fd < 90) {
		if (mod_info == 0x40) {/* QPSK */
			if (dm.wscn <=	2)
				dm.wscn = 0;
			else if (dm.wscn ==  3)
				dm.wscn = dm.wscn - 2;
			else if (dm.wscn ==  4)
				dm.wscn = dm.wscn - 1;
			else if (dm.wscn <= 8)
				dm.wscn = dm.wscn + 1;
			else if (dm.wscn ==  9)
				dm.wscn = dm.wscn + 2;
			else if (dm.wscn == 10)
				dm.wscn = dm.wscn + 3;
			else if (dm.wscn == 11)
				dm.wscn = dm.wscn + 4;
			else if (dm.wscn == 12)
				dm.wscn = dm.wscn + 2;
			else if (dm.wscn == 13)
				dm.wscn = dm.wscn + 3;
			else if (dm.wscn >= 14)
				dm.wscn = dm.wscn + 5;
		}
		else if (mod_info == 0x20) { /* 16QAM */
			if (dm.wscn >= 0 && dm.wscn <= 4)
				dm.wscn = 0;
			else if (dm.wscn >= 5 && dm.wscn <= 7)
				dm.wscn = dm.wscn - 4;
			else if (dm.wscn ==  8)
				dm.wscn = dm.wscn - 3;
			else if (dm.wscn ==  9)
				dm.wscn = dm.wscn - 2;
			else if (dm.wscn <= 12)
				dm.wscn = dm.wscn + 1;
			else if (dm.wscn == 13)
				dm.wscn = dm.wscn + 2;
			else if (dm.wscn == 14)
				dm.wscn = dm.wscn + 2;
			else if (dm.wscn >= 15)
				dm.wscn = dm.wscn + 3;
		}
	}
	else {
		if (mod_info == 0x40) {/* QPSK */
			if (dm.wscn <=	2)
				dm.wscn = 0;
			else if (dm.wscn == 3)
				dm.wscn = dm.wscn - 2;
			else if (dm.wscn == 4)
				dm.wscn = dm.wscn - 1;
			else if (dm.wscn <= 7)
				dm.wscn = dm.wscn + 1;
			else if (dm.wscn == 8)
				dm.wscn = dm.wscn + 2;
			else if (dm.wscn == 9)
				dm.wscn = dm.wscn + 3;
			else if (dm.wscn == 10)
				dm.wscn = dm.wscn + 5;
			else if (dm.wscn == 11)
				dm.wscn = dm.wscn + 3;
			else if (dm.wscn == 12)
				dm.wscn = dm.wscn + 4;
			else if (dm.wscn == 13)
				dm.wscn = dm.wscn + 5;
			else if (dm.wscn >= 14)
				dm.wscn = dm.wscn + 10;
		}
		else if (mod_info == 0x20) { /* 16QAM */
			if (dm.wscn >= 0 && dm.wscn <= 5)
				dm.wscn = 0;
			else if (dm.wscn == 6)
				dm.wscn = dm.wscn - 5;
			else if (dm.wscn == 7)
				dm.wscn = dm.wscn - 3;
			else if (dm.wscn == 8)
				dm.wscn = dm.wscn - 2;
			else if (dm.wscn == 9)
				dm.wscn = dm.wscn - 1;
			else if (dm.wscn <= 11)
				dm.wscn = dm.wscn + 2;
			else if (dm.wscn == 12)
				dm.wscn = dm.wscn + 4;
			else if (dm.wscn >= 13)
				dm.wscn = dm.wscn + 7;
		}
	}
	*CN = dm.wscn;

	return BBM_OK;
}

