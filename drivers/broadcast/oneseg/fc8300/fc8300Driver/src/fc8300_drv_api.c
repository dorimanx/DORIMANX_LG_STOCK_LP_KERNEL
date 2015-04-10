/*****************************************************************************
    Copyright(c) 2013 FCI Inc. All Rights Reserved

    File name : fc8300_drv_api.c

    Description : wrapper source of FC8300 tuner driver

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

    History :
    ----------------------------------------------------------------------
******************************************************************************/

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include "bbm.h"
#include "fci_oal.h"
#include "fci_tun.h"
#include "fc8300_regs.h"
#include "fc8300_isr.h"
#include "fci_hal.h"
#include "fci_types.h"
#include "fc8300_drv_api.h"

#define FEATURE_COMPENSATE_MEASURE
#ifdef FEATURE_COMPENSATE_MEASURE
#define MAX_BER_CNT 3
u32 arrnode[3][MAX_BER_CNT];
u32 compcnt;
#endif

/* user stop */
static int fc8300_user_stop_called = 0;

int fc8300_if_test(void)
{
    int res=0;
    int i;
    u16 wdata=0;
    u32 ldata=0;
    u8 data=0;
    u8 temp = 0;

	print_log(0, "fc8300_if_test Start!!!\n");
	for (i = 0; i < 100 ; i++) {
		bbm_com_byte_write(0, DIV_BROADCAST, 0xa4, i&0xff);
		bbm_com_byte_read(0, DIV_BROADCAST, 0xa4, &data);
		if ((i & 0xff) != data) {
			print_log(0, "fc8300_if_btest!   i=0x%x, data=0x%x\n"
				, i&0xff, data);
			res = 1;
		}
	}

	for (i = 0 ; i < 100 ; i++) {
		bbm_com_word_write(0, DIV_BROADCAST, 0xa4, i&0xffff);
		bbm_com_word_read(0, DIV_BROADCAST, 0xa4, &wdata);
		if ((i & 0xffff) != wdata) {
			print_log(0, "fc8300_if_wtest!   i=0x%x, data=0x%x\n"
				, i & 0xffff, wdata);
			res = 1;
		}
	}

	for (i = 0 ; i < 100; i++) {
		bbm_com_long_write(0, DIV_BROADCAST, 0xa4, i&0xffffffff);
		bbm_com_long_read(0, DIV_BROADCAST, 0xa4, &ldata);
		if ((i & 0xffffffff) != ldata) {
			print_log(0, "fc8300_if_ltest!   i=0x%x, data=0x%x\n"
				, i & 0xffffffff, ldata);
			res = 1;
		}
	}

	for (i = 0 ; i < 100 ; i++) {
		temp = i & 0xff;
		bbm_com_tuner_write(NULL, DIV_BROADCAST
			, 0x7a, 0x01, &temp, 0x01);
		bbm_com_tuner_read(NULL, DIV_BROADCAST
			, 0x7a, 0x01, &data, 0x01);
		if ((i & 0xff) != data)
			print_log(0, "FC8300 tuner test (0x%x,0x%x)\n"
			, i & 0xff, data);
	}

	print_log(0, "fc8300_if_test End!!!\n");

	return res;
}

int tunerbb_drv_fc8300_init(int mode)
{
    int res;

    print_log(0, "[1seg][MTK] FC8300 tunerbb_drv_fc8300_init\n");
#ifndef BBM_I2C_TSIF
    res = bbm_com_hostif_select(NULL, BBM_SPI);
#else
	res = bbm_com_hostif_select(NULL, BBM_I2C);
#endif
    res |= bbm_com_i2c_init(NULL, FCI_HPI_TYPE);

	res |= bbm_com_probe(NULL, DIV_BROADCAST);
	print_log(NULL, "[FC8300] FC8300 BBM_PROBE res : %d \n", res);

	if (res)
		print_log(NULL, "[FC8300] FC8300 Initialize Fail : %d \n", res);

	res |= bbm_com_init(NULL, DIV_BROADCAST);
	res |= bbm_com_tuner_select(NULL, DIV_BROADCAST, FC8300_TUNER, mode);

	if (res)
		print_log(0, "[FC8300] IOCTL_ISDBT_POWER_ON FAIL \n");
	else
		print_log(0, "[FC8300] IOCTL_ISDBT_POWER_OK \n");

	return res;
}

int tunerbb_drv_fc8300_stop(void)
{
    s32 res = BBM_OK;

    res = bbm_com_hostif_deselect(NULL);

    return res;
}

int tunerbb_drv_fc8300_set_channel(s32 f_rf, u16 mode, u8 subch)
{
	s32 res = BBM_OK;
	subch = 0x16;

	bbm_com_tuner_set_freq(NULL, DIV_BROADCAST, f_rf, subch);
	print_log(NULL, "FC8300 tunerbb_drv_fc8300_set_channel %d, %d, %d\n"
		, f_rf, mode, subch);
	if (mode)
		res = bbm_com_scan_status(NULL, DIV_BROADCAST);

	return res;
}

void tunerbb_drv_fc8300_set_user_stop(int ustop)
{
    fc8300_user_stop_called = ustop;
}

int get_fc8300_ustop_state(void)
{
    int ret = 0;
    ret = (fc8300_user_stop_called == 1)?1:0;

    return ret;
}

int tunerbb_drv_fc8300_Get_SyncStatus(void)
{
	u8 data;
	s32 sync = 0;


	bbm_com_read(NULL, DIV_BROADCAST, 0x3025, &data);
	if (data & 0x08) {
		sync = 1;

		bbm_com_read(NULL, DIV_BROADCAST, 0x3026, &data);
		if (data & 0x02) {
			sync = 2;

			bbm_com_read(NULL, DIV_BROADCAST, 0x50c5, &data);
			if (data)
				sync = 3;
		}
	} else
		sync = 0;

	return sync;
}

static int tmcc_inter_length(u8 inter_len, u8 mode)
{
	switch (inter_len) {
	case 0x0:
		return 0;
	case 0x04:
		switch (mode & 0x03) {
		case 3:
			return 1;
		case 2:
			return 2;
		case 1:
			return 4;
		}
	case 0x02:
		switch (mode & 0x03) {
		case 3:
			return 2;
		case 2:
			return 4;
		case 1:
			return 8;
		}
	case 0x06:
		switch (mode & 0x03) {
		case 3:
			return 4;
		case 2:
			return 8;
		case 1:
			return 16;
		}
	}

	return 63;
}

static int tmcc_mod_scheme(u8 mod_scheme)
{
	switch (mod_scheme) {
	case 0x0:
		return 0;	/*"DQPSK"*/
	case 0x04:
		return 1;	/*"QPSK"*/
	case 0x02:
		return 2;	/*"16QAM"*/
	case 0x06:
		return 3;	/*"64QAM"*/
	}

	return 7;
}

static int tmcc_code_rate(u8 code_rate)
{
	switch (code_rate) {
	case 0x0:
		return 0;	/*"1/2"*/
	case 0x04:
		return 1;	/*"2/3"*/
	case 0x02:
		return 2;	/*"3/4"*/
	case 0x06:
		return 3;	/*"5/6"*/
	case 0x01:
		return 4;	/*"7/8"*/
	}

	return 7;
}

static int tmcc_num_segment(u8 num_segment)
{
	switch (num_segment) {
	case 0x08:
		return 1;
	case 0x04:
		return 2;
	case 0x0c:
		return 3;
	case 0x02:
		return 4;
	case 0x0a:
		return 5;
	case 0x06:
		return 6;
	case 0x0e:
		return 7;
	case 0x01:
		return 8;
	case 0x09:
		return 9;
	case 0x05:
		return 10;
	case 0x0d:
		return 11;
	case 0x03:
		return 12;
	case 0x0b:
		return 13;
	}

	return 0x0f;
}

static int tmcc_system_identification(u8 id)
{
	switch (id) {
	case 0x00:
		return 0;
	case 0x02:
		return 1;
	case 0x01:
		return 2;
	}

	return id;
}

static int tmcc_GI_mode(u8 gi)
{
	switch (gi) {
	case 1:
		return 0;
	case 2:
		return 1;
	case 3:
		return 2;
	case 4:
		return 3;
	}

	return 0;
}

int tunerbb_drv_fc8300_get_oneseg_antenna(int ant, int ber, int cn, s32 brd_type)
{
    u32 antlvl;
    if(brd_type == ISDBT_13SEG)
    {
        switch(ant)
        {
            case 0:
                if(ber < 680)
                    antlvl = ant = 1;
                else
                    antlvl = ant;
            break;
            case 1:
                if((ber > 700) && (cn < 500))
                    antlvl = ant = 0;
                if((ber < 300) || (cn > 700))
                    antlvl = ant = 2;
                else
                    antlvl = ant;
            break;
            case 2:
                if((ber > 400) && (600 > cn))
                    antlvl = ant = 1;
                if((ber < 50) || (900 < cn))
                    antlvl = ant = 3;
                else
                    antlvl = ant;
            break;
            case 3:
                if((ber > 100) && (700 > cn))
                    antlvl = ant = 2;
                else
                    antlvl = ant;
            break;
            default:
                antlvl = ant = 0;
            break;
        }
    }
    else
    {
        switch(ant)
        {
            case 0:
                if(ber < 1000)
                    antlvl = ant = 1;
                else
                    antlvl = ant;
            break;
            case 1:
                if((ber > 1100) && (cn <= 600))
                    antlvl = ant = 0;
                if((ber < 600) || (cn > 700))
                    antlvl = ant = 2;
                else
                    antlvl = ant;
            break;
            case 2:
                if((ber > 800) && (800 > cn))
                    antlvl = ant = 1;
                if((ber < 100) || (1000 < cn))
                    antlvl = ant = 3;
                else
                    antlvl = ant;
            break;
            case 3:
                if((ber > 150) && (900 > cn))
                    antlvl = ant = 2;
                else
                    antlvl = ant;
            break;
            default:
                antlvl = ant = 0;
            break;
        }
    }
    return antlvl;
}

int tunerbb_drv_fc8300_get_fullseg_antenna(int ant, int ber, int cn)
{
    u32 antlvl;
    switch(ant)
    {
        case 0:
            if((ber < 510) && (cn >= 1600))
                antlvl = ant = 1;
            else
                antlvl = ant;
        break;
        case 1:
            if((ber > 530) && (cn <= 1650))
                antlvl = ant = 0;
            if((ber < 450) || (cn > 1720))
                antlvl = ant = 2;
            else
                antlvl = ant;
        break;
        case 2:
            if((ber > 480) && (cn < 1680))
                antlvl = ant = 1;
            if((ber < 250) || (cn > 2300))
                antlvl = ant = 3;
            else
                antlvl = ant;
        break;
        case 3:
            if((ber > 300) && (cn < 1820))
                antlvl = ant = 2;
            else
                antlvl = ant;
        break;
        default:
            antlvl = ant = 0;
        break;
    }

    return antlvl;
}

#ifdef FEATURE_COMPENSATE_MEASURE
u32 tunerbb_drv_fc8300_measure_compval(u32 node, u32 compval)
{
    u32 tot = 0;
    u32 avg = 0;
    u32 i = 0;

    arrnode[node][compcnt % MAX_BER_CNT] = compval;

    compcnt++;
    if(compcnt >= 10)
        compcnt = 5;

    if(compcnt < MAX_BER_CNT)
    {
        for(i = 0; i < compcnt; i++)
            tot += arrnode[node][i];

        avg = tot / compcnt;
    }
    else
    {
        for(i = 0; i < MAX_BER_CNT; i++)
            tot += arrnode[node][i];

        avg = tot / MAX_BER_CNT;
    }

    if(compval < avg)
        avg = compval;

    return avg;
}
#endif

void mtv_monitor_mfd(void)
{
	u8 mfd_status;
	u8 mfd_value;
	static u8 master;

	bbm_com_byte_read(NULL, DIV_MASTER, 0x4064, &mfd_status);

	if (mfd_status & 0x01) {
		bbm_com_byte_read(NULL, DIV_MASTER, 0x4065, &mfd_value);

		if (mfd_value < 12) {
			if (master != 1) {
				bbm_com_byte_write(NULL, DIV_MASTER
					, 0x3022, 0x0b);
				bbm_com_byte_write(NULL, DIV_MASTER
					, 0x255c, 0x10);
				bbm_com_byte_write(NULL, DIV_MASTER
					, 0x2542, 0x04);
				master = 1;
			}
		} else {
			if (master != 2) {
				bbm_com_byte_write(NULL, DIV_MASTER
					, 0x3022, 0x0f);
				bbm_com_byte_write(NULL, DIV_MASTER
					, 0x255c, 0x3f);
				bbm_com_byte_write(NULL, DIV_MASTER
					, 0x2542, 0x10);
				master = 2;
			}
		}
	}
}

#define DC_OFFSET 150
int tunerbb_drv_fc8300_Get_SignalInfo(struct fc8300Status_t *st, s32 brd_type)
{
    u8 ovr;
    s32 res;
    u8 tmcc_data[26];
    u8 tmcc_done = 0;
    s32 rssi;
    u8 mode;
    u8 cn;
    u8 sync, data;
    u8 auto_switch;
    u32 rsber_A, rsber_B, rsber_C;
    u8 transmission_parameter_switching;
    struct dm_st {
        u8    start;
        s8    rssi;
        u8    sync_0;
        u8    sync_1;

        u8    fec_on;
        u8    fec_layer;
        u8    wscn;
        u8    reserved;

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

    res = bbm_com_bulk_read(NULL, DIV_MASTER, BBM_DM_DATA, (u8*) &dm, sizeof(dm));

    dm.vit_a_ber_err_bits = dm.vit_a_ber_err_bits & 0x00ffffff;
    dm.vit_b_ber_err_bits = dm.vit_b_ber_err_bits & 0x00ffffff;
    dm.vit_c_ber_err_bits = dm.vit_c_ber_err_bits & 0x00ffffff;

    dm.dmp_a_ber_err_bits = dm.dmp_a_ber_err_bits & 0x00ffffff;
    dm.dmp_b_ber_err_bits = dm.dmp_b_ber_err_bits & 0x00ffffff;
    dm.dmp_c_ber_err_bits = dm.dmp_c_ber_err_bits & 0x00ffffff;
#if 1
    bbm_com_read(NULL, DIV_BROADCAST, 0x8001, &ovr);
    st->agc = ovr;
    if(ovr)
    {
        bbm_com_write(NULL, DIV_BROADCAST, 0x8001, ovr);
    }
#endif
    if(res)
        print_log(NULL, "mtv_signal_measure Error res : %d\n");

    if (dm.vit_a_ber_rxd_rsps) {
        if(dm.vit_a_ber_err_rsps > 429496)
            st->per_a= ((u32)dm.vit_a_ber_err_rsps * 100 / ((u32) dm.vit_a_ber_rxd_rsps / 100));
        else
            st->per_a= ((u32)dm.vit_a_ber_err_rsps * 10000 / (u32) dm.vit_a_ber_rxd_rsps);
    }
    else
        st->per_a = 10000;

    if (dm.vit_b_ber_rxd_rsps) {
        if(dm.vit_b_ber_err_rsps > 429496)
            st->per_b = ((u32) dm.vit_b_ber_err_rsps * 100 / ((u32) dm.vit_b_ber_rxd_rsps / 100));
        else
            st->per_b = ((u32) dm.vit_b_ber_err_rsps * 10000 / (u32) dm.vit_b_ber_rxd_rsps);
    }
    else
        st->per_b = 10000;

    if (dm.vit_c_ber_rxd_rsps) {
        if(dm.vit_c_ber_err_rsps > 429496)
            st->per_c = ((u32) dm.vit_c_ber_err_rsps * 100 / ((u32) dm.vit_c_ber_rxd_rsps / 100));
        else
            st->per_c = ((u32) dm.vit_c_ber_err_rsps * 10000 / (u32) dm.vit_c_ber_rxd_rsps);
    }
    else
        st->per_c = 10000;

    if (dm.dmp_a_ber_rxd_bits) {
        if(dm.dmp_a_ber_err_bits > 429496)
            st->ber_a = ((u32) dm.dmp_a_ber_err_bits * 100 / ((u32) dm.dmp_a_ber_rxd_bits / 100));
        else
            st->ber_a= ((u32) dm.dmp_a_ber_err_bits * 10000 / (u32) dm.dmp_a_ber_rxd_bits);
    }
    else {
        st->ber_a = 10000;
    }

    if (dm.dmp_b_ber_rxd_bits) {
        if(dm.dmp_b_ber_err_bits > 429496)
            st->ber_b = ((u32) dm.dmp_b_ber_err_bits * 100 / ((u32) dm.dmp_b_ber_rxd_bits / 100));
        else
            st->ber_b = ((u32) dm.dmp_b_ber_err_bits * 10000 / (u32) dm.dmp_b_ber_rxd_bits);
    }
    else {
        st->ber_b = 10000;
    }

    if (dm.dmp_c_ber_rxd_bits) {
        if(dm.dmp_c_ber_err_bits > 429496)
            st->ber_c = ((u32) dm.dmp_c_ber_err_bits * 100 / ((u32) dm.dmp_c_ber_rxd_bits / 100));
        else
            st->ber_c = ((u32) dm.dmp_c_ber_err_bits * 10000 / (u32) dm.dmp_c_ber_rxd_bits);
    }
    else {
        st->ber_c = 10000;
    }

    if (dm.vit_a_ber_rxd_rsps) {
         if (dm.vit_a_ber_err_bits & 0x00600000)
            rsber_A= ((u32)dm.vit_a_ber_err_bits * 250 / ((u32) dm.vit_a_ber_rxd_rsps * 204/5));
         else
            rsber_A= ((u32)dm.vit_a_ber_err_bits * 625 / ((u32) dm.vit_a_ber_rxd_rsps * 102));
    }
    else
        rsber_A =  10000;

    if (dm.vit_b_ber_rxd_rsps) {
         if (dm.vit_b_ber_err_bits & 0x00600000)
            rsber_B = ((u32) dm.vit_b_ber_err_bits * 250 / ((u32) dm.vit_b_ber_rxd_rsps * 204/5));
         else
            rsber_B = ((u32) dm.vit_b_ber_err_bits * 625 / ((u32) dm.vit_b_ber_rxd_rsps * 102));
    }
    else
        rsber_B =  10000;

    if (dm.vit_c_ber_rxd_rsps) {
         if (dm.vit_c_ber_err_bits & 0x00600000)
            rsber_C = ((u32) dm.vit_c_ber_err_bits * 250 / ((u32) dm.vit_c_ber_rxd_rsps * 204/5));
         else
            rsber_C = ((u32) dm.vit_c_ber_err_bits * 625 / ((u32) dm.vit_c_ber_rxd_rsps * 102));
    }
    else
        rsber_C = 10000;

    //st->rssi = (int) dm.rssi * 100;
    bbm_com_tuner_get_rssi(NULL, DIV_MASTER, &rssi);
    bbm_com_read(NULL, DIV_MASTER, 0x410a, &tmcc_done);
    bbm_com_read(NULL, DIV_MASTER, 0x302a, &mode);
    bbm_com_read(NULL, DIV_MASTER, 0x3025, &data);
    bbm_com_read(NULL, DIV_MASTER, 0x30ac, &auto_switch);
    //bbm_com_read(NULL, DIV_MASTER, 0x106e, &agc);
    bbm_com_read(NULL, DIV_MASTER, 0x403d, &cn);

    st->rssi = abs(rssi) * 100;
    st->cn= cn * 25;
    //st->agc = agc;

    st->lock = 0;

    if (data & 0x08)
    {
        sync = 2;

        bbm_com_read(NULL, DIV_MASTER, 0x3026, &data);
        if (data & 0x02)
        {
            sync = 1;

            bbm_com_read(NULL, DIV_MASTER, 0x50c5, &data);
            if(data) {
                sync = 0;
                st->lock = 1;
            }
        }
    }
    else
        sync = 3;

    if(st->lock)
    {
        if(auto_switch & 0x01)
            st->fullseg_oneseg_flag = 1;
        else
            st->fullseg_oneseg_flag = 2;
    }
    else
        st->fullseg_oneseg_flag = 0;


    if (!(tmcc_done & 0x01)) {
        st->layerinfo_a = 0xffff;
        st->layerinfo_b = 0xffff;
        st->layerinfo_b = 0xffff;
    }
    else {
        bbm_com_bulk_read(NULL, DIV_MASTER, 0x4110, &tmcc_data[0], 26);

        //layerA
        st->layerinfo_a = ((tmcc_mod_scheme((tmcc_data[3] & 0x70) >> 4)) & 0x07) << 13;
        st->layerinfo_a |= ((tmcc_code_rate(((tmcc_data[4] & 0x03) << 1) + ((tmcc_data[3] & 0x80) >> 7))) & 0x07) << 10;
        st->layerinfo_a |= ((tmcc_inter_length((tmcc_data[4] & 0x1c) >> 2, mode)) & 0x3f) << 4 ;
        st->layerinfo_a |= (tmcc_num_segment(((tmcc_data[5] & 0x01) << 3) + ((tmcc_data[4] & 0xe0) >> 5))) & 0x0f;


        //layerB
        st->layerinfo_b = ((tmcc_mod_scheme((tmcc_data[5] & 0x0e) >> 1)) & 0x07) << 13;
        st->layerinfo_b |= ((tmcc_code_rate((tmcc_data[5] & 0x70) >> 4)) & 0x07) << 10;
        st->layerinfo_b |= ((tmcc_inter_length(((tmcc_data[6] & 0x03) << 1) + ((tmcc_data[5] & 0x80) >> 7), mode)) & 0x3f) << 4 ;
        st->layerinfo_b |= (tmcc_num_segment((tmcc_data[6] & 0x3c) >> 2)) & 0x0f;


        //layerC
        st->layerinfo_c = ((tmcc_mod_scheme(((tmcc_data[7] & 0x01) << 2) + ((tmcc_data[6] & 0xc0) >> 6))) & 0x07) << 13;
        st->layerinfo_c |= ((tmcc_code_rate((tmcc_data[7] & 0x0e) >> 1)) & 0x07) << 10;
        st->layerinfo_c |= ((tmcc_inter_length((tmcc_data[7] & 0x70) >> 4, mode)) & 0x3f) << 4 ;
        st->layerinfo_c |= (tmcc_num_segment(((tmcc_data[8] & 0x07) << 1) + ((tmcc_data[7] & 0x80) >> 7))) & 0x0f;
    }

    if((st->layerinfo_a & 0x0f) == 1) {
        if(brd_type == ISDBT_13SEG) {
            if (rsber_A ==0) {
                st->ber_a = (st->ber_a > DC_OFFSET) ? (st->ber_a - DC_OFFSET) : (0);
            }
            else    {
                st->ber_a = (st->ber_a > (DC_OFFSET - (((rsber_A * 2) > DC_OFFSET) ? (DC_OFFSET) : (rsber_A * 2)))) ?
                (st->ber_a - (DC_OFFSET - (((rsber_A * 2) > DC_OFFSET) ? (DC_OFFSET) : (rsber_A * 2)))) : (0);
            }
        }
        st->ber_1seg = st->ber = st->ber_a;
        st->per_1seg = st->per = st->per_a;
        st->err_tsp_1seg = st->ErrTSP = dm.vit_a_ber_err_rsps;
        st->total_tsp_1seg = st->TotalTSP = dm.vit_a_ber_rxd_rsps;

    }
    else if((st->layerinfo_b & 0x0f) == 1) {
        if(brd_type == ISDBT_13SEG) {
            if (rsber_B ==0) {
                st->ber_b = (st->ber_b > DC_OFFSET) ? (st->ber_b - DC_OFFSET) : (0);
            }
            else    {
                st->ber_b = (st->ber_b > (DC_OFFSET - (((rsber_B * 2) > DC_OFFSET) ? (DC_OFFSET) : (rsber_B * 2)))) ?
                (st->ber_b - (DC_OFFSET - (((rsber_B * 2) > DC_OFFSET) ? (DC_OFFSET) : (rsber_B * 2)))) : (0);
            }
        }
        st->ber_1seg = st->ber = st->ber_b;
        st->per_1seg = st->per = st->per_b;
        st->err_tsp_1seg = st->ErrTSP = dm.vit_b_ber_err_rsps;
        st->total_tsp_1seg = st->TotalTSP = dm.vit_b_ber_rxd_rsps;
    }
    else if((st->layerinfo_c & 0x0f) == 1) {
        if(brd_type == ISDBT_13SEG) {
            if (rsber_C ==0) {
                st->ber_c = (st->ber_c > DC_OFFSET) ? (st->ber_c - DC_OFFSET) : (0);
            }
            else    {
                st->ber_c = (st->ber_c > (DC_OFFSET - (((rsber_C * 2) > DC_OFFSET) ? (DC_OFFSET) : (rsber_C * 2)))) ?
                (st->ber_c - (DC_OFFSET - (((rsber_C * 2) > DC_OFFSET) ? (DC_OFFSET) : (rsber_C * 2)))) : (0);
            }
        }
        st->ber_1seg = st->ber = st->ber_c;
        st->per_1seg = st->per = st->per_c;
        st->err_tsp_1seg = st->ErrTSP = dm.vit_c_ber_err_rsps;
        st->total_tsp_1seg = st->TotalTSP = dm.vit_c_ber_rxd_rsps;
    }

    if((st->layerinfo_a & 0x0f) >= 12) {
        st->ber_fullseg = st->ber_a;
        st->per_fullseg = st->per_a;
        st->err_tsp_fullseg = dm.vit_a_ber_err_rsps;
        st->total_tsp_fullseg = dm.vit_a_ber_rxd_rsps;

    }
    else if((st->layerinfo_b & 0x0f) >= 12) {
        st->ber_fullseg =  st->ber_b;
        st->per_fullseg =  st->per_b;
        st->err_tsp_fullseg = dm.vit_b_ber_err_rsps;
        st->total_tsp_fullseg = dm.vit_b_ber_rxd_rsps;
    }
    else if((st->layerinfo_c & 0x0f) >= 12) {
        st->ber_fullseg = st->ber_c;
        st->per_fullseg = st->per_c;
        st->err_tsp_fullseg = dm.vit_c_ber_err_rsps;
        st->total_tsp_fullseg = dm.vit_c_ber_rxd_rsps;
    }

    transmission_parameter_switching = ((tmcc_data[2] & 0x80) >> 5)
                                        + ((tmcc_data[2] & 0x40) >> 3)
                                        + ((tmcc_data[3] & 0x02) >> 1)
                                        + ((tmcc_data[3] & 0x01) << 1);
    if (tmcc_done & 0x01) {
        st->tmccinfo = (tmcc_system_identification((tmcc_data[2] & 0x30) >> 4) << 6)
                    | (((transmission_parameter_switching == 0x0f)?0xf:transmission_parameter_switching + 1) << 2)
                    | (((tmcc_data[3] & 0x04) >> 2) << 1)
                    | ((tmcc_data[3] & 0x08) >> 3);
    }
    else
        st->tmccinfo = 0xff;

    st->receive_status = ((tmcc_data[3] & 0x04) >> 2) | sync;
    st->scan_status = 0;
    st->sysinfo = (((mode & 0x03) - 1) << 6) | (tmcc_GI_mode((mode & 0x70) >> 4) << 4);

    st->total_tsp_a = dm.vit_a_ber_rxd_rsps;
    st->total_tsp_b = dm.vit_b_ber_rxd_rsps;
    st->total_tsp_c = dm.vit_c_ber_rxd_rsps;

#ifdef FEATURE_COMPENSATE_MEASURE
    st->ber_1seg = tunerbb_drv_fc8300_measure_compval(0, st->ber_1seg);
    //st->ber_fullseg = tunerbb_drv_fc8300_measure_compval(1, st->ber_fullseg);
    st->cn = tunerbb_drv_fc8300_measure_compval(2, st->cn);
#endif

    st->antenna_level_oneseg = tunerbb_drv_fc8300_get_oneseg_antenna(st->antenna_level_oneseg, st->ber_1seg, st->cn, brd_type);
    st->antenna_level_fullseg = tunerbb_drv_fc8300_get_fullseg_antenna(st->antenna_level_fullseg, st->ber_fullseg, st->cn);
    //print_log(NULL, "[FC8300]ant_oneseg : %d,  ber_oneseg : %d, ant_fullseg : %d,  ber_fullseg : %d, switching : %d\n", st->antenna_level_oneseg, st->ber_1seg, st->antenna_level_fullseg, st->ber_fullseg, st->fullseg_oneseg_flag);
    print_log(NULL, "LOCK : %d, CN : %d, RSSI : %d, msnr : %d, over : %d\n", st->lock, st->cn, st->rssi, cn*25, st->agc);
    //print_log(NULL, "[FC8300]BER_A : %d, BER_B : %d, PER_A : %d, PER_B : %d\n", st->ber_a, st->ber_b, st->per_a, st->per_b);
    //print_log(NULL, "[FC8300]layA : 0x%x, layB : 0x%x, layC : 0x%x, TMCC : 0x%x , sysinfo : 0x%x \n", st->layerinfo_a, st->layerinfo_b, st->layerinfo_c, st->tmccinfo, st->sysinfo);
    return res;
}
