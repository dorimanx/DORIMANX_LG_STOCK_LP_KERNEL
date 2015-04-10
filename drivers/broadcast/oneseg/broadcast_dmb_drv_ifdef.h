/*****************************************************************************

    Copyright(c) 2008 LG Electronics Inc. All Rights Reserved

    File name : broadcat_dmb_drv_ifdef.h

    Description :

    Hoistory
    ----------------------------------------------------------------------
    Nov. 05, 2009:        inb612        create
    Aug. 24, 2012:        hyewon.eum    modified
*******************************************************************************/
#ifndef __LINUX_LGE_BORADCAST_H
#define __LINUX_LGE_BORADCAST_H

#include <linux/types.h>
#include <linux/ioctl.h>

#define LGE_BROADCAST_DMB_IOCTL_MAGIC 'I'

#define LGE_BROADCAST_DMB_IOCTL_ON \
    _IO(LGE_BROADCAST_DMB_IOCTL_MAGIC, 30)

#define LGE_BROADCAST_DMB_IOCTL_OFF \
    _IO(LGE_BROADCAST_DMB_IOCTL_MAGIC, 31)

#define LGE_BROADCAST_DMB_IOCTL_OPEN \
    _IOW(LGE_BROADCAST_DMB_IOCTL_MAGIC, 32, struct broadcast_dmb_init_info)

#define LGE_BROADCAST_DMB_IOCTL_CLOSE \
    _IO(LGE_BROADCAST_DMB_IOCTL_MAGIC, 33)

#define LGE_BROADCAST_DMB_IOCTL_SET_CH \
    _IOW(LGE_BROADCAST_DMB_IOCTL_MAGIC, 35, struct broadcast_dmb_set_ch_info)

#define LGE_BROADCAST_DMB_IOCTL_RESYNC \
    _IOW(LGE_BROADCAST_DMB_IOCTL_MAGIC, 36, int)

#define LGE_BROADCAST_DMB_IOCTL_DETECT_SYNC \
    _IOR(LGE_BROADCAST_DMB_IOCTL_MAGIC, 37, struct broadcast_dmb_sync_info)

#define LGE_BROADCAST_DMB_IOCTL_GET_SIG_INFO \
    _IOWR(LGE_BROADCAST_DMB_IOCTL_MAGIC, 38, struct broadcast_dmb_control_info)

#define LGE_BROADCAST_DMB_IOCTL_GET_CH_INFO \
    _IOR(LGE_BROADCAST_DMB_IOCTL_MAGIC, 39, struct broadcast_dmb_ch_info)

#define LGE_BROADCAST_DMB_IOCTL_RESET_CH \
    _IO(LGE_BROADCAST_DMB_IOCTL_MAGIC, 40)

#define LGE_BROADCAST_DMB_IOCTL_USER_STOP \
    _IOW(LGE_BROADCAST_DMB_IOCTL_MAGIC, 41, int)

#define LGE_BROADCAST_DMB_IOCTL_GET_DMB_DATA \
    _IOW(LGE_BROADCAST_DMB_IOCTL_MAGIC, 42, struct broadcast_dmb_data_info)

#define LGE_BROADCAST_DMB_IOCTL_SELECT_ANTENNA \
    _IOW(LGE_BROADCAST_DMB_IOCTL_MAGIC, 43, int)

#define LGE_BROADCAST_DMB_IOCTL_START_STREAM \
    _IOW(LGE_BROADCAST_DMB_IOCTL_MAGIC, 44, int)

#define LGE_BROADCAST_DMB_IOCTL_STOP_STREAM \
    _IO(LGE_BROADCAST_DMB_IOCTL_MAGIC, 45)

#define LGE_BROADCAST_DMB_GET_MODE \
    _IO(LGE_BROADCAST_DMB_IOCTL_MAGIC, 46)

struct broadcast_dmb_init_info
{
    unsigned int rf_band;
    unsigned int segment;
};

struct broadcast_dmb_set_ch_info
{
    unsigned int    mode;
    unsigned int    rf_band;
    unsigned int    channel;
    unsigned int    subchannel;
    unsigned int    segment;
};

struct broadcast_dmb_sync_info
{
    unsigned int sync_status;
    unsigned int sync_ext_status;
};

typedef struct
{
    int lock;    /*baseband lock state                     (1: Lock, 0:Unlock)*/
    int cn;        /*Signal Level (C/N)                     (0 ~ 2800) - 100 scale */
    int ber;     /*Bit Error rate                     (0 ~ 100000)*/
    int per;      /*Packet Error rate                      (0 ~ 100000)*/
    int agc;      /*Auto Gain control                      (0 ~ 255)*/
    int rssi;      /*Received Signal Strength Indication      (0 ~ -99)*/
    int ErrTSP;
    int TotalTSP;
    int antenna_level;
    int Num;
    int Exp;
    int mode;
}oneseg_sig_info;

typedef struct
{
    int cn;
    int ber_a;
    int per_a;
    int layerinfo_a;
    int tmccinfo;
    int receive_status;
    int rssi;
    int scan_status;
    int sysinfo;
    int total_tsp_a;

    int ber_b;
    int per_b;
    int layerinfo_b;
    int total_tsp_b;

    int ber_c;
    int per_c;
    int layerinfo_c;
    int total_tsp_c;

    int antenna_level_fullseg;
    int antenna_level_oneseg;
    int agc;
    int ber_1seg;
    int per_1seg;
    int total_tsp_1seg;
    int err_tsp_1seg;
    int ber_fullseg;
    int per_fullseg;
    int total_tsp_fullseg;
    int err_tsp_fullseg;
}mmb_sig_info;

struct broadcast_dmb_sig_info
{
    union
    {
        oneseg_sig_info oneseg_info;
        mmb_sig_info mmb_info;
    }info;
};

struct broadcast_dmb_cmd_info
{
    unsigned int service_type;
    unsigned int cmd;
    unsigned int layer;
    unsigned int mode;
    unsigned int fullseg_oneseg_flag;
    unsigned int over;
};

struct broadcast_dmb_control_info
{
    struct broadcast_dmb_cmd_info cmd_info;
    struct broadcast_dmb_sig_info sig_info;
};

struct broadcast_dmb_ch_info
{
    unsigned int    ch_buf_addr;
    unsigned int    buf_len;
};

struct broadcast_dmb_data_info
{
    unsigned int    data_buf_addr;
    unsigned int    data_buf_size;
    unsigned int    copied_size;
    unsigned int    packet_cnt;
};

enum
{
    DMB_CMD_NONE = 0,
    DMB_CMD_GET_ALL,
    DMB_CMD_GET_BER,
    DMB_CMD_GET_PER,
    DMB_CMD_GET_CN,
    DMB_CMD_GET_CN_PER_LAYER,
    DMB_CMD_GET_LAYERINFO,
    DMB_CMD_GET_RECEIVE_STATUS,
    DMB_CMD_GET_RSSI,
    DMB_CMD_GET_SCAN_STATUS,
    DMB_CMD_GET_SYSINFO,
    DMB_CMD_GET_TMCCINFO,
};

enum
{
    LGE_BROADCAST_DMB_ANT_TYPE_INTENNA,
    LGE_BROADCAST_DMB_ANT_TYPE_EARANT,
    LGE_BROADCAST_DMB_ANT_TYPE_EXTERNAL
};

enum
{
    DMB_OP_CMD_NORMAL = 0x00,
    DMB_OP_CMD_SCAN,
    DMB_OP_CMD_CHANGE_MODE
};

enum
{
    DMB_RF_BAND_UHF = 0x00,
    DMB_RF_BAND_VHF = 0x01
};

enum
{
    DMB_SEG_13SEG = 0x00,
    DMB_SEG_1SEG = 0x01,
};

struct broadcast_drv_if {
    int (*broadcast_drv_if_power_on)(void);
    int (*broadcast_drv_if_power_off)(void);
    int (*broadcast_drv_if_open)(void);
    int (*broadcast_drv_if_close)(void);
    int (*broadcast_drv_if_set_channel)(struct broadcast_dmb_set_ch_info *udata);
    int (*broadcast_drv_if_resync)(void);
    int (*broadcast_drv_if_detect_sync)(struct broadcast_dmb_sync_info *udata);
    int (*broadcast_drv_if_get_sig_info)(struct broadcast_dmb_control_info *bb_info);
    int (*broadcast_drv_if_get_ch_info)(struct broadcast_dmb_ch_info *ch_info);
    int (*broadcast_drv_if_get_dmb_data)(struct broadcast_dmb_data_info *pdmb_data);
    int (*broadcast_drv_if_reset_ch)(void);
    int (*broadcast_drv_if_user_stop)(int mode);
    int (*broadcast_drv_if_select_antenna)(unsigned int sel);
    int (*broadcast_drv_if_read_control)(char *buf, unsigned int size);
    int (*broadcast_drv_if_get_mode)(unsigned short *mode);
};

typedef struct broadcast_drv_if Device_drv ;

extern int broadcast_dmb_drv_check_module_init(void);
extern int    broadcast_dmb_drv_start(Device_drv*);
extern int    broadcast_get_stop_mode(void);

#endif
