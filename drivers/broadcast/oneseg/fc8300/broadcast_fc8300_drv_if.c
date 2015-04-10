#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>

#include <linux/delay.h>
#include <linux/slab.h>

#include "bbm.h"
#include "fci_oal.h"
#include "fci_tun.h"
#include "fc8300_regs.h"
#include "fc8300_isr.h"
#include "fci_hal.h"
#include "fci_types.h"
#include "fc8300_drv_api.h"

#include "broadcast_fc8300.h"
#include "broadcast_dmb_typedef.h"
#include "broadcast_dmb_drv_ifdef.h"

/*#define _DISPLAY_MONITOR_DBG_LOG_*/
/*#define _USE_ONSEG_SIGINFO_MODIFIED_MODE_*/
#define _USE_MONITORING_TIME_GAP_

typedef enum {
    TMM_13SEG = 0,
    TMM_1SEG,
    UHF_1SEG,
    UHF_13SEG,
} EnumIsdbType;

typedef enum {
    ENUM_GET_ALL = 1,
    ENUM_GET_BER,
    ENUM_GET_PER,
    ENUM_GET_CN,
    ENUM_GET_CN_PER_LAYER,
    ENUM_GET_LAYER_INFO,
    ENUM_GET_RECEIVE_STATUS,
    ENUM_GET_RSSI,
    ENUM_GET_SCAN_STATUS,
    ENUM_GET_SYS_INFO,
    ENUM_GET_TMCC_INFO,
    ENUM_GET_ONESEG_SIG_INFO
} EnumSigInfo;

#ifdef CONFIG_LGE_BROADCAST_BRAZIL_FREQ
static unsigned int frequencyTable[56] = {
    473143,479143,485143,491143,497143,
    503143,509143,515143,521143,527143,
    533143,539143,545143,551143,557143,
    563143,569143,575143,581143,587143,
    593143,599143,605143,611143,617143,
    623143,629143,635143,641143,647143,
    653143,659143,665143,671143,677143,
    683143,689143,695143,701143,707143,
    713143,719143,725143,731143,737143,
    743143,749143,755143,761143,767143,
    773143,779143,785143,791143,797143,
    803143,
};
#else
static unsigned int frequencyTable[50] = {
	473143,479143,485143,491143,497143,
	503143,509143,515143,521143,527143,
	533143,539143,545143,551143,557143,
	563143,569143,575143,581143,587143,
	593143,599143,605143,611143,617143,
	623143,629143,635143,641143,647143,
	653143,659143,665143,671143,677143,
	683143,689143,695143,701143,707143,
	713143,719143,725143,731143,737143,
	743143,749143,755143,761143,767143
};
#endif

static int currentBroadCast = TMM_13SEG;
static int currentSelectedChannel = -1;

s32 OnAir = 0;
s32 broad_type;

#ifndef BBM_I2C_TSIF
extern void broadcast_fci_ringbuffer_flush();
#endif

/* Body of Internel function */
int    broadcast_fc8300_get_stop_mode(void)
{
    int rc;
    rc = OK;
    print_log(NULL, "[1seg]broadcast_get_stop_mode\n");
    return rc;
}

int    broadcast_fc8300_drv_if_power_on(void)
{
    int rc = ERROR;

    print_log(NULL, "[1seg]broadcast_drv_if_power_on\n");
    if (!fc8300_is_power_on())
        rc = fc8300_power_on();
    return rc;
}

int    broadcast_fc8300_drv_if_power_off(void)
{
    int rc = ERROR;
    print_log(NULL, "[1seg]broadcast_drv_if_power_off\n");
    if (fc8300_is_power_on()) {
        rc = fc8300_power_off();
    } else {
        print_log(NULL, "[1seg] warning-already power off\n");
    }
    return rc;
}

int    broadcast_fc8300_drv_if_open(void)
{
    int ret = 0;

    broad_type = ISDBT_13SEG;
    ret = tunerbb_drv_fc8300_init(broad_type);

    if(ret)
        return ERROR;
    else {
        //fci_irq_enable();
        OnAir = 1;
        return OK;
    }
}

int    broadcast_fc8300_drv_if_close(void)
{
    int ret = 0;

    ret = tunerbb_drv_fc8300_stop();
#ifndef BBM_I2C_TSIF
    fci_irq_disable();
#endif
    OnAir = 0;

    if(ret)
        return ERROR;
    else
        return OK;
}

unsigned long long fcTimer;
void setTimer(void)
{
    struct timeval tv;

    do_gettimeofday(&tv);
    fcTimer = (long long) tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

unsigned long long TimeCount_ms(void)
{
    unsigned long long tickcount = 0;
    struct timeval tv;

    do_gettimeofday(&tv);
    tickcount = (long long) tv.tv_sec * 1000 + tv.tv_usec / 1000;
    return tickcount;
}

int    broadcast_fc8300_drv_if_set_channel(struct broadcast_dmb_set_ch_info *udata)
{
    signed long frequency = 214714; /*tmm*/
    int ret;
#ifndef BBM_I2C_TSIF
    fci_irq_disable();
#endif
    setTimer();

    if(OnAir == 0 || udata == NULL) {
        print_log(NULL, "[1seg] broadcast_drv_if_set_channel error [!OnAir]\n");
        return ERROR;
    }

    /* uhf 1segment */
    currentSelectedChannel = udata->channel;

    if(udata->segment == 13) {
        currentBroadCast = UHF_13SEG;
    } else {
        currentBroadCast = UHF_1SEG;
    }
    #ifdef CONFIG_LGE_BROADCAST_BRAZIL_FREQ
    if(udata->channel<14 || udata->channel>69) {
        print_log(NULL, "[1seg] channel information error\n");
        return ERROR;
    }
    frequency = frequencyTable[udata->channel-14];
    #else
    if(udata->channel<13 || udata->channel>62) {
        print_log(NULL, "[1seg] channel information error\n");
        return ERROR;
    }
    frequency = frequencyTable[udata->channel-13];
    #endif

    /* Scan mode(udata->mode==1) & need lock check */

#ifndef BBM_I2C_TSIF
    fci_irq_enable();
#endif
    ret = tunerbb_drv_fc8300_set_channel(frequency, udata->mode, udata->subchannel);

#ifndef BBM_I2C_TSIF
    broadcast_fci_ringbuffer_flush();
#endif

    if(ret)
        return ERROR;

    print_log(NULL, "[fc8300] channel channel : %d, %d, %d, scan OK\n", udata->channel, udata->mode, udata->subchannel);

    return OK;
}

int    broadcast_fc8300_drv_if_resync(void)
{
    int rc;
    /*
    FC8300 use auto-resync
    */
    rc = OK;
    return rc;
}

int    broadcast_fc8300_drv_if_detect_sync(struct broadcast_dmb_sync_info *udata)
{
    int sync;
    if(OnAir == 0) {
        print_log(NULL, "[1seg] broadcast_drv_if_detect_sync error [!OnAir]\n");
        return ERROR;
    }

    sync = tunerbb_drv_fc8300_Get_SyncStatus();

    udata->sync_status = sync;
    udata->sync_ext_status = sync;

    return OK;
}

static void broadcast_fc8300_drv_if_get_oneseg_sig_info(struct fc8300Status_t *pst, struct broadcast_dmb_control_info *pInfo, s32 brd_type)
{
    int ber;
    int per;
    int cn;

    pInfo->sig_info.info.oneseg_info.lock = pst->lock;
    cn = pInfo->sig_info.info.oneseg_info.cn = pst->cn;
    ber = pInfo->sig_info.info.oneseg_info.ber = pst->ber;
    per = pInfo->sig_info.info.oneseg_info.per = pst->per;

    pInfo->sig_info.info.oneseg_info.agc = pst->agc;
    pInfo->sig_info.info.oneseg_info.rssi = pst->rssi;
    pInfo->sig_info.info.oneseg_info.ErrTSP = pst->ErrTSP;
    pInfo->sig_info.info.oneseg_info.TotalTSP = pst->TotalTSP;

    pInfo->cmd_info.over= pst->agc;

    pInfo->sig_info.info.oneseg_info.antenna_level = pst->antenna_level_oneseg;
    pInfo->sig_info.info.oneseg_info.Num = 0;
    pInfo->sig_info.info.oneseg_info.Exp = 0;
    pInfo->sig_info.info.oneseg_info.mode = 0;

#ifdef _DISPLAY_MONITOR_DBG_LOG_
    print_log(NULL, "[1seg][monitor] lock[%d] Antenna[%d] cn[%d] ber[%d] per[%d] rssi[%d] errTSP[%d/%d]\n",
            pInfo->sig_info.info.oneseg_info.lock,
            pInfo->sig_info.info.oneseg_info.antenna_level,
            pInfo->sig_info.info.oneseg_info.cn,
            pInfo->sig_info.info.oneseg_info.ber,
            pInfo->sig_info.info.oneseg_info.per,
            pInfo->sig_info.info.oneseg_info.rssi,
            pInfo->sig_info.info.oneseg_info.ErrTSP,
            pInfo->sig_info.info.oneseg_info.TotalTSP);

#endif
}


struct fc8300Status_t st;
unsigned int irq_cnt;

int    broadcast_fc8300_drv_if_get_sig_info(struct broadcast_dmb_control_info *pInfo)
{
    int layer;
    static unsigned int before_irq_flag=0;

    //print_log(NULL, "[FC8300] broadcast_drv_if_get_sig_info  %d, %d\n", OnAir, pInfo->cmd_info.cmd);
    if(OnAir == 0 || pInfo==NULL) {
        return ERROR;
    }

    layer = pInfo->cmd_info.layer;

    if((TimeCount_ms() - fcTimer) > 500) {
        if(before_irq_flag == irq_cnt)
        {
            tunerbb_drv_fc8300_Get_SignalInfo(&st, broad_type);
            //print_log(NULL, "[FC8300] direct broadcast_drv_if_get_sig_info\n");
        }
        else
        {
            before_irq_flag = irq_cnt;
        }
        setTimer();
    }

    switch(pInfo->cmd_info.cmd)
    {
    case ENUM_GET_ALL:
        pInfo->sig_info.info.mmb_info.cn =
            st.cn;

        if(layer==0) {
            pInfo->sig_info.info.mmb_info.ber_a =
                st.ber_a;
            pInfo->sig_info.info.mmb_info.per_a =
                st.per_a;
            pInfo->sig_info.info.mmb_info.total_tsp_a =
                st.total_tsp_a;
            pInfo->sig_info.info.mmb_info.layerinfo_a =
                st.layerinfo_a;
        } else if(layer==1) {
            pInfo->sig_info.info.mmb_info.ber_b =
                st.ber_b;
            pInfo->sig_info.info.mmb_info.per_b =
                st.per_b;
            pInfo->sig_info.info.mmb_info.total_tsp_b =
                st.total_tsp_b;
            pInfo->sig_info.info.mmb_info.layerinfo_b =
                st.layerinfo_b;
        } else if(layer==2) {
            pInfo->sig_info.info.mmb_info.ber_c =
                st.ber_c;
            pInfo->sig_info.info.mmb_info.per_c =
                st.per_c;
            pInfo->sig_info.info.mmb_info.total_tsp_c =
                st.total_tsp_c;
            pInfo->sig_info.info.mmb_info.layerinfo_c =
                st.layerinfo_c;
        } else {
            pInfo->sig_info.info.mmb_info.ber_a =
                st.ber_a;
            pInfo->sig_info.info.mmb_info.per_a =
                st.per_a;
            pInfo->sig_info.info.mmb_info.ber_b =
                st.ber_b;
            pInfo->sig_info.info.mmb_info.per_b =
                st.per_b;
            pInfo->sig_info.info.mmb_info.ber_c =
                st.ber_c;
            pInfo->sig_info.info.mmb_info.per_c =
                st.per_c;
            pInfo->sig_info.info.mmb_info.total_tsp_a =
                st.total_tsp_a;
            pInfo->sig_info.info.mmb_info.total_tsp_b =
                st.total_tsp_b;
            pInfo->sig_info.info.mmb_info.total_tsp_c =
                st.total_tsp_c;
            pInfo->sig_info.info.mmb_info.layerinfo_a =
                st.layerinfo_a;
            pInfo->sig_info.info.mmb_info.layerinfo_b =
                st.layerinfo_b;
            pInfo->sig_info.info.mmb_info.layerinfo_c =
                st.layerinfo_c;
        }

        pInfo->sig_info.info.mmb_info.tmccinfo =
            st.tmccinfo;

        pInfo->sig_info.info.mmb_info.receive_status =
            st.receive_status;

        pInfo->sig_info.info.mmb_info.rssi =
            st.rssi;

        pInfo->sig_info.info.mmb_info.scan_status =
            st.scan_status;

        pInfo->sig_info.info.mmb_info.sysinfo =
            st.sysinfo;

        pInfo->cmd_info.over=
            st.agc;

        pInfo->sig_info.info.mmb_info.antenna_level_fullseg =
            st.antenna_level_fullseg;

        pInfo->sig_info.info.mmb_info.antenna_level_oneseg =
            st.antenna_level_oneseg;

    break;

    case ENUM_GET_BER:
        if(layer==0) {
            pInfo->sig_info.info.mmb_info.ber_a =
                    st.ber_a;
            pInfo->sig_info.info.mmb_info.total_tsp_a =
                    st.total_tsp_a;
        } else if(layer==1) {
            pInfo->sig_info.info.mmb_info.ber_b =
                    st.ber_b;
            pInfo->sig_info.info.mmb_info.total_tsp_b =
                    st.total_tsp_b;
        } else if(layer==2) {
            pInfo->sig_info.info.mmb_info.ber_c =
                    st.ber_c;
            pInfo->sig_info.info.mmb_info.total_tsp_c =
                    st.total_tsp_c;
        } else {
            pInfo->sig_info.info.mmb_info.ber_a =
                    st.ber_a;
            pInfo->sig_info.info.mmb_info.total_tsp_a =
                    st.total_tsp_a;
            pInfo->sig_info.info.mmb_info.ber_b =
                    st.ber_b;
            pInfo->sig_info.info.mmb_info.total_tsp_b =
                    st.total_tsp_b;
            pInfo->sig_info.info.mmb_info.ber_c =
                    st.ber_c;
            pInfo->sig_info.info.mmb_info.total_tsp_c =
                    st.total_tsp_c;
        }

    break;

    case ENUM_GET_PER:
        if(layer==0) {
            pInfo->sig_info.info.mmb_info.per_a =
                    st.per_a;
            pInfo->sig_info.info.mmb_info.total_tsp_a =
                    st.total_tsp_a;
        } else if(layer==1) {
            pInfo->sig_info.info.mmb_info.per_b =
                    st.per_b;
            pInfo->sig_info.info.mmb_info.total_tsp_b =
                    st.total_tsp_b;
        } else if(layer==2) {
            pInfo->sig_info.info.mmb_info.per_c =
                    st.per_c;
            pInfo->sig_info.info.mmb_info.total_tsp_c =
                    st.total_tsp_c;
        } else {
            pInfo->sig_info.info.mmb_info.per_a =
                    st.per_a;
            pInfo->sig_info.info.mmb_info.total_tsp_a =
                    st.total_tsp_a;
            pInfo->sig_info.info.mmb_info.per_b =
                    st.per_b;
            pInfo->sig_info.info.mmb_info.total_tsp_b =
                    st.total_tsp_b;
            pInfo->sig_info.info.mmb_info.per_c =
                    st.per_c;
            pInfo->sig_info.info.mmb_info.total_tsp_c =
                    st.total_tsp_c;
        }
    break;

    case ENUM_GET_CN:
        pInfo->sig_info.info.mmb_info.cn =
            st.cn;
        print_log(NULL, "[mmbi][monitor] cn[%d]\n",
                pInfo->sig_info.info.mmb_info.cn);
    break;

    case ENUM_GET_CN_PER_LAYER:
        pInfo->sig_info.info.mmb_info.cn =
            st.cn;
        print_log(NULL, "[mmbi][monitor] cn[%d]\n",
                pInfo->sig_info.info.mmb_info.cn);
    break;

    case ENUM_GET_LAYER_INFO:
        if(layer==0) {
            pInfo->sig_info.info.mmb_info.layerinfo_a =
                st.layerinfo_a;
        } else if(layer==1) {
            pInfo->sig_info.info.mmb_info.layerinfo_b =
                st.layerinfo_b;
        } else if(layer==2) {
            pInfo->sig_info.info.mmb_info.layerinfo_c =
                st.layerinfo_c;
        } else {
            pInfo->sig_info.info.mmb_info.layerinfo_a =
                st.layerinfo_a;
            pInfo->sig_info.info.mmb_info.layerinfo_b =
                st.layerinfo_b;
            pInfo->sig_info.info.mmb_info.layerinfo_c =
                st.layerinfo_c;
        }
    break;

    case ENUM_GET_RECEIVE_STATUS:
        pInfo->sig_info.info.mmb_info.receive_status =
                st.receive_status;
        print_log(NULL, "[mmbi][monitor]  rcv status[%d]\n",
                pInfo->sig_info.info.mmb_info.receive_status);
        /* for debugging log */
    break;

    case ENUM_GET_RSSI:
        pInfo->sig_info.info.mmb_info.rssi =
            st.rssi;
        print_log(NULL, "[mmbi][monitor]  rssi[%d]\n",
                pInfo->sig_info.info.mmb_info.rssi);
    break;

    case ENUM_GET_SCAN_STATUS:
        pInfo->sig_info.info.mmb_info.scan_status =
            st.scan_status;
        print_log(NULL, "[mmbi][monitor]  scan status[%d]\n",
                pInfo->sig_info.info.mmb_info.scan_status);
        /* for debugging log */
    break;

    case ENUM_GET_SYS_INFO:
        pInfo->sig_info.info.mmb_info.sysinfo =
            st.sysinfo;
        print_log(NULL, "[mmbi][monitor]  sys info[%d]\n",
                pInfo->sig_info.info.mmb_info.sysinfo);
        /* for debugging log */
    break;

    case ENUM_GET_TMCC_INFO:
        pInfo->sig_info.info.mmb_info.tmccinfo =
            st.tmccinfo;
        print_log(NULL, "[mmbi][monitor]  tmcc info[%d]\n",
                pInfo->sig_info.info.mmb_info.tmccinfo);
        /* for debugging log */
    break;

    case ENUM_GET_ONESEG_SIG_INFO:
        broadcast_fc8300_drv_if_get_oneseg_sig_info(&st, pInfo, broad_type);
    break;

    default:
        print_log(NULL, "[mmbi][monitor] sig_info unknown command[%d]\n",
                pInfo->cmd_info.cmd);
        return -1;
    break;
    }

    return OK;
}

int    broadcast_fc8300_drv_if_get_ch_info(struct broadcast_dmb_ch_info *ch_info)
{
    int rc = ERROR;

    if(OnAir == 0) {
        print_log(NULL, "[1seg][no semaphore] broadcast_drv_if_get_ch_info error [!OnAir]\n");
        return ERROR;
    }

    /*
    Unused function
    */
    rc = OK;
    return rc;
}

int    broadcast_fc8300_drv_if_get_dmb_data(struct broadcast_dmb_data_info *pdmb_data)
{
#ifndef BBM_I2C_TSIF
    unsigned int total_len;

    if(OnAir == 0 || pdmb_data==NULL) {
        return ERROR;
    }

    if(pdmb_data->data_buf_addr == 0) {
        print_log(NULL, "[1seg] broadcast_drv_if_get_dmb_data[ERR] data_buf is null\n");
        return ERROR;
    }

    if(pdmb_data->data_buf_size < 188) {
        print_log(NULL, "[1seg] broadcast_drv_if_get_dmb_data[ERR] buffsize < 188\n");
        return ERROR;
    }

    total_len = fc8300_get_ts((void*)((unsigned long)pdmb_data->data_buf_addr), pdmb_data->data_buf_size);

    pdmb_data->copied_size = total_len;
    //pdmb_data->packet_cnt = total_len / 188;
    pdmb_data->packet_cnt = TS0_BUF_LENGTH;

    return OK;
#else
    return ERROR;
#endif
}

int    broadcast_fc8300_drv_if_reset_ch(void)
{
    //int ret = 0;
    int rc = OK;

    if(OnAir == 0) {
        print_log(NULL, "[1seg] broadcast_drv_if_reset_ch error [!OnAir]\n");
        return ERROR;
    }

    print_log(NULL, "[1seg]broadcast_drv_if_reset_ch\n");

    return rc;
}

int    broadcast_fc8300_drv_if_user_stop(int mode)
{
    int rc;
    rc = OK;
    if(OnAir == 0) {
        print_log(NULL, "[1seg][no semaphore] broadcast_drv_if_user_stop error [!OnAir]\n");
        return ERROR;
    }

    print_log(NULL, "[1seg][no semaphore] broadcast_drv_if_user_stop\n");

    return rc;
}

int    broadcast_fc8300_drv_if_select_antenna(unsigned int sel)
{
    int rc;
    rc = OK;
    if(OnAir == 0) {
        print_log(NULL, "[1seg][no semaphore] broadcast_drv_if_select_antenna error [!OnAir]\n");
        return ERROR;
    }
    return rc;
}

int    broadcast_fc8300_drv_if_isr(void)
{
    int rc;
    rc = OK;
    if(OnAir == 0) {
        print_log(NULL, "[1seg] broadcast_drv_if_isr error [!OnAir]\n");
        return ERROR;
    }
    return rc;
}

int    broadcast_fc8300_drv_if_read_control(char *buf, unsigned int size)
{
    return 0;
}

int    broadcast_fc8300_drv_if_get_mode (unsigned short *mode)
{
    int rc = ERROR;

    if(mode == NULL) {
        return ERROR;
    }

    if(OnAir == 0 || currentSelectedChannel== -1) {
        mode[0] = 0xFFFF;
    } else {
        unsigned short channel_num;
        unsigned short band = 0;
        unsigned short rcvSegment = 0;

        channel_num = currentSelectedChannel;

        if(currentBroadCast == UHF_1SEG) {
            band = 0;
            rcvSegment = 1;
        } else if (currentBroadCast == UHF_13SEG) {
            band = 0;
            rcvSegment = 0;
        } else if(currentBroadCast == TMM_1SEG) {
            band = 1;
            rcvSegment = 1;
        } else {
            band = 1;
            rcvSegment = 0;
        }

        mode[0] = ((channel_num&0xFF)<<8) |
                    ((band&0x0F)<<4) |
                    (rcvSegment&0x0F);
    }

    rc = OK;
    return rc;
}
/*                                                                          */
/*--------------------------------------------------------------------------*/


/* optional part when we include driver code to build-on
it's just used when we make device driver to module(.ko)
so it doesn't work in build-on */
MODULE_DESCRIPTION("FCI ISDB-Tmm device driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("FCI");
