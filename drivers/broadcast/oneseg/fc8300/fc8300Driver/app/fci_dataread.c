#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>
#include "fci_types.h"

int index=0;
static pthread_t p_thr_data;
static pthread_t p_thr_sig;
static pthread_t ptcheck_thr_sig;
#define MSC_BUF_SIZE 128*1024
#define DUMP_PATH "/data/"

typedef struct _data_dump_param{
	int num;
	int dev;
} data_dump_param;

#define FEATURE_TS_CHECK
//#define FEATURE_OVERRUN_CHECK

#ifdef FEATURE_TS_CHECK
#define MAX_DEMUX           2

/*
 * Sync Byte 0xb8
 */
#define SYNC_BYTE_INVERSION

struct pid_info {
	unsigned long count;
	unsigned long discontinuity;
	unsigned long continuity;
};

struct demux_info {
	struct pid_info  pids[8192];

	unsigned long    ts_packet_c;
	unsigned long    malformed_packet_c;
	unsigned long    tot_scraped_sz;
	unsigned long    packet_no;
	unsigned long    sync_err;
	unsigned long 	   sync_err_set;
};

static int is_sync(unsigned char* p) {
	int syncword = p[0];
#ifdef SYNC_BYTE_INVERSION
	if(0x47 == syncword || 0xb8 == syncword)
		return 1;
#else
	if(0x47 == syncword)
		return 1;
#endif
	return 0;
}
static struct demux_info demux[MAX_DEMUX];

int print_pkt_log()
{
	unsigned long i=0;

	print_log("\nPKT_TOT : %d, SYNC_ERR : %d, SYNC_ERR_BIT : %d, ERR_PKT : %d \n", demux[0].ts_packet_c, demux[0].sync_err, demux[0].sync_err_set, demux[0].malformed_packet_c);

	for(i=0;i<8192;i++)
	{
		if(demux[0].pids[i].count>0)
			print_log("PID : %d, TOT_PKT : %d, DISCONTINUITY : %d \n", i, demux[0].pids[i].count, demux[0].pids[i].discontinuity);
	}

}

int put_ts_packet(int no, unsigned char* packet, int sz) {
	unsigned char* p;
	int transport_error_indicator, pid, payload_unit_start_indicator, continuity_counter, last_continuity_counter;
	int i; // e = 0
	if((sz % 188)) {
		//e = 1;
		print_log("L : %d", sz);
		//print_log("Video %d Invalid size: %d\n", no, sz);
	} else {
		for(i = 0; i < sz; i += 188) {
			p = packet + i;

			pid = ((p[1] & 0x1f) << 8) + p[2];

			demux[no].ts_packet_c++;
			if(!is_sync(packet + i)) {
				//e = 1;
				print_log("S     ");
				demux[no].sync_err++;
				if(0x80==(p[1] & 0x80))
					demux[no].sync_err_set++;
				print_log("0x%x, 0x%x, 0x%x, 0x%x \n", *p, *(p+1),  *(p+2), *(p+3));
				//print_log("   Video %d Invalid sync: 0x%02x, Offset: %d, Frame No: %d\n", no, *(packet + i), i, i / 188);
				//break;
				continue;
			}

			// Error Indicator가 설정되면 Packet을 버림
			transport_error_indicator = (p[1] & 0x80) >> 7;
			if(1 == transport_error_indicator) {
				demux[no].malformed_packet_c++;
				//e++;
				//print_log("I      ");
				//print_log("   Video %d PID 0x%04x: err_ind, Offset: %d, Frame No: %d\n", no, pid, i, i / 188);
				continue;
			}

			payload_unit_start_indicator = (p[1] & 0x40) >> 6;

			demux[no].pids[pid].count++;

			// Continuity Counter Check
			continuity_counter = p[3] & 0x0f;

			if(demux[no].pids[pid].continuity == -1) {
				demux[no].pids[pid].continuity = continuity_counter;
			} else {
				last_continuity_counter = demux[no].pids[pid].continuity;

				demux[no].pids[pid].continuity = continuity_counter;

				if(((last_continuity_counter + 1) & 0x0f) != continuity_counter) {
					demux[no].pids[pid].discontinuity++;
					//e++;
					//print_log("D      ");
					//print_log("   Video %d PID 0x%04x: last counter %x ,current %x, Offset: %d,Frame No: %d, start_ind: %d\n", no, pid, last_continuity_counter, continuity_counter, i, i / 188, payload_unit_start_indicator);
				}
			}
		}
	}

	//if(e) {
	//	print_log("Video %d Received Size: %d, Frame Count: %d, Error Count: %d\n", no, sz, sz / 188, e);
	//}
	return 0;
}


void create_tspacket_anal() {
	int n, i;

	for(n = 0; n < MAX_DEMUX; n++) {
		memset((void*)&demux[n], 0, sizeof(demux[n]));

		for(i = 0; i < 8192; i++) {
			demux[n].pids[i].continuity = -1;
		}
	}

}
#endif

void data_dump_thread(void *param)
{
	data_dump_param *data = (data_dump_param*)param;
	int hDevice = data->dev;
	int num = data->num;
	u8 buf[MSC_BUF_SIZE];
	int i;
	int ret;
	int check_cnt_size=0;
	int monitor_size=0;
	int dump_size=0;
	u32 berA, perA, berB, perB, berC, perC, cnr, ui32Quality, ui32Antlvl;
	s32 i32RSSI;
	u8 slock=0;

	//while(1)
	{
		FILE *f;
		char *f_name[128];
		sprintf((void*)f_name,DUMP_PATH"data_dump_%04d.dat", index++);
		dump_size=0;
		f = fopen((void*)f_name, "wb");

#ifdef FEATURE_TS_CHECK
		create_tspacket_anal();
#endif

		print_log("Start data dump %s , pkt : %d\n", f_name, data->num);
		mtv_ts_start(hDevice);
		for(i=0;i<num;i++)
		{
			int k;
			int size;

			size = mtv_data_read(hDevice, buf, MSC_BUF_SIZE);

#ifdef FEATURE_TS_CHECK
			if(!(size%188)) {
				put_ts_packet(0, &buf[0], size);
				dump_size+=size;
				monitor_size+=size;
				check_cnt_size+=size;
#ifdef FEATURE_OVERRUN_CHECK
				{
					u8 over;
					BBM_READ(hDevice, 0x8001, &over);
					if(over)
					{
						BBM_WRITE(hDevice, 0x8001, over);
						print_log("TS OVERRUN : %d\n", over);
					}
				}
#endif
				if(check_cnt_size>188*320*40)
				{
					print_pkt_log();
					check_cnt_size=0;
				}
			}
#endif

			if(monitor_size>188*320*40) {
				monitor_size=0;
				mtv_signal_quality_info(hDevice, &slock, &cnr, &berA, &perA, &berB, &perB, &berC, &perC, &i32RSSI, &ui32Quality, &ui32Antlvl);
				print_log("Lock : 0x%x CN : %3d, RSSI : %3d, QT : %d, Ant : %d\n",slock, cnr, i32RSSI, ui32Quality, ui32Antlvl);
				print_log("BerA : %6d, PerA : %6d, BerB : %6d, PerB : %6d, BerC : %6d, PerC : %6d, \n", berA, perA, berB, perB, berC, perC);

			}

			if((dump_size<100*1024*1024)&&(size>0))
				fwrite(&buf[0], 1, size, f);
		}

		mtv_ts_stop(hDevice);
		fclose(f);
		print_log("\nEnd msc dump\n");
		index %= 10;
	}
}

int data_dump_start(int hDevice)
{
	int thr_id;
	static data_dump_param param;
	param.dev = hDevice;
	param.num = 100000000;// 6000

	thr_id = pthread_create(&p_thr_data, NULL, (void*)&data_dump_thread, (void*)&param);

	if(thr_id <0)
	{
		print_log("read thread create error.\n");
		return -1;
	}

	pthread_detach(p_thr_data);
	return 0;
}

u8 sig_start_thread;
#define MON_TIME 1000
void sig_thread(void *param)
{
	data_dump_param *data = (data_dump_param*)param;
	int hDevice = data->dev;
	int num = data->num;

	u8 wscn, lock;
	s32 rssi;
	u32 berA, perA, berB, perB, berC, perC, qty, ant;
 	sig_start_thread = 1;

	while(1)
	{
		mtv_signal_quality_info(hDevice, &lock, &wscn, &berA, &perA, &berB, &perB, &berC, &perC, &rssi, &qty, &ant);
		print_log("Lock : 0x%x CN : %3d, RSSI : %3d, QT : %d, Ant : %d\n",lock, wscn, rssi, qty, ant);
		print_log("BerA : %6d, PerA : %6d, BerB : %6d, PerB : %6d, BerC : %6d, PerC : %6d, \n", berA, perA, berB, perB, berC, perC);
		msWait(MON_TIME);

		if(!sig_start_thread)
			break;
	}
}


int data_sig_start(int hDevice)
{
	int thr_id;
	static data_dump_param param;
	param.dev = hDevice;
	param.num = 100000000;// 6000

	thr_id = pthread_create(&p_thr_sig, NULL, (void*)&sig_thread, (void*)&param);

	if(thr_id <0)
	{
		print_log("sig thread create error.\n");
		return -1;
	}

	pthread_detach(p_thr_sig);
	return 0;
}

int data_sig_stop(int hDevice)
{

	sig_start_thread = 0;
}

u8 ptcheck_thread_start;
void ptcheck_thread(void *param)
{
	data_dump_param *data = (data_dump_param*)param;
	int hDevice = data->dev;
	int num = data->num;
	u8 buf[MSC_BUF_SIZE];
	int check_cnt_size=0;
	int size;
	FILE *f;
	char *f_name[128];

	ptcheck_thread_start = 1;

	print_log("pattern check start \n");

	sprintf((void*)f_name,DUMP_PATH"pattern_ts.dat");
	f = fopen((void*)f_name, "wb");

#ifdef FEATURE_TS_CHECK
	create_tspacket_anal();
#endif
	mtv_ts_start(hDevice);
	while(1)
	{
		size = mtv_data_read(hDevice, buf, MSC_BUF_SIZE);

#ifdef FEATURE_TS_CHECK
		if(!(size%188)) {
			put_ts_packet(0, &buf[0], size);
			check_cnt_size+=size;
			fwrite(&buf[0], 1, size, f);
		}
#ifdef FEATURE_OVERRUN_CHECK
		{
			u8 over;
			BBM_READ(hDevice, 0x8001, &over);
			if(over)
			{
				BBM_WRITE(hDevice, 0x8001, over);
				print_log("TS OVERRUN : %d\n", over);
			}
		}
#endif
		if(check_cnt_size>188*100)
		{
			print_pkt_log();
			check_cnt_size=0;
		}
#endif
		if(!ptcheck_thread_start)
			break;
	}

	mtv_ts_stop(hDevice);
	fclose(f);
	print_log("\nEnd pattern check\n");
}

int ptcheck_start(int hDevice)
{
	int thr_id;
	static data_dump_param param;
	param.dev = hDevice;
	param.num = 100000000;// 6000

	thr_id = pthread_create(&ptcheck_thr_sig, NULL, (void*)&ptcheck_thread, (void*)&param);

	if(thr_id <0)
	{
		print_log("sig thread create error.\n");
		return -1;
	}

	pthread_detach(ptcheck_thr_sig);
	return 0;
}

int ptcheck_stop(int hDevice)
{

	ptcheck_thread_start = 0;
}

