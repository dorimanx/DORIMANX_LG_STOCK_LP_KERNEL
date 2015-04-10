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
 +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

/* FullRawCapacitance Support 0D button */

#include "RefCode_F54.h"
#if defined(CONFIG_MACH_MSM8974_VU3_KR)
#include "TestLimits_vu3.h"
#else
#include "TestLimits.h"
#endif

#define TRX_mapping_max 54
#define LOWER_ABS_ADC_RANGE_LIMIT 60
#define UPPER_ABS_ADC_RANGE_LIMIT 190
#define LOWER_ABS_RAW_CAP_LIMIT 1000 /*fF*/
#define UPPER_ABS_RAW_CAP_LIMIT 14000 /*fF*/
#define REPORT_DATA_OFFEST 3
#define VERSION "1.0"


unsigned int count;
const int DefaultTimeout = 15; /* In counts*/

int pageNum;
int scanMaxPageCount = 5;
int input;

bool bHaveF01;
bool bHaveF11;
bool bHaveF1A;
bool bHaveF12;
bool bHaveF34;
bool bHaveF54;
bool bHaveF55;
bool SignalClarityOn;
bool bHaveF54Ctrl07;
bool bHaveF54Ctrl41;
bool bHaveF54Ctrl57;
bool bHavePixelTouchThresholdTuning;
bool bHaveInterferenceMetric;
bool bHaveCtrl11;
bool bHaveRelaxationControl;
bool bHaveSensorAssignment;
bool bHaveSenseFrequencyControl;
bool bHaveFirmwareNoiseMitigation;
bool bHaveIIRFilter;
bool bHaveCmnRemoval;
bool bHaveCmnMaximum;
bool bHaveTouchHysteresis;
bool bHaveEdgeCompensation;
bool bHavePerFrequencyNoiseControl;
bool bHaveSignalClarity;
bool bHaveMultiMetricStateMachine;
bool bHaveVarianceMetric;
bool bHave0DRelaxationControl;
bool bHave0DAcquisitionControl;
bool bHaveSlewMetric;
bool bHaveHBlank;
bool bHaveVBlank;
bool bHaveLongHBlank;
bool bHaveNoiseMitigation2;
bool bHaveSlewOption;
bool bHaveEnhancedStretch;
bool bHaveStartupFastRelaxation;
bool bHaveESDControl;
bool bHaveEnergyRatioRelaxation;
bool bHaveCtrl86;
bool bHaveCtrl87;
bool bHaveCtrl88;
bool bHaveCtrl89;
bool bHaveCtrl90;
bool bHaveCtrl91;
bool bHaveCtrl92;
bool bHaveCtrl93;
bool bHaveCtrl94;
bool bHaveCtrl95;
bool bHaveCtrl96;
bool bHaveCtrl97;
bool bHaveCtrl98;
bool bHaveCtrl99;
bool bHaveCtrl100;
bool bHaveCtrl101;
bool bHaveCtrl102;
bool bHaveF54Query13;
bool bHaveF54Query15;
bool bHaveF54Query16;
bool bHaveF54Query17;
bool bHaveF54Query18;
bool bHaveF54Query19;
bool bHaveF54Query22;
bool bHaveF54Query23;
bool bHaveF54Query25;
bool ButtonShared;

/*RSP Product Test*/
bool bIncellDevice;
unsigned char F54Ctrl149Offset;
unsigned char F54Ctrl102Offset;
bool bHaveF54Ctrl103;
bool bHaveF54Ctrl104;
bool bHaveF54Ctrl105;
bool bHaveF54Ctrl106;
bool bHaveF54Ctrl107;
bool bHaveF54Ctrl108;
bool bHaveF54Ctrl109;
bool bHaveF54Ctrl110;
bool bHaveF54Ctrl111;
bool bHaveF54Ctrl112;
bool bHaveF54Ctrl113;
bool bHaveF54Ctrl114;
bool bHaveF54Ctrl115;
bool bHaveF54Ctrl116;
bool bHaveF54Ctrl117;
bool bHaveF54Ctrl118;
bool bHaveF54Ctrl119;
bool bHaveF54Ctrl120;
bool bHaveF54Ctrl121;
bool bHaveF54Ctrl122;
bool bHaveF54Ctrl123;
bool bHaveF54Ctrl124;
bool bHaveF54Ctrl125;
bool bHaveF54Ctrl126;
bool bHaveF54Ctrl127;
bool bHaveF54Ctrl128;
bool bHaveF54Ctrl129;
bool bHaveF54Ctrl130;
bool bHaveF54Ctrl131;
bool bHaveF54Ctrl132;
bool bHaveF54Ctrl133;
bool bHaveF54Ctrl134;
bool bHaveF54Ctrl135;
bool bHaveF54Ctrl136;
bool bHaveF54Ctrl137;
bool bHaveF54Ctrl138;
bool bHaveF54Ctrl139;
bool bHaveF54Ctrl140;
bool bHaveF54Ctrl141;
bool bHaveF54Ctrl142;
bool bHaveF54Ctrl143;
bool bHaveF54Ctrl144;
bool bHaveF54Ctrl145;
bool bHaveF54Ctrl146;
bool bHaveF54Ctrl147;
bool bHaveF54Ctrl148;
bool bHaveF54Ctrl149;
bool bHaveF54Query24;
bool bHaveF54Query26;
bool bHaveF54Query27;
bool bHaveF54Query28;
bool bHaveF54Query29;
bool bHaveF54Query30;
bool bHaveF54Query31;
bool bHaveF54Query32;
bool bHaveF54Query33;
bool bHaveF54Query34;
bool bHaveF54Query35;
bool bHaveF54Query36;
bool bHaveF54Query37;
bool bHaveF54Query38;

static const int RSP_COLLECT_FRAMES = 50;
static const int RSP_MAX_ROW = 32;
static const int RSP_MAX_COL = 18;
static const int RSP_MAX_PIXELS = 576;

unsigned char F51DataBase;
unsigned char F51ControlBase;

int RspRawImage[32][18];
int RspImageStack[50][32][18];
int RspAvgImage[32][18];
int RspSlopeImage[32][18];
int oneDArray[50][576];
int twoDShortArray[32][18];
int shortArray[576];

enum EReportType
{
	eRT_Image = 20, /* Full Raw Capacitance with Receiver Offset Removed */
	eRT_RSPGroupTest = 83 /* RSP ADC */
};
/*RSP Product Test*/
unsigned char F54DataBase;
unsigned char F54QueryBase;
unsigned char F54CommandBase;
unsigned char F54ControlBase;
unsigned char F55QueryBase;
unsigned char F55ControlBase;
unsigned char F01ControlBase;
unsigned char F01CommandBase;
unsigned char RxChannelCount;
unsigned char TxChannelCount;
unsigned char TouchControllerFamily;
unsigned char CurveCompensationMode;
unsigned char NumberOfSensingFrequencies;
unsigned char F54Ctrl07Offset;
unsigned char F54Ctrl41Offset;
unsigned char F54Ctrl57Offset;
unsigned char F54Ctrl88Offset;
unsigned char F54Ctrl89Offset;
unsigned char F54Ctrl98Offset;
unsigned char F1AControlBase;
unsigned char F12ControlBase;
unsigned char F12QueryBase;
unsigned char F12_2DTxCount;
unsigned char F12_2DRxCount;
unsigned char ButtonTx[8];
unsigned char ButtonRx[8];
unsigned char ButtonCount;
unsigned char F12Support;
unsigned char F12ControlRegisterPresence;
unsigned char mask;

/* Assuming Tx = 32 & Rx = 32 to accommodate any configuration*/
short Image1[TRX_max][TRX_max];
int ImagepF[TRX_max][TRX_max];
int AbsSigned32Data[TRX_mapping_max];
unsigned char AbsADCRangeData[TRX_mapping_max];
unsigned char Data[TRX_max * TRX_max * 4];
unsigned char TRxPhysical[TRX_mapping_max];

int MaxArrayLength;

unsigned char TREX_mapped[7] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x3f};
unsigned char TRX_Open[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char TRX_Gnd[7] = {0xff, 0xff, 0xff, 0xff, 0x3, 0xff, 0xfc};
unsigned char TRX_Short[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
int HighResistanceLowerLimit[3] = {-1000, -1000, -400};
int HighResistanceUpperLimit[3] = {450, 450, 20};
unsigned int AbsShort[TRX_max*2] = {0};
unsigned int AbsOpen[TRX_max*2] = {0};
int AbsTxShortLimit;
int AbsRxShortLimit;
int AbsTxOpenLimit;
int AbsRxOpenLimit = 1000;
int AbsRawRef[16] = {77, 11919, 14023, 15163, 16192, 18319, 19337, 21491,
	22633, 24692, 26853, 27993, 30147, 32253, 34411, 37605};
short NoiseDeltaMin[TRX_MAX][TRX_MAX];
short NoiseDeltaMax[TRX_MAX][TRX_MAX];
short NoiseLimitLow = -16;
short NoiseLimitHigh = 16;
int RspShortLimit = 15;

enum {
	STARTTIME,
	ENDTIME,
	TIME_PROFILE_MAX
};

#define get_time_interval(a, b) (a >= b ? a - b : 1000000 + a - b)
struct timeval t_interval[TIME_PROFILE_MAX];
static int outbuf;
static int out_buf;
char f54_wlog_buf[6000] = {0};
char wlog_buf[6000] = {0};



/* Function to switch beteen register pages.*/
bool switchPage(int page)
{
	unsigned char values[1] = {0};
	unsigned char data = 0;

	pageNum = values[0] = page;

	count = 0;
	do {
		Write8BitRegisters(0xFF, values, 1);
		msleep(1);
		Read8BitRegisters(0xFF, &data, 1);
		count++;
	} while ((int)data != page && (count < DefaultTimeout));
	if (count >= DefaultTimeout) {
		TOUCH_INFO_MSG("Timeout -- Page switch fail !\n");
		return -EAGAIN;
	}
	return true;
}

void Reset(void)
{
	unsigned char data;

	switchPage(0x00);

	data = 0x01;
	Write8BitRegisters(F01CommandBase, &data, 1);

	msleep(10);
}

/* Compare Report type #20 data against test limits*/
int CompareImageReport(void)
{
	bool result = true;
	int i, j;
	int node_crack_count = 0, rx_crack_count = 0, row_crack_count = 0;

	/*Compare 0D area*/
#if defined(CONFIG_MACH_MSM8974_VU3_KR)
	if (ButtonCount > 0) {
		for (i = 0; i < ButtonCount; i++) {
			for (j = 0; j < (int)F12_2DTxCount; j++) {
				if ((LowerImageLimit[j][F12_2DRxCount+i] > 0)
					&& (UpperImageLimit[j][F12_2DRxCount+i]
						> 0)) {
					if ((ImagepF[j][F12_2DRxCount+i] <
					LowerImageLimit[j][F12_2DRxCount+i]) ||
					(ImagepF[j][F12_2DRxCount+i] >
					 UpperImageLimit[j][F12_2DRxCount+i])) {
						TOUCH_INFO_MSG("[Touch] ButtonCheck-FAIL Tx[%d] Rx[%d]\n", j, F12_2DRxCount+i);
						result = false;
						break;
					}
				}
			}
		}
	}
#else
	if (ButtonCount > 0) {
		for (i = 0; i < ButtonCount; i++) {
			if ((ImagepF[TxChannelCount-1][F12_2DRxCount + i] <
			LowerImageLimit[TxChannelCount-1][F12_2DRxCount + i])
			|| (ImagepF[TxChannelCount-1][F12_2DRxCount + i] >
			UpperImageLimit[TxChannelCount-1][F12_2DRxCount + i])) {
				result = false;
				break;
			}
		}

	}
#endif
	/*Compare 2D area*/
	for (j = 0; j < (int)F12_2DRxCount; j++) {
		rx_crack_count = 0;

		for (i = 0; i < (int)F12_2DTxCount; i++) {
			if ((ImagepF[i][j] < LowerImageLimit[i][j])
					|| (ImagepF[i][j] >
						UpperImageLimit[i][j]))	{
				if (f54_window_crack_check_mode) {
					if (ImagepF[i][j] < 300) {
						rx_crack_count++;
						node_crack_count++;
					} else
						row_crack_count = 0;

					if (F12_2DTxCount <= rx_crack_count)
						row_crack_count++;

					if (2 < row_crack_count) {
						f54_window_crack = 1;
						break;
					}

					if ((int)(F12_2DTxCount *
						F12_2DRxCount * 20 / 100)
						< node_crack_count) {
						result = false;
						f54_window_crack = 1;
						break;
					}

					TOUCH_INFO_MSG("[Touch] Tx [%d] Rx [%d] node_crack_count %d, row_crack_count %d, raw cap %d\n",
							i, j,
							node_crack_count ,
							row_crack_count,
							ImagepF[i][j]);
				} else {
					outbuf += snprintf(f54_wlog_buf+outbuf,
							sizeof(f54_wlog_buf)-outbuf,
							"FAIL, %d,%d,%d\n",
							i, j, ImagepF[i][j]);
					result = false;
					break;
				}
			}
		}
	}

	if (result) {
		TOUCH_INFO_MSG("Full Raw Capacitance Test passed.\n");
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
				"\nFull Raw Capacitance Image "
				"Test passed.\n\n");
	} else {
		TOUCH_INFO_MSG("Full Raw Capacitance Test failed.\n");
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
				"\nFull Raw Capacitance Image "
				"Test failed.\n\n");
	}
	return (result) ? 1 : 0;
}

/* Compare Report type #4 data against test limits*/
int CompareHighResistance(int maxRxpF, int maxTxpF, int minpF)
{
	bool result = true;

	if (maxRxpF > HighResistanceUpperLimit[0]
			|| maxRxpF < HighResistanceLowerLimit[0])
		result = false;
	if (maxTxpF > HighResistanceUpperLimit[1]
			|| maxTxpF < HighResistanceLowerLimit[1])
		result = false;
	if (minpF > HighResistanceUpperLimit[2]
			|| minpF < HighResistanceLowerLimit[2])
		result = false;

	if (result == false) {
		TOUCH_INFO_MSG("HighResistance Test failed.\n");
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
				"HighResistance Test failed.\n\n");
	} else {
		TOUCH_INFO_MSG("HighResistance Test passed.\n");
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
				"HighResistance Test passed.\n\n");
	}
	return (result) ? 1 : 0;
}


/* Compare Report type #22 data against test limits*/
int CompareSensorSpeedReport(void)
{
	bool result = true;
	int i, j = 0;

	for (i = 0; i < (int)F12_2DTxCount; i++) {
		for (j = 0; j < (int)F12_2DRxCount; j++) {
			if ((ImagepF[i][j] < SensorSpeedLowerImageLimit[i][j])
			|| (ImagepF[i][j] > SensorSpeedUpperImageLimit[i][j])) {
				result = false;
				TOUCH_INFO_MSG("Failed : Tx[%d] Rx[%d] -> LOWER : %d Upper : %d  IMAGE DATA : %d\n",
					i, j,
					SensorSpeedLowerImageLimit[i][j],
					SensorSpeedUpperImageLimit[i][j],
					ImagepF[i][j]);
				out_buf += snprintf(wlog_buf+out_buf, sizeof(wlog_buf)-out_buf,
					"Failed : Tx[%2d] Rx[%2d] = %3d\n",
					i, j, ImagepF[i][j]);
				break;
			}
		}
	}

	if (result) {
		TOUCH_INFO_MSG("Sensor Speed Test passed.\n");
		out_buf += snprintf(wlog_buf+out_buf, sizeof(wlog_buf)-out_buf,
				"\nSensor Speed Test passed.\n\n");
	} else {
		TOUCH_INFO_MSG("Sensor Speed Test failed.\n");
		out_buf += snprintf(wlog_buf+out_buf, sizeof(wlog_buf)-out_buf,
				"\nSensor Speed Test failed.\n\n");
	}

	return (result) ? 1 : 0;
}

/* Compare Report type #23 data against test limits*/
int CompareADCReport(void)
{
	bool result = true;
	int i, j = 0;

	for (i = 0; i < (int)F12_2DTxCount; i++) {
		for (j = 0; j < (int)F12_2DRxCount; j++) {
			if ((Image1[i][j] < ADCLowerImageLimit[i][j]) ||
				(Image1[i][j] > ADCUpperImageLimit[i][j])) {
				TOUCH_INFO_MSG("[Touch] Failed : Tx[%d] Rx[%d] -> LOWER : %d Upper : %d IMAGE DATA : %u\n",
					i, j, ADCLowerImageLimit[i][j],
					ADCUpperImageLimit[i][j],
					Image1[i][j]);
				out_buf += snprintf(wlog_buf+out_buf, sizeof(wlog_buf)-out_buf,
					"Failed : Tx[%2d] Rx[%2d] = %3u\n",
					i, j, Image1[i][j]);
				result = false;
				break;
			}
		}
	}
	if (result) {
		TOUCH_INFO_MSG("ADC Range Test passed.\n");
		out_buf += snprintf(wlog_buf+out_buf, sizeof(wlog_buf)-out_buf,
				"\nADC Range Test passed.\n\n");
	} else {
		TOUCH_INFO_MSG("ADC Range Test failed.\n");
		out_buf += snprintf(wlog_buf+out_buf, sizeof(wlog_buf)-out_buf,
				"\nADC Range Test failed.\n\n");
	}

	return (result) ? 1 : 0;
}

void CompareAbsADCRangeReport(void)
{
	bool result = true;
	int i = 0;

	for (i = 0; i < RxChannelCount + F12_2DTxCount; i++) {
		if (i == F12_2DRxCount)
			i = RxChannelCount;
		if ((AbsADCRangeData[i] < LOWER_ABS_ADC_RANGE_LIMIT)
				|| (AbsADCRangeData[i] >
					UPPER_ABS_ADC_RANGE_LIMIT)) {
			result = false;
			break;
		}
	}

	if (result) {
		TOUCH_INFO_MSG("\nAbs Sensing ADC Range Test Passed.\n");
	} else {
		TOUCH_INFO_MSG("\nAbs Sensing ADC Range Test Failed.\n");
	}
}

void CompareAbsRawReport(void)
{
	bool result = true;
	int i = 0;

	for (i = 0; i < RxChannelCount + F12_2DTxCount; i++) {
		if (i == F12_2DRxCount)
			i = RxChannelCount;
		if ((AbsSigned32Data[i] < LOWER_ABS_RAW_CAP_LIMIT)
				|| (AbsSigned32Data[i]
					> UPPER_ABS_RAW_CAP_LIMIT)) {
			result = false;
			break;
		}
	}

	if (result) {
		TOUCH_INFO_MSG("\nAbs Sensing Raw Capacitance Test Passed.\n");
	} else {
		TOUCH_INFO_MSG("\nAbs Sensing Raw Capacitance Test Failed.\n");
	}
}

int CompareAbsOpen(void)
{
	bool result = true;
	int i = 0;

	for (i = 0; i < ((int)F12_2DRxCount + (int)F12_2DTxCount); i++) {
		if (i < (int)F12_2DRxCount) {
			if (AbsOpen[i] <= AbsRxOpenLimit) {
				result = false;
				TOUCH_INFO_MSG("RX[%d] failed value:  %d\n",
						i, AbsOpen[i]);
			}

		} else {
			if (AbsOpen[i] <= AbsTxOpenLimit) {
				result = false;
				TOUCH_INFO_MSG("TX[%d] failed value:  %d\n",
					i - (int)F12_2DRxCount, AbsOpen[i]);
			}
		}

	}

	TOUCH_INFO_MSG("AbsRxOpenLimit:  %d  AbsTxOpenLimit : %d\n",
			AbsRxOpenLimit, AbsTxOpenLimit);

	if (result) {
		TOUCH_INFO_MSG("Abs Sensing Open Test Passed.\n");
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
				"Abs Sensing Open Test passed.\n\n");
	} else {
		TOUCH_INFO_MSG("Abs Sensing Open Test Failed.\n");
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
				"Abs Sensing Open Test failed.\n\n");
	}

	return (result) ? 1 : 0;
}

int CompareAbsShort(void)
{
	bool result = true;
	int i = 0;

	for (i = 0; i < ((int)F12_2DRxCount + (int)F12_2DTxCount); i++) {
		if (i < (int)F12_2DRxCount) {
			if (AbsShort[i] >= AbsRxShortLimit) {
				result = false;
				TOUCH_INFO_MSG("RX[%d] failed value:  %d\n",
						i, AbsShort[i]);
			}

		} else {
			if (AbsShort[i] >= AbsTxShortLimit) {
				result = false;
				TOUCH_INFO_MSG("TX[%d] failed value:  %d\n",
					i - (int)F12_2DRxCount, AbsShort[i]);
			}

		}

	}

	TOUCH_INFO_MSG("AbsRxShortLimit:  %d  AbsTxShortLimit : %d\n",
			AbsRxShortLimit , AbsTxShortLimit);

	if (result) {
		TOUCH_INFO_MSG("Abs Sensing Short Test Passed.\n");
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
				"Abs Sensing Short Test passed.\n\n");
	} else {
		TOUCH_INFO_MSG("Abs Sensing Short Test Failed.\n");
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
				"Abs Sensing Short Test failed.\n\n");
	}

	return (result) ? 1 : 0;

}

/* Compare Report type #24 data against test limits*/
void CompareTRexOpenTestReport(int i)
{
	int index;
	for (index = 0; index < 7; index++) {
		if (Data[index] != TRX_Open[index]) {
			TOUCH_INFO_MSG("\nTRex Open Test failed.\n");
			return;
		}
	}

	TOUCH_INFO_MSG("\nTRex Open Test passed.\n");
}

/* Compare Report type #25 data against test limits*/
int CompareTRexGroundTestReport(int i)
{
	int index;

	for (index = 0; index < 7; index++) {
		if (Data[index] !=  TRX_Gnd[index]) {
			outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
					"\nTRex Ground Test failed.\n\n");
		}
	}

	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
			"\nTRex Ground Test passed.\n\n");
	return 1;
}

/* Compare Report type #26 data against test limits*/
int CompareTRexShortTestReport(int i)
{
	int index;
	for (index = 0; index < 7; index++) {
		if (Data[index] != TRX_Short[index]) {
			TOUCH_INFO_MSG("TRex-TRex Short Test failed.\n");
			outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
					"\nTRex-TRex Short Test failed.\n\n");
			return 0;
		}
	}

	TOUCH_INFO_MSG("TRex-TRex Short Test passed.\n");
	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
			"\nTRex-TRex Short Test passed.\n\n");

	return 1;
}

/* Compare Report type #2 data against test limits*/
int CompareNoiseReport(void)
{
	bool result = true;
	int i, j = 0;

	out_buf += snprintf(wlog_buf+out_buf, sizeof(wlog_buf)-out_buf, "\nNoise Test Data :\n");
	out_buf += snprintf(wlog_buf+out_buf, sizeof(wlog_buf)-out_buf, "==========================================================================================================\n         :");

	for (i = 0; i < (int)RxChannelCount; i++)
		out_buf += snprintf(wlog_buf+out_buf, sizeof(wlog_buf)-out_buf, "%5d ", i);

	out_buf += snprintf(wlog_buf+out_buf, sizeof(wlog_buf)-out_buf,
			"\n----------------------------------------------------------------------------------------------------------\n");
	for (i = 0; i < TxChannelCount; i++) {
		out_buf += snprintf(wlog_buf+out_buf, sizeof(wlog_buf)-out_buf, "   %5d : ", i);
		for (j = 0; j < RxChannelCount; j++) {
			ImagepF[i][j] = NoiseDeltaMax[i][j] -
				NoiseDeltaMin[i][j];
			out_buf += snprintf(wlog_buf+out_buf,
					sizeof(wlog_buf)-out_buf, "%5d ", ImagepF[i][j]);
		}
		out_buf += snprintf(wlog_buf+out_buf, sizeof(wlog_buf)-out_buf, "\n");
	}
	out_buf += snprintf(wlog_buf+out_buf, sizeof(wlog_buf)-out_buf,
			"------------------------------------------------------------------------------------------------------------\n");

	/*Compare 0D area*/
	/*for (int32_t i = 1; i <= pdt.ButtonCount; i++){
	if ((ImagepF[pdt.TxChannelCount - i][pdt._2DRxCount]
	< NoiseLimitLow) ||
	(ImagepF[pdt.TxChannelCount - i][pdt._2DRxCount] >
	NoiseLimitHigh)){
	printf("\tFailed: Button area: TxChannel [%d] RxChannel[%d]\n",
	pdt.TxChannelCount-i, pdt._2DRxCount);
	result = false;
	}
	}
	*/
	/*Compare 2D area*/
	for (i = 0; i < F12_2DTxCount; i++) {
		for (j = 0; j < F12_2DRxCount; j++) {
			if ((ImagepF[i][j] < NoiseLimitLow) ||
					(ImagepF[i][j] > NoiseLimitHigh)) {
				TOUCH_INFO_MSG(
					"\tFailed: 2D area: Tx [%d] Rx [%d]\n",
					i, j);
				out_buf += snprintf(wlog_buf+out_buf, sizeof(wlog_buf)-out_buf,
					"Failed Tx [%2d] Rx [%2d] = %3d\n",
					i, j, ImagepF[i][j]);
				result = false;
			}
		}
	}

	if (result == false) {
		TOUCH_INFO_MSG("Noise Test failed.\n");
		out_buf += snprintf(wlog_buf+out_buf, sizeof(wlog_buf)-out_buf,
				"\nNoise Test failed.\n\n");
	} else {
		TOUCH_INFO_MSG("Noise Test passed.\n");
		out_buf += snprintf(wlog_buf+out_buf, sizeof(wlog_buf)-out_buf,
				"\nNoise Test passed.\n\n");
	}

	return (result) ? 1 : 0;
}

int CompareGndReport(void)
{
	bool result = true;
	int i, j = 0;
	int negative_cnt = 0;
	int negative = 0;
	int negative_avg = 0;
	int negative_avg_m = 0;

	for (i = 0; i < F12_2DTxCount; i++) {
		for (j = 0; j < F12_2DRxCount; j++) {
			if (ImagepF[i][j] < 0) {
				negative -= (ImagepF[i][j] * ImagepF[i][j]);
				negative_cnt++;
			}
		}
	}

	negative_avg = negative/ negative_cnt;
	negative_avg_m = ((negative * 100) / negative_cnt) - (negative_avg * 100);

	if(negative_avg <= -5)
	{
		result = false;
	}


	out_buf += snprintf(wlog_buf+out_buf, sizeof(wlog_buf)-out_buf,
		"negative sum : %5d, negative count: %5d, delta: %5d.%02d, Result : %s\n",
		negative, negative_cnt, negative_avg, negative_avg_m * (-1), result == true? "PASS" : "FAIL");

	if (result == false) {
		TOUCH_INFO_MSG("GND Test failed.\n");
		out_buf += snprintf(wlog_buf+out_buf, sizeof(wlog_buf)-out_buf,
				"\nGND Test failed.\n\n");
	} else {
		TOUCH_INFO_MSG("GND Test passed.\n");
		out_buf += snprintf(wlog_buf+out_buf, sizeof(wlog_buf)-out_buf,
				"\nGND Test passed.\n\n");
	}

	return (result) ? 1 : 0;
}
void RspReadImageReport(void)
{
	int k = 0;
	int i = 0;
	int j = 0;
	Read8BitRegisters(F54DataBase + REPORT_DATA_OFFEST, &Data[0], MaxArrayLength);
	for (i = 0; i <TxChannelCount; i++){
		for (j = 0; j <RxChannelCount; j++){
			Image1[i][j] = (int)Data[k] | ((int)Data[k + 1] << 8);
			ImagepF[i][j] = Image1[i][j];
			if ((i<32) && (j<18))
			{
				RspRawImage[i][j] = Image1[i][j];
			}
			k = k + 2;
		}
	}
	/*Reset Device*/
	Reset();
}
/* Construct data with Report Type #20 data*/
int ReadImageReport(void)
{
	int ret = 0;
	int i, j, k = 0;

	Read8BitRegisters((F54DataBase+REPORT_DATA_OFFEST),
			&Data[0], MaxArrayLength);

	TOUCH_INFO_MSG("Full Raw Capacitance Test\n");
	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf, "\nInfo: Tx = %d Rx = %d\n",
			(int)TxChannelCount, (int)RxChannelCount);
	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf, "Image Data :\n");
	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
			"==========================================================================================================\n         :");

	for (i = 0; i < (int)RxChannelCount; i++)
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf, "%5d ", i);
	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
			"\n----------------------------------------------------------------------------------------------------------\n");

	for (i = 0; i < (int)TxChannelCount; i++) {
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf, "   %5d : ", i);
		for (j = 0; j < (int)RxChannelCount; j++) {
			Image1[i][j] = ((short)Data[k]
					| (short)Data[k + 1] << 8);
			ImagepF[i][j] = Image1[i][j];
			outbuf += snprintf(f54_wlog_buf + outbuf, sizeof(f54_wlog_buf)-outbuf, "%5d ",
					ImagepF[i][j]);
			k = k + 2;
		}
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf, "\n");
	}
	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
			"------------------------------------------------------------------------------------------------------------\n");

	ret = CompareImageReport();
	write_log(NULL, f54_wlog_buf);
	msleep(30);

	/*Reset Device*/
	Reset();

	return ret;
}

/* Construct data with Report Type #20 data*/
int GetImageReport(char *buf)
{
	int ret = 0;
	int i, j, k = 0;

	Read8BitRegisters((F54DataBase + REPORT_DATA_OFFEST),
			&Data[0], MaxArrayLength);

	*buf = 0;
	ret += snprintf(buf+ret, PAGE_SIZE-ret, "\n\nInfo: Tx = %d Rx = %d\n\n",
			(int)TxChannelCount, (int)RxChannelCount);
	ret += snprintf(buf+ret, PAGE_SIZE-ret,
			"========================================="
			"================================================="
			"================\n         :");

	for (i = 0; i < (int)RxChannelCount; i++)
		ret += snprintf(buf+ret, PAGE_SIZE-ret, "%5d ", i);

	ret += snprintf(buf+ret, PAGE_SIZE-ret,
			"\n---------------------------------------"
			"-------------------------------------------------"
			"------------------\n");

	for (i = 0; i < (int)TxChannelCount; i++) {
		ret += snprintf(buf+ret, PAGE_SIZE-ret, "   %5d : ", i);
		for (j = 0; j < (int)RxChannelCount; j++) {
			Image1[i][j] = ((short)Data[k]
					| (short)Data[k + 1] << 8);
			ImagepF[i][j] = Image1[i][j];
			ret += snprintf(buf + ret, PAGE_SIZE-ret, "%5d ", ImagepF[i][j]);
			k = k + 2;
		}
		ret += snprintf(buf + ret, PAGE_SIZE-ret, "\n");
	}
	ret += snprintf(buf + ret, PAGE_SIZE-ret,
			"-----------------------------------------"
			"-------------------------------------------------"
			"------------------\n");

	/*Reset Device*/
	Reset();

	return ret;
}

int ReadGndReport(void)
{
	int ret = 0;
	int i, j, k = 0;

	Read8BitRegisters((F54DataBase+REPORT_DATA_OFFEST),
			&Data[0], MaxArrayLength);

	TOUCH_INFO_MSG("GND Test\n");
	out_buf += snprintf(wlog_buf+out_buf, sizeof(wlog_buf)-out_buf, "\nInfo: Tx = %d Rx = %d\n",
			(int)TxChannelCount, (int)RxChannelCount);
	out_buf += snprintf(wlog_buf+out_buf, sizeof(wlog_buf)-out_buf, "Delta Data :\n");
	out_buf += snprintf(wlog_buf+out_buf, sizeof(wlog_buf)-out_buf,
			"==========================================================================================================\n         :");

	for (i = 0; i < (int)RxChannelCount; i++)
		out_buf += snprintf(wlog_buf+out_buf, sizeof(wlog_buf)-out_buf, "%5d ", i);
	out_buf += snprintf(wlog_buf+out_buf, sizeof(wlog_buf)-out_buf,
			"\n----------------------------------------------------------------------------------------------------------\n");

	for (i = 0; i < (int)TxChannelCount; i++) {
		out_buf += snprintf(wlog_buf+out_buf, sizeof(wlog_buf)-out_buf, "   %5d : ", i);
		for (j = 0; j < (int)RxChannelCount; j++) {
			Image1[i][j] = ((short)Data[k]
					| (short)Data[k + 1] << 8);
			ImagepF[i][j] = Image1[i][j];
			out_buf += snprintf(wlog_buf + out_buf, sizeof(wlog_buf)-out_buf, "%5d ",
					ImagepF[i][j]);
			k = k + 2;
		}
		out_buf += snprintf(wlog_buf+out_buf, sizeof(wlog_buf)-out_buf, "\n");
	}
	out_buf += snprintf(wlog_buf+out_buf, sizeof(wlog_buf)-out_buf,
			"------------------------------------------------------------------------------------------------------------\n");

	ret = CompareGndReport();
	write_log(NULL, wlog_buf);
	msleep(30);

	/*Reset Device*/
	Reset();

	return ret;
}

/* Construct data with Report Type #2 data*/
int ReadNoiseReport(void)
{
	int ret = 0;
	int i, j, k = 0;

	/*set FIFO index*/
	unsigned char fifoIndex[2] = {0, 0};
	Write8BitRegisters(F54DataBase + 1, fifoIndex, sizeof(fifoIndex));

	Read8BitRegisters((F54DataBase+REPORT_DATA_OFFEST),
			&Data[0], MaxArrayLength);

	TOUCH_INFO_MSG("Noise Test\n");
	for (i = 0; i < (int)TxChannelCount; i++) {
		for (j = 0; j < (int)RxChannelCount; j++) {
			Image1[i][j] = (short)Data[k]
				| ((short)Data[k + 1] << 8);
			ImagepF[i][j] = Image1[i][j];

			if (ImagepF[i][j] < NoiseDeltaMin[i][j])
				NoiseDeltaMin[i][j] = ImagepF[i][j];

			if (ImagepF[i][j] > NoiseDeltaMax[i][j])
				NoiseDeltaMax[i][j] = ImagepF[i][j];

			k = k + 2;
		}
	}
	ret = CompareNoiseReport();
	write_log(NULL, wlog_buf);
	msleep(30);

	Reset();

	return ret;
}

/* Construct data with Report Type #4 data*/
int ReadHighResistanceReport(void)
{
	short maxRx, maxTx, min;
	int maxRxpF, maxTxpF, minpF;
	int ret = 0;
	int i = 0;

	Read8BitRegisters((F54DataBase+REPORT_DATA_OFFEST), Data, 6);

	maxRx = ((short)Data[0] | (short)Data[1] << 8);
	maxTx = ((short)Data[2] | (short)Data[3] << 8);
	min = ((short)Data[4] | (short)Data[5] << 8);

	maxRxpF = maxRx;
	maxTxpF = maxTx;
	minpF = min;

	TOUCH_INFO_MSG("High Resistance Test\n");
	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
			"Max Rx Offset(pF) = %d\n", maxRxpF);
	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
			"Max Tx Offset(pF) = %d\n", maxTxpF);
	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
			"Min(pF) = %d\n", minpF);
	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
		"\n=====================================================\n");
	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
			"\tHigh Resistance Test\n");
	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
		"=====================================================\n");
	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf, " Parameters: ");
	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf, "%5d %5d %5d ",
			maxRxpF, maxTxpF, minpF);
	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf, "\n\n Limits(+) : ");
	for (i = 0; i < 3; i++)
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf, "%5d ",
				HighResistanceUpperLimit[i]);

	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf, "\n Limits(-) : ");
	for (i = 0; i < 3; i++)
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf, "%5d ",
				HighResistanceLowerLimit[i]);

	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
		"\n-----------------------------------------------------\n");

	ret = CompareHighResistance(maxRxpF, maxTxpF, minpF);
	write_log(NULL, f54_wlog_buf);
	msleep(30);

	/*Reset Device*/
	Reset();

	return ret;
}

/* Construct data with Report Type #13 data*/
void ReadMaxMinReport(void)
{
	short max, min;
	int maxpF, minpF;

	Read8BitRegisters((F54DataBase+REPORT_DATA_OFFEST), Data, 4);

	max = ((short)Data[0] | (short)Data[1] << 8);
	min = ((short)Data[2] | (short)Data[3] << 8);
	maxpF = max;
	minpF = min;

	TOUCH_INFO_MSG("\nRaw Capacitance Maximum and Minimum Test:\n");
	/*TOUCH_INFO_MSG("Max = 0x%x\n", max);
	  TOUCH_INFO_MSG("Min = 0x%x\n", min);*/
	TOUCH_INFO_MSG("Max(pF) = %d\n", maxpF);
	TOUCH_INFO_MSG("Min(pF) = %d\n", minpF);

	/*Reset Device*/
	Reset();
}

/* Construct data with Report Type #23 data*/
int ReadADCRangeReport(void)
{
	int temp = TxChannelCount;
	int ret = 0;
	int i, j, k = 0;

	if (SignalClarityOn) {
		if ((TxChannelCount / 4) != 0)	{
			temp = (4 - (TxChannelCount % 4)) +  TxChannelCount;
			Read8BitRegisters((F54DataBase+REPORT_DATA_OFFEST),
					Data, (temp * RxChannelCount * 2));
		}
	} else {
		Read8BitRegisters((F54DataBase+REPORT_DATA_OFFEST),
				&Data[0], MaxArrayLength);
	}

	k = 0;

	TOUCH_INFO_MSG("ADC Range Test\n");
	for (i = 0; i < (int)TxChannelCount; i++) {
		for (j = 0; j < (int)RxChannelCount; j++) {
			Image1[i][j] = ((unsigned short)Data[k]);
			k = k + 2;
		}
	}

	ret = CompareADCReport();
	write_log(NULL, wlog_buf);
	msleep(20);

	/*Reset Device*/
	Reset();

	return ret;
}

void ReadAbsADCRangeReport(void)
{
	int i, k = 0;

	Read8BitRegisters((F54DataBase + REPORT_DATA_OFFEST),
			&Data[0], 2 * (RxChannelCount + TxChannelCount));

	TOUCH_INFO_MSG("Abs Sensing ADC Range Data:\n");
	TOUCH_INFO_MSG("Rx: ");
	for (i = 0; i < (int)RxChannelCount; i++) {
		AbsADCRangeData[k / 2] = (unsigned char)Data[k];
		TOUCH_INFO_MSG("%d ", AbsADCRangeData[k / 2]);
		k = k + 2;
	}
	TOUCH_INFO_MSG("\n");
	TOUCH_INFO_MSG("Tx: ");
	for (i = 0; i < (int)TxChannelCount; i++) {
		AbsADCRangeData[k / 2] = (unsigned char)Data[k];
		TOUCH_INFO_MSG("%d ", AbsADCRangeData[k / 2]);
		k = k + 2;
	}
	TOUCH_INFO_MSG("\n");

	CompareAbsADCRangeReport();

	Reset();
}

void ReadAbsDeltaReport(void)
{
	int i, k = 0;
	int *p32data;

	Read8BitRegisters((F54DataBase + REPORT_DATA_OFFEST),
			&Data[0], 4 * (RxChannelCount + TxChannelCount));

	p32data = (int *)&Data[0];

	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
			"Abs Sensing Delta Capacitance Data:\n");
	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf, "Rx: ");
	for (i = 0; i < (int)RxChannelCount; i++) {
		AbsSigned32Data[k] = (int)*p32data;
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf, "%d ",
				AbsSigned32Data[k]);
		k++;
		p32data++;
	}

	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf, "\nTx: ");
	for (i = 0; i < (int)TxChannelCount; i++) {
		AbsSigned32Data[k] = (int)*p32data;
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf, "%d ",
				AbsSigned32Data[k]);
		k++;
		p32data++;
	}

	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf, "\n");
	Reset();
}

void ReadAbsRawReport(void)
{
	int i, k = 0;
	int *p32data;

	Read8BitRegisters((F54DataBase + REPORT_DATA_OFFEST),
			&Data[0], 4 * (RxChannelCount + TxChannelCount));

	p32data = (int *)&Data[0];

	TOUCH_INFO_MSG("Abs Sensing Raw Capacitance Data:\n");
	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
			"Abs Sensing Raw Capacitance Data:\n");
	TOUCH_INFO_MSG("Rx: ");
	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf, "Rx: ");
	for (i = 0; i < (int)RxChannelCount; i++) {
		AbsSigned32Data[k] = (int)*p32data;
		TOUCH_INFO_MSG("%d ", AbsSigned32Data[k]);
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf, "%d ",
				AbsSigned32Data[k]);
		k++;
		p32data++;
	}
	TOUCH_INFO_MSG("\n");
	TOUCH_INFO_MSG("Tx: ");
	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf, "\nTx: ");
	for (i = 0; i < (int)TxChannelCount; i++) {
		AbsSigned32Data[k] = (int)*p32data;
		TOUCH_INFO_MSG("%d ", AbsSigned32Data[k]);
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf, "%d ",
				AbsSigned32Data[k]);
		k++;
		p32data++;
	}
	TOUCH_INFO_MSG("\n");
	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf, "\n");

	CompareAbsRawReport();

	Reset();
}

/* Construct data with Report Type #38 data*/
int ReadAbsRawOpen(void)
{
	int i = 0;
	int ret = 0;
	unsigned char k = 0;

	Read8BitRegisters((F54DataBase + REPORT_DATA_OFFEST),
			&Data[0], (F12_2DRxCount + F12_2DTxCount) * 4);

	TOUCH_INFO_MSG("Abs Sensing Open Test Data:\n");
	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
			"Abs Sensing Open Test Data:\n");

	for (i = 0; i < ((int)F12_2DRxCount + (int)F12_2DTxCount); i++)	{
		AbsOpen[i] = (unsigned int)Data[k] |
			((unsigned int)Data[k + 1] << 8) |
			((unsigned int)Data[k + 2] << 16) |
			((unsigned int)Data[k + 3] << 24);

		k += 4;

		if (i < (int)F12_2DRxCount) {
			TOUCH_INFO_MSG("RX[%d]: %d, ",
					i, AbsOpen[i]);
			outbuf += snprintf(f54_wlog_buf+outbuf,
					sizeof(f54_wlog_buf)-outbuf, "%5d ", AbsOpen[i]);
		} else {
			TOUCH_INFO_MSG("TX[%d]: %d, ",
					i - (int)F12_2DRxCount, AbsOpen[i]);
			outbuf += snprintf(f54_wlog_buf+outbuf,
					sizeof(f54_wlog_buf)-outbuf, "%5d ", AbsOpen[i]);
		}

		if (i == ((int)F12_2DRxCount - 1)) {
			TOUCH_INFO_MSG("\n");
			outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf, "\n");
		}
	}
	TOUCH_INFO_MSG("\n");

	ret = CompareAbsOpen();

	return ret;
}

/* Construct data with Report Type #38 data*/
int ReadAbsRawShort(void)
{
	int i = 0;
	int ret = 0;
	unsigned char k = 0;

	Read8BitRegisters((F54DataBase + REPORT_DATA_OFFEST),
			&Data[0], (F12_2DRxCount + F12_2DTxCount) * 4);

	TOUCH_INFO_MSG("Abs Sensing Short Test Data:\n");
	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
			"Abs Sensing Short Test Data:\n");

	for (i = 0; i < ((int)F12_2DRxCount + (int)F12_2DTxCount); i++)	{

		AbsShort[i] = (unsigned int)Data[k] |
			((unsigned int)Data[k + 1] << 8) |
			((unsigned int)Data[k + 2] << 16) |
			((unsigned int)Data[k + 3] << 24);

		k += 4;

		if (i < (int)F12_2DRxCount) {
			TOUCH_INFO_MSG("RX[%d]: %d, ",
					i, AbsShort[i]);
			outbuf += snprintf(f54_wlog_buf+outbuf,
					sizeof(f54_wlog_buf)-outbuf, "%5d ", AbsShort[i]);
		} else {
			TOUCH_INFO_MSG("TX[%d]: %d, ",
					i - (int)F12_2DRxCount, AbsShort[i]);
			outbuf += snprintf(f54_wlog_buf+outbuf,
					sizeof(f54_wlog_buf)-outbuf, "%5d ", AbsShort[i]);
		}

		if (i == ((int)F12_2DRxCount - 1)) {
			TOUCH_INFO_MSG("\n");
			outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf, "\n");
		}
	}
	TOUCH_INFO_MSG("\n");

	ret = CompareAbsShort();

	return ret;
}

/* Construct data with Report Type #22 data*/
int ReadSensorSpeedReport(void)
{
	int i, j, k = 0;
	int ret = 0;

	Read8BitRegisters((F54DataBase+REPORT_DATA_OFFEST),
			&Data[0], MaxArrayLength);

	TOUCH_INFO_MSG("Sensor Speed Test\n");
	for (i = 0; i < (int)TxChannelCount; i++) {
		for (j = 0; j < (int)RxChannelCount; j++) {
			Image1[i][j] = ((short)Data[k]
					| (short)Data[k+1] << 8);
			ImagepF[i][j] = Image1[i][j];
			k = k + 2;
		}
	}

	ret = CompareSensorSpeedReport();
	write_log(NULL, wlog_buf);
	msleep(20);

	/*Reset Device*/
	Reset();

	return ret;
}

int pow_func(int x, int y)
{
	int result = 1;
	int i = 0;
	for (i = 0; i < y; i++)
		result *= x;
	return result;
}

/* Construct data with Report Type #24 data*/
void ReadTRexOpenReport(void)
{
	int i, j = 0;
	/* Hardcode for Waikiki Test and it support up to 54 Tx*/
	int k = 7, mask = 0x01, value;

	Read8BitRegisters((F54DataBase+REPORT_DATA_OFFEST), Data, k);

	for (i = 0; i < k; i++) {
		value = Data[i];
		Data[i] = 0;
		for (j = 0; j < 8; j++) {
			if ((value & mask) == 1) {
				Data[i] = Data[i] +
					(unsigned char)pow_func(2, (7 - j));
			}
			value >>= 1;
		}
		TOUCH_INFO_MSG("TRex-Open Test Data = %#x,", Data[i]);
	}
	TOUCH_INFO_MSG("\n");

	CompareTRexOpenTestReport(k * 8);

	/*Reset Device*/
	Reset();
}

/* Construct data with Report Type #25 data*/
int ReadTRexGroundReport(void)
{
	int ret = 0;
	int i, j = 0;
	/* Hardcode for Waikiki Test and it support up to 54 Tx*/
	int k = 7, mask = 0x01, value;
	Read8BitRegisters((F54DataBase+REPORT_DATA_OFFEST), Data, k);

	for (i = 0; i < k; i++) {
		value = Data[i];
		Data[i] = 0;
		for (j = 0; j < 8; j++) {
			if ((value & mask) == 1) {
				Data[i] = Data[i] +
					(unsigned char)pow_func(2, (7 - j));
			}
			value >>= 1;
		}

		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
				"TRex-Ground Test Data = %#x\n", Data[i]);
	}
	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf, "\n");

	ret = CompareTRexGroundTestReport(k * 8);
	write_log(NULL, f54_wlog_buf);
	msleep(30);
	/*Reset Device*/
	Reset();

	return ret;
}

/* Construct data with Report Type #26 data*/
int ReadTRexShortReport(void)
{
	int ret = 0;
	int i, j = 0;
	/* Hardcode for Waikiki Test and it support up to 54 Tx*/
	int k = 7, mask = 0x01, value;

	Read8BitRegisters((F54DataBase+REPORT_DATA_OFFEST), Data, k);

	TOUCH_INFO_MSG("TRex-TRex Short Test\n");
	for (i = 0; i < k; i++)	{
		value = Data[i];
		Data[i] = 0;
		for (j = 0; j < 8; j++) {
			if ((value & mask) == 1) {
				Data[i] = Data[i] +
					(unsigned char)pow_func(2, (7 - j));
			}
			value >>= 1;
		}
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
				"TRex-TRex Short Test Data = %#x\n", Data[i]);
	}

	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf, "\n");

	ret = CompareTRexShortTestReport(k * 8);
	write_log(NULL, f54_wlog_buf);
	msleep(30);

	/*Reset Device*/
	Reset();

	return ret;
}
void RspReadReport(enum EReportType input)
{
	unsigned char data=0;
	int count=0;
	Read8BitRegisters(F54CommandBase, &data, 1);
	if (data & 0x01)
		TOUCH_INFO_MSG("Getreport = 1\n");

	data = (int)(input);
	Write8BitRegisters(F54DataBase, &data, 1);
	data = 0x01;
	Write8BitRegisters(F54CommandBase, &data, 1);

	do {
		Read8BitRegisters(F54CommandBase, &data, 1);
		msleep(1);
		count++;
	} while (data != 0x00 && (count < DefaultTimeout));
	if(count >= DefaultTimeout){
		TOUCH_INFO_MSG("Timeout -- Not supported Report Type in FW\n");
		Reset();
		return;
	}
	switch (input){
		case eRT_RSPGroupTest:
			RspReadImageReport();
			break;
		case eRT_Image:
			RspReadImageReport();
			break;
		default:
			break;
	}
}
/* Function to handle report reads based on user input*/
int ReadReport(unsigned char input, char *buf)
{
	int ret = 0;
	unsigned char data;

	/*Set the GetReport bit to run the AutoScan*/
	data = 0x01;
	DO_SAFE(Write8BitRegisters(F54CommandBase, &data, 1), error);

	count = 0;
	do {
		DO_SAFE(Read8BitRegisters(F54CommandBase, &data, 1), error);
		msleep(1);
		count++;
	} while (data != 0x00 && (count < DefaultTimeout));
	if (count >= DefaultTimeout) {
		TOUCH_INFO_MSG("Timeout - Not supported Report Type in FW\n");
		Reset();
		return -EAGAIN;
	}

	do_gettimeofday(&t_interval[ENDTIME]);

	TOUCH_INFO_MSG("Takes %lu ticks\n",
			get_time_interval(t_interval[ENDTIME].tv_sec,
				t_interval[STARTTIME].tv_sec));

	switch (input) {
	case 'a':
		ret = ReadImageReport();
		break;
	case 'b':
		ret = ReadADCRangeReport();
		break;
	case 'c':
		ret = ReadSensorSpeedReport();
		break;
	case 'd':
		ReadTRexOpenReport();
		break;
	case 'e':
		ret = ReadTRexGroundReport();
		break;
	case 'f':
		ret = ReadTRexShortReport();
		break;
	case 'g':
		ret = ReadHighResistanceReport();
		break;
	case 'h':
		ReadMaxMinReport();
		break;
	case 'i':
		ReadAbsADCRangeReport();
		break;
	case 'j':
		ReadAbsDeltaReport();
		break;
	case 'k':
		ReadAbsRawReport();
		break;
	case 'l':
		ret = GetImageReport(buf);
		break;
	case 'm':
		ret = ReadNoiseReport();
		break;
	case 'n':
		ret = ReadAbsRawShort();
		break;
	case 'o':
		ret = ReadAbsRawOpen();
		break;
	case 'p':
		ret = ReadGndReport();
		break;
	default:
		break;
	}

	return ret;

error:
	TOUCH_ERR_MSG("[%s] ReadReport fail\n", __func__);
	return -EAGAIN;
}


/* Examples of reading query registers.
 Real applications often do not need to read query registers at all.
 */
void RunQueries(void)
{
	unsigned short cAddr = 0xEE;
	unsigned char cFunc = 0;
	int rxCount = 0;
	int txCount = 0;
	int offset = 0;
	int query_offset = 0;
	int i, j = 0;
#if defined(CONFIG_MACH_MSM8974_VU3_KR)
	int k = 0;
	int cnt = 0;
#endif
	/*RSP Production Test*/
	int tsvd_select;
	int tsvd;
	int tshd;
	int tsstb;
	int tsfrq;
	int tsfst;
	int exvcom_pin_type;
	int exvcom1;
	int exvcom_sel;
	int exvcom2;
	int enable_guard;
	int guard_ring;
	int enable_verf;
	int verf;
	int RspCalibrationStatus;
	int RspCrcStatus;
	int RspCalibrationOffsetLsb;
	int RspCalibrationOffsetMsb;
	int RspCalibrationValue;
	bool HasCtrl102Sub1;
	bool HasCtrl102Sub2;
	bool HasCtrl102Sub4;
	bool HasCtrl102Sub5;
	bool HasCtrl102Sub9;
	bool HasCtrl102Sub10;
	bool HasCtrl102Sub11;
	bool HasCtrl102Sub12;

	/* Scan Page Description Table (PDT)
	   to find all RMI functions presented by this device.
	   The Table starts at $00EE. This and every sixth register
	   (decrementing) is a function number
	   except when this "function number" is $00, meaning end of PDT.
	   In an actual use case this scan might be done only once
	   on first run or before compile.
	   */
	do {
		Read8BitRegisters(cAddr, &cFunc, 1);
		if (cFunc == 0)
			break;

		switch (cFunc) {
		case 0x01:
			if (!bHaveF01) {
				Read8BitRegisters((cAddr - 3),
						&F01ControlBase, 1);
				Read8BitRegisters((cAddr - 4),
						&F01CommandBase, 1);
			}
			break;
#if defined(CONFIG_MACH_MSM8974_VU3_KR)
		case 0x1a:
			if (!bHaveF1A) {
				k = 0;
				Read8BitRegisters((cAddr - 3),
						&F1AControlBase, 1);
				Read8BitRegisters(F1AControlBase + 1,
						&ButtonCount, 1);
				while (ButtonCount) {
					cnt++;
					ButtonCount = (ButtonCount >> 1);
				}
				ButtonCount = cnt;
				for (i = 0; i < ButtonCount; i++) {
					Read8BitRegisters(
						F1AControlBase + 3 + k,
						&ButtonTx[i], 1);
					Read8BitRegisters(
						F1AControlBase + 3 + k + 1,
						&ButtonRx[i], 1);
					k = k + 2;
				}
				bHaveF1A = true;
			}
			break;
#endif
		case 0x12:
			if (!bHaveF12) {
				Read8BitRegisters((cAddr - 3),
						&F12ControlBase, 1);
				Read8BitRegisters((cAddr - 5),
						&F12QueryBase, 1);
				Read8BitRegisters((F12QueryBase),
						&F12Support, 1);

				if ((F12Support | 0x00) == 0) {
					TOUCH_INFO_MSG(
						"Device not support F12.\n");
					break;
				}
				Read8BitRegisters((F12QueryBase + 5),
						Data, 2);
				mask = 0x01;
				for (j = 0; j < 8; j++) {
					if ((Data[1] & mask) == 1)
						offset++;
					Data[1] >>= 1;
				}
				Read8BitRegisters((F12ControlBase + offset),
						Data, 14);
				F12_2DRxCount = Data[12];
				F12_2DTxCount = Data[13];

				if (TRX_max <= F12_2DRxCount)
					F12_2DRxCount = TRX_max;
				if (TRX_max <= F12_2DTxCount)
					F12_2DTxCount = 16;

				offset = 0;
			}
			break;
		case 0x51:
			Read8BitRegisters((cAddr - 2), &F51DataBase, 1);
			Read8BitRegisters((cAddr - 3), &F51ControlBase, 1);
			Read8BitRegisters(F51DataBase, &Data[0], 4);
			/*Reads Calibration status & CRC status */
			RspCalibrationStatus = (Data[0] & 0xf0) >> 4;
			RspCrcStatus = (Data[0] & 0x0f);
			/* Reads Calibration offset */
			RspCalibrationOffsetLsb = Data[1];
			RspCalibrationOffsetMsb = Data[2];
			RspCalibrationValue = Data[3];
			break;
		case 0x54:
			if (!bHaveF54) {
				Read8BitRegisters((cAddr - 2),
						&F54DataBase, 1);
				Read8BitRegisters((cAddr - 3),
						&F54ControlBase, 1);
				Read8BitRegisters((cAddr - 4),
						&F54CommandBase, 1);
				Read8BitRegisters((cAddr - 5),
						&F54QueryBase, 1);
				Read8BitRegisters(F54QueryBase,
						&RxChannelCount, 1);
				Read8BitRegisters((F54QueryBase + 1),
						&TxChannelCount, 1);

				if (TRX_max <= RxChannelCount)
					RxChannelCount = TRX_max;
				if (TRX_max <= TxChannelCount)
					TxChannelCount = TRX_max;

				MaxArrayLength = (int)RxChannelCount
					* (int)TxChannelCount * 2;

				Read8BitRegisters(F54QueryBase,
						Data, 24);
				TouchControllerFamily = Data[5];
				offset++; /*Ctrl 00*/

				if (TouchControllerFamily == 0x0 ||
						TouchControllerFamily == 0x01)
					offset++; /*Ctrl 01*/
				offset += 2; /*Ctrl 02*/
				bHavePixelTouchThresholdTuning =
					((Data[6] & 0x01) == 0x01);

				if (bHavePixelTouchThresholdTuning)
					offset++; /*Ctrl 03;*/

				if (TouchControllerFamily == 0x0 ||
						TouchControllerFamily == 0x01)
					offset += 3; /*Ctrl 04/05/06*/

				if (TouchControllerFamily == 0x01) {
					F54Ctrl07Offset = offset;
					offset++; /*Ctrl 07;*/
					bHaveF54Ctrl07 = true;
				}

				/*Ctrl 08*/
				if (TouchControllerFamily == 0x0 ||
						TouchControllerFamily == 0x01)
					offset += 2;
				/*Ctrl 09*/
				if (TouchControllerFamily == 0x0 ||
						TouchControllerFamily
						== 0x01)
					offset++;
				bHaveInterferenceMetric =
					((Data[7] & 0x02) == 0x02);
				/* Ctrl 10*/
				if (bHaveInterferenceMetric)
					offset++;
				bHaveCtrl11 =
					((Data[7] & 0x10) == 0x10);
				/*Ctrl 11*/
				if (bHaveCtrl11)
					offset += 2;
				bHaveRelaxationControl =
					((Data[7] & 0x80) == 0x80);
				/*Ctrl 12/13*/
				if (bHaveRelaxationControl)
					offset += 2;
				bHaveSensorAssignment =
					((Data[7] & 0x01) == 0x01);
				/*Ctrl 14*/
				if (bHaveSensorAssignment)
					offset++;
				/*Ctrl 15*/
				if (bHaveSensorAssignment)
					offset += RxChannelCount;
				/*Ctrl 16*/
				if (bHaveSensorAssignment)
					offset += TxChannelCount;
				bHaveSenseFrequencyControl =
					((Data[7] & 0x04) == 0x04);
				NumberOfSensingFrequencies =
					(Data[13] & 0x0F);
				/*Ctrl 17/18/19*/
				if (bHaveSenseFrequencyControl)
					offset +=
					(3 * (int)NumberOfSensingFrequencies);
				offset++; /*Ctrl 20*/
				if (bHaveSenseFrequencyControl)
					offset += 2; /*Ctrl 21*/
				bHaveFirmwareNoiseMitigation =
					((Data[7] & 0x08) == 0x08);
				if (bHaveFirmwareNoiseMitigation)
					offset++; /*Ctrl 22*/
				if (bHaveFirmwareNoiseMitigation)
					offset += 2; /*Ctrl 23*/
				if (bHaveFirmwareNoiseMitigation)
					offset += 2; /*Ctrl 24*/
				if (bHaveFirmwareNoiseMitigation)
					offset++; /*Ctrl 25*/
				if (bHaveFirmwareNoiseMitigation)
					offset++; /*Ctrl 26*/
				bHaveIIRFilter = ((Data[9] & 0x02)
						== 0x02);
				if (bHaveIIRFilter)
					offset++; /*Ctrl 27*/
				if (bHaveFirmwareNoiseMitigation)
					offset += 2; /*Ctrl 28*/
				bHaveCmnRemoval = ((Data[9] & 0x04)
						== 0x04);
				bHaveCmnMaximum = ((Data[9] & 0x08)
						== 0x08);
				if (bHaveCmnRemoval)
					offset++; /*Ctrl 29*/
				if (bHaveCmnMaximum)
					offset++; /*Ctrl 30*/
				bHaveTouchHysteresis =
					((Data[9] & 0x10) == 0x10);
				if (bHaveTouchHysteresis)
					offset++; /*Ctrl 31*/
				bHaveEdgeCompensation =
					((Data[9] & 0x20) == 0x20);
				if (bHaveEdgeCompensation)
					offset += 2; /*Ctrl 32*/
				if (bHaveEdgeCompensation)
					offset += 2; /*Ctrl 33*/
				if (bHaveEdgeCompensation)
					offset += 2; /*Ctrl 34*/
				if (bHaveEdgeCompensation)
					offset += 2; /*Ctrl 35*/
				CurveCompensationMode =
					(Data[8] & 0x03);
				if (CurveCompensationMode == 0x02) {
					offset += (int)RxChannelCount;
				} else if (CurveCompensationMode == 0x01) {
					offset +=
					((int)RxChannelCount
					 > (int)TxChannelCount) ?
					(int)RxChannelCount
					: (int)TxChannelCount;
				} /*Ctrl 36*/

				if (CurveCompensationMode == 0x02) {
					/*Ctrl 37*/
					offset += (int)TxChannelCount;
				}

				bHavePerFrequencyNoiseControl =
					((Data[9] & 0x40) == 0x40);

				/*Ctrl 38/39/40*/
				if (bHavePerFrequencyNoiseControl)
					offset +=
					(3 * (int)NumberOfSensingFrequencies);

				bHaveSignalClarity =
					((Data[10] & 0x04) == 0x04);

				if (bHaveSignalClarity) {
					F54Ctrl41Offset = offset;
					offset++; /*Ctrl 41*/
					bHaveF54Ctrl41 = true;
				}

				bHaveMultiMetricStateMachine =
					((Data[10] & 0x02) == 0x02);
				bHaveVarianceMetric =
					((Data[10] & 0x08) == 0x08);
				if (bHaveVarianceMetric)
					offset += 2; /*Ctr 42*/
				if (bHaveMultiMetricStateMachine)
					offset += 2; /*Ctrl 43*/
				/*Ctrl 44/45/46/47/48/49/50/51/52/53/54*/
				if (bHaveMultiMetricStateMachine)
					offset += 11;

				bHave0DRelaxationControl =
					((Data[10] & 0x10) == 0x10);
				bHave0DAcquisitionControl =
					((Data[10] & 0x20) == 0x20);
				if (bHave0DRelaxationControl)
					offset += 2; /*Ctrl 55/56*/
				if (bHave0DAcquisitionControl) {
					F54Ctrl57Offset = offset;
					offset++; /*Ctrl 57;*/
					bHaveF54Ctrl57 = true;
				}
				if (bHave0DAcquisitionControl)
					offset += 1; /*Ctrl 58*/

				bHaveSlewMetric =
					((Data[10] & 0x80) == 0x80);
				bHaveHBlank = ((Data[11] & 0x01) == 0x01);
				bHaveVBlank = ((Data[11] & 0x02) == 0x02);
				bHaveLongHBlank = ((Data[11] & 0x04) == 0x04);
				bHaveNoiseMitigation2 =
					((Data[11] & 0x20) == 0x20);
				bHaveSlewOption = ((Data[12] & 0x02) == 0x02);

				if (bHaveHBlank)
					offset += 1; /*Ctrl 59*/

				if (bHaveHBlank || bHaveVBlank
						|| bHaveLongHBlank)
					offset += 3; /*Ctrl 60/61/62*/

				if (bHaveSlewMetric || bHaveHBlank
						|| bHaveVBlank
						|| bHaveLongHBlank
						|| bHaveNoiseMitigation2
						|| bHaveSlewOption)
					offset += 1; /*Ctrl 63*/

				if (bHaveHBlank)
					offset += 28; /*Ctrl 64/65/66/67*/
				else if (bHaveVBlank || bHaveLongHBlank)
					offset += 4; /*Ctrl 64/65/66/67*/

				if (bHaveHBlank || bHaveVBlank
						|| bHaveLongHBlank)
					offset += 8; /*Ctrl 68/69/70/71/72/73*/

				if (bHaveSlewMetric)
					offset += 2; /*Ctrl 74*/

				bHaveEnhancedStretch =
					((Data[9] & 0x80) == 0x80);
				/*Ctrl 75*/
				if (bHaveEnhancedStretch)
					offset +=
					(int)NumberOfSensingFrequencies;

				bHaveStartupFastRelaxation =
					((Data[11] & 0x08) == 0x08);
				if (bHaveStartupFastRelaxation)
					offset += 1; /*Ctrl 76*/

				bHaveESDControl =
					((Data[11] & 0x10) == 0x10);
				if (bHaveESDControl)
					offset += 2; /*Ctrl 77/78*/

				if (bHaveNoiseMitigation2)
					offset += 5; /*Ctrl 79/80/81/82/83*/

				bHaveEnergyRatioRelaxation =
					((Data[11] & 0x80) == 0x80);
				if (bHaveEnergyRatioRelaxation)
					offset += 2; /*Ctrl 84/85*/

				bHaveF54Query13 = ((Data[12] & 0x08) == 0x08);
				if (bHaveSenseFrequencyControl) {
					query_offset = 13;
					NumberOfSensingFrequencies =
						(Data[13] & 0x0F);
				} else
					query_offset = 12;
				if (bHaveF54Query13)
					query_offset++;
				bHaveCtrl86 = (bHaveF54Query13 &&
						((Data[13] & 0x01) == 0x01));
				bHaveCtrl87 = (bHaveF54Query13 &&
						((Data[13] & 0x02) == 0x02));
				bHaveCtrl88 = ((Data[12] & 0x40) == 0x40);

				if (bHaveCtrl86)
					offset += 1; /*Ctrl 86*/
				if (bHaveCtrl87)
					offset += 1; /*Ctrl 87*/
				if (bHaveCtrl88) {
					F54Ctrl88Offset = offset;
					offset++; /*Ctrl 88;*/
				}
				bHaveCtrl89 = ((Data[query_offset]
							& 0x20) == 0x20);
				if (bHaveCtrl89)
					offset++;
				bHaveF54Query15 = ((Data[12] & 0x80)
						== 0x80);
				if (bHaveF54Query15)
					query_offset++;  /*query_offset = 14*/
				bHaveCtrl90 = (bHaveF54Query15 &&
						((Data[query_offset]
						  & 0x01) == 0x01));
				if (bHaveCtrl90)
					offset++;
				bHaveF54Query16 = ((Data[query_offset]
							& 0x8) == 0x8);
				bHaveF54Query22 = ((Data[query_offset]
							& 0x40) == 0x40);
				bHaveF54Query25 = ((Data[query_offset]
							& 0x80) == 0x80);
				if (bHaveF54Query16)
					query_offset++; /*query_offset = 15*/
				bHaveF54Query17 = ((Data[query_offset]
							& 0x1) == 0x1);
				bHaveCtrl92 = ((Data[query_offset]
							& 0x4) == 0x4);
				bHaveCtrl93 = ((Data[query_offset]
							& 0x8) == 0x8);
				bHaveCtrl94 = ((Data[query_offset]
							& 0x10) == 0x10);
				bHaveF54Query18 = bHaveCtrl94;
				bHaveCtrl95 = ((Data[query_offset]
							& 0x20) == 0x20);
				bHaveF54Query19 = bHaveCtrl95;
				bHaveCtrl99 = ((Data[query_offset]
							& 0x40) == 0x40);
				bHaveCtrl100 = ((Data[query_offset]
							& 0x80) == 0x80);
				if (bHaveF54Query17)
					query_offset++; /*query_offset = 16*/
				if (bHaveF54Query18)
					query_offset++; /*query_offset = 17*/
				if (bHaveF54Query19)
					query_offset++; /*query_offset = 18*/

				/*query 20, 21 query_offset = 20*/
				query_offset = query_offset + 2;
				bHaveCtrl91 = ((Data[query_offset]
							& 0x4) == 0x4);
				bHaveCtrl96  = ((Data[query_offset]
							& 0x8) == 0x8);
				bHaveCtrl97  = ((Data[query_offset]
							& 0x10) == 0x10);
				bHaveCtrl98  = ((Data[query_offset]
							& 0x20) == 0x20);
				if (bHaveF54Query22)
					query_offset++; /*query_offset = 21*/
				bHaveCtrl101 = ((Data[query_offset]
							& 0x2) == 0x2);
				bHaveF54Query23 = ((Data[query_offset]
							& 0x8) == 0x8);
				if (bHaveF54Query23) {
					query_offset++; /*query_offset = 22*/
					bHaveCtrl102 = ((Data[query_offset]
							& 0x01) == 0x01);
				} else
					bHaveCtrl102 = false;
				if (bHaveCtrl91)
					offset++;
				if (bHaveCtrl92)
					offset++;
				if (bHaveCtrl93)
					offset++;
				if (bHaveCtrl94)
					offset++;
				if (bHaveCtrl95)
					offset++;
				if (bHaveCtrl96)
					offset++;
				if (bHaveCtrl97)
					offset++;
				if (bHaveCtrl98) {
					F54Ctrl98Offset = offset;
					offset++;
				}
				if (bHaveCtrl99)
					offset++;
				if (bHaveCtrl100)
					offset++;
				if (bHaveCtrl101)
					offset++;
				/* RSP Product Test */
				if (bHaveCtrl102){
					bIncellDevice = true;
					F54Ctrl102Offset = offset;
					HasCtrl102Sub1 = (bool)(Data[query_offset] & 0x02);
					HasCtrl102Sub2 = (bool)(Data[query_offset] & 0x04);
					HasCtrl102Sub4 = (bool)(Data[query_offset] & 0x08);
					HasCtrl102Sub5 = (bool)(Data[query_offset] & 0x010);
					HasCtrl102Sub9 = (bool)(Data[query_offset] & 0x020);
					HasCtrl102Sub10 = (Data[query_offset] & 0x40);
					HasCtrl102Sub11 =(Data[query_offset] & 0x80);
					HasCtrl102Sub12 = false;
					offset = 0;
					Read8BitRegisters((F54ControlBase + F54Ctrl102Offset), &Data[0], 27);
					tsvd_select = Data[0] & 0x03;
					tsvd = Data[1 + tsvd_select];
					offset = offset + 4;
					tshd = Data[offset];
					if (HasCtrl102Sub1) {
						offset = offset + 2;
						tsstb = Data[offset];
					}
					if (HasCtrl102Sub2) {
						tsfrq = Data[offset + 2];
						tsfst = Data[offset + 3];
						offset = offset + 3;
					}
					/*Ctrl102Sub3*/
					exvcom_pin_type = (Data[offset + 1] & 0x01); /* 0 = GPIO, 1 = TRX */
					exvcom1 = Data[offset + 2];
					offset = offset + 2;
					if (HasCtrl102Sub4) {
						exvcom_sel = (Data[offset + 1] & 0x03);
						exvcom2 = Data[offset +2];
						offset = offset + 4;
					}
					if (HasCtrl102Sub5) {
						enable_guard = (Data[offset + 1] & 0x01);
						guard_ring = Data[offset + 2];
						offset = offset +2;
					}
					/*Ctrl102Sub6, 7, 8*/
					offset = offset + 5;
					if (HasCtrl102Sub9) offset ++;
					if (HasCtrl102Sub10){
						exvcom_sel = Data[offset + 2];
						offset = offset + 2;
					}
					if (HasCtrl102Sub11) offset++;
					if (bHaveF54Query25)
						HasCtrl102Sub12 = (Data[query_offset +1] & 0x02);
					if (HasCtrl102Sub12){
						enable_verf = ((Data[offset +1]) & 0x01);
						verf = (Data[offset + 2]);
					}
				}
				if (bHaveF54Query24) query_offset++;
				query_offset++; /*Query 25*/
				bHaveF54Ctrl106 = ((Data[query_offset] & 0x01) == 0x01);
				bHaveF54Ctrl107 = ((Data[query_offset] & 0x04) == 0x04);
				bHaveF54Ctrl108 = ((Data[query_offset] & 0x08) == 0x08);
				bHaveF54Ctrl109 = ((Data[query_offset] & 0x10) == 0x10);
				bHaveF54Query27 = ((Data[query_offset] & 0x80) == 0x80);
				if (bHaveF54Query26) query_offset++;
				if (bHaveF54Query27) {
					query_offset++;
					bHaveF54Ctrl110 = ((Data[query_offset] & 0x01) == 0x01);
					bHaveF54Ctrl111 = ((Data[query_offset] & 0x04) == 0x04);
					bHaveF54Ctrl112 = ((Data[query_offset] & 0x08) == 0x08);
					bHaveF54Ctrl113 = ((Data[query_offset] & 0x10) == 0x10);
					bHaveF54Ctrl114 = ((Data[query_offset] & 0x40) == 0x40);
					bHaveF54Query29 = ((Data[query_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query28) query_offset++;
				if (bHaveF54Query29) {
					query_offset++;
					bHaveF54Ctrl115 = ((Data[query_offset] & 0x01) == 0x01);
					bHaveF54Ctrl116 = ((Data[query_offset] & 0x10) == 0x10);
					bHaveF54Query30 = ((Data[query_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query30) {
					query_offset++;
					bHaveF54Ctrl119 = ((Data[query_offset] & 0x02) == 0x02);
					bHaveF54Ctrl120 = ((Data[query_offset] & 0x04) == 0x04);
					bHaveF54Ctrl121 = ((Data[query_offset] & 0x08) == 0x08);
					bHaveF54Ctrl122 = ((Data[query_offset] & 0x10) == 0x10);
					bHaveF54Query31 = ((Data[query_offset] & 0x10) == 0x10);
					bHaveF54Ctrl123 = ((Data[query_offset] & 0x40) == 0x40);
					bHaveF54Query32 = ((Data[query_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query31) query_offset++;
				if (bHaveF54Query32) {
					query_offset++;
					bHaveF54Ctrl125 = ((Data[query_offset] & 0x01) == 0x01);
					bHaveF54Ctrl126 = ((Data[query_offset] & 0x02) == 0x02);
					bHaveF54Ctrl127 = ((Data[query_offset] & 0x04) == 0x04);
					bHaveF54Query33 = ((Data[query_offset] & 0x10) == 0x10);
					bHaveF54Query34 = ((Data[query_offset] & 0x40) == 0x40);
					bHaveF54Query35 = ((Data[query_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query33) {
					query_offset++;
					bHaveF54Ctrl132 = ((Data[query_offset] & 0x10) == 0x10);
					bHaveF54Ctrl133 = ((Data[query_offset] & 0x20) == 0x20);
					bHaveF54Ctrl134 = ((Data[query_offset] & 0x40) == 0x40);
					bHaveF54Query36 = ((Data[query_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query34) query_offset++;
				if (bHaveF54Query35) {
					query_offset++;
					bHaveF54Ctrl137 = ((Data[query_offset] & 0x08) == 0x08);
					bHaveF54Ctrl138 = ((Data[query_offset] & 0x10) == 0x10);
					bHaveF54Ctrl139 = ((Data[query_offset] & 0x20) == 0x20);
					bHaveF54Ctrl140 = ((Data[query_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query36) {
					query_offset++;
					bHaveF54Ctrl142 = ((Data[query_offset] & 0x02) == 0x02);
					bHaveF54Query37 = ((Data[query_offset] & 0x04) == 0x04);
					bHaveF54Ctrl143 = ((Data[query_offset] & 0x08) == 0x08);
					bHaveF54Ctrl144 = ((Data[query_offset] & 0x10) == 0x10);
					bHaveF54Ctrl145 = ((Data[query_offset] & 0x20) == 0x20);
					bHaveF54Ctrl146 = ((Data[query_offset] & 0x40) == 0x40);
					bHaveF54Query38 = ((Data[query_offset] & 0x80) == 0x80);
				}
				if (bHaveF54Query37) query_offset++;
				if (bHaveF54Query38) {
					query_offset++;
					bHaveF54Ctrl147 = ((Data[query_offset] & 0x01) == 0x01);
					bHaveF54Ctrl148 = ((Data[query_offset] & 0x02) == 0x02);
					bHaveF54Ctrl149 = ((Data[query_offset] & 0x04) == 0x04);
				}
				/*-----------------------------------------------------------from Ctrl 103*/
				bHaveF54Ctrl117 = true; /*Reserved*/
				bHaveF54Ctrl118 = false; /*Reserved*/
				bHaveF54Ctrl124 = false; /*Reserved*/
				bHaveF54Ctrl128 = false; /*Reserved*/
				bHaveF54Ctrl129 = false; /*Reserved*/
				bHaveF54Ctrl130 = false; /*Reserved*/
				bHaveF54Ctrl131 = false; /*Reserved*/
				bHaveF54Ctrl135 = false; /*Reserved*/
				bHaveF54Ctrl136 = false; /*Reserved*/
				bHaveF54Ctrl141 = false; /*Reserved*/
				offset = offset - 1;
				if (bHaveF54Ctrl103) offset++;
				if (bHaveF54Ctrl104) offset++;
				if (bHaveF54Ctrl105) offset++;
				if (bHaveF54Ctrl106) offset++;
				if (bHaveF54Ctrl107) offset++;
				if (bHaveF54Ctrl108) offset++;
				if (bHaveF54Ctrl109) offset++;
				if (bHaveF54Ctrl110) offset++;
				if (bHaveF54Ctrl111) offset++;
				if (bHaveF54Ctrl112) offset++;
				if (bHaveF54Ctrl113) offset++;
				if (bHaveF54Ctrl114) offset++;
				if (bHaveF54Ctrl115) offset++;
				if (bHaveF54Ctrl116) offset++;
				if (bHaveF54Ctrl117) offset++;
				if (bHaveF54Ctrl118) offset++;
				if (bHaveF54Ctrl119) offset++;
				if (bHaveF54Ctrl120) offset++;
				if (bHaveF54Ctrl121) offset++;
				if (bHaveF54Ctrl122) offset++;
				if (bHaveF54Ctrl123) offset++;
				if (bHaveF54Ctrl124) offset++;
				if (bHaveF54Ctrl125) offset++;
				if (bHaveF54Ctrl126) offset++;
				if (bHaveF54Ctrl127) offset++;
				if (bHaveF54Ctrl128) offset++;
				if (bHaveF54Ctrl129) offset++;
				if (bHaveF54Ctrl130) offset++;
				if (bHaveF54Ctrl131) offset++;
				if (bHaveF54Ctrl132) offset++;
				if (bHaveF54Ctrl133) offset++;
				if (bHaveF54Ctrl134) offset++;
				if (bHaveF54Ctrl135) offset++;
				if (bHaveF54Ctrl136) offset++;
				if (bHaveF54Ctrl137) offset++;
				if (bHaveF54Ctrl138) offset++;
				if (bHaveF54Ctrl139) offset++;
				if (bHaveF54Ctrl140) offset++;
				if (bHaveF54Ctrl141) offset++;
				if (bHaveF54Ctrl142) offset++;
				if (bHaveF54Ctrl143) offset++;
				if (bHaveF54Ctrl144) offset++;
				if (bHaveF54Ctrl145) offset++;
				if (bHaveF54Ctrl146) offset++;
				if (bHaveF54Ctrl147) offset++;
				if (bHaveF54Ctrl148) offset++;
				if (bHaveF54Ctrl149){
					offset++;
					F54Ctrl149Offset = offset;
				}
			}
			break;
		case 0x55:
			if (!bHaveF55) {
				Read8BitRegisters((cAddr - 3),
						&F55ControlBase, 1);
				Read8BitRegisters((cAddr - 5),
						&F55QueryBase, 1);

				Read8BitRegisters(F55QueryBase,
						&RxChannelCount, 1);
				Read8BitRegisters((F55QueryBase+1),
						&TxChannelCount, 1);

				rxCount = 0;
				txCount = 0;
				/*Read Sensor Mapping*/
				Read8BitRegisters((F55ControlBase + 1), Data,
						(int)RxChannelCount);
				for (i = 0; i < (int)RxChannelCount; i++) {
					if (Data[i] != 0xFF) {
						rxCount++;
						TRxPhysical[i] = Data[i];
					} else
						break;
				}
				Read8BitRegisters((F55ControlBase + 2), Data,
						(int)TxChannelCount);
				for (i = 0; i < (int)TxChannelCount; i++) {
					if (Data[i] != 0xFF) {
						TRxPhysical[rxCount + i]
							= Data[i];
						txCount++;
					} else
						break;
				}
				for (i = (rxCount + txCount);
						i < (TRX_mapping_max); i++) {
					TRxPhysical[i] = 0xFF;
				}

				RxChannelCount = rxCount;
				TxChannelCount = txCount;
				if (TRX_max <= RxChannelCount)
					RxChannelCount = TRX_max;
				if (TRX_max <= TxChannelCount)
					TxChannelCount = TRX_max;

				MaxArrayLength = (int)RxChannelCount
					* (int)TxChannelCount * 2;
				if (((int)TxChannelCount - F12_2DTxCount == 0)
						&& ButtonCount > 0) {
					ButtonShared = true;
				}

			}
			break;
		default: /* Any other function*/
			break;
		}
		cAddr -= 6;
	} while (true);
}
/*
The following function is necessary to setup the Function $54 tests.i
The setup needs to be done once
after entering into page 0x01. As long as the touch controller stays in page1
the setup does not
need to be repeated.
*/
bool TestPreparation(void)
{
	unsigned char data = 0;
	unsigned char addr = 0;

	/* Turn off CBC.*/
	if (bHaveF54Ctrl07) {
		addr = F54ControlBase + F54Ctrl07Offset;
		Read8BitRegisters(addr, &data, 1);
		/* data = data & 0xEF;*/
		data = 0;
		Write8BitRegisters(addr, &data, 1);
	} else if (bHaveCtrl88) {
		addr = F54ControlBase + F54Ctrl88Offset;
		Read8BitRegisters(addr, &data, 1);
		data = data & 0xDF;
		Write8BitRegisters(addr, &data, 1);
	}

	/* Turn off 0D CBC.*/
	if (bHaveF54Ctrl57) {
		addr = F54ControlBase + F54Ctrl57Offset;
		Read8BitRegisters(addr, &data, 1);
		/*ata = data & 0xEF;*/
		data = 0;
		Write8BitRegisters(addr, &data, 1);
	}

	/* Turn off SignalClarity.
	   ForceUpdate is required for the change to be effective
	   */
	if (bHaveF54Ctrl41) {
		addr = F54ControlBase + F54Ctrl41Offset;
		Read8BitRegisters(addr, &data, 1);
		data = data | 0x01;
		Write8BitRegisters(addr, &data, 1);
	}

	/* Apply ForceUpdate.*/
	Read8BitRegisters(F54CommandBase, &data, 1);
	data = data | 0x04;
	Write8BitRegisters(F54CommandBase, &data, 1);
	/* Wait complete*/
	count = 0;
	do {
		Read8BitRegisters(F54CommandBase, &data, 1);
		msleep(1);
		count++;
	} while (data != 0x00 && (count < DefaultTimeout));

	if (count >= DefaultTimeout) {
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
				"Timeout -- ForceUpdate can not complete\n");
		TOUCH_INFO_MSG("Timeout -- ForceUpdate can not complete\n");
		Reset();
		return -EAGAIN;
	}

	/* Apply ForceCal.*/
	Read8BitRegisters(F54CommandBase, &data, 1);
	data = data | 0x02;
	Write8BitRegisters(F54CommandBase, &data, 1);

	/* Wait complete*/
	count = 0;
	do {
		Read8BitRegisters(F54CommandBase, &data, 1);
		msleep(1);
		count++;
	} while (data != 0x00 && (count < DefaultTimeout));

	if (count >= DefaultTimeout) {
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
				"Timeout -- ForceCal can not complete\n");
		TOUCH_INFO_MSG("Timeout -- ForceUpdate can not complete\n");
		Reset();
		return -EAGAIN;
	}

	return true;
}
bool RspTestPreparation(void)
{
	unsigned char data = 0;
	unsigned char addr = 0;
	unsigned char count = 0;

	switchPage(0x01);
	msleep(10);

	/* Turn off CBC.*/
	if (bHaveF54Ctrl07) {
		addr = F54ControlBase + F54Ctrl07Offset;
		Read8BitRegisters(addr, &data, 1);
		/* data = data & 0xEF;*/
		data = 0;
		Write8BitRegisters(addr, &data, 1);
	} else if (bHaveCtrl88) {
		addr = F54ControlBase + F54Ctrl88Offset;
		Read8BitRegisters(addr, &data, 1);
		data = data & 0xDF;
		Write8BitRegisters(addr, &data, 1);
	}
	/* Turn off CBC2 */
	if (bHaveF54Ctrl149)
	{
		addr = F54ControlBase + F54Ctrl149Offset;
		Read8BitRegisters(addr, &data, 1);
		data = data & 0xFE;
		Write8BitRegisters(addr, &data, 1);
	}
	/* Turn off 0D CBC.*/
	if (bHaveF54Ctrl57) {
		addr = F54ControlBase + F54Ctrl57Offset;
		Read8BitRegisters(addr, &data, 1);
		/*data = data & 0xEF;*/
		data = 0;
		Write8BitRegisters(addr, &data, 1);
	}

	/* Turn off SignalClarity.
	   ForceUpdate is required for the change to be effective */
	if (bHaveSignalClarity){
		addr = F54ControlBase + F54Ctrl41Offset;
		Read8BitRegisters(addr, &data, 1);
		data = data | 0x01;
		Write8BitRegisters(addr, &data, 1);
	}

	/* Apply ForceUpdate.*/
	Read8BitRegisters(F54CommandBase, &data, 1);
	data = data | 0x04;
	Write8BitRegisters(F54CommandBase, &data, 1);

	/* Wait complete*/
	do {
		Read8BitRegisters(F54CommandBase, &data, 1);
		msleep(1);
		count++;
	} while (data != 0x00 && (count < DefaultTimeout));
	if(count >= DefaultTimeout){
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
				"Timeout -- ForceUpdate can not complete\n");
		TOUCH_INFO_MSG("Timeout -- ForceUpdate can not complete\n");
		Reset();
		return -EAGAIN;
	}

	/* Apply ForceCal.*/
	Read8BitRegisters(F54CommandBase, &data, 1);
	data = data | 0x02;
	Write8BitRegisters(F54CommandBase, &data, 1);

	/* Wait complete*/
	count = 0;
	do {
		Read8BitRegisters(F54CommandBase, &data, 1);
		msleep(1);
		count++;
	} while (data != 0x00 && (count < DefaultTimeout));
	if(count >= DefaultTimeout){
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
				"Timeout -- ForceCal can not complete\n");
		TOUCH_INFO_MSG("Timeout -- ForceUpdate can not complete\n");
		Reset();
		return -EAGAIN;
	}

	return true;
}

int diffnode(unsigned short *ImagepTest)
{

	int i = 0;
	int k = 0;
	unsigned char data;

	if (!bHaveF54) {
		TOUCH_INFO_MSG("not bHaveF54\n");
		return -EINVAL;
	}
	if (!switchPage(0x01)) {
		TOUCH_INFO_MSG("not switchPage(0x01)\n");
		return -EINVAL;
	}

	if (TestPreparation()) {

		/*memcpy(LowerImageLimit, LowerImage, sizeof(LowerImageLimit));
		  memcpy(UpperImageLimit, UpperImage, sizeof(UpperImageLimit));
		  */
		data = 20;/*rawdata mode*/
		Write8BitRegisters(F54DataBase, &data, 1);
		data = 0x01;
		Write8BitRegisters(F54CommandBase, &data, 1);
		count = 0;
		do {
			Read8BitRegisters(F54CommandBase, &data, 1);
			msleep(20);
			count++;
		} while (data != 0x00 && (count < DefaultTimeout));
		if (count >= DefaultTimeout) {
			TOUCH_INFO_MSG("Timeout -- Not supported Report Type in FW\n");
			Reset();
			return -EAGAIN;
		}

		Read8BitRegisters((F54DataBase+REPORT_DATA_OFFEST), &Data[0],
				MaxArrayLength);

		for (i = 0; i < (int)TxChannelCount * (int)RxChannelCount;
				i++) {
			ImagepTest[i] = ((short)Data[k]
					| (short)Data[k + 1] << 8);
			k = k + 2;
		}
		/*Reset Device*/
		Reset();
		TOUCH_INFO_MSG("diff_node success\n");
		return 0;

	} else {
		return -ECANCELED;
	}

}

/*
The following funtion illustrates the steps in getting
a full raw image report (report #20) by Function $54.
*/
int ImageTest(int mode, char *buf)
{
	unsigned char data;
	int ret = 0;

	ret = TestPreparation();

	if (ret == 1) {
		memcpy(LowerImageLimit, LowerImage, sizeof(LowerImageLimit));
		memcpy(UpperImageLimit, UpperImage, sizeof(UpperImageLimit));

		/* Assign report type for Full Raw Image*/

		data = 20;/*raw capacitance mode*/

		Write8BitRegisters(F54DataBase, &data, 1);

		do_gettimeofday(&t_interval[STARTTIME]);

		if (mode == 0)
			data = 'a';/*rawdata store mode*/
		else
			data = 'l';/*rawdata display mode*/

		ret = ReadReport(data, buf);

	}
	return ret;
}

int DeltaTest(char *buf)
{
	unsigned char data;

	memcpy(LowerImageLimit, LowerImage, sizeof(LowerImageLimit));
	memcpy(UpperImageLimit, UpperImage, sizeof(UpperImageLimit));

	/* Assign report type for Full Raw Image*/
	data = 0x02;/*delta mode*/

	Write8BitRegisters(F54DataBase, &data, 1);

	do_gettimeofday(&t_interval[STARTTIME]);

	data = 'l';/*rawdata display mode*/

	return ReadReport(data, buf);

}


int NoiseDeltaTest(char *buf)
{
	unsigned char data;

	memset(NoiseDeltaMin, 0, TRX_MAX * TRX_MAX * sizeof(short));
	memset(NoiseDeltaMax, 0, TRX_MAX * TRX_MAX * sizeof(short));

	/* Assign report type for Full Raw Image*/
	data = 0x02;/*delta mode*/

	Write8BitRegisters(F54DataBase, &data, 1);

	data = 'm';/*rawdata display modie*/

	return ReadReport(data, buf);

}

int GndTest(char *buf)
{
	unsigned char data;

	data = 0x02;/*delta mode*/

	Write8BitRegisters(F54DataBase, &data, 1);

	data = 'p';

	return ReadReport(data, buf);

}
/*
 * The following funtion illustrates the steps in getting
 a sensor speed test report (report #22) by Function $54.
 */
int SensorSpeed(char *buf)
{
	unsigned char data;

	memcpy(SensorSpeedLowerImageLimit, SensorSpeedLowerImage,
			sizeof(SensorSpeedLowerImageLimit));
	memcpy(SensorSpeedUpperImageLimit, SensorSpeedUpperImage,
			sizeof(SensorSpeedUpperImageLimit));

	/* Assign report type for Sensor Speed Test*/
	data = 22;
	Write8BitRegisters(F54DataBase, &data, 1);

	do_gettimeofday(&t_interval[STARTTIME]);

	data = 'c';
	return ReadReport(data, buf);

}

/* The following funtion illustrates
 the steps in getting a ADC Range report (report #23) by Function $54.
 */
int ADCRange(char *buf)
{
	unsigned char data = 0;

	memcpy(ADCLowerImageLimit, ADCLowerImage, sizeof(ADCLowerImageLimit));
	memcpy(ADCUpperImageLimit, ADCUpperImage, sizeof(ADCUpperImageLimit));
	Read8BitRegisters((F54ControlBase + F54Ctrl41Offset), &data, 1);
	if (data & 0x01)
		SignalClarityOn = false;
	else
		SignalClarityOn = true;

	/* Assign report type for ADC Range report
	*/
	data = 23;
	Write8BitRegisters(F54DataBase, &data, 1);

	do_gettimeofday(&t_interval[STARTTIME]);

	data = 'b';
	return  ReadReport(data, buf);
}

void AbsADCRange(char *buf)
{
	unsigned char data;

	if (TestPreparation()) {
		/* Assign report type for Abs Sensing ADC Range report
		 */
		data = 42;
		Write8BitRegisters(F54DataBase, &data, 1);

		do_gettimeofday(&t_interval[STARTTIME]);

		data = 'i';
		ReadReport(data, buf);
	}
}
/* report type 40
 */
int AbsDelta(char *buf)
{
	unsigned char data;


	/* Assign report type for Abs Sensing Delta Capacitance report
	 */
	data = 40;
	Write8BitRegisters(F54DataBase, &data, 1);

	do_gettimeofday(&t_interval[STARTTIME]);
	/*startTime = GetTickCount();
	 */

	data = 'j';
	return ReadReport(data, buf);
}
/* report type 38
 */
int AbsRaw(int mode, char *buf)
{
	unsigned char data;

	if (bHaveCtrl98) {
		Read8BitRegisters((F54ControlBase+F54Ctrl98Offset),
				&Data[0], 6);

		/* AbsRx Low Reference
		*/
		AbsRxShortLimit = AbsRawRef[Data[0]] * 275/100;

		/* AbsTx Low Reference
		 */
		AbsTxShortLimit = AbsRawRef[Data[5]] * 275/100;

		/* AbsTx Low Reference
		 */
		AbsTxOpenLimit =  AbsRawRef[Data[5]] * 75/100;
	}

	data = 38;
	Write8BitRegisters(F54DataBase, &data, 1);

	do_gettimeofday(&t_interval[STARTTIME]);

	if (mode == 1)
		data = 'n';/*Abs Sensing Short Test mode*/
	else if (mode == 2)
		data = 'o';/*Abs Sensing Open Test mode*/
	else
		data = 'k';/*Abs Sensing Raw Test mode*/

	return ReadReport(data, buf);
}

/* The following funtion illustrates the steps
 in getting a TRex-Opens(No sensor) report (report #24) by Function $54.
 */
void TRexOpenTest(char *buf)
{
	unsigned char data;


	if (TestPreparation()) {
		/* Assign report type for TRex Open Test*/
		data = 24;
		Write8BitRegisters(F54DataBase, &data, 1);

		data = 'd';
		ReadReport(data, buf);
	}
}

/*The following funtion illustrates the steps
in getting a TRex-to-GND(No sensor) report (report #25) by Function $54.
*/
int TRexGroundTest(char *buf)
{
	unsigned char data;

	if (TestPreparation()) {
		/* Assign report type for TRex Ground Test*/
		data = 25;
		Write8BitRegisters(F54DataBase, &data, 1);

		data = 'e';
		return ReadReport(data, buf);
	} else {
		return -ECANCELED;
	}
}

/* The following funtion illustrates the steps
 in getting a TRex-TRex short(No sensor) report (report #26) by Function $54.
 */
int TRexShortTest(char *buf)
{
	unsigned char data;
	int ret;

	ret = TestPreparation();

	if (ret == 1) {
		/* Assign report type for TRex Short Test*/
		data = 26;
		Write8BitRegisters(F54DataBase, &data, 1);
		data = 'f';
		ret = ReadReport(data, buf);
	}

	return ret;
}

/* This test is to retreive the high resistance report, report type #4.
 */
int HighResistanceTest(char *buf)
{
	unsigned char data;
	int ret = 0;

	ret = TestPreparation();
	if (ret == 1) {

		/* Assign report type for High Resistance report*/
		data = 4;
		Write8BitRegisters(F54DataBase, &data, 1);

		data = 'g';
		ret = ReadReport(data, buf);
	}

	return ret;
}

/* This test is to retreive the maximum and minimum pixel report,
 report type #13.
*/
void MaxMinTest(char *buf)
{
	unsigned char data;

	if (TestPreparation()) {
		/* Assign report type for Max Min report
		 */
		data = 13;
		Write8BitRegisters(F54DataBase, &data, 1);

		data = 'h';
		ReadReport(data, buf);
	}
}
int RspCalibrationTest(void)
{
	bool isPassed = true;
	bool isCompleted = false;
	unsigned char buffer[4]={0,0,0,0};
	unsigned char data = 0;
	int RspCalibrationStatus = 0;
	int RspCrcStatus = 0;

	Read8BitRegisters(F51ControlBase, &buffer[0], 1);
	RspCalibrationStatus = (buffer[0] & 0xf0) >> 4;
	RspCrcStatus = (buffer[0] & 0x0f);

	if (RspCalibrationStatus == 1)
	{
		TOUCH_INFO_MSG("RSP Calibration is in progress.  Try again later.");
		isPassed = false;
	}
	else
	{
		/* For production test, we will perform the calibration everytime
		 Set bit 0 of F51_CUSTOM_CTRL15 to start the calibration */
		Read8BitRegisters(F51ControlBase, &data, 1);
		data = data | 0x01;
		Write8BitRegisters(F51ControlBase, &data, 1);
		msleep(1000);
		Read8BitRegisters(F51ControlBase, &data, 1);

		TOUCH_INFO_MSG("RSP Calibration is in progress...");
		while (!isCompleted)
		{
			Read8BitRegisters(F51DataBase, &buffer[0], 1);
			RspCalibrationStatus = (buffer[0] & 0xf0) >> 4;
			RspCrcStatus = (buffer[0] & 0x0f);
			if (RspCalibrationStatus == 1) /* 0 - idle and 1 - calibration is in progress */
			{
				msleep(1000);
			}
			else if (RspCalibrationStatus == 0)
			{
				isCompleted = true;
				if (RspCrcStatus == 6)
				{
					isPassed = false;
				}
			}
		}
		if (RspTestPreparation())
		{
			RspReadReport(eRT_RSPGroupTest);
		}
		TOUCH_INFO_MSG("\nRSP Calibration Test - Completed\n");
	}
	return (isPassed)? 1:0;
}

int RspRawDataTest(void)
{
	bool isPassed = true;
	int f = 0;
	int c = 0;
	int r = 0;
	TOUCH_INFO_MSG("%s\n", __func__);
	memset(RspAvgImage,0,sizeof(RspAvgImage));

	/* Sum up all 50 frames for each pixel */
	for (f = 0; f < RSP_COLLECT_FRAMES; f++)
	{
		for (r = 0; r < RSP_MAX_ROW; r++)
		{
			for (c = 0; c < RSP_MAX_COL; c++)
			{
				RspAvgImage[r][c] += abs(RspImageStack[f][r][c]);
			}
		}
	}

	/* Divid each pixel by 50 for average & save average raw data*/
	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf, "RSP Full Raw Capacitance Image Data :\n");
	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf, "\nInfo: RSP Row = %d Col = %d\n",
			(int)TxChannelCount, (int)RxChannelCount);
	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
			"==========================================================================================================\n		  :");

	for (r = 0; r < (int)RxChannelCount; r++)
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf, "%5d ", r);
	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
			"\n----------------------------------------------------------------------------------------------------------\n");

	for (r = 0; r < RSP_MAX_ROW; r++) {
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf, "	 %5d : ", r);
		for (c = 0; c < RSP_MAX_COL; c++)
		{
			RspAvgImage[r][c] /= RSP_COLLECT_FRAMES;
			outbuf += snprintf(f54_wlog_buf + outbuf, sizeof(f54_wlog_buf)-outbuf, "%5d ",
					RspAvgImage[r][c]);
		}
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf, "\n");
	}

	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
				"------------------------------------------------------------------------------------------------------------\n");

	/* Compare the limits */
	for (r = 0; r < RSP_MAX_ROW; r++)
	{
		for (c = 0; c < RSP_MAX_COL; c++)
		{
			if (RspAvgImage[r][c] < LowerImageLimit[r][c] || RspAvgImage[r][c] > UpperImageLimit[r][c])
			{
				TOUCH_INFO_MSG("Failed 2D area : Row [%2d] Col [%2d] Value : %3d\n",r, c, RspAvgImage[r][c]);
				outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
									"Failed 2D area : Row [%2d] Col [%2d] Value : %3d\n",
									r, c, RspAvgImage[r][c]);
				isPassed = false;
			}
		}
	}

	if (isPassed){
		TOUCH_INFO_MSG("RSP Full Raw Capacitance Image Test passed.");
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
				"RSP Full Raw Capacitance Image Test passed.\n\n\n");
	} else {
		TOUCH_INFO_MSG("RSP Full Raw Capacitance Image Test failed.\n");
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
				"RSP Full Raw Capacitance Image Test failed.\n\n\n");
	}

	TOUCH_INFO_MSG("RSP Raw Data Test - Completed\n");
	return (isPassed)? 1:0;
}
int RspRawSlopeTest(void)
{
	bool isPassed = true;
	int r = 0;
	int c = 0;
	TOUCH_INFO_MSG("%s\n", __func__);
	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
			"RSP Raw Slope Test\n");

	/* Cs_slope=Cs_abs(c,r)/AVG{Cs_abs(c,r-1), Cs_abs(c,r+1)}*100 */
    for (r = 1; r < RSP_MAX_ROW - 1; r++) /* Exclude the first and last row */
    {
        for (c = 0; c < RSP_MAX_COL; c++)
        {
            RspSlopeImage[r][c] = abs(RspAvgImage[r][c])*100/
                                ((abs(RspAvgImage[r - 1][c]) + abs(RspAvgImage[r + 1][c]))/2);
        }
    }

	/* Compare the limits */
	for (r = 1; r < RSP_MAX_ROW-1; r++)
	{
		for (c = 0; c < RSP_MAX_COL; c++)
		{
			if (RspSlopeImage[r][c] < RspSlopeDataMin[r][c] || RspSlopeImage[r][c] > RspSlopeDataMax[r][c])
			{
				TOUCH_INFO_MSG("Failed 2D area : Row [%2d] Col [%2d] Value : %3d\n", r, c, RspSlopeImage[r][c]);
				outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
					"Failed 2D area : Row [%2d] Col [%2d] Value : %3d\n", r, c, RspSlopeImage[r][c]);
				isPassed = false;
			}
		}
	}
	if (isPassed){
		TOUCH_INFO_MSG("RSP Raw Slope Test Passed");
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
				"\nRSP Raw Slope Test passed.\n\n\n");
	} else {
		TOUCH_INFO_MSG("RSP Raw Slope Test Failed.\n");
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
				"\nRSP Raw Slope Test Failed.\n\n\n");
	}
	write_log(NULL, f54_wlog_buf);
	TOUCH_INFO_MSG ("RSP Raw Slope Test - Completed\n");
	return (isPassed)? 1:0;
}
int GetP2PMin(int row, int col)
{
	int min = 0;
	int f = 0;
	min = RspImageStack[0][row][col];
	for (f = 1; f < RSP_COLLECT_FRAMES; f++)
	{
		if (RspImageStack[f][row][col] < min)
		{
			min = abs(RspImageStack[f][row][col]);
		}
	}

	return min;
}

int GetP2PMax(int row, int col)
{
	int max = 0;
	int f = 0;
	max = RspImageStack[0][row][col];
	for (f = 1; f < RSP_COLLECT_FRAMES; f++)
	{
		if (RspImageStack[f][row][col] > max)
		{
			max = abs (RspImageStack[f][row][col]);
		}
	}

	return max;
}

int RspNoiseP2PTest(void)
{
	bool isPassed = true;
	int r = 0;
	int c = 0;
	int diff =0;
	TOUCH_INFO_MSG("%s\n", __func__);

	/* Loops through all pixel and applies the calc. through all frames
	     Cs_pp=Max(Cs_abs(c,r)) - Min(Cs_abs(c,r))
	     where c=0, 1 ... 17 & r=1, 2 ... 31 */
	out_buf += snprintf(wlog_buf+out_buf, sizeof(wlog_buf)-out_buf, "\nRSP Noise P-P Test Data :\n");
	out_buf += snprintf(wlog_buf+out_buf, sizeof(wlog_buf)-out_buf,
		"==========================================================================================================\n         :");
	for (r = 0; r < (int)RxChannelCount; r++)
		out_buf += snprintf(wlog_buf+out_buf, sizeof(wlog_buf)-out_buf, "%5d ", r);
	out_buf += snprintf(wlog_buf+out_buf, sizeof(wlog_buf)-out_buf,
			"\n----------------------------------------------------------------------------------------------------------\n");

	for (r = 0; r < RSP_MAX_ROW; r++)
	{
		out_buf += snprintf(wlog_buf+out_buf, sizeof(wlog_buf)-out_buf, "   %5d : ", r);
		for (c = 0; c < RSP_MAX_COL; c++)
		{
			diff = GetP2PMax(r, c) - GetP2PMin(r, c);
			ImagepF[r][c] = diff;
			out_buf += snprintf(wlog_buf+out_buf,
					sizeof(wlog_buf)-out_buf, "%5d ", ImagepF[r][c]);
		}
		out_buf += snprintf(wlog_buf+out_buf, sizeof(wlog_buf)-out_buf, "\n");
	}
	out_buf += snprintf(wlog_buf+out_buf, sizeof(wlog_buf)-out_buf,
			"------------------------------------------------------------------------------------------------------------\n");

	for (r = 0; r < RSP_MAX_ROW; r++)
	{
		for (c = 0; c < RSP_MAX_COL; c++)
		{
			if (ImagepF[r][c] > RspNoiseP2PLimit[r][c])
			{
				TOUCH_INFO_MSG("Failed: 2D area: Row [%2d] Col [%2d] : %2d\n",
									r, c, ImagepF[r][c]);
				out_buf += snprintf(wlog_buf+out_buf, sizeof(wlog_buf)-out_buf,
									"Failed 2D area : Row [%2d] Col [%2d] Value : %3d\n",
									r, c, ImagepF[r][c]);
				isPassed = false;
			}
		}
	}

	if (isPassed) {
		TOUCH_INFO_MSG("RSP Noise P-P Test Passed.");
		out_buf += snprintf(wlog_buf+out_buf, sizeof(wlog_buf)-out_buf,
				"RSP Noise P-P Test Passed.\n\n\n");
	} else {
		TOUCH_INFO_MSG("RSP Noise P-P Test Failed.\n");
		out_buf += snprintf(wlog_buf+out_buf, sizeof(wlog_buf)-out_buf,
				"RSP Noise P-P Test Failed.\n\n\n");
	}

	write_log(NULL, wlog_buf);
	msleep(50);
	TOUCH_INFO_MSG ("RSP Raw Noise P-P Test - Completed\n");
	return (isPassed)? 1:0;
}

int RspShortTest(void)
{
	bool isPassed = true;
	int f = 0;
	int k = 0;
	int c = 0;
	int r = 0;
	int i = 0;

	TOUCH_INFO_MSG("%s\n", __func__);

	/* Step 1 - Convert the RspImageStack into 50 frames of 1D arrays */
	for (f = 0; f < RSP_COLLECT_FRAMES; f++)
	{
		k = 0;
		for (c = RSP_MAX_COL - 1; c >= 0; c--)
		{
			for (r = RSP_MAX_ROW - 1; r >= 0; r--)
			{
				oneDArray[f][k++] = RspImageStack[f][r][c];
			}
		}
	}

	/* Step 2 - Compare the oneDArray[i] vs. oneDArray[i+1].
	 If the two pixels have the same value then shortArray[i] += 1; */
	/* Step 3 - Repeat step 2 for all 50 frames */
	for (f = 0; f < RSP_COLLECT_FRAMES; f++)
	{
		for (i = 1; i < RSP_MAX_PIXELS; i++)
		{
			if (oneDArray[f][i-1] == oneDArray[f][i])
			{
				shortArray[i-1] += 1;
			}
		}
	}

	/* Step 4 - Convert the oneDArray back to twoDShortArray */
	for (k = 0; k < RSP_MAX_PIXELS; k++)
	{
		for (c = RSP_MAX_COL - 1; c >= 0; c--)
		{
			for (r = RSP_MAX_ROW - 1; r >= 0; r--)
			{
				twoDShortArray[r][c] = shortArray[k];
			}
		}
	}

	/* Step 5 - Compare each pixel from the twoDArray with the limit */
	for (c = RSP_MAX_COL - 1; c >= 0; c--)
	{
		for (r = RSP_MAX_ROW - 1; r >= 0; r--)
		{
			if (twoDShortArray[r][c] > RspShortLimit)
			{
				TOUCH_INFO_MSG("Failed 2D area : Row [%2d] Col [%2d] Value : %3d\n",
									r, c, twoDShortArray[r][c]);
				out_buf += snprintf(wlog_buf+out_buf, sizeof(wlog_buf)-out_buf,
									"Failed 2D area : Row [%2d] Col [%2d] Value : %3d\n",
									r, c, twoDShortArray[r][c]);
				isPassed = false;
			}
		}
	}
	if (isPassed) {
		TOUCH_INFO_MSG("RSP Short Test Passed");
		out_buf += snprintf(wlog_buf+out_buf, sizeof(wlog_buf)-out_buf,
				"\nRSP Short Test passed.\n\n\n");
	} else {
		TOUCH_INFO_MSG("RSP Short Test Result - Short Limit Value : %d\n", RspShortLimit);
		TOUCH_INFO_MSG("RSP Short Test Failed\n");
		out_buf += snprintf(wlog_buf+out_buf, sizeof(wlog_buf)-out_buf,
				"\nRSP Short Test Result - Short Limit Value : %d\n", RspShortLimit);
		out_buf += snprintf(wlog_buf+out_buf, sizeof(wlog_buf)-out_buf,
				"\nRSP Short Test Failed.\n\n\n");
	}
	write_log(NULL, f54_wlog_buf);
	write_log(NULL, wlog_buf);
	msleep(50);
	TOUCH_INFO_MSG("RSP Short Test - Completed\n");
	return (isPassed)? 1:0;
}
int RspGroupTest(void)
{
	int result = 0;
	TOUCH_INFO_MSG("%s\n", __func__);
	result += RspRawDataTest();
	result += RspRawSlopeTest();
	result += RspNoiseP2PTest();
	if (result == 3)
		return 1;
	else
		return 0;
}
void RspSetFreq(int i)
{
	TOUCH_INFO_MSG("%s\n", __func__);
	switch(i)
	{
		case 0: /* f0 */
			break;
		case 1: /* f1 */
			break;
		case 2: /* f2 */
			break;
		case 3: /* LPWG */
			break;
	}
}
void RspCollectRawData(void)
{
	int i = 0;
	int j = 0;
	int k = 0;
	unsigned char data = 0;
	TOUCH_INFO_MSG("%s\n", __func__);
	for (i = 0; i < RSP_COLLECT_FRAMES; i++)
	{
		if(RspTestPreparation())
		{
			data = 20;
			Write8BitRegisters(F54DataBase, &data, 1);
			RspReadReport(eRT_RSPGroupTest);
		}
		for (j = 0; j < RSP_MAX_ROW; j++)
		{
			for (k = 0; k < RSP_MAX_COL; k++)
			{
				RspImageStack[i][j][k] = RspRawImage[j][k];
			}
		}
	}

	TOUCH_INFO_MSG("Collect raw data image completed.\n");
}

int GroupNormalTest(void)
{
	int ret = 0;
	memcpy(LowerImageLimit, LowerImage, sizeof(LowerImageLimit));
	memcpy(UpperImageLimit, UpperImage, sizeof(UpperImageLimit));
	memcpy(RspSlopeDataMin, RspLowerSlope, sizeof(RspSlopeDataMin));
	memcpy(RspSlopeDataMax, RspUpperSlope, sizeof(RspSlopeDataMax));
	memcpy(RspNoiseP2PLimit, RspNoise, sizeof(RspNoiseP2PLimit));
	RspSetFreq(0);
	RspCollectRawData();
	ret = RspGroupTest();
	return ret;
}
int GroupLPWGTest(void)
{
	int ret = 0;
	RspSetFreq(3);
	RspCollectRawData();
	ret = RspGroupTest();
	return ret;
}
int ShortTest(void)
{
	int ret = 0;
	RspSetFreq(0);
	RspCollectRawData();
	ret = RspShortTest();
	return ret;
}
void CheckCrash(char *rst, int min_caps_value)
{
	int i = 0;
	int j = 0;
	int k = 0;
	int node_crack_count = 0;
	int rx_crack_count = 0;
	int row_crack_count = 0;
	int criterion = 0;
	unsigned char cmd;

	if (TestPreparation() == false) {
		TOUCH_ERR_MSG("%s error\n", __func__);
		goto error;
	}

	cmd = 20;
	DO_SAFE(Write8BitRegisters(F54DataBase, &cmd, 1), error);

	cmd = 0x01;
	DO_SAFE(Write8BitRegisters(F54CommandBase, &cmd, 1), error);

	count = 0;
	do {
		DO_SAFE(Read8BitRegisters(F54CommandBase, &cmd, 1), error);
		msleep(1);
		count++;
	} while (cmd != 0x00 && (count < DefaultTimeout));

	DO_SAFE(Read8BitRegisters((F54DataBase+REPORT_DATA_OFFEST),
				Data, MaxArrayLength), error);

	criterion = F12_2DTxCount * F12_2DRxCount * 40 / 100;
	for (i = 0; i < (int)TxChannelCount; i++) {
		for (j = 0; j < (int)RxChannelCount; j++) {
			ImagepF[i][j] = ((short)Data[k] |
					(short)Data[k+1] << 8);
			if ((ImagepF[i][j] < LowerImageLimit[i][j]) ||
					(ImagepF[i][j] > UpperImageLimit[i][j])) {
				if (ImagepF[i][j] < min_caps_value) {
					rx_crack_count++;
					node_crack_count++;
				} else {
					row_crack_count = 0;
				}

				if (F12_2DRxCount <= rx_crack_count)
					row_crack_count++;

				if (5 < row_crack_count
						|| criterion < node_crack_count) {
					snprintf(rst, PAGE_SIZE, "%d", 1);
					return;
				}
			}
			k += 2;
		}
		rx_crack_count = 0;
	}
	snprintf(rst, PAGE_SIZE, "%d", 0);
	TOUCH_INFO_MSG("Tx [%d] Rx [%d] node_crack_count %d, "
			"row_crack_count %d\n",
			i, j, node_crack_count, row_crack_count);
	return;
error:
	snprintf(rst, PAGE_SIZE, "%d", 0);
	TOUCH_ERR_MSG("window crack check fail\n");
}

void SCAN_PDT(void)
{
	int i;

	for (i = 0; i < scanMaxPageCount; i++) {
		if (switchPage(i))
			RunQueries();
	}
}

/* Main entry point for the application
 */
int F54Test(int input, int mode, char *buf)
{

	int ret = 0;
	unsigned char data;
	int retry_cnt1 = 0;
	int retry_cnt2 = 0;

	/*
	if (PowerOnSensor())
	{
	  fatal("Error powering on sensor.\n");
	}

	These 4 function calls are to scan the Page Description Table (PDT)
	Function $01, $11 and $34 are on page 0
	Function $54 is on page 1
	Function $55 is on Page ?
	Scan up to Page 4
	for (int i = 0; i < scanMaxPageCount ;i++)
	{
	  if (switchPage(i))
	   RunQueries();
	}
	*/

	/* Application should exit with the absence of Function $54
	 */
	/*if (!bHaveF54)
		return -ECANCELED;*/
	/*
	   while (1) {
	   printf("\nPress these keys for tests:\n");
	   printf(" a ) - Full Raw Capacitance Test\n");
	   printf(" b ) - ADC Range Test\n");
	   printf(" c ) - Sensor Speed Test\n");
	   printf(" d ) - TRex Open Test\n");
	   printf(" e ) - TRex Gnd Test\n");
	   printf(" f ) - TRx-to-TRx and TRx-to-Vdd shorts\n");
	   printf(" g ) - High Resistance Test\n");
	   printf(" h ) - Full Raw Capacitance Max/Min Test\n");
	   printf(" i ) - Abs Sensing ADC Range Test\n");
	   printf(" j ) - Abs Sensing Delta Capacitance\n");
	   printf(" k ) - Abs Sensing Raw Capcitance Test\n");
	   printf("---------------------------------------------------------");
	   printf("\n z ) - Version\n");
	   printf("\nPress any other key to exit.\n");
	   input = _getch();
	 */
retry:
	ret = switchPage(0x01);
	if (ret == -EAGAIN && ++retry_cnt1 <= 3) {
		TOUCH_INFO_MSG("retry switchPage, count=%d\n", retry_cnt1);
		goto retry;
	} else if (ret == 0) {
		return ret;
	}

	data = 0x00;
	Write8BitRegisters(F54DataBase+1, &data, 1);
	Write8BitRegisters(F54DataBase+2, &data, 1);

	outbuf = 0;
	out_buf = 0;
	memset(f54_wlog_buf, 0, sizeof(f54_wlog_buf));
	memset(wlog_buf, 0, sizeof(wlog_buf));

	switch (input) {
	case 'a':
		outbuf = strlcpy(f54_wlog_buf,
				"a - Full Raw Capacitance Test\n",
				sizeof(f54_wlog_buf));
		ret = ImageTest(mode, buf);
		break;
	case 'b':
		out_buf = strlcpy(wlog_buf, "b - ADC Range Test\n",
				sizeof(wlog_buf));
		ret = ADCRange(buf);
		break;
	case 'c':
		out_buf = strlcpy(wlog_buf, "c - Sensor Speed Test\n",
				sizeof(wlog_buf));
		ret = SensorSpeed(buf);
		break;
	case 'd':
		outbuf = strlcpy(f54_wlog_buf, "d - TRex Open Test\n",
				sizeof(f54_wlog_buf));
		TRexOpenTest(buf);
		break;
	case 'e':
		outbuf = strlcpy(f54_wlog_buf, "e - TRex Gnd Test\n",
				sizeof(f54_wlog_buf));
		ret = TRexGroundTest(buf);
		break;
	case 'f':
		outbuf = strlcpy(f54_wlog_buf, "f - TRex Short Test\n",
				sizeof(f54_wlog_buf));
		ret = TRexShortTest(buf);
		break;
	case 'g':
		outbuf = strlcpy(f54_wlog_buf, "g - High Resistance Test\n",
				sizeof(f54_wlog_buf));
		ret = HighResistanceTest(buf);
		break;
	case 'h':
		outbuf = strlcpy(f54_wlog_buf,
				"h - Full Raw Capacitance Max/Min Test\n",
				sizeof(f54_wlog_buf));
		MaxMinTest(buf);
		break;
	case 'i':
		outbuf = strlcpy(f54_wlog_buf,
				"i - Abs Sensing ADC Range Test\n",
				sizeof(f54_wlog_buf));
		AbsADCRange(buf);
		break;
	case 'j':
		outbuf = strlcpy(f54_wlog_buf,
				"j - Abs Sensing Delta Capacitance\n",
				sizeof(f54_wlog_buf));
		ret = AbsDelta(buf);
		break;
	case 'k':
		outbuf = strlcpy(f54_wlog_buf,
				"k - Abs Sensing Raw Capcitance Test\n",
				sizeof(f54_wlog_buf));
		AbsRaw(mode, buf);
		break;
	case 'l':
		CheckCrash(buf, mode);
		break;
	case 'm':
		ret = DeltaTest(buf);
		break;
	case 'n':
		outbuf = strlcpy(f54_wlog_buf,
				"n - Abs Sensing Raw Short Capcitance Test\n",
				sizeof(f54_wlog_buf));
		ret = AbsRaw(mode, buf);
		break;
	case 'o':
		outbuf = strlcpy(f54_wlog_buf,
				"k - Abs Sensing Raw Open Capcitance Test\n",
				sizeof(f54_wlog_buf));
		ret = AbsRaw(mode, buf);
		break;
	case 'p':
		out_buf = strlcpy(wlog_buf, "p - RSP Calibration Test\n", sizeof(wlog_buf));
		ret = RspCalibrationTest();
		break;
	case 'q':
		outbuf = strlcpy(f54_wlog_buf, "q - RSP Group Normal Test\n", sizeof(f54_wlog_buf));
		ret = GroupNormalTest();
		break;
	case 'r':
		outbuf = strlcpy(f54_wlog_buf, "r - RSP Group LPWG Test\n", sizeof(f54_wlog_buf));
		ret = GroupLPWGTest();
		break;
	case 's':
		outbuf = strlcpy(f54_wlog_buf, "s - RSP Short Test\n", sizeof(f54_wlog_buf));
		ret = ShortTest();
		break;
	case 'x':
		out_buf = strlcpy(wlog_buf, "x - Noise Delta Test\n", sizeof(wlog_buf));
		ret = NoiseDeltaTest(buf);
		break;
	case 'y':
		out_buf = strlcpy(wlog_buf, "y - GND Test\n", sizeof(wlog_buf));
		ret = GndTest(buf);
		break;
	case 'z':
		TOUCH_INFO_MSG("Version: %s\n", VERSION);
		break;
	default:
		return -EINVAL;
	}

	if (switchPage(0x00) != true) {
		TOUCH_INFO_MSG("switchPage failed\n");
		/*Reset Device*/
		Reset();
	}

	if (ret == -EAGAIN && ++retry_cnt2 <= 3) {
		TOUCH_INFO_MSG("retry Test, count=%d\n", retry_cnt2);
		goto retry;
	} else
		return ret;
}
