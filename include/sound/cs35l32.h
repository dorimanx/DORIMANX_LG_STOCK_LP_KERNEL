/*
 * linux/sound/cs35l32.h -- Platform data for CS35l32
 *
 * Copyright (c) 2012 Cirrus Logic Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __CS35L32_H
#define __CS35L32_H

struct cs35l32_platform_data {
	int gpio_nreset;
	/* MCLK Freq */
	unsigned int mclk_freq;
	/* Low Battery Threshold */
	unsigned int batt_thresh;
	/* Low Battery Recovery */
	unsigned int batt_recov;
	/* Low Battery Disable */
	unsigned int batt_disable;

	/* LED/Audio Power Management */
	/* LED Current Management*/
	unsigned int led_mng;
	/* Audio Gain w/ LED */
	unsigned int audiogain_mng;
	/* Boost Management */
	unsigned int boost_mng;
	
	/* Clock and ADSP CTL */
	unsigned int adsp_drive;
	/* Master Slave Cfg */
	unsigned int adsp_clkctl;
	/* Data CFG for DUAL device */
	unsigned int sdout_datacfg;
	/* SDOUT Sharing */
	unsigned int sdout_share;
	
	/* Speaker Protection Release CTL */
	/* Amp Short Release */
	unsigned int amp_short;
	/* Over Temp Release */
	unsigned int overtemp_ctl;	
	
};

#endif /* __CS35L32_H */
