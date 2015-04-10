/*
 * Copyright (C) 2012, Kyungtae Oh <kyungtae.oh@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under  the terms of the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the License, or (at your
 * option) any later version.
 *
 */

#ifndef __LINUX_POWER_UNIFIED_WLC_CHARGER_H__
#define __LINUX_POWER_UNIFIED_WLC_CHARGER_H__

#define UNIFIED_WLC_DEV_NAME "unified_wlc"

struct unified_wlc_platform_data {
	unsigned int wlc_int_gpio;
	unsigned int wlc_full_chg;
	//unsigned int wireless_charging;
	//unsigned int chg_state_gpio;
	//int (*wlc_is_plugged)(void);
};
#ifdef CONFIG_CHARGER_UNIFIED_WLC_ALIGNMENT
#ifdef CONFIG_CHARGER_FACTORY_MODE
#define WLC_ALIGN_DEBUG
#endif
#define WLC_ALIGN_INTERVAL	(300)
#endif

typedef enum
{
	FAKE_DISCONNECTION,
	FAKE_DISCONNECTED,
	FAKE_CONNECTING,
	FAKE_CONNECTED,
	FAKE_UNKNOWN,
}otg_fake_status;

extern int is_wireless_charger_plugged(void);
extern int wireless_charging_completed(void);

#ifdef CONFIG_CHARGER_UNIFIED_WLC_BQ5102X
extern int bq5102x_set_output_voltage(int output_voltage);
extern int bq5102x_set_output_current(int output_current);
extern int bq5102x_set_fod_calibration(int fod_offset);
#endif

#endif
