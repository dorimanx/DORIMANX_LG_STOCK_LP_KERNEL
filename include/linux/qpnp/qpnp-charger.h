/*
 * Copyright (C) 2013 LG Electronics
 * G2 Task BSP powerteam.  <G2-Task-Power@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __QPNP_CHARGER_H_
#define __QPNP_CHARGER_H_

#ifdef CONFIG_LGE_PM
extern int32_t qpnp_charger_is_ready(void);
extern int32_t external_qpnp_enable_charging(bool enable);
#endif
#ifdef CONFIG_LGE_QC_2_0_SCENARIO
enum qpnp_quick_charging_state {
	QC20_STATUS_NONE		= 0,
	QC20_STATUS_LCD_ON	= 1,
	QC20_STATUS_LCD_OFF 	= 2,
	QC20_STATUS_CALL_ON	= 3,
	QC20_STATUS_CALL_OFF	= 4,
	QC20_STATUS_THERMAL_ON = 5,
	QC20_STATUS_THERMAL_OFF	= 6,
};
void qpnp_qc20_set_qc_state( int qc_state);
#endif


#endif
