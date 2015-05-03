/*
* Copyright(c) 2013, LGE Inc. All rights reserved.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
*/

#ifndef __BQ24296_CHARGER_H__
#define __BQ24296_CHARGER_H__

#define BQ24296_NAME  "bq24296"

struct bq24296_platform_data {
	int int_gpio;
	int ext_chg_en;
	int otg_en;
	int chg_current_ma;
	int term_current_ma;
	int vbat_max_mv;
	int pre_chg_current_ma;
	int sys_vmin_mv;
	int vin_limit_mv;
	int icl_vbus_mv;
};
extern int32_t external_bq24296_enable_charging(bool enable);
extern int bq24296_get_batt_temp_origin(void);
#endif
