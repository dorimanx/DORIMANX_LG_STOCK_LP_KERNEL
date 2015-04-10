/*
 * Atmel maXTouch Touchscreen driver
 *

 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include <linux/i2c/atmel_s540.h>

static u8 end_config_s[] = { MXT_RESERVED_T255 };

static u8 t72_on_config[] = { MXT_PROCG_NOISESUPPRESSION_T72, 1,
	0, 1,
};

static u8 t72_off_config[] = { MXT_PROCG_NOISESUPPRESSION_T72, 1,
	0, 0,
};

static u8 t24_off_config[] = { MXT_PROCI_ONETOUCH_T24, 1,
	0, 0,
};

static u8 t24_on_config[] = { MXT_PROCI_ONETOUCH_T24, 1,
	0, 3,
};

static u8 t7_normal_config[] = { MXT_GEN_POWER_T7, 5,
	0, 48,
	1, 255,
	2, 20,
	3, 67,
	4, 01,
};
static u8 t7_sus_config[] = { MXT_GEN_POWER_T7, 5,
	0, 64,
	1, 15,
	2, 5,
	3, 64,
	4, 1,
};

static u8 t8_normal_config[] = { MXT_GEN_ACQUIRE_T8, 2,
	2, 5,
	4, 5,
};

static u8 t8_sus_config[] = { MXT_GEN_ACQUIRE_T8, 2,
	2, 255,
	4, 0,
};

static u8 t46_normal_config[] = { MXT_SPT_CTECONFIG_T46, 5,
	0, 12,
	2, 8,
	3, 8,
	5, 1,
	6, 4,
};
static u8 t46_sus_config[] = { MXT_SPT_CTECONFIG_T46, 5,
	0, 0,
	2, 4,
	3, 16,
	5, 0,
	6, 0,
};

static u8 t47_normal_config[] = { MXT_PROCI_STYLUS_T47, 1,
	0, 0x09,
};

static u8 t47_sus_config[] = { MXT_PROCI_STYLUS_T47, 1,
	0, 0x08,
};

static u8 t65_normal_config[] = { MXT_PROCI_LENSBENDING_T65, 1,
	0, 1,
};

static u8 t65_sus_config[] = { MXT_PROCI_LENSBENDING_T65, 1,
	0, 0,
};

/*static u8 t56_normal_config[] = { MXT_PROCI_SHIELDLESS_T56, 1,
	0, 1,
};

static u8 t56_sus_config[] = { MXT_PROCI_SHIELDLESS_T56, 1,
	0, 0,
};*/

#ifdef MXT_LPWG
//UDF
static u8 t35_UDF_on_config[] = {	MXT_SPT_PROTOTYPE_T35, 7,
	0, 0x03,
	128, 100,
	129, 0,
	130, 24,
	131, 125,
	132, 1,
	133, 4,
};

static u8 t35_UDF_off_config[] = {	MXT_SPT_PROTOTYPE_T35, 1,
	0, 0,
};

//UDF
static const u8 *UDF_on_configs_[] = {
	t35_UDF_on_config,
	end_config_s
};

static const u8 *UDF_off_configs_[] = {
	t35_UDF_off_config,
	end_config_s
};
#endif
static const u8 *ta_configs[] = {
	t72_on_config,
	end_config_s
};
static const u8 *bat_configs[] = {
	t72_off_config,
	end_config_s
};

//DoubleTap
static const u8 *knockon_on_configs_[] = {
	t24_on_config,
	end_config_s
};

static const u8 *knockon_off_configs_[] = {
	t24_off_config,
	end_config_s
};

static const u8 *lowpower_on_configs[] = {
	//t100_sus_config,
	t47_sus_config,
	//t56_sus_config,
	t65_sus_config,
	t72_off_config,
	t7_sus_config,
	t8_sus_config,
	t46_sus_config,
	end_config_s
};
static const u8 *lowpower_off_configs[] = {
	//t100_normal_config,
	t47_normal_config,
	//t56_normal_config,
	t65_normal_config,
	t72_on_config,
	t7_normal_config,
	t8_normal_config,
	t46_normal_config,
	end_config_s
};
