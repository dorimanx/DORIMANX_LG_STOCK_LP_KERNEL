/* Copyright (c) 2013 LGE Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/i2c/bq24296_charger.h>
#include <linux/power_supply.h>
#include <linux/bitops.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <mach/board_lge.h>
#include <linux/max17048_battery.h>
#include <linux/qpnp/qpnp-adc.h>
#include "../../arch/arm/mach-msm/smd_private.h"
#include <linux/usb/otg.h>
#include "../usb/dwc3/dwc3_otg.h"
#include "../usb/dwc3/core.h"
#include <linux/reboot.h>
#include <linux/switch.h>
#include <linux/qpnp-misc.h>
#if defined(CONFIG_LGE_PM_CHARGING_SUPPORT_PHIHONG)
#include <linux/completion.h>
#include <linux/kthread.h>
#include <linux/timer.h>
#endif

#define I2C_SUSPEND_WORKAROUND
#if defined(CONFIG_CHARGER_UNIFIED_WLC)
#include <linux/power/unified_wireless_charger.h>
#endif

#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
#include <mach/lge_charging_scenario.h>
#define MONITOR_BATTEMP_POLLING_PERIOD          (60*HZ)
#endif

#if defined(CONFIG_LGE_PM_BATTERY_ID_CHECKER)
#include <linux/power/lge_battery_id.h>
#endif

#ifdef CONFIG_ZERO_WAIT
#include <linux/zwait.h>
#endif

#ifndef BIT
#define BIT(x)	(1 << (x))
#endif

/* Register definitions */
#define BQ00_INPUT_SRC_CONT_REG              0X00
#define BQ01_PWR_ON_CONF_REG                 0X01
#define BQ02_CHARGE_CUR_CONT_REG             0X02
#define BQ03_PRE_CHARGE_TERM_CUR_REG         0X03
#define BQ04_CHARGE_VOLT_CONT_REG            0X04
#define BQ05_CHARGE_TERM_TIMER_CONT_REG      0X05
#define BQ06_IR_COMP_THERM_CONT_REG          0X06
#define BQ07_MISC_OPERATION_CONT_REG         0X07
#define BQ08_SYSTEM_STATUS_REG               0X08
#define BQ09_FAULT_REG                       0X09
#define BQ0A_VENDOR_PART_REV_STATUS_REG      0X0A

/* BQ00 Input Source Control Register MASK */
#define EN_HIZ			BIT(7)
#define VINDPM_MASK 		(BIT(6)|BIT(5)|BIT(4)|BIT(3))
#define IINLIM_MASK 		(BIT(2)|BIT(1)|BIT(0))

/* BQ01 Power-On Configuration  Register MASK */
#define RESET_REG_MASK		BIT(7)
#define CHG_CONFIG_MASK 	(BIT(5)|BIT(4))
#define OTG_ENABLE_MASK         BIT(5)

/* #define SYSTEM_MIN_VOLTAGE_MASK    0x0E */
#define SYS_MIN_VOL_MASK	(BIT(3)|BIT(2)|BIT(1))
#define BOOST_LIM 		BIT(0)

/* BQ02 Charge Current Control Register MASK */
#define ICHG_MASK 		(BIT(7)|BIT(6)|BIT(5)|BIT(4)|BIT(3)|BIT(2))
#define FORCE_20PCT_MASK	BIT(0)

/* BQ03 Pre-Charge, Termination Current Control Register MASK */
#define IPRECHG_MASK 		(BIT(7)|BIT(6)|BIT(5)|BIT(4))
#define ITERM_MASK		(BIT(3)|BIT(2)|BIT(1)|BIT(0))

/* BQ04 Charge Voltage Control Register MASK */
#define CHG_VOLTAGE_LIMIT_MASK 	(BIT(7)|BIT(6)|BIT(5)|BIT(4)|BIT(3)|BIT(2))
#define BATLOWV_MASK 		BIT(1)
#define VRECHG_MASK 		BIT(0)

/* BQ05 Charge Termination, Timer-Control Register MASK */
#define EN_CHG_TERM_MASK 	BIT(7)
#define I2C_TIMER_MASK          (BIT(5)|BIT(4))
#define EN_CHG_TIMER_MASK	BIT(3)
#define CHG_TIMER_MASK 		(BIT(2)|BIT(1))

/* BQ06 IR Compensation, Thermal Regulation Control Register MASK */
#define IR_COMP_R_MASK		(BIT(7)|BIT(6)|BIT(5))
#define IR_COMP_VCLAMP_MASK 	(BIT(4)|BIT(3)|BIT(2))

/* BQ07 Misc-Operation Control Register MASK */
#define BATFET_DISABLE_MASK 	BIT(5)

/* BQ08 SYSTEM_STATUS_REG Mask */
#define VBUS_STAT_MASK 		(BIT(7)|BIT(6))
#define PRE_CHARGE_MASK 	BIT(4)
#define FAST_CHARGE_MASK 	BIT(5)
#define CHRG_STAT_MASK		(FAST_CHARGE_MASK|PRE_CHARGE_MASK)
#define DPM_STAT_MASK		BIT(3)
#define PG_STAT_MASK		BIT(2)
#define THERM_STAT_MASK 	BIT(1)
#define VSYS_STAT_MASK 		BIT(0)

/* BQ09 FAULT_REG Mask */
#define CHRG_FAULT_MASK 	(BIT(5)|BIT(4))

#define LT_CABLE_56K		6
#define LT_CABLE_130K		7
#define LT_CABLE_910K		11

#ifdef CONFIG_LGE_THERMALE_CHG_CONTROL_FOR_WLC
#define THERMALE_NOT_TRIGGER -1
#define THERMALE_ALL_CLEAR 0
#endif

#ifdef CONFIG_LGE_PM_LLK_MODE
bool battemp_work_cancel = false;
bool llk_mode = false;
#endif

#define _WIRELESS_ "wireless"
#define _BATTERY_     "battery"
#define _USB_		"usb"
#define _CN_		"cn"
#define _THIS_		"ac"

static inline struct power_supply *_psy_check_ext(struct power_supply *_psy,
					const char *_psp_name)
{
	if (!likely(_psy)) {
		pr_info("psp is not found %s\n", _psp_name);
		_psy = power_supply_get_by_name((char *)_psp_name);
	}
	return _psy;
}
#define bq24296_charger_psy_setprop(_me, _psy, _psp, _val) \
	({\
		struct power_supply *__psy = _me->_psy;\
		union power_supply_propval __propval = { .intval = _val };\
		int __rc = -ENXIO;\
		if (likely(__psy && __psy->set_property)) {\
			__rc = __psy->set_property(__psy,\
				POWER_SUPPLY_PROP_##_psp,\
				&__propval);\
		} \
		__rc;\
	})

#define bq24296_charger_psy_getprop(_me, _psy, _psp, _val)	\
	({\
		struct power_supply *__psy = _me->_psy;\
		int __rc = -ENXIO;\
		if (likely(__psy && __psy->get_property)) {\
			__rc = __psy->get_property(__psy,\
				POWER_SUPPLY_PROP_##_psp,\
				_val);\
		} \
		__rc;\
	})

#define bq24296_charger_psy_getprop_event(_me, _psy, _psp, _val, _psp_name)	\
	({\
		struct power_supply *__psy = _me->_psy;\
		int __rc = -ENXIO;\
		__psy = _psy_check_ext(__psy, _psp_name);\
		if (likely(__psy && __psy->get_event_property)) {\
			__rc = __psy->get_event_property(__psy,\
				POWER_SUPPLY_PROP_##_psp,\
				_val);\
		} \
		__rc;\
	})

#define bq24296_charger_psy_setprop_event(_me, _psy, _psp, _val, _psp_name) \
	({\
		struct power_supply *__psy = _me->_psy;\
		union power_supply_propval __propval = { .intval = _val };\
		int __rc = -ENXIO;\
		__psy = _psy_check_ext(__psy, _psp_name);\
		if (likely(__psy && __psy->set_property)) {\
			__rc = __psy->set_event_property(__psy,\
				POWER_SUPPLY_PROP_##_psp,\
				&__propval);\
		} \
		__rc;\
	})

enum bq24296_chg_status {
	BQ_CHG_STATUS_NONE 		= 0,
	BQ_CHG_STATUS_PRE_CHARGE	= 1,
	BQ_CHG_STATUS_FAST_CHARGE 	= 2,
	BQ_CHG_STATUS_FULL 		= 3,
	BQ_CHG_STATUS_EXCEPTION		= 4,
};

static const char * const bq24296_chg_status[] = {
	"none",
	"pre-charge",
	"fast-charge",
	"full"
	"exception"
};
#define NULL_CHECK_VOID(p)  \
			if (!(p)) { \
				pr_err("FATAL (%s)\n", __func__); \
				return ; \
			}
#define NULL_CHECK(p, err)  \
			if (!(p)) { \
				pr_err("FATAL (%s)\n", __func__); \
				return err; \
			}

#if defined(CONFIG_LGE_PM_CHARGING_SUPPORT_PHIHONG)
typedef enum {
	PHIHONG_PLUG_OUT,
	PHIHONG_NOT_VERIFIED,
	PHIHONG_VERIFYING,
	PHIHONG_VERIFYING_PLUG_OUT,
	PHIHONG_YES,
	PHIHONG_PERMANENT_YES,
	PHIHONG_NO,
	PHIHONG_NO_NEED,
	PHIHONG_STATUS_MAX,
} phihong_status;
#define CHECK_PHIHONG(_chip)						\
	({								\
		int ret = 0;						\
		if (_chip->phihong == PHIHONG_VERIFYING ||		\
			_chip->phihong == PHIHONG_VERIFYING_PLUG_OUT ||	\
			_chip->phihong == PHIHONG_YES ||		\
			_chip->phihong == PHIHONG_PERMANENT_YES) {	\
			ret = 1;					\
		}							\
		ret;							\
	})
#endif

#if defined(CONFIG_VZW_POWER_REQ)
typedef enum vzw_chg_state {
	VZW_NO_CHARGER,
	VZW_NORMAL_CHARGING,
	VZW_NOT_CHARGING,
	VZW_UNDER_CURRENT_CHARGING,
	VZW_USB_DRIVER_UNINSTALLED,
	VZW_CHARGER_STATUS_MAX,
} chg_state;
#endif
struct bq24296_chip {
	struct i2c_client  *client;
	struct dentry  *dent;
	struct switch_dev batt_removed;

	int chg_current_ma;
	int term_current_ma;
	int vbat_max_mv;
	int pre_chg_current_ma;
	int sys_vmin_mv;
	int vin_limit_mv;
	int int_gpio;
	int ext_chg_en;
	int otg_en;
	int irq;
#if defined(CONFIG_CHARGER_UNIFIED_WLC)
	int wlc_present;
#endif
	int usb_present;
	int usb_online;
	int ac_present;
	int ac_online;
	int chg_type;
	int charging_disabled;
	int full_design;
	bool chg_timeout;
	int icl_vbus_mv;
	int icl_idx;
	bool icl_first;
	int icl_fail_cnt;
	int set_icl_idx;
	struct wake_lock icl_wake_lock;
	enum bq24296_chg_status	chg_status;

	struct delayed_work  irq_work;
#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
	struct delayed_work		battemp_work;
	struct wake_lock		lcs_wake_lock;
	enum   lge_btm_states	btm_state;
	int pseudo_ui_chg;
	int otp_ibat_current;
#ifdef CONFIG_LGE_THERMALE_CHG_CONTROL
	int chg_current_te;
#endif
#ifdef CONFIG_LGE_THERMALE_CHG_CONTROL_FOR_WLC
	int wlc_input_current_te;
	int wlc_chg_current_te;
#endif
#endif
	struct wake_lock  chg_wake_lock;
#if defined(CONFIG_CHARGER_UNIFIED_WLC)
	struct power_supply  *wlc_psy;
#endif
	struct power_supply  *usb_psy;
	struct power_supply  ac_psy;
	struct power_supply  batt_psy;
	struct power_supply  *psy_this;
	struct power_supply  *cn_psy;
	/* TODO: should be verify */
	struct wake_lock uevent_wake_lock;

	int  set_chg_current_ma;
	struct wake_lock battgone_wake_lock;
	struct wake_lock chg_timeout_lock;
	struct qpnp_vadc_chip *vadc_dev;
#if defined(CONFIG_LGE_PM_BATTERY_ID_CHECKER)
	int batt_id_smem;
#endif
	bool watchdog;
#ifdef I2C_SUSPEND_WORKAROUND
	struct delayed_work check_suspended_work;
	bool suspend;
#endif
	struct delayed_work usbin_mon;
	struct mutex usbin_lock;
	int			usbin_ref_count;
	int			last_usbin_mv;
#if defined(CONFIG_VZW_POWER_REQ)
	chg_state		vzw_chg_mode;
	unsigned int	adc_sum;
	int			usbin_ref_count_vzw;
#endif
	bool batt_present;
#if defined(CONFIG_LGE_PM_LLK_MODE)
	bool store_demo_enabled;
#endif
#if defined(CONFIG_LGE_PM_CHARGING_SUPPORT_PHIHONG)
	struct timer_list phihong_timer;
	int pre_input_current_ma;
	phihong_status phihong;
	struct task_struct *phihong_task;
	struct completion phihong_complete;
#endif
#if defined(CONFIG_CHARGER_UNIFIED_WLC)
	bool	wlc_otg;
	otg_fake_status wlc_otg_status;
#endif
#if defined(CONFIG_MACH_MSM8974_G3_ATT) || defined(CONFIG_MACH_MSM8974_G3_CA)
	struct delayed_work pma_workaround_work;
	struct wake_lock pma_workaround_wake_lock;
#endif
};

#ifdef CONFIG_SENSORS_QPNP_ADC_VOLTAGE
int last_batt_temp;
#endif

int safety_timer_enabled;

#if defined(CONFIG_CHARGER_UNIFIED_WLC)
static int wireless_charging;
#endif

static struct bq24296_chip *the_chip;

extern struct pseudo_batt_info_type pseudo_batt_info;

struct debug_reg {
	char  *name;
	u8  reg;
};

#define BQ24296_DEBUG_REG(x, y) {#x#y, y##_REG}

static struct debug_reg bq24296_debug_regs[] = {
	BQ24296_DEBUG_REG(00_, BQ00_INPUT_SRC_CONT),
	BQ24296_DEBUG_REG(01_, BQ01_PWR_ON_CONF),
	BQ24296_DEBUG_REG(02_, BQ02_CHARGE_CUR_CONT),
	BQ24296_DEBUG_REG(03_, BQ03_PRE_CHARGE_TERM_CUR),
	BQ24296_DEBUG_REG(04_, BQ04_CHARGE_VOLT_CONT),
	BQ24296_DEBUG_REG(05_, BQ05_CHARGE_TERM_TIMER_CONT),
	BQ24296_DEBUG_REG(06_, BQ06_IR_COMP_THERM_CONT),
	BQ24296_DEBUG_REG(07_, BQ07_MISC_OPERATION_CONT),
	BQ24296_DEBUG_REG(08_, BQ08_SYSTEM_STATUS),
	BQ24296_DEBUG_REG(09_, BQ09_FAULT),
	BQ24296_DEBUG_REG(0A_, BQ0A_VENDOR_PART_REV_STATUS),
};

/* pre-define functions here */
static int bq24296_enable_charging(struct bq24296_chip *chip, bool enable);
static int bq24296_set_adjust_ibat(struct bq24296_chip *chip, int ma);
static int bq24296_get_adjust_ibat(struct bq24296_chip *chip, int *mv);
static int bq24296_get_force_ichg_decrease(struct bq24296_chip *chip, int *enable);
/* end of pre-define */

static unsigned int cable_type;
static unsigned int cable_smem_size;
static unsigned int factory_mode;

static bool is_factory_cable(void)
{
	unsigned int cable_info;
	cable_info = lge_pm_get_cable_type();

	if ((cable_info == CABLE_56K ||
		cable_info == CABLE_130K ||
		cable_info == CABLE_910K) ||
		(cable_type == LT_CABLE_56K ||
		cable_type == LT_CABLE_130K ||
		cable_type == LT_CABLE_910K))
		return true;
	else
		return false;
}

static bool is_factory_cable_130k(void)
{
	unsigned int cable_info;
	cable_info = lge_pm_get_cable_type();

	if (cable_info == CABLE_130K ||
		cable_type == LT_CABLE_130K)
		return true;
	else
		return false;
}

static int bq24296_read_reg(struct i2c_client *client, int reg, u8 *val)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev,
			"i2c read fail: can't read from %02x: %d\n",
			reg, ret);
		return ret;
	} else {
		*val = ret;
	}

	return 0;
}

static int bq24296_write_reg(struct i2c_client *client, int reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		dev_err(&client->dev,
			"i2c write fail: can't write %02x to %02x: %d\n",
			val, reg, ret);
		return ret;
	}
	return 0;
}

static int bq24296_masked_write(struct i2c_client *client, int reg,
			       u8 mask, u8 val)
{
	s32 rc;
	u8 temp;

	rc = bq24296_read_reg(client, reg, &temp);
	if (rc) {
		pr_err("bq24296_read_reg failed: reg=%03X, rc=%d\n",
				reg, rc);
		return rc;
	}

	temp &= ~mask;
	temp |= val & mask;

	rc = bq24296_write_reg(client, reg, temp);
	if (rc) {
		pr_err("bq24296_write failed: reg=%03X, rc=%d\n",
				reg, rc);
		return rc;
	}

	return 0;
}

static void bq24296_reginfo(struct bq24296_chip *chip)
{
	int i;
	int cnt = ARRAY_SIZE(bq24296_debug_regs);
	u8 val[cnt];

	NULL_CHECK_VOID(chip);

	for (i = 0; i < cnt; i++)
		bq24296_read_reg(chip->client,
			bq24296_debug_regs[i].reg, &val[i]);

	pr_info("0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
		val[0], val[1], val[2], val[3], val[4],
		val[5], val[6], val[7], val[8], val[9]);
}

#if defined(CONFIG_VZW_POWER_REQ)
static int bq24296_get_en_hiz(struct bq24296_chip *chip, bool *enable)
{
	int ret;
	u8 reg_val = 0;
	NULL_CHECK(chip, -EINVAL);
	ret = bq24296_read_reg(chip->client, BQ00_INPUT_SRC_CONT_REG,
		&reg_val);
	if (ret)
		pr_err("Fail to read EN_HIZ.\n");
	*enable = reg_val & EN_HIZ;
	return ret;
}

static bool bq24296_is_en_hiz(struct bq24296_chip *chip)
{
	bool enable;
	bq24296_get_en_hiz(chip, &enable);
	return enable;
}

static int bq24296_set_en_hiz(struct bq24296_chip *chip, bool enable)
{
	int ret;
	NULL_CHECK(chip, -EINVAL);
	pr_debug("Set EN_HIZ %s\n", enable ? "enable" : "disable");
	ret = bq24296_masked_write(chip->client, BQ00_INPUT_SRC_CONT_REG,
		EN_HIZ, enable ? EN_HIZ : 0);
	if (ret)
		pr_err("fail to write EN_HIZ.\n");
	return ret;
}
#endif

struct input_ma_limit_entry {
	int  icl_ma;
	u8  value;
};

static struct input_ma_limit_entry icl_ma_table[] = {
	{100, 0x00},
	{150, 0x01},
	{500, 0x02},
	{900, 0x03},
	{1200, 0x04},
	{1500, 0x05},
	{2000, 0x06},
	{3000, 0x07},
};

#ifdef CONFIG_LGE_THERMALE_CHG_CONTROL
static int iusb_control;
static int
bq24296_set_iusb_control(const char *val, struct kernel_param *kp)
{
	int ret;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}

	return 0;
}
module_param_call(iusb_control, bq24296_set_iusb_control,
	param_get_uint, &iusb_control, 0644);
#endif

#define INPUT_CURRENT_LIMIT_MIN_MA  100
#define INPUT_CURRENT_LIMIT_MAX_MA  3000
#define INPUT_CURRENT_LIMIT_TA      2000
#define INPUT_CURRENT_LIMIT_FACTORY 2000
#define INPUT_CURRENT_LIMIT_USB20   500
#define INPUT_CURRENT_LIMIT_USB30   900
#if defined(CONFIG_CHARGER_UNIFIED_WLC)
/* G3 HW requirement */
/* In wireless charging, change Iusb. */
#if		defined(CONFIG_MACH_MSM8974_G3_KR) || \
		defined(CONFIG_MACH_MSM8974_G3_GLOBAL_COM) || \
		defined(CONFIG_MACH_MSM8974_G3_SPR_US) || \
		defined(CONFIG_MACH_MSM8974_G3_USC_US) || \
		defined(CONFIG_MACH_MSM8974_G3_ACG_US) || \
		defined(CONFIG_MACH_MSM8974_G3_TMO_US)
#define INPUT_CURRENT_LIMIT_WLC		900	/* IDT IDT9025A(WPC) 900mA */
#elif	defined(CONFIG_MACH_MSM8974_G3_VZW) || defined(CONFIG_MACH_MSM8974_G3_LRA)
#define INPUT_CURRENT_LIMIT_WLC		900	/* TI BQ51020(WPC) 900mA */
#elif	defined(CONFIG_MACH_MSM8974_G3_ATT) || defined(CONFIG_MACH_MSM8974_G3_CA)
#define INPUT_CURRENT_LIMIT_WLC		900	/* TI BQ51221(PMA) 900mA */
#else
#define INPUT_CURRENT_LIMIT_WLC		900	/* Other 900mA */
#endif

#define INPUT_CURRENT_LIMIT_WLC_ADJUST	500
#endif
static int bq24296_set_input_i_limit(struct bq24296_chip *chip, int ma)
{
	int i;
	u8 temp;
	NULL_CHECK(chip, -EINVAL);
	if (is_factory_cable()) {
		if (is_factory_cable_130k()) {
			pr_info("factory cable detected(130k) iLimit 500mA\n");
			return bq24296_masked_write(chip->client,
				BQ00_INPUT_SRC_CONT_REG, IINLIM_MASK, 0x02);
		} else {
			pr_info("factory cable detected  iLimit 1500mA\n");
			return bq24296_masked_write(chip->client,
				BQ00_INPUT_SRC_CONT_REG, IINLIM_MASK, 0x06);
		}
	}
#if defined(CONFIG_VZW_POWER_REQ)
	bq24296_set_en_hiz(chip, (chip->usb_psy->is_floated_charger &&
				chip->usb_present) ? true : false);
#endif
#ifdef CONFIG_LGE_THERMALE_CHG_CONTROL
	if (ma > iusb_control && iusb_control >= INPUT_CURRENT_LIMIT_USB30 &&
			ma >= INPUT_CURRENT_LIMIT_USB30) {
		ma = iusb_control;
		pr_info("IUSB limit %dmA\n", iusb_control);
	}
#endif

	for (i = ARRAY_SIZE(icl_ma_table) - 1; i > 0; i--) {
		if (ma >= icl_ma_table[i].icl_ma)
			break;
	}
	temp = icl_ma_table[i].value;

	pr_info("input current limit=%d setting 0x%02x\n", ma, temp);
	return bq24296_masked_write(chip->client, BQ00_INPUT_SRC_CONT_REG,
			IINLIM_MASK, temp);
}

static int bq24296_get_input_i_limit(struct bq24296_chip *chip, int *ma)
{
	u8 reg_val = 0;
	int ret;
	NULL_CHECK(chip, -EINVAL);

	ret = bq24296_read_reg(chip->client, BQ00_INPUT_SRC_CONT_REG, &reg_val);
	if (ret) {
		pr_err("failed to read BQ08_SYSTEM_STATUS_REG ret=%d\n", ret);
		return -EIO;
	}
	*ma = icl_ma_table[reg_val & IINLIM_MASK].icl_ma;
	return ret;
}

#define IBAT_MAX_MA 3008
#define IBAT_MIN_MA  512
#define IBAT_DIS_MA  (100)
#define IBAT_STEP_MA  64
#define IBAT_DEFAULT  2048
#if defined(CONFIG_CHARGER_UNIFIED_WLC)
/* G3 HW requirement */
/* In wireless charging, change Ibat. */
#if		defined(CONFIG_MACH_MSM8974_G3_KR) || \
		defined(CONFIG_MACH_MSM8974_G3_GLOBAL_COM) || \
		defined(CONFIG_MACH_MSM8974_G3_SPR_US) || \
		defined(CONFIG_MACH_MSM8974_G3_USC_US) || \
		defined(CONFIG_MACH_MSM8974_G3_ACG_US) || \
		defined(CONFIG_MACH_MSM8974_G3_TMO_US)
#define IBAT_WLC			896		/* IDT IDT9025A(WPC) 896mA */
#elif	defined(CONFIG_MACH_MSM8974_G3_VZW) || defined(CONFIG_MACH_MSM8974_G3_LRA)
#define IBAT_WLC			896		/* TI BQ51020(WPC) 896mA */
#elif	defined(CONFIG_MACH_MSM8974_G3_ATT) || defined(CONFIG_MACH_MSM8974_G3_CA)
#define IBAT_WLC			896		/* TI BQ51221(PMA) 896mA */
#else
#define IBAT_WLC			896		/* Other 896mA */
#endif

#if		defined(CONFIG_MACH_MSM8974_G3_KR) || \
		defined(CONFIG_MACH_MSM8974_G3_GLOBAL_COM) || \
		defined(CONFIG_MACH_MSM8974_G3_SPR_US) || \
		defined(CONFIG_MACH_MSM8974_G3_USC_US) || \
		defined(CONFIG_MACH_MSM8974_G3_ACG_US) || \
		defined(CONFIG_MACH_MSM8974_G3_TMO_US)
#define IBAT_WLC_ADJUST		768		/* IDT IDT9025A(WPC) 768mA */
#elif	defined(CONFIG_MACH_MSM8974_G3_VZW) || defined(CONFIG_MACH_MSM8974_G3_LRA)
#define IBAT_WLC_ADJUST		704		/* TI BQ51020(WPC) 704mA */
#elif	defined(CONFIG_MACH_MSM8974_G3_ATT) || defined(CONFIG_MACH_MSM8974_G3_CA)
#define IBAT_WLC_ADJUST		768		/* TI BQ51221(PMA) 768mA */
#else
#define IBAT_WLC_ADJUST		512		/* Other 512mA */
#endif
#endif
static int bq24296_set_ibat_max(struct bq24296_chip *chip, int ma)
{
	u8 reg_val = 0;
	int set_ibat = 0;

	NULL_CHECK(chip, -EINVAL);
	if (is_factory_cable()) {
		return bq24296_masked_write(chip->client,
			BQ02_CHARGE_CUR_CONT_REG, ICHG_MASK, 0x5c);
	}
	if (ma < IBAT_MIN_MA) {
		bq24296_enable_charging(chip, false);
		ma = IBAT_MIN_MA;
	} else {
		bq24296_enable_charging(chip, true);
	}
	if (ma > IBAT_MAX_MA) {
		ma = IBAT_MAX_MA;
	}

	reg_val = (ma - IBAT_MIN_MA)/IBAT_STEP_MA;
	set_ibat = reg_val * IBAT_STEP_MA + IBAT_MIN_MA;
	reg_val = reg_val << 2;
	chip->set_chg_current_ma = set_ibat;
	pr_info("req_ibat = %d set_ibat = %d reg_val = 0x%02x\n",
				ma, set_ibat, reg_val);

	return bq24296_masked_write(chip->client, BQ02_CHARGE_CUR_CONT_REG,
			ICHG_MASK, reg_val);
}

static int bq24296_get_ibat_max(struct bq24296_chip *chip, int *mv)
{
	u8 reg_val = 0;
	int ret;

	NULL_CHECK(chip, -EINVAL);
	ret = bq24296_read_reg(chip->client, BQ02_CHARGE_CUR_CONT_REG, &reg_val);
	if (ret) {
		pr_err("failed to read BQ08_SYSTEM_STATUS_REG ret=%d\n", ret);
		return -EIO;
	}
	reg_val = reg_val >> 2;
	*mv = reg_val * IBAT_STEP_MA + IBAT_MIN_MA;
	return ret;
}

/* Use bq24296_get_adjust_ibat() instead of bq24296_get_ibat_max() */
static int bq24296_get_adjust_ibat(struct bq24296_chip *chip, int *mv)
{
	int ret, enable;
	NULL_CHECK(chip, -EINVAL);
	ret = bq24296_get_ibat_max(chip, mv);
	if (ret)
		return ret;
	bq24296_get_force_ichg_decrease(chip, &enable);
	if (enable)
		*mv /= 5;
	return 0;
}

#define VIN_LIMIT_MIN_MV  3880
#define VIN_LIMIT_MAX_MV  5080
#define VIN_LIMIT_STEP_MV  80
#if defined(CONFIG_CHARGER_UNIFIED_WLC)
/* G3 HW requirement */
/* In wireless charging, change Vin limit. */
#if		defined(CONFIG_MACH_MSM8974_G3_KR) || \
		defined(CONFIG_MACH_MSM8974_G3_GLOBAL_COM) || \
		defined(CONFIG_MACH_MSM8974_G3_SPR_US) || \
		defined(CONFIG_MACH_MSM8974_G3_USC_US) || \
		defined(CONFIG_MACH_MSM8974_G3_ACG_US) || \
		defined(CONFIG_MACH_MSM8974_G3_TMO_US)
#define VIN_LIMIT_WLC	4840	/* IDT IDT9025A(WPC) 4.84V */
#elif	defined(CONFIG_MACH_MSM8974_G3_VZW) || defined(CONFIG_MACH_MSM8974_G3_LRA)
#define VIN_LIMIT_WLC	5080	/* TI BQ51020(WPC) 5.08V */
#elif	defined(CONFIG_MACH_MSM8974_G3_ATT) || defined(CONFIG_MACH_MSM8974_G3_CA)
#define VIN_LIMIT_WLC	4520	/* TI BQ51221(PMA) 4.52V */
#else
#define VIN_LIMIT_WLC	4520	/* Other 4.52V(Charger default) */
#endif
#endif
static int bq24296_set_input_vin_limit(struct bq24296_chip *chip, int mv)
{
	u8 reg_val = 0;
	int set_vin = 0;
	NULL_CHECK(chip, -EINVAL);
	if (mv < VIN_LIMIT_MIN_MV)
		mv = VIN_LIMIT_MIN_MV;
	if (mv > VIN_LIMIT_MAX_MV)
		mv = VIN_LIMIT_MAX_MV;

	reg_val = (mv - VIN_LIMIT_MIN_MV)/VIN_LIMIT_STEP_MV;
	set_vin = reg_val * VIN_LIMIT_STEP_MV + VIN_LIMIT_MIN_MV;
	reg_val = reg_val << 3;

	pr_info("req_vin = %d set_vin = %d reg_val = 0x%02x\n",
				mv, set_vin, reg_val);

	return bq24296_masked_write(chip->client, BQ00_INPUT_SRC_CONT_REG,
			VINDPM_MASK, reg_val);
}

static int bq24296_get_input_vin_limit(struct bq24296_chip *chip, int *mv)
{
	u8 reg_val = 0;
	int ret;
	NULL_CHECK(chip, -EINVAL);

	ret = bq24296_read_reg(chip->client, BQ00_INPUT_SRC_CONT_REG, &reg_val);
	if (ret) {
		pr_err("failed to read BQ00_SYSTEM_STATUS_REG ret=%d\n", ret);
		return ret;
	}
	*mv = (reg_val & VINDPM_MASK >> 3) * VIN_LIMIT_STEP_MV +
		VIN_LIMIT_MIN_MV;
	return ret;
}

#define VBAT_MAX_MV  4400
#define VBAT_MIN_MV  3504
#define VBAT_STEP_MV  16
static int bq24296_set_vbat_max(struct bq24296_chip *chip, int mv)
{
	u8 reg_val = 0;
	int set_vbat = 0;
	NULL_CHECK(chip, -EINVAL);
	if (mv < VBAT_MIN_MV)
		mv = VBAT_MIN_MV;
	if (mv > VBAT_MAX_MV)
		mv = VBAT_MAX_MV;

	reg_val = (mv - VBAT_MIN_MV)/VBAT_STEP_MV;
	set_vbat = reg_val * VBAT_STEP_MV + VBAT_MIN_MV;
	reg_val = reg_val << 2;

	pr_info("req_vbat = %d set_vbat = %d reg_val = 0x%02x\n",
				mv, set_vbat, reg_val);

	return bq24296_masked_write(chip->client, BQ04_CHARGE_VOLT_CONT_REG,
			CHG_VOLTAGE_LIMIT_MASK, reg_val);
}

#define SYSTEM_VMIN_LOW_MV  3000
#define SYSTEM_VMIN_HIGH_MV  3700
#define SYSTEM_VMIN_STEP_MV  100
static int bq24296_set_system_vmin(struct bq24296_chip *chip, int mv)
{
	u8 reg_val = 0;
	int set_vmin = 0;
	NULL_CHECK(chip, -EINVAL);
	if (mv < SYSTEM_VMIN_LOW_MV)
		mv = SYSTEM_VMIN_LOW_MV;
	if (mv > SYSTEM_VMIN_HIGH_MV)
		mv = SYSTEM_VMIN_HIGH_MV;

	reg_val = (mv - SYSTEM_VMIN_LOW_MV)/SYSTEM_VMIN_STEP_MV;
	set_vmin = reg_val * SYSTEM_VMIN_STEP_MV + SYSTEM_VMIN_LOW_MV;
	reg_val = reg_val << 1;

	pr_info("req_vmin = %d set_vmin = %d reg_val = 0x%02x\n",
				mv, set_vmin, reg_val);

	return bq24296_masked_write(chip->client, BQ01_PWR_ON_CONF_REG,
			SYS_MIN_VOL_MASK, reg_val);
}

#define IPRECHG_MIN_MA  128
#define IPRECHG_MAX_MA  2048
#define IPRECHG_STEP_MA  128
static int bq24296_set_prechg_i_limit(struct bq24296_chip *chip, int ma)
{
	u8 reg_val = 0;
	int set_ma = 0;
	NULL_CHECK(chip, -EINVAL);

	if (ma < IPRECHG_MIN_MA)
		ma = IPRECHG_MIN_MA;
	if (ma > IPRECHG_MAX_MA)
		ma = IPRECHG_MAX_MA;


	reg_val = (ma - IPRECHG_MIN_MA)/IPRECHG_STEP_MA;
	set_ma = reg_val * IPRECHG_STEP_MA + IPRECHG_MIN_MA;
	reg_val = reg_val << 4;

	pr_info("req_i = %d set_i = %d reg_val = 0x%02x\n",
				ma, set_ma, reg_val);

	return bq24296_masked_write(chip->client, BQ03_PRE_CHARGE_TERM_CUR_REG,
			IPRECHG_MASK, reg_val);
}

#define ITERM_MIN_MA  128
#define ITERM_MAX_MA  2048
#define ITERM_STEP_MA  128
#if defined(CONFIG_CHARGER_UNIFIED_WLC)
/* G3 HW requirement */
/* In wireless charging, change Iterm. */
#if		defined(CONFIG_MACH_MSM8974_G3_KR) || \
		defined(CONFIG_MACH_MSM8974_G3_GLOBAL_COM) || \
		defined(CONFIG_MACH_MSM8974_G3_SPR_US) || \
		defined(CONFIG_MACH_MSM8974_G3_USC_US) || \
		defined(CONFIG_MACH_MSM8974_G3_ACG_US) || \
		defined(CONFIG_MACH_MSM8974_G3_TMO_US)
#define ITERM_MA_WLC	256		/* IDT IDT9025A(WPC) 256mA */
#elif	defined(CONFIG_MACH_MSM8974_G3_VZW) || defined(CONFIG_MACH_MSM8974_G3_LRA)
#define ITERM_MA_WLC	128		/* TI BQ51020(WPC) 128mA */
#elif	defined(CONFIG_MACH_MSM8974_G3_ATT) || defined(CONFIG_MACH_MSM8974_G3_CA)
#define ITERM_MA_WLC	256		/* TI BQ51221(PMA) 256mA */
#else
#define ITERM_MA_WLC	128		/* Other 128mA(Charger default) */
#endif
#endif
static int bq24296_set_term_current(struct bq24296_chip *chip, int ma)
{
	u8 reg_val = 0;
	int set_ma = 0;
	NULL_CHECK(chip, -EINVAL);
	if (ma < ITERM_MIN_MA)
		ma = ITERM_MIN_MA;
	if (ma > ITERM_MAX_MA)
		ma = ITERM_MAX_MA;

	reg_val = (ma - ITERM_MIN_MA)/ITERM_STEP_MA;
	set_ma = reg_val * ITERM_STEP_MA + ITERM_MIN_MA;

	pr_info("req_i = %d set_i = %d reg_val = 0x%02x\n",
				ma, set_ma, reg_val);

	return bq24296_masked_write(chip->client, BQ03_PRE_CHARGE_TERM_CUR_REG,
			ITERM_MASK, reg_val);
}

#define EN_TIMER_SHIFT 3
static int bq24296_set_chg_timer(struct bq24296_chip *chip, bool enable)
{
	int ret;
	u8 val = (u8)(!!enable << EN_TIMER_SHIFT);
	NULL_CHECK(chip, -EINVAL);

	pr_info("enable=%d\n", enable);
	safety_timer_enabled = enable;

	ret = bq24296_masked_write(chip->client, BQ05_CHARGE_TERM_TIMER_CONT_REG,
						EN_CHG_TIMER_MASK, val);
	if (ret) {
		pr_err("failed to set chg safety timer ret=%d\n", ret);
		return ret;
	}

	return 0;
}

#if !defined(CONFIG_MACH_MSM8974_G3_VZW)
#define CHG_TIMEOUT_SHIFT 1
static int bq24296_set_chg_timeout(struct bq24296_chip *chip)
{
	u8 reg_val = 1;
	NULL_CHECK(chip, -EINVAL);

	pr_info("req_chg_timeout set_h = 8hrs\n");

	chip->chg_timeout = false;

	reg_val = reg_val << CHG_TIMEOUT_SHIFT;

	return bq24296_masked_write(chip->client, BQ05_CHARGE_TERM_TIMER_CONT_REG,
			CHG_TIMER_MASK, reg_val);
}
#endif

#define EN_CHG_TERM_SHIFT 7
static int bq24296_set_chg_term(struct bq24296_chip *chip, bool enable)
{
	int ret;
	u8 val = (u8)(!!enable << EN_CHG_TERM_SHIFT);
	NULL_CHECK(chip, -EINVAL);

	pr_info("enable=%d\n", enable);

	ret = bq24296_masked_write(chip->client, BQ05_CHARGE_TERM_TIMER_CONT_REG,
						EN_CHG_TERM_MASK, val);
	if (ret) {
		pr_err("failed to disable chg term  ret=%d\n", ret);
		return ret;
	}

	return 0;

}

#define NOT_INIT_VBUS_UV 5000000
static int bq24296_get_usbin_adc(struct bq24296_chip *chip)
{
#ifdef CONFIG_SENSORS_QPNP_ADC_VOLTAGE
	struct qpnp_vadc_result results;
	int rc = 0;

	NULL_CHECK(chip, NOT_INIT_VBUS_UV);

	rc = qpnp_vadc_read(chip->vadc_dev, USBIN, &results);
	if (rc) {
		pr_err("Unable to read usbin adc rc=%d\n", rc);
		return NOT_INIT_VBUS_UV;
	} else {
		pr_debug("DC_IN voltage: %lld\n", results.physical);
		return results.physical;
	}
#else
	pr_err("CONFIG_SENSORS_QPNP_ADC_VOLTAGE is not defined.\n");
	return NOT_INIT_VBUS_UV;
#endif
}

static bool bq24296_is_charger_present(struct bq24296_chip *chip)
{
	int ret = 0;
	u8 sys_status, power_good;
	bool power_ok;

	NULL_CHECK(chip, false);
	ret = bq24296_read_reg(chip->client, BQ08_SYSTEM_STATUS_REG, &sys_status);
	if (ret) {
		pr_err("failed to read BQ08_SYSTEM_STATUS_REG ret=%d\n", ret);
		return false;
	}

	power_good = (sys_status & PG_STAT_MASK);
	sys_status &= VBUS_STAT_MASK;

	if ((power_good == 0) && (sys_status == 0 || sys_status == 0xC0)) {
		power_ok = false;
		pr_debug("DC is missing.\n");
	} else {
		power_ok = true;
		pr_debug("DC is present.\n");
	}

	return power_ok;
}

#ifdef CONFIG_BATFET_FORCE_CTRL
#define BATFET_DISABLE_SHIFT  5
static int bq24296_force_disable_batfet(struct bq24296_chip *chip, bool disable)
{
	int ret;
	u8 val = (u8)(!!disable << BATFET_DISABLE_SHIFT);
	NULL_CHECK(chip, -EINVAL);

	pr_info("disable=%d\n", disable);

	ret = bq24296_masked_write(chip->client, BQ07_MISC_OPERATION_CONT_REG,
						BATFET_DISABLE_MASK, val);
	if (ret) {
		pr_err("failed to set BATFET Disable ret=%d\n", ret);
		return ret;
	}

	return 0;
}
#endif

static int bq24296_force_ichg_decrease(struct bq24296_chip *chip, bool enable)
{
	int ret;
	u8 val = (u8)(!!enable);
	NULL_CHECK(chip, -EINVAL);

	pr_debug("enable=%d\n", enable);

	ret = bq24296_masked_write(chip->client, BQ02_CHARGE_CUR_CONT_REG,
			FORCE_20PCT_MASK, val);
	if (ret) {
		pr_err("failed to set FORCE_20PCT ret=%d\n", ret);
		return ret;
	}

	return 0;
}

static int bq24296_get_force_ichg_decrease(struct bq24296_chip *chip, int *enable)
{
	int ret;
	u8 val;
	NULL_CHECK(chip, -EINVAL);

	ret = bq24296_read_reg(chip->client, BQ02_CHARGE_CUR_CONT_REG, &val);
	if (ret) {
		pr_err("failed to get FORCE_20PCT ret=%d\n", ret);
		return ret;
	}
	*enable = (val & FORCE_20PCT_MASK) ? 1 : 0;

	return 0;
}

#define CHG_ENABLE_SHIFT  4
static int bq24296_enable_charging(struct bq24296_chip *chip, bool enable)
{
	int ret;
	u8 val;
	NULL_CHECK(chip, -EINVAL);

#ifdef CONFIG_LGE_PM_LLK_MODE
	if (chip->charging_disabled)
		enable = false;
#endif

	val = (u8)(!!enable << CHG_ENABLE_SHIFT);

	pr_info("enable=%d\n", enable);

	if (chip->chg_timeout) {
		pr_err("charging timeout state, never enabel charging\n");
		return 0;
	}

	ret = bq24296_masked_write(chip->client, BQ01_PWR_ON_CONF_REG,
						CHG_CONFIG_MASK, val);
	if (ret) {
		pr_err("failed to set CHG_CONFIG ret=%d\n", ret);
		return ret;
	}
#ifndef CONFIG_LGE_PM_LLK_MODE
	chip->charging_disabled = !enable;
#endif

	return 0;
}

static int __bq24296_get_prop_batt_present(struct bq24296_chip *chip)
{
	int temp = 0;
	bool batt_present;
	NULL_CHECK(chip, -EINVAL);

	temp = bq24296_get_batt_temp_origin();

	if (temp <= -300 || temp >= 790) {
		pr_err("\n\n  battery missing(%d) \n\n", temp);
		batt_present = 0;
	} else
		batt_present = 1;

	pr_info("present=%d, chip->batt_present=%d\n",
		batt_present ? 1 : 0, chip->batt_present);

	if (pseudo_batt_info.mode) {
		return 1;
	}
	return batt_present ? 1 : 0;
}

static int bq24296_get_prop_batt_present(struct bq24296_chip *chip)
{
	return chip->batt_present ? 1 :
		__bq24296_get_prop_batt_present(chip);
}

static void bq24296_chg_timeout(struct bq24296_chip *chip,
						bool chg_en)
{
	int ret;
	NULL_CHECK_VOID(chip);
	pr_info("charge safety timer expired!\n");
	cancel_delayed_work_sync(&chip->battemp_work);

	wake_lock(&chip->chg_timeout_lock);

	ret = bq24296_enable_charging(chip, chg_en);
	if (ret)
		pr_err("Failed to set CHG_CONFIG_MASK ret=%d\n", ret);

	chip->chg_timeout = true;

	schedule_delayed_work(&chip->battemp_work,
		MONITOR_BATTEMP_POLLING_PERIOD);
}

static int bootcompleted;
static int
bq24296_set_bootcompleted(const char *val, struct kernel_param *kp)
{
	int ret;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}
	pr_info(" %d\n", bootcompleted);

	return 0;
}
module_param_call(bootcompleted, bq24296_set_bootcompleted,
	param_get_uint, &bootcompleted, 0644);

struct current_limit_entry {
	int input_limit;
	int chg_limit;
};


#define ROUND_uA(x)	(x/10000)
#define ROUND_mA(x)	(x/10)
#define threshold 		ROUND_mA(4200)

static void bq24296_usbin_mon_worker(struct work_struct *work)
{
	unsigned int cur_dcin , conv_mv;
	struct bq24296_chip *chip =
		container_of(work, struct bq24296_chip, usbin_mon.work);
	union power_supply_propval val = {0,};

	NULL_CHECK_VOID(chip);
	if (!chip->usb_present) {
		goto out;
	}
	cur_dcin = bq24296_get_usbin_adc(chip);
	cur_dcin = ROUND_uA(cur_dcin);
	conv_mv = ROUND_mA(chip->icl_vbus_mv);

	if (cur_dcin >= conv_mv) {
		/* normal case
		  * VBUS is greater than DPM threshold
		  * Let charging of full current
		  * nothing to do
		  */
		  pr_info("\n\n normal case : %d\n\n", chip->usbin_ref_count);
		  if (chip->usbin_ref_count > 0) {
		/* check previous status
		 * if previous status is abnormal..
		 * reschedule work q
		 */
			goto out;
		  }

	} else if (cur_dcin > threshold && cur_dcin < conv_mv) {
		/* abnormal case #1
		  * VBUS is smaller than DPM threshold but greater than
		  * Verizon non-authority TA
		  * Let Charging as 800mA
		  */
		if (chip->last_usbin_mv <= cur_dcin) {
			/* working properly now I think..*/
			pr_info("\n\nDCIN Volt is good : last : %d  now : %d\n\n",
				chip->last_usbin_mv, cur_dcin);
			goto out;
		} else {
			 /* 64 is offeset of bq24296's charge current */
			mutex_lock(&chip->usbin_lock);
			val.intval = 900 - (chip->usbin_ref_count * 64) ;
			chip->usbin_ref_count++;
			chip->last_usbin_mv = cur_dcin;
			chip->ac_psy.set_event_property(&chip->ac_psy,
				POWER_SUPPLY_PROP_ABNORMAL_TA, &val);
			mutex_unlock(&chip->usbin_lock);
			schedule_delayed_work(&chip->usbin_mon, msecs_to_jiffies(1000));
			pr_info("\n\nabnormal TA detected..!set %dmA\n\n", val.intval);
			return;
		}
	} else if (cur_dcin <= threshold) {
		pr_info("\n\n abnormal charger!!! \n\n");
		return;
	}

out:
	mutex_lock(&chip->usbin_lock);
	chip->last_usbin_mv = ROUND_mA(chip->icl_vbus_mv);
	chip->usbin_ref_count = 0;
#if defined(CONFIG_VZW_POWER_REQ)
	chip->usbin_ref_count_vzw = 0;
#endif
	mutex_unlock(&chip->usbin_lock);
}

#if defined(CONFIG_LGE_PM_CHARGING_SUPPORT_PHIHONG)
static void phihong_timer_func_finish(unsigned long data)
{
	struct bq24296_chip *chip = (struct bq24296_chip *)data;
	if (chip->phihong == PHIHONG_YES) {
		pr_info("phihong detected!!!!\n");
	} else if (chip->phihong == PHIHONG_PERMANENT_YES) {
		pr_info("phihong activated!!!!\n");
	} else if (chip->usb_present) {
		chip->phihong = PHIHONG_NO;
	} else {
		chip->phihong = PHIHONG_PLUG_OUT;
	}
	complete(&chip->phihong_complete);
}
static void phihong_timer_func_start(unsigned long data)
{
	struct bq24296_chip *chip = (struct bq24296_chip *)data;
	chip->phihong = PHIHONG_VERIFYING;
	chip->phihong_timer.expires = jiffies + 4 * HZ;
	chip->phihong_timer.function = phihong_timer_func_finish;
	add_timer(&chip->phihong_timer);
	complete(&chip->phihong_complete);
	power_supply_changed(chip->psy_this);
}
static void trig_phihong_timer(struct bq24296_chip *chip)
{
	del_timer(&chip->phihong_timer);
	if (chip->phihong == PHIHONG_PERMANENT_YES ||
		chip->phihong == PHIHONG_NO) {
		phihong_timer_func_finish((unsigned long) chip);
		return;
	}
	chip->phihong_timer.expires = jiffies + 10 * HZ;
	chip->phihong_timer.function = phihong_timer_func_start;
	add_timer(&chip->phihong_timer);
}
static void cancel_phihong_timer(struct bq24296_chip *chip)
{
	del_timer(&chip->phihong_timer);
}
static __ref int do_phihong_checker(void *data)
{
	struct bq24296_chip *chip = (struct bq24296_chip *)data;
	static int count = 0;
	static int usb_present;
	while (!kthread_should_stop()) {
		wait_for_completion(&chip->phihong_complete);
		INIT_COMPLETION(chip->phihong_complete);
		usb_present = bq24296_is_charger_present(chip);
		switch (chip->phihong) {
		case PHIHONG_PLUG_OUT:
			count = 0;
			if (usb_present) {
				chip->phihong = PHIHONG_NOT_VERIFIED;
				trig_phihong_timer(chip);
			} else {
				cancel_phihong_timer(chip);
			}
			break;
		case PHIHONG_VERIFYING:
			if (!usb_present)
				chip->phihong = PHIHONG_VERIFYING_PLUG_OUT;
			break;
		case PHIHONG_VERIFYING_PLUG_OUT:
			if (usb_present)
				chip->phihong = PHIHONG_YES;
			break;
		case PHIHONG_YES:
			if (!usb_present) {
				chip->phihong = PHIHONG_PLUG_OUT;
			} else {
				if (count >= 3) {
					chip->phihong = PHIHONG_PERMANENT_YES;
				} else {
					trig_phihong_timer(chip);
					count++;
				}
			}
			break;
		case PHIHONG_PERMANENT_YES:
		case PHIHONG_NO:
		case PHIHONG_NO_NEED:
		case PHIHONG_NOT_VERIFIED:
			if (!usb_present) {
				chip->phihong = PHIHONG_PLUG_OUT;
			}
			break;
		default:
			chip->phihong = PHIHONG_PLUG_OUT;
			count = 0;
			break;
		}
		pr_debug("phihong = %d <<<<\n", chip->phihong);
	}
	return 0;

}
#endif

static void bq24296_irq_worker(struct work_struct *work)
{
	struct bq24296_chip *chip =
		container_of(work, struct bq24296_chip, irq_work.work);
	u8 reg08, reg09;
	int ret = 0, usb_present = 0;
#if defined(CONFIG_CHARGER_UNIFIED_WLC)
	int wlc_present = 0;
	union power_supply_propval wlc_ret = {0,};
#endif
	/* temporary debug for interrupts */
	pr_err("%s : occured\n", __func__);
	NULL_CHECK_VOID(chip);
#if defined(CONFIG_VZW_POWER_REQ)
	if (bq24296_is_en_hiz(chip))
		bq24296_set_en_hiz(chip, false);
#endif

	ret = bq24296_read_reg(chip->client, BQ08_SYSTEM_STATUS_REG, &reg08);
	if (ret)
		return;
	ret = bq24296_read_reg(chip->client, BQ09_FAULT_REG, &reg09);
	if (ret)
		return;

	pr_err("08:0x%02X, 09:0x%02X\n", reg08, reg09);

	if ((reg08 & VBUS_STAT_MASK) == VBUS_STAT_MASK)
		pr_info("otg detection!\n");
	else if (reg08 & BIT(7))
		pr_info("adapter port detected!\n");
	else if (reg08 & BIT(6))
		pr_info("usb host detected!\n");
	if ((reg08 & CHRG_STAT_MASK) == CHRG_STAT_MASK)
		pr_info("charging done!\n");
	else if (reg08 & FAST_CHARGE_MASK)
		pr_info("fast charging!\n");
	else if (reg08 & PRE_CHARGE_MASK)
		pr_info("pre-charging!\n");
	else
		pr_info("not charging!\n");
	if (reg08 & DPM_STAT_MASK)
		pr_info("dpm detected!\n");
	if (reg08 & PG_STAT_MASK)
		pr_info("power good!\n");
	if (reg08 & THERM_STAT_MASK)
		pr_info("thermal regulation!\n");
	if (reg08 & VSYS_STAT_MASK)
		pr_info("vsysmin regulation! battery is too low!\n");

	if (reg09 & BIT(6))
		pr_info("vbus ocp/ovp!\n");
	if ((reg09 & CHRG_FAULT_MASK) == CHRG_FAULT_MASK)
		bq24296_chg_timeout(chip, 0);
	else if (reg09 & BIT(5))
		pr_info("thermal shutdown!\n");
	else if (reg09 & BIT(4))
		pr_info("input fault!\n");
	if (reg09 & BIT(3))
		pr_info("battery ovp!\n");

	if (!bq24296_get_prop_batt_present(chip)) {
		bool charger = false;
		bool ftm_cable = is_factory_cable();

		wake_lock_timeout(&chip->battgone_wake_lock, HZ*10);
		msleep(2000); /* makes top-half i2c time margin */
		charger = bq24296_is_charger_present(chip);
		pr_info("battery removed %d-%d\n", charger, ftm_cable);
		if (charger && !ftm_cable) {
			cancel_delayed_work(&chip->irq_work);
			bq24296_enable_charging(chip, 0);
			switch_set_state(&chip->batt_removed, 1);
			power_supply_changed(&chip->batt_psy);
		}
		bq24296_set_vbat_max(chip, 3600);
	}

	usb_present = bq24296_is_charger_present(chip);
#if defined(CONFIG_CHARGER_UNIFIED_WLC)
	bq24296_charger_psy_getprop_event(chip, wlc_psy, WIRELESS_ONLINE,
		&wlc_ret, _WIRELESS_);
	wlc_present = wireless_charging || wlc_ret.intval;
	if ((chip->usb_present ^ usb_present) || (chip->wlc_present ^ wlc_present)) {
		chip->wlc_present = wlc_present;
#else
	if (chip->usb_present ^ usb_present) {
#endif
#if defined(CONFIG_LGE_PM_CHARGING_SUPPORT_PHIHONG)
		complete(&chip->phihong_complete);
#endif
#ifdef CONFIG_ZERO_WAIT
		zw_psy_irq_handler(usb_present);
#endif
#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
		cancel_delayed_work_sync(&chip->battemp_work);
		schedule_delayed_work(&chip->battemp_work, HZ*1);
		if (!usb_present &&
			wake_lock_active(&chip->lcs_wake_lock))
			wake_unlock(&chip->lcs_wake_lock);
#endif
		if (wake_lock_active(&chip->icl_wake_lock))
			wake_unlock(&chip->icl_wake_lock);

		wake_lock_timeout(&chip->uevent_wake_lock, HZ*2);
		if (usb_present == 0 && chip->icl_idx > 0 && bootcompleted) {
			pr_info("input_limit_worker exception\n");
		} else {
#if defined(CONFIG_CHARGER_UNIFIED_WLC)
			if (wlc_present) {
				pr_err("[WLC] set usb_present to 0 \n");
				chip->usb_present = 0;
			} else {
				chip->usb_present = usb_present;
			}
#else
			chip->usb_present = usb_present;
#endif
			pr_info("notify vbus to usb_present=%d\n", usb_present);
			power_supply_set_present(chip->usb_psy, chip->usb_present);
		}
	}
	pr_info("[IRQ_WORKER] plug %s, vin:%d\n",
		usb_present ? "in" : "out",
		bq24296_get_usbin_adc(chip));

	power_supply_changed(&chip->batt_psy);
}

static irqreturn_t bq24296_irq(int irq, void *dev_id)
{
	struct bq24296_chip *chip = dev_id;
	NULL_CHECK(chip, IRQ_NONE);

#ifdef I2C_SUSPEND_WORKAROUND
	schedule_delayed_work(&chip->check_suspended_work,
		msecs_to_jiffies(100));
#else
	schedule_delayed_work(&chip->irq_work, msecs_to_jiffies(100));
#endif

	return IRQ_HANDLED;
}

#ifdef I2C_SUSPEND_WORKAROUND
static void bq24296_check_suspended_worker(struct work_struct *work)
{
	struct bq24296_chip *chip =
		container_of(work, struct bq24296_chip, check_suspended_work.work);
	NULL_CHECK_VOID(chip);

	if (chip->suspend) {
		pr_debug("bq24296 suspended. try i2c operation after 100ms.\n");
		schedule_delayed_work(&chip->check_suspended_work, msecs_to_jiffies(100));
	} else {
		pr_debug("bq24296 resumed. do bq24296_irq.\n");
		schedule_delayed_work(&chip->irq_work, 0);
	}
}
#endif /* I2C_SUSPEND_WORKAROUND */

static int set_reg(void *data, u64 val)
{
	u32 addr = (u32) data;
	int ret;
	struct i2c_client *client = the_chip->client;

	ret = bq24296_write_reg(client, addr, (u8) val);

	return ret;
}

static int get_reg(void *data, u64 *val)
{
	u32 addr = (u32) data;
	u8 temp;
	int ret;
	struct i2c_client *client = the_chip->client;

	ret = bq24296_read_reg(client, addr, &temp);
	if (ret < 0)
		return ret;

	*val = temp;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(reg_fops, get_reg, set_reg, "0x%02llx\n");

#define OTG_ENABLE_SHIFT  5
static int bq24296_enable_otg(struct bq24296_chip *chip, bool enable)
{
	int ret;
	u8 val = (u8)(!!enable << OTG_ENABLE_SHIFT);

	pr_info("otg enable = %d\n", enable);
	NULL_CHECK(chip, -EINVAL);

	ret = bq24296_masked_write(chip->client, BQ01_PWR_ON_CONF_REG,
					OTG_ENABLE_MASK, val);
	if (ret) {
		pr_err("failed to set OTG_ENABLE_MASK rc=%d\n", ret);
		return ret;
	}

	return 0;
}

static bool bq24296_is_otg_mode(struct bq24296_chip *chip)
{
	u8 temp;
	int ret;
	NULL_CHECK(chip, -EINVAL);

	ret = bq24296_read_reg(chip->client, BQ01_PWR_ON_CONF_REG, &temp);
	if (ret) {
		pr_err("failed to read OTG_ENABLE_MASK rc=%d\n", ret);
		return false;
	}

	return !!(temp & OTG_ENABLE_MASK);
}

static char *bq24296_power_supplied_to[] = {
	"battery",
};

static enum power_supply_property bq24296_power_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_MAX,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED,
	POWER_SUPPLY_PROP_CHARGING_COMPLETE,
	POWER_SUPPLY_PROP_SAFTETY_CHARGER_TIMER,
};

static enum power_supply_property bq24296_batt_power_props[] = {
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_PSEUDO_BATT,
	POWER_SUPPLY_PROP_EXT_PWR_CHECK,
#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
	POWER_SUPPLY_PROP_BATTERY_ID_CHECKER,
#endif
#if defined(CONFIG_VZW_POWER_REQ)
	POWER_SUPPLY_PROP_VZW_CHG,
#endif
#if defined(CONFIG_LGE_PM_LLK_MODE)
	POWER_SUPPLY_PROP_STORE_DEMO_ENABLED,
#endif
};


static int bq24296_get_prop_charge_type(struct bq24296_chip *chip)
{
	int ret = 0;
	u8 sys_status;
	enum bq24296_chg_status status;
	int chg_type = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
#if defined(CONFIG_CHARGER_UNIFIED_WLC)
	union power_supply_propval wlc_ret = {0,};
#endif
	NULL_CHECK(chip, -EINVAL);
	ret = bq24296_read_reg(chip->client, BQ08_SYSTEM_STATUS_REG, &sys_status);
	if (ret) {
		pr_err("fail to read BQ08_SYSTEM_STATUS_REG. ret=%d\n", ret);
		chg_type = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
		goto exception_handling;
	}

	sys_status &= CHRG_STAT_MASK;
	if (sys_status == 0x10) {
		chg_type = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		status = BQ_CHG_STATUS_PRE_CHARGE;
	} else if (sys_status == 0x20) {
		chg_type = POWER_SUPPLY_CHARGE_TYPE_FAST;
		status = BQ_CHG_STATUS_FAST_CHARGE;
	} else if (sys_status == 0x30) {
		chg_type = POWER_SUPPLY_CHARGE_TYPE_NONE;
		status = BQ_CHG_STATUS_FULL;
	} else {
		chg_type = POWER_SUPPLY_CHARGE_TYPE_NONE;
		status = BQ_CHG_STATUS_NONE;
	}
	pr_debug("bq-chg-status (%d=%s).\n", status, bq24296_chg_status[status]);
	if (chip->chg_status != status) {
		if (status == BQ_CHG_STATUS_NONE
			|| status == BQ_CHG_STATUS_FULL) {
			pr_debug("Charging stopped.\n");
#if defined(CONFIG_CHARGER_UNIFIED_WLC)
			if (chip->wlc_present && ((sys_status & FAST_CHARGE_MASK) && (sys_status & PRE_CHARGE_MASK))) {
				bq24296_charger_psy_setprop_event(chip, wlc_psy,
					WIRELESS_CHARGE_COMPLETED,
					wlc_ret.intval, _WIRELESS_);
			}
#endif
			wake_unlock(&chip->chg_wake_lock);
		} else {
			pr_debug("Charging started.\n");
			wake_lock(&chip->chg_wake_lock);
		}
		chip->chg_status = status;
	}
#if defined(CONFIG_CHARGER_UNIFIED_WLC)
	if (gpio_get_value(chip->otg_en) == 1
		&& chip->wlc_otg_status == FAKE_DISCONNECTION){
		//pr_info("\n\n\n[ FAKE_DISCONNECTION ]\n\n\n");
		chg_type = BQ_CHG_STATUS_NONE;
	}
#endif
	return chg_type;

exception_handling:
	chip->chg_status = BQ_CHG_STATUS_EXCEPTION;
	if (wake_lock_active(&chip->chg_wake_lock)) {
		pr_err("exception_handling : unlock chg_wake_lock.\n");
		wake_unlock(&chip->chg_wake_lock);
	}
	return chg_type;

}

static int bq24296_get_prop_batt_health(struct bq24296_chip *chip)
{
	NULL_CHECK(chip, -EINVAL);
#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
	if (chip->btm_state == BTM_HEALTH_OVERHEAT)
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	if (chip->btm_state == BTM_HEALTH_COLD)
		return POWER_SUPPLY_HEALTH_COLD;
	else
		return POWER_SUPPLY_HEALTH_GOOD;
#else
	int batt_temp;
	batt_temp = bq24296_get_prop_batt_temp(chip);

	if (batt_temp >= 550)
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	if (batt_temp <= -100)
		return POWER_SUPPLY_HEALTH_COLD;
	else
		return POWER_SUPPLY_HEALTH_GOOD;
#endif
}

#define DEFAULT_VOLTAGE		4000000
static int bq24296_get_prop_batt_voltage_now(void)
{
#ifdef CONFIG_MAX17048_FUELGAUGE
	int voltage = 0;
	voltage = max17048_get_voltage() * 1000;
	return voltage;
#else
	pr_err("CONFIG_MAX17048_FUELGAUGE is not defined.\n");
	return DEFAULT_VOLTAGE;
#endif
}

#define DEFAULT_TEMP		250
int bq24296_get_batt_temp_origin(void)
{
#ifdef CONFIG_SENSORS_QPNP_ADC_VOLTAGE
	int rc = 0;
	struct qpnp_vadc_result results;

	NULL_CHECK(the_chip, -EINVAL);

	rc = qpnp_vadc_read(the_chip->vadc_dev, LR_MUX1_BATT_THERM, &results);
	if (rc) {
		pr_debug("Unable to read batt temperature rc=%d\n", rc);
		pr_debug("Report last_bat_temp %d again\n", last_batt_temp);
		return last_batt_temp;
	} else {
		pr_debug("get_bat_temp %d %lld\n", results.adc_code, results.physical);
		last_batt_temp = (int)results.physical;
		return (int)results.physical;
	}
#else
	pr_err("CONFIG_SENSORS_QPNP_ADC_VOLTAGE is not defined.\n");
	return DEFAULT_TEMP;
#endif
}
EXPORT_SYMBOL(bq24296_get_batt_temp_origin);

static int bq24296_get_prop_batt_temp(struct bq24296_chip *chip)
{
	NULL_CHECK(chip, -EINVAL);
	if (pseudo_batt_info.mode) {
		pr_debug("battery fake mode : %d \n", pseudo_batt_info.mode);
		return pseudo_batt_info.temp * 10;
	} else if (is_factory_cable()) {
		pr_debug("factory cable : %d \n", DEFAULT_TEMP / 10);
		return DEFAULT_TEMP;
	}

	/* Approach adc channel for read batt temp' */
#ifdef CONFIG_SENSORS_QPNP_ADC_VOLTAGE
	return bq24296_get_batt_temp_origin();
#else
	pr_err("CONFIG_SENSORS_QPNP_ADC_VOLTAGE is not defined.\n");
	return DEFAULT_TEMP;
#endif
}


#define DEFAULT_CAPACITY	50
static int bq24296_get_prop_batt_capacity(struct bq24296_chip *chip)
{
	NULL_CHECK(chip, -EINVAL);
#ifdef CONFIG_MAX17048_FUELGAUGE
	return max17048_get_capacity();
#else
	pr_err("CONFIG_MAX17048_FUELGAUGE is not defined.\n");
	return DEFAULT_CAPACITY;
#endif
}

#define DEFAULT_CURRENT		200000
static int bq24296_get_prop_batt_current_now(struct bq24296_chip *chip)
{
	int batt_current = 0;
#if defined(CONFIG_LGE_CURRENTNOW)
	union power_supply_propval ret = {0,};

	if (!chip->cn_psy)
		return DEFAULT_CURRENT;
	chip->cn_psy->get_property(chip->cn_psy,
		POWER_SUPPLY_PROP_PRESENT, &ret);
	if (!ret.intval)
		return DEFAULT_CURRENT;
	chip->cn_psy->get_property(chip->cn_psy,
		POWER_SUPPLY_PROP_CURRENT_NOW, &ret);
	batt_current = ret.intval;
#else
	if (bq24296_get_prop_charge_type(chip) >
		POWER_SUPPLY_CHARGE_TYPE_NONE) {
		bq24296_get_adjust_ibat(chip, &batt_current);
	} else {
		batt_current = 0;
	}
#endif
	return batt_current;
}
#define DEFAULT_FULL_DESIGN	2500
static int bq24296_get_prop_batt_full_design(struct bq24296_chip *chip)
{
	NULL_CHECK(chip, -EINVAL);
#ifdef CONFIG_MAX17048_FUELGAUGE
	return max17048_get_fulldesign();
#else
	pr_err("CONFIG_MAX17048_FUELGAUGE is not defined.\n");
	return DEFAULT_FULL_DESIGN;
#endif
}

static int bq24296_get_prop_batt_status(struct bq24296_chip *chip)
{
	int chg_type = bq24296_get_prop_charge_type(chip);
	int batt_present = bq24296_get_prop_batt_present(chip);
	int capacity = bq24296_get_prop_batt_capacity(chip);

	if (capacity >= 100 && batt_present
		&& bq24296_is_charger_present(chip))
		return POWER_SUPPLY_STATUS_FULL;

#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
	if (bq24296_is_charger_present(chip) && chip->pseudo_ui_chg)
		return POWER_SUPPLY_STATUS_CHARGING;
#endif

	if (chg_type == POWER_SUPPLY_CHARGE_TYPE_TRICKLE ||
		chg_type == POWER_SUPPLY_CHARGE_TYPE_FAST)
		return POWER_SUPPLY_STATUS_CHARGING;

	if (chip->usb_present)
		return POWER_SUPPLY_STATUS_NOT_CHARGING;

	return POWER_SUPPLY_STATUS_DISCHARGING;
}

static int bq24296_batt_power_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct bq24296_chip *chip = container_of(psy,
					struct bq24296_chip, batt_psy);
	NULL_CHECK(chip, -EINVAL);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bq24296_get_prop_batt_status(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = bq24296_get_prop_charge_type(chip);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = bq24296_get_prop_batt_health(chip);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = bq24296_get_prop_batt_present(chip);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = 4350 * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = 4350 * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if (pseudo_batt_info.mode) {
			val->intval = pseudo_batt_info.volt;
			break;
		}
		val->intval = bq24296_get_prop_batt_voltage_now();
		break;
	case POWER_SUPPLY_PROP_TEMP:
		if (pseudo_batt_info.mode) {
			val->intval = pseudo_batt_info.temp;
			break;
		}
		val->intval = bq24296_get_prop_batt_temp(chip);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (pseudo_batt_info.mode) {
			val->intval = pseudo_batt_info.capacity;
			break;
		}
		val->intval = bq24296_get_prop_batt_capacity(chip);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = bq24296_get_prop_batt_current_now(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = bq24296_get_prop_batt_full_design(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = bq24296_get_prop_charge_type(chip)
				!= POWER_SUPPLY_CHARGE_TYPE_NONE;
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		/* it makes ibat max set following themral mitigation.
		 * But, SMB349 cannot control ibat current like PMIC.
		 * if LGE charging scenario make charging thermal control,
		 * it is good interface to use LG mitigation level.
		 */
		val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_PSEUDO_BATT:
		val->intval = pseudo_batt_info.mode;
		break;
	case POWER_SUPPLY_PROP_EXT_PWR_CHECK:
		val->intval = lge_pm_get_cable_type();
		break;
#if defined(CONFIG_LGE_PM_BATTERY_ID_CHECKER)
	case POWER_SUPPLY_PROP_BATTERY_ID_CHECKER:
		if (is_factory_cable() && bq24296_is_charger_present(chip))
			val->intval = 1;
		else
			val->intval = chip->batt_id_smem;
		break;
#endif
#if defined(CONFIG_VZW_POWER_REQ)
	case POWER_SUPPLY_PROP_VZW_CHG:
		val->intval = chip->vzw_chg_mode;
		break;
#endif
#if defined(CONFIG_LGE_PM_LLK_MODE)
	case POWER_SUPPLY_PROP_STORE_DEMO_ENABLED:
		val->intval = chip->store_demo_enabled;
		if (val->intval)
			llk_mode = true;
		else
			llk_mode = false;
	break;
#endif
	default:
		return -EINVAL;
	}

	return 0;
}

static int bq24296_batt_power_set_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  const union power_supply_propval *val)
{
	struct bq24296_chip *chip = container_of(psy,
					struct bq24296_chip, batt_psy);
	NULL_CHECK(chip, -EINVAL);
	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
#if defined(CONFIG_LGE_PM_LLK_MODE)
		chip->charging_disabled = !val->intval;
	if (llk_mode) {
		if (val->intval) {
			battemp_work_cancel = false;
		} else {
			battemp_work_cancel = true;
		}
	}
#endif
		bq24296_enable_charging(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		chip->batt_present = !!(val->intval);
		printk("\nBattery %s!!!\n\n",
			val->intval ? "inserted" : "removed");
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		/* it makes ibat max set following themral mitigation.
		 * But, SMB349 cannot control ibat current like PMIC.
		 * if LGE charging scenario make charging thermal control,
		 * it is good interface to use LG mitigation level.
		 */
		break;
#if defined CONFIG_VZW_POWER_REQ
	case POWER_SUPPLY_PROP_VZW_CHG:
		#define vzw_max_lmt (500)
		#define vzw_cc (400)
		/* limit */
		bq24296_set_input_i_limit(chip, vzw_max_lmt);
		/* charging current */
		bq24296_set_adjust_ibat(chip, vzw_cc);
		pr_info("\n\nadjust charging current of slow charging : 400mA\n\n");
		power_supply_changed(&chip->batt_psy);

		#undef vzw_max_lmt
		#undef vzw_cc
		break;
#endif
#if defined(CONFIG_LGE_PM_LLK_MODE)
	case POWER_SUPPLY_PROP_STORE_DEMO_ENABLED:
		chip->store_demo_enabled = val->intval;
		break;
#endif
	default:
		return -EINVAL;
	}

	power_supply_changed(&chip->batt_psy);

	return 0;
}


static int
bq24296_batt_power_property_is_writeable(struct power_supply *psy,
						enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		return 1;
	default:
		break;
	}

	return 0;
}
#if 0
static int bq24296_get_dpm_stat(struct bq24296_chip *chip, int* dpm)
{
	u8 reg_val = 0;
	int ret;

	NULL_CHECK(chip, -EINVAL);
	ret = bq24296_read_reg(chip->client, BQ08_SYSTEM_STATUS_REG, &reg_val);
	if (ret) {
		pr_err("failed to read BQ08_SYSTEM_STATUS_REG ret=%d\n", ret);
		return -EIO;
	}
	*dpm = (reg_val & DPM_STAT_MASK) >> 3;
	return ret;
}

static bool bq24296_is_dpm_enabled(struct bq24296_chip *chip)
{
	int dpm;
	bq24296_get_dpm_stat(chip, &dpm);
	return dpm;
}
#endif

#if defined(CONFIG_LGE_PM_CHARGING_SUPPORT_PHIHONG)
static void bq24296_set_phihong_current(struct bq24296_chip *chip, int ma)
{
#if defined(CONFIG_CHARGER_UNIFIED_WLC)
	union power_supply_propval wlc_ret = {0,};
	if (wireless_charging) {
		bq24296_charger_psy_getprop_event(chip, wlc_psy,
			WIRELESS_ONLINE, &wlc_ret, _WIRELESS_);
		if (wlc_ret.intval) {
			ma = INPUT_CURRENT_LIMIT_WLC;
			pr_info("[WLC] Change dwc3 result to %dmA\n", ma);
		}
	}
#endif

	switch (chip->phihong) {
	case PHIHONG_PLUG_OUT:
	case PHIHONG_NOT_VERIFIED:
	case PHIHONG_VERIFYING_PLUG_OUT:
	case PHIHONG_YES:
	case PHIHONG_PERMANENT_YES:
		if (chip->pre_input_current_ma < ma)
			bq24296_charger_psy_setprop(chip, psy_this,
				INPUT_CURRENT_MAX, chip->pre_input_current_ma);
		else
			bq24296_charger_psy_setprop(chip, psy_this,
				INPUT_CURRENT_MAX, ma);
		break;
	case PHIHONG_VERIFYING:
	default:
		bq24296_charger_psy_setprop(chip, psy_this, INPUT_CURRENT_MAX,
			ma);
		break;
	}
}
#endif
#if defined(CONFIG_CHARGER_UNIFIED_WLC)
static void bq24296_wlc_otg_fake_proc(struct bq24296_chip *chip)
{
	uint8_t otg_status = gpio_get_value(chip->otg_en);
	union power_supply_propval wlc_ret = {0,};

	NULL_CHECK_VOID(chip);
	bq24296_charger_psy_getprop_event(chip, wlc_psy, WIRELESS_ONLINE,
		&wlc_ret, _WIRELESS_);

	if (!wlc_ret.intval)
		return;

	if (otg_status == 1 && chip->wlc_otg_status == FAKE_UNKNOWN) {
		/* fake remove wlc */
		wlc_ret.intval = FAKE_DISCONNECTION;
		chip->wlc_otg_status = FAKE_DISCONNECTION;
		chip->wlc_psy->set_event_property(chip->wlc_psy,
					POWER_SUPPLY_PROP_WIRELESS_FAKE_OTG, &wlc_ret);
		bq24296_enable_charging(chip, false);
		//pr_info("\n\n [FAKE DISCONNECTING....]\n\n");
	} else if (otg_status == 0 && chip->wlc_otg_status == FAKE_DISCONNECTION) {
		/* fake online wlc */
		wlc_ret.intval = FAKE_CONNECTING;
		chip->wlc_otg_status = FAKE_UNKNOWN;
		chip->wlc_psy->set_event_property(chip->wlc_psy,
					POWER_SUPPLY_PROP_WIRELESS_FAKE_OTG, &wlc_ret);
		bq24296_enable_charging(chip, true);
		//pr_info("\n\n [FAKE ONLINE...]\n\n");

	} else {
		/* nothing to do */
	}
}
#endif
static void bq24296_decide_otg_mode(struct bq24296_chip *chip)
{
	union power_supply_propval ret = {0,};
#if defined(CONFIG_CHARGER_UNIFIED_WLC)
	union power_supply_propval wlc_ret = {0,};
#endif
	hw_rev_type boardrev = lge_get_board_revno();

	NULL_CHECK_VOID(chip);

	chip->usb_psy->get_property(chip->usb_psy,
					  POWER_SUPPLY_PROP_SCOPE, &ret);
#if defined(CONFIG_CHARGER_UNIFIED_WLC)
	chip->wlc_psy = _psy_check_ext(chip->wlc_psy, _WIRELESS_);
	NULL_CHECK_VOID(chip->wlc_psy);
#endif
	if (boardrev < HW_REV_1_0) {
		if (ret.intval == POWER_SUPPLY_SCOPE_SYSTEM
					&& !bq24296_is_otg_mode(chip)) {
			gpio_set_value(chip->otg_en, 1);
			bq24296_enable_otg(chip, true);
		} else if (ret.intval == POWER_SUPPLY_SCOPE_DEVICE
			&& bq24296_is_otg_mode(chip)) {
			gpio_set_value(chip->otg_en, 0);
			bq24296_enable_otg(chip, false);
		}
	} else {
#if defined(CONFIG_CHARGER_UNIFIED_WLC)
		bq24296_charger_psy_getprop_event(chip, wlc_psy,
			WIRELESS_ONLINE, &wlc_ret, _WIRELESS_);
		if (wlc_ret.intval) {
			if (ret.intval == POWER_SUPPLY_SCOPE_SYSTEM
					&& !gpio_get_value(chip->otg_en)) {
				gpio_set_value(chip->otg_en, 1);
				bq24296_enable_otg(chip, true);
			} else if (ret.intval == POWER_SUPPLY_SCOPE_DEVICE
					&& gpio_get_value(chip->otg_en)) {
				gpio_set_value(chip->otg_en, 0);
				bq24296_enable_otg(chip, false);
			}
			bq24296_wlc_otg_fake_proc(chip);
		} else {
			if (ret.intval == POWER_SUPPLY_SCOPE_SYSTEM
						&& !bq24296_is_otg_mode(chip)) {
				gpio_set_value(chip->otg_en, 1);
				bq24296_enable_otg(chip, true);
			} else if (ret.intval == POWER_SUPPLY_SCOPE_DEVICE
					&& bq24296_is_otg_mode(chip)) {
				gpio_set_value(chip->otg_en, 0);
				bq24296_enable_otg(chip, false);
			}
		}
#else
		if (ret.intval == POWER_SUPPLY_SCOPE_SYSTEM
							&& !bq24296_is_otg_mode(chip)) {
				gpio_set_value(chip->otg_en, 1);
				bq24296_enable_otg(chip, true);
		} else if (ret.intval == POWER_SUPPLY_SCOPE_DEVICE
					&& bq24296_is_otg_mode(chip)) {
				gpio_set_value(chip->otg_en, 0);
				bq24296_enable_otg(chip, false);
		}
#endif
	}
}

#if defined(CONFIG_VZW_POWER_REQ)

/* ADC_TO_IINMAX: IINMAX = (1V/RILIM) x KLIM
 * VZW_UNDER_CURRENT_CHARGING_MA: Threshold current level
 *	to determine under current which is VZW requesting
 * VZW_UNDER_CURRENT_CHARGING_DETECT_MV: For CC mode charging,
 *	this routine mis-understanding normal CC mode charging
 *	as under current charging. So will not detect under
 *	current charging over this number.
 */

#define	ADC_TO_IINMAX(x) (((int)(x)*198)/100)
#define VZW_UNDER_CURRENT_CHARGING_MA	400000
#define VZW_UNDER_CURRENT_CHARGING_A	(VZW_UNDER_CURRENT_CHARGING_MA/1000)
#define VZW_UNDER_CURRENT_CHARGING_DETECT_MV	4200000
static void VZW_CHG_director(struct bq24296_chip *chip)
{
	struct qpnp_vadc_result result;
	struct power_supply *psy = NULL;
	union power_supply_propval val = {0, };

	bq24296_charger_psy_getprop(chip, psy_this, PRESENT, &val);
	if (!val.intval)
		goto normal_charger;

	if (chip->usb_psy->is_usb_driver_uninstall) {
		chip->vzw_chg_mode = VZW_USB_DRIVER_UNINSTALLED;
		pr_info("VZW usb driver uninstall detected!!\n");
		goto exit;
	}

	/* Invalid charger detect */
	if (lge_get_board_revno() < HW_REV_1_0)
		goto normal_charger;
	if (chip->usb_psy->is_floated_charger) {
		chip->vzw_chg_mode = VZW_NOT_CHARGING;
		pr_info("VZW invalid charging detected!!\n");
		goto exit;
	}

	/* slow charger detect */
	psy = _psy_check_ext(psy, _BATTERY_);

	if (likely(psy))
		psy->get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
	else
		goto normal_charger;
	if (val.intval > VZW_UNDER_CURRENT_CHARGING_DETECT_MV)
		goto normal_charger;
	bq24296_charger_psy_getprop(chip, psy_this, CURRENT_MAX, &val);
	if (val.intval < VZW_UNDER_CURRENT_CHARGING_A)
		goto normal_charger;
	bq24296_charger_psy_getprop(chip, psy_this, CHARGING_ENABLED, &val);
	if (!val.intval)
		goto normal_charger;
	bq24296_charger_psy_getprop(chip, usb_psy, TYPE, &val);
	if (val.intval != POWER_SUPPLY_TYPE_USB_DCP)
		goto normal_charger;
	qpnp_vadc_read(chip->vadc_dev, LR_MUX4_AMUX_THM1, &result);
	if (ADC_TO_IINMAX(result.physical) < VZW_UNDER_CURRENT_CHARGING_MA) {
		chip->vzw_chg_mode = VZW_UNDER_CURRENT_CHARGING;
		pr_info("VZW slow charging detected!!\n");
		goto exit;
	}

normal_charger:
	bq24296_charger_psy_getprop(chip, psy_this, PRESENT, &val);
	if (val.intval)
		chip->vzw_chg_mode = VZW_NORMAL_CHARGING;
	else
		chip->vzw_chg_mode = VZW_NO_CHARGER;
exit:
	return;
}
#endif

static void bq24296_batt_external_power_changed(struct power_supply *psy)
{
	struct bq24296_chip *chip = container_of(psy,
					struct bq24296_chip, batt_psy);
	union power_supply_propval ret = {0,};
#if defined(CONFIG_CHARGER_UNIFIED_WLC)
	union power_supply_propval wlc_ret = {0,};
#endif
	int	busy_wait = 20;
	NULL_CHECK_VOID(chip);

	pr_debug("\n");
	/*
	     Kernel crash patch due to i2c failure
	     i2c oeration is established by someone before
		 resuming i2c client.
		     <workQ pending -> kernel resume -> WorkQ resumt
		     -> invoke power supply changed
	*/
	if (chip->suspend) {
		/* In suspend, busy waiting... till resume */
		do {
			if (!chip->suspend || busy_wait == 0)
				break;
			--busy_wait;
			/* max 400ms */
			msleep(20);
			pr_info("\n\n busy waiting.... till resume \n\n");
		} while (true);
	}

	bq24296_charger_psy_getprop(chip, usb_psy, CURRENT_MAX, &ret);

	ret.intval = ret.intval / 1000; /* dwc3 treats uA */
	pr_info("dwc3 result=%dmA\n", ret.intval);

#if defined(CONFIG_LGE_PM_CHARGING_SUPPORT_PHIHONG)
	complete(&chip->phihong_complete);
	bq24296_set_phihong_current(chip, ret.intval);

	/* For MST, boost current up over 900mA in spite of USB */
	if (safety_timer_enabled == 0 && ret.intval < 900) {
		ret.intval = 900;
		bq24296_charger_psy_setprop(chip, psy_this, INPUT_CURRENT_MAX, ret.intval);
		pr_info("safety timer disabled.... input current limit = %d\n",ret.intval);
	}
#else
#if defined(CONFIG_CHARGER_UNIFIED_WLC)
	if (wireless_charging) {
		bq24296_charger_psy_getprop_event(chip, wlc_psy,
			WIRELESS_ONLINE, &wlc_ret, _WIRELESS_);
		if (wlc_ret.intval) {
			ret.intval = INPUT_CURRENT_LIMIT_WLC;
			pr_info("[WLC] Change dwc3 result to %dmA\n", ret.intval);
		}
	}
#endif
	/* For MST, boost current up over 900mA in spite of USB */
	if (pseudo_batt_info.mode && ret.intval < 900)
		ret.intval = 900;

	bq24296_charger_psy_setprop(chip, psy_this, INPUT_CURRENT_MAX, ret.intval);
#endif
	bq24296_charger_psy_setprop(chip, psy_this, CURRENT_MAX, chip->chg_current_ma);

	chip->usb_psy = _psy_check_ext(chip->usb_psy, _USB_);
	NULL_CHECK_VOID(chip->usb_psy);

	/*
	  * FLOATED_CHARGER is under control of USB
	  * below function is only for	TA (D+/D- short) -> online  is 1
	  * Floated charger -> online is ZERO
	*/
	bq24296_charger_psy_getprop(chip, usb_psy, ONLINE, &ret);
	if (_psy_check_ext(chip->psy_this, _THIS_) > 0) {
		if (ret.intval) {
			if (likely(delayed_work_pending(&chip->usbin_mon))) {
				cancel_delayed_work_sync(&chip->usbin_mon);
			}
			schedule_delayed_work(&chip->usbin_mon, msecs_to_jiffies(10));
		}
	}
	bq24296_decide_otg_mode(chip);
	/* 2 steps lower in case of factory cable */
	if (is_factory_cable())
		bq24296_set_input_vin_limit(chip,
			chip->vin_limit_mv - VIN_LIMIT_STEP_MV * 8);
	else
		bq24296_set_input_vin_limit(chip, chip->vin_limit_mv);

#if defined(CONFIG_CHARGER_UNIFIED_WLC)
	if (wireless_charging) {
		bq24296_charger_psy_getprop_event(chip, wlc_psy,
			WIRELESS_ONLINE, &wlc_ret, _WIRELESS_);
		if (wlc_ret.intval) {
			pr_info("[WLC] Set input voltage limit %d mV\n", VIN_LIMIT_WLC);
			bq24296_set_input_vin_limit(chip, VIN_LIMIT_WLC);

			bq24296_charger_psy_getprop_event(chip, wlc_psy,
				WIRELESS_THERMAL_MITIGATION, &wlc_ret, _WIRELESS_);
			if (wlc_ret.intval == THERMALE_NOT_TRIGGER ||
				wlc_ret.intval == THERMALE_ALL_CLEAR) {
				pr_info("[WLC] Set inuput current limit %d mA\n", INPUT_CURRENT_LIMIT_WLC);
				bq24296_set_input_i_limit(chip, INPUT_CURRENT_LIMIT_WLC);

				pr_info("[WLC] Set charging current %d mA\n", IBAT_WLC);
				bq24296_set_adjust_ibat(chip, IBAT_WLC);
			} else {
				pr_info("[WLC] Set inuput current limit %d mA\n", chip->wlc_input_current_te);
				bq24296_set_input_i_limit(chip, chip->wlc_input_current_te);

				pr_info("[WLC] Set charging current %d mA\n", chip->wlc_chg_current_te);
				bq24296_set_adjust_ibat(chip, chip->wlc_chg_current_te);
			}
		} else {
			pr_err("[WLC] GPIO SWING\n");
		}
	}
#endif

#if defined(CONFIG_VZW_POWER_REQ)
	VZW_CHG_director(chip);
#endif
	power_supply_changed(&chip->batt_psy);
}

static int bq24296_power_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct bq24296_chip *chip = container_of(psy,
						struct bq24296_chip,
						ac_psy);
	NULL_CHECK(chip, -EINVAL);
	switch (psp) {

	case POWER_SUPPLY_PROP_CURRENT_MAX:
		bq24296_get_adjust_ibat(chip, &val->intval);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
		bq24296_get_input_i_limit(chip, &val->intval);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
#if defined(CONFIG_LGE_PM_CHARGING_SUPPORT_PHIHONG)
		if (CHECK_PHIHONG(chip)) {
			val->intval = 1;
			break;
		}
#endif
		val->intval = bq24296_is_charger_present(chip);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = bq24296_get_prop_charge_type(chip)
				!= POWER_SUPPLY_CHARGE_TYPE_NONE;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = bq24296_get_prop_charge_type(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = bq24296_get_prop_charge_type(chip)
				!= POWER_SUPPLY_CHARGE_TYPE_NONE;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED:
		bq24296_get_input_vin_limit(chip, &val->intval);
		break;
	case POWER_SUPPLY_PROP_SAFTETY_CHARGER_TIMER:
	{
		int ret;
		u8 value = 0;
		ret = bq24296_read_reg(chip->client, BQ05_CHARGE_TERM_TIMER_CONT_REG, &value);
		if (ret) {
			pr_err("failed to read BQ05_CHARGE_TERM_TIMER_CONT_REG ret=%d\n", ret);
			return -EINVAL;
		}
		val->intval = (value >> 3) & 0x01;
		pr_info("get charger_timeout : %d[D]\n", val->intval);
	}
		break;
	case POWER_SUPPLY_PROP_CHARGING_COMPLETE:
		if (bq24296_get_prop_batt_capacity(chip) == 100)
			val->intval = 0;
		else
			val->intval = 1;

		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int bq24296_power_set_property(struct power_supply *psy,
					enum power_supply_property psp,
					const union power_supply_propval *val)
{
	struct bq24296_chip *chip = container_of(psy,
						struct bq24296_chip,
						ac_psy);
	NULL_CHECK(chip, -EINVAL);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		bq24296_enable_charging(chip, val->intval);
		chip->ac_online = val->intval;
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
#ifdef CONFIG_LGE_PM_LLK_MODE
		chip->charging_disabled = !val->intval;
#endif
		bq24296_enable_charging(chip, val->intval);
		power_supply_changed(&chip->batt_psy);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		bq24296_set_adjust_ibat(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
		bq24296_set_input_i_limit(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED:
		bq24296_force_ichg_decrease(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_SAFTETY_CHARGER_TIMER:
#if !defined(CONFIG_CHARGER_FACTORY_MODE)
		bq24296_set_chg_timer(chip, ((val->intval == 0) ? false : true));
		pr_info("charger_timeout : %d[D]\n", val->intval);
#endif
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int bq24296_power_get_event_property(struct power_supply *psy,
					enum power_supply_event_type psp,
					union power_supply_propval *val)
{
	struct bq24296_chip *chip = container_of(psy,
						struct bq24296_chip,
						ac_psy);

	NULL_CHECK(chip, -EINVAL);

	switch (psp) {
#if defined(CONFIG_CHARGER_UNIFIED_WLC)
	case POWER_SUPPLY_PROP_WIRELESS_ONLINE_OTG:
		pr_info("[WLC] get wlc otg mode : %d\n", chip->wlc_otg);
		val->intval = chip->wlc_otg;
		break;
	case POWER_SUPPLY_PROP_WIRELESS_DCIN_PRESENT:
		if (!chip) {
			pr_err("called before init\n");
			return -EINVAL;
		}
		val->intval = (int)bq24296_is_charger_present(chip);
		if (val->intval)
			pr_err("DC is present.\n");
		else
			pr_err("DC is missing.\n");
		break;
	case POWER_SUPPLY_PROP_WIRELESS_USB_PRESENT:
		if (!chip) {
			pr_err("called before init\n");
			return -EINVAL;
		}
		val->intval = chip->usb_present;
		if (val->intval)
			pr_err("usb is present.\n");
		else
			pr_err("usb is missing.\n");
		break;
#endif
	case POWER_SUPPLY_PROP_FLOATED_CHARGER:
		val->intval = psy->is_floated_charger;
		break;
	case POWER_SUPPLY_PROP_DRIVER_UNINSTALL:
		val->intval = psy->is_usb_driver_uninstall;
		break;
	default:
		break;
	}

	return 0;
}

static int bq24296_power_set_event_property(struct power_supply *psy,
					enum power_supply_event_type psp,
					const union power_supply_propval *val)
{
	struct bq24296_chip *chip = container_of(psy,
						struct bq24296_chip,
						ac_psy);

#if defined(CONFIG_CHARGER_UNIFIED_WLC)
	int vin_limit_wlc;
	int iterm_wlc;
	int ret;
#endif

#ifdef CONFIG_LGE_THERMALE_CHG_CONTROL_FOR_WLC
	int wlc_thermal_mitigation = -1;
	int i;
#endif

	NULL_CHECK(chip, -EINVAL);

	switch (psp) {
	case POWER_SUPPLY_PROP_ABNORMAL_TA:
		bq24296_set_input_i_limit(chip, 1000);
		bq24296_set_adjust_ibat(chip, val->intval);
		break;
#if defined(CONFIG_CHARGER_UNIFIED_WLC)
	case POWER_SUPPLY_PROP_WIRELESS_ONLINE_OTG:
		chip->wlc_otg = val->intval;
		break;
	case POWER_SUPPLY_PROP_WIRELESS_CHARGE_ENABLED:
		if (!chip) {
			pr_err("called before init\n");
			return -EINVAL;
		}
		pr_err(" called value : %d\n", val->intval);
		wireless_charging = val->intval;

		/* Start : G3 HW requirement */
		/* In wireless charging, change Vin limit and Iterm. */
		if (wireless_charging) {
			vin_limit_wlc = VIN_LIMIT_WLC;
			iterm_wlc = ITERM_MA_WLC;
		} else {
			vin_limit_wlc = chip->vin_limit_mv;
			iterm_wlc = chip->term_current_ma;
		}

		ret = bq24296_set_input_vin_limit(chip, vin_limit_wlc);
		if (ret) {
			pr_err("failed to set input voltage limit\n");
		} else {
			pr_info("[WLC] change Vin limit = %dmV\n", vin_limit_wlc);
		}

		ret = bq24296_set_term_current(chip, iterm_wlc);
		if (ret) {
			pr_err("failed to set termination current\n");
		} else {
			pr_info("[WLC] change Iterm = %dmA\n", iterm_wlc);
		}
		/* End : G3 HW requirement */
		break;
	case POWER_SUPPLY_PROP_WIRELESS_USB_PRESENT:
		if (!chip) {
			pr_err("called before init\n");
			return -EINVAL;
		}
		chip->wlc_present = !val->intval;
		chip->usb_present = val->intval;
		wake_lock_timeout(&chip->uevent_wake_lock, HZ*2);
		power_supply_set_present(chip->usb_psy, val->intval);
		break;
#ifdef CONFIG_LGE_THERMALE_CHG_CONTROL_FOR_WLC
	case POWER_SUPPLY_PROP_WIRELESS_THERMAL_MITIGATION:
		wlc_thermal_mitigation = val->intval;

		if (wlc_thermal_mitigation == THERMALE_ALL_CLEAR) {
			chip->wlc_input_current_te = INPUT_CURRENT_LIMIT_WLC;
		} else if (wlc_thermal_mitigation >= IBAT_MIN_MA && wlc_thermal_mitigation <= IBAT_WLC_ADJUST) {
			chip->wlc_input_current_te = INPUT_CURRENT_LIMIT_WLC_ADJUST;
		} else {
			/* When WLC thermal mitigation is triggered, upscaling input current limit. */
			for (i = 0; i < ARRAY_SIZE(icl_ma_table) - 1; i++) {
				if (wlc_thermal_mitigation <= icl_ma_table[i].icl_ma)
					break;
			}
			chip->wlc_input_current_te = icl_ma_table[i].icl_ma;
		}

		if (wlc_thermal_mitigation == THERMALE_ALL_CLEAR) {
			chip->wlc_chg_current_te = IBAT_WLC;
		} else if (wlc_thermal_mitigation >= IBAT_MIN_MA && wlc_thermal_mitigation <= IBAT_WLC_ADJUST) {
			chip->wlc_chg_current_te = IBAT_WLC_ADJUST;
		} else {
			chip->wlc_chg_current_te = wlc_thermal_mitigation;
		}

		pr_info("thermal-engine set wlc_input_current_te = %d, wlc_chg_current_te = %d\n",
			chip->wlc_input_current_te, chip->wlc_chg_current_te);

		cancel_delayed_work_sync(&chip->battemp_work);
		schedule_delayed_work(&chip->battemp_work, HZ*1);
		break;
#endif
#endif
	default:
		break;
	}

	return 0;
}

static int
bq24296_power_property_is_writeable(struct power_supply *psy,
						enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
	case POWER_SUPPLY_PROP_SAFTETY_CHARGER_TIMER:
		return 1;
	default:
		break;
	}

	return 0;
}

static ssize_t at_chg_status_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int r;
	bool b_chg_ok = false;
	int chg_type;
	struct bq24296_chip *chip = dev_get_drvdata(dev);
	NULL_CHECK(chip, -EINVAL);

	chg_type = bq24296_get_prop_charge_type(chip);
	if (chg_type != POWER_SUPPLY_CHARGE_TYPE_NONE) {
		b_chg_ok = true;
		r = snprintf(buf, 3, "%d\n", b_chg_ok);
		pr_info("[Diag] true ! buf = %s, charging=1\n", buf);
	} else {
		b_chg_ok = false;
		r = snprintf(buf, 3, "%d\n", b_chg_ok);
		pr_info("[Diag] false ! buf = %s, charging=0\n", buf);
	}

	return r;
}

static ssize_t at_chg_status_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret = 0;
	struct bq24296_chip *chip = dev_get_drvdata(dev);

	NULL_CHECK(chip, -EINVAL);

	NULL_CHECK(count, -EINVAL);

	if (strncmp(buf, "0", 1) == 0) {
		/* stop charging */
		pr_info("[Diag] stop charging start\n");
		ret = bq24296_enable_charging(chip, false);

	} else if (strncmp(buf, "1", 1) == 0) {
		/* start charging */
		pr_info("[Diag] start charging start\n");
		ret = bq24296_enable_charging(chip, true);
	}

	if (ret)
		return -EINVAL;

	return 1;
}

static ssize_t at_chg_complete_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int guage_level = 0;
	int r = 0;

	struct bq24296_chip *chip = dev_get_drvdata(dev);

	NULL_CHECK(chip, -EINVAL);

	guage_level = bq24296_get_prop_batt_capacity(chip);

	if (guage_level == 100) {
		r = snprintf(buf, 3, "%d\n", 0);
		pr_info("[Diag] buf = %s, gauge==100\n", buf);
	} else {
		r = snprintf(buf, 3, "%d\n", 1);
		pr_info("[Diag] buf = %s, gauge<=100\n", buf);
	}

	return r;
}

static ssize_t at_chg_complete_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret = 0;

	struct bq24296_chip *chip = dev_get_drvdata(dev);

	NULL_CHECK(chip, -EINVAL);

	NULL_CHECK(count, -EINVAL);

	if (strncmp(buf, "0", 1) == 0) {
		/* charging not complete */
		pr_info("[Diag] charging not complete start\n");
		ret = bq24296_enable_charging(chip, true);
	} else if (strncmp(buf, "1", 1) == 0) {
		/* charging complete */
		pr_info("[Diag] charging complete start\n");
		ret = bq24296_enable_charging(chip, false);
	}

	if (ret)
		return -EINVAL;

	return 1;
}

static ssize_t at_pmic_reset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int r = 0;
	bool pm_reset = true;
	struct bq24296_chip *chip = dev_get_drvdata(dev);

	NULL_CHECK(chip, -EINVAL);
	msleep(3000); /* for waiting return values of testmode */

	machine_restart(NULL);

	r = snprintf(buf, 3, "%d\n", pm_reset);

	return r;
}
static ssize_t at_otg_status_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int otg_mode;
	int r = 0;

	struct bq24296_chip *chip = dev_get_drvdata(dev);

	NULL_CHECK(chip, -EINVAL);

	otg_mode = bq24296_is_otg_mode(chip);
	if (otg_mode) {
		otg_mode = 1;
		r = snprintf(buf, 3, "%d\n", otg_mode);
		pr_info("[Diag] true ! buf = %s, OTG Enabled\n", buf);
	} else {
		otg_mode = 0;
		r = snprintf(buf, 3, "%d\n", otg_mode);
		pr_info("[Diag] false ! buf = %s, OTG Disabled\n", buf);
	}
	return r;
}

static ssize_t at_otg_status_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret = 0;
	struct bq24296_chip *chip = dev_get_drvdata(dev);

	NULL_CHECK(chip, -EINVAL);

	NULL_CHECK(count, -EINVAL);

	if (strncmp(buf, "0", 1) == 0) {
		pr_info("[Diag] OTG Disable start\n");
		if (bq24296_is_otg_mode(chip))
			ret = bq24296_enable_otg(chip, false);

	} else if (strncmp(buf, "1", 1) == 0) {
		pr_info("[Diag] OTG Enable start\n");
		if (!bq24296_is_otg_mode(chip))
			ret = bq24296_enable_otg(chip, true);
	}

	if (ret)
		return -EINVAL;
	return 1;
}
DEVICE_ATTR(at_charge, 0644, at_chg_status_show, at_chg_status_store);
DEVICE_ATTR(at_chcomp, 0644, at_chg_complete_show, at_chg_complete_store);
DEVICE_ATTR(at_pmrst, 0640, at_pmic_reset_show, NULL);
DEVICE_ATTR(at_otg, 0644, at_otg_status_show, at_otg_status_store);

struct charging_current_ma_entry {
	int	ibat;
	int 	adjust_ibat;
};

/* Use bq24296_set_adjust_ibat() instead of bq24296_set_ibat_max() */
#define MIN_CHG_CURRENT (100)
static int bq24296_set_adjust_ibat(struct bq24296_chip *chip, int ma)
{
	bool is_charger;
	NULL_CHECK(chip, -EINVAL);
	is_charger = bq24296_is_charger_present(chip);

	if (!is_charger)
		return 0;

#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
	if (ma > chip->otp_ibat_current)
		ma = chip->otp_ibat_current;
#endif
	if (ma < IBAT_MIN_MA) {
		bq24296_set_ibat_max(chip, ma * 5);
		bq24296_force_ichg_decrease(chip, 1);
	} else {
		bq24296_set_ibat_max(chip, ma);
		bq24296_force_ichg_decrease(chip, 0);
	}
	pr_info("charging current limit=%d\n", ma);
	return 0;
}

#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
static int bq24296_thermal_mitigation;
static int
bq24296_set_thermal_chg_current_set(const char *val, struct kernel_param *kp)
{
	int ret;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}

	NULL_CHECK(the_chip, -EINVAL);

	if (is_factory_cable()) {
		pr_err("plugged factory cable\n");
		return 0;
	}

#ifdef CONFIG_LGE_THERMALE_CHG_CONTROL
	pr_info("thermal-engine set chg current to %d\n",
			bq24296_thermal_mitigation);

	the_chip->chg_current_te = bq24296_thermal_mitigation;

	cancel_delayed_work_sync(&the_chip->battemp_work);
#ifdef CONFIG_LGE_PM_LLK_MODE
	if (!battemp_work_cancel)
		schedule_delayed_work(&the_chip->battemp_work, HZ*1);
#else
	schedule_delayed_work(&the_chip->battemp_work, HZ*1);
#endif
#else
	pr_err("thermal-engine chg current control not enabled\n");
#endif
	return 0;
}
module_param_call(bq24296_thermal_mitigation, bq24296_set_thermal_chg_current_set,
	param_get_uint, &bq24296_thermal_mitigation, 0644);

#if defined(CONFIG_MACH_MSM8974_G3_ATT) || defined(CONFIG_MACH_MSM8974_G3_CA)
static void pma_workaround_worker(struct work_struct *work)
{
	struct bq24296_chip *chip =
		container_of(work, struct bq24296_chip, pma_workaround_work.work);
	union power_supply_propval val = {0, };

	bq24296_charger_psy_getprop(chip, usb_psy, SCOPE, &val);
	pr_err("[WLC] otg state = %d\n", val.intval);
	if (val.intval != POWER_SUPPLY_SCOPE_SYSTEM) {
		gpio_set_value(chip->otg_en, 0);
		bq24296_enable_otg(chip, false);
		pr_err("[WLC] unset pma workaround\n");
	}

	if (wake_lock_active(&chip->pma_workaround_wake_lock)) {
		wake_unlock(&chip->pma_workaround_wake_lock);
		pr_err("[WLC] unset pma wake lock\n");
	}
}

static void pma_workaround(struct bq24296_chip *chip, int temp)
{
	if (temp >= 55) {
		gpio_set_value(chip->otg_en, 1);
		bq24296_enable_otg(chip, true);
		pr_err("[WLC] set pma workaround\n");

		schedule_delayed_work(&chip->pma_workaround_work, 15 * HZ);
		wake_lock(&chip->pma_workaround_wake_lock);
		pr_err("[WLC] set pma wake lock\n");
		pr_err("[WLC] after 15sec, unset pma workaround\n");
	}
}
#endif

static int temp_before;
static void bq24296_monitor_batt_temp(struct work_struct *work)
{
	struct bq24296_chip *chip =
		container_of(work, struct bq24296_chip, battemp_work.work);
	struct charging_info req;
	struct charging_rsp res;
	bool is_changed = false;
	union power_supply_propval ret = {0,};
#if defined(CONFIG_CHARGER_UNIFIED_WLC)
	union power_supply_propval wlc_ret = {0,};
	int wlc_online = -1;
#ifdef CONFIG_LGE_THERMALE_CHG_CONTROL_FOR_WLC
	int wlc_thermal_mitigation = -1;
#endif
#endif
	NULL_CHECK_VOID(chip);
	if (chip->chg_timeout) {
		int ret;

		pr_err("escape charging timeout, charging enable and unlocked\n");
		chip->chg_timeout = false;
		ret = bq24296_enable_charging(chip, true);
		if (ret)
			pr_err("Failed to set CHG_CONFIG ret=%d\n", ret);

		wake_unlock(&chip->chg_timeout_lock);
	}

	chip->batt_psy.get_property(&(chip->batt_psy),
			  POWER_SUPPLY_PROP_TEMP, &ret);
	req.batt_temp = ret.intval / 10;

	/* caution!! Scale from mV to uV for lge_monitor_batt_temp() */
	chip->batt_psy.get_property(&(chip->batt_psy),
			  POWER_SUPPLY_PROP_VOLTAGE_NOW, &ret);
	req.batt_volt = ret.intval;

	chip->batt_psy.get_property(&(chip->batt_psy),
			  POWER_SUPPLY_PROP_CURRENT_NOW, &ret);
	req.current_now = ret.intval;

#ifdef CONFIG_LGE_THERMALE_CHG_CONTROL
	bq24296_thermal_mitigation = chip->chg_current_ma;
	req.chg_current_ma = chip->chg_current_ma;
	req.chg_current_te = chip->chg_current_te;

#if defined(CONFIG_CHARGER_UNIFIED_WLC)
	bq24296_charger_psy_getprop_event(chip, wlc_psy, WIRELESS_ONLINE,
		&wlc_ret, _WIRELESS_);
	wlc_online = wlc_ret.intval;

	if (wlc_online) {
#ifdef CONFIG_LGE_THERMALE_CHG_CONTROL_FOR_WLC
#if defined(CONFIG_MACH_MSM8974_G3_ATT) || defined(CONFIG_MACH_MSM8974_G3_CA)
		pma_workaround(chip, req.batt_temp);
#endif
		bq24296_charger_psy_getprop_event(chip, wlc_psy,
			WIRELESS_THERMAL_MITIGATION, &wlc_ret, _WIRELESS_);
		wlc_thermal_mitigation = wlc_ret.intval;

		/* When WLC tharmal mitigation is not triggered first yet. */
		if (wlc_thermal_mitigation == THERMALE_NOT_TRIGGER) {
			chip->wlc_input_current_te = INPUT_CURRENT_LIMIT_WLC;
			chip->wlc_chg_current_te = IBAT_WLC;
		}

		req.input_current_te = chip->wlc_input_current_te;
		req.chg_current_te = chip->wlc_chg_current_te;

		/* In WLC tharmal mitigation, adjust wireless charging Iin limit.*/
		pr_info("thermal-engine set req.input_current_te = %d\n",
			req.input_current_te);
		bq24296_set_input_i_limit(chip, req.input_current_te);
#else
		req.chg_current_te = IBAT_WLC;
#endif
	}

#endif
	pr_info("thermal-engine set req.chg_current_ma = %d, req.chg_current_te = %d\n",
		req.chg_current_ma, req.chg_current_te);
#endif

	req.is_charger = bq24296_is_charger_present(chip);

	lge_monitor_batt_temp(req, &res);

	if (((res.change_lvl != STS_CHE_NONE) && req.is_charger) ||
		(res.force_update == true)) {
		if (res.change_lvl == STS_CHE_NORMAL_TO_DECCUR ||
			((res.force_update == true) && (res.state == CHG_BATT_DECCUR_STATE) &&
			(res.dc_current != DC_CURRENT_DEF) &&
			(res.change_lvl != STS_CHE_STPCHG_TO_DECCUR)
			)) {
			chip->otp_ibat_current = res.dc_current;
		} else if (res.change_lvl == STS_CHE_NORMAL_TO_STPCHG ||
			(res.force_update == true &&
			res.state == CHG_BATT_STPCHG_STATE)) {
			wake_lock(&chip->lcs_wake_lock);
			chip->otp_ibat_current = 0;
			bq24296_enable_charging(chip, !res.disable_chg);
		} else if (res.change_lvl == STS_CHE_DECCUR_TO_NORAML) {
			chip->otp_ibat_current = res.dc_current;
		} else if (res.change_lvl == STS_CHE_DECCUR_TO_STPCHG) {
			wake_lock(&chip->lcs_wake_lock);
			chip->otp_ibat_current = 0;
			bq24296_enable_charging(chip, !res.disable_chg);
		} else if (res.change_lvl == STS_CHE_STPCHG_TO_NORMAL) {
			chip->otp_ibat_current = res.dc_current;
			bq24296_enable_charging(chip, !res.disable_chg);
			wake_unlock(&chip->lcs_wake_lock);
		} else if (res.change_lvl == STS_CHE_STPCHG_TO_DECCUR) {
			chip->otp_ibat_current = res.dc_current;
			wake_unlock(&chip->lcs_wake_lock);
		} else if (res.force_update == true && res.state == CHG_BATT_NORMAL_STATE &&
			res.dc_current != DC_CURRENT_DEF) {
			chip->otp_ibat_current = res.dc_current;
		}
	}
	pr_info(" otp_ibat_current=%d\n", chip->otp_ibat_current);
	bq24296_set_adjust_ibat(chip, chip->chg_current_ma);
	if (chip->pseudo_ui_chg ^ res.pseudo_chg_ui) {
		is_changed = true;
		chip->pseudo_ui_chg = res.pseudo_chg_ui;
	}

	if (chip->btm_state ^ res.btm_state) {
		is_changed = true;
		chip->btm_state = res.btm_state;
	}

	if (temp_before != req.batt_temp) {
		is_changed = true;
		temp_before = req.batt_temp;
	}

	if (is_changed == true)
		power_supply_changed(&chip->batt_psy);

	bq24296_reginfo(chip);

	schedule_delayed_work(&chip->battemp_work,
		MONITOR_BATTEMP_POLLING_PERIOD);
}
#endif

static ssize_t batt_removed_print_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", "battery_removed");
}

static ssize_t batt_removed_print_state(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%d\n", sdev->state);
}

static int bq24296_create_debugfs_entries(struct bq24296_chip *chip)
{
	int i;
	NULL_CHECK(chip, -EINVAL);
	chip->dent = debugfs_create_dir(BQ24296_NAME, NULL);
	if (IS_ERR(chip->dent)) {
		pr_err("bq24296 driver couldn't create debugfs dir\n");
		return -EFAULT;
	}

	for (i = 0 ; i < ARRAY_SIZE(bq24296_debug_regs) ; i++) {
		char *name = bq24296_debug_regs[i].name;
		u32 reg = bq24296_debug_regs[i].reg;
		struct dentry *file;

		file = debugfs_create_file(name, 0644, chip->dent,
					(void *) reg, &reg_fops);
		if (IS_ERR(file)) {
			pr_err("debugfs_create_file %s failed.\n", name);
			return -EFAULT;
		}
	}

	return 0;
}

static void bq24296_remove_debugfs_entries(struct bq24296_chip *chip)
{
	NULL_CHECK_VOID(chip);
	if (chip->dent)
		debugfs_remove_recursive(chip->dent);
}

static int __devinit bq24296_init_batt_psy(struct bq24296_chip *chip)
{
	int ret;
	NULL_CHECK(chip, -EINVAL);

	chip->batt_psy.name = "battery";
	chip->batt_psy.type = POWER_SUPPLY_TYPE_BATTERY;
	chip->batt_psy.properties = bq24296_batt_power_props;
	chip->batt_psy.num_properties =
					ARRAY_SIZE(bq24296_batt_power_props);
	chip->batt_psy.get_property = bq24296_batt_power_get_property;
	chip->batt_psy.set_property = bq24296_batt_power_set_property;
	chip->batt_psy.property_is_writeable =
					bq24296_batt_power_property_is_writeable;
	chip->batt_psy.external_power_changed =
					bq24296_batt_external_power_changed;

	ret = power_supply_register(&chip->client->dev,
				&chip->batt_psy);
	if (ret) {
		pr_err("failed to register power_supply. ret=%d.\n", ret);
		return ret;
	}

	return 0;
}

static int __devinit bq24296_init_ac_psy(struct bq24296_chip *chip)
{
	int ret = 0;
	NULL_CHECK(chip, -EINVAL);

	chip->ac_psy.name = "ac";
	chip->ac_psy.type = POWER_SUPPLY_TYPE_MAINS;
	chip->ac_psy.supplied_to = bq24296_power_supplied_to;
	chip->ac_psy.num_supplicants = ARRAY_SIZE(bq24296_power_supplied_to);
	chip->ac_psy.properties = bq24296_power_props;
	chip->ac_psy.num_properties = ARRAY_SIZE(bq24296_power_props);
	chip->ac_psy.get_property = bq24296_power_get_property;
	chip->ac_psy.set_property = bq24296_power_set_property;
	chip->ac_psy.get_event_property = bq24296_power_get_event_property;
	chip->ac_psy.set_event_property = bq24296_power_set_event_property;
	chip->ac_psy.property_is_writeable =
				bq24296_power_property_is_writeable;
	ret = power_supply_register(&chip->client->dev,
				&chip->ac_psy);
	if (ret) {
		pr_err("failed to register power_supply. ret=%d.\n", ret);
		return ret;
	}
	chip->psy_this = &chip->ac_psy;

	return 0;
}

static int bq24296_hw_init(struct bq24296_chip *chip)
{
	int ret = 0;
	NULL_CHECK(chip, -EINVAL);

	ret = bq24296_set_input_vin_limit(chip, chip->vin_limit_mv);
	if (ret) {
		pr_err("failed to set input voltage limit\n");
		return ret;
	}

	ret = bq24296_set_input_i_limit(chip, INPUT_CURRENT_LIMIT_TA);
	if (ret) {
		pr_err("failed to set input current limit\n");
		return ret;
	}

	ret = bq24296_set_system_vmin(chip, chip->sys_vmin_mv);
	if (ret) {
		pr_err("failed to set system min voltage\n");
		return ret;
	}

	ret = bq24296_set_ibat_max(chip, chip->chg_current_ma);
	if (ret) {
		pr_err("failed to set charging current\n");
		return ret;
	}

	ret = bq24296_force_ichg_decrease(chip, 0);
	if (ret) {
		pr_err("failed to set charging current as reg[ICHG] programmed\n");
		return ret;
	}

	ret = bq24296_set_prechg_i_limit(chip, chip->pre_chg_current_ma);
	if (ret) {
		pr_err("failed to set pre-charge current\n");
		return ret;
	}

	ret = bq24296_set_term_current(chip, chip->term_current_ma);
	if (ret) {
		pr_err("failed to set charge termination current\n");
		return ret;
	}

	ret = bq24296_set_vbat_max(chip, chip->vbat_max_mv);
	if (ret) {
		pr_err("failed to set vbat max\n");
		return ret;
	}
#if defined(CONFIG_CHARGER_FACTORY_MODE)
		ret = bq24296_set_chg_term(chip, 1);
		if (ret) {
			pr_err("failed to disable chg termination\n");
			return ret;
		}
		ret = bq24296_set_chg_timer(chip, 0);
		if (ret) {
			pr_err("failed to enable chg safety timer\n");
			return ret;
		}
#if !defined(CONFIG_MACH_MSM8974_G3_VZW)
		ret = bq24296_set_chg_timeout(chip);
		if (ret) {
			pr_err("Failed to set CHG_TIMEOUT rc=%d\n", ret);
			return ret;
		}
#endif
#else
		ret = bq24296_set_chg_term(chip, 1);
		if (ret) {
			pr_err("failed to enable chg termination\n");
			return ret;
		}

		ret = bq24296_set_chg_timer(chip, 1);
		if (ret) {
			pr_err("failed to enable chg safety timer\n");
			return ret;
		}
#if !defined(CONFIG_MACH_MSM8974_G3_VZW)
		ret = bq24296_set_chg_timeout(chip);
		if (ret) {
			pr_err("Failed to set CHG_TIMEOUT rc=%d\n", ret);
			return ret;
		}
#endif
#endif
	return 0;
}

static int bq24296_parse_dt(struct device_node *dev_node,
			   struct bq24296_chip *chip)
{
	int ret = 0;
	NULL_CHECK(chip, -EINVAL);

	chip->int_gpio =
		of_get_named_gpio(dev_node, "ti,int-gpio", 0);
	pr_debug("int_gpio = %d.\n", chip->int_gpio);
	if (chip->int_gpio < 0) {
		pr_err("failed to get int-gpio.\n");
		ret = chip->int_gpio;
		goto out;
	}
	chip->ext_chg_en = of_get_named_gpio(dev_node, "ti,ext-chg-en-gpio", 0);
	pr_info("ext_chg_en = %d\n", chip->ext_chg_en);
	if (chip->ext_chg_en < 0) {
		pr_err("failed to get ext_chg_en\n");
		ret = chip->ext_chg_en;
		goto out;
	}
	chip->otg_en = of_get_named_gpio(dev_node, "ti,otg-en-gpio", 0);
	pr_info("otg_en = %d\n", chip->otg_en);
	if (chip->otg_en < 0) {
		pr_err("failed to get otg_en\n");
		ret = chip->otg_en;
		goto out;
	}

	chip->watchdog = of_property_read_bool(dev_node, "ti,watchdog-en");
	pr_info("watchdog-en = %s\n", chip->watchdog ? "enabled" : "disabled");

	ret = of_property_read_u32(dev_node, "ti,chg-current-ma",
				   &(chip->chg_current_ma));
#ifdef CONFIG_LGE_THERMALE_CHG_CONTROL
	bq24296_thermal_mitigation = chip->chg_current_ma;
	chip->chg_current_te = chip->chg_current_ma;
#endif
	pr_debug("bq24296 chg_current_ma = %d.\n",
			chip->chg_current_ma);
	if (ret) {
		pr_err("Unable to read chg-current-ma.\n");
		return ret;
	}
#if defined(CONFIG_LGE_PM_CHARGING_SUPPORT_PHIHONG)
	ret = of_property_read_u32(dev_node, "ti,pre-input-current-ma",
				   &(chip->pre_input_current_ma));
	pr_debug("bq24296 pre_input_current-ma = %d.\n",
				chip->pre_input_current_ma);
	if (ret) {
		pr_err("Unable to read pre-fastchg-current-ma. Set 0.\n");
		chip->pre_input_current_ma = 1000;
	}
#endif
	ret = of_property_read_u32(dev_node, "ti,term-current-ma",
				   &(chip->term_current_ma));
	pr_debug("bq24296 term_current_ma = %d.\n",
				chip->term_current_ma);
	if (ret) {
		pr_err("Unable to read term-current-ma.\n");
		return ret;
	}

	ret = of_property_read_u32(dev_node, "ti,vbat-max-mv",
				   &chip->vbat_max_mv);
	pr_debug("bq24296 vbat_max_mv = %d.\n",
				chip->vbat_max_mv);
	if (ret) {
		pr_err("Unable to read vbat-max-mv.\n");
		return ret;
	}

	ret = of_property_read_u32(dev_node, "ti,pre-chg-current-ma",
				   &chip->pre_chg_current_ma);
	pr_debug("bq24296 pre_chg_current_ma = %d.\n",
				chip->pre_chg_current_ma);
	if (ret) {
		pr_err("Unable to read pre-chg-current-ma.\n");
		return ret;
	}

	ret = of_property_read_u32(dev_node, "ti,sys-vmin-mv",
				   &chip->sys_vmin_mv);
	pr_debug("bq24296 sys_vmin_mv = %d.\n",
				chip->sys_vmin_mv);
	if (ret) {
		pr_err("Unable to read sys-vmin-mv.\n");
		return ret;
	}

	ret = of_property_read_u32(dev_node, "ti,vin-limit-mv",
				   &chip->vin_limit_mv);
	pr_debug("bq24296 vin_limit_mv = %d.\n",
				chip->vin_limit_mv);
	if (ret) {
		pr_err("Unable to read vin-limit-mv.\n");
		return ret;
	}

	ret = of_property_read_u32(dev_node, "ti,icl-vbus-mv",
				   &chip->icl_vbus_mv);
	if (ret) {
		pr_err("Unable to read icl threshod voltage.\n");
		return ret;
	}
out:
	return ret;
}

int lge_get_sbl_cable_type(void)
{
	int ret_cable_type = 0;
	unsigned int *p_cable_type = (unsigned int *)(smem_get_entry(SMEM_ID_VENDOR1, &cable_smem_size));

	if (p_cable_type)
		ret_cable_type = *p_cable_type;
	else
		ret_cable_type = 0;

	return ret_cable_type;
}
EXPORT_SYMBOL(lge_get_sbl_cable_type);

static int bq24296_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	const struct bq24296_platform_data *pdata;
	struct device_node *dev_node = client->dev.of_node;
	struct bq24296_chip *chip;
	int ret = 0;
#if defined(CONFIG_LGE_PM_BATTERY_ID_CHECKER)
	uint *smem_batt = 0;
#if defined(CONFIG_LGE_LOW_BATT_LIMIT)
	uint _smem_batt_ = 0;
#endif
#endif
	unsigned int *p_cable_type = (unsigned int *)
		(smem_get_entry(SMEM_ID_VENDOR1, &cable_smem_size));

	pr_err("bq24296 charger probe : start\n");

	if (p_cable_type)
		cable_type = *p_cable_type;
	else
		cable_type = 0;

	if (cable_type == LT_CABLE_56K || cable_type == LT_CABLE_130K ||
					cable_type == LT_CABLE_910K)
		factory_mode = 1;
	 else
		factory_mode = 0;

	pr_err("cable_type is = %d factory_mode = %d\n",
		cable_type, factory_mode);

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_BYTE_DATA)) {
		pr_err("i2c func fail.\n");
		return -EIO;
	}

	chip = kzalloc(sizeof(struct bq24296_chip), GFP_KERNEL);
	if (!chip) {
		pr_err("failed to alloc memory\n");
		return -ENOMEM;
	}

	chip->client = client;
	chip->batt_present = true;
#if defined(CONFIG_VZW_POWER_REQ)
	chip->vzw_chg_mode = VZW_NO_CHARGER;
#endif

#if defined(CONFIG_LGE_PM_CHARGING_SUPPORT_PHIHONG)
	chip->phihong = PHIHONG_NOT_VERIFIED;
	init_completion(&chip->phihong_complete);
#endif

#if defined(CONFIG_LGE_CURRENTNOW)
	chip->cn_psy = power_supply_get_by_name("cn");
	if (!chip->cn_psy) {
		pr_err("cn supply not found deferring probe\n");
		ret = -EPROBE_DEFER;
		goto error;
	}
#endif

	chip->usb_psy = power_supply_get_by_name("usb");
	if (!chip->usb_psy) {
		pr_err("usb supply not found deferring probe\n");
		ret = -EPROBE_DEFER;
		goto error;
	}

	get_cable_data_from_dt(dev_node);

	if (dev_node) {
		ret = bq24296_parse_dt(dev_node, chip);
		if (ret) {
			pr_err("failed to parse dt\n");
			goto error;
		}

		chip->vadc_dev = qpnp_get_vadc(&(client->dev), "bq24296");
		if (IS_ERR(chip->vadc_dev)) {
			ret = PTR_ERR(chip->vadc_dev);
			if (ret != -EPROBE_DEFER)
				pr_err("vadc property missing\n");
			else
				pr_err("probe defer due to not initializing vadc\n");

			goto error;
		}
		lge_pm_read_cable_info(chip->vadc_dev);
	} else {
		pdata = client->dev.platform_data;
		if (pdata == NULL) {
			pr_err("no platform data.\n");
			return -EINVAL;
		}

		chip->int_gpio = pdata->int_gpio;
		chip->ext_chg_en = pdata->ext_chg_en;
		chip->otg_en = pdata->otg_en;
		chip->chg_current_ma = pdata->chg_current_ma;
		chip->term_current_ma = pdata->term_current_ma;
		chip->vbat_max_mv = pdata->vbat_max_mv;
		chip->pre_chg_current_ma = pdata->pre_chg_current_ma;
		chip->sys_vmin_mv = pdata->sys_vmin_mv;
		chip->vin_limit_mv = pdata->vin_limit_mv;
		chip->icl_vbus_mv = pdata->icl_vbus_mv;
	}
	chip->set_chg_current_ma = chip->chg_current_ma;
#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
	chip->otp_ibat_current = chip->chg_current_ma;
#endif

	ret = gpio_request_one(chip->int_gpio, GPIOF_DIR_IN,
			"bq24296_int");
	if (ret) {
		pr_err("failed to request int_gpio\n");
		goto error;
	}
	chip->irq = gpio_to_irq(chip->int_gpio);
	pr_debug("int_gpio irq#=%d.\n", chip->irq);

	ret = gpio_request_one(chip->ext_chg_en, GPIOF_DIR_OUT | GPIOF_INIT_LOW,
			"bq24296_en");
	if (ret) {
		pr_err("failed to request bq24296_en\n");
		goto error;
	}
	ret = gpio_request(chip->otg_en, "otg_en");
	if (ret) {
		printk("otg_en gpio_request failed for %d ret=%d\n",
			   chip->otg_en, ret);
		goto error;
	}
	gpio_direction_output(chip->otg_en, 0);

	i2c_set_clientdata(client, chip);

	ret = bq24296_hw_init(chip);
#if defined(CONFIG_CHARGER_UNIFIED_WLC)
	chip->wlc_otg_status = FAKE_UNKNOWN;
#endif
	if (ret) {
		pr_err("bq24296_hwinit failed.ret=%d\n", ret);
		goto err_hw_init;
	}

	the_chip = chip;

	wake_lock_init(&chip->chg_wake_lock,
		       WAKE_LOCK_SUSPEND, BQ24296_NAME);
	wake_lock_init(&chip->uevent_wake_lock,
		       WAKE_LOCK_SUSPEND, "bq24296_chg_uevent");
#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
	wake_lock_init(&chip->lcs_wake_lock,
			WAKE_LOCK_SUSPEND, "LGE charging scenario");
#endif
	wake_lock_init(&chip->battgone_wake_lock,
		       WAKE_LOCK_SUSPEND, "batt removed");
	wake_lock_init(&chip->chg_timeout_lock,
			       WAKE_LOCK_SUSPEND, "chg timeout");
	wake_lock_init(&chip->icl_wake_lock,
			       WAKE_LOCK_SUSPEND, "icl_wake_lock");
#if defined(CONFIG_MACH_MSM8974_G3_ATT) || defined(CONFIG_MACH_MSM8974_G3_CA)
	wake_lock_init(&chip->pma_workaround_wake_lock,
		       WAKE_LOCK_SUSPEND, "pma_wake_lock");
#endif

	chip->batt_removed.name = "battery_removed";
	chip->batt_removed.state = 0; /*if batt is removed, state will be set to 1 */
	chip->batt_removed.print_name = batt_removed_print_name;
	chip->batt_removed.print_state = batt_removed_print_state;

	INIT_DELAYED_WORK(&chip->usbin_mon, bq24296_usbin_mon_worker);
	INIT_DELAYED_WORK(&chip->irq_work, bq24296_irq_worker);
#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
	INIT_DELAYED_WORK(&chip->battemp_work, bq24296_monitor_batt_temp);
#endif
#ifdef I2C_SUSPEND_WORKAROUND
	INIT_DELAYED_WORK(&chip->check_suspended_work,
			bq24296_check_suspended_worker);
#endif
#if defined(CONFIG_MACH_MSM8974_G3_ATT) || defined(CONFIG_MACH_MSM8974_G3_CA)
	INIT_DELAYED_WORK(&chip->pma_workaround_work, pma_workaround_worker);
#endif

	mutex_init(&chip->usbin_lock);
	chip->usbin_ref_count = 0;
	chip->last_usbin_mv = ROUND_mA(chip->icl_vbus_mv);
#if defined(CONFIG_VZW_POWER_REQ)
	chip->usbin_ref_count_vzw = chip->adc_sum = 0;
#endif
	ret = switch_dev_register(&chip->batt_removed);
	if (ret < 0) {
		pr_err("Failed to register switch device, battery_removed\n");
	}

	ret = bq24296_init_batt_psy(chip);
	if (ret) {
		pr_err("bq24296_init_batt_psy failed ret=%d\n", ret);
		goto err_init_batt_psy;
	}

	ret = bq24296_init_ac_psy(chip);
	if (ret) {
		pr_err("bq24296_init_ac_psy failed ret=%d\n", ret);
		goto err_init_ac_psy;
	}

#if defined(CONFIG_LGE_PM_BATTERY_ID_CHECKER)
	smem_batt = (uint *)smem_alloc(SMEM_BATT_INFO, sizeof(smem_batt));
	if (smem_batt == NULL) {
		pr_err("%s : smem_alloc returns NULL\n", __func__);
		chip->batt_id_smem = 0;
	} else {
#if defined(CONFIG_LGE_LOW_BATT_LIMIT)
		_smem_batt_ = (*smem_batt >> 8) & 0x00ff; /* batt id -> HSB */
		pr_info("Battery was read in sbl is = %d\n", _smem_batt_);
		if (_smem_batt_ == BATT_ID_DS2704_L ||
			_smem_batt_ == BATT_ID_DS2704_C ||
			_smem_batt_ == BATT_ID_ISL6296_L ||
			_smem_batt_ == BATT_ID_ISL6296_C)
#else
		pr_info("Battery was read in sbl is = %d\n", *smem_batt);
		if (*smem_batt == BATT_ID_DS2704_L ||
			*smem_batt == BATT_ID_DS2704_C ||
			*smem_batt == BATT_ID_ISL6296_L ||
			*smem_batt == BATT_ID_ISL6296_C)
#endif
			chip->batt_id_smem = 1;
		else
			chip->batt_id_smem = 0;
	}
#endif

	if (!chip->watchdog) {
		ret = bq24296_masked_write(chip->client, BQ05_CHARGE_TERM_TIMER_CONT_REG,
			I2C_TIMER_MASK, 0x00);
		if (ret) {
			pr_err("failed to set I2C_TIMER rc=%d\n", ret);
			return ret;
		}
	}
	if (is_factory_cable()) {
		if (is_factory_cable_130k()) {
			pr_info("factory cable detected(130k) iLimit 500mA\n");
			bq24296_set_input_i_limit(chip, INPUT_CURRENT_LIMIT_USB20);
		} else {
			pr_info("factory cable detected  iLimit 1500mA\n");
			bq24296_set_input_i_limit(chip, INPUT_CURRENT_LIMIT_FACTORY);
			bq24296_force_ichg_decrease(chip, 1);
		}
	}


	ret = bq24296_create_debugfs_entries(chip);
	if (ret) {
		pr_err("bq24296_create_debugfs_entries failed ret=%d\n", ret);
		goto err_debugfs;
	}
	if (chip->irq) {
		ret = request_irq(chip->irq, bq24296_irq,
				IRQF_TRIGGER_FALLING,
				"bq24296_irq", chip);
		if (ret) {
			pr_err("request_irq %d failed\n", chip->irq);
			goto err_req_irq;
		}
		enable_irq_wake(chip->irq);
	}

	chip->usb_present = bq24296_is_charger_present(chip);
	power_supply_set_present(chip->usb_psy, chip->usb_present);

	bq24296_enable_charging(chip, 1);

	ret = device_create_file(&client->dev, &dev_attr_at_charge);
	if (ret < 0) {
		pr_err("%s:File dev_attr_at_charge creation failed: %d\n",
				__func__, ret);
		ret = -ENODEV;
		goto err_at_charge;
	}

	ret = device_create_file(&client->dev, &dev_attr_at_chcomp);
	if (ret < 0) {
		pr_err("%s:File dev_attr_at_chcomp creation failed: %d\n",
				__func__, ret);
		ret = -ENODEV;
		goto err_at_chcomp;
	}

	ret = device_create_file(&client->dev, &dev_attr_at_pmrst);
	if (ret < 0) {
		pr_err("%s:File device creation failed: %d\n", __func__, ret);
		ret = -ENODEV;
		goto err_at_pmrst;
	}

	ret = device_create_file(&client->dev, &dev_attr_at_otg);
	if (ret < 0) {
		pr_err("%s:File device creation failed: %d\n", __func__, ret);
		ret = -ENODEV;
		goto err_at_otg;
	}

#ifdef CONFIG_SENSORS_QPNP_ADC_VOLTAGE
	last_batt_temp = DEFAULT_TEMP;
#endif

	safety_timer_enabled = 1;

#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
	schedule_delayed_work(&chip->battemp_work,
		MONITOR_BATTEMP_POLLING_PERIOD / 3);
#endif
	schedule_delayed_work(&chip->irq_work, msecs_to_jiffies(2000));
#ifdef CONFIG_ZERO_WAIT
	ret = zw_psy_wakeup_source_register(&chip->chg_wake_lock.ws);
	if (ret < 0)
		goto err_zw_ws_register;
#endif
#if defined(CONFIG_LGE_PM_CHARGING_SUPPORT_PHIHONG)
	chip->phihong_task = kthread_run(do_phihong_checker, chip,
			"bq24296:phihong_checker");
	if (IS_ERR(chip->phihong_task)) {
		pr_err("Fail to create phihong detector thread");
		goto probe_fail;
	}
	init_timer(&chip->phihong_timer);
	chip->phihong_timer.data = (unsigned long) chip;
	trig_phihong_timer(chip);
#endif
	if (ret)
		goto probe_fail;
	pr_err("bq24296 charger probe : success\n");
	return 0;
probe_fail:
#ifdef CONFIG_ZERO_WAIT
err_zw_ws_register:
	device_remove_file(&client->dev, &dev_attr_at_otg);
#endif
err_at_otg:
	device_remove_file(&client->dev, &dev_attr_at_pmrst);
err_at_pmrst:
	device_remove_file(&client->dev, &dev_attr_at_chcomp);
err_at_chcomp:
	device_remove_file(&client->dev, &dev_attr_at_charge);
err_at_charge:
err_req_irq:
	bq24296_remove_debugfs_entries(chip);
err_debugfs:
	power_supply_unregister(&chip->ac_psy);
err_init_ac_psy:
	power_supply_unregister(&chip->batt_psy);
err_init_batt_psy:
#if defined(CONFIG_MACH_MSM8974_G3_ATT) || defined(CONFIG_MACH_MSM8974_G3_CA)
	wake_lock_destroy(&chip->pma_workaround_wake_lock);
#endif
	wake_lock_destroy(&chip->icl_wake_lock);
	wake_lock_destroy(&chip->chg_wake_lock);
	wake_lock_destroy(&chip->uevent_wake_lock);
#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
	wake_lock_destroy(&chip->lcs_wake_lock);
#endif
	wake_lock_destroy(&chip->battgone_wake_lock);
	wake_lock_destroy(&chip->chg_timeout_lock);

err_hw_init:
	if (chip->int_gpio)
		gpio_free(chip->int_gpio);
error:
	kfree(chip);
	chip = NULL;
	pr_info("fail to probe\n");
	the_chip = NULL;
	return ret;

}

static int bq24296_remove(struct i2c_client *client)
{
	struct bq24296_chip *chip = i2c_get_clientdata(client);
#ifdef CONFIG_ZERO_WAIT
	zw_psy_wakeup_source_unregister(&chip->chg_wake_lock.ws);
#endif
	bq24296_remove_debugfs_entries(chip);
	device_remove_file(&client->dev, &dev_attr_at_charge);
	device_remove_file(&client->dev, &dev_attr_at_chcomp);
	device_remove_file(&client->dev, &dev_attr_at_pmrst);
	device_remove_file(&client->dev, &dev_attr_at_otg);

	wake_lock_destroy(&chip->chg_wake_lock);
	wake_lock_destroy(&chip->uevent_wake_lock);
#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
	wake_lock_destroy(&chip->lcs_wake_lock);
#endif
	wake_lock_destroy(&chip->battgone_wake_lock);
	wake_lock_destroy(&chip->icl_wake_lock);
#if defined(CONFIG_MACH_MSM8974_G3_ATT) || defined(CONFIG_MACH_MSM8974_G3_CA)
	wake_lock_destroy(&chip->pma_workaround_wake_lock);
#endif

	power_supply_unregister(&chip->ac_psy);
	power_supply_unregister(&chip->batt_psy);

	if (chip->irq)
		free_irq(chip->irq, chip);
	if (chip->int_gpio)
		gpio_free(chip->int_gpio);
	if (chip->otg_en)
		gpio_free(chip->otg_en);

	kfree(chip);
	chip = NULL;
	the_chip = NULL;
	return 0;
}

static const struct i2c_device_id bq24296_id[] = {
	{BQ24296_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, bq24296_id);

static const struct of_device_id bq24296_match[] = {
	{ .compatible = "ti,bq24296-charger", },
	{ },
};

#ifdef CONFIG_PM
static int bq24296_resume(struct i2c_client *client)
{
	struct bq24296_chip *chip = i2c_get_clientdata(client);
	NULL_CHECK(chip, -EINVAL);
	chip->suspend = false;
	schedule_delayed_work(&chip->battemp_work, HZ*1);
	return 0;
}

static int bq24296_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct bq24296_chip *chip = i2c_get_clientdata(client);
	NULL_CHECK(chip, -EINVAL);
	chip->suspend = true;
	cancel_delayed_work_sync(&chip->battemp_work);
	return 0;
}
#endif

static struct i2c_driver bq24296_driver = {
	.driver	= {
			.name	= BQ24296_NAME,
			.owner	= THIS_MODULE,
			.of_match_table = of_match_ptr(bq24296_match),
	},
	.probe		= bq24296_probe,
	.remove		= bq24296_remove,
	.id_table	= bq24296_id,
#ifdef CONFIG_PM
	.resume		= bq24296_resume,
	.suspend	= bq24296_suspend,
#endif
};

static int __init bq24296_init(void)
{
	return i2c_add_driver(&bq24296_driver);
}
module_init(bq24296_init);

static void __exit bq24296_exit(void)
{
	return i2c_del_driver(&bq24296_driver);
}
module_exit(bq24296_exit);

MODULE_DESCRIPTION("Driver for BQ24296 charger chip");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:" BQ24296_NAME);
