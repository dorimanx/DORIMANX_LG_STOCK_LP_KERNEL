/*
 * IDPT9025A / BQ51020 Wireless Charging(WLC) control driver
 *
 * Copyright (C) 2012 LG Electronics, Inc
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 */

/*#define pr_fmt(fmt)	"%s: " fmt, __func__*/

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>


#include <linux/leds-pm8xxx.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/errno.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/power/unified_wireless_charger.h>
#include <linux/delay.h>

#if defined(CONFIG_SMB349_CHARGER)
#include <linux/i2c/smb349_charger.h>
#elif defined(CONFIG_BQ24192_CHARGER)
#include <linux/i2c/bq24192_charger.h>
#elif defined(CONFIG_BQ24296_CHARGER)
#include <linux/i2c/bq24296_charger.h>
#elif defined (CONFIG_CHARGER_MAX77819)
#include <linux/power/max77819.h>
#include <linux/mfd/max77819-charger.h>
#endif

#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
#include <mach/lge_charging_scenario.h>
#endif

#include <mach/board_lge.h>
#ifdef CONFIG_CHARGER_UNIFIED_WLC_ALIGNMENT
#include <linux/power/unified_wireless_charger_alignment.h>
#endif

#define MAX_CHECKING_COUNT	10
#define MAX_USB_CHECKING_COUNT	4

#define I2C_SUSPEND_WORKAROUND (1)

#ifdef CONFIG_LGE_THERMALE_CHG_CONTROL_FOR_WLC
#define THERMALE_NOT_TRIGGER -1
#define THERMALE_ALL_CLEAR 0
#define THERMALE_SETTABLE_RANGE_MAX 3008
#define THERMALE_SETTABLE_RANGE_MIN 512
#endif

#define _WIRELESS_ "wireless"
#define _BATTERY_     "battery"
#define _USB_		"usb"
#define _CN_		"cn"
#define _EXT_		"ac"
static inline struct power_supply *_psy_check_ext(struct power_supply *_psy,
					const char *_psp_name)
{
	if(!likely(_psy)) {
		pr_info("psp is not found %s\n", _psp_name);
		_psy = power_supply_get_by_name((char*)_psp_name);
	}
	return _psy;
}

#define wlc_charger_psy_getprop_event(_me, _psy, _psp, _val, _psp_name)	\
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

#define wlc_charger_psy_setprop_event(_me, _psy, _psp, _val, _psp_name) \
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

#if I2C_SUSPEND_WORKAROUND
extern bool i2c_suspended;
#endif

struct unified_wlc_chip {
	struct device *dev;
	struct power_supply 	*psy_ext;
	struct power_supply	*psy_batt;
	struct power_supply wireless_psy;
	struct delayed_work wireless_interrupt_work;
	struct delayed_work wireless_set_online_work;
	struct delayed_work wireless_set_offline_work;
	struct delayed_work wireless_eoc_work;
#ifdef CONFIG_CHARGER_UNIFIED_WLC_ALIGNMENT
	struct delayed_work wireless_align_work;
#endif
	struct wake_lock wireless_chip_wake_lock;
	struct wake_lock wireless_eoc_wake_lock;

	unsigned int wlc_int_gpio;
	unsigned int wlc_full_chg;
#if I2C_SUSPEND_WORKAROUND
	struct delayed_work 	check_suspended_work;
	int suspended;
#endif /*I2C_SUSPEND_WORKAROUND */
	int		enabled;
#ifdef CONFIG_CHARGER_UNIFIED_WLC_ALIGNMENT
	struct mutex align_lock;
	unsigned int align_values;
#endif
};

static const struct platform_device_id unified_id[] = {
	{UNIFIED_WLC_DEV_NAME, 0},
	{},
};

static struct of_device_id unified_match[] = {
	{ .compatible = "unified_wlc", },
	{}
};

static struct unified_wlc_chip *the_chip;
static bool wireless_charging;

static enum power_supply_property pm_power_props_wireless[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_WIRELESS_CHARGER_SWITCH,
#ifdef CONFIG_CHARGER_UNIFIED_WLC_ALIGNMENT
	POWER_SUPPLY_PROP_ALIGNMENT,
#endif
};

static char *pm_power_supplied_to[] = {
	"battery",
};
int checking_insert_cnt = 0;
int checking_usb_insert_cnt = 0;

static int wlc_thermal_mitigation = -1;
static void wireless_inserted(struct unified_wlc_chip *chip);
static void wireless_removed(struct unified_wlc_chip *chip);

static int
set_wlc_thermal_mitigation(const char *val, struct kernel_param *kp)
{
	int ret;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}

	if (!the_chip) {
		pr_err("called before init\n");
		return ret;
	}

#ifdef CONFIG_LGE_THERMALE_CHG_CONTROL_FOR_WLC
	pr_info("%s : thermal-engine set wlc_thermal_mitigation to %d\n",
		__func__, wlc_thermal_mitigation);

	if(!(wlc_thermal_mitigation == THERMALE_ALL_CLEAR)
		&& !(wlc_thermal_mitigation >= THERMALE_SETTABLE_RANGE_MIN
		&& wlc_thermal_mitigation <= THERMALE_SETTABLE_RANGE_MAX)) {
		pr_err("invalid input!\n");
		return ret;
	}

	{
		struct unified_wlc_chip *chip = the_chip;
		wlc_charger_psy_setprop_event(chip, psy_ext,
			WIRELESS_THERMAL_MITIGATION, wlc_thermal_mitigation, _EXT_);
	}
#else
	pr_err("thermal-engine wlc chg current control not enabled\n");
#endif

	return 0;
}
module_param_call(wlc_thermal_mitigation, set_wlc_thermal_mitigation,
	param_get_int, &wlc_thermal_mitigation, 0644);

int is_wireless_charger_plugged_internal(struct unified_wlc_chip *chip)
{
	int ret = 0;

/*When WLC power is supplied, wlc_int_gpio is low
 *(Because it is connected to inverse ACOK)*/
	ret = !(gpio_get_value(chip->wlc_int_gpio));
	return ret;
}

#ifdef CONFIG_CHARGER_UNIFIED_WLC_ALIGNMENT
static int wireless_align_start(struct unified_wlc_chip *chip)
{
	int align = 0;

	chip->align_values = 0;
#ifdef CONFIG_CHARGER_UNIFIED_WLC_ALIGNMENT_IDT9025A
	align = idtp9025_align_start();
#elif defined(CONFIG_CHARGER_UNIFIED_WLC_ALIGNMENT_BQ5102X)
	align = bq5102x_align_start();
#endif
	if(align > 0)
		chip->align_values = align;

	return align;
}

static int wireless_align_stop(struct unified_wlc_chip *chip)
{
	int ret = 0;
#ifdef CONFIG_CHARGER_UNIFIED_WLC_ALIGNMENT_IDT9025A
	idtp9025_align_stop();
#elif defined(CONFIG_CHARGER_UNIFIED_WLC_ALIGNMENT_BQ5102X)
	bq5102x_align_stop();
#endif
	chip->align_values = 0;
	return ret;
}

static int wireless_align_get_value(struct unified_wlc_chip *chip)
{
	int align = 0;
	int align_changed = 0;

#ifdef CONFIG_CHARGER_UNIFIED_WLC_ALIGNMENT_IDT9025A
	align = idtp9025_align_get_value();
#elif defined(CONFIG_CHARGER_UNIFIED_WLC_ALIGNMENT_BQ5102X)
	align = bq5102x_align_get_value();
#endif

	if((align > 0) && (chip->align_values != align)) {
		pr_info("\n alignment has been changed!! [%d][%d] \n\n", chip->align_values, align);
		chip->align_values = align;
		align_changed = true;
	}

	return align_changed;
}
static void wireless_align_proc(struct unified_wlc_chip *chip,
									bool attached)
{
	if (likely(attached)) {
		/* start work queue for alignment */
		wireless_align_start(chip);
		if (likely(delayed_work_pending(&chip->wireless_align_work))) {
			flush_delayed_work_sync(&chip->wireless_align_work);
		}
		schedule_delayed_work(&chip->wireless_align_work,
					msecs_to_jiffies(WLC_ALIGN_INTERVAL));
	}
	else {
		cancel_delayed_work_sync(&chip->wireless_align_work);
		wireless_align_stop(chip);
	}
}
static void wireless_align_work(struct work_struct *work)
{
	struct unified_wlc_chip *chip = container_of(work,
				struct unified_wlc_chip, wireless_align_work.work);
	union power_supply_propval ret = {0,};
	int battery_capacity = 0;
	int align_changed = 0;

	if(!chip) return;

	chip->psy_batt = _psy_check_ext(chip->psy_batt, _BATTERY_);

	if( !chip->psy_batt ) return;


	chip->psy_batt->get_property(chip->psy_batt, POWER_SUPPLY_PROP_CAPACITY, &ret);
	battery_capacity = ret.intval;

	if((!wireless_charging) || (battery_capacity == 100)) {
		goto check_status;
	}

	mutex_lock(&chip->align_lock);
	align_changed = wireless_align_get_value(chip);
	mutex_unlock(&chip->align_lock);

	if(align_changed)
		power_supply_changed(&chip->wireless_psy);

check_status:
	schedule_delayed_work(&chip->wireless_align_work,
					msecs_to_jiffies(WLC_ALIGN_INTERVAL));
}
#endif

static int pm_power_set_property_wireless(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	struct unified_wlc_chip *chip =
			container_of(psy, struct unified_wlc_chip, wireless_psy);
	switch(psp) {
		case POWER_SUPPLY_PROP_WIRELESS_CHARGER_SWITCH:
			chip->enabled = val->intval;
			pr_info("%s : POWER_SUPPLY_PROP_WIRELESS_CHARGER_SWITCH : %d\n",
				__func__, chip->enabled);
			break;
		default:
			return -EINVAL;
	}
	return 0;
}
static int pm_power_get_property_wireless(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	struct unified_wlc_chip *chip =
			container_of(psy, struct unified_wlc_chip, wireless_psy);

	/* Check if called before init */
	/* todo workaround for below kmsg
	power_supply wireless: driver failed to report `present' property: 4294967274
	if (!the_chip)
		return -EINVAL;
	*/
	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = wireless_charging;
		break;
	case POWER_SUPPLY_PROP_WIRELESS_CHARGER_SWITCH:
		val->intval = chip->enabled;
		pr_info("%s : POWER_SUPPLY_PROP_WIRELESS_CHARGER_SWITCH : %d\n",
			__func__, val->intval);
		break;
#ifdef CONFIG_CHARGER_UNIFIED_WLC_ALIGNMENT
	case POWER_SUPPLY_PROP_ALIGNMENT:
		if (unlikely(!wireless_charging)) {
			val->intval = 0;
		} else {
			if(chip->align_values == 0) {
				mutex_lock(&chip->align_lock);
				wireless_align_get_value(chip);
				mutex_unlock(&chip->align_lock);
			}
			val->intval = chip->align_values;
		}
		break;
#endif
	default:
		return -EINVAL;
	}

	return 0;
}

static int pm_power_set_event_property_wireless(struct power_supply *psy,
		enum power_supply_event_type psp,
		const union power_supply_propval *val)
{
	struct unified_wlc_chip *chip =
			container_of(psy, struct unified_wlc_chip, wireless_psy);
	switch(psp) {
	case POWER_SUPPLY_PROP_WIRELESS_CHARGE_COMPLETED:
		pr_info("%s : ask POWER_SUPPLY_PROP_WIRELESS_CHARGE_COMPLETED", __func__);
		schedule_delayed_work(&chip->wireless_eoc_work,
			round_jiffies_relative(msecs_to_jiffies(2000)));
		break;
	case POWER_SUPPLY_PROP_WIRELESS_FAKE_OTG:
		if(val->intval == FAKE_DISCONNECTION) {
			// fake disconnect wlc
			wireless_removed(chip);
		}
		else {
			// fake online wlc
			wireless_inserted(chip);
		}
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int pm_power_get_event_property_wireless(struct power_supply *psy,
					 enum power_supply_event_type psp,
					 union power_supply_propval *val)
{
	struct unified_wlc_chip *chip =
			container_of(psy, struct unified_wlc_chip, wireless_psy);

	/* Check if called before init */
	/* todo workaround for below kmsg
	power_supply wireless: driver failed to report `present' property: 4294967274
	if (!the_chip)
		return -EINVAL;
	*/
	switch (psp) {
	case POWER_SUPPLY_PROP_WIRELESS_ONLINE:
		if (likely(chip)) {
			val->intval = is_wireless_charger_plugged_internal(chip);
			pr_info("%s : POWER_SUPPLY_PROP_WIRELESS_ONLINE :%d\n",
				__func__, val->intval);
		}
		break;
#ifdef CONFIG_LGE_THERMALE_CHG_CONTROL_FOR_WLC
	case POWER_SUPPLY_PROP_WIRELESS_THERMAL_MITIGATION:
		val->intval = wlc_thermal_mitigation;
		pr_info("%s : POWER_SUPPLY_PROP_WIRELESS_THERMAL_MITIGATION : %d\n",
			__func__, val->intval);
		break;
#endif
	default:
		return -EINVAL;
	}

	return 0;
}

/* Update WLC psy to online after checking vbus
  * Sometimes WLC OVP GPIO swing when USB is removed
  * because of remained WLC power on backcover
  * Check every 50msec from 200msec to 700msec */

static void wireless_set_online_work (struct work_struct *work)
{
	int wlc = 0;
	bool vbus = 0;
	union power_supply_propval ret = {0,};
	struct unified_wlc_chip *chip = container_of(work,
				struct unified_wlc_chip, wireless_set_online_work.work);
#ifdef CONFIG_CHARGER_UNIFIED_WLC_BQ5102X
	int ret_wlc;
#endif

	wlc = is_wireless_charger_plugged_internal(chip);
#if defined(CONFIG_SMB349_CHARGER)
	vbus = external_smb349_is_charger_present();
#elif defined(CONFIG_BQ24192_CHARGER)
	vbus = external_bq24192_is_charger_present();
#elif defined(CONFIG_BQ24296_CHARGER) || defined(CONFIG_CHARGER_MAX77819)
	wlc_charger_psy_getprop_event(chip, psy_ext, WIRELESS_DCIN_PRESENT,
		&ret, _EXT_);
	pr_info("[WLC] %s  PROP_PRESENT result : %d\n", __func__, ret.intval);
	vbus = ret.intval;
#endif

#ifdef CONFIG_CHARGER_UNIFIED_WLC_BQ5102X
	/* Set output voltage to 5.5V(0x2) */
	ret_wlc = bq5102x_set_output_voltage(0x2);
	if (ret_wlc < 0) {
		pr_err("[WLC] %s : failed to set output voltage\n", __func__);
	}

	/* Set output current to 90%(0x6) */
	ret_wlc = bq5102x_set_output_current(0x6);
	if (ret_wlc < 0) {
		pr_err("[WLC] %s : failed to set output currnet\n", __func__);
	}

	/* Set FOD calibration to 156mW(0x4) */
	ret_wlc = bq5102x_set_fod_calibration(0x4);
	if (ret_wlc < 0) {
		pr_err("[WLC] %s : failed to set FOD calibration\n", __func__);
	}
#endif

	checking_insert_cnt++;

	if (wlc && vbus) {
		pr_err("[WLC] %s : Wireless psy is updated to ONLINE !!!!!\n", __func__);
		wireless_charging = true;
		power_supply_changed(&chip->wireless_psy);
		return;
	}

	if (!wlc || (checking_insert_cnt > MAX_CHECKING_COUNT)) {
		pr_err("[WLC] %s : It was Ghost WLC (wlc(%d) vbus(%d))........count : %d\n"
			, __func__, wlc, vbus, checking_insert_cnt);
	} else {
		pr_err("[WLC] %s : Checking Ghost WLC (wlc(%d) vbus(%d)).......count : %d\n",
			__func__, wlc, vbus, checking_insert_cnt);
		schedule_delayed_work(&chip->wireless_set_online_work,
			round_jiffies_relative(msecs_to_jiffies(50)));
	}
}

/* Update USB psy to online after checking vbus
  * Sometimes SOURCE is switched from WLC to USB
  * without smb349 irq (Charger IRQ)
  * Check every 50msec from 500msec to 700msec */

static void wireless_set_offline_work(struct work_struct *work)
{
	int wlc = 0;
	bool vbus = 0;
	int usb_present = 0;
	union power_supply_propval ret = {0,};
	struct unified_wlc_chip *chip = container_of(work,
		struct unified_wlc_chip, wireless_set_offline_work.work);

	wlc = is_wireless_charger_plugged_internal(chip);
#if defined(CONFIG_SMB349_CHARGER)
	vbus = external_smb349_is_charger_present();
#elif defined(CONFIG_BQ24192_CHARGER)
	vbus = external_bq24192_is_charger_present();
#elif defined(CONFIG_BQ24296_CHARGER) || defined(CONFIG_CHARGER_MAX77819)
	wlc_charger_psy_getprop_event(chip, psy_ext, WIRELESS_DCIN_PRESENT,
		&ret, _EXT_);
	pr_info("[WLC] %s  PROP_PRESENT result : %d\n", __func__, ret.intval);
	vbus = ret.intval;
#endif
#if defined(CONFIG_SMB349_CHARGER) || defined(CONFIG_BQ24192_CHARGER)
	usb_present = get_usb_present();
#elif defined(CONFIG_BQ24296_CHARGER) || defined(CONFIG_CHARGER_MAX77819)
	wlc_charger_psy_getprop_event(chip, psy_ext, WIRELESS_USB_PRESENT,
		&ret, _EXT_);
	pr_info("[WLC] %s  USB_ONLINE result : %d\n", __func__, ret.intval);
	usb_present = ret.intval;
#endif
	checking_usb_insert_cnt++;

	if (!wlc) {
		/*Need to W/R*/
		if (!usb_present && vbus) {
			pr_err("[WLC] %s : USB is inserted and No SMB349 IRQ \n", __func__);
			pr_err("[WLC] %s : USB psy is updated to ONLINE !!!\n", __func__);

#if defined(CONFIG_SMB349_CHARGER) || defined(CONFIG_BQ24192_CHARGER)
			set_usb_present(1); /*W/R*/
#elif defined(CONFIG_BQ24296_CHARGER) || defined(CONFIG_CHARGER_MAX77819)
			ret.intval = 1;
			wlc_charger_psy_getprop_event(chip, psy_ext,
				WIRELESS_USB_PRESENT, &ret, _EXT_);
#endif
			if (wake_lock_active(&chip->wireless_chip_wake_lock)) {
				pr_err("[WLC] %s : RELEASE wakelock\n", __func__);
				wake_unlock(&chip->wireless_chip_wake_lock);
			}
			return;
		}

		/*Normal Case*/
		if (usb_present || (checking_usb_insert_cnt > MAX_USB_CHECKING_COUNT)) {
			pr_err("[WLC] %s : USB Already updated or W/O USB (usb(%d) cnt(%d)) \n"
						, __func__, usb_present, checking_usb_insert_cnt);
			if (wake_lock_active(&chip->wireless_chip_wake_lock)) {
				pr_err("[WLC] %s : RELEASE wakelock\n", __func__);
				wake_unlock(&chip->wireless_chip_wake_lock);
			}
			return;
		} else {
			pr_err("[WLC] %s : Checking USB ........ count : %d\n \n"
						, __func__, checking_usb_insert_cnt);
			schedule_delayed_work(&chip->wireless_set_offline_work,
					round_jiffies_relative(msecs_to_jiffies(1000)));
			return;
		}
	}
}

static void wireless_inserted(struct unified_wlc_chip *chip)
{
	pr_err("[WLC] %s \n", __func__);

	if (!wake_lock_active(&chip->wireless_chip_wake_lock)) {
		pr_err("[WLC] %s : ACQAURE wakelock\n", __func__);
		wake_lock(&chip->wireless_chip_wake_lock);
	}

#if defined(CONFIG_SMB349_CHARGER) || defined(CONFIG_BQ24192_CHARGER)
	set_wireless_power_supply_control(true);	/*Notify to Charger driver*/
#elif defined(CONFIG_BQ24296_CHARGER) || defined(CONFIG_CHARGER_MAX77819)
	wlc_charger_psy_setprop_event(chip, psy_ext,
		WIRELESS_CHARGE_ENABLED, 1, _EXT_);
#ifdef CONFIG_CHARGER_UNIFIED_WLC_ALIGNMENT
	wireless_align_proc(chip, true);
#endif
#endif
	checking_insert_cnt = 0;
	schedule_delayed_work(&chip->wireless_set_online_work,
			round_jiffies_relative(msecs_to_jiffies(200)));
}

static void wireless_removed(struct unified_wlc_chip *chip)
{
	pr_err("[WLC] %s \n", __func__);

#if defined(CONFIG_SMB349_CHARGER) || defined(CONFIG_BQ24192_CHARGER)
	set_wireless_power_supply_control(false);	/*Notify to Charger driver*/
#elif defined(CONFIG_BQ24296_CHARGER) || defined(CONFIG_CHARGER_MAX77819)
	wlc_charger_psy_setprop_event(chip, psy_ext,
		WIRELESS_CHARGE_ENABLED, 0, _EXT_);
#ifdef CONFIG_CHARGER_UNIFIED_WLC_ALIGNMENT
	wireless_align_proc(chip, false);
#endif
#endif

	checking_usb_insert_cnt = 0;
	pr_err("[WLC] %s : Wireless psy is updated to OFFLINE !!!\n", __func__);
	wireless_charging = false;
	power_supply_changed(&chip->wireless_psy);
	schedule_delayed_work(&chip->wireless_set_offline_work,
		round_jiffies_relative(msecs_to_jiffies(1000)));
}

static void wireless_interrupt_worker(struct work_struct *work)
{
	struct unified_wlc_chip *chip =
		container_of(work, struct unified_wlc_chip,
			 wireless_interrupt_work.work);

	pr_err("[WLC] %s \n", __func__);

	if (is_wireless_charger_plugged_internal(chip))
		wireless_inserted(chip);
	else{
		wireless_removed(chip);
	}
}
static bool get_bootmode(void)
{
	enum lge_boot_mode_type bootmode = LGE_BOOT_MODE_NORMAL;

	bootmode = lge_get_boot_mode();
	if (bootmode == LGE_BOOT_MODE_FACTORY ||
		bootmode == LGE_BOOT_MODE_FACTORY2 ||
		bootmode == LGE_BOOT_MODE_PIFBOOT ||
		bootmode == LGE_BOOT_MODE_PIFBOOT2) {
		return true;
	}
	else {
		return false;
	}
}
static irqreturn_t wireless_interrupt_handler(int irq, void *data)
{
	int chg_state;
	struct unified_wlc_chip *chip = data;

	chg_state = is_wireless_charger_plugged_internal(chip);
	if (chg_state)
		pr_err("[WLC] %s : I'm on the WLC PAD\n", __func__);
	else
		pr_err("[WLC] %s : I'm NOT on the WLC PAD\n", __func__);

	if(get_bootmode()) {
		pr_info("[WLC] Factory AFG Mode, notify to usb..\n");
		wlc_charger_psy_setprop_event(chip, psy_ext,
			WIRELESS_ONLINE_OTG, chg_state, _EXT_);
	}
#if I2C_SUSPEND_WORKAROUND
	schedule_delayed_work(&chip->check_suspended_work, msecs_to_jiffies(100));
#else
	schedule_work(&chip->wireless_interrupt_work.work);
#endif
	return IRQ_HANDLED;
}
#if I2C_SUSPEND_WORKAROUND
static void wlc_check_suspended_worker(struct work_struct *work)
{
	struct unified_wlc_chip *chip =
		container_of(work, struct unified_wlc_chip, check_suspended_work.work);

	if (chip->suspended && i2c_suspended) {
		pr_debug("WLC suspended. try i2c operation after 100ms.\n");
		schedule_delayed_work(&chip->check_suspended_work, msecs_to_jiffies(100));
	} else {
		pr_debug("WLC resumed. .\n");
		schedule_delayed_work(&chip->wireless_interrupt_work, 0);
	}
}
#endif /*I2C_SUSPEND_WORKAROUND*/

static void wireless_eoc_work(struct work_struct *work)
{
	struct unified_wlc_chip *chip = container_of(work,
		struct unified_wlc_chip, wireless_eoc_work.work);

	wake_lock(&chip->wireless_eoc_wake_lock);

	gpio_set_value(chip->wlc_full_chg, 1);
	pr_err("[WLC] %s : Send EPT signal!! \n", __func__);
	msleep(3500);
	gpio_set_value(chip->wlc_full_chg, 0);
	pr_err("[WLC] %s : Re-enable RX!! \n", __func__);

	wake_unlock(&chip->wireless_eoc_wake_lock);
}
#if 0
int wireless_charging_completed()
{
	pr_err("[WLC] %s \n", __func__);
	schedule_delayed_work(&the_chip->wireless_eoc_work,
			round_jiffies_relative(msecs_to_jiffies(2000)));
	return 1;
}

EXPORT_SYMBOL(wireless_charging_completed);
#endif
static int __devinit unified_wlc_hw_init(struct unified_wlc_chip *chip)
{
	int ret;
	pr_err("[WLC] %s \n", __func__);

	/* wlc_int_gpio DIR_IN and register IRQ */
	ret = gpio_request(chip->wlc_int_gpio, "wlc_int_gpio");
	if (ret < 0) {
		pr_err("[WLC] %s : failed to request gpio wlc_int\n", __func__);
		goto err_request_gpio1_failed;
	}

	ret = gpio_direction_input(chip->wlc_int_gpio);
	if (ret < 0) {
		pr_err("[WLC] %s : failed to set gpio wlc_int\n", __func__);
		goto err_request_irq_failed;
	}

	ret = request_irq(gpio_to_irq(chip->wlc_int_gpio),
			wireless_interrupt_handler,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"wireless_charger", chip);
	if (ret < 0) {
		pr_err("[WLC] %s : failed to request IRQ\n", __func__);
		goto err_request_irq_failed;
	}

	ret = enable_irq_wake(gpio_to_irq(chip->wlc_int_gpio));
	if (ret < 0) {
		pr_err("[WLC] %s : failed to request weake up IRQ\n", __func__);
		goto err_request_wakeup_irq_failed;
	}

#if defined(CONFIG_MACH_MSM8974_G3_VZW) || defined(CONFIG_MACH_MSM8974_G3_LRA)
	if (lge_get_board_revno() <= HW_REV_B){
		/* wlc_full_chg DIR_OUT and Low */
		ret = gpio_request_one(chip->wlc_full_chg, GPIOF_OUT_INIT_LOW,
				"wlc_full_chg");
	} else if (lge_get_board_revno() >= HW_REV_C) {
		/* wlc_full_chg DIR_OUT and OPEN_SOURCE */
		ret = gpio_request_one(chip->wlc_full_chg, GPIOF_DIR_OUT | GPIOF_OPEN_SOURCE,
				"wlc_full_chg");
	}
#else
	/* wlc_full_chg DIR_OUT and Low */
	ret = gpio_request_one(chip->wlc_full_chg, GPIOF_OUT_INIT_LOW,
			"wlc_full_chg");
#endif
	if (ret < 0) {
		pr_err("[WLC] %s : failed to request gpio wlc_full_chg\n", __func__);
		goto err_request_gpio2_failed;
	}

	return 0;

err_request_gpio2_failed:
	disable_irq_wake(gpio_to_irq(chip->wlc_int_gpio));
err_request_wakeup_irq_failed:
	free_irq(gpio_to_irq(chip->wlc_int_gpio), NULL);
err_request_irq_failed:
	gpio_free(chip->wlc_int_gpio);
err_request_gpio1_failed:

	return ret;

}

static int unified_wlc_resume(struct device *dev)
{
	struct unified_wlc_chip *chip = dev_get_drvdata(dev);

	if (!chip) {
		pr_err("called before init\n");
		return 0;
	}

#if I2C_SUSPEND_WORKAROUND
		chip->suspended = 0;
#endif /*I2C_SUSPEND_WORKAROUND*/
	return 0;
}

static int unified_wlc_suspend(struct device *dev)
{
	struct unified_wlc_chip *chip = dev_get_drvdata(dev);

	if (!chip) {
		pr_err("called before init\n");
		return 0;
	}

#if I2C_SUSPEND_WORKAROUND
	chip->suspended = 1;
#endif /*I2C_SUSPEND_WORKAROUND*/
	return 0;
}

static void unified_parse_dt(struct device *dev,
		struct unified_wlc_platform_data *pdata)
{
	struct device_node *np = dev->of_node;

	pdata->wlc_int_gpio = of_get_named_gpio(np, "wlc_int_gpio", 0);
	pdata->wlc_full_chg = of_get_named_gpio(np, "wlc_full_chg", 0);

}
static int wlc_charger_property_is_writeable(struct power_supply *psy,
	enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_WIRELESS_CHARGER_SWITCH:
		return 1;

	default:
		break;
	}

	return -EINVAL;
}


static int __devinit unified_wlc_probe(struct platform_device *pdev)
{
	int rc = 0;

	struct unified_wlc_chip *chip;
	struct unified_wlc_platform_data *pdata;

	pr_err("[WLC] %s : probe start\n", __func__);

	/*Read platform data from dts file*/

	pdata = devm_kzalloc(&pdev->dev,
					sizeof(struct unified_wlc_platform_data),
					GFP_KERNEL);
	if (!pdata) {
		pr_err("[WLC] %s : missing platform data\n", __func__);
		return -ENODEV;
	}

	if (pdev->dev.of_node) {
		pdev->dev.platform_data = pdata;
		unified_parse_dt(&pdev->dev, pdata);
	} else {
		pdata = pdev->dev.platform_data;
	}

	chip = kzalloc(sizeof(struct unified_wlc_chip), GFP_KERNEL);
	if (!chip) {
		pr_err("[WLC] %s : Cannot allocate unified_wlc_chip\n", __func__);
		return -ENOMEM;
	}

	chip->dev = &pdev->dev;

	/*Check charger IC probe is finshed or not , before starting probe*/
#if defined(CONFIG_SMB349_CHARGER)
	rc = smb349_is_ready();
	if (rc)
		goto free_chip;
#elif defined(CONFIG_BQ24192_CHARGER)
	rc = bq24192_is_ready();
	if (rc)
		goto free_chip;
#endif
#if defined(CONFIG_BQ24296_CHARGER) || defined(CONFIG_CHARGER_MAX77819)
	chip->psy_ext = power_supply_get_by_name(_EXT_);
	if (!likely(chip->psy_ext)) {
		pr_info("============ not found external power supply ============\n");
		rc = -EPROBE_DEFER;
		goto free_chip;
	} else {
		pr_info("============ psy ac supply found[%s] ============\n", chip->psy_ext->name);
	}
#endif

	/*Set Power Supply type for wlc*/
	chip->wireless_psy.name = "wireless";
	chip->wireless_psy.type = POWER_SUPPLY_TYPE_WIRELESS;
	chip->wireless_psy.supplied_to = pm_power_supplied_to;
	chip->wireless_psy.num_supplicants = ARRAY_SIZE(pm_power_supplied_to);
	chip->wireless_psy.properties = pm_power_props_wireless;
	chip->wireless_psy.num_properties = ARRAY_SIZE(pm_power_props_wireless);
	chip->wireless_psy.get_property = pm_power_get_property_wireless;
	chip->wireless_psy.set_property = pm_power_set_property_wireless;
	chip->wireless_psy.get_event_property = pm_power_get_event_property_wireless;
	chip->wireless_psy.set_event_property = pm_power_set_event_property_wireless;
	chip->wireless_psy.property_is_writeable	=
		wlc_charger_property_is_writeable;
	rc = power_supply_register(chip->dev, &chip->wireless_psy);
	if (rc < 0) {
		pr_err("[WLC] %s : power_supply_register wireless failed rx = %d\n", __func__, rc);
		goto free_chip;
	}

#if I2C_SUSPEND_WORKAROUND
	INIT_DELAYED_WORK(&chip->check_suspended_work, wlc_check_suspended_worker);
#endif /*I2C_SUSPEND_WORKAROUND*/
	INIT_DELAYED_WORK(&chip->wireless_interrupt_work, wireless_interrupt_worker);
	INIT_DELAYED_WORK(&chip->wireless_set_online_work, wireless_set_online_work);
	INIT_DELAYED_WORK(&chip->wireless_set_offline_work, wireless_set_offline_work);
	INIT_DELAYED_WORK(&chip->wireless_eoc_work, wireless_eoc_work);
#ifdef CONFIG_CHARGER_UNIFIED_WLC_ALIGNMENT
	INIT_DELAYED_WORK(&chip->wireless_align_work, wireless_align_work);
	mutex_init(&chip->align_lock);
	chip->align_values = 0;
#endif
	/*Set  Wake lock for wlc*/
	wake_lock_init(&chip->wireless_chip_wake_lock, WAKE_LOCK_SUSPEND,
				"unified_wireless_chip");
	wake_lock_init(&chip->wireless_eoc_wake_lock, WAKE_LOCK_SUSPEND,
				"unified_wireless_eoc");

	/*Set GPIO & Enable GPIO IRQ for wlc*/
	chip->wlc_int_gpio = pdata->wlc_int_gpio;
	pr_err("[WLC] %s : wlc_int_gpio = %d.\n", __func__, chip->wlc_int_gpio);

	chip->wlc_full_chg = pdata->wlc_full_chg;
	pr_err("[WLC] %s : wlc_full_chg = %d.\n", __func__, chip->wlc_full_chg);

	rc = unified_wlc_hw_init(chip);
	if (rc) {
		pr_err("[WLC] %s : couldn't init hardware rc = %d\n", __func__, rc);
		goto free_chip;
	}

	platform_set_drvdata(pdev, chip);

	the_chip = chip;

	/* For Booting Wireless_charging and For Power Charging Logo In Wireless Charging */
	if (is_wireless_charger_plugged_internal(chip)) {
		pr_err("[WLC] %s : I'm on WLC PAD during booting\n ", __func__);

#if defined(CONFIG_SMB349_CHARGER) || defined(CONFIG_BQ24192_CHARGER)
		set_usb_present(0); /*W/R*/
#elif defined(CONFIG_BQ24296_CHARGER) || defined(CONFIG_CHARGER_MAX77819)
		wlc_charger_psy_setprop_event(chip, psy_ext,
			WIRELESS_USB_PRESENT, 0, _EXT_);
#endif
		wireless_inserted(chip);
	}
	/*else
		wireless_removed(chip);*/
	chip->enabled = 0;
	pr_err("[WLC] %s : probe done\n", __func__);
	return 0;
/*
bail:
	power_supply_unregister(&chip->wireless_psy);
	gpio_free(chip->wlc_int_gpio);
	kfree(chip);
	return -EPROBE_DEFER;
*/
free_chip:
	kfree(chip);
	return rc;
}

static int __devexit unified_wlc_remove(struct platform_device *pdev)
{
	struct unified_wlc_chip *chip = platform_get_drvdata(pdev);

	pr_err("[WLC] %s :remove\n", __func__);
	wake_lock_destroy(&chip->wireless_chip_wake_lock);
	wake_lock_destroy(&chip->wireless_eoc_wake_lock);
	the_chip = NULL;
	platform_set_drvdata(pdev, NULL);
	power_supply_unregister(&chip->wireless_psy);
	free_irq(gpio_to_irq(chip->wlc_int_gpio), chip);
	gpio_free(chip->wlc_int_gpio);
	gpio_free(chip->wlc_full_chg);
	kfree(chip);
	return 0;
}

static const struct dev_pm_ops unified_pm_ops = {
	.suspend = unified_wlc_suspend,
	.resume = unified_wlc_resume,
};

static struct platform_driver unified_wlc_driver = {
	.probe = unified_wlc_probe,
	.remove = __devexit_p(unified_wlc_remove),
	.id_table = unified_id,
	.driver = {
		.name = UNIFIED_WLC_DEV_NAME,
		.owner = THIS_MODULE,
		.pm = &unified_pm_ops,
		.of_match_table = unified_match,
	},
};

static int __init unified_wlc_init(void)
{
	return platform_driver_register(&unified_wlc_driver);
}

static void __exit unified_wlc_exit(void)
{
	platform_driver_unregister(&unified_wlc_driver);
}

late_initcall(unified_wlc_init);
module_exit(unified_wlc_exit);

MODULE_AUTHOR("Kyungtae Oh <Kyungtae.oh@lge.com>");
MODULE_DESCRIPTION("unified Wireless Charger Control Driver");
MODULE_LICENSE("GPL v2");
