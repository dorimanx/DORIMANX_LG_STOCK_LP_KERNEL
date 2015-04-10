/*
 * Maxim MAX77819 Charger Driver Header
 *
 * Copyright (C) 2013 Maxim Integrated Product
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MAX77819_CHARGER_H__
#define __MAX77819_CHARGER_H__

/* Constant of special cases */
#define MAX77819_CHARGER_CURRENT_SUSPEND  (0)
#define MAX77819_CHARGER_CURRENT_UNLIMIT  (1)
#define MAX77819_CHARGER_CURRENT_MAXIMUM  (2)
#define MAX77819_CHARGER_CURRENT_MINIMUM  (3)

struct max77819_charger_platform_data {
    bool disable_interrupt;
    int irq;

    char *psy_name;
    char *ext_psy_name;

    char **supplied_to;
    size_t num_supplicants;

    u32 fast_charge_current;        /* in uA; 0.00A ~ 1.80A */
    u32 charge_termination_voltage; /* in uV; 4.10V ~ 4.35V */
    u32 topoff_timer;               /* in min; 0min ~ 60min, infinite */
    u32 topoff_current;             /* in uA; 50mA ~ 400mA */
    u32 charge_restart_threshold;   /* in uV; 100mV ~ 150mV */

    /* Co-operating charger */
    bool enable_coop;
    char *coop_psy_name;

    /* Temperature regulation */
    bool enable_thermistor;

    /* AICL control */
    bool enable_aicl;
    u32 aicl_detection_voltage;     /* in uV; 3.9V ~ 4.8V  */
    u32 aicl_reset_threshold;       /* in uV; 100mV or 200mV */
    int current_limit_usb;
    int current_limit_ac;
	int otg_en;
};

/* for compatibility with kernel not from android/kernel/msm */
#if !defined(CONFIG_LGE_PM)
#ifndef POWER_SUPPLY_PROP_CHARGING_ENABLED
#define POWER_SUPPLY_PROP_CHARGING_ENABLED_REPLACED
#define POWER_SUPPLY_PROP_CHARGING_ENABLED \
        POWER_SUPPLY_PROP_ONLINE
#endif
#endif
#ifndef POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT
#define POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT \
        POWER_SUPPLY_PROP_CURRENT_NOW
#endif
#ifndef POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX
#define POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX \
        POWER_SUPPLY_PROP_CURRENT_MAX
#endif

#endif /* !__MAX77819_CHARGER_H__ */
