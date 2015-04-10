/*
 *  Maxim MAX8971 Charger Driver Header
 *
 * Copyright 2013 Maxim Integrated Products, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MAX8971_H__
#define __MAX8971_H__

#define MAX8971_NAME  "max8971"

/* Constant of special cases */
#define MAX8971_CURRENT_SUSPEND  (0)
#define MAX8971_CURRENT_UNLIMIT  (1)
#define MAX8971_CURRENT_MAXIMUM  (2)
#define MAX8971_CURRENT_MINIMUM  (3)

struct max8971_platform_data {
    int irq;

    char *psy_name;
    char *ext_psy_name;

    char **supplied_to;
    size_t num_supplicants;

    int fast_charge_current;        /* in uA; 0.00A ~ 1.80A */
    int charge_termination_voltage; /* in uV; 4.10V ~ 4.35V */
    int topoff_timer;               /* in min; 0min ~ 70min */
    int topoff_current;             /* in uA; 50mA ~ 200mA */
    int charge_restart_threshold;   /* in uV; 100mV ~ 150mV */

    /* Co-operating charger */
    bool enable_coop;
    char *coop_psy_name;

    /* Temperature regulation */
    bool enable_thermistor;

    /* AICL control */
    bool enable_aicl;

    /* Run time PM */
    atomic_t suspened;
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

#endif /* __MAX8971_H__ */
