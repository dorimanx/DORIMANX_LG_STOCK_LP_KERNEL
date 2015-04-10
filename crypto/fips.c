/*
 * FIPS 200 support.
 *
 * Copyright (c) 2008 Neil Horman <nhorman@tuxdriver.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 */

#include "internal.h"

int fips_enabled;
EXPORT_SYMBOL_GPL(fips_enabled);

static int fips_error;

void set_fips_error(void)
{
	fips_error = 1;
}

int get_fips_error_state(void)
{
	return fips_error;
}
EXPORT_SYMBOL_GPL(get_fips_error_state);

static int cc_mode;

int get_cc_mode_state(void)
{
	return cc_mode;
}
EXPORT_SYMBOL_GPL(get_cc_mode_state);

void fips_init_proc()
{
	crypto_init_proc(&fips_error, &cc_mode);
}
/* Process kernel command-line parameter at boot time. fips=0 or fips=1 */
static int fips_enable(char *str)
{
	fips_enabled = !!simple_strtol(str, NULL, 0);
	cc_mode = !!simple_strtol(str, NULL, 0);
	printk(KERN_INFO "fips mode: %s\n",
		fips_enabled ? "enabled" : "disabled");
	return 1;
}

static int cc_mode_enable(char *str)
{
	fips_enabled = !!simple_strtol(str, NULL, 0);
	cc_mode = !!simple_strtol(str, NULL, 0);
	printk(KERN_INFO "cc_mode: %s\n",
		cc_mode ? "enabled" : "disabled");
	return 1;
}

__setup("fips=", fips_enable);
__setup("cc_mode=", cc_mode_enable);
