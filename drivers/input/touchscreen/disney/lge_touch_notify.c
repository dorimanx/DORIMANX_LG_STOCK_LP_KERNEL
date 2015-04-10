/*
 *drivers/input/touchscreen/lge_touch_notify.c
 *
 * Copyright (C) 2014 LGE Inc
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

//#include "lge_touch_notify.h"
#include <linux/input/lge_touch_notify.h>
#include <linux/notifier.h>
#include <linux/export.h>

static BLOCKING_NOTIFIER_HEAD(touch_notifier_list);

/**
 *	touch_register_client - register a client notifier
 *	@nb: notifier block to callback on events
 */
int touch_register_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&touch_notifier_list, nb);
}
EXPORT_SYMBOL(touch_register_client);

/**
 *	touch_unregister_client - unregister a client notifier
 *	@nb: notifier block to callback on events
 */
int touch_unregister_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&touch_notifier_list, nb);
}
EXPORT_SYMBOL(touch_unregister_client);

/**
 *	touch_notifier_call_chain - notify clients on lcd_events
 *	@val: Value passed unmodified to notifier function
 *	@v: pointer passed unmodified to notifier function
 *
 */
int touch_notifier_call_chain(unsigned long val, void *v)
{
	return blocking_notifier_call_chain(&touch_notifier_list, val, v);
}
EXPORT_SYMBOL_GPL(touch_notifier_call_chain);
