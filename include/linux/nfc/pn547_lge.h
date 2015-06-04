/*
 *
 * Copyright (C) 2010 NXP Semiconductors
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */
#ifndef _PN547_LGE_H_
#define _PN547_LGE_H_
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>

#include <mach/board_lge.h>

#define PN547_MAGIC	0xE9

#define PN547_DRV_NAME	"pn547"

/*
 * pn547 power control via ioctl
 * pn547_SET_PWR(0): power off
 * pn547_SET_PWR(1): power on
 * pn547_SET_PWR(2): reset and power on with firmware download enabled
 */
#define pn547_SET_PWR	_IOW(PN547_MAGIC, 0x01, unsigned int)

#define pn547_HW_REVISION _IOR(PN547_MAGIC, 0x02, unsigned int)

struct pn547_i2c_platform_data {
    unsigned int sda_gpio;
    unsigned int scl_gpio;
    unsigned int irq_gpio;
    unsigned int ven_gpio;
    unsigned int firm_gpio;
};

struct pn547_dev {
    wait_queue_head_t   read_wq;
    struct mutex        read_mutex;
    struct i2c_client   *client;
    struct miscdevice   pn547_device;
    unsigned int        ven_gpio;
    unsigned int        firm_gpio;
    unsigned int        irq_gpio;
#ifdef CONFIG_LGE_NFC_USE_PMIC
    struct clk			*clk_cont;
    struct clk			*clk_pin;
#endif
    bool            irq_enabled;
    spinlock_t      irq_enabled_lock;
};

struct pn547_gpio {
    unsigned int        sda_gpio;
    unsigned int        scl_gpio;
    unsigned int        ven_gpio;
    unsigned int        firm_gpio;
    unsigned int        irq_gpio;
};

#if defined(CONFIG_LGE_NFC_DEBUG_MESSAGE)
#define dprintk(fmt, args...) printk(fmt, ##args)
#else
#define dprintk(fmt, args...) do { } while (0)
#endif

#endif /*               */
