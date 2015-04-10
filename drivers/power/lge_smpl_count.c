/*
 * driver/power/lge_smpl_count.c
 *
 * Copyright (C) 2013 LGE, Inc
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
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/qpnp/power-on.h>

#define MODULE_NAME "lge_smpl_count"
#define PWR_ON_EVENT_KEYPAD           0x80
#define PWR_ON_EVENT_CABLE            0x40
#define PWR_ON_EVENT_PON1             0x20
#define PWR_ON_EVENT_USB              0x10
#define PWR_ON_EVENT_DC               0x08
#define PWR_ON_EVENT_RTC              0x04
#define PWR_ON_EVENT_SMPL             0x02
#define PWR_ON_EVENT_HARD_RESET       0x01

extern uint16_t power_on_status_info_get(void);

static int dummy_arg;

struct lge_smpl_count_data {
	uint32_t smpl_boot;
};

static struct lge_smpl_count_data *data;


static int read_smpl_count(char *buffer, const struct kernel_param *kp)
{
	uint16_t boot_cause = 0;
	int warm_reset = 0;

	boot_cause = power_on_status_info_get();
	warm_reset = qpnp_pon_is_warm_reset();
	printk("[BOOT_CAUSE] %d, warm_reset = %d \n", boot_cause, warm_reset);

	if ((boot_cause &= PWR_ON_EVENT_SMPL) && (warm_reset == 0)) {
		printk("[SMPL_CNT] ===> is smpl boot\n");
		data->smpl_boot = 1;
	} else {
		printk("[SMPL_CNT] ===> not smpl boot!!!!!\n");
		data->smpl_boot = 0;
	}

	return sprintf(buffer, "%d", data->smpl_boot);
}
module_param_call(smpl_boot, NULL, read_smpl_count, &dummy_arg,
		S_IWUSR | S_IRUGO);


static int lge_smpl_count_probe(struct platform_device *pdev)
{
	int ret = 0;

	data = kmalloc(sizeof(struct lge_smpl_count_data),
			GFP_KERNEL);

	return ret;
}

static int __devexit
lge_smpl_count_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver lge_smpl_count_driver = {
	.probe = lge_smpl_count_probe,
	.remove = __devexit_p(lge_smpl_count_remove),
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
	},
};

static struct platform_device lge_smpl_count_device = {
	.name = MODULE_NAME,
	.dev = {
		.platform_data = NULL,
	}
};

static int __init lge_smpl_count_init(void)
{
	platform_device_register(&lge_smpl_count_device);

	return platform_driver_register(&lge_smpl_count_driver);
}

static void __exit lge_smpl_count_exit(void)
{
	kfree(data);
	platform_driver_unregister(&lge_smpl_count_driver);
}

module_init(lge_smpl_count_init);
module_exit(lge_smpl_count_exit);

MODULE_DESCRIPTION("LGE smpl_count");
MODULE_LICENSE("GPL");

