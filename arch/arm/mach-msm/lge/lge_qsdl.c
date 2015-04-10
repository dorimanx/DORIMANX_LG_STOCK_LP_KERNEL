/* Copyright (c) 2014 LG Electronics, Inc.
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

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of.h>

#include <mach/lge_handle_panic.h>
#include <mach/lge_qsdl.h>
#include <mach/board_lge.h>

#define is_crash(reason)     ((reason & 0xFFFF0000) == LGE_RB_MAGIC)
#define which_reason(reason) (reason & 0x0000FFFF)

/* uevent version */
struct qsdl_uevent_table {
	int reason;
	char *uevent_key[2];
};

/* sysfs version */
struct qsdl_sysfs_table {
	int reason;
	char *uevent_key;
};

struct qsdl {
	struct platform_device *pdev;
	bool oneshot_read;
	bool using_uevent;
};

static struct device *lge_qsdl_dev;
static unsigned int modem_ssr_count;

int lge_qsdl_trigger_modem_uevent(void)
{
	char *noti_modem_info[2] = {"QSDL=Q005", NULL};
	char **uevent_envp = NULL;

	if (lge_qsdl_dev) {
		uevent_envp = noti_modem_info;
		kobject_uevent_env(&lge_qsdl_dev->kobj, KOBJ_CHANGE,
				uevent_envp);
		pr_debug("%s: Notify modem qsdl infomation\n", __func__);
	} else {
		pr_debug("%s: Fail to Notify modem qsdl infomation\n", __func__);
	}

	return 0;
}

int lge_qsdl_increase_modem_ssr(void)
{
	modem_ssr_count++;
	return 0;
}

static int lge_qsdl_trigger_notify(struct platform_device *pdev)
{
	struct qsdl_uevent_table info[] = {
		{0x0301, {"QSDL=Q002", NULL}, },
		{0x0302, {"QSDL=Q002", NULL}, },
		{0x0303, {"QSDL=Q002", NULL}, },
		{0x0304, {"QSDL=Q002", NULL}, },
		{0x0308, {"QSDL=Q002", NULL}, },
		{0x0400, {"QSDL=Q002", NULL}, },
		{0x0100, {"QSDL=Q003", NULL}, },
		{0x3001, {"QSDL=Q005", NULL}, },
		{0x3002, {"QSDL=Q005", NULL}, },
	};
	int i, reboot_reason = 0;
	char **uevent_envp = NULL;
	struct device *dev = &pdev->dev;

	reboot_reason = lge_get_bootreason();
	pr_debug("%s: reboot reason - 0x%x\n", __func__, reboot_reason);

	if ((reboot_reason == -1) || !is_crash(reboot_reason))
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(info); i++) {
		pr_debug("%s: searching reason - (0x%x - 0x%x)\n", __func__,
				which_reason(reboot_reason), info[i].reason);
		if (which_reason(reboot_reason) == info[i].reason) {
			/* send notify to userspace via uevent */
			if (dev) {
				uevent_envp = info[i].uevent_key;
				kobject_uevent_env(&dev->kobj, KOBJ_CHANGE,
						uevent_envp);
				pr_debug("%s: sent QSDL notify uevent to userspace - %d\n",
						__func__, i);
			}
			break;
		}
	}

	return 0;
}

static ssize_t lge_qsdl_trigger_show(struct device *dev,
		struct device_attribute *attr, char *buffer)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct qsdl *data = platform_get_drvdata(pdev);

	int ret = 0;
	static bool already_read = false;

	if (!data->using_uevent)
		return snprintf(buffer, PAGE_SIZE, "Nothing to report");

	if (data->oneshot_read) {
		if (already_read)
			return snprintf(buffer, PAGE_SIZE, "Already read");

		already_read = true;
	}

	ret = lge_qsdl_trigger_notify(pdev);
	if (ret == -EINVAL)
		return snprintf(buffer, PAGE_SIZE, "Nothing to report");

	return snprintf(buffer, PAGE_SIZE, "Sent report via uevent");
}

static ssize_t lge_qsdl_apps_info_show(struct device *dev,
		struct device_attribute *attr, char *buffer)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct qsdl *data = platform_get_drvdata(pdev);

	struct qsdl_sysfs_table info[] = {
		{0x0301, "Q002"},
		{0x0302, "Q002"},
		{0x0303, "Q002"},
		{0x0304, "Q002"},
		{0x0308, "Q002"},
		{0x0400, "Q002"},
		{0x0100, "Q003"},
		{0x3001, "Q005"},
		{0x3002, "Q005"},
	};

	int i, reboot_reason = 0;
	static bool already_read = false;

	if (data->oneshot_read) {
		if (already_read) {
			pr_info("%s: Already read infomation, not permitted\n", __func__);
			goto nothing;
		}

		already_read = true;
	}

	reboot_reason = lge_get_bootreason();
	if ((reboot_reason == -1) || !is_crash(reboot_reason))
		goto nothing;

	for (i = 0; i < ARRAY_SIZE(info); i++) {
		if (which_reason(reboot_reason) == info[i].reason)
			break;
	}

	/* etc case */
	if (i >= ARRAY_SIZE(info))
		goto nothing;

	/* found it */
	return snprintf(buffer, PAGE_SIZE, "%s", info[i].uevent_key);

nothing:
	return snprintf(buffer, PAGE_SIZE, "Q000");
}

static ssize_t lge_qsdl_modem_info_show(struct device *dev,
		struct device_attribute *attr, char *buffer)
{
	/*
	struct platform_device *pdev = to_platform_device(dev);
	struct qsdl *data = platform_get_drvdata(pdev);
	*/

	const char noti_modem_info[] = "Q005";
	unsigned int count = modem_ssr_count;

	/* reset count */
	modem_ssr_count = 0;

	return snprintf(buffer, PAGE_SIZE, "%s=%d", noti_modem_info,
			count);
}

static DEVICE_ATTR(trigger, S_IRUGO, lge_qsdl_trigger_show, NULL);
static DEVICE_ATTR(apps_info, S_IRUGO, lge_qsdl_apps_info_show, NULL);
static DEVICE_ATTR(modem_info, S_IRUGO, lge_qsdl_modem_info_show, NULL);

static struct attribute *lge_qsdl_uevent_attrs[] = {
	&dev_attr_trigger.attr,
	&dev_attr_apps_info.attr,
	&dev_attr_modem_info.attr,
	NULL
};

static struct attribute *lge_qsdl_attrs[] = {
	&dev_attr_apps_info.attr,
	&dev_attr_modem_info.attr,
	NULL
};

static const struct attribute_group lge_qsdl_uevent_files = {
	.attrs  = lge_qsdl_uevent_attrs,
};

static const struct attribute_group lge_qsdl_files = {
	.attrs  = lge_qsdl_attrs,
};

static int __init lge_qsdl_handler_probe(struct platform_device *pdev)
{
	struct lge_qsdl_platform_data *pdata = pdev->dev.platform_data;
	struct qsdl *data;
	int ret = 0;

	pr_debug("%s: probe enter\n", __func__);

	data = devm_kzalloc(&pdev->dev, sizeof(struct qsdl), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	/* use legacy platform data to get static sysfs path */
	if (pdata) {
		data->oneshot_read = pdata->oneshot_read;
		data->using_uevent = pdata->using_uevent;
	} else {
		pr_err("%s: probe fail - no platform data\n", __func__);
		devm_kfree(&pdev->dev, data);
		return -ENODEV;
	}

	if (data->using_uevent)
		ret = sysfs_create_group(&pdev->dev.kobj, &lge_qsdl_uevent_files);
	else
		ret = sysfs_create_group(&pdev->dev.kobj, &lge_qsdl_files);

	if (ret)
		goto fail_sysfs_group;

	data->pdev = pdev;

	/* for triggering modem event */
	lge_qsdl_dev = &pdev->dev;
	platform_set_drvdata(pdev, data);

	pr_debug("%s: probe done\n", __func__);
	return 0;

fail_sysfs_group:
	devm_kfree(&pdev->dev, data);
	pr_err("%s: probe fail - %d\n", __func__, ret);
	return ret;
}

static int __devexit lge_qsdl_handler_remove(struct platform_device *pdev)
{
	struct qsdl *data = platform_get_drvdata(pdev);
	kfree(data);
	return 0;
}

static struct platform_driver qsdl_handler_driver __refdata = {
	.probe = lge_qsdl_handler_probe,
	.remove = __devexit_p(lge_qsdl_handler_remove),
	.driver = {
		.name = LGE_QSDL_DEV_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init lge_qsdl_handler_init(void)
{
	return platform_driver_register(&qsdl_handler_driver);
}

static void __exit lge_qsdl_handler_exit(void)
{
	platform_driver_unregister(&qsdl_handler_driver);
}

module_init(lge_qsdl_handler_init);
module_exit(lge_qsdl_handler_exit);

MODULE_DESCRIPTION("LGE qsdl handler driver");
MODULE_AUTHOR("Hyeon H. Park <hyunhui.park@lge.com>");
MODULE_LICENSE("GPL");
