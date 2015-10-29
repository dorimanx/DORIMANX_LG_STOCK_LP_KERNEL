/*
 * MSM CPU Frequency Limiter Driver
 *
 * Copyright (c) 2012-2014, Paul Reioux Faux123 <reioux@gmail.com>
 * Copyright (c) 2014, Dorimanx <yuri@bynet.co.il>
 * Copyright (c) 2014, Pranav Vashi <neobuddy89@gmail.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <mach/cpufreq.h>
#include <linux/state_notifier.h>

#define MSM_CPUFREQ_LIMIT_MAJOR		5
#define MSM_CPUFREQ_LIMIT_MINOR		1

#define MSM_LIMIT			"msm_cpufreq_limit"

#define DEFAULT_MAX_SUSPEND_FREQUENCY	0
#define DEFAULT_MIN_SUSPEND_FREQUENCY	0
#define DEFAULT_MAX_RESUME_FREQUENCY	2265600
#define DEFAULT_MIN_RESUME_FREQUENCY	0

static unsigned int debug = 1;
module_param_named(debug_mask, debug, uint, 0644);

#define dprintk(msg...)		\
do {				\
	if (debug)		\
		pr_info(msg);	\
} while (0)

static struct cpu_limit {
	uint32_t suspend_max_freq;
	uint32_t suspend_min_freq;
	uint32_t resume_max_freq[4];
	uint32_t resume_min_freq[4];
	unsigned int suspended;
	struct mutex msm_limiter_mutex;
	struct notifier_block notif;
} limit = {
	.suspend_max_freq = DEFAULT_MAX_SUSPEND_FREQUENCY,
	.suspend_min_freq = DEFAULT_MIN_SUSPEND_FREQUENCY,
	.resume_max_freq[0] = DEFAULT_MAX_RESUME_FREQUENCY,
	.resume_max_freq[1] = DEFAULT_MAX_RESUME_FREQUENCY,
	.resume_max_freq[2] = DEFAULT_MAX_RESUME_FREQUENCY,
	.resume_max_freq[3] = DEFAULT_MAX_RESUME_FREQUENCY,
	.resume_min_freq[0] = DEFAULT_MIN_RESUME_FREQUENCY,
	.resume_min_freq[1] = DEFAULT_MIN_RESUME_FREQUENCY,
	.resume_min_freq[2] = DEFAULT_MIN_RESUME_FREQUENCY,
	.resume_min_freq[3] = DEFAULT_MIN_RESUME_FREQUENCY,
	.suspended = 0,
};

static void msm_limit_suspend(void)
{
	unsigned int cpu = 0;

	if (limit.suspended)
		return;

	/* Save current instance */
	for_each_possible_cpu(cpu) {
		limit.resume_max_freq[cpu] = get_max_lock(cpu);
		limit.resume_min_freq[cpu] = get_cpu_min_lock(cpu);
	}

	mutex_lock(&limit.msm_limiter_mutex);
	limit.suspended = 1;

	for_each_possible_cpu(cpu) {
		if (limit.suspend_min_freq) {
			cpufreq_set_freq(0, limit.suspend_min_freq, cpu);
			set_cpu_min_lock(cpu, limit.suspend_min_freq);
		}
		if (limit.suspend_max_freq) {
			cpufreq_set_freq(limit.suspend_max_freq, 0, cpu);
			set_max_lock(cpu, limit.suspend_max_freq);
		}
	}
	mutex_unlock(&limit.msm_limiter_mutex);

	if (limit.suspend_min_freq)
		dprintk("Limit all cores min freq to %d\n",
			limit.suspend_min_freq);
	if (limit.suspend_max_freq)
		dprintk("Limit all cores max freq to %d\n",
			limit.suspend_max_freq);
}

static void msm_limit_resume(void)
{
	unsigned int cpu = 0;

	/* Do not resume if didnt suspended */
	if (!limit.suspended)
		return;

	mutex_lock(&limit.msm_limiter_mutex);
	limit.suspended = 0;

	for_each_possible_cpu(cpu) {
		if (get_cpu_min_lock(cpu)) {
			set_cpu_min_lock(cpu, limit.resume_min_freq[cpu]);
			dprintk("Restore cpu%d min freq to %d\n",
					cpu, limit.resume_min_freq[cpu]);
		}
		if (get_max_lock(cpu)) {
			set_max_lock(cpu, limit.resume_max_freq[cpu]);
			dprintk("Restore cpu%d max freq to %d\n",
					cpu, limit.resume_max_freq[cpu]);
		}
		cpufreq_set_freq(get_max_lock(cpu),
				get_cpu_min_lock(cpu), cpu);
	}
	mutex_unlock(&limit.msm_limiter_mutex);
}

static int state_notifier_callback(struct notifier_block *this,
				unsigned long event, void *data)
{
	if (!limit.suspend_max_freq)
		return NOTIFY_OK;

	switch (event) {
		case STATE_NOTIFIER_ACTIVE:
			msm_limit_resume();
			break;
		case STATE_NOTIFIER_SUSPEND:
			msm_limit_suspend();
			break;
		default:
			break;
	}

	return NOTIFY_OK;
}

static int msm_cpufreq_limit_start(void)
{
	unsigned int cpu = 0;
	int ret = 0;

	limit.notif.notifier_call = state_notifier_callback;
	if (state_register_client(&limit.notif)) {
		pr_err("%s: Failed to register State notifier callback\n",
			MSM_LIMIT);
		goto err_dev;
	}

	mutex_init(&limit.msm_limiter_mutex);

	/* Save current instance */
	for_each_possible_cpu(cpu) {
		limit.resume_max_freq[cpu] = get_max_lock(cpu);
		limit.resume_min_freq[cpu] = get_cpu_min_lock(cpu);
	}

err_dev:
	return ret;
}

static void msm_cpufreq_limit_stop(void)
{
	limit.suspended = 1;
	mutex_destroy(&limit.msm_limiter_mutex);
	state_unregister_client(&limit.notif);
	limit.notif.notifier_call = NULL;
}

static ssize_t suspend_max_freq_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", limit.suspend_max_freq);
}

static ssize_t suspend_max_freq_store(struct kobject *kobj,
				      struct kobj_attribute *attr,
				      const char *buf, size_t count)
{
	int ret;
	unsigned int val;
	struct cpufreq_policy *policy = cpufreq_cpu_get(0);

	ret = sscanf(buf, "%u\n", &val);
	if (ret != 1)
		return -EINVAL;

	if (val == 0)
		goto out;

	if (val == limit.suspend_max_freq)
		return count;

	if (val < policy->cpuinfo.min_freq)
		val = policy->cpuinfo.min_freq;
	else if (val > policy->cpuinfo.max_freq)
		val = policy->cpuinfo.max_freq;

out:
	limit.suspend_max_freq = val;

	return count;
}

static ssize_t suspend_min_freq_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", limit.suspend_min_freq);
}

static ssize_t suspend_min_freq_store(struct kobject *kobj,
				      struct kobj_attribute *attr,
				      const char *buf, size_t count)
{
	int ret;
	unsigned int val;
	struct cpufreq_policy *policy = cpufreq_cpu_get(0);

	ret = sscanf(buf, "%u\n", &val);
	if (ret != 1)
		return -EINVAL;

	if (val == 0)
		goto out;

	if (val == limit.suspend_min_freq)
		return count;

	if (val < policy->cpuinfo.min_freq)
		val = policy->cpuinfo.min_freq;
	else if (val > policy->cpuinfo.max_freq)
		val = policy->cpuinfo.max_freq;

out:
	limit.suspend_min_freq = val;

	return count;
}

#define multi_cpu(cpu)					\
static ssize_t store_msm_cpufreq_max_limit_cpu##cpu	\
(struct kobject *kobj, 					\
 struct kobj_attribute *attr, 				\
 const char *buf, size_t count)				\
{							\
	int ret;					\
	unsigned int val;				\
	ret = sscanf(buf, "%u\n", &val);		\
	if (ret != 1)					\
		return -EINVAL;				\
	if (val == 0)					\
		goto out;				\
	if (val < 300000 || val > 2803200)		\
		val = 2265600;				\
out:							\
	mutex_lock(&limit.msm_limiter_mutex);		\
	set_max_lock(cpu, val);				\
	mutex_unlock(&limit.msm_limiter_mutex);		\
	return count;					\
}							\
static ssize_t show_msm_cpufreq_max_limit_cpu##cpu	\
(struct kobject *kobj,					\
 struct kobj_attribute *attr, char *buf)		\
{							\
	return sprintf(buf, "%u\n",			\
			get_max_lock(cpu));		\
}							\
static ssize_t store_msm_cpufreq_min_limit_cpu##cpu	\
(struct kobject *kobj,					\
 struct kobj_attribute *attr,				\
 const char *buf, size_t count)				\
{							\
	int ret;					\
	unsigned int val;				\
	ret = sscanf(buf, "%u\n", &val);		\
	if (ret != 1)					\
		return -EINVAL;				\
	if (val < 300000 || val > 2803200)		\
		val = 0;				\
	mutex_lock(&limit.msm_limiter_mutex);		\
	set_cpu_min_lock(cpu, val);			\
	mutex_unlock(&limit.msm_limiter_mutex);		\
	return count;					\
}							\
static ssize_t show_msm_cpufreq_min_limit_cpu##cpu	\
(struct kobject *kobj,					\
 struct kobj_attribute *attr, char *buf)		\
{							\
	return sprintf(buf, "%u\n",			\
		get_cpu_min_lock(cpu));			\
}							\
static struct kobj_attribute msm_cpufreq_max_limit_cpu##cpu =	\
	__ATTR(cpufreq_max_limit_cpu##cpu, 0666,		\
		show_msm_cpufreq_max_limit_cpu##cpu,		\
		store_msm_cpufreq_max_limit_cpu##cpu);		\
static struct kobj_attribute msm_cpufreq_min_limit_cpu##cpu =	\
	__ATTR(cpufreq_min_limit_cpu##cpu, 0666,		\
		show_msm_cpufreq_min_limit_cpu##cpu,		\
		store_msm_cpufreq_min_limit_cpu##cpu);		\

multi_cpu(0);
multi_cpu(1);
multi_cpu(2);
multi_cpu(3);

static ssize_t msm_cpufreq_limit_version_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "version: %u.%u\n",
			MSM_CPUFREQ_LIMIT_MAJOR, MSM_CPUFREQ_LIMIT_MINOR);
}

static struct kobj_attribute msm_cpufreq_limit_version_attribute =
	__ATTR(msm_cpufreq_limit_version, 0444,
		msm_cpufreq_limit_version_show,
		NULL);

static struct kobj_attribute suspend_max_freq_attribute =
	__ATTR(suspend_max_freq, 0666,
		suspend_max_freq_show,
		suspend_max_freq_store);

static struct kobj_attribute suspend_min_freq_attribute =
	__ATTR(suspend_min_freq, 0666,
		suspend_min_freq_show,
		suspend_min_freq_store);

static struct attribute *msm_cpufreq_limit_attrs[] =
	{
		&msm_cpufreq_max_limit_cpu0.attr,
		&msm_cpufreq_max_limit_cpu1.attr,
		&msm_cpufreq_max_limit_cpu2.attr,
		&msm_cpufreq_max_limit_cpu3.attr,
		&msm_cpufreq_min_limit_cpu0.attr,
		&msm_cpufreq_min_limit_cpu1.attr,
		&msm_cpufreq_min_limit_cpu2.attr,
		&msm_cpufreq_min_limit_cpu3.attr,
		&msm_cpufreq_limit_version_attribute.attr,
		&suspend_max_freq_attribute.attr,
		&suspend_min_freq_attribute.attr,
		NULL,
	};

static struct attribute_group msm_cpufreq_limit_attr_group =
	{
		.attrs = msm_cpufreq_limit_attrs,
	};

static struct kobject *msm_cpufreq_limit_kobj;

static int msm_cpufreq_limit_init(void)
{
	int ret;

	msm_cpufreq_limit_kobj =
		kobject_create_and_add(MSM_LIMIT, kernel_kobj);
	if (!msm_cpufreq_limit_kobj) {
		pr_err("%s: kobject create failed!\n",
			MSM_LIMIT);
		return -ENOMEM;
        }

	ret = sysfs_create_group(msm_cpufreq_limit_kobj,
			&msm_cpufreq_limit_attr_group);

        if (ret) {
		pr_err("%s: create failed!\n",
			MSM_LIMIT);
		goto err_dev;
	}

	msm_cpufreq_limit_start();

	return ret;
err_dev:
	if (msm_cpufreq_limit_kobj != NULL)
		kobject_put(msm_cpufreq_limit_kobj);
	return ret;
}

static void msm_cpufreq_limit_exit(void)
{
	if (msm_cpufreq_limit_kobj != NULL)
		kobject_put(msm_cpufreq_limit_kobj);

	msm_cpufreq_limit_stop();
}

module_init(msm_cpufreq_limit_init);
module_exit(msm_cpufreq_limit_exit);

MODULE_AUTHOR("Paul Reioux <reioux@gmail.com>, \
		Dorimanx <yuri@bynet.co.il>, \
		Pranav Vashi <neobuddy89@gmail.com>");
MODULE_DESCRIPTION("MSM Krait CPU Frequency Limiter Driver");
MODULE_LICENSE("GPL v2");
