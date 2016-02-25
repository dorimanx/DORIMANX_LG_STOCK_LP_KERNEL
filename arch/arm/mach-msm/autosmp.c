/*
 * arch/arm/kernel/autosmp.c
 *
 * automatically hotplug/unplug multiple cpu cores
 * based on cpu load and suspend state
 *
 * based on the msm_mpdecision code by
 * Copyright (c) 2012-2013, Dennis Rassmann <showp1984@gmail.com>
 *
 * Copyright (C) 2013-2014, Rauf Gungor, http://github.com/mrg666
 * rewrite to simplify and optimize, Jul. 2013, http://goo.gl/cdGw6x
 * optimize more, generalize for n cores, Sep. 2013, http://goo.gl/448qBz
 * generalize for all arch, rename as autosmp, Dec. 2013, http://goo.gl/x5oyhy
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version. For more details, see the GNU
 * General Public License included with the Linux kernel or available
 * at www.gnu.org/licenses
 */

#include <linux/moduleparam.h>
#include <linux/cpufreq.h>
#include <linux/workqueue.h>
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/hrtimer.h>
#ifdef CONFIG_STATE_NOTIFIER
#include <linux/state_notifier.h>
#endif
#include <linux/mutex.h>

#define DEBUG 0

#define ASMP_TAG "AutoSMP:"
#define ASMP_STARTDELAY 10000

struct asmp_cpudata_t {
	long long unsigned int times_hotplugged;
};

static struct delayed_work asmp_work;
static struct workqueue_struct *asmp_workq;
static DEFINE_PER_CPU(struct asmp_cpudata_t, asmp_cpudata);
struct notifier_block notify;

static struct asmp_param_struct {
	unsigned int delay;
	unsigned int suspended;
	unsigned int max_cpus;
	unsigned int min_cpus;
	unsigned int cpufreq_up;
	unsigned int cpufreq_down;
	unsigned int cycle_up;
	unsigned int cycle_down;
	struct notifier_block notif;
	struct mutex autosmp_hotplug_mutex;
} asmp_param = {
	.delay = 100,
	.suspended = 0,
	.max_cpus = CONFIG_NR_CPUS,
	.min_cpus = 1,
	.cpufreq_up = 90,
	.cpufreq_down = 60,
	.cycle_up = 1,
	.cycle_down = 1,
};

static unsigned int cycle = 0;
static int autosmp_enabled __read_mostly = 0;
static int enable_switch = 0;
/*
 * suspend mode, if set = 1 hotplug will sleep,
 * if set = 0, then hoplug will be active all the time.
 */
static unsigned int hotplug_suspend = 0;
module_param_named(hotplug_suspend, hotplug_suspend, uint, 0644);

static void reschedule_hotplug_work(void)
{
	unsigned long delay;

	delay = msecs_to_jiffies(asmp_param.delay);
	queue_delayed_work(asmp_workq, &asmp_work, delay);
}

static void __cpuinit asmp_work_fn(struct work_struct *work)
{
	unsigned int cpu = 0, slow_cpu = 0;
	unsigned int rate, cpu0_rate, slow_rate = UINT_MAX, fast_rate;
	unsigned int max_rate, up_rate, down_rate;
	int nr_cpu_online;
	
	if (!autosmp_enabled)
		return;

	cycle++;
	
	/* get maximum possible freq for cpu0 and
	   calculate up/down limits */
	max_rate  = cpufreq_quick_get_max(cpu);
	up_rate   = (asmp_param.cpufreq_up * max_rate) / 100;
	down_rate = (asmp_param.cpufreq_down * max_rate) / 100;

	/* find current max and min cpu freq to estimate load */
	nr_cpu_online = num_online_cpus();
	cpu0_rate = cpufreq_quick_get(cpu);
	fast_rate = cpu0_rate;

	get_online_cpus();
	for_each_online_cpu(cpu) {
		if (cpu) {
			rate = cpufreq_quick_get(cpu);
			if (rate <= slow_rate) {
				slow_cpu = cpu;
				slow_rate = rate;
			} else if (rate > fast_rate)
				fast_rate = rate;
		}
	}
	put_online_cpus();

	if (cpu0_rate < slow_rate)
		slow_rate = cpu0_rate;

	/* hotplug one core if all online cores are over up_rate limit */
	if (slow_rate > up_rate) {
		if ((nr_cpu_online < asmp_param.max_cpus) &&
		    (cycle >= asmp_param.cycle_up)) {
			cpu = cpumask_next_zero(0, cpu_online_mask);
			cpu_up(cpu);
			cycle = 0;
#if DEBUG
			pr_info(ASMP_TAG"CPU [%d] On  | Mask [%d%d%d%d]\n",
				cpu, cpu_online(0), cpu_online(1), cpu_online(2), cpu_online(3));
#endif
		}
	/* unplug slowest core if all online cores are under down_rate limit */
	} else if (slow_cpu && (fast_rate < down_rate)) {
		if ((nr_cpu_online > asmp_param.min_cpus) &&
		    (cycle >= asmp_param.cycle_down)) {
 			cpu_down(slow_cpu);
			cycle = 0;
#if DEBUG
			pr_info(ASMP_TAG"CPU [%d] Off | Mask [%d%d%d%d]\n",
				slow_cpu, cpu_online(0), cpu_online(1), cpu_online(2), cpu_online(3));
			per_cpu(asmp_cpudata, cpu).times_hotplugged += 1;
#endif
		}
	} /* else do nothing */

	reschedule_hotplug_work();
}

#ifdef CONFIG_STATE_NOTIFIER
static void __ref asmp_suspend(void)
{
	unsigned int cpu;

	if (!hotplug_suspend)
		return;

	if (!asmp_param.suspended) {
		mutex_lock(&asmp_param.autosmp_hotplug_mutex);
		asmp_param.suspended = 1;
		mutex_unlock(&asmp_param.autosmp_hotplug_mutex);

		/* Flush hotplug workqueue */
		flush_workqueue(asmp_workq);
		cancel_delayed_work_sync(&asmp_work);

		/* unplug online cpu cores */
		for_each_online_cpu(cpu) {
			if (cpu == 0)
				continue;
			cpu_down(cpu);
		}
		/*
		 * Enabled core 1,2 so we will have 0-2 online
		 * when screen is OFF to reduce system lags and reboots.
		 */
		cpu_up(1);
		cpu_up(2);

		pr_info(ASMP_TAG"Screen -> Off. Suspended.\n");
	}
}

static void __ref asmp_resume(void)
{
	unsigned int cpu;
	int required_reschedule = 0, required_wakeup = 0;

	if (!hotplug_suspend)
		return;

	/* hotplug online cpu cores */
	if (asmp_param.suspended) {
		mutex_lock(&asmp_param.autosmp_hotplug_mutex);
		asmp_param.suspended = 0;
		mutex_unlock(&asmp_param.autosmp_hotplug_mutex);
		required_wakeup = 1;
		required_reschedule = 1;
		INIT_DELAYED_WORK(&asmp_work, asmp_work_fn);
		pr_info(ASMP_TAG"Screen -> On. Resumed.\n");
	}

	if (required_wakeup) {
		/* Fire up all CPUs */
		for_each_cpu_not(cpu, cpu_online_mask) {
			if (cpu == 0)
				continue;
			cpu_up(cpu);
		}
		pr_info(ASMP_TAG" wakeup boosted.\n");
	}

	/* Resume hotplug workqueue if required */
	if (required_reschedule)
		reschedule_hotplug_work();
}

static int state_notifier_callback(struct notifier_block *this,
				unsigned long event, void *data)
{
	if (!autosmp_enabled)
                return NOTIFY_OK;

	switch (event) {
		case STATE_NOTIFIER_ACTIVE:
			asmp_resume();
			break;
		case STATE_NOTIFIER_SUSPEND:
			asmp_suspend();
			break;
		default:
			break;
	}

	return NOTIFY_OK;
}
#endif

static int __cpuinit set_enabled(const char *val, const struct kernel_param *kp)
{
	int ret = 0;
	unsigned int cpu;

	ret = param_set_bool(val, kp);
	if (autosmp_enabled) {
		if (!enable_switch) {
			enable_switch = 1;
			INIT_DELAYED_WORK(&asmp_work, asmp_work_fn);
			queue_delayed_work_on(0, asmp_workq, &asmp_work,
					msecs_to_jiffies(asmp_param.delay));
			pr_info(ASMP_TAG "Enabled.\n");
		} else
			pr_info(ASMP_TAG "Already Enabled.\n");
	} else {
		enable_switch = 0;
		flush_workqueue(asmp_workq);
		cancel_delayed_work_sync(&asmp_work);
		/* Put all sibling cores to sleep */
		for_each_online_cpu(cpu) {
			if (cpu == 0)
				continue;
			cpu_down(cpu);
		}
		pr_info(ASMP_TAG "Disabled.\n");
	}
	return ret;
}

static struct kernel_param_ops module_ops = {
	.set = set_enabled,
	.get = param_get_bool,
};

module_param_cb(autosmp_enabled, &module_ops, &autosmp_enabled, 0644);
MODULE_PARM_DESC(autosmp_enabled, "hotplug/unplug cpu cores based on cpu load");

/***************************** SYSFS START *****************************/
#define define_one_global_ro(_name)					\
static struct global_attr _name =					\
__ATTR(_name, 0444, show_##_name, NULL)

#define define_one_global_rw(_name)					\
static struct global_attr _name =					\
__ATTR(_name, 0644, show_##_name, store_##_name)

struct kobject *asmp_kobject;

#define show_one(file_name, object)					\
static ssize_t show_##file_name						\
(struct kobject *kobj, struct attribute *attr, char *buf)		\
{									\
	return sprintf(buf, "%u\n", asmp_param.object);			\
}
show_one(delay, delay);
show_one(min_cpus, min_cpus);
show_one(max_cpus, max_cpus);
show_one(cpufreq_up, cpufreq_up);
show_one(cpufreq_down, cpufreq_down);
show_one(cycle_up, cycle_up);
show_one(cycle_down, cycle_down);

#define store_one(file_name, object)					\
static ssize_t store_##file_name					\
(struct kobject *a, struct attribute *b, const char *buf, size_t count)	\
{									\
	unsigned int input;						\
	int ret;							\
	ret = sscanf(buf, "%u", &input);				\
	if (ret != 1)							\
		return -EINVAL;						\
	asmp_param.object = input;					\
	return count;							\
}									\
define_one_global_rw(file_name);
store_one(delay, delay);
store_one(min_cpus, min_cpus);
store_one(max_cpus, max_cpus);
store_one(cpufreq_up, cpufreq_up);
store_one(cpufreq_down, cpufreq_down);
store_one(cycle_up, cycle_up);
store_one(cycle_down, cycle_down);

static struct attribute *asmp_attributes[] = {
	&delay.attr,
	&min_cpus.attr,
	&max_cpus.attr,
	&cpufreq_up.attr,
	&cpufreq_down.attr,
	&cycle_up.attr,
	&cycle_down.attr,
	NULL
};

static struct attribute_group asmp_attr_group = {
	.attrs = asmp_attributes,
	.name = "conf",
};

#if DEBUG
static ssize_t show_times_hotplugged(struct kobject *a,
					struct attribute *b, char *buf)
{
	ssize_t len = 0;
	int cpu;

	for_each_possible_cpu(cpu) {
		len += sprintf(buf + len, "%i %llu\n", cpu,
			per_cpu(asmp_cpudata, cpu).times_hotplugged);
	}
	return len;
}
define_one_global_ro(times_hotplugged);

static struct attribute *asmp_stats_attributes[] = {
	&times_hotplugged.attr,
	NULL
};

static struct attribute_group asmp_stats_attr_group = {
	.attrs = asmp_stats_attributes,
	.name = "stats",
};
#endif
/****************************** SYSFS END ******************************/

static int __init asmp_init(void)
{
	unsigned int cpu;
	int rc, ret = 0;

	asmp_param.max_cpus = nr_cpu_ids;
	for_each_possible_cpu(cpu)
		per_cpu(asmp_cpudata, cpu).times_hotplugged = 0;

	asmp_workq =
		alloc_workqueue("autosmp_wq", WQ_HIGHPRI | WQ_FREEZABLE, 0);
	if (!asmp_workq) {
		pr_err("%s: Failed to allocate hotplug workqueue\n",
			ASMP_TAG);
		ret = -ENOMEM;
		goto err_out;
	}

#ifdef CONFIG_STATE_NOTIFIER
	asmp_param.notif.notifier_call = state_notifier_callback;
	if (state_register_client(&asmp_param.notif)) {
		pr_err("%s: Failed to register State notifier callback\n",
			ASMP_TAG);
		goto err_dev;
	}
#endif

	mutex_init(&asmp_param.autosmp_hotplug_mutex);

	if (autosmp_enabled) {
		INIT_DELAYED_WORK(&asmp_work, asmp_work_fn);
		queue_delayed_work(asmp_workq, &asmp_work,
				   msecs_to_jiffies(ASMP_STARTDELAY));
	}

	asmp_kobject = kobject_create_and_add("autosmp", kernel_kobj);
	if (asmp_kobject) {
		rc = sysfs_create_group(asmp_kobject, &asmp_attr_group);
		if (rc) {
			pr_warn(ASMP_TAG "sysfs: ERROR, create sysfs group.");
			goto err_dev;
		}
#if DEBUG
		rc = sysfs_create_group(asmp_kobject, &asmp_stats_attr_group);
		if (rc) {
			pr_warn(ASMP_TAG "sysfs: ERROR, create sysfs stats group.");
			goto err_dev;
		}
#endif
	} else {
		pr_warn(ASMP_TAG "sysfs: ERROR, create sysfs kobj");
		goto err_dev;
	}

	pr_info(ASMP_TAG "Init complete.\n");
	return ret;

err_dev:
	destroy_workqueue(asmp_workq);
err_out:
	autosmp_enabled = 0;
	return ret;
}
late_initcall(asmp_init);
