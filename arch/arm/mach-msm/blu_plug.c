/*
 * Dynamic Hotplug for mako / hammerhead / shamu
 *
 * Copyright (C) 2013 Stratos Karafotis <stratosk@semaphore.gr> (dyn_hotplug for mako)
 *
 * Copyright (C) 2015 engstk <eng.stk@sapo.pt> (hammerhead & shamu implementation, fixes and changes to blu_plug)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#define DEBUG 0
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cpu.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/fb.h>
#include <soc/qcom/cpufreq.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/slab.h>

#define INIT_DELAY		(60 * HZ) /* Initial delay to 60 sec, 4 cores while boot */
#define DELAY			(HZ / 2)
#define UP_THRESHOLD		(80)
#define MIN_ONLINE		(1)
#define MAX_ONLINE		(4)
#define DEF_DOWN_TIMER_CNT	(6)	/* 3 secs */
#define DEF_UP_TIMER_CNT	(2)	/* 1 sec */
#define MAX_CORES_SCREENOFF     (1)
#define MAX_FREQ_SCREENOFF      (0)
#define MAX_FREQ_PLUG           (3091200)
#define DEF_PLUG_THRESHOLD      (70)
#define BLU_PLUG_ENABLED	(0)

static unsigned int blu_plug_enabled = BLU_PLUG_ENABLED;

static unsigned int up_threshold = UP_THRESHOLD;
static unsigned int delay = DELAY;
static unsigned int min_online = MIN_ONLINE;
static unsigned int max_online = MAX_ONLINE;
static unsigned int down_timer;
static unsigned int up_timer;
static unsigned int down_timer_cnt = DEF_DOWN_TIMER_CNT;
static unsigned int up_timer_cnt = DEF_UP_TIMER_CNT;
static unsigned int max_cores_screenoff = MAX_CORES_SCREENOFF;
static unsigned int max_freq_screenoff = MAX_FREQ_SCREENOFF;
static unsigned int max_freq_plug = MAX_FREQ_PLUG;
static unsigned int plug_threshold[MAX_ONLINE] = {[0 ... MAX_ONLINE-1] = DEF_PLUG_THRESHOLD};

static struct delayed_work dyn_work;
static struct workqueue_struct *dyn_workq;
static struct work_struct suspend, resume;
static struct notifier_block notify;


/* Bring online each possible CPU up to max_online cores */
static inline void up_all(void)
{
	unsigned int cpu;

	for_each_possible_cpu(cpu)
		if (cpu_is_offline(cpu) && num_online_cpus() < max_online)
			cpu_up(cpu);

	down_timer = 0;
}

/* Put offline each possible CPU down to min_online threshold */
static inline void down_all(void)
{
	unsigned int cpu;

	for_each_online_cpu(cpu)
		if (cpu && num_online_cpus() > min_online)
			cpu_down(cpu);
}

/* Iterate through possible CPUs and bring online the first offline found */
static inline void up_one(void)
{
	unsigned int cpu;

	/* All CPUs are online, return */
	if (num_online_cpus() == max_online)
		goto out;

	cpu = cpumask_next_zero(0, cpu_online_mask);
	if (cpu < nr_cpu_ids)
		cpu_up(cpu);
out:
	down_timer = 0;
	up_timer = 0;
}

/* Iterate through online CPUs and take offline the lowest loaded one */
static inline void down_one(void)
{
	unsigned int cpu;
	unsigned int l_cpu = 0;
	unsigned int l_freq = ~0;
	unsigned int p_cpu = 0;
	unsigned int p_thres = 0;
	bool all_equal = false;

	/* Min online CPUs, return */
	if (num_online_cpus() == min_online)
		goto out;

	get_online_cpus();

	for_each_online_cpu(cpu) {
		unsigned int thres = plug_threshold[cpu];
		
		if (!cpu || thres == p_thres) {
			p_thres = thres;
			p_cpu = cpu;
			all_equal = true;
		} else if (thres > p_thres) {
			p_thres = thres;
			p_cpu = cpu;
			all_equal = false;
		}
		
		if (cpu) {
			unsigned int cur = cpufreq_quick_get(cpu);

			if (l_freq > cur) {
				l_freq = cur;
				l_cpu = cpu;
			}
		}
	}

	put_online_cpus();

	if (all_equal)
		cpu_down(l_cpu);
	else
		cpu_down(p_cpu);
out:
	down_timer = 0;
	up_timer = 0;
}

/*
 * Every DELAY, check the average load of online CPUs. If the average load
 * is above up_threshold bring online one more CPU if up_timer has expired.
 * If the average load is below up_threshold offline one more CPU if the
 * down_timer has expired.
 */
static __ref void load_timer(struct work_struct *work)
{
	unsigned int cpu;
	unsigned int avg_load = 0;
	
	if (down_timer < down_timer_cnt)
		down_timer++;

	if (up_timer < up_timer_cnt)
		up_timer++;
	
	for_each_online_cpu(cpu)
		avg_load += cpufreq_quick_get_util(cpu);
		
	avg_load /= num_online_cpus();
	
#if DEBUG
	pr_debug("%s: avg_load: %u, num_online_cpus: %u\n", __func__, avg_load, num_online_cpus());
	pr_debug("%s: up_timer: %u, down_timer: %u\n", __func__, up_timer, down_timer);
#endif

	if (avg_load >= up_threshold && up_timer >= up_timer_cnt)
		up_one();
	else if (down_timer >= down_timer_cnt)
		down_one();

	queue_delayed_work_on(0, dyn_workq, &dyn_work, delay);
}

/* 
 * Manages driver behavior on screenoff mode
 * It sets max online CPUs to max_cores_screenoff and freq to max_freq_screenoff
 * Restores previous values on resume work
 *
 */
static __ref void max_screenoff(bool screenoff)
{
	uint32_t cpu, freq;
	
	if (screenoff) {
		max_freq_plug = cpufreq_quick_get_max(0);
		freq = min(max_freq_screenoff, max_freq_plug);

		cancel_delayed_work_sync(&dyn_work);
		
		for_each_possible_cpu(cpu) {
			msm_cpufreq_set_freq_limits(cpu, MSM_CPUFREQ_NO_LIMIT, freq);
			
			if (cpu && num_online_cpus() > max_cores_screenoff)
				cpu_down(cpu);
		}
		cpufreq_update_policy(cpu);
	}
	else {
		freq = max_freq_plug;
		
		up_all();
		
		for_each_possible_cpu(cpu) {
			msm_cpufreq_set_freq_limits(cpu, MSM_CPUFREQ_NO_LIMIT, freq);
		}
		cpufreq_update_policy(cpu);
		
		queue_delayed_work_on(0, dyn_workq, &dyn_work, delay);
	}
	
#if DEBUG
	pr_debug("%s: num_online_cpus: %u, freq_online_cpus: %u\n", __func__, num_online_cpus(), freq);
#endif
}

/* On suspend put offline all cores except cpu0*/
static void dyn_lcd_suspend(struct work_struct *work)
{	
	max_screenoff(true);
}

/* On resume bring online CPUs until max_online to prevent lags */
static void dyn_lcd_resume(struct work_struct *work)
{
	max_screenoff(false);
}

static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;

	if (evdata && evdata->data && event == FB_EVENT_BLANK) {
		blank = evdata->data;
		switch (*blank) {
			case FB_BLANK_UNBLANK:
				//display on
				queue_work_on(0, dyn_workq, &resume);
				break;
			case FB_BLANK_POWERDOWN:
			case FB_BLANK_HSYNC_SUSPEND:
			case FB_BLANK_VSYNC_SUSPEND:
			case FB_BLANK_NORMAL:
				//display off
				queue_work_on(0, dyn_workq, &suspend);
				break;
		}
	}

	return 0;
}

/******************** Module parameters *********************/

/* up_threshold */
static int set_up_threshold(const char *val, const struct kernel_param *kp)
{
	int ret = 0;
	unsigned int i;

	ret = kstrtouint(val, 10, &i);
	if (ret)
		return -EINVAL;
	if (i < 1 || i > 100)
		return -EINVAL;

	ret = param_set_uint(val, kp);

	return ret;
}

static struct kernel_param_ops up_threshold_ops = {
	.set = set_up_threshold,
	.get = param_get_uint,
};

module_param_cb(up_threshold, &up_threshold_ops, &up_threshold, 0644);

/* min_online */
static __ref int set_min_online(const char *val, const struct kernel_param *kp)
{
	int ret = 0;
	unsigned int i;

	ret = kstrtouint(val, 10, &i);
	if (ret)
		return -EINVAL;
	if (i < 1 || i > max_online || i > num_possible_cpus())
		return -EINVAL;

	ret = param_set_uint(val, kp);
	
	if (ret == 0) {
			up_all();
	}
	
	return ret;
}

static struct kernel_param_ops min_online_ops = {
	.set = set_min_online,
	.get = param_get_uint,
};

module_param_cb(min_online, &min_online_ops, &min_online, 0644);

/* max_online */
static __ref int set_max_online(const char *val, const struct kernel_param *kp)
{
	int ret = 0;
	unsigned int i;

	ret = kstrtouint(val, 10, &i);
	if (ret)
		return -EINVAL;
	if (i < 1 || i < min_online || i > num_possible_cpus())
		return -EINVAL;

	ret = param_set_uint(val, kp);
	
	if (ret == 0) {
		down_all();
		up_all();
	}
	
	return ret;
}

static struct kernel_param_ops max_online_ops = {
	.set = set_max_online,
	.get = param_get_uint,
};

module_param_cb(max_online, &max_online_ops, &max_online, 0644);

/* max_cores_screenoff */
static __ref int set_max_cores_screenoff(const char *val, const struct kernel_param *kp)
{
	int ret = 0;
	unsigned int i;

	ret = kstrtouint(val, 10, &i);
	if (ret)
		return -EINVAL;
	if (i < 1 || i > max_online || i > num_possible_cpus())
		return -EINVAL;
	if (i > max_online)
		max_cores_screenoff = max_online;

	ret = param_set_uint(val, kp);
	
	if (ret == 0) {
		down_all();
		up_all();
	}
	
	return ret;
}

static struct kernel_param_ops max_cores_screenoff_ops = {
	.set = set_max_cores_screenoff,
	.get = param_get_uint,
};

module_param_cb(max_cores_screenoff, &max_cores_screenoff_ops, &max_cores_screenoff, 0644);

/* max_freq_screenoff */
static int set_max_freq_screenoff(const char *val, const struct kernel_param *kp)
{
	int ret = MAX_FREQ_SCREENOFF;
	unsigned int i;

	ret = kstrtouint(val, 10, &i);
	if (ret)
		return -EINVAL;
	if (i < 35800 || i > 3091200)
		return -EINVAL;

	ret = param_set_uint(val, kp);
	
	return ret;
}

static struct kernel_param_ops max_freq_screenoff_ops = {
	.set = set_max_freq_screenoff,
	.get = param_get_uint,
};

module_param_cb(max_freq_screenoff, &max_freq_screenoff_ops, &max_freq_screenoff, 0644);

/* down_timer_cnt */
static int set_down_timer_cnt(const char *val, const struct kernel_param *kp)
{
	int ret = 0;
	unsigned int i;

	ret = kstrtouint(val, 10, &i);
	if (ret)
		return -EINVAL;
	if (i < 1 || i > 50)
		return -EINVAL;
		
	if (i < up_timer_cnt)
		down_timer_cnt = up_timer_cnt;

	ret = param_set_uint(val, kp);
	
	return ret;
}

static struct kernel_param_ops down_timer_cnt_ops = {
	.set = set_down_timer_cnt,
	.get = param_get_uint,
};

module_param_cb(down_timer_cnt, &down_timer_cnt_ops, &down_timer_cnt, 0644);

/* up_timer_cnt */
static int set_up_timer_cnt(const char *val, const struct kernel_param *kp)
{
	int ret = 0;
	unsigned int i;

	ret = kstrtouint(val, 10, &i);
	if (ret)
		return -EINVAL;
	if (i < 1 || i > 50)
		return -EINVAL;

	ret = param_set_uint(val, kp);

	return ret;
}

static struct kernel_param_ops up_timer_cnt_ops = {
	.set = set_up_timer_cnt,
	.get = param_get_uint,
};

module_param_cb(up_timer_cnt, &up_timer_cnt_ops, &up_timer_cnt, 0644);

/* plug_threshold */
module_param_array(plug_threshold, uint, NULL, 0644);

/***************** end of module parameters *****************/

static int dyn_hp_init(void)
{
	if (!blu_plug_enabled) {
		return 0;
	}
	notify.notifier_call = fb_notifier_callback;
	if (fb_register_client(&notify) != 0)
		pr_info("%s: Failed to register FB notifier callback\n", __func__);
	
	dyn_workq = alloc_workqueue("dyn_hotplug_workqueue", WQ_HIGHPRI | WQ_FREEZABLE, 0);
	if (!dyn_workq)
		return -ENOMEM;

	INIT_WORK(&resume, dyn_lcd_resume);
	INIT_WORK(&suspend, dyn_lcd_suspend);
	INIT_DELAYED_WORK(&dyn_work, load_timer);
	queue_delayed_work_on(0, dyn_workq, &dyn_work, INIT_DELAY);

	pr_info("%s: activated\n", __func__);

	return 0;
}

static void dyn_hp_exit(void)
{
	cancel_delayed_work_sync(&dyn_work);
	fb_unregister_client(&notify);
	destroy_workqueue(dyn_workq);
	
	pr_info("%s: deactivated\n", __func__);
}


/* enabled */
static int set_enabled(const char *val, const struct kernel_param *kp)
{
	int ret = 0;
	unsigned int i;
	int blu = 0;

	ret = kstrtouint(val, 10, &i);
	if (ret)
		return -EINVAL;
	if (i < 0 || i > 1)
		return 0;
		
	if (i == blu_plug_enabled)
		return i;

	ret = param_set_uint(val, kp);
	blu_plug_enabled = i;
	if ((blu_plug_enabled == 1))
		blu = dyn_hp_init();
	if ((blu_plug_enabled == 0))
		dyn_hp_exit();
	return i;
}

static struct kernel_param_ops enabled_ops = {
	.set = set_enabled,
	.get = param_get_uint,
};

module_param_cb(enabled, &enabled_ops, &blu_plug_enabled, 0644);

MODULE_AUTHOR("Stratos Karafotis <stratosk@semaphore.gr");
MODULE_AUTHOR("engstk <eng.stk@sapo.pt>");
MODULE_DESCRIPTION("'dyn_hotplug' - A dynamic hotplug driver for mako / hammerhead / shamu (blu_plug)");
MODULE_LICENSE("GPLv2");

late_initcall(dyn_hp_init);
module_exit(dyn_hp_exit);
