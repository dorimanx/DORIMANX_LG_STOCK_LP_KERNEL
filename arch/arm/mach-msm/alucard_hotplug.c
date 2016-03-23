/*
 * Author: Alucard_24@XDA
 *
 * Copyright 2012 Alucard_24@XDA
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
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/jiffies.h>
#include <linux/kernel_stat.h>
#include <linux/tick.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/slab.h>

#ifdef CONFIG_STATE_NOTIFIER
#include <linux/state_notifier.h>
#endif

/*
 * alucard_hotplug_mutex protects hotplug start/stop phase.
 */
static DEFINE_MUTEX(alucard_hotplug_mutex);

struct hotplug_cpuinfo {
	u64 prev_cpu_wall;
	u64 prev_cpu_idle;
	unsigned int prev_load;
	ktime_t time_stamp;
};

static unsigned int last_online_cpus;

struct hotplug_cpuparm {
	unsigned int up_load;
	unsigned int up_freq;
	unsigned int up_rq;
	unsigned int up_rate;
	unsigned int cur_up_rate;
	unsigned int down_load;
	unsigned int down_freq;
	unsigned int down_rq;
	unsigned int down_rate;
	unsigned int cur_down_rate;
};

static DEFINE_PER_CPU_SHARED_ALIGNED(struct hotplug_cpuinfo, ac_hp_cpuinfo);
static DEFINE_PER_CPU(struct hotplug_cpuparm, ac_hp_cpuparm);

static struct notifier_block notif;
static struct delayed_work alucard_hotplug_work;
static struct workqueue_struct *alucard_hp_wq;

static struct hotplug_tuners {
	unsigned int hotplug_sampling_rate;
	unsigned int hotplug_enable;
	unsigned int min_cpus_online;
	unsigned int maxcoreslimit;
	unsigned int maxcoreslimit_sleep;
	unsigned int hotplug_suspend;
	bool suspended;
} hotplug_tuners_ins = {
#ifdef CONFIG_MACH_JF
	.hotplug_sampling_rate = 50,
	.hotplug_enable = 1,
#else
	.hotplug_sampling_rate = 50,
	.hotplug_enable = 0,
#endif
	.min_cpus_online = 1,
	.maxcoreslimit = NR_CPUS,
	.maxcoreslimit_sleep = 3,
	.hotplug_suspend = 0,
	.suspended = false,
};

#define DOWN_INDEX		(0)
#define UP_INDEX		(1)

static unsigned int adjust_avg_freq(unsigned int cpu, unsigned int tmp_freq)
{
	struct cpufreq_frequency_table *table = cpufreq_frequency_get_table(cpu);
	unsigned int i = 0, target_freq = 0;

	if (!table)
		return 0;

	for (i = 0; (table[i].frequency != CPUFREQ_TABLE_END); i++) {
		unsigned int freq = table[i].frequency;

		if (freq != CPUFREQ_ENTRY_INVALID) {
			if (freq < tmp_freq) {
				target_freq = freq;
			} else if (freq >= tmp_freq) {
				if (freq - tmp_freq < tmp_freq - target_freq) {
					target_freq = freq;
				}
				break;
			}
		}
	}

	return target_freq;
}

static void reset_last_cur_cpu_rate(void)
{
	struct hotplug_cpuparm *pcpu_parm = NULL;

	if (unlikely(!last_online_cpus))
		return;
	pcpu_parm = &per_cpu(ac_hp_cpuparm, last_online_cpus - 1);
	pcpu_parm->cur_up_rate = 1;
	pcpu_parm->cur_down_rate = 1;
}

static void __ref hotplug_work_fn(struct work_struct *work)
{
	struct hotplug_cpuparm *pcpu_parm = NULL;
	unsigned int upmaxcoreslimit =
		hotplug_tuners_ins.maxcoreslimit;
	unsigned int min_cpus_online =
		hotplug_tuners_ins.min_cpus_online;
	unsigned int downcpu = 0;
	unsigned int offcpu = NR_CPUS;
	unsigned int cpu = 0;
	unsigned int rq_avg = 0;
	unsigned int min_freq = MAX_FREQ_LIMIT;
	unsigned int avg_freq = 0;
	unsigned int min_load = 100;
	unsigned int avg_load = 0;
	unsigned int n = 0;
	unsigned int sum_load = 0, sum_freq = 0;
	bool rq_avg_calc = true;
	int online_cpus = 0, delay;
	unsigned int sampling_rate =
		hotplug_tuners_ins.hotplug_sampling_rate;

	if (hotplug_tuners_ins.suspended) {
		upmaxcoreslimit = hotplug_tuners_ins.maxcoreslimit_sleep;
		rq_avg_calc = false;
	}

	for_each_online_cpu(cpu) {
		struct hotplug_cpuinfo *pcpu_info =
			&per_cpu(ac_hp_cpuinfo, cpu);
		u64 cur_wall_time, cur_idle_time;
		unsigned int wall_time, idle_time;
		unsigned int cur_load = 0;
		unsigned int cur_freq = 0;
		ktime_t time_now = ktime_get();
		s64 delta_us;

		delta_us = ktime_us_delta(time_now, pcpu_info->time_stamp);
		/* Do nothing if cpu recently has become online */
		if (delta_us < (s64)(sampling_rate / 2)) {
			continue;
		}

		cur_idle_time = get_cpu_idle_time(
				cpu, &cur_wall_time,
				0);

		wall_time = (unsigned int)
				(cur_wall_time -
					pcpu_info->prev_cpu_wall);
		pcpu_info->prev_cpu_wall = cur_wall_time;

		idle_time = (unsigned int)
				(cur_idle_time -
					pcpu_info->prev_cpu_idle);
		pcpu_info->prev_cpu_idle = cur_idle_time;

		pcpu_info->time_stamp = time_now;

		/* if wall_time < idle_time or wall_time == 0, evaluate cpu load next time */
		if (unlikely(!wall_time || wall_time < idle_time))
			continue;

		/*
		 * If the CPU had gone completely idle, and a task just woke up
		 * on this CPU now, it would be unfair to calculate 'load' the
		 * usual way for this elapsed time-window, because it will show
		 * near-zero load, irrespective of how CPU intensive that task
		 * actually is. This is undesirable for latency-sensitive bursty
		 * workloads.
		 *
		 * To avoid this, we reuse the 'load' from the previous
		 * time-window and give this task a chance to start with a
		 * reasonably high CPU frequency. (However, we shouldn't over-do
		 * this copy, lest we get stuck at a high load (high frequency)
		 * for too long, even when the current system load has actually
		 * dropped down. So we perform the copy only once, upon the
		 * first wake-up from idle.)
		 *
		 * Detecting this situation is easy: the governor's deferrable
		 * timer would not have fired during CPU-idle periods. Hence
		 * an unusually large 'wall_time' (as compared to the sampling
		 * rate) indicates this scenario.
		 *
		 * prev_load can be zero in two cases and we must recalculate it
		 * for both cases:
		 * - during long idle intervals
		 * - explicitly set to zero
		 */
		if (unlikely(wall_time > (2 * sampling_rate) &&
			     pcpu_info->prev_load)) {
			cur_load = pcpu_info->prev_load;

			/*
			 * Perform a destructive copy, to ensure that we copy
			 * the previous load only once, upon the first wake-up
			 * from idle.
			 */
			pcpu_info->prev_load = 0;
		} else {
			cur_load = 100 * (wall_time - idle_time) / wall_time;
			pcpu_info->prev_load = cur_load;
		}

		/* get the cpu current frequency */
		cur_freq = cpufreq_quick_get(cpu);

		if (cur_freq > 0) {
			n++;
			sum_load += cur_load;
			sum_freq += cur_freq;
			if (rq_avg_calc) {
				rq_avg += (avg_cpu_nr_running(cpu) * 100) >> FSHIFT;
				/*pr_info("cpu[%u], rq_avg[%u]\n", cpu, rq_avg);*/
			}
			if (cur_load < min_load 
				 && cur_freq < min_freq
				 && cpu > 0) {
				min_load = cur_load;
				min_freq = cur_freq;
				downcpu = cpu;
			}
		}
	}
	if (unlikely(!sum_freq))
		goto next_loop;

	avg_freq = adjust_avg_freq(BOOT_CPU, (sum_freq / n));

	if (avg_freq) {
		avg_load = (sum_load / n);
		/* get nr online cpus */
		online_cpus = num_online_cpus();
		if (online_cpus != last_online_cpus) {
			reset_last_cur_cpu_rate();
			last_online_cpus = online_cpus;
		}
		cpu = (online_cpus - 1);
		pcpu_parm = &per_cpu(ac_hp_cpuparm, cpu);
		/* get the first offline cpu */
		offcpu = cpumask_next_zero(0, cpu_online_mask);
		if (downcpu > 0	&& 
			 online_cpus > upmaxcoreslimit) {
				cpu_down(downcpu);
		} else if (online_cpus < min_cpus_online
					&& offcpu < upmaxcoreslimit) {
					cpu_up(offcpu);
		} else if (offcpu > 0
			&& offcpu < upmaxcoreslimit
			&& online_cpus < upmaxcoreslimit
			&& avg_load >= pcpu_parm->up_load
			&& avg_freq >= pcpu_parm->up_freq
			&& (rq_avg > pcpu_parm->up_rq
				 || rq_avg_calc == false)) {
				if (pcpu_parm->cur_up_rate %
						pcpu_parm->up_rate == 0) {
	#if 0
					pr_info("CPU[%u], \
						avg_freq[%u], avg_load[%u], \
						rq_avg[%u], up_rate[%u]\n",
						offcpu, avg_freq,
						avg_load, rq_avg,
						pcpu_parm->cur_up_rate);
	#endif
					cpu_up(offcpu);
				} else {
					if (pcpu_parm->cur_up_rate < pcpu_parm->up_rate)
						++pcpu_parm->cur_up_rate;
					else
						pcpu_parm->cur_up_rate = 1;
				}
		} else if (downcpu >= min_cpus_online && (
				avg_load < pcpu_parm->down_load
				|| (avg_freq <= pcpu_parm->down_freq
				&& (rq_avg <= pcpu_parm->down_rq
					 || rq_avg_calc == false)))) {
					if (pcpu_parm->cur_down_rate %
							pcpu_parm->down_rate == 0) {
	#if 0
						pr_info("CPU[%u], \
							avg_freq[%u], \
							avg_load[%u], \
							rq_avg[%u], \
							down_rate[%u]\n",
							downcpu, avg_freq,
							avg_load, rq_avg,
							pcpu_parm->
							cur_down_rate);
	#endif
						cpu_down(downcpu);
					} else {
						if (pcpu_parm->cur_down_rate < pcpu_parm->down_rate)
							++pcpu_parm->cur_down_rate;
						else
							pcpu_parm->cur_down_rate = 1;
					}
		} else {
			pcpu_parm->cur_up_rate = 1;
			pcpu_parm->cur_down_rate = 1;
		}
	}

next_loop:
	delay = msecs_to_jiffies(sampling_rate);
	/*
	 * We want hotplug governor to do sampling
	 * just one jiffy later on cpu governor sampling
	*/
	delay = delay - (jiffies % delay) + 1;

	queue_delayed_work_on(BOOT_CPU, alucard_hp_wq,
				&alucard_hotplug_work,
				delay);
}

#ifdef CONFIG_STATE_NOTIFIER
static void alucard_hotplug_suspend(void)
{
	if (hotplug_tuners_ins.hotplug_suspend == 1 &&
				hotplug_tuners_ins.suspended == false) {
			hotplug_tuners_ins.suspended = true;
			pr_info("Alucard HotPlug suspended.\n");
	}
}

static void alucard_hotplug_resume(void)
{
	if (hotplug_tuners_ins.suspended == true) {
			hotplug_tuners_ins.suspended = false;
			pr_info("Alucard HotPlug Resumed.\n");
	}
}

static int state_notifier_callback(struct notifier_block *this,
				unsigned long event, void *data)
{
	if (hotplug_tuners_ins.hotplug_enable == 0)
		return NOTIFY_OK;

	switch (event) {
		case STATE_NOTIFIER_ACTIVE:
			alucard_hotplug_resume();
			break;
		case STATE_NOTIFIER_SUSPEND:
			alucard_hotplug_suspend();
			break;
		default:
			break;
	}

	return NOTIFY_OK;
}
#endif

static int alucard_hotplug_callback(struct notifier_block *nb,
			unsigned long action, void *data)
{
	unsigned int cpu = (unsigned long)data;
	struct hotplug_cpuinfo *pcpu_info = NULL;
	unsigned int prev_load;

	switch (action) {
	case CPU_ONLINE:
		pcpu_info = &per_cpu(ac_hp_cpuinfo, cpu);
		pcpu_info->prev_cpu_idle = get_cpu_idle_time(cpu,
				&pcpu_info->prev_cpu_wall,
				0);
		pcpu_info->time_stamp = ktime_get();
		prev_load = (unsigned int)
				(pcpu_info->prev_cpu_wall - pcpu_info->prev_cpu_idle);
		pcpu_info->prev_load = 100 * prev_load /
				(unsigned int) pcpu_info->prev_cpu_wall;
		break;
	default:
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block alucard_hotplug_nb =
{
   .notifier_call = alucard_hotplug_callback,
};

static int hotplug_start(void)
{
	unsigned int cpu;
	int delay;

	mutex_lock(&alucard_hotplug_mutex);
	hotplug_tuners_ins.suspended = false;

	alucard_hp_wq = alloc_workqueue("alu_hp_wq", WQ_HIGHPRI | 
			WQ_MEM_RECLAIM | 
			WQ_UNBOUND | 
			__WQ_ORDERED, 0);
	if (!alucard_hp_wq) {
		printk(KERN_ERR "Failed to create alu_hp_wq workqueue\n");
		mutex_unlock(&alucard_hotplug_mutex);
		return -EINVAL;
	}

	get_online_cpus();
	for_each_possible_cpu(cpu) {
		struct hotplug_cpuinfo *pcpu_info = NULL;
		struct hotplug_cpuparm *pcpu_parm =
			&per_cpu(ac_hp_cpuparm, cpu);
		unsigned int prev_load;

		if (cpu_online(cpu)) {
			pcpu_info = &per_cpu(ac_hp_cpuinfo, cpu);
			pcpu_info->prev_cpu_idle = get_cpu_idle_time(cpu,
					&pcpu_info->prev_cpu_wall,
					0);
			pcpu_info->time_stamp = ktime_get();
			prev_load = (unsigned int)
				(pcpu_info->prev_cpu_wall - pcpu_info->prev_cpu_idle);
			pcpu_info->prev_load = 100 * prev_load /
					(unsigned int) pcpu_info->prev_cpu_wall;
		}
		pcpu_parm->cur_up_rate = 1;
		pcpu_parm->cur_down_rate = 1;
	}
	last_online_cpus = num_online_cpus();
	register_hotcpu_notifier(&alucard_hotplug_nb);
	put_online_cpus();

	delay = msecs_to_jiffies(hotplug_tuners_ins.hotplug_sampling_rate);
	/*
	 * We want hotplug governor to do sampling
	 * just one jiffy later on cpu governor sampling
	*/
	delay = delay - (jiffies % delay) + 1;

	INIT_DEFERRABLE_WORK(&alucard_hotplug_work, hotplug_work_fn);
	queue_delayed_work_on(BOOT_CPU, alucard_hp_wq,
				&alucard_hotplug_work,
				delay);

#ifdef CONFIG_STATE_NOTIFIER
	notif.notifier_call = state_notifier_callback;
	if (state_register_client(&notif))
		pr_err("Failed to register State notifier callback for Alucard Hotplug\n");
#endif
	mutex_unlock(&alucard_hotplug_mutex);

	return 0;
}

static void hotplug_stop(void)
{
	mutex_lock(&alucard_hotplug_mutex);
#ifdef CONFIG_STATE_NOTIFIER
	state_unregister_client(&notif);
	notif.notifier_call = NULL;
#endif
	cancel_delayed_work_sync(&alucard_hotplug_work);
	get_online_cpus();
	unregister_hotcpu_notifier(&alucard_hotplug_nb);
	put_online_cpus();
	destroy_workqueue(alucard_hp_wq);
	mutex_unlock(&alucard_hotplug_mutex);
}

#define show_one(file_name, object)					\
static ssize_t show_##file_name						\
(struct kobject *kobj, struct attribute *attr, char *buf)		\
{									\
	return sprintf(buf, "%d\n",					\
			hotplug_tuners_ins.object);			\
}

show_one(hotplug_sampling_rate, hotplug_sampling_rate);
show_one(hotplug_enable, hotplug_enable);
show_one(min_cpus_online, min_cpus_online);
show_one(maxcoreslimit, maxcoreslimit);
show_one(maxcoreslimit_sleep, maxcoreslimit_sleep);
show_one(hotplug_suspend, hotplug_suspend);

#define show_pcpu_param(file_name, var_name, num_core)			\
static ssize_t show_##file_name						\
(struct kobject *kobj, struct attribute *attr, char *buf)		\
{									\
	struct hotplug_cpuparm *pcpu_parm =				\
			&per_cpu(ac_hp_cpuparm, num_core - 1);	\
	return sprintf(buf, "%u\n",					\
			pcpu_parm->var_name);				\
}

show_pcpu_param(hotplug_freq_1_1, up_freq, 1);
show_pcpu_param(hotplug_freq_2_1, up_freq, 2);
show_pcpu_param(hotplug_freq_3_1, up_freq, 3);
show_pcpu_param(hotplug_freq_2_0, down_freq, 2);
show_pcpu_param(hotplug_freq_3_0, down_freq, 3);
show_pcpu_param(hotplug_freq_4_0, down_freq, 4);

show_pcpu_param(hotplug_load_1_1, up_load, 1);
show_pcpu_param(hotplug_load_2_1, up_load, 2);
show_pcpu_param(hotplug_load_3_1, up_load, 3);
show_pcpu_param(hotplug_load_2_0, down_load, 2);
show_pcpu_param(hotplug_load_3_0, down_load, 3);
show_pcpu_param(hotplug_load_4_0, down_load, 4);

show_pcpu_param(hotplug_rq_1_1, up_rq, 1);
show_pcpu_param(hotplug_rq_2_1, up_rq, 2);
show_pcpu_param(hotplug_rq_3_1, up_rq, 3);
show_pcpu_param(hotplug_rq_2_0, down_rq, 2);
show_pcpu_param(hotplug_rq_3_0, down_rq, 3);
show_pcpu_param(hotplug_rq_4_0, down_rq, 4);

show_pcpu_param(hotplug_rate_1_1, up_rate, 1);
show_pcpu_param(hotplug_rate_2_1, up_rate, 2);
show_pcpu_param(hotplug_rate_3_1, up_rate, 3);
show_pcpu_param(hotplug_rate_2_0, down_rate, 2);
show_pcpu_param(hotplug_rate_3_0, down_rate, 3);
show_pcpu_param(hotplug_rate_4_0, down_rate, 4);

#define store_pcpu_param(file_name, var_name, num_core)			\
static ssize_t store_##file_name					\
(struct kobject *kobj, struct attribute *attr,				\
	const char *buf, size_t count)					\
{									\
	unsigned int input;						\
	struct hotplug_cpuparm *pcpu_parm;				\
	int ret;							\
									\
	ret = sscanf(buf, "%u", &input);				\
	if (ret != 1)							\
		return -EINVAL;						\
									\
	pcpu_parm = &per_cpu(ac_hp_cpuparm, num_core - 1);		\
									\
	if (input == pcpu_parm->var_name) {				\
		return count;						\
	}								\
									\
	pcpu_parm->var_name = input;					\
	return count;							\
}

store_pcpu_param(hotplug_freq_1_1, up_freq, 1);
store_pcpu_param(hotplug_freq_2_1, up_freq, 2);
store_pcpu_param(hotplug_freq_3_1, up_freq, 3);
store_pcpu_param(hotplug_freq_2_0, down_freq, 2);
store_pcpu_param(hotplug_freq_3_0, down_freq, 3);
store_pcpu_param(hotplug_freq_4_0, down_freq, 4);

store_pcpu_param(hotplug_load_1_1, up_load, 1);
store_pcpu_param(hotplug_load_2_1, up_load, 2);
store_pcpu_param(hotplug_load_3_1, up_load, 3);
store_pcpu_param(hotplug_load_2_0, down_load, 2);
store_pcpu_param(hotplug_load_3_0, down_load, 3);
store_pcpu_param(hotplug_load_4_0, down_load, 4);

store_pcpu_param(hotplug_rq_1_1, up_rq, 1);
store_pcpu_param(hotplug_rq_2_1, up_rq, 2);
store_pcpu_param(hotplug_rq_3_1, up_rq, 3);
store_pcpu_param(hotplug_rq_2_0, down_rq, 2);
store_pcpu_param(hotplug_rq_3_0, down_rq, 3);
store_pcpu_param(hotplug_rq_4_0, down_rq, 4);

store_pcpu_param(hotplug_rate_1_1, up_rate, 1);
store_pcpu_param(hotplug_rate_2_1, up_rate, 2);
store_pcpu_param(hotplug_rate_3_1, up_rate, 3);
store_pcpu_param(hotplug_rate_2_0, down_rate, 2);
store_pcpu_param(hotplug_rate_3_0, down_rate, 3);
store_pcpu_param(hotplug_rate_4_0, down_rate, 4);

define_one_global_rw(hotplug_freq_1_1);
define_one_global_rw(hotplug_freq_2_0);
define_one_global_rw(hotplug_freq_2_1);
define_one_global_rw(hotplug_freq_3_0);
define_one_global_rw(hotplug_freq_3_1);
define_one_global_rw(hotplug_freq_4_0);

define_one_global_rw(hotplug_load_1_1);
define_one_global_rw(hotplug_load_2_0);
define_one_global_rw(hotplug_load_2_1);
define_one_global_rw(hotplug_load_3_0);
define_one_global_rw(hotplug_load_3_1);
define_one_global_rw(hotplug_load_4_0);

define_one_global_rw(hotplug_rq_1_1);
define_one_global_rw(hotplug_rq_2_0);
define_one_global_rw(hotplug_rq_2_1);
define_one_global_rw(hotplug_rq_3_0);
define_one_global_rw(hotplug_rq_3_1);
define_one_global_rw(hotplug_rq_4_0);

define_one_global_rw(hotplug_rate_1_1);
define_one_global_rw(hotplug_rate_2_0);
define_one_global_rw(hotplug_rate_2_1);
define_one_global_rw(hotplug_rate_3_0);
define_one_global_rw(hotplug_rate_3_1);
define_one_global_rw(hotplug_rate_4_0);

static void cpus_hotplugging(int status)
{
	int ret = 0;

	if (status) {
		ret = hotplug_start();
	} else {
		hotplug_stop();
	}

	if (!ret)
		hotplug_tuners_ins.hotplug_enable = status;
}

/* hotplug_sampling_rate */
static ssize_t store_hotplug_sampling_rate(struct kobject *a,
				struct attribute *b,
				const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	input = max(input, 10);

	if (input == hotplug_tuners_ins.hotplug_sampling_rate)
		return count;

	hotplug_tuners_ins.hotplug_sampling_rate = input;

	return count;
}

/* hotplug_enable */
static ssize_t store_hotplug_enable(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	input = input > 0;

	if (hotplug_tuners_ins.hotplug_enable == input)
		return count;

	if (input > 0)
		cpus_hotplugging(1);
	else
		cpus_hotplugging(0);

	return count;
}

/* min_cpus_online */
static ssize_t store_min_cpus_online(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	input = max(input > NR_CPUS ? NR_CPUS : input, 1);

	if (hotplug_tuners_ins.min_cpus_online == input)
		return count;

	hotplug_tuners_ins.min_cpus_online = input;

	return count;
}

/* maxcoreslimit */
static ssize_t store_maxcoreslimit(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	input = max(input > NR_CPUS ? NR_CPUS : input, 1);

	if (hotplug_tuners_ins.maxcoreslimit == input)
		return count;

	hotplug_tuners_ins.maxcoreslimit = input;

	return count;
}

/* maxcoreslimit_sleep */
static ssize_t store_maxcoreslimit_sleep(struct kobject *a,
				struct attribute *b,
				const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	input = max(input > NR_CPUS ? NR_CPUS : input, 1);

	if (hotplug_tuners_ins.maxcoreslimit_sleep == input)
		return count;

	hotplug_tuners_ins.maxcoreslimit_sleep = input;

	return count;
}

/*
 * hotplug_suspend control
 * if set = 1 hotplug will sleep,
 * if set = 0, then hoplug will be active all the time.
 */
static ssize_t store_hotplug_suspend(struct kobject *a,
				struct attribute *b,
				const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	input = input > 0;

	if (hotplug_tuners_ins.hotplug_suspend == input)
		return count;

	if (input > 0)
		hotplug_tuners_ins.hotplug_suspend = 1;
	else
		hotplug_tuners_ins.hotplug_suspend = 0;

	return count;
}

define_one_global_rw(hotplug_sampling_rate);
define_one_global_rw(hotplug_enable);
define_one_global_rw(min_cpus_online);
define_one_global_rw(maxcoreslimit);
define_one_global_rw(maxcoreslimit_sleep);
define_one_global_rw(hotplug_suspend);

static struct attribute *alucard_hotplug_attributes[] = {
	&hotplug_sampling_rate.attr,
	&hotplug_enable.attr,
	&hotplug_freq_1_1.attr,
	&hotplug_freq_2_0.attr,
	&hotplug_freq_2_1.attr,
	&hotplug_freq_3_0.attr,
	&hotplug_freq_3_1.attr,
	&hotplug_freq_4_0.attr,
	&hotplug_load_1_1.attr,
	&hotplug_load_2_0.attr,
	&hotplug_load_2_1.attr,
	&hotplug_load_3_0.attr,
	&hotplug_load_3_1.attr,
	&hotplug_load_4_0.attr,
	&hotplug_rq_1_1.attr,
	&hotplug_rq_2_0.attr,
	&hotplug_rq_2_1.attr,
	&hotplug_rq_3_0.attr,
	&hotplug_rq_3_1.attr,
	&hotplug_rq_4_0.attr,
	&hotplug_rate_1_1.attr,
	&hotplug_rate_2_0.attr,
	&hotplug_rate_2_1.attr,
	&hotplug_rate_3_0.attr,
	&hotplug_rate_3_1.attr,
	&hotplug_rate_4_0.attr,
	&min_cpus_online.attr,
	&maxcoreslimit.attr,
	&maxcoreslimit_sleep.attr,
	&hotplug_suspend.attr,
	NULL
};

static struct attribute_group alucard_hotplug_attr_group = {
	.attrs = alucard_hotplug_attributes,
	.name = "alucard_hotplug",
};

static int __init alucard_hotplug_init(void)
{
	struct hotplug_cpuparm *pcpu_parm = NULL;
	unsigned int cpu;
	int ret;
	unsigned int hotplug_freq[NR_CPUS][2] = {
#ifdef CONFIG_MACH_LGE
		{0, 1497600},
		{652800, 1190400},
		{652800, 1190400},
		{652800, 0}
#else
		{0, 1242000},
		{810000, 1242000},
		{810000, 1242000},
		{810000, 0}
#endif
	};
	unsigned int hotplug_load[NR_CPUS][2] = {
		{0, 60},
		{20, 60},
		{20, 60},
		{20, 0}
	};
	unsigned int hotplug_rq[NR_CPUS][2] = {
		{0, 100},
		{100, 150},
		{150, 150},
		{150, 0}
	};
	unsigned int hotplug_rate[NR_CPUS][2] = {
		{0, 1},
		{1, 2},
		{2, 2},
		{2, 0}
	};

	ret = sysfs_create_group(kernel_kobj, &alucard_hotplug_attr_group);
	if (ret) {
		printk(KERN_ERR "failed at(%d)\n", __LINE__);
		return ret;
	}

	/* INITIALIZE PCPU VARS */
	for_each_possible_cpu(cpu) {
		pcpu_parm = &per_cpu(ac_hp_cpuparm, cpu);
		pcpu_parm->up_freq = hotplug_freq[cpu][UP_INDEX];
		pcpu_parm->down_freq = hotplug_freq[cpu][DOWN_INDEX];
		pcpu_parm->up_load = hotplug_load[cpu][UP_INDEX];
		pcpu_parm->down_load = hotplug_load[cpu][DOWN_INDEX];
		pcpu_parm->up_rq = hotplug_rq[cpu][UP_INDEX];
		pcpu_parm->down_rq = hotplug_rq[cpu][DOWN_INDEX];
		pcpu_parm->up_rate = hotplug_rate[cpu][UP_INDEX];
		pcpu_parm->down_rate = hotplug_rate[cpu][DOWN_INDEX];
	}

	if (hotplug_tuners_ins.hotplug_enable > 0) {
		hotplug_start();
	}

	return ret;
}

static void __exit alucard_hotplug_exit(void)
{
	if (hotplug_tuners_ins.hotplug_enable > 0) {
		hotplug_stop();
	}

	sysfs_remove_group(kernel_kobj, &alucard_hotplug_attr_group);
}
MODULE_AUTHOR("Alucard_24@XDA");
MODULE_DESCRIPTION("'alucard_hotplug' - A cpu hotplug driver for "
	"capable processors");
MODULE_LICENSE("GPL");

late_initcall(alucard_hotplug_init);
