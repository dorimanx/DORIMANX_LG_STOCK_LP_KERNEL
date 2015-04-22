/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Added code to work as a standalone intelligent thermal throttling driver
 * for many Qualcomm SOCs by Paul Reioux (Faux123)
 * Merged with MSM Thermal code by Yuri Sh. (Dorimanx)
 * Modifications copyright (c) 2014
 *
 */

#define pr_fmt(fmt) "%s:%s " fmt, KBUILD_MODNAME, __func__

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/msm_tsens.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/msm_tsens.h>
#include <linux/msm_thermal.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/thermal.h>
#include <mach/rpm-regulator.h>
#include <mach/rpm-regulator-smd.h>
#include <linux/regulator/consumer.h>
#include <linux/msm_thermal_ioctl.h>
#include <mach/rpm-smd.h>
#include <mach/scm.h>
#include <linux/sched.h>
#include <linux/suspend.h>

#define MAX_CURRENT_UA 1000000
#define MAX_RAILS 5
#define MAX_THRESHOLD 2
#define MONITOR_ALL_TSENS -1
#define THERM_SECURE_BITE_CMD 8

#define DEFAULT_POLLING_MS	250

#ifdef CONFIG_INTELLI_THERMAL_STATS
/* last 3 minutes based on $DEFAULT_POLLING_MS polling cycle */
#define MAX_HISTORY_SZ		((3*60*1000) / DEFAULT_POLLING_MS)

struct msm_thermal_stat_data {
	int32_t temp_history[MAX_HISTORY_SZ];
	uint32_t throttled;
	uint32_t warning;
	uint32_t normal;
};
static struct msm_thermal_stat_data msm_thermal_stats;
static uint32_t hist_index = 0;
#endif

static struct msm_thermal_data msm_thermal_info;

static struct msm_thermal_data_intelli msm_thermal_info_local = {
	.sensor_id = 0,
	.poll_ms = DEFAULT_POLLING_MS,
	.limit_temp_degC = 70,
	.temp_hysteresis_degC = 10,
	.freq_step = 2,
	.freq_control_mask = 0xf,
	.core_limit_temp_degC = 80,
	.core_temp_hysteresis_degC = 10,
	.core_control_mask = 0xe,
};

static struct delayed_work check_temp_work;

/* Always enable Intelli Thermal and core control on boot */
static int intelli_enabled = 1;
bool core_control = true;

#ifdef CONFIG_ALUCARD_TOUCHSCREEN_BOOST
int cpu_temp_for_touch_boost;
#endif

/* dummy parameter for rom thermal and apps */
static bool enabled = true;

static unsigned int debug_mode = 0;
static uint32_t cpus_offlined;
static DEFINE_MUTEX(core_control_mutex);
static struct kobject *cc_kobj;
static struct task_struct *hotplug_task;
static struct task_struct *freq_mitigation_task;
static struct task_struct *thermal_monitor_task;
static struct completion hotplug_notify_complete;
static struct completion freq_mitigation_complete;
static struct completion thermal_monitor_complete;

static int psm_rails_cnt;
static int limit_idx;

/*
 * min limit is set to 1190400 Mhz!
 * check your FREQ Table and set corect limit_idx_low freq number.
 */
static int limit_idx_low = 7;

static int limit_idx_high;
static int max_idx;
static int max_tsens_num;
static bool immediately_limit_stop = false;
static struct cpufreq_frequency_table *table;
static bool psm_enabled;
static bool psm_nodes_called;
static bool psm_probed;
static bool hotplug_enabled;
static bool freq_mitigation_enabled;
static bool interrupt_mode_enable;
static bool msm_thermal_probed;
static bool therm_reset_enabled;
static bool online_core;
static int *tsens_id_map;
static DEFINE_MUTEX(psm_mutex);
static uint32_t min_freq_limit;

enum thermal_threshold {
	HOTPLUG_THRESHOLD_HIGH,
	HOTPLUG_THRESHOLD_LOW,
	FREQ_THRESHOLD_HIGH,
	FREQ_THRESHOLD_LOW,
	THRESHOLD_MAX_NR,
};

enum sensor_id_type {
	THERM_ZONE_ID,
	THERM_TSENS_ID,
	THERM_ID_MAX_NR,
};

struct cpu_info {
	uint32_t cpu;
	const char *sensor_type;
	enum sensor_id_type id_type;
	uint32_t sensor_id;
	bool offline;
	bool user_offline;
	bool hotplug_thresh_clear;
	struct sensor_threshold threshold[THRESHOLD_MAX_NR];
	bool max_freq;
	uint32_t user_max_freq;
	uint32_t user_min_freq;
	uint32_t limited_max_freq;
	uint32_t limited_min_freq;
	bool freq_thresh_clear;
};

/* module parameters */
module_param_named(poll_ms, msm_thermal_info_local.poll_ms, uint, 0664);
module_param_named(limit_temp_degC, msm_thermal_info_local.limit_temp_degC,
			int, 0664);
module_param_named(temp_hysteresis_degC,
			msm_thermal_info_local.temp_hysteresis_degC,
			int, 0664);
module_param_named(freq_step, msm_thermal_info_local.freq_step,
			int, 0664);
module_param_named(immediately_limit_stop, immediately_limit_stop,
			bool, 0664);
module_param_named(core_limit_temp_degC,
			msm_thermal_info_local.core_limit_temp_degC,
			int, 0664);
module_param_named(core_temp_hysteresis_degC,
			msm_thermal_info_local.core_temp_hysteresis_degC,
			int, 0664);
module_param_named(freq_control_mask, msm_thermal_info_local.freq_control_mask,
			uint, 0664);
module_param_named(core_control_mask, msm_thermal_info_local.core_control_mask,
			uint, 0664);

module_param_named(thermal_limit_high, limit_idx_high, int, 0664);
module_param_named(thermal_limit_low, limit_idx_low, int, 0664);

module_param_named(thermal_debug_mode, debug_mode, int, 0664);

static unsigned int safety = 1;
module_param_named(temp_safety, safety, int, 0664);

static unsigned int freq_debug = 0;
module_param_named(freq_limit_debug, freq_debug, uint, 0644);

#define dprintk(msg...)		\
do {				\
	if (freq_debug)		\
		pr_info(msg);	\
} while (0)

struct threshold_info;
struct therm_threshold {
	int32_t sensor_id;
	struct sensor_threshold threshold[MAX_THRESHOLD];
	int32_t trip_triggered;
	void (*notify)(struct therm_threshold *);
	struct threshold_info *parent;
};

struct threshold_info {
	uint32_t thresh_ct;
	bool thresh_triggered;
	struct therm_threshold *thresh_list;
};

struct rail {
	const char *name;
	uint32_t freq_req;
	uint32_t min_level;
	uint32_t num_levels;
	int32_t curr_level;
	uint32_t levels[3];
	struct kobj_attribute value_attr;
	struct kobj_attribute level_attr;
	struct regulator *reg;
	struct attribute_group attr_gp;
};

struct psm_rail {
	const char *name;
	uint8_t init;
	uint8_t mode;
	struct kobj_attribute mode_attr;
	struct rpm_regulator *reg;
	struct regulator *phase_reg;
	struct attribute_group attr_gp;
};

enum msm_thresh_list {
	MSM_THERM_RESET,
	MSM_LIST_MAX_NR,
};

static struct psm_rail *psm_rails;
static struct cpu_info cpus[NR_CPUS];
static struct threshold_info *thresh;

/* For SMPS only*/
enum PMIC_SW_MODE {
	PMIC_AUTO_MODE  = RPM_REGULATOR_MODE_AUTO,
	PMIC_IPEAK_MODE = RPM_REGULATOR_MODE_IPEAK,
	PMIC_PWM_MODE   = RPM_REGULATOR_MODE_HPM,
};

#define PSM_RW_ATTRIB(_rail, ko_attr, j, _name) \
	ko_attr.attr.name = __stringify(_name); \
	ko_attr.attr.mode = 0644; \
	ko_attr.show = psm_reg_##_name##_show; \
	ko_attr.store = psm_reg_##_name##_store; \
	sysfs_attr_init(&ko_attr.attr); \
	_rail.attr_gp.attrs[j] = &ko_attr.attr;

#define PSM_REG_MODE_FROM_ATTRIBS(attr) \
	(container_of(attr, struct psm_rail, mode_attr));

static int msm_thermal_cpufreq_callback(struct notifier_block *nfb,
		unsigned long event, void *data)
{
	struct cpufreq_policy *policy = data;
	uint32_t max_freq_req = cpus[policy->cpu].limited_max_freq;
	uint32_t min_freq_req = cpus[policy->cpu].limited_min_freq;

	switch (event) {
	case CPUFREQ_INCOMPATIBLE:
		pr_debug("mitigating CPU%d to freq max: %u min: %u\n",
		policy->cpu, max_freq_req, min_freq_req);

		cpufreq_verify_within_limits(policy, min_freq_req,
				max_freq_req);

		if (max_freq_req < min_freq_req)
			pr_err("Invalid frequency request Max:%u Min:%u\n",
					max_freq_req, min_freq_req);
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block msm_thermal_cpufreq_notifier = {
	.notifier_call = msm_thermal_cpufreq_callback,
};

static void update_cpu_freq(int cpu)
{
	int ret = 0;

	if (cpu_online(cpu)) {
		ret = cpufreq_update_policy(cpu);
		if (ret)
			pr_err("Unable to update policy for cpu:%d. err:%d\n",
				cpu, ret);
	}
}

/* Setting all rails the same mode */
static int psm_set_mode_all(int mode)
{
	int i = 0;
	int fail_cnt = 0;
	int ret = 0;

	pr_debug("Requesting PMIC Mode: %d\n", mode);
	for (i = 0; i < psm_rails_cnt; i++) {
		if (psm_rails[i].mode != mode) {
			ret = rpm_regulator_set_mode(psm_rails[i].reg, mode);
			if (ret) {
				pr_err("Cannot set mode:%d for %s. err:%d",
					mode, psm_rails[i].name, ret);
				fail_cnt++;
			} else
				psm_rails[i].mode = mode;
		}
	}

	return fail_cnt ? (-EFAULT) : ret;
}

static int psm_reg_mode_show(
	struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct psm_rail *reg = PSM_REG_MODE_FROM_ATTRIBS(attr);
	return snprintf(buf, PAGE_SIZE, "%d\n", reg->mode);
}

static ssize_t psm_reg_mode_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	int val = 0;
	struct psm_rail *reg = PSM_REG_MODE_FROM_ATTRIBS(attr);

	mutex_lock(&psm_mutex);
	ret = kstrtoint(buf, 10, &val);
	if (ret) {
		pr_err("Invalid input %s for mode\n", buf);
		goto done_psm_store;
	}

	if ((val != PMIC_PWM_MODE) && (val != PMIC_AUTO_MODE)) {
		pr_err("Invalid number %d for mode\n", val);
		goto done_psm_store;
	}

	if (val != reg->mode) {
		ret = rpm_regulator_set_mode(reg->reg, val);
		if (ret) {
			pr_err("Fail to set Mode:%d for %s. err:%d\n",
			val, reg->name, ret);
			goto done_psm_store;
		}
		reg->mode = val;
	}

done_psm_store:
	mutex_unlock(&psm_mutex);
	return count;
}

static int check_sensor_id(int sensor_id)
{
	int i = 0;
	bool hw_id_found = false;
	int ret = 0;

	for (i = 0; i < max_tsens_num; i++) {
		if (sensor_id == tsens_id_map[i]) {
			hw_id_found = true;
			break;
		}
	}
	if (!hw_id_found) {
		pr_err("Invalid sensor hw id:%d\n", sensor_id);
		return -EINVAL;
	}

	return ret;
}

static int create_sensor_id_map(void)
{
	int i = 0;
	int ret = 0;

	tsens_id_map = kzalloc(sizeof(int) * max_tsens_num,
			GFP_KERNEL);
	if (!tsens_id_map) {
		pr_err("Cannot allocate memory for tsens_id_map\n");
		return -ENOMEM;
	}

	for (i = 0; i < max_tsens_num; i++) {
		ret = tsens_get_hw_id_mapping(i, &tsens_id_map[i]);
		/* If return -ENXIO, hw_id is default in sequence */
		if (ret) {
			if (ret == -ENXIO) {
				tsens_id_map[i] = i;
				ret = 0;
			} else {
				pr_err("Failed to get hw id for id:%d.err:%d\n",
						i, ret);
				goto fail;
			}
		}
	}

	return ret;
fail:
	kfree(tsens_id_map);
	return ret;
}

static int msm_thermal_get_freq_table(void)
{
	int ret = 0;
	int i = 0;

	table = cpufreq_frequency_get_table(0);
	if (table == NULL) {
		pr_err("error reading cpufreq table\n");
		ret = -EINVAL;
		goto fail;
	}

	while (table[i].frequency != CPUFREQ_TABLE_END)
		i++;

	limit_idx_high = limit_idx = max_idx = i - 1;
	BUG_ON(limit_idx_high <= 0 || limit_idx_high <= limit_idx_low);
fail:
	return ret;
}

static int set_and_activate_threshold(uint32_t sensor_id,
	struct sensor_threshold *threshold)
{
	int ret = 0;

	ret = sensor_set_trip(sensor_id, threshold);
	if (ret != 0) {
		pr_err("sensor:%u Error in setting trip:%d. err:%d\n",
			sensor_id, threshold->trip, ret);
		goto set_done;
	}

	ret = sensor_activate_trip(sensor_id, threshold, true);
	if (ret != 0) {
		pr_err("sensor:%u Error in enabling trip:%d. err:%d\n",
			sensor_id, threshold->trip, ret);
		goto set_done;
	}

set_done:
	return ret;
}

static int therm_get_temp(uint32_t id, enum sensor_id_type type, long *temp)
{
	int ret = 0;
	struct tsens_device tsens_dev;

	if (!temp) {
		pr_err("Invalid value\n");
		ret = -EINVAL;
		goto get_temp_exit;
	}

	switch (type) {
	case THERM_ZONE_ID:
		tsens_dev.sensor_num = tsens_id_map[id];
		break;
	case THERM_TSENS_ID:
		tsens_dev.sensor_num = id;
		break;
	default:
		pr_err("Invalid type\n");
		ret = -EINVAL;
		goto get_temp_exit;
		break;
	}

	ret = tsens_get_temp(&tsens_dev, temp);
	if (ret) {
		pr_err("Unable to read TSENS sensor:%d\n",
			tsens_dev.sensor_num);
		goto get_temp_exit;
	}

get_temp_exit:
	return ret;
}

static int set_threshold(uint32_t zone_id,
	struct sensor_threshold *threshold)
{
	int i = 0, ret = 0;
	long temp;

	if ((!threshold) || (zone_id >= max_tsens_num)) {
		pr_err("Invalid input\n");
		ret = -EINVAL;
		goto set_threshold_exit;
	}

	ret = therm_get_temp(zone_id, THERM_ZONE_ID, &temp);
	if (ret) {
		pr_err("Unable to read temperature for zone:%d. err:%d\n",
			zone_id, ret);
		goto set_threshold_exit;
	}

	while (i < MAX_THRESHOLD) {
		switch (threshold[i].trip) {
		case THERMAL_TRIP_CONFIGURABLE_HI:
			if (threshold[i].temp >= temp) {
				ret = set_and_activate_threshold(zone_id,
					&threshold[i]);
				if (ret)
					goto set_threshold_exit;
			}
			break;
		case THERMAL_TRIP_CONFIGURABLE_LOW:
			if (threshold[i].temp <= temp) {
				ret = set_and_activate_threshold(zone_id,
					&threshold[i]);
				if (ret)
					goto set_threshold_exit;
			}
			break;
		default:
			pr_err("zone:%u Invalid trip:%d\n", zone_id,
					threshold[i].trip);
			break;
		}
		i++;
	}
set_threshold_exit:
	return ret;
}

static void msm_thermal_bite(int tsens_id, long temp)
{
	pr_err("TSENS:%d reached temperature:%ld. System reset\n",
		tsens_id, temp);
	scm_call_atomic1(SCM_SVC_BOOT, THERM_SECURE_BITE_CMD, 0);
}

static int do_therm_reset(void)
{
	int ret = 0, i;
	long temp = 0;

	if (!therm_reset_enabled)
		return ret;

	for (i = 0; i < thresh[MSM_THERM_RESET].thresh_ct; i++) {
		ret = therm_get_temp(
			thresh[MSM_THERM_RESET].thresh_list[i].sensor_id,
			THERM_TSENS_ID,
			&temp);
		if (ret) {
			pr_err("Unable to read TSENS sensor:%d. err:%d\n",
			thresh[MSM_THERM_RESET].thresh_list[i].sensor_id,
			ret);
			continue;
		}

		if (temp >= msm_thermal_info.therm_reset_temp_degC)
			msm_thermal_bite(
			thresh[MSM_THERM_RESET].thresh_list[i].sensor_id, temp);
	}

	return ret;
}

static void therm_reset_notify(struct therm_threshold *thresh_data)
{
	long temp;
	int ret = 0;

	if (!therm_reset_enabled)
		return;

	if (!thresh_data) {
		pr_err("Invalid input\n");
		return;
	}

	switch (thresh_data->trip_triggered) {
	case THERMAL_TRIP_CONFIGURABLE_HI:
		ret = therm_get_temp(thresh_data->sensor_id,
				THERM_TSENS_ID, &temp);
		if (ret)
			pr_err("Unable to read TSENS sensor:%d. err:%d\n",
				thresh_data->sensor_id, ret);
		msm_thermal_bite(tsens_id_map[thresh_data->sensor_id],
					temp);
		break;
	case THERMAL_TRIP_CONFIGURABLE_LOW:
		break;
	default:
		pr_err("Invalid trip type\n");
		break;
	}
	set_threshold(thresh_data->sensor_id, thresh_data->threshold);
}

#ifdef CONFIG_SMP
static void __ref do_core_control(long temp)
{
	int i = 0;
	int ret = 0;

	if (!core_control)
		return;

	/**
	 *  Offline cores starting from the max MPIDR to 1, when above limit,
	 *  The core control mask is non zero and allows the core to be turned
	 *  off.
	 *  The core was not previously offlined by this module
	 *  The core is the next in sequence.
	 *  If the core was online for some reason, even after it was offlined
	 *  by this module, offline it again.
	 *  Online the back on if the temp is below the hysteresis and was
	 *  offlined by this module and not already online.
	 */
	mutex_lock(&core_control_mutex);
	if (msm_thermal_info_local.core_control_mask &&
		temp >= msm_thermal_info_local.core_limit_temp_degC) {
		for (i = num_possible_cpus(); i > 0; i--) {
			if (!(msm_thermal_info_local.core_control_mask &
					BIT(i)))
				continue;
			if (cpus_offlined & BIT(i) && !cpu_online(i))
				continue;
			dprintk("%s: Set Offline: CPU%d Temp: %ld\n",
					KBUILD_MODNAME, i, temp);
			ret = cpu_down(i);
			if (ret)
				pr_err("%s: Error %d offline core %d\n",
					KBUILD_MODNAME, ret, i);
			cpus_offlined |= BIT(i);
			break;
		}
	} else if (msm_thermal_info_local.core_control_mask && cpus_offlined &&
		temp <= (msm_thermal_info_local.core_limit_temp_degC -
			msm_thermal_info_local.core_temp_hysteresis_degC)) {
		for (i = 0; i < num_possible_cpus(); i++) {
			if (!(cpus_offlined & BIT(i)))
				continue;
			cpus_offlined &= ~BIT(i);
			dprintk("%s: Allow Online CPU%d Temp: %ld\n",
					KBUILD_MODNAME, i, temp);
			/*
			 * If this core is already online, then bring up the
			 * next offlined core.
			 */
			if (cpu_online(i))
				continue;
			ret = cpu_up(i);
			if (ret)
				pr_err("Error %d online core %d\n",
						ret, i);
			break;
		}
	}
	mutex_unlock(&core_control_mutex);
}

static int msm_thermal_suspend_callback(
	struct notifier_block *nfb, unsigned long action, void *data)
{
	switch (action) {
	case PM_POST_HIBERNATION:
	case PM_POST_SUSPEND:
		if (hotplug_task)
			complete(&hotplug_notify_complete);
		else
			pr_debug("Hotplug task not initialized\n");
		break;

	default:
		return NOTIFY_DONE;
	}

	return NOTIFY_OK;
}

/* Call with core_control_mutex locked */
static int __ref update_offline_cores(int val)
{
	uint32_t cpu = 0;
	int ret = 0;
	uint32_t previous_cpus_offlined = 0;

	if (!core_control)
		return 0;

	previous_cpus_offlined = cpus_offlined;
	cpus_offlined = msm_thermal_info_local.core_control_mask & val;

	for_each_possible_cpu(cpu) {
		if (cpus_offlined & BIT(cpu)) {
			if (!cpu_online(cpu))
				continue;
			ret = cpu_down(cpu);
			if (ret)
				pr_err("Unable to offline CPU%d. err:%d\n",
					cpu, ret);
			else
				pr_debug("Offlined CPU%d\n", cpu);
		} else if (online_core && (previous_cpus_offlined & BIT(cpu))) {
			if (cpu_online(cpu))
				continue;
			ret = cpu_up(cpu);
			if (ret && ret == notifier_to_errno(NOTIFY_BAD)) {
				pr_debug("Onlining CPU%d is vetoed\n", cpu);
			} else if (ret) {
				cpus_offlined |= BIT(cpu);
				pr_err("Unable to online CPU%d. err:%d\n",
					cpu, ret);
			} else {
				pr_debug("Onlined CPU%d\n", cpu);
			}
		}
	}
	return ret;
}

static __ref int do_hotplug(void *data)
{
	int ret = 0;
	uint32_t cpu = 0, mask = 0;
	struct sched_param param = {.sched_priority = MAX_RT_PRIO-2};

	if (!core_control) {
		pr_debug("Core control disabled\n");
		return 0;
	}

	sched_setscheduler(current, SCHED_FIFO, &param);
	while (!kthread_should_stop()) {
		while (wait_for_completion_interruptible(
			&hotplug_notify_complete) != 0)
			;
		INIT_COMPLETION(hotplug_notify_complete);
		mask = 0;

		mutex_lock(&core_control_mutex);
		for_each_possible_cpu(cpu) {
			if (hotplug_enabled &&
				cpus[cpu].hotplug_thresh_clear) {
				set_threshold(cpus[cpu].sensor_id,
				&cpus[cpu].threshold[HOTPLUG_THRESHOLD_HIGH]);

				cpus[cpu].hotplug_thresh_clear = false;
			}
			if (cpus[cpu].offline || cpus[cpu].user_offline)
				mask |= BIT(cpu);
		}
		update_offline_cores(mask);
		mutex_unlock(&core_control_mutex);
		sysfs_notify(cc_kobj, NULL, "cpus_offlined");
	}

	return ret;
}
#else
static void do_core_control(long temp)
{
	return;
}

static __ref int do_hotplug(void *data)
{
	return 0;
}
#endif

static int do_psm(void)
{
	long temp = 0;
	int ret = 0;
	int i = 0;
	int auto_cnt = 0;

	mutex_lock(&psm_mutex);
	for (i = 0; i < max_tsens_num; i++) {
		ret = therm_get_temp(tsens_id_map[i], THERM_TSENS_ID, &temp);
		if (ret) {
			pr_err("Unable to read TSENS sensor:%d. err:%d\n",
					tsens_id_map[i], ret);
			auto_cnt++;
			continue;
		}

		/*
		 * As long as one sensor is above the threshold, set PWM mode
		 * on all rails, and loop stops. Set auto mode when all rails
		 * are below thershold
		 */
		if (temp >  msm_thermal_info.psm_temp_degC) {
			ret = psm_set_mode_all(PMIC_PWM_MODE);
			if (ret) {
				pr_err("Set pwm mode for all failed. err:%d\n",
						ret);
				goto exit;
			}
			pr_debug("Requested PMIC PWM Mode tsens:%d. Temp:%ld\n",
					tsens_id_map[i], temp);
			break;
		} else if (temp <= msm_thermal_info.psm_temp_hyst_degC)
			auto_cnt++;
	}

	if (auto_cnt == max_tsens_num) {
		ret = psm_set_mode_all(PMIC_AUTO_MODE);
		if (ret) {
			pr_err("Set auto mode for all failed. err:%d\n", ret);
			goto exit;
		}
		pr_debug("Requested PMIC AUTO Mode\n");
	}

exit:
	mutex_unlock(&psm_mutex);
	return ret;
}

static void __ref do_freq_control(long temp)
{
	uint32_t cpu = 0;
	uint32_t max_freq = cpus[cpu].limited_max_freq;

	if (safety == 0) {
		if (msm_thermal_info_local.limit_temp_degC > 83)
			msm_thermal_info_local.limit_temp_degC = 83;
	} else {
		if (msm_thermal_info_local.limit_temp_degC > 78)
			msm_thermal_info_local.limit_temp_degC = 78;
	}

	if (debug_mode == 1)
		printk(KERN_INFO "pre-check do_freq_control temp[%ld], \
				limit_idx[%d], limit_idx_low[%d], \
				limited_idx_high[%d]\n",
				temp, limit_idx, limit_idx_low,
				limit_idx_high);

	if (temp >= msm_thermal_info_local.limit_temp_degC) {
		if (limit_idx == limit_idx_low)
			return;

		limit_idx -= msm_thermal_info_local.freq_step;
		if (limit_idx < limit_idx_low)
			limit_idx = limit_idx_low;
		/* Consider saved policy->max freq */
		max_freq = table[min(max_idx -
					(int)msm_thermal_info_local.freq_step,
					limit_idx)].frequency;
	} else if (temp < msm_thermal_info_local.limit_temp_degC -
			msm_thermal_info_local.temp_hysteresis_degC) {
		if (limit_idx == limit_idx_high)
			return;

		limit_idx += msm_thermal_info_local.freq_step;
		limit_idx_high = min(max_idx, limit_idx_high);
		if ((limit_idx >= limit_idx_high) ||
				immediately_limit_stop == true) {
			limit_idx = limit_idx_high;
			max_freq = UINT_MAX;
		} else
			max_freq = table[limit_idx].frequency;
	}

	if (debug_mode == 1)
		printk(KERN_INFO "do_freq_control temp[%ld], \
				limit_idx[%d], max_freq[%d], \
				limited_max_freq[%d]\n",
				temp, limit_idx, max_freq,
				cpus[cpu].limited_max_freq);

	if (max_freq == cpus[cpu].limited_max_freq)
		return;

	/* Update new limits */
	for_each_possible_cpu(cpu) {
		if (!(msm_thermal_info_local.freq_control_mask & BIT(cpu)))
			continue;
		cpus[cpu].limited_max_freq = max_freq;
		if (cpu_online(cpu)) {
			if (cpufreq_update_policy(cpu))
				pr_info("Unable to update policy for cpu:%d\n", cpu);
		}
	}
}

static void __ref check_temp(struct work_struct *work)
{
	static int limit_init;
	long temp = 0;
	int ret = 0;

	if (!msm_thermal_probed)
		return;

	do_therm_reset();

	ret = therm_get_temp(msm_thermal_info_local.sensor_id, THERM_TSENS_ID, &temp);
	if (ret) {
		pr_err("Unable to read TSENS sensor:%d. err:%d\n",
				msm_thermal_info_local.sensor_id, ret);
		goto reschedule;
	}

#ifdef CONFIG_INTELLI_THERMAL_STATS
	if (hist_index < MAX_HISTORY_SZ)
		msm_thermal_stats.temp_history[hist_index] = temp;
	else {
		hist_index = 0;
		msm_thermal_stats.temp_history[hist_index] = temp;
	}
	hist_index++;
#endif

	if (!limit_init) {
		ret = msm_thermal_get_freq_table();
		if (ret)
			goto reschedule;
		else
			limit_init = 1;
	}

#ifdef CONFIG_ALUCARD_TOUCHSCREEN_BOOST
	cpu_temp_for_touch_boost = temp;
#endif

	do_core_control(temp);
	do_psm();
	do_freq_control(temp);

reschedule:
	if (intelli_enabled)
		schedule_delayed_work(&check_temp_work, msecs_to_jiffies(
					msm_thermal_info_local.poll_ms));
}

static int __ref msm_thermal_cpu_callback(struct notifier_block *nfb,
		unsigned long action, void *hcpu)
{
	uint32_t cpu = (uint32_t)hcpu;

	if (action == CPU_UP_PREPARE || action == CPU_UP_PREPARE_FROZEN) {
		if (core_control && (
				msm_thermal_info_local.core_control_mask &
				BIT(cpu)) && (cpus_offlined & BIT(cpu))) {
			if (debug_mode == 1)
				pr_info("%s: Preventing cpu%d from coming \
						online.\n", KBUILD_MODNAME,
						cpu);
			return NOTIFY_BAD;
		}
	}

	if (debug_mode == 1)
		pr_debug("voting for CPU%d to be online\n", cpu);
	return NOTIFY_OK;
}

static struct notifier_block __refdata msm_thermal_cpu_notifier = {
	.notifier_call = msm_thermal_cpu_callback,
};

static int hotplug_notify(enum thermal_trip_type type, int temp, void *data)
{
	struct cpu_info *cpu_node = (struct cpu_info *)data;

	pr_info("%s reach temp threshold: %d\n", cpu_node->sensor_type, temp);

	if (!(msm_thermal_info_local.core_control_mask & BIT(cpu_node->cpu)))
		return 0;
	switch (type) {
	case THERMAL_TRIP_CONFIGURABLE_HI:
		if (!(cpu_node->offline))
			cpu_node->offline = 1;
		break;
	case THERMAL_TRIP_CONFIGURABLE_LOW:
		if (cpu_node->offline)
			cpu_node->offline = 0;
		break;
	default:
		break;
	}
	if (hotplug_task) {
		cpu_node->hotplug_thresh_clear = true;
		complete(&hotplug_notify_complete);
	} else {
		pr_err("Hotplug task is not initialized\n");
	}
	return 0;
}
/* Adjust cpus offlined bit based on temperature reading. */
static int hotplug_init_cpu_offlined(void)
{
	long temp = 0;
	uint32_t cpu = 0;

	if (!hotplug_enabled)
		return 0;

	mutex_lock(&core_control_mutex);
	for_each_possible_cpu(cpu) {
		if (!(msm_thermal_info_local.core_control_mask & BIT(cpus[cpu].cpu)))
			continue;
		if (therm_get_temp(cpus[cpu].sensor_id, cpus[cpu].id_type,
					&temp)) {
			pr_err("Unable to read TSENS sensor:%d.\n",
				cpus[cpu].sensor_id);
			mutex_unlock(&core_control_mutex);
			return -EINVAL;
		}

		if (temp >= msm_thermal_info.hotplug_temp_degC)
			cpus[cpu].offline = 1;
		else if (temp <= (msm_thermal_info.hotplug_temp_degC -
			msm_thermal_info.hotplug_temp_hysteresis_degC))
			cpus[cpu].offline = 0;
	}
	mutex_unlock(&core_control_mutex);

	if (hotplug_task)
		complete(&hotplug_notify_complete);
	else {
		pr_err("Hotplug task is not initialized\n");
		return -EINVAL;
	}
	return 0;
}

static void hotplug_init(void)
{
	uint32_t cpu = 0;
	struct sensor_threshold *hi_thresh = NULL, *low_thresh = NULL;

	if (hotplug_task)
		return;

	if (!hotplug_enabled)
		goto init_kthread;

	for_each_possible_cpu(cpu) {
		cpus[cpu].sensor_id =
			sensor_get_id((char *)cpus[cpu].sensor_type);
		cpus[cpu].id_type = THERM_ZONE_ID;
		if (!(msm_thermal_info_local.core_control_mask & BIT(cpus[cpu].cpu)))
			continue;

		hi_thresh = &cpus[cpu].threshold[HOTPLUG_THRESHOLD_HIGH];
		low_thresh = &cpus[cpu].threshold[HOTPLUG_THRESHOLD_LOW];
		hi_thresh->temp = msm_thermal_info.hotplug_temp_degC;
		hi_thresh->trip = THERMAL_TRIP_CONFIGURABLE_HI;
		low_thresh->temp = msm_thermal_info.hotplug_temp_degC -
				msm_thermal_info.hotplug_temp_hysteresis_degC;
		low_thresh->trip = THERMAL_TRIP_CONFIGURABLE_LOW;
		hi_thresh->notify = low_thresh->notify = hotplug_notify;
		hi_thresh->data = low_thresh->data = (void *)&cpus[cpu];

		set_threshold(cpus[cpu].sensor_id, hi_thresh);
	}
init_kthread:
	init_completion(&hotplug_notify_complete);
	hotplug_task = kthread_run(do_hotplug, NULL, "msm_thermal:hotplug");
	if (IS_ERR(hotplug_task)) {
		pr_err("Failed to create do_hotplug thread. err:%ld\n",
				PTR_ERR(hotplug_task));
		return;
	}
	/*
	 * Adjust cpus offlined bit when hotplug intitializes so that the new
	 * cpus offlined state is based on hotplug threshold range
	 */
	if (hotplug_init_cpu_offlined())
		kthread_stop(hotplug_task);
}

static __ref int do_freq_mitigation(void *data)
{
	int ret = 0;
	uint32_t cpu = 0, max_freq_req = 0, min_freq_req = 0;
	struct sched_param param = {.sched_priority = MAX_RT_PRIO-1};

	sched_setscheduler(current, SCHED_FIFO, &param);
	while (!kthread_should_stop()) {
		while (wait_for_completion_interruptible(
			&freq_mitigation_complete) != 0)
			;
		INIT_COMPLETION(freq_mitigation_complete);

		for_each_possible_cpu(cpu) {
			max_freq_req = (cpus[cpu].max_freq) ?
					msm_thermal_info.freq_limit :
					UINT_MAX;
			max_freq_req = min(max_freq_req,
					cpus[cpu].user_max_freq);

			min_freq_req = max(min_freq_limit,
					cpus[cpu].user_min_freq);

			if ((max_freq_req == cpus[cpu].limited_max_freq)
				&& (min_freq_req ==
				cpus[cpu].limited_min_freq))
				goto reset_threshold;

			cpus[cpu].limited_max_freq = max_freq_req;
			cpus[cpu].limited_min_freq = min_freq_req;
			update_cpu_freq(cpu);
reset_threshold:
			if (freq_mitigation_enabled &&
				cpus[cpu].freq_thresh_clear) {
				set_threshold(cpus[cpu].sensor_id,
				&cpus[cpu].threshold[FREQ_THRESHOLD_HIGH]);

				cpus[cpu].freq_thresh_clear = false;
			}
		}
	}
	return ret;
}

static int freq_mitigation_notify(enum thermal_trip_type type,
	int temp, void *data)
{
	struct cpu_info *cpu_node = (struct cpu_info *) data;

	pr_debug("%s reached temp threshold: %d\n",
		cpu_node->sensor_type, temp);

	if (!(msm_thermal_info.freq_mitig_control_mask &
		BIT(cpu_node->cpu)))
		return 0;

	switch (type) {
	case THERMAL_TRIP_CONFIGURABLE_HI:
		if (!cpu_node->max_freq) {
			pr_info("Mitigating CPU%d frequency to %d\n",
				cpu_node->cpu,
				msm_thermal_info.freq_limit);

			cpu_node->max_freq = true;
		}
		break;
	case THERMAL_TRIP_CONFIGURABLE_LOW:
		if (cpu_node->max_freq) {
			pr_info("Removing frequency mitigation for CPU%d\n",
				cpu_node->cpu);

			cpu_node->max_freq = false;
		}
		break;
	default:
		break;
	}

	if (freq_mitigation_task) {
		cpu_node->freq_thresh_clear = true;
		complete(&freq_mitigation_complete);
	} else {
		pr_err("Frequency mitigation task is not initialized\n");
	}

	return 0;
}

static void freq_mitigation_init(void)
{
	uint32_t cpu = 0;
	struct sensor_threshold *hi_thresh = NULL, *low_thresh = NULL;

	if (freq_mitigation_task)
		return;
	if (!freq_mitigation_enabled)
		goto init_freq_thread;

	for_each_possible_cpu(cpu) {
		if (!(msm_thermal_info.freq_mitig_control_mask & BIT(cpu)))
			continue;
		hi_thresh = &cpus[cpu].threshold[FREQ_THRESHOLD_HIGH];
		low_thresh = &cpus[cpu].threshold[FREQ_THRESHOLD_LOW];

		hi_thresh->temp = msm_thermal_info.freq_mitig_temp_degc;
		hi_thresh->trip = THERMAL_TRIP_CONFIGURABLE_HI;
		low_thresh->temp = msm_thermal_info.freq_mitig_temp_degc -
			msm_thermal_info.freq_mitig_temp_hysteresis_degc;
		low_thresh->trip = THERMAL_TRIP_CONFIGURABLE_LOW;
		hi_thresh->notify = low_thresh->notify =
			freq_mitigation_notify;
		hi_thresh->data = low_thresh->data = (void *)&cpus[cpu];

		set_threshold(cpus[cpu].sensor_id, hi_thresh);
	}
init_freq_thread:
	init_completion(&freq_mitigation_complete);
	freq_mitigation_task = kthread_run(do_freq_mitigation, NULL,
		"msm_thermal:freq_mitig");

	if (IS_ERR(freq_mitigation_task)) {
		pr_err("Failed to create frequency mitigation thread. err:%ld\n",
				PTR_ERR(freq_mitigation_task));
		return;
	}
}

int msm_thermal_set_frequency(uint32_t cpu, uint32_t freq, bool is_max)
{
	int ret = 0;

	if (cpu >= num_possible_cpus()) {
		pr_err("Invalid input\n");
		ret = -EINVAL;
		goto set_freq_exit;
	}

	pr_debug("Userspace requested %s frequency %u for CPU%u\n",
			(is_max) ? "Max" : "Min", freq, cpu);
	if (is_max) {
		if (cpus[cpu].user_max_freq == freq)
			goto set_freq_exit;

		cpus[cpu].user_max_freq = freq;
	} else {
		if (cpus[cpu].user_min_freq == freq)
			goto set_freq_exit;

		cpus[cpu].user_min_freq = freq;
	}

	if (freq_mitigation_task) {
		complete(&freq_mitigation_complete);
	} else {
		pr_err("Frequency mitigation task is not initialized\n");
		ret = -ESRCH;
		goto set_freq_exit;
	}

set_freq_exit:
	return ret;
}

int therm_set_threshold(struct threshold_info *thresh_inp)
{
	int ret = 0, i = 0, err = 0;
	struct therm_threshold *thresh_ptr;

	if (!thresh_inp) {
		pr_err("Invalid input\n");
		ret = -EINVAL;
		goto therm_set_exit;
	}

	thresh_inp->thresh_triggered = false;
	for (i = 0; i < thresh_inp->thresh_ct; i++) {
		thresh_ptr = &thresh_inp->thresh_list[i];
		thresh_ptr->trip_triggered = -1;
		err = set_threshold(thresh_ptr->sensor_id,
			thresh_ptr->threshold);
		if (err) {
			ret = err;
			err = 0;
		}
	}

therm_set_exit:
	return ret;
}

static __ref int do_thermal_monitor(void *data)
{
	int ret = 0, i, j;
	struct therm_threshold *sensor_list;

	while (!kthread_should_stop()) {
		while (wait_for_completion_interruptible(
			&thermal_monitor_complete) != 0)
			;
		INIT_COMPLETION(thermal_monitor_complete);

		for (i = 0; i < MSM_LIST_MAX_NR; i++) {
			if (!thresh[i].thresh_triggered)
				continue;
			thresh[i].thresh_triggered = false;
			for (j = 0; j < thresh[i].thresh_ct; j++) {
				sensor_list = &thresh[i].thresh_list[j];
				if (sensor_list->trip_triggered < 0)
					continue;
				sensor_list->notify(sensor_list);
				sensor_list->trip_triggered = -1;
			}
		}
	}
	return ret;
}

static void thermal_monitor_init(void)
{
	if (thermal_monitor_task)
		return;

	init_completion(&thermal_monitor_complete);
	thermal_monitor_task = kthread_run(do_thermal_monitor, NULL,
		"msm_thermal:therm_monitor");
	if (IS_ERR(thermal_monitor_task)) {
		pr_err("Failed to create thermal monitor thread. err:%ld\n",
				PTR_ERR(thermal_monitor_task));
		goto init_exit;
	}

	if (therm_reset_enabled)
		therm_set_threshold(&thresh[MSM_THERM_RESET]);

init_exit:
	return;
}

static int msm_thermal_notify(enum thermal_trip_type type, int temp, void *data)
{
	struct therm_threshold *thresh_data = (struct therm_threshold *)data;

	if (thermal_monitor_task) {
		thresh_data->trip_triggered = type;
		thresh_data->parent->thresh_triggered = true;
		complete(&thermal_monitor_complete);
	} else {
		pr_err("Thermal monitor task is not initialized\n");
	}
	return 0;
}

static int init_threshold(enum msm_thresh_list index,
	int sensor_id, int32_t hi_temp, int32_t low_temp,
	void (*callback)(struct therm_threshold *))
{
	int ret = 0, i;
	struct therm_threshold *thresh_ptr;

	if (!callback || index >= MSM_LIST_MAX_NR || index < 0
		|| sensor_id == -ENODEV) {
		pr_err("Invalid input. sensor:%d. index:%d\n",
				sensor_id, index);
		ret = -EINVAL;
		goto init_thresh_exit;
	}
	if (thresh[index].thresh_list) {
		pr_err("threshold id:%d already initialized\n", index);
		ret = -EEXIST;
		goto init_thresh_exit;
	}

	thresh[index].thresh_ct = (sensor_id == MONITOR_ALL_TSENS) ?
						max_tsens_num : 1;
	thresh[index].thresh_triggered = false;
	thresh[index].thresh_list = kzalloc(sizeof(struct therm_threshold) *
					thresh[index].thresh_ct, GFP_KERNEL);
	if (!thresh[index].thresh_list) {
		pr_err("kzalloc failed for thresh index:%d\n", index);
		ret = -ENOMEM;
		goto init_thresh_exit;
	}

	thresh_ptr = thresh[index].thresh_list;
	if (sensor_id == MONITOR_ALL_TSENS) {
		for (i = 0; i < max_tsens_num; i++) {
			thresh_ptr[i].sensor_id = tsens_id_map[i];
			thresh_ptr[i].notify = callback;
			thresh_ptr[i].trip_triggered = -1;
			thresh_ptr[i].parent = &thresh[index];
			thresh_ptr[i].threshold[0].temp = hi_temp;
			thresh_ptr[i].threshold[0].trip =
				THERMAL_TRIP_CONFIGURABLE_HI;
			thresh_ptr[i].threshold[1].temp = low_temp;
			thresh_ptr[i].threshold[1].trip =
				THERMAL_TRIP_CONFIGURABLE_LOW;
			thresh_ptr[i].threshold[0].notify =
			thresh_ptr[i].threshold[1].notify = msm_thermal_notify;
			thresh_ptr[i].threshold[0].data =
			thresh_ptr[i].threshold[1].data =
				(void *)&thresh_ptr[i];
		}
	} else {
		thresh_ptr->sensor_id = sensor_id;
		thresh_ptr->notify = callback;
		thresh_ptr->trip_triggered = -1;
		thresh_ptr->parent = &thresh[index];
		thresh_ptr->threshold[0].temp = hi_temp;
		thresh_ptr->threshold[0].trip =
			THERMAL_TRIP_CONFIGURABLE_HI;
		thresh_ptr->threshold[1].temp = low_temp;
		thresh_ptr->threshold[1].trip =
			THERMAL_TRIP_CONFIGURABLE_LOW;
		thresh_ptr->threshold[0].notify =
		thresh_ptr->threshold[1].notify = msm_thermal_notify;
		thresh_ptr->threshold[0].data =
		thresh_ptr->threshold[1].data = (void *)thresh_ptr;
	}

init_thresh_exit:
	return ret;
}

/*
 * We will reset the cpu frequencies limits here. The core online/offline
 * status will be carried over to the process stopping the msm_thermal, as
 * we dont want to online a core and bring in the thermal issues.
 */
static void __ref disable_msm_thermal(void)
{
	uint32_t cpu = 0;

	/* make sure check_temp is no longer running */
	cancel_delayed_work_sync(&check_temp_work);

	get_online_cpus();
	for_each_possible_cpu(cpu) {
		if (cpus[cpu].limited_max_freq == UINT_MAX &&
				cpus[cpu].limited_min_freq == 0)
			continue;
		cpus[cpu].limited_max_freq = UINT_MAX;
		cpus[cpu].limited_min_freq = 0;
		if (cpu_online(cpu)) {
			if (cpufreq_update_policy(cpu))
				pr_info("Unable to update policy for cpu:%d\n", cpu);
		}
	}
	put_online_cpus();
}

static void interrupt_mode_init(void)
{
	if (!msm_thermal_probed) {
		interrupt_mode_enable = true;
		return;
	}
	if (intelli_enabled) {
		pr_info("Interrupt mode init\n");
		intelli_enabled = 0;
		disable_msm_thermal();
		hotplug_init();
		freq_mitigation_init();
		thermal_monitor_init();
	}
}

static int __ref set_enabled(const char *val, const struct kernel_param *kp)
{
	int ret = 0;

	if (*val == '0' || *val == 'n' || *val == 'N') {
		interrupt_mode_init();
		pr_info("%s: disabled!\n", KBUILD_MODNAME);
	} else {
		if (!intelli_enabled) {
			intelli_enabled = 1;
			schedule_delayed_work(&check_temp_work,
					msecs_to_jiffies(1000));
			pr_info("%s: rescheduling...\n", KBUILD_MODNAME);
		} else
			pr_info("%s: already running...\n \
				if you wish to disable echo N > \
				intelli_enabled\n", KBUILD_MODNAME);
	}
	ret = param_set_bool(val, kp);
	pr_info("%s: intelli_enabled = %d\n", KBUILD_MODNAME, intelli_enabled);

	return ret;
}

static struct kernel_param_ops module_ops = {
	.set = set_enabled,
	.get = param_get_bool,
};

module_param_cb(intelli_enabled, &module_ops, &intelli_enabled, 0644);
MODULE_PARM_DESC(intelli_enabled, "enforce thermal limit on cpu");

#ifdef CONFIG_INTELLI_THERMAL_STATS
static ssize_t show_thermal_stats(struct kobject *kobj,
                struct kobj_attribute *attr, char *buf)
{

	int i = 0;
	int tmp = 0;

	/* clear out old stats */
	msm_thermal_stats.throttled = 0;
	msm_thermal_stats.warning = 0;
	msm_thermal_stats.normal = 0;

	for (i = 0; i < MAX_HISTORY_SZ; i++) {
		tmp = msm_thermal_stats.temp_history[i];
		if (tmp >= msm_thermal_info_local.limit_temp_degC)
			msm_thermal_stats.throttled++;
		else if (tmp < msm_thermal_info_local.limit_temp_degC &&
			 tmp >= (msm_thermal_info_local.limit_temp_degC -
				 msm_thermal_info_local.temp_hysteresis_degC))
			msm_thermal_stats.warning++;
		else
			msm_thermal_stats.normal++;
	}
        return snprintf(buf, PAGE_SIZE, "%u %u %u\n",
			msm_thermal_stats.throttled,
			msm_thermal_stats.warning,
			msm_thermal_stats.normal);
}
static __refdata struct kobj_attribute msm_thermal_stat_attr =
__ATTR(statistics, 0444, show_thermal_stats, NULL);

static __refdata struct attribute *msm_thermal_stat_attrs[] = {
        &msm_thermal_stat_attr.attr,
        NULL,
};

static __refdata struct attribute_group msm_thermal_stat_attr_group = {
        .attrs = msm_thermal_stat_attrs,
};

static __init int msm_thermal_add_stat_nodes(void)
{
	struct kobject *module_kobj = NULL;
	struct kobject *stat_kobj = NULL;
	int ret = 0;

	module_kobj = kset_find_obj(module_kset, KBUILD_MODNAME);
	if (!module_kobj) {
		pr_err("%s: cannot find kobject for module\n",
			KBUILD_MODNAME);
		ret = -ENOENT;
		goto done_stat_nodes;
	}

	stat_kobj = kobject_create_and_add("thermal_stats", module_kobj);
	if (!stat_kobj) {
		pr_err("%s: cannot create core control kobj\n",
				KBUILD_MODNAME);
		ret = -ENOMEM;
		goto done_stat_nodes;
	}

	ret = sysfs_create_group(stat_kobj, &msm_thermal_stat_attr_group);
	if (ret) {
		pr_err("%s: cannot create group\n", KBUILD_MODNAME);
		goto done_stat_nodes;
	}

	return 0;

done_stat_nodes:
	if (stat_kobj)
		kobject_del(stat_kobj);
	return ret;
}
#endif

static ssize_t show_cc_enabled_dummy(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", enabled);
}

static ssize_t __ref store_cc_enabled_dummy(struct kobject *kobj,
                struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	int val = 0;

	ret = kstrtoint(buf, 10, &val);
	if (ret)
		pr_err("Invalid input %s. err:%d\n", buf, ret);

	if (enabled == !!val)
		return count;

	enabled = !!val;

	return count;
}

static ssize_t show_cc_enabled(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", core_control);
}

static ssize_t __ref store_cc_enabled(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	int val = 0;

	ret = kstrtoint(buf, 10, &val);
	if (ret) {
		pr_err("Invalid input %s. err:%d\n", buf, ret);
		goto done_store_cc;
	}

	if (core_control == !!val)
		goto done_store_cc;

	core_control = !!val;
	if (core_control) {
		pr_info("Core control enabled\n");
		register_cpu_notifier(&msm_thermal_cpu_notifier);
		enabled = true;
		if (hotplug_task)
			complete(&hotplug_notify_complete);
		else
			pr_err("Hotplug task is not initialized\n");
	} else {
		pr_info("Core control disabled\n");
		unregister_cpu_notifier(&msm_thermal_cpu_notifier);
		enabled = false;
	}

done_store_cc:
	return count;
}

static ssize_t show_cpus_offlined(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", cpus_offlined);
}

static ssize_t __ref store_cpus_offlined(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	uint32_t val = 0;
	uint32_t cpu;

	mutex_lock(&core_control_mutex);
	ret = kstrtouint(buf, 10, &val);
	if (ret) {
		pr_err("Invalid input %s. err:%d\n", buf, ret);
		goto done_cc;
	}

	if (intelli_enabled) {
		pr_err("Ignoring request; polling thread is enabled.\n");
		goto done_cc;
	}

	for_each_possible_cpu(cpu) {
		if (!(msm_thermal_info_local.core_control_mask & BIT(cpu)))
			continue;
		cpus[cpu].user_offline = !!(val & BIT(cpu));
		pr_debug("\"%s\"(PID:%i) requests %s CPU%d.\n", current->comm,
			current->pid, (cpus[cpu].user_offline) ? "offline" :
			"online", cpu);
	}

	if (hotplug_task)
		complete(&hotplug_notify_complete);
	else
		pr_err("Hotplug task is not initialized\n");
done_cc:
	mutex_unlock(&core_control_mutex);
	return count;
}

static __refdata struct kobj_attribute cc_enabled_attr =
__ATTR(core_control, 0664, show_cc_enabled, store_cc_enabled);

static __refdata struct kobj_attribute cc_enabled_dummy_attr =
__ATTR(enabled, 0664, show_cc_enabled_dummy, store_cc_enabled_dummy);

static __refdata struct kobj_attribute cpus_offlined_attr =
__ATTR(cpus_offlined, 0664, show_cpus_offlined, store_cpus_offlined);

static __refdata struct attribute *cc_attrs[] = {
	&cc_enabled_attr.attr,
	&cc_enabled_dummy_attr.attr,
	&cpus_offlined_attr.attr,
	NULL,
};

static __refdata struct attribute_group cc_attr_group = {
	.attrs = cc_attrs,
};

static __init int msm_thermal_add_cc_nodes(void)
{
	struct kobject *module_kobj = NULL;
	int ret = 0;

	module_kobj = kset_find_obj(module_kset, KBUILD_MODNAME);
	if (!module_kobj) {
		pr_err("cannot find kobject\n");
		ret = -ENOENT;
		goto done_cc_nodes;
	}

	cc_kobj = kobject_create_and_add("core_control", module_kobj);
	if (!cc_kobj) {
		pr_err("cannot create core control kobj\n");
		ret = -ENOMEM;
		goto done_cc_nodes;
	}

	ret = sysfs_create_group(cc_kobj, &cc_attr_group);
	if (ret) {
		pr_err("cannot create sysfs group. err:%d\n", ret);
		goto done_cc_nodes;
	}

	return 0;

done_cc_nodes:
	if (cc_kobj)
		kobject_del(cc_kobj);
	return ret;
}

int msm_thermal_pre_init(void)
{
	int ret = 0;

	tsens_get_max_sensor_num(&max_tsens_num);
	if (create_sensor_id_map()) {
		pr_err("Creating sensor id map failed\n");
		ret = -EINVAL;
		goto pre_init_exit;
	}

	if (!thresh) {
		thresh = kzalloc(
				sizeof(struct threshold_info) * MSM_LIST_MAX_NR,
				GFP_KERNEL);
		if (!thresh) {
			pr_err("kzalloc failed\n");
			ret = -ENOMEM;
			goto pre_init_exit;
		}
		memset(thresh, 0, sizeof(struct threshold_info) *
			MSM_LIST_MAX_NR);
	}
pre_init_exit:
	return ret;
}

int msm_thermal_init(struct msm_thermal_data *pdata)
{
	int ret = 0;
	uint32_t cpu;

	for_each_possible_cpu(cpu) {
		cpus[cpu].cpu = cpu;
		cpus[cpu].offline = 0;
		cpus[cpu].user_offline = 0;
		cpus[cpu].hotplug_thresh_clear = false;
		cpus[cpu].max_freq = false;
		cpus[cpu].user_max_freq = UINT_MAX;
		cpus[cpu].user_min_freq = 0;
		cpus[cpu].limited_max_freq = UINT_MAX;
		cpus[cpu].limited_min_freq = 0;
		cpus[cpu].freq_thresh_clear = false;
	}
	BUG_ON(!pdata);
	memcpy(&msm_thermal_info, pdata, sizeof(struct msm_thermal_data));

	if (check_sensor_id(msm_thermal_info_local.sensor_id)) {
		pr_err("Invalid sensor:%d for polling\n",
				msm_thermal_info_local.sensor_id);
		return -EINVAL;
	}

	pr_info("%s: polling enabled!\n", KBUILD_MODNAME);

	ret = cpufreq_register_notifier(&msm_thermal_cpufreq_notifier,
			CPUFREQ_POLICY_NOTIFIER);
	if (ret)
		pr_err("cannot register cpufreq notifier. err:%d\n", ret);

	pm_notifier(msm_thermal_suspend_callback, 0);
	INIT_DELAYED_WORK(&check_temp_work, check_temp);
	schedule_delayed_work(&check_temp_work,
			msecs_to_jiffies(5000));

	if (num_possible_cpus() > 1)
		register_cpu_notifier(&msm_thermal_cpu_notifier);

	return ret;
}

static int psm_reg_init(struct platform_device *pdev)
{
	int ret = 0;
	int i = 0;
	int j = 0;

	for (i = 0; i < psm_rails_cnt; i++) {
		psm_rails[i].reg = rpm_regulator_get(&pdev->dev,
				psm_rails[i].name);
		if (IS_ERR_OR_NULL(psm_rails[i].reg)) {
			ret = PTR_ERR(psm_rails[i].reg);
			if (ret != -EPROBE_DEFER) {
				pr_err("couldn't get rpm regulator %s. err%d\n",
					psm_rails[i].name, ret);
				psm_rails[i].reg = NULL;
				goto psm_reg_exit;
			}
			pr_info("Defer regulator %s probe\n",
					psm_rails[i].name);
			return ret;
		}
		/* Apps default vote for PWM mode */
		psm_rails[i].init = PMIC_PWM_MODE;
		ret = rpm_regulator_set_mode(psm_rails[i].reg,
				psm_rails[i].init);
		if (ret) {
			pr_err("Cannot set PMIC PWM mode. err:%d\n", ret);
			return ret;
		} else
			psm_rails[i].mode = PMIC_PWM_MODE;
	}

	return ret;

psm_reg_exit:
	if (ret) {
		for (j = 0; j < i; j++) {
			if (psm_rails[j].reg != NULL)
				rpm_regulator_put(psm_rails[j].reg);
		}
	}

	return ret;
}

static int msm_thermal_add_psm_nodes(void)
{
	struct kobject *module_kobj = NULL;
	struct kobject *psm_kobj = NULL;
	struct kobject *psm_reg_kobj[MAX_RAILS] = {0};
	int rc = 0;
	int i = 0;

	if (!psm_probed) {
		psm_nodes_called = true;
		return rc;
	}

	if (psm_probed && psm_rails_cnt == 0)
		return rc;

	module_kobj = kset_find_obj(module_kset, KBUILD_MODNAME);
	if (!module_kobj) {
		pr_err("cannot find kobject\n");
		rc = -ENOENT;
		goto psm_node_exit;
	}

	psm_kobj = kobject_create_and_add("pmic_sw_mode", module_kobj);
	if (!psm_kobj) {
		pr_err("cannot create psm kobject\n");
		rc = -ENOMEM;
		goto psm_node_exit;
	}

	for (i = 0; i < psm_rails_cnt; i++) {
		psm_reg_kobj[i] = kobject_create_and_add(psm_rails[i].name,
					psm_kobj);
		if (!psm_reg_kobj[i]) {
			pr_err("cannot create kobject for %s\n",
					psm_rails[i].name);
			rc = -ENOMEM;
			goto psm_node_exit;
		}
		psm_rails[i].attr_gp.attrs = kzalloc( \
				sizeof(struct attribute *) * 2, GFP_KERNEL);
		if (!psm_rails[i].attr_gp.attrs) {
			pr_err("kzalloc failed\n");
			rc = -ENOMEM;
			goto psm_node_exit;
		}

		PSM_RW_ATTRIB(psm_rails[i], psm_rails[i].mode_attr, 0, mode);
		psm_rails[i].attr_gp.attrs[1] = NULL;

		rc = sysfs_create_group(psm_reg_kobj[i],
				&psm_rails[i].attr_gp);
		if (rc) {
			pr_err("cannot create attribute group for %s. err:%d\n",
					psm_rails[i].name, rc);
			goto psm_node_exit;
		}
	}

	return rc;

psm_node_exit:
	if (rc) {
		for (i = 0; i < psm_rails_cnt; i++) {
			kobject_del(psm_reg_kobj[i]);
			kfree(psm_rails[i].attr_gp.attrs);
		}
		if (psm_kobj)
			kobject_del(psm_kobj);
	}
	return rc;
}

static int probe_psm(struct device_node *node, struct msm_thermal_data *data,
		struct platform_device *pdev)
{
	int ret = 0;
	int j = 0;
	char *key = NULL;

	psm_rails = NULL;

	key = "qcom,pmic-sw-mode-temp";
	ret = of_property_read_u32(node, key, &data->psm_temp_degC);
	if (ret)
		goto read_node_fail;

	key = "qcom,pmic-sw-mode-temp-hysteresis";
	ret = of_property_read_u32(node, key, &data->psm_temp_hyst_degC);
	if (ret)
		goto read_node_fail;

	key = "qcom,pmic-sw-mode-regs";
	psm_rails_cnt = of_property_count_strings(node, key);
	psm_rails = kzalloc(sizeof(struct psm_rail) * psm_rails_cnt,
			GFP_KERNEL);
	if (!psm_rails) {
		pr_err("Fail to allocate memory for psm rails\n");
		psm_rails_cnt = 0;
		return -ENOMEM;
	}

	for (j = 0; j < psm_rails_cnt; j++) {
		ret = of_property_read_string_index(node, key, j,
				&psm_rails[j].name);
		if (ret)
			goto read_node_fail;
	}

	if (psm_rails_cnt) {
		ret = psm_reg_init(pdev);
		if (ret) {
			pr_err("Err regulator init. err:%d. KTM continues.\n",
					ret);
			goto read_node_fail;
		}
		psm_enabled = true;
	}

read_node_fail:
	psm_probed = true;
	if (ret) {
		dev_info(&pdev->dev,
		"%s:Failed reading node=%s, key=%s. err=%d. KTM continues\n",
			__func__, node->full_name, key, ret);
		kfree(psm_rails);
		psm_rails_cnt = 0;
	}
	if (ret == -EPROBE_DEFER)
		psm_probed = false;
	return ret;
}

static int probe_cc(struct device_node *node, struct msm_thermal_data *data,
		struct platform_device *pdev)
{
	char *key = NULL;
	uint32_t cpu_cnt = 0;
	int ret = 0;
	uint32_t cpu = 0;

	if (num_possible_cpus() > 1)
		hotplug_enabled = 1;

	key = "qcom,hotplug-temp";
	ret = of_property_read_u32(node, key, &data->hotplug_temp_degC);
	if (ret)
		goto hotplug_node_fail;

	key = "qcom,hotplug-temp-hysteresis";
	ret = of_property_read_u32(node, key,
			&data->hotplug_temp_hysteresis_degC);
	if (ret)
		goto hotplug_node_fail;

	key = "qcom,cpu-sensors";
	cpu_cnt = of_property_count_strings(node, key);
	if (cpu_cnt < num_possible_cpus()) {
		pr_err("Wrong number of cpu sensors:%d\n", cpu_cnt);
		ret = -EINVAL;
		goto hotplug_node_fail;
	}

	for_each_possible_cpu(cpu) {
		ret = of_property_read_string_index(node, key, cpu,
				&cpus[cpu].sensor_type);
		if (ret)
			goto hotplug_node_fail;
	}

hotplug_node_fail:
	if (ret) {
		dev_info(&pdev->dev,
		"%s:Failed reading node=%s, key=%s. err=%d. KTM continues\n",
			KBUILD_MODNAME, node->full_name, key, ret);
		hotplug_enabled = 0;
	}

	return ret;
}

static int probe_therm_reset(struct device_node *node,
		struct msm_thermal_data *data,
		struct platform_device *pdev)
{
	char *key = NULL;
	int ret = 0;

	key = "qcom,therm-reset-temp";
	ret = of_property_read_u32(node, key, &data->therm_reset_temp_degC);
	if (ret)
		goto PROBE_RESET_EXIT;

	ret = init_threshold(MSM_THERM_RESET, MONITOR_ALL_TSENS,
		data->therm_reset_temp_degC, data->therm_reset_temp_degC - 10,
		therm_reset_notify);
	if (ret) {
		pr_err("Therm reset data structure init failed\n");
		goto PROBE_RESET_EXIT;
	}

	therm_reset_enabled = true;

PROBE_RESET_EXIT:
	if (ret) {
		dev_info(&pdev->dev,
		"%s:Failed reading node=%s, key=%s err=%d. KTM continues\n",
			__func__, node->full_name, key, ret);
		therm_reset_enabled = false;
	}
	return ret;
}

static int probe_freq_mitigation(struct device_node *node,
		struct msm_thermal_data *data,
		struct platform_device *pdev)
{
	char *key = NULL;
	int ret = 0;

	key = "qcom,freq-mitigation-temp";
	ret = of_property_read_u32(node, key, &data->freq_mitig_temp_degc);
	if (ret)
		goto PROBE_FREQ_EXIT;

	key = "qcom,freq-mitigation-temp-hysteresis";
	ret = of_property_read_u32(node, key,
		&data->freq_mitig_temp_hysteresis_degc);
	if (ret)
		goto PROBE_FREQ_EXIT;

	key = "qcom,freq-mitigation-value";
	ret = of_property_read_u32(node, key, &data->freq_limit);
	if (ret)
		goto PROBE_FREQ_EXIT;

	key = "qcom,freq-mitigation-control-mask";
	ret = of_property_read_u32(node, key, &data->freq_mitig_control_mask);
	if (ret)
		goto PROBE_FREQ_EXIT;

	freq_mitigation_enabled = 1;

PROBE_FREQ_EXIT:
	if (ret) {
		dev_info(&pdev->dev,
		"%s:Failed reading node=%s, key=%s. err=%d. KTM continues\n",
			__func__, node->full_name, key, ret);
		freq_mitigation_enabled = 0;
	}
	return ret;
}

static int __devinit msm_thermal_dev_probe(struct platform_device *pdev)
{
	int ret = 0;
	char *key = NULL;
	struct device_node *node = pdev->dev.of_node;
	struct msm_thermal_data data;

	pr_info("%s: msm_thermal_dev_probe begin...\n", KBUILD_MODNAME);

	memset(&data, 0, sizeof(struct msm_thermal_data));
	ret = msm_thermal_pre_init();
	if (ret) {
		pr_err("thermal pre init failed. err:%d\n", ret);
		goto fail;
	}

	ret = probe_cc(node, &data, pdev);

	ret = probe_freq_mitigation(node, &data, pdev);
	ret = probe_therm_reset(node, &data, pdev);

	/*
	 * Probe optional properties below. Call probe_psm before
	 * probe_vdd_rstr because rpm_regulator_get has to be called
	 * before devm_regulator_get
	 * probe_ocr should be called after probe_vdd_rstr to reuse the
	 * regualtor handle. calling devm_regulator_get more than once
	 * will fail.
	 */
	ret = probe_psm(node, &data, pdev);
	if (ret == -EPROBE_DEFER)
		goto fail;

	key = "qcom,online-hotplug-core";
	if (of_property_read_bool(node, key))
		online_core = true;
	else
		online_core = false;

	/*
	 * In case sysfs add nodes get called before probe function.
	 * Need to make sure sysfs node is created again
	 */
	if (psm_nodes_called) {
		msm_thermal_add_psm_nodes();
		psm_nodes_called = false;
	}
	msm_thermal_ioctl_init();
	ret = msm_thermal_init(&data);
	msm_thermal_probed = true;

	if (interrupt_mode_enable) {
		interrupt_mode_init();
		interrupt_mode_enable = false;
	}

	pr_info("%s: msm_thermal_dev_probe completed!\n", KBUILD_MODNAME);

	/* start intelli thermal again */
	intelli_enabled = 1;
	pr_info("%s: rescheduling...\n", KBUILD_MODNAME);
	schedule_delayed_work(&check_temp_work,
			msecs_to_jiffies(1000));

	return ret;
fail:
	if (ret)
		pr_err("Failed reading node=%s, key=%s. err:%d\n",
			node->full_name, key, ret);

	pr_info("%s: msm_thermal_dev_probe failed!\n", KBUILD_MODNAME);

	return ret;
}

static int msm_thermal_dev_exit(struct platform_device *inp_dev)
{
	msm_thermal_ioctl_cleanup();
	if (thresh) {
		kfree(thresh);
		thresh = NULL;
	}

	pr_info("msm_thermal_dev: removed!\n");
	return 0;
}

static struct of_device_id msm_thermal_match_table[] = {
	{.compatible = "qcom,msm-thermal"},
	{},
};

static struct platform_driver msm_thermal_device_driver = {
	.probe = msm_thermal_dev_probe,
	.driver = {
		.name = "msm-thermal",
		.owner = THIS_MODULE,
		.of_match_table = msm_thermal_match_table,
	},
	.remove = msm_thermal_dev_exit,
};

int __init msm_thermal_device_init(void)
{
	return platform_driver_register(&msm_thermal_device_driver);
}

int __init msm_thermal_late_init(void)
{
	if (num_possible_cpus() > 1)
		msm_thermal_add_cc_nodes();
	msm_thermal_add_psm_nodes();
	interrupt_mode_init();

#ifdef CONFIG_INTELLI_THERMAL_STATS
	msm_thermal_add_stat_nodes();
#endif
	return 0;
}
late_initcall(msm_thermal_late_init);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Paul Reioux <reioux@gmail.com>, Yuri Sh. <yurboss@gmail.com>, \
		Praveen Chidambaram <pchidamb@codeaurora.org>");
MODULE_DESCRIPTION("intelligent thermal driver for Qualcomm based SOCs");
