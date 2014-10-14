/*
 * Copyright (c) 2013 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __MACH_CPUFREQ_H
#define __MACH_CPUFREQ_H

#if defined(CONFIG_DEVFREQ_GOV_MSM_CPUFREQ)
extern int devfreq_msm_cpufreq_update_bw(void);
extern int register_devfreq_msm_cpufreq(void);
extern unsigned int get_cpu_min_lock(unsigned int cpu);
extern void set_cpu_min_lock(unsigned int cpu, int freq);
extern unsigned int get_max_lock(unsigned int cpu);
extern void set_max_lock(unsigned int cpu, unsigned int freq);

#else
static int devfreq_msm_cpufreq_update_bw(void)
{
	return 0;
}
static int register_devfreq_msm_cpufreq(void)
{
	return 0;
}
static unsigned int get_cpu_min_lock(unsigned int cpu)
{
	return -ENOSYS;
}
static void set_cpu_min_lock(unsigned int cpu, int freq)
{
	return -ENOSYS;
}
static unsigned int get_max_lock(unsigned int cpu)
{
	return -ENOSYS;
}
static void set_max_lock(unsigned int cpu, unsigned int freq)
{
	return -ENOSYS;
}
#endif

#if defined(CONFIG_CPU_FREQ_MSM)
extern unsigned long msm_cpufreq_get_bw(void);
#else
extern unsigned long msm_cpufreq_get_bw(void)
{
	return ULONG_MAX;
}
#endif

#endif
