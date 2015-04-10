/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*

        Display driver for dynamic scaling support

Copyright(c) 2013 by LG Electronics. All Rights Reserved.
        Sangduck Kim <sangduck.kim@lge.com>
*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/
/*==========================================================================

                      EDIT HISTORY FOR FILE

  This section contains comments describing changes made to this file.
  Notice that changes are listed in reverse chronological order.

when       who        what, where, why
--------   --------   -------------------------------------------------------
12/01/13   sdkim   Created file
======================================================-====================*/
#ifndef __g3_DISPLAY_DEVFREQ_H__
#define __g3_DISPLAY_DEVFREQ_H__

struct display_fps_data {
	u32 prev_fcount;
	unsigned int fps;
};

struct g3_display_data {
	struct devfreq *devfreq;
	struct opp *curr_opp;
	struct device *dev;

	struct delayed_work wq_lowfreq;
	struct delayed_work wq_highfreq;
	struct notifier_block nb_pm;
	struct mutex lock;
	struct display_fps_data fps_data;
};

struct display_opp_table {
	unsigned int idx;
	unsigned long freq;
	unsigned long volt;
};

enum g3_display_clk_level_idx {
	LV_0 = 0,
	LV_1,
//	LV_2,
	_LV_END_
};

enum g3_display_type {
	TYPE_DISPLAY_G3,
};

void g3_release_pm(struct device *device);
#ifdef CONFIG_PM
int g3_display_suspend(struct device *dev);
int g3_display_resume(struct device *dev);
#endif
int __devexit g3_display_remove(struct platform_device *pdev);
int g3_display_probe(struct platform_device *pdev);
int g3_display_profile_target(struct device *dev,
unsigned long *_freq, u32 options);
int g3_display_get_dev_status(struct device *dev,
struct devfreq_dev_status *stat);
void g3_display_profile_exit(struct device *dev);
int g3_opp_get_idx(unsigned long new_freq,
struct display_opp_table *table);
void g3_display_read_fps(struct g3_display_data *data,
struct devfreq_dev_status *stat);
void g3_display_set_lowfreq(struct work_struct *work);
void g3_display_set_highfreq(struct work_struct *work);
int g3_display_send_event_to_mdss_display(unsigned long val, void *v);
int g3_display_pm_notifier_callback(struct notifier_block *this,
unsigned long event, void *_data);
int g3_display_dev_platform_register(void);
void g3_display_dev_platform_unregister(void);

#endif
