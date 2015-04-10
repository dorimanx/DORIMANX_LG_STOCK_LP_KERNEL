/*
  * Copyright (C) 2011 LGE, Inc.
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

/* android vibrator platform data */
struct android_irrc_platform_data {
    int enable_status;
    int (*irrc_init)(void);
    int (*pwm_set)(int enable, int gain, int n_value); /* PWM Set Function */
};


/* Debug Mask setting */
#define IRRC_DEBUG_PRINT   (0)
#define IRRC_ERROR_PRINT   (1)
#define IRRC_INFO_PRINT    (1)

#if (IRRC_INFO_PRINT)
#define INFO_MSG(fmt, args...) \
            printk(KERN_INFO "irrc: %s() " \
                fmt, __FUNCTION__, ##args);
#else
#define INFO_MSG(fmt, args...)
#endif

#if (IRRC_DEBUG_PRINT)
#define DEBUG_MSG(fmt, args...) \
            printk(KERN_INFO "irrc: %s() " \
                fmt, __FUNCTION__, ##args);
#else
#define DEBUG_MSG(fmt, args...)
#endif

#if (IRRC_ERROR_PRINT)
#define ERR_MSG(fmt, args...) \
            printk(KERN_ERR "irrc: %s() " \
                fmt, __FUNCTION__, ##args);
#else
#define ERR_MSG(fmt, args...)
#endif




#define IRRC_IOCTL_MAGIC 'a'

#define IRRC_START        _IOW(IRRC_IOCTL_MAGIC, 0, int)
#define IRRC_STOP         _IOW(IRRC_IOCTL_MAGIC, 1, int)
//#endif
