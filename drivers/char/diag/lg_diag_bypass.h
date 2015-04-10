#ifndef LG_DIAG_BYPASS_H
#define LG_DIAG_BYPASS_H

#include <linux/list.h>
#include <linux/kthread.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/diagchar.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>

#include "diagchar.h"
#include "diagfwd.h"
#include "diagfwd_bridge.h"
#include "diagfwd_hsic.h"
#include "diagmem.h"

#define BYPASS_MAX_DRV 0x1
#define BYPASS_MAX_PACKET_SIZE 0x1000

struct bypass_driver {
    wait_queue_head_t waitq;
    struct tty_driver *tty_drv;
    struct tty_struct *tty_str;
    struct tty_port *port;
    int enable;
};

extern struct bypass_driver *lge_bypass_drv;
extern int lge_bypass_process(char *buf, int count);
extern int lge_bypass_status(void);
extern int lge_bypass_is_opened(void);

#endif /* LG_DIAG_BYPASS_H */
