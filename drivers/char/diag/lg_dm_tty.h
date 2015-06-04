#ifndef TTY_LGE_DM_H_
#define TTY_LGE_DM_H_

#ifdef CONFIG_LGE_DM_APP
struct dm_tty {
	wait_queue_head_t   waitq;
	struct task_struct *tty_ts;
	struct tty_driver *tty_drv;
	struct tty_struct *tty_str;
	int tty_state;
	int logging_mode;
	int set_logging;
	struct workqueue_struct *dm_wq;
	struct work_struct dm_usb_work;
    struct work_struct dm_dload_work;
};

extern struct dm_tty *lge_dm_tty;

#endif
#endif /*              */
