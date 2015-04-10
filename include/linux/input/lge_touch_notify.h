#ifndef __LINUX_TOUCH_NOTIFY_H
#define __LINUX_TOUCH_NOTIFY_H

#include <linux/notifier.h>


/* the dsv on */
#define LCD_EVENT_TOUCH_LPWG_ON		0x01
/* the dsv off */
#define LCD_EVENT_TOUCH_LPWG_OFF	0x02
 /* the display on process started */
#define LCD_EVENT_ON_START		0x03
 /* the display on process end */
#define LCD_EVENT_ON_END		0x04
 /* the display off process started */
#define LCD_EVENT_OFF_START		0x05
 /* the display off process end */
#define LCD_EVENT_OFF_END		0x06
/* the display off process end */
#define TOUCH_EVENT_RESET_START		0x07
/* Touch REGISTER_DONE to the display process  */
#define TOUCH_EVENT_REGISTER_DONE    		0x08

struct touch_event {
	void *data;
};


int touch_register_client(struct notifier_block *nb);
int touch_unregister_client(struct notifier_block *nb);
int touch_notifier_call_chain(unsigned long val, void *v);
#endif /* _LINUX_TOUCH_NOTIFY_H */
