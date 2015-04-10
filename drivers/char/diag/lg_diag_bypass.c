/*
 * DIAG Bypass Driver for LGE WRFT
 *
 * Sungchul Park <sungchul.park@lge.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
*/

#include "lg_diag_bypass.h"

static char bypass_request[0x1000] = {0, };
static char bypass_response[0x1000] = {0, };

static int is_opened = 0;

struct bypass_driver *lge_bypass_drv = NULL;

int lge_bypass_is_opened()
{
    return is_opened;
}

int lge_bypass_status()
{
    struct bypass_driver *bypass_drv = NULL;

    bypass_drv = lge_bypass_drv;
    if(!bypass_drv) {
        pr_err("%s: bypass_drv is null\n", __func__);
        return 0;
    }

    if(!bypass_drv->enable) {
        return 0;
    }

    return 1;
}

int lge_bypass_process(char *buf, int count)
{
    int left = 0;
    int nwrite = 0;
    int offset = 0;
    struct bypass_driver *bypass_drv = NULL;

    bypass_drv = lge_bypass_drv;
    if(!bypass_drv) {
        pr_err("%s: bypass_drv is null\n", __func__);
        return 0;
    }

    if(!bypass_drv->enable) {
        return 0;
    }

    if(count < 4) {
        pr_err("%s: packet size is too small (expected:%d, count:%d)\n", __func__, 4, count);
        return 0;
    }

    if(count > BYPASS_MAX_PACKET_SIZE) {
        pr_err("%s: packet size is too large (expected:%d, count:%d)\n", __func__, BYPASS_MAX_PACKET_SIZE, count);
        return 0;
    }

    if((buf[0] == 0x79) || (buf[0] == 0x92)) {
        return 0;
    }

    left = count;
	memcpy(bypass_response, buf, count);
    pr_info("%s: write to tty (count:%d)\n", __func__, count);
    do {
#if defined(CONFIG_ARCH_MSM8916) || defined(CONFIG_ARCH_APQ8084) || defined(CONFIG_ARCH_ODIN)
        nwrite = tty_insert_flip_string(bypass_drv->port, (unsigned char *)&bypass_response[offset], left);
        tty_flip_buffer_push(bypass_drv->port);
#else
        nwrite = tty_insert_flip_string(bypass_drv->tty_str, (unsigned char *)&bypass_response[offset], left);
        tty_flip_buffer_push(bypass_drv->tty_str);
#endif

        offset += nwrite;
        left -= nwrite;
    } while(left > 0);

    return count;
}

static int lge_bypass_open(struct tty_struct *tty, struct file *file)
{
    struct bypass_driver *bypass_drv = NULL;

    bypass_drv = lge_bypass_drv;
    if(!bypass_drv) {
        pr_err("%s: bypass_drv is null\n", __func__);
        return -ENODEV;
    }

#ifdef CONFIG_DIAGFWD_BRIDGE_CODE
    diagfwd_cancel_hsic(REOPEN_HSIC); // QCT 161032 migration - NEED TO CHECK
    diagfwd_connect_bridge(0);
#endif

    tty_port_tty_set(bypass_drv->port, tty);
#if defined(CONFIG_ARCH_MSM8916) || defined(CONFIG_ARCH_APQ8084) || defined(CONFIG_ARCH_ODIN)
    bypass_drv->port->low_latency = 0;
#endif

    tty->driver_data = bypass_drv;
    bypass_drv->tty_str = tty;

    set_bit(TTY_NO_WRITE_SPLIT, &bypass_drv->tty_str->flags);

    is_opened = 1;

    pr_info("%s success\n", __func__);

    return 0;
}

static void lge_bypass_close(struct tty_struct *tty, struct file *file)
{
    struct bypass_driver *bypass_drv = NULL;

#ifdef CONFIG_DIAGFWD_BRIDGE_CODE
    diagfwd_cancel_hsic(REOPEN_HSIC); // QCT 161032 migration - NEED TO CHECK
    diagfwd_disconnect_bridge(0);
#endif

    is_opened = 0;

    if(!tty) {
        pr_err( "%s tty is null\n", __func__);
    }
    else {
        bypass_drv = tty->driver_data;
        bypass_drv->enable = 0;
        tty_port_tty_set(bypass_drv->port, NULL);
        pr_info( "%s success\n", __func__);
    }
}

static int lge_bypass_write(struct tty_struct *tty, const unsigned char *buf, int count)
{
    struct bypass_driver *bypass_drv = NULL;

	pr_info("%s. count : %d\n", __func__, count);
    bypass_drv = lge_bypass_drv;
    tty->driver_data = bypass_drv;
    bypass_drv->tty_str = tty;

	memcpy(bypass_request, buf, count);
#ifdef CONFIG_DIAGFWD_BRIDGE_CODE
    if(buf[0] == 0xFA || buf[0] == 0xEF || buf[0] == 0x3A || (buf[0]==0x29 && buf[1]==0x01) || (buf[0]==0x29 && buf[1]==0x02)) {
        diag_bypass_request(bypass_request, count);
    }
    else {
        diag_bypass_request_bridge(bypass_request, count);
    }
#else
    diag_bypass_request(bypass_request, count);
#endif

    bypass_drv->enable = 1;

    return count;
}

static int lge_bypass_write_room(struct tty_struct *tty)
{
    return BYPASS_MAX_PACKET_SIZE;
}

static void lge_bypass_unthrottle(struct tty_struct *tty)
{
}

static int lge_bypass_ioctl(struct tty_struct *tty, unsigned int cmd, unsigned long arg)
{
    return 0;
}

static const struct tty_operations lge_bypass_ops = {
    .open = lge_bypass_open,
    .close = lge_bypass_close,
    .write = lge_bypass_write,
    .write_room = lge_bypass_write_room,
    .unthrottle = lge_bypass_unthrottle,
    .ioctl = lge_bypass_ioctl,
};

static int __init lge_bypass_init(void)
{
    int ret = 0;

    struct device *tty_dev = NULL;
    struct bypass_driver *bypass_drv = NULL;

    pr_info("%s\n", __func__);

    bypass_drv = kzalloc(sizeof(struct bypass_driver), GFP_KERNEL);
    if (!bypass_drv) {
        pr_err( "%s: memory alloc fail\n", __func__);
        return 0;
    }

    bypass_drv->tty_drv = alloc_tty_driver(BYPASS_MAX_DRV);
    if(!bypass_drv->tty_drv) {
        pr_err( "%s: tty alloc driver fail\n", __func__);
        kfree(bypass_drv);
        return 0;
    }

    bypass_drv->port = kzalloc(sizeof(struct tty_port), GFP_KERNEL);
    if(!bypass_drv->port) {
        pr_err( "%s: memory alloc port fail\n", __func__);
        kfree(bypass_drv->tty_drv);
        kfree(bypass_drv);
        return 0;
    }
    tty_port_init(bypass_drv->port);

    lge_bypass_drv = bypass_drv;

    bypass_drv->tty_drv->name = "lge_diag_bypass";
    bypass_drv->tty_drv->owner = THIS_MODULE;
    bypass_drv->tty_drv->driver_name = "lge_diag_bypass";

    bypass_drv->tty_drv->type = TTY_DRIVER_TYPE_SERIAL;
    bypass_drv->tty_drv->subtype = SERIAL_TYPE_NORMAL;
    bypass_drv->tty_drv->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV | TTY_DRIVER_RESET_TERMIOS;

    bypass_drv->tty_drv->init_termios = tty_std_termios;
    bypass_drv->tty_drv->init_termios.c_iflag = IGNBRK | IGNPAR;
    bypass_drv->tty_drv->init_termios.c_oflag = 0;
    bypass_drv->tty_drv->init_termios.c_cflag = B9600 | CS8 | CREAD | HUPCL | CLOCAL;
    bypass_drv->tty_drv->init_termios.c_lflag = 0;

    tty_set_operations(bypass_drv->tty_drv, &lge_bypass_ops);
#if defined(CONFIG_ARCH_MSM8916) || defined(CONFIG_ARCH_APQ8084) || defined(CONFIG_ARCH_ODIN)
    tty_port_link_device(bypass_drv->port, bypass_drv->tty_drv, 0);
#endif
    ret = tty_register_driver(bypass_drv->tty_drv);

    if (ret) {
        pr_err("%s: fail to tty_register_driver\n", __func__);
        put_tty_driver(bypass_drv->tty_drv);
#if defined(CONFIG_ARCH_MSM8916) || defined(CONFIG_ARCH_APQ8084) || defined(CONFIG_ARCH_ODIN)
        tty_port_destroy(bypass_drv->port);
#endif
        bypass_drv->tty_drv = NULL;
        kfree(bypass_drv->port);
        kfree(bypass_drv);
        return 0;
    }

    tty_dev = tty_register_device(bypass_drv->tty_drv, 0, NULL);

    if (IS_ERR(tty_dev)) {
        pr_err("%s: fail to tty_register_device\n", __func__);
        tty_unregister_driver(bypass_drv->tty_drv);
        put_tty_driver(bypass_drv->tty_drv);
#if defined(CONFIG_ARCH_MSM8916) || defined(CONFIG_ARCH_APQ8084) || defined(CONFIG_ARCH_ODIN)
        tty_port_destroy(bypass_drv->port);
#endif
        kfree(bypass_drv->port);
        kfree(bypass_drv);
        return 0;
    }

    bypass_drv->enable = 0;

    pr_info( "%s: success\n", __func__);

    return 0;
}

static void __exit lge_bypass_exit(void)
{
    struct bypass_driver *bypass_drv = NULL;

    bypass_drv = lge_bypass_drv;

    if (!bypass_drv) {
        pr_err("%s: bypass_dev is null\n", __func__);
    }
    else {
#if defined(CONFIG_ARCH_MSM8916) || defined(CONFIG_ARCH_APQ8084) || defined(CONFIG_ARCH_ODIN)
        tty_port_destroy(bypass_drv->port);
#endif
        mdelay(20);
        tty_unregister_device(bypass_drv->tty_drv, 0);
        if(tty_unregister_driver(bypass_drv->tty_drv)) {
            pr_err("%s: tty_unregister_driver failed\n", __func__);
        }
        put_tty_driver(bypass_drv->tty_drv);
        bypass_drv->tty_drv = NULL;
        kfree(bypass_drv->port);
        kfree(bypass_drv);
        bypass_drv = NULL;

        pr_info( "%s: success\n", __func__);
    }
}

module_init(lge_bypass_init);
module_exit(lge_bypass_exit);

MODULE_DESCRIPTION("LGE DIAG BYPASS");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sungchul Park <sungchul.park@lge.com>");

