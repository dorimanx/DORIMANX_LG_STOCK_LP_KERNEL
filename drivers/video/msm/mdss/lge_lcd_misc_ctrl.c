/*  arch/arm/mach-msm/mdss/lge_audio_misc_ctl.c
 *
 * Copyright (C) 2012 LGE, Inc.
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

#include <linux/sched.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <mach/board_lge.h>
#include <mach/board.h>
#include "mdss_dsi.h"

struct dsi_cmd_desc *tun_dsi_panel_on_cmds;
u32 tun_porch_value[6] = {0, 0, 0, 0, 0, 0};

#define IOCTL_READ_INIT _IOW('a', 0, int)
#define IOCTL_WRITE_INIT _IOW('a', 1, int)
#define IOCTL_READ_PORCH _IOW('a', 2, int)
#define IOCTL_WRITE_PORCH _IOW('a', 3, int)
#define IOCTL_READ_LUT _IOW('a', 4, int)
#define IOCTL_WRITE_LUT _IOW('a', 5, int)

#define TUNING_REGSIZE 1600
#define REGNUM 40

#define TUNING_DBG 0

int num_cmds;
int tun_len[REGNUM];

char init_buf[TUNING_REGSIZE];
struct tuning_init{
	char buf[TUNING_REGSIZE];
	int size[REGNUM];
	int numcmds;
};

int read_porch(unsigned long tmp)
{
	int size = ARRAY_SIZE(tun_porch_value)*4;
	pr_info(KERN_INFO "read_porch_value \n");
	if (copy_to_user((uint32_t *)tmp, tun_porch_value, size)) {
		pr_info(KERN_ERR "read_file : error of copy_to_user_buff\n");
		return -EFAULT;
	}

	return 0;
}

/* need to check porch value in mdss_fb_register at mdss_fb.c,
 *  in dsi_panel_device_register at mdss_dsi.c,
 *  in mdss_edp_edid2pinfo at mdss_edp.c
 */
static int write_porch(unsigned long tmp)
{
	u32 *buf;
	int size = ARRAY_SIZE(tun_porch_value)*4;

	pr_info(KERN_INFO "write porch file\n");

	buf = kmalloc(size, GFP_KERNEL);
	if (copy_from_user(buf, (unsigned int *)tmp, size)) {
		pr_info(KERN_ERR "write_file : error of copy_from_user\n");
		return -EFAULT;
	}

	memcpy(tun_porch_value, buf, size);
	kfree(buf);
	return 0;
}

static int read_initset(unsigned long tmp)
{
	struct tuning_init *rbuf = (struct tuning_init *)tmp;

	pr_info("read_init_file\n");

	if (!tun_len[0]) {
		pr_err("read_init_file:  No cmds ready");
		return -EFAULT;
	}
	if (copy_to_user(rbuf->buf, init_buf, TUNING_REGSIZE)) {
		pr_err("read_file : error of copy_to_user_buff\n");
		return -EFAULT;
	}
	if (copy_to_user(rbuf->size, tun_len, sizeof(tun_len))) {
		pr_err("read_file : error of copy_to_user_buffsize\n");
		return -EFAULT;
	}

	if (put_user(num_cmds, &(rbuf->numcmds))) {
		pr_err("read_file : error of copy_to_user_cmds_num\n");
		return -EFAULT;
	}

	return 0;
}

static int write_initset(unsigned long tmp)
{
	struct tuning_init *wbuf = (struct tuning_init *)tmp;
	int i = 0;
#if TUNING_DBG
	int j = 0, k = 0;
#endif
	int data_offset;

	pr_info("write_init_file\n");

	memset(init_buf, 0x00, TUNING_REGSIZE);

	if (copy_from_user(init_buf, wbuf->buf, sizeof(wbuf->buf))) {
		pr_err("write_file : error of copy_from_user\n");
		return -EFAULT;
	}

	if (get_user(num_cmds, &(wbuf->numcmds))) {
		pr_err("write_file : error of copy_to_user_buffsize\n");
		return -EFAULT;
	}

	pr_info("%s: num_cmds: %d\n", __func__, num_cmds);
	tun_dsi_panel_on_cmds =
		kzalloc((num_cmds * sizeof(struct dsi_cmd_desc)), GFP_KERNEL);
	if (!tun_dsi_panel_on_cmds) {
		pr_err("%s : error of memory allocation\n", __func__);
		return -ENOMEM;
	}

	data_offset = 0;

#if TUNING_DBG
	pr_info("========================\n");
	while (i < sizeof(wbuf->buf)) {
		pr_info("%x ", init_buf[i]);
		j++;
		if (j == wbuf->size[k]) {
			pr_info("\n");
			j = 0;
			k++;
		}
		if (k == wbuf->numcmds)
			break;
		i++;
	}
	pr_info("========================\n");
#endif

	for (i = 0; i < num_cmds; i++) {
		tun_dsi_panel_on_cmds[i].dchdr.dtype = init_buf[data_offset++];
		tun_dsi_panel_on_cmds[i].dchdr.last = init_buf[data_offset++];
		tun_dsi_panel_on_cmds[i].dchdr.vc = init_buf[data_offset++];
		tun_dsi_panel_on_cmds[i].dchdr.ack = init_buf[data_offset++];
		tun_dsi_panel_on_cmds[i].dchdr.wait = init_buf[data_offset++];
		data_offset++;
		tun_dsi_panel_on_cmds[i].dchdr.dlen = init_buf[data_offset++];
		tun_dsi_panel_on_cmds[i].payload = &init_buf[data_offset];
		data_offset += (int)(tun_dsi_panel_on_cmds[i].dchdr.dlen);
	}

	memcpy(tun_len, wbuf->size, sizeof(wbuf->size));

#if TUNING_DBG
	for (i = 0; i < num_cmds; i++) {
		pr_info("%dth data <%x, %x, %x, %x, %x, %d, %x...>\n",
				i+1,
				tun_dsi_panel_on_cmds[i].dchdr.dtype,
				tun_dsi_panel_on_cmds[i].dchdr.last,
				tun_dsi_panel_on_cmds[i].dchdr.vc,
				tun_dsi_panel_on_cmds[i].dchdr.ack,
				tun_dsi_panel_on_cmds[i].dchdr.wait,
				tun_dsi_panel_on_cmds[i].dchdr.dlen,
				*(tun_dsi_panel_on_cmds[i].payload));
	}
#endif
	return 0;
}
long device_ioctl(struct file *file, unsigned int ioctl_num,
		unsigned long ioctl_param)
{

	switch (ioctl_num) {

	case IOCTL_READ_INIT:
		read_initset(ioctl_param);
		break;
	case IOCTL_WRITE_INIT:
		write_initset(ioctl_param);
		break;
	case IOCTL_READ_PORCH:
		read_porch(ioctl_param);
		break;
	case IOCTL_WRITE_PORCH:
		write_porch(ioctl_param);
		break;
	}
	return 0;
}

static const struct file_operations lcd_misc_fops = {
	.owner	= THIS_MODULE,
	.unlocked_ioctl = device_ioctl
};

struct miscdevice lcd_misc_dev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "lcd_misc",
	.fops	= &lcd_misc_fops
};

static int lcd_misc_probe(struct platform_device *pdev)
{
	return misc_register(&lcd_misc_dev);
}
static struct platform_driver this_driver = {
	.probe  = lcd_misc_probe,
	.driver = {
		.name   = "lcd_misc_msm",
	},
};

int __init lcd_misc_init(void)
{
	pr_debug("lcd_misc_init \n");
	return platform_driver_register(&this_driver);
}

device_initcall(lcd_misc_init);

MODULE_DESCRIPTION("MSM MISC driver");
MODULE_LICENSE("GPL v2");
