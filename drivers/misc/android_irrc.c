/*
 * android vibrator driver
 *
 * Copyright (C) 2009-2012 LGE, Inc.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#if defined(CONFIG_ANDROID_SW_IRRC)
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/err.h>
#include <linux/android_irrc.h>
#include <linux/spinlock.h>
#include "../staging/android/timed_output.h"
#include <linux/types.h>
#include <linux/err.h>
#include <mach/msm_iomap.h>
#include <linux/io.h>
#include <mach/gpiomux.h>
#include <mach/board_lge.h>
#include <linux/i2c.h>
#include <mach/msm_xo.h>
#include <linux/slab.h>

#include <linux/ioctl.h>
#include <asm/ioctls.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>

#include <mach/board_lge.h>
#include <linux/delay.h>

#define GPIO_IRRC_PWEN              69
#define GPIO_IRRC_PWEN_ENABLE       1
#define GPIO_IRRC_PWEN_DISABLE      0
/*
    For ADB debugging
    if you want to turn on with pwm clock(33Khz), duty(50%),    #echo 1 33 50 > /sys/kernel/debug/sw_irrc/poke
    if you want to turn on with gpio level high.                #echo 1 0 0 > /sys/kernel/debug/sw_irrc/poke
    if you want to turn off,                           #echo 0 33 50 > /sys/kernel/debug/sw_irrc/poke

*/

#define REG_WRITEL(value, reg)		writel(value, reg)
#define REG_READL(reg)				readl(reg)

#define MMSS_GP0_CMD_RCGR(x) (void __iomem *)(virt_bases_v + (x))
//#define MMSS_CC_PWM_SET		0xFD8C3450 //B2:0xFD8C3450, Wx:0xFD8C3420. The value will be from device tree. gp_cmd_rcgr
#define MMSS_CC_PWM_SIZE	SZ_1K

struct timed_irrc_data {
    struct platform_device dev;

    struct regulator *vreg;

    unsigned int gp_cmd_rcgr;
    int pwm_gpio;
    int pwm_gpio_func;

    struct clk *gp_clk;
    const char *clk_name;
    unsigned int clk_rate;

    struct workqueue_struct *workqueue;
    struct delayed_work gpio_off_work;
};

struct irrc_compr_params {
    int frequency;
    int duty;
    int length;
};

static void __iomem *virt_bases_v = NULL;
static struct timed_irrc_data *irrc_data_ptr;
static struct platform_device *irrc_dev_ptr;
static int gpio_high_flag = 0;


static struct gpiomux_setting irrc_active = {
    .func = 0, //[WX project] The value will be from device tree. GPIO for GP clock has alternative function.
    .drv = GPIOMUX_DRV_2MA,
    .pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting irrc_suspend = {
    .func = GPIOMUX_FUNC_GPIO,
    .drv = GPIOMUX_DRV_2MA,
    .pull = GPIOMUX_PULL_NONE,
};

static struct msm_gpiomux_config irrc_config[] = {
    {
        .gpio = 0, //[WX project] The value will be from device tree. GPIO_IRRC_PWM gpio number
        .settings = {
            [GPIOMUX_ACTIVE] =    &irrc_active,
            [GPIOMUX_SUSPENDED] = &irrc_suspend,

        },
    },
};

static int android_irrc_set_pwm(int enable,int PWM_CLK, int duty)
{
    int M_VAL = 1;
    int N_VAL = 1;
    int D_VAL = 1;

    N_VAL = (9600+PWM_CLK)/(PWM_CLK*2); //Formular in case SRC is 19.2Mhz. N_VAL = SRC/(div*PWM_CLK) + 0.5
    D_VAL = (N_VAL*duty+50)/100;
    if (D_VAL == 0)
        D_VAL = 1;

    INFO_MSG("enable:%d, pwm_clk:%d, duty:%d, M:%d,N:%d,D:%d\n", enable,PWM_CLK,duty, M_VAL,N_VAL,D_VAL);

    if (enable) {
        REG_WRITEL(
            ((~(N_VAL-M_VAL)) & 0xffU),/* N[7:0] */
            MMSS_GP0_CMD_RCGR(0x0C));
        REG_WRITEL(
            ((~(D_VAL << 1)) & 0xffU),	/* D[7:0] */
            MMSS_GP0_CMD_RCGR(0x10));
        REG_WRITEL(
            (1 << 1U) + /* ROOT_EN[1] */
            (1),		/* UPDATE[0] */
            MMSS_GP0_CMD_RCGR(0));
    } else {
        REG_WRITEL(
            (0 << 1U) + /* ROOT_EN[1] */
            (0),		/* UPDATE[0] */
            MMSS_GP0_CMD_RCGR(0));
    }
    return 0;
}

static void android_irrc_enable_pwm(struct timed_irrc_data *irrc, int PWM_CLK, int duty)
{
    int rc;

    if(lge_get_board_revno() >= HW_REV_B) {
        if( gpio_cansleep(GPIO_IRRC_PWEN) )
            gpio_set_value_cansleep(GPIO_IRRC_PWEN, GPIO_IRRC_PWEN_ENABLE);
        else
            gpio_set_value(GPIO_IRRC_PWEN, GPIO_IRRC_PWEN_ENABLE);
        udelay(30);
    } else if (!(regulator_is_enabled(irrc->vreg) > 0)) {
        rc = regulator_enable(irrc->vreg);
        if (rc < 0)
            ERR_MSG("regulator_enable failed\n");
    }

    cancel_delayed_work_sync(&irrc->gpio_off_work); //android_irrc_disable_pwm
    //clk_disable_unprepare(irrc->gp_clk);
    if((PWM_CLK == 0) || (duty == 100)){
        INFO_MSG("gpio set to high!!!\n");

        gpio_tlmm_config(GPIO_CFG(irrc->pwm_gpio, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
        gpio_set_value(irrc->pwm_gpio, 1);

        gpio_high_flag = 1;

    } else if((PWM_CLK < 23) || (PWM_CLK > 1200) || (duty > 60) || (duty < 20) ){
        INFO_MSG("Out of range ! \n");

    } else {
        INFO_MSG("gpio set to gp!!!\n");

        gpio_tlmm_config(GPIO_CFG(irrc->pwm_gpio, irrc->pwm_gpio_func, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
        clk_prepare_enable(irrc->gp_clk);

        android_irrc_set_pwm(1, PWM_CLK, duty);
        gpio_high_flag = 0;
    }

}

static void android_irrc_disable_pwm(struct work_struct *work)
{
    int rc;
    struct timed_irrc_data *irrc = container_of(work, struct timed_irrc_data, gpio_off_work.work);

    INFO_MSG("bk gpio_high_flag = %d\n", gpio_high_flag);

    if(lge_get_board_revno() >= HW_REV_B) {
        if( gpio_cansleep(GPIO_IRRC_PWEN) )
            gpio_set_value_cansleep(GPIO_IRRC_PWEN, GPIO_IRRC_PWEN_DISABLE);
        else
            gpio_set_value(GPIO_IRRC_PWEN, GPIO_IRRC_PWEN_DISABLE);
        udelay(150);
    } else if (regulator_is_enabled(irrc->vreg) > 0) {
        rc = regulator_disable(irrc->vreg);
        if (rc < 0)
            ERR_MSG("regulator_disable failed\n");
    }

    if(gpio_high_flag == 1){
        gpio_set_value(irrc->pwm_gpio, 0);

    } else {
        //android_irrc_set_pwm(0,38,30); //no need
        do {
            //INFO_MSG("get_clk_count(irrc->gp_clk) = %d\n", get_clk_count(irrc->gp_clk));
            clk_disable_unprepare(irrc->gp_clk);
        } while( get_clk_count(irrc->gp_clk) > 0 );
    }
}

static int android_irrc_open(struct inode *inode, struct file *file)
{
    struct timed_irrc_data *irrc = platform_get_drvdata(irrc_dev_ptr);
    file->private_data = irrc;

    return 0;
}

static int android_irrc_release(struct inode *inode, struct file *file)
{
    return 0;
}

static ssize_t android_irrc_write(struct file *file, const char __user *buf, size_t count, loff_t *pos)
{
    return 0;
}

static long android_irrc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct timed_irrc_data *irrc = file->private_data;
    struct irrc_compr_params test;
    int rc = 0;

    switch (cmd) {
        case IRRC_START:
            rc = copy_from_user(&test, (void __user *)arg, sizeof(test));

            INFO_MSG("IRRC_START: freq:%d, duty:%d\n", test.frequency/1000, test.duty);
            android_irrc_enable_pwm(irrc, test.frequency/1000, test.duty);
            break;

        case IRRC_STOP:
            INFO_MSG("IRRC_STOP\n");
            cancel_delayed_work_sync(&irrc->gpio_off_work); //android_irrc_disable_pwm
            queue_delayed_work(irrc->workqueue, &irrc->gpio_off_work, msecs_to_jiffies(1500));
            break;
        default:
        INFO_MSG("CMD ERROR: cmd:%d\n", cmd);
        rc = -EINVAL;
    }

    return rc;
}

static int android_irrc_pcm_fsync(struct file *file, loff_t a, loff_t b, int datasync)
{
	return 0;
}

static const struct file_operations IRRC_pcm_fops = {
    .owner          = THIS_MODULE,
    .open           = android_irrc_open,
    .release        = android_irrc_release,
    .write          = android_irrc_write,
    .unlocked_ioctl     = android_irrc_ioctl,
    .fsync = android_irrc_pcm_fsync,
};

struct miscdevice irrc_misc = {
    .minor	= MISC_DYNAMIC_MINOR,
    .name	= "msm_IRRC_pcm_dec",
    .fops	= &IRRC_pcm_fops,
};

#ifdef CONFIG_DEBUG_FS
static struct dentry *debugfs_sw_irrc_dir;
static struct dentry *debugfs_poke;

static int codec_debug_open(struct inode *inode, struct file *file)
{
    file->private_data = inode->i_private;
    return 0;
}

static int get_parameters(char *buf, long int *param1, int num_of_par)
{
    char *token;
    int base, cnt = 0;
    token = strsep(&buf, " ");

    while (token != NULL) {
        if ((strlen(token) > 2) && ((token[1] == 'x') || (token[1] == 'X')))
            base = 16;
        else
            base = 10;

        if (strict_strtoul(token, base, &param1[cnt]) != 0){
            ERR_MSG("strict_strtoul error!!\n");
            return -EINVAL;
        }
        cnt ++;
        token = strsep(&buf, " ");
    }
    return 0;
}

static ssize_t codec_debug_write(struct file *filp,
	const char __user *ubuf, size_t cnt, loff_t *ppos)
{
    char *access_str = filp->private_data;
    char lbuf[32];
    int rc;
    long int param[5] = {0,};
    struct timed_irrc_data *irrc = platform_get_drvdata(irrc_dev_ptr);

    if (cnt > sizeof(lbuf) - 1)
        return -EINVAL;

    rc = copy_from_user(lbuf, ubuf, cnt);
    if (rc)
        return -EFAULT;

    lbuf[cnt] = '\0';

    //INFO_MSG("access_str:%s lbuf:%s cnt:%d\n", access_str, lbuf, cnt);

    if (!strncmp(access_str, "poke", 6)) {
        rc = get_parameters(lbuf, param, 3);
        if (rc) {
            ERR_MSG("error!!! get_parameters rc = %d\n", rc);
            return rc;
        }

        switch (param[0]) {
            case 1:
                INFO_MSG("IRRC_START\n");
                android_irrc_enable_pwm(irrc, param[1], param[2]);
                break;

            case 0:
                INFO_MSG("IRRC_STOP\n");
                cancel_delayed_work_sync(&irrc->gpio_off_work);
                queue_delayed_work(irrc->workqueue,&irrc->gpio_off_work, msecs_to_jiffies(1500));
                break;
            default:
                rc = -EINVAL;
        }

    }

    if (rc == 0)
        rc = cnt;
    else
        ERR_MSG("rc = %d\n", rc);

    return rc;
}

static const struct file_operations codec_debug_ops = {
    .open = codec_debug_open,
    .write = codec_debug_write,
};
#endif

#ifdef CONFIG_OF
static void android_irrc_parse_dt(struct device *dev, struct timed_irrc_data *data)
{
    struct device_node *np = dev->of_node;
    int len;

    of_property_read_u32(np, "lge,gp-cmd-rcgr", &data->gp_cmd_rcgr);
    data->pwm_gpio = of_get_named_gpio_flags(np, "lge,pwm-gpio", 0, NULL);
    of_property_read_u32(np, "lge,pwm-gpio-func", &data->pwm_gpio_func);
    data->clk_name = of_get_property(np, "lge,clk-name", &len);
    of_property_read_u32(np, "lge,clk-rate", &data->clk_rate);

    INFO_MSG("rcgr:%x, gpio:%d, gpio-func:%d, clk name: %s, clk rate: %u\n",
        data->gp_cmd_rcgr, data->pwm_gpio, data->pwm_gpio_func, data->clk_name, data->clk_rate);
}

static struct of_device_id irrc_match_table[] = {
    { .compatible = "lge,sw_irrc",},
    { },
};
#endif

static void android_irrc_install(int pwm_gpio, int pwm_gpio_func)
{
    irrc_config[0].gpio = pwm_gpio;
    switch(pwm_gpio_func){
        case 0: irrc_active.func = GPIOMUX_FUNC_GPIO; break;
        case 1: irrc_active.func = GPIOMUX_FUNC_1; break;
        case 2: irrc_active.func = GPIOMUX_FUNC_2; break;
        case 3: irrc_active.func = GPIOMUX_FUNC_3; break;
        case 4: irrc_active.func = GPIOMUX_FUNC_4; break;
        case 5: irrc_active.func = GPIOMUX_FUNC_5; break;
        case 6: irrc_active.func = GPIOMUX_FUNC_6; break;
        case 7: irrc_active.func = GPIOMUX_FUNC_7; break;
        case 8: irrc_active.func = GPIOMUX_FUNC_8; break;
        case 9: irrc_active.func = GPIOMUX_FUNC_9; break;
        default:
            irrc_active.func = GPIOMUX_FUNC_GPIO;
            ERR_MSG("gpiomux function error!\n");
            break;
    }
    msm_gpiomux_install(irrc_config, ARRAY_SIZE(irrc_config));
}

static int android_irrc_probe(struct platform_device *pdev)
{
    int rc;
    struct timed_irrc_data *irrc;

    INFO_MSG("probe\n");

    irrc = kzalloc(sizeof(struct timed_irrc_data), GFP_KERNEL);
    if (irrc == NULL) {
        ERR_MSG("Can not allocate memory.\n");
        goto err_1;
    }

#ifdef CONFIG_OF
    if (pdev->dev.of_node) {
        android_irrc_parse_dt(&pdev->dev, irrc);
    }
#endif
    android_irrc_install(irrc->pwm_gpio, irrc->pwm_gpio_func);

    rc = gpio_request(irrc->pwm_gpio, "irrc_pwm");

    if (rc) {
        ERR_MSG("IRRC GPIO set failed.\n");
        goto err_2;
    }

    virt_bases_v = ioremap(irrc->gp_cmd_rcgr, MMSS_CC_PWM_SIZE);

    rc = misc_register(&irrc_misc);
    if (rc) {
        ERR_MSG("misc_register failed.\n");
        goto err_3;
    }

    irrc->workqueue = create_workqueue("irrc_ts_workqueue");
    if (!irrc->workqueue) {
        ERR_MSG("Unable to create workqueue\n");
        goto err_4;
    }

    INIT_DELAYED_WORK(&irrc->gpio_off_work, android_irrc_disable_pwm);

    irrc->dev.name = "sw_irrc";
    pdev->dev.init_name = irrc->dev.name;
    INFO_MSG("dev->init_name : %s, dev->kobj : %s\n", pdev->dev.init_name, pdev->dev.kobj.name);

    irrc->gp_clk = clk_get(&pdev->dev, irrc->clk_name);
    clk_set_rate(irrc->gp_clk, (unsigned long)irrc->clk_rate);

    if(lge_get_board_revno() >= HW_REV_B) {
#if 0   /* GPIO_IRRC_PWEN(GPIO 69) is shared with IrDA and IrRC in B2 KDDI targets. */
        /* Proving of IrDA driver is invoked earlier than IrRC driver's one, so IrRC doesn't need to init. */
        if((rc = gpio_request_one(GPIO_IRRC_PWEN, GPIOF_OUT_INIT_LOW, "IrRC_PWEN")) != 0) {
            ERR_MSG("failed to gpio_request_one an external LDO(GPIO:%d) for SW IrRC \n", GPIO_IRRC_PWEN);
            goto err_4;
        }
#endif
    } else {
        // for VREG_L19_2V85 on irrc sensor.
        irrc->vreg = regulator_get(&pdev->dev, "vreg_irrc");
        if (IS_ERR(irrc->vreg)) {
            ERR_MSG("regulator_get failed (%ld)\n", PTR_ERR(irrc->vreg));
            irrc->vreg = NULL;
            goto err_4;
        }
    }

    irrc_data_ptr = irrc;
    irrc_dev_ptr = pdev;

    platform_set_drvdata(pdev, irrc);

#ifdef CONFIG_DEBUG_FS
    debugfs_sw_irrc_dir = debugfs_create_dir("sw_irrc", 0);
    if (!IS_ERR(debugfs_sw_irrc_dir)) {
        debugfs_poke = debugfs_create_file("poke", S_IFREG | S_IRUGO, debugfs_sw_irrc_dir, (void *) "poke", &codec_debug_ops);
    }
#endif
    return 0;

err_4:
    misc_deregister(&irrc_misc);
err_3:
    iounmap(virt_bases_v);
    gpio_free(irrc->pwm_gpio);
err_2:
    kfree(irrc);
err_1:
    ERR_MSG("probe error.\n");
    return -ENODEV;
}

static int android_irrc_remove(struct platform_device *pdev)
{
    struct timed_irrc_data *irrc = platform_get_drvdata(pdev);
    platform_set_drvdata(pdev, NULL);

    misc_deregister(&irrc_misc);
    irrc_dev_ptr = NULL;
    iounmap(virt_bases_v);
    gpio_free(irrc->pwm_gpio);
    kfree(irrc);

#ifdef CONFIG_DEBUG_FS
    debugfs_remove(debugfs_poke);
    debugfs_remove(debugfs_sw_irrc_dir);
#endif

    return 0;
}

#ifdef CONFIG_PM
static int android_irrc_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int android_irrc_resume(struct platform_device *pdev)
{
	return 0;
}
#endif

static void android_irrc_shutdown(struct platform_device *pdev)
{
}

static struct platform_driver android_irrc_driver = {
    .probe = android_irrc_probe,
    .remove = android_irrc_remove,
    .shutdown = android_irrc_shutdown,
#ifdef CONFIG_PM
    .suspend = android_irrc_suspend,
    .resume = android_irrc_resume,
#else
    .suspend = NULL,
    .resume = NULL,
#endif
    .driver = {
        .name = "android-irrc",
#ifdef CONFIG_OF
        .of_match_table = irrc_match_table,
#endif
	},
};

static int __init android_irrc_init(void)
{
    INFO_MSG("init\n");
    return platform_driver_register(&android_irrc_driver);
}

static void __exit android_irrc_exit(void)
{
    INFO_MSG("exit\n");
    platform_driver_unregister(&android_irrc_driver);
}

late_initcall_sync(android_irrc_init); /* to let init lately */
module_exit(android_irrc_exit);

MODULE_AUTHOR("LG Electronics Inc.");
MODULE_DESCRIPTION("Android IRRC Driver");
MODULE_LICENSE("GPL");
#endif

