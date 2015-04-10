#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h> 

#include <linux/err.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>
#include <linux/pm_qos.h>

#include "broadcast_dmb_typedef.h"
#include "broadcast_dmb_drv_ifdef.h"
#include "broadcast_fc8300.h"
#include "fci_types.h"
#include "fci_oal.h"
#include "fc8300_drv_api.h"

/*#define _NOT_USE_WAKE_LOCK_*/

#define FEATURE_DMB_USE_XO
//#define FEATURE_DMB_USE_PINCTRL
#define FEATURE_DMB_USE_REGULATOR

#ifdef FEATURE_DMB_USE_PINCTRL
#include <linux/pinctrl/consumer.h>
#endif 

#ifdef FEATURE_DMB_USE_REGULATOR
#include <linux/regulator/consumer.h>
#endif

struct broadcast_fc8300_ctrl_data
{
	int                     pwr_state;
	struct wake_lock        wake_lock;
	struct spi_device*      spi_dev;
	struct i2c_client*      pclient;
    struct platform_device* pdev;
    uint32                  enable_gpio;
    uint32                  reset_gpio;
    /* Interrupt pin is not used in TSIF mode */
    uint32                  interrupt_gpio;
#ifdef FEATURE_DMB_USE_XO
    struct clk*             xo_clk;
#endif
#ifdef FEATURE_DMB_USE_REGULATOR
    struct regulator*       vdd_io;
#endif
};

static struct broadcast_fc8300_ctrl_data  IsdbCtrlInfo;

struct i2c_client*	FCI_GET_I2C_DRIVER(void)
{
	return IsdbCtrlInfo.pclient;
}

static Device_drv device_fc8300 = {
    &broadcast_fc8300_drv_if_power_on,
    &broadcast_fc8300_drv_if_power_off,
    &broadcast_fc8300_drv_if_open,
    &broadcast_fc8300_drv_if_close,
    &broadcast_fc8300_drv_if_set_channel,
    &broadcast_fc8300_drv_if_resync,
    &broadcast_fc8300_drv_if_detect_sync,
    &broadcast_fc8300_drv_if_get_sig_info,
    &broadcast_fc8300_drv_if_get_ch_info,
    &broadcast_fc8300_drv_if_get_dmb_data,
    &broadcast_fc8300_drv_if_reset_ch,
    &broadcast_fc8300_drv_if_user_stop,
    &broadcast_fc8300_drv_if_select_antenna,
    &broadcast_fc8300_drv_if_read_control,
    &broadcast_fc8300_drv_if_get_mode,
};

#ifdef FEATURE_DMB_USE_PINCTRL
static int isdbt_pinctrl_init(void)
{
	struct pinctrl *tdmb_pinctrl;
	struct pinctrl_state *gpio_state_suspend;

	tdmb_pinctrl = devm_pinctrl_get(&(IsdbCtrlInfo.pdev->dev));


	if(IS_ERR_OR_NULL(tdmb_pinctrl)) {
		pr_err("%s: Getting pinctrl handle failed\n", __func__);
		return -EINVAL;
	}
	gpio_state_suspend
	 = pinctrl_lookup_state(tdmb_pinctrl, "isdbt_gpio_suspend");

	 if(IS_ERR_OR_NULL(gpio_state_suspend)) {
	 	pr_err("%s: [dtv]Failed to get the suspend state pinctrl handle\n", __func__);
	 	return -EINVAL;
	}

	if(pinctrl_select_state(tdmb_pinctrl, gpio_state_suspend)) {
		pr_err("%s: [dtv]error on pinctrl_select_state DTV GPIOs\n", __func__);
		return -EINVAL;
	}
	else {
		printk("%s: success to set pinctrl_select_state for DTV GPIOss\n", __func__);
	}

	return 0;
}
#endif

#ifdef FEATURE_DMB_USE_REGULATOR
static int broadcast_isdbt_set_regulator(int onoff)
{
    int rc = -1;

    if(!IsdbCtrlInfo.vdd_io)
    {
        IsdbCtrlInfo.vdd_io = devm_regulator_get(&(IsdbCtrlInfo.pdev->dev),"isdbt_vdd_io");
        if(IS_ERR(IsdbCtrlInfo.vdd_io)){
            dev_err(&(IsdbCtrlInfo.pdev->dev), "[dtv]could not get regulator %s \n","isdbt_vdd_io-supply");
            return rc;
        }
    }

    if(onoff)
    {
        rc = regulator_set_voltage(IsdbCtrlInfo.vdd_io,2800000,2800000);
        if(rc)
        {
            dev_err(&(IsdbCtrlInfo.pdev->dev), "[dtv] could not set regulator ret=%d \n",rc);
            return rc;
        }

        rc = regulator_enable(IsdbCtrlInfo.vdd_io);
        if(rc)
        {
            dev_err(&(IsdbCtrlInfo.pdev->dev), "[dtv]could not enable regulator ret=%d\n",rc);
            return rc;
        }
    }
    else
    {
        rc = regulator_disable(IsdbCtrlInfo.vdd_io);
        if(rc)
        {
            dev_err(&(IsdbCtrlInfo.pdev->dev), "[dtv]could not disable regulator %d \n",rc);
            return rc;
        }
    }

    printk("[dtv] %s: success to set pm8994 voltage control(mode:%d)\n", __func__,onoff);
    return rc;
}
#endif

int fc8300_power_on(void)
{
    int rc = OK;

	if(IsdbCtrlInfo.pwr_state != 1)
	{
#ifndef _NOT_USE_WAKE_LOCK_
        wake_lock(&IsdbCtrlInfo.wake_lock);
#endif

        gpio_set_value(IsdbCtrlInfo.reset_gpio, 0);
        gpio_set_value(IsdbCtrlInfo.enable_gpio, 1);
        mdelay(3);
        gpio_set_value(IsdbCtrlInfo.reset_gpio, 1);
        mdelay(2);

#ifdef FEATURE_DMB_USE_REGULATOR
        broadcast_isdbt_set_regulator(1);
#endif

#ifdef FEATURE_DMB_USE_XO
        if(IsdbCtrlInfo.xo_clk!= NULL) {
            printk("[dtv]fc8300_power on clk enable\n");
            rc = clk_prepare_enable(IsdbCtrlInfo.xo_clk);
        }
        mdelay(2);
#endif
	}
	else
	{
		printk("[dtv]aready on!! \n");
	}

	IsdbCtrlInfo.pwr_state = 1;
	return rc;
}

int fc8300_is_power_on()
{
	return (int)IsdbCtrlInfo.pwr_state;
}


int fc8300_power_off(void)
{
	if(IsdbCtrlInfo.pwr_state == 0)
	{
		print_log(NULL, "Isdb_tcc3530_power is immediately off\n");
		return OK;
	}
	else
	{
#ifdef FEATURE_DMB_USE_XO
        if(IsdbCtrlInfo.xo_clk != NULL)
        {
            clk_disable_unprepare(IsdbCtrlInfo.xo_clk);
        }
#endif

#ifdef FEATURE_DMB_USE_REGULATOR
        broadcast_isdbt_set_regulator(0);
#endif

		printk("[dtv]Isdb_fc8300_power_off\n");
        gpio_set_value(IsdbCtrlInfo.enable_gpio, 0);
        mdelay(1);
        gpio_set_value(IsdbCtrlInfo.reset_gpio, 0);
        mdelay(5);
	}

#ifndef _NOT_USE_WAKE_LOCK_
	wake_unlock(&IsdbCtrlInfo.wake_lock);
#endif
	IsdbCtrlInfo.pwr_state = 0;

	return OK;
}

static int broadcast_Isdb_config_gpios(void)
{
    int rc = OK;
    int err_count = 0;

    IsdbCtrlInfo.enable_gpio = of_get_named_gpio(IsdbCtrlInfo.pdev->dev.of_node,
        "isdbt-fc8300,en-gpio" ,0);
    IsdbCtrlInfo.reset_gpio = of_get_named_gpio(IsdbCtrlInfo.pdev->dev.of_node,
        "isdbt-fc8300,reset-gpio" ,0);

    printk("[dtv]enable_gpio =(%d), reset_gpio =(%d)",
            IsdbCtrlInfo.enable_gpio, IsdbCtrlInfo.reset_gpio);

    rc =  gpio_request(IsdbCtrlInfo.enable_gpio, "DMB_EN");
    if(rc < 0) {
        err_count++;
        printk("[dtv]Failed DMB_EN GPIO request\n");
    }
    udelay(10);
    gpio_direction_output(IsdbCtrlInfo.enable_gpio, 0);

    rc =  gpio_request(IsdbCtrlInfo.reset_gpio, "DMB_RESET");
    if(rc < 0) {
        err_count++;
        printk("[dtv]Failed DMB_RESET GPIO request\n");
    }
    udelay(10);
    gpio_direction_output(IsdbCtrlInfo.reset_gpio, 0);

    return rc;
}

static int broadcast_Isdb_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int rc = 0;
	int addr = 0;

#if defined (CONFIG_ARCH_MSM8994)
    printk("[dtv]broadcast_Isdb_i2c_probe client:0x%lX\n", (UDynamic_32_64)client);
#else
    printk("[dtv]broadcast_Isdb_i2c_probe client:0x%X\n", (UDynamic_32_64)client);
#endif

	if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		print_log(NULL, "need I2C_FUNC_I2C\n");
		return -ENODEV;
	}
    IsdbCtrlInfo.pdev = to_platform_device(&client->dev);

	/* taew00k.kang added for Device Tree Structure 2013-06-04 [start] */
	addr = client->addr; //Slave Addr
	pr_err("[dtv] i2c Slaveaddr [%x] \n", addr);

	IsdbCtrlInfo.pclient = client;
	//i2c_set_clientdata(client, (void*)&IsdbCtrlInfo.pclient);

#ifdef FEATURE_DMB_USE_XO
    IsdbCtrlInfo.xo_clk = clk_get(&IsdbCtrlInfo.pclient->dev, "isdbt_xo");
    if(IS_ERR(IsdbCtrlInfo.xo_clk)){
        rc = PTR_ERR(IsdbCtrlInfo.xo_clk);
        dev_err(&IsdbCtrlInfo.pclient->dev, "[dtv]could not get clock\n");
        return rc;
    }
    /* We enable/disable the clock only to assure it works */
    rc = clk_prepare_enable(IsdbCtrlInfo.xo_clk);
    if(rc) {
        dev_err(&IsdbCtrlInfo.pclient->dev, "[dtv] could not enable clock\n");
        return rc;
    }
    clk_disable_unprepare(IsdbCtrlInfo.xo_clk);
#endif

#ifdef FEATURE_DMB_USE_PINCTRL
    isdbt_pinctrl_init();
#endif

    /* Config GPIOs */
    broadcast_Isdb_config_gpios();

#ifdef FEATURE_DMB_USE_REGULATOR
    broadcast_isdbt_set_regulator(1);
    broadcast_isdbt_set_regulator(0);
#endif

#ifndef _NOT_USE_WAKE_LOCK_
	wake_lock_init(&IsdbCtrlInfo.wake_lock, WAKE_LOCK_SUSPEND,
					dev_name(&client->dev));	
#endif

	return rc;
}

static int broadcast_Isdb_i2c_remove(struct i2c_client* client)
{
	int rc = 0;

	print_log(NULL, "[%s]\n", __func__);
#ifndef _NOT_USE_WAKE_LOCK_
	wake_lock_destroy(&IsdbCtrlInfo.wake_lock);
#endif
	memset((unsigned char*)&IsdbCtrlInfo, 0x0, sizeof(struct broadcast_fc8300_ctrl_data));
	//TcpalDeleteSemaphore(&fc8300DrvSem);
	return rc;
}

static int broadcast_Isdb_i2c_suspend(struct i2c_client* client, pm_message_t mesg)
{
	int rc = 0;
	print_log(NULL, "[%s]\n", __func__);
	return rc;
}

static int broadcast_Isdb_i2c_resume(struct i2c_client* client)
{
	int rc = 0;
	print_log(NULL, "[%s]\n", __func__);
	return rc;
}

static const struct i2c_device_id isdbt_fc8300_id[] = {
/* taew00k.kang added for Device Tree Structure 2013-06-04 [start] */
	{"tcc3535_i2c",	0},
/* taew00k.kang added for Device Tree Structure 2013-06-04 [end] */
	{},
};

MODULE_DEVICE_TABLE(i2c, isdbt_fc8300_id);


/* taew00k.kang added for Device Tree Structure 2013-06-04 [start] */
static struct of_device_id tcc3535_i2c_table[] = {
{ .compatible = "telechips,tcc3535-i2c",}, //Compatible node must match dts
{ },
};
/* taew00k.kang added for Device Tree Structure 2013-06-04 [end] */

static struct i2c_driver broadcast_Isdb_driver = {
	.driver = {
		.name = "tcc3535_i2c",
		.owner = THIS_MODULE,
		.of_match_table = tcc3535_i2c_table,
	},
	.probe = broadcast_Isdb_i2c_probe,
	.remove	= broadcast_Isdb_i2c_remove,
	.id_table = isdbt_fc8300_id,
	.suspend = broadcast_Isdb_i2c_suspend,
	.resume  = broadcast_Isdb_i2c_resume,
};

int broadcast_dmb_fc8300_drv_init(void)
{
	int rc;
	print_log(NULL, "[%s]\n", __func__);
	rc = broadcast_dmb_drv_start(&device_fc8300);
	if (rc) 
	{
		print_log(NULL, "failed to load\n");
		return rc;
	}
	print_log(NULL, "[%s add i2c driver]\n", __func__);
	rc = i2c_add_driver(&broadcast_Isdb_driver);
	print_log(NULL, "broadcast_add_driver rc = (%d)\n", rc);
	return rc;
}

static void __exit broadcast_dmb_fc8300_drv_exit(void)
{
	i2c_del_driver(&broadcast_Isdb_driver);
}

module_init(broadcast_dmb_fc8300_drv_init);
module_exit(broadcast_dmb_fc8300_drv_exit);
MODULE_DESCRIPTION("broadcast_dmb_drv_init");
MODULE_LICENSE("FCI");
