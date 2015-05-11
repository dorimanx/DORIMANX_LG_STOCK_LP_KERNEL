/*
 *
 * Copyright (C) 2010 NXP Semiconductors
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

#include <linux/nfc/pn547_lge.h>
#include <linux/nfc/pn547_lge_hwadapter.h>
#include <linux/of_gpio.h>

#include <linux/wakelock.h>
#include <linux/async.h>

#define NFC_POWER_OFF   false
#define NFC_POWER_ON    true

#define MAX_BUFFER_SIZE 512
#define NFC_TIMEOUT_MS 2000

#ifdef CONFIG_LGE_NFC_USE_PMIC // [NFC-367]
#define CLK_DISABLE 0
#define CLK_PIN 1
#define CLK_CONT 2
#endif

static bool sIsWakeLocked = false;

#ifdef CONFIG_LGE_NFC_SET_IRQ_WAKEUP
static bool sIrqState = false;
#endif

static bool sPowerState = NFC_POWER_OFF;

static struct i2c_client *pn547_client;

struct wake_lock nfc_wake_lock;

#ifdef CONFIG_LGE_NFC_USE_PMIC
static void pn547_change_clk(struct pn547_dev *pn547_dev, unsigned int clk_state)
{
    static unsigned int nOldClkState = CLK_DISABLE;
    int ret = 0;

    if (nOldClkState == clk_state) {
        pr_err("%s: Desired clock state(%d) is same as previous state(%d)! Skip!\n", __func__, clk_state, nOldClkState);
    }
    else {
        switch (clk_state) {
            case CLK_DISABLE:
                if (nOldClkState == CLK_PIN) {
                    if (pn547_dev->clk_pin != NULL) {
                        clk_disable_unprepare(pn547_dev->clk_pin);
                        nOldClkState = CLK_DISABLE;
                        //pr_err("%s: PMIC Clock is Disabled\n", __func__); // for debug
                    }
                    else {
                        pr_err("%s: PN547 could not get clock!\n", __func__);
                    }
                }
                else if (nOldClkState == CLK_CONT) {
                    if (pn547_dev->clk_cont != NULL) {
                        clk_disable_unprepare(pn547_dev->clk_cont);
                        nOldClkState = CLK_DISABLE;
                        //pr_err("%s: PMIC Clock is Disabled\n", __func__); // for debug
                    }
                    else {
                        pr_err("%s: PN547 could not get clock!\n", __func__);
                    }
                }
                break;
            case CLK_PIN:
                if (pn547_dev->clk_pin != NULL) {
                    ret = clk_prepare_enable(pn547_dev->clk_pin);
                    if (ret) {
                        pr_err("%s: PN547 could not enable clock (%d)\n", __func__, ret);
                        clk_disable_unprepare(pn547_dev->clk_pin);
                        nOldClkState = CLK_DISABLE;
                    }
                    nOldClkState = CLK_PIN;
                    //pr_err("%s: PMIC Clock source is CXO_D1_PIN!\n", __func__); // for debug
                }
                else {
                    pr_err("%s: PN547 could not get pin clock!\n", __func__);
                }
                break;
            case CLK_CONT:
                if (pn547_dev->clk_cont != NULL) {
                    ret = clk_prepare_enable(pn547_dev->clk_cont);
                    if (ret) {
                        pr_err("%s: PN547 could not enable clock (%d)\n", __func__, ret);
                        clk_disable_unprepare(pn547_dev->clk_cont);
                        nOldClkState = CLK_DISABLE;
                    }
                    nOldClkState = CLK_CONT;
                    //pr_err("%s: PMIC Clock source is CXO_D1!\n", __func__); // for debug
                }
                else {
                    pr_err("%s: PN547 could not get cont. clock!\n", __func__);
                }
                break;
            default:
                pr_err("%s: Undefined Clock Setting!\n", __func__);
                break;
        }
    }
}
#endif

static void pn547_disable_irq(struct pn547_dev *pn547_dev)
{
    unsigned long flags;

    spin_lock_irqsave(&pn547_dev->irq_enabled_lock, flags);
    if (pn547_dev->irq_enabled) {
        disable_irq_nosync(pn547_get_irq_pin(pn547_dev));
        disable_irq_wake(pn547_get_irq_pin(pn547_dev));
        pn547_dev->irq_enabled = false;
    }
    spin_unlock_irqrestore(&pn547_dev->irq_enabled_lock, flags);
}

static void pn547_enable_irq(struct pn547_dev *pn547_dev)
{
    unsigned long flags;
    spin_lock_irqsave(&pn547_dev->irq_enabled_lock, flags);
    if (!pn547_dev->irq_enabled) {
        pn547_dev->irq_enabled = true;
        enable_irq(pn547_dev->client->irq);
        enable_irq_wake(pn547_dev->client->irq);
    }
    spin_unlock_irqrestore(&pn547_dev->irq_enabled_lock, flags);
}

static irqreturn_t pn547_dev_irq_handler(int irq, void *dev_id)
{
    struct pn547_dev *pn547_dev = dev_id;
    unsigned long flags;
    unsigned int irq_gpio_val;

    irq_gpio_val = gpio_get_value(pn547_dev->irq_gpio);

    if (irq_gpio_val == 0) {
        pr_err("%s: False Interrupt!\n", __func__);
        return IRQ_HANDLED;
    }

    if (sPowerState == NFC_POWER_ON) {
        spin_lock_irqsave(&pn547_dev->irq_enabled_lock, flags);
        /* Wake up waiting readers */
        wake_up(&pn547_dev->read_wq);
        if (sIsWakeLocked == false) {
            wake_lock(&nfc_wake_lock);
            sIsWakeLocked = true;
        }
        else {
            //pr_err("%s already wake locked!\n", __func__); // for debug
        }
        spin_unlock_irqrestore(&pn547_dev->irq_enabled_lock, flags);
        //pr_err("%s: wake_lock (%d)\n", __func__, gpio_get_value(pn547_dev->irq_gpio)); // for debug
    }
    else {
         pr_err("%s, NFC IRQ Triggered during NFC OFF\n", __func__);
    }

    return IRQ_HANDLED;
}

static ssize_t pn547_dev_read(struct file *filp, char __user *buf,
        size_t count, loff_t *offset)
{
    struct pn547_dev *pn547_dev = filp->private_data;
    static char tmp[MAX_BUFFER_SIZE];
    int ret;
    static bool isFinalPacket = true;
    unsigned long flags;

    if (count > MAX_BUFFER_SIZE)
        count = MAX_BUFFER_SIZE;

    //pr_err("%s : reading %zu bytes.\n", __func__, count); // for debug

wait:
    if (isFinalPacket == true) {
        ret = wait_event_interruptible_timeout(pn547_dev->read_wq, gpio_get_value(pn547_dev->irq_gpio), msecs_to_jiffies(NFC_TIMEOUT_MS));
        if (ret == 0) {
            pr_err("%s: pass wait_event_interruptible by %dms timeout. restart waiting!\n", __func__, NFC_TIMEOUT_MS); // for debug
            spin_lock_irqsave(&pn547_dev->irq_enabled_lock, flags);
            if (sIsWakeLocked == true) {
                wake_unlock(&nfc_wake_lock);
                sIsWakeLocked = false;
            }
            spin_unlock_irqrestore(&pn547_dev->irq_enabled_lock, flags);
            //pr_err("%s: wake_unlock\n", __func__); // for debug
            isFinalPacket = false;
            goto wait;
        }
    }
    else {
        ret = wait_event_interruptible(pn547_dev->read_wq, gpio_get_value(pn547_dev->irq_gpio));
        isFinalPacket = true;
    }

    if (ret == -ERESTARTSYS) {
        //pr_err("%s: pass wait_event_interruptible by signal. Skip!\n", __func__); // for debug
        return -0xFF;
    }
    else {
        //pr_err("%s: pass wait_event_interruptible by condition (%d)\n", __func__, gpio_get_value(pn547_dev->irq_gpio)); // for debug
    }

    /* Read data */
    mutex_lock(&pn547_dev->read_mutex);
    memset(tmp, 0x00, MAX_BUFFER_SIZE);
    ret = i2c_master_recv(pn547_dev->client, tmp, count);
    mutex_unlock(&pn547_dev->read_mutex);

    if (count == 0) {
        pr_err("%s: reading 0 bytes! skip! (%d)\n", __func__, ret);
        return ret;
    }

    if (ret < 0) {
        pr_err("%s: i2c_master_recv returned %d\n", __func__, ret);
        return ret;
    }
    if (ret > count) {
        pr_err("%s: received too many bytes from i2c (%d)\n",
            __func__, ret);
        return -EIO;
    }
    if (copy_to_user(buf, tmp, ret)) {
        pr_warning("%s : failed to copy to user space\n", __func__);
        return -EFAULT;
    }

    //pr_err("%s: i2c_master_recv success (%d)\n", __func__, ret); // for debug
    return ret;
}

static ssize_t pn547_dev_write(struct file *filp, const char __user *buf,
        size_t count, loff_t *offset)
{
    struct pn547_dev  *pn547_dev;
    static char tmp[MAX_BUFFER_SIZE];
    int ret;

    pn547_dev = filp->private_data;

    if (count > MAX_BUFFER_SIZE)
        count = MAX_BUFFER_SIZE;

    memset(tmp, 0x00, MAX_BUFFER_SIZE);
    if (copy_from_user(tmp, buf, count)) {
        pr_err(PN547_DRV_NAME ":%s : failed to copy from user space\n", __func__);
        return -EFAULT;
    }

    //pr_debug("%s : writing %zu bytes.\n", __func__, count);
    /* Write data */
    //dprintk(PN547_DRV_NAME ":write: pn547_write len=:%d\n", count);

    mutex_lock(&pn547_dev->read_mutex);
    ret = i2c_master_send(pn547_dev->client, tmp, count);
    mutex_unlock(&pn547_dev->read_mutex);

    if (ret != count) {
        pr_err("%s : i2c_master_send returned %d\n", __func__, ret);
        ret = -EIO;
    }

    return ret;
}

static int pn547_dev_open(struct inode *inode, struct file *filp)
{
    struct pn547_dev *pn547_dev = i2c_get_clientdata(pn547_client);
    filp->private_data = pn547_dev;
    pn547_enable_irq(pn547_dev);
    pr_debug("%s : %d,%d\n", __func__, imajor(inode), iminor(inode));

    return 0;
}

static long pn547_dev_unlocked_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct pn547_dev *pn547_dev = filp->private_data;
    unsigned long flags;

    switch (cmd) {
    case pn547_SET_PWR:
        if (arg == 2) {
            /*
            power on with firmware download (requires hw reset)
            */
            dprintk(PN547_DRV_NAME ":%s power on with firmware\n", __func__);

            gpio_set_value(pn547_dev->ven_gpio, 1);
            gpio_set_value(pn547_dev->firm_gpio, 1);
            msleep(10);
            gpio_set_value(pn547_dev->ven_gpio, 0);
            msleep(10);
            gpio_set_value(pn547_dev->ven_gpio, 1);
            msleep(10);
        } else if (arg == 1) {
            /* power on */
            dprintk(PN547_DRV_NAME ":%s power on\n", __func__);
            if (sPowerState == NFC_POWER_OFF) {
#ifdef CONFIG_LGE_NFC_USE_PMIC
                pn547_change_clk(pn547_dev, CLK_PIN);
#endif
                gpio_set_value(pn547_dev->firm_gpio, 0);
                gpio_set_value(pn547_dev->ven_gpio, 1);
                msleep(10);

                spin_lock_irqsave(&pn547_dev->irq_enabled_lock, flags);
#ifdef CONFIG_LGE_NFC_SET_IRQ_WAKEUP
                if (sIrqState == false) {
                    irq_set_irq_wake(pn547_dev->client->irq,1);
                    sIrqState = true;
                    dprintk(PN547_DRV_NAME ":%s enable IRQ\n", __func__);
                }
                else {
                    pr_err("%s IRQ is already enabled!\n", __func__);
                }
#endif
                //pr_err("%s NFC_POWER_ON\n", __func__); // for debug
                sPowerState = NFC_POWER_ON;
                spin_unlock_irqrestore(&pn547_dev->irq_enabled_lock, flags);
            }
            else {
                pr_err("%s NFC is alread On!\n", __func__);
            }
        } else  if (arg == 0) {
            /* power off */
            dprintk(PN547_DRV_NAME ":%s power off\n", __func__);
            if (sPowerState == NFC_POWER_ON) {
#ifdef CONFIG_LGE_NFC_USE_PMIC
                pn547_change_clk(pn547_dev, CLK_DISABLE);
#endif
                gpio_set_value(pn547_dev->firm_gpio, 0);
                gpio_set_value(pn547_dev->ven_gpio, 0);
                msleep(10);

                spin_lock_irqsave(&pn547_dev->irq_enabled_lock, flags);
#ifdef CONFIG_LGE_NFC_SET_IRQ_WAKEUP
                if (sIrqState == true) {
                    irq_set_irq_wake(pn547_dev->client->irq,0);
                    sIrqState = false;
                    dprintk(PN547_DRV_NAME ":%s disable IRQ\n", __func__);
                }
                else {
                    pr_err("%s IRQ is already disabled!\n", __func__);
                }
#endif
                if (sIsWakeLocked == true) {
                    pr_err("%s: Release Wake_Lock\n", __func__);
                    wake_unlock(&nfc_wake_lock);
                    sIsWakeLocked = false;
                }
                //pr_err("%s NFC_POWER_OFF\n", __func__); // for debug
                sPowerState = NFC_POWER_OFF;
                spin_unlock_irqrestore(&pn547_dev->irq_enabled_lock, flags);
            }
            else {
                pr_err("%s NFC is alread Off!\n", __func__);
            }
        } else {
                pr_err("%s bad arg %ld\n", __func__, arg);
            return -EINVAL;
        }
        break;
    case pn547_HW_REVISION:
        {
            return pn547_get_hw_revision();
        }
    default:
        pr_err("%s bad ioctl %d\n", __func__, cmd);
        return -EINVAL;
    }

    return 0;
}

static const struct file_operations pn547_dev_fops = {
    .owner  = THIS_MODULE,
    .llseek = no_llseek,
    .read   = pn547_dev_read,
    .write  = pn547_dev_write,
    .open   = pn547_dev_open,
    .unlocked_ioctl = pn547_dev_unlocked_ioctl,
};

static int pn547_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
    int ret;
    struct pn547_dev *pn547_dev = NULL;
    pn547_client = client;

    dprintk(PN547_DRV_NAME ": pn547_probe() start\n");

    pn547_dev = kzalloc(sizeof(*pn547_dev), GFP_KERNEL);
    if (pn547_dev == NULL) {
        dev_err(&client->dev,
                "failed to allocate memory for module data\n");
        ret = -ENOMEM;
        goto err_exit;
    }

    pn547_parse_dt(&client->dev, pn547_dev);

    pn547_dev->client   = client;
    dprintk(PN547_DRV_NAME ":IRQ : %d\nVEN : %d\nFIRM : %d\n",
            pn547_dev->irq_gpio, pn547_dev->ven_gpio, pn547_dev->firm_gpio);

    ret = gpio_request(pn547_dev->irq_gpio, "nfc_int");
    if (ret) {
        dprintk(PN547_DRV_NAME ":pn547_probe() : nfc_int request failed!\n");
        goto err_int;
    }
    ret = gpio_request(pn547_dev->ven_gpio, "nfc_ven");
    if (ret) {
        dprintk(PN547_DRV_NAME ":pn547_probe() : nfc_ven request failed!\n");
        goto err_ven;
    }
    ret = gpio_request(pn547_dev->firm_gpio, "nfc_firm");
    if (ret) {
        dprintk(PN547_DRV_NAME ":pn547_probe() : nfc_firm request failed!\n");
        goto err_firm;
    }

    pn547_gpio_enable(pn547_dev);

    ret = gpio_direction_output(pn547_dev->ven_gpio,1);
    ret = gpio_direction_output(pn547_dev->firm_gpio,0);
    ret = gpio_direction_input(pn547_dev->irq_gpio);

#ifdef CONFIG_LGE_NFC_USE_PMIC
    pn547_get_clk_source(pn547_dev);
    pn547_change_clk(pn547_dev, CLK_PIN);
#endif
    /* init mutex and queues */
    init_waitqueue_head(&pn547_dev->read_wq);
    mutex_init(&pn547_dev->read_mutex);
    spin_lock_init(&pn547_dev->irq_enabled_lock);

    pn547_dev->pn547_device.minor = MISC_DYNAMIC_MINOR;
    pn547_dev->pn547_device.name = PN547_DRV_NAME;
    pn547_dev->pn547_device.fops = &pn547_dev_fops;

    ret = misc_register(&pn547_dev->pn547_device);
    if (ret) {
        pr_err("%s : misc_register failed\n", __FILE__);
        goto err_misc_register;
    }

    wake_lock_init(&nfc_wake_lock, WAKE_LOCK_SUSPEND, "NFCWAKE");

    /* request irq.  the irq is set whenever the chip has data available
     * for reading.  it is cleared when all data has been read.
     */
    pr_info("%s : requesting IRQ %d\n", __func__, client->irq);
    pn547_dev->irq_enabled = true;
    ret = request_irq(pn547_gpio_to_irq(pn547_dev), pn547_dev_irq_handler,
              IRQF_TRIGGER_RISING|IRQF_NO_SUSPEND, client->name, pn547_dev);
    if (ret) {
        dev_err(&client->dev, "request_irq failed\n");
        goto err_request_irq_failed;
    }
    enable_irq_wake(pn547_get_irq_pin(pn547_dev));
    pn547_disable_irq(pn547_dev);
    i2c_set_clientdata(client, pn547_dev);
    dprintk(PN547_DRV_NAME ": pn547_probe() end\n");

    return 0;

err_request_irq_failed:
    misc_deregister(&pn547_dev->pn547_device);

err_misc_register:
    mutex_destroy(&pn547_dev->read_mutex);
    gpio_free(pn547_dev->firm_gpio);

err_firm:
    gpio_free(pn547_dev->ven_gpio);

err_ven:
    gpio_free(pn547_dev->irq_gpio);

err_int:
    kfree(pn547_dev);

err_exit:
    pr_err(PN547_DRV_NAME ": pn547_dev is null\n");
    pr_err(PN547_DRV_NAME ": pn547_probe() end with error!\n");

    return ret;
}

static __devexit int pn547_remove(struct i2c_client *client)
{
    struct pn547_dev *pn547_dev;

    pn547_dev = i2c_get_clientdata(client);
    free_irq(pn547_gpio_to_irq(pn547_dev), pn547_dev);
    misc_deregister(&pn547_dev->pn547_device);
    mutex_destroy(&pn547_dev->read_mutex);
#ifdef CONFIG_LGE_NFC_USE_PMIC
    pn547_change_clk(pn547_dev, CLK_DISABLE);
#endif
    gpio_free(pn547_dev->firm_gpio);
    gpio_free(pn547_dev->ven_gpio);
    gpio_free(pn547_dev->irq_gpio);
    kfree(pn547_dev);

    return 0;
}

static void pn547_shutdown(struct i2c_client *client)
{
    struct pn547_dev *pn547_dev;
    // Get PN547 Device Structure data
    pn547_dev = i2c_get_clientdata(client);

    pn547_shutdown_cb(pn547_dev);
    return;
}

static const struct i2c_device_id pn547_id[] = {
    { PN547_DRV_NAME, 0 },
    { }
};

static struct of_device_id pn547_match_table[] = {
    { .compatible = "nxp,pn547",},
    { },
};

static struct i2c_driver pn547_driver = {
    .driver = {
        .owner = THIS_MODULE,
        .name = PN547_DRV_NAME,
        .of_match_table = pn547_match_table,
    },
    .probe = pn547_probe,
    .remove = __devexit_p(pn547_remove),
    .shutdown   = pn547_shutdown,
    .id_table = pn547_id,
};

/*
 * module load/unload record keeping
 */

static void async_dev_init(void *data, async_cookie_t cookie)
{
    int ret = 0;
    pr_info(PN547_DRV_NAME ": Start async init\n");

    ret = i2c_add_driver(&pn547_driver);
    if (ret < 0) {
        pr_err("[NFC]failed to i2c_add_driver\n");
    }
    pr_info(PN547_DRV_NAME ": Loading PN547 driver Success! \n");
    return;
}

static int __init pn547_dev_init(void)
{
    pr_info("Loading PN547 driver\n");
    async_schedule(async_dev_init, NULL);

    return 0;
}

module_init(pn547_dev_init);

static void __exit pn547_dev_exit(void)
{
    pr_info("Unloading PN547 driver\n");
    i2c_del_driver(&pn547_driver);
}
module_exit(pn547_dev_exit);

MODULE_DEVICE_TABLE(i2c, pn547_id);
MODULE_AUTHOR("Sylvain Fonteneau");
MODULE_DESCRIPTION("NFC PN547 driver");
MODULE_LICENSE("GPL");
