/*
 * Maxim MAX77819 MFD Core
 *
 * Copyright (C) 2013 Maxim Integrated Product
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/* #define DEBUG
#define VERBOSE_DEBUG */
#define log_level  1

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>

/* for Regmap */
#include <linux/regmap.h>

/* for Device Tree */
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/irqdomain.h>

#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/mfd/core.h>
#include <linux/mfd/max77819.h>

#define DRIVER_DESC    "MAX77819 MFD Driver"
#define DRIVER_NAME    MAX77819_NAME
#define DRIVER_VERSION MAX77819_DRIVER_VERSION".1-rc"
#define DRIVER_AUTHOR  "Gyungoh Yoo <jack.yoo@maximintegrated.com>"

enum {
	MAX77819_DEV_PMIC = 0,  /* PMIC (Charger, Flash LED) */
	MAX77819_DEV_PERIPH,    /* WLED, Motor */
	MAX77819_DEV_FUELGAUGE, /* Fuel Gauge */
	/***/
	MAX77819_DEV_NUM_OF_DEVICES,
};

struct max77819_dev;

struct max77819_core {
	struct mutex         lock;
	struct max77819_dev *dev[MAX77819_DEV_NUM_OF_DEVICES];
};

struct max77819_dev {
	struct max77819_core        *core;
	int                          dev_id;
	void                        *pdata;
	struct mutex                 lock;
	struct device               *dev;
	struct max77819_io           io;
	struct kobject              *kobj;
	struct attribute_group      *attr_grp;
	struct regmap_irq_chip_data *regmap_irq_chip_data;
	int                          irq;
	int                          irq_gpio;
	struct irq_domain           *irq_domain;
};

#define __lock(_me)    mutex_lock(&(_me)->lock)
#define __unlock(_me)  mutex_unlock(&(_me)->lock)

static struct max77819_core max77819;

static const struct regmap_config max77819_regmap_config = {
	.reg_bits   = 8,
	.val_bits   = 8,
	.cache_type = REGCACHE_NONE,
};

static int max77819_add_devices (struct max77819_dev *me,
		struct mfd_cell *cells, int n_devs)
{
	struct device *dev = me->dev;
	int rc;

	if (!dev->of_node) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 6, 0)
		rc = mfd_add_devices(dev, -1, cells, n_devs, NULL, 0);
#else /* LINUX_VERSION_CODE ... */
		rc = mfd_add_devices(dev, -1, cells, n_devs, NULL, 0, NULL);
#endif /* LINUX_VERSION_CODE ... */
		goto out;
	}

	rc = of_platform_populate(dev->of_node, NULL, NULL, dev);

out:
	return rc;
}

/*******************************************************************************
 *** MAX77819 PMIC
 ******************************************************************************/

/* Register map */
#define PMICID              0x20
#define PMICREV             0x21
#define INTSRC              0x22
#define INTSRC_MASK         0x23
#define TOPSYS_INT          0x24
/*      RESERVED            0x25 */
#define TOPSYS_INT_MASK     0x26
/*      RESERVED            0x27 */
#define TOPSYS_STAT         0x28
/*      RESERVED            0x29 */
#define MAINCTRL1           0x2A
#define LSCONFIG            0x2B

/* Interrupt corresponding bit */
#define CHGR_INT            BIT (0)
#define TOP_INT             BIT (1)
#define FLASH_INT           BIT (2)
#define WLED_INT            BIT (4)

static void *max77819_pmic_get_platdata (struct max77819_dev *pmic)
{
#ifdef CONFIG_OF
	struct device *dev = pmic->dev;
	struct device_node *np = dev->of_node;
	struct i2c_client *client = to_i2c_client(dev);
	struct max77819_pmic_platform_data *pdata;
	int rc;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (unlikely(!pdata)) {
		log_err("<%s> out of memory (%uB requested)\n", client->name,
				sizeof(*pdata));
		pdata = ERR_PTR(-ENOMEM);
		goto out;
	}

	pmic->irq_gpio = of_get_named_gpio(np, "max77819,int-gpio", 0);

	if (pmic->irq_gpio < 0) {
		pdata->irq = irq_of_parse_and_map(np, 0);
	} else {
		unsigned gpio = (unsigned)pmic->irq_gpio;

		rc = gpio_request(gpio, DRIVER_NAME"-irq");
		if (unlikely(IS_ERR_VALUE(rc))) {
			log_err("<%s> failed to request gpio %u [%d]\n", client->name, gpio,
					rc);
			pmic->irq_gpio = -1;
			pdata = ERR_PTR(rc);
			goto out;
		}

		gpio_direction_input(gpio);
		log_dbg("<%s> INTGPIO %u assigned\n", client->name, gpio);

		/* override pdata irq */
		pdata->irq = gpio_to_irq(gpio);
	}

	log_dbg("<%s> property:INTGPIO %d\n", client->name, pmic->irq_gpio);
	log_dbg("<%s> property:IRQ     %d\n", client->name, pdata->irq);

out:
	return pdata;
#else /* CONFIG_OF */
	return dev_get_platdata(pmic->dev) ?
		dev_get_platdata(pmic->dev) : ERR_PTR(-EINVAL);
#endif /* CONFIG_OF */
}

static struct regmap_irq max77819_pmic_regmap_irqs[] = {
#define REGMAP_IRQ_PMIC(_irq) \
	[MAX77819_IRQ_##_irq] = { .mask = _irq##_INT, }

	REGMAP_IRQ_PMIC(CHGR),
	REGMAP_IRQ_PMIC(TOP),
	REGMAP_IRQ_PMIC(FLASH),
	REGMAP_IRQ_PMIC(WLED),
};

static struct regmap_irq_chip max77819_pmic_regmap_irq_chip = {
	.name        = DRIVER_NAME,
	.irqs        = max77819_pmic_regmap_irqs,
	.num_irqs    = ARRAY_SIZE(max77819_pmic_regmap_irqs),
	.num_regs    = 1,
	.status_base = INTSRC,
	.mask_base   = INTSRC_MASK,
};

static int max77819_pmic_setup_irq (struct max77819_dev *pmic)
{
	struct device *dev = pmic->dev;
	struct i2c_client *client = to_i2c_client(dev);
	struct max77819_pmic_platform_data *pdata = pmic->pdata;
	struct irq_desc *irq_desc;
	int irq_base, rc = 0;

	/* disable all interrupts */
	max77819_write(&pmic->io, INTSRC_MASK, 0xFF);

	pmic->irq = pdata->irq;
	if (unlikely(pmic->irq <= 0)) {
		log_warn("<%s> interrupt disabled\n", client->name);
		goto out;
	}

	log_dbg("<%s> requesting IRQ %d\n", client->name, pmic->irq);

	rc = regmap_add_irq_chip(pmic->io.regmap, pmic->irq,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT, -1, &max77819_pmic_regmap_irq_chip,
			&pmic->regmap_irq_chip_data);
	if (unlikely(IS_ERR_VALUE(rc))) {
		log_err("<%s> failed to add regmap irq chip [%d]\n", client->name,
				rc);
		pmic->irq = -1;
		goto out;
	}

	irq_desc = irq_to_desc(pmic->irq);
	BUG_ON(!irq_desc);
	pmic->regmap_irq_chip_data = irq_desc->action->dev_id;
	BUG_ON(!pmic->regmap_irq_chip_data);
	irq_base = regmap_irq_chip_get_base(pmic->regmap_irq_chip_data);
	BUG_ON(irq_base < 0);

#ifdef CONFIG_IRQ_DOMAIN
	pmic->irq_domain = irq_domain_add_legacy(dev->of_node,
			max77819_pmic_regmap_irq_chip.num_irqs, irq_base, 0,
			&irq_domain_simple_ops, NULL);
	if (unlikely(!pmic->irq_domain)) {
		log_err("<%s> failed to add irq domain\n", client->name);
		rc = -EIO;
		goto out;
	}
#endif /* CONFIG_IRQ_DOMAIN */

out:
	return rc;
}

static struct mfd_cell max77819_pmic_devices[] = {
	{ .name = MAX77819_CHARGER_NAME, },
	{ .name = MAX77819_SFO_NAME,     },
	{ .name = MAX77819_FLASH_NAME,   },
};

static int max77819_pmic_setup (struct max77819_dev *pmic)
{
	struct device *dev = pmic->dev;
	struct i2c_client *client = to_i2c_client(dev);
	int rc = 0;
	u8 chip_id, chip_rev;

	pmic->pdata = max77819_pmic_get_platdata(pmic);
	if (unlikely(IS_ERR(pmic->pdata))) {
		rc = PTR_ERR(pmic->pdata);
		pmic->pdata = NULL;
		log_err("<%s> platform data is missing [%d]\n", client->name, rc);
		goto out;
	}

	rc = max77819_pmic_setup_irq(pmic);
	if (unlikely(rc)) {
		log_err("<%s> pmic setup irq failed [%d]\n", client->name, rc);
		goto out;
	}

	rc = max77819_add_devices(pmic, max77819_pmic_devices,
			ARRAY_SIZE(max77819_pmic_devices));
	if (unlikely(IS_ERR_VALUE(rc))) {
		log_err("<%s> failed to add sub-devices [%d]\n", client->name, rc);
		goto out;
	}

	/* set device able to wake up system */
	device_init_wakeup(dev, true);
	if (likely(pmic->irq > 0)) {
		enable_irq_wake((unsigned int)pmic->irq);
	}

	log_info("<%s> driver core "DRIVER_VERSION" installed\n", client->name);

	chip_id = 0;
	chip_rev = 0;

	max77819_read(&pmic->io, PMICID,  &chip_id);
	max77819_read(&pmic->io, PMICREV, &chip_rev);

	log_info("CHIP ID %Xh REV %Xh\n", chip_id, chip_rev);

out:
	return rc;
}

/*******************************************************************************
 *** MAX77819 Periph
 ******************************************************************************/

static struct mfd_cell max77819_periph_devices[] = {
	{ .name = MAX77819_WLED_NAME,  },
	{ .name = MAX77819_MOTOR_NAME, },
};

static int max77819_periph_setup (struct max77819_dev *periph)
{
	struct device *dev = periph->dev;
	struct i2c_client *client = to_i2c_client(dev);
	int rc = 0;

	rc = max77819_add_devices(periph, max77819_periph_devices,
			ARRAY_SIZE(max77819_periph_devices));
	if (unlikely(IS_ERR_VALUE(rc))) {
		log_err("<%s> failed to add sub-devices [%d]\n", client->name, rc);
		goto out;
	}

	/* set device able to wake up system */
	/* device_init_wakeup(dev, true); */

	log_info("<%s> driver core "DRIVER_VERSION" installed\n", client->name);

out:
	return rc;
}

/*******************************************************************************
 *** MAX77819 FuelGauge
 ******************************************************************************/

static int max77819_fuelgauge_setup (struct max77819_dev *fuelgauge)
{
	return -ENOTSUPP;
}

/*******************************************************************************
 *** MAX77819 MFD Core
 ******************************************************************************/

static __always_inline void max77819_destroy (struct max77819_dev *me)
{
	struct device *dev = me->dev;

	if (likely(me->irq > 0)) {
		if (me->regmap_irq_chip_data) {
			regmap_del_irq_chip(me->irq, me->regmap_irq_chip_data);
		} else {
			devm_free_irq(dev, me->irq, me);
		}
	}

	if (likely(me->irq_gpio >= 0)) {
		gpio_free((unsigned)me->irq_gpio);
	}

	if (likely(me->attr_grp)) {
		sysfs_remove_group(me->kobj, me->attr_grp);
	}

	if (likely(me->io.regmap)) {
		regmap_exit(me->io.regmap);
	}

#ifdef CONFIG_OF
	if (likely(me->pdata)) {
		devm_kfree(dev, me->pdata);
	}
#endif /* CONFIG_OF */

	mutex_destroy(&me->lock);
	devm_kfree(dev, me);
}

#ifdef CONFIG_OF
static struct of_device_id max77819_of_ids[] = {
	{ .compatible = "maxim,"MAX77819_PMIC_NAME      },
	{ .compatible = "maxim,"MAX77819_PERIPH_NAME    },
	{ .compatible = "maxim,"MAX77819_FUELGAUGE_NAME },
	{ },
};
MODULE_DEVICE_TABLE(of, max77819_of_ids);
#endif /* CONFIG_OF */

static const struct i2c_device_id max77819_i2c_ids[] = {
	{ MAX77819_PMIC_NAME,      MAX77819_DEV_PMIC      },
	{ MAX77819_PERIPH_NAME,    MAX77819_DEV_PERIPH    },
	{ MAX77819_FUELGAUGE_NAME, MAX77819_DEV_FUELGAUGE },
	{ },
};
MODULE_DEVICE_TABLE(i2c, max77819_i2c_ids);

static __devinit int max77819_i2c_probe (struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct max77819_core *core = &max77819;
	struct max77819_dev *me;
	int rc;

	log_dbg("<%s> attached\n", client->name);

	me = devm_kzalloc(&client->dev, sizeof(*me), GFP_KERNEL);
	if (unlikely(!me)) {
		log_err("<%s> out of memory (%uB requested)\n", client->name,
				sizeof(*me));
		return -ENOMEM;
	}

	i2c_set_clientdata(client, me);

	mutex_init(&me->lock);
	me->core     = core;
	me->dev      = &client->dev;
	me->kobj     = &client->dev.kobj;
	me->irq      = -1;
	me->irq_gpio = -1;

	me->io.regmap = devm_regmap_init_i2c(client, &max77819_regmap_config);
	if (unlikely(IS_ERR(me->io.regmap))) {
		rc = PTR_ERR(me->io.regmap);
		me->io.regmap = NULL;
		log_err("<%s> failed to initialize i2c regmap [%d]\n", client->name,
				rc);
		goto abort;
	}

	/* detect device ID & post-probe */
	me->dev_id = (int)id->driver_data;
	switch (me->dev_id) {
	case MAX77819_DEV_PMIC:
		rc = max77819_pmic_setup(me);
		break;

	case MAX77819_DEV_PERIPH:
		rc = max77819_periph_setup(me);
		break;

	case MAX77819_DEV_FUELGAUGE:
		rc = max77819_fuelgauge_setup(me);
		break;

		break;
		log_err("<%s> unknown device\n", client->name);
		BUG();
		rc = -ENOTSUPP;
		goto abort;
	}

	/* all done successfully */
	core->dev[me->dev_id] = me;
	log_dbg("<%s> probe DONE,\n", client->name);

	return 0;

abort:
	i2c_set_clientdata(client, NULL);
	max77819_destroy(me);
	return rc;
}

static __devexit int max77819_i2c_remove (struct i2c_client *client)
{
	struct max77819_dev *me = i2c_get_clientdata(client);

	me->core->dev[me->dev_id] = NULL;

	i2c_set_clientdata(client, NULL);
	max77819_destroy(me);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max77819_suspend (struct device *dev)
{
	struct max77819_dev *me = dev_get_drvdata(dev);
	struct i2c_client *client = to_i2c_client(dev);

	__lock(me);

	log_vdbg("<%s> suspending\n", client->name);

	__unlock(me);
	return 0;
}

static int max77819_resume (struct device *dev)
{
	struct max77819_dev *me = dev_get_drvdata(dev);
	struct i2c_client *client = to_i2c_client(dev);

	__lock(me);

	log_vdbg("<%s> resuming\n", client->name);

	__unlock(me);
	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(max77819_pm, max77819_suspend, max77819_resume);

static struct i2c_driver max77819_i2c_driver = {
	.driver.name            = DRIVER_NAME,
	.driver.owner           = THIS_MODULE,
	.driver.pm              = &max77819_pm,
#ifdef CONFIG_OF
	.driver.of_match_table  = max77819_of_ids,
#endif /* CONFIG_OF */
	.id_table               = max77819_i2c_ids,
	.probe                  = max77819_i2c_probe,
	.remove                 = __devexit_p(max77819_i2c_remove),
};

static __init int max77819_init (void)
{
	mutex_init(&max77819.lock);
	return i2c_add_driver(&max77819_i2c_driver);
}
module_init(max77819_init);

static __exit void max77819_exit (void)
{
	i2c_del_driver(&max77819_i2c_driver);
	mutex_destroy(&max77819.lock);
}
module_exit(max77819_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_VERSION(DRIVER_VERSION);

/*******************************************************************************
 * EXTERNAL SERVICES
 ******************************************************************************/

struct max77819_io *max77819_get_io (struct max77819_dev *chip)
{
	if (unlikely(!chip)) {
		log_err("not ready\n");
		return NULL;
	}

	return &chip->io;
} EXPORT_SYMBOL(max77819_get_io);
