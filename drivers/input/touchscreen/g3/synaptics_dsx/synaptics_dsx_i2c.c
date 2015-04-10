/*
 * Synaptics DSX touchscreen driver
 *
 * Copyright (C) 2012 Synaptics Incorporated
 *
 * Copyright (C) 2012 Alexandra Chin <alexandra.chin@tw.synaptics.com>
 * Copyright (C) 2012 Scott Lin <scott.lin@tw.synaptics.com>
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
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/input/synaptics_dsx.h>
#include "synaptics_dsx_core.h"
#ifdef CUST_LGE_TOUCH_BRING_UP
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/machine.h>
#endif

#ifdef CUST_LGE_TOUCH_BRING_UP
#define I2C_DRIVER_NAME "synaptics_dsx_i2c"
#endif
#define SYN_I2C_RETRY_TIMES 10

static int synaptics_rmi4_i2c_set_page(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr)
{
	int retval;
	unsigned char retry;
	unsigned char buf[PAGE_SELECT_LEN];
	unsigned char page;
	struct i2c_client *i2c = to_i2c_client(rmi4_data->pdev->dev.parent);

	page = ((addr >> 8) & MASK_8BIT);
	if (page != rmi4_data->current_page) {
		buf[0] = MASK_8BIT;
		buf[1] = page;
		for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
			retval = i2c_master_send(i2c, buf, PAGE_SELECT_LEN);
			if (retval != PAGE_SELECT_LEN) {
				dev_err(rmi4_data->pdev->dev.parent,
						"%s: I2C retry %d\n",
						__func__, retry + 1);
				msleep(20);
			} else {
				rmi4_data->current_page = page;
				break;
			}
		}
	} else {
		retval = PAGE_SELECT_LEN;
	}

	return retval;
}

static int synaptics_rmi4_i2c_read(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data, unsigned short length)
{
	int retval;
	unsigned char retry;
	unsigned char buf;
	struct i2c_client *i2c = to_i2c_client(rmi4_data->pdev->dev.parent);
	struct i2c_msg msg[] = {
		{
			.addr = i2c->addr,
			.flags = 0,
			.len = 1,
			.buf = &buf,
		},
		{
			.addr = i2c->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		},
	};

	buf = addr & MASK_8BIT;

	mutex_lock(&rmi4_data->rmi4_io_ctrl_mutex);

	retval = synaptics_rmi4_i2c_set_page(rmi4_data, addr);
	if (retval != PAGE_SELECT_LEN) {
		retval = -EIO;
		goto exit;
	}

	for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(i2c->adapter, msg, 2) == 2) {
			retval = length;
			break;
		}
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: I2C retry %d\n",
				__func__, retry + 1);
		msleep(20);
	}

	if (retry == SYN_I2C_RETRY_TIMES) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: I2C read over retry limit\n",
				__func__);
		retval = -EIO;
	}

exit:
	mutex_unlock(&rmi4_data->rmi4_io_ctrl_mutex);

	return retval;
}

static int synaptics_rmi4_i2c_write(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data, unsigned short length)
{
	int retval;
	unsigned char retry;
	unsigned char buf[length + 1];
	struct i2c_client *i2c = to_i2c_client(rmi4_data->pdev->dev.parent);
	struct i2c_msg msg[] = {
		{
			.addr = i2c->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		}
	};

	mutex_lock(&rmi4_data->rmi4_io_ctrl_mutex);

	retval = synaptics_rmi4_i2c_set_page(rmi4_data, addr);
	if (retval != PAGE_SELECT_LEN) {
		retval = -EIO;
		goto exit;
	}

	buf[0] = addr & MASK_8BIT;
	memcpy(&buf[1], &data[0], length);

	for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(i2c->adapter, msg, 1) == 1) {
			retval = length;
			break;
		}
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: I2C retry %d\n",
				__func__, retry + 1);
		msleep(20);
	}

	if (retry == SYN_I2C_RETRY_TIMES) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: I2C write over retry limit\n",
				__func__);
		retval = -EIO;
	}

exit:
	mutex_unlock(&rmi4_data->rmi4_io_ctrl_mutex);

	return retval;
}

static struct synaptics_dsx_bus_access bus_access = {
	.type = BUS_I2C,
	.read = synaptics_rmi4_i2c_read,
	.write = synaptics_rmi4_i2c_write,
};

static struct synaptics_dsx_hw_interface hw_if;

static struct platform_device *synaptics_dsx_i2c_device;

static void synaptics_rmi4_i2c_dev_release(struct device *dev)
{
	kfree(synaptics_dsx_i2c_device);

	return;
}

#ifdef CUST_LGE_TOUCH_BRING_UP
int rmi4_power_on(struct i2c_client *client, int on)
{
	int rc = 0;
	static struct regulator *vreg_l22;
	static struct regulator *vreg_lvs3;

		/* 3.3V_TOUCH_VDD, VREG_L15: 2.75 ~ 3.3 */
		if (!vreg_l22) {
			vreg_l22 = regulator_get(&client->dev, "vdd_ana");
			if (IS_ERR(vreg_l22)) {
				pr_err("%s: regulator get of pm8941_l22 failed (%ld)\n",
						__func__,
					   PTR_ERR(vreg_l22));
				rc = PTR_ERR(vreg_l22);
				vreg_l22 = NULL;
				return rc;
			}
		}
		/* 1.8V_TOUCH_IO, VREG_L22: 1.7 ~ 2.85 */
		if (!vreg_lvs3) {
			vreg_lvs3 = regulator_get(&client->dev, "vcc_i2c");
			if (IS_ERR(vreg_lvs3)) {
				pr_err("%s: regulator get of pm8941_lvs3 failed (%ld)\n",
						__func__,
					   PTR_ERR(vreg_lvs3));
				rc = PTR_ERR(vreg_lvs3);
				vreg_lvs3 = NULL;
				return rc;
			}
		}

			rc = regulator_set_voltage(vreg_l22, 3300000, 3300000);

		if (rc < 0) {
			printk(KERN_INFO "[Touch D] %s: cannot control regulator:%d\n",
				   __func__, rc);
			return rc;
		}

		if (on) {
			printk("[Touch D]touch enable\n");
			regulator_enable(vreg_l22);
			regulator_enable(vreg_lvs3);
		} else {
			printk("[Touch D]touch disable\n");
			regulator_disable(vreg_l22);
			regulator_disable(vreg_lvs3);
		}

		printk("[Touch D] Complete %s !!! \n",__func__);

		return rc;

}
static int synaptics_rmi4_parse_dt(struct device *dev, struct synaptics_dsx_board_data *p_data)
 {
	 struct device_node *node = dev->of_node;
	 int rc = 0;
	 u32 temp_val;

	 printk("[Touch] %s START!!!!!!!!!!!!!!\n",__func__);

	 /* irq gpio  info */
	 if (node == NULL)
		 return -ENODEV;

	if(!of_property_read_bool(node, "rmi4,x_flip"))
		p_data->x_flip = 0;
	if(!of_property_read_bool(node, "rmi4,y_flip"))
		p_data->y_flip = 0;
	if(of_property_read_bool(node, "rmi4,regulator_en"))
		p_data->regulator_en = 1;
	p_data->reset_gpio = of_get_named_gpio_flags(node, "rmi4,reset_gpio", 0, NULL);
	p_data->irq_gpio = of_get_named_gpio_flags(node, "rmi4,irq_gpio", 0, NULL);

	rc = of_property_read_u32(node, "rmi4,irq_flags", &temp_val);
	p_data->irq_flags = (!rc ? temp_val : IRQF_TRIGGER_FALLING);

	rc = of_property_read_u32(node, "rmi4,panel_x", &temp_val);
	p_data->panel_x = (!rc ? temp_val : 1440);

	rc = of_property_read_u32(node, "rmi4,panel_y", &temp_val);
	p_data->panel_y = (!rc ? temp_val : 2560);

	rc = of_property_read_u32(node, "rmi4,reset_delay_ms", &temp_val);
	p_data->reset_delay_ms = (!rc ? temp_val : 10);

	return 0;
 }
#endif

static int synaptics_rmi4_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *dev_id)
{
	int retval;
	int ret = 0;
#ifdef CUST_LGE_TOUCH_BRING_UP
	struct synaptics_rmi4_data *rmi4_data;
	struct synaptics_rmi4_device_info *rmi;
	const struct synaptics_dsx_board_data *platform_data =
			client->dev.platform_data;
	struct synaptics_dsx_board_data *p_data;

	printk("[Touch] %s START!!!!!!!!!!!!!!\n",__func__);
#endif

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev,
				"%s: SMBus byte data commands not supported by host\n",
				__func__);
		return -EIO;
	}
#ifdef CUST_LGE_TOUCH_BRING_UP
	rmi4_data = kzalloc(sizeof(*rmi4_data), GFP_KERNEL);
	if (!rmi4_data) {
		dev_err(&client->dev,
				"%s: Failed to alloc mem for rmi4_data\n",
				__func__);
		return -ENOMEM;
	}
#endif

	synaptics_dsx_i2c_device = kzalloc(
			sizeof(struct platform_device),
			GFP_KERNEL);
	if (!synaptics_dsx_i2c_device) {
		dev_err(&client->dev,
				"%s: Failed to allocate memory for synaptics_dsx_i2c_device\n",
				__func__);
		ret = -ENOMEM;
		goto err_alloc_dsx_i2c_device;
	}

#ifdef CUST_LGE_TOUCH_BRING_UP
	if (client->dev.of_node) {
		p_data = devm_kzalloc(&client->dev,
			sizeof(struct synaptics_dsx_board_data), GFP_KERNEL);
		if (!p_data) {
			dev_err(&client->dev, "%s : Failed to allocate memory\n", __func__);
			ret = -ENOMEM;
			goto error_alloc_dsx_board_data;
		}

		retval = synaptics_rmi4_parse_dt(&client->dev, p_data);
		platform_data = p_data;
		client->dev.platform_data = p_data;
		if (retval) {
			printk("%s : Failed to parse device Tree",__func__);
			ret = retval;
			goto error;
		}

	} else {
		platform_data  = client->dev.platform_data;
		if (!platform_data) {
			printk("%s : Failed to copy platform_data\n", __func__);
			retval = -EINVAL;
		}
	}

	printk("%s: Probing i2c RMI device, addr: 0x%02x\n", __func__, client->addr);
	
	rmi = &(rmi4_data->rmi4_mod_info);
	
	if (!platform_data) {
		dev_err(&client->dev,
				"%s: No platform data found\n",
				__func__);
		ret = -EINVAL;
		goto error;
	}

	printk("[Touch D] x_flip = %d\n", platform_data->x_flip);
	printk("[Touch D] y_flip = %d\n", platform_data->y_flip);
	printk("[Touch D] regulator_en = %d\n", platform_data->regulator_en);
	printk("[Touch D] reset_gpio = %d\n", platform_data->reset_gpio);
	printk("[Touch D] irq_gpio = %d\n", platform_data->irq_gpio);
	printk("[Touch D] irq_flags = %lx\n", platform_data->irq_flags);
	printk("[Touch D] panel_x = %d\n", platform_data->panel_x);
	printk("[Touch D] panel_y = %d\n", platform_data->panel_y);
	printk("[Touch D] reset_delay_ms = %d\n", platform_data->reset_delay_ms);
	
	 if (platform_data->regulator_en) { 
	 	rmi4_data->vreg_l22 = regulator_get(&client->dev, "vdd_ana"); 
		if (IS_ERR(rmi4_data->vreg_l22)) { 
	 		dev_err(&client->dev, 
	 				"%s: Failed to get regulator vreg_l22\n", 
	 				__func__); 
			retval = PTR_ERR(rmi4_data->vreg_l22);
			ret = -retval;
			goto error;
	 	} 
		regulator_set_voltage(rmi4_data->vreg_l22, 3300000, 3300000); 
	 	regulator_enable(rmi4_data->vreg_l22); 

	 	rmi4_data->vreg_lvs3 = regulator_get(&client->dev, "vcc_i2c"); 
	 	if (IS_ERR(rmi4_data->vreg_lvs3)) { 
	 		dev_err(&client->dev, 
					"%s: Failed to get regulator vreg_l22\n",
					__func__); 
			retval = PTR_ERR(rmi4_data->vreg_lvs3);
			ret = -retval;
			goto error;
	 	} 
	 	regulator_enable(rmi4_data->vreg_lvs3); 
	 	}else
	 	rmi4_power_on(client, 1);
		msleep(400);

	printk("[Touch D] %s : Done regulator set!!! \n",__func__);
#endif

	hw_if.board_data = client->dev.platform_data;
	hw_if.bus_access = &bus_access;

	synaptics_dsx_i2c_device->name = PLATFORM_DRIVER_NAME;
	synaptics_dsx_i2c_device->id = 0;
	synaptics_dsx_i2c_device->num_resources = 0;
	synaptics_dsx_i2c_device->dev.parent = &client->dev;
	synaptics_dsx_i2c_device->dev.platform_data = &hw_if;
	synaptics_dsx_i2c_device->dev.release = synaptics_rmi4_i2c_dev_release;

	retval = platform_device_register(synaptics_dsx_i2c_device);
	if (retval) {
		dev_err(&client->dev,
				"%s: Failed to register platform device\n",
				__func__);
		ret = -ENODEV;
		goto error;
	}

#ifdef CUST_LGE_TOUCH_BRING_UP
	printk("[Touch D] Complete %s !!! \n",__func__);
#endif
	return 0;

#ifdef CUST_LGE_TOUCH_BRING_UP
error:
	devm_kfree(&client->dev, p_data);
#endif
error_alloc_dsx_board_data:
	kfree(synaptics_dsx_i2c_device);
err_alloc_dsx_i2c_device:
#ifdef CUST_LGE_TOUCH_BRING_UP
	kfree(rmi4_data);
#endif
	return ret;
}

static int synaptics_rmi4_i2c_remove(struct i2c_client *client)
{
	platform_device_unregister(synaptics_dsx_i2c_device);

	return 0;
}
#ifdef CUST_LGE_TOUCH_BRING_UP
static struct of_device_id rmi_match_table[] = {
	{ .compatible = "rmi,s3528",},
	{ },
};
#endif

static const struct i2c_device_id synaptics_rmi4_id_table[] = {
	{I2C_DRIVER_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, synaptics_rmi4_id_table);

static struct i2c_driver synaptics_rmi4_i2c_driver = {
	.driver = {
		.name = I2C_DRIVER_NAME,
		.owner = THIS_MODULE,
#ifdef CUST_LGE_TOUCH_BRING_UP
		.of_match_table = rmi_match_table,
#endif
	},
	.probe = synaptics_rmi4_i2c_probe,
	.remove = __devexit_p(synaptics_rmi4_i2c_remove),
	.id_table = synaptics_rmi4_id_table,
};

int synaptics_rmi4_bus_init(void)
{
#ifdef CUST_LGE_TOUCH_BRING_UP
	printk("[Touch D] synaptics_rmi4_init \n");
#endif
	return i2c_add_driver(&synaptics_rmi4_i2c_driver);
}
EXPORT_SYMBOL(synaptics_rmi4_bus_init);

void synaptics_rmi4_bus_exit(void)
{
	i2c_del_driver(&synaptics_rmi4_i2c_driver);

	return;
}
EXPORT_SYMBOL(synaptics_rmi4_bus_exit);

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("Synaptics DSX I2C Bus Support Module");
MODULE_LICENSE("GPL v2");
