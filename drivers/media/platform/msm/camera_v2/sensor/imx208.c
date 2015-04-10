/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include "msm_sensor.h"
#include <mach/board_lge.h>		//to use lge_get_board_revno()

#define IMX208_SENSOR_NAME "imx208"
DEFINE_MSM_MUTEX(imx208_mut);

// LGE_CHNAGE_S sungsik.kim 2013/04/07 {
#define CONFIG_IMX208_DEBUG
// LGE_CHNAGE_E sungsik.kim 2013/04/07 }

#undef CDBG
#ifdef CONFIG_IMX208_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

static struct msm_sensor_ctrl_t imx208_s_ctrl;
#if defined (CONFIG_MACH_MSM8974_G3_GLOBAL_COM) || defined (CONFIG_MACH_MSM8974_G3_ATT)|| defined (CONFIG_MACH_MSM8974_G3_VZW) || defined(CONFIG_MACH_MSM8974_G3_CA) || defined(CONFIG_MACH_MSM8974_G3_LRA)
static struct msm_sensor_power_setting imx208_power_setting_rev_b[] = {
	{  /* Set GPIO_RESET to low to disable power on reset*/
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{										//VDIG, PMIC_GPIO 10
		.seq_type = SENSOR_VREG, //VREG_L18
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO, //VDD DIG GPIO 89
		.seq_val = SENSOR_GPIO_VDIG,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG, //VREG_LVS2
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 50,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 0,
		.delay = 50,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 10,
	},
};
#elif defined(CONFIG_MACH_MSM8974_TIGERS_KR) || defined(CONFIG_MACH_MSM8974_B1_KR) || defined(CONFIG_MACH_MSM8974_B1W)
static struct msm_sensor_power_setting imx208_power_setting_rev_b[] = {
	{  /* Set GPIO_RESET to low to disable power on reset*/
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
#if defined(CONFIG_MACH_MSM8974_TIGERS_KR)
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VDIG,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
#else
	{										//VDIG, PMIC_GPIO 10
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VANA,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
#endif
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 0,
	},
#if defined(CONFIG_MACH_MSM8974_TIGERS_KR)
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 1,
	},
#else
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 1,
	},
#endif
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 10,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 0,
		.delay = 30,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};
#else
static struct msm_sensor_power_setting imx208_power_setting_rev_b[] = {
	{  /* Set GPIO_RESET to low to disable power on reset*/
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{										//VDIG, PMIC_GPIO 10
		.seq_type = SENSOR_VREG, //VREG_L18
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG, //VREG_L3
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG, //VREG_LVS2
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 50,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 0,
		.delay = 50,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 10,
	},
};
#endif

static struct msm_sensor_power_setting imx208_power_setting_rev_a[] = {
	{  /* Set GPIO_RESET to low to disable power on reset*/
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{										//VDIG, PMIC_GPIO 10
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 50,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 0,
		.delay = 50,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 10,
	},
};

static struct v4l2_subdev_info imx208_subdev_info[] = {
	{
		.code		= V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace	= V4L2_COLORSPACE_JPEG,
		.fmt		= 1,
		.order		= 0,
	},
};

static const struct i2c_device_id imx208_i2c_id[] = {
	{IMX208_SENSOR_NAME, (kernel_ulong_t)&imx208_s_ctrl},
	{ }
};

static int32_t msm_imx208_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &imx208_s_ctrl);
}

static struct i2c_driver imx208_i2c_driver = {
	.id_table = imx208_i2c_id,
	.probe  = msm_imx208_i2c_probe,
	.driver = {
		.name = IMX208_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client imx208_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id imx208_dt_match[] = {
	{.compatible = "qcom,imx208", .data = &imx208_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, imx208_dt_match);

static struct platform_driver imx208_platform_driver = {
	.driver = {
		.name = "qcom,imx208",
		.owner = THIS_MODULE,
		.of_match_table = imx208_dt_match,
	},
};

static int32_t imx208_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	CDBG("%s E\n", __func__);
	match = of_match_device(imx208_dt_match, &pdev->dev);

/* LGE_CHANGE_S : WBT, 2013-5-31, jonghwan.ko@lge.com */
	if(!match)
	{
		pr_err(" %s failed ",__func__);
	      return -ENODEV;
	}
/* LGE_CHANGE_E : WBT, 2013-5-31, jonghwan.ko@lge.com */

	rc = msm_sensor_platform_probe(pdev, match->data);
	CDBG("%s: X, rc = %d\n", __func__, rc);
	return rc;
}

static int __init imx208_init_module(void)
{
	int32_t rc = 0;
	CDBG("%s E\n", __func__);
#if defined(CONFIG_MACH_LGE)
		switch(lge_get_board_revno()) {
			case HW_REV_A:
				CDBG("%s: Sensor power is set REV An", __func__);
				imx208_s_ctrl.power_setting_array.power_setting = imx208_power_setting_rev_a;
				imx208_s_ctrl.power_setting_array.size = ARRAY_SIZE(imx208_power_setting_rev_a);
				break;
			case HW_REV_B:
			default:
				CDBG("%s: Sensor power is set REV B\n", __func__);
				imx208_s_ctrl.power_setting_array.power_setting = imx208_power_setting_rev_b;
				imx208_s_ctrl.power_setting_array.size = ARRAY_SIZE(imx208_power_setting_rev_b);
				break;
		}
#endif

#if defined(CONFIG_MACH_MSM8974_TIGERS_KR) || defined(CONFIG_MACH_MSM8974_B1_KR) || defined(CONFIG_MACH_MSM8974_B1W)
		CDBG("%s: Sensor power is set for B1\n", __func__);
		imx208_s_ctrl.power_setting_array.power_setting = imx208_power_setting_rev_b;
		imx208_s_ctrl.power_setting_array.size = ARRAY_SIZE(imx208_power_setting_rev_b);
#endif

	rc = platform_driver_probe(&imx208_platform_driver,
		imx208_platform_probe);
	if (!rc) {
		CDBG("%s: X, rc = %d\n", __func__, rc);
		return rc;
	}
	return i2c_add_driver(&imx208_i2c_driver);
}

static void __exit imx208_exit_module(void)
{
	if (imx208_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&imx208_s_ctrl);
		platform_driver_unregister(&imx208_platform_driver);
	} else
		i2c_del_driver(&imx208_i2c_driver);
	return;
}

static struct msm_sensor_ctrl_t imx208_s_ctrl = {
	.sensor_i2c_client = &imx208_sensor_i2c_client,
	//.power_setting_array.power_setting = imx208_power_setting,
	//.power_setting_array.size = ARRAY_SIZE(imx208_power_setting),
	.msm_sensor_mutex = &imx208_mut,
	.sensor_v4l2_subdev_info = imx208_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(imx208_subdev_info),
};

module_init(imx208_init_module);
module_exit(imx208_exit_module);
MODULE_DESCRIPTION("imx208");
MODULE_LICENSE("GPL v2");
