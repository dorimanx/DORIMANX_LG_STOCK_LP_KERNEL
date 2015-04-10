/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
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
#define IMX091_SENSOR_NAME "imx091"
DEFINE_MSM_MUTEX(imx091_mut);

static struct msm_sensor_ctrl_t imx091_s_ctrl;

static struct msm_sensor_power_setting imx091_power_setting[] = {
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VANA,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VIO,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VAF,
		.config_val = GPIO_OUT_HIGH,
		.delay = 10,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 30,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

static struct v4l2_subdev_info imx091_subdev_info[] = {
	{
		.code = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt = 1,
		.order = 0,
	},
};

static const struct i2c_device_id imx091_i2c_id[] = {
	{IMX091_SENSOR_NAME, (kernel_ulong_t)&imx091_s_ctrl},
	{ }
};

static int32_t msm_imx091_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &imx091_s_ctrl);
}

static struct i2c_driver imx091_i2c_driver = {
	.id_table = imx091_i2c_id,
	.probe  = msm_imx091_i2c_probe,
	.driver = {
		.name = IMX091_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client imx091_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id imx091_dt_match[] = {
	{.compatible = "qcom,imx091", .data = &imx091_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, imx091_dt_match);

static struct platform_driver imx091_platform_driver = {
	.driver = {
		.name = "qcom,imx091",
		.owner = THIS_MODULE,
		.of_match_table = imx091_dt_match,
	},
};

static int32_t imx091_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(imx091_dt_match, &pdev->dev);
    /* LGE_CHANGE_S : WBT 2014-03-21 sungmin.cho@lge.com */
    if(!match)
    {
      pr_err(" %s failed ",__func__);
      return -ENODEV;
    }
    /* LGE_CHANGE_E : WBT 2014-03-21 sungmin.cho@lge.com */
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init imx091_init_module(void)
{
	int32_t rc = 0;
	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&imx091_platform_driver,
		imx091_platform_probe);
	if (!rc)
		return rc;
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&imx091_i2c_driver);
}

static void __exit imx091_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (imx091_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&imx091_s_ctrl);
		platform_driver_unregister(&imx091_platform_driver);
	} else
		i2c_del_driver(&imx091_i2c_driver);
	return;
}

static struct msm_sensor_ctrl_t imx091_s_ctrl = {
	.sensor_i2c_client = &imx091_sensor_i2c_client,
	.power_setting_array.power_setting = imx091_power_setting,
	.power_setting_array.size = ARRAY_SIZE(imx091_power_setting),
	.msm_sensor_mutex = &imx091_mut,
	.sensor_v4l2_subdev_info = imx091_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(imx091_subdev_info),
};

module_init(imx091_init_module);
module_exit(imx091_exit_module);
MODULE_DESCRIPTION("imx091");
MODULE_LICENSE("GPL v2");