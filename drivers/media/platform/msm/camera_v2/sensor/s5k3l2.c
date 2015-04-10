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
#define S5K3L2_SENSOR_NAME "s5k3l2"
#include <mach/board_lge.h>
DEFINE_MSM_MUTEX(s5k3l2_mut);

static struct msm_sensor_ctrl_t s5k3l2_s_ctrl;

static struct msm_sensor_power_setting s5k3l2_power_setting[] = {
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 0,
	},
	{										//VIO, GPIO 96
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VIO,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
	{										//VDIG
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 0,
	},
	{										//VANA, GPIO 16
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VANA,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
#if defined(CONFIG_LG_PROXY)
	{										//LDAF_EN, PMIC_GPIO 1
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_LDAF_EN,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
#endif
	{										//VCM, GPIO 145
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VAF,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 0,
		.delay = 10,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 1,
	},
};

static struct v4l2_subdev_info s5k3l2_subdev_info[] = {
	{
		.code = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt = 1,
		.order = 0,
	},
};

static const struct i2c_device_id s5k3l2_i2c_id[] = {
	{S5K3L2_SENSOR_NAME, (kernel_ulong_t)&s5k3l2_s_ctrl},
	{ }
};

static int32_t msm_s5k3l2_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &s5k3l2_s_ctrl);
}

static struct i2c_driver s5k3l2_i2c_driver = {
	.id_table = s5k3l2_i2c_id,
	.probe  = msm_s5k3l2_i2c_probe,
	.driver = {
		.name = S5K3L2_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client s5k3l2_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id s5k3l2_dt_match[] = {
	{.compatible = "qcom,s5k3l2", .data = &s5k3l2_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, s5k3l2_dt_match);

static struct platform_driver s5k3l2_platform_driver = {
	.driver = {
		.name = "qcom,s5k3l2",
		.owner = THIS_MODULE,
		.of_match_table = s5k3l2_dt_match,
	},
};

static int32_t s5k3l2_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(s5k3l2_dt_match, &pdev->dev);
	if(!match)
	{
		  pr_err(" %s failed ",__func__);
		  return -ENODEV;
	 }
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init s5k3l2_init_module(void)
{
	int32_t rc = 0;
	pr_info("%s:%d\n", __func__, __LINE__);

	rc = platform_driver_probe(&s5k3l2_platform_driver,
		s5k3l2_platform_probe);
	if (!rc)
		return rc;
	pr_info("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&s5k3l2_i2c_driver);
}

static struct msm_sensor_fn_t s5k3l2_sensor_func_tbl = {
	.sensor_config = msm_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = msm_sensor_match_id,
};

static void __exit s5k3l2_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (s5k3l2_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&s5k3l2_s_ctrl);
		platform_driver_unregister(&s5k3l2_platform_driver);
	} else
		i2c_del_driver(&s5k3l2_i2c_driver);
	return;
}

static struct msm_sensor_ctrl_t s5k3l2_s_ctrl = {
	.sensor_i2c_client = &s5k3l2_sensor_i2c_client,
	.power_setting_array.power_setting = s5k3l2_power_setting,
	.power_setting_array.size = ARRAY_SIZE(s5k3l2_power_setting),
	.msm_sensor_mutex = &s5k3l2_mut,
	.sensor_v4l2_subdev_info = s5k3l2_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(s5k3l2_subdev_info),
	.func_tbl = &s5k3l2_sensor_func_tbl,
};

module_init(s5k3l2_init_module);
module_exit(s5k3l2_exit_module);
MODULE_DESCRIPTION("s5k3l2");
MODULE_LICENSE("GPL v2");

