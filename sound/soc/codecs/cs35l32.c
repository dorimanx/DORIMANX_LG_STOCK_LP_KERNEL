/*
 * cs35l32.c -- CS35L32 ALSA SoC audio driver
 *
 * Copyright 2012 CirrusLogic, Inc.
 *
 * Author: Brian Austin <brian.austin@cirrus.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <sound/cs35l32.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>

#include "cs35l32.h"

struct  cs35l32_private {
	struct regmap *regmap;
	struct snd_soc_codec *codec;
	struct cs35l32_platform_data pdata;
	u32 fmt;
	unsigned int adsp_ctl;
	unsigned int clk_ctl;
	u32 sysclk;
	u32 mclk;
	u8 flags;
};

static const struct reg_default cs35l32_reg_defaults[] = {
	
	{ 0x06, 0x00 }, /* Power Ctl 1 */
	{ 0x07, 0xE8 }, /* Power Ctl 2 */
	{ 0x08, 0x40 }, /* Clock Ctl */
	{ 0x09, 0x20 }, /* Low Battery Threshold */
	{ 0x0A, 0x00 }, /* Voltage Monitor [RO] */
	{ 0x0B, 0x40 }, /* Conv Peak Curr Protection CTL */
	{ 0x0C, 0x07 }, /* IMON Scaling */
	{ 0x0D, 0x03 }, /* Audio/LED Pwr Manager */
	{ 0x0F, 0x20 }, /* Serial Port Control */
	{ 0x10, 0x14 }, /* Class D Amp CTL */
	{ 0x11, 0x00 }, /* Protection Release CTL */
	{ 0x12, 0xFF }, /* Interrupt Mask 1 */
	{ 0x13, 0xFF }, /* Interrupt Mask 2 */
	{ 0x14, 0xFF }, /* Interrupt Mask 3 */

};

static bool cs35l32_readable_register(struct device *dev, unsigned int
reg) {
	switch (reg) {
	case CS35L32_CHIP_ID:
	case CS35L32_DEVID_AB:
	case CS35L32_DEVID_CD:
	case CS35L32_DEVID_E:
	case CS35L32_FAB_ID:
	case CS35L32_REV_ID:
	case CS35L32_PWRCTL1:
	case CS35L32_PWRCTL2:
	case CS35L32_CLK_CTL:
	case CS35L32_BATT_THRESHOLD:
	case CS35L32_VMON:
	case CS35L32_BST_CPCP_CTL:
	case CS35L32_IMON_SCALING:
	case CS35L32_AUDIO_LED_MNGR:
	case CS35L32_ADSP_CTL:
	case CS35L32_CLASSD_CTL:
	case CS35L32_PROTECT_CTL:
	case CS35L32_INT_MASK_1:
	case CS35L32_INT_MASK_2:
	case CS35L32_INT_MASK_3:
	case CS35L32_INT_STATUS_1:
	case CS35L32_INT_STATUS_2:
	case CS35L32_INT_STATUS_3:
		return true;
	default:
		return false;
	}
}

static bool cs35l32_volatile_register(struct device *dev, unsigned int
reg) {
	switch (reg) {
	case CS35L32_INT_STATUS_1:
	case CS35L32_INT_STATUS_2:
	case CS35L32_INT_STATUS_3:
		return 1;
	default:
		return 0;
	}
}

static DECLARE_TLV_DB_SCALE(classd_ctl_tlv, 900, 300, 0);

static const struct snd_kcontrol_new imon_ctl =
	SOC_DAPM_SINGLE("Switch", CS35L32_PWRCTL2, 6, 1, 1);
	
static const struct snd_kcontrol_new vmon_ctl =
	SOC_DAPM_SINGLE("Switch", CS35L32_PWRCTL2, 7, 1, 1);
	
static const struct snd_kcontrol_new vpmon_ctl =
	SOC_DAPM_SINGLE("Switch", CS35L32_PWRCTL2, 5, 1, 1);
	
static const struct snd_kcontrol_new cs35l32_snd_controls[] = {
	
	SOC_SINGLE_TLV("SPK Amp Volume", CS35L32_CLASSD_CTL,
		       3, 0x04, 0, classd_ctl_tlv),
	SOC_SINGLE("SPK VBoost Mng", CS35L32_AUDIO_LED_MNGR, 0, 4, 0),
	SOC_SINGLE("Gain Zero Cross", CS35L32_CLASSD_CTL, 2, 1, 0),
	
	SOC_SINGLE("Monitor Select", CS35L32_PWRCTL2, 3, 1, 1), };

static int int_clear(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event) {
	struct snd_soc_codec *codec = w->codec;
	u8 reg, reg2, reg3;
	
	if (SND_SOC_DAPM_EVENT_ON(event)) {
		snd_soc_read(codec, CS35L32_INT_STATUS_1); 
		snd_soc_read(codec, CS35L32_INT_STATUS_2);
		pr_info("%s Read and Clear\n",__func__);
	} else {
		return 0;
	}
	mdelay(15);
	reg = snd_soc_read(codec, CS35L32_INT_STATUS_1);
	printk("INT 1 REG = %x\n", reg);
	reg2 = snd_soc_read(codec, CS35L32_INT_STATUS_2);
	printk("INT 2 REG = %x\n", reg2);
	
	snd_soc_update_bits(codec, CS35L32_PWRCTL1,
				    CS35L32_PDN_ALL, 1);
	reg3 = snd_soc_read(codec, CS35L32_PWRCTL1);
	printk("PWRCTL1 REG = %x\n", reg3);
	mdelay(20);
	snd_soc_update_bits(codec, CS35L32_PWRCTL1,
				    CS35L32_PDN_ALL, 0);
	reg3 = snd_soc_read(codec, CS35L32_PWRCTL1);
	printk("PWRCTL1 REG = %x\n", reg3);
	return 0;
}

static const struct snd_soc_dapm_widget cs35l32_dapm_widgets[] = {
	
	SND_SOC_DAPM_SUPPLY("BOOST", CS35L32_PWRCTL1, 2, 1, NULL, 0),
	SND_SOC_DAPM_OUT_DRV_E("Speaker", CS35L32_PWRCTL1, 7, 1, NULL, 0, 
		int_clear, SND_SOC_DAPM_PRE_PMU),

	SND_SOC_DAPM_AIF_OUT("SDOUT", "Capture", 0, CS35L32_PWRCTL2, 3, 1),
	
	SND_SOC_DAPM_INPUT("VP"),
	SND_SOC_DAPM_INPUT("ISENSE"),
	SND_SOC_DAPM_INPUT("VSENSE"),

	SND_SOC_DAPM_SWITCH("VMON ADC", CS35L32_PWRCTL2, 7, 1, &vmon_ctl),
	SND_SOC_DAPM_SWITCH("IMON ADC", CS35L32_PWRCTL2, 6, 1, &imon_ctl),
	SND_SOC_DAPM_SWITCH("VPMON ADC", CS35L32_PWRCTL2, 5, 1, &vpmon_ctl), 
};

static const struct snd_soc_dapm_route cs35l32_audio_map[] = {
	
	{"Speaker", NULL, "BOOST"},
	
	{"VMON ADC", "Switch", "VSENSE"}, 
	{"IMON ADC", "Switch", "ISENSE"}, 
	{"VPMON ADC", "Switch", "VP"},
	
};

static int cs35l32_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned 
int fmt) {
	struct snd_soc_codec *codec = codec_dai->codec;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		snd_soc_update_bits(codec, CS35L32_ADSP_CTL, 
				    CS35L32_ADSP_MASTER_MASK, 1 << 6);
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		snd_soc_update_bits(codec, CS35L32_ADSP_CTL, 
				    CS35L32_ADSP_MASTER_MASK, 0 << 6); 
		break;
	default:
		return -EINVAL;
	}
	
	return 0;
}

static int cs35l32_set_tristate(struct snd_soc_dai *dai, int tristate) 
{
	struct snd_soc_codec *codec = dai->codec;
	
	return snd_soc_update_bits(codec, CS35L32_PWRCTL2,
					CS35L32_SDOUT_3ST, tristate << 3); 
}

static const struct snd_soc_dai_ops cs35l32_ops = {
	.set_fmt = cs35l32_set_dai_fmt,
	.set_tristate = cs35l32_set_tristate,
};

static struct snd_soc_dai_driver cs35l32_dai[] = {
	{
		.name = "cs35l32-monitor",
		.id = 0,
		.capture = {
			.stream_name = "Capture",
			.channels_min = 2,
			.channels_max = 2,
			.rates = CS35L32_RATES,
			.formats = CS35L32_FORMATS,
		},
		.ops = &cs35l32_ops,
		.symmetric_rates = 1,
	}
};

static int cs35l32_codec_set_sysclk(struct snd_soc_codec *codec,
			      int clk_id, int source, unsigned int freq, int dir) {
		
	switch (freq) {
	case CS35L32_MCLK_6MHZ:
		snd_soc_update_bits(codec, CS35L32_CLK_CTL, 
				    CS35L32_MCLK_DIV2_MASK, 0);
		snd_soc_update_bits(codec, CS35L32_CLK_CTL, 
				    CS35L32_MCLK_RATIO_MASK,
					CS35L32_MCLK_RATIO);
		break;
	case CS35L32_MCLK_12MHZ:
		snd_soc_update_bits(codec, CS35L32_CLK_CTL, 
				    CS35L32_MCLK_DIV2_MASK,
					CS35L32_MCLK_DIV2 << 6);
		snd_soc_update_bits(codec, CS35L32_CLK_CTL, 
				    CS35L32_MCLK_RATIO_MASK,
					CS35L32_MCLK_RATIO);
		break;
	case CS35L32_MCLK_6144MHZ:
		snd_soc_update_bits(codec, CS35L32_CLK_CTL, 
				    CS35L32_MCLK_DIV2_MASK, 0);
		snd_soc_update_bits(codec, CS35L32_CLK_CTL, 
				    CS35L32_MCLK_RATIO_MASK, 0);
		break;
	case CS35L32_MCLK_12288MHZ:
		snd_soc_update_bits(codec, CS35L32_CLK_CTL, 
				    CS35L32_MCLK_DIV2_MASK,
					CS35L32_MCLK_DIV2 << 6);
		snd_soc_update_bits(codec, CS35L32_CLK_CTL, 
				    CS35L32_MCLK_RATIO_MASK, 0);
		break;
	default:
		return -EINVAL;
	}
	
	return 0;
}

static int cs35l32_suspend(struct snd_soc_codec *codec) {
	return 0;
}

static int cs35l32_resume(struct snd_soc_codec *codec) {
	return 0;
}

static int cs35l32_probe(struct snd_soc_codec *codec) {
	int ret;
	struct cs35l32_private *cs35l32 = snd_soc_codec_get_drvdata(codec);

	codec->control_data = cs35l32->regmap;
	
	ret = snd_soc_codec_set_cache_io(codec, 8, 8, SND_SOC_REGMAP);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		return ret;
	}

	regcache_cache_only(cs35l32->regmap, false);

	/* Power down the AMP */
	snd_soc_update_bits(codec, CS35L32_PWRCTL1, CS35L32_PDN_AMP, 1);
	
	/* Setup ADSP Format Config */
	snd_soc_update_bits(codec, CS35L32_ADSP_CTL, CS35L32_ADSP_SHARE_MASK,
			    cs35l32->pdata.sdout_share << 3); 
	/* Setup ADSP Data Configuration */
	snd_soc_update_bits(codec, CS35L32_ADSP_CTL, CS35L32_ADSP_DATACFG_MASK,
			    cs35l32->pdata.sdout_datacfg << 4);

	/* Setup VBoost Manager  */
	snd_soc_update_bits(codec, CS35L32_AUDIO_LED_MNGR, CS35L32_BOOST_MASK,
			    cs35l32->pdata.boost_mng);

	/* Clear MCLK Error Bit since we don't have the clock yet */
	snd_soc_read(codec, CS35L32_INT_STATUS_1);
	
	return ret;
}

static int cs35l32_remove(struct snd_soc_codec *codec) {
	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_cs35l32 = {
	.probe = cs35l32_probe,
	.remove = cs35l32_remove,
	.suspend = cs35l32_suspend,
	.resume = cs35l32_resume,
	.set_sysclk = cs35l32_codec_set_sysclk,

	.dapm_widgets = cs35l32_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(cs35l32_dapm_widgets),
	.dapm_routes = cs35l32_audio_map,
	.num_dapm_routes = ARRAY_SIZE(cs35l32_audio_map),

	.controls = cs35l32_snd_controls,
	.num_controls = ARRAY_SIZE(cs35l32_snd_controls), };

/* Current and threshold powerup sequence Pg37 */ static const struct 
reg_default cs35l32_monitor_patch[] = {

	{ 0x00, 0x99 },
	{ 0x49, 0x56 },
	{ 0x43, 0x01 },
	{ 0x3B, 0x62 },
	{ 0x3C, 0x80 },
	{ 0x24, 0x40 },
	{ 0x00, 0x00 },	
};

static struct regmap_config cs35l32_regmap = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = CS35L32_MAX_REGISTER,
	.reg_defaults = cs35l32_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(cs35l32_reg_defaults),
	.volatile_reg = cs35l32_volatile_register,
	.readable_reg = cs35l32_readable_register,
	.cache_type = REGCACHE_RBTREE,
};

static __devinit int cs35l32_i2c_probe(struct i2c_client *i2c_client,
				       const struct i2c_device_id *id) {
	struct cs35l32_private *cs35l32;
	struct device_node *np = i2c_client->dev.of_node;
	u32 val32;
	int ret;
	unsigned int devid = 0;
	unsigned int reg;
	
	
	cs35l32 = devm_kzalloc(&i2c_client->dev, sizeof(struct cs35l32_private),
			       GFP_KERNEL);
	if (!cs35l32) {
		dev_err(&i2c_client->dev, "could not allocate codec\n");
		return -ENOMEM;
	}

	i2c_set_clientdata(i2c_client, cs35l32);

	cs35l32->regmap = devm_regmap_init_i2c(i2c_client, &cs35l32_regmap);
	if (IS_ERR(cs35l32->regmap)) {
		ret = PTR_ERR(cs35l32->regmap);
		dev_err(&i2c_client->dev, "regmap_init() failed: %d\n", ret);
		return ret;
	}

	if (of_property_read_u32(np, "cs35l32,boost-mng", &val32) >= 0)
		cs35l32->pdata.boost_mng = val32;
	
	if (of_property_read_u32(np, "cs35l32,sdout-datacfg", &val32) >= 0)
		cs35l32->pdata.sdout_datacfg = val32;
		
	if (of_property_read_u32(np, "cs35l32,sdout-share", &val32) >= 0)
		cs35l32->pdata.sdout_share = val32;

	cs35l32->pdata.gpio_nreset = of_get_named_gpio_flags(np,
								"cs35l32,gpio-nreset", 0, NULL);

	ret = gpio_request(cs35l32->pdata.gpio_nreset, "cs35l32 reset");
	if (ret < 0) {
		dev_err(&i2c_client->dev,
			 "failed to request the gpio %d\n",
				cs35l32->pdata.gpio_nreset);
		return ret;
	}
	/*bring the chip out of reset*/
	gpio_direction_output(cs35l32->pdata.gpio_nreset, 0);
	msleep(20);
	gpio_set_value_cansleep(cs35l32->pdata.gpio_nreset, 1);
	msleep(20);

	ret = regmap_register_patch(cs35l32->regmap, cs35l32_monitor_patch,
				    ARRAY_SIZE(cs35l32_monitor_patch));
	if (ret < 0) {
		dev_err(&i2c_client->dev, "failed to register regmap(try again : 0x40)\n");
		i2c_client->addr = 0x40;
		ret = regmap_register_patch(cs35l32->regmap, cs35l32_monitor_patch,
				    ARRAY_SIZE(cs35l32_monitor_patch));
		if (ret < 0)
			dev_err(&i2c_client->dev, "failed to register regmap retry again\n");
	}

	/* initialize codec */
	ret = regmap_read(cs35l32->regmap, CS35L32_DEVID_AB, &reg);
	devid = (reg & 0xFF) << 12;

	ret = regmap_read(cs35l32->regmap, CS35L32_DEVID_CD, &reg);
	devid |= (reg & 0xFF) << 4;

	ret = regmap_read(cs35l32->regmap, CS35L32_DEVID_E, &reg);
	devid |= (reg & 0xF0) >> 4;

	if (devid != CS35L32_CHIP_ID) {
		ret = -ENODEV;
		dev_err(&i2c_client->dev,
			"CS35L32 Device ID (%X). Expected %X\n",
			devid, CS35L32_CHIP_ID);
		return ret;
	}

	ret = regmap_read(cs35l32->regmap, CS35L32_REV_ID, &reg);
	if (ret < 0) {
		dev_err(&i2c_client->dev, "Get Revision ID failed\n");
		return ret;;
	}

	dev_info(&i2c_client->dev,
		 "Cirrus Logic CS35L32, Revision: %02X\n", reg & 0xFF);

	regcache_cache_only(cs35l32->regmap, false);

	ret =  snd_soc_register_codec(&i2c_client->dev,
			&soc_codec_dev_cs35l32, cs35l32_dai, ARRAY_SIZE(cs35l32_dai));
	
	if (ret < 0)
		return ret;
	return 0;
}

static __devexit int cs35l32_i2c_remove(struct i2c_client *client) {
	snd_soc_unregister_codec(&client->dev);
	return 0;
}

static const struct of_device_id cs35l32_of_match[] = {
	{ .compatible = "crus,cs35l32", },
	{},
};
MODULE_DEVICE_TABLE(of, cs35l32_of_match);


static const struct i2c_device_id cs35l32_id[] = {
	{"cs35l32", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, cs35l32_id);

static struct i2c_driver cs35l32_i2c_driver = {
	.driver = {
		   .name = "cs35l32",
		   .owner = THIS_MODULE,
		   .of_match_table = cs35l32_of_match,
		   },
	.id_table = cs35l32_id,
	.probe = cs35l32_i2c_probe,
	.remove = __devexit_p(cs35l32_i2c_remove),

};

module_i2c_driver(cs35l32_i2c_driver);

MODULE_DESCRIPTION("ASoC CS35L32 driver");
MODULE_AUTHOR("Brian Austin, Cirrus Logic Inc, <brian.austin@cirrus.com>");
MODULE_LICENSE("GPL");
