#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/debugfs.h>
#include <linux/jiffies.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/mfd/pm8xxx/pm8xxx-adc.h>
#include <linux/qpnp/qpnp-adc.h>
#include <mach/gpiomux.h>
#include "tpa2015d1.h"


#define EXT_CLASS_D_EN 13000
#define EXT_CLASS_D_DIS 3000
#define EXT_CLASS_D_DELTA 2000

struct tpa2015d1_info {
	unsigned int gpio_spkamp_en;
	unsigned char *pdev_name;
};

static struct tpa2015d1_info *hi = NULL;

void tpa2015d1_ext_spk_power_amp_enable(u32 on)
{
	unsigned int ext_spk_amp_gpio = 0;

	pr_info("%s: enter- amp on:%d\n",__func__,on);

	if (hi == NULL)
		return;
	else
		ext_spk_amp_gpio = hi->gpio_spkamp_en;

	if (on) {
		gpio_direction_output(ext_spk_amp_gpio, on);
		/*time takes enable the external power amplifier*/
		usleep_range(EXT_CLASS_D_EN,
				EXT_CLASS_D_EN + EXT_CLASS_D_DELTA);
	} else {
		gpio_direction_output(ext_spk_amp_gpio, on);
		/*time takes disable the external power amplifier*/
		usleep_range(EXT_CLASS_D_DIS,
				EXT_CLASS_D_DIS + EXT_CLASS_D_DELTA);
	}

	pr_debug("%s: %s external speaker PAs.\n", __func__,
			on ? "Enable" : "Disable");
}

static void tpa2015d1_parse_dt(struct device *dev,
		struct tpa2015d1_platform_data *pdata)
{
	struct device_node *np = dev->of_node;

	pdata->gpio_spkamp_en = of_get_named_gpio_flags(np,
			"tpa2015d1,gpio_spkamp_en", 0, NULL);
}

static int tpa2015d1_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct tpa2015d1_platform_data *pdata = NULL;

	pr_info("%s enter\n",__func__);

	hi = kzalloc(sizeof(struct tpa2015d1_info), GFP_KERNEL);

	if (hi == NULL) {
		pr_err("Failed to allloate amp info\n");
		return -ENOMEM;
	}

	if (pdev->dev.of_node){
		pdata = devm_kzalloc(&pdev->dev,
				sizeof(struct tpa2015d1_platform_data),
				GFP_KERNEL);
		if(!pdata){
			pr_err("Failed to allocate memory\n");
			return -ENOMEM;
		}
		pdev->dev.platform_data = pdata;

		tpa2015d1_parse_dt(&pdev->dev, pdata);
	} else {
		pdata = devm_kzalloc(&pdev->dev,
				sizeof(struct tpa2015d1_platform_data),
				GFP_KERNEL);
		if(!pdata){
			pr_err("Failed to allocate memory\n");
			return -ENOMEM;
		} else {
			pdev->dev.platform_data = pdata;
		}
	}

	hi->gpio_spkamp_en = pdata->gpio_spkamp_en;


	ret = gpio_request(hi->gpio_spkamp_en, "gpio_detect");
	if (ret < 0) {
		pr_err("Failed to configure gpio%d (gpio_spkamp_en) "
				"gpio_request\n", hi->gpio_spkamp_en);
		goto error_gpio;
	}

	ret = gpio_direction_input(hi->gpio_spkamp_en);
	if (ret < 0) {
		pr_err("Failed to configure gpio%d (gpio_spkamp_en) "
				"gpio_direction_input\n", hi->gpio_spkamp_en);
		goto error;
	}

	return ret;

error:
	gpio_free(hi->gpio_spkamp_en);
error_gpio:
	kfree(hi);

	return ret;
}

static int tpa2015d1_remove(struct platform_device *pdev)
{
	pr_info("%s enter\n",__func__);

	gpio_free(hi->gpio_spkamp_en);
	kfree(hi);
	return 0;
}

static struct of_device_id tpa2015d1_match_table[] = {
	{ .compatible = "ti,tpa2015d1",},
	{},
};

static struct platform_driver ext_amp_driver = {
	.probe          = tpa2015d1_probe,
	.remove         = tpa2015d1_remove,
	.driver         = {
		.name           = "tpa2015d1",
		.owner          = THIS_MODULE,
		.of_match_table = tpa2015d1_match_table,
	},
};

static int __init ext_amp_init(void)
{
	int ret;

	pr_info("%s enter\n",__func__);
	ret = platform_driver_register(&ext_amp_driver);
	if (ret) {
		pr_err("Fail to register platform driver\n");
	}

	return ret;
}

static void __exit ext_amp_exit(void)
{
	platform_driver_unregister(&ext_amp_driver);
	pr_info("%s exit\n",__func__);
}

late_initcall_sync(ext_amp_init);
module_exit(ext_amp_exit);
