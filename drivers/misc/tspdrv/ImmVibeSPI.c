/*
** =========================================================================
** File:
**     ImmVibeSPI.c
**
** Description: 
**     Device-dependent functions called by Immersion TSP API
**     to control PWM duty cycle, amp enable/disable, save IVT file, etc...
**
** Portions Copyright (c) 2008-2010 Immersion Corporation. All Rights Reserved. 
**
** This file contains Original Code and/or Modifications of Original Code 
** as defined in and that are subject to the GNU Public License v2 - 
** (the 'License'). You may not use this file except in compliance with the 
** License. You should have received a copy of the GNU General Public License 
** along with this program; if not, write to the Free Software Foundation, Inc.,
** 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or contact 
** TouchSenseSales@immersion.com.
**
** The Original Code and all software distributed under the License are 
** distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER 
** EXPRESS OR IMPLIED, AND IMMERSION HEREBY DISCLAIMS ALL SUCH WARRANTIES, 
** INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY, FITNESS 
** FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT. Please see 
** the License for the specific language governing rights and limitations 
** under the License.
** =========================================================================
*/

/* Debug Mask setting */
#define VIBRATOR_DEBUG_PRINT   (0)
#define VIBRATOR_ERROR_PRINT   (1)
#define VIBRATOR_INFO_PRINT    (1)

#if (VIBRATOR_INFO_PRINT)
#define INFO_MSG(fmt, args...) \
			printk(KERN_INFO "vib: %s() " \
				fmt, __FUNCTION__, ##args);
#else
#define INFO_MSG(fmt, args...)
#endif

#if (VIBRATOR_DEBUG_PRINT)
#define DEBUG_MSG(fmt, args...) \
			printk(KERN_INFO "vib: %s() " \
				fmt, __FUNCTION__, ##args);
#else
#define DEBUG_MSG(fmt, args...)
#endif

#if (VIBRATOR_ERROR_PRINT)
#define ERR_MSG(fmt, args...) \
			printk(KERN_ERR "vib: %s() " \
				fmt, __FUNCTION__, ##args);
#else
#define ERR_MSG(fmt, args...)
#endif


/*USE THE QPNP-VIBRATOR START*/
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/of_device.h>
#include <linux/spmi.h>
#include <linux/qpnp/vibrator.h>
#include "../../staging/android/timed_output.h"

extern struct qpnp_vib *vib_dev;
#ifdef CONFIG_TSPDRV_PMIC_VIBRATOR
extern int qpnp_vib_set_with_vtglevel(struct qpnp_vib *vib, int vtglevel, int on);
#endif
/*USE THE QPNP-VIBRATOR END*/


/*USE THE SM100 START*/
#include <linux/types.h>
#include <linux/err.h>
#include <mach/msm_iomap.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <mach/gpiomux.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <mach/board_lge.h>


/* When use SM100 with GP_CLK
  175Hz motor : 22.4KHz - M=1, N=214 ,
  205Hz motor : 26.24Khz - M=1, N=183 ,
  230Hz motor : 29.4KHZ - M=1, N=163 ,
  */

#define DEVICE_NAME		"lge_sm100"

#define MMSS_CC_PWM_SET		0xFD8C3450
#define MMSS_CC_PWM_SIZE	SZ_1K

static void __iomem *virt_bases_v = NULL;
#define MMSS_CC_GP1_CMD_RCGR(x) (void __iomem *)(virt_bases_v + (x))

#define REG_WRITEL(value, reg)		writel(value, reg)
#define REG_READL(reg)			readl(reg)

#ifdef IMMVIBESPIAPI
#undef IMMVIBESPIAPI
#endif
#define IMMVIBESPIAPI static

/*
** This SPI supports only one actuator.
*/
#define NUM_ACTUATORS 1

#define PWM_DUTY_MAX    579 /* 13MHz / (579 + 1) = 22.4kHz */

static bool g_bAmpEnabled = false;


static struct clk *cam_gp1_clk;

static int mmss_cc_n_default;
static int mmss_cc_d_max;
static int mmss_cc_d_half;

#define PRE_FORCE_DEF	128
static int previous_nForce = PRE_FORCE_DEF;

IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpDisable(VibeUInt8 nActuatorIndex);

struct timed_vibrator_data {
	atomic_t gp1_clk_flag;
	int haptic_en_gpio;
	int motor_pwm_gpio;
	int vpwr_on;
	struct regulator *vreg_l21;
	int vibe_n_value;
    unsigned int clk_rate;

};
struct timed_vibrator_data vib;
static DEFINE_MUTEX(vib_lock);

bool sm100_flag = false; //default is QPNP(PMIC)
extern void touch_fops_init(void);

static int sm100_pwm_set(int enable, int amp)
{
	uint d_val;
	d_val = mmss_cc_d_half + (mmss_cc_n_default-1)*amp/256;

	if (virt_bases_v == NULL)
		virt_bases_v = ioremap(MMSS_CC_PWM_SET, MMSS_CC_PWM_SIZE);


	DEBUG_MSG("enable:%d, amp:%d, d:%d\n", enable, amp, d_val);

	if (enable) {
		REG_WRITEL(
			((~(d_val << 1)) & 0xffU),	/* D[7:0] */
			MMSS_CC_GP1_CMD_RCGR(0x10));
		REG_WRITEL(
			(1 << 1U) +	/* ROOT_EN[1] */
			(1),		/* UPDATE[0] */
			MMSS_CC_GP1_CMD_RCGR(0));
	} else {
		REG_WRITEL(
			(0 << 1U) +	/* ROOT_EN[1] */
			(0),		/* UPDATE[0] */
			MMSS_CC_GP1_CMD_RCGR(0));
	}
	return 0;
}

static int sm100_power_set(int enable, struct timed_vibrator_data *vib_data)
{
	int rc;

	DEBUG_MSG("pwr_enable=%d\n", enable);

	mutex_lock(&vib_lock);
	if (vib_data->vpwr_on != 1) {
		if (enable) {
			rc = regulator_enable(vib_data->vreg_l21);
			if (rc < 0)
				ERR_MSG("regulator_enable failed\n");
		} else {
			if (regulator_is_enabled(vib_data->vreg_l21) > 0) {
				rc = regulator_disable(vib_data->vreg_l21);
				if (rc < 0)
					ERR_MSG("regulator_disable failed\n");
			}
		}
	}
	mutex_unlock(&vib_lock);

	return 0;
}

static int sm100_ic_enable_set(int enable, struct timed_vibrator_data *vib_data)
{
	DEBUG_MSG("enable:%d\n", enable);

	if (enable)
		gpio_direction_output(vib_data->haptic_en_gpio, 1);
	else
		gpio_direction_output(vib_data->haptic_en_gpio, 0);

	return 0;
}

#ifdef CONFIG_OF
static void sm100_parse_dt(struct device *dev, struct timed_vibrator_data *vib_data)
{
	struct device_node *np = dev->of_node;

	of_property_read_u32(np, "syncoam,vpwr-on", &vib_data->vpwr_on);

	vib_data->haptic_en_gpio = of_get_named_gpio_flags(np, "syncoam,haptic-pwr-gpio", 0, NULL);
	vib_data->motor_pwm_gpio = of_get_named_gpio_flags(np, "syncoam,motor-pwm-gpio", 0, NULL);

	of_property_read_u32(np, "syncoam,n-value", &vib_data->vibe_n_value);
	of_property_read_u32(np, "syncoam,clk-rate", &vib_data->clk_rate);

	INFO_MSG("vpwr_on:%d, en_gpio:%d, pwm_gpio:%d, vibe_n_value:%d, clk_rate:%u\n",
           vib_data->vpwr_on,
		   vib_data->haptic_en_gpio, vib_data->motor_pwm_gpio,
		   vib_data->vibe_n_value, vib_data->clk_rate);
}

static struct of_device_id sm100_match_table[] = {
    { .compatible = "syncoam,sm100",},
    { },
};
#endif

static int sm100_probe(struct platform_device *pdev)
{
	int rc;
	INFO_MSG("\n");
	if (pdev->dev.of_node) {
		sm100_parse_dt(&pdev->dev, &vib);
	}

	if (vib.vpwr_on != 1) {
		if (!(vib.vreg_l21)) {
			vib.vreg_l21 = regulator_get(&pdev->dev, "vdd_ana");
			if (IS_ERR(vib.vreg_l21)) {
				ERR_MSG("regulator_get failed (%ld)\n", PTR_ERR(vib.vreg_l21));
				vib.vreg_l21 = NULL;
				return 0;
			}
		}
	}

	rc = gpio_request(vib.haptic_en_gpio, "lin_motor_en");
	if (rc) {
		ERR_MSG("haptic_en_gpio %d request failed\n", vib.haptic_en_gpio);
		return 0;
	}

	rc = gpio_request(vib.motor_pwm_gpio, "lin_motor_pwm");
	if (unlikely(rc < 0)) {
		ERR_MSG("not able to get gpio %d\n", vib.motor_pwm_gpio);
		return 0;
	}

	mmss_cc_n_default = vib.vibe_n_value;
	mmss_cc_d_max = mmss_cc_n_default;
	mmss_cc_d_half = (mmss_cc_n_default >> 1);

	pdev->dev.init_name = "vibrator";
	INFO_MSG("dev->init_name : %s, dev->kobj : %s\n", pdev->dev.init_name, pdev->dev.kobj.name);
	cam_gp1_clk = clk_get(&pdev->dev, "cam_gp1_clk");
	clk_set_rate(cam_gp1_clk, (unsigned long)vib.clk_rate);

	atomic_set(&vib.gp1_clk_flag, 0);

    sm100_flag = true;
	return 0;
}

static int sm100_remove(struct platform_device *pdev)
{
	sm100_flag = false;
	return 0;
}

static void sm100_shutdown(struct platform_device *pdev)
{
}

static int sm100_suspend(struct platform_device *pdev, pm_message_t state)
{
	if (g_bAmpEnabled)
		ImmVibeSPI_ForceOut_AmpDisable(0);
	return 0;
}

static int sm100_resume(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver sm100_driver = {
	.probe = sm100_probe,
	.remove = sm100_remove,
	.shutdown = sm100_shutdown,
	.suspend = sm100_suspend,
	.resume = sm100_resume,
	.driver = {
		.name = DEVICE_NAME,
#ifdef CONFIG_OF
		.of_match_table = sm100_match_table,
#endif
	},
};
/*USE THE SM100 END*/



/*
** Called to disable amp (disable output force)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpDisable(VibeUInt8 nActuatorIndex)
{
	printk("%s : g_bAmpEnabled:%d\n", __func__, g_bAmpEnabled);
    if (g_bAmpEnabled)
    {
		if(sm100_flag) {
	        sm100_ic_enable_set(0, &vib);
	        sm100_pwm_set(0, 0);
	        sm100_power_set(0, &vib);

			if (atomic_read(&vib.gp1_clk_flag) == 1) {
				clk_disable_unprepare(cam_gp1_clk);
				atomic_set(&vib.gp1_clk_flag, 0);
			}
		} else {
#ifdef CONFIG_TSPDRV_PMIC_VIBRATOR
			if(vib_dev != NULL)
				qpnp_vib_set_with_vtglevel(vib_dev, 0, false);
#endif
		}

		g_bAmpEnabled = false;
		previous_nForce = 0;
    }

    return VIBE_S_SUCCESS;
}

/*
** Called to enable amp (enable output force)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpEnable(VibeUInt8 nActuatorIndex, VibeInt8 nForce)
{
	printk("%s : g_bAmpEnabled:%d\n", __func__, g_bAmpEnabled);
    if (!g_bAmpEnabled)
    {
		if(sm100_flag) {
			if (atomic_read(&vib.gp1_clk_flag) == 0) {
				clk_prepare_enable(cam_gp1_clk);
				atomic_set(&vib.gp1_clk_flag, 1);
			}

			sm100_power_set(1, &vib);
			//sm100_pwm_set(1, 0); //MSM GP CLK update bit issue.
			sm100_ic_enable_set(1, &vib);

		}
        g_bAmpEnabled = true;
	 previous_nForce= PRE_FORCE_DEF;
    }

    return VIBE_S_SUCCESS;
}

/*
** Called at initialization time to set PWM freq, disable amp, etc...
*/

IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Initialize(void)
{

	int rc;
	rc = platform_driver_register(&sm100_driver);

    INFO_MSG("\n");

    g_bAmpEnabled = true;   /* to force ImmVibeSPI_ForceOut_AmpDisable disabling the amp */

    /* 
    ** Disable amp.
    ** If multiple actuators are supported, please make sure to call
    ** ImmVibeSPI_ForceOut_AmpDisable for each actuator (provide the actuator index as
    ** input argument).
    */
    ImmVibeSPI_ForceOut_AmpDisable(0);

	touch_fops_init();

    return VIBE_S_SUCCESS;
}

/*
** Called at termination time to set PWM freq, disable amp, etc...
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Terminate(void)
{
    INFO_MSG("\n");

    /* 
    ** Disable amp.
    ** If multiple actuators are supported, please make sure to call
    ** ImmVibeSPI_ForceOut_AmpDisable for each actuator (provide the actuator index as
    ** input argument).
    */
    ImmVibeSPI_ForceOut_AmpDisable(0);

    platform_driver_unregister(&sm100_driver);
    return VIBE_S_SUCCESS;
}

/*
** Called by the real-time loop to set PWM duty cycle
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetSamples(VibeUInt8 nActuatorIndex, VibeUInt16 nOutputSignalBitDepth, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer)
{
    VibeInt8 nForce;

//    g_bStarted = true;

    switch (nOutputSignalBitDepth)
    {
        case 8:
            /* pForceOutputBuffer is expected to contain 1 byte */
            if (nBufferSizeInBytes != 1) return VIBE_E_FAIL;

            nForce = pForceOutputBuffer[0];
            break;
        case 16:
            /* pForceOutputBuffer is expected to contain 2 byte */
            if (nBufferSizeInBytes != 2) return VIBE_E_FAIL;

            /* Map 16-bit value to 8-bit */
            nForce = ((VibeInt16*)pForceOutputBuffer)[0] >> 8;
            break;
        default:
            /* Unexpected bit depth */
            return VIBE_E_FAIL;
    }

	if(nForce == previous_nForce)
		return VIBE_S_SUCCESS;

	previous_nForce = nForce;

	if(IMMR_DEB)
		printk("[IMMR] Force set = %d\n", nForce);

	// nForce range: SM100: -127~127,  PMIC:0~127
    if (nForce <= 0)
    {      
		if(sm100_flag && nForce < 0)
		{
			sm100_pwm_set(1, nForce); //MSM GP CLK update bit issue.
		}
	    else ImmVibeSPI_ForceOut_AmpDisable(nActuatorIndex);
    }
    else
    {
        ImmVibeSPI_ForceOut_AmpEnable(nActuatorIndex, nForce);

		if(sm100_flag) {
	        sm100_pwm_set(1, nForce); //MSM GP CLK update bit issue.
		} else {
			if(vib_dev != NULL) {
#ifdef CONFIG_TSPDRV_PMIC_VIBRATOR
#if defined CONFIG_TSPDRV_3_0V_VIBRATOR
				qpnp_vib_set_with_vtglevel(vib_dev, (nForce * 31) / 128 + 1, true);
#elif defined CONFIG_TSPDRV_2_9V_VIBRATOR
				qpnp_vib_set_with_vtglevel(vib_dev, (nForce * 31) / 128 + 0, true);
#else
				qpnp_vib_set_with_vtglevel(vib_dev, (nForce * 31) / 128 + 3, true);
#endif
#endif
			}
		}
    }
    return VIBE_S_SUCCESS;
}

/*
** Called to set force output frequency parameters
*/
#if 0
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetFrequency(VibeUInt8 nActuatorIndex, VibeUInt16 nFrequencyParameterID, VibeUInt32 nFrequencyParameterValue)
{
    /* This function is not called for ERM device */

    return VIBE_S_SUCCESS;
}
#endif

/*
** Called to get the device name (device name must be returned as ANSI char)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_Device_GetName(VibeUInt8 nActuatorIndex, char *szDevName, int nSize)
{
#if 0   /* The following code is provided as a sample. Please modify as required. */
	INFO_MSG("\n");
    if ((!szDevName) || (nSize < 1)) return VIBE_E_FAIL;

    strncpy(szDevName, "W7", nSize-1);
    szDevName[nSize - 1] = '\0';    /* make sure the string is NULL terminated */
#endif

    return VIBE_S_SUCCESS;
}
