#include <linux/kernel.h>
#include <linux/string.h>

#include <mach/board_lge.h>

#include <linux/platform_device.h>
#include <linux/persistent_ram.h>
#include <asm/setup.h>
#include <asm/system_info.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/vmalloc.h>
#include <linux/memblock.h>
#ifdef CONFIG_LGE_HANDLE_PANIC
#include <mach/lge_handle_panic.h>
#endif

#ifdef CONFIG_LGE_PM
#include <linux/qpnp/qpnp-adc.h>
#include <mach/board_lge.h>
#include <linux/power_supply.h>
#endif

#if defined(CONFIG_LGE_PM_BATTERY_ID_CHECKER)
#include <linux/power/lge_battery_id.h>
#endif
#define PROP_VAL_MAX_SIZE 50

#ifdef CONFIG_ANDROID_PERSISTENT_RAM
#include <mach/msm_memtypes.h>
#endif

#ifdef CONFIG_LGE_QSDL_SUPPORT
#include <mach/lge_qsdl.h>
#endif

#ifdef CONFIG_USB_G_LGE_ANDROID
#include <linux/platform_data/lge_android_usb.h>
#endif
/* in drivers/staging/android */
#include "ram_console.h"

#if defined(CONFIG_LCD_KCAL)
/*             
                             
                                   
   */
#include <linux/module.h>
#include "../../../../drivers/video/msm/mdss/mdss_fb.h"
extern int update_preset_lcdc_lut(void);
#endif /* CONFIG_LCD_KCAL */

static int cn_arr_len = 3;

struct cn_prop {
	char *name;
	enum cn_prop_type type;
	uint32_t cell_u32;
	uint64_t cell_u64;
	char str[PROP_VAL_MAX_SIZE];
	uint8_t is_valid;
};

static struct cn_prop cn_array[] = {
	{
		.name = "lge,log_buffer_phy_addr",
		.type = CELL_U32,
	},
	{
		.name = "lge,sbl_delta_time",
		.type = CELL_U32,
	},
	{
		.name = "lge,lk_delta_time",
		.type = CELL_U64,
	},
};

int __init lge_init_dt_scan_chosen(unsigned long node, const char *uname,
								int depth, void *data)
{
	unsigned long len;
	int i;
	enum cn_prop_type type;
	char *p;
	uint32_t *u32;
	void *temp;

	if (depth != 1 || (strcmp(uname, "chosen") != 0
					   && strcmp(uname, "chosen@0") != 0))
		return 0;
	for (i = 0; i < cn_arr_len; i++) {
		type = cn_array[i].type;
		temp = of_get_flat_dt_prop(node, cn_array[i].name, &len);
		if (temp == NULL)
			continue;
		if (type == CELL_U32) {
			u32 = of_get_flat_dt_prop(node, cn_array[i].name, &len);
			if (u32 != NULL)
				cn_array[i].cell_u32 = of_read_ulong(u32, 1);
		} else if (type == CELL_U64) {
			u32 = of_get_flat_dt_prop(node, cn_array[i].name, &len);
			if (u32 != NULL)
				cn_array[i].cell_u64 = of_read_number(u32, 2);
		} else {
			p = of_get_flat_dt_prop(node, cn_array[i].name, &len);
			if (p != NULL)
				strlcpy(cn_array[i].str, p, len);
		}
		cn_array[i].is_valid = 1;
	}

	return 0;
}

void get_dt_cn_prop_u32(const char *name, uint32_t *u32)
{
	int i;
	for (i = 0; i < cn_arr_len; i++) {
		if (cn_array[i].is_valid &&
			!strcmp(name, cn_array[i].name)) {
			*u32 = cn_array[i].cell_u32;
			return;
		}
	}
	printk(KERN_ERR "The %s node have not property value\n", name);
}

void get_dt_cn_prop_u64(const char *name, uint64_t *u64)
{
	int i;
	for (i = 0; i < cn_arr_len; i++) {
		if (cn_array[i].is_valid &&
			!strcmp(name, cn_array[i].name)) {
			*u64 = cn_array[i].cell_u64;
			return;
		}
	}
	printk(KERN_ERR "The %s node have not property value\n", name);
}

void get_dt_cn_prop_str(const char *name, char *value)
{
	int i;
	for (i = 0; i < cn_arr_len; i++) {
		if (cn_array[i].is_valid &&
			!strcmp(name, cn_array[i].name)) {
			strlcpy(value, cn_array[i].str, strlen(cn_array[i].str));
			return;
		}
	}
	printk(KERN_ERR "The %s node have not property value\n", name);
}

#ifdef CONFIG_ANDROID_RAM_CONSOLE
static struct ram_console_platform_data ram_console_pdata = {
	.bootinfo = "UTS_VERSION\n",
};

static struct platform_device ram_console_device = {
	.name = "ram_console",
	.id = -1,
	.dev = {
		.platform_data = &ram_console_pdata,
	}
};
#endif /*CONFIG_ANDROID_RAM_CONSOLE*/

#ifdef CONFIG_PERSISTENT_TRACER
static struct platform_device persistent_trace_device = {
	.name = "persistent_trace",
	.id = -1,
};
#endif

#ifdef CONFIG_ANDROID_PERSISTENT_RAM
static struct persistent_ram_descriptor lge_pram_descs[] = {
#ifdef CONFIG_ANDROID_RAM_CONSOLE
	{
		.name = "ram_console",
		.size = LGE_RAM_CONSOLE_SIZE,
	},
#endif
#ifdef CONFIG_PERSISTENT_TRACER
	{
		.name = "persistent_trace",
		.size = LGE_RAM_CONSOLE_SIZE,
	},
#endif
};

static struct persistent_ram lge_persist_ram = {
	.size = LGE_PERSISTENT_RAM_SIZE,
	.num_descs = ARRAY_SIZE(lge_pram_descs),
	.descs = lge_pram_descs,
};

void __init lge_add_persist_ram_devices(void)
{
	int ret;
	struct memtype_reserve *mt = &reserve_info->memtype_reserve_table[MEMTYPE_EBI1];

	/* ram->start = 0x7D600000; */
	/* change to variable value to ram->start value */
	lge_persist_ram.start = mt->start - LGE_PERSISTENT_RAM_SIZE;
	pr_info("PERSIST RAM CONSOLE START ADDR : 0x%x\n", lge_persist_ram.start);

	ret = persistent_ram_early_init(&lge_persist_ram);
	if (ret) {
		pr_err("%s: failed to initialize persistent ram\n", __func__);
		return;
	}
}
#endif /*CONFIG_ANDROID_PERSISTENT_RAM*/

void __init lge_reserve(void)
{
#if defined(CONFIG_ANDROID_PERSISTENT_RAM)
	lge_add_persist_ram_devices();
#endif
}

void __init lge_add_persistent_device(void)
{
#ifdef CONFIG_ANDROID_RAM_CONSOLE
	platform_device_register(&ram_console_device);
#ifdef CONFIG_LGE_HANDLE_PANIC
	/* write ram console addr to imem */
	lge_set_ram_console_addr(lge_persist_ram.start,
			LGE_RAM_CONSOLE_SIZE);
#endif
#endif
#ifdef CONFIG_PERSISTENT_TRACER
	platform_device_register(&persistent_trace_device);
#endif

}



/*                                                              */
#ifdef CONFIG_LGE_PM
struct chg_cable_info_table {
	int threshhold;
	acc_cable_type type;
	unsigned ta_ma;
	unsigned usb_ma;
};


#ifdef CONFIG_LGE_QFPROM_INTERFACE
static struct platform_device qfprom_device = {
	.name = "lge-qfprom",
	.id = -1,
};

void __init lge_add_qfprom_devices(void)
{
	platform_device_register(&qfprom_device);
}
#endif
#define ADC_NO_INIT_CABLE   0
#define C_NO_INIT_TA_MA     0
#define C_NO_INIT_USB_MA    0
#define ADC_CABLE_NONE      1900000
#define C_NONE_TA_MA        700
#define C_NONE_USB_MA       500

#define MAX_CABLE_NUM		15
static bool cable_type_defined;
static struct chg_cable_info_table pm8941_acc_cable_type_data[MAX_CABLE_NUM];
#endif
/*                                        */
#ifdef CONFIG_LGE_DIAG_USB_ACCESS_LOCK
static struct platform_device lg_diag_cmd_device = {
	.name = "lg_diag_cmd",
	.id = -1,
	.dev    = {
		.platform_data = 0, /* &lg_diag_cmd_pdata */
	},
};

void __init lge_add_diag_devices(void)
{
	platform_device_register(&lg_diag_cmd_device);
}
#endif

/*                                                              */
#ifdef CONFIG_LGE_PM
void get_cable_data_from_dt(void *of_node)
{
	int i;
	u32 cable_value[3];
	struct device_node *node_temp = (struct device_node *)of_node;
	const char *propname[MAX_CABLE_NUM] = {
		"lge,no-init-cable",
		"lge,cable-mhl-1k",
		"lge,cable-u-28p7k",
		"lge,cable-28p7k",
		"lge,cable-56k",
		"lge,cable-100k",
		"lge,cable-130k",
		"lge,cable-180k",
		"lge,cable-200k",
		"lge,cable-220k",
		"lge,cable-270k",
		"lge,cable-330k",
		"lge,cable-620k",
		"lge,cable-910k",
		"lge,cable-none"
	};
	if (cable_type_defined) {
		pr_info("Cable type is already defined\n");
		return;
	}

	for (i = 0 ; i < MAX_CABLE_NUM ; i++) {
		of_property_read_u32_array(node_temp, propname[i],
				cable_value, 3);
		pm8941_acc_cable_type_data[i].threshhold = cable_value[0];
		pm8941_acc_cable_type_data[i].type = i;
		pm8941_acc_cable_type_data[i].ta_ma = cable_value[1];
		pm8941_acc_cable_type_data[i].usb_ma = cable_value[2];
	}
	cable_type_defined = 1;
}

int lge_pm_get_cable_info(struct qpnp_vadc_chip *vadc,
		struct chg_cable_info *cable_info)
{
	char *type_str[] = {
		"NOT INIT", "MHL 1K", "U_28P7K", "28P7K", "56K",
		"100K", "130K", "180K", "200K", "220K",
		"270K", "330K", "620K", "910K", "OPEN"
	};

	struct qpnp_vadc_result result;
	struct chg_cable_info *info = cable_info;
	struct chg_cable_info_table *table;

	int table_size = ARRAY_SIZE(pm8941_acc_cable_type_data);
	int acc_read_value = 0;
	int i, rc;
	int count = 1;

	if (!info) {
		printk(KERN_ERR "%s : invalid info parameters\n",
				__func__);
		return -1;
	}

	if (!vadc) {
		printk(KERN_ERR "%s : invalid vadc parameters\n",
				__func__);
		return -1;
	}

	if (!cable_type_defined) {
		printk(KERN_ERR "%s : cable type is not defined yet.\n",
				__func__);
		return -1;
	}

	for (i = 0; i < count; i++) {
		/* LIMIT: Include ONLY A1, B1, Vu3, Z models used MSM8974 AA/AB */
#ifdef CONFIG_ADC_READY_CHECK_JB
		rc = qpnp_vadc_is_ready();
		if (rc) {
			printk(KERN_INFO "%s is skipped once\n", __func__);
			continue;
		}
		rc = qpnp_vadc_read_lge(LR_MUX10_USB_ID_LV, &result);
#else
		rc = qpnp_vadc_read(vadc, LR_MUX10_USB_ID_LV, &result);
#endif
		if (rc < 0) {
			if (rc == -ETIMEDOUT) {
				/* reason: adc read timeout,
				 * assume it is open cable
				 */
				info->cable_type = CABLE_NONE;
				info->ta_ma = C_NONE_TA_MA;
				info->usb_ma = C_NONE_USB_MA;
			}

			printk(KERN_ERR "%s : adc read error - %d\n",
					__func__, rc);
			return rc;
		}

		acc_read_value = (int)result.physical;
		printk(KERN_ERR "%s : acc_read_value - %d\n",
				__func__, (int)result.physical);
		/* mdelay(10); */
	}

	info->cable_type = NO_INIT_CABLE;
	info->ta_ma = C_NO_INIT_TA_MA;
	info->usb_ma = C_NO_INIT_USB_MA;

	/* assume : adc value must be existed in ascending order */
	for (i = 0; i < table_size; i++) {
		table = &pm8941_acc_cable_type_data[i];

		if (acc_read_value <= table->threshhold) {
			info->cable_type = table->type;
			info->ta_ma = table->ta_ma;
			info->usb_ma = table->usb_ma;
			break;
		}
	}

	printk(KERN_ERR "\n\n[PM] Cable detected: %d(%s)(%d, %d)\n\n",
			acc_read_value, type_str[info->cable_type],
			info->ta_ma, info->usb_ma);
	return 0;
}

struct pseudo_batt_info_type pseudo_batt_info = {
	.mode = 0,
};

void pseudo_batt_set(struct pseudo_batt_info_type *info)
{
	struct power_supply *batt_psy;

	pr_err("pseudo_batt_set\n");

	batt_psy = power_supply_get_by_name("battery");

	if (!batt_psy) {
		pr_err("called before init\n");
		return;
	}

	pseudo_batt_info.mode = info->mode;
	pseudo_batt_info.id = info->id;
	pseudo_batt_info.therm = info->therm;
	pseudo_batt_info.temp = info->temp;
	pseudo_batt_info.volt = info->volt;
	pseudo_batt_info.capacity = info->capacity;
	pseudo_batt_info.charging = info->charging;

	power_supply_changed(batt_psy);
}

/* Belows are for using in interrupt context */
static struct chg_cable_info lge_cable_info;

acc_cable_type lge_pm_get_cable_type(void)
{
	return lge_cable_info.cable_type;
}

unsigned lge_pm_get_ta_current(void)
{
	return lge_cable_info.ta_ma;
}

unsigned lge_pm_get_usb_current(void)
{
	return lge_cable_info.usb_ma;
}

/* This must be invoked in process context */
void lge_pm_read_cable_info(struct qpnp_vadc_chip *vadc)
{
	lge_cable_info.cable_type = NO_INIT_CABLE;
	lge_cable_info.ta_ma = C_NO_INIT_TA_MA;
	lge_cable_info.usb_ma = C_NO_INIT_USB_MA;

	lge_pm_get_cable_info(vadc, &lge_cable_info);
}
#endif
/*                                                            */

#if defined(CONFIG_LGE_KSWITCH)
static int kswitch_status;
#endif

#ifdef CONFIG_EARJACK_DEBUGGER
static unsigned int uart_console_mode;  /* Not initialized */
#else
static unsigned int uart_console_mode = 1;  /* Alway Off */
#endif

unsigned int lge_get_uart_mode(void)
{
#ifdef CONFIG_LGE_KSWITCH
	if ((kswitch_status & LGE_KSWITCH_UART_DISABLE) >> 3)
		uart_console_mode = 0;
#endif
	return uart_console_mode;
}

void lge_set_uart_mode(unsigned int um)
{
	uart_console_mode = um;
}

static int __init lge_uart_mode(char *uart_mode)
{
	if (!strncmp("enable", uart_mode, 6)) {
		printk(KERN_INFO"UART CONSOLE : enable\n");
		lge_set_uart_mode((UART_MODE_ALWAYS_ON_BMSK | UART_MODE_EN_BMSK)
				& ~UART_MODE_ALWAYS_OFF_BMSK);
	} else if (!strncmp("detected", uart_mode, 8)) {
		printk(KERN_INFO"UART CONSOLE : detected\n");
		lge_set_uart_mode(UART_MODE_EN_BMSK & ~UART_MODE_ALWAYS_OFF_BMSK);
	} else {
		printk(KERN_INFO"UART CONSOLE : disable\n");
	}

	return 1;
}
__setup("uart_console=", lge_uart_mode);

/*
	for download complete using LAF image
	return value : 1 --> right after laf complete & reset
*/
#ifdef CONFIG_LGE_SUPPORT_LCD_MAKER_ID
/* get panel maker ID from cmdline */
static lcd_maker_id lge_panel_maker;

/* CAUTION : These strings are come from LK */
char *panel_maker[] = {"0", "1", "2"};

static int __init board_panel_maker(char *maker_id)
{
 int i;

 for (i = 0; i < LCD_MAKER_MAX; i++) {
 	if (!strncmp(maker_id, panel_maker[i], 1)) {
 		lge_panel_maker = (lcd_maker_id) i;
 		break;
 	}
 }

 printk(KERN_DEBUG "MAKER : %s\n", panel_maker[lge_panel_maker]);
 return 1;
}
__setup("lcd_maker_id=", board_panel_maker);

lcd_maker_id lge_get_panel_maker(void)
{
 return lge_panel_maker;
}
#endif

int android_dlcomplete;

int __init lge_android_dlcomplete(char *s)
{
	if (strncmp(s, "1", 1) == 0)   /* if same string */
		android_dlcomplete = 1;
	else	/* not same string */
		android_dlcomplete = 0;
	printk("androidboot.dlcomplete = %d\n", android_dlcomplete);

	return 1;
}
__setup("androidboot.dlcomplete=", lge_android_dlcomplete);

int lge_get_android_dlcomplete(void)
{
	return android_dlcomplete;
}
/* get boot mode information from cmdline.
 * If any boot mode is not specified,
 * boot mode is normal type.
 */
static enum lge_boot_mode_type lge_boot_mode = LGE_BOOT_MODE_NORMAL;
int __init lge_boot_mode_init(char *s)
{
	if (!strcmp(s, "charger"))
		lge_boot_mode = LGE_BOOT_MODE_CHARGER;
	else if (!strcmp(s, "chargerlogo"))
		lge_boot_mode = LGE_BOOT_MODE_CHARGERLOGO;
	else if (!strcmp(s, "qem_130k") || !strcmp(s, "factory"))
		lge_boot_mode = LGE_BOOT_MODE_FACTORY;
	else if (!strcmp(s, "qem_56k") || !strcmp(s, "factory2"))
		lge_boot_mode = LGE_BOOT_MODE_FACTORY2;
	else if (!strcmp(s, "qem_910k"))
		lge_boot_mode = LGE_BOOT_MODE_FACTORY3;
	else if (!strcmp(s, "pif_130k") || !strcmp(s, "pifboot"))
		lge_boot_mode = LGE_BOOT_MODE_PIFBOOT;
	else if (!strcmp(s, "pif_56k") || !strcmp(s, "pifboot2"))
		lge_boot_mode = LGE_BOOT_MODE_PIFBOOT2;
	else if (!strcmp(s, "pif_910k"))
		lge_boot_mode = LGE_BOOT_MODE_PIFBOOT3;
	printk("ANDROID BOOT MODE : %d %s\n", lge_boot_mode, s);
	/*                            */

	return 1;
}
__setup("androidboot.mode=", lge_boot_mode_init);

enum lge_boot_mode_type lge_get_boot_mode(void)
{
	return lge_boot_mode;
}

#ifdef CONFIG_MACH_MSM8974_G2_VZW
static int lge_battery_low = 0;
int __init lge_is_battery_low(char *status)
{

	if(!strcmp(status, "trickle"))
		lge_battery_low = 1;

	return 1;
	printk("charging status : %s/n", status);
}
__setup("is_battery_low=", lge_is_battery_low);

int lge_get_battery_low(void)
{
	return lge_battery_low;
}
#endif

int lge_get_factory_boot(void)
{
	int res;

	/*   if boot mode is factory,
	 *   cable must be factory cable.
	 */
	switch (lge_boot_mode) {
	case LGE_BOOT_MODE_FACTORY:
	case LGE_BOOT_MODE_FACTORY2:
	case LGE_BOOT_MODE_FACTORY3:
	case LGE_BOOT_MODE_PIFBOOT:
	case LGE_BOOT_MODE_PIFBOOT2:
	case LGE_BOOT_MODE_PIFBOOT3:
		res = 1;
		break;
	default:
		res = 0;
		break;
	}
	return res;
}

int lge_get_factory_cable(void)
{
	int res;

	switch (lge_cable_info.cable_type) {
	case CABLE_56K:
	case CABLE_130K:
	case CABLE_910K:
		res = 1;
		break;
	default:
		res = 0;
		break;
	}
	return res;
}

/* for board revision */
static hw_rev_type lge_bd_rev = HW_REV_1_0; /* HW_REV_B; */

/* CAUTION: These strings are come from LK. */
#if defined (CONFIG_MACH_MSM8974_G3_GLOBAL_COM)
char *rev_str[] = {"evb1", "evb2", "rev_a", "rev_a1", "rev_b", "rev_c", "rev_d",
	"rev_e", "rev_g", "rev_h", "rev_10", "rev_11", "rev_12",
	"reserved"};
#elif defined (CONFIG_MACH_MSM8974_G3_KDDI)
char *rev_str[] = {"evb1", "evb2", "rev_a", "rev_a1", "rev_b", "rev_c", "rev_d",
	"rev_e","rev_f", "rev_g", "rev_h", "rev_10", "rev_11", "rev_12",
	"reserved"};
#elif defined (CONFIG_MACH_MSM8974_DZNY_DCM)
char *rev_str[] = {"evb1", "evb2", "rev_a", "rev_b", "rev_c", "rev_d",
	"rev_e","rev_f", "rev_g", "rev_h", "rev_10", "rev_11", "rev_12",
	"reserved"};
#else
char *rev_str[] = {"evb1", "evb2", "rev_a", "rev_b", "rev_c", "rev_d",
	"rev_e", "rev_f", "rev_g", "rev_h", "rev_10", "rev_11", "rev_12",
	"reserved"};
#endif

static int __init board_revno_setup(char *rev_info)
{
	int i;

	for (i = 0; i < HW_REV_MAX; i++) {
		if (!strncmp(rev_info, rev_str[i], 6)) {
			lge_bd_rev = (hw_rev_type) i;
			/* it is defined externally in <asm/system_info.h> */
			system_rev = lge_bd_rev;
			break;
		}
	}

	printk(KERN_INFO "BOARD : LGE %s \n", rev_str[lge_bd_rev]);
	return 1;
}
__setup("lge.rev=", board_revno_setup);

hw_rev_type lge_get_board_revno(void)
{
    return lge_bd_rev;
}

#ifdef CONFIG_LGE_LCD_TUNING
static struct platform_device lcd_misc_device = {
	.name = "lcd_misc_msm",
	.id = 0,
};

void __init lge_add_lcd_misc_devices(void)
{
	platform_device_register(&lcd_misc_device);
}
#endif

#ifdef CONFIG_LCD_KCAL
/*             
                          
                                
*/
int g_kcal_r = 255;
int g_kcal_g = 255;
int g_kcal_b = 255;

int kcal_set_values(int kcal_r, int kcal_g, int kcal_b)
{
#if 0
	int is_update = 0;

	int kcal_r_limit = 250;
	int kcal_g_limit = 250;
	int kcal_b_limit = 253;

	g_kcal_r = kcal_r < kcal_r_limit ? kcal_r_limit : kcal_r;
	g_kcal_g = kcal_g < kcal_g_limit ? kcal_g_limit : kcal_g;
	g_kcal_b = kcal_b < kcal_b_limit ? kcal_b_limit : kcal_b;

	if (kcal_r < kcal_r_limit || kcal_g < kcal_g_limit || kcal_b < kcal_b_limit)
		is_update = 1;
	if (is_update)
		update_preset_lcdc_lut();
#else
	g_kcal_r = kcal_r;
	g_kcal_g = kcal_g;
	g_kcal_b = kcal_b;
#endif
	return 0;
}

static int kcal_get_values(int *kcal_r, int *kcal_g, int *kcal_b)
{
	*kcal_r = g_kcal_r;
	*kcal_g = g_kcal_g;
	*kcal_b = g_kcal_b;
	return 0;
}

static int kcal_refresh_values(void)
{
	return update_preset_lcdc_lut();
}

static struct kcal_platform_data kcal_pdata = {
	.set_values = kcal_set_values,
	.get_values = kcal_get_values,
	.refresh_display = kcal_refresh_values
};

static struct platform_device kcal_platrom_device = {
	.name   = "kcal_ctrl",
	.dev = {
		.platform_data = &kcal_pdata,
	}
};

static int __init display_kcal_setup(char *kcal)
{
	char vaild_k = 0;
	int kcal_r = 255;
	int kcal_g = 255;
	int kcal_b = 255;

	sscanf(kcal, "%d|%d|%d|%c", &kcal_r, &kcal_g, &kcal_b, &vaild_k);
	pr_info("kcal is %d|%d|%d|%c\n", kcal_r, kcal_g, kcal_b, vaild_k);

	if (vaild_k != 'K') {
		pr_info("kcal not calibrated yet : %d\n", vaild_k);
		kcal_r = kcal_g = kcal_b = 255;
	}

	kcal_set_values(kcal_r, kcal_g, kcal_b);
	return 1;
}
__setup("lge.kcal=", display_kcal_setup);

void __init lge_add_lcd_kcal_devices(void)
{
	pr_info(" KCAL_DEBUG : %s\n", __func__);
	platform_device_register(&kcal_platrom_device);
}
#endif

#if defined(CONFIG_LGE_PM_BATTERY_ID_CHECKER)
struct lge_battery_id_platform_data lge_battery_id_plat = {
	.id = 13,
	.pullup = 14,
};

static struct platform_device lge_battery_id_device = {
	.name   = "lge_battery_id",
	.id     = 0,
	.dev    = {
		.platform_data  = &lge_battery_id_plat,
	},
};

void __init lge_battery_id_devices(void)
{
	platform_device_register(&lge_battery_id_device);
}
#endif

static enum lge_laf_mode_type lge_laf_mode = LGE_LAF_MODE_NORMAL;

int __init lge_laf_mode_init(char *s)
{
    if (strcmp(s, "") && strcmp(s, "MID"))
        lge_laf_mode = LGE_LAF_MODE_LAF;

	return 1;
}
__setup("androidboot.laf=", lge_laf_mode_init);

enum lge_laf_mode_type lge_get_laf_mode(void)
{
	return lge_laf_mode;
}

#if defined(CONFIG_LGE_KSWITCH)
static int atoi(const char *name)
{
	int val = 0;

	for (;; name++) {
		switch (*name) {
		case '0' ... '9':
			val = 10*val+(*name-'0');
			break;
		default:
			return val;
		}
	}
}

static int __init kswitch_setup(char *value)
{
	kswitch_status = atoi(value);

	if (kswitch_status < 0)
		kswitch_status = 0;

	printk(KERN_INFO "[KSwitch] %d \n", kswitch_status);
	return 1;
}
__setup("kswitch=", kswitch_setup);

int lge_get_kswitch_status(void)
{
    return kswitch_status;
}
#endif

static int lge_boot_reason = -1; /* undefined for error checking */
static int __init lge_check_bootreason(char *reason)
{
	int ret = 0;

	/* handle corner case of kstrtoint */
	if (!strcmp(reason, "0xffffffff")) {
		lge_boot_reason = 0xffffffff;
		return 1;
	}

	ret = kstrtoint(reason, 16, &lge_boot_reason);
	if (!ret)
		printk(KERN_INFO "LGE REBOOT REASON: %x\n", lge_boot_reason);
	else
		printk(KERN_INFO "LGE REBOOT REASON: Couldn't get bootreason - %d\n",
				ret);

	return 1;
}
__setup("lge.bootreason=", lge_check_bootreason);

int lge_get_bootreason(void)
{
	return lge_boot_reason;
}

#ifdef CONFIG_LGE_QSDL_SUPPORT
static struct lge_qsdl_platform_data lge_qsdl_pdata = {
	.oneshot_read = 0,
	.using_uevent = 0
};

static struct platform_device lge_qsdl_device = {
	.name = LGE_QSDL_DEV_NAME,
	.id = -1,
	.dev = {
		.platform_data = &lge_qsdl_pdata,
	}
};

void __init lge_add_qsdl_device(void)
{
	platform_device_register(&lge_qsdl_device);
}
#endif /*                         */

#ifdef CONFIG_USB_G_LGE_ANDROID
static int get_factory_cable(void)
{
	int res;

	switch (lge_boot_mode) {
	case LGE_BOOT_MODE_FACTORY:
	case LGE_BOOT_MODE_PIFBOOT:
		res = LGEUSB_FACTORY_130K;
		break;
	case LGE_BOOT_MODE_FACTORY2:
	case LGE_BOOT_MODE_PIFBOOT2:
		res = LGEUSB_FACTORY_56K;
		break;
	case LGE_BOOT_MODE_FACTORY3:
	case LGE_BOOT_MODE_PIFBOOT3:
		res = LGEUSB_FACTORY_910K;
		break;
	default:
		res = 0;
		break;
	}
	return res;
}

struct lge_android_usb_platform_data lge_android_usb_pdata = {
	.vendor_id = 0x1004,
	.factory_pid = 0x6000,
	.iSerialNumber = 0,
	.product_name = "LGE Android Phone",
	.manufacturer_name = "LG Electronics Inc.",
	.factory_composition = "acm,diag",
	.get_factory_cable = get_factory_cable,
};

struct platform_device lge_android_usb_device = {
	.name = "lge_android_usb",
	.id = -1,
	.dev = {
		.platform_data = &lge_android_usb_pdata,
	},
};
void __init lge_add_android_usb_devices(void)
{
	platform_device_register(&lge_android_usb_device);
}
#endif
