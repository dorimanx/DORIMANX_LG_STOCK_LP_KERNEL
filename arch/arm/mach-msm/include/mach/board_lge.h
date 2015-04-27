#ifndef __ASM_ARCH_MSM_BOARD_LGE_H
#define __ASM_ARCH_MSM_BOARD_LGE_H

#if defined (CONFIG_MACH_MSM8974_G3_GLOBAL_COM)
typedef enum {
	HW_REV_EVB1 = 0,
	HW_REV_EVB2,
	HW_REV_A,
	HW_REV_A1,
	HW_REV_B,
	HW_REV_C,
	HW_REV_D,
	HW_REV_E,
	HW_REV_G,
	HW_REV_H,
	HW_REV_1_0,
	HW_REV_1_1,
	HW_REV_1_2,
	HW_REV_MAX
} hw_rev_type;
#elif defined (CONFIG_MACH_MSM8974_G3_KDDI)
typedef enum {
	HW_REV_EVB1 = 0,
	HW_REV_EVB2,
	HW_REV_A,
	HW_REV_A1,
	HW_REV_B,
	HW_REV_C,
	HW_REV_D,
	HW_REV_E,
	HW_REV_F,
	HW_REV_G,
	HW_REV_H,
	HW_REV_1_0,
	HW_REV_1_1,
	HW_REV_1_2,
	HW_REV_MAX
} hw_rev_type;
#elif defined (CONFIG_MACH_MSM8974_DZNY_DCM)
typedef enum {
	HW_REV_EVB1 = 0,
	HW_REV_EVB2,
	HW_REV_A,
	HW_REV_B,
	HW_REV_C,
	HW_REV_D,
	HW_REV_E,
	HW_REV_F,
	HW_REV_G,
	HW_REV_H,
	HW_REV_1_0,
	HW_REV_1_1,
	HW_REV_1_2,
	HW_REV_MAX
} hw_rev_type;
#else
typedef enum {
	HW_REV_EVB1 = 0,
	HW_REV_EVB2,
	HW_REV_A,
	HW_REV_B,
	HW_REV_C,
	HW_REV_D,
	HW_REV_E,
	HW_REV_F,
	HW_REV_G,
	HW_REV_H,
	HW_REV_1_0,
	HW_REV_1_1,
	HW_REV_1_2,
	HW_REV_MAX
} hw_rev_type;
#endif

extern char *rev_str[];

hw_rev_type lge_get_board_revno(void);

#ifdef CONFIG_LGE_PM
typedef enum {
	NO_INIT_CABLE = 0,
	CABLE_MHL_1K,
	CABLE_U_28P7K,
	CABLE_28P7K,
	CABLE_56K,
	CABLE_100K,
	CABLE_130K,
	CABLE_180K,
	CABLE_200K,
	CABLE_220K,
	CABLE_270K,
	CABLE_330K,
	CABLE_620K,
	CABLE_910K,
	CABLE_NONE
} acc_cable_type;

struct chg_cable_info {
	acc_cable_type cable_type;
	unsigned ta_ma;
	unsigned usb_ma;
};

void get_cable_data_from_dt(void *of_node);

struct qpnp_vadc_chip;
int lge_pm_get_cable_info(struct qpnp_vadc_chip *, struct chg_cable_info *);
void lge_pm_read_cable_info(struct qpnp_vadc_chip *);
acc_cable_type lge_pm_get_cable_type(void);
unsigned lge_pm_get_ta_current(void);
unsigned lge_pm_get_usb_current(void);
int lge_get_android_dlcomplete(void);

struct pseudo_batt_info_type {
	int mode;
	int id;
	int therm;
	int temp;
	int volt;
	int capacity;
	int charging;
};

struct pseudo_batt_info_type;
void pseudo_batt_set(struct pseudo_batt_info_type *);
#endif
#ifdef CONFIG_LGE_SUPPORT_LCD_MAKER_ID
typedef enum {
	LCD_RENESAS_LGD = 0,
	LCD_RENESAS_JDI,
	LCD_MAKER_MAX,
} lcd_maker_id;

typedef struct {
	lcd_maker_id maker_id;
	int min_mvol;
	int max_mvol;
} lcd_vol_maker_tbl_type;

lcd_maker_id lge_get_panel_maker(void);
#endif

enum lge_boot_mode_type {
	LGE_BOOT_MODE_NORMAL = 0,
	LGE_BOOT_MODE_CHARGER,
	LGE_BOOT_MODE_CHARGERLOGO,
	LGE_BOOT_MODE_FACTORY,
	LGE_BOOT_MODE_FACTORY2,
	LGE_BOOT_MODE_FACTORY3,
	LGE_BOOT_MODE_PIFBOOT,
	LGE_BOOT_MODE_PIFBOOT2,
	LGE_BOOT_MODE_PIFBOOT3,
};
enum lge_boot_mode_type lge_get_boot_mode(void);
int lge_get_factory_boot(void);
int lge_get_factory_cable(void);

#ifdef CONFIG_MACH_MSM8974_G2_VZW
int lge_get_battery_low(void);
#endif

#ifdef CONFIG_USB_G_LGE_ANDROID
void __init lge_add_android_usb_devices(void);
#endif

enum lge_laf_mode_type {
	LGE_LAF_MODE_NORMAL = 0,
	LGE_LAF_MODE_LAF,
};

enum lge_laf_mode_type lge_get_laf_mode(void);

enum cn_prop_type {
	CELL_U32 = 0,
	CELL_U64,
	STRING,
};

int __init lge_init_dt_scan_chosen(unsigned long node, const char *uname,
								  int depth, void *data);

void get_dt_cn_prop(const char *name, void *value);
void get_dt_cn_prop_str(const char *name, char *value);
void get_dt_cn_prop_u64(const char *name, uint64_t *u64);
void get_dt_cn_prop_u32(const char *name, uint32_t *u32);

extern bool lge_get_cont_splash_enabled(void);

#define UART_MODE_ALWAYS_OFF_BMSK   BIT(0)
#define UART_MODE_ALWAYS_ON_BMSK    BIT(1)
#define UART_MODE_INIT_BMSK         BIT(2)
#define UART_MODE_EN_BMSK           BIT(3)

extern unsigned int lge_get_uart_mode(void);
extern void lge_set_uart_mode(unsigned int um);

#ifdef CONFIG_ANDROID_PERSISTENT_RAM
#define LGE_PERSISTENT_RAM_SIZE (SZ_1M)
#endif

extern int lge_get_bootreason(void);

void xo_therm_logging(void);

#if defined(CONFIG_ANDROID_RAM_CONSOLE)
#define LGE_RAM_CONSOLE_SIZE (128 * SZ_1K * 2)
#endif

void __init lge_reserve(void);
void __init lge_add_persistent_device(void);


#if defined(CONFIG_ANDROID_PERSISTENT_RAM)
void __init lge_add_persist_ram_devices(void);
#endif

#ifdef CONFIG_LGE_LCD_TUNING
void __init lge_add_lcd_misc_devices(void);
#endif

int gpio_debug_init(void);
void gpio_debug_print(void);

#ifdef CONFIG_LGE_QFPROM_INTERFACE
void __init lge_add_qfprom_devices(void);
#endif

#ifdef CONFIG_LGE_DIAG_USB_ACCESS_LOCK
void __init lge_add_diag_devices(void);
#endif
#if defined(CONFIG_LGE_PM_BATTERY_ID_CHECKER)
void __init lge_battery_id_devices(void);
#endif

#if defined(CONFIG_LGE_KSWITCH)
#define LGE_KSWITCH_UART_DISABLE     0x1 << 3
int lge_get_kswitch_status(void);
#endif

#ifdef CONFIG_LGE_QSDL_SUPPORT
void __init lge_add_qsdl_device(void);
#endif

#endif
