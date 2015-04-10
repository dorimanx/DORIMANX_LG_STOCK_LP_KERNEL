/*
 * Maxim MAX8971 Charger Driver
 * 1.55A Switch-Mode Li+/LiPoly Charger in Small 4x5 WLP 0.4mm
 *
 * Copyright (C) 2013 Maxim Integrated Product
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#if 0
#define DEBUG
#define VERBOSE_DEBUG
#endif
#define log_level	0
#define log_worker 0

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
#include <linux/of_irq.h>
#include <linux/of_gpio.h>

#include <linux/power_supply.h>
#include <linux/power/max8971.h>

#define DRIVER_DESC	"MAX8971 Charger Driver"
#define DRIVER_NAME	MAX8971_NAME
#define DRIVER_VERSION "7.2"
#define DRIVER_AUTHOR	"Clark Kim <clark.kim@maxim-ic.com>"

#if defined(CONFIG_LGE_PM)
/* LGE specific */
#include <mach/board_lge.h>
#include <linux/max17048_battery.h>
#include <linux/reboot.h>
#include <linux/delay.h>
#endif

#define WORKQUEUE_USE_ISR

#define IRQ_WORK_DELAY				0
#define IRQ_WORK_INTERVAL			 msecs_to_jiffies(5000)
#define LOG_WORK_INTERVAL			 msecs_to_jiffies(10000)

#define VERIFICATION_UNLOCK		 0
#define I2C_SUSPEND_WORKAROUND (1)
#if I2C_SUSPEND_WORKAROUND
extern bool i2c_suspended;
#endif

/* define register map */
#define CHGINT					0x0F
#define CHGINT_MSK				0x01
#define CHGINT_AICL				BIT(7)
#define CHGINT_TOPOFF				BIT(6)
#define CHGINT_DC_OVP				BIT(5)
#define CHGINT_DC_UVP				BIT(4)
#define CHGINT_CHG				BIT(3)
#define CHGINT_BAT				BIT(2)
#define CHGINT_THM				BIT(1)
#define CHGINT_PWRUP				BIT(0)
#define CHG_STAT				0x02
#define DETAILS1				0x03
#define DETAILS1_DC_V				BIT(7)
#define DETAILS1_DC_I				BIT(6)
#define DETAILS1_DC_OVP			 	BIT(5)
#define DETAILS1_DC_UVP			 	BIT(4)
#define DETAILS1_THM_DTLS			BITS(2, 0)
#define DETAILS2				0x04
#define DETAILS2_BAT_DTLS			BITS(5, 4)
#define DETAILS2_CHG_DTLS			BITS(3, 0)
#define CHGCNTL1				0x05
#define CHGCNTL1_DCMON_DIS			BIT(1)
#define CHGCNTL1_USB_SUS			BIT(0)
#define FCHGCRNT				0x06
#define FCHGCRNT_FCHGT				BITS(7, 5)
#define FCHGCRNT_CHGCC				BITS(4, 0)
#define DCCRNT					0x07
#define DCCRNT_CHGRSTRT				BIT(6)
#define DCCRNT_DCILMT				BITS(5, 0)
#define TOPOFF					0x08
#define TOPOFF_TOFFT				BITS(7, 5)
#define TOPOFF_IFST2P8				BIT(4)
#define TOPOFF_TOFFS				BITS(3, 2)
#define TOPOFF_CHGCV				BITS(1, 0)
#define TEMPREG					0x09
#define TEMPREG_RTEMP				BITS(7, 6)
#define TEMPREG_THM_CNFG			BIT(5)
#define TEMPREG_THMRES				BITS(2, 1)
#define TEMPREG_SAFETYREG			BIT(0)
#define PROTCMD					0x0A
#define PROTCMD_CHGPROT				BITS(3, 2)

/*******************************************************************************
 * Useful Macros
 ******************************************************************************/

#undef	__CONST_FFS
#define __CONST_FFS(_x) \
	((_x) & 0x0F ? ((_x) & 0x03 ? ((_x) & 0x01 ? 0 : 1) :\
	((_x) & 0x04 ? 2 : 3)) :\
	((_x) & 0x30 ? ((_x) & 0x10 ? 4 : 5) :\
	((_x) & 0x40 ? 6 : 7)))

#undef	FFS
#if 0
#define FFS(_x) \
	((_x) ? (__builtin_constant_p(_x) ? __CONST_FFS(_x) : __ffs(_x)) : 0)
#else
#define FFS(_x) \
		((_x) ? __CONST_FFS(_x) : 0)
#endif

#undef	BIT_RSVD
#define BIT_RSVD	0

#undef	BITS
#define BITS(_end, _start) \
		((BIT(_end) - BIT(_start)) + BIT(_end))

#undef	__BITS_GET
#define __BITS_GET(_word, _mask, _shift) \
		(((_word) & (_mask)) >> (_shift))

#undef	BITS_GET
#define BITS_GET(_word, _bit) \
		__BITS_GET(_word, _bit, FFS(_bit))

#undef	__BITS_SET
#define __BITS_SET(_word, _mask, _shift, _val) \
		(((_word) & ~(_mask)) | (((_val) << (_shift)) & (_mask)))

#undef	BITS_SET
#define BITS_SET(_word, _bit, _val) \
		__BITS_SET(_word, _bit, FFS(_bit), _val)

#undef	BITS_MATCH
#define BITS_MATCH(_word, _bit) \
		(((_word) & (_bit)) == (_bit))

/*******************************************************************************
 * Chip IO
 ******************************************************************************/

struct max8971_io {
	struct regmap *regmap;
};

static __always_inline int max8971_read(struct max8971_io *io,
	u8 addr, u8 *val)
{
	unsigned int buf = 0;
	int rc = regmap_read(io->regmap, (unsigned int)addr, &buf);

	if (likely(!IS_ERR_VALUE(rc)))
		*val = (u8)buf;
	return rc;
}

static __always_inline int max8971_write(struct max8971_io *io,
	u8 addr, u8 val)
{
	unsigned int buf = (unsigned int)val;
	return regmap_write(io->regmap, (unsigned int)addr, buf);
}

static inline int max8971_masked_read(struct max8971_io *io,
	u8 addr, u8 mask, u8 shift, u8 *val)
{
	u8 buf = 0;
	int rc;

	if (unlikely(!mask)) {
		/* no actual access */
		*val = 0;
		rc	 = 0;
		goto out;
	}

	rc = max8971_read(io, addr, &buf);
	if (likely(!IS_ERR_VALUE(rc)))
		*val = __BITS_GET(buf, mask, shift);

out:
	return rc;
}

static inline int max8971_masked_write(struct max8971_io *io,
	u8 addr, u8 mask, u8 shift, u8 val)
{
	u8 buf = 0;
	int rc;

	if (unlikely(!mask)) {
		/* no actual access */
		rc = 0;
		goto out;
	}

	rc = max8971_read(io, addr, &buf);
	if (likely(!IS_ERR_VALUE(rc)))
		rc = max8971_write(io, addr, __BITS_SET(buf, mask, shift, val));

out:
	return rc;
}

static __always_inline int max8971_bulk_read(struct max8971_io *io,
	u8 addr, u8 *dst, u16 len)
{
	return regmap_bulk_read(io->regmap,
		(unsigned int)addr, dst, (size_t)len);
}

static __always_inline int max8971_bulk_write(struct max8971_io *io,
	u8 addr, const u8 *src, u16 len)
{
	return regmap_bulk_write(io->regmap,
		(unsigned int)addr, src, (size_t)len);
}

/*** Simplifying bitwise configurations for individual subdevices drivers ***/
#ifndef MAX8971_REG_ADDR_INVALID
#define MAX8971_REG_ADDR_INVALID	0x00
#endif

struct max8971_bitdesc {
	u8 reg, mask, shift;
};

#define MAX8971_BITDESC(_reg, _bit) \
		{ .reg = _reg, .mask = _bit, .shift = (u8)FFS(_bit), }

#define MAX8971_BITDESC_INVALID \
		MAX8971_BITDESC(MAX8971_REG_ADDR_INVALID, BIT_RSVD)

#define MAX8971_BITDESC_PER_BIT(_reg, _index, _per, _bitsz) \
		MAX8971_BITDESC((_reg) + ((_index) / (_per)),\
		BITS((_bitsz) - 1, 0) << (((_index) % (_per)) * (_bitsz)))

#define MAX8971_BITDESC_PER_1BIT(_reg, _index) \
		MAX8971_BITDESC_PER_BIT(_reg, _index, 8, 1)
#define MAX8971_BITDESC_PER_2BIT(_reg, _index) \
		MAX8971_BITDESC_PER_BIT(_reg, _index, 4, 2)
#define MAX8971_BITDESC_PER_4BIT(_reg, _index) \
		MAX8971_BITDESC_PER_BIT(_reg, _index, 2, 4)

#define is_valid_max8971_bitdesc(_bitdesc) \
		((_bitdesc)->mask != BIT_RSVD &&\
		(_bitdesc)->reg	!= MAX8971_REG_ADDR_INVALID)

static __always_inline int max8971_read_bitdesc(struct max8971_io *io,
	const struct max8971_bitdesc *desc, u8 *val)
{
	return max8971_masked_read(io, desc->reg,
		desc->mask, desc->shift, val);
}

static __always_inline int max8971_write_bitdesc(struct max8971_io *io,
	const struct max8971_bitdesc *desc, u8 val)
{
	return max8971_masked_write(io, desc->reg,
		desc->mask, desc->shift, val);
}

#define max8971_read_reg_bit(_io, _reg, _bit, _val_ptr) \
		max8971_masked_read(_io, _reg, _reg##_##_bit,\
			(u8)FFS(_reg##_##_bit), _val_ptr)

#define max8971_write_reg_bit(_io, _reg, _bit, _val) \
		max8971_masked_write(_io, _reg, _reg##_##_bit,\
			(u8)FFS(_reg##_##_bit), _val)

/*******************************************************************************
 * Debugging Stuff
 ******************************************************************************/

#undef	log_fmt
#define log_fmt(format) \
		DRIVER_NAME ": " format
#undef	log_err
#define log_err(format, ...) \
		printk(KERN_ERR log_fmt(format), ##__VA_ARGS__)
#undef	log_warn
#define log_warn(format, ...) \
		printk(KERN_WARNING log_fmt(format), ##__VA_ARGS__)
#undef	log_info
#define log_info(format, ...) \
		if (likely(log_level >= 0)) {\
			printk(KERN_INFO log_fmt(format), ##__VA_ARGS__);\
		}
#undef	log_dbg
#define log_dbg(format, ...) \
		if (likely(log_level >= 1)) {\
			printk(KERN_DEFAULT log_fmt(format), ##__VA_ARGS__);\
		}
#undef	log_vdbg
#define log_vdbg(format, ...) \
		if (likely(log_level >= 2)) {\
			printk(KERN_DEFAULT log_fmt(format), ##__VA_ARGS__);\
		}

/*******************************************************************************
 * Driver Context
 ******************************************************************************/

struct max8971 {
	struct mutex				lock;
	struct max8971_io			io;
	struct device				*dev;
	struct kobject				*kobj;
	struct attribute_group		 	*attr_grp;
	struct max8971_platform_data		*pdata;
	int					irq;
	int					irq_gpio;
	u8					irq_saved;
	spinlock_t				irq_lock;
#ifdef WORKQUEUE_USE_ISR
	struct delayed_work			irq_work;
#endif
	struct delayed_work			log_work;
	struct power_supply			psy;
	struct power_supply			*psy_this;
	struct power_supply			*psy_ext;
	struct power_supply			*psy_coop;
	bool					dev_enabled;
	bool					dev_initialized;
	int					current_limit_volatile;
	int					current_limit_permanent;
	int					charge_current_volatile;
	int					charge_current_permanent;
	int					present;
	int					health;
	int					status;
	int					charge_type;
	struct power_supply			*batt;
	int					te;
#if I2C_SUSPEND_WORKAROUND
	struct delayed_work 	check_suspended_work;
	atomic_t		suspended;
	u8 irq1_current;
	u8 irq2_current;
	u8 irq1_mask;
	bool irq_disabled;
#endif
};

#if defined(CONFIG_LGE_PM)
#define __lock(_me)	{}
#define __unlock(_me)	{}
#else
#define __lock(_me)	mutex_lock(&(_me)->lock)
#define __unlock(_me)	mutex_unlock(&(_me)->lock)
#endif

enum {
	DC_V_VALID	= 0,
	DC_V_INVALID	= 1,
};

static char *max8971_dcv_details[] = {
	[DC_V_VALID]	= "VDC is valid; VDC > 4.5V",
	[DC_V_INVALID]	= "VDC is in AICL mode; 3V < VDC < 4.5V",
};

enum {
	DC_I_VALID	= 0,
	DC_I_INVALID	= 1,
};

static char *max8971_dci_details[] = {
	[DC_I_VALID]	= "IDC is valid; IDC < DCILMT",
	[DC_I_INVALID]	= "IDC is invalid; IDC > DCILMT",
};

enum {
	DC_OVP_VALID	= 0,
	DC_OVP_INVALID	= 1,
};

static char *max8971_dcovp_details[] = {
	[DC_OVP_VALID]		= "VDC is valid; VDC < VDC_OVLO",
	[DC_OVP_INVALID]	= "VDC is invalid; VDC > VDC_OVLO",
};

enum {
	DC_UVP_INVALID	= 0,
	DC_UVP_VALID	= 1,
};

static char *max8971_dcuvp_details[] = {
	[DC_UVP_INVALID] = "VDC is invalid; VDC < VDC_UVLO",
	[DC_UVP_VALID]	 = "VDC is valid; VDC > VDC_UVLO",
};

enum {
	THM_DTLS_LOW_TEMP_SUSPEND	= 0b001,
	THM_DTLS_LOW_TEMP_CHARGING	= 0b010,
	THM_DTLS_STD_TEMP_CHARGING	= 0b011,
	THM_DTLS_HIGH_TEMP_CHARGING	= 0b100,
	THM_DTLS_HIGH_TEMP_SUSPEND	= 0b101,
};

static char *max8971_thm_details[] = {
	[THM_DTLS_LOW_TEMP_SUSPEND]	= "cold; T < T1",
	[THM_DTLS_LOW_TEMP_CHARGING]	= "cool; T1 < T < T2",
	[THM_DTLS_STD_TEMP_CHARGING]	= "normal; T2 < T < T3",
	[THM_DTLS_HIGH_TEMP_CHARGING]	= "warm; T3 < T < T4",
	[THM_DTLS_HIGH_TEMP_SUSPEND]	= "hot; T4 < T",
};

enum {
	BAT_DTLS_UVP	 = 0b00,
	BAT_DTLS_TIMEOUT = 0b01,
	BAT_DTLS_OK		= 0b10,
	BAT_DTLS_OVP	 = 0b11,
};

static char *max8971_bat_details[] = {
	[BAT_DTLS_UVP]	 = "battery voltage < 2.1V",
	[BAT_DTLS_TIMEOUT] = "timer fault",
	[BAT_DTLS_OK]		= "battery okay",
	[BAT_DTLS_OVP]	 = "battery overvoltage",
};

enum {
	CHG_DTLS_DEAD_BATTERY	= 0b0000,
	CHG_DTLS_PRECHARGE	= 0b0001,
	CHG_DTLS_FASTCHARGE_CC	= 0b0010,
	CHG_DTLS_FASTCHARGE_CV	= 0b0011,
	CHG_DTLS_TOPOFF		= 0b0100,
	CHG_DTLS_DONE		= 0b0101,
	CHG_DTLS_TIMER_FAULT	= 0b0110,
	CHG_DTLS_TEMP_SUSPEND	= 0b0111,
	CHG_DTLS_OFF		= 0b1000,
	CHG_DTLS_THM_LOOP	= 0b1001,
};

static char *max8971_chg_details[] = {
	[CHG_DTLS_DEAD_BATTERY] =
		"charger is in dead-battery region",
	[CHG_DTLS_PRECHARGE] =
		"charger is in precharge mode",
	[CHG_DTLS_FASTCHARGE_CC] =
		"charger is in fast-charge constant current mode",
	[CHG_DTLS_FASTCHARGE_CV] =
		"charger is in fast-charge constant voltage mode",
	[CHG_DTLS_TOPOFF] =
		"charger is in top-off mode",
	[CHG_DTLS_DONE] =
		"charger is in done mode",
	[CHG_DTLS_TIMER_FAULT] =
		"charger is in timer fault mode",
	[CHG_DTLS_TEMP_SUSPEND] =
		"charger is in temperature suspend mode",
	[CHG_DTLS_OFF] =
		"charger is off",
	[CHG_DTLS_THM_LOOP] =
		"charger is operating with its thermal loop active",
};

#define max8971_read_irq_status(_me) \
	({\
		u8 __irq_current = 0;\
		int __rc = max8971_read(&(_me)->io, CHGINT, &__irq_current);\
		if (unlikely(IS_ERR_VALUE(__rc))) {\
			log_err("CHGINT read error [%d]\n", __rc);\
			__irq_current = 0;\
		} \
		__irq_current;\
	})

enum {
	CFG_CHGPROT = 0,
	CFG_USB_SUS,
	CFG_DCMON_DIS,
	CFG_CHGRSTRT,
	CFG_DCILMT,
	CFG_CHGCC,
	CFG_FCHGT,
	CFG_THM_CNFG,
	CFG_TOFFT,
	CFG_TOFFS,
	CFG_CHGCV,
};

static struct max8971_bitdesc max8971_cfg_bitdesc[] = {
	#define CFG_BITDESC(_cfg, _reg) \
			[CFG_##_cfg] = MAX8971_BITDESC(_reg, _reg##_##_cfg)

	CFG_BITDESC(CHGPROT		, PROTCMD),
	CFG_BITDESC(DCMON_DIS		, CHGCNTL1),
	CFG_BITDESC(USB_SUS		, CHGCNTL1),
	CFG_BITDESC(CHGRSTRT		, DCCRNT),
	CFG_BITDESC(DCILMT		, DCCRNT),
	CFG_BITDESC(CHGCC		, FCHGCRNT),
	CFG_BITDESC(FCHGT		, FCHGCRNT),
	CFG_BITDESC(THM_CNFG		, TEMPREG),
	CFG_BITDESC(TOFFT		, TOPOFF),
	CFG_BITDESC(TOFFS		, TOPOFF),
	CFG_BITDESC(CHGCV		, TOPOFF),
};
#define __cfg_bitdesc(_cfg) (&max8971_cfg_bitdesc[CFG_##_cfg])

#if defined(CONFIG_LGE_PM)
static int max8971_charger_lge_probe(struct max8971 *me);
#endif

#define PROTCMD_UNLOCK	3
#define PROTCMD_LOCK	0

static __always_inline int max8971_unlock(struct max8971 *me)
{
	int rc;

	rc = max8971_write_bitdesc(&me->io, __cfg_bitdesc(CHGPROT),
		PROTCMD_UNLOCK);
	if (unlikely(IS_ERR_VALUE(rc))) {
		log_err("failed to unlock [%d]\n", rc);
		goto out;
	}

#if VERIFICATION_UNLOCK
	do {
		u8 chgprot = 0;

		rc = max8971_read_bitdesc(&me->io,
			__cfg_bitdesc(CHGPROT), &chgprot);
		if (unlikely(IS_ERR_VALUE(rc) || chgprot != PROTCMD_UNLOCK)) {
			log_err("access denied - CHGPROT %X [%d]\n",
				chgprot, rc);
			rc = -EACCES;
			goto out;
		}
	} while (0);
#endif /* VERIFICATION_UNLOCK */

out:
	return rc;
}

static __always_inline int max8971_lock(struct max8971 *me)
{
	int rc;

	rc = max8971_write_bitdesc(&me->io, __cfg_bitdesc(CHGPROT),
		PROTCMD_LOCK);
	if (unlikely(IS_ERR_VALUE(rc)))
		log_err("failed to lock [%d]\n", rc);

	return rc;
}

#define max8971_read_config(_me, _cfg, _val_ptr) \
	({\
		int __rc = max8971_read_bitdesc(&(_me)->io, \
			__cfg_bitdesc(_cfg),\
			_val_ptr);\
		if (unlikely(IS_ERR_VALUE(__rc))) {\
			log_err("read config "#_cfg" error [%d]\n", __rc);\
		} else {\
			log_vdbg("read config "#_cfg": %Xh\n", *(_val_ptr));\
		} \
		__rc; \
	})
#define max8971_write_config(_me, _cfg, _val) \
	({\
		int __rc = max8971_unlock(_me);\
		if (likely(!IS_ERR_VALUE(__rc))) {\
			__rc = max8971_write_bitdesc(&(_me)->io, \
				__cfg_bitdesc(_cfg), _val);\
			if (unlikely(IS_ERR_VALUE(__rc))) {\
				log_err("write config "#_cfg" error [%d]\n", \
				__rc);\
			} else {\
				log_vdbg("write config "#_cfg": %Xh\n", _val);\
			} \
			max8971_lock(_me); \
		} \
		__rc; \
	})

static inline int max8971_get_dcilmt(struct max8971 *me, int *ua)
{
	int rc;
	u8 dcilmt = 0;

	rc = max8971_read_config(me, DCILMT, &dcilmt);
	if (unlikely(IS_ERR_VALUE(rc)))
		goto out;

	if (unlikely(dcilmt >= 0x3F)) {
		*ua = MAX8971_CURRENT_UNLIMIT;
		log_vdbg("<get_dcilmt> no limit\n");
		goto out;
	}

	*ua = dcilmt < 0x0A ? 100000 : (int)(dcilmt - 0x0A) * 25000 + 250000;
	log_vdbg("<get_dcilmt> %Xh -> %dua\n", dcilmt, *ua);

out:
	return rc;
}

static int max8971_set_dcilmt(struct max8971 *me, int ua)
{
	u8 dcilmt;

	if (unlikely(ua == MAX8971_CURRENT_UNLIMIT)) {
		dcilmt = 0x3F;
		log_vdbg("<set_dcilmt> no limit\n");
		goto out;
	}

	dcilmt = ua < 250000 ? 0x00 : DIV_ROUND_UP(ua -	250000, 25000) + 0x0A;
	log_vdbg("<set_dcilmt> %dua -> %Xh\n", ua, dcilmt);

out:
	return max8971_write_config(me, DCILMT, dcilmt);
}

static inline int max8971_get_enable(struct max8971 *me, int *en)
{
	int rc;
	u8 usb_sus = 0;

	rc = max8971_read_config(me, USB_SUS, &usb_sus);
	if (unlikely(IS_ERR_VALUE(rc)))
		goto out;

	*en = (usb_sus == 0);
	log_dbg("<get_enable> %s\n", en ? "enabled" : "disabled");

out:
	return rc;
}

static int max8971_set_enable(struct max8971 *me, int en)
{
	log_dbg("<max8971_set_enable> %s\n", en ? "enabling" : "disabling");
#ifndef CONFIG_CHARGER_MAX8971_FORCE_SINGLE_CHARGING
	return max8971_write_config(me, USB_SUS, (en == false));
#else
	return max8971_write_config(me, USB_SUS, 1);
#endif
}

static inline int max8971_get_chgcc(struct max8971 *me, int *ua)
{
	int rc;
	u8 chgcc = 0;

	rc = max8971_read_config(me, CHGCC, &chgcc);
	if (unlikely(IS_ERR_VALUE(rc)))
		goto out;

	*ua = chgcc < 0x06 ? 250000 : (int)(chgcc - 0x06) * 50000 +	300000;
	log_vdbg("<get_chgcc> %Xh -> %dua\n", chgcc, *ua);

out:
	return rc;
}

static int max8971_set_chgcc(struct max8971 *me, int ua)
{
	u8 chgcc;

	pr_info("[8971]me->te=%d, ua=%d\n", me->te, ua);
	if (me->te < ua)
		ua = me->te;

	chgcc = ua < 300000 ? 0x00 :
			ua < 1550000 ? DIV_ROUND_UP(ua - 300000, 50000)
				+ 0x06 : 0x1F;
	log_dbg("<set_chgcc> %dua -> %Xh\n", ua, chgcc);

	return max8971_write_config(me, CHGCC, chgcc);
}

static int max8971_set_charge_current(struct max8971 *me, int limit_ua,
	int charge_ua)
{
	int rc;

	#define DCILMT_MIN	100000
	#define DCILMT_MAX	1500000
	#define CHGCC_MIN	 250000
	#define CHGCC_MAX	 1550000

	if (limit_ua == MAX8971_CURRENT_MAXIMUM)
		limit_ua = DCILMT_MAX;
	else if (limit_ua == MAX8971_CURRENT_MINIMUM)
		limit_ua = DCILMT_MIN;
	else if (limit_ua != MAX8971_CURRENT_UNLIMIT)
		limit_ua	= max(DCILMT_MIN, limit_ua);

	if (charge_ua == MAX8971_CURRENT_UNLIMIT ||
		charge_ua == MAX8971_CURRENT_MAXIMUM)
		charge_ua = CHGCC_MAX;
	else if (limit_ua == MAX8971_CURRENT_MINIMUM)
		charge_ua = CHGCC_MIN;
	else
		charge_ua = max(CHGCC_MIN , charge_ua);

	if (likely(limit_ua == MAX8971_CURRENT_UNLIMIT ||
		limit_ua >= charge_ua)) {
		rc = max8971_set_dcilmt(me, limit_ua);
		if (unlikely(IS_ERR_VALUE(rc)))
			goto out;

		if (likely(me->dev_enabled))
			rc = max8971_set_chgcc(me, charge_ua);

		goto out;
	}

	if (likely(me->dev_enabled)) {
		log_dbg("setting current %dua but limited up to %dua\n",
			charge_ua, limit_ua);
		rc = max8971_set_chgcc(me, limit_ua);
		if (unlikely(IS_ERR_VALUE(rc)))
			goto out;
	}

	rc = max8971_set_dcilmt(me, limit_ua);

out:
	return rc;
}

static bool max8971_present_input(struct max8971 *me)
{
	u8 dc_uvp = 0;
	int rc;

	rc = max8971_read_reg_bit(&me->io, DETAILS1, DC_UVP, &dc_uvp);
	if (unlikely(IS_ERR_VALUE(rc)))
		return false;

	return (dc_uvp == DC_UVP_VALID);
}

#define max8971_psy_setprop(_me, _psy, _psp, _val) \
	({\
		struct power_supply *__psy = _me->_psy;\
		union power_supply_propval __propval = { .intval = _val };\
		int __rc = -ENXIO;\
		if (likely(__psy && __psy->set_property)) {\
			__rc = __psy->set_property(__psy, \
				POWER_SUPPLY_PROP_##_psp, &__propval);\
		} \
		__rc;\
	})

static void max8971_psy_init(struct max8971 *me)
{
	if (unlikely(!me->psy_this))
		me->psy_this = &me->psy;

	if (unlikely(!me->psy_ext && me->pdata->ext_psy_name)) {
		me->psy_ext = power_supply_get_by_name(me->pdata->ext_psy_name);
		max8971_psy_setprop(me, psy_ext, PRESENT, false);
	}

	if (unlikely(!me->psy_coop && me->pdata->coop_psy_name))
		me->psy_coop =
			power_supply_get_by_name(me->pdata->coop_psy_name);
}

static void max8971_psy_changed(struct max8971 *me)
{
	max8971_psy_init(me);

	if (likely(me->psy_this))
		power_supply_changed(me->psy_this);

	if (likely(me->psy_ext))
		power_supply_changed(me->psy_ext);

	if (likely(me->psy_coop))
		power_supply_changed(me->psy_coop);
}

static int max8971_exit_dev(struct max8971 *me)
{
	struct max8971_platform_data *pdata = me->pdata;

	max8971_set_charge_current(me, me->current_limit_permanent, 0);
	max8971_set_enable(me, false);

	me->current_limit_volatile	= me->current_limit_permanent;
	me->charge_current_volatile = me->charge_current_permanent;

	me->dev_enabled	 = (!pdata->enable_coop || pdata->coop_psy_name);
	me->dev_initialized = false;
	return 0;
}

static int max8971_init_dev(struct max8971 *me)
{
	struct max8971_platform_data *pdata = me->pdata;
#if 0
	unsigned long irq_flags;
#endif
	int rc;
	u8 val;

	val	= 0;
/*	val |= CHGINT_AICL;*/
/*	val |= CHGINT_TOPOFF;*/
/*	val |= CHGINT_DC_OVP;*/
	val |= CHGINT_DC_UVP;
/*	val |= CHGINT_CHG;*/
/*	val |= CHGINT_BAT;*/
/*	val |= CHGINT_THM;*/
/*	val |= CHGINT_PWRUP;*/

	rc = max8971_write(&me->io, CHGINT_MSK, ~val);
	if (unlikely(IS_ERR_VALUE(rc))) {
		log_err("CHGINTM1 write error [%d]\n", rc);
		goto out;
	}

#if 1
	rc = max8971_read(&me->io, CHGINT_MSK, &val);
	log_dbg("CHGINT_MSK %02Xh\n", val);
#endif
#if 0
	val = max8971_read_irq_status(me);

	spin_lock_irqsave(&me->irq_lock, irq_flags);
	me->irq_saved |= (val & ~CHGINT_PWRUP);
	spin_unlock_irqrestore(&me->irq_lock, irq_flags);

	log_dbg("CHGINT CURR %02Xh SAVED %02Xh\n", val, me->irq_saved);
#endif

#ifndef CONFIG_CHARGER_MAX8971_FORCE_SINGLE_CHARGING
	/*dual charging */
#else
	/* charger disable */
	rc = max8971_set_enable(me, 0);
	if (unlikely(IS_ERR_VALUE(rc)))
		goto out;

	/* charger enable */
#if 0
	rc = max8971_set_enable(me, me->dev_enabled);
	if (unlikely(IS_ERR_VALUE(rc)))
		goto out;
#endif
#endif

	/* charge current */
	rc = max8971_set_charge_current(me, me->current_limit_volatile,
		me->charge_current_volatile);
	if (unlikely(IS_ERR_VALUE(rc)))
		goto out;

	/* topoff timer */
	val = pdata->topoff_timer <	0 ? 0x00 :
			pdata->topoff_timer < 70 ?
			DIV_ROUND_UP(pdata->topoff_timer, 10) : 0x07;
	rc = max8971_write_config(me, TOFFT, val);
	if (unlikely(IS_ERR_VALUE(rc)))
		goto out;

	/* topoff current */
	val = pdata->topoff_current <	50000 ? 0x00 :
			pdata->topoff_current < 200000 ?
			DIV_ROUND_UP(pdata->topoff_current - 50000, 50000) :
			0x07;
	rc = max8971_write_config(me, TOFFS, val);
	if (unlikely(IS_ERR_VALUE(rc)))
		goto out;

	/* charge restart threshold */
	val = (pdata->charge_restart_threshold < 150000);
	rc = max8971_write_config(me, CHGRSTRT, val);
	if (unlikely(IS_ERR_VALUE(rc)))
		goto out;

	/* charge termination voltage */
	val = pdata->charge_termination_voltage < 4100000 ? 0x01 :
			pdata->charge_termination_voltage < 4150000 ? 0x03 :
			pdata->charge_termination_voltage < 4200000 ? 0x00 :
			0x02;
	rc = max8971_write_config(me, CHGCV, val);
	if (unlikely(IS_ERR_VALUE(rc)))
		goto out;
	#if defined(CONFIG_CHARGER_FACTORY_MODE)
	 max8971_write_config(me,	FCHGT, 0);
	#endif
	/* thermistor control */
	val = (pdata->enable_thermistor == false);
	rc = max8971_write_config(me, THM_CNFG, val);
	if (unlikely(IS_ERR_VALUE(rc)))
		goto out;

	/* AICL control */

	val = (pdata->enable_aicl == false);
	rc = max8971_write_config(me, DCMON_DIS, val);
	if (unlikely(IS_ERR_VALUE(rc)))
		goto out;

	me->dev_initialized = true;
	log_dbg("device initialized\n");

out:
	return rc;
}

struct max8971_status_map {
	int health, status, charge_type;
};

static struct max8971_status_map max8971_status_map[] = {
	#define STATUS_MAP(_chg_dtls, _health, _status, _charge_type) \
		[CHG_DTLS_##_chg_dtls] = {\
			.health = POWER_SUPPLY_HEALTH_##_health,\
			.status = POWER_SUPPLY_STATUS_##_status,\
			.charge_type = POWER_SUPPLY_CHARGE_TYPE_##_charge_type,\
		}
	/* health	status	charge_type*/
	STATUS_MAP(DEAD_BATTERY,	DEAD,		NOT_CHARGING,	NONE),
	STATUS_MAP(PRECHARGE,	UNKNOWN,	CHARGING,	TRICKLE),
	STATUS_MAP(FASTCHARGE_CC,	UNKNOWN,	CHARGING,	FAST),
	STATUS_MAP(FASTCHARGE_CV,	UNKNOWN,	CHARGING,	FAST),
	STATUS_MAP(TOPOFF,		UNKNOWN,	CHARGING,	FAST),
	STATUS_MAP(DONE,		UNKNOWN,	FULL,		NONE),
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
	STATUS_MAP(TIMER_FAULT,	SAFETY_TIMER_EXPIRE,	NOT_CHARGING,	NONE),
#else /* LINUX_VERSION_CODE ... */
	STATUS_MAP(TIMER_FAULT,		UNKNOWN,	NOT_CHARGING,	NONE),
#endif /* LINUX_VERSION_CODE ... */
	STATUS_MAP(TEMP_SUSPEND,	UNKNOWN,	NOT_CHARGING,	NONE),
	STATUS_MAP(OFF,			UNKNOWN,	NOT_CHARGING,	NONE),
	STATUS_MAP(THM_LOOP,		UNKNOWN,	CHARGING,	NONE),
};

static int max8971_update(struct max8971 *me)
{
	int rc;
	u8 details1, details2;
	u8 dcv, dci, dcovp, dcuvp, thm, bat, chg;

	me->health		= POWER_SUPPLY_HEALTH_UNKNOWN;
	me->status		= POWER_SUPPLY_STATUS_UNKNOWN;
	me->charge_type = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;

	rc = max8971_read(&me->io, DETAILS1, &details1);
	if (unlikely(IS_ERR_VALUE(rc))) {
		log_err("DETAILS1 read error [%d]\n", rc);
		goto out;
	}

	rc = max8971_read(&me->io, DETAILS2, &details2);
	if (unlikely(IS_ERR_VALUE(rc))) {
		log_err("DETAILS2 read error [%d]\n", rc);
		goto out;
	}

	log_dbg("DETAILS1 %Xh DETAILS2 %Xh\n", details1, details2);

	dcv = BITS_GET(details1, DETAILS1_DC_V);
	log_vdbg("*** DC_V	 %s\n", max8971_dcv_details[dcv]);

	dci = BITS_GET(details1, DETAILS1_DC_I);
	log_vdbg("*** DC_I	 %s\n", max8971_dci_details[dci]);

	dcovp = BITS_GET(details1, DETAILS1_DC_OVP);
	log_vdbg("*** DC_OVP %s\n", max8971_dcovp_details[dcovp]);

	dcuvp = BITS_GET(details1, DETAILS1_DC_UVP);
	log_vdbg("*** DC_UVP %s\n", max8971_dcuvp_details[dcuvp]);

	thm = BITS_GET(details1, DETAILS1_THM_DTLS);
	log_vdbg("*** THM	%s\n", max8971_thm_details[thm]);

	bat = BITS_GET(details2, DETAILS2_BAT_DTLS);
	log_vdbg("*** BAT	%s\n", max8971_bat_details[bat]);

	chg = BITS_GET(details2, DETAILS2_CHG_DTLS);
	log_vdbg("*** CHG	%s\n", max8971_chg_details[chg]);

	me->present = (dcuvp == DC_UVP_VALID);
	if (unlikely(!me->present)) {
		/* no charger present */
		me->health		= POWER_SUPPLY_HEALTH_UNKNOWN;
		me->status		= POWER_SUPPLY_STATUS_DISCHARGING;
		me->charge_type = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
		goto out;
	}

	if (unlikely(dcovp != DC_OVP_VALID)) {
		me->status		= POWER_SUPPLY_STATUS_NOT_CHARGING;
		me->health		= POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		me->charge_type = POWER_SUPPLY_CHARGE_TYPE_NONE;
		goto out;
	}

	me->health		= max8971_status_map[chg].health;
	me->status		= max8971_status_map[chg].status;
	me->charge_type = max8971_status_map[chg].charge_type;

	if (likely(me->health != POWER_SUPPLY_HEALTH_UNKNOWN))
		goto out;

	/* override health by THM_DTLS */
	switch (thm) {
	case THM_DTLS_LOW_TEMP_SUSPEND:
		me->health = POWER_SUPPLY_HEALTH_COLD;
		break;
	case THM_DTLS_HIGH_TEMP_SUSPEND:
		me->health = POWER_SUPPLY_HEALTH_OVERHEAT;
		break;
	default:
		me->health = POWER_SUPPLY_HEALTH_GOOD;
		break;
	}

out:
	log_vdbg("PRESENT %d HEALTH %d STATUS %d CHARGE_TYPE %d\n",
		me->present, me->health, me->status, me->charge_type);
	return rc;
}

#define max8971_resume_log_work(_me) \
	do {\
		if (likely(log_worker)) {\
			if (likely(!delayed_work_pending(&(_me)->log_work))) {\
				schedule_delayed_work(&(_me)->log_work, \
					LOG_WORK_INTERVAL);\
			} \
		} \
	} while (0)

#define max8971_suspend_log_work(_me) \
		cancel_delayed_work_sync(&(_me)->log_work)

static void max8971_log_work(struct work_struct *work)
{
	struct max8971 *me =
		container_of(work, struct max8971, log_work.work);
	int val = 0;
	u8 regval = 0;

	__lock(me);

	max8971_update(me);

	max8971_get_enable(me, &val);
	log_info("charger = %s\n", val ? "on" : "off");

	max8971_get_dcilmt(me, &val);
	log_info("current limit = %dua (regval %dua)\n",
		me->current_limit_volatile, val);

	max8971_get_chgcc(me, &val);
	log_info("charge current = %dua (regval %dua)\n",
		me->charge_current_volatile, val);

	max8971_read(&me->io, TOPOFF, &regval);
	log_info("TOPOFF register %02Xh\n", regval);

	max8971_resume_log_work(me);

	__unlock(me);
}

#ifdef WORKQUEUE_USE_ISR
static void max8971_irq_work(struct work_struct *work)
#else
static void max8971_irq_work(struct max8971 *me)
#endif
{
#ifdef WORKQUEUE_USE_ISR
	struct max8971 *me = container_of(work, struct max8971, irq_work.work);
#endif
	unsigned long irq_flags;
	u8 irq_current;
	bool present_input;

	__lock(me);

	irq_current = max8971_read_irq_status(me);

	spin_lock_irqsave(&me->irq_lock, irq_flags);
	irq_current |= me->irq_saved;
	me->irq_saved = 0;
	spin_unlock_irqrestore(&me->irq_lock, irq_flags);

	log_dbg("<IRQ_WORK> CCHGINT %02Xh\n", irq_current);

	if (unlikely(!irq_current))
		goto done;

	/* just check DC_UVP */
	if (irq_current & CHGINT_DC_UVP) {
		present_input = max8971_present_input(me);
		log_info("<IRQ_WORK> present_input=%d\n", present_input);

		if (likely(present_input)) {
			max8971_psy_init(me);
			max8971_init_dev(me);
			log_info("<IRQ_WORK> after max8971_init_dev\n");
		} else {
			log_info("<IRQ_WORK> before max8971_exit_dev\n");
			max8971_exit_dev(me);
		}

		/* notify psy changed */
		max8971_psy_changed(me);

		log_dbg("DC input %s\n", present_input ?
			"inserted" : "removed");
		goto out;
	}

done:
#ifdef WORKQUEUE_USE_ISR
	if (unlikely(me->irq <= 0)) {
		if (likely(!delayed_work_pending(&me->irq_work)))
			schedule_delayed_work(&me->irq_work, IRQ_WORK_INTERVAL);
	}
#endif
out:
	__unlock(me);
#if I2C_SUSPEND_WORKAROUND
	if (likely(me->irq_disabled)) {
		me->irq_disabled = false;
		enable_irq(me->irq);
	}
#endif
	return;
}
#if I2C_SUSPEND_WORKAROUND
static void max8971_check_suspended_worker(struct work_struct *work)
{
	struct max8971 *chip =
		container_of(work, struct max8971, check_suspended_work.work);

	bool is_resumed_all = (atomic_read(&chip->suspended) == 0) && !i2c_suspended;
	pr_info("%s     charger : %d qup : %d\n", __func__,
		atomic_read(&chip->suspended), i2c_suspended);
	if (is_resumed_all) {
		schedule_delayed_work(&chip->irq_work, msecs_to_jiffies(150));
	} else {
		pr_info("max8971 charger suspended. try i2c operation after 100ms.\n");
		schedule_delayed_work(&chip->check_suspended_work, msecs_to_jiffies(50));
	}
}
#endif /*I2C_SUSPEND_WORKAROUND*/
static irqreturn_t max8971_isr(int irq, void *data)
{
	struct max8971 *me = data;
#if I2C_SUSPEND_WORKAROUND
	disable_irq_nosync(me->irq);
	me->irq_disabled = true;
	schedule_delayed_work(&me->check_suspended_work, msecs_to_jiffies(100));
#else
	u8 irq_current;

	irq_current = max8971_read_irq_status(me);
	log_dbg("<ISR> CHGINT CURR %02Xh SAVED %02Xh\n",
		irq_current, me->irq_saved);
	me->irq_saved |= irq_current;

#ifdef WORKQUEUE_USE_ISR
	if (delayed_work_pending(&me->irq_work))
		cancel_delayed_work_sync(&me->irq_work);
	schedule_delayed_work(&me->irq_work, IRQ_WORK_DELAY);
#else
	max8971_irq_work(me);
#endif
#endif
#ifndef CONFIG_CHARGER_MAX8971_FORCE_SINGLE_CHARGING
#else
	max8971_set_enable(me, false);
#endif
	return IRQ_HANDLED;
}

static int max8971_get_property(struct power_supply *psy,
		enum power_supply_property psp, union power_supply_propval *val)
{
	struct max8971 *me = container_of(psy, struct max8971, psy);
	int rc;
	__lock(me);

	rc = max8971_update(me);
	if (unlikely(IS_ERR_VALUE(rc)))
		goto out;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = me->present;
		break;

#ifndef POWER_SUPPLY_PROP_CHARGING_ENABLED_REPLACED
	case POWER_SUPPLY_PROP_ONLINE:
#endif /* !POWER_SUPPLY_PROP_CHARGING_ENABLED_REPLACED */
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = me->dev_enabled;
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = me->health;
		break;

	case POWER_SUPPLY_PROP_STATUS:
		val->intval = me->status;
		break;

	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = me->charge_type;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		val->intval = me->charge_current_volatile;
		log_dbg("<max8971 get_property> psp %d val %d [%d]\n",
			psp, val->intval, rc);
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = me->current_limit_volatile;
		break;
	case POWER_SUPPLY_PROP_SAFTETY_CHARGER_TIMER:
{
		u8	prop_val = 0;
		max8971_read_config(me, FCHGT, &prop_val);
		val->intval = prop_val;
		if (unlikely(IS_ERR_VALUE(rc)))
			val->intval = -1;
		pr_info("%s    timer :%d(D)\n", __func__, val->intval);
}
		break;
	default:
		rc = -EINVAL;
		goto out;
	}

out:
	log_vdbg("<get_property> psp %d val %d [%d]\n", psp, val->intval, rc);
	__unlock(me);
	return rc;
}

static int max8971_set_property(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	struct max8971 *me = container_of(psy, struct max8971, psy);
	int ua, rc = 0;

	__lock(me);

	switch (psp) {
#if defined(CONFIG_LGE_PM)
	case POWER_SUPPLY_PROP_ONLINE:
#endif
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		/* apply charge current */
		rc = max8971_set_charge_current(me, me->current_limit_volatile,
			me->charge_current_volatile);
		rc = max8971_set_enable(me, val->intval);
		if (unlikely(IS_ERR_VALUE(rc)))
			goto out;

		me->dev_enabled = val->intval;

		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ua = abs(val->intval);
		rc = max8971_set_charge_current(me,
			me->current_limit_volatile, ua);
		if (unlikely(IS_ERR_VALUE(rc)))
			goto out;

		me->charge_current_volatile	= ua;
		me->charge_current_permanent =
			val->intval > 0 ? ua : me->charge_current_permanent;
		log_dbg("<max8971_set_property> CC, uA %d limit_volatile %d\n",
					ua, me->current_limit_volatile);
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		ua = abs(val->intval);
		rc = max8971_set_charge_current(me, ua,
			me->charge_current_volatile);
		if (unlikely(IS_ERR_VALUE(rc)))
			goto out;

		me->current_limit_volatile	= ua;
		me->current_limit_permanent =
			val->intval > 0 ? ua : me->current_limit_permanent;
		break;
	case POWER_SUPPLY_PROP_TYPE:
		me->psy.type = val->intval;
		break;
	case POWER_SUPPLY_PROP_SAFTETY_CHARGER_TIMER:
#if !defined(CONFIG_CHARGER_FACTORY_MODE)
		rc = max8971_write_config(me,
				FCHGT, (val->intval == 0) ? 0 : 0x02);
		if (unlikely(IS_ERR_VALUE(rc)))
			rc = -EINVAL;
		pr_info("%s    timer :%d(D)\n", __func__, val->intval);
#endif
		break;
	default:
		rc = -EINVAL;
		goto out;
	}

out:
	log_vdbg("<set_property> psp %d val %d [%d]\n", psp, val->intval, rc);
	__unlock(me);
	return rc;
}

static int max8971_property_is_writeable(struct power_supply *psy,
	enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_SAFTETY_CHARGER_TIMER:
		return 1;

	default:
		break;
	}

	return -EINVAL;
}

static void max8971_external_power_changed(struct power_supply *psy)
{
	struct max8971 *me = container_of(psy, struct max8971, psy);
	struct power_supply *supplicant;
	int i;

	__lock(me);

	for (i = 0; i < me->pdata->num_supplicants; i++) {
		supplicant =
			power_supply_get_by_name(me->pdata->supplied_to[i]);
		if (likely(supplicant))
			power_supply_changed(supplicant);
	}

	__unlock(me);
}

static enum power_supply_property max8971_psy_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
#if !defined(CONFIG_LGE_PM)
#ifndef POWER_SUPPLY_PROP_CHARGING_ENABLED_REPLACED
	POWER_SUPPLY_PROP_ONLINE,
#endif /* !POWER_SUPPLY_PROP_CHARGING_ENABLED_REPLACED */
#endif
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,	 /* charging current */
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, /* input current limit */
	POWER_SUPPLY_PROP_SAFTETY_CHARGER_TIMER,
};

static void *max8971_get_platdata(struct max8971 *me)
{
#ifdef CONFIG_OF
	struct device *dev = me->dev;
	struct device_node *np = dev->of_node;
	struct max8971_platform_data *pdata;
	size_t sz;
	int num_supplicants, i, rc;

	num_supplicants = of_property_count_strings(np, "supplied_to");
	num_supplicants = max(0, num_supplicants);

	sz = sizeof(*pdata) + num_supplicants * sizeof(char *);
	pdata = devm_kzalloc(dev, sz, GFP_KERNEL);
	if (unlikely(!pdata)) {
		log_err("out of memory (%uB requested)\n", sz);
		pdata = ERR_PTR(-ENOMEM);
		goto out;
	}

	me->irq_gpio = of_get_named_gpio(np, "max8971,int-gpio", 0);

	if (me->irq_gpio < 0) {
		pdata->irq = irq_of_parse_and_map(np, 0);
	} else {
		unsigned gpio = (unsigned)me->irq_gpio;

		rc = gpio_request(gpio, DRIVER_NAME"-irq");
		if (unlikely(IS_ERR_VALUE(rc))) {
			log_err("failed to request gpio %u [%d]\n", gpio, rc);
			me->irq_gpio = -1;
			pdata = ERR_PTR(rc);
			goto out;
		}

		gpio_direction_input(gpio);
		log_dbg("INTGPIO %u assigned\n", gpio);

		/* override pdata irq */
		pdata->irq = gpio_to_irq(gpio);
	}

	log_dbg("property:INTGPIO	%d\n", me->irq_gpio);
	log_dbg("property:IRQ		%d\n", pdata->irq);

	pdata->psy_name = "ac";
	of_property_read_string(np, "psy_name",
		(char const **)&pdata->psy_name);
	log_dbg("property:PSY NAME	%s\n", pdata->psy_name);

	of_property_read_string(np, "ext_psy_name",
		(char const **)&pdata->ext_psy_name);
	log_dbg("property:EXT PSY	%s\n",
		pdata->ext_psy_name ? pdata->ext_psy_name : "null");

	if (unlikely(num_supplicants <= 0)) {
		pdata->supplied_to	 = NULL;
		pdata->num_supplicants = 0;
		log_dbg("property:SUPPLICANTS	 null\n");
	} else {
		pdata->num_supplicants = (size_t)num_supplicants;
		log_dbg("property:SUPPLICANTS	 %d\n", num_supplicants);
		pdata->supplied_to = (char **)(pdata + 1);
		for (i = 0; i < num_supplicants; i++) {
			of_property_read_string_index(np, "supplied_to", i,
				(char const **)&pdata->supplied_to[i]);
			log_dbg("property:SUPPLICANTS	 %s\n",
				pdata->supplied_to[i]);
		}
	}

	pdata->fast_charge_current = 375000;
	of_property_read_u32(np, "fast_charge_current",
		&pdata->fast_charge_current);
	log_dbg("property:CHGCC		%dua\n", pdata->fast_charge_current);

	pdata->charge_termination_voltage = 4200000;
	of_property_read_u32(np, "charge_termination_voltage",
		&pdata->charge_termination_voltage);
	log_dbg("property:CHGCV		%duV\n",
		pdata->charge_termination_voltage);

	pdata->topoff_timer = 30;
	of_property_read_u32(np, "topoff_timer", &pdata->topoff_timer);
	log_dbg("property:TOFFT		%dmin\n", pdata->topoff_timer);

	pdata->topoff_current = 50000;
	of_property_read_u32(np, "topoff_current", &pdata->topoff_current);
	log_dbg("property:TOFFS		%dua\n", pdata->topoff_current);

	pdata->charge_restart_threshold = 150000;
	of_property_read_u32(np, "charge_restart_threshold",
		&pdata->charge_restart_threshold);
	log_dbg("property:CHGRSTRT	%duV\n",
		pdata->charge_restart_threshold);

	pdata->enable_coop = of_property_read_bool(np, "enable_coop");
	log_dbg("property:COOP CHG	%s\n",
		pdata->enable_coop ? "enabled" : "disabled");

	if (likely(pdata->enable_coop)) {
		of_property_read_string(np, "coop_psy_name",
			(char const **)&pdata->coop_psy_name);
		log_dbg("property:COOP CHG	%s\n",
			pdata->coop_psy_name ? pdata->coop_psy_name : "null");
	}

	pdata->enable_thermistor =
		of_property_read_bool(np, "enable_thermistor");
	log_dbg("property:THERMISTOR	%s\n",
		pdata->enable_thermistor ? "enabled" : "disabled");

	pdata->enable_aicl = of_property_read_bool(np, "enable_aicl");
	log_dbg("property:AICL		%s\n",
		pdata->enable_aicl ? "enabled" : "disabled");

out:
	return pdata;
#else /* CONFIG_OF */
	return dev_get_platdata(me->dev) ?
		dev_get_platdata(me->dev) : ERR_PTR(-EINVAL);
#endif /* CONFIG_OF */
}

static __always_inline
void max8971_destroy(struct max8971 *me)
{
	struct device *dev = me->dev;

	cancel_delayed_work_sync(&me->log_work);

	if (likely(me->irq > 0))
		devm_free_irq(dev, me->irq, me);

#ifdef WORKQUEUE_USE_ISR
	cancel_delayed_work_sync(&me->irq_work);
#endif

	if (likely(me->irq_gpio >= 0))
		gpio_free((unsigned)me->irq_gpio);

	if (likely(me->attr_grp))
		sysfs_remove_group(me->kobj, me->attr_grp);

	if (likely(me->psy_this))
		power_supply_unregister(me->psy_this);

	if (likely(me->io.regmap))
		regmap_exit(me->io.regmap);

#ifdef CONFIG_OF
	if (likely(me->pdata))
		devm_kfree(dev, me->pdata);
#endif /* CONFIG_OF */

	mutex_destroy(&me->lock);
/*	spin_lock_destroy(&me->irq_lock);*/

	devm_kfree(dev, me);
}

#ifdef CONFIG_OF
static struct of_device_id max8971_of_ids[] = {
	{ .compatible = "maxim,"MAX8971_NAME },
	{ },
};
MODULE_DEVICE_TABLE(of, max8971_of_ids);
#endif /* CONFIG_OF */

static const struct i2c_device_id max8971_i2c_ids[] = {
	{ MAX8971_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, max8971_i2c_ids);

static const struct regmap_config max8971_regmap_config = {
	.reg_bits	 = 8,
	.val_bits	 = 8,
	.cache_type = REGCACHE_NONE,
};

static __devinit int max8971_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct max8971 *me;
	int rc;

	log_dbg("attached\n");

	me = devm_kzalloc(&client->dev, sizeof(*me), GFP_KERNEL);
	if (unlikely(!me)) {
		log_err("out of memory (%uB requested)\n", sizeof(*me));
		return -ENOMEM;
	}

	i2c_set_clientdata(client, me);

	spin_lock_init(&me->irq_lock);
	mutex_init(&me->lock);
	me->dev	= &client->dev;
	me->kobj = &client->dev.kobj;
	me->irq	= -1;
	me->irq_gpio = -1;
#if I2C_SUSPEND_WORKAROUND
	me->irq_disabled = false;
	INIT_DELAYED_WORK(&me->check_suspended_work, max8971_check_suspended_worker);
#endif

#ifdef WORKQUEUE_USE_ISR
	INIT_DELAYED_WORK(&me->irq_work, max8971_irq_work);
#endif
	INIT_DELAYED_WORK(&me->log_work, max8971_log_work);

	me->io.regmap = devm_regmap_init_i2c(client, &max8971_regmap_config);
	if (unlikely(IS_ERR(me->io.regmap))) {
		rc = PTR_ERR(me->io.regmap);
		me->io.regmap = NULL;
		log_err("failed to initialize i2c regmap [%d]\n", rc);
		goto abort;
	}

	me->pdata = max8971_get_platdata(me);
	if (unlikely(!me->pdata)) {
		log_err("platform data is missing\n");
		rc = -EINVAL;
		goto abort;
	}

	/* disable all IRQ */
	max8971_write(&me->io, CHGINT_MSK, 0xFF);

	me->dev_enabled				 =
		(!me->pdata->enable_coop || me->pdata->coop_psy_name);
	me->current_limit_permanent	 = MAX8971_CURRENT_UNLIMIT;
	me->charge_current_permanent	= me->pdata->fast_charge_current;
	me->current_limit_volatile	= me->current_limit_permanent;
	me->charge_current_volatile	 = me->charge_current_permanent;

	if (likely(max8971_present_input(me))) {
		rc = max8971_init_dev(me);
		if (unlikely(IS_ERR_VALUE(rc)))
			goto abort;
	}

	me->psy.name			= me->pdata->psy_name;
	me->psy.type			= POWER_SUPPLY_TYPE_MAINS;
	me->psy.supplied_to		= me->pdata->supplied_to;
	me->psy.num_supplicants		= me->pdata->num_supplicants;
	me->psy.properties		= max8971_psy_props;
	me->psy.num_properties		= ARRAY_SIZE(max8971_psy_props);
	me->psy.get_property		= max8971_get_property;
	me->psy.set_property		= max8971_set_property;
	me->psy.property_is_writeable	= max8971_property_is_writeable;
	me->psy.external_power_changed	= max8971_external_power_changed;

	max8971_psy_init(me);

	rc = power_supply_register(&client->dev, me->psy_this);
	if (unlikely(IS_ERR_VALUE(rc))) {
		log_err("failed to register power_supply class [%d]\n", rc);
		me->psy_this = NULL;
		goto abort;
	}

	 me->irq = me->pdata->irq;

	if (unlikely(me->irq <= 0)) {
		log_warn("interrupt disabled\n");
#ifdef WORKQUEUE_USE_ISR
		schedule_delayed_work(&me->irq_work, IRQ_WORK_INTERVAL);
#endif
	} else {
		rc = devm_request_threaded_irq(&client->dev,
			me->irq, NULL, max8971_isr,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT, DRIVER_NAME, me);
		if (unlikely(IS_ERR_VALUE(rc))) {
			log_err("failed to request IRQ %d [%d]\n", me->irq, rc);
			me->irq = -1;
			goto abort;
		}
	}

	log_info("driver "DRIVER_VERSION" installed\n");

	/* enable IRQ we want */
	max8971_write(&me->io, CHGINT_MSK, (u8)~CHGINT_DC_UVP);
#ifdef WORKQUEUE_USE_ISR
	schedule_delayed_work(&me->irq_work, IRQ_WORK_DELAY);
#else
	max8971_irq_work(me);
#endif

	max8971_resume_log_work(me);

#if defined(CONFIG_LGE_PM)
	rc = max8971_charger_lge_probe(me);
	if (rc)
		goto lge_probe_fail;

#ifndef CONFIG_CHARGER_MAX8971_FORCE_SINGLE_CHARGING
#else
	rc = max8971_set_enable(me, false);
	if (unlikely(IS_ERR_VALUE(rc)))
		return 0;
#endif

#endif

	return 0;

lge_probe_fail:
abort:
	i2c_set_clientdata(client, NULL);
	max8971_destroy(me);
	return rc;
}

static __devexit int max8971_remove(struct i2c_client *client)
{
	struct max8971 *me = i2c_get_clientdata(client);

	i2c_set_clientdata(client, NULL);
	max8971_destroy(me);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max8971_suspend(struct device *dev)
{
#if I2C_SUSPEND_WORKAROUND
	struct max8971 *me = dev_get_drvdata(dev);
	atomic_set(&me->suspended, 1);
#endif
	return 0;
}

static int max8971_resume(struct device *dev)
{
#if I2C_SUSPEND_WORKAROUND
	struct max8971 *me = dev_get_drvdata(dev);
	atomic_set(&me->suspended, 0);
#endif
	return 0;
}
#endif

#if defined(CONFIG_LGE_PM)
static ssize_t at_chg_status_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	union power_supply_propval ret = {0,};
	struct max8971 *me = dev_get_drvdata(dev);
	if (!me->batt)
		me->batt = power_supply_get_by_name("battery");

	if(likely(me->batt)) {
		me->batt->get_property(me->batt,
			POWER_SUPPLY_PROP_CHARGING_ENABLED, &ret);
	} else {
		pr_err("psy battery not found\n");
	}

	return snprintf(buf, 1, "%d\n", ret.intval);
}

static ssize_t at_chg_status_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct max8971 *me = dev_get_drvdata(dev);
	if (strncmp(buf, "0", 1) == 0) {
		/* stop charging */
		pr_info("[Diag] stop charging start\n");
		max8971_set_enable(me, 0);
	} else if (strncmp(buf, "1", 1) == 0) {
		/* start charging */
		pr_info("[Diag] start charging start\n");
		max8971_set_enable(me, 1);
	}

	return 1;
}

static ssize_t at_chg_complete_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int r;
	struct max8971 *me = dev_get_drvdata(dev);
	union power_supply_propval ret = {0,};
	if (!me->batt)
		me->batt = power_supply_get_by_name("battery");

	if(likely(me->batt)) {
		me->batt->get_property(me->batt, POWER_SUPPLY_PROP_CAPACITY, &ret);
	} else {
		pr_err("psy battery not found\n");
	}

	if (ret.intval == 100) {
		r = snprintf(buf, 3, "%d\n", 0);
		pr_info("[Diag] buf = %s, gauge==100\n", buf);
	} else {
		r = snprintf(buf, 3, "%d\n", 1);
		pr_info("[Diag] buf = %s, gauge<=100\n", buf);
	}

	return r;
}

static ssize_t at_pmic_reset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int r = 0;
	bool pm_reset = true;

	msleep(3000); /* for waiting return values of testmode */

	machine_restart(NULL);

	r = snprintf(buf, 3, "%d\n", pm_reset);

	return r;
}

DEVICE_ATTR(at_charge_8971, 0644, at_chg_status_show, at_chg_status_store);
DEVICE_ATTR(at_chcomp_8971, 0644, at_chg_complete_show, at_chg_status_store);
DEVICE_ATTR(at_pmrst_8971, 0640, at_pmic_reset_show, NULL);

static ssize_t curr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int a, b;
	struct max8971 *me = dev_get_drvdata(dev);
	max8971_get_dcilmt(me, &a);
	max8971_get_chgcc(me, &b);

	return snprintf(buf, 50, "dci:%d chg:%d\n", a, b);
}

static ssize_t curr_max_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int curr, ret;
	char str_curr[10];
	struct max8971 *me = dev_get_drvdata(dev);

	strcpy(str_curr, (const char *)buf);
	ret = kstrtoint((const char *)str_curr, 0, &curr);
	if (ret)
		return ret;
	pr_info("Set dcilmt:%d\n", curr);
	max8971_set_dcilmt(me, curr);
	msleep(msecs_to_jiffies(500));
	max8971_get_dcilmt(me, &curr);
	pr_info("Result:%d\n", curr);


	return count;
}

static ssize_t curr_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int curr, ret;
	char str_curr[10];
	struct max8971 *me = dev_get_drvdata(dev);

	strcpy(str_curr, (const char *)buf);
	ret = kstrtoint((const char *)str_curr, 0, &curr);
	if (ret)
		return ret;
	pr_info("Set chgcc:%d\n", curr);
	max8971_set_chgcc(me, curr);
	msleep(msecs_to_jiffies(500));
	max8971_get_chgcc(me, &curr);
	pr_info("Result:%d\n", curr);

	return count;
}

DEVICE_ATTR(force_set_dci_8971, 0644, curr_show, curr_max_store);
DEVICE_ATTR(force_set_chgcc_8971, 0644, curr_show, curr_store);

static ssize_t te_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct max8971 *me = dev_get_drvdata(dev);

	return snprintf(buf, 10, "%d\n", me->te);
}

static ssize_t te_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret;
	char str_curr[10];
	struct max8971 *me = dev_get_drvdata(dev);

	strcpy(str_curr, (const char *)buf);
	ret = kstrtoint((const char *)str_curr, 0, &me->te);
	if (ret)
		return ret;
	pr_info("Set me->te:%d\n", me->te);
	max8971_set_chgcc(me, me->charge_current_volatile);

	return count;
}
DEVICE_ATTR(te_8971, 0644, te_show, te_store);

static int max8971_charger_lge_probe(struct max8971 *me)
{
	int ret = 0;
	me->te = 1800000;
	me->batt = power_supply_get_by_name("battery");
	ret = device_create_file(me->dev, &dev_attr_at_charge_8971);
	if (ret < 0) {
		pr_err("%s:File dev_attr_at_charge_8971 creation failed: %d\n",
				__func__, ret);
		ret = -ENODEV;
		goto err_at_charge_8971;
	}

	ret = device_create_file(me->dev, &dev_attr_at_chcomp_8971);
	if (ret < 0) {
		pr_err("%s:File dev_attr_at_chcomp_8971 creation failed: %d\n",
				__func__, ret);
		ret = -ENODEV;
		goto err_at_chcomp_8971;
	}

	ret = device_create_file(me->dev, &dev_attr_at_pmrst_8971);
	if (ret < 0) {
		pr_err("%s:File device creation failed: %d\n", __func__, ret);
		ret = -ENODEV;
		goto err_at_pmrst_8971;
	}
	device_create_file(me->dev, &dev_attr_force_set_chgcc_8971);
	device_create_file(me->dev, &dev_attr_force_set_dci_8971);
	device_create_file(me->dev, &dev_attr_te_8971);

	return 0;
err_at_pmrst_8971:
	device_remove_file(me->dev, &dev_attr_at_pmrst_8971);
err_at_chcomp_8971:
	device_remove_file(me->dev, &dev_attr_at_chcomp_8971);
err_at_charge_8971:
	device_remove_file(me->dev, &dev_attr_at_charge_8971);
	pr_err("max8971_lge_probe failed!!!!!!");
	return ret;
}
#endif

static SIMPLE_DEV_PM_OPS(max8971_pm, max8971_suspend, max8971_resume);


static struct i2c_driver max8971_i2c_driver = {
	.driver.name			= DRIVER_NAME,
	.driver.owner			 = THIS_MODULE,
	.driver.pm				= &max8971_pm,

#ifdef CONFIG_OF
	.driver.of_match_table	= max8971_of_ids,
#endif /* CONFIG_OF */
	.id_table			= max8971_i2c_ids,
	.probe				= max8971_probe,
	.remove				= __devexit_p(max8971_remove),
};

static __init int max8971_init(void)
{
	return i2c_add_driver(&max8971_i2c_driver);
}
module_init(max8971_init);

static __exit void max8971_exit(void)
{
	i2c_del_driver(&max8971_i2c_driver);
}
module_exit(max8971_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_VERSION(DRIVER_VERSION);
