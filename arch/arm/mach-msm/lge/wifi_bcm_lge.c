#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_device.h>
#endif /* CONFIG_OF */
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>

#include <linux/of_gpio.h>

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/if.h>
#include <linux/random.h>
#include <asm/io.h>
#ifdef CONFIG_WIFI_CONTROL_FUNC
#include <linux/skbuff.h>
#include <linux/wlan_plat.h>
#endif
#ifdef CONFIG_BCMDHD_PCIE
#include <linux/pci.h>
#endif

#include <linux/pm_qos.h>

#undef SUPPORT_DTS // for support DeviceTree

#if defined(CONFIG_MACH_MSM8974_G2_KR) || defined(CONFIG_MACH_MSM8974_VU3_KR) \
	|| defined(CONFIG_MACH_MSM8974_G2_ATT) || defined(CONFIG_MACH_MSM8974_G2_TEL_AU) \
	|| defined(CONFIG_MACH_MSM8974_G2_TMO_US) || defined(CONFIG_MACH_MSM8974_G2_VZW) || defined(CONFIG_MACH_MSM8974_G2_SPR) \
	|| defined(CONFIG_MACH_MSM8974_G2_DCM) || defined(CONFIG_MACH_MSM8974_G2_CA) || defined(CONFIG_MACH_MSM8974_G2_OPEN_COM) || defined(CONFIG_MACH_MSM8974_G2_OPT_AU) || defined(CONFIG_MACH_MSM8974_G2_OPEN_AME) \
	|| defined(CONFIG_MACH_MSM8974_G2_KDDI) || defined(CONFIG_MACH_MSM8974_TIGERS_KR) \
	|| defined(CONFIG_MACH_MSM8974_Z_KR) || defined(CONFIG_MACH_MSM8974_Z_US) || defined(CONFIG_MACH_MSM8974_Z_KDDI) || defined(CONFIG_MACH_MSM8974_Z_CN) || defined(CONFIG_MACH_MSM8974_Z_OPEN_COM) || defined(CONFIG_MACH_MSM8974_Z_CA) \
	|| defined(CONFIG_MACH_MSM8974_TIGERS)
// G2 or Z or G3A
#define WLAN_POWER	26
#define WLAN_HOSTWAKE	44
#else
// G3 or G3 cat6
#define WLAN_POWER	69
#define WLAN_HOSTWAKE	44
#endif
static int gpio_wlan_power = WLAN_POWER;
static int gpio_wlan_hostwake = WLAN_HOSTWAKE;

#ifdef SUPPORT_DTS
static struct pinctrl *wifi_reg_on_pinctrl = NULL;
#else
/* for wifi power supply */
static unsigned wifi_config_power_on[] = {
	    GPIO_CFG(WLAN_POWER, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA) };

static unsigned wlan_wakes_msm[] = {
	    GPIO_CFG(WLAN_HOSTWAKE, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA) };
#endif

#define LGE_BCM_WIFI_DMA_QOS_CONTROL

#if defined(CONFIG_BCM4335BT) 
extern int bcm_bt_lock(int cookie);
extern void bcm_bt_unlock(int cookie);
static int lock_cookie_wifi = 'W' | 'i'<<8 | 'F'<<16 | 'i'<<24; /* cookie is "WiFi" */
#endif // defined(CONFIG_BCM4335BT) 

/*
	Memory allocation is done at dhd_attach
	so static allocation is only necessary in module type driver
*/
#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM

#define PREALLOC_WLAN_NUMBER_OF_SECTIONS	12
#define PREALLOC_WLAN_NUMBER_OF_BUFFERS		160
#define PREALLOC_WLAN_SECTION_HEADER		24

/*
This definition is from driver's dhd.h

enum dhd_prealloc_index {
	DHD_PREALLOC_PROT = 0, 
	DHD_PREALLOC_RXBUF, 
	DHD_PREALLOC_DATABUF, 
	DHD_PREALLOC_OSL_BUF, 
#if defined(STATIC_WL_PRIV_STRUCT) 
	DHD_PREALLOC_WIPHY_ESCAN0 = 5, 
#if defined(CUSTOMER_HW4) && defined(DUAL_ESCAN_RESULT_BUFFER) 
	DHD_PREALLOC_WIPHY_ESCAN1, 
#endif 
#endif
	DHD_PREALLOC_DHD_INFO = 7 
	DHD_PREALLOC_DHD_WLFC_INFO = 8,
	DHD_PREALLOC_IF_FLOW_LKUP = 9,
	DHD_PREALLOC_FLOWRING = 10	
};
*/

#ifdef CONFIG_BCMDHD_SDIO
#define DHD_SKB_HDRSIZE			336
#define DHD_SKB_1PAGE_BUFSIZE		((PAGE_SIZE*1)-DHD_SKB_HDRSIZE)
#define DHD_SKB_2PAGE_BUFSIZE		((PAGE_SIZE*2)-DHD_SKB_HDRSIZE)
#define DHD_SKB_4PAGE_BUFSIZE		((PAGE_SIZE*4)-DHD_SKB_HDRSIZE)
#define WLAN_SKB_BUF_NUM		17

static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];
#endif

#define WLAN_SECTION_SKBUFF_IDX		4

#define WLAN_SECTION_SIZE_0		(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#ifdef CONFIG_BCMDHD_SDIO
#define WLAN_SECTION_SIZE_1		(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_2		(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 512)
#else
#define WLAN_SECTION_SIZE_1		0
#define WLAN_SECTION_SIZE_2		0
#endif
#define WLAN_SECTION_SIZE_3		(PREALLOC_WLAN_NUMBER_OF_BUFFERS*1024)
#define WLAN_SECTION_SIZE_4		0 /* Index 4 is static socket buffer */
#define WLAN_SECTION_SIZE_5		(65536)
#define WLAN_SECTION_SIZE_6		(65536)
#define WLAN_SECTION_SIZE_7		(16 * 1024)
#define WLAN_SECTION_SIZE_8		(64 * 1024) // 23032
#ifdef CONFIG_BCMDHD_SDIO
#define WLAN_SECTION_SIZE_9		0
#define WLAN_SECTION_SIZE_10		0
#else
#define WLAN_SECTION_SIZE_9		(18 * 1024) // 16338
#define WLAN_SECTION_SIZE_10		(32 * 1024)
#endif
#ifdef CONFIG_BCMDHD_SDIO
#define WLAN_SECTION_SIZE_11		(73760)	/* sizeof(WLFC_HANGER_SIZE(3072)) */
#else
#define WLAN_SECTION_SIZE_11		0
#endif

struct wlan_mem_prealloc {
	void *mem_ptr;
	unsigned long size;
};

static struct wlan_mem_prealloc wlan_mem_array[PREALLOC_WLAN_NUMBER_OF_SECTIONS] = {
	{ NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_1) },
	{ NULL, (WLAN_SECTION_SIZE_2) },
	{ NULL, (WLAN_SECTION_SIZE_3) },
	{ NULL, (WLAN_SECTION_SIZE_4) },       
	{ NULL, (WLAN_SECTION_SIZE_5) },
	{ NULL, (WLAN_SECTION_SIZE_6) },
	{ NULL, (WLAN_SECTION_SIZE_7) },
	{ NULL, (WLAN_SECTION_SIZE_8) },
	{ NULL, (WLAN_SECTION_SIZE_9) },
	{ NULL, (WLAN_SECTION_SIZE_10) },
	{ NULL, (WLAN_SECTION_SIZE_11) }
};

static void *bcm_wlan_get_mem(int section, unsigned long size)
{
#ifdef CONFIG_BCMDHD_SDIO
	if (section == WLAN_SECTION_SKBUFF_IDX)
		return wlan_static_skb;
#endif

	if ((section < 0) || (section > PREALLOC_WLAN_NUMBER_OF_SECTIONS))
		return NULL;

	if (wlan_mem_array[section].size < size)
		return NULL;

	return wlan_mem_array[section].mem_ptr;
}

static int brcm_init_wlan_mem(void)
{
	int i;
	int j;

#ifdef CONFIG_BCMDHD_SDIO
	for (i = 0; i < 8; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_1PAGE_BUFSIZE);
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}

	for (; i < 16; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_2PAGE_BUFSIZE);
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}

	wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_4PAGE_BUFSIZE);
	if (!wlan_static_skb[i])
		goto err_skb_alloc;
#endif

	for (i = 0 ; i < PREALLOC_WLAN_NUMBER_OF_SECTIONS; i++) {
		if ((i != WLAN_SECTION_SKBUFF_IDX) && (wlan_mem_array[i].size)) {
			wlan_mem_array[i].mem_ptr =
					kmalloc(wlan_mem_array[i].size, GFP_KERNEL);
			if (!wlan_mem_array[i].mem_ptr)
				goto err_mem_alloc;
		}
	}

	printk("%s: WIFI MEM Allocated\n", __FUNCTION__);
	return 0;

err_mem_alloc:
	pr_err("Failed to mem_alloc for WLAN\n");
	for (j = 0 ; j < i ; j++) {
		if ((j != WLAN_SECTION_SKBUFF_IDX) && wlan_mem_array[j].size) {
			kfree(wlan_mem_array[j].mem_ptr);
		}
	}

#ifdef CONFIG_BCMDHD_SDIO
err_skb_alloc:
	i = WLAN_SKB_BUF_NUM;
	pr_err("Failed to skb_alloc for WLAN\n");
	for (j = 0 ; j < i ; j++)
		dev_kfree_skb(wlan_static_skb[j]);
#endif
	return -ENOMEM;
}
#endif /* CONFIG_BROADCOM_WIFI_RESERVED_MEM */


#ifdef LGE_BCM_WIFI_DMA_QOS_CONTROL
static int wifi_dma_state; // 0 : INATIVE, 1:INIT, 2:IDLE, 3:ACTIVE
static struct pm_qos_request wifi_dma_qos;
static struct delayed_work req_dma_work;
static uint32_t packet_transfer_cnt = 0;

static void bcm_wifi_req_dma_work(struct work_struct * work)
{
	switch ( wifi_dma_state ) {
		case 2: //IDLE State
			if ( packet_transfer_cnt < 100 ) {
				// IDLE -> INIT
				wifi_dma_state = 1;
				//printk(KERN_ERR "%s: schedule work : %d : (IDLE -> INIT)\n", __func__, packet_transfer_cnt);
			}
			else {
				// IDLE -> ACTIVE
				wifi_dma_state = 3;
				pm_qos_update_request(&wifi_dma_qos, 7);
				schedule_delayed_work(&req_dma_work, msecs_to_jiffies(50));
				//printk(KERN_ERR "%s: schedule work : %d : (IDLE -> ACTIVE)\n", __func__, packet_transfer_cnt);
			}
			break;

		case 3: //ACTIVE State
			if ( packet_transfer_cnt < 10 ) {
				// ACTIVE -> IDLE
				wifi_dma_state = 2;
				pm_qos_update_request(&wifi_dma_qos, PM_QOS_DEFAULT_VALUE);
				schedule_delayed_work(&req_dma_work, msecs_to_jiffies(1000));
				//printk(KERN_ERR "%s: schedule work : %d : (ACTIVE -> IDLE)\n", __func__, packet_transfer_cnt);
			}
			else {
				// Keep ACTIVE
				schedule_delayed_work(&req_dma_work, msecs_to_jiffies(50));
				//printk(KERN_ERR "%s: schedule work : %d :  (ACTIVE -> ACTIVE)\n", __func__, packet_transfer_cnt);
			}
			break;

		default:
			break;
		
	}

	packet_transfer_cnt = 0;
}

void bcm_wifi_req_dma_qos(int vote)
{
	if (vote) {
		packet_transfer_cnt++;
	}

	// INIT -> IDLE
	if ( wifi_dma_state == 1 && vote ) {
		wifi_dma_state = 2; // IDLE
		schedule_delayed_work(&req_dma_work, msecs_to_jiffies(1000));
		//printk(KERN_ERR "%s: schedule work (INIT -> IDLE)\n", __func__);
	}
}
#endif

int bcm_wifi_reinit_gpio( void )
{
#if defined(CONFIG_MACH_MSM8974_G2_KR) || defined(CONFIG_MACH_MSM8974_VU3_KR) \
	|| defined(CONFIG_MACH_MSM8974_G2_ATT) || defined(CONFIG_MACH_MSM8974_G2_TEL_AU) \
	|| defined(CONFIG_MACH_MSM8974_G2_TMO_US) || defined(CONFIG_MACH_MSM8974_G2_VZW) || defined(CONFIG_MACH_MSM8974_G2_SPR) \
	|| defined(CONFIG_MACH_MSM8974_G2_DCM) || defined(CONFIG_MACH_MSM8974_G2_CA) || defined(CONFIG_MACH_MSM8974_G2_OPEN_COM) || defined(CONFIG_MACH_MSM8974_G2_OPT_AU) || defined(CONFIG_MACH_MSM8974_G2_OPEN_AME) \
	|| defined(CONFIG_MACH_MSM8974_G2_KDDI) || defined(CONFIG_MACH_MSM8974_TIGERS_KR) \
	|| defined(CONFIG_MACH_MSM8974_Z_KR) || defined(CONFIG_MACH_MSM8974_Z_US) || defined(CONFIG_MACH_MSM8974_Z_KDDI) || defined(CONFIG_MACH_MSM8974_Z_CN) || defined(CONFIG_MACH_MSM8974_Z_OPEN_COM) || defined(CONFIG_MACH_MSM8974_Z_CA)
	int ret = 0;

	wifi_config_power_on[0] = GPIO_CFG(gpio_wlan_power, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA);
	wlan_wakes_msm[0] = GPIO_CFG(gpio_wlan_hostwake, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);

	//WLAN_POWER
	ret = gpio_tlmm_config(wifi_config_power_on[0], GPIO_CFG_ENABLE);
	if (ret) {
		printk(KERN_ERR "%s: Failed to configure WiFi Reset GPIO:[%d]\n", __func__, ret);
	}
	//HOST_WAKEUP
	ret = gpio_tlmm_config(wlan_wakes_msm[0], GPIO_CFG_ENABLE);
	if (ret) {
		printk(KERN_ERR "%s: Failed to configure Hostwakeup:[%d]\n",__func__, ret);
	}
	printk(KERN_ERR "%s: gpio_power=%d, gpio_hostwakeup=%d\n", __func__, gpio_wlan_power, gpio_wlan_hostwake);
#endif
	return 0;
}

int bcm_wifi_set_power(int enable)
{
	int ret = 0;
	static int is_initialized = 0;

	if (is_initialized == 0) {
		bcm_wifi_reinit_gpio();
		mdelay(10);
		is_initialized = 1;
	}
		
#if defined(CONFIG_BCM4335BT) 
	printk("%s: trying to acquire BT lock\n", __func__);
	if (bcm_bt_lock(lock_cookie_wifi) != 0)
		printk("%s:** WiFi: timeout in acquiring bt lock**\n", __func__);
	else 
		printk("%s: btlock acquired\n", __func__);
#endif // defined(CONFIG_BCM4335BT) 


	if (enable) {
		ret = gpio_direction_output(gpio_wlan_power, 1);
		if (ret) {
			printk(KERN_ERR "%s: WL_REG_ON  failed to pull up (%d)\n",
					__func__, ret);
			ret = -EIO;
			goto out;
		}

		// WLAN chip to reset
		mdelay(150);
		printk(KERN_ERR "%s: wifi power successed to pull up\n",__func__);
	} else {
		ret = gpio_direction_output(gpio_wlan_power, 0); 
		if (ret) {
			printk(KERN_ERR "%s:  WL_REG_ON  failed to pull down (%d)\n",
					__func__, ret);
			ret = -EIO;
			goto out;
		}
		// WLAN chip down 
		printk(KERN_ERR "%s: wifi power successed to pull down\n",__func__);
	}

#if defined(CONFIG_BCM4335BT) 
	bcm_bt_unlock(lock_cookie_wifi);
#endif // defined(CONFIG_BCM4335BT) 

	return ret;

out : 
#if defined(CONFIG_BCM4335BT) 
	/* For a exceptional case, release btlock */
	printk("%s: exceptional bt_unlock\n", __func__);
	bcm_bt_unlock(lock_cookie_wifi);
#endif // defined(CONFIG_BCM4335BT) 

	return ret;
}

static int bcm_wifi_reset(int on)
{
	return 0;
}

#ifdef CONFIG_BCMDHD_SDIO
static unsigned int g_wifi_detect;
static void *sdc_dev;
void (*sdc_status_cb)(int card_present, void *dev);

int wcf_status_register(void (*cb)(int card_present, void *dev), void *dev)
{
	pr_info("%s\n", __func__);

	if (sdc_status_cb)
		return -EINVAL;

	sdc_status_cb = cb;
	sdc_dev = dev;

	return 0;
}
#endif

unsigned int wcf_status(struct device *dev)
{
	pr_info("%s: wifi_detect = %d\n", __func__, g_wifi_detect);
	return g_wifi_detect;
}

static int bcm_wifi_carddetect(int val)
{
	int ret = 0;

#ifdef CONFIG_BCMDHD_SDIO

	g_wifi_detect = val;

	if (sdc_status_cb)
		sdc_status_cb(val, sdc_dev);
	else
		pr_warn("%s: There is no callback for notify\n", __func__);
	return ret;

#elif defined(CONFIG_BCMDHD_PCIE)

#define PCIE_VENDOR_ID_RCP	0x17cb
#define PCIE_DEVICE_ID_RCP	0x0300
#define PCIE_RCP_NAME		"0001:00:00.0"
#define PCIE_POWERUP_RETRY	10

	int found = 0;
	int count = 0;
	struct pci_dev *pcidev = NULL;

	if (val == 1) {
		do {
			pcidev = pci_get_device(PCIE_VENDOR_ID_RCP, PCIE_DEVICE_ID_RCP, pcidev);
			if (pcidev && (!strcmp(pci_name(pcidev), (const char *)PCIE_RCP_NAME))) {
				printk("P:%s:PCI device found[%X:%X]!!!\n", __func__, pcidev->vendor, pcidev->device);
				found = 1;
			} else {
				count++;
				printk("P:%s:retry count[%d]\n", __func__, count);
				msleep(100);
			}
		} while(!found && (count < PCIE_POWERUP_RETRY));

		if (!found) {
			ret = -1;
		}
	}

#endif
	return ret;
}

static int bcm_wifi_get_mac_addr(unsigned char *buf)
{
	uint rand_mac;
	static unsigned char mymac[6] = {0,};
	const unsigned char nullmac[6] = {0,};
	pr_debug("%s: %p\n", __func__, buf);

	if( buf == NULL ) return -EAGAIN;

	if( memcmp( mymac, nullmac, 6 ) != 0 )
	{
		/* Mac displayed from UI are never updated..
		   So, mac obtained on initial time is used */
		memcpy( buf, mymac, 6 );
		return 0;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0))
	prandom_seed((uint)jiffies);
	rand_mac = prandom_u32();
#else
	srandom32((uint)jiffies);
	rand_mac = random32();
#endif
	buf[0] = 0x00;
	buf[1] = 0x90;
	buf[2] = 0x4c;
	buf[3] = (unsigned char)rand_mac;
	buf[4] = (unsigned char)(rand_mac >> 8);
	buf[5] = (unsigned char)(rand_mac >> 16);

	memcpy(mymac, buf, 6);

	printk(KERN_INFO "[%s] Exiting. MyMac :  %x : %x : %x : %x : %x : %x\n", __func__ , buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);

	return 0;
}

#define COUNTRY_BUF_SZ	4
struct cntry_locales_custom {
	char iso_abbrev[COUNTRY_BUF_SZ];
	char custom_locale[COUNTRY_BUF_SZ];
	int custom_locale_rev;
};

/* Customized Locale table */
#ifdef CONFIG_BCM4335
const struct cntry_locales_custom bcm_wifi_translate_custom_table[] = {
/* Table should be filled out based on custom platform regulatory requirement */
	{"",   "XZ", 11},	/* Universal if Country code is unknown or empty */
	{"IR", "XZ", 11},	/* Universal if Country code is IRAN, (ISLAMIC REPUBLIC OF) */
	{"SD", "XZ", 11},	/* Universal if Country code is SUDAN */
	{"SY", "XZ", 11},	/* Universal if Country code is SYRIAN ARAB REPUBLIC */
	{"GL", "XZ", 11},	/* Universal if Country code is GREENLAND */
	{"PS", "XZ", 11},	/* Universal if Country code is PALESTINIAN TERRITORY, OCCUPIED */
	{"TL", "XZ", 11},	/* Universal if Country code is TIMOR-LESTE (EAST TIMOR) */
	{"MH", "XZ", 11},	/* Universal if Country code is MARSHALL ISLANDS */
	{"PK", "XZ", 11},	/* Universal if Country code is PAKISTAN */
	{"CK", "XZ", 11},	/* Universal if Country code is Cook Island (13.4.27)*/
	{"CU", "XZ", 11},	/* Universal if Country code is Cuba (13.4.27)*/
	{"FK", "XZ", 11},	/* Universal if Country code is Falkland Island (13.4.27)*/
	{"FO", "XZ", 11},	/* Universal if Country code is Faroe Island (13.4.27)*/
	{"GI", "XZ", 11},	/* Universal if Country code is Gibraltar (13.4.27)*/
	{"IM", "XZ", 11},	/* Universal if Country code is Isle of Man (13.4.27)*/
	{"CI", "XZ", 11},	/* Universal if Country code is Ivory Coast (13.4.27)*/
	{"JE", "XZ", 11},	/* Universal if Country code is Jersey (13.4.27)*/
	{"KP", "XZ", 11},	/* Universal if Country code is North Korea (13.4.27)*/
	{"FM", "XZ", 11},	/* Universal if Country code is Micronesia (13.4.27)*/
	{"MM", "XZ", 11},	/* Universal if Country code is Myanmar (13.4.27)*/
	{"NU", "XZ", 11},	/* Universal if Country code is Niue (13.4.27)*/
	{"NF", "XZ", 11},	/* Universal if Country code is Norfolk Island (13.4.27)*/
	{"PN", "XZ", 11},	/* Universal if Country code is Pitcairn Islands (13.4.27)*/
	{"PM", "XZ", 11},	/* Universal if Country code is Saint Pierre and Miquelon (13.4.27)*/
	{"SS", "XZ", 11},	/* Universal if Country code is South_Sudan (13.4.27)*/
	{"AL", "AL", 2},
	{"DZ", "DZ", 1},
	{"AS", "AS", 12},  /* changed 2 -> 12*/
	{"AI", "AI", 1},
	{"AG", "AG", 2},
	{"AR", "AR", 21},
	{"AW", "AW", 2},
	{"AU", "AU", 6},
	{"AT", "AT", 4},
	{"AZ", "AZ", 2},
	{"BS", "BS", 2},
	{"BH", "BH", 4},  /* changed 24 -> 4*/
	{"BD", "BD", 2},
	{"BY", "BY", 3},
	{"BE", "BE", 4},
	{"BM", "BM", 12},
	{"BA", "BA", 2},
	{"BR", "BR", 4},
	{"VG", "VG", 2},
	{"BN", "BN", 4},
	{"BG", "BG", 4},
	{"KH", "KH", 2},
	{"CA", "CA", 31},
	{"KY", "KY", 3},
	{"CN", "CN", 24},
	{"CO", "CO", 17},
	{"CR", "CR", 17},
	{"HR", "HR", 4},
	{"CY", "CY", 4},
	{"CZ", "CZ", 4},
	{"DK", "DK", 4},
	{"EE", "EE", 4},
	{"ET", "ET", 2},
	{"FI", "FI", 4},
	{"FR", "FR", 5},
	{"GF", "GF", 2},
	{"DE", "DE", 7},
	{"GR", "GR", 4},
	{"GD", "GD", 2},
	{"GP", "GP", 2},
	{"GU", "GU", 12},
	{"HK", "HK", 2},
	{"HU", "HU", 4},
	{"IS", "IS", 4},
	{"IN", "IN", 3},
	{"ID", "ID", 1},
	{"IE", "IE", 5},
	{"IL", "IL", 7},
	{"IT", "IT", 4},
#if defined(CONFIG_MACH_MSM8974_G2_DCM)
	{"JP", "JP", 45},
#else
	{"JP", "JP", 58},
#endif
	{"JO", "JO", 3},
	{"KW", "KW", 5},
	{"LA", "LA", 2},
	{"LV", "LV", 4},
	{"LB", "LB", 5},
	{"LS", "LS", 2},
	{"LI", "LI", 4},
	{"LT", "LT", 4},
	{"LU", "LU", 3},
	{"MO", "MO", 2},
	{"MK", "MK", 2},
	{"MW", "MW", 1},
	{"MY", "MY", 3},
	{"MV", "MV", 3},
	{"MT", "MT", 4},
	{"MQ", "MQ", 2},
	{"MR", "MR", 2},
	{"MU", "MU", 2},
	{"YT", "YT", 2},
	{"MX", "MX", 20},
	{"MD", "MD", 2},
	{"MC", "MC", 1},
	{"ME", "ME", 2},
	{"MA", "MA", 2},
	{"NP", "NP", 3},
	{"NL", "NL", 4},
	{"AN", "AN", 2},
	{"NZ", "NZ", 4},
	{"NO", "NO", 4},
	{"OM", "OM", 4},
	{"PA", "PA", 17},
	{"PG", "PG", 2},
	{"PY", "PY", 2},
	{"PE", "PE", 20},
	{"PH", "PH", 5},
	{"PL", "PL", 4},
	{"PT", "PT", 4},
	{"PR", "PR", 20},
	{"RE", "RE", 2},
	{"RO", "RO", 4},
	{"SN", "SN", 2},
	{"RS", "RS", 2},
	{"SG", "SG", 4},
	{"SK", "SK", 4},
	{"SI", "SI", 4},
	{"ES", "ES", 4},
	{"LK", "LK", 1},
	{"SE", "SE", 4},
	{"CH", "CH", 4},
	{"TW", "TW", 1},
	{"TH", "TH", 5},
	{"TT", "TT", 3},
	{"TR", "TR", 7},
	{"AE", "AE", 6},
	{"UG", "UG", 2},
	{"GB", "GB", 6},
	{"UY", "UY", 1},
	{"VI", "VI", 13},
	{"VA", "VA", 12},   /* changed 2 -> 12*/
	{"VE", "VE", 3},
	{"VN", "VN", 4},
	{"MA", "MA", 1},
	{"ZM", "ZM", 2},
	{"EC", "EC", 21},
	{"SV", "SV", 19},
	{"KR", "KR", 57},
	{"RU", "RU", 13},
	{"UA", "UA", 8},
	{"GT", "GT", 1},
	{"MN", "MN", 1},
	{"NI", "NI", 2},
	{"US", "US", 118},
};
#else
const struct cntry_locales_custom bcm_wifi_translate_custom_table[] = {
/* Table should be filled out based on custom platform regulatory requirement */
	{"",   "XZ", 11},	/* Universal if Country code is unknown or empty */
	{"CK", "XZ", 11},	/* Universal if Country code is Cook Island (13.4.27)*/
	{"CU", "XZ", 11},	/* Universal if Country code is Cuba (13.4.27)*/
	{"FO", "XZ", 11},	/* Universal if Country code is Faroe Island (13.4.27)*/
	{"GI", "XZ", 11},	/* Universal if Country code is Gibraltar (13.4.27)*/
	{"IM", "XZ", 11},	/* Universal if Country code is Isle of Man (13.4.27)*/
	{"IR", "XZ", 11},	/* Universal if Country code is IRAN, (ISLAMIC REPUBLIC OF) */
	{"JE", "XZ", 11},	/* Universal if Country code is Jersey (13.4.27)*/
	{"KP", "XZ", 11},	/* Universal if Country code is North Korea (13.4.27)*/
	{"MH", "XZ", 11},	/* Universal if Country code is MARSHALL ISLANDS */
	{"NF", "XZ", 11},	/* Universal if Country code is Norfolk Island (13.4.27)*/
	{"NU", "XZ", 11},	/* Universal if Country code is Niue (13.4.27)*/
	{"PM", "XZ", 11},	/* Universal if Country code is Saint Pierre and Miquelon (13.4.27)*/
	{"PN", "XZ", 11},	/* Universal if Country code is Pitcairn Islands (13.4.27)*/
	{"PS", "XZ", 11},	/* Universal if Country code is PALESTINIAN TERRITORY, OCCUPIED */
	{"SD", "XZ", 11},	/* Universal if Country code is SUDAN */
	{"SS", "XZ", 11},	/* Universal if Country code is South_Sudan (13.4.27)*/
	{"SY", "XZ", 11},	/* Universal if Country code is SYRIAN ARAB REPUBLIC */
	{"TL", "XZ", 11},	/* Universal if Country code is TIMOR-LESTE (EAST TIMOR) */
	{"AD", "AD", 0},
	{"AE", "AE", 6},
	{"AF", "AF", 0},
	{"AG", "AG", 2},
	{"AI", "AI", 1},
	{"AL", "AL", 2},
	{"AM", "AM", 0},
	{"AN", "AN", 3},
	{"AO", "AO", 0},
	{"AR", "AU", 6},
	{"AS", "AS", 12},  /* changed 2 -> 12*/
	{"AT", "AT", 4},
	{"AU", "AU", 6},
	{"AW", "AW", 2},
	{"AZ", "AZ", 2},
	{"BA", "BA", 2},
	{"BB", "BB", 0},
	{"BD", "BD", 2},
	{"BE", "BE", 4},
	{"BF", "BF", 0},
	{"BG", "BG", 4},
	{"BH", "BH", 4},
	{"BI", "BI", 0},
	{"BJ", "BJ", 0},
	{"BM", "BM", 12},
	{"BN", "BN", 4},
	{"BO", "BO", 0},
	{"BR", "BR", 15},
	{"BS", "BS", 2},
	{"BT", "BT", 0},
	{"BW", "BW", 0},
	{"BY", "BY", 3},
	{"BZ", "BZ", 0},
	{"CA", "US", 118},
	{"CD", "CD", 0},
	{"CF", "CF", 0},
	{"CG", "CG", 0},
	{"CH", "CH", 4},
	{"CI", "CI", 0},
	{"CL", "CL", 0},
	{"CM", "CM", 0},
	{"CN", "CN", 38},
	{"CO", "CO", 17},
	{"CR", "CR", 17},
	{"CV", "CV", 0},
	{"CX", "CX", 0},
	{"CY", "CY", 4},
	{"CZ", "CZ", 4},
	{"DE", "DE", 7},
	{"DJ", "DJ", 0},
	{"DK", "DK", 4},
	{"DM", "DM", 0},
	{"DO", "DO", 0},
	{"DZ", "DZ", 1},
	{"EC", "EC", 21},
	{"EE", "EE", 4},
	{"EG", "EG", 0},
	{"ER", "ER", 0},
	{"ES", "ES", 4},
	{"ET", "ET", 2},
	{"FI", "FI", 4},
	{"FJ", "FJ", 0},
	{"FK", "FK", 0},
	{"FM", "FM", 0},
	{"FR", "FR", 5},
	{"GA", "GA", 0},
	{"GB", "GB", 6},
	{"GD", "GD", 2},
	{"GE", "GE", 0},
	{"GF", "GF", 2},
	{"GH", "GH", 0},
	{"GM", "GM", 0},
	{"GN", "GN", 0},
	{"GP", "GP", 2},
	{"GQ", "GQ", 0},
	{"GR", "GR", 4},
	{"GT", "GT", 1},
	{"GU", "GU", 12},
	{"GW", "GW", 0},
	{"GY", "GY", 0},
	{"HK", "HK", 2},
	{"HN", "HN", 0},
	{"HR", "HR", 4},
	{"HT", "HT", 0},
	{"HU", "HU", 4},
	{"ID", "ID", 1},
	{"IE", "IE", 5},
	{"IL", "IL", 7},
	{"IN", "IN", 3},
	{"IQ", "IQ", 0},
	{"IS", "IS", 4},
	{"IT", "IT", 4},
	{"JM", "JM", 0},
	{"JO", "JO", 3},
	{"JP", "JP", 58},
	{"KE", "SA", 0},
	{"KG", "KG", 0},
	{"KH", "KH", 2},
	{"KI", "KI", 0},
	{"KM", "KM", 0},
	{"KN", "KN", 0},
	{"KR", "KR", 57},
	{"KW", "KW", 5},
	{"KY", "KY", 3},
	{"KZ", "KZ", 0},
	{"LA", "LA", 2},
	{"LB", "LB", 5},
	{"LC", "LC", 0},
	{"LI", "LI", 4},
	{"LK", "LK", 1},
	{"LR", "LR", 0},
	{"LS", "LS", 2},
	{"LT", "LT", 4},
	{"LU", "LU", 3},
	{"LV", "LV", 4},
	{"LY", "LY", 0},
	{"MA", "MA", 2},
	{"MC", "MC", 1},
	{"MD", "MD", 2},
	{"ME", "ME", 2},
	{"MF", "MF", 0},
	{"MG", "MG", 0},
	{"MK", "MK", 2},
	{"ML", "ML", 0},
	{"MM", "MM", 0},
	{"MN", "MN", 1},
	{"MO", "MO", 2},
	{"MP", "MP", 0},
	{"MQ", "MQ", 2},
	{"MR", "MR", 2},
	{"MS", "MS", 0},
	{"MT", "MT", 4},
	{"MU", "MU", 2},
	{"MV", "MV", 3},
	{"MW", "MW", 1},
	{"MX", "AU", 6},
	{"MY", "MY", 3},
	{"MZ", "MZ", 0},
	{"NA", "NA", 0},
	{"NC", "NC", 0},
	{"NE", "NE", 0},
	{"NG", "NG", 0},
	{"NI", "NI", 2},
	{"NL", "NL", 4},
	{"NO", "NO", 4},
	{"NP", "ID", 5},
	{"NR", "NR", 0},
	{"NZ", "NZ", 4},
	{"OM", "OM", 4},
	{"PA", "PA", 17},
	{"PE", "PE", 20},
	{"PF", "PF", 0},
	{"PG", "AU", 6},
	{"PH", "PH", 5},
	{"PK", "PK", 0},
	{"PL", "PL", 4},
	{"PR", "US", 118},
	{"PT", "PT", 4},
	{"PW", "PW", 0},
	{"PY", "PY", 2},
	{"QA", "QA", 0},
	{"RE", "RE", 2},
	{"RKS", "KG", 0},
	{"RO", "RO", 4},
	{"RS", "RS", 2},
	{"RU", "RU", 13},
	{"RW", "RW", 0},
	{"SA", "SA", 0},
	{"SB", "SB", 0},
	{"SC", "SC", 0},
	{"SE", "SE", 4},
	{"SG", "SG", 0},
	{"SI", "SI", 4},
	{"SK", "SK", 4},
	{"SL", "SL", 0},
	{"SM", "SM", 0},
	{"SN", "SN", 2},
	{"SO", "SO", 0},
	{"SR", "SR", 0},
	{"ST", "ST", 0},
	{"SV", "SV", 25},
	{"SZ", "SZ", 0},
	{"TC", "TC", 0},
	{"TD", "TD", 0},
	{"TF", "TF", 0},
	{"TG", "TG", 0},
	{"TH", "TH", 5},
	{"TJ", "TJ", 0},
	{"TM", "TM", 0},
	{"TN", "TN", 0},
	{"TO", "TO", 0},
	{"TR", "TR", 7},
	{"TT", "TT", 3},
	{"TV", "TV", 0},
	{"TW", "TW", 1},
	{"TZ", "TZ", 0},
	{"UA", "UA", 8},
	{"UG", "UG", 2},
	{"US", "US", 118},
	{"UY", "UY", 1},
	{"UZ", "MA", 2},
	{"VA", "VA", 2},
	{"VC", "VC", 0},
	{"VE", "VE", 3},
	{"VG", "VG", 2},
	{"VI", "US", 118},
	{"VN", "VN", 4},
	{"VU", "VU", 0},
	{"WS", "WS", 0},
	{"YE", "YE", 0},
	{"YT", "YT", 2},
	{"ZA", "ZA", 6},
	{"ZM", "LA", 2},
	{"ZW", "ZW", 0},
};
#endif

static void *bcm_wifi_get_country_code(char *ccode)
{
	int size, i;
	static struct cntry_locales_custom country_code;

	size = ARRAY_SIZE(bcm_wifi_translate_custom_table);

	if ((size == 0) || (ccode == NULL))
		return NULL;

	for (i = 0; i < size; i++) {
		if (strcmp(ccode, bcm_wifi_translate_custom_table[i].iso_abbrev) == 0) {
			return (void *)&bcm_wifi_translate_custom_table[i];
		}
	}   

	memset(&country_code, 0, sizeof(struct cntry_locales_custom));
	strlcpy(country_code.custom_locale, ccode, COUNTRY_BUF_SZ);

	return (void *)&country_code;
}

static struct wifi_platform_data bcm_wifi_control = {
#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
	.mem_prealloc	= bcm_wlan_get_mem,
#endif /* CONFIG_BROADCOM_WIFI_RESERVED_MEM */
	.set_power		= bcm_wifi_set_power,
	.set_reset      = bcm_wifi_reset,
	.set_carddetect = bcm_wifi_carddetect,
	.get_mac_addr   = bcm_wifi_get_mac_addr, 
	.get_country_code = bcm_wifi_get_country_code,
};

static struct resource wifi_resource[] = {
	[0] = {
		.name = "bcmdhd_wlan_irq",
		.start = 0,  //assigned later
		.end   = 0,  //assigned later
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL | IORESOURCE_IRQ_SHAREABLE, // for HW_OOB
		//.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE | IORESOURCE_IRQ_LOWEDGE | IORESOURCE_IRQ_SHAREABLE, // for SW_OOB
	},
};

static struct platform_device bcm_wifi_device = {
	.name           = "bcmdhd_wlan",
	.id             = 1,
	.num_resources  = ARRAY_SIZE(wifi_resource),
	.resource       = wifi_resource,
	.dev            = {
		.platform_data = &bcm_wifi_control,
	},
};

int bcm_wifi_init_mem(struct platform_device *platdev)
{
#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
	brcm_init_wlan_mem();
#endif
	printk(KERN_INFO "bcm_wifi_init_mem successfully\n");

	return 0;
}

int bcm_wifi_init_gpio(struct platform_device *platdev)
{
	int ret = 0;
#ifdef SUPPORT_DTS
	struct device_node *np = platdev->dev.of_node;

	wifi_reg_on_pinctrl = devm_pinctrl_get(&platdev->dev);
	if (IS_ERR_OR_NULL(wifi_reg_on_pinctrl)) {
		printk("%s: target does not use pinctrl for wifi reg on\n", __func__);
	}

	gpio_wlan_power = of_get_named_gpio_flags(np, "wlan-en-gpio", 0, NULL);
	printk(KERN_INFO "%s: gpio_wlan_power : %d\n", __FUNCTION__, gpio_wlan_power);
	if (!gpio_is_valid(gpio_wlan_power)) {
		printk("P:%s:gpio %d for reset is not valid.\n", __func__, gpio_wlan_power);
	}

	gpio_wlan_hostwake = of_get_named_gpio_flags(np, "wlan-hostwake-gpio", 0, NULL);
	printk(KERN_INFO "%s: gpio_wlan_hostwake : %d\n", __FUNCTION__, gpio_wlan_hostwake);
	if (!gpio_is_valid(gpio_wlan_hostwake)) {
		printk("P:%s:gpio %d for reset is not valid.\n", __func__, gpio_wlan_hostwake);
	}
#else
	wifi_config_power_on[0] = GPIO_CFG(gpio_wlan_power, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA);
	wlan_wakes_msm[0] = GPIO_CFG(gpio_wlan_hostwake, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);

	//WLAN_POWER
	ret = gpio_tlmm_config(wifi_config_power_on[0], GPIO_CFG_ENABLE);
	if (ret) {
		printk(KERN_ERR "%s: Failed to configure WiFi Reset GPIO:[%d]\n", __func__, ret);
	}
	//HOST_WAKEUP
	ret = gpio_tlmm_config(wlan_wakes_msm[0], GPIO_CFG_ENABLE);	
	if (ret) {
		printk(KERN_ERR "%s: Failed to configure Hostwakeup:[%d]\n",__func__, ret);
	}
#endif

	/* WLAN_POWER */
	if ((ret = gpio_request_one(gpio_wlan_power, GPIOF_OUT_INIT_LOW, "wifi_reg_on")) < 0)
		printk("%s: Failed to request gpio %d for bcmdhd_wifi_reg_on:[%d]\n", __func__, gpio_wlan_power, ret);

	msleep(10);

#if defined(CONFIG_BCMDHD_SDIO) && defined(SUPPORT_DTS)
	gpio_free(gpio_wlan_power); // for cd-gpios
#endif

	ret = gpio_request_one(gpio_wlan_hostwake, GPIOF_IN, "wifi_hostwakeup");
	if (ret) {
		printk("Failed to request gpio %d for wlan_wakes_msm:[%d]\n", gpio_wlan_hostwake, ret);
	}

	if (gpio_is_valid(gpio_wlan_hostwake)) {
		wifi_resource[0].start = wifi_resource[0].end = gpio_to_irq(gpio_wlan_hostwake);
	}

	printk(KERN_INFO "bcm_wifi_init_gpio successfully\n");

	return 0;
}

#ifdef SUPPORT_DTS
static int bcm_wifi_probe(struct platform_device *pdev)
{
	bcm_wifi_init_mem(pdev);
	bcm_wifi_init_gpio(pdev);

	return 0;
}

static int bcm_wifi_remove(struct platform_device *pdev)
{

	return 0;
}

static struct of_device_id bcm_wifi_match_table[] = {
	{ .compatible = "lge,bcmdhd_wlan" },
	{ },
};

static struct platform_driver bcm_wifi_driver = {
	.probe = bcm_wifi_probe,
	.remove = bcm_wifi_remove,
	.driver = {
		.name = "wifi_bcm_lge",
		.owner = THIS_MODULE,
		.of_match_table = bcm_wifi_match_table,
	},
};
#endif

static int __init init_bcm_wifi(void)
{
#ifdef CONFIG_WIFI_CONTROL_FUNC

#ifdef LGE_BCM_WIFI_DMA_QOS_CONTROL
	INIT_DELAYED_WORK(&req_dma_work, bcm_wifi_req_dma_work);
	pm_qos_add_request(&wifi_dma_qos, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);
	wifi_dma_state = 1; //INIT
	printk("%s: wifi_dma_qos is added\n", __func__);
#endif

	platform_device_register(&bcm_wifi_device);
#ifdef SUPPORT_DTS
	platform_driver_register(&bcm_wifi_driver);
#else
	bcm_wifi_init_mem(NULL);
	bcm_wifi_init_gpio(NULL);
#endif
#endif

	return 0;
}

subsys_initcall(init_bcm_wifi);

