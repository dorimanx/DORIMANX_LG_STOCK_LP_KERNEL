/*
 * Gadget Function Driver for MTP
 *
 * Copyright (C) 2010 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/* #define DEBUG */
/* #define VERBOSE_DEBUG */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/interrupt.h>

#include <linux/types.h>
#include <linux/file.h>
#include <linux/device.h>
#include <linux/miscdevice.h>

#include <linux/usb.h>
#include <linux/usb_usual.h>
#include <linux/usb/ch9.h>
#include <linux/usb/f_mtp.h>
#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif

#if defined CONFIG_DEBUG_FS && defined CONFIG_USB_G_LGE_ANDROID
#define CONFIG_USB_G_LGE_MTP_PROFILING
#endif

#define MTP_BULK_BUFFER_SIZE       16384
#define INTR_BUFFER_SIZE           28

/* String IDs */
#define INTERFACE_STRING_INDEX	0

/* values for mtp_dev.state */
#define STATE_OFFLINE               0   /* initial state, disconnected */
#define STATE_READY                 1   /* ready for userspace calls */
#define STATE_BUSY                  2   /* processing userspace calls */
#define STATE_CANCELED              3   /* transaction canceled by host */
#define STATE_ERROR                 4   /* error from completion routine */

/* number of tx and rx requests to allocate */
#define MTP_TX_REQ_MAX 8
#define RX_REQ_MAX 2
#define INTR_REQ_MAX 5

/* ID for Microsoft MTP OS String */
#define MTP_OS_STRING_ID   0xEE

/* MTP class reqeusts */
#define MTP_REQ_CANCEL              0x64
#define MTP_REQ_GET_EXT_EVENT_DATA  0x65
#define MTP_REQ_RESET               0x66
#define MTP_REQ_GET_DEVICE_STATUS   0x67

/* constants for device status */
#define MTP_RESPONSE_OK             0x2001
#define MTP_RESPONSE_DEVICE_BUSY    0x2019

#ifdef CONFIG_USB_G_LGE_ANDROID
unsigned int mtp_rx_req_len = 262144;
#else
unsigned int mtp_rx_req_len = MTP_BULK_BUFFER_SIZE;
#endif
module_param(mtp_rx_req_len, uint, S_IRUGO | S_IWUSR);

unsigned int mtp_tx_req_len = MTP_BULK_BUFFER_SIZE;
module_param(mtp_tx_req_len, uint, S_IRUGO | S_IWUSR);

#ifdef CONFIG_USB_G_LGE_ANDROID
unsigned int mtp_tx_reqs = 32;
#else
unsigned int mtp_tx_reqs = MTP_TX_REQ_MAX;
#endif
module_param(mtp_tx_reqs, uint, S_IRUGO | S_IWUSR);

static const char mtp_shortname[] = "mtp_usb";

struct mtp_dev {
	struct usb_function function;
#ifdef CONFIG_USB_G_LGE_MULTIPLE_CONFIGURATION
	struct usb_function function2;
#endif
	struct usb_composite_dev *cdev;
	spinlock_t lock;

	struct usb_ep *ep_in;
	struct usb_ep *ep_out;
	struct usb_ep *ep_intr;

#ifdef CONFIG_USB_G_LGE_MULTIPLE_CONFIGURATION
	bool first_mtp_binded;
	int allocated_func;
	/*ep for swap*/
	struct usb_ep *ep_in1;
	struct usb_ep *ep_out1;
	struct usb_ep *ep_intr1;

	struct usb_ep *ep_in2;
	struct usb_ep *ep_out2;
	struct usb_ep *ep_intr2;
#endif

	int state;

	/* synchronize access to our device file */
	atomic_t open_excl;
	/* to enforce only one ioctl at a time */
	atomic_t ioctl_excl;

	struct list_head tx_idle;
	struct list_head intr_idle;

	wait_queue_head_t read_wq;
	wait_queue_head_t write_wq;
	wait_queue_head_t intr_wq;
	struct usb_request *rx_req[RX_REQ_MAX];
	int rx_done;

	/* for processing MTP_SEND_FILE, MTP_RECEIVE_FILE and
	 * MTP_SEND_FILE_WITH_HEADER ioctls on a work queue
	 */
	struct workqueue_struct *wq;
	struct work_struct send_file_work;
	struct work_struct receive_file_work;
	struct file *xfer_file;
	loff_t xfer_file_offset;
	int64_t xfer_file_length;
	unsigned xfer_send_header;
	uint16_t xfer_command;
	uint32_t xfer_transaction_id;
	int xfer_result;
#ifdef CONFIG_USB_G_LGE_MTP_PROFILING
	struct {
		uint64_t rbytes, t_rbytes;
		uint64_t wbytes, t_wbytes;
		ktime_t rtime, t_rtime;
		ktime_t wtime, t_wtime;
		ktime_t send_time, t_send_time;
		ktime_t receive_time, t_receive_time;
		int r_count, w_count;
		ktime_t first_start_rtime, first_start_wtime;
		ktime_t last_end_rtime, last_end_wtime;

	} perf;
#endif
};

static struct usb_interface_descriptor mtp_interface_desc = {
	.bLength                = USB_DT_INTERFACE_SIZE,
	.bDescriptorType        = USB_DT_INTERFACE,
	.bInterfaceNumber       = 0,
	.bNumEndpoints          = 3,
	.bInterfaceClass        = USB_CLASS_VENDOR_SPEC,
	.bInterfaceSubClass     = USB_SUBCLASS_VENDOR_SPEC,
	.bInterfaceProtocol     = 0,
};

static struct usb_interface_descriptor ptp_interface_desc = {
	.bLength                = USB_DT_INTERFACE_SIZE,
	.bDescriptorType        = USB_DT_INTERFACE,
	.bInterfaceNumber       = 0,
	.bNumEndpoints          = 3,
	.bInterfaceClass        = USB_CLASS_STILL_IMAGE,
	.bInterfaceSubClass     = 1,
	.bInterfaceProtocol     = 1,
};

#ifdef CONFIG_USB_G_LGE_MULTIPLE_CONFIGURATION
static struct usb_interface_descriptor mtp_interface_desc2 = {
	.bLength                = USB_DT_INTERFACE_SIZE,
	.bDescriptorType        = USB_DT_INTERFACE,
	.bInterfaceNumber       = 0,
	.bNumEndpoints          = 3,
	.bInterfaceClass        = USB_CLASS_VENDOR_SPEC,
	.bInterfaceSubClass     = USB_SUBCLASS_VENDOR_SPEC,
	.bInterfaceProtocol     = 0,
};

static struct usb_interface_descriptor ptp_interface_desc2 = {
	.bLength                = USB_DT_INTERFACE_SIZE,
	.bDescriptorType        = USB_DT_INTERFACE,
	.bInterfaceNumber       = 0,
	.bNumEndpoints          = 3,
	.bInterfaceClass        = USB_CLASS_STILL_IMAGE,
	.bInterfaceSubClass     = 1,
	.bInterfaceProtocol     = 1,
};
#endif

static struct usb_endpoint_descriptor mtp_superspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(1024),
};

static struct usb_ss_ep_comp_descriptor mtp_superspeed_in_comp_desc = {
	.bLength =		sizeof mtp_superspeed_in_comp_desc,
	.bDescriptorType =	USB_DT_SS_ENDPOINT_COMP,

	/* the following 2 values can be tweaked if necessary */
	.bMaxBurst =		2,
	/* .bmAttributes =	0, */
};

static struct usb_endpoint_descriptor mtp_superspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(1024),
};

static struct usb_ss_ep_comp_descriptor mtp_superspeed_out_comp_desc = {
	.bLength =		sizeof mtp_superspeed_out_comp_desc,
	.bDescriptorType =	USB_DT_SS_ENDPOINT_COMP,

	/* the following 2 values can be tweaked if necessary */
	 .bMaxBurst =		2,
	/* .bmAttributes =	0, */
};

static struct usb_endpoint_descriptor mtp_highspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor mtp_highspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor mtp_fullspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor mtp_fullspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor mtp_intr_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize         = __constant_cpu_to_le16(INTR_BUFFER_SIZE),
	.bInterval              = 6,
};

static struct usb_ss_ep_comp_descriptor mtp_superspeed_intr_comp_desc = {
	.bLength =		sizeof mtp_superspeed_intr_comp_desc,
	.bDescriptorType =	USB_DT_SS_ENDPOINT_COMP,

	/* the following 3 values can be tweaked if necessary */
	/* .bMaxBurst =		0, */
	/* .bmAttributes =	0, */
	.wBytesPerInterval =	cpu_to_le16(INTR_BUFFER_SIZE),
};

static struct usb_descriptor_header *fs_mtp_descs[] = {
	(struct usb_descriptor_header *) &mtp_interface_desc,
	(struct usb_descriptor_header *) &mtp_fullspeed_in_desc,
	(struct usb_descriptor_header *) &mtp_fullspeed_out_desc,
	(struct usb_descriptor_header *) &mtp_intr_desc,
	NULL,
};

static struct usb_descriptor_header *hs_mtp_descs[] = {
	(struct usb_descriptor_header *) &mtp_interface_desc,
	(struct usb_descriptor_header *) &mtp_highspeed_in_desc,
	(struct usb_descriptor_header *) &mtp_highspeed_out_desc,
	(struct usb_descriptor_header *) &mtp_intr_desc,
	NULL,
};

static struct usb_descriptor_header *ss_mtp_descs[] = {
	(struct usb_descriptor_header *) &mtp_interface_desc,
	(struct usb_descriptor_header *) &mtp_superspeed_in_desc,
	(struct usb_descriptor_header *) &mtp_superspeed_in_comp_desc,
	(struct usb_descriptor_header *) &mtp_superspeed_out_desc,
	(struct usb_descriptor_header *) &mtp_superspeed_out_comp_desc,
	(struct usb_descriptor_header *) &mtp_intr_desc,
	(struct usb_descriptor_header *) &mtp_superspeed_intr_comp_desc,
	NULL,
};

static struct usb_descriptor_header *fs_ptp_descs[] = {
	(struct usb_descriptor_header *) &ptp_interface_desc,
	(struct usb_descriptor_header *) &mtp_fullspeed_in_desc,
	(struct usb_descriptor_header *) &mtp_fullspeed_out_desc,
	(struct usb_descriptor_header *) &mtp_intr_desc,
	NULL,
};

static struct usb_descriptor_header *hs_ptp_descs[] = {
	(struct usb_descriptor_header *) &ptp_interface_desc,
	(struct usb_descriptor_header *) &mtp_highspeed_in_desc,
	(struct usb_descriptor_header *) &mtp_highspeed_out_desc,
	(struct usb_descriptor_header *) &mtp_intr_desc,
	NULL,
};

static struct usb_descriptor_header *ss_ptp_descs[] = {
	(struct usb_descriptor_header *) &ptp_interface_desc,
	(struct usb_descriptor_header *) &mtp_superspeed_in_desc,
	(struct usb_descriptor_header *) &mtp_superspeed_in_comp_desc,
	(struct usb_descriptor_header *) &mtp_superspeed_out_desc,
	(struct usb_descriptor_header *) &mtp_superspeed_out_comp_desc,
	(struct usb_descriptor_header *) &mtp_intr_desc,
	(struct usb_descriptor_header *) &mtp_superspeed_intr_comp_desc,
	NULL,
};

#ifdef CONFIG_USB_G_LGE_MULTIPLE_CONFIGURATION
static struct usb_endpoint_descriptor mtp_superspeed_in_desc2 = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(1024),
};

static struct usb_ss_ep_comp_descriptor mtp_superspeed_in_comp_desc2 = {
	.bLength =		sizeof mtp_superspeed_in_comp_desc2,
	.bDescriptorType =	USB_DT_SS_ENDPOINT_COMP,

	/* the following 2 values can be tweaked if necessary */
	.bMaxBurst =		2,
	/* .bmAttributes =	0, */
};

static struct usb_endpoint_descriptor mtp_superspeed_out_desc2 = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(1024),
};

static struct usb_ss_ep_comp_descriptor mtp_superspeed_out_comp_desc2 = {
	.bLength =		sizeof mtp_superspeed_out_comp_desc2,
	.bDescriptorType =	USB_DT_SS_ENDPOINT_COMP,

	/* the following 2 values can be tweaked if necessary */
	 .bMaxBurst =		2,
	/* .bmAttributes =	0, */
};

static struct usb_endpoint_descriptor mtp_highspeed_in_desc2 = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor mtp_highspeed_out_desc2 = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor mtp_fullspeed_in_desc2 = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor mtp_fullspeed_out_desc2 = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor mtp_intr_desc2 = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize         = __constant_cpu_to_le16(INTR_BUFFER_SIZE),
	.bInterval              = 6,
};

static struct usb_ss_ep_comp_descriptor mtp_superspeed_intr_comp_desc2 = {
	.bLength =		sizeof mtp_superspeed_intr_comp_desc2,
	.bDescriptorType =	USB_DT_SS_ENDPOINT_COMP,

	/* the following 3 values can be tweaked if necessary */
	/* .bMaxBurst =		0, */
	/* .bmAttributes =	0, */
	.wBytesPerInterval =	cpu_to_le16(INTR_BUFFER_SIZE),
};

static struct usb_descriptor_header *fs_mtp_descs2[] = {
	(struct usb_descriptor_header *) &mtp_interface_desc2,
	(struct usb_descriptor_header *) &mtp_fullspeed_in_desc2,
	(struct usb_descriptor_header *) &mtp_fullspeed_out_desc2,
	(struct usb_descriptor_header *) &mtp_intr_desc2,
	NULL,
};

static struct usb_descriptor_header *hs_mtp_descs2[] = {
	(struct usb_descriptor_header *) &mtp_interface_desc2,
	(struct usb_descriptor_header *) &mtp_highspeed_in_desc2,
	(struct usb_descriptor_header *) &mtp_highspeed_out_desc2,
	(struct usb_descriptor_header *) &mtp_intr_desc2,
	NULL,
};

static struct usb_descriptor_header *ss_mtp_descs2[] = {
	(struct usb_descriptor_header *) &mtp_interface_desc2,
	(struct usb_descriptor_header *) &mtp_superspeed_in_desc2,
	(struct usb_descriptor_header *) &mtp_superspeed_in_comp_desc2,
	(struct usb_descriptor_header *) &mtp_superspeed_out_desc2,
	(struct usb_descriptor_header *) &mtp_superspeed_out_comp_desc2,
	(struct usb_descriptor_header *) &mtp_intr_desc2,
	(struct usb_descriptor_header *) &mtp_superspeed_intr_comp_desc2,
	NULL,
};

static struct usb_descriptor_header *fs_ptp_descs2[] = {
	(struct usb_descriptor_header *) &ptp_interface_desc2,
	(struct usb_descriptor_header *) &mtp_fullspeed_in_desc2,
	(struct usb_descriptor_header *) &mtp_fullspeed_out_desc2,
	(struct usb_descriptor_header *) &mtp_intr_desc2,
	NULL,
};

static struct usb_descriptor_header *hs_ptp_descs2[] = {
	(struct usb_descriptor_header *) &ptp_interface_desc2,
	(struct usb_descriptor_header *) &mtp_highspeed_in_desc2,
	(struct usb_descriptor_header *) &mtp_highspeed_out_desc2,
	(struct usb_descriptor_header *) &mtp_intr_desc2,
	NULL,
};

static struct usb_descriptor_header *ss_ptp_descs2[] = {
	(struct usb_descriptor_header *) &ptp_interface_desc2,
	(struct usb_descriptor_header *) &mtp_superspeed_in_desc2,
	(struct usb_descriptor_header *) &mtp_superspeed_in_comp_desc2,
	(struct usb_descriptor_header *) &mtp_superspeed_out_desc2,
	(struct usb_descriptor_header *) &mtp_superspeed_out_comp_desc2,
	(struct usb_descriptor_header *) &mtp_intr_desc2,
	(struct usb_descriptor_header *) &mtp_superspeed_intr_comp_desc2,
	NULL,
};
#endif

static struct usb_string mtp_string_defs[] = {
	/* Naming interface "MTP" so libmtp will recognize us */
	[INTERFACE_STRING_INDEX].s	= "MTP",
	{  },	/* end of list */
};

static struct usb_gadget_strings mtp_string_table = {
	.language		= 0x0409,	/* en-US */
	.strings		= mtp_string_defs,
};

static struct usb_gadget_strings *mtp_strings[] = {
	&mtp_string_table,
	NULL,
};

#ifdef CONFIG_USB_G_LGE_MULTIPLE_CONFIGURATION
static struct usb_string mtp_string_defs2[] = {
	/* Naming interface "MTP" so libmtp will recognize us */
	[INTERFACE_STRING_INDEX].s	= "MTP",
	{  },	/* end of list */
};

static struct usb_gadget_strings mtp_string_table2 = {
	.language		= 0x0409,	/* en-US */
	.strings		= mtp_string_defs2,
};

static struct usb_gadget_strings *mtp_strings2[] = {
	&mtp_string_table2,
	NULL,
};
#endif

/* Microsoft MTP OS String */
static u8 mtp_os_string[] = {
	18, /* sizeof(mtp_os_string) */
	USB_DT_STRING,
	/* Signature field: "MSFT100" */
	'M', 0, 'S', 0, 'F', 0, 'T', 0, '1', 0, '0', 0, '0', 0,
	/* vendor code */
	1,
	/* padding */
	0
};

/* Microsoft Extended Configuration Descriptor Header Section */
struct mtp_ext_config_desc_header {
	__le32	dwLength;
	__u16	bcdVersion;
	__le16	wIndex;
	__u8	bCount;
	__u8	reserved[7];
};

/* Microsoft Extended Configuration Descriptor Function Section */
struct mtp_ext_config_desc_function {
	__u8	bFirstInterfaceNumber;
	__u8	bInterfaceCount;
	__u8	compatibleID[8];
	__u8	subCompatibleID[8];
	__u8	reserved[6];
};

#ifndef CONFIG_USB_G_LGE_MULTIPLE_CONFIGURATION
#ifdef NOT_CONFIG_USB_G_LGE_ANDROID
/* LGE_CHANGE
 * MS Ext Desciptor for MTP and adb (to use in testing driver).
 * NOTE: this remains for reference code about MTP setting with ADB enabled.
 * Therefore we do not use this officially(so NOT_ prefix is used).
 * 2011-02-09, hyunhui.park@lge.com
 */

/* MTP Extended Configuration Descriptor */
struct {
	struct mtp_ext_config_desc_header	header;
	struct mtp_ext_config_desc_function    function;
	struct mtp_ext_config_desc_function    adb_function;
} mtp_ext_config_desc = {
	.header = {
		.dwLength = __constant_cpu_to_le32(sizeof(mtp_ext_config_desc)),
		.bcdVersion = __constant_cpu_to_le16(0x0100),
		.wIndex = __constant_cpu_to_le16(4),
		/* It has two functions (mtp, adb) */
		.bCount = __constant_cpu_to_le16(2),
	},
	.function = {
		.bFirstInterfaceNumber = 0,
		.bInterfaceCount = 1,
		.compatibleID = { 'M', 'T', 'P' },
	},
	/* adb */
	.adb_function = {
		.bFirstInterfaceNumber = 1,
		.bInterfaceCount = 1,
	},
};

#else /* This is Google Original */
/* MTP Extended Configuration Descriptor */
struct {
	struct mtp_ext_config_desc_header	header;
	struct mtp_ext_config_desc_function    function;
} mtp_ext_config_desc = {
	.header = {
		.dwLength = __constant_cpu_to_le32(sizeof(mtp_ext_config_desc)),
		.bcdVersion = __constant_cpu_to_le16(0x0100),
		.wIndex = __constant_cpu_to_le16(4),
		.bCount = __constant_cpu_to_le16(1),
	},
	.function = {
		.bFirstInterfaceNumber = 0,
		.bInterfaceCount = 1,
		.compatibleID = { 'M', 'T', 'P' },
	},
};
#endif
#else
struct ext_config_desc {
	struct mtp_ext_config_desc_header	header;
	struct mtp_ext_config_desc_function    function;
};

struct ext_config_desc mtp_ext_config_desc = {
	.header = {
		.dwLength = __constant_cpu_to_le32(sizeof(mtp_ext_config_desc)),
		.bcdVersion = __constant_cpu_to_le16(0x0100),
		.wIndex = __constant_cpu_to_le16(4),
		.bCount = __constant_cpu_to_le16(1),
	},
	.function = {
		.bFirstInterfaceNumber = 0,
		.bInterfaceCount = 1,
		.compatibleID = { 'M', 'T', 'P' },
	},
};
#endif

struct mtp_device_status {
	__le16	wLength;
	__le16	wCode;
};

/* temporary variable used between mtp_open() and mtp_gadget_bind() */
static struct mtp_dev *_mtp_dev;

#ifdef CONFIG_USB_G_LGE_MULTIPLE_CONFIGURATION
static struct mtp_dev *func_to_mtp(struct usb_function *f)
{
	struct mtp_dev *dev;
	dev = container_of(f, struct mtp_dev, function);
	if ((dev->function.name) && !strcmp(dev->function.name, "mtp")) {
		return dev;
	} else {
		return container_of(f, struct mtp_dev, function2);
	}
}
#else
static inline struct mtp_dev *func_to_mtp(struct usb_function *f)
{
	return container_of(f, struct mtp_dev, function);
}
#endif

static struct usb_request *mtp_request_new(struct usb_ep *ep, int buffer_size)
{
	struct usb_request *req = usb_ep_alloc_request(ep, GFP_KERNEL);
	if (!req)
		return NULL;

	/* now allocate buffers for the requests */
	req->buf = kmalloc(buffer_size, GFP_KERNEL);
	if (!req->buf) {
		usb_ep_free_request(ep, req);
		return NULL;
	}

	return req;
}

static void mtp_request_free(struct usb_request *req, struct usb_ep *ep)
{
	if (req) {
#ifdef CONFIG_USB_G_LGE_ANDROID
		/*
		 * B2-BSP-USB@lge.com 2014-04-17
		 * Just preventive codes.
		 * If function called normally,
		 * panic possibility doesn't exist.
		 */
		if (req->buf)
#endif
		kfree(req->buf);
		usb_ep_free_request(ep, req);
	}
}

static inline int mtp_lock(atomic_t *excl)
{
	if (atomic_inc_return(excl) == 1) {
		return 0;
	} else {
		atomic_dec(excl);
		return -1;
	}
}

static inline void mtp_unlock(atomic_t *excl)
{
	atomic_dec(excl);
}

/* add a request to the tail of a list */
static void mtp_req_put(struct mtp_dev *dev, struct list_head *head,
		struct usb_request *req)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);
	list_add_tail(&req->list, head);
	spin_unlock_irqrestore(&dev->lock, flags);
}

/* remove a request from the head of a list */
static struct usb_request
*mtp_req_get(struct mtp_dev *dev, struct list_head *head)
{
	unsigned long flags;
	struct usb_request *req;

	spin_lock_irqsave(&dev->lock, flags);
	if (list_empty(head)) {
		req = 0;
	} else {
		req = list_first_entry(head, struct usb_request, list);
		list_del(&req->list);
	}
	spin_unlock_irqrestore(&dev->lock, flags);
	return req;
}

static void mtp_complete_in(struct usb_ep *ep, struct usb_request *req)
{
	struct mtp_dev *dev = _mtp_dev;

	if (req->status != 0)
		dev->state = STATE_ERROR;

	mtp_req_put(dev, &dev->tx_idle, req);

	wake_up(&dev->write_wq);
}

static void mtp_complete_out(struct usb_ep *ep, struct usb_request *req)
{
	struct mtp_dev *dev = _mtp_dev;

	dev->rx_done = 1;
	if (req->status != 0)
		dev->state = STATE_ERROR;

	wake_up(&dev->read_wq);
}

static void mtp_complete_intr(struct usb_ep *ep, struct usb_request *req)
{
	struct mtp_dev *dev = _mtp_dev;

	if (req->status != 0)
		dev->state = STATE_ERROR;

	mtp_req_put(dev, &dev->intr_idle, req);

	wake_up(&dev->intr_wq);
}

#ifdef CONFIG_USB_G_LGE_MULTIPLE_CONFIGURATION
static void mtp_ep_backup(struct mtp_dev *dev, const char *func_name)
{
	struct usb_composite_dev *cdev = dev->cdev;

	if (!strcmp(func_name, "mtp")) {
		dev->ep_in1 = dev->ep_in;
		dev->ep_out1 = dev->ep_out;
		dev->ep_intr1 = dev->ep_intr;
	} else if (!strcmp(func_name, "mtp2")) {
		dev->ep_in2 = dev->ep_in;
		dev->ep_out2 = dev->ep_out;
		dev->ep_intr2 = dev->ep_intr;
	} else {
		DBG(cdev, "%s : abnormal function name.\n", __func__);
	}
}

static void mtp_ep_swap(struct mtp_dev *dev, u8 config_num)
{
	struct usb_composite_dev *cdev = dev->cdev;

	if (config_num == 1) {
		dev->ep_in = dev->ep_in1;
		dev->ep_out = dev->ep_out1;
		dev->ep_intr = dev->ep_intr1;
	} else if (config_num == 2) {
		dev->ep_in = dev->ep_in2;
		dev->ep_out = dev->ep_out2;
		dev->ep_intr = dev->ep_intr2;
	} else {
		DBG(cdev, "%s : func has abnormal config.\n", __func__);
	}
}

static void mtp_ep_yield_req(struct mtp_dev	*dev)
{
	struct usb_request *req;
	int i;

	list_for_each_entry(req, &dev->tx_idle, list) {
		lge_usb_ep_yield_request(dev->ep_in, req);
	}

	for (i = 0; i < RX_REQ_MAX; i++)
		lge_usb_ep_yield_request(dev->ep_out, dev->rx_req[i]);

	list_for_each_entry(req, &dev->intr_idle, list) {
		lge_usb_ep_yield_request(dev->ep_intr, req);
	}
}

static int mtp_create_endpoints_only(struct mtp_dev *dev,
				struct usb_endpoint_descriptor *in_desc,
				struct usb_endpoint_descriptor *out_desc,
				struct usb_endpoint_descriptor *intr_desc)
{
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_ep *ep;

	DBG(cdev, "create_bulk_endpoints dev: %p\n", dev);

	ep = usb_ep_autoconfig(cdev->gadget, in_desc);
	if (!ep) {
		DBG(cdev, "usb_ep_autoconfig for ep_in failed\n");
		return -ENODEV;
	}
	DBG(cdev, "usb_ep_autoconfig for ep_in got %s\n", ep->name);
	ep->driver_data = dev;		/* claim the endpoint */
	dev->ep_in = ep;

	ep = usb_ep_autoconfig(cdev->gadget, out_desc);
	if (!ep) {
		DBG(cdev, "usb_ep_autoconfig for ep_out failed\n");
		return -ENODEV;
	}
	DBG(cdev, "usb_ep_autoconfig for mtp ep_out got %s\n", ep->name);
	ep->driver_data = dev;		/* claim the endpoint */
	dev->ep_out = ep;

	ep = usb_ep_autoconfig(cdev->gadget, intr_desc);
	if (!ep) {
		DBG(cdev, "usb_ep_autoconfig for ep_intr failed\n");
		return -ENODEV;
	}
	DBG(cdev, "usb_ep_autoconfig for mtp ep_intr got %s\n", ep->name);
	ep->driver_data = dev;		/* claim the endpoint */
	dev->ep_intr = ep;

	return 0;

}

static int mtp_req_alloc(struct mtp_dev *dev)
{
	struct usb_request *req;
	int i;

retry_tx_alloc:
	if (mtp_tx_req_len > MTP_BULK_BUFFER_SIZE)
		mtp_tx_reqs = 4;

	/* now allocate requests for our endpoints */
	for (i = 0; i < mtp_tx_reqs; i++) {
		req = mtp_request_new(dev->ep_in, mtp_tx_req_len);
		if (!req) {
			while ((req = mtp_req_get(dev, &dev->tx_idle)))
				mtp_request_free(req, dev->ep_in);

			if (mtp_tx_req_len <= MTP_BULK_BUFFER_SIZE)
				goto tx_fail;

			mtp_tx_req_len = MTP_BULK_BUFFER_SIZE;
			mtp_tx_reqs = MTP_TX_REQ_MAX;
			goto retry_tx_alloc;
		}
		req->complete = mtp_complete_in;
		mtp_req_put(dev, &dev->tx_idle, req);
	}

	/*
	 * The RX buffer should be aligned to EP max packet for
	 * some controllers.  At bind time, we don't know the
	 * operational speed.  Hence assuming super speed max
	 * packet size.
	 */
	if (mtp_rx_req_len % 1024)
		mtp_rx_req_len = MTP_BULK_BUFFER_SIZE;

retry_rx_alloc:
	for (i = 0; i < RX_REQ_MAX; i++) {
		req = mtp_request_new(dev->ep_out, mtp_rx_req_len);
		if (!req) {
			for (i--; i >= 0; i--)
				mtp_request_free(dev->rx_req[i], dev->ep_out);

			if (mtp_rx_req_len <= MTP_BULK_BUFFER_SIZE)
				goto rx_fail;

			mtp_rx_req_len = MTP_BULK_BUFFER_SIZE;
			goto retry_rx_alloc;
		}
		req->complete = mtp_complete_out;
		dev->rx_req[i] = req;
	}

	for (i = 0; i < INTR_REQ_MAX; i++) {
		req = mtp_request_new(dev->ep_intr, INTR_BUFFER_SIZE);
		if (!req) {
			while ((req = mtp_req_get(dev, &dev->intr_idle)))
				mtp_request_free(req, dev->ep_intr);
			goto intr_fail;
		}
		req->complete = mtp_complete_intr;
		mtp_req_put(dev, &dev->intr_idle, req);
	}

	return 0;

intr_fail:
	for (i = 0; i < RX_REQ_MAX; i++)
		mtp_request_free(dev->rx_req[i], dev->ep_out);
rx_fail:
	while ((req = mtp_req_get(dev, &dev->tx_idle)))
		mtp_request_free(req, dev->ep_in);
tx_fail:
	printk(KERN_ERR "mtp_bind() could not allocate requests\n");
	return -1;
}
#else
static int mtp_create_bulk_endpoints(struct mtp_dev *dev,
				struct usb_endpoint_descriptor *in_desc,
				struct usb_endpoint_descriptor *out_desc,
				struct usb_endpoint_descriptor *intr_desc)
{
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req;
	struct usb_ep *ep;
	int i;

	DBG(cdev, "create_bulk_endpoints dev: %p\n", dev);

	ep = usb_ep_autoconfig(cdev->gadget, in_desc);
	if (!ep) {
		DBG(cdev, "usb_ep_autoconfig for ep_in failed\n");
		return -ENODEV;
	}
	DBG(cdev, "usb_ep_autoconfig for ep_in got %s\n", ep->name);
	ep->driver_data = dev;		/* claim the endpoint */
	dev->ep_in = ep;

	ep = usb_ep_autoconfig(cdev->gadget, out_desc);
	if (!ep) {
		DBG(cdev, "usb_ep_autoconfig for ep_out failed\n");
		return -ENODEV;
	}
	DBG(cdev, "usb_ep_autoconfig for mtp ep_out got %s\n", ep->name);
	ep->driver_data = dev;		/* claim the endpoint */
	dev->ep_out = ep;

	ep = usb_ep_autoconfig(cdev->gadget, intr_desc);
	if (!ep) {
		DBG(cdev, "usb_ep_autoconfig for ep_intr failed\n");
		return -ENODEV;
	}
	DBG(cdev, "usb_ep_autoconfig for mtp ep_intr got %s\n", ep->name);
	ep->driver_data = dev;		/* claim the endpoint */
	dev->ep_intr = ep;

retry_tx_alloc:
	if (mtp_tx_req_len > MTP_BULK_BUFFER_SIZE)
		mtp_tx_reqs = 4;

	/* now allocate requests for our endpoints */
	for (i = 0; i < mtp_tx_reqs; i++) {
		req = mtp_request_new(dev->ep_in, mtp_tx_req_len);
		if (!req) {
#ifndef CONFIG_USB_G_LGE_ANDROID
			if (mtp_tx_req_len <= MTP_BULK_BUFFER_SIZE)
				goto fail;
#endif
			while ((req = mtp_req_get(dev, &dev->tx_idle)))
				mtp_request_free(req, dev->ep_in);
#ifdef CONFIG_USB_G_LGE_ANDROID
			/*
			 * B2-BSP-USB@lge.com 2014-04-17
			 * If mem alloc fail in mtp_request_new function,
			 * before goto fail we need to free request buf that already allocated.
			 * Or if mtp_tx_req_len is larger than MTP_BULK_BUFFER_SIZE,
			 * decrease buffer size to MTP_BULK_BUFFER_SIZE and retry allocation.
			 */
			if (mtp_tx_req_len <= MTP_BULK_BUFFER_SIZE)
				goto tx_fail;
#endif
			mtp_tx_req_len = MTP_BULK_BUFFER_SIZE;
			mtp_tx_reqs = MTP_TX_REQ_MAX;
			goto retry_tx_alloc;
		}
		req->complete = mtp_complete_in;
		mtp_req_put(dev, &dev->tx_idle, req);
	}

	/*
	 * The RX buffer should be aligned to EP max packet for
	 * some controllers.  At bind time, we don't know the
	 * operational speed.  Hence assuming super speed max
	 * packet size.
	 */
	if (mtp_rx_req_len % 1024)
		mtp_rx_req_len = MTP_BULK_BUFFER_SIZE;

retry_rx_alloc:
	for (i = 0; i < RX_REQ_MAX; i++) {
		req = mtp_request_new(dev->ep_out, mtp_rx_req_len);
		if (!req) {
#ifndef CONFIG_USB_G_LGE_ANDROID
			if (mtp_rx_req_len <= MTP_BULK_BUFFER_SIZE)
				goto fail;
			for (; i > 0; i--)
				mtp_request_free(dev->rx_req[i], dev->ep_out);
#else
			/*
			 * B2-BSP-USB@lge.com 2014-04-17
			 * If mem alloc fail in mtp_request_new function,
			 * before goto fail we need to free request buf that already allocated.
			 * Or if mtp_tx_req_len is larger than MTP_BULK_BUFFER_SIZE,
			 * decrease buffer size to MTP_BULK_BUFFER_SIZE and retry allocation.
			 * And fixed pagecompound bug_on during abnormal kfree.
			 */
			for (i--; i >= 0; i--)
				mtp_request_free(dev->rx_req[i], dev->ep_out);

			if (mtp_rx_req_len <= MTP_BULK_BUFFER_SIZE)
				goto rx_fail;
#endif
			mtp_rx_req_len = MTP_BULK_BUFFER_SIZE;
			goto retry_rx_alloc;
		}
		req->complete = mtp_complete_out;
		dev->rx_req[i] = req;
	}

	for (i = 0; i < INTR_REQ_MAX; i++) {
		req = mtp_request_new(dev->ep_intr, INTR_BUFFER_SIZE);
#ifdef CONFIG_USB_G_LGE_ANDROID
		if (!req) {
			while ((req = mtp_req_get(dev, &dev->intr_idle)))
				mtp_request_free(req, dev->ep_intr);
			goto intr_fail;
		}
#else
		if (!req)
			goto fail;
#endif
		req->complete = mtp_complete_intr;
		mtp_req_put(dev, &dev->intr_idle, req);
	}

	return 0;

#ifdef CONFIG_USB_G_LGE_ANDROID
intr_fail:
	for (i = 0; i < RX_REQ_MAX; i++)
		mtp_request_free(dev->rx_req[i], dev->ep_out);
rx_fail:
	while ((req = mtp_req_get(dev, &dev->tx_idle)))
		mtp_request_free(req, dev->ep_in);
tx_fail:
#else
fail:
#endif
	printk(KERN_ERR "mtp_bind() could not allocate requests\n");
	return -1;
}
#endif

static ssize_t mtp_read(struct file *fp, char __user *buf,
	size_t count, loff_t *pos)
{
	struct mtp_dev *dev = fp->private_data;
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req;
	int r = count, xfer, len;
	int ret = 0;

	DBG(cdev, "mtp_read(%d)\n", count);

#ifdef CONFIG_USB_G_LGE_ANDROID
	if (!dev->ep_out)
		return -EINVAL;
#endif
	len = ALIGN(count, dev->ep_out->maxpacket);

	if (len > mtp_rx_req_len)
		return -EINVAL;

	/* we will block until we're online */
	DBG(cdev, "mtp_read: waiting for online state\n");
	ret = wait_event_interruptible(dev->read_wq,
		dev->state != STATE_OFFLINE);
	if (ret < 0) {
		r = ret;
		goto done;
	}
	spin_lock_irq(&dev->lock);
	if (dev->state == STATE_CANCELED) {
		/* report cancelation to userspace */
		dev->state = STATE_READY;
		spin_unlock_irq(&dev->lock);
		return -ECANCELED;
	}
	dev->state = STATE_BUSY;
	spin_unlock_irq(&dev->lock);

requeue_req:
	/* queue a request */
	req = dev->rx_req[0];
	req->length = len;
	dev->rx_done = 0;
	ret = usb_ep_queue(dev->ep_out, req, GFP_KERNEL);
	if (ret < 0) {
		r = -EIO;
		goto done;
	} else {
		DBG(cdev, "rx %p queue\n", req);
	}

	/* wait for a request to complete */
	ret = wait_event_interruptible(dev->read_wq,
				dev->rx_done || dev->state != STATE_BUSY);
	if (dev->state == STATE_CANCELED) {
		r = -ECANCELED;
		if (!dev->rx_done)
			usb_ep_dequeue(dev->ep_out, req);
		spin_lock_irq(&dev->lock);
		dev->state = STATE_CANCELED;
		spin_unlock_irq(&dev->lock);
		goto done;
	}
	if (ret < 0) {
		r = ret;
		usb_ep_dequeue(dev->ep_out, req);
		goto done;
	}
	if (dev->state == STATE_BUSY) {
		/* If we got a 0-len packet, throw it back and try again. */
		if (req->actual == 0)
			goto requeue_req;

		DBG(cdev, "rx %p %d\n", req, req->actual);
		xfer = (req->actual < count) ? req->actual : count;
		r = xfer;
		if (copy_to_user(buf, req->buf, xfer))
			r = -EFAULT;
	} else
		r = -EIO;

done:
	spin_lock_irq(&dev->lock);
	if (dev->state == STATE_CANCELED)
		r = -ECANCELED;
	else if (dev->state != STATE_OFFLINE)
		dev->state = STATE_READY;
	spin_unlock_irq(&dev->lock);

	DBG(cdev, "mtp_read returning %d\n", r);
	return r;
}

static ssize_t mtp_write(struct file *fp, const char __user *buf,
	size_t count, loff_t *pos)
{
	struct mtp_dev *dev = fp->private_data;
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req = 0;
	int r = count, xfer;
	int sendZLP = 0;
	int ret;

	DBG(cdev, "mtp_write(%d)\n", count);

	spin_lock_irq(&dev->lock);
	if (dev->state == STATE_CANCELED) {
		/* report cancelation to userspace */
		dev->state = STATE_READY;
		spin_unlock_irq(&dev->lock);
		return -ECANCELED;
	}
	if (dev->state == STATE_OFFLINE) {
		spin_unlock_irq(&dev->lock);
		return -ENODEV;
	}
	dev->state = STATE_BUSY;
	spin_unlock_irq(&dev->lock);

	/* we need to send a zero length packet to signal the end of transfer
	 * if the transfer size is aligned to a packet boundary.
	 */
	if ((count & (dev->ep_in->maxpacket - 1)) == 0)
		sendZLP = 1;

	while (count > 0 || sendZLP) {
		/* so we exit after sending ZLP */
		if (count == 0)
			sendZLP = 0;

		if (dev->state != STATE_BUSY) {
			DBG(cdev, "mtp_write dev->error\n");
			r = -EIO;
			break;
		}

		/* get an idle tx request to use */
		req = 0;
		ret = wait_event_interruptible(dev->write_wq,
			((req = mtp_req_get(dev, &dev->tx_idle))
				|| dev->state != STATE_BUSY));
		if (!req) {
			r = ret;
			break;
		}

		if (count > mtp_tx_req_len)
			xfer = mtp_tx_req_len;
		else
			xfer = count;
		if (xfer && copy_from_user(req->buf, buf, xfer)) {
			r = -EFAULT;
			break;
		}

		req->length = xfer;
		ret = usb_ep_queue(dev->ep_in, req, GFP_KERNEL);
		if (ret < 0) {
			DBG(cdev, "mtp_write: xfer error %d\n", ret);
			r = -EIO;
			break;
		}

		buf += xfer;
		count -= xfer;

		/* zero this so we don't try to free it on error exit */
		req = 0;
	}

	if (req)
		mtp_req_put(dev, &dev->tx_idle, req);

	spin_lock_irq(&dev->lock);
	if (dev->state == STATE_CANCELED)
		r = -ECANCELED;
	else if (dev->state != STATE_OFFLINE)
		dev->state = STATE_READY;
	spin_unlock_irq(&dev->lock);

	DBG(cdev, "mtp_write returning %d\n", r);
	return r;
}

/* read from a local file and write to USB */
static void send_file_work(struct work_struct *data)
{
	struct mtp_dev *dev = container_of(data, struct mtp_dev,
						send_file_work);
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req = 0;
	struct mtp_data_header *header;
	struct file *filp;
	loff_t offset;
	int64_t count;
	int xfer, ret, hdr_size;
	int r = 0;
	int sendZLP = 0;
#ifdef CONFIG_USB_G_LGE_MTP_PROFILING
	ktime_t	send_start, start, diff;
#endif

	/* read our parameters */
	smp_rmb();
	filp = dev->xfer_file;
	offset = dev->xfer_file_offset;
	count = dev->xfer_file_length;

	DBG(cdev, "send_file_work(%lld %lld)\n", offset, count);
#ifdef CONFIG_USB_G_LGE_MTP_PROFILING
	if (!ktime_to_ms(dev->perf.first_start_rtime)) {
		dev->perf.first_start_rtime = ktime_get();
		dev->perf.r_count = 0;
	}
	dev->perf.rbytes = (uint64_t)0;
	memset(&dev->perf.rtime, 0, sizeof(dev->perf.rtime));
	memset(&dev->perf.send_time, 0, sizeof(dev->perf.send_time));
#endif

	if (dev->xfer_send_header) {
		hdr_size = sizeof(struct mtp_data_header);
		count += hdr_size;
	} else {
		hdr_size = 0;
	}

	/* we need to send a zero length packet to signal the end of transfer
	 * if the transfer size is aligned to a packet boundary.
	 */
	if ((count & (dev->ep_in->maxpacket - 1)) == 0)
		sendZLP = 1;

	while (count > 0 || sendZLP) {
#ifdef CONFIG_USB_G_LGE_MTP_PROFILING
		send_start = ktime_get();
#endif
		/* so we exit after sending ZLP */
		if (count == 0)
			sendZLP = 0;

		/* get an idle tx request to use */
		req = 0;
		ret = wait_event_interruptible(dev->write_wq,
			(req = mtp_req_get(dev, &dev->tx_idle))
			|| dev->state != STATE_BUSY);
		if (dev->state == STATE_CANCELED) {
			r = -ECANCELED;
			break;
		}
		if (!req) {
			r = ret;
			break;
		}

		if (count > mtp_tx_req_len)
			xfer = mtp_tx_req_len;
		else
			xfer = count;

		if (hdr_size) {
			/* prepend MTP data header */
			header = (struct mtp_data_header *)req->buf;
			header->length = __cpu_to_le32(count);
			header->type = __cpu_to_le16(2); /* data packet */
			header->command = __cpu_to_le16(dev->xfer_command);
			header->transaction_id =
					__cpu_to_le32(dev->xfer_transaction_id);
		}

#ifdef CONFIG_USB_G_LGE_MTP_PROFILING
		start = ktime_get();
#endif
		ret = vfs_read(filp, req->buf + hdr_size, xfer - hdr_size,
								&offset);
#ifdef CONFIG_USB_G_LGE_MTP_PROFILING
		diff = ktime_sub(ktime_get(), start);
		dev->perf.rbytes += ret;
		dev->perf.t_rbytes += ret;
		dev->perf.rtime = ktime_add(dev->perf.rtime, diff);
		dev->perf.t_rtime = ktime_add(dev->perf.t_rtime, diff);
#endif
		if (ret < 0) {
			r = ret;
			break;
		}
		xfer = ret + hdr_size;
		hdr_size = 0;

		req->length = xfer;
		ret = usb_ep_queue(dev->ep_in, req, GFP_KERNEL);
		if (ret < 0) {
			DBG(cdev, "send_file_work: xfer error %d\n", ret);
			if (dev->state != STATE_OFFLINE)
				dev->state = STATE_ERROR;
			r = -EIO;
			break;
		}

		count -= xfer;

		/* zero this so we don't try to free it on error exit */
		req = 0;
#ifdef CONFIG_USB_G_LGE_MTP_PROFILING
		diff = ktime_sub(ktime_get(), send_start);
		dev->perf.send_time = ktime_add(dev->perf.send_time, diff);
		dev->perf.t_send_time = ktime_add(dev->perf.t_send_time, diff);
#endif
	}

	if (req)
		mtp_req_put(dev, &dev->tx_idle, req);

#ifdef CONFIG_USB_G_LGE_MTP_PROFILING
	dev->perf.r_count++;
	dev->perf.last_end_rtime = ktime_get();
#endif
	DBG(cdev, "send_file_work returning %d\n", r);
	/* write the result */
	dev->xfer_result = r;
	smp_wmb();
}

/* read from USB and write to a local file */
static void receive_file_work(struct work_struct *data)
{
	struct mtp_dev *dev = container_of(data, struct mtp_dev,
						receive_file_work);
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *read_req = NULL, *write_req = NULL;
	struct file *filp;
	loff_t offset;
	int64_t count;
	int ret, cur_buf = 0;
	int r = 0;
#ifdef CONFIG_USB_G_LGE_MTP_PROFILING
	ktime_t	receive_start, start, diff;
#endif

	/* read our parameters */
	smp_rmb();
	filp = dev->xfer_file;
	offset = dev->xfer_file_offset;
	count = dev->xfer_file_length;

	DBG(cdev, "receive_file_work(%lld)\n", count);
#ifdef CONFIG_USB_G_LGE_MTP_PROFILING
	if (!ktime_to_ms(dev->perf.first_start_wtime)) {
		dev->perf.first_start_wtime = ktime_get();
		dev->perf.w_count = 0;
	}
	dev->perf.wbytes = (uint64_t)0;
	memset(&dev->perf.wtime, 0, sizeof(dev->perf.wtime));
	memset(&dev->perf.receive_time, 0, sizeof(dev->perf.receive_time));
#endif
#ifdef CONFIG_USB_G_LGE_ANDROID
	if (dev->ep_out && !IS_ALIGNED(count, dev->ep_out->maxpacket))
#else
	if (!IS_ALIGNED(count, dev->ep_out->maxpacket))
#endif
		DBG(cdev, "%s- count(%lld) not multiple of mtu(%d)\n", __func__,
						count, dev->ep_out->maxpacket);

	while (count > 0 || write_req) {
#ifdef CONFIG_USB_G_LGE_MTP_PROFILING
		receive_start = ktime_get();
#endif
		if (count > 0) {
			/* queue a request */
			read_req = dev->rx_req[cur_buf];
			cur_buf = (cur_buf + 1) % RX_REQ_MAX;

			/* some h/w expects size to be aligned to ep's MTU */
			read_req->length = mtp_rx_req_len;

			dev->rx_done = 0;
			ret = usb_ep_queue(dev->ep_out, read_req, GFP_KERNEL);
			if (ret < 0) {
				r = -EIO;
				if (dev->state != STATE_OFFLINE)
					dev->state = STATE_ERROR;
				break;
			}
		}

		if (write_req) {
			DBG(cdev, "rx %p %d\n", write_req, write_req->actual);
#ifdef CONFIG_USB_G_LGE_MTP_PROFILING
			start = ktime_get();
#endif
			ret = vfs_write(filp, write_req->buf, write_req->actual,
				&offset);
#ifdef CONFIG_USB_G_LGE_MTP_PROFILING
			diff = ktime_sub(ktime_get(), start);
			dev->perf.wbytes += ret;
			dev->perf.t_wbytes += ret;
			dev->perf.wtime = ktime_add(dev->perf.wtime, diff);
			dev->perf.t_wtime = ktime_add(dev->perf.t_wtime, diff);
#endif
			DBG(cdev, "vfs_write %d\n", ret);
			if (ret != write_req->actual) {
				r = -EIO;
				if (dev->state != STATE_OFFLINE)
					dev->state = STATE_ERROR;
				break;
			}
			write_req = NULL;
		}

		if (read_req) {
			/* wait for our last read to complete */
			ret = wait_event_interruptible(dev->read_wq,
				dev->rx_done || dev->state != STATE_BUSY);
			if (dev->state == STATE_CANCELED
					|| dev->state == STATE_OFFLINE) {
				if (dev->state == STATE_OFFLINE)
					r = -EIO;
				else
					r = -ECANCELED;
				if (!dev->rx_done)
					usb_ep_dequeue(dev->ep_out, read_req);
				break;
			}
			/* Check if we aligned the size due to MTU constraint */
			if (count < read_req->length)
				read_req->actual = (read_req->actual > count ?
						count : read_req->actual);
			/* if xfer_file_length is 0xFFFFFFFF, then we read until
			 * we get a zero length packet
			 */
			if (count != 0xFFFFFFFF)
				count -= read_req->actual;
			if (read_req->actual < read_req->length) {
				/*
				 * short packet is used to signal EOF for
				 * sizes > 4 gig
				 */
				DBG(cdev, "got short packet\n");
				count = 0;
			}

			write_req = read_req;
			read_req = NULL;
		}
#ifdef CONFIG_USB_G_LGE_MTP_PROFILING
		diff = ktime_sub(ktime_get(), receive_start);
		dev->perf.receive_time = ktime_add(dev->perf.receive_time, diff);
		dev->perf.t_receive_time = ktime_add(dev->perf.t_receive_time, diff);
#endif
	}

#ifdef CONFIG_USB_G_LGE_MTP_PROFILING
	dev->perf.w_count++;
	dev->perf.last_end_wtime = ktime_get();
#endif
	DBG(cdev, "receive_file_work returning %d\n", r);
	/* write the result */
	dev->xfer_result = r;
	smp_wmb();
}

static int mtp_send_event(struct mtp_dev *dev, struct mtp_event *event)
{
	struct usb_request *req = NULL;
	int ret;
	int length = event->length;

	DBG(dev->cdev, "mtp_send_event(%d)\n", event->length);

	if (length < 0 || length > INTR_BUFFER_SIZE)
		return -EINVAL;
	if (dev->state == STATE_OFFLINE)
		return -ENODEV;

	ret = wait_event_interruptible_timeout(dev->intr_wq,
			(req = mtp_req_get(dev, &dev->intr_idle)),
			msecs_to_jiffies(1000));
	if (!req)
		return -ETIME;

	if (copy_from_user(req->buf, (void __user *)event->data, length)) {
		mtp_req_put(dev, &dev->intr_idle, req);
		return -EFAULT;
	}
	req->length = length;
	ret = usb_ep_queue(dev->ep_intr, req, GFP_KERNEL);
	if (ret)
		mtp_req_put(dev, &dev->intr_idle, req);

	return ret;
}

static long mtp_ioctl(struct file *fp, unsigned code, unsigned long value)
{
	struct mtp_dev *dev = fp->private_data;
	struct file *filp = NULL;
	int ret = -EINVAL;

	if (mtp_lock(&dev->ioctl_excl))
		return -EBUSY;

	switch (code) {
	case MTP_SEND_FILE:
	case MTP_RECEIVE_FILE:
	case MTP_SEND_FILE_WITH_HEADER:
	{
		struct mtp_file_range	mfr;
		struct work_struct *work;

		spin_lock_irq(&dev->lock);
		if (dev->state == STATE_CANCELED) {
			/* report cancelation to userspace */
			dev->state = STATE_READY;
			spin_unlock_irq(&dev->lock);
			ret = -ECANCELED;
			goto out;
		}
		if (dev->state == STATE_OFFLINE) {
			spin_unlock_irq(&dev->lock);
			ret = -ENODEV;
			goto out;
		}
		dev->state = STATE_BUSY;
		spin_unlock_irq(&dev->lock);

		if (copy_from_user(&mfr, (void __user *)value, sizeof(mfr))) {
			ret = -EFAULT;
			goto fail;
		}
		/* hold a reference to the file while we are working with it */
		filp = fget(mfr.fd);
		if (!filp) {
			ret = -EBADF;
			goto fail;
		}

		/* write the parameters */
		dev->xfer_file = filp;
		dev->xfer_file_offset = mfr.offset;
		dev->xfer_file_length = mfr.length;
		smp_wmb();

		if (code == MTP_SEND_FILE_WITH_HEADER) {
			work = &dev->send_file_work;
			dev->xfer_send_header = 1;
			dev->xfer_command = mfr.command;
			dev->xfer_transaction_id = mfr.transaction_id;
		} else if (code == MTP_SEND_FILE) {
			work = &dev->send_file_work;
			dev->xfer_send_header = 0;
		} else {
			work = &dev->receive_file_work;
		}

		/* We do the file transfer on a work queue so it will run
		 * in kernel context, which is necessary for vfs_read and
		 * vfs_write to use our buffers in the kernel address space.
		 */
		queue_work(dev->wq, work);
		/* wait for operation to complete */
		flush_workqueue(dev->wq);
		fput(filp);

		/* read the result */
		smp_rmb();
		ret = dev->xfer_result;
		break;
	}
	case MTP_SEND_EVENT:
	{
		struct mtp_event	event;
		/* return here so we don't change dev->state below,
		 * which would interfere with bulk transfer state.
		 */
		if (copy_from_user(&event, (void __user *)value, sizeof(event)))
			ret = -EFAULT;
		else
			ret = mtp_send_event(dev, &event);
		goto out;
	}
	}

fail:
	spin_lock_irq(&dev->lock);
	if (dev->state == STATE_CANCELED)
		ret = -ECANCELED;
	else if (dev->state != STATE_OFFLINE)
		dev->state = STATE_READY;
	spin_unlock_irq(&dev->lock);
out:
	mtp_unlock(&dev->ioctl_excl);
	DBG(dev->cdev, "ioctl returning %d\n", ret);
	return ret;
}

static int mtp_open(struct inode *ip, struct file *fp)
{
	printk(KERN_INFO "mtp_open\n");
	if (mtp_lock(&_mtp_dev->open_excl))
		return -EBUSY;

	/* clear any error condition */
	if (_mtp_dev->state != STATE_OFFLINE)
		_mtp_dev->state = STATE_READY;

	fp->private_data = _mtp_dev;
	return 0;
}

static int mtp_release(struct inode *ip, struct file *fp)
{
	printk(KERN_INFO "mtp_release\n");

	mtp_unlock(&_mtp_dev->open_excl);
	return 0;
}

/* file operations for /dev/mtp_usb */
static const struct file_operations mtp_fops = {
	.owner = THIS_MODULE,
	.read = mtp_read,
	.write = mtp_write,
	.unlocked_ioctl = mtp_ioctl,
	.open = mtp_open,
	.release = mtp_release,
};

static struct miscdevice mtp_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = mtp_shortname,
	.fops = &mtp_fops,
};

static int mtp_ctrlrequest(struct usb_composite_dev *cdev,
				const struct usb_ctrlrequest *ctrl)
{
	struct mtp_dev *dev = _mtp_dev;
	int	value = -EOPNOTSUPP;
	u16	w_index = le16_to_cpu(ctrl->wIndex);
	u16	w_value = le16_to_cpu(ctrl->wValue);
	u16	w_length = le16_to_cpu(ctrl->wLength);
	unsigned long	flags;

	VDBG(cdev, "mtp_ctrlrequest "
			"%02x.%02x v%04x i%04x l%u\n",
			ctrl->bRequestType, ctrl->bRequest,
			w_value, w_index, w_length);

	/* Handle MTP OS string */
	if (ctrl->bRequestType ==
			(USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_DEVICE)
			&& ctrl->bRequest == USB_REQ_GET_DESCRIPTOR
			&& (w_value >> 8) == USB_DT_STRING
			&& (w_value & 0xFF) == MTP_OS_STRING_ID) {
		value = (w_length < sizeof(mtp_os_string)
				? w_length : sizeof(mtp_os_string));
		memcpy(cdev->req->buf, mtp_os_string, value);
	} else if ((ctrl->bRequestType & USB_TYPE_MASK) == USB_TYPE_VENDOR) {
		/* Handle MTP OS descriptor */
		DBG(cdev, "vendor request: %d index: %d value: %d length: %d\n",
			ctrl->bRequest, w_index, w_value, w_length);

		if (ctrl->bRequest == 1
				&& (ctrl->bRequestType & USB_DIR_IN)
				&& (w_index == 4 || w_index == 5)) {
			value = (w_length < sizeof(mtp_ext_config_desc) ?
					w_length : sizeof(mtp_ext_config_desc));
			memcpy(cdev->req->buf, &mtp_ext_config_desc, value);
		}
	} else if ((ctrl->bRequestType & USB_TYPE_MASK) == USB_TYPE_CLASS) {
		DBG(cdev, "class request: %d index: %d value: %d length: %d\n",
			ctrl->bRequest, w_index, w_value, w_length);

		if (ctrl->bRequest == MTP_REQ_CANCEL && (w_index == 0 || w_index == 3)
				&& w_value == 0) {
			DBG(cdev, "MTP_REQ_CANCEL\n");

			spin_lock_irqsave(&dev->lock, flags);
			if (dev->state == STATE_BUSY) {
				dev->state = STATE_CANCELED;
				wake_up(&dev->read_wq);
				wake_up(&dev->write_wq);
			}
			spin_unlock_irqrestore(&dev->lock, flags);

			/* We need to queue a request to read the remaining
			 *  bytes, but we don't actually need to look at
			 * the contents.
			 */
			value = w_length;
		} else if (ctrl->bRequest == MTP_REQ_GET_DEVICE_STATUS
				&& (w_index == 0 || w_index == 3) && w_value == 0) {
			struct mtp_device_status *status = cdev->req->buf;
			status->wLength =
				__constant_cpu_to_le16(sizeof(*status));

			DBG(cdev, "MTP_REQ_GET_DEVICE_STATUS\n");
			spin_lock_irqsave(&dev->lock, flags);
			/* device status is "busy" until we report
			 * the cancelation to userspace
			 */
			if (dev->state == STATE_CANCELED)
				status->wCode =
					__cpu_to_le16(MTP_RESPONSE_DEVICE_BUSY);
			else
				status->wCode =
					__cpu_to_le16(MTP_RESPONSE_OK);
			spin_unlock_irqrestore(&dev->lock, flags);
			value = sizeof(*status);
		}
	}

	/* respond with data transfer or status phase? */
	if (value >= 0) {
		int rc;
		cdev->req->zero = value < w_length;
		cdev->req->length = value;
		rc = usb_ep_queue(cdev->gadget->ep0, cdev->req, GFP_ATOMIC);
		if (rc < 0)
			ERROR(cdev, "%s: response queue error\n", __func__);
	}
	return value;
}

#ifdef CONFIG_USB_G_LGE_MULTIPLE_CONFIGURATION
static int multi_mtp_bind(struct mtp_dev	*dev, struct usb_function *f, struct usb_configuration *c)
{
	struct usb_composite_dev *cdev = c->cdev;

	int			id;
	int 		ret;

	/* allocate interface ID(s) */
	id = usb_interface_id(f->config, f);
	if (id < 0)
		return id;
	mtp_interface_desc2.bInterfaceNumber = id;
#ifdef CONFIG_USB_G_LGE_ANDROID
	/* for ptp & MS desc */
	ptp_interface_desc2.bInterfaceNumber = id;
	if (dev->first_mtp_binded == false)
		mtp_ext_config_desc.function.bFirstInterfaceNumber = id;
#endif
	/* allocate endpoints */
	ret = mtp_create_endpoints_only(dev, &mtp_fullspeed_in_desc2,
				&mtp_fullspeed_out_desc2, &mtp_intr_desc2);
	if (ret)
		return ret;
	mtp_ep_backup(dev, f->name);
	if (dev->first_mtp_binded == false) {
		ret = mtp_req_alloc(dev);
		if (ret)
			return ret;
	}
	/* support high speed hardware */
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		mtp_highspeed_in_desc2.bEndpointAddress =
			mtp_fullspeed_in_desc2.bEndpointAddress;
		mtp_highspeed_out_desc2.bEndpointAddress =
			mtp_fullspeed_out_desc2.bEndpointAddress;
	}

	/* support super speed hardware */
	if (gadget_is_superspeed(c->cdev->gadget)) {
		mtp_superspeed_in_desc2.bEndpointAddress =
			mtp_fullspeed_in_desc2.bEndpointAddress;
		mtp_superspeed_out_desc2.bEndpointAddress =
			mtp_fullspeed_out_desc2.bEndpointAddress;
	}

	DBG(cdev, "%s speed %s: IN/%s, OUT/%s\n",
			gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
			f->name, dev->ep_in->name, dev->ep_out->name);

	return 0;
}
#endif

static int
mtp_function_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct mtp_dev	*dev = func_to_mtp(f);
	int			id;
	int			ret;

#ifdef CONFIG_USB_G_LGE_MULTIPLE_CONFIGURATION
	if (dev->allocated_func)
		return -1;

	if (c->bConfigurationValue == 2) {
		return multi_mtp_bind(dev, f, c);
	}
#endif

	dev->cdev = cdev;
	DBG(cdev, "mtp_function_bind dev: %p\n", dev);

	/* allocate interface ID(s) */
	id = usb_interface_id(c, f);
	if (id < 0)
		return id;
	mtp_interface_desc.bInterfaceNumber = id;
#ifdef CONFIG_USB_G_LGE_ANDROID
	/* for ptp & MS desc */
	ptp_interface_desc.bInterfaceNumber = id;
	mtp_ext_config_desc.function.bFirstInterfaceNumber = id;
#endif

	/* allocate endpoints */
#ifdef CONFIG_USB_G_LGE_MULTIPLE_CONFIGURATION
	ret = mtp_create_endpoints_only(dev, &mtp_fullspeed_in_desc,
			&mtp_fullspeed_out_desc, &mtp_intr_desc);
	if (ret)
		return ret;

	mtp_ep_backup(dev, f->name);

	ret = mtp_req_alloc(dev);
	if (ret)
		return ret;
	dev->first_mtp_binded = true;
#else
	ret = mtp_create_bulk_endpoints(dev, &mtp_fullspeed_in_desc,
			&mtp_fullspeed_out_desc, &mtp_intr_desc);
	if (ret)
		return ret;
#endif

	/* support high speed hardware */
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		mtp_highspeed_in_desc.bEndpointAddress =
			mtp_fullspeed_in_desc.bEndpointAddress;
		mtp_highspeed_out_desc.bEndpointAddress =
			mtp_fullspeed_out_desc.bEndpointAddress;
	}

	/* support super speed hardware */
	if (gadget_is_superspeed(c->cdev->gadget)) {
		mtp_superspeed_in_desc.bEndpointAddress =
			mtp_fullspeed_in_desc.bEndpointAddress;
		mtp_superspeed_out_desc.bEndpointAddress =
			mtp_fullspeed_out_desc.bEndpointAddress;
	}

	DBG(cdev, "%s speed %s: IN/%s, OUT/%s\n",
			gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
			f->name, dev->ep_in->name, dev->ep_out->name);
	return 0;
}

static void
mtp_function_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct mtp_dev	*dev = func_to_mtp(f);
	struct usb_request *req;
	int i;
#ifdef CONFIG_USB_G_LGE_MULTIPLE_CONFIGURATION
	if (!dev->allocated_func) {
		dev->state = STATE_OFFLINE;
		return;
	}

	if (dev->allocated_func == f->config->bConfigurationValue) {
		mtp_ep_swap(dev, dev->allocated_func);
	} else {
		return;
	}
#endif

	while ((req = mtp_req_get(dev, &dev->tx_idle)))
		mtp_request_free(req, dev->ep_in);
	for (i = 0; i < RX_REQ_MAX; i++)
		mtp_request_free(dev->rx_req[i], dev->ep_out);
	while ((req = mtp_req_get(dev, &dev->intr_idle)))
		mtp_request_free(req, dev->ep_intr);
	dev->state = STATE_OFFLINE;
#ifdef CONFIG_USB_G_LGE_MULTIPLE_CONFIGURATION
	dev->allocated_func = 0;
	dev->first_mtp_binded = false;
#endif
}

static int mtp_function_set_alt(struct usb_function *f,
		unsigned intf, unsigned alt)
{
	struct mtp_dev	*dev = func_to_mtp(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	int ret;

	DBG(cdev, "mtp_function_set_alt intf: %d alt: %d\n", intf, alt);
#ifdef CONFIG_USB_G_LGE_MULTIPLE_CONFIGURATION
	mtp_ep_swap(dev, f->config->bConfigurationValue);
	if ((dev->first_mtp_binded == true) && f->config->bConfigurationValue == 2)
		mtp_ep_yield_req(dev);
#endif

	ret = config_ep_by_speed(cdev->gadget, f, dev->ep_in);
	if (ret) {
		dev->ep_in->desc = NULL;
		ERROR(cdev, "config_ep_by_speed failes for ep %s, result %d\n",
			dev->ep_in->name, ret);
		return ret;
	}
	ret = usb_ep_enable(dev->ep_in);
	if (ret) {
		ERROR(cdev, "failed to enable ep %s, result %d\n",
			dev->ep_in->name, ret);
		return ret;
	}

	ret = config_ep_by_speed(cdev->gadget, f, dev->ep_out);
	if (ret) {
		dev->ep_out->desc = NULL;
		ERROR(cdev, "config_ep_by_speed failes for ep %s, result %d\n",
			dev->ep_out->name, ret);
		usb_ep_disable(dev->ep_in);
		return ret;
	}
#ifdef CONFIG_USB_G_LGE_ANDROID
	if(!lge_get_laf_mode())
#endif
	ret = usb_ep_enable(dev->ep_out);
	if (ret) {
		ERROR(cdev, "failed to enable ep %s, result %d\n",
			dev->ep_out->name, ret);
		usb_ep_disable(dev->ep_in);
		return ret;
	}
	dev->ep_intr->desc = &mtp_intr_desc;
	ret = usb_ep_enable(dev->ep_intr);
	if (ret) {
		usb_ep_disable(dev->ep_out);
		usb_ep_disable(dev->ep_in);
		return ret;
	}
#ifdef CONFIG_USB_G_LGE_MULTIPLE_CONFIGURATION
	dev->allocated_func = f->config->bConfigurationValue;
#endif
	dev->state = STATE_READY;

	/* readers may be blocked waiting for us to go online */
	wake_up(&dev->read_wq);
	return 0;
}

static void mtp_function_disable(struct usb_function *f)
{
	struct mtp_dev	*dev = func_to_mtp(f);
	struct usb_composite_dev	*cdev = dev->cdev;

#ifdef CONFIG_USB_G_LGE_MULTIPLE_CONFIGURATION
	if (dev->allocated_func == f->config->bConfigurationValue) {
		mtp_ep_swap(dev, dev->allocated_func);
	} else {
		return;
	}
#endif

	DBG(cdev, "mtp_function_disable\n");
	dev->state = STATE_OFFLINE;
	usb_ep_disable(dev->ep_in);
	usb_ep_disable(dev->ep_out);
	usb_ep_disable(dev->ep_intr);

	/* readers may be blocked waiting for us to go online */
	wake_up(&dev->read_wq);

	VDBG(cdev, "%s disabled\n", dev->function.name);
}

#ifdef CONFIG_USB_G_LGE_MULTIPLE_CONFIGURATION
static int lge_mtp_desc_change(struct usb_function *f, bool is_mac)
{
	struct usb_interface_descriptor *mtp_desc;

	if (!strcmp(f->name, "mtp"))
		mtp_desc = &mtp_interface_desc;
	else
		mtp_desc = &mtp_interface_desc2;

	if (is_mac == true) {
		mtp_desc->bInterfaceClass = USB_CLASS_VENDOR_SPEC;
		mtp_desc->bInterfaceSubClass = USB_SUBCLASS_VENDOR_SPEC;
		mtp_desc->bInterfaceProtocol = 0;
	} else {
		mtp_desc->bInterfaceClass = USB_CLASS_STILL_IMAGE;
		mtp_desc->bInterfaceSubClass = 1;
		mtp_desc->bInterfaceProtocol = 1;
	}
	return 0;
}
#endif

static int mtp_bind_config(struct usb_configuration *c, bool ptp_config)
{
	struct mtp_dev *dev = _mtp_dev;
	int ret = 0;

	printk(KERN_INFO "mtp_bind_config\n");
#ifdef CONFIG_USB_G_LGE_MULTIPLE_CONFIGURATION
	if (c->bConfigurationValue == 2)
		goto multiple_mtp;
#endif

	/* allocate a string ID for our interface */
	if (mtp_string_defs[INTERFACE_STRING_INDEX].id == 0) {
		ret = usb_string_id(c->cdev);
		if (ret < 0)
			return ret;
		mtp_string_defs[INTERFACE_STRING_INDEX].id = ret;
		mtp_interface_desc.iInterface = ret;
	}

	dev->cdev = c->cdev;
	dev->function.name = "mtp";
	dev->function.strings = mtp_strings;
	if (ptp_config) {
		dev->function.descriptors = fs_ptp_descs;
		dev->function.hs_descriptors = hs_ptp_descs;
		if (gadget_is_superspeed(c->cdev->gadget))
			dev->function.ss_descriptors = ss_ptp_descs;
	} else {
		dev->function.descriptors = fs_mtp_descs;
		dev->function.hs_descriptors = hs_mtp_descs;
		if (gadget_is_superspeed(c->cdev->gadget))
			dev->function.ss_descriptors = ss_mtp_descs;
	}
	dev->function.bind = mtp_function_bind;
	dev->function.unbind = mtp_function_unbind;
	dev->function.set_alt = mtp_function_set_alt;
	dev->function.disable = mtp_function_disable;
#ifdef CONFIG_USB_G_LGE_MULTIPLE_CONFIGURATION
	dev->function.desc_change = lge_mtp_desc_change;
#endif

	return usb_add_function(c, &dev->function);

#ifdef CONFIG_USB_G_LGE_MULTIPLE_CONFIGURATION
multiple_mtp:
	/* allocate a string ID for our interface */
		if (mtp_string_defs2[INTERFACE_STRING_INDEX].id == 0) {
			ret = usb_string_id(c->cdev);
			if (ret < 0)
				return ret;
			mtp_string_defs2[INTERFACE_STRING_INDEX].id = ret;
			mtp_interface_desc2.iInterface = ret;
		}

	dev->cdev = c->cdev;
	dev->function2.name = "mtp2";
	dev->function2.strings = mtp_strings2;
	if (ptp_config) {
		dev->function2.descriptors = fs_ptp_descs2;
		dev->function2.hs_descriptors = hs_ptp_descs2;
		if (gadget_is_superspeed(c->cdev->gadget))
			dev->function2.ss_descriptors = ss_ptp_descs2;
	} else {
		dev->function2.descriptors = fs_mtp_descs2;
		dev->function2.hs_descriptors = hs_mtp_descs2;
		if (gadget_is_superspeed(c->cdev->gadget))
			dev->function2.ss_descriptors = ss_mtp_descs2;
	}
	dev->function2.bind = mtp_function_bind;
	dev->function2.unbind = mtp_function_unbind;
	dev->function2.set_alt = mtp_function_set_alt;
	dev->function2.disable = mtp_function_disable;
#ifdef CONFIG_USB_G_LGE_MULTIPLE_CONFIGURATION
	dev->function2.desc_change = lge_mtp_desc_change;
#endif

	return usb_add_function(c, &dev->function2);
#endif
}

#if defined CONFIG_DEBUG_FS && defined CONFIG_USB_G_LGE_MTP_PROFILING
static char debug_buffer[PAGE_SIZE];

static ssize_t debug_profile_write(struct file *file, const char __user *ubuf,
		size_t count, loff_t *ppos)
{
	struct mtp_dev *dev = file->private_data;
	int value;

	sscanf(ubuf, "%d", &value);
	if (!value) {
		spin_lock(&dev->lock);
		memset(&dev->perf, 0, sizeof(dev->perf));
		spin_unlock(&dev->lock);
	}

	return count;
}

static ssize_t debug_profile_read(struct file *file, char __user *ubuf,
		size_t count, loff_t *ppos)
{
	struct mtp_dev *dev = file->private_data;

	char *buf = debug_buffer;
	unsigned long flags;
	int i = 0;
	uint64_t rbytes, wbytes;
	uint64_t t_rbytes, t_wbytes;
	unsigned long rtemp, wtemp, t_rtemp, t_wtemp;
	int64_t rtime, wtime, send_time, receive_time;
	int64_t t_rtime, t_wtime, t_send_time, t_receive_time;
	int64_t file_ready_rtime, file_ready_wtime;
	int64_t fr_avg_rtime, fr_avg_wtime;

	if (!dev)
		return 0;

	spin_lock_irqsave(&dev->lock, flags);
	rbytes = dev->perf.rbytes;
	wbytes = dev->perf.wbytes;

	t_rbytes = dev->perf.t_rbytes;
	t_wbytes = dev->perf.t_wbytes;

	rtime = ktime_to_ms(dev->perf.rtime);
	wtime = ktime_to_ms(dev->perf.wtime);
	send_time = ktime_to_ms(dev->perf.send_time);
	receive_time = ktime_to_ms(dev->perf.receive_time);

	t_rtime = ktime_to_ms(dev->perf.t_rtime);
	t_wtime = ktime_to_ms(dev->perf.t_wtime);
	t_send_time = ktime_to_ms(dev->perf.t_send_time);
	t_receive_time = ktime_to_ms(dev->perf.t_receive_time);

	file_ready_rtime = ktime_to_ms(ktime_sub(ktime_sub(dev->perf.last_end_rtime,
				   dev->perf.first_start_rtime), dev->perf.t_send_time));
	file_ready_wtime = ktime_to_ms(ktime_sub(ktime_sub(dev->perf.last_end_wtime,
					dev->perf.first_start_wtime), dev->perf.t_receive_time));
	fr_avg_rtime = dev->perf.r_count ?
		DIV_ROUND_UP_ULL(file_ready_rtime, dev->perf.r_count) : 0;
	fr_avg_wtime = dev->perf.w_count ?
		DIV_ROUND_UP_ULL(file_ready_wtime, dev->perf.w_count) : 0;

	wtemp = receive_time ?
		DIV_ROUND_UP_ULL(wbytes, receive_time) * 1000 / 1024 : 0;
	rtemp = send_time + file_ready_rtime ?
		DIV_ROUND_UP_ULL(rbytes, send_time) * 1000 / 1024 : 0;
	t_wtemp = t_receive_time + file_ready_wtime ?
		DIV_ROUND_UP_ULL(t_wbytes, t_receive_time + file_ready_wtime) * 1000 / 1024 : 0;
	t_rtemp = t_send_time ?
		DIV_ROUND_UP_ULL(t_rbytes, t_send_time + file_ready_rtime) * 1000 / 1024 : 0 ;

	i += snprintf(buf + i, PAGE_SIZE - i,
			"\nLast file throughput\n");
	i += snprintf(buf + i, PAGE_SIZE - i,
			"Receive performance(vfs write+usb_delay=receive time)\n"
			"%llu bytes in (%lld+%lld=%lld) miliseconds (%luKB/s)\n",
			wbytes, wtime, receive_time-wtime, receive_time, wtemp);
	i += snprintf(buf + i, PAGE_SIZE - i,
			"\nSend performance(vfs read+usb delay=send time)\n"
			"%llu bytes in (%lld+%lld=%lld) miliseconds (%luKB/s)\n",
			rbytes, rtime, send_time-rtime, send_time, rtemp);

	i += snprintf(buf + i, PAGE_SIZE - i,
			"\n\nTotal file throughput\n");
	i += snprintf(buf + i, PAGE_SIZE - i,
			"Receive performance(vfs write+usb delay+file interval=total receive time)\n"
			"%llu bytes in (%lld+%lld+%lld=%lld) miliseconds (%luKB/s)\n",
			t_wbytes, t_wtime, t_receive_time-t_wtime, file_ready_wtime,
			t_receive_time+file_ready_wtime, t_wtemp);
	i += snprintf(buf + i, PAGE_SIZE - i,
			"Receive file count: %d\n"
			"Average of interval to ready file: %lld miliseconds\n",
			dev->perf.w_count, fr_avg_wtime);
	i += snprintf(buf + i, PAGE_SIZE - i,
			"\nSend performance(vfs read+usb delay+file interval=total send time)\n"
			"%llu bytes in (%lld+%lld+%lld=%lld) miliseconds (%luKB/s)\n",
			t_rbytes, t_rtime, t_send_time-t_rtime, file_ready_rtime,
			t_send_time+file_ready_rtime, t_rtemp);
	i += snprintf(buf + i, PAGE_SIZE - i,
			"Send file count: %d\n"
			"Average of interval to ready file %lld miliseconds\n",
			dev->perf.r_count, fr_avg_wtime);

	spin_unlock_irqrestore(&dev->lock, flags);

	return simple_read_from_buffer(ubuf, count, ppos, buf, i);
}

static int debug_profile_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

const struct file_operations debug_profile_ops = {
	.open = debug_profile_open,
	.read = debug_profile_read,
	.write = debug_profile_write,
};

static void mtp_debugfs_init(struct mtp_dev *dev)
{
	struct dentry *dent;
	dent = debugfs_create_dir("usb_mtp", 0);
	if (IS_ERR(dent))
		return;

	debugfs_create_file("profile", 0444, dent, dev, &debug_profile_ops);
}
#endif /* CONFIG_USB_G_LGE_MTP_PROFILING && CONFIG_DEBUG_FS */

static int mtp_setup(void)
{
	struct mtp_dev *dev;
	int ret;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	spin_lock_init(&dev->lock);
	init_waitqueue_head(&dev->read_wq);
	init_waitqueue_head(&dev->write_wq);
	init_waitqueue_head(&dev->intr_wq);
	atomic_set(&dev->open_excl, 0);
	atomic_set(&dev->ioctl_excl, 0);
	INIT_LIST_HEAD(&dev->tx_idle);
	INIT_LIST_HEAD(&dev->intr_idle);

	dev->wq = create_singlethread_workqueue("f_mtp");
	if (!dev->wq) {
		ret = -ENOMEM;
		goto err1;
	}
	INIT_WORK(&dev->send_file_work, send_file_work);
	INIT_WORK(&dev->receive_file_work, receive_file_work);

	_mtp_dev = dev;

	ret = misc_register(&mtp_device);
	if (ret)
		goto err2;

#if defined CONFIG_DEBUG_FS && defined CONFIG_USB_G_LGE_MTP_PROFILING
	mtp_debugfs_init(dev);
#endif
	return 0;

err2:
	destroy_workqueue(dev->wq);
err1:
	_mtp_dev = NULL;
	kfree(dev);
	printk(KERN_ERR "mtp gadget driver failed to initialize\n");
	return ret;
}

static void mtp_cleanup(void)
{
	struct mtp_dev *dev = _mtp_dev;

	if (!dev)
		return;

	misc_deregister(&mtp_device);
	destroy_workqueue(dev->wq);
	_mtp_dev = NULL;
	kfree(dev);
}
