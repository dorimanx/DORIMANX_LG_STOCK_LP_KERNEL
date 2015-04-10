#include <linux/syscalls.h>
#include <linux/fs.h>

#include "diag_acg.h"

/* refer from android/bootable/bootloader/lk/platform/lge_shared/include/lge_ftm.h */
//#define LGFTM_ACG_CARRIER_CODE       103
#define LGFTM_ACG_CARRIER_CODE      34 
#define LGFTM_ACG_CARRIER_CODE_SIZE    1
#define FTM_PAGE_SIZE               2048

static const char *ftmdev = "/dev/block/platform/msm_sdcc.1/by-name/misc";

int set_carrier_code(unsigned long int in)
{
	int misc_fd;
	char buf = (char)in;

	misc_fd = sys_open(ftmdev, O_RDWR, 0);
	if (misc_fd < 0) {
		pr_err("%s: sys_open error(%d)\n", __func__, misc_fd);
		return -1;
	}

	if (sys_lseek(misc_fd, (LGFTM_ACG_CARRIER_CODE * FTM_PAGE_SIZE), SEEK_SET) < 0) {
		pr_err("%s: sys_lseek error\n", __func__);
		sys_close(misc_fd);
		return -1;
	}

	if (sys_write(misc_fd, &buf, LGFTM_ACG_CARRIER_CODE_SIZE) != 1) {
		pr_err("%s: sys_write error\n", __func__);
		sys_close(misc_fd);
		return -1;
	}

	if (sys_close(misc_fd) != 0) {
		pr_err("%s: sys_close error\n", __func__);
		return -1;
	}

	return 0;
}

int get_carrier_code(void)
{
	int misc_fd;
	char carrier_code;

	misc_fd = sys_open(ftmdev, O_RDWR, 0);
	if (misc_fd < 0) {
		pr_err("%s: sys_open error(%d)\n", __func__, misc_fd);
		return -1;
	}

	if (sys_lseek(misc_fd, (LGFTM_ACG_CARRIER_CODE * FTM_PAGE_SIZE), SEEK_SET) < 0) {
		pr_err("%s: sys_lseek error\n", __func__);
		sys_close(misc_fd);
		return -1;
	}

	if (sys_read(misc_fd, &carrier_code, LGFTM_ACG_CARRIER_CODE_SIZE) != 1) {
		pr_err("%s: sys_write error\n", __func__);
		sys_close(misc_fd);
		return -1;
	}

	if (sys_close(misc_fd) != 0) {
		pr_err("%s: sys_close error\n", __func__);
		return -1;
	}
	return (int)carrier_code;
}

