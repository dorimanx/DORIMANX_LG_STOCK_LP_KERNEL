/* arch/arm/mach-msm/include/mach/memory.h
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2009-2013, The Linux Foundation. All rights reserved.
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
#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H
#include <linux/types.h>

/* physical offset of RAM */
#define PLAT_PHYS_OFFSET UL(CONFIG_PHYS_OFFSET)

#if defined(CONFIG_KEXEC_HARDBOOT)
#if defined(CONFIG_MACH_APQ8064_FLO)
#define KEXEC_HB_PAGE_ADDR		UL(0x88C00000)
#elif defined(CONFIG_MACH_APQ8064_MAKO)
#define KEXEC_HB_PAGE_ADDR		UL(0x88600000)
#elif defined(CONFIG_MACH_MSM8974_HAMMERHEAD)
#define KEXEC_HB_PAGE_ADDR		UL(0x10100000)
#define KEXEC_HB_KERNEL_LOC		UL(0x3208000)
#elif defined(CONFIG_MACH_MSM8974_G2_OPEN_COM) || \
		defined(CONFIG_MACH_MSM8974_G2_ATT) || \
		defined(CONFIG_MACH_MSM8974_G2_TMO_US) || \
		defined(CONFIG_MACH_MSM8974_G2_CA) || \
		defined(CONFIG_MACH_MSM8974_G2_TLS) || \
		defined(CONFIG_MACH_MSM8974_G2_KR) || \
		defined(CONFIG_MACH_MSM8974_G2_KT) || \
		defined(CONFIG_MACH_MSM8974_G2_SPR) || \
		defined(CONFIG_MACH_MSM8974_G2_VZW)
#define KEXEC_HB_PAGE_ADDR		UL(0x07b00000)
#else
#error "Adress for kexec hardboot page not defined"
#endif
#endif

#define MAX_PHYSMEM_BITS 32
#define SECTION_SIZE_BITS 28

/* Maximum number of Memory Regions
*  The largest system can have 4 memory banks, each divided into 8 regions
*/
#define MAX_NR_REGIONS 32

/* The number of regions each memory bank is divided into */
#define NR_REGIONS_PER_BANK 8

/* Certain configurations of MSM7x30 have multiple memory banks.
*  One or more of these banks can contain holes in the memory map as well.
*  These macros define appropriate conversion routines between the physical
*  and virtual address domains for supporting these configurations using
*  SPARSEMEM and a 3G/1G VM split.
*/

#ifndef __ASSEMBLY__
void *allocate_contiguous_ebi(unsigned long, unsigned long, int);
phys_addr_t allocate_contiguous_ebi_nomap(unsigned long, unsigned long);
void clean_and_invalidate_caches(unsigned long, unsigned long, unsigned long);
void clean_caches(unsigned long, unsigned long, unsigned long);
void invalidate_caches(unsigned long, unsigned long, unsigned long);
int msm_get_memory_type_from_name(const char *memtype_name);

#if defined(CONFIG_ARCH_MSM_ARM11) || defined(CONFIG_ARCH_MSM_CORTEX_A5)
void write_to_strongly_ordered_memory(void);
void map_page_strongly_ordered(void);
#endif

#ifdef CONFIG_CACHE_L2X0
extern void l2x0_cache_sync(void);
#define finish_arch_switch(prev)     do { l2x0_cache_sync(); } while (0)
#endif

#define MAX_HOLE_ADDRESS    (PHYS_OFFSET + 0x10000000)
/*
 * Need a temporary unique variable that no one will ever see to
 * hold the compat string. Line number gives this easily.
 * Need another layer of indirection to get __LINE__ to expand
 * properly as opposed to appending and ending up with
 * __compat___LINE__
 */
#define __CONCAT(a, b)	___CONCAT(a, b)
#define ___CONCAT(a, b)	a ## b

#define EXPORT_COMPAT(com)	\
static char *__CONCAT(__compat_, __LINE__)  __used \
	__attribute((__section__(".exportcompat.init"))) = com

extern char *__compat_exports_start[];
extern char *__compat_exports_end[];

#endif

#if defined CONFIG_ARCH_MSM_SCORPION || defined CONFIG_ARCH_MSM_KRAIT
#define arch_has_speculative_dfetch()	1
#endif

#endif

#define CONSISTENT_DMA_SIZE	(SZ_1M * 14)
