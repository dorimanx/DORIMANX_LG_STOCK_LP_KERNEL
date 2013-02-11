#include <linux/linkage.h>
#include <linux/init.h>

#include <asm/cache.h>

#include <mach/iomap.h>

#include "flowctrl.h"
#include "reset.h"

#define APB_MISC_GP_HIDREV	0x804
#define PMC_SCRATCH41	0x140

#define RESET_DATA(x)	((TEGRA_RESET_##x)*4)

	.macro mov32, reg, val
	movw	\reg, #:lower16:\val
	movt	\reg, #:upper16:\val
	.endm

        .section ".text.head", "ax"
	__CPUINIT


ENTRY(tegra_secondary_startup)
        bl      v7_invalidate_l1
	/* Enable coresight */
	mov32	r0, 0xC5ACCE55
	mcr	p14, 0, r0, c7, c12, 6
        b       secondary_startup
ENDPROC(tegra_secondary_startup)

	.align L1_CACHE_SHIFT
ENTRY(__tegra_cpu_reset_handler_start)

/*
 * __tegra_cpu_reset_handler:
 *
 * Common handler for all CPU reset events.
 *
 * Register usage within the reset handler:
 *
 *      R7  = CPU present (to the OS) mask
 *      R8  = CPU in LP1 state mask
 *      R9  = CPU in LP2 state mask
 *      R10 = CPU number
 *      R11 = CPU mask
 *      R12 = pointer to reset handler data
 *
 * NOTE: This code is copied to IRAM. All code and data accesses
 *       must be position-independent.
 */

	.align L1_CACHE_SHIFT
ENTRY(__tegra_cpu_reset_handler)

	cpsid	aif, 0x13			@ SVC mode, interrupts disabled
	mrc	p15, 0, r10, c0, c0, 5		@ MPIDR
	and	r10, r10, #0x3			@ R10 = CPU number
	mov	r11, #1
	mov	r11, r11, lsl r10  		@ R11 = CPU mask
	adr	r12, __tegra_cpu_reset_handler_data

#ifdef CONFIG_SMP
	/* Does the OS know about this CPU? */
	ldr	r7, [r12, #RESET_DATA(MASK_PRESENT)]
	tst	r7, r11 			@ if !present
	bleq	__die				@ CPU not present (to OS)
#endif

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	/* Are we on Tegra20? */
	mov32	r6, TEGRA_APB_MISC_BASE
	ldr	r0, [r6, #APB_MISC_GP_HIDREV]
	and	r0, r0, #0xff00
	cmp	r0, #(0x20 << 8)
	bne	1f
	/* If not CPU0, don't let CPU0 reset CPU1 now that CPU1 is coming up. */
	mov32	r6, TEGRA_PMC_BASE
	mov	r0, #0
	cmp	r10, #0
	strne	r0, [r6, #PMC_SCRATCH41]
1:
#endif

#ifdef CONFIG_SMP
	/*
	 * Can only be secondary boot (initial or hotplug) but CPU 0
	 * cannot be here.
	 */
	cmp	r10, #0
	bleq	__die				@ CPU0 cannot be here
	ldr	lr, [r12, #RESET_DATA(STARTUP_SECONDARY)]
	cmp	lr, #0
	bleq	__die				@ no secondary startup handler
	bx	lr
#endif

/*
 * We don't know why the CPU reset. Just kill it.
 * The LR register will contain the address we died at + 4.
 */

__die:
	sub	lr, lr, #4
	mov32	r7, TEGRA_PMC_BASE
	str	lr, [r7, #PMC_SCRATCH41]

	mov32	r7, TEGRA_CLK_RESET_BASE

	/* Are we on Tegra20? */
	mov32	r6, TEGRA_APB_MISC_BASE
	ldr	r0, [r6, #APB_MISC_GP_HIDREV]
	and	r0, r0, #0xff00
	cmp	r0, #(0x20 << 8)
	bne	1f

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	mov32	r0, 0x1111
	mov	r1, r0, lsl r10
	str	r1, [r7, #0x340]		@ CLK_RST_CPU_CMPLX_SET
#endif
1:
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
	mov32	r6, TEGRA_FLOW_CTRL_BASE

	cmp	r10, #0
	moveq	r1, #FLOW_CTRL_HALT_CPU0_EVENTS
	moveq	r2, #FLOW_CTRL_CPU0_CSR
	movne	r1, r10, lsl #3
	addne	r2, r1, #(FLOW_CTRL_CPU1_CSR-8)
	addne	r1, r1, #(FLOW_CTRL_HALT_CPU1_EVENTS-8)

	/* Clear CPU "event" and "interrupt" flags and power gate
	   it when halting but not before it is in the "WFI" state. */
	ldr	r0, [r6, +r2]
	orr	r0, r0, #FLOW_CTRL_CSR_INTR_FLAG | FLOW_CTRL_CSR_EVENT_FLAG
	orr	r0, r0, #FLOW_CTRL_CSR_ENABLE
	str	r0, [r6, +r2]

	/* Unconditionally halt this CPU */
	mov	r0, #FLOW_CTRL_WAITEVENT
	str	r0, [r6, +r1]
	ldr	r0, [r6, +r1]			@ memory barrier

	dsb
	isb
	wfi					@ CPU should be power gated here

	/* If the CPU didn't power gate above just kill it's clock. */

	mov	r0, r11, lsl #8
	str	r0, [r7, #348]			@ CLK_CPU_CMPLX_SET
#endif

	/* If the CPU still isn't dead, just spin here. */
	b	.
ENDPROC(__tegra_cpu_reset_handler)

	.align L1_CACHE_SHIFT
	.type	__tegra_cpu_reset_handler_data, %object
	.globl	__tegra_cpu_reset_handler_data
__tegra_cpu_reset_handler_data:
	.rept	TEGRA_RESET_DATA_SIZE
	.long	0
	.endr
	.align L1_CACHE_SHIFT

ENTRY(__tegra_cpu_reset_handler_end)
