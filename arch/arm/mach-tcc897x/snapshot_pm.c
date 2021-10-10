//+[TCCQB] snapshot functions...
/****************************************************************************
 * plat-tcc/snapshot.c
 * Copyright (C) 2014 Telechips Inc.
 *
 * This program is free software; you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation;
 * either version 2 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA 02111-1307 USA
****************************************************************************/

#include <linux/module.h>
#include <linux/suspend.h>
#include <asm/io.h>
#include <asm/suspend.h>
#include <mach/bsp.h>
#include <mach/pm.h>
#include <mach/sram_map.h>
#include <plat/suspend.h>
#include <plat/pmu_wakeup.h>
#include <mach/iomap.h>
#include <asm/tlbflush.h>
#include <asm/cacheflush.h>		// local_flush_tlb_all(), flush_cache_all();
#include <linux/cpu_pm.h>		// cpu_pm_enter(), cpu_pm_exit()
#include <mach/tcc_sram.h>


#define snapshot_writel	__raw_writel

#define SNAPSHOT_RESTORE_PHY_ADDR	sram_p2v(CPU_DATA_REPOSITORY_ADDR)
#define SNAPSHOT_STORE_SRAM_ADDR	sram_p2v(CPU_DATA_REPOSITORY_ADDR+0x10)
#define SNAPSHOT_STORE_SRAM_SIZE	0x100

TCC_REG TCC_SnapShot_Regs;
static TCC_REG *RegRepo = &TCC_SnapShot_Regs;
static struct tcc_suspend_ops *snapshot_ops = NULL;
static unsigned gpio_regs[GPIO_REPOSITORY_SIZE/4];
static unsigned pmu_wakeup[4];

extern void v7_invalidate_l1(void);
extern void v7_flush_icache_all(void);
extern unsigned int get_do_snapshot_boot(void);

void snapshot_restore(void)
{
	/* GPIO Reg. Back-up */
	memcpy((void *)TCC_PA_GPIO, (void*)gpio_regs, GPIO_REPOSITORY_SIZE);

	v7_invalidate_l1();
	v7_flush_icache_all();

	cpu_resume();
}

static void snapshot_store(unsigned arg)
{
	unsigned mmu;

	/* change stack pointer to SRAM */
	asm volatile ("mov r13, %0" :: "r" (SRAM_STACK_ADDR));

	/* mmu & cache off */
	isb();
	asm volatile ("mrc p15, 0, %0, c1, c0, 0" : "=r" (mmu));
	asm volatile ("mcr p15, 0, %0, c1, c0, 0" :: "r" (mmu & ~((1<<12)|(1<<2)|(1<<0))));
	isb();


	/* call snapshot_restore() */
	asm volatile (
		"ldr	r0, [%0] \n"
		"mov	pc, r0 \n"
		 :: "r" (SNAPSHOT_RESTORE_PHY_ADDR)
	);
	while(1);
}

void snapshot_state_restore(void)
{
	if (get_do_snapshot_boot()) {
		/* restore cpu offset */
		asm volatile ( "mcr p15, 0, %0, c13, c0, 4" :: "r" (RegRepo->TPIDRPRW));

		// Sangwon_temp, 2015.02.03, Temporally add for restoring GPIO staus
		memcpy((void *)io_p2v(TCC_PA_GPIO), (void*)gpio_regs, GPIO_REPOSITORY_SIZE);

		tcc_ckc_restore_regs();
		tcc_pm_nop_delay(100);

		__cpu_early_init();

		/* SMU & PMU */
		memcpy((unsigned *)io_p2v(TCC_PA_PIC), RegRepo->pic, PM_PIC_SIZE);
		memcpy((unsigned *)io_p2v(TCC_PA_VIC), RegRepo->vic, PM_VIC_SIZE);
		memcpy((unsigned *)io_p2v(TCC_PA_TIMER), RegRepo->timer, PM_TIMER_SIZE);
	}

#ifdef CONFIG_ARM_GIC
	cpu_cluster_pm_exit();
	cpu_pm_exit();
#endif

#if defined(CONFIG_PM_CONSOLE_NOT_SUSPEND)
	if (snapshot_ops->uart_resume)
		snapshot_ops->uart_resume(RegRepo->uart, RegRepo->uartcfg);
#endif

	/* NFC resume */
	if (snapshot_ops->nfc_resume)
		snapshot_ops->nfc_resume(RegRepo->nfc);

	/* Re-Init SRAM Functions */
	if (get_do_snapshot_boot()) {
		tcc_sram_init();
	}
	pmu_wakeup_param(PMU_WAKEUP_WKUP0) = pmu_wakeup[0];
	pmu_wakeup_param(PMU_WAKEUP_WKUP1) = pmu_wakeup[1];
	pmu_wakeup_param(PMU_WAKEUP_WKPOL0) = pmu_wakeup[2];
	pmu_wakeup_param(PMU_WAKEUP_WKPOL1) = pmu_wakeup[3];

#ifdef CONFIG_LK_DEBUG_LOG_BUF
	/* lk_log resume */
	if (snapshot_ops->lk_log_resume)
		snapshot_ops->lk_log_resume();
#endif
}

void snapshot_state_store(void)
{
	snapshot_writel(virt_to_phys(snapshot_restore),	IOMEM(SNAPSHOT_RESTORE_PHY_ADDR));
	memcpy((void*)(SNAPSHOT_STORE_SRAM_ADDR), (void*)snapshot_store, SNAPSHOT_STORE_SRAM_SIZE);

	memcpy((void*)gpio_regs, (void *)io_p2v(TCC_PA_GPIO), GPIO_REPOSITORY_SIZE);


	/* NFC suspend */
	if (snapshot_ops->nfc_suspend)
		snapshot_ops->nfc_suspend(RegRepo->nfc);

#if defined(CONFIG_PM_CONSOLE_NOT_SUSPEND)
	if (snapshot_ops->uart_suspend)
		snapshot_ops->uart_suspend(RegRepo->uart, RegRepo->uartcfg);
#endif

#ifdef CONFIG_ARM_GIC
	cpu_pm_enter();
	cpu_cluster_pm_enter();
#endif

	memcpy(RegRepo->timer, (unsigned *)io_p2v(TCC_PA_TIMER), PM_TIMER_SIZE);
	memcpy(RegRepo->vic, (unsigned *)io_p2v(TCC_PA_VIC), PM_VIC_SIZE);
	memcpy(RegRepo->pic, (unsigned *)io_p2v(TCC_PA_PIC), PM_PIC_SIZE);

	pmu_wakeup[0] = pmu_wakeup_param(PMU_WAKEUP_WKUP0);
	pmu_wakeup[1] = pmu_wakeup_param(PMU_WAKEUP_WKUP1);
	pmu_wakeup[2] = pmu_wakeup_param(PMU_WAKEUP_WKPOL0);
	pmu_wakeup[3] = pmu_wakeup_param(PMU_WAKEUP_WKPOL1);

	tcc_ckc_backup_regs(0);	// <- ckc & pmu setting.
					// pmu : init pmu except some pmu register which is already set on bootloader

	/* backup cpu offset */
	asm volatile ( "mrc p15, 0, %0, c13, c0, 4" : "=r" (RegRepo->TPIDRPRW));
	
	/*	L2 cache & scu is not needed to restore at CORE A15		*/

	local_flush_tlb_all();
	flush_cache_all();

//	cpu_suspend(0, (void *)SNAPSHOT_STORE_SRAM_ADDR);
}

void tcc_snapshot_set_ops(struct tcc_suspend_ops *ops)
{
	snapshot_ops = ops;
}
//-[TCCQB]
//
