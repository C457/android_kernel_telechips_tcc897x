/*
 * arch/arm/mach-tcc897x/tcc897x_mpcm.c
 *
 * Copyright:	(C) 2014  Telechips Inc.
 *
 * Created by:	Nicolas Pitre, October 2012
 * Copyright:	(C) 2012  Linaro Limited
 *
 * Some portions of this file were originally written by Achin Gupta
 * Copyright:   (C) 2012  ARM Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/errno.h>
#include <linux/irqchip/arm-gic.h>

#include <asm/mcpm.h>
#include <asm/proc-fns.h>
#include <asm/cacheflush.h>
#include <asm/cputype.h>
#include <asm/cp15.h>
#include <asm/psci.h>
#include <asm/io.h>

#include <linux/vexpress.h>
#include <linux/arm-cci.h>

#include <plat/platsmp.h>
#include <mach/iomap.h>

#define SEC_CPU_START_ADDR	0x14400160
#define SEC_CPU_START_CFG	0x14400130
#define SEC_CPU_START_VALUE	0x00000001
#define PMU_SEC_VALID		0x130	/* offset: 0x08 */
#define PMU_SEC_READY		0x134	/* offset: 0x08 */
#define PMU_SEC_START		0x160	/* offset: 0x04 */

#define	TCC897X_MAX_CPUS	4
#define TCC897X_MAX_CLUSTERS	2

extern void tcc_mcpm_power_up_setup(unsigned int affinity_level);

/*
 * We can't use regular spinlocks. In the switcher case, it is possible
 * for an outbound CPU to call power_down() after its inbound counterpart
 * is already live using the same logical CPU number which trips lockdep
 * debugging.
 */
static arch_spinlock_t tcc897x_mcpm_lock = __ARCH_SPIN_LOCK_UNLOCKED;
static int tcc897x_mcpm_use_count[TCC897X_MAX_CPUS][TCC897X_MAX_CLUSTERS];

static int actived_cluster = 0;

static inline int tcc897x_get_nb_cpus(u32 cluster)
{
	switch (cluster) {
	case 0: return 2;
	case 1: return 4;
	}

	return 0;
}

static void tcc_set_wakeup_irq(u32 cluster, u32 cpu, bool set)
{
	// TODO:
}

static void tcc_write_resume_reg(u32 cluster, u32 cpu, u32 addr)
{
	void __iomem *baseaddr;

	if (WARN_ON_ONCE(cluster >= TCC897X_MAX_CLUSTERS))
		return;

	if (cluster == 0)
		baseaddr = (void __iomem *)io_p2v(SEC_CPU_START_ADDR+(cpu*0x4));
	else if (cluster == 1)
		baseaddr = (void __iomem *)io_p2v(SEC_CPU_START_ADDR+0x8+(cpu*0x4));
	else
		BUG();

	writel_relaxed(addr, baseaddr);
}

static void tcc_write_secondary_cfg_reg(u32 cluster, u32 cpu)
{
	void __iomem *baseaddr;

	if (WARN_ON_ONCE(cluster >= TCC897X_MAX_CLUSTERS))
		return;

	// sangwon_temp, 20141104, do not wake-up other clusters
	if (actived_cluster != cluster)
		return;

	if (cluster == 0)
		baseaddr = (void __iomem *)io_p2v(SEC_CPU_START_CFG+(cpu*0x8));
	else if (cluster == 1)
		baseaddr = (void __iomem *)io_p2v(SEC_CPU_START_CFG+(0x8*2)+(cpu*0x8));
	else
		BUG();

	writel_relaxed(1, baseaddr);
}

static int tcc897x_mcpm_power_up(unsigned int cpu, unsigned int cluster)
{
	pr_debug("%s: cpu %u cluster %u\n", __func__, cpu, cluster);
	if (cluster >= TCC897X_MAX_CLUSTERS ||
	    cpu >= tcc897x_get_nb_cpus(cluster))
		return -EINVAL;

	/*
	 * Since this is called with IRQs enabled, and no arch_spin_lock_irq
	 * variant exists, we need to disable IRQs manually here.
	 */
	local_irq_disable();
	arch_spin_lock(&tcc897x_mcpm_lock);

	if (!tcc897x_mcpm_use_count[0][cluster] &&
	    !tcc897x_mcpm_use_count[1][cluster] &&
	    !tcc897x_mcpm_use_count[2][cluster])
		;
//		vexpress_spc_powerdown_enable(cluster, 0);

	tcc897x_mcpm_use_count[cpu][cluster]++;
	if (tcc897x_mcpm_use_count[cpu][cluster] == 1) {
		tcc_write_resume_reg(cluster, cpu, virt_to_phys(mcpm_entry_point));
		tcc_write_secondary_cfg_reg(cluster, cpu);
		tcc_set_wakeup_irq(cpu, cluster, 1);
	} else if (tcc897x_mcpm_use_count[cpu][cluster] != 2) {
		/*
		 * The only possible values are:
		 * 0 = CPU down
		 * 1 = CPU (still) up
		 * 2 = CPU requested to be up before it had a chance
		 *     to actually make itself down.
		 * Any other value is a bug.
		 */
		BUG();
	}

	arch_spin_unlock(&tcc897x_mcpm_lock);
	local_irq_enable();

	return 0;
}

static void tcc897x_mcpm_down(u64 residency)
{
	unsigned int mpidr, cpu, cluster;
	bool last_man = false, skip_wfi = false;

	mpidr = read_cpuid_mpidr();
	cpu = MPIDR_AFFINITY_LEVEL(mpidr, 0);
	cluster = MPIDR_AFFINITY_LEVEL(mpidr, 1);

	pr_debug("%s: cpu %u cluster %u\n", __func__, cpu, cluster);
	BUG_ON(cluster >= TCC897X_MAX_CLUSTERS ||
	       cpu >= tcc897x_get_nb_cpus(cluster));

	__mcpm_cpu_going_down(cpu, cluster);

	arch_spin_lock(&tcc897x_mcpm_lock);
	BUG_ON(__mcpm_cluster_state(cluster) != CLUSTER_UP);
	tcc897x_mcpm_use_count[cpu][cluster]--;
	if (tcc897x_mcpm_use_count[cpu][cluster] == 0) {
		//tcc_set_wakeup_irq(cpu, cluster, 1);
		if (!tcc897x_mcpm_use_count[0][cluster] &&
		    !tcc897x_mcpm_use_count[1][cluster] &&
		    !tcc897x_mcpm_use_count[2][cluster] &&
		    (!residency || residency > 5000)) {
//			vexpress_spc_powerdown_enable(cluster, 1);
//			vexpress_spc_set_global_wakeup_intr(1);
			last_man = true;
		}
	} else if (tcc897x_mcpm_use_count[cpu][cluster] == 1) {
		/*
		 * A power_up request went ahead of us.
		 * Even if we do not want to shut this CPU down,
		 * the caller expects a certain state as if the WFI
		 * was aborted.  So let's continue with cache cleaning.
		 */
		skip_wfi = true;
	} else
		BUG();

	/*
	 * If the CPU is committed to power down, make sure
	 * the power controller will be in charge of waking it
	 * up upon IRQ, ie IRQ lines are cut from GIC CPU IF
	 * to the CPU by disabling the GIC CPU IF to prevent wfi
	 * from completing execution behind power controller back
	 */
	if (!skip_wfi)
		gic_cpu_if_down();

	if (last_man && __mcpm_outbound_enter_critical(cpu, cluster)) {
		arch_spin_unlock(&tcc897x_mcpm_lock);

#if (0)
		if (read_cpuid_part_number() == ARM_CPU_PART_CORTEX_A15) {
			/*
			 * On the Cortex-A15 we need to disable
			 * L2 prefetching before flushing the cache.
			 */
			asm volatile(
			"mcr	p15, 1, %0, c15, c0, 3 \n\t"
			"isb	\n\t"
			"dsb	"
			: : "r" (0x400) );
		}
#endif

		v7_exit_coherency_flush(all);

		cci_disable_port_by_cpu(mpidr);

		__mcpm_outbound_leave_critical(cluster, CLUSTER_DOWN);
	} else {
		/*
		 * If last man then undo any setup done previously.
		 */
		if (last_man) {
//			vexpress_spc_powerdown_enable(cluster, 0);
//			vexpress_spc_set_global_wakeup_intr(0);
		}

		arch_spin_unlock(&tcc897x_mcpm_lock);

		v7_exit_coherency_flush(louis);
	}

	__mcpm_cpu_down(cpu, cluster);

	/* Now we are prepared for power-down, do it: */
	if (!skip_wfi) {
#if (1) // sangwon_temp, 20150107, move pc to mcpm_boot code for waiting wfe instead of power down
		volatile void __iomem *reg_pmu = (void __iomem *)io_p2v(TCC_PA_PMU);
		volatile unsigned cpu_id = (cluster*2+cpu);

		writel_relaxed(0, reg_pmu + PMU_SEC_VALID + (cpu_id*0x8));
		writel_relaxed(1, reg_pmu + PMU_SEC_READY + (cpu_id*0x8));

		asm volatile ("mov	pc, #0xF0000000");
		while(1);
#else
		wfi();
#endif
	}

	/* Not dead at this point?  Let our caller cope. */
}

static void tcc897x_mcpm_power_down(void)
{
	unsigned int mpidr, cpu, cluster;
	mpidr = read_cpuid_mpidr();
	cpu = MPIDR_AFFINITY_LEVEL(mpidr, 0);
	cluster = MPIDR_AFFINITY_LEVEL(mpidr, 1);

	tcc897x_mcpm_down(0);
}

static void tcc897x_mcpm_suspend(u64 residency)
{
	extern void tcc_mcpm_resume(void);
	unsigned int mpidr, cpu, cluster;

	mpidr = read_cpuid_mpidr();
	cpu = MPIDR_AFFINITY_LEVEL(mpidr, 0);
	cluster = MPIDR_AFFINITY_LEVEL(mpidr, 1);

//	BUG();

	tcc_write_resume_reg(cluster, cpu,
				      virt_to_phys(tcc_mcpm_resume));

/*
	{
		void __iomem *reg_pmu = (void __iomem *)io_p2v(TCC_PA_PMU);

		writel(0, reg_pmu + PMU_SEC_VALID + (0x8*(cpu+cluster*2)));
		writel(1, reg_pmu + PMU_SEC_READY + (0x8*(cpu+cluster*2)));
		asm volatile("mov  pc, #0xF0000000");
	}
*/
	tcc897x_mcpm_down(residency);
}

static void tcc897x_mcpm_powered_up(void)
{
	unsigned int mpidr, cpu, cluster;
	unsigned long flags;

	mpidr = read_cpuid_mpidr();
	cpu = MPIDR_AFFINITY_LEVEL(mpidr, 0);
	cluster = MPIDR_AFFINITY_LEVEL(mpidr, 1);

	pr_debug("%s: cpu %u cluster %u\n", __func__, cpu, cluster);
	BUG_ON(cluster >= TCC897X_MAX_CLUSTERS ||
	       cpu >= tcc897x_get_nb_cpus(cluster));

	local_irq_save(flags);
	arch_spin_lock(&tcc897x_mcpm_lock);

	if (!tcc897x_mcpm_use_count[0][cluster] &&
	    !tcc897x_mcpm_use_count[1][cluster] &&
	    !tcc897x_mcpm_use_count[2][cluster]) {
//		vexpress_spc_powerdown_enable(cluster, 0);
//		vexpress_spc_set_global_wakeup_intr(0);
	}

	if (!tcc897x_mcpm_use_count[cpu][cluster])
		tcc897x_mcpm_use_count[cpu][cluster] = 1;

	//tcc_set_wakeup_irq(cpu, cluster, 0);
	tcc_write_resume_reg(cluster, cpu, 0);

	arch_spin_unlock(&tcc897x_mcpm_lock);
	local_irq_restore(flags);
}

static const struct mcpm_platform_ops tcc897x_mcpm_power_ops = {
	.power_up	= tcc897x_mcpm_power_up,
	.power_down	= tcc897x_mcpm_power_down,
	.suspend	= tcc897x_mcpm_suspend,
	.powered_up	= tcc897x_mcpm_powered_up,
};

static void __init tcc897x_mcpm_usage_count_init(void)
{
	unsigned int mpidr, cpu, cluster;

	mpidr = read_cpuid_mpidr();
	cpu = MPIDR_AFFINITY_LEVEL(mpidr, 0);
	cluster = MPIDR_AFFINITY_LEVEL(mpidr, 1);

	pr_debug("%s: cpu %u cluster %u\n", __func__, cpu, cluster);

//	BUG_ON(cluster >= TCC897X_MAX_CLUSTERS ||
//	       cpu >= vexpress_spc_get_nb_cpus(cluster));

	actived_cluster = cluster;
	tcc897x_mcpm_use_count[cpu][cluster] = 1;
}

static int __init tcc897x_mcpm_init(void)
{
	int ret;

	tcc897x_mcpm_usage_count_init();

	ret = mcpm_platform_register(&tcc897x_mcpm_power_ops);
	if (!ret)
		ret = mcpm_sync_init(tcc_mcpm_power_up_setup);
	if (!ret)
		pr_info("MCPM power management initialized\n");
	return ret;
}

early_initcall(tcc897x_mcpm_init);
