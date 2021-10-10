/****************************************************************************
 * arch/arm/mach-tcc893x/pm.c
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
#include <linux/cpu_pm.h>		// cpu_pm_enter(), cpu_pm_exit()
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <asm/io.h>
#include <asm/cacheflush.h>		// local_flush_tlb_all(), flush_cache_all();
#include <asm/suspend.h>
#include <asm/cputype.h>
//#include <mach/bsp.h>
#include <video/tcc/gpio.h>
#include "pm.h"
//#include <mach/sram_map.h>
#include "sram_map.h"
//#include <mach/iomap.h>
#include <video/tcc/tcc_types.h>
#include "pmu_wakeup.h"
#include "suspend.h"
#include "tcc_sram.h"
//#include "../../clk/tcc/clk-tcc898x.h"
#ifdef CONFIG_ARM_TRUSTZONE
#include <mach/smc.h>
#endif

#define pmu_wakeup_param_phys(x) (*(volatile unsigned *)(PMU_WAKEUP_ADDR+(4*x)))

extern TCC_REG *RegRepo;
extern void tcc_ckc_backup_regs(unsigned int);
extern void tcc_ckc_restore_regs(void);

#define PMU_WDTCTRL	0x0D0
#define PMU_WKUP0	0x020
#define PMU_WKUP1	0x024
#define PMU_WKPOL0	0x028
#define PMU_WKPOL1	0x02C
#define PMU_WKCLR0	0x030
#define PMU_WKCLR1	0x034
#define PMU_WKSTS0	0x038
#define PMU_WKSTS1	0x03C
#define PMU_TOP_PWRCTRL	0x058
#define PMU_PWRDN_TOP	0x080
#define PMU_PWRDN_XIN	0x098
#define PMU_WKMOD0	0x120
#define PMU_WKMOD1	0x124

#define CKC_CPU0	0x008
#define CKC_CM		0x00C
#define CKC_MEM		0x010
#define CKC_IO		0x02C
#define CKC_HSIO	0x018
#define CKC_SMU		0x01C
#define CKC_PLL0	0x040
#define CKC_PLL1	0x044
#define CKC_PLL2	0x048
#define CKC_PLL3	0x04C
#define CKC_PLL4	0x050

#define CPU_CLK		0x000
#define CPU_PLLPMS	0x008

#define GPU3D_CLK		0x000
#define GPU3D_CORE_CLK	0x004
#define GPU3D_PLL		0x008

#define GPU2D_CLK		0x000
#define GPU2D_CORE_CLK	0x004
#define GPU2D_PLL		0x008

#define EDI_CTRL	0x000
#define EDI_CSNCFG0	0x004
#define EDI_CSNCFG1	0x008
#define EDI_OENCFG	0x00C
#define EDI_WENCFG	0x010
#define EDI_RDYCFG	0x014

#define NFC_CACYC	0x044
#define NFC_WRCYC	0x048
#define NFC_RECYC	0x04C
#define NFC_CTRL	0x050
#define NFC_IRQCFG	0x064
#define NFC_IRQ		0x068
#define NFC_RFWBASE	0x070

#define DDR_CKC		0x004
#define DDR_PLL		0x008
/* temp define */
static void __iomem *ckc_base = NULL;
static void __iomem *pmu_base = NULL;

static void __iomem *nfc_base = NULL;
static void __iomem *edi_base = NULL;
static void __iomem *timer_base = NULL;
static void __iomem *uart0_base = NULL;
static void __iomem *uart_cfg_base = NULL;
static void __iomem *pic_base = NULL;
static void __iomem *vic_base = NULL;
static void __iomem *gpio_base = NULL;

#define TCC_PA_GPIO		0x14200000
#define TCC_PA_CKC		0x14000000
#define TCC_PA_PMU		0x14400000
#define TCC_PA_CPU		0x17110000
#define TCC_PA_CPU2ND	0x17210000
#define TCC_PA_UART0	0x16600000
#define TCC_PA_TIMER		0x14300000
#define TCC_PA_VIC		0x14100200
#define TCC_PA_PIC		0x14100000
#define TCC_PA_UARTPORTCFG	0x16684000
#define TCC_PA_DDRPHY		0x13540000
#define TCC_PA_GPU3D		0x10000000
#define TCC_PA_GPU2D		0x1A000000
#define HwNFC_BASE                              (0x16400000)
#define HwEDI_BASE                              (0x16430000)
/* temp define */

struct pwrctrl_gpio {
	int			gpio;
	enum of_gpio_flags	flag;
};

struct pm_power_gpios {
	int			count;
	struct pwrctrl_gpio	*pwrctrl;
};

static struct pm_power_gpios pwrctrl_gpios;

static void tcc_nfc_suspend(unsigned *nfc)
{
	void __iomem *reg = (void __iomem *)io_p2v(HwNFC_BASE);
	//void __iomem *reg = nfc_base;

	nfc[PM_NFC_CACYC] = pm_readl(reg + NFC_CACYC);
	nfc[PM_NFC_WRCYC] = pm_readl(reg + NFC_WRCYC);
	nfc[PM_NFC_RECYC] = pm_readl(reg + NFC_RECYC);
	nfc[PM_NFC_CTRL] = pm_readl(reg + NFC_CTRL);
	nfc[PM_NFC_IRQCFG] = pm_readl(reg + NFC_IRQCFG);
	nfc[PM_NFC_RFWBASE] = pm_readl(reg + NFC_RFWBASE);
}

static void tcc_nfc_resume(unsigned *nfc)
{
	void __iomem *reg = (void __iomem *)io_p2v(HwNFC_BASE);
	//void __iomem *reg = nfc_base;
	void __iomem *edi_reg = (void __iomem *)io_p2v(HwEDI_BASE);
	//void __iomem *edi_reg = edi_base;

	pm_writel((pm_readl(edi_reg+EDI_RDYCFG) & ~(0x000FFFFF)) | 0x32104, edi_reg+EDI_RDYCFG);
	pm_writel((pm_readl(edi_reg+EDI_CSNCFG0) & ~(0x0000FFFF)) | 0x8765, edi_reg+EDI_CSNCFG0);
	pm_writel((pm_readl(edi_reg+EDI_OENCFG) & ~(0x0000000F)) | 0x1, edi_reg+EDI_OENCFG);
	pm_writel((pm_readl(edi_reg+EDI_WENCFG) & ~(0x0000000F)) | 0x1, edi_reg+EDI_WENCFG);

	pm_writel(nfc[PM_NFC_CACYC], reg + NFC_CACYC);
	pm_writel(nfc[PM_NFC_WRCYC], reg + NFC_WRCYC);
	pm_writel(nfc[PM_NFC_RECYC], reg + NFC_RECYC);
	pm_writel(nfc[PM_NFC_CTRL], reg + NFC_CTRL);
	pm_writel(nfc[PM_NFC_IRQCFG], reg + NFC_IRQCFG);
	pm_writel(0xFFFFFFFF, reg + NFC_IRQ);
	pm_writel(nfc[PM_NFC_RFWBASE], reg + NFC_RFWBASE);
}

#ifdef CONFIG_PM_CONSOLE_NOT_SUSPEND
static void tcc_pm_uart_suspend(unsigned *uart, unsigned *portcfg)
{
	void __iomem *reg = (void __iomem *)io_p2v(TCC_PA_UART0);
	//void __iomem *reg = uart0_base;
	void __iomem *portcfg_reg = (void __iomem *)io_p2v(TCC_PA_UARTPORTCFG);
	//void __iomem *portcfg_reg = uart_cfg_base ;
	
	/* backup uart portcfg */
	portcfg[0] = pm_readl(portcfg_reg + 0x00);
	portcfg[1] = pm_readl(portcfg_reg + 0x04);

	/* backup uart0 registers */
	uart[IMSC] 	= pm_readl(reg+0x4*IMSC);
	pm_writel(0x0, reg+0x4*IMSC);	/* disable all interrupts */
	uart[FR] 	= pm_readl(reg+0x4*FR);
	uart[ILPR] 	= pm_readl(reg+0x4*ILPR);
	uart[IBRD] 	= pm_readl(reg+0x4*IBRD);
	uart[FBRD] 	= pm_readl(reg+0x4*FBRD);
	uart[LCRH] 	= pm_readl(reg+0x4*LCRH);
	uart[CR] 	= pm_readl(reg+0x4*CR);
	uart[IFLS] 	= pm_readl(reg+0x4*IFLS);
	uart[DMACR] = pm_readl(reg+0x4*DMACR);
//	pm_writel(pm_readl(reg+0x4*IER) & ~(1<<2), reg+0x4*IER);	/* disable interrupt : ELSI */
}

static void tcc_pm_uart_resume(unsigned *uart, unsigned *portcfg)
{
	void __iomem *reg = (void __iomem *)io_p2v(TCC_PA_UART0);
	//void __iomem *reg = uart0_base;
	void __iomem *portcfg_reg = (void __iomem *)io_p2v(TCC_PA_UARTPORTCFG);
	//void __iomem *portcfg_reg = uart_cfg_base;
 
	/* restore uart portcfg */
	pm_writel(portcfg[0], portcfg_reg + 0x00);
	pm_writel(portcfg[1], portcfg_reg + 0x04);

	/* restore uart0 registers */
	pm_writel(0x0, reg+0x4*IMSC);	/* disable all interrupts */
	pm_writel(uart[FR], reg + 0x4*FR);
	pm_writel(uart[ILPR], reg + 0x4*ILPR);
	pm_writel(uart[IBRD], reg + 0x4*IBRD);
	pm_writel(uart[FBRD], reg + 0x4*FBRD);
	pm_writel(uart[LCRH], reg + 0x4*LCRH);
	pm_writel(uart[CR], reg + 0x4*CR);
	pm_writel(uart[IFLS], reg + 0x4*IFLS);
	pm_writel(uart[DMACR], reg + 0x4*DMACR);
	pm_writel(uart[IMSC], reg + 0x4*IMSC);

	printk("\n======== wakeup source ========\n");
	printk("wakeup0_reg:0x%08x\n", (unsigned int)pmu_wakeup_param(PMU_WAKEUP_WUSTS0));
	printk("wakeup1_reg:0x%08x\n\n\n", (unsigned int)pmu_wakeup_param(PMU_WAKEUP_WUSTS1));
}
#endif

static void tcc_pm_register_backup(void)
{
	void __iomem *sram_reg = (void __iomem *)sram_p2v(SRAM_BOOT_ADDR);
	void __iomem *ckc_reg = (void __iomem *)io_p2v(TCC_PA_CKC);
	void __iomem *ddrphy_reg = (void __iomem *)io_p2v(TCC_PA_DDRPHY);
	int i;
	unsigned a, b;

	/* set CNTVOFF (counter offset) to CNTVCT (curr. counter) */
	isb();
	asm volatile("mrrc p15, 1, %0, %1, c14" : "=r" (a), "=r" (b));
	pm_writel(~b, (void __iomem *)sram_p2v(ARM_TIMER_BACKUP_ADDR));
	pm_writel(~a, (void __iomem *)sram_p2v(ARM_TIMER_BACKUP_ADDR+0x4));

	cpu_pm_enter();
	cpu_cluster_pm_enter();

#if 1
	memcpy(RegRepo->timer, (unsigned *)io_p2v(TCC_PA_TIMER), PM_TIMER_SIZE);
	memcpy(RegRepo->vic, (unsigned *)io_p2v(TCC_PA_VIC), PM_VIC_SIZE);
	memcpy(RegRepo->pic, (unsigned *)io_p2v(TCC_PA_PIC), PM_PIC_SIZE);
#endif

	/* GPIO Reg. Back-up */
	memcpy((void*)sram_p2v(GPIO_REPOSITORY_ADDR), (void *)io_p2v(TCC_PA_GPIO), GPIO_REPOSITORY_SIZE);
	//memcpy((void*)sram_p2v(GPIO_REPOSITORY_ADDR), (void *)gpio_base, GPIO_REPOSITORY_SIZE);

	/* backup pll,cpu,mem clocks for restore before exit sdram self-refresh */
	pm_writel(pm_readl(ckc_reg + CKC_PLL3), (void *)sram_p2v(CPU_DATA_REPOSITORY_ADDR+0x10));
	pm_writel(pm_readl(ckc_reg + CKC_MEM), (void *)sram_p2v(CPU_DATA_REPOSITORY_ADDR+0x14));
	pm_writel(pm_readl(ddrphy_reg + DDR_PLL), (void *)sram_p2v(CPU_DATA_REPOSITORY_ADDR+0x18));
	pm_writel(pm_readl(ddrphy_reg + DDR_CKC), (void *)sram_p2v(CPU_DATA_REPOSITORY_ADDR+0x1c));

	tcc_ckc_backup_regs(1);

	/* clear secondary cpu's config values. */
	for (i=0 ; i<4 ; i++)
		pm_writel(0x0, sram_reg + SEC_VALID + (i*0x4));

#ifdef CONFIG_SHUTDOWN_MODE
	{
		unsigned int mpidr = read_cpuid_mpidr();
		unsigned int cpu = ((mpidr>>8)&0xF)*2 + (mpidr&0xF);

		/* set main(resume) cpu's info. */
		pm_writel(1, sram_reg + SEC_VALID + (cpu*0x4));
		pm_writel(SRAM_RESUME_FUNC_ADDR, sram_reg + SEC_START + (cpu*0x4));
	}
#endif

	if (pwrctrl_gpios.count) {
		for (i=0 ; i<pwrctrl_gpios.count; i++)
			gpio_direction_output(pwrctrl_gpios.pwrctrl[i].gpio, pwrctrl_gpios.pwrctrl[i].flag);
	}

	/* backup cpu offset */
	asm volatile ( "mrc p15, 0, %0, c13, c0, 4" : "=r" (RegRepo->TPIDRPRW));

#if defined(CONFIG_ARM_TRUSTZONE) && defined(CONFIG_SHUTDOWN_MODE)
	_tz_smc(SMC_CMD_SHUTDOWN, 0, 0, 0);
#endif
}

static void tcc_pm_register_restore(void)
{
	/* restore cpu offset */
	asm volatile ( "mcr p15, 0, %0, c13, c0, 4" :: "r" (RegRepo->TPIDRPRW));

	tcc_ckc_restore_regs();
	tcc_pm_nop_delay(100);

	//__cpu_early_init();

 	/* SMU & PMU */
#if 1
	memcpy((unsigned *)io_p2v(TCC_PA_PIC), RegRepo->pic, PM_PIC_SIZE);
	memcpy((unsigned *)io_p2v(TCC_PA_VIC), RegRepo->vic, PM_VIC_SIZE);
	memcpy((unsigned *)io_p2v(TCC_PA_TIMER), RegRepo->timer, PM_TIMER_SIZE);
#endif

	cpu_cluster_pm_exit();
	cpu_pm_exit();
}

void tcc_resume_from_sram(void)
{
	void __iomem *pmu_reg = (void __iomem *)TCC_PA_PMU;
	//void __iomem *pmu_reg = pmu_base;
	void __iomem *ckc_reg = (void __iomem *)TCC_PA_CKC;
	//void __iomem *ckc_reg = ckc_base;
	void __iomem	*cpu_reg = (void __iomem *)TCC_PA_CPU;
	//void __iomem	*cpu2nd_reg = (void __iomem *)TCC_PA_CPU2ND;
	void __iomem *ddrphy_reg = (void __iomem *)TCC_PA_DDRPHY;
	void __iomem	*gpu3d_reg = (void __iomem *)TCC_PA_GPU3D;
	void __iomem	*gpu2d_reg = (void __iomem *)TCC_PA_GPU2D;
	unsigned *src, *dest;
	unsigned int cnt, mode;

#ifdef CONFIG_SHUTDOWN_MODE
	FuncPtr pSdramFunc = (FuncPtr)SDRAM_INIT_FUNC_ADDR;
	mode = 1;
#else
	FuncPtr pSdramFunc = (FuncPtr)(SDRAM_SELF_REF_EXIT_ADDR);
	mode = 2;
#endif
	//asm volatile ("resume_debug:");
	//asm volatile ("b resume_debug");
	pmu_wakeup_param_phys(PMU_WAKEUP_WUSTS0) = pm_readl(pmu_reg + PMU_WKSTS0);
	pmu_wakeup_param_phys(PMU_WAKEUP_WUSTS1) = pm_readl(pmu_reg + PMU_WKSTS1);

#if 1
	/* GPIO restore */
	src = (unsigned*)GPIO_REPOSITORY_ADDR;
	dest = (unsigned*)TCC_PA_GPIO;
	for(cnt=0 ; cnt<(GPIO_REPOSITORY_SIZE/4) ; cnt++)
		dest[cnt] = src[cnt];

#endif
	/* clear & disable wake-up */
	pm_writel(0x00000000, pmu_reg + PMU_WKUP0);
	pm_writel(0x00000000, pmu_reg + PMU_WKUP1);
	pm_writel(0xFFFFFFFF, pmu_reg + PMU_WKCLR0);
	pm_writel(0xFFFFFFFF, pmu_reg + PMU_WKCLR1);

	/* SSTL & IO Retention Disable */
	while (!(pm_readl(pmu_reg + PMU_TOP_PWRCTRL) & (1<<1)))	/* DMC0 */
		pm_writel(pm_readl(pmu_reg + PMU_TOP_PWRCTRL) | (1<<1), pmu_reg + PMU_TOP_PWRCTRL);
	while (!(pm_readl(pmu_reg + PMU_TOP_PWRCTRL) & (1<<2)))	/* DMC1 */
		pm_writel(pm_readl(pmu_reg + PMU_TOP_PWRCTRL) | (1<<2), pmu_reg + PMU_TOP_PWRCTRL);
	while (!(pm_readl(pmu_reg + PMU_TOP_PWRCTRL) & (1<<0)))
		pm_writel(pm_readl(pmu_reg + PMU_TOP_PWRCTRL) | (1<<0), pmu_reg + PMU_TOP_PWRCTRL);

	/* set cpu clock */
#if 1
	pm_writel(0x86813182, ckc_reg+CKC_PLL2);		/* CPU0 (Cortex-A7SP) */
	while((pm_readl(ckc_reg+CKC_PLL2)&(1<<23)) == 0);	/* check lock status */
	pm_writel(0x005E3E42, ckc_reg+CKC_CPU0);		/* CPU0 (Cortex-A7SP) */
#endif

	pm_writel(0x86815146, cpu_reg+CPU_PLLPMS);		/* CPU0 (Cortex-A53MP) */
	while((pm_readl(cpu_reg+CPU_PLLPMS)&(1<<23)) == 0);	/* check lock status */
	pm_writel((pm_readl(cpu_reg + CPU_CLK) | (1<<0))  , cpu_reg + CPU_CLK);

#if 0
	pm_writel(0x86813203, cpu2nd_reg+CPU_PLLPMS);		/* CPU0 (Cortex-A53MP) */
	while((pm_readl(cpu2nd_reg+CPU_PLLPMS)&(1<<23)) == 0);	/* check lock status */
	pm_writel((pm_readl(cpu2nd_reg + CPU_CLK) | (1<<0))  , cpu2nd_reg + CPU_CLK);
#endif

#if 0
	pm_writel(0x86823E83, gpu3d_reg+GPU3D_PLL);		/* CPU0 (Cortex-A53MP) */
	while((pm_readl(gpu3d_reg+GPU3D_PLL)&(1<<23)) == 0);	/* check lock status */
	//pm_writel((pm_readl(gpu3d_reg+GPU3D_CLK) | (1<<0))  , cpu_reg + CPU_CLK);
	
	pm_writel(0x86813203, gpu2d_reg+GPU2D_PLL);		/* CPU0 (Cortex-A53MP) */
	while((pm_readl(gpu2d_reg+GPU2D_PLL)&(1<<23)) == 0);	/* check lock status */
	//pm_writel((pm_readl(gpu2d_reg+GPU2D_CLK) | (1<<0))  , cpu_reg + CPU_CLK);
	//
#endif
	/* restore MEM clock */
	//pm_writel(0x00001f00, ckc_reg + CKC_MEM);	// MEM: XIN/2
	pm_writel(pm_readl((void *)(CPU_DATA_REPOSITORY_ADDR+0x18)), ddrphy_reg+DDR_PLL);
	pm_writel(pm_readl((void *)(CPU_DATA_REPOSITORY_ADDR+0x10)), ckc_reg+CKC_PLL3);
	
	tcc_pm_nop_delay(0x1000);	
	pm_writel(pm_readl((void *)(CPU_DATA_REPOSITORY_ADDR+0x14)), ckc_reg+CKC_MEM);
	pm_writel(pm_readl((void *)(CPU_DATA_REPOSITORY_ADDR+0x1C)), ddrphy_reg+DDR_CKC);

	/* shudown: sdram re-init, sleep: self-refreah exit */
	tcc_pm_nop_delay(0x1000);
	pSdramFunc();
	tcc_pm_nop_delay(0x1000);

	//asm volatile ("resume_debug:");
	//asm volatile ("b resume_debug");

#ifdef CONFIG_ARM_TRUSTZONE
	asm volatile (
		"stmfd	sp!, {r0, lr} \n"
		"mov	r0, %1 \n"
		"mov	lr, pc \n"
		"mov	pc, %0 \n"
		"ldmfd	sp!, {r0, lr} \n"
		 :: "r" (TZ_SECUREOS_BASE), "r" (mode)
	);
#endif

	/* call tcc_cpu_resume() */
	asm volatile (
		"ldr	r0, [%0] \n"
		"mov	pc, r0 \n"
		 :: "r" (CPU_DATA_REPOSITORY_ADDR+0x04)
	);
	while(1);
}

void tcc_suspend_to_sram(unsigned mode)
{
	void __iomem	*pmu_reg = (void __iomem *)TCC_PA_PMU;
	//void __iomem	*pmu_reg = pmu_base;
	void __iomem	*ckc_reg = (void __iomem *)TCC_PA_CKC;
	void __iomem	*cpu_reg = (void __iomem *)TCC_PA_CPU;
	//void __iomem	*cpu2nd_reg = (void __iomem *)TCC_PA_CPU2ND;
	void __iomem	*ddrphy_reg = (void __iomem *)TCC_PA_DDRPHY;
	void __iomem	*gpu3d_reg = (void __iomem *)TCC_PA_GPU3D;
	void __iomem	*gpu2d_reg = (void __iomem *)TCC_PA_GPU2D;
	//void __iomem	*ckc_reg = ckc_base;
	FuncPtr1	sdram_self_ref_enter = (FuncPtr1)SDRAM_SELF_REF_ENTER_ADDR;

	sdram_self_ref_enter(mode);
	tcc_pm_nop_delay(0x1000);

	/* Clock <- XIN */
	pm_writel((pm_readl(ckc_reg + CKC_CPU0) & ~(0xf))|5  , ckc_reg + CKC_CPU0);
	pm_writel((pm_readl(cpu_reg + CPU_CLK) & ~(0x3))  , cpu_reg + CPU_CLK);
	//pm_writel((pm_readl(cpu2nd_reg + CPU_CLK) & ~(0x3))  , cpu2nd_reg + CPU_CLK);
	pm_writel((pm_readl(ddrphy_reg + DDR_CKC) & ~(0x2))  , ddrphy_reg + DDR_CKC);
	pm_writel((pm_readl(gpu3d_reg + GPU3D_CLK) & ~(0x2))  , gpu3d_reg + GPU3D_CLK);
	pm_writel((pm_readl(gpu2d_reg + GPU2D_CLK) & ~(0x2))  , gpu2d_reg + GPU2D_CLK);
	pm_writel((pm_readl(gpu2d_reg + GPU2D_CORE_CLK) & ~(0x2))  , gpu2d_reg + GPU2D_CORE_CLK);
	pm_writel(0x00400025, ckc_reg + CKC_MEM);
	pm_writel(0x00400025, ckc_reg + CKC_IO);
	pm_writel(0x00400025, ckc_reg + CKC_HSIO);
	pm_writel(0x00400025, ckc_reg + CKC_SMU);
	pm_writel(0x00400025, ckc_reg + CKC_CM);

	/* PLL <- OFF */
	pm_writel(pm_readl(cpu_reg + CPU_PLLPMS) & ~(1<<31), cpu_reg + CPU_PLLPMS);
	//pm_writel(pm_readl(cpu2nd_reg + CPU_PLLPMS) & ~(1<<31), cpu2nd_reg + CPU_PLLPMS);
	pm_writel(pm_readl(ddrphy_reg + DDR_PLL) & ~(1<<31), ddrphy_reg + DDR_PLL);
	pm_writel(pm_readl(gpu3d_reg + GPU3D_PLL) & ~(1<<31), gpu3d_reg + GPU3D_PLL);
	pm_writel(pm_readl(gpu2d_reg + GPU2D_PLL) & ~(1<<31), gpu2d_reg + GPU2D_PLL);
	pm_writel(pm_readl(ckc_reg + CKC_PLL0) & ~(1<<31), ckc_reg + CKC_PLL0);
	pm_writel(pm_readl(ckc_reg + CKC_PLL1) & ~(1<<31), ckc_reg + CKC_PLL1);
	pm_writel(pm_readl(ckc_reg + CKC_PLL2) & ~(1<<31), ckc_reg + CKC_PLL2);
	pm_writel(pm_readl(ckc_reg + CKC_PLL3) & ~(1<<31), ckc_reg + CKC_PLL3);
	pm_writel(pm_readl(ckc_reg + CKC_PLL4) & ~(1<<31), ckc_reg + CKC_PLL4);
	
	/* SSTL & IO Retention */
	while (pm_readl(pmu_reg + PMU_TOP_PWRCTRL) & (1<<1))	/* DMC0 */
		pm_writel(pm_readl(pmu_reg + PMU_TOP_PWRCTRL) & ~(1<<1), pmu_reg + PMU_TOP_PWRCTRL);
	while (pm_readl(pmu_reg + PMU_TOP_PWRCTRL) & (1<<2))	/* DMC1 */
		pm_writel(pm_readl(pmu_reg + PMU_TOP_PWRCTRL) & ~(1<<2), pmu_reg + PMU_TOP_PWRCTRL);
	while (pm_readl(pmu_reg + PMU_TOP_PWRCTRL) & (1<<0))
		pm_writel(pm_readl(pmu_reg + PMU_TOP_PWRCTRL) & ~(1<<0), pmu_reg + PMU_TOP_PWRCTRL);

	/*set wake-up trigger mode : edge */
	pm_writel(0xFFFFFFFF, pmu_reg + PMU_WKMOD0);
	pm_writel(0xFFFFFFFF, pmu_reg + PMU_WKMOD1);
	pm_writel(0x00000000, pmu_reg + PMU_WKUP0);
	pm_writel(0x00000000, pmu_reg + PMU_WKUP1);

	pm_writel(pmu_wakeup_param_phys(PMU_WAKEUP_WKPOL0), pmu_reg + PMU_WKPOL0);
	pm_writel(pmu_wakeup_param_phys(PMU_WAKEUP_WKUP0), pmu_reg + PMU_WKUP0);
	pm_writel(pmu_wakeup_param_phys(PMU_WAKEUP_WKPOL1), pmu_reg + PMU_WKPOL1);
	pm_writel(pmu_wakeup_param_phys(PMU_WAKEUP_WKUP1), pmu_reg + PMU_WKUP1);

	/* RTC Alarm Wake Up */
	pm_writel(pm_readl(pmu_reg + PMU_WKPOL0) & ~(1<<3), pmu_reg + PMU_WKPOL0);
	pm_writel(pm_readl(pmu_reg + PMU_WKUP0) | (1<<3), pmu_reg + PMU_WKUP0);

	/* Remocon Wake Up */
	pm_writel(pm_readl(pmu_reg + PMU_WKPOL1) & ~(1<<31), pmu_reg + PMU_WKPOL1);
	pm_writel(pm_readl(pmu_reg + PMU_WKUP1) | (1<<31), pmu_reg + PMU_WKUP1);

#ifdef CONFIG_SHUTDOWN_MODE
	while(1) {
		unsigned cnt;
		pm_writel(0x1, pmu_reg + PMU_PWRDN_TOP);
		tcc_pm_nop_delay(100);

		cnt = 0;
		if(cnt++ > 100){
			/* reboot */
			if (pm_readl(pmu_reg + PMU_WKSTS0) || pm_readl(pmu_reg + PMU_WKSTS1)) {
				pm_writel((1<<31)|(1<<0), pmu_reg + PMU_WDTCTRL);
				while(1);
			}
		}
	}
#else
	{
		FuncPtr pWakeUpFunc = (FuncPtr)SRAM_RESUME_FUNC_ADDR;
//		void __iomem *memcfg_reg = (void __iomem *)TCC_PA_MEMBUSCFG;

		/* Uniquify, Phy, SDRAM controller reset enable */
//		pm_writel(pm_readl(memcfg_reg + 0x4) & ~0x00700016, memcfg_reg + 0x4);

		// -------------------------------------------------------------------------
		// Enter Sleep !!
		////////////////////////////////////////////////////////////////////////////
		pm_writel(pm_readl(pmu_reg + PMU_PWRDN_XIN) | 0x1, pmu_reg + PMU_PWRDN_XIN);
		////////////////////////////////////////////////////////////////////////////

		/* Uniquify, Phy, SDRAM controller reset disable */
//		pm_writel(pm_readl(memcfg_reg + 0x4) | 0x00700016, memcfg_reg + 0x4);

		pWakeUpFunc();
	}
#endif
}

static int powerkey_wakeup_idx;
static int wakeup_by_powerkey(void)
{
	if (powerkey_wakeup_idx > 31) {
		if (pmu_wakeup_param(PMU_WAKEUP_WUSTS1) & (1<<(powerkey_wakeup_idx-32)))
			return 1;
	} else {
		if (pmu_wakeup_param(PMU_WAKEUP_WUSTS0) & (1<<(powerkey_wakeup_idx)))
			return 1;
	}
	return 0;
}

static void suspend_board_config(void)
{
	struct device_node *np;
	int gpio, i, wakeup_act;

	np = of_find_compatible_node(NULL, NULL, "telechips,pm");
	if (!np)
		return;

	ckc_base = of_iomap(np, 0);
	BUG_ON(!ckc_base);
	pmu_base = of_iomap(np, 1);
	BUG_ON(!pmu_base);
	nfc_base = of_iomap(np, 2);
	BUG_ON(!nfc_base);
	edi_base = of_iomap(np, 3);
	BUG_ON(!edi_base);
	timer_base = of_iomap(np, 4);
	BUG_ON(!timer_base);
	uart0_base= of_iomap(np, 5);
	BUG_ON(!uart0_base);
	uart_cfg_base = of_iomap(np, 6);
	BUG_ON(!uart_cfg_base);
	pic_base = of_iomap(np, 7);
	BUG_ON(!pic_base);
	vic_base = of_iomap(np, 8);
	BUG_ON(!vic_base);
	gpio_base = of_iomap(np, 9);
	BUG_ON(!gpio_base);

	/* set gpio wakeup source */
	if (of_property_read_u32_index(np, "gpio-key,wakeup",0, &gpio)) {
		        printk("wake up power key regist fail.\n");
	}

	if (of_property_read_u32_index(np, "gpio-key,wakeup",1, &wakeup_act)) {
		        printk("wake up power key regist fail.\n");
	}

	if (gpio_is_valid(gpio))
		powerkey_wakeup_idx = set_gpio_to_wakeup(gpio, WAKEUP_EN, wakeup_act);

	/* get power-ctrl gpios */
	pwrctrl_gpios.count = of_gpio_named_count(np, "gpio-powers");
	if (pwrctrl_gpios.count > 0) {
		pwrctrl_gpios.pwrctrl = kzalloc(sizeof(struct pwrctrl_gpio)*(pwrctrl_gpios.count), GFP_KERNEL);

		for (i=0 ; i < pwrctrl_gpios.count ; i++)
			pwrctrl_gpios.pwrctrl[i].gpio = of_get_named_gpio_flags(np, "gpio-powers", i, &(pwrctrl_gpios.pwrctrl[i].flag));
	}
	else
		pwrctrl_gpios.count = 0;
}


static struct tcc_suspend_ops suspend_ops = {
	.reg_backup = tcc_pm_register_backup,
	.reg_restore = tcc_pm_register_restore,
#ifdef CONFIG_PM_CONSOLE_NOT_SUSPEND
	.uart_suspend = tcc_pm_uart_suspend,
	.uart_resume = tcc_pm_uart_resume,
#endif
	.nfc_suspend = tcc_nfc_suspend,
	.nfc_resume = tcc_nfc_resume,
	.wakeup_by_powerkey = wakeup_by_powerkey,
	.remap = NULL,
};

static int __init tcc898x_suspend_init(void)
{
	tcc_sram_init();
	suspend_board_config();
	tcc_suspend_set_ops(&suspend_ops);

//+[TCCQB] snapshot functions...
#ifdef CONFIG_SNAPSHOT_BOOT
	tcc_snapshot_set_ops(&suspend_ops);
#endif
//-[TCCQB]
//
	return 0;
}
__initcall(tcc898x_suspend_init);

