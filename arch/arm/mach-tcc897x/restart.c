#include <linux/kernel.h>
#include <mach/iomap.h>
#include <asm/io.h>
#ifdef CONFIG_DAUDIO_KK
#include <mach/gpio.h>
#endif

#define	rst_readl	__raw_readl
#define	rst_writel	__raw_writel

#define PMU_WDTCTRL	0x08
#define PMU_USSTATUS	0x1C

extern uint32_t restart_reason;

void tcc897x_restart(char mode, const char *cmd)
{
	void __iomem *pmu_reg = (void __iomem *)io_p2v(TCC_PA_PMU);
	void __iomem *ckc_reg = (void __iomem *)io_p2v(TCC_PA_CKC);

	if (restart_reason != 0x776655AA)
		rst_writel(restart_reason, pmu_reg+PMU_USSTATUS);

	rst_writel(0x28000000, ckc_reg+0xC8);	/* watchdog pclk */

#ifdef CONFIG_DAUDIO_KK
	tcc_gpio_config(TCC_GPG(5), GPIO_FN0|GPIO_OUTPUT|GPIO_HIGH);
	gpio_direction_output(TCC_GPG(5), 1);
#endif

	while (1) {
		writel(0, pmu_reg+PMU_WDTCTRL);
 	}
}

//+[TCCQB] To read USSTATUS Register ( LK Boot time )
u32 read_usstatus(void)
{
	void __iomem *pmu_reg = (void __iomem *)io_p2v(TCC_PA_PMU);

	return rst_readl(pmu_reg+PMU_USSTATUS);
}
EXPORT_SYMBOL(read_usstatus);
//-[TCCQB]
//
