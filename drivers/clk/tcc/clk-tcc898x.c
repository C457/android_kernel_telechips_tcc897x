/****************************************************************************
 * Copyright (C) 2016 Telechips Inc.
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

#include <linux/clocksource.h>
#include <linux/of_address.h>
#include <linux/clk/tcc.h>
#include <linux/clk-provider.h>
#include <linux/irqflags.h>
#include <linux/slab.h>

#define TCC898X_CKC_DRIVER
#include "clk-tcc898x.h"
//#if !defined(CONFIG_PLATFORM_AVN)
#define HDMIA_SRC_SHOULD_BE_SAME_WITH_AUDIO
//#endif

/**********************************
 *  Pre Defines
 **********************************/
#define ckc_writel	__raw_writel
#define ckc_readl	__raw_readl

#define MAX_TCC_PLL	5
#define MAX_CLK_SRC	(MAX_TCC_PLL*2 + 2)	// XIN, XTIN
#define DITHERED_PLL_CH	4

static void __iomem	*ckc_base = NULL;
static void __iomem	*pmu_base = NULL;

static void __iomem	*membus_cfg_base = NULL;
static void __iomem	*ddibus_cfg_base = NULL;
static void __iomem	*iobus_cfg_base = NULL;
static void __iomem	*iobus_cfg_base1 = NULL;
static void __iomem	*vpubus_cfg_base = NULL;
static void __iomem	*hsiobus_cfg_base = NULL;
static void __iomem	*cpubus_cfg_base = NULL;

static void __iomem	*cpu0_base = NULL;
static void __iomem	*cpu1_base = NULL;
static void __iomem	*gpu_3d_base = NULL;
static void __iomem	*gpu_2d_base = NULL;
static void __iomem	*mem_ckc_base = NULL;

static unsigned int	stClockSource[MAX_CLK_SRC];
static bool		audio_pll_used = false;
static u32		audio_pll_ch = 0;

#ifdef HDMIA_SRC_SHOULD_BE_SAME_WITH_AUDIO
static unsigned int	stHDMIASrc = 0xFFFFFFFF;
#endif

static struct tcc_ckc_ops tcc898x_ops;
static inline int tcc_find_audio_pclk(tPCLKCTRL *PCLKCTRL);
static inline void tcc_ckc_reset_clock_source(int id);
static int tcc_dckc_store(unsigned int target) ;

static tDCKC dckc_backup[DCKC_MAX];

/**********************************
 *  CKC Register Control
 **********************************/
#define CKC2_CLKCTRL	0x000
#define CKC2_PLL	0x008
#define CKC2_CLKDIVC	0x014

#define CKC_CLKCTRL	0x000
#define CKC_PLL		0x040
#define CKC_CLKDIVC	0x0A0
#define CKC_SWRESET	0x0B4
#define CKC_PCLKCTRL	0x0C8

/* PLL */
#define PLL_MAX_RATE		(2500*1000*1000UL)
#define PLL_MIN_RATE		(40*1000*1000UL)
#define PLL_VCO_MAX		(2500*1000*1000UL)
#define PLL_VCO_MIN		(1250*1000*1000UL)
#define PLL_P_MAX		12	// 63	FREF = FIN/p  (2MHz ~ 12MHz)
#define PLL_P_MIN		2	// 1	FREF = FIN/p  (2MHz ~ 12MHz)
#define PLL_P_SHIFT		0
#define PLL_P_MASK		0x3F
#define PLL_M_MAX		1023
#define PLL_M_MIN		64
#define PLL_M_SHIFT		6
#define PLL_M_MASK		0x3FF
#define PLL_S_MAX		5
#define PLL_S_MIN		0
#define PLL_S_SHIFT		16
#define PLL_S_MASK		0x7
#define PLL_SRC_SHIFT		19
#define PLL_SRC_MASK		0x3
#define PLL_BYPASS_SHIFT	21
#define PLL_LOCKST_SHIFT	23
#define PLL_CHGPUMP_SHIFT	24
#define PLL_CHGPUMP_MASK	0x3
#define PLL_LOCKEN_SHIFT	26
#define PLL_RSEL_SHIFT		27
#define PLL_RSEL_MASK		0xF
#define PLL_EN_SHIFT		31
#define tcc_pll_write(reg,en,p,m,s,src) { \
	if (en) { \
		volatile unsigned int i; \
		ckc_writel(0 \
			|(1<<PLL_LOCKEN_SHIFT)|(2<<PLL_CHGPUMP_SHIFT) \
			|((src&PLL_SRC_MASK)<<PLL_SRC_SHIFT)|((s&PLL_S_MASK)<<PLL_S_SHIFT) \
			|((m&PLL_M_MASK)<<PLL_M_SHIFT)|((p&PLL_P_MASK)<<PLL_P_SHIFT), reg); \
		/* need to delay at least 1us. */ \
		for (i=100 ; i ; i--);  /* if cpu clokc is 1HGz then loop 100. */ \
		ckc_writel(ckc_readl(reg) | ((en&1)<<PLL_EN_SHIFT), reg); \
		while((ckc_readl(reg)&(1<<PLL_LOCKST_SHIFT))==0); \
	} else \
		ckc_writel(ckc_readl(reg) & ~(1<<PLL_EN_SHIFT), reg); \
}
/* for pll of cpu0/cpu1/gpu3d/gpu2d/mem */
#define tcc_pll2_write(reg,en,p,m,s) { \
	if (en) { \
		volatile unsigned int i; \
		ckc_writel(0 \
			|(1<<PLL_LOCKEN_SHIFT)|(2<<PLL_CHGPUMP_SHIFT) \
			|((s&PLL_S_MASK)<<PLL_S_SHIFT) \
			|((m&PLL_M_MASK)<<PLL_M_SHIFT)|((p&PLL_P_MASK)<<PLL_P_SHIFT), reg); \
		/* need to delay at least 1us. */ \
		if (reg == cpu0_base+CKC2_PLL) \
			; /* no need to delay. cpu clock is XIN(24MHz). */ \
		else \
			for (i=100 ; i ; i--);  /* if cpu clokc is 1HGz then loop 100. */ \
		ckc_writel(ckc_readl(reg) | ((en&1)<<PLL_EN_SHIFT), reg); \
		while((ckc_readl(reg)&(1<<PLL_LOCKST_SHIFT))==0); \
	} else { \
		ckc_writel(ckc_readl(reg) & ~(1<<PLL_EN_SHIFT), reg); \
	} \
}
enum{ /* PLL Clock Source */
	PLLSRC_XIN=0,
	PLLSRC_HDMIXI,
	PLLSRC_EXTCLK0,
	PLLSRC_EXTCLK1,
	PLLSRC_MAX
};

/* Dithered PLL */
#define DPLL_MAX_RATE		(2500*1000*1000UL)
#define DPLL_MIN_RATE		(40*1000*1000UL)
#define DPLL_VCO_MAX		(2500*1000*1000UL)
#define DPLL_VCO_MIN		(1250*1000*1000UL)
#define DPLL_P_MAX		12	// 63	FREF = FIN/p  (2MHz ~ 12MHz)
#define DPLL_P_MIN		2	// 1	FREF = FIN/p  (2MHz ~ 12MHz)
#define DPLL_P_SHIFT		0
#define DPLL_P_MASK		0x3F
#define DPLL_M_MAX		511
#define DPLL_M_MIN		16
#define DPLL_M_SHIFT		6
#define DPLL_M_MASK		0x1FF
#define DPLL_S_MAX		5
#define DPLL_S_MIN		0
#define DPLL_S_SHIFT		15
#define DPLL_S_MASK		0x7
#define DPLL_SRC_SHIFT		19
#define DPLL_SRC_MASK		0x3
#define DPLL_BYPASS_SHIFT	21
#define DPLL_CHGPUMP_SHIFT	24
#define DPLL_CHGPUMP_MASK	0x3
#define DPLL_SSCG_EN_SHIFT	28
#define DPLL_SEL_PF_SHIFT	29
#define DPLL_SEL_PF_MASK	0x3
#define DPLL_EN_SHIFT		31
#define tcc_dpll_write(reg,en,p,m,s,src) { \
	if (en) { \
		ckc_writel((2<<DPLL_CHGPUMP_SHIFT) \
			|((src&DPLL_SRC_MASK)<<DPLL_SRC_SHIFT)|((s&DPLL_S_MASK)<<DPLL_S_SHIFT) \
			|((m&DPLL_M_MASK)<<DPLL_M_SHIFT)|((p&DPLL_P_MASK)<<DPLL_P_SHIFT), reg); \
		ckc_writel(ckc_readl(reg) | ((en&1)<<DPLL_EN_SHIFT), reg); \
	} else \
		ckc_writel(ckc_readl(reg) & ~(1<<DPLL_EN_SHIFT), reg); \
}

/* CLKCTRL2 - CPU0/CPU1/GPU3D/GPU2D/MEM */
#define CLKCTRL2_SEL_MIN	0
#define CLKCTRL2_SEL_MAX	3
#define CLKCTRL2_SEL_SHIFT	0
#define CLKCTRL2_SEL_MASK	0x3
#define CLKCTRL2_XTIN_SEL_SHIFT	2
#define CLKCTRL2_SYNRQ_SHIFT	30
#define CLKCTRL2_CHGRQ_SHIFT	31
#define tcc_clkctrl2_write(reg,sel) { \
	ckc_writel((sel&CLKCTRL2_SEL_MASK)<<CLKCTRL2_SEL_SHIFT, reg); \
	while(ckc_readl(reg) & (1<<CLKCTRL2_CHGRQ_SHIFT)); \
}
enum { /* CLKCTRL2 SEL */
	CLKCTRL2_SEL_XIN=0,
	CLKCTRL2_SEL_PLL,
	CLKCTRL2_SEL_PLLDIV,
	CLKCTRL2_SEL_XTIN,
};

/* CLKCTRL - Other BUS */
#define CLKCTRL_SEL_MIN		0
#define CLKCTRL_SEL_MAX		15
#define CLKCTRL_SEL_SHIFT	0
#define CLKCTRL_SEL_MASK	0xF
#define CLKCTRL_CONFIG_MIN	1
#define CLKCTRL_CONFIG_MAX	15
#define CLKCTRL_CONFIG_SHIFT	5
#define CLKCTRL_CONFIG_MASK	0xF
#define CLKCTRL_EN_SHIFT	22
#define CLKCTRL_CFGRQ_SHIFT	29
#define CLKCTRL_SYNRQ_SHIFT	30
#define CLKCTRL_CHGRQ_SHIFT	31
#define tcc_clkctrl_write(reg,en,config,sel) { \
	ckc_writel((ckc_readl(reg)&(~(CLKCTRL_SEL_MASK<<CLKCTRL_SEL_SHIFT))) \
			|((sel&CLKCTRL_SEL_MASK)<<CLKCTRL_SEL_SHIFT), reg); \
	while(ckc_readl(reg) & (1<<CLKCTRL_CHGRQ_SHIFT)); \
	ckc_writel((ckc_readl(reg)&(~(CLKCTRL_CONFIG_MASK<<CLKCTRL_CONFIG_SHIFT))) \
			|((config&CLKCTRL_CONFIG_MASK)<<CLKCTRL_CONFIG_SHIFT), reg); \
	while(ckc_readl(reg) & (1<<CLKCTRL_CFGRQ_SHIFT)); \
	ckc_writel((ckc_readl(reg)&(~(1<<CLKCTRL_EN_SHIFT))) \
			|((en&1)<<CLKCTRL_EN_SHIFT), reg); \
	while(ckc_readl(reg) & (1<<CLKCTRL_CFGRQ_SHIFT)); \
}
enum { /* CLKCTRL SEL */
	CLKCTRL_SEL_PLL0=0,
	CLKCTRL_SEL_PLL1,
	CLKCTRL_SEL_PLL2,
	CLKCTRL_SEL_PLL3,
	CLKCTRL_SEL_PLL4,
	CLKCTRL_SEL_XIN,	// 5
	CLKCTRL_SEL_XTIN,
	CLKCTRL_SEL_PLL0DIV,
	CLKCTRL_SEL_PLL1DIV,
	CLKCTRL_SEL_PLL2DIV,
	CLKCTRL_SEL_PLL3DIV,	// 10
	CLKCTRL_SEL_PLL4DIV,
	CLKCTRL_SEL_XINDIV,
	CLKCTRL_SEL_XTINDIV,
	CLKCTRL_SEL_EXTIN2,
	CLKCTRL_SEL_EXTIN3	// 15

};

/* PeriPheral Clocks */
#define PCLKCTRL_MAX_FCKS	(1600*1000*1000)
#define PCLKCTRL_DIV_MIN	0
#define PCLKCTRL_DIV_DCO_MIN	1
#define PCLKCTRL_DIV_SHIFT	0
#define PCLKCTRL_DIV_XXX_MAX	0xFFF
#define PCLKCTRL_DIV_XXX_MASK	PCLKCTRL_DIV_XXX_MAX
#define PCLKCTRL_DIV_YYY_MAX	0xFFFFFF
#define PCLKCTRL_DIV_YYY_MASK	PCLKCTRL_DIV_YYY_MAX
#define PCLKCTRL_SEL_MIN	0
#define PCLKCTRL_SEL_MAX	26
#define PCLKCTRL_SEL_SHIFT	24
#define PCLKCTRL_SEL_MASK	0x1F
#define PCLKCTRL_EN_SHIFT	29
#define PCLKCTRL_OUTEN_SHIFT	30
#define PCLKCTRL_MD_SHIFT	31
#define tcc_pclkctrl_write(reg,md,en,sel,div,type) { \
	ckc_writel(ckc_readl(reg) & ~(1<<PCLKCTRL_EN_SHIFT), reg); \
	if (type == PCLKCTRL_TYPE_XXX) { \
		ckc_writel((ckc_readl(reg) & ~(PCLKCTRL_DIV_XXX_MASK<<PCLKCTRL_DIV_SHIFT)) | \
				((div&PCLKCTRL_DIV_XXX_MASK)<<PCLKCTRL_DIV_SHIFT), reg); \
		ckc_writel((ckc_readl(reg) & ~(PCLKCTRL_SEL_MASK<<PCLKCTRL_SEL_SHIFT)) | \
				((sel&PCLKCTRL_SEL_MASK)<<PCLKCTRL_SEL_SHIFT), reg); \
	} else if (type == PCLKCTRL_TYPE_YYY) { \
		ckc_writel((ckc_readl(reg) & ~(PCLKCTRL_DIV_YYY_MASK<<PCLKCTRL_DIV_SHIFT)) | \
				((div&PCLKCTRL_DIV_YYY_MASK)<<PCLKCTRL_DIV_SHIFT), reg); \
		ckc_writel((ckc_readl(reg) & ~(PCLKCTRL_SEL_MASK<<PCLKCTRL_SEL_SHIFT)) | \
				((sel&PCLKCTRL_SEL_MASK)<<PCLKCTRL_SEL_SHIFT), reg); \
		ckc_writel((ckc_readl(reg) & ~(1<<PCLKCTRL_MD_SHIFT)) | \
				((md&1)<<PCLKCTRL_MD_SHIFT), reg); \
	} \
	ckc_writel((ckc_readl(reg) & ~(1<<PCLKCTRL_EN_SHIFT)) \
			| ((en&1)<<PCLKCTRL_EN_SHIFT), reg); \
}
typedef enum { /* PCLK Type */
	PCLKCTRL_TYPE_XXX=0,
	PCLKCTRL_TYPE_YYY,
	PCLKCTRL_TYPE_MAX
} tPCLKTYPE;
enum { /* PCLK Mode Selection */
	PCLKCTRL_MODE_DCO=0,
	PCLKCTRL_MODE_DIVIDER,
	PCLKCTRL_MODE_MAX
};
enum{ /* Peri. Clock Source */
	PCLKCTRL_SEL_PLL0=0,
	PCLKCTRL_SEL_PLL1,
	PCLKCTRL_SEL_PLL2,
	PCLKCTRL_SEL_PLL3,
	PCLKCTRL_SEL_PLL4,
	PCLKCTRL_SEL_XIN,
	PCLKCTRL_SEL_XTIN,
	PCLKCTRL_SEL_PLL0DIV=10,
	PCLKCTRL_SEL_PLL1DIV,
	PCLKCTRL_SEL_PLL2DIV,
	PCLKCTRL_SEL_PLL3DIV,
	PCLKCTRL_SEL_PLL4DIV,
	PCLKCTRL_SEL_PCIPHY_CLKOUT=18,
	PCLKCTRL_SEL_HDMITMDS,
	PCLKCTRL_SEL_HDMIPCLK,
	PCLKCTRL_SEL_HDMIXIN,	// 27Mhz
	PCLKCTRL_SEL_XINDIV=23,
	PCLKCTRL_SEL_XTINDIV,
	PCLKCTRL_SEL_EXTCLK0,
	PCLKCTRL_SEL_EXTCLK1,
};

/* io bus configuration */
#define IOBUS_PWDN0	0x000000
#define IOBUS_PWDN1	0x000004
#define IOBUS_PWDN2	0x000008
#define IOBUS_PWDN3	0x000000  /* base: 0x16494400 */
#define IOBUS_RESET0	0x00000C
#define IOBUS_RESET1	0x000010
#define IOBUS_RESET2	0x000014
#define IOBUS_RESET3	0x000004  /* base: 0x16494404 */

/* hsio bus configuration */
#define HSIOBUS_PWDN	0x000000
#define HSIOBUS_RESET	0x000004

/* display bus configuration */
#define DDIBUS_PWDN	0x000000
#define DDIBUS_RESET	0x000004

/* video bus configuraion */
#define VPUBUS_PWDN	0x000000
#define VPUBUS_RESET	0x000004

/**********************************
 *  PMU PWDN/SWRESET Control
 **********************************/
#define PMU_PWRSTS0		0x000 		// Up state
#define PMU_PWRSTS1		0x004 		// Down state
#define PMU_SYSRST		0x010
#define PMU_FSMSTS 		0x018
#define PMU_GB_PWRCTRL		0x050
#define PMU_VB_PWRCTRL		0x054
#define PMU_ISOIP_GB		0x064
#define PMU_ISOIP_DDI		0x068
#define PMU_ISOIP_TOP		0x06C
#define PMU_PWRDN_TOP		0x080
#define PMU_PWRDN_DDIB		0x084
#define PMU_PWRDN_GBUSALL	0x088
#define PMU_PWRDN_VBUSALL	0x08C
#define PMU_PWRDN_A53MP		0x090
#define PMU_PWRDN_A7SP 		0x094
//#define PMU_PWRDN_SHADER0	0x0A8
//#define PMU_PWRDN_SHADER1	0x0AC
#define PMU_PWRUP_DDIB		0x0B4
#define PMU_PWRUP_GBUSALL	0x0B8
#define PMU_PWRUP_VBUSALL	0x0BC
#define PMU_PWRUP_A53MP		0x0C0
#define PMU_PWRUP_A7SP		0x0C4

//#define PMU_PWRUP_SHADER0	0x10C
//#define PMU_PWRUP_SHADER1	0x110

// PMU Power mode status 
#define PMU_PWSTS_TOP 			(1 << 5) 	// Top power up/down state
#define PMU_PWSTS_DDIBUS		(1 << 4) 	// Display bus power up/down state
#define PMU_PWSTS_GBUSALL 		(1 << 3) 	// Graphic bus all (2D, 3D) power up/down state
#define PMU_PWSTS_VBUSALL 		(1 << 2)	// Video bus all power up/down state
#define PMU_PWSTS_A53MPBUS 		(1 << 1) 	// Cortex A53MP bus power up/down state
#define PMU_PWSTS_A7SPBUS 		(1 << 0) 	// Cortex A7SP bus power up/down state

// PMU FSM Status 
#define PMU_FSMSTS_MAINSTS_Msk 	(0x3F << 25)

/**********************************
 *  Mem bus Control
 **********************************/
#define MBUSCLK0			0x000
#define MBUSCLK1			0x004
#define MBUS_SWRESET0		0x008
#define MBUS_SWRESET1		0x00C
#define MBUS_STS			0x018

#define MBUS_STS_CMBUS 		(1 << 0)
#define MBUS_STS_IOBUS 		(1 << 1)
#define MBUS_STS_HSIOBUS 	(1 << 2) 
#define MBUS_STS_DDIBUS1 	(1 << 3)
#define MBUS_STS_DDIBUS0 	(1 << 4)
#define MBUS_STS_G2DBUS 	(1 << 5)
#define MBUS_STS_GPUBUS 	(1 << 6)
#define MBUS_STS_VBUS1 		(1 << 7)
#define MBUS_STS_VBUS0 		(1 << 8)
#define MBUS_STS_CBUS1 		(1 << 9)
#define MBUS_STS_CBUS0 		(1 << 10)
#define MBUS_STS_TOP_ACT 	(1 << 30)
#define MBUS_STS_TOP_IDLE 	(1 << 31)

/**********************************
 *  Graphic bus Control
 **********************************/
#define GRAPHIC_PWRDN 	0x008

#define GRAPHIC_PWRDNACKN 		(1 << 0)
#define GRAPHIC_PWRDNREQN 		(1 << 1)

/**********************************
 *  Display bus Control
 **********************************/
#define DDIBUS_X2XCFG0 	0x080
#define DDIBUS_X2XCFG1 	0x084 

#define DDIBUS_X2XCFG_PWRDNREQN 	(1 << 0)
#define DDIBUS_X2XCFG_PWRDNACKN 	(1 << 1)

/**********************************
 *  Video bus Control
 **********************************/
#define VBUS_X2XCFG0 0x058
#define VBUS_X2XCFG1 0x05C

#define VBUS_X2XCFG_PWRDNREQN	(1 << 0)

static inline int tcc_find_pms(tPMS *PLL, unsigned int srcfreq)
{
	u64	u64_pll, u64_src, fvco, srch_p, srch_m, u64_tmp;
	unsigned int srch_pll, err, srch_err;
	int	srch_s;

	if (PLL->fpll ==0) {
		PLL->en = 0;
		return 0;
	}

	u64_pll = (u64)PLL->fpll;
	u64_src = (u64)srcfreq;

	err = 0xFFFFFFFF;
	srch_err = 0xFFFFFFFF;
	for (srch_s=PLL_S_MAX,fvco=(u64_pll<<PLL_S_MAX) ; srch_s >= PLL_S_MIN ; fvco=(u64_pll<<(--srch_s))) {
		if (fvco >= PLL_VCO_MIN && fvco <= PLL_VCO_MAX) {
			for (srch_p=PLL_P_MIN ; srch_p<=PLL_P_MAX ; srch_p++) {
				srch_m = fvco*srch_p;
				do_div(srch_m, srcfreq);
		                if (srch_m < PLL_M_MIN || srch_m > PLL_M_MAX)
		                        continue;
				u64_tmp = srch_m*u64_src;
				do_div(u64_tmp, srch_p);
		                srch_pll = (unsigned int)(u64_tmp>>srch_s);
				if (srch_pll < PLL_MIN_RATE || srch_pll > PLL_MAX_RATE)
					continue;
		                srch_err = (srch_pll > u64_pll) ? srch_pll - u64_pll : u64_pll - srch_pll;
		                if (srch_err < err) {
		                        err = srch_err;
		                        PLL->p = (unsigned int)srch_p;
		                        PLL->m = (unsigned int)srch_m;
					PLL->s = (unsigned int)srch_s;
		                }
			}
		}
	}
	if (err == 0xFFFFFFFF)
		return -1;

	u64_tmp = u64_src*(unsigned long long)PLL->m;
	do_div(u64_tmp, PLL->p);
	PLL->fpll = (unsigned int)(u64_tmp>>PLL->s);
	PLL->en = 1;
	return 0;
}

static inline int tcc_find_dithered_pms(tPMS *PLL, unsigned int srcfreq)
{
	u64	u64_pll, u64_src, fvco, srch_p, srch_m, u64_tmp;
	unsigned int srch_pll, err, srch_err;
	int	srch_s;

	if (PLL->fpll ==0) {
		PLL->en = 0;
		return 0;
	}

	u64_pll = (u64)PLL->fpll;
	u64_src = (u64)srcfreq;

	err = 0xFFFFFFFF;
	srch_err = 0xFFFFFFFF;
	for (srch_s=DPLL_S_MAX,fvco=(u64_pll<<DPLL_S_MAX) ; srch_s >= DPLL_S_MIN ; fvco=(u64_pll<<(--srch_s))) {
		if (fvco >= DPLL_VCO_MIN && fvco <= DPLL_VCO_MAX) {
			for (srch_p=DPLL_P_MIN ; srch_p<=DPLL_P_MAX ; srch_p++) {
				srch_m = fvco*srch_p;
				do_div(srch_m, srcfreq);
		                if (srch_m < DPLL_M_MIN || srch_m > DPLL_M_MAX)
		                        continue;
				u64_tmp = srch_m*u64_src;
				do_div(u64_tmp, srch_p);
		                srch_pll = (unsigned int)(u64_tmp>>srch_s);
				if (srch_pll < DPLL_MIN_RATE || srch_pll > DPLL_MAX_RATE)
					continue;
		                srch_err = (srch_pll > u64_pll) ? srch_pll - u64_pll : u64_pll - srch_pll;
		                if (srch_err < err) {
		                        err = srch_err;
		                        PLL->p = (unsigned int)srch_p;
		                        PLL->m = (unsigned int)srch_m;
					PLL->s = (unsigned int)srch_s;
		                }
			}
		}
	}
	if (err == 0xFFFFFFFF)
		return -1;

	u64_tmp = u64_src*(unsigned long long)PLL->m;
	do_div(u64_tmp, PLL->p);
	PLL->fpll = (unsigned int)(u64_tmp>>PLL->s);
	PLL->en = 1;
	return 0;
}

static int tcc_ckc_pll_set_rate_2nd(void __iomem *reg, unsigned long rate)
{
	tPMS		nPLL;

	nPLL.fpll = rate;
	if (tcc_find_pms(&nPLL, XIN_CLK_RATE))
		goto tcc_ckc_setpll2_failed;
	tcc_pll2_write(reg, nPLL.en, nPLL.p, nPLL.m, nPLL.s);
	return 0;

tcc_ckc_setpll2_failed:
	tcc_pll2_write(reg, 0, PLL_P_MIN, (PLL_P_MIN*PLL_VCO_MIN+XIN_CLK_RATE)/XIN_CLK_RATE, PLL_S_MIN);
	return -1;
}

static unsigned long tcc_ckc_pll_get_rate_2nd(void __iomem *reg)
{
	unsigned	reg_values = ckc_readl(reg);
	tPMS		nPLLCFG;
	u64		u64_tmp;

	nPLLCFG.p = (reg_values>>PLL_P_SHIFT)&(PLL_P_MASK);
	nPLLCFG.m = (reg_values>>PLL_M_SHIFT)&(PLL_M_MASK);
	nPLLCFG.s = (reg_values>>PLL_S_SHIFT)&(PLL_S_MASK);
	nPLLCFG.en = (reg_values>>PLL_EN_SHIFT)&(1);
	nPLLCFG.src = (reg_values>>PLL_SRC_SHIFT)&(PLL_SRC_MASK);

	u64_tmp = (u64)XIN_CLK_RATE*(u64)nPLLCFG.m;
	do_div(u64_tmp, nPLLCFG.p);
	return (unsigned int)((u64_tmp)>>nPLLCFG.s);
}

static int tcc_ckc_pll_set_rate(int id, unsigned long rate)
{
	void __iomem	*reg = ckc_base+CKC_PLL+id*4;
	unsigned int	srcfreq = 0;
	unsigned int	src = PLLSRC_XIN;
	tPMS		nPLL;

	if (id >= MAX_TCC_PLL)
		return -1;

	if (rate < PLL_MIN_RATE || rate > PLL_MAX_RATE)
		return -1;

	memset(&nPLL, 0x0, sizeof(tPMS));

	switch(src) {
	case PLLSRC_XIN:
		srcfreq = XIN_CLK_RATE;
		break;
	case PLLSRC_HDMIXI:
		srcfreq = HDMI_CLK_RATE;
		break;
	case PLLSRC_EXTCLK0:
		srcfreq = EXT0_CLK_RATE;
		break;
	case PLLSRC_EXTCLK1:
		srcfreq = EXT1_CLK_RATE;
		break;
	default:
		goto tcc_ckc_setpll_failed;
	}
	if (srcfreq==0)
		goto tcc_ckc_setpll_failed;

	nPLL.fpll = rate;
	if (id == DITHERED_PLL_CH) {
		if (tcc_find_dithered_pms(&nPLL, srcfreq)) {
			tcc_dpll_write(reg, 0, DPLL_P_MIN, (DPLL_P_MIN*DPLL_VCO_MIN+XIN_CLK_RATE)/XIN_CLK_RATE, DPLL_S_MIN, src);
			tcc_ckc_reset_clock_source(id);
			return -1;
		}
		tcc_dpll_write(reg, nPLL.en, nPLL.p, nPLL.m, nPLL.s, src);
	}
	else {
		if (tcc_find_pms(&nPLL, srcfreq))
			goto tcc_ckc_setpll_failed;
		tcc_pll_write(reg, nPLL.en, nPLL.p, nPLL.m, nPLL.s, src);
	}
	tcc_ckc_reset_clock_source(id);
	return 0;

tcc_ckc_setpll_failed:
	tcc_pll_write(reg, 0, PLL_P_MIN, (PLL_P_MIN*PLL_VCO_MIN+XIN_CLK_RATE)/XIN_CLK_RATE, PLL_S_MIN, src);
	tcc_ckc_reset_clock_source(id);
	return -1;
}

static unsigned long tcc_ckc_pll_get_rate(int id)
{
	void __iomem	*reg = ckc_base+CKC_PLL+id*4;
	unsigned	reg_values = ckc_readl(reg);
	tPMS		nPLLCFG;
	unsigned int	src_freq;
	u64		u64_tmp;

	if (id >= MAX_TCC_PLL)
		return 0;

	if (id == DITHERED_PLL_CH) {
		nPLLCFG.p = (reg_values>>DPLL_P_SHIFT)&(DPLL_P_MASK);
		nPLLCFG.m = (reg_values>>DPLL_M_SHIFT)&(DPLL_M_MASK);
		nPLLCFG.s = (reg_values>>DPLL_S_SHIFT)&(DPLL_S_MASK);
		nPLLCFG.en = (reg_values>>DPLL_EN_SHIFT)&(1);
		nPLLCFG.src = (reg_values>>DPLL_SRC_SHIFT)&(DPLL_SRC_MASK);
	}
	else {
		nPLLCFG.p = (reg_values>>PLL_P_SHIFT)&(PLL_P_MASK);
		nPLLCFG.m = (reg_values>>PLL_M_SHIFT)&(PLL_M_MASK);
		nPLLCFG.s = (reg_values>>PLL_S_SHIFT)&(PLL_S_MASK);
		nPLLCFG.en = (reg_values>>PLL_EN_SHIFT)&(1);
		nPLLCFG.src = (reg_values>>PLL_SRC_SHIFT)&(PLL_SRC_MASK);
	}

	if (nPLLCFG.en == 0)
		return 0;

	switch (nPLLCFG.src) {
	case PLLSRC_XIN:
		src_freq = XIN_CLK_RATE;
		break;
	case PLLSRC_HDMIXI:
		src_freq = HDMI_CLK_RATE;
		break;
	case PLLSRC_EXTCLK0:
		src_freq = EXT0_CLK_RATE;
		break;
	case PLLSRC_EXTCLK1:
		src_freq = EXT1_CLK_RATE;
		break;
	default:
		return 0;
	}

	u64_tmp = (u64)src_freq*(u64)nPLLCFG.m;
	do_div(u64_tmp, nPLLCFG.p);
	return (unsigned int)((u64_tmp)>>nPLLCFG.s);
}

static unsigned long tcc_ckc_plldiv_get_rate(int id)
{
	void __iomem	*reg;
	unsigned	reg_values;
	unsigned int	offset=0, fpll=0, pdiv=0;

	if (id >= MAX_TCC_PLL)
		return 0;

	switch(id) {
	case PLL_0:
	case PLL_1:
	case PLL_2:
	case PLL_3:
		reg = ckc_base+CKC_CLKDIVC;
		offset = (3-id)*8;
		break;
	case PLL_4:
		reg = ckc_base+CKC_CLKDIVC+0x4;
		offset = 24;
		break;
	default:
		return 0;
	}

	reg_values = ckc_readl(reg);
	if (((reg_values >> offset)&0x80) == 0)	/* check plldivc enable bit */
		return 0;
	pdiv = (reg_values >> offset)&0x3F;
	if (!pdiv)	/* should not be zero */
		return 0;
	fpll = tcc_ckc_pll_get_rate(id);
	return (unsigned int)fpll/(pdiv+1);
}

static inline int tcc_find_clkctrl(tCLKCTRL *CLKCTRL)
{
	unsigned int i, div[MAX_CLK_SRC], err[MAX_CLK_SRC], searchsrc, clk_rate;
	searchsrc = 0xFFFFFFFF;

	if (CLKCTRL->freq <= (XIN_CLK_RATE/2)) {
		CLKCTRL->sel = CLKCTRL_SEL_XIN;
		CLKCTRL->freq = XIN_CLK_RATE/2;
		CLKCTRL->config = 1;
	}
	else {
		for (i=0 ; i<MAX_CLK_SRC ; i++) {
			if (stClockSource[i] == 0)
				continue;
			div[i] = (stClockSource[i]+CLKCTRL->freq-1)/CLKCTRL->freq;
			if (div[i] > (CLKCTRL_CONFIG_MAX+1))
				div[i] = CLKCTRL_CONFIG_MAX+1;
			else if (div[i] < (CLKCTRL_CONFIG_MIN+1))
				div[i] = CLKCTRL_CONFIG_MIN+1;
			clk_rate = stClockSource[i]/div[i];
			if (CLKCTRL->freq < clk_rate)
				continue;
			err[i] = CLKCTRL->freq - clk_rate;
			if (searchsrc == 0xFFFFFFFF)
				searchsrc = i;
			else {
				/* find similar clock */
				if (err[i] < err[searchsrc])
					searchsrc = i;
				/* find even division vlaue */
				else if(err[i] == err[searchsrc]) {
					if (div[i]%2 == 0)
						searchsrc = i;
				}
			}
			if (err[searchsrc] == 0)
				break;
		}
		if (searchsrc == 0xFFFFFFFF)
			return -1;
		switch(searchsrc) {
		case PLL_0:	CLKCTRL->sel = CLKCTRL_SEL_PLL0; break;
		case PLL_1:	CLKCTRL->sel = CLKCTRL_SEL_PLL1; break;
		case PLL_2:	CLKCTRL->sel = CLKCTRL_SEL_PLL2; break;
		case PLL_3:	CLKCTRL->sel = CLKCTRL_SEL_PLL3; break;
		case PLL_4:	CLKCTRL->sel = CLKCTRL_SEL_PLL4; break;
		case PLL_DIV_0:	CLKCTRL->sel = CLKCTRL_SEL_PLL0DIV; break;
		case PLL_DIV_1:	CLKCTRL->sel = CLKCTRL_SEL_PLL1DIV; break;
		case PLL_DIV_2:	CLKCTRL->sel = CLKCTRL_SEL_PLL2DIV; break;
		case PLL_DIV_3:	CLKCTRL->sel = CLKCTRL_SEL_PLL3DIV; break;
		case PLL_DIV_4:	CLKCTRL->sel = CLKCTRL_SEL_PLL4DIV; break;
		case PLL_XIN:	CLKCTRL->sel = CLKCTRL_SEL_XIN; break;
		default: return -1;
		}
		if (div[searchsrc] > (CLKCTRL_CONFIG_MAX+1))
			div[searchsrc] = CLKCTRL_CONFIG_MAX+1;
		else if (div[searchsrc] <= CLKCTRL_CONFIG_MIN)
			div[searchsrc] = CLKCTRL_CONFIG_MIN+1;
		CLKCTRL->freq = stClockSource[searchsrc]/div[searchsrc];
		CLKCTRL->config = div[searchsrc] - 1;
	}
	return 0;
}

static int tcc_ckc_clkctrl_enable(int id)
{
	void __iomem	*reg = ckc_base+CKC_CLKCTRL+id*4;
	void __iomem	*subpwdn_reg = NULL;
	void __iomem	*subrst_reg = NULL;

	switch (id) {
		case FBUS_CPU0:
		case FBUS_CPU1:
		case FBUS_MEM:
		case FBUS_MEM_PHY:
			return 0;
		case FBUS_GPU:
			ckc_writel(ckc_readl(gpu_3d_base + CKC2_PLL) | (1<<PLL_LOCKEN_SHIFT), gpu_3d_base + CKC2_PLL);
			ckc_writel(ckc_readl(gpu_3d_base + CKC2_PLL) | (1<<PLL_EN_SHIFT), gpu_3d_base + CKC2_PLL);
			while((ckc_readl(gpu_3d_base + CKC2_PLL)&(1<<PLL_LOCKEN_SHIFT))==0);
			tcc_clkctrl2_write((gpu_3d_base + CKC2_CLKCTRL), CLKCTRL2_SEL_PLL);
			tcc_clkctrl2_write((gpu_3d_base + CKC2_CLKCTRL+0x4), CLKCTRL2_SEL_PLL);
			return 0;
		case FBUS_G2D:
			ckc_writel(ckc_readl(gpu_2d_base + CKC2_PLL) | (1<<PLL_LOCKEN_SHIFT), gpu_2d_base + CKC2_PLL);
			ckc_writel(ckc_readl(gpu_2d_base + CKC2_PLL) | (1<<PLL_EN_SHIFT), gpu_2d_base + CKC2_PLL);
			while((ckc_readl(gpu_2d_base + CKC2_PLL)&(1<<PLL_LOCKEN_SHIFT))==0);
			ckc_writel(0x00000041, gpu_2d_base + CKC2_CLKDIVC);
			tcc_clkctrl2_write((gpu_2d_base + CKC2_CLKCTRL), CLKCTRL2_SEL_PLLDIV);
			tcc_clkctrl2_write((gpu_2d_base + CKC2_CLKCTRL+0x4), CLKCTRL2_SEL_PLL);
			return 0;
	}

	/* disable sub-block hclk/swrst, when en status chagned 0->1 */
	if ((ckc_readl(reg)&(1<<CLKCTRL_EN_SHIFT)) == 0) {
		switch (id) {
		case FBUS_VBUS:
			subpwdn_reg = vpubus_cfg_base + VPUBUS_PWDN;
			subrst_reg = vpubus_cfg_base + VPUBUS_RESET;
			break;
		case FBUS_HSIO:
			subpwdn_reg = hsiobus_cfg_base + HSIOBUS_PWDN;
			subrst_reg = hsiobus_cfg_base + HSIOBUS_RESET;
			break;
		case FBUS_DDI:
			subpwdn_reg = ddibus_cfg_base + DDIBUS_PWDN;
			subrst_reg = ddibus_cfg_base + DDIBUS_RESET;
			break;
		}
	}

	ckc_writel(ckc_readl(reg) | (1<<CLKCTRL_EN_SHIFT), reg);
	while (ckc_readl(reg) & (1<<CLKCTRL_CFGRQ_SHIFT));

	/* disable sub-block hclk/swrst */
	if (subpwdn_reg)
		ckc_writel(0x0, subpwdn_reg);
	if (subrst_reg)
		ckc_writel(0x0, subrst_reg);

	return 0;
}

static int tcc_ckc_clkctrl_disable(int id)
{
	void __iomem	*reg = ckc_base+CKC_CLKCTRL+id*4;
	void __iomem	*reg_cpu_pll = NULL;

	switch (id) {
		case FBUS_CPU0:
		case FBUS_CPU1:
		case FBUS_MEM:
		case FBUS_MEM_PHY:
			return 0;
		case FBUS_GPU:
			reg_cpu_pll = gpu_3d_base;
			break;
		case FBUS_G2D:
			reg_cpu_pll = gpu_2d_base;
			break;
	}

	if (reg_cpu_pll) {
		int i;
		tcc_clkctrl2_write((reg_cpu_pll + CKC2_CLKCTRL), CLKCTRL2_SEL_XIN);
		tcc_clkctrl2_write((reg_cpu_pll + CKC2_CLKCTRL+0x4), CLKCTRL2_SEL_XIN);
		for (i=100 ; i ; i--);
		ckc_writel(0x00000001, reg_cpu_pll + CKC2_CLKDIVC);
		ckc_writel(ckc_readl(reg_cpu_pll + CKC2_PLL) & ~(1<<PLL_EN_SHIFT), reg_cpu_pll + CKC2_PLL);
		return 0;
	}

	ckc_writel(ckc_readl(reg) & ~(1<<CLKCTRL_EN_SHIFT), reg);
	return 0;
}

static int tcc_ckc_is_pmu_pwdn(int id)
{
	void __iomem *pwrsts_reg = NULL;
	unsigned pwrsts_mask = 0;

	switch (id) {
	case FBUS_CPU0:
		pwrsts_reg = pmu_base + PMU_PWRSTS1;
		pwrsts_mask = PMU_PWSTS_A53MPBUS;
		break;
	case FBUS_CPU1:
		pwrsts_reg = pmu_base + PMU_PWRSTS1;
		pwrsts_mask = PMU_PWSTS_A7SPBUS;
		break;
	case FBUS_MEM:
		return -1;
	case FBUS_DDI:
		pwrsts_reg = pmu_base + PMU_PWRSTS1;
		pwrsts_mask = PMU_PWSTS_DDIBUS;
		break;
	case FBUS_GPU:
		pwrsts_reg = pmu_base + PMU_GB_PWRCTRL;
		pwrsts_mask = (1 << 8);
		break;
	case FBUS_G2D:
		pwrsts_reg = pmu_base + PMU_GB_PWRCTRL;
		pwrsts_mask = (1 << 5);
		break;
	case FBUS_IO:
		break;
	case FBUS_VBUS:
		pwrsts_reg = pmu_base + PMU_PWRSTS1;
		pwrsts_mask = PMU_PWSTS_VBUSALL;
		break;
	case FBUS_CODA:
		return -1;
	case FBUS_HSIO:
		return -1;
	case FBUS_SMU:
		return -1;
	case FBUS_CMBUS:
		return -1;
	case FBUS_CHEVC:
	case FBUS_VHEVC:
	case FBUS_BHEVC:
		return -1;
	default:
		return -1;
	}

	return (ckc_readl(pwrsts_reg)&pwrsts_mask) ? 1 : 0;
}

static int tcc_ckc_is_clkctrl_enabled(int id)
{
	void __iomem	*reg = ckc_base+CKC_CLKCTRL+id*4;
	void __iomem	*reg_cpu_pll = NULL;

	switch (id) {
		case FBUS_CPU0:
			return 1;
		case FBUS_CPU1:
			if ((ckc_readl(cpubus_cfg_base + 0x4) & (1<<17)) == 0)
				return 0;
			return 1;
		case FBUS_MEM:
		case FBUS_MEM_PHY:
			return 1;
		case FBUS_GPU:
			if(tcc_ckc_is_pmu_pwdn(id))
				return 0;
			reg_cpu_pll = gpu_3d_base;
			break;
		case FBUS_G2D:
			if(tcc_ckc_is_pmu_pwdn(id))
				return 0;
			reg_cpu_pll = gpu_2d_base;
			break;
	}

	if (reg_cpu_pll)
		return (ckc_readl(reg_cpu_pll+CKC2_PLL) & (1<<PLL_EN_SHIFT)) ? 1 : 0;

	return (ckc_readl(reg) & (1<<CLKCTRL_EN_SHIFT)) ? 1 : 0;
}

static int tcc_ckc_clkctrl_set_rate(int id, unsigned long rate)
{
	void __iomem	*reg = ckc_base+CKC_CLKCTRL+id*4;
	tCLKCTRL	nCLKCTRL;
	volatile unsigned long  flags;

	nCLKCTRL.en = (ckc_readl(reg) & (1<<CLKCTRL_EN_SHIFT)) ? 1 : 0;
	switch (id) {
	case FBUS_CPU0:
		local_irq_save(flags);
		tcc_clkctrl2_write((cpu0_base + CKC2_CLKCTRL), CLKCTRL2_SEL_XIN);
		tcc_ckc_pll_set_rate_2nd((cpu0_base + CKC2_PLL), rate);
		tcc_clkctrl2_write((cpu0_base + CKC2_CLKCTRL), CLKCTRL2_SEL_PLL);
		local_irq_restore(flags);
		break;
	case FBUS_CPU1:
		return -1;
	case FBUS_GPU:
		tcc_clkctrl2_write((gpu_3d_base + CKC2_CLKCTRL), CLKCTRL2_SEL_XIN);
		tcc_clkctrl2_write((gpu_3d_base + CKC2_CLKCTRL+0x4), CLKCTRL2_SEL_XIN);
		tcc_ckc_pll_set_rate_2nd((gpu_3d_base + CKC2_PLL), rate);
		tcc_clkctrl2_write((gpu_3d_base + CKC2_CLKCTRL), CLKCTRL2_SEL_PLL);
		tcc_clkctrl2_write((gpu_3d_base + CKC2_CLKCTRL+0x4), CLKCTRL2_SEL_PLL);
		tcc_dckc_store(id);
		return 0;
	case FBUS_G2D:
		tcc_clkctrl2_write((gpu_2d_base + CKC2_CLKCTRL), CLKCTRL2_SEL_XIN);
		tcc_clkctrl2_write((gpu_2d_base + CKC2_CLKCTRL+0x4), CLKCTRL2_SEL_XIN);
		tcc_ckc_pll_set_rate_2nd((gpu_2d_base + CKC2_PLL), rate);
		ckc_writel(0x00000041, gpu_2d_base + CKC2_CLKDIVC);
		tcc_clkctrl2_write((gpu_2d_base + CKC2_CLKCTRL), CLKCTRL2_SEL_PLLDIV);
		tcc_clkctrl2_write((gpu_2d_base + CKC2_CLKCTRL+0x4), CLKCTRL2_SEL_PLL);
		tcc_dckc_store(id);
		return 0;
	case FBUS_MEM:
	case FBUS_MEM_PHY:
		//printk("memory clock cannot change !!\n");
		BUG();
		return 0;
	}

	nCLKCTRL.freq = rate;
	if (tcc_find_clkctrl(&nCLKCTRL))
		return -1;

	tcc_clkctrl_write(reg, nCLKCTRL.en, nCLKCTRL.config, nCLKCTRL.sel);
	return 0;
}

static unsigned long tcc_ckc_clkctrl_get_rate(int id)
{
	void __iomem	*reg = ckc_base+CKC_CLKCTRL+id*4;
	void __iomem	*reg_2nd = NULL;
	unsigned	reg_values = ckc_readl(reg);
	tCLKCTRL	nCLKCTRL;
	unsigned int	src_freq = 0;

	switch (id) {
		case FBUS_CPU0:
			reg_2nd = cpu0_base + CKC2_PLL;
			break;
		case FBUS_CPU1:
			return 0;
		case FBUS_GPU:
			reg_2nd = &(dckc_backup[DCKC_GPU].pms);
			break;
		case FBUS_G2D:
			reg_2nd = &(dckc_backup[DCKC_G2D].pms);
			break;
		case FBUS_MEM_PHY:
			reg_2nd = mem_ckc_base + CKC2_PLL;
			break;
	}

	if (reg_2nd)
		return tcc_ckc_pll_get_rate_2nd(reg_2nd);

	nCLKCTRL.sel = (reg_values & (CLKCTRL_SEL_MASK<<CLKCTRL_SEL_SHIFT))>>CLKCTRL_SEL_SHIFT;
	switch (nCLKCTRL.sel) {
		case CLKCTRL_SEL_PLL0:
			src_freq = tcc_ckc_pll_get_rate(PLL_0);
			break;
		case CLKCTRL_SEL_PLL1:
			src_freq = tcc_ckc_pll_get_rate(PLL_1);
			break;
		case CLKCTRL_SEL_PLL2:
			src_freq = tcc_ckc_pll_get_rate(PLL_2);
			break;
		case CLKCTRL_SEL_PLL3:
			src_freq = tcc_ckc_pll_get_rate(PLL_3);
			break;
		case CLKCTRL_SEL_PLL4:
			src_freq = tcc_ckc_pll_get_rate(PLL_4);
			break;
		case CLKCTRL_SEL_XIN:
			src_freq = XIN_CLK_RATE;
			break;
		case CLKCTRL_SEL_XTIN:
			src_freq = XTIN_CLK_RATE;
			break;
		case CLKCTRL_SEL_PLL0DIV:
			src_freq = tcc_ckc_plldiv_get_rate(PLL_0);
			break;
		case CLKCTRL_SEL_PLL1DIV:
			src_freq = tcc_ckc_plldiv_get_rate(PLL_1);
			break;
		case CLKCTRL_SEL_PLL2DIV:
			src_freq = tcc_ckc_plldiv_get_rate(PLL_2);
			break;
		case CLKCTRL_SEL_PLL3DIV:
			src_freq = tcc_ckc_plldiv_get_rate(PLL_3);
			break;
		case CLKCTRL_SEL_PLL4DIV:
			src_freq = tcc_ckc_plldiv_get_rate(PLL_4);
			break;
		case CLKCTRL_SEL_XINDIV:
		case CLKCTRL_SEL_XTINDIV:
		case CLKCTRL_SEL_EXTIN2:
		case CLKCTRL_SEL_EXTIN3:
		default: return 0;
	}

	nCLKCTRL.config = (reg_values & (CLKCTRL_CONFIG_MASK<<CLKCTRL_CONFIG_SHIFT))>>CLKCTRL_CONFIG_SHIFT;
	nCLKCTRL.freq = src_freq / (nCLKCTRL.config+1);

	return (unsigned long)nCLKCTRL.freq;
}

static inline unsigned int tcc_pclk_support_below_freq(unsigned int periname)
{
	if ((periname == PERI_SDMMC0) || (periname == PERI_SDMMC1)
	 || (periname == PERI_SDMMC2)) {
	 	/* calc. freq. must be below(same or under) value */
	 	return 1;
	}

	return 0;
}

static inline unsigned int tcc_ckc_pclk_divider(tPCLKCTRL *PCLKCTRL, unsigned int *div,
	const unsigned int src_CLK, unsigned int div_min, unsigned int div_max)
{
	unsigned int	clk_rate1, clk_rate2, err1, err2;

	if (src_CLK <= PCLKCTRL->freq)
		*div = 1;
	else
		*div = src_CLK/PCLKCTRL->freq;

#ifdef HDMIA_SRC_SHOULD_BE_SAME_WITH_AUDIO
	switch (PCLKCTRL->periname) {
	case PERI_MDAI0:
	case PERI_MSPDIF0:
	case PERI_MDAI1:
	case PERI_MSPDIF1:
		if (*div < 2) /* dai should be (hdmi_audio*2) */
			*div = 2;
		break;
	}
#endif

	if (*div > div_max)
		*div = div_max;

	clk_rate1 = src_CLK/(*div);
	clk_rate2 = src_CLK/((*div < div_max) ? (*div+1) : *div);
	if (tcc_pclk_support_below_freq(PCLKCTRL->periname)) {
		err1 = (clk_rate1 > PCLKCTRL->freq)?0xFFFFFFFF:(PCLKCTRL->freq - clk_rate1);
		err2 = (clk_rate2 > PCLKCTRL->freq)?0xFFFFFFFF:(PCLKCTRL->freq - clk_rate2);
	}
	else {
		err1 = (clk_rate1 > PCLKCTRL->freq)?(clk_rate1 - PCLKCTRL->freq):(PCLKCTRL->freq - clk_rate1);
		err2 = (clk_rate2 > PCLKCTRL->freq)?(clk_rate2 - PCLKCTRL->freq):(PCLKCTRL->freq - clk_rate2);
	}

	if (err1 > err2)
		*div += 1;

	return (err1 < err2) ? err1 : err2;
}

static inline unsigned int tcc_ckc_pclk_dco(	tPCLKCTRL *PCLKCTRL, unsigned int *div,
	const unsigned int src_CLK, unsigned int div_min, unsigned int div_max)
{
	unsigned int	clk_rate1, clk_rate2, err1, err2;
	u64 u64_tmp;

	if (src_CLK < PCLKCTRL->freq)
		return 0xFFFFFFFF;

	u64_tmp = (unsigned long long)(PCLKCTRL->freq)*(unsigned long long)div_max;
	do_div(u64_tmp,src_CLK);
	*div = (unsigned int)u64_tmp;

	if (*div > (div_max+1)/2)
		return 0xFFFFFFFF;

#ifdef HDMIA_SRC_SHOULD_BE_SAME_WITH_AUDIO
	switch (PCLKCTRL->periname) {
	case PERI_MDAI0:
	case PERI_MSPDIF0:
	case PERI_MDAI1:
	case PERI_MSPDIF1:
		if (*div > (div_max+1)/4) /* dai should be (hdmi_audio*2) */
			return 0xFFFFFFFF;
		break;
	}
#endif

	u64_tmp = (unsigned long long)src_CLK*(unsigned long long)(*div);
	do_div(u64_tmp,(div_max+1));
	clk_rate1 = (unsigned int)u64_tmp;
	u64_tmp = (unsigned long long)src_CLK*(unsigned long long)(*div+1);
	do_div(u64_tmp,(div_max+1));
	clk_rate2 = (unsigned int)u64_tmp;

	if (tcc_pclk_support_below_freq(PCLKCTRL->periname)) {
		err1 = (clk_rate1 > PCLKCTRL->freq) ? 0xFFFFFFFF : PCLKCTRL->freq - clk_rate1;
		err2 = (clk_rate2 > PCLKCTRL->freq) ? 0xFFFFFFFF : PCLKCTRL->freq - clk_rate2;
	}
	else {
		err1 = (clk_rate1 > PCLKCTRL->freq) ? clk_rate1 - PCLKCTRL->freq : PCLKCTRL->freq - clk_rate1;
		err2 = (clk_rate2 > PCLKCTRL->freq) ? clk_rate2 - PCLKCTRL->freq : PCLKCTRL->freq - clk_rate2;
	}
	if (err1 > err2)
		*div += 1;

	return (err1 < err2) ? err1 : err2;
}

static inline int tcc_find_pclk(tPCLKCTRL *PCLKCTRL, tPCLKTYPE type)
{
	int i;
	unsigned int div_max, searchsrc, err_dco, err_div, md;
	unsigned int div[MAX_CLK_SRC], err[MAX_CLK_SRC];
	unsigned int div_dco = PCLKCTRL_DIV_DCO_MIN;
	unsigned int div_div = PCLKCTRL_DIV_MIN;

	switch (type) {
	case PCLKCTRL_TYPE_XXX:
		PCLKCTRL->md = PCLKCTRL_MODE_DIVIDER;
		div_max = PCLKCTRL_DIV_XXX_MAX;
		break;
	case PCLKCTRL_TYPE_YYY:
		PCLKCTRL->md = PCLKCTRL_MODE_DCO;
		div_max = PCLKCTRL_DIV_YYY_MAX;
		if (((PCLKCTRL->periname == PERI_MDAI0)
#ifdef HDMIA_SRC_SHOULD_BE_SAME_WITH_AUDIO
			|| ((PCLKCTRL->periname == PERI_HDMI_SPDIFCLK) && (stHDMIASrc == audio_pll_ch))
#endif
		     ) && audio_pll_used) {
			if (tcc_find_audio_pclk(PCLKCTRL) == 0)
				return 0;
		}
		break;
	default:
		return -1;
	}

	searchsrc = 0xFFFFFFFF;
	for (i=(MAX_CLK_SRC-1) ; i>=0 ; i--) {
#ifdef HDMIA_SRC_SHOULD_BE_SAME_WITH_AUDIO
		if ((stHDMIASrc < MAX_CLK_SRC) && (PCLKCTRL->periname == PERI_HDMI_SPDIFCLK))
			i = stHDMIASrc;
		else
#endif
		{
			if (stClockSource[i] == 0)
				continue;
			if ((stClockSource[i] >= PCLKCTRL_MAX_FCKS) && (type == PCLKCTRL_TYPE_XXX))
				continue;
		}

		/* dco mode */
		if (type == PCLKCTRL_TYPE_XXX)
			err_dco = 0xFFFFFFFF;
		else {
			if((PCLKCTRL->periname == PERI_MSPDIF0)||(PCLKCTRL->periname == PERI_MSPDIF1))
				err_dco = 0xFFFFFFFF;
			else
				err_dco = tcc_ckc_pclk_dco(PCLKCTRL, &div_dco, stClockSource[i], PCLKCTRL_DIV_DCO_MIN, div_max);
		}

		/* divider mode */
		err_div = tcc_ckc_pclk_divider(PCLKCTRL, &div_div, stClockSource[i], PCLKCTRL_DIV_MIN+1, div_max+1);

		/* common */
		if (err_dco < err_div) {
			err[i] = err_dco;
			div[i] = div_dco;
			md = PCLKCTRL_MODE_DCO;
		}
		else {
			err[i] = err_div;
			div[i] = div_div;
			md = PCLKCTRL_MODE_DIVIDER;
		}

		if (searchsrc == 0xFFFFFFFF) {
			searchsrc = i;
			PCLKCTRL->md = md;
		}
		else {
			/* find similar clock */
			if (err[i] < err[searchsrc]) {
				searchsrc = i;
				PCLKCTRL->md = md;
			}
		}

#ifdef HDMIA_SRC_SHOULD_BE_SAME_WITH_AUDIO
		if ((stHDMIASrc < MAX_CLK_SRC) && (PCLKCTRL->periname == PERI_HDMI_SPDIFCLK))
			break;
#endif

		if (err[searchsrc] == 0)
			break;
	}

	switch(searchsrc) {
	case PLL_0:	PCLKCTRL->sel = PCLKCTRL_SEL_PLL0; break;
	case PLL_1:	PCLKCTRL->sel = PCLKCTRL_SEL_PLL1; break;
	case PLL_2:	PCLKCTRL->sel = PCLKCTRL_SEL_PLL2; break;
	case PLL_3:	PCLKCTRL->sel = PCLKCTRL_SEL_PLL3; break;
	case PLL_4:	PCLKCTRL->sel = PCLKCTRL_SEL_PLL4; break;
	case PLL_DIV_0:	PCLKCTRL->sel = PCLKCTRL_SEL_PLL0DIV; break;
	case PLL_DIV_1:	PCLKCTRL->sel = PCLKCTRL_SEL_PLL1DIV; break;
	case PLL_DIV_2:	PCLKCTRL->sel = PCLKCTRL_SEL_PLL2DIV; break;
	case PLL_DIV_3:	PCLKCTRL->sel = PCLKCTRL_SEL_PLL3DIV; break;
	case PLL_DIV_4:	PCLKCTRL->sel = PCLKCTRL_SEL_PLL4DIV; break;
	case PLL_XIN:	PCLKCTRL->sel = PCLKCTRL_SEL_XIN; break;
	case PLL_XTIN:	PCLKCTRL->sel = PCLKCTRL_SEL_XTIN; break;
	default: return -1;
	}

	if (PCLKCTRL->md == PCLKCTRL_MODE_DCO) {
		u64 u64_tmp;
		PCLKCTRL->div = div[searchsrc];
		if (PCLKCTRL->div > div_max/2)
			u64_tmp = (unsigned long long)stClockSource[searchsrc]*(unsigned long long)(div_max-PCLKCTRL->div);
		else
			u64_tmp = (unsigned long long)stClockSource[searchsrc]*(unsigned long long)PCLKCTRL->div;
		do_div(u64_tmp,div_max);
		PCLKCTRL->freq = (unsigned int)u64_tmp;

		if ((PCLKCTRL->div < PCLKCTRL_DIV_DCO_MIN) || (PCLKCTRL->div > (div_max-1)))
			return -1;
	}
	else { /* Divider mode */
		PCLKCTRL->div = div[searchsrc];
		if (PCLKCTRL->div >= (PCLKCTRL_DIV_MIN+1) && PCLKCTRL->div <= (div_max+1))
			PCLKCTRL->div -= 1;
		else
			return -1;
		PCLKCTRL->freq = stClockSource[searchsrc]/(PCLKCTRL->div+1);
	}
	return 0;
}

static inline tPCLKTYPE tcc_check_pclk_type(unsigned int periname)
{
	switch (periname) {
		case PERI_MDAI0:
		case PERI_MSPDIF0:
		case PERI_MDAI1:
		case PERI_MSPDIF1:
		case PERI_TSRX0:
		case PERI_TSRX1:
		case PERI_TSRX2:
		case PERI_TSRX3:
			return PCLKCTRL_TYPE_YYY;
	}
	return PCLKCTRL_TYPE_XXX;
}

static int tcc_ckc_peri_enable(int id)
{
	void __iomem	*reg = ckc_base+CKC_PCLKCTRL+id*4;
	ckc_writel(ckc_readl(reg) | 1<<PCLKCTRL_EN_SHIFT, reg);

	return 0;
}

static int tcc_ckc_peri_disable(int id)
{
	void __iomem	*reg = ckc_base+CKC_PCLKCTRL+id*4;
	ckc_writel(ckc_readl(reg) & ~(1<<PCLKCTRL_EN_SHIFT), reg);

	return 0;
}

static int tcc_ckc_is_peri_enabled(int id)
{
	void __iomem	*reg = ckc_base+CKC_PCLKCTRL+id*4;
	return (ckc_readl(reg) & (1<<PCLKCTRL_EN_SHIFT)) ? 1 : 0;
}

static int tcc_ckc_peri_set_rate(int id, unsigned long rate)
{
	void __iomem	*reg = ckc_base+CKC_PCLKCTRL+id*4;
	tPCLKCTRL	nPCLKCTRL;
	tPCLKTYPE	type = tcc_check_pclk_type(id);

	nPCLKCTRL.freq = rate;
	nPCLKCTRL.periname = id;
	nPCLKCTRL.div = 0;
	nPCLKCTRL.md = PCLKCTRL_MODE_DCO;
	nPCLKCTRL.sel = PCLKCTRL_SEL_XIN;

	/* if input rate are 27(dummy value for set HDMIPCLK) then set the lcdc pclk source to HDMIPCLK */
	if (((nPCLKCTRL.periname == PERI_LCD0) || (nPCLKCTRL.periname == PERI_LCD1))
			&& (nPCLKCTRL.freq == HDMI_PCLK_RATE)) {
		nPCLKCTRL.sel = PCLKCTRL_SEL_HDMIPCLK;
		nPCLKCTRL.div = 0;
		nPCLKCTRL.md = PCLKCTRL_MODE_DIVIDER;
		nPCLKCTRL.freq = HDMI_PCLK_RATE;
	}
	else {
		if (tcc_find_pclk(&nPCLKCTRL, type))
			goto tcc_ckc_setperi_failed;
	}

	nPCLKCTRL.en = (ckc_readl(reg) & (1<<PCLKCTRL_EN_SHIFT)) ? 1 : 0;

	tcc_pclkctrl_write(reg, nPCLKCTRL.md, nPCLKCTRL.en, nPCLKCTRL.sel, nPCLKCTRL.div, type);

	return 0;

tcc_ckc_setperi_failed:
	tcc_pclkctrl_write(reg, PCLKCTRL_MODE_DIVIDER, CKC_DISABLE, PCLKCTRL_SEL_XIN, 1, type);
	return -1;
}

static unsigned long tcc_ckc_peri_get_rate(int id)
{
	void __iomem	*reg = ckc_base+CKC_PCLKCTRL+id*4;
	unsigned	reg_values = ckc_readl(reg);
	tPCLKCTRL	nPCLKCTRL;
	tPCLKTYPE	type = tcc_check_pclk_type(id);
	unsigned int	src_freq = 0, div_mask;

	nPCLKCTRL.md = reg_values & (1<<PCLKCTRL_MD_SHIFT) ? PCLKCTRL_MODE_DIVIDER : PCLKCTRL_MODE_DCO;
	nPCLKCTRL.sel = (reg_values&(PCLKCTRL_SEL_MASK<<PCLKCTRL_SEL_SHIFT))>>PCLKCTRL_SEL_SHIFT;
	switch(nPCLKCTRL.sel) {
	case PCLKCTRL_SEL_PLL0:
		src_freq = tcc_ckc_pll_get_rate(PLL_0);
		break;
	case PCLKCTRL_SEL_PLL1:
		src_freq = tcc_ckc_pll_get_rate(PLL_1);
		break;
	case PCLKCTRL_SEL_PLL2:
		src_freq = tcc_ckc_pll_get_rate(PLL_2);
		break;
	case PCLKCTRL_SEL_PLL3:
		src_freq = tcc_ckc_pll_get_rate(PLL_3);
		break;
	case PCLKCTRL_SEL_PLL4:
		src_freq = tcc_ckc_pll_get_rate(PLL_4);
		break;
	case PCLKCTRL_SEL_XIN:
		src_freq = XIN_CLK_RATE;
		break;
	case PCLKCTRL_SEL_XTIN:
		src_freq = XTIN_CLK_RATE;
		break;
	case PCLKCTRL_SEL_PLL0DIV:
		src_freq = tcc_ckc_plldiv_get_rate(PLL_0);
		break;
	case PCLKCTRL_SEL_PLL1DIV:
		src_freq = tcc_ckc_plldiv_get_rate(PLL_1);
		break;
	case PCLKCTRL_SEL_PLL2DIV:
		src_freq = tcc_ckc_plldiv_get_rate(PLL_2);
		break;
	case PCLKCTRL_SEL_PLL3DIV:
		src_freq = tcc_ckc_plldiv_get_rate(PLL_3);
		break;
	case PCLKCTRL_SEL_PLL4DIV:
		src_freq = tcc_ckc_plldiv_get_rate(PLL_4);
		break;
	case PCLKCTRL_SEL_PCIPHY_CLKOUT:
	case PCLKCTRL_SEL_HDMITMDS:
	case PCLKCTRL_SEL_HDMIPCLK:
	case PCLKCTRL_SEL_HDMIXIN:
	case PCLKCTRL_SEL_XINDIV:
	case PCLKCTRL_SEL_XTINDIV:
	case PCLKCTRL_SEL_EXTCLK0:
	case PCLKCTRL_SEL_EXTCLK1:
	default :
		return 0;
	}

	switch (type) {
	case PCLKCTRL_TYPE_XXX:
		div_mask = PCLKCTRL_DIV_XXX_MASK;
		nPCLKCTRL.md = PCLKCTRL_MODE_DIVIDER;
		break;
	case PCLKCTRL_TYPE_YYY:
		div_mask = PCLKCTRL_DIV_YYY_MASK;
		break;
	default:
		return 0;
	}
	nPCLKCTRL.freq = 0;
	nPCLKCTRL.div = (reg_values&(div_mask<<PCLKCTRL_DIV_SHIFT))>>PCLKCTRL_DIV_SHIFT;
	if (nPCLKCTRL.md == PCLKCTRL_MODE_DIVIDER)
		nPCLKCTRL.freq = src_freq/(nPCLKCTRL.div+1);
	else {
		u64 u64_tmp;
		if (nPCLKCTRL.div > (div_mask+1)/2)
			u64_tmp = (unsigned long long)src_freq*(unsigned long long)((div_mask+1)-nPCLKCTRL.div);
		else
			u64_tmp = (unsigned long long)src_freq*(unsigned long long)nPCLKCTRL.div;
		do_div(u64_tmp ,div_mask+1);
		nPCLKCTRL.freq = (unsigned int)u64_tmp;
	}
	return (unsigned long)nPCLKCTRL.freq;
}

static inline int tcc_ckc_set_audio_pll(unsigned long rate)
{
	void __iomem	*reg = ckc_base+CKC_PLL+audio_pll_ch*4;
	tPMS		nPLL, dco_PLL, div_PLL;
	unsigned int	pll, real_pll, cmp_value, int_tmp, real_pclk, dco_err, div_err;
	u64		u64_tmp;

	memset(&dco_PLL, 0x0, sizeof(tPMS));
	memset(&div_PLL, 0x0, sizeof(tPMS));
	dco_err = 0xFFFFFFFF;
	div_err = 0xFFFFFFFF;

	/* divider mode */
#ifdef HDMIA_SRC_SHOULD_BE_SAME_WITH_AUDIO
	for (int_tmp = (PCLKCTRL_DIV_YYY_MAX+1) ; int_tmp > 1 ; int_tmp--)
#else
	for (int_tmp = (PCLKCTRL_DIV_YYY_MAX+1) ; int_tmp ; int_tmp--)
#endif
	{
		u64_tmp = (u64)int_tmp * (u64)rate;
		if ((u64_tmp > PLL_MAX_RATE) || (u64_tmp < PLL_MIN_RATE))
			continue;
		if (u64_tmp > PCLKCTRL_MAX_FCKS)
			continue;
		nPLL.fpll = (unsigned int)u64_tmp;
		if (tcc_find_pms(&nPLL, XIN_CLK_RATE))
			continue;
		real_pll = nPLL.fpll;
		real_pclk = real_pll/int_tmp;

		if (rate > real_pclk)
			cmp_value = rate - real_pclk;
		else
			cmp_value = real_pclk - rate;

		if (div_err > cmp_value) {
			memcpy(&div_PLL, &nPLL, sizeof(tPMS));
			div_err = cmp_value;
		}

		if (!div_err)
			goto tcc_set_audio_pll_found;
	}

	/* dco mode */
#ifdef HDMIA_SRC_SHOULD_BE_SAME_WITH_AUDIO
	for (int_tmp = (PCLKCTRL_DIV_YYY_MAX+1)/4 ; int_tmp ; int_tmp--)
#else
	for (int_tmp = (PCLKCTRL_DIV_YYY_MAX+1)/2 ; int_tmp ; int_tmp--)
#endif
	{
		u64_tmp = (u64)(PCLKCTRL_DIV_YYY_MAX+1)*(u64)rate;
		do_div(u64_tmp, int_tmp);
		pll = (unsigned int)u64_tmp;
		if (pll > PLL_MAX_RATE)
			break;
		if (pll < PLL_MIN_RATE)
			continue;
		nPLL.fpll = pll;
		if (tcc_find_pms(&nPLL, XIN_CLK_RATE))
			continue;
		real_pll = nPLL.fpll;
		u64_tmp = (u64)real_pll*(u64)int_tmp;
		do_div(u64_tmp, PCLKCTRL_DIV_YYY_MAX+1);
		real_pclk = (unsigned int)u64_tmp;

		if (rate > real_pclk)
			cmp_value = rate - real_pclk;
		else
			cmp_value = real_pclk - rate;

		if (dco_err > cmp_value) {
			memcpy(&dco_PLL, &nPLL, sizeof(tPMS));
			dco_err = cmp_value;
		}

		if (!dco_err)
			break;
	}

	if ((dco_err == 0xFFFFFFFF) && (div_err == 0xFFFFFFFF))
		goto tcc_set_audio_pll_finish;

tcc_set_audio_pll_found:
	(dco_err < div_err) ? memcpy(&nPLL, &dco_PLL, sizeof(tPMS)) : memcpy(&nPLL, &div_PLL, sizeof(tPMS));

	//printk("\n@@@@ %s:, pclk:%d, pll:%d, p:%2d, m:%3d, s:%d @@@@\n\n", __func__, rate, nPLL.fpll, nPLL.p, nPLL.m, nPLL.s);
	tcc_pll_write(reg, nPLL.en, nPLL.p, nPLL.m, nPLL.s, PLLSRC_XIN);
	return nPLL.fpll;

tcc_set_audio_pll_finish:
	tcc_pll_write(reg, 0, PLL_P_MIN, (PLL_P_MIN*PLL_VCO_MIN+XIN_CLK_RATE)/XIN_CLK_RATE, PLL_S_MIN, PLLSRC_XIN);
	return 0;
}

static inline int tcc_find_audio_pclk(tPCLKCTRL *PCLKCTRL)
{
	unsigned int audio_pll, div_max;
	unsigned int err_dco = 0xFFFFFFFF;
	unsigned int err_div = 0xFFFFFFFF;
	unsigned int div_div = PCLKCTRL_DIV_MIN;
	unsigned int div_dco = PCLKCTRL_DIV_DCO_MIN;

#ifdef HDMIA_SRC_SHOULD_BE_SAME_WITH_AUDIO
	if (PCLKCTRL->periname == PERI_HDMI_SPDIFCLK) {
		// hdmi audio clock must not change the pll rate. (hdmi audio = dai*2 or spdif*2)
		if (stHDMIASrc == audio_pll_ch)
			audio_pll = tcc_ckc_pll_get_rate(audio_pll_ch);
		else
			return -1;
	}
	else
#endif
	{
		void __iomem *reg = ckc_base+CKC_PCLKCTRL+(PCLKCTRL->periname)*4;
		PCLKCTRL->en = (ckc_readl(reg) & (1<<PCLKCTRL_EN_SHIFT)) ? 1 : 0;
		tcc_pclkctrl_write(reg, PCLKCTRL_MODE_DIVIDER , PCLKCTRL->en, PCLKCTRL_SEL_XIN, 0, PCLKCTRL_TYPE_YYY);
		audio_pll = tcc_ckc_set_audio_pll(PCLKCTRL->freq);
	}
	div_max = PCLKCTRL_DIV_YYY_MAX;

	/* dco mode */
	err_dco = tcc_ckc_pclk_dco(PCLKCTRL, &div_dco, audio_pll, PCLKCTRL_DIV_DCO_MIN, div_max);

	/* divider mode */
	err_div = tcc_ckc_pclk_divider(PCLKCTRL, &div_div, audio_pll, PCLKCTRL_DIV_MIN+1, div_max+1);

	/* select mode/sel */
	PCLKCTRL->sel = (unsigned int)audio_pll_ch;
	if (err_dco < err_div) {
		u64 u64_tmp;
		PCLKCTRL->md = PCLKCTRL_MODE_DCO;
		PCLKCTRL->div = div_dco;
		if (PCLKCTRL->div > (div_max+1)/2)
			u64_tmp = (unsigned long long)audio_pll*(unsigned long long)((div_max+1)-PCLKCTRL->div);
		else
			u64_tmp = (unsigned long long)audio_pll*(unsigned long long)PCLKCTRL->div;
		do_div(u64_tmp,(div_max+1));
		PCLKCTRL->freq = (unsigned int)u64_tmp;
	}
	else {
		PCLKCTRL->md = PCLKCTRL_MODE_DIVIDER;
		PCLKCTRL->div = div_div ? div_div-1 : div_div;
		PCLKCTRL->freq = (audio_pll/(PCLKCTRL->div+1));
	}

	return 0;
}

static inline void tcc_ckc_reset_clock_source(int id)
{
	if (id >= MAX_CLK_SRC)
		return;

 	if (audio_pll_used && ((int)audio_pll_ch == id)) {
		stClockSource[id] = 0;
		stClockSource[MAX_TCC_PLL+id] = 0;
		return;
	}

	if (id < MAX_TCC_PLL) {
		stClockSource[id] = tcc_ckc_pll_get_rate(id);
		stClockSource[MAX_TCC_PLL+id] = tcc_ckc_plldiv_get_rate(id);
	}
}

static int tcc_vbus_manual_pwdn(unsigned int id, bool pwdn)
{
	void __iomem *reg = pmu_base + PMU_VB_PWRCTRL;
	void __iomem *swrst_reg = ckc_base + CKC_SWRESET;
	unsigned int iso, sca, sc;

	return 0;

	sc = id * 3;
	sca = sc + 1;
	iso = sca + 1;

	if(pwdn) {
		ckc_writel(ckc_readl(reg) | (1 << 31), reg);
		ckc_writel((ckc_readl(reg) & ~((1 << sc) | (1 << sca))) | (1 << iso), reg);
		ckc_writel(ckc_readl(reg) | (1 << sc) | (1 << sca) | (1 << iso), reg);
		ckc_writel(ckc_readl(reg) & ~(1 << 31), reg);
	}
	else {
		ckc_writel(ckc_readl(reg) & ~((1 << sc) | (1 << sca) | (1 << iso)), reg);
		ckc_writel(ckc_readl(reg) & ~(1 << 31), reg);
	}

	if(id == VBUS_HEVC) {
		if(pwdn)
			ckc_writel(ckc_readl(swrst_reg) & ~((1 << 13) | (1 << 14) | (1 << 15)), swrst_reg);
		else
			ckc_writel(ckc_readl(swrst_reg) | (1 << 13) | (1 << 14) | (1 << 15), swrst_reg);
	}
	else if(id == VBUS_CODA) {
		if(pwdn)
			ckc_writel(ckc_readl(swrst_reg) & ~(1 << 12), swrst_reg);
		else
			ckc_writel(ckc_readl(swrst_reg) | (1 << 12), swrst_reg);
	}
	return 0;
}

static int tcc_ckc_iobus_pwdn(int id, bool pwdn)
{
	void __iomem *ckc_reg = ckc_base+CKC_CLKCTRL+(FBUS_IO*0x4);
	void __iomem *reg;

	if ((ckc_readl(ckc_reg) & (1<<CLKCTRL_EN_SHIFT)) == 0)
		return -2;

	if      (id < 32*1)
		reg = iobus_cfg_base+IOBUS_PWDN0;
	else if (id < 32*2)
		reg = iobus_cfg_base+IOBUS_PWDN1;
	else if (id < 32*3)
		reg = iobus_cfg_base+IOBUS_PWDN2;
	else if (id < 32*4)
		reg = iobus_cfg_base1+IOBUS_PWDN3;
	else
		return -1;

	id %= 32;

	if (pwdn)
		ckc_writel(ckc_readl(reg) & ~(1<<id), reg);
	else
		ckc_writel(ckc_readl(reg) | (1<<id), reg);

	return 0;
}

static int tcc_ckc_is_iobus_pwdn(int id)
{
	void __iomem *ckc_reg = ckc_base+CKC_CLKCTRL+(FBUS_IO*0x4);
	void __iomem *reg;

	if ((ckc_readl(ckc_reg) & (1<<CLKCTRL_EN_SHIFT)) == 0)
		return -2;

	if      (id < 32*1)
		reg = iobus_cfg_base+IOBUS_PWDN0;
	else if (id < 32*2)
		reg = iobus_cfg_base+IOBUS_PWDN1;
	else if (id < 32*3)
		reg = iobus_cfg_base+IOBUS_PWDN2;
	else if (id < 32*4)
		reg = iobus_cfg_base1+IOBUS_PWDN3;
	else
		return -1;

	id %= 32;

	return (ckc_readl(reg) & (1<<id)) ? 0 : 1;
}

static int tcc_ckc_iobus_swreset(int id, bool reset)
{
	void __iomem *ckc_reg = ckc_base+CKC_CLKCTRL+(FBUS_IO*0x4);
	void __iomem *reg;

	if ((ckc_readl(ckc_reg) & (1<<CLKCTRL_EN_SHIFT)) == 0)
		return -2;

	if      (id < 32*1)
		reg = iobus_cfg_base+IOBUS_RESET0;
	else if (id < 32*2)
		reg = iobus_cfg_base+IOBUS_RESET1;
	else if (id < 32*3)
		reg = iobus_cfg_base+IOBUS_RESET2;
	else if (id < 32*4)
		reg = iobus_cfg_base1+IOBUS_RESET3;
	else
		return -1;

	id %= 32;

	if (reset)
		ckc_writel(ckc_readl(reg) & ~(1<<id), reg);
	else
		ckc_writel(ckc_readl(reg) | (1<<id), reg);

	return 0;
}

static int tcc_ckc_ddibus_pwdn(int id, bool pwdn)
{
	void __iomem *ckc_reg = ckc_base+CKC_CLKCTRL+(FBUS_DDI*0x4);
	void __iomem *reg = ddibus_cfg_base+DDIBUS_PWDN;

	if (id >= DDIBUS_MAX)
		return -1;

	if ((ckc_readl(ckc_reg) & (1<<CLKCTRL_EN_SHIFT)) == 0)
		return -2;

	if (pwdn)
		ckc_writel(ckc_readl(reg) & ~(1<<id), reg);
	else
		ckc_writel(ckc_readl(reg) | (1<<id), reg);

	return 0;
}

static int tcc_ckc_is_ddibus_pwdn(int id)
{
	void __iomem *ckc_reg = ckc_base+CKC_CLKCTRL+(FBUS_DDI*0x4);
	void __iomem *reg = ddibus_cfg_base+DDIBUS_PWDN;

	if (id >= DDIBUS_MAX)
		return -1;

	if ((ckc_readl(ckc_reg) & (1<<CLKCTRL_EN_SHIFT)) == 0)
		return -2;

	return (ckc_readl(reg) & (0x1 << id)) ? 0 : 1;
}

static int tcc_ckc_ddibus_swreset(int id, bool reset)
{
	void __iomem *ckc_reg = ckc_base+CKC_CLKCTRL+(FBUS_DDI*0x4);
	void __iomem *reg = ddibus_cfg_base+DDIBUS_RESET;

	if (id >= DDIBUS_MAX)
		return -1;

	if ((ckc_readl(ckc_reg) & (1<<CLKCTRL_EN_SHIFT)) == 0)
		return -2;

	if (reset)
		ckc_writel(ckc_readl(reg) & ~(1<<id), reg);
	else
		ckc_writel(ckc_readl(reg) | (1<<id), reg);

	return 0;
}

static int tcc_ckc_vpubus_pwdn(int id, bool pwdn)
{
	void __iomem *ckc_reg = ckc_base+CKC_CLKCTRL+(FBUS_VBUS*0x4);
	void __iomem *reg = vpubus_cfg_base+VPUBUS_PWDN;

	if (id >= VIDEOBUS_MAX)
		return -1;

	if ((ckc_readl(ckc_reg) & (1<<CLKCTRL_EN_SHIFT)) == 0)
		return -2;

	if (pwdn)
		ckc_writel(ckc_readl(reg) & ~(1<<id), reg);
	else
		ckc_writel(ckc_readl(reg) | (1<<id), reg);

	if(id == VIDEOBUS_CODA_CORE)
		tcc_vbus_manual_pwdn(VBUS_CODA, pwdn);
	else if(id == VIDEOBUS_HEVC_CORE)
		tcc_vbus_manual_pwdn(VBUS_HEVC, pwdn);
	else if(id == VIDEOBUS_VP9)
		tcc_vbus_manual_pwdn(VBUS_VP9, pwdn);

	return 0;
}

static int tcc_ckc_is_vpubus_pwdn(int id)
{
	void __iomem *ckc_reg = ckc_base+CKC_CLKCTRL+(FBUS_VBUS*0x4);
	void __iomem *reg = vpubus_cfg_base+VPUBUS_PWDN;

	if (id >= VIDEOBUS_MAX)
		return -1;

	if ((ckc_readl(ckc_reg) & (1<<CLKCTRL_EN_SHIFT)) == 0)
		return -2;

	return (ckc_readl(reg) & (0x1 << id)) ? 0 : 1;
}

static int tcc_ckc_vpubus_swreset(int id, bool reset)
{
	void __iomem *ckc_reg = ckc_base+CKC_CLKCTRL+(FBUS_VBUS*0x4);
	void __iomem *reg = vpubus_cfg_base+VPUBUS_RESET;

	if (id >= VIDEOBUS_MAX)
		return -1;

	if ((ckc_readl(ckc_reg) & (1<<CLKCTRL_EN_SHIFT)) == 0)
		return -2;

	if (reset)
		ckc_writel(ckc_readl(reg) & ~(1<<id), reg);
	else
		ckc_writel(ckc_readl(reg) | (1<<id), reg);	

	return 0;
}

static int tcc_ckc_hsiobus_pwdn(int id, bool pwdn)
{
	void __iomem *ckc_reg = ckc_base+CKC_CLKCTRL+(FBUS_HSIO*0x4);
	void __iomem *reg = hsiobus_cfg_base+HSIOBUS_PWDN;

	if (id >= HSIOBUS_MAX)
		return -1;

	if ((ckc_readl(ckc_reg) & (1<<CLKCTRL_EN_SHIFT)) == 0)
		return -2;

	if (pwdn)
		ckc_writel(ckc_readl(reg) & ~(1<<id), reg);
	else
		ckc_writel(ckc_readl(reg) | (1<<id), reg);

	return 0;
}

static int tcc_ckc_is_hsiobus_pwdn(int id)
{
	void __iomem *ckc_reg = ckc_base+CKC_CLKCTRL+(FBUS_HSIO*0x4);
	void __iomem *reg = hsiobus_cfg_base+HSIOBUS_PWDN;

	if (id >= HSIOBUS_MAX)
		return -1;

	if ((ckc_readl(ckc_reg) & (1<<CLKCTRL_EN_SHIFT)) == 0)
		return -2;

	return (ckc_readl(reg) & (0x1 << id)) ? 0 : 1;
}

static int tcc_ckc_hsiobus_swreset(int id, bool reset)
{
	void __iomem *ckc_reg = ckc_base+CKC_CLKCTRL+(FBUS_HSIO*0x4);
	void __iomem *reg = hsiobus_cfg_base+HSIOBUS_RESET;

	if (id >= HSIOBUS_MAX)
		return -1;

	if ((ckc_readl(ckc_reg) & (1<<CLKCTRL_EN_SHIFT)) == 0)
		return -2;

	if (reset)
		ckc_writel(ckc_readl(reg) & ~(1<<id), reg);
	else
		ckc_writel(ckc_readl(reg) | (1<<id), reg);

	return 0;
}


int tcc_gpu_manual_pwdn(bool pwdn)
{
	void __iomem *gb_pwrctrl_reg = pmu_base + PMU_GB_PWRCTRL;
	void __iomem *gb_isoip_reg = pmu_base + PMU_ISOIP_GB;
	// power down
	if (pwdn) {
		ckc_writel(ckc_readl(gb_isoip_reg) | (1 << 31), gb_isoip_reg);
		ckc_writel(ckc_readl(gb_pwrctrl_reg) | (1 << 31), gb_pwrctrl_reg);
		ckc_writel(ckc_readl(gb_isoip_reg) | (1 << 16), gb_isoip_reg);
		ckc_writel(ckc_readl(gb_pwrctrl_reg) & ~(1<<6), gb_pwrctrl_reg);
		ckc_writel(ckc_readl(gb_pwrctrl_reg) & ~(1<<7), gb_pwrctrl_reg);
		ckc_writel(ckc_readl(gb_pwrctrl_reg) | (1<<8), gb_pwrctrl_reg);
		ckc_writel(ckc_readl(gb_pwrctrl_reg) | (1<<6), gb_pwrctrl_reg);
		ckc_writel(ckc_readl(gb_pwrctrl_reg) | (1<<7), gb_pwrctrl_reg);
		ckc_writel(ckc_readl(gb_isoip_reg) & ~(1 << 31), gb_isoip_reg);
		ckc_writel(ckc_readl(gb_pwrctrl_reg) & ~(1 << 31), gb_pwrctrl_reg);
	}
	// power up
	else {
		ckc_writel(ckc_readl(gb_isoip_reg) | (1 << 31), gb_isoip_reg);
		ckc_writel(ckc_readl(gb_pwrctrl_reg) | (1 << 31), gb_pwrctrl_reg);
		ckc_writel(ckc_readl(gb_pwrctrl_reg) & ~(1<<6), gb_pwrctrl_reg);
		ckc_writel(ckc_readl(gb_pwrctrl_reg) & ~(1<<7), gb_pwrctrl_reg);
		ckc_writel(ckc_readl(gb_pwrctrl_reg) & ~(1<<8), gb_pwrctrl_reg);
		ckc_writel(ckc_readl(gb_isoip_reg) & ~(1 << 16), gb_isoip_reg);
		ckc_writel(ckc_readl(gb_isoip_reg) & ~(1 << 31), gb_isoip_reg);
		ckc_writel(ckc_readl(gb_pwrctrl_reg) & ~(1 << 31), gb_pwrctrl_reg);
	}
	return 0;
}

int tcc_g2d_manual_pwdn(bool pwdn)
{
	void __iomem *gb_pwrctrl_reg = pmu_base + PMU_GB_PWRCTRL;
	void __iomem *gb_isoip_reg = pmu_base + PMU_ISOIP_GB;

	if(pwdn) {
		ckc_writel(ckc_readl(gb_isoip_reg) | (1 << 15), gb_isoip_reg);
		ckc_writel(ckc_readl(gb_pwrctrl_reg) | (1 << 31), gb_pwrctrl_reg);
		ckc_writel(ckc_readl(gb_isoip_reg) | (1 << 0), gb_isoip_reg);
		ckc_writel(ckc_readl(gb_pwrctrl_reg) & ~(1<<3), gb_pwrctrl_reg);
		ckc_writel(ckc_readl(gb_pwrctrl_reg) & ~(1<<4), gb_pwrctrl_reg);
		ckc_writel(ckc_readl(gb_pwrctrl_reg) | (1<<5), gb_pwrctrl_reg);
		ckc_writel(ckc_readl(gb_pwrctrl_reg) | (1<<3), gb_pwrctrl_reg);
		ckc_writel(ckc_readl(gb_pwrctrl_reg) | (1<<4), gb_pwrctrl_reg);
		ckc_writel(ckc_readl(gb_isoip_reg) & ~(1 << 15), gb_isoip_reg);
		ckc_writel(ckc_readl(gb_pwrctrl_reg) & ~(1 << 31), gb_pwrctrl_reg);
	}
	else {
		ckc_writel(ckc_readl(gb_isoip_reg) | (1 << 15), gb_isoip_reg);
		ckc_writel(ckc_readl(gb_pwrctrl_reg) | (1 << 31), gb_pwrctrl_reg);
		ckc_writel(ckc_readl(gb_pwrctrl_reg) & ~(1<<3), gb_pwrctrl_reg);
		ckc_writel(ckc_readl(gb_pwrctrl_reg) & ~(1<<4), gb_pwrctrl_reg);
		ckc_writel(ckc_readl(gb_pwrctrl_reg) & ~(1<<5), gb_pwrctrl_reg);
		ckc_writel(ckc_readl(gb_isoip_reg) & ~(1 << 0), gb_isoip_reg);
		ckc_writel(ckc_readl(gb_isoip_reg) & ~(1 << 15), gb_isoip_reg);
		ckc_writel(ckc_readl(gb_pwrctrl_reg) & ~(1 << 31), gb_pwrctrl_reg);
	}

	return 0;
}

static int tcc_mbus_hclk_ctrl(unsigned int con, unsigned int target, bool set)
{
	void __iomem *reg = membus_cfg_base + con;

	if(reg == NULL) {
		return -1;
	}

	if(set)
		ckc_writel(ckc_readl(reg) | target, reg);
	else
		ckc_writel(ckc_readl(reg) & ~target, reg);

	return 0;
}

static int tcc_mbus_swreset(unsigned int con, unsigned int target, bool set)
{
	void __iomem *reg = membus_cfg_base + con;

	if(reg == NULL) {
		return -1;
	}

	if(set)
		ckc_writel(ckc_readl(reg) | target, reg);
	else
		ckc_writel(ckc_readl(reg) & ~target, reg);

	return 0;
}

static int tcc_ckc_swreset(unsigned int id, bool reset)
{
	void __iomem *reg = ckc_base + CKC_SWRESET;

	if(reg == NULL) {
		return -1;
	}

	if(reset)
		ckc_writel(ckc_readl(reg) & ~id, reg);
	else
		ckc_writel(ckc_readl(reg) | id, reg);

	return 0;
}

static int tcc_dckc_store(unsigned int target) 
{
	void __iomem	*dckc_base;

	switch(target) {
		case FBUS_GPU:
			target = DCKC_GPU;
			dckc_base = gpu_3d_base;
			break;
		case FBUS_G2D:
			target = DCKC_G2D;
			dckc_base = gpu_2d_base;
			break;
		default:
			return -1;
	}
	dckc_backup[target].clkctrl0 = ckc_readl(dckc_base);
	dckc_backup[target].clkctrl1 = ckc_readl(dckc_base+0x04);
	dckc_backup[target].pms = ckc_readl(dckc_base+0x08);
	dckc_backup[target].con = ckc_readl(dckc_base+0x0c);
	dckc_backup[target].mon = ckc_readl(dckc_base+0x10);
	dckc_backup[target].divc = ckc_readl(dckc_base+0x14);

	printk("CTRL : 0x%08x, PMS : 0x%08x , CON : 0x%08x, MON : 0x%08x, DIVC : 0x%08x\n", 
			dckc_backup[target].clkctrl0,
			dckc_backup[target].pms,
			dckc_backup[target].con,
			dckc_backup[target].mon,
			dckc_backup[target].divc);

	return 0;
}

static int tcc_dckc_restore(unsigned int target) 
{
	void __iomem	*dckc_base;

	switch(target) {
		case FBUS_GPU:
			target = DCKC_GPU;
			dckc_base = gpu_3d_base;
			break;
		case FBUS_G2D:
			target = DCKC_G2D;
			dckc_base = gpu_2d_base;
			break;
		default:
			return -1;
	}

	printk("CTRL : 0x%08x, PMS : 0x%08x , CON : 0x%08x, MON : 0x%08x, DIVC : 0x%08x\n", 
			dckc_backup[target].clkctrl0,
			dckc_backup[target].pms,
			dckc_backup[target].con,
			dckc_backup[target].mon,
			dckc_backup[target].divc);

 	//	Set PLL Enable register
	ckc_writel(dckc_backup[target].pms | (1 << 31), dckc_base+0x08);
 	//	Set PLL Divider Enable register
	ckc_writel(dckc_backup[target].divc | (1 << 6), dckc_base+0x14);
	ckc_writel(dckc_backup[target].con, dckc_base+0x0c);
	ckc_writel(dckc_backup[target].mon, dckc_base+0x10);
	ckc_writel(dckc_backup[target].clkctrl0, dckc_base);
	ckc_writel(dckc_backup[target].clkctrl1, dckc_base+0x04);

	return 0;
}

static int tcc_ckc_ip_pwdn(int id, bool pwdn)
{
	void __iomem *reg = pmu_base+PMU_ISOIP_TOP;
	unsigned value;

	if (id >= ISOIP_TOP_MAX)
		return -1;

	value = ckc_readl(reg);
	if (pwdn)
		value |= (1<<id);
	else
		value &= ~(1<<id);
	ckc_writel(value, reg);

	return 0;
}

static int tcc_ckc_is_ip_pwdn(int id)
{
	void __iomem *reg = pmu_base+PMU_ISOIP_TOP;

	if (id >= ISOIP_TOP_MAX)
		return -1;

	return (ckc_readl(reg) & (1<<id)) ? 1 : 0;
}

static int tcc_ckc_isoip_ddi_pwdn(int id, bool pwdn)
{
	void __iomem *reg = pmu_base+PMU_ISOIP_DDI;
	unsigned value;

	if (id >= ISOIP_DDB_MAX)
		return -1;

	value = ckc_readl(reg);
	if (pwdn)
		value |= (1<<id);
	else
		value &= ~(1<<id);
	ckc_writel(value, reg);

	return 0;
}

static int tcc_ckc_is_isoip_ddi_pwdn(int id)
{
	void __iomem *reg = pmu_base+PMU_ISOIP_DDI;

	if (id >= ISOIP_DDB_MAX)
		return -1;

	return (ckc_readl(reg) & (1<<id)) ? 1 : 0;
}

static int tcc_ckc_pmu_pwdn(int id, bool pwdn)
{
	void __iomem *pwrsts_reg = NULL;
	void __iomem *pwrupdn_reg = NULL; 
	void __iomem *pwrdnreq_reg0 = NULL;
	void __iomem *pwrdnreq_reg1 = NULL;
	void __iomem *memsts_reg = membus_cfg_base + MBUS_STS;
	unsigned ckcrst_id = 0, pwrsts_mask=0;
	unsigned memhclk_mask=0, memrst_mask=0, memsts_mask=0;
	unsigned pwrdnreq_mask=0;
	unsigned cpuclk_mask = 0, cpuswrst = 0;

	// Must fix this line when release revision chip(tcc898x)
	switch(id) {
		case FBUS_CPU0:
		case FBUS_CPU1:
		case FBUS_MEM:
			return -1;
		case FBUS_DDI:
			ckcrst_id = (1 << SWRST_DDIBUS);
			pwrsts_mask = PMU_PWSTS_DDIBUS;
			// power down
			if (pwdn) {
				pwrsts_reg = pmu_base + PMU_PWRSTS1;
				pwrupdn_reg = pmu_base + PMU_PWRDN_DDIB;
				pwrdnreq_reg0 = ddibus_cfg_base + DDIBUS_X2XCFG0;
				pwrdnreq_reg1 = ddibus_cfg_base + DDIBUS_X2XCFG1;
				pwrdnreq_mask = DDIBUS_X2XCFG_PWRDNREQN | DDIBUS_X2XCFG_PWRDNACKN;
			}
			// power up
			else {
				pwrsts_reg = pmu_base + PMU_PWRSTS0;
				pwrupdn_reg = pmu_base + PMU_PWRUP_DDIB;
			}
			memhclk_mask = MBUSCLK_DDIBUS_X2X | MBUSCLK_DDIBUS_H2H;
			memrst_mask = memhclk_mask;
			//memsts_mask = MBUS_STS_DDIBUS0 | MBUS_STS_DDIBUS1;
			memsts_mask = MBUS_STS_DDIBUS1;
			break;
		case FBUS_GPU:
			if(tcc_ckc_is_pmu_pwdn(id) && (pwdn == true))
				return 0;
			ckcrst_id = (1 << SWRST_GPU);
			memhclk_mask = MBUSCLK_GPUBUS_X2X | MBUSCLK_GPU_H2H;
			memrst_mask = memhclk_mask;
			memsts_mask = MBUS_STS_GPUBUS;
			pwrdnreq_reg0 = gpu_3d_base + GRAPHIC_PWRDN;
			pwrdnreq_mask = GRAPHIC_PWRDNREQN | GRAPHIC_PWRDNACKN;
			break;
		case FBUS_G2D:
			if(tcc_ckc_is_pmu_pwdn(id) && (pwdn == true))
				return 0;
			ckcrst_id = (1 << SWRST_G2D);
			memhclk_mask = MBUSCLK_G2DBUS_X2X | MBUSCLK_G2D_H2H;
			memrst_mask = memhclk_mask;
			memsts_mask = MBUS_STS_G2DBUS;
			pwrdnreq_reg0 = gpu_2d_base + GRAPHIC_PWRDN;
			pwrdnreq_mask = GRAPHIC_PWRDNREQN | GRAPHIC_PWRDNACKN;
			break;
		case FBUS_IO:
			return -1;
		case FBUS_VBUS:
			ckcrst_id = (1 << SWRST_VBUS) | (1 << SWRST_VCORE);
			pwrsts_mask = PMU_PWSTS_VBUSALL;
			if (pwdn) {
				pwrsts_reg = pmu_base + PMU_PWRSTS1;
				pwrupdn_reg = pmu_base + PMU_PWRDN_VBUSALL;
				pwrdnreq_reg0 = vpubus_cfg_base + VBUS_X2XCFG0;
				pwrdnreq_reg1 = vpubus_cfg_base + VBUS_X2XCFG1;
				pwrdnreq_mask = VBUS_X2XCFG_PWRDNREQN;
			}
			else {
				pwrsts_reg = pmu_base + PMU_PWRSTS0;
				pwrupdn_reg = pmu_base + PMU_PWRUP_VBUSALL;
			}
			memhclk_mask = MBUSCLK_VBUS_X2X | MBUSCLK_VBUS_H2H;
			memrst_mask = memhclk_mask;
			memsts_mask = MBUS_STS_VBUS0 | MBUS_STS_VBUS1;
			break;
		case FBUS_CODA:
			return -1;
		case FBUS_HSIO:
			memhclk_mask = MBUSCLK_HSIOBUS_X2X | MBUSCLK_HSIOBUS_H2H;
			memrst_mask = memhclk_mask;
			memsts_mask = MBUS_STS_HSIOBUS;
			break;
		case FBUS_SMU:
			return -1;
		case FBUS_CMBUS:
			memhclk_mask = MBUSCLK_CMBUS_X2X | MBUSCLK_CMBUS_H2H;
			memrst_mask = memhclk_mask;
			memsts_mask = MBUS_STS_CMBUS;
			break;
		case FBUS_CHEVC:
			//	case FBUS_VHEVC:
			//	case FBUS_BHEVC:
			return -1;
		default:
			return -1;
	}

	/* check pmu main_state. 0x0 is idle */
	while(ckc_readl(pmu_base+PMU_FSMSTS) & PMU_FSMSTS_MAINSTS_Msk);
	
	// power down
	if (pwdn) {
		if(id==FBUS_GPU||id == FBUS_G2D)
			tcc_dckc_store(id);
		if (cpuclk_mask) {
			ckc_writel(ckc_readl(cpubus_cfg_base) & ~(cpuclk_mask), cpubus_cfg_base);
			ckc_writel(ckc_readl(cpubus_cfg_base + 0x4) & ~(cpuswrst), cpubus_cfg_base + 0x04);
		}
		// Disable memory bus clock
		tcc_mbus_hclk_ctrl(MBUSCLK0, memhclk_mask, 0);
		// Check memory bus status
		if (memsts_mask)
			while (ckc_readl(memsts_reg)&memsts_mask);
		// Soft Reset memory bus 
		tcc_mbus_swreset(MBUS_SWRESET0, memrst_mask, 0);
		// Soft Reset clock
		tcc_ckc_swreset(ckcrst_id, true);
		// Request power-down
		if (pwrdnreq_reg0) {
			ckc_writel(ckc_readl(pwrdnreq_reg0) & ~(pwrdnreq_mask), pwrdnreq_reg0);
			// Wait until PWRDNACKN low
			while(ckc_readl(pwrdnreq_reg0) & pwrdnreq_mask);
		}
		if (pwrdnreq_reg1) {
			ckc_writel(ckc_readl(pwrdnreq_reg1) & ~(pwrdnreq_mask), pwrdnreq_reg1);
			while(ckc_readl(pwrdnreq_reg1) & pwrdnreq_mask);
		}
		// Enter power-down
		if (pwrupdn_reg) {
			ckc_writel(1, pwrupdn_reg);
			while((ckc_readl(pwrsts_reg)&pwrsts_mask) == 0);
		}
		if(id == FBUS_GPU)
			tcc_gpu_manual_pwdn(true);
		if(id == FBUS_G2D)
			tcc_g2d_manual_pwdn(true);
	}
	// power up
	else {
		if(id == FBUS_GPU)
			tcc_gpu_manual_pwdn(false);
		else if(id == FBUS_G2D)
			tcc_g2d_manual_pwdn(false);
		// Exit power-down
		if (pwrupdn_reg) {
			ckc_writel(1, pwrupdn_reg);
			while((ckc_readl(pwrsts_reg)&pwrsts_mask) == 0);
		}
		// Soft Not-reset
		tcc_ckc_swreset(ckcrst_id, false);
		// Release reset memory bus
		tcc_mbus_swreset(MBUS_SWRESET0, memrst_mask, 1);
		// Enable memory bus clock
		tcc_mbus_hclk_ctrl(MBUSCLK0, memhclk_mask, 1);
		if (cpuclk_mask) {
			ckc_writel(ckc_readl(cpubus_cfg_base)|(cpuclk_mask), cpubus_cfg_base);
			ckc_writel(ckc_readl(cpubus_cfg_base + 0x4)|(cpuswrst), cpubus_cfg_base + 0x04);

			cpuswrst = 1 << 17;
			ckc_writel(ckc_readl(cpubus_cfg_base + 0x4) & ~(cpuswrst), cpubus_cfg_base + 0x04);
			ckc_writel(ckc_readl(cpubus_cfg_base + 0x4)|(cpuswrst), cpubus_cfg_base + 0x04);
		}
		if((id == FBUS_GPU) || (id == FBUS_G2D))
			tcc_dckc_restore(id);
	}

	return 0;
}



/**********************************
 *  MISC. Functions
 **********************************/
struct ckc_backup_sts {
	unsigned long rate;
	unsigned long en;
};

struct ckc_backup_reg {
	struct ckc_backup_sts	pll[MAX_TCC_PLL];
	struct ckc_backup_sts	clk[FBUS_MAX];
	struct ckc_backup_sts	peri[PERI_MAX];
	unsigned long		display_sub[2];
	unsigned long		io_sub[8];
	unsigned long		hsio_sub[2];
	unsigned long		video_sub[2];
	unsigned long		plldiv[2];
	unsigned long		isoip_pwdn[2];
};

static struct ckc_backup_reg *ckc_backup = NULL;
void tcc_ckc_backup_regs(unsigned int clk_down)
{
	void __iomem	*peri_reg = ckc_base+CKC_PCLKCTRL;
	int i;

	BUG_ON(ckc_backup);
	ckc_backup = kzalloc(sizeof(struct ckc_backup_reg), GFP_KERNEL);
	BUG_ON(!ckc_backup);

	/* PCLKCTRL */
	for (i=0 ; i<PERI_MAX ; i++) {
		ckc_backup->peri[i].rate = tcc_ckc_peri_get_rate(i);
		ckc_backup->peri[i].en = tcc_ckc_is_peri_enabled(i);
		if (clk_down)
			tcc_pclkctrl_write((peri_reg+(i*4)), 0, ckc_backup->peri[i].en, PCLKCTRL_SEL_XIN, 1, 0);
	}

	/* CLKCTRL */
	for (i=0 ; i<FBUS_MAX ; i++) {
		ckc_backup->clk[i].rate = tcc_ckc_clkctrl_get_rate(i);
		ckc_backup->clk[i].en = tcc_ckc_is_clkctrl_enabled(i);
		/* save sub-block pwdn/swreset */
		if (ckc_backup->clk[i].en) {
			switch (i) {
			case FBUS_VBUS:
				ckc_backup->video_sub[0] = ckc_readl(vpubus_cfg_base+VPUBUS_PWDN);
				ckc_backup->video_sub[1] = ckc_readl(vpubus_cfg_base+VPUBUS_RESET);
				break;
			case FBUS_HSIO:
				ckc_backup->hsio_sub[0] = ckc_readl(hsiobus_cfg_base+HSIOBUS_PWDN);
				ckc_backup->hsio_sub[1] = ckc_readl(hsiobus_cfg_base+HSIOBUS_RESET);
				break;
			case FBUS_DDI:
				ckc_backup->display_sub[0] = ckc_readl(ddibus_cfg_base+DDIBUS_PWDN);
				ckc_backup->display_sub[1] = ckc_readl(ddibus_cfg_base+DDIBUS_RESET);
				break;
			case FBUS_IO:
				ckc_backup->io_sub[0] = ckc_readl(iobus_cfg_base+IOBUS_PWDN0);
				ckc_backup->io_sub[1] = ckc_readl(iobus_cfg_base+IOBUS_RESET0);
				ckc_backup->io_sub[2] = ckc_readl(iobus_cfg_base+IOBUS_PWDN1);
				ckc_backup->io_sub[3] = ckc_readl(iobus_cfg_base+IOBUS_RESET1);
				ckc_backup->io_sub[4] = ckc_readl(iobus_cfg_base+IOBUS_PWDN2);
				ckc_backup->io_sub[5] = ckc_readl(iobus_cfg_base+IOBUS_RESET2);
				ckc_backup->io_sub[6] = ckc_readl(iobus_cfg_base1+IOBUS_PWDN3);
				ckc_backup->io_sub[7] = ckc_readl(iobus_cfg_base1+IOBUS_RESET3);
				break;
			}
		}
#if 1
		if ( i==FBUS_CPU0 || i==FBUS_CPU1 || i==FBUS_CBUS || i==FBUS_CMBUS || i==FBUS_MEM || i==FBUS_SMU || i==FBUS_IO || i==FBUS_MEM_PHY)
			;
		else if (clk_down) {
			tcc_ckc_clkctrl_disable(i);
			if (tcc898x_ops.ckc_pmu_pwdn)
				tcc898x_ops.ckc_pmu_pwdn(i, true);
		}
#endif
	}

	/* PLL */
	ckc_backup->plldiv[0] = ckc_readl(ckc_base+CKC_CLKDIVC);
	ckc_backup->plldiv[1] = ckc_readl(ckc_base+CKC_CLKDIVC+0x4);
	for (i=0 ; i<MAX_TCC_PLL ; i++)
	{
		if(i == CLKCTRL_SEL_PLL2 || i == CLKCTRL_SEL_PLL3)
			continue;
		ckc_backup->pll[i].rate = tcc_ckc_pll_get_rate(i);
	}

	/* PMU IP */
	ckc_backup->isoip_pwdn[0] = ckc_readl(pmu_base+PMU_ISOIP_TOP);
	ckc_backup->isoip_pwdn[0] = ckc_readl(pmu_base+PMU_ISOIP_DDI);

//	if (clk_down) {
//		ckc_writel(ckc_backup->isoip_pwdn[0] | 0xFFFFFFF9, pmu_base+PMU_ISOL);	/* phys pwdn except (PLL, RTC) */
//		ckc_writel(ckc_backup->isoip_pwdn[1] | 0xFFFFFFF9, pmu_base+PMU_ISOL);	/* phys pwdn except (PLL, RTC) */
//	}
	return;
}

#ifdef CONFIG_SNAPSHOT_BOOT
extern unsigned int do_hibernate_boot;
#endif//CONFIG_SNAPSHOT_BOOT
void tcc_ckc_restore_regs(void)
{
	void __iomem	*clk_reg = ckc_base+CKC_CLKCTRL;
	int i;

	tcc_clkctrl_write((clk_reg+(FBUS_IO*4)), 1, 1, CLKCTRL_SEL_XIN);
	tcc_clkctrl_write((clk_reg+(FBUS_SMU*4)), 1, 1, CLKCTRL_SEL_XIN);
	tcc_clkctrl_write((clk_reg+(FBUS_HSIO*4)), 1, 1, CLKCTRL_SEL_XIN);
	tcc_clkctrl_write((clk_reg+(FBUS_CMBUS*4)), 1, 1, CLKCTRL_SEL_XIN);

	BUG_ON(!ckc_backup);

	/* PMU IP */
	ckc_writel(ckc_backup->isoip_pwdn[0], pmu_base+PMU_ISOIP_TOP);
	ckc_writel(ckc_backup->isoip_pwdn[1], pmu_base+PMU_ISOIP_DDI);

#ifdef CONFIG_SNAPSHOT_BOOT
	if(!do_hibernate_boot)
#endif
	{
		/* PLL */
		ckc_writel(ckc_backup->plldiv[0], ckc_base+CKC_CLKDIVC);
		ckc_writel(ckc_backup->plldiv[1], ckc_base+CKC_CLKDIVC+0x4);
		for (i=0 ; i<(MAX_TCC_PLL) ; i++)
		{
			if(i == CLKCTRL_SEL_PLL2 || i == CLKCTRL_SEL_PLL3)
				continue;
			tcc_ckc_pll_set_rate(i, ckc_backup->pll[i].rate);
		}
	}

	/* CLKCTRL */
	for (i=0 ; i<FBUS_MAX ; i++) {
		if (i==FBUS_MEM || i==FBUS_MEM_PHY)
			continue;

		if (ckc_backup->clk[i].en) {
			if (tcc898x_ops.ckc_pmu_pwdn)
				tcc898x_ops.ckc_pmu_pwdn(i, false);
			tcc_ckc_clkctrl_enable(i);

			/* restore sub-block pwdn/swreset */
			switch (i) {
			case FBUS_DDI:
				ckc_writel(ckc_backup->display_sub[1], ddibus_cfg_base+DDIBUS_RESET);
				ckc_writel(ckc_backup->display_sub[0], ddibus_cfg_base+DDIBUS_PWDN);
				break;
			case FBUS_IO:
				ckc_writel(ckc_backup->io_sub[1], iobus_cfg_base+IOBUS_RESET0);
				ckc_writel(ckc_backup->io_sub[0], iobus_cfg_base+IOBUS_PWDN0);
				ckc_writel(ckc_backup->io_sub[3], iobus_cfg_base+IOBUS_RESET1);
				ckc_writel(ckc_backup->io_sub[2], iobus_cfg_base+IOBUS_PWDN1);
				ckc_writel(ckc_backup->io_sub[5], iobus_cfg_base+IOBUS_RESET2);
				ckc_writel(ckc_backup->io_sub[4], iobus_cfg_base+IOBUS_PWDN2);
				ckc_writel(ckc_backup->io_sub[7], iobus_cfg_base1+IOBUS_RESET3);
				ckc_writel(ckc_backup->io_sub[6], iobus_cfg_base1+IOBUS_PWDN3);
				break;
			case FBUS_VBUS:
				ckc_writel(ckc_backup->video_sub[1], vpubus_cfg_base+VPUBUS_RESET);
				ckc_writel(ckc_backup->video_sub[0], vpubus_cfg_base+VPUBUS_PWDN);
				break;
			case FBUS_HSIO:
				ckc_writel(ckc_backup->hsio_sub[1], hsiobus_cfg_base+HSIOBUS_RESET);
				ckc_writel(ckc_backup->hsio_sub[0], hsiobus_cfg_base+HSIOBUS_PWDN);
				break;
			}
		}
		else {
			tcc_ckc_clkctrl_disable(i);
			if (tcc898x_ops.ckc_pmu_pwdn)
				tcc898x_ops.ckc_pmu_pwdn(i, true);
		}
		tcc_ckc_clkctrl_set_rate(i, ckc_backup->clk[i].rate);
		if (tcc_ckc_clkctrl_get_rate(i) < ckc_backup->clk[i].rate)
			tcc_ckc_clkctrl_set_rate(i, ckc_backup->clk[i].rate + 1);	/* add 1Hz */
	}

	/* PCLKCTRL */
	for (i=0 ; i<PERI_MAX ; i++) {
#ifdef CONFIG_SNAPSHOT_BOOT
		if(do_hibernate_boot && (i==PERI_LCD0||i==PERI_LCD1))
			continue;
#endif
		tcc_ckc_peri_set_rate(i, ckc_backup->peri[i].rate ? : XIN_CLK_RATE );
		if (ckc_backup->peri[i].en)
			tcc_ckc_peri_enable(i);
		else
			tcc_ckc_peri_disable(i);
	}

	kfree(ckc_backup);
	ckc_backup = NULL;
}

int tcc_ckc_set_hdmi_audio_src(unsigned int src_id)
{
#ifdef HDMIA_SRC_SHOULD_BE_SAME_WITH_AUDIO
	void __iomem *reg = ckc_base+CKC_PCLKCTRL+(src_id*4);
	unsigned int sel;

	if (src_id != PERI_MDAI0 && src_id != PERI_MSPDIF0 &&
	    src_id != PERI_MDAI1 && src_id != PERI_MSPDIF1) {
		stHDMIASrc = 0xFFFFFFFF;
		return -1;
	}

	sel = (ckc_readl(reg)&(PCLKCTRL_SEL_MASK<<PCLKCTRL_SEL_SHIFT))>>PCLKCTRL_SEL_SHIFT;
	switch (sel) {
		case PCLKCTRL_SEL_PLL0:
		case PCLKCTRL_SEL_PLL1:
		case PCLKCTRL_SEL_PLL2:
		case PCLKCTRL_SEL_PLL3:
		case PCLKCTRL_SEL_PLL4:
			stHDMIASrc = sel;
			break;
		case PCLKCTRL_SEL_XIN:
		case PCLKCTRL_SEL_XTIN:
			stHDMIASrc = sel + MAX_TCC_PLL;
			break;
		case PCLKCTRL_SEL_PLL0DIV:
		case PCLKCTRL_SEL_PLL1DIV:
		case PCLKCTRL_SEL_PLL2DIV:
		case PCLKCTRL_SEL_PLL3DIV:
		case PCLKCTRL_SEL_PLL4DIV:
			stHDMIASrc = sel;
			break;
		default:
			stHDMIASrc = 0xFFFFFFFF;
	}

	return 0;
#else
	return -1;
#endif
}


static struct tcc_ckc_ops tcc898x_ops = {
	.ckc_pmu_pwdn			= NULL, //&tcc_ckc_pmu_pwdn,
	.ckc_is_pmu_pwdn		= NULL, //&tcc_ckc_is_pmu_pwdn,
	.ckc_swreset			= NULL,
	.ckc_isoip_top_pwdn		= &tcc_ckc_ip_pwdn,
	.ckc_is_isoip_top_pwdn		= &tcc_ckc_is_ip_pwdn,
	.ckc_isoip_ddi_pwdn		= &tcc_ckc_isoip_ddi_pwdn,
	.ckc_is_isoip_ddi_pwdn		= &tcc_ckc_is_isoip_ddi_pwdn,
	.ckc_pll_set_rate		= &tcc_ckc_pll_set_rate,
	.ckc_pll_get_rate		= &tcc_ckc_pll_get_rate,
	.ckc_clkctrl_enable		= &tcc_ckc_clkctrl_enable,
	.ckc_clkctrl_disable		= &tcc_ckc_clkctrl_disable,
	.ckc_clkctrl_set_rate		= &tcc_ckc_clkctrl_set_rate,
	.ckc_clkctrl_get_rate		= &tcc_ckc_clkctrl_get_rate,
	.ckc_is_clkctrl_enabled		= &tcc_ckc_is_clkctrl_enabled,
	.ckc_peri_enable		= &tcc_ckc_peri_enable,
	.ckc_peri_disable		= &tcc_ckc_peri_disable,
	.ckc_peri_set_rate		= &tcc_ckc_peri_set_rate,
	.ckc_peri_get_rate		= &tcc_ckc_peri_get_rate,
	.ckc_is_peri_enabled		= &tcc_ckc_is_peri_enabled,
	.ckc_ddibus_pwdn		= &tcc_ckc_ddibus_pwdn,
	.ckc_is_ddibus_pwdn		= &tcc_ckc_is_ddibus_pwdn,
	.ckc_ddibus_swreset		= &tcc_ckc_ddibus_swreset,
	.ckc_gpubus_pwdn		= NULL,
	.ckc_is_gpubus_pwdn		= NULL,
	.ckc_gpubus_swreset		= NULL,
	.ckc_iobus_pwdn			= &tcc_ckc_iobus_pwdn,
	.ckc_is_iobus_pwdn		= &tcc_ckc_is_iobus_pwdn,
	.ckc_iobus_swreset		= &tcc_ckc_iobus_swreset,
	.ckc_vpubus_pwdn		= &tcc_ckc_vpubus_pwdn,
	.ckc_is_vpubus_pwdn		= &tcc_ckc_is_vpubus_pwdn,
	.ckc_vpubus_swreset		= &tcc_ckc_vpubus_swreset,
	.ckc_hsiobus_pwdn		= &tcc_ckc_hsiobus_pwdn,
	.ckc_is_hsiobus_pwdn		= &tcc_ckc_is_hsiobus_pwdn,
	.ckc_hsiobus_swreset		= &tcc_ckc_hsiobus_swreset,
	.ckc_g2dbus_pwdn		= NULL,
	.ckc_is_g2dbus_pwdn		= NULL,
	.ckc_g2dbus_swreset		= NULL,
	.ckc_cmbus_pwdn			= NULL,
	.ckc_is_cmbus_pwdn		= NULL,
	.ckc_cmbus_swreset		= NULL,
};


void __init tcc898x_ckc_init(struct device_node *np)
{
	int i;
	unsigned clk_src=0;

	ckc_base = of_iomap(np, 0);
	BUG_ON(!ckc_base);
	pmu_base = of_iomap(np, 1);
	BUG_ON(!pmu_base);
	membus_cfg_base = of_iomap(np, 2);
	BUG_ON(!membus_cfg_base);
	ddibus_cfg_base = of_iomap(np, 3);
	BUG_ON(!ddibus_cfg_base);
	iobus_cfg_base = of_iomap(np, 5);
	BUG_ON(!iobus_cfg_base);
	iobus_cfg_base1 = of_iomap(np, 6);
	BUG_ON(!iobus_cfg_base1);
	vpubus_cfg_base = of_iomap(np, 7);
	BUG_ON(!vpubus_cfg_base);
	hsiobus_cfg_base = of_iomap(np, 8);
	BUG_ON(!hsiobus_cfg_base);
	cpubus_cfg_base = of_iomap(np, 9);
	BUG_ON(!cpubus_cfg_base);
	gpu_3d_base = of_iomap(np, 10);
	BUG_ON(!gpu_3d_base);
	gpu_2d_base = of_iomap(np, 11);
	BUG_ON(!gpu_2d_base);
	mem_ckc_base = of_iomap(np, 12);
	BUG_ON(!mem_ckc_base);

	cpu0_base = cpubus_cfg_base + 0x00110000;
	cpu1_base = cpubus_cfg_base + 0x00210000;

	audio_pll_used = of_property_read_bool(np, "audio-pll-use");
	if (audio_pll_used) {
		BUG_ON(of_property_read_u32(np, "audio-pll-ch", &audio_pll_ch));

		if (audio_pll_ch > 2) {
			printk("Can not set audio pll to channel %d\n", audio_pll_ch);
			audio_pll_used = false;
		}
	}

	/*
	 * get public PLLs, XIN and XTIN rates
	 */
	for (i=0 ; i<MAX_TCC_PLL ; i++)
		tcc_ckc_reset_clock_source(i);
	stClockSource[MAX_TCC_PLL*2] = XIN_CLK_RATE;	/* XIN */
	stClockSource[MAX_TCC_PLL*2+1] = XTIN_CLK_RATE;	/* XTIN */

	/*
	 * check clock source (change clock source when it's source is audio pll)
	 */
	if (audio_pll_used) {
		for (i=0 ; i<FBUS_MAX ; i++) {
			clk_src = ckc_readl(ckc_base + 0x4*i)&CLKCTRL_SEL_MASK;
			if ((clk_src == audio_pll_ch) || (clk_src == audio_pll_ch+CLKCTRL_SEL_PLL0DIV)
				|| (clk_src == audio_pll_ch+CLKCTRL_SEL_PLL2DIV))
				tcc_ckc_clkctrl_set_rate(i, tcc_ckc_clkctrl_get_rate(i));
		}
		for (i=0 ; i<PERI_MAX ; i++) {
			clk_src = (ckc_readl(ckc_base + 0x4*i + CKC_PCLKCTRL)>>PCLKCTRL_SEL_SHIFT)&PCLKCTRL_SEL_MASK;
			if ((clk_src == audio_pll_ch) || (clk_src == audio_pll_ch+PCLKCTRL_SEL_PLL0DIV))
				tcc_ckc_peri_set_rate(i, tcc_ckc_peri_get_rate(i));
		}

		/* disable pll_div */
		switch (audio_pll_ch) {
		case 0:
			ckc_writel(ckc_readl(ckc_base + CKC_CLKDIVC) & ~(1<<31), ckc_base + CKC_CLKDIVC);
			break;
		case 1:
			ckc_writel(ckc_readl(ckc_base + CKC_CLKDIVC) & ~(1<<23), ckc_base + CKC_CLKDIVC);
			break;
		case 2:
			ckc_writel(ckc_readl(ckc_base + CKC_CLKDIVC) & ~(1<<15), ckc_base + CKC_CLKDIVC);
			break;
		case 3:
			ckc_writel(ckc_readl(ckc_base + CKC_CLKDIVC) & ~(1<<7), ckc_base + CKC_CLKDIVC);
			break;
		case 4:
			ckc_writel(ckc_readl(ckc_base + CKC_CLKDIVC + 0x4) & ~(1<<23), ckc_base + CKC_CLKDIVC + 0x4);
			break;
		}
	}

	for (i=0 ; i<MAX_TCC_PLL ; i++) {
		if (stClockSource[i])
			printk("PLL_%d    : %10u Hz\n", i, stClockSource[i]);
	}
	for ( ; i<(MAX_TCC_PLL*2) ; i++) {
		if (stClockSource[i])
			printk("PLLDIV_%d : %10u Hz\n", i-MAX_TCC_PLL, stClockSource[i]);
	}
	printk("XIN      : %10u Hz\n", stClockSource[i++]);
	printk("XTIN     : %10u Hz\n", stClockSource[i++]);

	tcc_ckc_set_ops(&tcc898x_ops);	
	tcc_ckctest_set_ops(&tcc898x_ops);	

	tcc_dckc_store(FBUS_GPU);
	tcc_dckc_store(FBUS_G2D);
}

CLOCKSOURCE_OF_DECLARE(tcc_ckc, "telechips,ckc", tcc898x_ckc_init);

#include <linux/proc_fs.h>
#include <linux/seq_file.h>

static void *tcc_clk_start(struct seq_file *m, loff_t *pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}

static void *tcc_clk_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}

static void tcc_clk_stop(struct seq_file *m, void *v)
{
}

static int tcc_clk_show(struct seq_file *m, void *v)
{
	seq_printf(m, "cpu(a53_mp) : %10lu Hz %s\n", tcc_ckc_clkctrl_get_rate(FBUS_CPU0),
		tcc_ckc_is_clkctrl_enabled(FBUS_CPU0)?"":"(disabled)");
//	seq_printf(m, "cpu(a7_sp)  : %10lu Hz %s\n", tcc_ckc_clkctrl_get_rate(FBUS_CPU1),
//		tcc_ckc_is_clkctrl_enabled(FBUS_CPU1)?"":"(disabled)");
	seq_printf(m, "cpu(m4)     : %10lu Hz %s\n", tcc_ckc_clkctrl_get_rate(FBUS_CMBUS),
		tcc_ckc_is_clkctrl_enabled(FBUS_CMBUS)?"":"(disabled)");
	seq_printf(m, "cpu bus     : %10lu Hz %s\n", tcc_ckc_clkctrl_get_rate(FBUS_CBUS),
		tcc_ckc_is_clkctrl_enabled(FBUS_CBUS)?"":"(disabled)");
	seq_printf(m, "memory bus  : %10lu Hz %s\n", tcc_ckc_clkctrl_get_rate(FBUS_MEM),
		tcc_ckc_is_clkctrl_enabled(FBUS_MEM)?"":"(disabled)");
	seq_printf(m, "memory phy  : %10lu Hz %s\n", tcc_ckc_clkctrl_get_rate(FBUS_MEM_PHY),
		tcc_ckc_is_clkctrl_enabled(FBUS_MEM_PHY)?"":"(disabled)");
	seq_printf(m, "smu bus     : %10lu Hz %s\n", tcc_ckc_clkctrl_get_rate(FBUS_SMU),
		tcc_ckc_is_clkctrl_enabled(FBUS_SMU)?"":"(disabled)");
	seq_printf(m, "i/o bus     : %10lu Hz %s\n", tcc_ckc_clkctrl_get_rate(FBUS_IO),
		tcc_ckc_is_clkctrl_enabled(FBUS_IO)?"":"(disabled)");
	seq_printf(m, "hsio bus    : %10lu Hz %s\n", tcc_ckc_clkctrl_get_rate(FBUS_HSIO),
		tcc_ckc_is_clkctrl_enabled(FBUS_HSIO)?"":"(disabled)");
	seq_printf(m, "display bus : %10lu Hz %s\n", tcc_ckc_clkctrl_get_rate(FBUS_DDI),
		tcc_ckc_is_clkctrl_enabled(FBUS_DDI)?"":"(disabled)");
	seq_printf(m, "graphic 3d  : %10lu Hz %s\n", tcc_ckc_clkctrl_get_rate(FBUS_GPU),
		tcc_ckc_is_clkctrl_enabled(FBUS_GPU)?"":"(disabled)");
	seq_printf(m, "graphic 2d  : %10lu Hz %s\n", tcc_ckc_clkctrl_get_rate(FBUS_G2D),
		tcc_ckc_is_clkctrl_enabled(FBUS_G2D)?"":"(disabled)");
	seq_printf(m, "video bus   : %10lu Hz %s\n", tcc_ckc_clkctrl_get_rate(FBUS_VBUS),
		tcc_ckc_is_clkctrl_enabled(FBUS_VBUS)?"":"(disabled)");
	seq_printf(m, "codec(CODA) : %10lu Hz %s\n", tcc_ckc_clkctrl_get_rate(FBUS_CODA),
		tcc_ckc_is_clkctrl_enabled(FBUS_CODA)?"":"(disabled)");
	seq_printf(m, "hevc_c      : %10lu Hz %s\n", tcc_ckc_clkctrl_get_rate(FBUS_CHEVC),
		tcc_ckc_is_clkctrl_enabled(FBUS_CHEVC)?"":"(disabled)");
	seq_printf(m, "hevc_v      : %10lu Hz %s\n", tcc_ckc_clkctrl_get_rate(FBUS_VHEVC),
		tcc_ckc_is_clkctrl_enabled(FBUS_VHEVC)?"":"(disabled)");
	seq_printf(m, "hevc_b      : %10lu Hz %s\n", tcc_ckc_clkctrl_get_rate(FBUS_BHEVC),
		tcc_ckc_is_clkctrl_enabled(FBUS_BHEVC)?"":"(disabled)");
	return 0;
}

static const struct seq_operations tcc_clk_op = {
	.start	= tcc_clk_start,
	.next	= tcc_clk_next,
	.stop	= tcc_clk_stop,
	.show	= tcc_clk_show
};

static int tcc_clk_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &tcc_clk_op);
}

static const struct file_operations proc_tcc_clk_operations = {
	.open		= tcc_clk_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

static int __init tcc_clk_proc_init(void)
{
	proc_create("clocks", 0, NULL, &proc_tcc_clk_operations);
	return 0;
}

__initcall(tcc_clk_proc_init);
