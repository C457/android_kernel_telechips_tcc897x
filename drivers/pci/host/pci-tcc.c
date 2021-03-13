/*
 * PCIe host controller driver for Telechips SoCs
 *
 * Copyright (C) 2016 Telechips Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
//#include <linux/kernel.h>
//#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
//#include <linux/signal.h>
//#include <linux/types.h>

#include "pcie-designware.h"

#define to_tcc_pcie(x)	container_of(x, struct tcc_pcie, pp)

struct tcc_pcie {
	void __iomem		*phy_base;
	void __iomem		*cfg_base;
	int			reset_gpio;
	struct clk		*clk_aux;
	struct clk		*clk_apb;
	struct clk		*clk_ref_ext;
	struct clk		*clk_hsio;
	struct clk		*clk_phy;
	struct pcie_port	pp;
};

/* PCIe PHY registers */
#define PCIE_PHY_IMPEDANCE		0x004
#define PCIE_PHY_PLL_DIV_0		0x008
#define PCIE_PHY_PLL_BIAS		0x00c
#define PCIE_PHY_DCC_FEEDBACK		0x014
#define PCIE_PHY_PLL_DIV_1		0x05c
#define PCIE_PHY_COMMON_POWER		0x064
#define PCIE_PHY_COMMON_PD_CMN		(0x1 << 3)
#define PCIE_PHY_TRSV0_EMP_LVL		0x084
#define PCIE_PHY_TRSV0_DRV_LVL		0x088
#define PCIE_PHY_TRSV0_RXCDR		0x0ac
#define PCIE_PHY_TRSV0_POWER		0x0c4
#define PCIE_PHY_TRSV0_PD_TSV		(0x1 << 7)
#define PCIE_PHY_TRSV0_LVCC		0x0dc
#define PCIE_PHY_TRSV1_EMP_LVL		0x144
#define PCIE_PHY_TRSV1_RXCDR		0x16c
#define PCIE_PHY_TRSV1_POWER		0x184
#define PCIE_PHY_TRSV1_PD_TSV		(0x1 << 7)
#define PCIE_PHY_TRSV1_LVCC		0x19c
#define PCIE_PHY_TRSV2_EMP_LVL		0x204
#define PCIE_PHY_TRSV2_RXCDR		0x22c
#define PCIE_PHY_TRSV2_POWER		0x244
#define PCIE_PHY_TRSV2_PD_TSV		(0x1 << 7)
#define PCIE_PHY_TRSV2_LVCC		0x25c
#define PCIE_PHY_TRSV3_EMP_LVL		0x2c4
#define PCIE_PHY_TRSV3_RXCDR		0x2ec
#define PCIE_PHY_TRSV3_POWER		0x304
#define PCIE_PHY_TRSV3_PD_TSV		(0x1 << 7)
#define PCIE_PHY_TRSV3_LVCC		0x31c

/* PCIe CFG registers */
#define PCIE_CFG00			0x000
#define PCIE_CFG00_VEN_MSG_FMT		(1<<26)
#define PCIE_CFG01			0x004
#define PCIE_CFG02			0x008
#define PCIE_CFG03			0x00c
#define PCIE_CFG04			0x010
#define PCIE_CFG04_APP_LTSSM_ENABLE	(1<<2)
#define PCIE_CFG04_APP_INIT_RST		(1<<0)
#define PCIE_CFG05			0x014
#define PCIE_CFG06			0x018
#define PCIE_ELBI_SLV_DBI_ENABLE	(1<<21)
#define PCIE_CFG07			0x01c
#define PCIE_CFG08			0x020
#define PCIE_CFG08_PHY_PLL_LOCKED	(1<<1)
#define PCIE_CFG09			0x024
#define PCIE_CFG10			0x028
#define PCIE_CFG11			0x02c
#define PCIE_CFG12			0x030
#define PCIE_CFG13			0x034
#define PCIE_CFG14			0x038
#define PCIE_CFG15			0x03c
#define PCIE_CFG16			0x040
#define PCIE_CFG17			0x044
#define PCIE_CFG18			0x048
#define PCIE_CFG19			0x04c
#define PCIE_CFG20			0x050
#define PCIE_CFG21			0x054
#define PCIE_CFG22			0x058
#define PCIE_CFG23			0x05c
#define PCIE_CFG24			0x060
#define PCIE_CFG25			0x064
#define PCIE_CFG26			0x068
#define PCIE_CFG26_RDLH_LINK_UP		(1<<22)
#define PCIE_CFG27			0x06c
#define PCIE_CFG28			0x070
#define PCIE_CFG29			0x074
#define PCIE_CFG30			0x078
#define PCIE_CFG31			0x07c
#define PCIE_CFG32			0x080
#define PCIE_CFG33			0x084
#define PCIE_CFG34			0x088
#define PCIE_CFG35			0x08c
#define PCIE_CFG36			0x090
#define PCIE_CFG37			0x094
#define PCIE_CFG38			0x098
#define PCIE_CFG39			0x09c
#define PCIE_CFG40			0x0a0
#define PCIE_CFG41			0x0a4
#define PCIE_CFG42			0x0a8
#define PCIE_CFG43			0x0ac
#define PCIE_CFG44			0x0b0
#define PCIE_CFG44_CFG_POWER_UP_RST	(1<<6)
#define PCIE_CFG44_CFG_PERST		(1<<5)
#define PCIE_CFG44_PHY_CMN_REG_RST	(1<<4)
#define PCIE_CFG44_PHY_CMN_RST		(1<<3)
#define PCIE_CFG44_PHY_GLOBAL_RST	(1<<2)
#define PCIE_CFG44_PHY_TRSV_REG_RST	(1<<1)
#define PCIE_CFG44_PHY_TRSV_RST		(1<<0)
#define PCIE_CFG45			0x0b4

#define PCIE_PHY_TRSVRST		0x108

static unsigned int pcie_i2c_full_reg[] = {
	0x01, 0x00, 0x24, 0xe3, 0xb9, 0x69, 0x94, 0x19,
	0x66, 0x13, 0x40, 0x1f, 0x38, 0x69, 0xfc, 0x00,
	0x3b, 0xff, 0x0d, 0x00, 0x00, 0x04, 0x01, 0x04,
	0x00, 0x43, 0x00,
};

static unsigned int pcie_phy_full_reg[] = {
	0x21, 0x16, 0x6a, 0xfc, 0x48, 0x42, 0xbc, 0x20,
	0xc4, 0x0c, 0x00, 0x81, 0x42, 0x00, 0x00, 0x0c,
	0x40, 0x35, 0xfa, 0x20, 0x87, 0x3f, 0xb0, 0xa0,
	0x00, 0x00, 0x08, 0x42, 0x00, 0x32, 0x04, 0x09,
	0x00, 0x12, 0x00,
};

static inline void tcc_phy_writel(struct tcc_pcie *pcie, u32 val, u32 reg)
{
	writel(val, pcie->phy_base + reg);
}

static inline u32 tcc_phy_readl(struct tcc_pcie *pcie, u32 reg)
{
	return readl(pcie->phy_base + reg);
}

static inline void tcc_cfg_write(struct tcc_pcie *pcie, u32 val, u32 reg, u32 mask)
{
	if (val)
		writel(readl(pcie->cfg_base + reg) | mask, pcie->cfg_base + reg);
	else
		writel(readl(pcie->cfg_base + reg) & ~mask, pcie->cfg_base + reg);
}

static inline u32 tcc_cfg_read(struct tcc_pcie *pcie, u32 reg, u32 mask)
{
	return readl(pcie->cfg_base + reg) & mask;
}

static void tcc_pcie_assert_core_reset(struct pcie_port *pp)
{
	u32 val;
	struct tcc_pcie *tcc_pcie = to_tcc_pcie(pp);

	// Default value of ven_msg_fmt is wrong. must set to 0
	tcc_cfg_write(tcc_pcie, 0, PCIE_CFG00, PCIE_CFG00_VEN_MSG_FMT);

	tcc_cfg_write(tcc_pcie, 0, PCIE_CFG44, PCIE_CFG44_CFG_POWER_UP_RST);

	for (val = 1 ; val < ARRAY_SIZE(pcie_i2c_full_reg) ; val++) {
		if (val == 0xd)
			continue;
		tcc_phy_writel(tcc_pcie, pcie_i2c_full_reg[val], val*4);
	}

	for (val = 1 ; val < ARRAY_SIZE(pcie_phy_full_reg) ; val++)
		tcc_phy_writel(tcc_pcie, pcie_phy_full_reg[val], val*4 + 0x20);
}

static void tcc_pcie_deassert_core_reset(struct pcie_port *pp)
{
	struct tcc_pcie *tcc_pcie = to_tcc_pcie(pp);

	tcc_cfg_write(tcc_pcie, 1, PCIE_CFG44, PCIE_CFG44_CFG_POWER_UP_RST);
	tcc_cfg_write(tcc_pcie, 1, PCIE_CFG04, PCIE_CFG04_APP_INIT_RST);
	tcc_cfg_write(tcc_pcie, 0, PCIE_CFG04, PCIE_CFG04_APP_INIT_RST);
}

static void tcc_pcie_assert_phy_reset(struct pcie_port *pp)
{
	struct tcc_pcie *tcc_pcie = to_tcc_pcie(pp);

	tcc_cfg_write(tcc_pcie, 0, PCIE_CFG44, PCIE_CFG44_PHY_TRSV_RST);
	tcc_cfg_write(tcc_pcie, 0, PCIE_CFG44, PCIE_CFG44_PHY_TRSV_REG_RST);
	tcc_cfg_write(tcc_pcie, 0, PCIE_CFG44, PCIE_CFG44_PHY_CMN_REG_RST);
	tcc_cfg_write(tcc_pcie, 0, PCIE_CFG44, PCIE_CFG44_PHY_CMN_RST);
	tcc_cfg_write(tcc_pcie, 0, PCIE_CFG44, PCIE_CFG44_CFG_PERST);
	tcc_cfg_write(tcc_pcie, 0, PCIE_CFG44, PCIE_CFG44_PHY_GLOBAL_RST);
}

static void tcc_pcie_deassert_phy_reset(struct pcie_port *pp)
{
	struct tcc_pcie *tcc_pcie = to_tcc_pcie(pp);

	tcc_cfg_write(tcc_pcie, 1, PCIE_CFG44, PCIE_CFG44_PHY_GLOBAL_RST);
	tcc_cfg_write(tcc_pcie, 1, PCIE_CFG44, PCIE_CFG44_CFG_PERST);
	tcc_cfg_write(tcc_pcie, 1, PCIE_CFG44, PCIE_CFG44_PHY_CMN_RST);
	tcc_cfg_write(tcc_pcie, 1, PCIE_CFG44, PCIE_CFG44_PHY_CMN_REG_RST);
	tcc_cfg_write(tcc_pcie, 1, PCIE_CFG44, PCIE_CFG44_PHY_TRSV_REG_RST);
	tcc_cfg_write(tcc_pcie, 1, PCIE_CFG44, PCIE_CFG44_PHY_TRSV_RST);
}

static void tcc_pcie_power_on_phy(struct pcie_port *pp)
{
	u32 val;
	struct tcc_pcie *tcc_pcie = to_tcc_pcie(pp);

	val = tcc_phy_readl(tcc_pcie, PCIE_PHY_COMMON_POWER);
	val &= ~PCIE_PHY_COMMON_PD_CMN;
	tcc_phy_writel(tcc_pcie, val, PCIE_PHY_COMMON_POWER);

	val = tcc_phy_readl(tcc_pcie, PCIE_PHY_TRSV0_POWER);
	val &= ~PCIE_PHY_TRSV0_PD_TSV;
	tcc_phy_writel(tcc_pcie, val, PCIE_PHY_TRSV0_POWER);

	val = tcc_phy_readl(tcc_pcie, PCIE_PHY_TRSV1_POWER);
	val &= ~PCIE_PHY_TRSV1_PD_TSV;
	tcc_phy_writel(tcc_pcie, val, PCIE_PHY_TRSV1_POWER);

	val = tcc_phy_readl(tcc_pcie, PCIE_PHY_TRSV2_POWER);
	val &= ~PCIE_PHY_TRSV2_PD_TSV;
	tcc_phy_writel(tcc_pcie, val, PCIE_PHY_TRSV2_POWER);

	val = tcc_phy_readl(tcc_pcie, PCIE_PHY_TRSV3_POWER);
	val &= ~PCIE_PHY_TRSV3_PD_TSV;
	tcc_phy_writel(tcc_pcie, val, PCIE_PHY_TRSV3_POWER);
}

static void tcc_pcie_power_off_phy(struct pcie_port *pp)
{
	u32 val;
	struct tcc_pcie *tcc_pcie = to_tcc_pcie(pp);

	return;

	val = tcc_phy_readl(tcc_pcie, PCIE_PHY_COMMON_POWER);
	val |= PCIE_PHY_COMMON_PD_CMN;
	tcc_phy_writel(tcc_pcie, val, PCIE_PHY_COMMON_POWER);

	val = tcc_phy_readl(tcc_pcie, PCIE_PHY_TRSV0_POWER);
	val |= PCIE_PHY_TRSV0_PD_TSV;
	tcc_phy_writel(tcc_pcie, val, PCIE_PHY_TRSV0_POWER);

	val = tcc_phy_readl(tcc_pcie, PCIE_PHY_TRSV1_POWER);
	val |= PCIE_PHY_TRSV1_PD_TSV;
	tcc_phy_writel(tcc_pcie, val, PCIE_PHY_TRSV1_POWER);

	val = tcc_phy_readl(tcc_pcie, PCIE_PHY_TRSV2_POWER);
	val |= PCIE_PHY_TRSV2_PD_TSV;
	tcc_phy_writel(tcc_pcie, val, PCIE_PHY_TRSV2_POWER);

	val = tcc_phy_readl(tcc_pcie, PCIE_PHY_TRSV3_POWER);
	val |= PCIE_PHY_TRSV3_PD_TSV;
	tcc_phy_writel(tcc_pcie, val, PCIE_PHY_TRSV3_POWER);
}

static void tcc_pcie_init_phy(struct pcie_port *pp)
{
	struct tcc_pcie *tcc_pcie = to_tcc_pcie(pp);

	/* DCC feedback control off */
	tcc_phy_writel(tcc_pcie, 0x29, PCIE_PHY_DCC_FEEDBACK);

	/* set TX/RX impedance */
	tcc_phy_writel(tcc_pcie, 0xd5, PCIE_PHY_IMPEDANCE);

	/* set 50Mhz PHY clock */
//	tcc_phy_writel(tcc_pcie, 0x14, PCIE_PHY_PLL_DIV_0);
//	tcc_phy_writel(tcc_pcie, 0x12, PCIE_PHY_PLL_DIV_1);

	/* set TX Differential output for lane 0 */
	tcc_phy_writel(tcc_pcie, 0x7f, PCIE_PHY_TRSV0_DRV_LVL);

	/* set TX Pre-emphasis Level Control for lane 0 to minimum */
	tcc_phy_writel(tcc_pcie, 0x0, PCIE_PHY_TRSV0_EMP_LVL);

	/* set RX clock and data recovery bandwidth */
	tcc_phy_writel(tcc_pcie, 0xe7, PCIE_PHY_PLL_BIAS);
	tcc_phy_writel(tcc_pcie, 0x82, PCIE_PHY_TRSV0_RXCDR);
	tcc_phy_writel(tcc_pcie, 0x82, PCIE_PHY_TRSV1_RXCDR);
	tcc_phy_writel(tcc_pcie, 0x82, PCIE_PHY_TRSV2_RXCDR);
	tcc_phy_writel(tcc_pcie, 0x82, PCIE_PHY_TRSV3_RXCDR);

	/* change TX Pre-emphasis Level Control for lanes */
	tcc_phy_writel(tcc_pcie, 0x39, PCIE_PHY_TRSV0_EMP_LVL);
	tcc_phy_writel(tcc_pcie, 0x39, PCIE_PHY_TRSV1_EMP_LVL);
	tcc_phy_writel(tcc_pcie, 0x39, PCIE_PHY_TRSV2_EMP_LVL);
	tcc_phy_writel(tcc_pcie, 0x39, PCIE_PHY_TRSV3_EMP_LVL);

	/* set LVCC */
	tcc_phy_writel(tcc_pcie, 0x20, PCIE_PHY_TRSV0_LVCC);
	tcc_phy_writel(tcc_pcie, 0xa0, PCIE_PHY_TRSV1_LVCC);
	tcc_phy_writel(tcc_pcie, 0xa0, PCIE_PHY_TRSV2_LVCC);
	tcc_phy_writel(tcc_pcie, 0xa0, PCIE_PHY_TRSV3_LVCC);
}

static void tcc_pcie_assert_reset(struct pcie_port *pp)
{
	struct tcc_pcie *tcc_pcie = to_tcc_pcie(pp);

	if (tcc_pcie->reset_gpio >= 0)
		devm_gpio_request_one(pp->dev, tcc_pcie->reset_gpio,
				GPIOF_OUT_INIT_HIGH, "RESET");
	return;
}

static int tcc_pcie_establish_link(struct pcie_port *pp)
{
	int count = 0;
	struct tcc_pcie *tcc_pcie = to_tcc_pcie(pp);

	if (dw_pcie_link_up(pp)) {
		dev_err(pp->dev, "Link already up\n");
		return 0;
	}

	/* Enable bit for PHY reference clock from CKC (1: Propagates clock from CKC to PHY) */
	tcc_cfg_write(tcc_pcie, 1, PCIE_CFG08, 1<<7);
	/* Select reference clock source to PHY (1:CKC(PCLKCTRL15), 0:Oscillator) */
	tcc_cfg_write(tcc_pcie, 1, PCIE_CFG08, 1<<6);

	/* assert reset signals */
	tcc_pcie_assert_core_reset(pp);
	tcc_pcie_assert_phy_reset(pp);

	/* de-assert phy reset */
	tcc_pcie_deassert_phy_reset(pp);

	/* power on phy */
	tcc_pcie_power_on_phy(pp);

	/* initialize phy */
	tcc_pcie_init_phy(pp);

	/* pulse for common reset */
	tcc_cfg_write(tcc_pcie, 0, PCIE_CFG44, PCIE_CFG44_PHY_CMN_RST);
	udelay(500);
	tcc_cfg_write(tcc_pcie, 1, PCIE_CFG44, PCIE_CFG44_PHY_CMN_RST);

	/* de-assert core reset */
	tcc_pcie_deassert_core_reset(pp);

	/* setup root complex */
	dw_pcie_setup_rc(pp);

	/* assert reset signal */
	tcc_pcie_assert_reset(pp);

	/* assert LTSSM enable */
	tcc_cfg_write(tcc_pcie, 1, PCIE_CFG04, PCIE_CFG04_APP_LTSSM_ENABLE);

	/* check if the link is up or not */
	while (!dw_pcie_link_up(pp)) {
		mdelay(100);
		count++;
		if (count == 10) {
			unsigned int val;
			val = tcc_cfg_read(tcc_pcie, PCIE_CFG08,
				PCIE_CFG08_PHY_PLL_LOCKED) ? 1 : 0;
			dev_info(pp->dev, "PLL Locked: 0x%x\n", val);

			/* power off phy */
			tcc_pcie_power_off_phy(pp);

			dev_err(pp->dev, "PCIe Link Fail\n");
			return -EINVAL;
		}
	}

	dev_info(pp->dev, "Link up\n");

	/* DBI_RO_WR_EN = 1 */
	writel(readl(pp->dbi_base + 0x8bc)|0x1, pp->dbi_base + 0x8bc);

	return 0;
}

#define PCIE_CFG_INT_CLR_MASK	0x0000FF00
static void tcc_pcie_clear_irq_pulse(struct pcie_port *pp)
{
	u32 val;
	struct tcc_pcie *tcc_pcie = to_tcc_pcie(pp);

	val = tcc_cfg_read(tcc_pcie, PCIE_CFG24, PCIE_CFG_INT_CLR_MASK);
	tcc_cfg_write(tcc_pcie, val, PCIE_CFG25, PCIE_CFG_INT_CLR_MASK);
	return;
}

static irqreturn_t tcc_pcie_irq_handler(int irq, void *arg)
{
	struct pcie_port *pp = arg;

	tcc_pcie_clear_irq_pulse(pp);
	return IRQ_HANDLED;
}

static irqreturn_t tcc_pcie_msi_irq_handler(int irq, void *arg)
{
	struct pcie_port *pp = arg;

	return dw_handle_msi_irq(pp);
}

static void tcc_pcie_enable_interrupts(struct pcie_port *pp)
{
	if (IS_ENABLED(CONFIG_PCI_MSI))
		dw_pcie_msi_init(pp);

	return;
}

static inline void tcc_pcie_writel_rc(struct pcie_port *pp,
					u32 val, void __iomem *dbi_base)
{
	if ((dbi_base == pp->dbi_base+0x90c) || (dbi_base == pp->dbi_base+0x914)) {
		if ((val & 0xFFF00000) == 0x11400000)
			val &= 0x00FFFFFF;
	}
	writel(val, dbi_base);
	return;
}

static int tcc_pcie_link_up(struct pcie_port *pp)
{
	struct tcc_pcie *tcc_pcie = to_tcc_pcie(pp);

	if (!!tcc_cfg_read(tcc_pcie, PCIE_CFG26, PCIE_CFG26_RDLH_LINK_UP) &&
	    !!tcc_cfg_read(tcc_pcie, PCIE_CFG33, (1<<17)))
		return 1;

	return 0;
}

static void tcc_pcie_host_init(struct pcie_port *pp)
{
	tcc_pcie_establish_link(pp);
	tcc_pcie_enable_interrupts(pp);
}

static struct pcie_host_ops tcc_pcie_host_ops = {
	.writel_rc = tcc_pcie_writel_rc,
	.link_up = tcc_pcie_link_up,
	.host_init = tcc_pcie_host_init,
};

static int __init add_pcie_port(struct pcie_port *pp,
				struct platform_device *pdev)
{
	int ret;

	pp->irq = platform_get_irq(pdev, 0);
	if (!pp->irq) {
		dev_err(&pdev->dev, "failed to get irq\n");
		return -ENODEV;
	}
	ret = devm_request_irq(&pdev->dev, pp->irq, tcc_pcie_irq_handler,
				IRQF_SHARED, "tcc-pcie", pp);
	if (ret) {
		dev_err(&pdev->dev, "failed to request irq\n");
		return ret;
	}

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		pp->msi_irq = platform_get_irq(pdev, 0);
		if (!pp->msi_irq) {
			dev_err(&pdev->dev, "failed to get msi irq\n");
			return -ENODEV;
		}

		ret = devm_request_irq(&pdev->dev, pp->msi_irq,
					tcc_pcie_msi_irq_handler,
					IRQF_SHARED, "tcc-pcie", pp);
		if (ret) {
			dev_err(&pdev->dev, "failed to request msi irq\n");
			return ret;
		}
	}

	pp->root_bus_nr = -1;
	pp->ops = &tcc_pcie_host_ops;

	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(&pdev->dev, "failed to initialize host\n");
		return ret;
	}

	return 0;
}

static int __init tcc_pcie_probe(struct platform_device *pdev)
{
	struct tcc_pcie *tcc_pcie;
	struct pcie_port *pp;
	int ret;

	tcc_pcie = devm_kzalloc(&pdev->dev, sizeof(*tcc_pcie), GFP_KERNEL);
	if (!tcc_pcie)
		return -ENOMEM;

	pp = &tcc_pcie->pp;
	pp->dev = &pdev->dev;

	pp->dbi_base = of_iomap(pdev->dev.of_node, 0);
	if (IS_ERR(pp->dbi_base)) {
		ret = PTR_ERR(pp->dbi_base);
		goto fail_get_address;
	}
	tcc_pcie->phy_base = of_iomap(pdev->dev.of_node, 1);
	if (IS_ERR(tcc_pcie->phy_base)) {
		ret = PTR_ERR(tcc_pcie->phy_base);
		goto fail_get_address;
	}
	tcc_pcie->cfg_base = of_iomap(pdev->dev.of_node, 2);
	if (IS_ERR(tcc_pcie->cfg_base)) {
		ret = PTR_ERR(tcc_pcie->cfg_base);
		goto fail_get_address;
	}

	tcc_pcie->reset_gpio = of_get_named_gpio(pdev->dev.of_node, "reset-gpio", 0);


	tcc_pcie->clk_phy = devm_clk_get(&pdev->dev, "pcie_phy");
	if (IS_ERR(tcc_pcie->clk_phy)) {
		dev_err(&pdev->dev, "Failed to get pcie phy\n");
		return PTR_ERR(tcc_pcie->clk_phy);
	}
	ret = clk_prepare_enable(tcc_pcie->clk_phy);
	if (ret)
		return ret;

	tcc_pcie->clk_aux = devm_clk_get(&pdev->dev, "pcie_aux");
	if (IS_ERR(tcc_pcie->clk_aux)) {
		dev_err(&pdev->dev, "Failed to get pcie aux clock\n");
		ret = PTR_ERR(tcc_pcie->clk_aux);
		goto fail_clk_phy;
	}
	clk_set_rate(tcc_pcie->clk_aux, 250000000);
	ret = clk_prepare_enable(tcc_pcie->clk_aux);
	if (ret)
		goto fail_clk_phy;

	tcc_pcie->clk_apb = devm_clk_get(&pdev->dev, "pcie_apb");
	if (IS_ERR(tcc_pcie->clk_apb)) {
		dev_err(&pdev->dev, "Failed to get pcie apb clock\n");
		ret = PTR_ERR(tcc_pcie->clk_apb);
		goto fail_clk_aux;
	}
	clk_set_rate(tcc_pcie->clk_apb, 100000000);
	ret = clk_prepare_enable(tcc_pcie->clk_apb);
	if (ret)
		goto fail_clk_aux;

	tcc_pcie->clk_ref_ext = devm_clk_get(&pdev->dev, "pcie_ref_ext");
	if (IS_ERR(tcc_pcie->clk_ref_ext)) {
		dev_err(&pdev->dev, "Failed to get pcie ref ext clock\n");
		ret = PTR_ERR(tcc_pcie->clk_ref_ext);
		goto fail_clk_apb;
	}
	clk_set_rate(tcc_pcie->clk_ref_ext, 100000000);
	ret = clk_prepare_enable(tcc_pcie->clk_ref_ext);
	if (ret)
		goto fail_clk_apb;

	tcc_pcie->clk_hsio = devm_clk_get(&pdev->dev, "fbus_hsio");
	if (IS_ERR(tcc_pcie->clk_hsio)) {
		dev_err(&pdev->dev, "Failed to get hsio clock\n");
		ret = PTR_ERR(tcc_pcie->clk_hsio);
		goto fail_clk_ref_ext;
	}
	ret = clk_prepare_enable(tcc_pcie->clk_apb);
	if (ret)
		goto fail_clk_ref_ext;

	ret = add_pcie_port(pp, pdev);
	if (ret < 0)
		goto fail_clk_hsio;

	platform_set_drvdata(pdev, tcc_pcie);
	return 0;

fail_clk_hsio:
	clk_disable_unprepare(tcc_pcie->clk_hsio);
fail_clk_ref_ext:
	clk_disable_unprepare(tcc_pcie->clk_ref_ext);
fail_clk_apb:
	clk_disable_unprepare(tcc_pcie->clk_apb);
fail_clk_aux:
	clk_disable_unprepare(tcc_pcie->clk_aux);
fail_clk_phy:
	clk_disable_unprepare(tcc_pcie->clk_phy);
fail_get_address:
	return ret;
}

static int __exit tcc_pcie_remove(struct platform_device *pdev)
{
	struct tcc_pcie *tcc_pcie = platform_get_drvdata(pdev);

	clk_disable_unprepare(tcc_pcie->clk_hsio);
	clk_disable_unprepare(tcc_pcie->clk_ref_ext);
	clk_disable_unprepare(tcc_pcie->clk_apb);
	clk_disable_unprepare(tcc_pcie->clk_aux);
	clk_disable_unprepare(tcc_pcie->clk_phy);

	return 0;
}

static const struct of_device_id tcc_pcie_of_match[] = {
	{ .compatible = "telechips,pcie", },
	{},
};

MODULE_DEVICE_TABLE(of, tcc_pcie_of_match);

static struct platform_driver tcc_pcie_driver = {
	.remove		= __exit_p(tcc_pcie_remove),
	.driver = {
		.name	= "tcc-pcie",
		.owner	= THIS_MODULE,
		.of_match_table = tcc_pcie_of_match,
	},
};

/* Telechips PCIe driver does not allow module unload */
static int __init pcie_init(void)
{
	return platform_driver_probe(&tcc_pcie_driver, tcc_pcie_probe);
}
subsys_initcall(pcie_init);

MODULE_DESCRIPTION("Telechips PCIe host controller driver");
MODULE_LICENSE("GPL v2");
