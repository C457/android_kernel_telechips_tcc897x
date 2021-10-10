/****************************************************************************
 * irq.c
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

#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/arm-gic.h>
#include <asm/io.h>
#if defined(CONFIG_ARCH_TCC897X)
#include <mach/iomap.h>
#include <mach/vioc_intr.h>
#elif defined(CONFIG_ARCH_TCC898X)
#define TCC_PA_PIC	0x14100000
#define TCC_PA_GIC_POL	0x1460000C
#else
	#error
#endif

#ifndef GIC_SPI_OFFSET
#define GIC_SPI_OFFSET	32
#endif

#define irq_readl	__raw_readl
#define irq_writel	__raw_writel

#define PIC_IEN		0x000
#define PIC_SEL		0x018
#define PIC_TIG		0x030
#define PIC_POL		0x038
#define PIC_MODE	0x060
#define PIC_SYNC	0x068
#define PIC_WKEN	0x070
#define PIC_MODEA	0x078
#define PIC_INTMASK	0x100
#define PIC_ALLMASK	0x108

static spinlock_t irq_lock;

static void __iomem *pic_base = NULL;
static void __iomem *gicpol_base = NULL;

extern void vioc_intr_init(void);

inline static int tcc_irq_set_type(struct irq_data *data, unsigned int flow_type)
{
	void __iomem *reg;
	u32 irq = data->hwirq - GIC_SPI_OFFSET;
	u32 mask = 1 << (irq % 32);

	spin_lock(&irq_lock);

	switch (irq/32) {
	case 0:
		reg = pic_base + PIC_POL;
		break;
	case 1:
		reg = pic_base + PIC_POL + 0x4;
		break;
	case 2:
		reg = gicpol_base;
		break;
	case 3:
		reg = gicpol_base + 0x4;
		break;
	case 4:
		reg = gicpol_base + 0x8;
		break;
	default:
		spin_unlock(&irq_lock);
		return -1;
	}

	if (flow_type == IRQ_TYPE_LEVEL_LOW || flow_type == IRQ_TYPE_EDGE_FALLING)
		irq_writel(irq_readl(reg) | mask, reg);
	else
		irq_writel(irq_readl(reg) & ~mask, reg);

	spin_unlock(&irq_lock);
	return 0;
}

inline static int tcc_irq_set_wake(struct irq_data *data, unsigned int on)
{
	return 0;
}

/*
 * Change interrupt polarity.
 */
int irq_set_polarity(unsigned int irq, unsigned int act_high)
{
	void __iomem *reg;
	u32 mask;

	if (irq < 32)
		return -1;

	spin_lock(&irq_lock);

	irq -= GIC_SPI_OFFSET;
	mask = 1 << (irq % 32);

	switch (irq/32) {
	case 0:
		reg = pic_base + PIC_POL;
		break;
	case 1:
		reg = pic_base + PIC_POL + 0x4;
		break;
	case 2:
		reg = gicpol_base;
		break;
	case 3:
		reg = gicpol_base + 0x4;
		break;
	case 4:
		reg = gicpol_base + 0x8;
		break;
	default:
		spin_unlock(&irq_lock);
		return -1;
	}

	if (act_high)
		irq_writel(irq_readl(reg) & ~mask, reg);
	else
		irq_writel(irq_readl(reg) | mask, reg);

	spin_unlock(&irq_lock);
	return 0;
}

static void __init tcc_init_irq(void)
{
	unsigned		reg_values;

	pr_debug("%s\n", __func__);

	pic_base = ioremap(TCC_PA_PIC, 0x300);
	gicpol_base = ioremap(TCC_PA_GIC_POL, 0x100);

	spin_lock(&irq_lock);

	/* All Interrupt Disable */
	irq_writel(0x00000000, pic_base+PIC_IEN);
	irq_writel(0x00000000, pic_base+PIC_IEN+0x4);

	/* using IRQ */
	irq_writel(0xFFFFFFFF, pic_base+PIC_SEL);
	irq_writel(0xFFFFFFFF, pic_base+PIC_SEL+0x4);

	/* Test Interrupt Disable */
	irq_writel(0x00000000, pic_base+PIC_TIG);
	irq_writel(0x00000000, pic_base+PIC_TIG+0x4);

	/* Default ACTIVE Low */
	irq_writel(0x00000000, pic_base+PIC_POL);
	irq_writel(0x00000000, pic_base+PIC_POL+0x4);

	/* Level Trigger Mode */
	irq_writel(0xFFFFFFFF, pic_base+PIC_MODE);
	irq_writel(0xFFFFFFFF, pic_base+PIC_MODE+0x4);

	/* SYNC Enable */
	irq_writel(0xFFFFFFFF, pic_base+PIC_SYNC);
	irq_writel(0xFFFFFFFF, pic_base+PIC_SYNC+0x4);

	/* Wakeup all disable */
	irq_writel(0x00000000, pic_base+PIC_WKEN);
	irq_writel(0x00000000, pic_base+PIC_WKEN+0x4);

	/* both edge - all disable */
	irq_writel(0x00000000, pic_base+PIC_MODEA);
	irq_writel(0x00000000, pic_base+PIC_MODEA+0x4);

	/* not use INTMSK */
	irq_writel(0x00000000, pic_base+PIC_INTMASK);
	irq_writel(0x00000000, pic_base+PIC_INTMASK+0x4);

	/* use IRQ:b'0 only  (FIQ:b'1) */
	reg_values = irq_readl(pic_base + PIC_ALLMASK);
	irq_writel((reg_values & ~0x3) | 0x1, pic_base + PIC_ALLMASK);

	/*********************************************************
	 * Initialize the GIC block
	 *********************************************************/
	gic_arch_extn.irq_set_type	= tcc_irq_set_type;
	gic_arch_extn.irq_set_wake	= tcc_irq_set_wake;

#ifdef CONFIG_TCC_VIOC_CONTROLLER
	vioc_intr_init();
#endif

	spin_unlock(&irq_lock);
}

#ifdef CONFIG_OF
void __init tcc_irq_dt_init(void)
{
	spin_lock_init(&irq_lock);
	irqchip_init();
	tcc_init_irq();
}
#endif
/* end of file */
