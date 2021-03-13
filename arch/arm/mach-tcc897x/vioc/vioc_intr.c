/****************************************************************************
 * Copyright (C) 2015 Telechips Inc.
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

#include <asm/io.h>
#include <mach/iomap.h>
#include <mach/vioc_intr.h>

/* register overview */
#define VIOC_DISP		(TCC_PA_VIOC+0x0000)
#define VIOC_RDMA		(TCC_PA_VIOC+0x0400)
#define VIOC_MC			(TCC_PA_VIOC+0x1600)
#define VIOC_WMIX		(TCC_PA_VIOC+0x1800)
#define VIOC_SC			(TCC_PA_VIOC+0x2000)
#define VIOC_WDMA		(TCC_PA_VIOC+0x2800)

/* disp */
#define VIOC_DISP_DSTATUS	0x0050
#define VIOC_DISP_DIM		0x0054

/* wdma */
#define VIOC_WDMA_IRQSTS	0x0040
#define VIOC_WDMA_IRQMASK	0x0044

/* cfgint */
#define VIOC_RAWSTATUS		0x0000
#define VIOC_SYNCSTATUS		0x0008
#define VIOC_IRQSELECT		0x0010
#define VIOC_IRQMASKSET		0x0018
#define VIOC_IRQMASKCLR		0x0020

int vioc_intr_enable(int id, unsigned mask)
{
	void __iomem *cfgint_reg = (void __iomem *)io_p2v(TCC_PA_VIOC_CFGINT);
	void __iomem *reg;
	unsigned int sub_id;

	if ((id < 0) || (id > VIOC_INTR_NUM))
		return -1;

	switch (id) {
	case VIOC_INTR_DEV0:
	case VIOC_INTR_DEV1:
		sub_id = id - VIOC_INTR_DEV0;
		reg = (void __iomem *)io_p2v(VIOC_DISP + (0x100*sub_id) + VIOC_DISP_DIM);
		__raw_writel(__raw_readl(reg) & ~(mask&VIOC_DISP_INT_MASK), reg);
		break;
	case VIOC_INTR_WD0:
	case VIOC_INTR_WD1:
	case VIOC_INTR_WD2:
	case VIOC_INTR_WD3:
	case VIOC_INTR_WD4:
	case VIOC_INTR_WD5:
	case VIOC_INTR_WD6:
	case VIOC_INTR_WD7:
	case VIOC_INTR_WD8:
		sub_id = id - VIOC_INTR_WD0;

		/* clera irq status */
		reg = (void __iomem *)io_p2v(VIOC_WDMA + (0x100*sub_id) + VIOC_WDMA_IRQSTS);
		__raw_writel((mask&VIOC_WDMA_INT_MASK), reg);

		/* enable irq */
		reg = (void __iomem *)io_p2v(VIOC_WDMA + (0x100*sub_id) + VIOC_WDMA_IRQMASK);
		__raw_writel(__raw_readl(reg) & ~(mask&VIOC_WDMA_INT_MASK), reg);
		break;
	}

	if (id < 32)
		__raw_writel(1<<id, cfgint_reg + VIOC_IRQMASKCLR);
	else 
		__raw_writel(1<<(id-32), cfgint_reg + VIOC_IRQMASKCLR+0x4);

	return 0;
}

int vioc_intr_disable(int id, unsigned mask)
{
	void __iomem *cfgint_reg = (void __iomem *)io_p2v(TCC_PA_VIOC_CFGINT);
	void __iomem *reg;
	unsigned int sub_id;
	unsigned do_irq_mask = 1;

	if ((id < 0) || (id > VIOC_INTR_NUM))
		return -1;

	switch (id) {
	case VIOC_INTR_DEV0:
	case VIOC_INTR_DEV1:
		sub_id = id - VIOC_INTR_DEV0;
		reg = (void __iomem *)io_p2v(VIOC_DISP + (0x100*sub_id) + VIOC_DISP_DIM);
		__raw_writel(__raw_readl(reg) | (mask&VIOC_DISP_INT_MASK), reg);
		if ((__raw_readl(reg)&VIOC_DISP_INT_MASK) != VIOC_DISP_INT_MASK)
			do_irq_mask = 0;
		break;
	case VIOC_INTR_WD0:
	case VIOC_INTR_WD1:
	case VIOC_INTR_WD2:
	case VIOC_INTR_WD3:
	case VIOC_INTR_WD4:
	case VIOC_INTR_WD5:
	case VIOC_INTR_WD6:
	case VIOC_INTR_WD7:
	case VIOC_INTR_WD8:
		sub_id = id - VIOC_INTR_WD0;
		reg = (void __iomem *)io_p2v(VIOC_WDMA + (0x100*sub_id) + VIOC_WDMA_IRQMASK);
		__raw_writel(__raw_readl(reg) | (mask&VIOC_WDMA_INT_MASK), reg);
		if ((__raw_readl(reg)&VIOC_WDMA_INT_MASK) != VIOC_WDMA_INT_MASK)
			do_irq_mask = 0;
		break;
	}

	if (do_irq_mask) {
		if (id < 32)
			__raw_writel(1<<id, cfgint_reg + VIOC_IRQMASKSET);
		else
			__raw_writel(1<<(id-32), cfgint_reg + VIOC_IRQMASKSET+0x4);
	}

	return 0;
}

unsigned int vioc_intr_get_status(int id)
{
	void __iomem *reg;

	if ((id < 0) || (id > VIOC_INTR_NUM))
		return 0;

	switch (id) {
	case VIOC_INTR_DEV0:
	case VIOC_INTR_DEV1:
		reg = (void __iomem *)io_p2v(VIOC_DISP + (0x100*id) + VIOC_DISP_DSTATUS);
		return (__raw_readl(reg) & VIOC_DISP_INT_MASK);

	case VIOC_INTR_WD0:
	case VIOC_INTR_WD1:
	case VIOC_INTR_WD2:
	case VIOC_INTR_WD3:
	case VIOC_INTR_WD4:
	case VIOC_INTR_WD5:
	case VIOC_INTR_WD6:
	case VIOC_INTR_WD7:
	case VIOC_INTR_WD8:
		id -= VIOC_INTR_WD0;
		reg = (void __iomem *)io_p2v(VIOC_WDMA + (0x100*id) + VIOC_WDMA_IRQSTS);
		return (__raw_readl(reg) &VIOC_WDMA_INT_MASK);

	}
	return 0;
}

bool check_vioc_irq_status(void __iomem *reg, int id)
{
	unsigned flag;

	if ((id < 0) || (id > VIOC_INTR_NUM))
		return false;

	if (id < 32) {
		flag = (__raw_readl(reg + VIOC_IRQSELECT) & (1<<id)) ?
				(__raw_readl(reg + VIOC_SYNCSTATUS) & (1<<id)) : 
				(__raw_readl(reg + VIOC_RAWSTATUS) & (1<<id));
	}
	else {
		flag = (__raw_readl(reg + VIOC_IRQSELECT + 0x4) & (1<<(id-32))) ?
				(__raw_readl(reg + VIOC_SYNCSTATUS + 0x4) & (1<<(id-32))) : 
				(__raw_readl(reg + VIOC_RAWSTATUS + 0x4) & (1<<(id-32)));
	}
	if (flag)
		return true;
	return false;
}

bool is_vioc_intr_activatied(int id, unsigned mask)
{
	void __iomem *reg = (void __iomem *)io_p2v(TCC_PA_VIOC_CFGINT);

	if ((id < 0) || (id > VIOC_INTR_NUM))
		return false;

	switch (id) {
	case VIOC_INTR_DEV0:
	case VIOC_INTR_DEV1:
		reg = (void __iomem *)io_p2v(VIOC_DISP + (0x100*id) + VIOC_DISP_DSTATUS);
		if (__raw_readl(reg) & (mask&VIOC_DISP_INT_MASK))
			return true;
		return false;
	case VIOC_INTR_WD0:
	case VIOC_INTR_WD1:
	case VIOC_INTR_WD2:
	case VIOC_INTR_WD3:
	case VIOC_INTR_WD4:
	case VIOC_INTR_WD5:
	case VIOC_INTR_WD6:
	case VIOC_INTR_WD7:
	case VIOC_INTR_WD8:
		id -= VIOC_INTR_WD0;
		reg = (void __iomem *)io_p2v(VIOC_WDMA + (0x100*id) + VIOC_WDMA_IRQSTS);
		if (__raw_readl(reg) & (mask&VIOC_WDMA_INT_MASK))
			return true;
		return false;
	}
	return false;
}

bool is_vioc_intr_unmasked(int id, unsigned mask)
{
        void __iomem *reg = (void __iomem *)io_p2v(TCC_PA_VIOC_CFGINT);

        if ((id < 0) || (id > VIOC_INTR_NUM))
                return false;

        switch (id) {
        case VIOC_INTR_DEV0:
        case VIOC_INTR_DEV1:
                reg = (void __iomem *)io_p2v(VIOC_DISP + (0x100*id) + VIOC_DISP_DIM);
                if (__raw_readl(reg) & (mask&VIOC_DISP_INT_MASK))
                        return true;
                return false;
        case VIOC_INTR_WD0:
        case VIOC_INTR_WD1:
        case VIOC_INTR_WD2:
        case VIOC_INTR_WD3:
        case VIOC_INTR_WD4:
        case VIOC_INTR_WD5:
        case VIOC_INTR_WD6:
        case VIOC_INTR_WD7:
        case VIOC_INTR_WD8:
                id -= VIOC_INTR_WD0;
                reg = (void __iomem *)io_p2v(VIOC_WDMA + (0x100*id) + VIOC_WDMA_IRQMASK);
                if (__raw_readl(reg) & (mask&VIOC_WDMA_INT_MASK))
                        return false;
                return true;
        }
        return false;
}

int vioc_intr_clear(int id, unsigned mask)
{
	void __iomem *reg;

	if ((id < 0) || (id > VIOC_INTR_NUM))
		return -1;

	switch (id) {
	case VIOC_INTR_DEV0:
	case VIOC_INTR_DEV1:
		reg = (void __iomem *)io_p2v(VIOC_DISP + (0x100*id) + VIOC_DISP_DSTATUS);
		__raw_writel((mask&VIOC_DISP_INT_MASK), reg);
		break;
	case VIOC_INTR_WD0:
	case VIOC_INTR_WD1:
	case VIOC_INTR_WD2:
	case VIOC_INTR_WD3:
	case VIOC_INTR_WD4:
	case VIOC_INTR_WD5:
	case VIOC_INTR_WD6:
	case VIOC_INTR_WD7:
	case VIOC_INTR_WD8:
		id -= VIOC_INTR_WD0;
		reg = (void __iomem *)io_p2v(VIOC_WDMA + (0x100*id) + VIOC_WDMA_IRQSTS);
		__raw_writel((mask&VIOC_WDMA_INT_MASK), reg);
		break;
	}
	return 0;
}

void vioc_intr_init(void)
{
	void __iomem *reg = (void __iomem *)io_p2v(TCC_PA_VIOC_CFGINT);
	int i;

	__raw_writel(0xffffffff, reg + VIOC_IRQMASKCLR);
	__raw_writel(0xffffffff, reg + VIOC_IRQMASKCLR + 0x4);

	/* disp irq mask & status clear */
	for (i=0 ; i<(VIOC_INTR_DEV2-VIOC_INTR_DEV0) ; i++) {
		__raw_writel(VIOC_DISP_INT_MASK, (void __iomem *)io_p2v(VIOC_DISP + (i*100) + VIOC_DISP_DIM));
		__raw_writel(VIOC_DISP_INT_MASK, (void __iomem *)io_p2v(VIOC_DISP + (i*100) + VIOC_DISP_DSTATUS));
	}

	/* wdma irq mask & status clear */
	for (i=0 ; i<(VIOC_INTR_WD8-VIOC_INTR_WD0) ; i++) {
		__raw_writel(VIOC_WDMA_INT_MASK, (void __iomem *)io_p2v(VIOC_WDMA + (i*100) + VIOC_WDMA_IRQMASK));
		__raw_writel(VIOC_WDMA_INT_MASK, (void __iomem *)io_p2v(VIOC_WDMA + (i*100) + VIOC_WDMA_IRQSTS));
	}
}
