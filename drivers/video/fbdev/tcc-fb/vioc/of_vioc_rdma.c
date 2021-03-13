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

#include <linux/device.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <asm/io.h>

#ifdef CONFIG_ARCH_TCC897X
#include <mach/vioc_intr.h>
#include <mach/of_vioc_rdma.h>
#include <mach/tccfb_ioctrl.h>
#else
#include <video/tcc/vioc_intr.h>
#include <video/tcc/of_vioc_rdma.h>
#include <video/tcc/tccfb_ioctrl.h>
#endif

#define RDMACTRL		0x00
#define RDMAPTS			0x04
#define RDMASIZE		0x08
#define RDMABASE0		0x0C
#define RDMACADDR		0x10
#define RDMABASE1		0x14
#define RDMABASE2		0x18
#define RDMAOFFS		0x1C
#define RDMAMISC		0x20
#define RDMAALPHA		0x24
#define	RDMASTAT		0x28
#define RDMAIRQMSK		0x2C
#define RDMASBASE0		0x30
#define RDMA_RBASE0		0x34
#define RDMA_RBASE1		0x38
#define RDMA_RBASE2		0x3C
#define RDMA_CROP_SIZE	0x40
#define RDMA_CROP_POS	0x44


static inline unsigned calc_offset(unsigned int format, unsigned int offset)
{
	unsigned int off_L = 0;
	unsigned int off_H = 0;

	switch (format)
	{
		case TCC_LCDC_IMG_FMT_1BPP:	// 1bpp indexed color
			off_L = (1*offset)/8;
			break;
		case TCC_LCDC_IMG_FMT_2BPP:	// 2bpp indexed color
			off_L = (1*offset)/4;
			break;
		case TCC_LCDC_IMG_FMT_4BPP:	// 4bpp indexed color
			off_L = (1*offset)/2;
			break;
		case TCC_LCDC_IMG_FMT_8BPP:	// 8bpp indexed color
			off_L = (1*offset);
			break;
			//...
		case TCC_LCDC_IMG_FMT_RGB332:	// RGB332 - 1bytes aligned - R[7:5],G[4:2],B[1:0]
			off_L = 1*offset;
			break;
		case TCC_LCDC_IMG_FMT_RGB444:	// RGB444 - 2bytes aligned - A[15:12],R[11:8],G[7:3],B[3:0]
		case TCC_LCDC_IMG_FMT_RGB565:	// RGB565 - 2bytes aligned - R[15:11],G[10:5],B[4:0]
		case TCC_LCDC_IMG_FMT_RGB555:	// RGB555 - 2bytes aligned - A[15],R[14:10],G[9:5],B[4:0]
			off_L = 2*offset;
			break;
		//case TCC_LCDC_IMG_FMT_RGB888:
		case TCC_LCDC_IMG_FMT_RGB888:	// RGB888 - 4bytes aligned - A[31:24],R[23:16],G[15:8],B[7:0]
		case TCC_LCDC_IMG_FMT_RGB666:	// RGB666 - 4bytes aligned - A[23:18],R[17:12],G[11:6],B[5:0]
			off_L = 4*offset;
			break;
		case TCC_LCDC_IMG_FMT_RGB888_3: //RGB888 - 3 bytes aligned : B1[31:24],R0[23:16],G0[15:8],B0[7:0]
		case TCC_LCDC_IMG_FMT_ARGB6666_3: //ARGB6666 - 3 bytes aligned : A[23:18],R[17:12],G[11:6],B[5:0]
			off_L = 3*offset;
			break;

		case TCC_LCDC_IMG_FMT_444SEP:	/* YUV444 or RGB444 Format */
			off_L = offset;
			off_H = offset;
			break;
	
			//...
		case TCC_LCDC_IMG_FMT_YUV420SP:	// YCbCr 4:2:0 Separated format - Not Supported for Image 1 and 2
			//!!!!
			off_L = offset;
			off_H = offset/2;
			break;
		case TCC_LCDC_IMG_FMT_YUV422SP:		// YCbCr 4:2:2 Separated format - Not Supported for Image 1 and 2
			//!!!!
			off_L = offset;
			off_H = offset/2;
			break;
		case TCC_LCDC_IMG_FMT_UYVY:	// YCbCr 4:2:2 Sequential format
		case TCC_LCDC_IMG_FMT_VYUY:	// YCbCr 4:2:2 Sequential format
		case TCC_LCDC_IMG_FMT_YUYV:	// YCbCr 4:2:2 Sequential format
		case TCC_LCDC_IMG_FMT_YVYU:	// YCbCr 4:2:2 Sequential format
			off_L = 2*offset;
			break;
			//...
		case TCC_LCDC_IMG_FMT_YUV420ITL0:	// YCbCr 4:2:0 interleved type 0 format - Not Supported for Image 1 and 2
		case TCC_LCDC_IMG_FMT_YUV420ITL1:	// YCbCr 4:2:0 interleved type 1 format - Not Supported for Image 1 and 2
			//!!!!
			off_L = offset;
			off_H = offset;
			break;
		case TCC_LCDC_IMG_FMT_YUV422ITL0:	// YCbCr 4:2:2 interleved type 0 format - Not Supported for Image 1 and 2
		case TCC_LCDC_IMG_FMT_YUV422ITL1:	// YCbCr 4:2:2 interleved type 1 format - Not Supported for Image 1 and 2
			//!!!!
			off_L = offset;
			off_H = offset;
			break;
		default:
			off_L = offset;
			off_H = offset;
			break;
	}

	return (((off_H & 0xFFFF)<<16) | (off_L & 0xFFFF));
}

void vioc_rdma_set_image(struct vioc_rdma_device *rdma, unsigned int en)
{
	volatile unsigned value;

	if (!rdma)
		return;

	if (!en) {
		value = (__raw_readl(rdma->reg + RDMACTRL) & ~(1<<28 | 1<<16));
		__raw_writel(value | (0<<28 | 1<<16), rdma->reg + RDMACTRL);
		return;
	}

	/* alpha select/enable */
	value = __raw_readl(rdma->reg + RDMACTRL) & ~(1<<24|1<<11);
	__raw_writel(value | (rdma->data.asel&0x1)<<24 | (rdma->data.aen&0x1)<<11, rdma->reg + RDMACTRL);

	/* format */
	value = __raw_readl(rdma->reg + RDMACTRL) & ~(0x1F);
	__raw_writel(value | (rdma->data.format & 0x1F), rdma->reg + RDMACTRL);

	/* size */
	value = ((rdma->data.height & 0xFFFF) << 16) | (rdma->data.width & 0xFFFF);
	__raw_writel(value, rdma->reg + RDMASIZE);

	/* offset */
	__raw_writel(calc_offset(rdma->data.format, rdma->data.offset), rdma->reg + RDMAOFFS);

	/* base */
	__raw_writel(rdma->data.base0, rdma->reg + RDMABASE0);
	__raw_writel(rdma->data.base1, rdma->reg + RDMABASE1);
	__raw_writel(rdma->data.base2, rdma->reg + RDMABASE2);

	/* mode */
	value = __raw_readl(rdma->reg + RDMACTRL) & ~((1<<6)|(3<<18)|(1<<17) | (7<<12) | (1<<5)|(3<<9)|(1<<8));
	value |= (rdma->data.mode.rgb_swap&0x7)<<12;	/* rgb swap mode */
	if (rdma->data.mode.mode == VIOC_RDMA_MODE_RGB2YUV) {
		value |= (rdma->data.mode.convert&0x3)<<18 | (!!(rdma->data.mode.convert&0x4))<<6;
		value |= 1<<17;
	} else if (rdma->data.mode.mode == VIOC_RDMA_MODE_YUV2RGB) {
		value |= (rdma->data.mode.convert&0x3)<<9 | (!!(rdma->data.mode.convert&0x4))<<5;
		value |= 1<<8;
	}
	__raw_writel(value, rdma->reg + RDMACTRL);

	/* enable */
	value = __raw_readl(rdma->reg + RDMACTRL) & ~(1<<28 | 1<<16);
	__raw_writel(value | (1<<28 | 1<<16), rdma->reg + RDMACTRL);
}

void vioc_rdma_update(struct vioc_rdma_device *rdma)
{
	if (!rdma)
		return;

	__raw_writel(__raw_readl(rdma->reg + RDMACTRL) | (1<<16), rdma->reg + RDMACTRL);
}

/*
 * reset: 1(reset), 0(normal)
 */
void vioc_rdma_swreset(struct vioc_rdma_device *rdma, int reset)
{
	if (!rdma)
		return;

	if (reset) {
		__raw_writel(__raw_readl(rdma->rst_reg) | (1<<rdma->id), rdma->rst_reg);
		memset(&rdma->data, 0x0, sizeof(struct vioc_rdma_data));
		rdma->data.mode.convert = 2;	/* default value */
	}
	else
		__raw_writel(__raw_readl(rdma->rst_reg) & ~(1<<rdma->id), rdma->rst_reg);
}

static struct vioc_rdma_device *of_vioc_rdma_get(struct device_node *np, int index)
{
	struct vioc_rdma_device *rdma = NULL;
	struct of_phandle_args args;
	int err;

	err = of_parse_phandle_with_args(np, "rdmas", "#vioc_rdma-cells", index, &args);
	if (err) {
		pr_debug("%s(): can't parse \"rdmas\" property\n", __func__);
		return ERR_PTR(err);
	}

	rdma = kzalloc(sizeof(struct vioc_rdma_device), GFP_KERNEL);
	if (!rdma) {
		err = -ENOMEM;
		goto err_alloc_rdma_dev;
	}
	rdma->data.mode.convert = 2;	/* default value */

	rdma->intr = kzalloc(sizeof(struct vioc_intr_type), GFP_KERNEL);
	if (!rdma->intr) {
		err = -ENOMEM;
		goto err_alloc_vioc_intr;
	}

	rdma->reg = of_iomap(args.np, 0);
	rdma->rst_reg = of_iomap(args.np, 1);
	rdma->id = of_alias_get_id(args.np, "vioc_rdma");
	if (rdma->id < 0) {
		err = -ENODEV;
		goto err_get_property;
	}
	rdma->irq = of_irq_to_resource(args.np, 0, NULL);
	rdma->intr->id = VIOC_INTR_RD0 + rdma->id;
	rdma->intr->bits = VIOC_RDMA_INT_MASK;

	of_node_put(args.np);

	return rdma;

err_get_property:
	kfree(rdma->intr);
err_alloc_vioc_intr:
	kfree(rdma);
err_alloc_rdma_dev:

	return ERR_PTR(err);
}

static void vioc_rdma_put(struct vioc_rdma_device *rdma)
{
	if (!rdma)
		return;
	if (rdma->intr)
		kfree(rdma->intr);
	kfree(rdma);
}

static void devm_vioc_rdma_release(struct device *dev, void *res)
{
	vioc_rdma_put(*(struct vioc_rdma_device **)res);
}

int get_count_vioc_rdma(struct device *dev)
{
	return of_count_phandle_with_args(dev->of_node, "rdmas", "#vioc_rdma-cells");
}

struct vioc_rdma_device *devm_vioc_rdma_get(struct device *dev, int index)
{
	struct vioc_rdma_device **ptr, *rdma = ERR_PTR(-ENODEV);

	ptr = devres_alloc(devm_vioc_rdma_release, sizeof(**ptr), GFP_KERNEL);
	if (!ptr)
		return ERR_PTR(-ENOMEM);

	rdma = of_vioc_rdma_get(dev->of_node, index);	/* get a rdma */
	if (!IS_ERR(rdma)) {
		*ptr = rdma;
		devres_add(dev, ptr);
	} else
		devres_free(ptr);

	return rdma;
}
