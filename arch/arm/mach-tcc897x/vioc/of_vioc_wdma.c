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
#include <mach/vioc_intr.h>
#include <mach/of_vioc_wdma.h>
#include <mach/tccfb_ioctrl.h>

#define WDMACTRL	0x00
#define WDMARATE	0x04
#define WDMASIZE	0x08
#define WDMABASE0	0x0C
#define WDMACADDR	0x10
#define WDMABASE1	0x14
#define WDMABASE2	0x18
#define WDMAOFFS	0x1C
#define WDMABG		0x24
#define	WDMAPTS		0x28
#define WDMADMAT0	0x2C
#define WDMADMAT1	0x30
#define WDMAENH		0x34
#define WDMAROLL	0x38
#define WDMASBASE	0x3C
#define WDMAIRQSTS	0x40
#define WDMAIRQMSK	0x44

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
		case TCC_LCDC_IMG_FMT_YUV420SP:	// YCbCr 4:2:0 Separated format - Not Supported for Image 1 and 2
			off_L = offset;
			off_H = offset/2;
			break;
		case TCC_LCDC_IMG_FMT_YUV422SP:		// YCbCr 4:2:2 Separated format - Not Supported for Image 1 and 2
			off_L = offset;
			off_H = offset/2;
			break;
		case TCC_LCDC_IMG_FMT_UYVY:	// YCbCr 4:2:2 Sequential format
		case TCC_LCDC_IMG_FMT_VYUY:	// YCbCr 4:2:2 Sequential format
		case TCC_LCDC_IMG_FMT_YUYV:	// YCbCr 4:2:2 Sequential format
		case TCC_LCDC_IMG_FMT_YVYU:	// YCbCr 4:2:2 Sequential format
			off_L = 2*offset;
			break;
		case TCC_LCDC_IMG_FMT_YUV420ITL0:	// YCbCr 4:2:0 interleved type 0 format - Not Supported for Image 1 and 2
		case TCC_LCDC_IMG_FMT_YUV420ITL1:	// YCbCr 4:2:0 interleved type 1 format - Not Supported for Image 1 and 2
			off_L = offset;
			off_H = offset;
			break;
		case TCC_LCDC_IMG_FMT_YUV422ITL0:	// YCbCr 4:2:2 interleved type 0 format - Not Supported for Image 1 and 2
		case TCC_LCDC_IMG_FMT_YUV422ITL1:	// YCbCr 4:2:2 interleved type 1 format - Not Supported for Image 1 and 2
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

void vioc_wdma_set_image(struct vioc_wdma_device *wdma, unsigned int en)
{
	volatile unsigned value;

	if (!wdma)
		return;

	if (!en) {
		value = (__raw_readl(wdma->reg + WDMACTRL) & ~(1<<28 | 1<<16));
		__raw_writel(value | (0<<28 | 1<<16), wdma->reg + WDMACTRL);
		return;
	}

	/* format */
	value = (__raw_readl(wdma->reg + WDMACTRL) & ~(0x1F));
	__raw_writel(value | (wdma->data.format & 0x1F), wdma->reg + WDMACTRL);

	/* size */
	value = ((wdma->data.height & 0xFFFF) << 16) | (wdma->data.width & 0xFFFF);
	__raw_writel(value, wdma->reg + WDMASIZE);

	/* offset */
	__raw_writel(calc_offset(wdma->data.format, wdma->data.offset), wdma->reg + WDMAOFFS);

	/* base */
	__raw_writel(wdma->data.base0, wdma->reg + WDMABASE0);
	__raw_writel(wdma->data.base1, wdma->reg + WDMABASE1);
	__raw_writel(wdma->data.base2, wdma->reg + WDMABASE2);

	/* mode */
	value = __raw_readl(wdma->reg + WDMACTRL) & ~((3<<18)|(1<<17) | (7<<12) | (3<<9)|(1<<8));
	value |= (wdma->data.mode.rgb_swap&0x7)<<12;	/* rgb swap mode */
	value |= (wdma->data.mode.convert&0x3)<<18 | (wdma->data.mode.convert&0x3)<<9;	/* converter mode */
	if (wdma->data.mode.mode == VIOC_WDMA_MODE_RGB2YUV)
		value |= 1<<8;
	else if (wdma->data.mode.mode == VIOC_WDMA_MODE_YUV2RGB)
		value |= 1<<17;
	__raw_writel(value, wdma->reg + WDMACTRL);

	/* enable */
	value = __raw_readl(wdma->reg + WDMACTRL) & ~(1<<28 | 1<<23 | 1<<16);
	__raw_writel(value | (1<<28 | (wdma->data.cont?1:0)<<23 | 1<<16), wdma->reg + WDMACTRL);

	/* status all clear */
	__raw_writel(wdma->intr->bits, wdma->reg + WDMAIRQSTS);
}

void vioc_wdma_update(struct vioc_wdma_device *wdma)
{
	if (!wdma)
		return;

	__raw_writel(__raw_readl(wdma->reg + WDMACTRL) | (1<<16), wdma->reg + WDMACTRL);
}

/*
 * reset: 1(reset), 0(normal)
 */
void vioc_wdma_swreset(struct vioc_wdma_device *wdma, int reset)
{
	if (!wdma)
		return;

	if (reset) {
		__raw_writel(__raw_readl(wdma->rst_reg) | (1<<wdma->id), wdma->rst_reg);
		memset(&wdma->data, 0x0, sizeof(struct vioc_wdma_data));
		wdma->data.mode.convert = 2;	/* default value */
	}
	else
		__raw_writel(__raw_readl(wdma->rst_reg) & ~(1<<wdma->id), wdma->rst_reg);
}

static struct vioc_wdma_device *of_vioc_wdma_get(struct device_node *np, int index)
{
	struct vioc_wdma_device *wdma = NULL;
	struct of_phandle_args args;
	int err;

	err = of_parse_phandle_with_args(np, "wdmas", "#vioc_wdma-cells", index, &args);
	if (err) {
		pr_debug("%s(): can't parse \"wdmas\" property\n", __func__);
		return ERR_PTR(err);
	}

	wdma = kzalloc(sizeof(struct vioc_wdma_device), GFP_KERNEL);
	if (!wdma) {
		err = -ENOMEM;
		goto err_alloc_wdma_dev;
	}
	wdma->data.mode.convert = 2;	/* default value */

	wdma->intr = kzalloc(sizeof(struct vioc_intr_type), GFP_KERNEL);
	if (!wdma->intr) {
		err = -ENOMEM;
		goto err_alloc_vioc_intr;
	}

	wdma->reg = of_iomap(args.np, 0);
	wdma->rst_reg = of_iomap(args.np, 1);
	wdma->id = of_alias_get_id(args.np, "vioc_wdma");
	if (wdma->id < 0) {
		err = -ENODEV;
		goto err_get_property;
	}
	wdma->irq = of_irq_to_resource(args.np, 0, NULL);
	wdma->intr->id = VIOC_INTR_WD0 + wdma->id;
	wdma->intr->bits = (1<<VIOC_WDMA_INTR_EOFR);

	of_node_put(args.np);

	return wdma;

err_get_property:
	kfree(wdma->intr);
err_alloc_vioc_intr:
	kfree(wdma);
err_alloc_wdma_dev:

	return ERR_PTR(err);
}

static void vioc_wdma_put(struct vioc_wdma_device *wdma)
{
	if (!wdma)
		return;
	if (wdma->intr)
		kfree(wdma->intr);
	kfree(wdma);
}

static void devm_vioc_wdma_release(struct device *dev, void *res)
{
	vioc_wdma_put(*(struct vioc_wdma_device **)res);
}

int get_count_vioc_wdma(struct device *dev)
{
	return of_count_phandle_with_args(dev->of_node, "wdmas", "#vioc_wdma-cells");
}

struct vioc_wdma_device *devm_vioc_wdma_get(struct device *dev, int index)
{
	struct vioc_wdma_device **ptr, *wdma = ERR_PTR(-ENODEV);

	ptr = devres_alloc(devm_vioc_wdma_release, sizeof(**ptr), GFP_KERNEL);
	if (!ptr)
		return ERR_PTR(-ENOMEM);

	wdma = of_vioc_wdma_get(dev->of_node, index);	/* get a wdma */
	if (!IS_ERR(wdma)) {
		*ptr = wdma;
		devres_add(dev, ptr);
	} else
		devres_free(ptr);

	return wdma;
}
