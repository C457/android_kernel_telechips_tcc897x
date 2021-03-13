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
#include <mach/of_vioc_wmix.h>
#include <mach/iomap.h>

#define WMIXCTRL	0x00
#define WMIXSIZE	0x08
#define WMIXPOS(n)	(0x10 + (0x4*n))
#define MACON		0x40
#define MCCON		0x44
#define MROPC		0x48
#define MRAT		0x4C

#define WMIX_SWRST_OFF	9

enum {
	VIOC_WMIX00 = 0,
	VIOC_WMIX03,
	VIOC_WMIX10,
	VIOC_WMIX13,
	VIOC_WMIX30,
	VIOC_WMIX40,
	VIOC_WMIX50,
	VIOC_WMIX60,
	VIOC_WMIX_MAX
};

static void vioc_wmix_path(unsigned int path,  unsigned int mode)
{
	void __iomem *cfg_misc0 = (void __iomem *)io_p2v(TCC_PA_VIOC_CFGINT + 0x40);
	int offset;
	unsigned value;

	switch (path) {
		case VIOC_WMIX00:
			offset = 16;
			break;
		case VIOC_WMIX03:
			offset = 17;
			break;
		case VIOC_WMIX10:
			offset = 18;
			break;
		case VIOC_WMIX13:
			offset = 19;
			break;
		case VIOC_WMIX30:
			offset = 22;
			break;
		case VIOC_WMIX40:
			return;	/* should be 0 */
		case VIOC_WMIX50:
			offset = 26;
			break;
		case VIOC_WMIX60:
			return;	/* should be 0 */
		default:
			return;
	}

	value = __raw_readl(cfg_misc0) & ~(1<<offset);
	if (mode)
		value |= 1<<offset;
	__raw_writel(value, cfg_misc0);
}


void vioc_wmix_set_position(struct vioc_wmix_device *wmix, unsigned int update)
{
	volatile unsigned value;

	if (!wmix)
		return;

	/* position */
	value = ((wmix->pos.sy & 0x0FFF) << 16) | (wmix->pos.sx & 0x0FFF);
	__raw_writel(value, wmix->reg + (WMIXPOS(wmix->pos.layer)));

	/* enable */
	if(update)
		__raw_writel(__raw_readl(wmix->reg + WMIXCTRL) | 1<<16, wmix->reg + WMIXCTRL);
}


void vioc_wmix_set_image(struct vioc_wmix_device *wmix, unsigned int en)
{
	volatile unsigned value;

	if (!wmix)
		return;

	/* set wmix path */
	vioc_wmix_path(wmix->path, wmix->mixmode);

	if (!en) {
		__raw_writel(__raw_readl(wmix->reg + WMIXCTRL) & ~(1<<16), wmix->reg + WMIXCTRL);
		return;
	}

	if (wmix->data.layer < 4) {
		if (wmix->data.region<4) {
			/* alpha control */
			value = ((wmix->data.acon1&0x7)<<4|(wmix->data.acon0&0x7))<<(wmix->data.region*8);
			__raw_writel(value, wmix->reg + MACON + wmix->data.layer*0x10);

			/* color control */
			value = ((wmix->data.ccon1&0xF)<<4|(wmix->data.ccon0&0xF))<<(wmix->data.region*8);
			__raw_writel(value, wmix->reg + MCCON + wmix->data.layer*0x10);
		}

		/* ROP control */
		value = __raw_readl(wmix->reg + MROPC + wmix->data.layer*0x10) & ~(0xFFFF<<16 | 0x3<<1 | 0x1F);
		value |= (wmix->data.asel&0x3)|(wmix->data.mode&0x1F);
		value |= ((wmix->data.alpha1&0xFF)<<24) | ((wmix->data.alpha0&0xFF)<<16);
		__raw_writel(value, wmix->reg + MROPC + wmix->data.layer*0x10);

		/* ROP pattern */
	}

	/* size */
	value = ((wmix->data.height & 0xFFFF) << 16) | (wmix->data.width & 0xFFFF);
	__raw_writel(value, wmix->reg + WMIXSIZE);

	/* enable */
	__raw_writel(__raw_readl(wmix->reg + WMIXCTRL) | 1<<16, wmix->reg + WMIXCTRL);
}

/*
 * reset: 1(reset), 0(normal)
 */
void vioc_wmix_swreset(struct vioc_wmix_device *wmix, int reset)
{
	if (!wmix)
		return;

	if (wmix->id == 4)	/* not used */
		return;

	if (reset) {
		__raw_writel(__raw_readl(wmix->rst_reg) | (1<<(wmix->id+WMIX_SWRST_OFF)), wmix->rst_reg);
		memset(&wmix->data, 0x0, sizeof(struct vioc_wmix_data));
		wmix->data.layer = -1;
	}
	else
		__raw_writel(__raw_readl(wmix->rst_reg) & ~(1<<(wmix->id+WMIX_SWRST_OFF)), wmix->rst_reg);
}

static struct vioc_wmix_device *of_vioc_wmix_get(struct device_node *np, int index)
{
	struct vioc_wmix_device *wmix = NULL;
	struct of_phandle_args args;
	int err;

	err = of_parse_phandle_with_args(np, "wmixs", "#vioc_wmix-cells", index, &args);
	if (err) {
		pr_debug("%s(): can't parse \"wmixs\" property(path)\n", __func__);
		return ERR_PTR(err);
	}

	wmix = kzalloc(sizeof(struct vioc_wmix_device), GFP_KERNEL);
	if (!wmix) {
		err = -ENOMEM;
		goto err_alloc_wmix_dev;
	}

	/* set default values */
	wmix->data.layer = -1;

	wmix->intr = kzalloc(sizeof(struct vioc_intr_type), GFP_KERNEL);
	if (!wmix->intr) {
		err = -ENOMEM;
		goto err_alloc_vioc_intr;
	}

	wmix->reg = of_iomap(args.np, 0);
	wmix->rst_reg = of_iomap(args.np, 1);
	wmix->id = of_alias_get_id(args.np, "vioc_wmix");
	if (wmix->id < 0) {
		err = -ENODEV;
		goto err_get_property;
	}
	wmix->irq = of_irq_to_resource(args.np, 0, NULL);
	wmix->intr->id = VIOC_INTR_WMIX0 + wmix->id;
	wmix->intr->bits = VIOC_WMIX_INT_MASK;

	if (args.args_count)
		wmix->path = args.args[0];
	else
		wmix->path = VIOC_WMIX_MAX;

	of_node_put(args.np);

	return wmix;

err_get_property:
	kfree(wmix->intr);
err_alloc_vioc_intr:
	kfree(wmix);
err_alloc_wmix_dev:

	return ERR_PTR(err);
}

static void vioc_wmix_put(struct vioc_wmix_device *wmix)
{
	if (!wmix)
		return;
	if (wmix->intr)
		kfree(wmix->intr);
	kfree(wmix);
}

static void devm_vioc_wmix_release(struct device *dev, void *res)
{
	vioc_wmix_put(*(struct vioc_wmix_device **)res);
}

int get_count_vioc_wmix(struct device *dev)
{
	return of_count_phandle_with_args(dev->of_node, "wmixs", "#vioc_wmix-cells");
}

struct vioc_wmix_device *devm_vioc_wmix_get(struct device *dev, int index)
{
	struct vioc_wmix_device **ptr, *wmix = ERR_PTR(-ENODEV);

	ptr = devres_alloc(devm_vioc_wmix_release, sizeof(**ptr), GFP_KERNEL);
	if (!ptr)
		return ERR_PTR(-ENOMEM);

	wmix = of_vioc_wmix_get(dev->of_node, index);	/* get a wmix */
	if (!IS_ERR(wmix)) {
		*ptr = wmix;
		devres_add(dev, ptr);
	} else
		devres_free(ptr);

	return wmix;
}
