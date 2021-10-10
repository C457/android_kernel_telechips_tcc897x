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
#include <linux/delay.h>
#include <asm/io.h>
#include <mach/vioc_intr.h>
#include <mach/of_vioc_sc.h>
#include <mach/iomap.h>

#define SCCTRL		0x00
#define SCSSIZE		0x08	/* read only regs. */
#define SCDSIZE		0x0C
#define SCOPOS		0x10
#define SCOSIZE		0x14

#define SC_SWRST_OFF	28

enum {
	VIOC_SCPATH_RDMA_00 = 0,
	VIOC_SCPATH_RDMA_01,
	VIOC_SCPATH_RDMA_02,
	VIOC_SCPATH_RDMA_03,
	VIOC_SCPATH_RDMA_04,
	VIOC_SCPATH_RDMA_05,
	VIOC_SCPATH_RDMA_06,
	VIOC_SCPATH_RDMA_07,
	VIOC_SCPATH_RDMA_08,
	VIOC_SCPATH_RDMA_09,
	VIOC_SCPATH_RDMA_10,
	VIOC_SCPATH_RDMA_11,
	VIOC_SCPATH_RDMA_12,
	VIOC_SCPATH_RDMA_13,
	VIOC_SCPATH_RDMA_14,
	VIOC_SCPATH_RDMA_15,
	VIOC_SCPATH_VIDEOIN_00,
	VIOC_SCPATH_RDMA_16,
	VIOC_SCPATH_VIDEOIN_01,
	VIOC_SCPATH_RDMA_17,
	VIOC_SCPATH_WDMA_00,
	VIOC_SCPATH_WDMA_01,
	VIOC_SCPATH_WDMA_02,
	VIOC_SCPATH_WDMA_03,
	VIOC_SCPATH_WDMA_04,
	VIOC_SCPATH_WDMA_05,
	VIOC_SCPATH_WDMA_06,
	VIOC_SCPATH_WDMA_07,
	VIOC_SCPATH_WDMA_08,
	VIOC_SCPATH_MAX
};

enum {
	VIOC_SC_DISCONNECTED = 0,
	VIOC_SC_CONNECTING,
	VIOC_SC_CONNECTED,
	VIOC_SC_DISCONNECTING
};

static int vioc_sc_path_ctrl(struct vioc_sc_device *sc, void __iomem *cfg_path, int en)
{
	unsigned value, loop = 100;

	if (en) {
		value = __raw_readl(cfg_path) & ~(0xFF);
		__raw_writel(value | 1<<31 | (sc->path & 0xFF), cfg_path);
	}
	else
		__raw_writel(__raw_readl(cfg_path) & ~(1<<31), cfg_path);

	if (__raw_readl(cfg_path) & 1<<18) {
		printk("%s: scaler_%d path configuration error(1). device is busy\n", __func__, sc->id);
		__raw_writel(__raw_readl(cfg_path) & ~(1<<31), cfg_path);
		return -1;
	}

	do {
		mdelay(1);
		value = (__raw_readl(cfg_path)>>16) & 0x3;
		if ((value == VIOC_SC_CONNECTED) && en)
			return 0;
		else if ((value == VIOC_SC_DISCONNECTED) && !en)
			return 0;
	} while (loop--);

	printk("%s: scaler_%d path configuration error(2). device is busy\n", __func__, sc->id);
	return -1;
}

int vioc_sc_set_path(struct vioc_sc_device *sc, int en)
{
	void __iomem *cfg_path = NULL;
	unsigned value;
	unsigned int status, select;

	if (!sc)
		return -3;

	switch (sc->id) {
		case 0:
		case 1:
		case 2:
		case 3:
			cfg_path = (void __iomem *)io_p2v(TCC_PA_VIOC_CFGINT + 0x44 + 0x4*sc->id);
		case 4:
			cfg_path = (void __iomem *)io_p2v(TCC_PA_VIOC_CFGINT + 0xF8);
		default:
			return -2;
	}

	if (en) {
		value = __raw_readl(cfg_path);
		status = (value >> 16) & 0x3;
		select = value & 0xFF;
		if ((status == VIOC_SC_CONNECTED) && (sc->path != select)) {
			printk("%s, id:%d disconnect old scaler path(%d). (new: %d)\n", __func__, sc->id, select, sc->path);
			vioc_sc_path_ctrl(sc, cfg_path, 0);
		}
		return vioc_sc_path_ctrl(sc, cfg_path, 1);
	}
	else {
		status = (__raw_readl(cfg_path) >> 16) & 0x3;
		if (status == VIOC_SC_DISCONNECTED)
			return 0;
		return vioc_sc_path_ctrl(sc, cfg_path, 0);
	}

	return 0;
}

void vioc_sc_set_image(struct vioc_sc_device *sc, unsigned int en)
{
	volatile unsigned value;

	if (!sc)
		return;

	if (!en) {
		value = __raw_readl(sc->reg + SCCTRL) & ~(1<<16 | 1<<0);
		__raw_writel(value, sc->reg + SCCTRL);
		return;
	}

	/* set bypass */
	value = __raw_readl(sc->reg + SCCTRL) & ~(1<<0);
	__raw_writel(value | (sc->data.bypass ? 1 : 0), sc->reg + SCCTRL);

	/* set dst size */
	value = (sc->data.dst_height&0xFFFF) << 16 | (sc->data.dst_width&0xFFFF);
	__raw_writel(value, sc->reg + SCDSIZE);

	/* set out position */
	value = (sc->data.out_y&0xFFFF) << 16 | (sc->data.out_x&0xFFFF);
	__raw_writel(value, sc->reg + SCOPOS);

	/* set out size */
	value = (sc->data.out_height&0xFFFF) << 16 | (sc->data.out_width&0xFFFF);
	__raw_writel(value, sc->reg + SCOSIZE);

	/* set plug-in */
	vioc_sc_set_path(sc, 1);

	/* set update */
	value = __raw_readl(sc->reg + SCCTRL) & ~(1<<16);
	__raw_writel(value | ((!!en) << 16), sc->reg + SCCTRL);
}

/*
 * reset: 1(reset), 0(normal)
 */
void vioc_sc_swreset(struct vioc_sc_device *sc, int reset)
{
	if (!sc)
		return;

	/* sc4/5 does not support swreset */
	if (sc->id > 3)
		return;

	if (reset)
		__raw_writel(__raw_readl(sc->rst_reg) | (1<<(sc->id+SC_SWRST_OFF)), sc->rst_reg);
	else
		__raw_writel(__raw_readl(sc->rst_reg) & ~(1<<(sc->id+SC_SWRST_OFF)), sc->rst_reg);
}

static struct vioc_sc_device *of_vioc_sc_get(struct device_node *np, int index)
{
	struct vioc_sc_device *sc = NULL;
	struct of_phandle_args args;
	int err;

	err = of_parse_phandle_with_args(np, "scalers", "#vioc_sc-cells", index, &args);
	if (err) {
		pr_debug("%s(): can't parse \"scalers\" property(path)\n", __func__);
		return ERR_PTR(err);
	}

	sc = kzalloc(sizeof(struct vioc_sc_device), GFP_KERNEL);
	if (!sc) {
		err = -ENOMEM;
		goto err_alloc_sc_dev;
	}

	sc->intr = kzalloc(sizeof(struct vioc_intr_type), GFP_KERNEL);
	if (!sc->intr) {
		err = -ENOMEM;
		goto err_alloc_vioc_intr;
	}

	sc->reg = of_iomap(args.np, 0);
	sc->rst_reg = of_iomap(args.np, 1);
	sc->id = of_alias_get_id(args.np, "vioc_sc");
	if (sc->id < 0) {
		err = -ENODEV;
		goto err_get_property;
	}
	sc->irq = of_irq_to_resource(args.np, 0, NULL);
	sc->intr->id = VIOC_INTR_SC0 + sc->id;
	sc->intr->bits = VIOC_SC_INT_MASK;

	if (args.args_count)
		sc->path = args.args[0];
	else
		sc->path = VIOC_SCPATH_MAX;

	of_node_put(args.np);

	return sc;

err_get_property:
	kfree(sc->intr);
err_alloc_vioc_intr:
	kfree(sc);
err_alloc_sc_dev:

	return ERR_PTR(err);
}

static void vioc_sc_put(struct vioc_sc_device *sc)
{
	if (!sc)
		return;
	if (sc->intr)
		kfree(sc->intr);
	kfree(sc);
}

static void devm_vioc_sc_release(struct device *dev, void *res)
{
	vioc_sc_put(*(struct vioc_sc_device **)res);
}

int get_count_vioc_sc(struct device *dev)
{
	return of_count_phandle_with_args(dev->of_node, "scalers", "#vioc_sc-cells");
}

struct vioc_sc_device *devm_vioc_sc_get(struct device *dev, int index)
{
	struct vioc_sc_device **ptr, *sc = ERR_PTR(-ENODEV);

	ptr = devres_alloc(devm_vioc_sc_release, sizeof(**ptr), GFP_KERNEL);
	if (!ptr)
		return ERR_PTR(-ENOMEM);

	sc = of_vioc_sc_get(dev->of_node, index);	/* get a sc */
	if (!IS_ERR(sc)) {
		*ptr = sc;
		devres_add(dev, ptr);
	} else
		devres_free(ptr);

	return sc;
}
