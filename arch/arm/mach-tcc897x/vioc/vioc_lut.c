/*
 * linux/arch/arm/mach-tcc893x/vioc_lut.c
 * Author:  <linux@telechips.com>
 * Created: June 10, 2008
 * Description: TCC VIOC h/w block 
 *
 * Copyright (C) 2008-2009 Telechips
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <linux/kernel.h>
#include <linux/module.h>

#include <asm/io.h>
#include <mach/vioc_lut.h>
#include <mach/vioc_blk.h>

#include <mach/globals.h>
#include <linux/slab.h>
#include <linux/of_address.h>
#include <asm/io.h>

static void __iomem	*lut_base;
#define REG_VIOC_LUT(offset)     		(lut_base + (offset))
#define LUT_CTRL_R                    		REG_VIOC_LUT(0)
#define LUT_CONFIG_R(x)                	REG_VIOC_LUT(0x04 + (4*x))
#define LUT_TABLE_R					REG_VIOC_LUT(0x400)

// LUT Control
#define L_TABLE_SEL_SHIFT		0
#define L_TABLE_SEL_MASK		(0xF << L_TABLE_SEL_SHIFT)

// LUT VIOCk Config
#define L_CONFIG_SEL_SHIFT		0
#define L_CONFIG_SEL_MASK		(0xFF << L_CONFIG_SEL_SHIFT)

#define L_CFG_EN_SHIFT			31
#define L_CFG_EN_MASK			BIT(L_CFG_EN_SHIFT)

//vioc lut config register write & read
#define lut_writel		writel	
#define lut_readl		readl


int lut_get_pluginComponent_index(unsigned int tvc_n)
{
	if(tvc_n <= TVC_RDMA(15))
		return (tvc_n - TVC_RDMA(0));
	else if(tvc_n == TVC_RDMA(16))
		return 0x11;
	else if(tvc_n == TVC_RDMA(17))
		return 0x13;
	else if(tvc_n == TVC_VIN(0))
		return 0x10;	
	else if(tvc_n == TVC_VIN(1))
		return 0x12;	
	else if((tvc_n >= TVC_WDMA(0)) && (tvc_n < TVC_WDMA(TCV_WDMA_N)))
		return (0x14 + (tvc_n - (TVC_WDMA(0))));
	else 
		return -1;

	return -1;
}

int lut_get_Component_index_to_tvc(unsigned int plugin_n)
{
	if(plugin_n <= 0xf)
		return TVC_RDMA(plugin_n);
	else if(plugin_n == 0x10)
		return TVC_VIN(0);
	else if(plugin_n == 0x11)
		return TVC_RDMA(16);
	else if(plugin_n == 0x12)
		return TVC_VIN(1);
	else if(plugin_n == 0x13)
		return TVC_RDMA(17);
	else if(/*(plugin_n >= 0x14) &&*/ (plugin_n <= 0x1c))
		return (TVC_WDMA(0) + (plugin_n - 0x14));

	return -1;
}

void tcc_set_lut_table(unsigned int lut_n, unsigned int *table)
{
	unsigned int i, reg_off, lut_index;
	void __iomem	*table_reg = (void __iomem *)LUT_TABLE_R;
	void __iomem	*ctrl_reg = (void __iomem *)LUT_CTRL_R;
		
	
	//lut table select
	lut_index = lut_n - TVC_LUT(0);
	lut_writel(lut_index, ctrl_reg);

	// lut table setting
	for(i = 0; i < LUT_TABLE_SIZE; i++)
	{
		reg_off = (0xFF & i);
		lut_writel(table[i], table_reg + (reg_off * 0x4));
	}
}

void tcc_set_lut_plugin(unsigned int lut_n, unsigned int plugComp)
{
	void __iomem	*reg;

	unsigned int value , lut_index;

	lut_index = lut_n - TVC_LUT(0);

	// select lut config register
	reg = (void __iomem *)LUT_CONFIG_R(lut_index);
	value = lut_readl(reg);
	value = (value & ~(L_CONFIG_SEL_MASK)) | (lut_get_pluginComponent_index(plugComp) << L_TABLE_SEL_SHIFT);
	lut_writel(value , reg);
}

int tcc_get_lut_plugin (unsigned int lut_n)
{
	void __iomem	*reg;
	unsigned int value , lut_index, ret;
		
	lut_index = lut_n - TVC_LUT(0);

	
	reg = (void __iomem *)LUT_CONFIG_R(lut_index);
	value = lut_readl(reg);

	ret = lut_get_Component_index_to_tvc(value & (L_CONFIG_SEL_MASK));

	return ret;
}

void tcc_set_lut_enable(unsigned int lut_n, unsigned int enable)
{
	void __iomem	*reg;
	unsigned int lut_index;

	lut_index = lut_n - TVC_LUT(0);
	reg = (void __iomem *)LUT_CONFIG_R(lut_index);

	//enable , disable 
	if(enable)
		lut_writel(readl(reg) | (L_CFG_EN_MASK), reg);
	else
		lut_writel(readl(reg) & (~(L_CFG_EN_MASK)), reg);
}

int tcc_get_lut_enable(unsigned int lut_n)
{
	void __iomem	*reg;
	unsigned int lut_index;

	lut_index = lut_n - TVC_LUT(0);
	reg = (void __iomem *)LUT_CONFIG_R(lut_index);

	//enable , disable 
	if(lut_readl(reg) & (L_CFG_EN_MASK))
		return 1;
	else
		return 0;
}


static int __init vioc_lut_init(void)
{
	struct device_node *ViocLUT_np;

	ViocLUT_np = of_find_compatible_node(NULL, NULL, "telechips,vioc_lut");
	if(ViocLUT_np == NULL)
		pr_err("cann't find vioc lut \n");

	lut_base = of_iomap(ViocLUT_np, 0);
	return 0;
}
arch_initcall(vioc_lut_init);


MODULE_AUTHOR("linux <linux@telechips.com>");
MODULE_DESCRIPTION("Telechips TCC HUE driver");
MODULE_LICENSE("GPL");

