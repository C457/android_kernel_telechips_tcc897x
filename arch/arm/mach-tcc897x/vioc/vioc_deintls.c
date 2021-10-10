/*
 * linux/arch/arm/mach-tcc893x/vioc_rdma.c
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
#include <linux/delay.h>
#include <linux/of_address.h>
#include <asm/io.h>
#include <mach/bsp.h>
#include <mach/vioc_deintls.h>

void VIOC_DEINTLS_SetDeIntlMode(unsigned int * pDeintls, unsigned int mode) {
//	printk("%s() - DeInterlacing mode = 0x%x\n", __func__, mode);
	BITCSET(*pDeintls, 0x0000000F, mode);
}

struct device_node *ViocDeintls_np;

static int __init vioc_deintls_init(void)
{
	ViocDeintls_np = of_find_compatible_node(NULL, NULL, "telechips,vioc_deintls");
	if(ViocDeintls_np == NULL)
		pr_err("cann't find vioc deintls \n");
	return 0;
}
arch_initcall(vioc_deintls_init);

