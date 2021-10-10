/****************************************************************************
linux/arch/arm/mach-tcc893x/tcc893x-ion.c
Description: TCC ION device driver

Copyright (C) 2013 Telechips Inc.

This program is free software; you can redistribute it and/or modify it under the terms
of the GNU General Public License as published by the Free Software Foundation;
either version 2 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program; if not, write to the Free Software Foundation, Inc., 59 Temple Place,
Suite 330, Boston, MA 02111-1307 USA
****************************************************************************/

#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <mach/tcc_ion.h>
#include <soc/tcc/pmap.h>
#include "uapi/ion.h"
#include "ion/ion.h"

struct platform_device tcc_ion_device = {
	.name	= "ion-tcc",
	.id		= -1,
};

void __init tcc_ion_set_platdata(void)
{
	struct ion_platform_data *pdata;
	pdata = kzalloc(sizeof(struct ion_platform_data)
			+ 4 * sizeof(struct ion_platform_heap), GFP_KERNEL);
	pmap_t pmap_ump_reserved;
	pmap_get_info("ump_reserved", &pmap_ump_reserved);

	if (pdata) {
		pdata->nr = 3;
		pdata->heaps[0].type = ION_HEAP_TYPE_SYSTEM;
		pdata->heaps[0].name = "ion_noncontig_heap";
		pdata->heaps[0].id = ION_HEAP_TYPE_SYSTEM;
		pdata->heaps[1].type = ION_HEAP_TYPE_SYSTEM_CONTIG;
		pdata->heaps[1].name = "ion_contig_heap";
		pdata->heaps[1].id = ION_HEAP_TYPE_SYSTEM_CONTIG;		
		pdata->heaps[2].type = ION_HEAP_TYPE_CARVEOUT;
		pdata->heaps[2].name = "ion_carveout_heap";
		pdata->heaps[2].id = ION_HEAP_TYPE_CARVEOUT;
		pdata->heaps[2].base = pmap_ump_reserved.base;
		pdata->heaps[2].size = pmap_ump_reserved.size;
//		pdata->heaps[2].align = ;
/*
		pdata->heaps[2].type = ION_HEAP_TYPE_EXYNOS;
		pdata->heaps[2].name = "exynos_noncontig_heap";
		pdata->heaps[2].id = ION_HEAP_TYPE_EXYNOS;
		pdata->heaps[3].type = ION_HEAP_TYPE_EXYNOS_CONTIG;
		pdata->heaps[3].name = "exynos_contig_heap";
		pdata->heaps[3].id = ION_HEAP_TYPE_EXYNOS_CONTIG;
*/
		tcc_ion_device.dev.platform_data = pdata;
	}
}

