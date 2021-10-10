
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
#include <asm/io.h>
#include <mach/bsp.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>

#include <mach/vioc_mc.h>

void    VIOC_MC_Start_OnOff   (VIOC_MC *pMC, uint OnOff)
{
	BITCSET(pMC->uCTRL.nReg, 0x00010001, (1<<16) | (OnOff & 0x1) );
//    pMC->uCTRL.bReg.START   =   OnOff;
//    pMC->uCTRL.bReg.UPD     =   1;
}

void    VIOC_MC_UPD (VIOC_MC *pMC)
{
	BITCSET(pMC->uCTRL.nReg, 0x00010000, (1<<16));
//	    pMC->uCTRL.bReg.UPD     =   1;
}

void    VIOC_MC_Y2R_OnOff   (VIOC_MC *pMC, uint OnOff)
{
	BITCSET(pMC->uCTRL.nReg, (1<<19) , ((OnOff & 0x1)<<19)  );
//    pMC->uCTRL.bReg.START   =   OnOff;
//    pMC->uCTRL.bReg.UPD     =   1;
}

void    VIOC_MC_Start_BitDepth(VIOC_MC *pMC, uint Chroma, uint Luma)
{
	BITCSET(pMC->uCTRL.nReg, 0x000000F0, (Chroma<<6)|(Luma<<4));
}
void    VIOC_MC_OFFSET_BASE (VIOC_MC *pMC, uint base_y, uint base_c)
{
	BITCSET(pMC->uOFS_BASE_Y.nReg, 0xFFFFFFFF, base_y);
	BITCSET(pMC->uOFS_BASE_C.nReg, 0xFFFFFFFF, base_c);
//    pMC->uOFS_BASE_Y.bReg.base  =   base_y;
//    pMC->uOFS_BASE_C.bReg.base  =   base_c;
}

void    VIOC_MC_FRM_BASE    (VIOC_MC *pMC, uint base_y, uint base_c)
{
	BITCSET(pMC->uFRM_BASE_Y.nReg, 0xFFFFFFFF, base_y);
	BITCSET(pMC->uFRM_BASE_C.nReg, 0xFFFFFFFF, base_c);
//    pMC->uFRM_BASE_Y.bReg.base  =   base_y;
//    pMC->uFRM_BASE_C.bReg.base  =   base_c;	
}

void    VIOC_MC_FRM_SIZE(VIOC_MC *pMC, uint xsize, uint ysize)
{
	BITCSET(pMC->uFRM_SIZE.nReg, 0xFFFFFFFF, (ysize<<16) | (xsize));
//    pMC->uFRM_SIZE.bReg.xsize   =   xsize;
//    pMC->uFRM_SIZE.bReg.ysize   =   ysize;
}

void    VIOC_MC_FRM_SIZE_MISC   (VIOC_MC *pMC, uint pic_height, uint stride_y, uint stride_c)
{
	BITCSET(pMC->uPIC.nReg, 0x3FFF, pic_height);
	BITCSET(pMC->uFRM_STRIDE.nReg, 0xFFFFFFFF, (stride_c<<16) | (stride_y));
//    pMC->uPIC.bReg.pic_height   =   pic_height;
//    pMC->uFRM_STRIDE.bReg.frame_stride_y =   stride_y;
//    pMC->uFRM_STRIDE.bReg.frame_stride_c =   stride_c;
}

void    VIOC_MC_FRM_POS (VIOC_MC *pMC,  uint xpos, uint ypos)
{
	BITCSET(pMC->uFRM_POS.nReg, 0xFFFFFFFF, (ypos<<16) | (xpos));
//    pMC->uFRM_POS.bReg.xpos = xpos;
//    pMC->uFRM_POS.bReg.ypos = ypos;
}

void    VIOC_MC_ENDIAN  (VIOC_MC *pMC, uint ofs_endian, uint comp_endian)
{
	BITCSET(pMC->uFRM_MISC0.nReg, 0xF0, ofs_endian<<4);
	BITCSET(pMC->uFRM_MISC0.nReg, 0xF00, comp_endian<<8);
//    pMC->uFRM_MISC0.bReg.ofs_endian     = ofs_endian;
//    pMC->uFRM_MISC0.bReg.comp_endian    = comp_endian;
}
void    VIOC_MC_DITH_CONT(VIOC_MC *pMC, uint en, uint sel)
{
	BITSET(pMC->uDITH_CTRL.nReg, en | sel<<8);
}


struct device_node *ViocMC_np;

VIOC_MC* VIOC_MC_GetAddress(unsigned int Num)
{
	VIOC_MC *pRdma = of_iomap(ViocMC_np, Num);

	if(pRdma == NULL)
		pr_err("%s num:%d \n", __func__, Num);

	return pRdma;
}

void VIOC_MC_DUMP(unsigned int Num)
{
	VIOC_MC *pMC;
	struct device_node *ViocMapCV_np;
	char *pReg;
	unsigned int cnt = 0;

	ViocMapCV_np = of_find_compatible_node(NULL, NULL, "telechips,vioc_mc");
	if(ViocMapCV_np == NULL )
		return;

	pMC = (VIOC_MC *)of_iomap(ViocMapCV_np, 1);
	pReg = (char*)pMC;

	printk("MapConvertor :: %p \n", pMC);
	while(cnt < 0x100)
	{
		printk("%p: 0x%08lx 0x%08lx 0x%08lx 0x%08lx\n",
			pReg+cnt,
			*((unsigned long*)(pReg+cnt)),
			*((unsigned long*)(pReg+cnt+0x4)),
			*((unsigned long*)(pReg+cnt+0x8)),
			*((unsigned long*)(pReg+cnt+0xC)));
		cnt += 0x10;
	}
}

static int __init vioc_mc_init(void)
{

	ViocMC_np = of_find_compatible_node(NULL, NULL, "telechips,vioc_mc");
	if(ViocMC_np == NULL)
		pr_err("cann't find vioc MC \n");

	pr_info("%s addr0:0x%p,   addr1:0x%p \n",__func__, of_iomap(ViocMC_np, 0), of_iomap(ViocMC_np, 1)); 

	return 0;
}
arch_initcall(vioc_mc_init);
