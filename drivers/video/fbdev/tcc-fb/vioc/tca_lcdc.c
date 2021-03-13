/****************************************************************************
 * linux/drivers/video/tca_lcdc.c
 *
 * Author:  <linux@telechips.com>
 * Created: March 18, 2012
 * Description: TCC lcd Driver
 *
 * Copyright (C) 20010-2011 Telechips 
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
 *****************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/div64.h>
#include <asm/mach/map.h>
#ifdef CONFIG_PM
#include <linux/pm.h>
#endif

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_address.h>
#endif

#ifdef CONFIG_ARCH_TCC897X
#include <mach/bsp.h>
#include <mach/tcc_fb.h>
#include <mach/tca_lcdc.h>
#include <mach/tccfb_ioctrl.h>
#include <mach/vioc_outcfg.h>
#include <mach/vioc_rdma.h>
#include <mach/vioc_wdma.h>
#include <mach/vioc_wmix.h>
#include <mach/vioc_disp.h>
#include <mach/tccfb.h>
#else
#include <video/tcc/tcc_fb.h>
#include <video/tcc/tca_lcdc.h>
#include <video/tcc/tccfb_ioctrl.h>
#include <video/tcc/vioc_outcfg.h>
#include <video/tcc/vioc_rdma.h>
#include <video/tcc/vioc_wdma.h>
#include <video/tcc/vioc_wmix.h>
#include <video/tcc/vioc_disp.h>
#include <video/tcc/tccfb.h>
#endif

#if defined(CONFIG_TCC_VIOCMG)
#include <video/tcc/viocmg.h>
#endif



void tca_lcdc_interrupt_onoff(char onoff, char lcdc)
{
	VIOC_DISP * pDISPBase;
	unsigned int mask;

	pDISPBase = VIOC_DISP_GetAddress(lcdc);
	if (unlikely(pDISPBase == NULL)) {
		return;
	}

	// init INT mask reg
	pDISPBase->uLSTATUS.nREG = 0x39;

#if defined(CONFIG_TCC_M2M_USE_INTERLACE_OUTPUT)
	if(pDISPBase->uCTRL.nREG & HwDISP_NI)	//Non-interlace mode
#endif
	mask = (VIOC_DISP_IREQ_RU_MASK);

	#if defined(CONFIG_VIOC_FIFO_UNDER_RUN_COMPENSATION)
		mask |= (VIOC_DISP_IREQ_FU_MASK | VIOC_DISP_IREQ_DD_MASK);
	#endif

	if(onoff)		//VIOC INT en
	{
		VIOC_DISP_SetIreqMask(pDISPBase, mask, 0);
	}
	else			//VIOC INT dis
	{
		VIOC_DISP_SetIreqMask(pDISPBase, mask, mask);
	}
}
EXPORT_SYMBOL(tca_lcdc_interrupt_onoff);

void lcdc_initialize(struct lcd_panel *lcd_spec, struct tcc_dp_device *pdata)
{
	
	VIOC_DISP *pDISPBase;
	VIOC_WMIX *pWMIXBase;

	pDISPBase = (VIOC_DISP *)pdata->ddc_info.virt_addr;
	pWMIXBase = (VIOC_WMIX *)pdata->wmixer_info.virt_addr;
	
	VIOC_WMIX_SetSize(pWMIXBase , lcd_spec->xres, lcd_spec->yres);	

        #if defined(CONFIG_TCC_VIOCMG)
        viocmg_set_wmix_ovp(VIOCMG_CALLERID_FB, pdata->wmixer_info.blk_num, viocmg_get_main_display_ovp());
        #else
	VIOC_WMIX_SetOverlayPriority(pWMIXBase , 24);
        #endif
        
	VIOC_WMIX_SetBGColor(pWMIXBase , 0, 0, 0, 0);
	BITCSET(pDISPBase->uCTRL.nREG, 1 << DCTRL_Y2R, 0 << DCTRL_Y2R);


	if (lcd_spec->bus_width == 24)
		VIOC_DISP_SetPXDW(pDISPBase, 0xC);
	else if(lcd_spec->bus_width == 18)
		VIOC_DISP_SetPXDW(pDISPBase, 0x5);
	else
		VIOC_DISP_SetPXDW(pDISPBase, 0x3);

	#ifdef CONFIG_ARCH_TCC898X
	if (lcd_spec->bus_width <= 24)
		VIOC_DISP_SetAlign(pDISPBase, 0x1);	// Convert to 10-to-8 bits bus
	else
		VIOC_DISP_SetAlign(pDISPBase, 0x0);
	#endif

	if(lcd_spec->sync_invert & ID_INVERT)
		BITCSET(pDISPBase->uCTRL.nREG, 1 << DCTRL_ID , 1 << DCTRL_ID);

	if(lcd_spec->sync_invert & IV_INVERT)
		BITCSET(pDISPBase->uCTRL.nREG, 1 << DCTRL_IV , 1 << DCTRL_IV);

	if(lcd_spec->sync_invert & IH_INVERT)
		BITCSET(pDISPBase->uCTRL.nREG, 1 << DCTRL_IH , 1 << DCTRL_IH);

	if(lcd_spec->sync_invert & IP_INVERT)
		BITCSET(pDISPBase->uCTRL.nREG, 1 << DCTRL_IP , 1 << DCTRL_IP);


	BITCSET(pDISPBase->uCTRL.nREG, 1 << 23 , 0 << 23);
	BITCSET(pDISPBase->uCTRL.nREG, 1 << 22 , 0 << 22);
	
	BITCSET(pDISPBase->uCTRL.nREG, 1 << 8 , 1 << 8);

	BITCSET(pDISPBase->uCLKDIV.nREG,0x00FF0000,  1 << 16 );
	BITCSET(pDISPBase->uCLKDIV.nREG,0x000000FF, lcd_spec->clk_div/2);

	#ifdef CONFIG_ARCH_TCC897X
	BITCSET(pDISPBase->uBG.nREG,0xFFFFFFFF,0x00000000);
	#else
	BITCSET(pDISPBase->uBG.nREG[0],0xFFFFFFFF,0x00000000);
	BITCSET(pDISPBase->uBG.nREG[1],0xFFFFFFFF,0x00000000);
	#endif

	BITCSET(pDISPBase->uLHTIME1.nREG, 0x00003FFF, (lcd_spec->xres - 1) );
	BITCSET(pDISPBase->uLHTIME1.nREG, 0x01FF0000, lcd_spec->lpw<< 16 );	
	BITCSET(pDISPBase->uLHTIME2.nREG, 0x01FF01FF, (lcd_spec->lswc << 16) | lcd_spec->lewc );

	BITCSET(pDISPBase->uLVTIME1.nREG, 0x3F << 16 , lcd_spec->fpw1 << 16);	
	BITCSET(pDISPBase->uLVTIME1.nREG, 0x00003FFF, lcd_spec->yres -1 );
	BITCSET(pDISPBase->uLVTIME2.nREG, 0x01FF01FF, (lcd_spec->fswc1 << 16) | lcd_spec->fewc1 );
		
	BITCSET(pDISPBase->uLVTIME3.nREG, 0x3F << 16 , lcd_spec->fpw2 << 16);	
	BITCSET(pDISPBase->uLVTIME3.nREG, 0x00003FFF, lcd_spec->yres -1);
	BITCSET(pDISPBase->uLVTIME4.nREG, 0x01FF01FF, (lcd_spec->fswc2 << 16) | lcd_spec->fewc2 );
	
	BITCSET(pDISPBase->uLSIZE.nREG, 0x1FFF1FFF, (lcd_spec->yres << 16) | lcd_spec->xres );

//pjj
//	BITCSET(pDISPBase->uCTRL.nREG,1,1);

}

EXPORT_SYMBOL(lcdc_initialize);

void tcc_lcdc_dithering_setting(struct tcc_dp_device *pdata)
{
	VIOC_DISP * pDISPBase = pdata->ddc_info.virt_addr;

	/* dithering option */
	BITCSET(pDISPBase->uCTRL.nREG, 0xF << DCTRL_PXDW, 0x5 << DCTRL_PXDW) ;
	BITCSET(pDISPBase->uDITHCTRL.nREG, 0xFFFFFFFF, 0xC0000000) ;
	BITCSET(pDISPBase->uDMAT.nREG[0],  0xFFFFFFFF, 0x6e4ca280) ;
	BITCSET(pDISPBase->uDMAT.nREG[1],  0xFFFFFFFF, 0x5d7f91b3) ;
}


/*===========================================================================
FUNCTION
 - index : 0 = lcdc0, 1 = lcdc1
===========================================================================*/
unsigned int DEV_LCDC_Wait_signal(char lcdc)
{
	// TO DO
	#define MAX_LCDC_WAIT_TIEM 		0x70000000

	unsigned loop = 1;
	VIOC_DISP * pDISPBase;
	VIOC_RDMA * pRDMABase;
	PDDICONFIG pDDICONFIG = VIOC_DDICONFIG_GetAddress();

#if 0 //pjj
	PPMU pPMU = (PPMU)(tcc_p2v(HwPMU_BASE));
	PCKC pCKC  = (PCKC)(tcc_p2v(HwCKC_BASE)); 
	if ( ISZERO(pCKC->CLKCTRL2.nREG, Hw21) || ISZERO(pPMU->PMU_PWRSTS.nREG, Hw2) )
		return false;
#else
	struct clk *ddi_clk;
	 ddi_clk = clk_get(NULL, "ddi");
	 if (IS_ERR(ddi_clk)) {
		 ddi_clk = NULL;
		 goto failed_get_clks;
	}

	 if(!(__clk_is_enabled(ddi_clk)))
	 	return false;
#endif//

	pDISPBase = VIOC_DISP_GetAddress(lcdc);
	if (unlikely(pDISPBase == NULL)) {
		return false;
	}

	if(lcdc == 0 ){
 		pRDMABase = VIOC_RDMA_GetAddress(0);
		if (unlikely(pRDMABase == NULL)) {
			return false;
		}

		if(ISZERO(pDDICONFIG->PWDN.nREG, HwDDIC_PWDN_LCDC))
			return false;
	}
	else{
 		pRDMABase = VIOC_RDMA_GetAddress(4);
		if (unlikely(pRDMABase == NULL)) {
			return false;
		}
 
		if(ISZERO(pDDICONFIG->PWDN.nREG, HwDDIC_PWDN_LCDC))
			return false;
	}
	
	if ( ISZERO(pDISPBase->uCTRL.nREG, HwDISP_LEN) )
		return false;

	// UI rdma channel check enable.
	if ( ISZERO(pRDMABase->uCTRL.nREG, HwDMA_IEN))
		return false;

	BITSET(pRDMABase->uSTATUS.nREG, Hw4);

	while(true && (loop < MAX_LCDC_WAIT_TIEM))
	{
		if(ISSET(pRDMABase->uSTATUS.nREG, Hw4))
			break;
		loop++;
	}
#if 0
	loop=1;
	while(true && (loop < MAX_LCDC_WAIT_TIEM) )
	{
		if(ISSET(pDISPBase->uLSTATUS.nREG , HwLSTATUS_DEOF))
			break;
		loop++;
	}
#endif//

 failed_get_clks:
 	return loop;
}
EXPORT_SYMBOL(DEV_LCDC_Wait_signal);

unsigned int DEV_LCDC_Wait_signal_Ext(void)
{
	#define MAX_LCDC_WAIT_TIEM 		0x70000000

	struct clk *ddi_clk;
	int disp_en0=0, disp_en1=0;
	VIOC_DISP *pDISPBase0 = VIOC_DISP_GetAddress(0);
	VIOC_DISP *pDISPBase1 = VIOC_DISP_GetAddress(1);
	VIOC_OUTCFG *pOUTCFGBase = VIOC_OUTCONFIG_GetAddress();
	PDDICONFIG pDDICONFIG = VIOC_DDICONFIG_GetAddress();

	if (unlikely(pDISPBase0 == NULL
			|| pDISPBase1 == NULL
			|| pOUTCFGBase == NULL
			|| pDDICONFIG == NULL)) {
		return false;
	}

	ddi_clk = clk_get(NULL, "ddi");
	if (IS_ERR(ddi_clk)) {
		ddi_clk = NULL;
		goto failed_get_clks;
	}

	if(!(__clk_is_enabled(ddi_clk)))
		return false;

	if(ISZERO(pDDICONFIG->PWDN.nREG, HwDDIC_PWDN_LCDC))
		return false;
			
	if(ISSET(pDISPBase0->uCTRL.nREG, HwDISP_LEN))
		disp_en0 = 1;

	if(ISSET(pDISPBase1->uCTRL.nREG, HwDISP_LEN))
		disp_en1 = 1;

	if(disp_en0 && disp_en1)
	{
		if((pOUTCFGBase->uMISCCFG.nREG & 0x3) == 1)
		{
			DEV_LCDC_Wait_signal(0);
			DEV_LCDC_Wait_signal(1);
		}
		else
		{
			DEV_LCDC_Wait_signal(1);
			DEV_LCDC_Wait_signal(0);
		}
	}
	else
	{
		if(disp_en0)
			DEV_LCDC_Wait_signal(0);
			
		if(disp_en1)
			DEV_LCDC_Wait_signal(1);
	}
 failed_get_clks:
 	return 1;
}
EXPORT_SYMBOL(DEV_LCDC_Wait_signal_Ext);
