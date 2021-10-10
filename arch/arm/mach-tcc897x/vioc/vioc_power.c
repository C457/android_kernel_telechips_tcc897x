/*
 * linux/arch/arm/mach-tcc893x/vioc_power.c
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
#include <mach/vioc_outcfg.h>


void VIOC_OUTCFG_SetOutConfig (unsigned nType, unsigned nDisp)
{
	static VIOC_OUTCFG *gpOutConfig = (VIOC_OUTCFG *)tcc_p2v(HwVIOC_OUTCFG);

	switch (nType)
	{
		case VIOC_OUTCFG_HDMI :
			gpOutConfig->uMISCCFG.bREG.HDMISEL   = nDisp;
			break;
		case VIOC_OUTCFG_SDVENC :
			gpOutConfig->uMISCCFG.bREG.SDVESEL   = nDisp;
			break;
		case VIOC_OUTCFG_HDVENC :
			gpOutConfig->uMISCCFG.bREG.HDVESEL   = nDisp;
			break;
		case VIOC_OUTCFG_M80 :
			gpOutConfig->uMISCCFG.bREG.M80SEL    = nDisp;
			break;
		case VIOC_OUTCFG_MRGB :
			gpOutConfig->uMISCCFG.bREG.MRGBSEL   = nDisp;
			break;
		default :
			printk("Not supported type ...");
			break;
	}
}

// vim:ts=4:et:sw=4:sts=4
