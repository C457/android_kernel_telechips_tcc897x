/*
 * linux/arch/arm/mach-tcc893x/vioc_wmix.c
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

#ifdef CONFIG_ARCH_TCC897X
#include <mach/bsp.h>
#include <mach/io.h>

#include <mach/vioc_config.h>
#include <mach/vioc_wmix.h>
#include <mach/tccfb_ioctrl.h>
#include <mach/vioc_global.h>
#else
#include <video/tcc/vioc_config.h>
#include <video/tcc/vioc_wmix.h>
#include <video/tcc/tccfb_ioctrl.h>
#include <video/tcc/vioc_global.h>
#endif

#define VIOC_WMIX_IREQ_MUPD_MASK	0x00000001UL
#define VIOC_WMIX_IREQ_MEOFWF_MASK	0x00000002UL
#define VIOC_WMIX_IREQ_MEOFWR_MASK 	0x00000004UL
#define VIOC_WMIX_IREQ_MEOFR_MASK 	0x00000008UL
#define VIOC_WMIX_IREQ_MEOFF_MASK 	0x00000010UL

void VIOC_WMIX_SetOverlayPriority(VIOC_WMIX *pWMIX, unsigned int nOverlayPriority)
{
	BITCSET(pWMIX->uCTRL.nREG, 0x0000001F, nOverlayPriority);
}

void VIOC_WMIX_GetOverlayPriority(VIOC_WMIX *pWMIX, unsigned int *nOverlayPriority)
{
	*nOverlayPriority = pWMIX->uCTRL.nREG & 0x1F;
}

void VIOC_WMIX_SetUpdate(VIOC_WMIX *pWMIX)
{
	BITCSET(pWMIX->uCTRL.nREG, 1<<16, 1<<16);
}

void VIOC_WMIX_SetSize(VIOC_WMIX *pWMIX, unsigned int nWidth, unsigned int nHeight)
{
	BITCSET(pWMIX->uSIZE.nREG, 0xFFFFFFFF, (nHeight << 16) | (nWidth));
}

void VIOC_WMIX_GetSize(VIOC_WMIX *pWMIX, unsigned int *nWidth, unsigned int *nHeight)
{
	*nWidth = pWMIX->uSIZE.nREG & 0x0000FFFF;
	*nHeight = (pWMIX->uSIZE.nREG >> 16) & 0x0000FFFF;
}

void VIOC_WMIX_SetBGColor(VIOC_WMIX *pWMIX, unsigned int nBG0, unsigned int nBG1, unsigned int nBG2, unsigned int nBG3)
{
	BITCSET(pWMIX->uBG.nREG[0], 0xFFFFFFFF, nBG1 << 16 | nBG0);
	BITCSET(pWMIX->uBG.nREG[1], 0xFFFFFFFF, nBG3 << 16 | nBG2);
}

void VIOC_WMIX_SetPosition(VIOC_WMIX *pWMIX, unsigned int nChannel, unsigned int nX, unsigned int nY)
{
	switch (nChannel) {
	case 0:
		BITCSET(pWMIX->uPOS0.nREG, 0x1FFFFFFF,  nY << 16 | nX);
		break;
	case 1:
		BITCSET(pWMIX->uPOS1.nREG, 0x1FFFFFFF,  nY << 16 | nX);
		break;
	case 2:
		BITCSET(pWMIX->uPOS2.nREG, 0x1FFFFFFF,  nY << 16 | nX);
		break;
	case 3:
		BITCSET(pWMIX->uPOS3.nREG, 0x1FFFFFFF,  nY << 16 | nX);
		break;
	default:
		break;
	}
}

void VIOC_WMIX_GetPosition(VIOC_WMIX *pWMIX, unsigned int nChannel, unsigned int *nX, unsigned int *nY)
{
	switch (nChannel) {
	case 0:
		*nX = pWMIX->uPOS0.nREG & 0x00001FFF;
		*nY = pWMIX->uPOS0.nREG & 0x1FFF0000;
		break;
	case 1:
		*nX = pWMIX->uPOS1.nREG & 0x00001FFF;
		*nY = pWMIX->uPOS1.nREG & 0x1FFF0000;
		break;
	case 2:
		*nX = pWMIX->uPOS2.nREG & 0x00001FFF;
		*nY = pWMIX->uPOS2.nREG & 0x1FFF0000;
		break;
	case 3:
		*nX = pWMIX->uPOS3.nREG & 0x00001FFF;
		*nY = pWMIX->uPOS3.nREG & 0x1FFF0000;
		break;
	default:
		break;
	}
}

void VIOC_WMIX_SetChromaKey(VIOC_WMIX *pWMIX, unsigned int nLayer, unsigned int nKeyEn,
					unsigned int nKeyR, unsigned int nKeyG, unsigned int nKeyB,
					unsigned int nKeyMaskR, unsigned int nKeyMaskG, unsigned int nKeyMaskB)
{
	switch (nLayer) {
	case 0:    // top layer
		BITCSET(pWMIX->uKEY0.nREG[0], 0x8000FFFF, (nKeyEn << 31) | nKeyR);
		BITCSET(pWMIX->uKEY0.nREG[1], 0xFFFFFFFF, (nKeyG << 16) | nKeyB);
		BITCSET(pWMIX->uKEY0.nREG[2], 0x0000FFFF, nKeyMaskR);
		BITCSET(pWMIX->uKEY0.nREG[3], 0xFFFFFFFF, (nKeyMaskG << 16) | nKeyMaskB);
		break;
	case 1:
		BITCSET(pWMIX->uKEY1.nREG[0], 0x8000FFFF, (nKeyEn << 31) | nKeyR);
		BITCSET(pWMIX->uKEY1.nREG[1], 0xFFFFFFFF, (nKeyG << 16) | nKeyB);
		BITCSET(pWMIX->uKEY1.nREG[2], 0x0000FFFF, nKeyMaskR);
		BITCSET(pWMIX->uKEY1.nREG[3], 0xFFFFFFFF, (nKeyMaskG << 16) | nKeyMaskB);
		break;
	case 2:
		BITCSET(pWMIX->uKEY2.nREG[0], 0x8000FFFF, (nKeyEn << 31) | nKeyR);
		BITCSET(pWMIX->uKEY2.nREG[1], 0xFFFFFFFF, (nKeyG << 16) | nKeyB);
		BITCSET(pWMIX->uKEY2.nREG[2], 0x0000FFFF, nKeyMaskR);
		BITCSET(pWMIX->uKEY2.nREG[3], 0xFFFFFFFF, (nKeyMaskG << 16) | nKeyMaskB);
		break;
	default:
		break;
	}
}

void VIOC_WMIX_GetChromaKey(VIOC_WMIX *pWMIX, unsigned int nLayer, unsigned int *nKeyEn,
					unsigned int *nKeyR, unsigned int *nKeyG, unsigned int *nKeyB,
					unsigned int *nKeyMaskR, unsigned int *nKeyMaskG, unsigned int *nKeyMaskB)
{
	switch (nLayer) {
	case 0:    // top layer
		*nKeyEn = pWMIX->uKEY0.nREG[0] & 0x80000000;
		*nKeyR = pWMIX->uKEY0.nREG[0] & 0x0000FFFF;
		*nKeyG = pWMIX->uKEY0.nREG[1] & 0xFFFF0000;
		*nKeyB = pWMIX->uKEY0.nREG[1] & 0x0000FFFF;
		*nKeyMaskR = pWMIX->uKEY0.nREG[2] & 0x0000FFFF;
		*nKeyMaskG = pWMIX->uKEY0.nREG[3] & 0xFFFF0000;
		*nKeyMaskB = pWMIX->uKEY0.nREG[3] & 0x0000FFFF;
		break;
	case 1:
		*nKeyEn = pWMIX->uKEY1.nREG[0] & 0x80000000;
		*nKeyR = pWMIX->uKEY1.nREG[0] & 0x0000FFFF;
		*nKeyG = pWMIX->uKEY1.nREG[1] & 0xFFFF0000;
		*nKeyB = pWMIX->uKEY1.nREG[1] & 0x0000FFFF;
		*nKeyMaskR = pWMIX->uKEY1.nREG[2] & 0x0000FFFF;
		*nKeyMaskG = pWMIX->uKEY1.nREG[3] & 0xFFFF0000;
		*nKeyMaskB = pWMIX->uKEY1.nREG[3] & 0x0000FFFF;
		break;
	case 2:
		*nKeyEn = pWMIX->uKEY2.nREG[0] & 0x80000000;
		*nKeyR = pWMIX->uKEY2.nREG[0] & 0x0000FFFF;
		*nKeyG = pWMIX->uKEY2.nREG[1] & 0xFFFF0000;
		*nKeyB = pWMIX->uKEY2.nREG[1] & 0x0000FFFF;
		*nKeyMaskR = pWMIX->uKEY2.nREG[2] & 0x0000FFFF;
		*nKeyMaskG = pWMIX->uKEY2.nREG[3] & 0xFFFF0000;
		*nKeyMaskB = pWMIX->uKEY2.nREG[3] & 0x0000FFFF;
		break;
	default:
		break;
	}
}

void VIOC_WMIX_ALPHA_SetAlphaValueControl(VIOC_WMIX_ALPHA * pWALPHA,
					unsigned int region, unsigned int acon0, unsigned int acon1)
{
	switch (region) {
	case 0:	/*Region A*/
		//pWALPHA->uACON.bREG.ACON0_00 = acon0;
		//pWALPHA->uACON.bREG.ACON1_00 = acon1;
		BITCSET(pWALPHA->uACON.nREG, 0xFFFFFFFF, (acon1 << 4 | acon0) );
		break;
	case 1:	/*Region B*/
		//pWALPHA->uACON.bREG.ACON0_10 = acon0;
		//pWALPHA->uACON.bREG.ACON1_10 = acon1;
		BITCSET(pWALPHA->uACON.nREG,0xFFFFFFFF, (acon1 << 4 | acon0) << 16 );
		break;
	case 2:	/*Region C*/
		//pWALPHA->uACON.bREG.ACON0_11 = acon0;
		//pWALPHA->uACON.bREG.ACON1_11 = acon1;
		BITCSET(pWALPHA->uACON.nREG,0xFFFFFFFF, (acon1 << 4 | acon0) << 24 );
		break;
	case 3:	/*Region D*/
		//pWALPHA->uACON.bREG.ACON0_01 = acon0;
		//pWALPHA->uACON.bREG.ACON1_01 = acon1;
		BITCSET(pWALPHA->uACON.nREG,0xFFFFFFFF, (acon1 << 4 | acon0) << 8 );
		break;
	default:
		break;
	}

}

void VIOC_WMIX_ALPHA_SetColorControl( VIOC_WMIX_ALPHA * pWALPHA,
					unsigned int region, unsigned int ccon0, unsigned int ccon1)
{
	switch (region) {
	case 0:	/*Region A*/
		//pWALPHA->uCCON.bREG.CCON0_00 = ccon0;
		//pWALPHA->uCCON.bREG.CCON1_00 = ccon1;
		BITCSET(pWALPHA->uCCON.nREG,0xFFFFFFFF, ccon1 << 4 | ccon0 );
		break;
	case 1:	/*Region B*/
		//pWALPHA->uCCON.bREG.CCON0_10 = ccon0;
		//pWALPHA->uCCON.bREG.CCON1_10 = ccon1;
		BITCSET(pWALPHA->uCCON.nREG,0xFFFFFFFF, (ccon1 << 4 | ccon0) << 16 );
		break;
	case 2:	/*Region C*/
		//pWALPHA->uCCON.bREG.CCON0_11 = ccon0;
		//pWALPHA->uCCON.bREG.CCON1_11 = ccon1;
		BITCSET(pWALPHA->uCCON.nREG,0xFFFFFFFF, (ccon1 << 4 | ccon0) << 24 );
		break;
	case 3:	/*Region D*/
		//pWALPHA->uCCON.bREG.CCON0_01 = ccon0;
		//pWALPHA->uCCON.bREG.CCON1_01 = ccon1;
		BITCSET(pWALPHA->uCCON.nREG,0xFFFFFFFF, (ccon1 << 4 | ccon0) << 8 );
		break;
	default:
		break;
	}

}

void VIOC_WMIX_ALPHA_SetROPMode(VIOC_WMIX_ALPHA * pWALPHA, unsigned int mode)
{
	BITCSET(pWALPHA->uROPC.nREG[0], 0x1F, mode);
}

void VIOC_WMIX_ALPHA_SetAlphaSelection(VIOC_WMIX_ALPHA * pWALPHA, unsigned int asel)
{
	BITCSET(pWALPHA->uROPC.nREG[0], 0x3 << 14, asel << 14);
}

void VIOC_WMIX_ALPHA_SetAlphaValue(VIOC_WMIX_ALPHA * pWALPHA,
									unsigned int alpha0, unsigned int alpha1)
{
	BITCSET(pWALPHA->uROPC.nREG[1], 0xFFFFFFFF,  (alpha1 << 16) | alpha0);
}

void VIOC_WMIX_ALPHA_SetROPPattern( VIOC_WMIX_ALPHA * pWALPHA,
						unsigned int patR, unsigned int patG, unsigned int patB)
{
	BITCSET(pWALPHA->uPAT.nREG[0], 0xFFFFFFFF, (patB << 16) | patR);
	BITCSET(pWALPHA->uPAT.nREG[1], 0x0000FFFF, patG);
}

void VIOC_WMIX_SetInterruptMask(VIOC_WMIX *pWMIX, unsigned int nMask, unsigned int set)
{
	if (set == 0) {
		/* Interrupt Enable*/
		BITCSET(pWMIX->uIRQMSK.nREG, 0x1F, ~nMask);
	} else {
		/* Interrupt Diable*/
		BITCSET(pWMIX->uIRQMSK.nREG, 0x1F, ~nMask);
	}
}

unsigned int VIOC_WMIX_GetStatus(VIOC_WMIX *pWMIX )
{
	volatile unsigned int STATUS = 0;
	STATUS = pWMIX->uSTATUS.nREG;
	return STATUS;
}

void VIOC_WMIX_IreqHandler( unsigned int vectorID )
{
#if 0	
	unsigned int 	WMIXBase;
	VIOC_WMIX *pWMIX;
	unsigned int idx;

	if ( vectorID > VIOC_IREQ_MAX)
		return;

	idx = vectorID - VIOC_IREQ_WMIX0;
	
	WMIXBase = ((unsigned int)HwVIOC_WMIX0 + idx*0x100);
	pWMIX = (VIOC_WMIX *)WMIXBase;

	if (pWMIX->uSTATUS.nREG & VIOC_WMIX_IREQ_MUPD_MASK && pWMIX->uIRQMSK.bREG.UPDDONE == 0 )
	{
		/* Interrput Source Clear */
		/* Status Clear */
		pWMIX->uSTATUS.bREG.UPDDONE = 1;
		/* DO Action for Interrupt */

		/* Interrput Re-Setting */
	}

	if (pWMIX->uSTATUS.nREG & VIOC_WMIX_IREQ_MEOFWF_MASK && pWMIX->uIRQMSK.bREG.EOFWAITFALL == 0 )
	{
		/* Interrput Source Clear */
		/* Status Clear */
		pWMIX->uSTATUS.bREG.EOFWAITFALL = 1;

		/* DO Action for Interrupt */

		/* Interrput Re-Setting */
	}

	if (pWMIX->uSTATUS.nREG & VIOC_WMIX_IREQ_MEOFWR_MASK && pWMIX->uIRQMSK.bREG.EOFWAITRISE == 0 )
	{
		/* Interrput Source Clear */
		/* Status Clear */
		pWMIX->uSTATUS.bREG.EOFWAITRISE = 1;

		/* DO Action for Interrupt */

		/* Interrput Re-Setting */
	}

	if (pWMIX->uSTATUS.nREG & VIOC_WMIX_IREQ_MEOFR_MASK && pWMIX->uIRQMSK.bREG.EOFRISE == 0 )
	{
		/* Interrput Source Clear */
		/* Status Clear */
		pWMIX->uSTATUS.bREG.EOFRISE = 1;

		/* DO Action for Interrupt */

		/* Interrput Re-Setting */
	}
	
	if (pWMIX->uSTATUS.nREG & VIOC_WMIX_IREQ_MEOFF_MASK && pWMIX->uIRQMSK.bREG.EOFFALL == 0 )
	{
		/* Interrput Source Clear */
		/* Status Clear */
		pWMIX->uSTATUS.bREG.EOFFALL = 1;

		/* DO Action for Interrupt */

		/* Interrput Re-Setting */
	}
#endif//	
}


void VIOC_API_WMIX_SetOverlayAlphaROPMode( VIOC_WMIX *pWMIX, unsigned int layer, unsigned int opmode )
{
	unsigned int baseAddr;
	baseAddr = ((unsigned int)pWMIX + 0x60  + (0x20 * layer));
	VIOC_WMIX_ALPHA_SetROPMode((VIOC_WMIX_ALPHA *)baseAddr, opmode);
}

void VIOC_API_WMIX_SetOverlayAlphaValue( VIOC_WMIX *pWMIX, unsigned int layer, unsigned int alpha0, unsigned int alpha1 )
{
	unsigned int baseAddr;
	baseAddr = ((unsigned int)pWMIX + 0x60  + (0x20 * layer));
	VIOC_WMIX_ALPHA_SetAlphaValue((VIOC_WMIX_ALPHA *)baseAddr, alpha0, alpha1);
}

void VIOC_API_WMIX_SetOverlayAlphaSelection(VIOC_WMIX *pWMIX, unsigned int layer,unsigned int asel )
{
	unsigned int baseAddr;
	baseAddr = ((unsigned int)pWMIX + 0x60  + (0x20 * layer));
	VIOC_WMIX_ALPHA_SetAlphaSelection((VIOC_WMIX_ALPHA *)baseAddr, asel);
}

void VIOC_API_WMIX_SetOverlayAlphaValueControl(VIOC_WMIX *pWMIX, unsigned int layer, unsigned int region, unsigned int acon0, unsigned int acon1 )
{
	unsigned int baseAddr = 0;
	baseAddr = ((unsigned int)pWMIX + 0x60  + (0x20 * layer));
	VIOC_WMIX_ALPHA_SetAlphaValueControl((VIOC_WMIX_ALPHA *)baseAddr, region, acon0, acon1);
}

void VIOC_API_WMIX_SetOverlayAlphaColorControl(VIOC_WMIX *pWMIX, unsigned int layer, unsigned int region, unsigned int ccon0, unsigned int ccon1 )
{
	unsigned int baseAddr;
	baseAddr = ((unsigned int)pWMIX + 0x60  + (0x20 * layer));
	VIOC_WMIX_ALPHA_SetColorControl((VIOC_WMIX_ALPHA *)baseAddr, region, ccon0, ccon1);
}

void VIOC_WMIX_SetSWReset(unsigned int WMIX, unsigned int RDMA, unsigned int WDMA)
{
	volatile PVIOC_IREQ_CONFIG pIREQConfig = VIOC_IREQConfig_GetAddress();

	if(WMIX <= VIOC_WMIX6) {
		BITCSET(pIREQConfig->uSOFTRESET.nREG[1], (0x1<<(9+WMIX)), (0x1<<(9+WMIX))); // wmix reset
		BITCSET(pIREQConfig->uSOFTRESET.nREG[1], (0x1<<(9+WMIX)), (0x0<<(9+WMIX))); // wmix reset
	}

	if(RDMA <= VIOC_WMIX_RDMA_17) {
		BITCSET(pIREQConfig->uSOFTRESET.nREG[0], (0x1<<RDMA), (0x1<<RDMA)); // rdma reset
		BITCSET(pIREQConfig->uSOFTRESET.nREG[0], (0x1<<RDMA), (0x0<<RDMA)); // rdma reset
	}

	if(WDMA <= VIOC_WMIX_WDMA_08) {
		BITCSET(pIREQConfig->uSOFTRESET.nREG[1], (0x1<<WDMA), (0x1<<WDMA)); // wdma reset
		BITCSET(pIREQConfig->uSOFTRESET.nREG[1], (0x1<<WDMA), (0x0<<WDMA)); // wdma reset
	}
}

char VIOC_WMIX_GetChannelToRDMA(unsigned int RDMA)
{
	char channel;
	
	channel = (RDMA - HwVIOC_RDMA00)/HwVIOC_RDMA_GAP;
	channel = (channel >= 4) ?  (channel -4) : channel;

	return channel; 
}

void VIOC_WMIX_DUMP(VIOC_WMIX *pWMIX)
{
	unsigned int cnt = 0;
	char *pReg = (char*)pWMIX;

	printk("WMIX :: 0x%8x \n", pWMIX);
	while(cnt < 0x100)
	{
		printk("0x%8x: 0x%8x 0x%8x 0x%8x 0x%8x \n", pReg+cnt, *((unsigned long*)(pReg+cnt)), *((unsigned long*)(pReg+cnt+0x4)), *((unsigned long*)(pReg+cnt+0x8)), *((unsigned long*)(pReg+cnt+0xC)));
		cnt += 0x10;
	}
}
