/*
 * linux/arch/arm/mach-tcc893x/vioc_wdma.c
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

#include <linux/module.h>

#include <video/tcc/tcc_types.h>
#ifdef CONFIG_ARCH_TCC897X
#include <mach/tccfb_ioctrl.h>
#include <mach/vioc_wdma.h>
#include <mach/tcc_gpu_align.h>
#else
#include <video/tcc/tccfb_ioctrl.h>
#include <video/tcc/vioc_wdma.h>
#include <video/tcc/tcc_gpu_align.h>
#endif

void VIOC_WDMA_SetWifiDisplayImageConfig(VIOC_WDMA * pWDMA, VIOC_WDMA_IMAGE_INFO_Type * ImageCfg, unsigned int r2yEn)
{
	VIOC_WDMA_SetImageBase(pWDMA, ImageCfg->BaseAddress, ImageCfg->BaseAddress1, ImageCfg->BaseAddress2);
	VIOC_WDMA_SetImageSize(pWDMA, ImageCfg->TargetWidth, ImageCfg->TargetHeight);
	VIOC_WDMA_SetImageFormat(pWDMA, ImageCfg->ImgFormat);

	VIOC_WDMA_SetImageR2YMode(pWDMA, 0); /* R2YMode Default 0 (Studio Color) */
	VIOC_WDMA_SetImageR2YEnable(pWDMA, r2yEn);

	VIOC_WDMA_SetImageOffset(pWDMA, ImageCfg->ImgFormat, ImageCfg->TargetWidth);
	VIOC_WDMA_SetImageEnhancer(pWDMA, ImageCfg->Contrast, ImageCfg->Bright, ImageCfg->Hue);
	VIOC_WDMA_SetImageInterlaced(pWDMA, ImageCfg->Interlaced);

	pWDMA->uCTRL.bREG.BR	= 0;	/* Bit-Reverse Default Off */
	pWDMA->uCTRL.bREG.SWAP	= 0;	/* RGB Swap Mode Default Off */
	pWDMA->uCTRL.bREG.SREQ	= 0;	/* Stop Request Enable Default Off */
	pWDMA->uCTRL.bREG.FU	= 0;	/* Field Update Enable Default Off */
	
	VIOC_WDMA_SetImageSyncControl(pWDMA, 0, 0);
	/* Register Update */
	VIOC_WDMA_SetImageEnable(pWDMA, ImageCfg->ContinuousMode);
}
EXPORT_SYMBOL(VIOC_WDMA_SetWifiDisplayImageConfig);

void VIOC_WDMA_SetImageEnable(VIOC_WDMA * pWDMA, unsigned int nContinuous)
{
	/*
	pWDMA->uCTRL.bREG.CONT = nContinuous;
	pWDMA->uCTRL.bREG.IEN = 1;
	pWDMA->uCTRL.bREG.UPD = 1;
	*/
	
	BITCSET(pWDMA->uCTRL.nREG, (1<<28 | 1<<23 | 1<<16), (1<<28 | nContinuous<<23 | 1<<16));
}

void VIOC_WDMA_SetImageDisable(VIOC_WDMA * pWDMA)
{
	BITCSET(pWDMA->uCTRL.nREG, (1<<28 | 1<<16), (0<<28  | 1<<16));
}

void VIOC_WDMA_SetImageUpdate(VIOC_WDMA * pWDMA)
{
	//pWDMA->uCTRL.bREG.UPD = 1;
	BITCSET(pWDMA->uCTRL.nREG, 1<<16,  1<<16);
}

void VIOC_WDMA_SetContinuousMode(VIOC_WDMA * pWDMA, unsigned int enable)
{
	BITCSET(pWDMA->uCTRL.nREG, 1<<23, enable<<23);
}

void VIOC_WDMA_SetImageFormat(VIOC_WDMA *pWDMA, unsigned int nFormat)
{
	//pWDMA->uCTRL.bREG.FMT = nFormat;
	BITCSET(pWDMA->uCTRL.nREG, 0x1F, nFormat);
}

void VIOC_WDMA_SetImageRGBSwapMode(VIOC_WDMA *pWDMA, unsigned int rgb_mode)
{
	BITCSET(pWDMA->uCTRL.nREG, 0x7<<12, rgb_mode << 12);
}

void VIOC_WDMA_SetImageInterlaced(VIOC_WDMA *pWDMA, unsigned int intl)
{
	//pWDMA->uCTRL.bREG.INTL = intl;
	BITCSET(pWDMA->uCTRL.nREG, 1<<31, intl << 31);
}

void VIOC_WDMA_SetImageR2YMode(VIOC_WDMA *pWDMA, unsigned int r2y_mode)
{
	//pWDMA->uCTRL.bREG.R2YMD= r2y_mode;
	BITCSET(pWDMA->uCTRL.nREG, 0x7 << 9, r2y_mode << 9);
}

void VIOC_WDMA_SetImageR2YEnable(VIOC_WDMA *pWDMA, unsigned int enable)
{
	//pWDMA->uCTRL.bREG.R2Y = enable;
	BITCSET(pWDMA->uCTRL.nREG, 1<<8, enable << 8);
}

void VIOC_WDMA_SetImageY2RMode(VIOC_WDMA *pWDMA, unsigned int y2r_mode)
{
	BITCSET(pWDMA->uCTRL.nREG, 0x7 << 18, y2r_mode << 18);
}

void VIOC_WDMA_SetImageY2REnable(VIOC_WDMA *pWDMA, unsigned int enable)
{
	BITCSET(pWDMA->uCTRL.nREG, 1<<17, enable << 17);
}


void VIOC_WDMA_SetImageSyncControl(VIOC_WDMA *pWDMA, unsigned int enable, unsigned int RDMA)
{
	//if (((volatile VIOC_WDMA *)pWDMA)->uIRQSTS.bREG.STS_EOF == 1)
	if(pWDMA->uIRQSTS.nREG & Hw31)
	{
		//pWDMA->uIRQSTS.bREG.STS_EOF = 0;
		BITCSET(pWDMA->uIRQSTS.nREG, 1<<31, 0<<31);
	}
	
	if ( enable == true)
	{
		//pWDMA->uRATE.bREG.SEN = 1;
		////pWDMA->uRATE.bREG.SYNCMD_ADDR = 0;/*Use Recommended Set*/
		////pWDMA->uRATE.bREG.SYNCMD_SENS = 0;/*Use Recommended Set*/
		//pWDMA->uRATE.bREG.SYNCSEL = RDMA;/*RDMA00 ~~ RDMA06*/
		
		BITCSET(pWDMA->uRATE.nREG, 0x800000FF, 1<<31 | RDMA );				
	}
	else
	{
		//pWDMA->uRATE.bREG.SEN = 0;
		BITCSET(pWDMA->uRATE.nREG, 1<<8,  0<<8);
	}
}

void VIOC_WDMA_SetImageDither(VIOC_WDMA *pWDMA, unsigned int nDithEn, unsigned int nDithSel, unsigned char mat[4][4])
{
	#if 0
	pWDMA->uDMAT.bREG.DITH00 = mat[0][0];
	pWDMA->uDMAT.bREG.DITH01 = mat[0][1];
	pWDMA->uDMAT.bREG.DITH02 = mat[0][2];
	pWDMA->uDMAT.bREG.DITH03 = mat[0][3];

	pWDMA->uDMAT.bREG.DITH10 = mat[1][0];
	pWDMA->uDMAT.bREG.DITH11 = mat[1][1];
	pWDMA->uDMAT.bREG.DITH12 = mat[1][2];
	pWDMA->uDMAT.bREG.DITH13 = mat[1][3];

	pWDMA->uDMAT.bREG.DITH20 = mat[2][0];
	pWDMA->uDMAT.bREG.DITH21 = mat[2][1];
	pWDMA->uDMAT.bREG.DITH22 = mat[2][2];
	pWDMA->uDMAT.bREG.DITH23 = mat[2][3];

	pWDMA->uDMAT.bREG.DITH30 = mat[3][0];
	pWDMA->uDMAT.bREG.DITH31 = mat[3][1];
	pWDMA->uDMAT.bREG.DITH32 = mat[3][2];
	pWDMA->uDMAT.bREG.DITH33 = mat[3][3];

	pWDMA->uCTRL.bREG.DITHEN = nDithEn; /* Dither Enable*/
	pWDMA->uCTRL.bREG.DITHSEL = nDithSel; /*Dither Mode 0: LSB Toggle mode, 1: Adder Mode */
	#endif
	
	BITCSET(pWDMA->uDMAT.nREG[0], 0x00007777, mat[0][3] << 12 | mat[0][2] << 8 | mat[0][1] << 4 | mat[0][0] );
	BITCSET(pWDMA->uDMAT.nREG[0], 0x77770000, mat[1][3] << 28 | mat[1][2] << 24 | mat[1][1] << 18 | mat[1][0] << 16 );

	BITCSET(pWDMA->uDMAT.nREG[1], 0x00007777, mat[2][3] << 12 | mat[2][2] << 8 | mat[2][1] << 4 | mat[2][0] );
	BITCSET(pWDMA->uDMAT.nREG[1], 0x77770000, mat[3][3] << 28 | mat[3][2] << 24 | mat[3][1] << 18 | mat[3][0] << 16 );

	BITCSET(pWDMA->uCTRL.nREG, 1<<27 | 1<<24, nDithEn << 24 | nDithSel << 27 );
	

	#if 0
	pWDMA->uCTRL.bREG.DITHEN = nDithEn;
	pWDMA->uCTRL.bREG.DITHSEL = nDithSel;
	pWDMA->uDMAT.bREG.MAT0 = nDithMatrix[0];
	pWDMA->uDMAT.bREG.MAT1 = nDithMatrix[1];
	pWDMA->uDMAT.bREG.MAT2 = nDithMatrix[2];
	pWDMA->uDMAT.bREG.MAT3 = nDithMatrix[3];
	#endif
}

void VIOC_WDMA_SetImageEnhancer(VIOC_WDMA *pWDMA, unsigned int nContrast, unsigned int nBright, unsigned int nHue)
{
	/*
	pWDMA->uENH.bREG.CONTRAST   = nContrast;
	pWDMA->uENH.bREG.BRIGHT     = nBright;
	pWDMA->uENH.bREG.HUE        = nHue;
	*/
	
	BITCSET(pWDMA->uENH.nREG, 0x00FFFFFF, nHue << 24 | nBright<< 16 || nContrast );
}

void VIOC_WDMA_SetImageSize(VIOC_WDMA *pWDMA, unsigned int sw, unsigned int sh)
{
	//pWDMA->uSIZE.bREG.WIDTH   = sw;
	//pWDMA->uSIZE.bREG.HEIGHT  = sh;

	BITCSET(pWDMA->uSIZE.nREG, 0xFFFFFFFF, (sh << 16) | (sw) );		
}

void VIOC_WDMA_SetImageBase(VIOC_WDMA *pWDMA, unsigned int nBase0, unsigned int nBase1, unsigned int nBase2)
{
	/*
	pWDMA->uBASE0  = nBase0;
	pWDMA->uBASE1  = nBase1;
	pWDMA->uBASE2  = nBase2;
	*/

	BITCSET(pWDMA->uBASE0, 0xFFFFFFFF, nBase0);
	BITCSET(pWDMA->uBASE1, 0xFFFFFFFF, nBase1);
	BITCSET(pWDMA->uBASE2, 0xFFFFFFFF, nBase2);
}

void VIOC_WDMA_SetImageOffset(VIOC_WDMA *pWDMA, unsigned int imgFmt, unsigned int imgWidth)
{
	unsigned int offset0 = 0;
	unsigned int offset1 = 0;

	switch (imgFmt)
	{
		case TCC_LCDC_IMG_FMT_1BPP:	// 1bpp indexed color
			offset0 = (1*imgWidth)/8;
			break;
		case TCC_LCDC_IMG_FMT_2BPP:	// 2bpp indexed color
			offset0 = (1*imgWidth)/4;
			break;
		case TCC_LCDC_IMG_FMT_4BPP:	// 4bpp indexed color
			offset0 = (1*imgWidth)/2;
			break;
		case TCC_LCDC_IMG_FMT_8BPP:	// 8bpp indexed color
			offset0 = (1*imgWidth);
			break;
			//...
		case TCC_LCDC_IMG_FMT_RGB332:	// RGB332 - 1bytes aligned - R[7:5],G[4:2],B[1:0]
			offset0 = 1*imgWidth;
			break;
		case TCC_LCDC_IMG_FMT_RGB444:	// RGB444 - 2bytes aligned - A[15:12],R[11:8],G[7:3],B[3:0]
		case TCC_LCDC_IMG_FMT_RGB565:	// RGB565 - 2bytes aligned - R[15:11],G[10:5],B[4:0]
		case TCC_LCDC_IMG_FMT_RGB555:	// RGB555 - 2bytes aligned - A[15],R[14:10],G[9:5],B[4:0]
			offset0 = 2*imgWidth;
			break;
		//case TCC_LCDC_IMG_FMT_RGB888:
		case TCC_LCDC_IMG_FMT_RGB888:	// RGB888 - 4bytes aligned - A[31:24],R[23:16],G[15:8],B[7:0]
		case TCC_LCDC_IMG_FMT_RGB666:	// RGB666 - 4bytes aligned - A[23:18],R[17:12],G[11:6],B[5:0]
			offset0 = 4*imgWidth;
			break;
		case TCC_LCDC_IMG_FMT_RGB888_3: //RGB888 - 3 bytes aligned : B1[31:24],R0[23:16],G0[15:8],B0[7:0]
		case TCC_LCDC_IMG_FMT_ARGB6666_3: //ARGB6666 - 3 bytes aligned : A[23:18],R[17:12],G[11:6],B[5:0]
			offset0 = 3*imgWidth;
			break;

		case TCC_LCDC_IMG_FMT_444SEP:	/* YUV444 or RGB444 Format */
			offset0 = imgWidth;
			offset1 = imgWidth;
			break;
	
			//...
		case TCC_LCDC_IMG_FMT_YUV420SP:	// YCbCr 4:2:0 Separated format - Not Supported for Image 1 and 2
			//!!!!
			offset0 = imgWidth;
			offset1 = imgWidth/2;
			break;
		case TCC_LCDC_IMG_FMT_YUV422SP:		// YCbCr 4:2:2 Separated format - Not Supported for Image 1 and 2
			//!!!!
			offset0 = imgWidth;
			offset1 = imgWidth/2;
			break;
		case TCC_LCDC_IMG_FMT_UYVY:	// YCbCr 4:2:2 Sequential format
		case TCC_LCDC_IMG_FMT_VYUY:	// YCbCr 4:2:2 Sequential format
		case TCC_LCDC_IMG_FMT_YUYV:	// YCbCr 4:2:2 Sequential format
		case TCC_LCDC_IMG_FMT_YVYU:	// YCbCr 4:2:2 Sequential format
			offset0 = 2*imgWidth;
			break;
			//...
		case TCC_LCDC_IMG_FMT_YUV420ITL0:	// YCbCr 4:2:0 interleved type 0 format - Not Supported for Image 1 and 2
		case TCC_LCDC_IMG_FMT_YUV420ITL1:	// YCbCr 4:2:0 interleved type 1 format - Not Supported for Image 1 and 2
			//!!!!
			offset0 = imgWidth;
			offset1 = imgWidth;
			break;
		case TCC_LCDC_IMG_FMT_YUV422ITL0:	// YCbCr 4:2:2 interleved type 0 format - Not Supported for Image 1 and 2
		case TCC_LCDC_IMG_FMT_YUV422ITL1:	// YCbCr 4:2:2 interleved type 1 format - Not Supported for Image 1 and 2
			//!!!!
			offset0 = imgWidth;
			offset1 = imgWidth;
			break;
		default:
			offset0 = imgWidth;
			offset1 = imgWidth;
			break;
	}

	//pWDMA->uOFFS.bREG.OFFSET0 = offset0;
	//pWDMA->uOFFS.bREG.OFFSET1 = offset1;

	BITCSET(pWDMA->uOFFS.nREG, 0xFFFFFFFF, (offset1 << 16) | offset0 ) ;
}

void VIOC_WDMA_SetImageOffset_withYV12(VIOC_WDMA *pWDMA, unsigned int imgWidth)
{
	unsigned int stride, stride_c;

	stride = ALIGNED_BUFF(imgWidth, L_STRIDE_ALIGN);
	stride_c = ALIGNED_BUFF((stride/2), C_STRIDE_ALIGN);

	BITCSET(pWDMA->uOFFS.nREG, 0xFFFFFFFF, (stride_c << 16) | stride);
}

void VIOC_WDMA_SetIreqMask(VIOC_WDMA * pWDMA, unsigned int mask, unsigned int set)
{	/* set 1 : IREQ Masked(interrupt disable), set 0 : IREQ UnMasked(interrput enable) */
	if(set == 0)	/* Interrupt Enable*/
	{
		//pWDMA->uIRQMSK.nREG &= ~mask;
		BITCSET(pWDMA->uIRQMSK.nREG, 0x000001FF, ~mask);
	}
	else/* Interrupt Diable*/
	{
		//pWDMA->uIRQMSK.nREG |=  mask;
		BITCSET(pWDMA->uIRQMSK.nREG, 0x000001FF, mask);
	}
}
EXPORT_SYMBOL(VIOC_WDMA_SetIreqMask);

void VIOC_WDMA_SetIreqStatus(VIOC_WDMA * pWDMA, unsigned int mask, unsigned int set)
{	
	if(set == 0)	
	{
		//pWDMA->uIRQMSK.nREG &= ~mask;
		BITCSET(pWDMA->uIRQSTS.nREG, mask, ~mask);
	}
	else
	{
		//pWDMA->uIRQMSK.nREG |=  mask;
		BITCSET(pWDMA->uIRQSTS.nREG, mask, mask);
	}
}

void VIOC_WDMA_ClearEORF(VIOC_WDMA * pWDMA)
{
	BITCSET(pWDMA->uIRQSTS.nREG, 1<<5, 1<<5); 
}

void VIOC_WDMA_ClearEOFF(VIOC_WDMA * pWDMA)
{
	BITCSET(pWDMA->uIRQSTS.nREG, 1<<6, 1<<6);
}

void VIOC_WDMA_IreqHandler(unsigned int vectorID)
{
#if 0
	VIOC_WDMA *pWDMA;

	switch(vectorID)
	{
		case VIOC_IREQ_WDMA00:
		{
			pWDMA = (VIOC_WDMA *)HwVIOC_WDMA00;
			break;
		}
		case VIOC_IREQ_WDMA01:
		{
			pWDMA = (VIOC_WDMA *)HwVIOC_WDMA01;		
			break;
		}
		case VIOC_IREQ_WDMA02:
		{
			pWDMA = (VIOC_WDMA *)HwVIOC_WDMA02;		
			break;
		}
		case VIOC_IREQ_WDMA03:
		{
			pWDMA = (VIOC_WDMA *)HwVIOC_WDMA03;		
			break;
		}
		case VIOC_IREQ_WDMA04:
		{
			pWDMA = (VIOC_WDMA *)HwVIOC_WDMA04;		
			break;
		}
		case VIOC_IREQ_WDMA05:
		{
			pWDMA = (VIOC_WDMA *)HwVIOC_WDMA05;		
			break;
		}		
		case VIOC_IREQ_WDMA06:
		{
			pWDMA = (VIOC_WDMA *)HwVIOC_WDMA06;		
			break;
		}				
		default:
		{
			return;
		}	
	}

	if(pWDMA->uIRQSTS.nREG & VIOC_WDMA_IREQ_UPD_MASK)
	{
		/* Interrput Source Clear */
		/* Status Clear */
		pWDMA->uIRQSTS.bREG.UPDDONE = 1;
		/* DO Action for Interrupt */

		/* Interrput Re-Setting */
	}
	
	if(pWDMA->uIRQSTS.nREG & VIOC_WDMA_IREQ_SREQ_MASK)
	{
		/* Interrput Source Clear */
		/* Status Clear */
		pWDMA->uIRQSTS.bREG.STOPREQ = 1;
		/* DO Action for Interrupt */

		/* Interrput Re-Setting */
	}

	if(pWDMA->uIRQSTS.nREG & VIOC_WDMA_IREQ_ROLL_MASK)
	{
		/* Interrput Source Clear */
		/* Status Clear */
		pWDMA->uIRQSTS.bREG.ROLL = 1;
		/* DO Action for Interrupt */

		/* Interrput Re-Setting */
	}
	
	if(pWDMA->uIRQSTS.nREG & VIOC_WDMA_IREQ_ENR_MASK)
	{
		/* Interrput Source Clear */
		/* Status Clear */
		pWDMA->uIRQSTS.bREG.ENRISE = 1;
		/* DO Action for Interrupt */

		/* Interrput Re-Setting */
	}
	
	if(pWDMA->uIRQSTS.nREG & VIOC_WDMA_IREQ_ENF_MASK)
	{
		/* Interrput Source Clear */
		/* Status Clear */
		pWDMA->uIRQSTS.bREG.ENFALL = 1;
		/* DO Action for Interrupt */

		/* Interrput Re-Setting */
	}
	
	if(pWDMA->uIRQSTS.nREG & VIOC_WDMA_IREQ_EOFR_MASK)
	{
		/* Interrput Source Clear */
		/* Status Clear */
		pWDMA->uIRQSTS.bREG.EOFRISE = 1;
		/* DO Action for Interrupt */

		/* Interrput Re-Setting */
	}
	
	if(pWDMA->uIRQSTS.nREG & VIOC_WDMA_IREQ_EOFF_MASK)
	{
		/* Interrput Source Clear */
		/* Status Clear */
		pWDMA->uIRQSTS.bREG.EOFFALL = 1;
		/* DO Action for Interrupt */

		/* Interrput Re-Setting */
	}

	if(pWDMA->uIRQSTS.nREG & VIOC_WDMA_IREQ_SEOFR_MASK)
	{
		/* Interrput Source Clear */
		/* Status Clear */
		pWDMA->uIRQSTS.bREG.SEOFRISE = 1;
		/* DO Action for Interrupt */

		/* Interrput Re-Setting */
	}

	if(pWDMA->uIRQSTS.nREG & VIOC_WDMA_IREQ_SEOFF_MASK)
	{
		/* Interrput Source Clear */
		/* Status Clear */
		pWDMA->uIRQSTS.bREG.SEOFFALL = 1;
		/* DO Action for Interrupt */

		/* Interrput Re-Setting */
	}
#endif//
}

void VIOC_WDMA_SWReset(PVIOC_IREQ_CONFIG pIrgConfig, unsigned int WDMA)
{
	BITCSET(pIrgConfig->uSOFTRESET.nREG[1], (0x1<<WDMA), (0x1<<WDMA)); // wdma reset
	BITCSET(pIrgConfig->uSOFTRESET.nREG[1], (0x1<<WDMA), (0x00<<WDMA)); // wdma reset
}

void VIOC_WDMA_GetStatus(VIOC_WDMA *pWDMA, unsigned int *status)
{
	*status = pWDMA->uIRQSTS.nREG;
}

unsigned int VIOC_WDMA_IsImageEnable(VIOC_WDMA * pWDMA)
{
        unsigned int wdma_enable;
        wdma_enable = (pWDMA->uCTRL.nREG  >> 28) & 1;
        return wdma_enable;
}

void VIOC_WDMA_DUMP(VIOC_WDMA *pWDMA)
{
	unsigned int cnt = 0;
	char *pReg = (char*)pWDMA;

	printk("WDMA :: 0x%8x \n", pWDMA);
	while(cnt < 0x100)
	{
		printk("0x%8x: 0x%8x 0x%8x 0x%8x 0x%8x \n", pReg+cnt, *((unsigned long*)(pReg+cnt)), *((unsigned long*)(pReg+cnt+0x4)), *((unsigned long*)(pReg+cnt+0x8)), *((unsigned long*)(pReg+cnt+0xC)));
		cnt += 0x10;
	}
}

/* EOF */
