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

#ifdef CONFIG_ARCH_TCC897X
#include <asm/io.h>
#include <mach/bsp.h>
#include <mach/vioc_rdma.h>
#include <mach/tccfb_ioctrl.h>
#else
#include <video/tcc/vioc_rdma.h>
#include <video/tcc/tccfb_ioctrl.h>
#endif

#define VIOC_RDMA_IREQ_SRC_MAX	7 



#define NOP __asm("NOP")


void VIOC_RDMA_SetImageUpdate(VIOC_RDMA *pRDMA)
{
	//pRDMA->uCTRL.bREG.UPD = 1;
	BITCSET(pRDMA->uCTRL.nREG, 1<<16, 1<<16 );
}

void VIOC_RDMA_SetImageEnable(VIOC_RDMA *pRDMA )
{
	/*
	pRDMA->uCTRL.bREG.IEN = 1;
	pRDMA->uCTRL.bREG.UPD = 1;
	*/
	
	BITCSET(pRDMA->uCTRL.nREG, 1<<28, 1<<28 );
	BITCSET(pRDMA->uCTRL.nREG, 1<<16, 1<<16 );

}

void VIOC_RDMA_GetImageEnable(VIOC_RDMA *pRDMA, unsigned int *enable)
{
	/*
	pRDMA->uCTRL.bREG.IEN = 1;
	pRDMA->uCTRL.bREG.UPD = 1;
	*/
	*enable = ISSET(pRDMA->uCTRL.nREG, 1<<28 );
}

void VIOC_RDMA_SetImageDisable(VIOC_RDMA *pRDMA )
{
	int i;

	/* Check RDMA is enabled */
	if(!(pRDMA->uCTRL.nREG & Hw28))
		return;

	BITCSET(pRDMA->uSTATUS.nREG, 1<< 1, 1<<1);

	BITCSET(pRDMA->uCTRL.nREG, 1<<28, 0<<28 );
	BITCSET(pRDMA->uCTRL.nREG, 1<<16, 1<<16 );

	/* RDMA Scale require to check STD_DEVEOF status bit, For synchronous updating with EOF Status  */
	/* VIOC_RDMA_SetImageDisable(pRDMA); */ /* Channel turn off when scaler changed, so now blocking. */
	
	/* Wait for EOF */
	for ( i =0; i <60; i++)
	{
		mdelay(1);
		if ( pRDMA->uSTATUS.bREG.STS_DEVEOF == 1)

		//if( pRDMA->uSTATUS.nREG & Hw19 )
		{
			break;
		}
	}
	if( i == 60 )	{
		pr_err("%s : [RDMA:0x%p] is not disabled, pRDMA->uSTATUS.nREG = 0x%08x, CTRL 0x%08x i = %d \n",
			__func__, pRDMA, (int)pRDMA->uSTATUS.nREG,  (int)pRDMA->uCTRL.nREG, i);
	}
}

// disable and no wait
void VIOC_RDMA_SetImageDisableNW(VIOC_RDMA *pRDMA )
{
	BITCSET(pRDMA->uCTRL.nREG, 1<<28, 0<<28 );
	BITCSET(pRDMA->uCTRL.nREG, 1<<16, 1<<16 );
}

void VIOC_RDMA_SetImageFormat(VIOC_RDMA *pRDMA, unsigned int nFormat)
{
	//pRDMA->uCTRL.bREG.FMT = nFormat;
	BITCSET(pRDMA->uCTRL.nREG, 0x0000001F, nFormat);
}

void VIOC_RDMA_SetImageRGBSwapMode(VIOC_RDMA *pRDMA, unsigned int rgb_mode)
{
	BITCSET(pRDMA->uCTRL.nREG, 0x7<<12, rgb_mode << 12);
}

void VIOC_RDMA_SetImageAlphaEnable(VIOC_RDMA *pRDMA, unsigned int enable)
{
	//DMA->uCTRL.bREG.AEN = enable;
	BITCSET(pRDMA->uCTRL.nREG, 1<<11, enable << 11);
}

void VIOC_RDMA_GetImageAlphaEnable(VIOC_RDMA *pRDMA, unsigned int *enable)
{
	//*enable = pRDMA->uCTRL.bREG.AEN;
	*enable = pRDMA->uCTRL.nREG & Hw11 ;
}

void VIOC_RDMA_SetImageAlphaSelect(VIOC_RDMA *pRDMA, unsigned int select)
{
	/* ALPHA0(0) or ALPHA1(1) select */
	//pRDMA->uCTRL.bREG.ASEL = select;
	BITCSET(pRDMA->uCTRL.nREG, 1<<24, select << 24);
}

void VIOC_RDMA_SetImageY2RMode(VIOC_RDMA *pRDMA, unsigned int y2r_mode)
{
	/* Y2RMD[5,10,9] */
	BITCSET(pRDMA->uCTRL.nREG, 0x3 << 9, (y2r_mode & 0x3) << 9);
	BITCSET(pRDMA->uCTRL.nREG, 0x1 << 5, (!!(y2r_mode & 0x4)) << 5);
}

void VIOC_RDMA_SetImageY2REnable(VIOC_RDMA *pRDMA, unsigned int enable)
{
	//pRDMA->uCTRL.bREG.Y2R = enable;
	BITCSET(pRDMA->uCTRL.nREG, 1<<8, enable << 8);

}

void VIOC_RDMA_SetImageR2YMode(VIOC_RDMA *pRDMA, unsigned int r2y_mode)
{
	/* R2YMD[6,19,18] */
	BITCSET(pRDMA->uCTRL.nREG, 0x3 << 18, (r2y_mode & 0x3) << 18);
	BITCSET(pRDMA->uCTRL.nREG, 0x1 << 6, (!!(r2y_mode & 0x4)) << 6);
}

void VIOC_RDMA_SetImageR2YEnable(VIOC_RDMA *pRDMA, unsigned int enable)
{
	//pRDMA->uCTRL.bREG.Y2R = enable;
	BITCSET(pRDMA->uCTRL.nREG, 1<<17, enable << 17);

}

void VIOC_RDMA_SetImageAlpha(VIOC_RDMA *pRDMA, unsigned int nAlpha0, unsigned int nAlpha1)
{
	//pRDMA->uALPHA.bREG.ALPHA0 = nAlpha0;
	//pRDMA->uALPHA.bREG.ALPHA1 = nAlpha1;

	BITCSET(pRDMA->uALPHA.nREG, 0x00FF00FF, ( nAlpha1 << 16 ) | nAlpha0 );
}

void VIOC_RDMA_GetImageAlpha(VIOC_RDMA *pRDMA, unsigned int *nAlpha0, unsigned int *nAlpha1)
{
	//*nAlpha0 = pRDMA->uALPHA.bREG.ALPHA0;
	//*nAlpha1 = pRDMA->uALPHA.bREG.ALPHA1;

	*nAlpha0 = pRDMA->uALPHA.nREG & 0x00FF0000;
	*nAlpha1 = pRDMA->uALPHA.nREG & 0x000000FF;
}

void VIOC_RDMA_SetImageUVIEnable(VIOC_RDMA *pRDMA, unsigned int enable)
{
	BITCSET(pRDMA->uCTRL.nREG, 1<<23, enable << 23);

}

void VIOC_RDMA_SetImageSize(VIOC_RDMA *pRDMA, unsigned int sw, unsigned int sh)
{
	//pRDMA->uSIZE.bREG.WIDTH = sw;
	//pRDMA->uSIZE.bREG.HEIGHT = sh;

	BITCSET(pRDMA->uSIZE.nREG, 0xFFFFFFFF, (sh << 16) | (sw) );	
}

void VIOC_RDMA_GetImageSize(VIOC_RDMA *pRDMA, unsigned int *sw, unsigned int *sh)
{
	//pRDMA->uSIZE.bREG.WIDTH = sw;
	//pRDMA->uSIZE.bREG.HEIGHT = sh;
	*sw = pRDMA->uSIZE.nREG & 0xFF;
	*sh = (pRDMA->uSIZE.nREG>>16) & 0xFF;
}

void VIOC_RDMA_SetImageBase(VIOC_RDMA *pRDMA, unsigned int nBase0, unsigned int nBase1, unsigned int nBase2)
{
	/*
	pRDMA->nBASE0  = nBase0;
	pRDMA->nBASE1  = nBase1;
	pRDMA->nBASE2  = nBase2;
	*/

	BITCSET(pRDMA->nBASE0, 0xFFFFFFFF, nBase0);
	BITCSET(pRDMA->nBASE1, 0xFFFFFFFF, nBase1);
	BITCSET(pRDMA->nBASE2, 0xFFFFFFFF, nBase2);

}

/* In case of down scalling, If there are huge size distence between RDMA and Scaler. It makes under run situation in WMIX. So RDMA down size is needed.  */
void VIOC_RDMA_SetImageSizeDownForScaler(VIOC_RDMA *pRDMA, unsigned int sw, unsigned int sh, unsigned int ratio)
{
	unsigned int offset0 = 0;
	unsigned int offset1 = 0;
	unsigned int bpp = 2; /* For RGB565 */
	
	offset0 = sw*bpp*ratio;
	offset1 = sw*bpp*ratio;

	/*
	pRDMA->uSIZE.bREG.WIDTH = sw;
	pRDMA->uSIZE.bREG.HEIGHT = sh/ratio;

	pRDMA->uOFFSET.bREG.OFFSET0 = offset0;
	pRDMA->uOFFSET.bREG.OFFSET1 = offset1;
	*/
	
	BITCSET(pRDMA->uSIZE.nREG, 0xFFFFFFFF, ( (sh/ratio) << 16) | (sw) );	
	BITCSET(pRDMA->uOFFSET.nREG, 0xFFFFFFFF, (offset1 << 16) | offset0 );
	
}

void VIOC_RDMA_SetImageSizeDownForUpScaler(VIOC_RDMA *pRDMA, unsigned int sw, unsigned int sh, unsigned int ratio)
{
	unsigned int offset0 = 0;
	unsigned int offset1 = 0;
	unsigned int bpp = 2; /* For RGB565 */
	
	offset0 = sw*bpp;
	//offset1 = sw*bpp;

	/*
	pRDMA->uSIZE.bREG.WIDTH = sw;
	pRDMA->uSIZE.bREG.HEIGHT = sh/ratio;

	pRDMA->uOFFSET.bREG.OFFSET0 = offset0;
	pRDMA->uOFFSET.bREG.OFFSET1 = offset1;
	*/
	
	BITCSET(pRDMA->uSIZE.nREG, 0xFFFFFFFF, ( (sh/ratio) << 16) | (sw) );	
	BITCSET(pRDMA->uOFFSET.nREG, 0xFFFFFFFF, (offset1 << 16) | offset0 );
}

void VIOC_RDMA_SetImageOffset(VIOC_RDMA *pRDMA, unsigned int imgFmt, unsigned int imgWidth)
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

	//pRDMA->uOFFSET.bREG.OFFSET0 = offset0;
	//pRDMA->uOFFSET.bREG.OFFSET1 = offset1;

	BITCSET(pRDMA->uOFFSET.nREG, 0xFFFFFFFF, (offset1 << 16) | offset0 ) ;

}

void VIOC_RDMA_SetImageScale(VIOC_RDMA *pRDMA, unsigned int scaleX, unsigned int scaleY)
{
#ifdef AnD_TODO
	int i;
	
	/* RDMA Scale require to check STD_DEVEOF status bit, For synchronous updating with EOF Status  */
	/* VIOC_RDMA_SetImageDisable(pRDMA); */ /* Channel turn off when scaler changed, so now blocking. */
	
	/* Wait for EOF */
	for ( i =0; i <0x20000; i++)
	{
		NOP;
		//if ( pRDMA->uSTATUS.bREG.STS_DEVEOF == 1)
		if(pRDMA->uSTATUS.nREG & Hw19)
		{
			//pRDMA->uSCALE.bREG.XSCALE  = scaleX;
			//pRDMA->uSCALE.bREG.YSCALE  = scaleY;
			BITCSET(pRDMA->uSCALE.nREG, 0x00070007, (scaleY << 16) | scaleX ); 
			
			break;
		}
	}
	
//	VIOC_RDMA_SetImageEnable(pRDMA);
#endif
}

void VIOC_RDMA_SetTestConfig(VIOC_RDMA * pRDMA)
{
}

void VIOC_RDMA_SetImageBfield(VIOC_RDMA * pRDMA, unsigned int bfield)
{
	BITCSET(pRDMA->uCTRL.nREG, 1<<29, bfield << 29);
}

void VIOC_RDMA_SetImageBFMD(VIOC_RDMA * pRDMA, unsigned int bfmd)
{
	BITCSET(pRDMA->uCTRL.nREG, 1<<30, bfmd << 30);
}


void VIOC_RDMA_SetImageIntl(VIOC_RDMA * pRDMA, unsigned int intl_en)
{
	//pRDMA->uCTRL.bREG.INTL = intl_en;
	BITCSET(pRDMA->uCTRL.nREG, 1<<31, intl_en << 31);
}


/* set 1 : IREQ Masked( interrupt disable), set 0 : IREQ UnMasked( interrput enable)  */
void VIOC_RDMA_SetIreqMask(VIOC_RDMA * pRDMA, unsigned int mask, unsigned int set)
{
	if( set == 0 ) /* Interrupt Enable*/
	{
		//pRDMA->uIRQMSK.nREG &= ~mask;
		BITCSET(pRDMA->uIRQMSK.nREG, 0x0000007F, ~mask);
	}
	else/* Interrupt Diable*/
	{
		//pRDMA->uIRQMSK.nREG |= mask;
		BITCSET(pRDMA->uIRQMSK.nREG, 0x0000007F, mask);
	}
}

/* STAT set : to clear status*/
void VIOC_RDMA_SetStatus(VIOC_RDMA * pRDMA, unsigned int mask)
{
	BITCSET(pRDMA->uSTATUS.nREG, 0x0000007F, mask);
}

struct device_node *ViocRdma_np;

VIOC_RDMA* VIOC_RDMA_GetAddress(unsigned int Num)
{
	VIOC_RDMA *pRdma = of_iomap(ViocRdma_np, Num);

	if(pRdma == NULL)
		pr_err("%s num:%d \n", __func__, Num);

	return pRdma;
}

#ifdef CONFIG_ARCH_TCC898X
void VIOC_RDMA_DEC_CTRL (VIOC_RDMA *pRDMA, unsigned int base, unsigned int length, unsigned int has_alpha, unsigned int has_comp)
{
    pRDMA->nDEC_BASE                    = base;
    pRDMA->nDEC_LENGTH                  = length;
    pRDMA->uDEC_HAS.bReg.has_alpha      = has_alpha;
    pRDMA->uDEC_HAS.bReg.has_comp       = has_comp;
}

void VIOC_RDMA_DEC_EN (VIOC_RDMA *pRDMA, unsigned int OnOff)
{
    if (OnOff == 0)
    {
        pRDMA->uDEC_HAS.bReg.has_comp   = 0x0;
    }
    pRDMA->uDEC_HAS.bReg.bypass         = ~OnOff;
    pRDMA->uDEC_CTRL.bReg.enable        = OnOff;
    pRDMA->uDEC_CTRL.bReg.cfgupd        = 1;
}
#endif

void VIOC_RDMA_DUMP(VIOC_RDMA *pRDMA)
{
	unsigned int cnt = 0;
	char *pReg = (char*)pRDMA;

	printk("RDMA :: 0x%8x \n", pRDMA);
	while(cnt < 0x100)
	{
		printk("0x%8x: 0x%8x 0x%8x 0x%8x 0x%8x \n", pReg+cnt, *((unsigned long*)(pReg+cnt)), *((unsigned long*)(pReg+cnt+0x4)), *((unsigned long*)(pReg+cnt+0x8)), *((unsigned long*)(pReg+cnt+0xC)));
		cnt += 0x10;
	}
}

static int __init vioc_rdma_init(void)
{

	ViocRdma_np = of_find_compatible_node(NULL, NULL, "telechips,vioc_rdma");
	if(ViocRdma_np == NULL)
		pr_err("cann't find vioc rdma \n");
	return 0;
}
arch_initcall(vioc_rdma_init);

