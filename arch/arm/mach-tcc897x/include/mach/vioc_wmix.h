/*
 * linux/arch/arm/mach-tcc893x/include/mach/vioc_wmix.h
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

#ifndef __VIOC_WMIX_H__
#define __VIOC_WMIX_H__
#include <mach/reg_physical.h>

typedef	struct
{
	unsigned		OVP		:  5;
	unsigned		RESERVED0 	:  1;
	unsigned		STR		:  2;
	unsigned		EN0_3D	:  1;
	unsigned		EN1_3D	:  1;
	unsigned		EN2_3D	:  1;
	unsigned		EN3_3D	:  1;
	unsigned		RESERVED	:  4;	
	unsigned		UPD 	:  1;		//16
	unsigned		MD0_3D	:  3;
	unsigned		MD1_3D	:  3;
	unsigned		MD2_3D	:  3;
	unsigned		MD3_3D	:  3;
	unsigned		RESERVED1	: 3;	
} VIOC_WMIX_CTRL;

typedef	union
{
	unsigned long		nREG;
	VIOC_WMIX_CTRL		bREG;
} VIOC_WMIX_CTRL_u;

typedef	struct
{
	unsigned	BG0		: 8;
	unsigned	BG1		: 8;
	unsigned	BG2		: 8;
	unsigned	BG3		: 8;
} VIOC_WMIX_BG;

typedef	union
{
	unsigned long		nREG;
	VIOC_WMIX_BG		bREG;
} VIOC_WMIX_BG_u;

typedef	struct
{
	unsigned	WIDTH    	: 13;
	unsigned 	RESERVED0	:  3;
	unsigned	HEIGHT     	: 13;
	unsigned 	RESERVED1	:  3;
} VIOC_WMIX_SIZE;

typedef	union
{
	unsigned long		nREG;
	VIOC_WMIX_SIZE		bREG;
} VIOC_WMIX_SIZE_u;

typedef	struct
{
	unsigned	XPOS    	: 13;
	unsigned 	RESERVED0:  3;
	unsigned	YPOS     	: 13;
	unsigned 	RESERVED1:  3;
} VIOC_WMIX_POS;

typedef	union
{
	unsigned	long			nREG;
	VIOC_WMIX_POS		bREG;
} VIOC_WMIX_POS_u;

typedef	struct
{
	unsigned	KEYB    	:  8;
	unsigned	KEYG		:  8;
	unsigned	KEYR     	:  8;
	unsigned RESERVED0	:  7;
	unsigned	KEYEN    	:  1;
	unsigned	KEYMB    	:  8;
	unsigned	KEYMG		:  8;
	unsigned	KEYMR     	:  8;
	unsigned 	RESERVED1	:  8;
} VIOC_WMIX_KEY;

typedef	union
{
	unsigned long		nREG[2];
	VIOC_WMIX_KEY		bREG;
} VIOC_WMIX_KEY_u;

typedef	struct
{
	unsigned	UPDDONE		:  1;	// Register Update Done
	unsigned	EOFWAITFALL	:  1;	// EOF Rising
	unsigned	EOFWAITRISE	:  1;	// EOF Falling
	unsigned	EOFRISE		:  1;	// Error
	unsigned	EOFFALL		:  1;	// Error
	unsigned	RESERVE0			: 11;
	unsigned	RESERVE1			:  8;
	unsigned	UP_EOF    	:  1;	// Source
	unsigned	UP_ENABLE 	:  1;	// Source
	unsigned	UP_UPDATE 	:  1;	// Source
	unsigned	UP_INTL   	:  1;	// Source
	unsigned	DN_EOFWAIT	:  1;	// Source
	unsigned	DN_ENABLE	:  1;	// Source
	unsigned	DN_BFIELD	:  1;	// Source
	unsigned	DN_INTL		:  1;	// Source
} VIOC_WMIX_STATUS;

typedef	union
{
	unsigned long		nREG;
	VIOC_WMIX_STATUS	bREG;
} VIOC_WMIX_STATUS_u;

typedef	struct
{
	unsigned	UPDDONE		:  1;	// Register Update Done
	unsigned	EOFWAITFALL	:  1;	// EOF Rising
	unsigned	EOFWAITRISE	:  1;	// EOF Falling
	unsigned	EOFRISE		:  1;	// Error
	unsigned	EOFFALL		:  1;	// Error
	unsigned	RESERVE0	: 11;
	unsigned	RESERVE1	: 16;
} VIOC_WMIX_IRQMSK;

typedef	union
{
	unsigned long		nREG;
	VIOC_WMIX_IRQMSK	bREG;
} VIOC_WMIX_IRQMSK_u;

/*WMIX ALPHA CONTROL*/
typedef	struct
{
	unsigned	ACON0_00		:  3;
	unsigned	RESERVE0   		:  1;
	unsigned	ACON1_00		:  3;
	unsigned	RESERVE1   		:  1;
	unsigned	ACON0_01		:  3;
	unsigned	RESERVE2   		:  1;
	unsigned	ACON1_01		:  3;
	unsigned	RESERVE3   		:  1;
	unsigned	ACON0_10		:  3;
	unsigned	RESERVE4   		:  1;
	unsigned	ACON1_10		:  3;
	unsigned	RESERVE5   		:  1;
	unsigned	ACON0_11		:  3;
	unsigned	RESERVE6   		:  1;
	unsigned	ACON1_11		:  3;
	unsigned	RESERVE7   		:  1;
} VIOC_WMIX_MACON;

typedef	union
{
	unsigned long		nREG;
	VIOC_WMIX_MACON		bREG;
} VIOC_WMIX_MACON_u;

typedef	struct
{
	unsigned	CCON0_00		:  4;
	unsigned	CCON1_00		:  4;
	unsigned	CCON0_01		:  4;
	unsigned	CCON1_01		:  4;
	unsigned	CCON0_10		:  4;
	unsigned	CCON1_10		:  4;
	unsigned	CCON0_11		:  4;
	unsigned	CCON1_11		:  4;
} VIOC_WMIX_MCCON;

typedef	union
{
	unsigned long		nREG;
	VIOC_WMIX_MCCON		bREG;
} VIOC_WMIX_MCCON_u;

typedef	struct
{
	unsigned	ROPMODE		:  5;
	unsigned	RESERVE1		:  9;
	unsigned	ASEL			:  2;
	unsigned	ALPHA0			:  8;
	unsigned	ALPHA1			:  8;
} VIOC_WMIX_MROPC;

typedef	union
{
	unsigned long		nREG;
	VIOC_WMIX_MROPC		bREG;
} VIOC_WMIX_MROPC_u;

typedef	struct
{
	unsigned	BLUE		:  8;
	unsigned	GREEN		:  8;
	unsigned	RED			:  8;
	unsigned	RESERVE0	:  8;
} VIOC_WMIX_MPAT;

typedef	union
{
	unsigned long		nREG;
	VIOC_WMIX_MPAT		bREG;
} VIOC_WMIX_MPAT_u;

typedef	struct _VIOC_WMIX
{
	volatile VIOC_WMIX_CTRL_u 		uCTRL; 			// 0x00  R/W  0x00000005 WMIX Control Register
	volatile VIOC_WMIX_BG_u 		uBG; 			// 0x04  R/W  0x00000000 WMIX Background Color Register
	volatile VIOC_WMIX_SIZE_u 		uSIZE; 			// 0x08  R/W  0x00000000 WMIx Size Register
	volatile unsigned  int 			reserved0; 		// 0x0C
	volatile VIOC_WMIX_POS_u 		uPOS0; 			// 0x10  R/W  0x00000000 WMIX Position 0 Register
	volatile VIOC_WMIX_POS_u 		uPOS1; 			// 0x14  R/W  0x00000000 WMIX Position 1 Register
	volatile VIOC_WMIX_POS_u 		uPOS2; 			// 0x18  R/W  0x00000000 WMIX Position 2 Register
	volatile VIOC_WMIX_POS_u 		uPOS3; 			// 0x1C  R/W  0x00000000 WMIX Position 3 Register
	volatile VIOC_WMIX_KEY_u 		uKEY0; 			// 0x20  R/W  0x00000000 WMIX Chroma-Key 0,1 Image 0 Register
	volatile VIOC_WMIX_KEY_u 		uKEY1; 			// 0x28  R/W  0x00000000 WMIX Chroma-key 0,1 Image 1 Register
	volatile VIOC_WMIX_KEY_u 		uKEY2; 			// 0x30  R/W  0x00000000 WMIX Chroma-key 0,1 Image 2 Register
	volatile VIOC_WMIX_STATUS_u 	uSTATUS; 		// 0x38  R/W  0x00000000 WMIX Status Register
	volatile VIOC_WMIX_IRQMSK_u 	uIRQMSK; 		// 0x3C  R/W  0x0000001F WMIX Interrupt Masked Register

	volatile VIOC_WMIX_MACON_u 		uACON0; 		// 0x40  R/W  0x10622662 WMIX Alpha Control Register for 0'th Layer
	volatile VIOC_WMIX_MCCON_u 		uCCON0; 		// 0x44  R/W  0x20C44CC4 WMIX Color Control Register for 0'th Layer
	volatile VIOC_WMIX_MROPC_u 		uROPC0; 		// 0x48  R/W  0x0000C018 WMIX ROP Control Register for 0'th Layer
	volatile VIOC_WMIX_MPAT_u 		uPAT0; 			// 0x4C  R/W  0x00000000 WMIX Patterm Register for 0'th Layer
	
	volatile VIOC_WMIX_MACON_u 		uACON1; 		// 0x50  R/W  0x10622662 WMIX Alpha Control Register for 1'th Layer
	volatile VIOC_WMIX_MCCON_u 		uCCON1; 		// 0x54  R/W  0x20C44CC4 WMIX Color Control Register for 1'th Layer
	volatile VIOC_WMIX_MROPC_u 		uROPC1; 		// 0x58  R/W  0x0000C018 WMIX ROP Control Register for 1'th Layer
	volatile VIOC_WMIX_MPAT_u 		uPAT1; 			// 0x5C  R/W  0x00000000 WMIX Patterm Register for 1'th Layer
	
	volatile VIOC_WMIX_MACON_u 		uACON2; 		// 0x60  R/W  0x10622662 WMIX Alpha Control Register for 2'th Layer
	volatile VIOC_WMIX_MCCON_u 		uCCON2; 		// 0x64  R/W  0x20C44CC4 WMIX Color Control Register for 2'th Layer
	volatile VIOC_WMIX_MROPC_u 		uROPC2; 		// 0x68  R/W  0x0000C018 WMIX ROP Control Register for 2'th Layer
	volatile VIOC_WMIX_MPAT_u 		uPAT2; 			// 0x6C  R/W  0x00000000 WMIX Patterm Register for 2'th Layer
} VIOC_WMIX,*PVIOC_WMIX;

typedef	struct
{
	volatile VIOC_WMIX_MACON_u	uACON;				// 0
	volatile VIOC_WMIX_MCCON_u	uCCON;				// 1
	volatile VIOC_WMIX_MROPC_u	uROPC;				// 2
	volatile VIOC_WMIX_MPAT_u 	uPAT;				// 3
} VIOC_WMIX_ALPHA;

/* Interface APIs */
extern void VIOC_WMIX_SetOverlayPriority(VIOC_WMIX *pWMIX, unsigned int nOverlayPriority);
extern void VIOC_WMIX_GetOverlayPriority(VIOC_WMIX *pWMIX, unsigned int *nOverlayPriority);
extern void VIOC_WMIX_SetUpdate(VIOC_WMIX *pWMIX);
#if 0 //[[ hskim_20110920_BEGIN -- Not Support
extern void VIOC_WMIX_SetAlphOpt(VIOC_WMIX *pWMIX, unsigned int opt);
#endif //]] hskim_20110920_END -- Not Support
extern void VIOC_WMIX_SetSize(VIOC_WMIX *pWMIX, unsigned int nWidth, unsigned int nHeight);
extern void VIOC_WMIX_GetSize(VIOC_WMIX *pWMIX, unsigned int *nWidth, unsigned int *nHeight);
extern void VIOC_WMIX_SetBGColor(VIOC_WMIX *pWMIX, unsigned int nBG0, unsigned int nBG1, unsigned int nBG2, unsigned int nBG3);
extern void VIOC_WMIX_SetPosition(VIOC_WMIX *pWMIX, unsigned int nChannel, unsigned int nX, unsigned int nY);
extern void VIOC_WMIX_SetChromaKey(VIOC_WMIX *pWMIX, unsigned int nLayer, unsigned int nKeyEn, unsigned int nKeyR, unsigned int nKeyG, unsigned int nKeyB, unsigned int nKeyMaskR, unsigned int nKeyMaskG, unsigned int nKeyMaskB);
extern void VIOC_WMIX_GetChromaKey(VIOC_WMIX *pWMIX, unsigned int nLayer, unsigned int *nKeyEn, unsigned int *nKeyR, unsigned int *nKeyG, unsigned int *nKeyB, unsigned int *nKeyMaskR, unsigned int *nKeyMaskG, unsigned int *nKeyMaskB);
extern void VIOC_WMIX_ALPHA_SetAlphaValueControl( VIOC_WMIX_ALPHA * pWALPHA, unsigned int region, unsigned int acon0, unsigned int acon1);
extern void VIOC_WMIX_ALPHA_SetColorControl( VIOC_WMIX_ALPHA * pWALPHA, unsigned int region, unsigned int ccon0, unsigned int ccon1);
extern void VIOC_WMIX_ALPHA_SetROPMode( VIOC_WMIX_ALPHA * pWALPHA, unsigned int mode);
extern void VIOC_WMIX_ALPHA_SetAlphaSelection( VIOC_WMIX_ALPHA * pWALPHA, unsigned int asel);
extern void VIOC_WMIX_ALPHA_SetAlphaValue( VIOC_WMIX_ALPHA * pWALPHA, unsigned int alpha0, unsigned int alpha1);
extern void VIOC_WMIX_ALPHA_SetROPPattern( VIOC_WMIX_ALPHA * pWALPHA, unsigned int patR, unsigned int patG, unsigned int patB);
extern void VIOC_WMIX_SetInterruptMask(VIOC_WMIX *pWMIX, unsigned int nMask, unsigned int set);
extern unsigned int VIOC_WMIX_GetStatus(VIOC_WMIX *pWMIX );
extern void VIOC_WMIX_IreqHandler( unsigned int vectorID );

extern char VIOC_WMIX_GetChannelToRDMA(unsigned int RDMA);


// VIOC API FUNCTION  //

/* 4X2 */
typedef enum
{
	VIOC_WMIX0_ALPHA_L0 =0,
	VIOC_WMIX0_ALPHA_L1,
	VIOC_WMIX0_ALPHA_L2,
	VIOC_WMIX0_ALPHA_LAYER_MAX	
}VIOC_WMIX0_ALPHA_LAYER_Type;

/* 2X2 #1*/
typedef enum
{
	VIOC_WMIX1_ALPHA_L0 =0,
	VIOC_WMIX1_ALPHA_LAYER_MAX	
}VIOC_WMIX1_ALPHA_LAYER_Type;

/* 2X2 #2*/
typedef enum
{
	VIOC_WMIX2_ALPHA_L0 =0,
	VIOC_WMIX2_ALPHA_LAYER_MAX	
}VIOC_WMIX2_ALPHA_LAYER_Type;

typedef enum
{
	VIOC_WMIX_ALPHA_SEL0 = 0, 	/* image alpha 0 ~ 255 ( 0% ~ 99.6% )*/
	VIOC_WMIX_ALPHA_SEL1, 		/*not defined*/
	VIOC_WMIX_ALPHA_SEL2, 		/*image alpha 1 ~ 256 ( 0.39% ~ 100% )*/
	VIOC_WMIX_ALPHA_SEL3, 		/*image alpha 0 ~ 127, 129 ~ 256 ( 0% ~ 49.6%, 50.3% ~ 100% )*/
	VIOC_WMIX_ALPHA_SEL_MAX
}VIOC_WMIX_ALPHA_SELECT_Type;

typedef enum
{
	VIOC_WMIX_ALPHA_ROP_NOTDEFINE = 0, 	/*not defined*/
	VIOC_WMIX_ALPHA_ROP_GLOBAL = 2, 	/*Global Alpha*/
	VIOC_WMIX_ALPHA_ROP_PIXEL, 	/*Pixel Alpha*/
	VIOC_WMIX_ALPHA_ROP_MAX
}VIOC_WMIX_ALPHA_ROPMODE_Type;

typedef enum
{
	VIOC_WMIX_ALPHA_ACON0_0 = 0, 	/* Result_A = ALPHA0 * SEL0_Out, SEL0_Out = ALPHA1*/
	VIOC_WMIX_ALPHA_ACON0_1, 		/* Result_A = ALPHA0 * SEL0_Out, SEL0_Out = 256 - ALPHA1*/
	VIOC_WMIX_ALPHA_ACON0_2, 		/* Result_A = ALPHA0 * 256, SEL0_Out = ALPHA1*/
	VIOC_WMIX_ALPHA_ACON0_3, 		/* Result_A = ALPHA0 * 256, SEL0_Out = 256 - ALPHA1*/
	VIOC_WMIX_ALPHA_ACON0_4, 		/* Result_A = ALPHA0 * 128, SEL0_Out = ALPHA1*/
	VIOC_WMIX_ALPHA_ACON0_5, 		/* Result_A = ALPHA0 * 128, SEL0_Out = 256 - ALPHA1*/
	VIOC_WMIX_ALPHA_ACON0_6, 		/* Result_A = 0, SEL0_Out = ALPHA1*/
	VIOC_WMIX_ALPHA_ACON0_7, 		/* Result_A = 0, SEL0_Out = 256 - ALPHA1*/
	VIOC_WMIX_ALPHA_ACON0_MAX		/* Result : AlphaDataOut = (Result_A + Result_B + 128) / 256*/
}VIOC_WMIX_ALPHA_ACON0_Type;

typedef enum
{
	VIOC_WMIX_ALPHA_ACON1_0 = 0, 	/* Result_B = ALPHA1 * SEL1_Out, SEL1_Out = ALPHA0*/            
	VIOC_WMIX_ALPHA_ACON1_1, 		/* Result_B = ALPHA1 * SEL1_Out, SEL1_Out = 256 - ALPHA0*/      
	VIOC_WMIX_ALPHA_ACON1_2, 		/* Result_B = ALPHA1 * 256, SEL1_Out = ALPHA0*/                 
	VIOC_WMIX_ALPHA_ACON1_3, 		/* Result_B = ALPHA1 * 256, SEL1_Out = 256 - ALPHA0*/           
	VIOC_WMIX_ALPHA_ACON1_4, 		/* Result_B = ALPHA1 * 128, SEL1_Out = ALPHA0*/                 
	VIOC_WMIX_ALPHA_ACON1_5, 		/* Result_B = ALPHA1 * 128, SEL1_Out = 256 - ALPHA0*/           
	VIOC_WMIX_ALPHA_ACON1_6, 		/* Result_B = 0, SEL1_Out = ALPHA0*/                            
	VIOC_WMIX_ALPHA_ACON1_7, 		/* Result_B = 0, SEL1_Out = 256 - ALPHA0*/                      
	VIOC_WMIX_ALPHA_ACON1_MAX		/* Result : AlphaDataOut = (Result_A + Result_B + 128) / 256*/  
}VIOC_WMIX_ALPHA_ACON1_Type;

typedef enum
{
	VIOC_WMIX_ALPHA_CCON0_0 = 0, 	/* Result_A = PixelDataA * SEL0_Out, SEL0_Out = ALPHA0*/
	VIOC_WMIX_ALPHA_CCON0_1, 		/* Result_A = PixelDataA * SEL0_Out, SEL0_Out = ALPHA1*/
	VIOC_WMIX_ALPHA_CCON0_2, 		/* Result_A = PixelDataA * SEL0_Out, SEL0_Out = 256 - ALPHA0*/
	VIOC_WMIX_ALPHA_CCON0_3, 		/* Result_A = PixelDataA * SEL0_Out, SEL0_Out = 256 - ALPHA1*/
	VIOC_WMIX_ALPHA_CCON0_4, 		/* Result_A = PixelDataA * 256, SEL0_Out = ALPHA0*/
	VIOC_WMIX_ALPHA_CCON0_5, 		/* Result_A = PixelDataA * 256, SEL0_Out = ALPHA1*/
	VIOC_WMIX_ALPHA_CCON0_6, 		/* Result_A = PixelDataA * 256, SEL0_Out = 256 - ALPHA0*/
	VIOC_WMIX_ALPHA_CCON0_7, 		/* Result_A = PixelDataA * 256, SEL0_Out = 256 - ALPHA1*/
	VIOC_WMIX_ALPHA_CCON0_8, 		/* Result_A = PixelDataA * 128, SEL0_Out = ALPHA0*/
	VIOC_WMIX_ALPHA_CCON0_9, 		/* Result_A = PixelDataA * 128, SEL0_Out = ALPHA1*/
	VIOC_WMIX_ALPHA_CCON0_10, 		/* Result_A = PixelDataA * 128, SEL0_Out = 256 - ALPHA0*/
	VIOC_WMIX_ALPHA_CCON0_11, 		/* Result_A = PixelDataA * 128, SEL0_Out = 256 - ALPHA1*/
	VIOC_WMIX_ALPHA_CCON0_12, 		/* Result_A = 0, SEL0_Out = ALPHA0*/
	VIOC_WMIX_ALPHA_CCON0_13, 		/* Result_A = 0, SEL0_Out = ALPHA1*/
	VIOC_WMIX_ALPHA_CCON0_14, 		/* Result_A = 0, SEL0_Out = 256 - ALPHA0*/
	VIOC_WMIX_ALPHA_CCON0_15, 		/* Result_A = 0, SEL0_Out = 256 - ALPHA1*/
	VIOC_WMIX_ALPHA_CCON0_MAX		/* Result : PixelDataOut = (Result_A + Result_B + 128) / 256*/
}VIOC_WMIX_ALPHA_CCON0_Type;

typedef enum
{
	VIOC_WMIX_ALPHA_CCON1_0 = 0,		/* Result_B = PixelDataB * SEL1_Out, SEL1_Out = ALPHA0*/      
	VIOC_WMIX_ALPHA_CCON1_1, 		/* Result_B = PixelDataB * SEL1_Out, SEL1_Out = ALPHA1*/      
	VIOC_WMIX_ALPHA_CCON1_2, 		/* Result_B = PixelDataB * SEL1_Out, SEL1_Out = 256 - ALPHA0*/
	VIOC_WMIX_ALPHA_CCON1_3, 		/* Result_B = PixelDataB * SEL1_Out, SEL1_Out = 256 - ALPHA1*/
	VIOC_WMIX_ALPHA_CCON1_4, 		/* Result_B = PixelDataB * 256, SEL1_Out = ALPHA0*/           
	VIOC_WMIX_ALPHA_CCON1_5, 		/* Result_B = PixelDataB * 256, SEL1_Out = ALPHA1*/           
	VIOC_WMIX_ALPHA_CCON1_6, 		/* Result_B = PixelDataB * 256, SEL1_Out = 256 - ALPHA0*/     
	VIOC_WMIX_ALPHA_CCON1_7, 		/* Result_B = PixelDataB * 256, SEL1_Out = 256 - ALPHA1*/     
	VIOC_WMIX_ALPHA_CCON1_8, 		/* Result_B = PixelDataB * 128, SEL1_Out = ALPHA0*/           
	VIOC_WMIX_ALPHA_CCON1_9, 		/* Result_B = PixelDataB * 128, SEL1_Out = ALPHA1*/           
	VIOC_WMIX_ALPHA_CCON1_10, 		/* Result_B = PixelDataB * 128, SEL1_Out = 256 - ALPHA0*/     
	VIOC_WMIX_ALPHA_CCON1_11, 		/* Result_B = PixelDataB * 128, SEL1_Out = 256 - ALPHA1*/     
	VIOC_WMIX_ALPHA_CCON1_12, 		/* Result_B = 0, SEL1_Out = ALPHA0*/                          
	VIOC_WMIX_ALPHA_CCON1_13, 		/* Result_B = 0, SEL1_Out = ALPHA1*/                          
	VIOC_WMIX_ALPHA_CCON1_14, 		/* Result_B = 0, SEL1_Out = 256 - ALPHA0*/                    
	VIOC_WMIX_ALPHA_CCON1_15, 		/* Result_B = 0, SEL1_Out = 256 - ALPHA1*/                    
	VIOC_WMIX_ALPHA_CCON1_MAX		/* Result : PixelDataOut = (Result_A + Result_B + 128) / 256*/
}VIOC_WMIX_ALPHA_CCON1_Type;

#define VIOC_WMIX_ALPHA_REGION_A		0x0
#define VIOC_WMIX_ALPHA_REGION_B		0x1
#define VIOC_WMIX_ALPHA_REGION_C		0x2
#define VIOC_WMIX_ALPHA_REGION_D		0x3


extern void VIOC_API_WMIX_SetOverlayAlphaROPMode( VIOC_WMIX *pWMIX, unsigned int layer, unsigned int opmode );
extern void VIOC_API_WMIX_SetOverlayAlphaValue( VIOC_WMIX *pWMIX, unsigned int layer, unsigned int alpha0, unsigned int alpha1 );
extern void VIOC_API_WMIX_SetOverlayAlphaSelection(VIOC_WMIX *pWMIX, unsigned int layer,unsigned int asel );
extern void VIOC_API_WMIX_SetOverlayAlphaValueControl(VIOC_WMIX *pWMIX, unsigned int layer, unsigned int region, unsigned int acon0, unsigned int acon1 );
extern void VIOC_API_WMIX_SetOverlayAlphaColorControl(VIOC_WMIX *pWMIX, unsigned int layer, unsigned int region, unsigned int ccon0, unsigned int ccon1 );
extern void VIOC_WMIX_SetSWReset(unsigned int WMIX, unsigned int RDMA, unsigned int WDMA);
extern void VIOC_WMIX_DUMP(VIOC_WMIX *pWMIX);

#endif


