/*
 * linux/arch/arm/mach-tcc897x/include/mach/io_g2d.h
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


 #ifndef __VIOC_G2D_H__
#define	__VIOC_G2D_H__

/*******************************************************************************
*   7. Overlay Mixer                                    (Base Addr = 0x12400000)
********************************************************************************/

typedef struct
{
	unsigned		RESERVED0	:	2;
	unsigned		FCH0_SADDR0 	:	30;
}	OM_FCH0_SADDR0;

typedef union
{
	unsigned 	long		nREG;
	OM_FCH0_SADDR0	bREG;
}	OM_FCH0_SADDR0_u;

typedef struct
{
	unsigned		SFSIZE_X	:	12;
	unsigned				:	4;
	unsigned		SFSIZE_Y	:	12;
	unsigned				:	4;
}	OM_FCH0_SFSIZE;

typedef union
{
	unsigned long		nREG;
	OM_FCH0_SFSIZE		bREG;
}	OM_FCH0_SFSIZE_u;

typedef struct
{
	unsigned		SOFF_X	:	12;
	unsigned				:	4;
	unsigned		SOFF_Y	:	12;
	unsigned				:	4;
}	OM_FCH0_SOFF;

typedef union
{
	unsigned long		nREG;
	OM_FCH0_SOFF		bREG;
}	OM_FCH0_SOFF_u;

typedef struct
{
	unsigned		SISIZE_X	:	12;
	unsigned				:	4;
	unsigned		SISIZE_Y	:	12;
	unsigned				:	4;
}	OM_FCH0_SISIZE;

typedef union
{
	unsigned long		nREG;
	OM_FCH0_SISIZE		bREG;
}	OM_FCH0_SISIZE_u;


typedef struct
{
	unsigned		WOFF_X	:	12;
	unsigned				:	4;
	unsigned		WOFF_Y	:	12;
	unsigned				:	4;
}	OM_FCH0_WOFF;

typedef union
{
	unsigned long		nREG;
	OM_FCH0_WOFF		bREG;
}	OM_FCH0_WOFF_u;

typedef struct
{
	unsigned		SDFRM	:	5;
	unsigned		ZF		:	2;
	unsigned				:	1;
	unsigned		OPMODE 	:	3;
	unsigned		SSUV	:	1;
	unsigned		LUTE	:	1;
	unsigned				:	2;
	unsigned		MABC	:	1;
	unsigned				:	8;
	unsigned		SSB		:	3;
	unsigned				:	5;
}	OM_FCH0_SCTRL;
	
typedef union
{
	unsigned	long		nREG;
	OM_FCH0_SCTRL	bREG;
}	OM_FCH0_SCTRL_u;

typedef struct
{
	unsigned		CHROMA_BV	:	8;
	unsigned		CHROMA_GU	:	8;
	unsigned		CHROMA_RY	:	8;
	unsigned		 			:	8;

}	OM_S0_CHROMA;
	
typedef union
{
	unsigned	long		nREG;
	OM_S0_CHROMA	bREG;
}	OM_S0_CHROMA_u;


typedef struct
{
	unsigned		PAR_BV	:	8;
	unsigned		PAR_GU	:	8;
	unsigned		PAR_RY	:	8;
	unsigned	 			:	8;
}	OM_S0_PAR;
	
typedef union
{
	unsigned	long		nREG;
	OM_S0_PAR		bREG;
}	OM_S0_PAR_u;


typedef struct
{
	unsigned		S0SEL	:	1;
	unsigned				:	1;
	unsigned		S1SEL	:	1;
	unsigned			 	:	13;
	unsigned		S0YM	:	2;
	unsigned		S1YM	:	2;
	unsigned				:	4;
	unsigned		S0YE	:	1;
	unsigned		S1YE	:	1;
	unsigned				:	6;
}	OM_SF_CTRL;
	
typedef union
{
	unsigned	long		nREG;
	OM_SF_CTRL	bREG;
}	OM_SF_CTRL_u;

typedef struct
{
	unsigned		S0ARIM	:	3;
	unsigned				:	1;
	unsigned		S1ARIM	:	3;
	unsigned			 	:	9;
	unsigned		S0CE	:	1;
	unsigned		S1CE	:	1;
	unsigned				:	14;
}	OM_SA_CTRL;
	
typedef union
{
	unsigned	long		nREG;
	OM_SA_CTRL	bREG;
}	OM_SA_CTRL_u;

typedef struct
{
	unsigned		ALPHA0	:	8;
	unsigned		ALPHA1	:	8;
	unsigned				:	16;
}	OM_OP0_ALPHA;
	
typedef union
{
	unsigned	long		nREG;
	OM_OP0_ALPHA	bREG;
}	OM_OP0_ALPHA_u;


typedef struct
{
	unsigned		PAT_BV	:	8;
	unsigned		PAT_GU	:	8;
	unsigned		PAT_RY	:	8;
	unsigned				:	8;
}	OM_OP0_PAT;
	
typedef union
{
	unsigned	long		nREG;
	OM_OP0_PAT		bREG;
}	OM_OP0_PAT_u;

typedef struct
{
	unsigned		O_MODE	:	5;
	unsigned				:	3;
	unsigned		CSEL	:	2;
	unsigned				:	2;
	unsigned		ATUNE	:	2;
	unsigned				:	2;
	unsigned		CCON0	:	4;
	unsigned		CCON1	:	4;
	unsigned		ACON0	:	3;
	unsigned				:	1;
	unsigned		ACON1	:	3;
	unsigned				:	1;			
}	OM_OP0_CTRL;
	
typedef union
{
	unsigned	long		nREG;
	OM_OP0_CTRL		bREG;
}	OM_OP0_CTRL_u;


typedef struct
{
	unsigned		DDFRM	:	5;
	unsigned		DEN		:	1;
	unsigned		DOP		:	1;
	unsigned				:	1;
	unsigned		OPMODE	:	3;
	unsigned		DSUV	:	1;
	unsigned				:	1;
	unsigned		R2YMD	:	2;
	unsigned		R2YE	:	1;
	unsigned		XSEL	:	2;
	unsigned		YSEL	:	1;
	unsigned				:	2;
	unsigned		MABC	:	1;
	unsigned				:	2;
	unsigned		DSB		:	3;	
	unsigned				:	5;
}	OM_BCH_DCTRL;
	
typedef union
{
	unsigned	long		nREG;
	OM_BCH_DCTRL		bREG;
}	OM_BCH_DCTRL_u;


typedef struct
{
	unsigned		DDMAT00	:	5;
	unsigned				:	3;
	unsigned		DDMAT01	:	5;
	unsigned				:	3;
	unsigned		DDMAT02	:	5;
	unsigned				:	3;
	unsigned		DDMAT03	:	5;
	unsigned				:	3;	
}	OM_BCH_DDMAT0;
	
typedef union
{
	unsigned	long			nREG;
	OM_BCH_DDMAT0		bREG;
}	OM_BCH_DDMAT0_u;

typedef struct
{
	unsigned		EN		:	2;
	unsigned				:	14;
	unsigned		IEN		:	1;
	unsigned				:	14;
	unsigned		MODE	:	1;	
}	OM_OM_CTRL;
	
typedef union
{
	unsigned	long		nREG;
	OM_OM_CTRL		bREG;
}	OM_OM_CTRL_u;

typedef struct
{
	unsigned		IRQ		:	1;
	unsigned				:	15;
	unsigned		FLG		:	1;
	unsigned				:	15;
} 	OM_OM_IREQ;

typedef union
{
	unsigned	long		nREG;
	OM_OM_IREQ		bREG;
}	OM_OM_IREQ_u;


typedef struct
{
	unsigned		LUT		:	32;
} 	OM_OM_LUT;

typedef union
{
	unsigned	long		nREG;
	OM_OM_LUT		bREG;
}	OM_OM_LUT_u;



typedef struct _OVERLAYMIXER{

	volatile OM_FCH0_SADDR0_u		FCH0_SADDR0;    // 0x000  R/W  0x00000000   Front-End Channel 0 Source Address 0
	volatile OM_FCH0_SADDR0_u  	FCH0_SADDR1;    // 0x004  R/W  0x00000000   Front-End Channel 0 Source Address 1
	volatile OM_FCH0_SADDR0_u 	FCH0_SADDR2;    // 0x008  R/W  0x00000000   Front-End Channel 0 Source Address 2
	volatile OM_FCH0_SFSIZE_u  	FCH0_SFSIZE;    // 0x00C  R/W  0x00000000   Front-End Channel 0 Source Frame Pixel Size
	volatile OM_FCH0_SOFF_u		FCH0_SOFF;      // 0x010  R/W  0x00000000   Front-End Channel 0 Source Pixel Offset
	volatile OM_FCH0_SISIZE_u		FCH0_SISIZE;    // 0x014  R/W  0x00000000   Front-End Channel 0 Source Image Pixel Size
	volatile OM_FCH0_WOFF_u		FCH0_WOFF;      // 0x018  R/W  0x00000000   Front-End Channel 0 Window Pixel Offset
	volatile OM_FCH0_SCTRL_u		FCH0_SCTRL;     // 0x01C  R/W  0x00000000   Front-End Channel 0 Control
	volatile OM_FCH0_SADDR0_u  	FCH1_SADDR0;    // 0x020  R/W  0x00000000   Front-End Channel 1 Source Address 0
	volatile OM_FCH0_SADDR0_u  	FCH1_SADDR1;    // 0x024  R/W  0x00000000   Front-End Channel 1 Source Address 1
	volatile OM_FCH0_SADDR0_u  	FCH1_SADDR2;    // 0x028  R/W  0x00000000   Front-End Channel 1 Source Address 2
	volatile OM_FCH0_SFSIZE_u  	FCH1_SFSIZE;    // 0x02C  R/W  0x00000000   Front-End Channel 1 Source Frame Pixel Size
	volatile OM_FCH0_SOFF_u  	FCH1_SOFF;      // 0x030  R/W  0x00000000   Front-End Channel 1 Source Pixel Offset
	volatile OM_FCH0_SISIZE_u  	FCH1_SISIZE;    // 0x034  R/W  0x00000000   Front-End Channel 1 Source Image Pixel Size
	volatile OM_FCH0_WOFF_u 		FCH1_WOFF;      // 0x038  R/W  0x00000000   Front-End Channel 1 Window Pixel Offset
	volatile OM_FCH0_SCTRL_u  		FCH1_SCTRL;     // 0x03C  R/W  0x00000000   Front-End Channel 1 Control
	volatile unsigned  int 			reserved0[16];  // 0x40 ~ 0x7C 	

	volatile OM_S0_CHROMA_u		S0_CHROMA;     // 0x080  R/W  0x00000000   Source 0 Chroma-Key Parameter
	volatile OM_S0_PAR_u			S0_PAR;           // 0x084  R/W  0x00000000   Source 0 Arithmetic Parameter
	volatile unsigned  int 			reserved1[6];	   // 0x88 ~ 0x9C

	volatile OM_SF_CTRL_u			SF_CTRL;        // 0x0A0  R/W  0x00000000   Source Function Control Register
	volatile OM_SA_CTRL_u			SA_CTRL;        // 0x0A4  R/W  0x00000000   Source Arithmetic Control Register
	volatile unsigned  int 			reserved2[2];	  // 0x0A8 ~ 0x0AC

	volatile OM_OP0_ALPHA_u		OP0_ALPHA;      // 0x0B0  R/W  0x00000000   Source Operator 0 Alpha
	volatile unsigned  int 			reserved3[3];    // 0x0B4 ~ 0x0BC
	
	volatile OM_OP0_PAT_u			OP0_PAT;        // 0x0C0  R/W  0x00000000   Source Operator 0 Pattern
	volatile unsigned  int 			reserved4[3];   // 0x0C4 ~ 0x0CC
	
	volatile OM_OP0_CTRL_u		OP0_CTRL;       // 0x0D0  R/W  0x00000000   Source Operation 0 Control Register
	volatile unsigned  int 			reserved5[3];   // 0x0D4 ~ 0x0DC
	
	volatile OM_FCH0_SADDR0_u		BCH_DADDR0;     // 0x0E0  R/W  0x00000000   Back-End Channel Destination Address 0
	volatile OM_FCH0_SADDR0_u		BCH_DADDR1;     // 0x0E4  R/W  0x00000000   Back-End Channel Destination Address 1
	volatile OM_FCH0_SADDR0_u  	BCH_DADDR2;     // 0x0E8  R/W  0x00000000   Back-End Channel Destination Address 2
	volatile OM_FCH0_SFSIZE_u  	BCH_DFSIZE;     // 0x0EC  R/W  0x00000000   Back-End Channel Destination Frame Pixel Size
	volatile OM_FCH0_SOFF_u	  	BCH_DOFF;       // 0x0F0  R/W  0x00000000   Back-End Channel Destination Pixel Offset
	volatile OM_BCH_DCTRL_u		BCH_DCTRL;      // 0x0F4  R/W  0x00000000   Back-End Channel Control
	volatile unsigned  int 			reserved6[2];	   // 0x0F8 ~ 0x0FC

	volatile OM_BCH_DDMAT0_u		BCH_DDMAT0;     // 0x100  R/W  0x00000000   Back-End Channel Destination Dither Matrix 0
	volatile OM_BCH_DDMAT0_u		BCH_DDMAT1;     // 0x104  R/W  0x00000000   Back-End Channel Destination Dither Matrix 1
	volatile OM_BCH_DDMAT0_u		BCH_DDMAT2;     // 0x108  R/W  0x00000000   Back-End Channel Destination Dither Matrix 2
	volatile OM_BCH_DDMAT0_u		BCH_DDMAT3;     // 0x10C  R/W  0x00000000   Back-End Channel Destination Dither Matrix 3
	
	volatile OM_OM_CTRL_u			OM_CTRL;        // 0x110  R/W  0x00000000   Overlay Mixer Control
	volatile OM_OM_IREQ_u			OM_IREQ;        // 0x114  R/W  0x00000000   Overlay Mixer Interrupt Request
	volatile unsigned  int 			reserved7[186];   // 0x118 ~ 0x3FC
	
	volatile OM_OM_LUT_u			FCH0_LUT[256];  // 0x400 ~ 0x7FF Front-End Channel 1 Lookup Table
	volatile OM_OM_LUT_u			FCH1_LUT[256];  // 0x800 ~ 0xBFF Front-End Channel 1 Lookup Table	
	} OVERLAYMIXER, *POVERLAYMIXER;

#define HwGE_FCH_SSB                    (Hw24+Hw25+Hw26)                    // Operation Mode
#define HwGE_DCH_SSB                    (Hw24+Hw25+Hw26)                    // Operation Mode

// Front-End Channel 0 Control
#define HwGE_FCHO_OPMODE                (Hw8+Hw9+Hw10)                      // Operation Mode
#define HwGE_FCHO_SDFRM                 (Hw0+Hw1+Hw2+Hw3+Hw4)               // Source Data Format

// Front-End Channel 1 Control
#define HwGE_FCH1_OPMODE                (Hw8+Hw9+Hw10)                      // Operation Mode
#define HwGE_FCH1_SDFRM                 (Hw0+Hw1+Hw2+Hw3+Hw4)               // Source Data Format

// Front-End Channel 2 Control
#define HwGE_FCH2_OPMODE                (Hw8+Hw9+Hw10)                      // Operation Mode
#define HwGE_FCH2_SDFRM                 (Hw0+Hw1+Hw2+Hw3+Hw4)               // Source Data Format

// Source Control
#define Hw2D_SACTRL_S2_ARITHMODE        (Hw10+Hw9+Hw8)
#define Hw2D_SACTRL_S1_ARITHMODE        (Hw6+Hw5+Hw4)
#define Hw2D_SACTRL_S0_ARITHMODE        (Hw2+Hw1+Hw0)
#define Hw2D_SFCTRL_S2_Y2REN            (Hw26)
#define Hw2D_SFCTRL_S1_Y2REN            (Hw25)
#define Hw2D_SFCTRL_S0_Y2REN            (Hw24)
#define Hw2D_SFCTRL_S2_Y2RMODE          (Hw21+Hw20)
#define Hw2D_SFCTRL_S1_Y2RMODE          (Hw19+Hw18)
#define Hw2D_SFCTRL_S0_Y2RMODE          (Hw17+Hw16)
#define Hw2D_SACTRL_S2_CHROMAEN         (Hw18)
#define Hw2D_SACTRL_S1_CHROMAEN         (Hw17)
#define Hw2D_SACTRL_S0_CHROMAEN         (Hw16)
#define Hw2D_SFCTRL_S3_SEL			(Hw6+Hw7)
#define Hw2D_SFCTRL_S2_SEL              (Hw5+Hw4)
#define Hw2D_SFCTRL_S1_SEL              (Hw3+Hw2)
#define Hw2D_SFCTRL_S0_SEL              (Hw1+Hw0)

// Source Operator Pattern
#define HwGE_OP_ALL                     (HwGE_ALPHA + HwGE_PAT_RY + HwGE_PAT_GU + HwGE_PAT_BV)
#define HwGE_ALPHA                      (HwGE_PAT_GU + HwGE_PAT_BV )                // ALPHA VALUE
#define HwGE_PAT_RY                     (Hw16+Hw17+Hw18+Hw19+Hw20+Hw21+Hw22+Hw23)   // Pattern Value RED,   Y
#define HwGE_PAT_GU                     (Hw8+Hw9+Hw10+Hw11+Hw12+Hw13+Hw14+Hw15)     // Pattern Value GREEN, U
#define HwGE_PAT_BV                     (Hw0+Hw1+Hw2+Hw3+Hw4+Hw5+Hw6+Hw7)           // Pattern Value BULE,  V

// Source Operation Control
#define HwGE_OP_CTRL_ACON1              (Hw30+Hw29+Hw28)                    // Alpha-value control 1
#define HwGE_OP_CTRL_ACON0              (Hw26+Hw25+Hw24)                    // Alpha-value control 0
#define HwGE_OP_CTRL_CCON1              (Hw23+Hw22+Hw21+Hw20)               // color control 1
#define HwGE_OP_CTRL_CCON0              (Hw19+Hw18+Hw17+Hw16)               // color control 1
#define HwGE_OP_CTRL_ATUNE              (Hw13+Hw12)                         // Alpha value tuning 
#define HwGE_OP_CTRL_CSEL               (Hw9+Hw8)                           // chroma-key 
#define HwGE_OP_CTRL_OPMODE             (Hw4+Hw3+Hw2+Hw1+Hw0)               // operation mode

// Back -End Channel Control
#define HwGE_BCH_DCTRL_MABC             Hw21
#define HwGE_BCH_DCTRL_YSEL             Hw18                                // YUV4:4:4 to YUVx:x:x Y Control
#define HwGE_BCH_DCTRL_XSEL             (Hw16+Hw17)                         // YUV4:4:4 to YUVx:x:x X Control
#define HwGE_BCH_DCTRL_CEN              Hw15                                // Destination Format Converter Control
#define HwGE_BCH_DCTRL_CMODE            (Hw13+Hw14)                         // RGBtoYUV Converter Type
#define HwGE_BCH_DCTRL_DSUV             Hw11
#define HwGE_BCH_DCTRL_OPMODE           (Hw8+Hw9+Hw10)                      // Operation Mode COPY, MIRROR, ROTATE
#define HwGE_BCH_DCTRL_DOP              Hw6
#define HwGE_BCH_DCTRL_DEN              Hw5
#define HwGE_BCH_DCTRL_DDFRM            (Hw0+Hw1+Hw2+Hw3+Hw4)               // Destination Data Format

// Graphic Engine Control
#define HwGE_GE_INT_EN                  Hw16                                // Graphic Engine Interrupt Enable
#define HwGE_GE_CTRL_EN                 (Hw0+Hw1+Hw2)                       // Graphic Engine Enable

// Graphic Engine Interrupt Request
#define HwGE_GE_IREQ_FLG                Hw16                                // Graphic Engine Flag Bit
#define HwGE_GE_IREQ_IRQ                Hw0

#endif

 
