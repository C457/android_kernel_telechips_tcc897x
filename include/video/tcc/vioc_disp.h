/*
 * linux/arch/arm/mach-tcc893x/include/mach/vioc_disp.h
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

#ifndef __VIOC_DISP_H__
#define	__VIOC_DISP_H__

#include  "tcc_video_regs.h"
#include  "vioc_config.h"

enum {
	VIOC_HDMI_1920X1080P_60Hz = 0,
	VIOC_HDMI_1920X1080P_59Hz,
	VIOC_HDMI_1920X1080P_50Hz,
	VIOC_HDMI_1920X1080I_60Hz,
	VIOC_HDMI_1920X1080I_59Hz,
	VIOC_HDMI_1920X1080I_50Hz,
	VIOC_HDMI_1280X720P_60Hz,
	VIOC_HDMI_1280X720P_59Hz,
	VIOC_HDMI_1280X720P_50Hz,
	VIOC_HDMI_720X480P_60Hz,
	VIOC_HDMI_720X480P_59Hz,
	VIOC_HDMI_720X576P_50Hz,
	VIOC_HDMI_720X480I_60Hz,
	VIOC_HDMI_720X480I_59Hz,
	VIOC_HDMI_720X576I_50Hz,
	VIOC_HDVE_720X480I_60Hz,
	VIOC_HDVE_720X576I_50Hz,
	VIOC_HDVE_720X480P_60Hz,
	VIOC_HDVE_720X576P_50Hz,
	VIOC_HDVE_1280X720P_60Hz,
	VIOC_HDVE_1280X720P_50Hz,
	VIOC_HDVE_1920X1080I_60Hz,
	VIOC_HDVE_1920X1080I_50Hz,
	VIOC_HDVE_1920X1080P_30Hz,
	VIOC_HDVE_SMPTE295I_50Hz,
	VIOC_HDVE_SMPTE295P_30Hz,
	VIOC_SDVE_NTSC_CVBS,
	VIOC_SDVE_PAL_CVBS,
	VIOC_SDVE_NTSC_SVIDEO,
	VIOC_SDVE_PAL_SVIDEO,
	VIOC_SDVE_NTSC_YPbPr,
	VIOC_SDVE_PAL_YPbPr,
	VIOC_SDVE_NTSC_RGB,
	VIOC_SDVE_PAL_RGB,
	VIOC_DEFAUKLT_TIME_MAX
};

typedef struct {
	unsigned int	nType;
	unsigned int	CLKDIV;
	unsigned int	IV;
	unsigned int	IH;
	unsigned int	IP;
	unsigned int	DP;
	unsigned int	NI;
	unsigned int	TV;
	unsigned int	LPW;
	unsigned int	LPC;
	unsigned int	LSWC;
	unsigned int	LEWC;
	unsigned int	FPW;
	unsigned int	FLC;
	unsigned int	FSWC;
	unsigned int	FEWC;
	unsigned int	FPW2;
	unsigned int	FLC2;
	unsigned int	FSWC2;
	unsigned int	FEWC2;
} VIOC_TIMING_INFO;

typedef	struct
{
	unsigned		LEN		: 1;	// bit 0
	unsigned		SWAP	: 3;
	unsigned		Y2R		: 1;
	unsigned		SRST	: 1;
	unsigned		TV		: 1;
	unsigned		NI		: 1;
	unsigned		DP		: 1;
	unsigned		R2Y		: 1;
	unsigned		CLEN	: 1;	// bit 10
	unsigned		IP		: 1;
	unsigned		IH		: 1;
	unsigned		IV		: 1;
	unsigned		ID		: 1;
	unsigned		SREQ	: 1;
	unsigned		PXDW	: 5;	// bit 20
	unsigned		Y2RMD	: 3;
	unsigned		CKG		: 1;
	unsigned		C656	: 1;
	unsigned		ADVI	: 1;
	unsigned		R2YMD	: 3;
	unsigned		EVS		: 1;
	unsigned		EVP		: 1;	// bit 31
}	VIOC_DISP_LCTRL;

typedef	union
{
	unsigned long		nREG;
	VIOC_DISP_LCTRL		bREG;
}	VIOC_DISP_LCTRL_u;

typedef union
{
	unsigned	ALIGN	: 2;
	unsigned	SWAPBF	: 3;
	unsigned 	RESERVE	: 27;
}	VIOC_DISP_DALIGN;

typedef union
{
	unsigned long		nREG;
	VIOC_DISP_DALIGN	bREG;
}	VIOC_DISP_DALIGN_u;

typedef	struct
{
	unsigned 		PXCLKDIV	: 8; /* PXCLK = LCLK/(2*PXCLKDIV) LCLK : Display Device Pixel clock form CKC Block, PXCLK : Pixel Clock to the External Display Device */
	unsigned 		RESERVE0	: 8;
	unsigned 		ACDIV   		: 8;
	unsigned 		RESERVE1	: 8;
}	VIOC_DISP_LCLKDIV;

typedef	union
{
	unsigned long		nREG;
	VIOC_DISP_LCLKDIV	bREG;
}	VIOC_DISP_LCLKDIV_u;

typedef	struct
{
	unsigned 		LPC		:14;
	unsigned 		RESERVE0	: 2;
	unsigned 		LPW		: 9;
	unsigned 		RESERVE1	: 7;
}	VIOC_DISP_LHTIME1;

typedef	union
{
	unsigned long		nREG;
	VIOC_DISP_LHTIME1	bREG;
}	VIOC_DISP_LHTIME1_u;

typedef	struct
{
	unsigned		LEWC	: 9;
	unsigned		RESERVE0	: 7;
	unsigned		LSWC	: 9;
	unsigned		RESERVE1	: 7;
}	VIOC_DISP_LHTIME2;

typedef	union
{
	unsigned	long		nREG;
	VIOC_DISP_LHTIME2	bREG;
}	VIOC_DISP_LHTIME2_u;

typedef	struct
{
	unsigned		FLC		:14;
	unsigned		RESERVE0	: 2;
	unsigned		FPW	: 6;
	unsigned		VDF		: 4;
	unsigned		RESERVE1	: 1;
	unsigned		VDB		: 5;
}	VIOC_DISP_LVTIME1;

typedef	union
{
	unsigned long		nREG;
	VIOC_DISP_LVTIME1	bREG;
}	VIOC_DISP_LVTIME1_u;

typedef	struct
{
	unsigned		FEWC	: 9;
	unsigned		RESERVE0		: 7;
	unsigned		FSWC	: 9;
	unsigned		RESERVE1		: 7;
}	VIOC_DISP_LVTIME2;

typedef	union
{
	unsigned long		nREG;
	VIOC_DISP_LVTIME2	bREG;
}	VIOC_DISP_LVTIME2_u;

typedef	struct
{
	unsigned 		CTH		: 4;
	unsigned		STH		: 4;
	unsigned 		RESERVED0	: 8;
	unsigned		HDMIVS	: 2;
	unsigned 		HDMIFLD	: 2;
	unsigned		EOFM	: 2;
	unsigned		BM		: 1;
	unsigned		RESERVED1	: 7;
	unsigned		MEN		: 1;
	unsigned		EN		: 1;
}	VIOC_DISP_DEFR;


typedef	union
{
	unsigned long		nREG;
	VIOC_DISP_DEFR	bREG;
}	VIOC_DISP_DEFR_u;

typedef struct
{
	unsigned		FLC		: 14;
	unsigned		RESERVED0	: 2;
	unsigned		FPW		: 6;
	unsigned		RESERVED1	: 9;
	unsigned		MD		: 1;
} 	VIOC_DISP_DVTIME1;

typedef union
{
	unsigned long		nREG;
	VIOC_DISP_DVTIME1	bREG;
}	VIOC_DISP_DVTIME1_u;


typedef	struct
{
	unsigned		XPOS	:14;
	unsigned		RESERVE0   	: 2;
	unsigned		YPOS	:14;
	unsigned		RESERVE1   	: 2;
}	VIOC_DISP_LPOS;

typedef union
{
	unsigned long		nREG;
	VIOC_DISP_LPOS		bREG;
}	VIOC_DISP_LPOS_u;

typedef struct
{
	unsigned		VALUE	: 30;
	unsigned 		RESERVE	: 1;
	unsigned		BM		: 1;
}	VIOC_DISP_DBLK_VAL;

typedef union
{
	unsigned	long		nREG;
	VIOC_DISP_DBLK_VAL bREG;
}	VIOC_DISP_DBLK_VAL_u;


typedef	struct
{
	unsigned		RESERVE0		:30;
	unsigned		DSEL	: 1;
	unsigned		DEN 	: 1;
}	VIOC_DISP_DITHCTRL;

typedef	union
{
	unsigned long		nREG;
	VIOC_DISP_DITHCTRL	bREG;
}	VIOC_DISP_DITHCTRL_u;

typedef	struct
{
	unsigned		CLPH	: 8;
	unsigned		RESERVE0		: 8;
	unsigned		CLPL	: 8;
	unsigned		RESERVE1		: 8;
}	VIOC_DISP_CLIP;

typedef	union
{
	unsigned long		nREG;
	VIOC_DISP_CLIP		bREG;
}	VIOC_DISP_CLIP_u;

typedef	struct
{
	unsigned		HSIZE	: 13;
	unsigned		RESERVE0    	:  3;
	unsigned		VSIZE	: 13;
	unsigned		RESERVE1     	:  3;
}	VIOC_DISP_SIZE;

typedef	union
{
	unsigned long		nREG;
	VIOC_DISP_SIZE		bREG;
}	VIOC_DISP_SIZE_u;

typedef	struct
{
	unsigned 		BG0		: 16;
	unsigned 		BG1		: 16;

	unsigned 		BG2		: 16;
	unsigned 		BG3		: 16;
}	VIOC_DISP_BG;

typedef	union
{
	unsigned long		nREG[2];
	VIOC_DISP_BG		bREG;
}	VIOC_DISP_BG_u;

typedef	struct
{
	unsigned		HUE			: 8;
	unsigned		HEN			: 1;
	unsigned		RESERVE0    : 23;
}	VIOC_DISP_ENH0;

typedef	union
{
	unsigned long		nREG;
	VIOC_DISP_ENH0		bREG;
}	VIOC_DISP_ENH0_u;

typedef	struct
{
	unsigned		CONTRAST	: 10;
	unsigned		SAT			: 10;
	unsigned		BRIGHTNESS	: 10;
	unsigned		ENE			: 1;
	unsigned		RESERVE0    : 1;
}	VIOC_DISP_ENH1;

typedef	union
{
	unsigned long		nREG;
	VIOC_DISP_ENH1		bREG;
}	VIOC_DISP_ENH1_u;

typedef	struct
{
	unsigned		C0	:  4;
	unsigned		C1	:  4;
	unsigned		RESERVE0  	: 24;
}	VIOC_DISP_ADVI;

typedef	union
{
	unsigned long		nREG;
	VIOC_DISP_ADVI		bREG;
}	VIOC_DISP_ADVI_u;

typedef	struct
{
	#if 0
	unsigned		MAT0	: 16;
	unsigned		MAT1	: 16;
	unsigned		MAT2	: 16;
	unsigned		MAT3	: 16;
	#else
	unsigned	DITH00	: 3;
	unsigned	RESERVE0	: 1;
	unsigned	DITH01	: 3;
	unsigned	RESERVE1	: 1;
	unsigned	DITH02	: 3;
	unsigned	RESERVE2	: 1;
	unsigned	DITH03	: 3;
	unsigned	RESERVE3	: 1;
	unsigned	DITH10	: 3;
	unsigned	RESERVE4	: 1;
	unsigned	DITH11	: 3;
	unsigned	RESERVE5	: 1;
	unsigned	DITH12	: 3;
	unsigned	RESERVE6	: 1;
	unsigned	DITH13	: 3;
	unsigned	RESERVE7	: 1;	

	unsigned	DITH20	: 3;
	unsigned	RESERVE8	: 1;
	unsigned	DITH21	: 3;
	unsigned	RESERVE9	: 1;
	unsigned	DITH22	: 3;
	unsigned	RESERVE10	: 1;
	unsigned	DITH23	: 3;
	unsigned	RESERVE11	: 1;
	unsigned	DITH30	: 3;
	unsigned	RESERVE12	: 1;
	unsigned	DITH31	: 3;
	unsigned	RESERVE13	: 1;
	unsigned	DITH32	: 3;
	unsigned	RESERVE14	: 1;
	unsigned	DITH33	: 3;
	unsigned	RESERVE15	: 1;	
	#endif
}	VIOC_DISP_DMAT;

typedef	union
{
	unsigned	long		nREG[2];
	VIOC_DISP_DMAT		bREG;
}	VIOC_DISP_DMAT_u;

typedef	struct
{
	unsigned		FU		:  1;	// fifo underrun
	unsigned		VSR		:  1;	// VSYNC rising
	unsigned		VSF		:  1;	// VSYNC falling
	unsigned		RU		:  1;	// Register Update
	unsigned		DD		:  1;	// Disable Done
	unsigned		SREQ	:  1;	// Stop Request
	unsigned		RESERVE0	: 10;
	unsigned		RESERVE1	: 12;
	unsigned		DEOF	:  1;	// Device EOF status
	unsigned		TFIELD	:  1;	// Top field indicator
	unsigned		BUSY_	:  1;	// busy status
	unsigned		VS		:  1;	// VSYNC status
}	VIOC_DISP_STATUS;

typedef	union
{
	unsigned	long		nREG;
	VIOC_DISP_STATUS	bREG;
}	VIOC_DISP_STATUS_u;

typedef	struct
{
	unsigned		FU		:  1;	// fifo underrun
	unsigned		VSR		:  1;	// VSYNC rising
	unsigned		VSF		:  1;	// VSYNC falling
	unsigned		RU		:  1;	// Register Update
	unsigned		DD		:  1;	// Disable Done
	unsigned		SREQ	:  1;	// Stop Request
	unsigned		RESERVE0	: 10;
	unsigned		RESERVE1	: 16;
}	VIOC_DISP_IRQMASK;

typedef	union
{
	unsigned	long		nREG;
	VIOC_DISP_IRQMASK	bREG;
}	VIOC_DISP_IRQMASK_u;


typedef	struct _VIOC_DISP
{
	volatile VIOC_DISP_LCTRL_u		uCTRL;			// 0x000  R/W  0x30400000  Display Device Register
	volatile VIOC_DISP_DALIGN_u		uDALIGN;		// 0x004  R/W  0x00000000  Display Device Align Register
	volatile VIOC_DISP_LCLKDIV_u	uCLKDIV;		// 0x008  R/W  0x00000000  Display Device Clock Control Register
	volatile VIOC_DISP_LHTIME1_u	uLHTIME1;		// 0x00c  R/W  0x00000000  Display Device Horizontal Timing Register 1
	volatile VIOC_DISP_LHTIME2_u	uLHTIME2;		// 0x010  R/W  0x00000000  Display Device Horizontal Timing Register 2
	volatile VIOC_DISP_LVTIME1_u	uLVTIME1;		// 0x014	 R/W  0x00000000  Display Device Vertical Timing Register 1
	volatile VIOC_DISP_LVTIME2_u	uLVTIME2;		// 0x018	 R/W  0x00000000  Display Device Vertical Timing Register 2
	volatile VIOC_DISP_LVTIME1_u	uLVTIME3;		// 0x01C	 R/W  0x00000000  Display Device Vertical Timing Register 3
	volatile VIOC_DISP_LVTIME2_u	uLVTIME4;		// 0x020	 R/W  0x00000000  Display Device Vertical Timing Register 4
	volatile VIOC_DISP_DEFR_u		uDEFR;			// 0x024	 R/W  0xC00000F1  Display Emerfency Flag Register
	volatile VIOC_DISP_DVTIME1_u	uDVTIME1_3D;	// 0x028	 R/W  0x00000000  Display Device Vertical Timing Register 1 in 3D
	volatile VIOC_DISP_LVTIME2_u	uDVTIME2_3D;	// 0x02C  R/W  0x00000000  Display Device Vertical Timing Register 2 in 3D	
	volatile VIOC_DISP_LPOS_u		uLPOS;			// 0x030  R/W  0x00000000  Display Device Position Register 
	volatile VIOC_DISP_DVTIME1_u	uDVTIME3_3D;	// 0x034	 R/W  0x00000000  Display Device Vertical Timing Register 3 in 3D
	volatile VIOC_DISP_LVTIME2_u	uDVTIME4_3D;	// 0x038  R/W  0x00000000  Display Device Vertical Timing Register 4 in 3D	
	volatile VIOC_DISP_DBLK_VAL_u	uDBLK_VAL;		// 0x03C  R/W  0x00000000  Display Device Blank Value Register	
	volatile VIOC_DISP_DITHCTRL_u	uDITHCTRL;		// 0x040  R/W  0x00000000  Display Device Dithering Register  
	volatile VIOC_DISP_CLIP_u		uCLIPY;			// 0x044  R/W  0x000000FF  Display Device Luma clipping Register 
	volatile VIOC_DISP_CLIP_u		uCLIPC;			// 0x048  R/W  0x000000FF  Display Device chroma Clipping Register
	volatile VIOC_DISP_SIZE_u		uLSIZE;			// 0x04c  R/W  0x00000000  Display Device Size Register
	volatile VIOC_DISP_STATUS_u		uLSTATUS;		// 0x050  R/W  0x00000000  Display Device Status Register
	volatile VIOC_DISP_IRQMASK_u	uLIRQMASK;		// 0x054  R/W  0x00000000  Display Device Interrupt Masking Register
	volatile VIOC_DISP_BG_u			uBG;			// 0x058, 0x05c  R/W  0x00000000  Display Device BackGround Color 0, 1 Register
	volatile unsigned				reserved[3];	// reserved
	volatile VIOC_DISP_ENH0_u		uCENH0;			// 0x06C  R/W  0x00000020  Display Device Color Enhancement 0 Register
	volatile VIOC_DISP_ENH1_u		uCENH1;			// 0x070  R/W  0x00000020  Display Device Color Enhancement 1 Register
	volatile VIOC_DISP_ADVI_u		uADVI;			// 0x074  R/W  0x00000000  Display Device Interlacer Coefficient Register
	volatile VIOC_DISP_DMAT_u		uDMAT;			// 0x078, 0x07c  R/W  0x00000000  Display Device Dithering Matrix Register 0, 1
} VIOC_DISP,*PVIOC_DISP;

typedef struct LCDCDEFAULT
{
	unsigned   evp;    // External VSYNC Polarity,		   [0:Direct Input 1:Inverted Input]
	unsigned   evs;    // External VSYNC Enable,		   [0:Disabled	   1:Enabled]
	unsigned   r2ymd;  // RGB to YCbCr Conversion Option,  [0: 1: 2: 3:]
	unsigned   advi;   // Advanced Interlaced Mode
	unsigned   dtype;  // LCD DMA Type					   [0: AHB DDIC on, 1:AXI DDIC Off]
	unsigned   gen;    // Gamma Correction Enable Bit,	   [0:Disabled 1:Enabled]
	unsigned   ccir656;// CCIR 656 Mode,				   [0:Disable	1:Enable]
	unsigned   ckg;    // Clock Gating Enable for Timing   [0:cannot  1:can]
	unsigned   bpp;    // Bit Per Pixel for STN-LCD 	   [0:1bpp 1:2bpp 2:4bpp 3:RGB332 4:RGB444 5~7:reserved]
	unsigned   pxdw;   // Pixel Data Width, Refer to Datasheet
	unsigned   id;	   // Inverted Data Enable, 		   [0:Active High	1:Active Low]
	unsigned   iv;	   // Inverted Vertical Sync,		   [0:Active High	1:Active Low]
	unsigned   ih;	   // Inverted Horizontal Sync		   [0:Active High	1:Active Low]
	unsigned   ip;	   // Inverted pixel Clock, 		   [0:Rising Edge	1:Falling Edge]
	unsigned   clen;   // clipping Enable				   [0:Disable	1:Enable]
	unsigned   r2y;    // RGB to YCbCr Converter Enable    [0:Disable 1:Converted]
	unsigned   dp;	   // Double Pixel Data,			   [0: 1:]
	unsigned   ni;	   // Non-Interlaced.				   [0:Interlaced	1:non-interlaced]
	unsigned   tv;	   // TV Mode,						   [0: Normal mode	1:TV Mode]
	unsigned   y2r;    // YUV to RGB Converter Enable    [0:Disable 1:Converted]
} stLCDCTR;

typedef struct LDCTIMING
{
	// LHTIME1
	unsigned	lpw;	// Line Pulse Width, HSync width
	unsigned	lpc;	// Line Pulse Count, HActive width
	// LHTIME2
	unsigned	lswc;	// Line Start Wait Clock, HFront porch
	unsigned	lewc;	// Line End wait clock, HBack porch
	// LVTIME1
	unsigned	vdb;	// Back Porch Delay
	unsigned	vdf;	// Front Porch Delay
	unsigned	fpw;	// Frame Pulse Width, VSync Width
	unsigned	flc;	// Frame Line Count, VActive width
	// LVTIME2
	unsigned	fswc;	// Frame Start Wait Cycle
	unsigned	fewc;	// Frame End Wait Cycle
	// LVTIME3 [in Interlaced, even field timing, otherwise should be same with LVTIME1]
	unsigned	fpw2;	// Frame Pulse Width,
	unsigned	flc2;	// Frame Line count,
	// LVTIME4 [in Interlaced, even field timing, otherwise should be same with LVTIME2]
	unsigned	fswc2;	 // Frame Start Wait Cycle
	unsigned	fewc2;	 // Frame End Wait Cycle
} stLTIMING;

#define VIOC_DISP_IREQ_FU_MASK		0x00000001UL /*  fifo underrun */
#define VIOC_DISP_IREQ_VSR_MASK		0x00000002UL /*  VSYNC rising */
#define VIOC_DISP_IREQ_VSF_MASK 	0x00000004UL /* VSYNC falling */
#define VIOC_DISP_IREQ_RU_MASK 		0x00000008UL /* Register Update */
#define VIOC_DISP_IREQ_DD_MASK 		0x00000010UL /* Disable Done */
#define VIOC_DISP_IREQ_SREQ_MASK 	0x00000020UL /* Stop Request */

#define VIOC_DISP_IREQ_DEOF_MASK 	0x10000000UL
#define VIOC_DISP_IREQ_TFIELD_MASK 	0x20000000UL
#define VIOC_DISP_IREQ_BUSY_MASK 	0x40000000UL
#define VIOC_DISP_IREQ_VS_MASK 		0x80000000UL

// DISP Control Reg
#define HwDISP_EVP                      Hw31                                // External Vsync Polarity
#define HwDISP_EVS                      Hw30                                // External Vsync Enable
#define HwDISP_R2YMD                    (Hw29+Hw28+Hw27)                    // RGB to YCbCr Conversion Option
#define HwDISP_ADVI                     Hw26                                // Advanced interlaced mode
#define HwDISP_656                      Hw25                                // CCIR 656 Mode
#define HwDISP_CKG                      Hw24                                // Clock Gating Enable for Timing Generator
#define HwDISP_Y2RMD					(Hw23+Hw22+Hw21)					// YUV to RGB converter mode register
#define HwDISP_PXDW                     (Hw20+Hw19+Hw18+Hw17+Hw16)          // PXDW
#define HwDISP_SREQ                     Hw15				                // Stop Request
#define HwDISP_ID                       Hw14                                // Inverted Data Enable
#define HwDISP_IV                       Hw13                                // Inverted Vertical Sync
#define HwDISP_IH                       Hw12                                // Inverted Horizontal Sync
#define HwDISP_IP                       Hw11                                // Inverted Pixel Clock
#define HwDISP_CLEN                     Hw10                                // Clipping Enable
#define HwDISP_R2Y                      Hw9                                 // RGB to YCbCr Converter Enable for Output
#define HwDISP_DP                       Hw8                                 // Double Pixel Data
#define HwDISP_NI                       Hw7                                 // Non-Interlace
#define HwDISP_TV                       Hw6                                 // TV mode
#define HwDISP_SRST                     Hw5                                 // Device display reset
#define HwDISP_Y2R						Hw4                                 // YUV to RGB converter enable register
#define HwDISP_SWAP                     (Hw3+Hw2+Hw1)                       // Output RGB overlay swap
#define HwDISP_LEN                      Hw0                                 // LCD Controller Enable

#define DCTRL_EVP		31		// External Vsync Polarity
#define DCTRL_EVS		30		// External Vsync Enable
#define DCTRL_R2YMD		27		// RGB to YCbCr Conversion Option
#define DCTRL_ADVI		26		// Advanced interlaced mode
#define DCTRL_656		25		// CCIR 656 Mode
#define DCTRL_CKG		24		// Clock Gating Enable for Timing Generator
#define DCTRL_Y2RMD		21		// YUV to RGB converter mode register
#define DCTRL_PXDW		16		// PXDW
#define DCTRL_SREQ		15		// Stop Request
#define DCTRL_ID		14		// Inverted Data Enable
#define DCTRL_IV		13		// Inverted Vertical Sync
#define DCTRL_IH		12		// Inverted Horizontal Sync
#define DCTRL_IP		11		// Inverted Pixel Clock
#define DCTRL_CLEN		10		// Clipping Enable
#define DCTRL_R2Y		9		// RGB to YCbCr Converter Enable for Output
#define DCTRL_DP		8		// Double Pixel Data
#define DCTRL_NI		7		// Non-Interlace
#define DCTRL_TV		6		// TV mode
#define DCTRL_SRST		5		// Device display reset
#define DCTRL_Y2R		4		// YUV to RGB converter enable register
#define DCTRL_SWAP		1		// Output RGB overlay swap
#define DCTRL_LEN		0		// LCD Controller Enable

//DISP status
#define HwLSTATUS_VS                    Hw31                                // Monitoring vertical sync
#define HwLSTATUS_BUSY                  Hw30                                // Busy signal
#define HwLSTATUS_EF                    Hw29                                // Even-Field(Read Only). 0:Odd field or frame, 1:Even field or frame
#define HwLSTATUS_DEOF                  Hw28                                // DMA End of Frame flag
#define HwLSTATUS_I0EOF                 Hw27                                // Image 0 End of Frame flag
#define HwLSTATUS_I1EOF                 Hw26                                // Image 1 End of Frame flag
#define HwLSTATUS_I2EOF                 Hw25                                // Image 2 End of Frame flag
#define HwLSTATUS_IE2F                  Hw12                                // Image 2 end-of-frame falling edge flag
#define HwLSTATUS_IE2R                  Hw11                                // Image 2 end-of-frame rising edge flag
#define HwLSTATUS_IE1F                  Hw10                                // Image 1 end-of-frame falling edge flag
#define HwLSTATUS_IE1R                  Hw9                                 // Image 1 end-of-frame rising edge flag
#define HwLSTATUS_IE0F                  Hw8                                 // Image 0 end-of-frame falling edge flag
#define HwLSTATUS_IE0R                  Hw7                                 // Image 0 end-of-frame rising edge flag
#define HwLSTATUS_DEF                   Hw6                                 // DMA end-of-frame falling edge flag
#define HwLSTATUS_SREQ                  Hw5                                 // Device stop request
#define HwLSTATUS_DD                    Hw4                                 // Disable done
#define HwLSTATUS_RU                    Hw3                                 // Register update flag
#define HwLSTATUS_VSF                   Hw2                                 // VS falling flag
#define HwLSTATUS_VSR                   Hw1                                 // VS rising flag
#define HwLSTATUS_FU                    Hw0                                 // LCD output fifo under-run flag.
typedef struct LCDC_PARAM
{
	stLCDCTR	 LCDCTRL;
	stLTIMING	 LCDCTIMING;
} stLCDCPARAM;


/* Interface APIs */
extern void VIOC_DISP_SetDefaultTimingParam(VIOC_DISP *pDISP, unsigned int nType);
extern void VIOC_DISP_SetControlConfigure(VIOC_DISP *pDISP, stLCDCTR *pCtrlParam);
extern void VIOC_DISP_SetPXDW(VIOC_DISP *pDISP, unsigned char PXWD);
extern void VIOC_DISP_SetR2YMD(VIOC_DISP *pDISP, unsigned char R2YMD);
extern void VIOC_DISP_SetR2Y(VIOC_DISP *pDISP, unsigned char R2Y);
extern void VIOC_DISP_SetSWAP(VIOC_DISP *pDISP, unsigned char SWAP);
extern void VIOC_DISP_SetSize (VIOC_DISP *pDISP, unsigned int nWidth, unsigned int nHeight);
extern void VIOC_DISP_GetSize(VIOC_DISP *pDISP, unsigned int *nWidth, unsigned int *nHeight);
extern void VIOC_DISP_SetAlign(VIOC_DISP *pDISP, unsigned int align);
extern void VIOC_DISP_GetAlign(VIOC_DISP *pDISP, unsigned int *align);
extern void VIOC_DISP_SetSwapbf(VIOC_DISP *pDISP, unsigned int swapbf);
extern void VIOC_DISP_GetAlign(VIOC_DISP *pDISP, unsigned int *swapbf);

extern void VIOC_DISP_SetBGColor(VIOC_DISP *pDISP, unsigned int BG0, unsigned int BG1, unsigned int BG2);
extern void VIOC_DISP_SetPosition(VIOC_DISP *pDISP, unsigned int startX, unsigned int startY );
extern void VIOC_DISP_GetPosition(VIOC_DISP *pDISP, unsigned int *startX, unsigned int *startY );
extern void VIOC_DISP_SetColorEnhancement(VIOC_DISP *pDISP, signed char contrast, signed char brightness, signed char hue );
extern void VIOC_DISP_GetColorEnhancement(VIOC_DISP *pDISP, signed char *contrast, signed char *brightness, signed char *hue );
extern void VIOC_DISP_SetClippingEnable(VIOC_DISP *pDISP, unsigned int enable);
extern void VIOC_DISP_GetClippingEnable(VIOC_DISP *pDISP, unsigned int *enable);
extern void VIOC_DISP_SetClipping(VIOC_DISP *pDISP, unsigned int uiUpperLimitY, unsigned int uiLowerLimitY, unsigned int uiUpperLimitUV, unsigned int uiLowerLimitUV);
extern void VIOC_DISP_GetClipping(VIOC_DISP *pDISP, unsigned int *uiUpperLimitY, unsigned int *uiLowerLimitY, unsigned int *uiUpperLimitUV, unsigned int *uiLowerLimitUV);
extern void VIOC_DISP_SetDither(VIOC_DISP *pDISP, unsigned int ditherEn, unsigned int ditherSel, unsigned char mat[4][4]);
extern void VIOC_DISP_SetTimingParam (VIOC_DISP *pDISP, stLTIMING *pTimeParam);
//extern void VIOC_DISP_SetPixelClockDiv(VIOC_DISP *pDISP, stLTIMING *pTimeParam);
extern void VIOC_DISP_SetPixelClockDiv(VIOC_DISP *pDISP, unsigned int div);
extern void VIOC_DISP_TurnOn (VIOC_DISP *pDISP);
extern void VIOC_DISP_TurnOff (VIOC_DISP *pDISP);
extern unsigned int  VIOC_DISP_Get_TurnOnOff(VIOC_DISP *pDISP);
extern int VIOC_DISP_Wait_DisplayDone(VIOC_DISP *pDISP);
extern int VIOC_DISP_sleep_DisplayDone(VIOC_DISP *pDISP);
extern void VIOC_DISP_SetControl(VIOC_DISP *pDISP, stLCDCPARAM *pLcdParam);
extern void VIOC_DISP_SWReset( unsigned int DISP );
extern void VIOC_DISP_DisplayOnOff( unsigned int onOff );
extern void VIOC_DISP_SetIreqMask(VIOC_DISP * pDISP, unsigned int mask, unsigned int set);
extern void VIOC_DISP_SetStatus(VIOC_DISP * pDISP, unsigned int set);
extern void VIOC_DISP_GetStatus(VIOC_DISP * pDISP, unsigned int *status);
extern void VIOC_DISP_EmergencyFlagDisable(VIOC_DISP *pDISP);
extern void VIOC_DISP_EmergencyFlag_SetEofm(VIOC_DISP *pDISP, unsigned int eofm);
extern void VIOC_DISP_EmergencyFlag_SetHdmiVs(VIOC_DISP *pDISP, unsigned int hdmivs);
extern DDICONFIG* VIOC_DDICONFIG_GetAddress(void);
extern VIOC_DISP* VIOC_DISP_GetAddress(unsigned int Num);
#endif
