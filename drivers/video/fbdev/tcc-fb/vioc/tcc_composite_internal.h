/****************************************************************************
FileName    : kernel/drivers/video/tcc/vioc/tcc_composite_internal.h
Description : 

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
#ifndef __TCC_COMPOSITE_INTERNAL_H__
#define __TCC_COMPOSITE_INTERNAL_H__

#ifdef CONFIG_ARCH_TCC897X
#include <mach/reg_physical.h>
#else
#include <video/tcc/tcc_video_regs.h>
#endif

typedef struct _NTSCPAL {
	volatile TCC_DEF32BIT_TYPE	STATA;			// 0x000  R    -			Status register A
	volatile TCC_DEF32BIT_TYPE	ECMDA;			// 0x004  R/W  0x00000000	Encoder control register A
	volatile TCC_DEF32BIT_TYPE	ECMDB;			// 0x008  R/W  0x00000000	Encoder control register B
	volatile TCC_DEF32BIT_TYPE	GLK;			// 0x00C  R/W  0x00000000	Chroma gunlock control register
	volatile TCC_DEF32BIT_TYPE	SCH;			// 0x010  R/W  0x00000000	SCH phase control
	volatile TCC_DEF32BIT_TYPE	HUE;			// 0x014  R/W  0x00000000	HUE phase control
	volatile TCC_DEF32BIT_TYPE	SAT;			// 0x018  R/W  0x00000000	Chroma saturation control
	volatile TCC_DEF32BIT_TYPE	CONT;			// 0x01C  R/W  0x00000000	Luma gain control
	volatile TCC_DEF32BIT_TYPE	BRIGHT;			// 0x020  R/W  0x00000000	Luma offset control
	volatile TCC_DEF32BIT_TYPE	FSC_ADJM;		// 0x024  R/W  0x00000000	Color burst frequency adjustment A
	volatile TCC_DEF32BIT_TYPE	FSC_ADJL;		// 0x028  R/W  0x00000000	Color burst frequency adjustment B
	volatile TCC_DEF32BIT_TYPE	ECMDC;			// 0x02C  R/W  0x00000000	Encoder control register C
	unsigned :32; unsigned :32; unsigned :32; unsigned :32;
	volatile TCC_DEF32BIT_TYPE	DACSEL;			// 0x040  R/W  0x00000000	DAC output select
	unsigned :32; unsigned :32; unsigned :32;
	volatile TCC_DEF32BIT_TYPE	DACPD;			// 0x050  R/W  0x00000000	DAC power down /enable control
	unsigned :32; unsigned :32; unsigned :32;
	unsigned :32; unsigned :32; unsigned :32; unsigned :32;
	unsigned :32; unsigned :32; unsigned :32; unsigned :32;
	volatile TCC_DEF32BIT_TYPE	ICNTL;			// 0x080  R/W  0x00000000	Timing configuration register
	volatile TCC_DEF32BIT_TYPE	HVOFFST;		// 0x084  R/W  0x00000000	Horizontal active video location A
	volatile TCC_DEF32BIT_TYPE	HOFFST;			// 0x088  R/W  0x00000000	Horizontal active video location B
	volatile TCC_DEF32BIT_TYPE	VOFFST;			// 0x08C  R/W  0x00000000	Channel P vertical active location
	volatile TCC_DEF32BIT_TYPE	HSVSO;			// 0x090  R/W  0x00000000	Interlaced horizontal sync trailing location A
	volatile TCC_DEF32BIT_TYPE	HSOE;			// 0x094  R/W  0x00000000	Interlaced horizontal sync trailing location B
	volatile TCC_DEF32BIT_TYPE	HSOB;			// 0x098  R/W  0x00000000	Interlaced horizontal sync beginning location
	volatile TCC_DEF32BIT_TYPE	VSOB;			// 0x09C  R/W  0x00000000	Vertical sync beginning location
	volatile TCC_DEF32BIT_TYPE	VSOE;			// 0x0A0  R/W  0x00000000	Vertical sync ending location
} NTSCPAL, *PNTSCPAL;

typedef struct _NTSCPAL_ENCODER_CTRL {
	volatile TCC_DEF32BIT_TYPE	VENCON;			// 0x800  R/W  0x00000000	VENC control register
	volatile TCC_DEF32BIT_TYPE	VENCIF;			// 0x804  R/W  0x00000000	VENC interface register
} NTSCPAL_ENCODER_CTRL, *PNTSCPAL_ENCODER_CTRL;

extern void internal_tve_clock_onoff(unsigned int onoff);
extern void internal_tve_enable(unsigned int type, unsigned int onoff);
extern void internal_tve_init(void);

#endif //__TCC_COMPOSITE_INTERNAL_H__
