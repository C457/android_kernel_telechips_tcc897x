/*
 * linux/arch/arm/mach-tcc893x/include/mach/vioc_lut.h
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

#include <mach/reg_physical.h>

#ifndef __VIOC_LUT_H__
#define	__VIOC_LUT_H__
#define	VIOC_LUT_DEV0			0
#define	VIOC_LUT_DEV1			1
#define	VIOC_LUT_DEV2			2
#define	VIOC_LUT_COMP0		3
#define	VIOC_LUT_COMP1		4
#define	VIOC_LUT_COMP2		5
#define	VIOC_LUT_COMP3		6

#define LUT_COLOR_DEPTH		8
#define LUT_TABLE_SIZE			(1 << LUT_COLOR_DEPTH)

typedef	struct {
	unsigned				SELECT		:  4;		// 
	unsigned 							: 28;
}	VIOC_LUT_SEL;

typedef	union {
	unsigned	long			nREG;
	VIOC_LUT_SEL		bREG;
}	VIOC_LUT_SEL_u;

typedef	struct {
	unsigned				SEL	    	:  8;		// no meaning for device case 
	unsigned 						: 23;
	unsigned				EN	     	:  1;
}	VIOC_LUT_CONFIG;

typedef	union {
	unsigned	long			nREG;
	VIOC_LUT_CONFIG		bREG;
}	VIOC_LUT_CONFIG_u;

typedef	struct _VIOC_LUT
{
	volatile VIOC_LUT_SEL_u		uSEL;					// 0x00
	volatile VIOC_LUT_CONFIG_u	uDEV0CFG;				// 0x04
	volatile VIOC_LUT_CONFIG_u	uDEV1CFG;				// 0x08
	volatile VIOC_LUT_CONFIG_u	uDEV2CFG;				// 0x0C
	volatile VIOC_LUT_CONFIG_u	uCOMP0CFG;				// 0x10
	volatile VIOC_LUT_CONFIG_u	uCOMP1CFG;				// 0x14
	volatile VIOC_LUT_CONFIG_u	uCOMP2CFG;				// 0x18
	volatile VIOC_LUT_CONFIG_u	uCOMP3CFG;				// 0x1C
}	VIOC_LUT,*PVIOC_LUT;

typedef	struct {
	unsigned	int nEntry[256];
}	VIOC_LUT_TABLE;


extern void tcc_set_lut_table(unsigned int lut_n, unsigned int *table);
extern void tcc_set_lut_plugin(unsigned int lut_n, unsigned int plugComp);
extern int tcc_get_lut_plugin (unsigned int lut_n);
extern void tcc_set_lut_enable(unsigned int lut_n, unsigned int enable);
extern int tcc_get_lut_enable(unsigned int lut_n);

#endif
