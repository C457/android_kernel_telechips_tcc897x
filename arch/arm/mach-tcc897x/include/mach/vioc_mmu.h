/*
 * linux/arch/arm/mach-tcc897x/include/mach/vioc_mmu.h
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
#ifndef __VIOC_MMU_H__
#define	__VIOC_MMU_H__


/************************************************************************
*   MMU						(Base Addr = 0x1200F000)
*************************************************************************/
typedef	struct {
	unsigned				EN			:  1;		// MMU Enable
	unsigned 							: 31;
}	VIOC_MMU_CTRL;

typedef	union {
	unsigned	long			nREG;
	VIOC_MMU_CTRL		bREG;
}	VIOC_MMU_CTRL_u;

typedef	struct {
	unsigned				SIZE    	:  3;		// 0, 1, 2, 3, 4, 5
	unsigned				BASE    	: 29;
}	VIOC_MMU_TTB;

typedef	union {
	unsigned	long			nREG;
	VIOC_MMU_TTB		bREG;
}	VIOC_MMU_TTB_u;

typedef struct{
	unsigned				FAULT 	:  1;
	unsigned 							: 31;
}	VIOC_MMU_IRQSTAT;

typedef union{
	unsigned	long			nREG;
	VIOC_MMU_IRQSTAT		bREG;
}	VIOC_MMU_IRQSTAT_u;

typedef	struct _VIOC_MMU
{
	volatile VIOC_MMU_CTRL_u			uCTRL;			// 0x00	R/W 0x00000000 MMU Control Register
	volatile VIOC_MMU_TTB_u			uTTB;			// 0x04	R/W 0x00000000 MMU Table Control Register
	unsigned	int						nFAULTADDR;		// 0x08  R			     Status of Fault Address
	volatile VIOC_MMU_IRQSTAT_u		nIRQSTAT;		// 0x0C  R			     Status of Fault
	volatile VIOC_MMU_IRQSTAT_u		nIRQMASK;		// 0x10	R/W	0x00000000 Time Interrupt Control 0 Register
}	VIOC_MMU,*PVIOC_MMU;

#endif

