/*
 * linux/arch/arm/mach-tcc893x/include/mach/vioc_fifo.h
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
#ifndef _VIOC_FIFO_H_
#define	_VIOC_FIFO_H_

#include <mach/reg_physical.h>

/************************************************************************
*   3.6.13   Asynchronous Frame FIFO 			(Base Addr = 0x72003900)
*************************************************************************/

typedef	struct {
	unsigned				WEN			:  1;
	unsigned				REN0		:  1;
	unsigned				REN1		:  1;
	unsigned				REN2		:  1;
	unsigned				RMODE_TIME	:  1;
	unsigned 				   			:  3;
	unsigned				NENTRY    	:  6;
	unsigned				WMODE_TIME	:  1;
	unsigned				WMODE_FULL	:  1;
	unsigned				EFULL    	:  2;
	unsigned				EEMPTY   	:  2;
	unsigned 				   			:  4;
	unsigned				IE_FULL		:  1;
	unsigned				IE_EFULL	:  1;
	unsigned				IE_EMPTY	:  1;
	unsigned				IE_EEMPTY	:  1;
	unsigned				IE_WEOF		:  1;
	unsigned				IE_REOF0	:  1;
	unsigned				IE_REOF1	:  1;
	unsigned				IE_REOF2	:  1;
}	VIOC_FIFO_CTRL0;

typedef	union {
	unsigned	long			nREG;
	VIOC_FIFO_CTRL0		bREG;
}	VIOC_FIFO_CTRL0_u;

typedef	struct {
	unsigned				WDMA		:  4;
	unsigned				RDMA0		:  4;
	unsigned				RDMA1		:  4;
	unsigned				RDMA2		:  4;
	unsigned				FAST_RDMA	:  2;
	unsigned 				   			: 14;
}	VIOC_FIFO_CTRL1;

typedef	union {
	unsigned	long			nREG;
	VIOC_FIFO_CTRL1		bREG;
}	VIOC_FIFO_CTRL1_u;

typedef	struct {
	unsigned				IS_FULL		:  1;
	unsigned				IS_EFULL	:  1;
	unsigned				IS_EMPTY	:  1;
	unsigned				IS_EEMPTY	:  1;
	unsigned				IS_WEOF		:  1;
	unsigned				IS_REOF0	:  1;
	unsigned				IS_REOF1	:  1;
	unsigned				IS_REOF2	:  1;
	unsigned 								: 24;
}	VIOC_FIFO_IRQSTAT;

typedef	union {
	unsigned	long			nREG;
	VIOC_FIFO_IRQSTAT	bREG;
}	VIOC_FIFO_IRQSTAT_u;

typedef	struct {
	unsigned				FULL		:  1;
	unsigned				EFULL		:  1;
	unsigned				EMPTY		:  1;
	unsigned				EEMPTY		:  1;
	unsigned				WEOF		:  1;
	unsigned				REOF0		:  1;
	unsigned				REOF1		:  1;
	unsigned				REOF2		:  1;
	unsigned 							: 16;
	unsigned				FILLED		:  8;
}	VIOC_FIFO_FIFOSTAT;

typedef	union {
	unsigned	long		nREG;
	VIOC_FIFO_FIFOSTAT	bREG;
}	VIOC_FIFO_FIFOSTAT_u;

typedef	struct	_VIOC_FIFO
{
	volatile VIOC_FIFO_CTRL0_u			uCH0_CTRL0;				// 0x000
	volatile VIOC_FIFO_CTRL1_u			uCH0_CTRL1;				// 0x004
	volatile VIOC_FIFO_IRQSTAT_u		uCH0_IRQSTAT;			// 0x008
	volatile VIOC_FIFO_FIFOSTAT_u		uCH0_FIFOSTAT;			// 0x00C
	volatile VIOC_FIFO_CTRL0_u			uCH1_CTRL0;				// 0x010
	volatile VIOC_FIFO_CTRL1_u			uCH1_CTRL1;				// 0x014
	volatile VIOC_FIFO_IRQSTAT_u		uCH1_IRQSTAT;			// 0x018
	volatile VIOC_FIFO_FIFOSTAT_u		uCH1_FIFOSTAT;			// 0x01C
	unsigned	int				nCH0_BASE[16];		// 0x020~0x05C
	unsigned	int				nCH1_BASE[16];		// 0x060~0x09C
}	VIOC_FIFO,*PVIOC_FIFO;

extern void VIOC_ASYNC_FIFO_ConfigDMA(VIOC_FIFO * pFIFO, unsigned int nWDMA, unsigned int nRDMA0, unsigned int nRDMA1, unsigned int nRDMA2);
extern void VIOC_ASYNC_FIFO_ConfigEntry(VIOC_FIFO * pFIFO, unsigned int * buf);
extern void VIOC_ASYNC_FIFO_SetEnable(VIOC_FIFO * pFIFO, unsigned int nWDMA, unsigned int nRDMA0, unsigned int nRDMA1, unsigned int nRDMA2);

#endif //__VIOC_FIFO_H__

