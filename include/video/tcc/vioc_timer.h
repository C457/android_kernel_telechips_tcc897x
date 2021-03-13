/*
 * linux/arch/arm/mach-tcc897x/include/mach/vioc_timer.h
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
#ifndef __VIOC_TIMER_H__
#define	__VIOC_TIMER_H__

/************************************************************************
*  Timer						(Base Addr = 0x1200C000)
*************************************************************************/
typedef	struct {
	unsigned				USECDIV		:  8;		// to make usec pulse from TCLK (freq(TCLK) / (USECDIV+1))
	unsigned				UNITDIV		:  8;		// to make 100usec pulse from usec (normally 99)
	unsigned 							: 15;
	unsigned				EN			:  1;
}	VIOC_TIMER_USEC;

typedef	union {
	unsigned	long			nREG;
	VIOC_TIMER_USEC		bREG;
}	VIOC_TIMER_USEC_u;

typedef	struct {
	unsigned				COUNTER    	: 16;
	unsigned 							: 15;
	unsigned				EN	     	:  1;
}	VIOC_TIMER_CORE;

typedef	union {
	unsigned	long			nREG;
	VIOC_TIMER_CORE		bREG;
}	VIOC_TIMER_CORE_u;

typedef	struct {
	unsigned				TIME    	: 16;
	unsigned 							: 15;
	unsigned				EN	     	:  1;
}	VIOC_TIMER_TIREQ;

typedef	union {
	unsigned	long			nREG;
	VIOC_TIMER_TIREQ	bREG;
}	VIOC_TIMER_TIREQ_u;

typedef	struct {
	unsigned				TIMER0    	:  1;
	unsigned				TIMER1    	:  1;
	unsigned				TIREQ0    	:  1;
	unsigned				TIREQ1    	:  1;
	unsigned 							: 28;
}	VIOC_TIMER_IREQ;

typedef	union {
	unsigned	long			nREG;
	VIOC_TIMER_IREQ		bREG;
}	VIOC_TIMER_IREQ_u;

typedef	struct	_VIOC_TIMER
{
	volatile VIOC_TIMER_USEC_u	uUSEC;				// 				0x00		R/W	0x8000630B	Micro second(usec) Register
	unsigned	int					nCURTIME;			// 1	: 100us unit 	0x04  R					Current Time Register
	volatile VIOC_TIMER_CORE_u	uTIMER0;				// 2				0x08		R/W	0x00000000	Timer core 0 Register
	volatile VIOC_TIMER_CORE_u	uTIMER1;				// 3				0x0C	R/W	0x00000000	Timer core 1 Register
	volatile VIOC_TIMER_TIREQ_u	uTIREQ0;				// 4				0x10		R/W	0x00000000	Timer Interrupt Contrl 0 Register
	volatile VIOC_TIMER_TIREQ_u	uTIREQ1;				// 5				0x14		R/W	0x00000000	Timer Interrupt Contrl 1 Register
	volatile VIOC_TIMER_IREQ_u		uIRQMASK;			// 6				0x18						Interrupt Mask Register
	volatile VIOC_TIMER_IREQ_u		uIRQSTAT;			// 7				0x1C					Interrupt Status Register
}	VIOC_TIMER,*PVIOC_TIMER;

#endif

