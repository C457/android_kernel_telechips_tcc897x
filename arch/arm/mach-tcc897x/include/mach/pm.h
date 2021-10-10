/****************************************************************************
 * arch/arm/mach-tcc893x/tcc_sram.c
 * Copyright (C) 2014 Telechips Inc.
 *
 * This program is free software; you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation;
 * either version 2 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA 02111-1307 USA
 ****************************************************************************/

#ifndef __TCC_PM_H__
#define __TCC_PM_H__

#define pm_writel	__raw_writel
#define pm_readl	__raw_readl

extern void __cpu_early_init(void);

typedef void (*FuncPtr)(void);
typedef void (*FuncPtr1)(unsigned int arg0);
typedef unsigned (*FuncPtr1_1)(unsigned int arg0);

#define PM_PIC_SIZE	0x110
#define PM_VIC_SIZE	0x060
#define PM_TIMER_SIZE	0x0A0

enum pm_uart {
	PBR = 0,
	THR = 0,
	DLL = 0,
	IER = 1,
	DLM = 1,
	IIR = 2,
	FCR = 2,
	LCR,
	MCR,
	LSR,
	MSR,
	SCR,
	AFT,
	UCR,
	PM_UART_MAX
};

enum pm_nfc {
	PM_NFC_CACYC = 0,
	PM_NFC_WRCYC,
	PM_NFC_RECYC,
	PM_NFC_CTRL,
	PM_NFC_IRQCFG,
	PM_NFC_RFWBASE,
	PM_NFC_MAX
};

typedef struct _TCC_REG_{
	unsigned	pic[PM_PIC_SIZE/4];
	unsigned	vic[PM_VIC_SIZE/4];
	unsigned	timer[PM_TIMER_SIZE/4];
#if defined(CONFIG_PM_CONSOLE_NOT_SUSPEND)
	unsigned	uart[PM_UART_MAX];
	unsigned	uartcfg[2];
#endif
	unsigned	nfc[PM_NFC_MAX];
	unsigned	TPIDRPRW;	/* cp15: Software Thread ID (cpu offset) */
} TCC_REG, *PTCC_REG;


inline static void tcc_pm_nop_delay(unsigned x)
{
	unsigned int cnt;
	for(cnt=0 ; cnt<x ; cnt++) __asm__ __volatile__ ("nop\n");
}

#endif	/*__TCC_PM_H__*/
