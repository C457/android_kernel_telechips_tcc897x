/*
 * IR driver for remote controller : tca_remocon.c
 *
 * Copyright (C) 2010 Telechips, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include "tcc_remocon.h"
#include <linux/clk.h>
#include <linux/err.h>

extern unsigned long rmt_clk_rate;

//======================================================
// REMOCON initialize
//======================================================
void    RemoconCommandOpen (void)
{
	PREMOTECON      pRcu = (volatile PREMOTECON)tcc_p2v(TCC_PA_REMOTECTRL);

	BITCLR(pRcu->CMD.nREG, Hw0); //FF
	BITSET(pRcu->CMD.nREG, Hw1|Hw2); //EN|CLEAR

  #if defined(CONFIG_PBUS_DIVIDE_CLOCK_WITH_XTIN) || defined(CONFIG_XTIN_CLOCK)
	BITSET(pRcu->CMD.nREG, (0x1E<<3)); //TH
  #else
	BITSET(pRcu->CMD.nREG, (IR_FIFO_READ_COUNT<<3)); //TH
  #endif

	BITSET(pRcu->CMD.nREG, Hw12); //WS
	BITSET(pRcu->CMD.nREG, Hw13); //DEN
	BITSET(pRcu->CMD.nREG, Hw14); //FWEN
}

//======================================================
// configure address
//======================================================
void    RemoconConfigure (void)
{
	PREMOTECON      pRcu = (volatile PREMOTECON)tcc_p2v(TCC_PA_REMOTECTRL);

	BITSET(pRcu->INPOL.nREG, Hw0); //INV

#if defined(CONFIG_ARCH_TCC896X) || defined(CONFIG_ARCH_TCC897X) || defined(CONFIG_ARCH_TCC898X)
	BITSET(pRcu->INPOL.nREG, Hw14); //ES
#endif
#if defined(CONFIG_PBUS_DIVIDE_CLOCK_WITH_XTIN) || defined(CONFIG_PBUS_DIVIDE_CLOCK_WITHOUT_XTIN)
	BITCLR(pRcu->INPOL.nREG, Hw8); //SCLK
#else
	BITSET(pRcu->INPOL.nREG, Hw8); //SCLK
#endif
  
	BITSET(pRcu->INPOL.nREG, Hw9); //FIL
	BITCLR(pRcu->INPOL.nREG, Hw11); //FT
	BITSET(pRcu->INPOL.nREG, Hw10); //FT
	BITCLR(pRcu->INPOL.nREG, Hw12); //FCLK
	BITSET(pRcu->INPOL.nREG, Hw13); //CXTIN
	BITSET(pRcu->BDD.nREG, Hw1); //BDDR
	BITSET(pRcu->BDD.nREG, Hw0); //BE
	BITSET(pRcu->BDD.nREG, Hw5); //BDXD
  #if defined(CONFIG_ARCH_TCC896X) || defined(CONFIG_ARCH_TCC897X) || defined(CONFIG_ARCH_TCC898X)
	BITSET(pRcu->BDD.nREG, Hw26); //UXIN
  #endif
#if defined(CONFIG_PBUS_DIVIDE_CLOCK_WITH_XTIN) || defined(CONFIG_PBUS_DIVIDE_CLOCK_WITHOUT_XTIN)
	BITCLR(pRcu->BDD.nREG, Hw23); //BDSC
#else
	BITSET(pRcu->BDD.nREG, Hw23); //BDSC
#endif
  #if defined(CONFIG_ARCH_TCC896X) || defined(CONFIG_ARCH_TCC897X) || defined(CONFIG_ARCH_TCC898X)
	BITCLR(pRcu->BDD.nREG, Hw25); //MISM
  #endif
	BITSET(pRcu->BDD.nREG, Hw24); //BDDC

//	PGPIO pGpioG = (volatile PGPIO)tcc_p2v(HwGPIOG_BASE);
//	BITSET(pGpioG->GPGIS.nREG, Hw17);
}

//======================================================
// REMOCON STATUS : irq enacle and fifo overflow (active low)
//======================================================
void    RemoconStatus (void)
{
	PREMOTECON      pRcu = (volatile PREMOTECON)tcc_p2v(TCC_PA_REMOTECTRL);

#if defined(CONFIG_PBUS_DIVIDE_CLOCK_WITH_XTIN) || defined(CONFIG_XTIN_CLOCK)
	BITCLR(pRcu->STA.nREG, Hw12-Hw0); //ICF
#else
	BITCLR(pRcu->STA.nREG, Hw12-Hw0); //ICF
#endif
	BITSET(pRcu->STA.nREG, Hw12); //OF
}

//======================================================
// REMOCON DIVIDE enable & ir clk & end count setting
// (end count use remocon clk)
//======================================================
void    RemoconDivide (int state)
{
	PREMOTECON      pRcu = (volatile PREMOTECON)tcc_p2v(TCC_PA_REMOTECTRL);
	unsigned int	uiclock;
	unsigned int	uidiv = 0;
	unsigned long remote_clock_rate;

	remote_clock_rate = rmt_clk_rate;
	uiclock           = state == 1? 24*1000*1000:(remote_clock_rate);
	uidiv             = uiclock/32768;

	//printk("##[%s] , rmt_clk_rate : %d ,remote clock_rate : %d \n", __func__ , rmt_clk_rate, remote_clock_rate);
	//printk("+++++++++++++++++++++++++++++++++++++[%d][%d]\n", uiclock, uidiv);

	BITSET(pRcu->CLKDIV.nREG, 0xC8); //END_CNT
	BITSET(pRcu->CLKDIV.nREG, uidiv<<14); //CLK_DIV
	BITSET(pRcu->CLKDIV.nREG, Hw14); //CLK_DIV
}

//======================================================
// REMOCON diable 
//======================================================
void    DisableRemocon (void)
{
	PREMOTECON      pRcu = (volatile PREMOTECON)tcc_p2v(TCC_PA_REMOTECTRL);

	BITSET(pRcu->CMD.nREG, Hw0); //FF
	BITCLR(pRcu->CMD.nREG, Hw1); //EN
	BITCLR(pRcu->CMD.nREG, Hw2); //CLEAR
	BITCLR(pRcu->CMD.nREG, Hw13); //DEN
	BITCLR(pRcu->CMD.nREG, Hw14); //FWEN
	
	BITCLR(pRcu->BDD.nREG, Hw1); //BDDR
}

//======================================================
// REMOCON functions
//======================================================
void    RemoconInit (int state)
{
	RemoconDivide(state);
	RemoconIntClear();
}

void    RemoconIntClear ()
{
	PREMOTECON      pRcu = (volatile PREMOTECON)tcc_p2v(TCC_PA_REMOTECTRL);

	BITSET(pRcu->INPOL.nREG, Hw0); //INV
#if defined(CONFIG_PBUS_DIVIDE_CLOCK_WITH_XTIN) || defined(CONFIG_PBUS_DIVIDE_CLOCK_WITHOUT_XTIN)
	BITCLR(pRcu->INPOL.nREG, Hw8); //SCLK
#else
	BITSET(pRcu->INPOL.nREG, Hw8); //SCLK
#endif
	BITSET(pRcu->INPOL.nREG, Hw9); //FIL
	BITCLR(pRcu->INPOL.nREG, Hw11); //FT
	BITSET(pRcu->INPOL.nREG, Hw10); //FT
	BITCLR(pRcu->INPOL.nREG, Hw12); //FCLK
	BITSET(pRcu->INPOL.nREG, Hw13); //CXTIN

	BITCLR(pRcu->CMD.nREG, Hw13); //DEN
	BITCLR(pRcu->CMD.nREG, Hw2); //CLEAR
	BITCLR(pRcu->BDD.nREG, Hw1); //BDDR
	BITSET(pRcu->CMD.nREG, Hw0); //FF

	//udelay(120);

#if defined(CONFIG_PBUS_DIVIDE_CLOCK_WITH_XTIN) || defined(CONFIG_PBUS_DIVIDE_CLOCK_WITHOUT_XTIN)
#else
	BITCLR(pRcu->STA.nREG, Hw12-Hw0); //ICF
#endif
	BITSET(pRcu->CMD.nREG, Hw2); //CLEAR
	BITSET(pRcu->BDD.nREG, Hw1); //BDDR
	BITCLR(pRcu->CMD.nREG, Hw0); //FF
	BITSET(pRcu->CMD.nREG, Hw13); //DEN
#if defined(CONFIG_PBUS_DIVIDE_CLOCK_WITH_XTIN) || defined(CONFIG_XTIN_CLOCK)
	BITSET(pRcu->STA.nREG, 0xFFF); //ICF
#else
	BITSET(pRcu->STA.nREG, 0x1FE); //ICF
#endif

	BITSET(pRcu->BDD.nREG, Hw0); //BE
	BITSET(pRcu->BDD.nREG, Hw5); //BDXD
  
#if defined(CONFIG_PBUS_DIVIDE_CLOCK_WITH_XTIN) || defined(CONFIG_PBUS_DIVIDE_CLOCK_WITHOUT_XTIN)
	BITCLR(pRcu->BDD.nREG, Hw23); //BDSC
#else
	BITSET(pRcu->BDD.nREG, Hw23); //BDSC
#endif
	BITSET(pRcu->BDD.nREG, Hw24); //BDDC
#if defined(CONFIG_PBUS_DIVIDE_CLOCK_WITHOUT_XTIN)
  #if defined(CONFIG_ARCH_TCC893X)
	pRcu->PBD00.nREG = 0x00000004;
	pRcu->PBD01.nREG = 0x00000000;
  #endif
#else
#endif
}
