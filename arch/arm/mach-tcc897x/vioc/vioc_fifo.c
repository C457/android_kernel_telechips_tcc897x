/*
 * linux/arch/arm/mach-tcc893x/vioc_fifo.c
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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/of_address.h>
#include <mach/bsp.h>
#include <mach/vioc_fifo.h>

#if 0
#define dprintk(msg...)	 { printk( "Tcc897x_fifo: " msg); }
#else
#define dprintk(msg...)	 
#endif

void VIOC_ASYNC_FIFO_ConfigDMA(VIOC_FIFO * pFIFO, unsigned int nWDMA, unsigned int nRDMA0, unsigned int nRDMA1, unsigned int nRDMA2) {
	dprintk("pFIFO = 0x%x, WDMA = %d, RDMA0 = %d, RDMA1 = %d, RDMA2 = %d\n",	\
		pFIFO, nWDMA, nRDMA0, nRDMA1, nRDMA2);
	BITCSET(pFIFO->uCH0_CTRL1.nREG, 0x000FFFF, ((nRDMA2<<12)|(nRDMA1<<8)|(nRDMA0<<4)|(nWDMA<<0)));
}

void VIOC_ASYNC_FIFO_ConfigEntry(VIOC_FIFO * pFIFO, unsigned int * buf) {
	unsigned int EEMPTY	= 0; // emergency empty
	unsigned int EFULL	= 0; // emergency full
	unsigned int WMT 	= 0; // wdma mode - time
	unsigned int NENTRY 	= 4; // frame memory number  ->  max. frame count is 4.
	unsigned int RMT 	= 0; // rdma mode - time
	unsigned int idxBuf;

	for(idxBuf=0; idxBuf < NENTRY; idxBuf++) {
		dprintk("buf[%d] = 0x%08x\n", idxBuf, buf[idxBuf]);
		BITCSET(pFIFO->nCH0_BASE[idxBuf], 0xFFFFFFFF, buf[idxBuf]);
	}
	BITCSET(pFIFO->uCH0_CTRL0.nREG, 0x000FFFFF, ((EEMPTY<<18)|(EFULL<<16)|(WMT<<14)|(NENTRY<<8)|(RMT<<4)));
}

void VIOC_ASYNC_FIFO_SetEnable(VIOC_FIFO * pFIFO, unsigned int nWDMA, unsigned int nRDMA0, unsigned int nRDMA1, unsigned int nRDMA2) {
	dprintk("pFIFO = 0x%x, WDMA = %d, RDMA0 = %d, RDMA1 = %d, RDMA2 = %d\n",	\
		pFIFO, nWDMA, nRDMA0, nRDMA1, nRDMA2);
	BITCSET(pFIFO->uCH0_CTRL0.nREG, 0x0000000F, (((nRDMA2&1)<<3)|((nRDMA1&1)<<2)|((nRDMA0&1)<<1)|(nWDMA&1)));
}

static VIOC_FIFO * pFIFO_Addr;

VIOC_FIFO * VIOC_FIFO_GetAddress(void) {
	if(pFIFO_Addr == NULL)
		pr_err("%s - pDDICONFIG = %p\n", __func__, pFIFO_Addr);

	return pFIFO_Addr;
}

static int __init vioc_fifo_init(void) {
	struct device_node * ViocFIFO_np = of_find_compatible_node(NULL, NULL, "telechips,vioc_fifo");
	
	if(ViocFIFO_np == NULL) {
		pr_err("Can't find telechips,vioc_fifo.\n");
	} else {
		pFIFO_Addr = of_iomap(ViocFIFO_np, 0);
		pr_info("%s - Vioc FIFO = %p \n", __func__, pFIFO_Addr);
	}
	return 0;
}
arch_initcall(vioc_fifo_init);

