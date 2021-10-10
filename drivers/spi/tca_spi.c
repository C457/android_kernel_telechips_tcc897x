/* 
 * Author:  <linux@telechips.com>
 * Created: 10th Jun, 2008 
 * Description: Telechips Linux SPI Driver
 *
 * Copyright (c) Telechips, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/gpio.h>
#include "tca_spi.h"
//#include <mach/bsp.h>
//#include <mach/tca_spi.h>
//#include <mach/io.h>
//#include <mach/iomap.h>
//#include <mach/gpio.h>
#include <asm/io.h>
#include <linux/delay.h>

void tca_spi_print_gdma_config(struct tca_spi_gdma_config gdma)
{
	printk("\x1b[1;33m ## [%s:%d] ##\x1b[0m\n", __func__, __LINE__);
	printk("\x1b[1;33m gpsb_use_gdma : %d\x1b[0m\n", gdma.gpsb_use_gdma);
	printk("\x1b[1;33m reg_base : %p\x1b[0m\n", (void *)gdma.reg_base);
	printk("\x1b[1;33m gdma_irq : %d\x1b[0m\n", gdma.gdma_irq);
	printk("\x1b[1;33m tx_reg : %p\x1b[0m\n", (void *)gdma.tx_reg);
	printk("\x1b[1;33m tx_ctrl : %d\x1b[0m\n", gdma.tx_ctrl);
	printk("\x1b[1;33m tx_ch : %d\x1b[0m\n", gdma.tx_ch);
	printk("\x1b[1;33m tx_ext : %d\x1b[0m\n", gdma.tx_ext);
	printk("\x1b[1;33m rx_reg : %p\x1b[0m\n", (void *)gdma.rx_reg);
	printk("\x1b[1;33m rx_ctrl : %d\x1b[0m\n", gdma.rx_ctrl);
	printk("\x1b[1;33m rx_ch : %d\x1b[0m\n", gdma.rx_ch);
	printk("\x1b[1;33m rx_ext : %d\x1b[0m\n", gdma.rx_ext);
}

int tca_spi_gdma_intr_ch(struct tca_spi_handle *tspi)
{
	unsigned int gdma_chconfig;
	unsigned int intr_mask;
	int i;

	gdma_chconfig = tca_spi_readl(tspi->gdma_config.reg_base + TCC_GPSB_GDMA_CHCONFIG);
	intr_mask = 0x0 | (1 << TCC_GPSB_GDMA_CH_MSK_OFFSET);

	for(i=0;i<TCC_GPSB_GDMA_CH_NUM;i++)
	{
		if(gdma_chconfig & intr_mask)
		{
			//printk("\x1b[1;33m[%s:%d] CH # %d\x1b[0m\n", __func__, __LINE__,i);
			return i;
		}

		intr_mask = intr_mask << 1;
		//printk("\x1b[1;33m[%s:%d] intr mask : 0x%08X\x1b[0m\n", __func__, __LINE__,intr_mask);
	}
	//printk("\x1b[1;33m[%s:%d] Fail to find interrupted channel (CHCONFIG : %08X)\x1b[0m\n", __func__, __LINE__,gdma_chconfig);
	return -1;
}

int tca_spi_is_use_gdma(tca_spi_handle_t *h)
{
	return h->gdma_config.gpsb_use_gdma;
}

#ifndef TCC_DMA_ENGINE
static int tca_spi_isenable_gdma(struct tca_spi_handle *h)
{
	return (tca_spi_readl(h->gdma_config.rx_reg+TCC_GPSB_GDMA_CHCTRL) & Hw0) ? 1 : 0;
}

static int tca_spi_gdmastop(struct tca_spi_handle *h)
{
	void __iomem *gdma_tx_regs, *gdma_rx_regs;

	gdma_tx_regs = h->gdma_config.tx_reg;
	gdma_rx_regs = h->gdma_config.rx_reg;

	tca_spi_writel(tca_spi_readl(gdma_tx_regs+TCC_GPSB_GDMA_CHCTRL) & ~(Hw0 | Hw2), gdma_tx_regs+TCC_GPSB_GDMA_CHCTRL);
	tca_spi_writel(tca_spi_readl(gdma_rx_regs+TCC_GPSB_GDMA_CHCTRL) & ~(Hw0 | Hw2), gdma_rx_regs+TCC_GPSB_GDMA_CHCTRL);

	return 0;
}

static int tca_spi_gdmastop_slave(struct tca_spi_handle *h)
{
	void __iomem *gdma_rx_regs;

	gdma_rx_regs = h->gdma_config.rx_reg;

	tca_spi_writel(tca_spi_readl(gdma_rx_regs+TCC_GPSB_GDMA_CHCTRL) & ~(Hw0 | Hw2), gdma_rx_regs+TCC_GPSB_GDMA_CHCTRL);

	return 0;
}

static int tca_spi_gdmastart(struct tca_spi_handle *h)
{
	void __iomem *gdma_tx_regs, *gdma_rx_regs;
	gdma_tx_regs = h->gdma_config.tx_reg;
	gdma_rx_regs = h->gdma_config.rx_reg;

	tca_spi_writel(tca_spi_readl(gdma_tx_regs+TCC_GPSB_GDMA_CHCTRL) | (Hw0 | Hw2), gdma_tx_regs+TCC_GPSB_GDMA_CHCTRL);
	tca_spi_writel(tca_spi_readl(gdma_rx_regs+TCC_GPSB_GDMA_CHCTRL) | (Hw0 | Hw2), gdma_rx_regs+TCC_GPSB_GDMA_CHCTRL);

	return 0;
}

static int tca_spi_gdmastart_slave(struct tca_spi_handle *h)
{
	void __iomem *gdma_rx_regs;

	gdma_rx_regs = h->gdma_config.rx_reg;

	tca_spi_writel(tca_spi_readl(gdma_rx_regs+TCC_GPSB_GDMA_CHCTRL) | (Hw0 | Hw2), gdma_rx_regs+TCC_GPSB_GDMA_CHCTRL);

	return 0;
}

static void tca_spi_set_gdma_addr(struct tca_spi_handle *h)
{
	void __iomem *gdma_tx_regs, *gdma_rx_regs;
	u32 gpsb_port_reg;


	gdma_tx_regs = h->gdma_config.tx_reg;
	gdma_rx_regs = h->gdma_config.rx_reg;
	gpsb_port_reg = h->phy_reg_base;

	//printk("\x1b[1;33m[%s:%d] 0x%08X\x1b[0m\n", __func__, __LINE__,gpsb_port_reg);
	// GDMA Tx Source and Destination Address
	// Start Address
	tca_spi_writel(h->tx_dma.dma_addr,gdma_tx_regs+TCC_GPSB_GDMA_ST_SADR);
	// Destination Address : GPSB PORT Register
	tca_spi_writel(gpsb_port_reg,gdma_tx_regs+TCC_GPSB_GDMA_ST_DADR);

	// GDMA Rx Source and Destination Address
	// Start Address : GPSB PORT Register
	tca_spi_writel(gpsb_port_reg,gdma_rx_regs+TCC_GPSB_GDMA_ST_SADR);
	// Destination Address
	tca_spi_writel(h->rx_dma.dma_addr,gdma_rx_regs+TCC_GPSB_GDMA_ST_DADR);
#if 0
	printk("\x1b[1;33m[%s:%d] PORT Addr: %08X\x1b[0m\n", __func__, __LINE__,gpsb_port_reg);
	printk("\x1b[1;33m[%s:%d]TX SADR: %08X\x1b[0m\n", __func__, __LINE__,tca_spi_readl(gdma_tx_regs+TCC_GPSB_GDMA_ST_SADR));
	printk("\x1b[1;33m[%s:%d]TX DADR: %08X\x1b[0m\n", __func__, __LINE__,tca_spi_readl(gdma_tx_regs+TCC_GPSB_GDMA_ST_DADR));
	printk("\x1b[1;33m[%s:%d]RX SADR: %08X\x1b[0m\n", __func__, __LINE__,tca_spi_readl(gdma_rx_regs+TCC_GPSB_GDMA_ST_SADR));
	printk("\x1b[1;33m[%s:%d]RX DADR: %08X\x1b[0m\n", __func__, __LINE__,tca_spi_readl(gdma_rx_regs+TCC_GPSB_GDMA_ST_DADR));
#endif
}

static void tca_spi_set_gdma_addr_slave(struct tca_spi_handle *h)
{
	void __iomem *gdma_rx_regs;
	u32 gpsb_port_reg;

	gdma_rx_regs = h->gdma_config.rx_reg;
	gpsb_port_reg = h->phy_reg_base;

	// GDMA Rx Source and Destination Address
	// Start Address : GPSB PORT Register
	tca_spi_writel(gpsb_port_reg,gdma_rx_regs+TCC_GPSB_GDMA_ST_SADR);
	// Destination Address
	tca_spi_writel(h->rx_dma.dma_addr,gdma_rx_regs+TCC_GPSB_GDMA_ST_DADR);
}
#endif

static int tca_spi_isenabledma(struct tca_spi_handle *h)
{
	if(tca_spi_is_use_gdma(h))
	{
#ifndef TCC_DMA_ENGINE
		return tca_spi_isenable_gdma(h);
#else
		return 0;
#endif
	}

	return tca_spi_readl(h->regs + TCC_GPSB_DMACTR) & Hw0 ? 1 : 0;
}

static int tca_spi_dmastop(struct tca_spi_handle *h)
{
    if (!h->tx_pkt_remain) {
        TCC_GPSB_BITCLR(h->regs + TCC_GPSB_MODE, Hw3); /* Operation Disable */
    }

	// If GPSB use GDMA
	if(tca_spi_is_use_gdma(h))
	{
#ifndef TCC_DMA_ENGINE
		return tca_spi_gdmastop(h);
#else
		return 0;
#endif
	}

    TCC_GPSB_BITCLR(h->regs + TCC_GPSB_DMACTR, Hw31 | Hw30); /* disable DMA Transmit & Receive */
    TCC_GPSB_BITSET(h->regs + TCC_GPSB_DMAICR, Hw29| Hw28);
    TCC_GPSB_BITCLR(h->regs + TCC_GPSB_DMACTR, Hw0); /* DMA disable */

    return 0;
}

static int tca_spi_dmastop_slave(struct tca_spi_handle *h)
{
	// If GPSB use GDMA
	if(tca_spi_is_use_gdma(h))
	{
#ifndef TCC_DMA_ENGINE
		return tca_spi_gdmastop_slave(h);
#else
		return 0;
#endif
	}

    TCC_GPSB_BITCLR(h->regs + TCC_GPSB_DMACTR, Hw31 | Hw30); /* disable DMA Transmit & Receive */
    TCC_GPSB_BITSET(h->regs + TCC_GPSB_DMAICR, Hw29| Hw28);
    TCC_GPSB_BITCLR(h->regs + TCC_GPSB_DMACTR, Hw0); /* DMA disable */

    return 0;
}

static int tca_spi_dmastart(struct tca_spi_handle *h)
{

    if (h->ctf) {
        TCC_GPSB_BITSET(h->regs + TCC_GPSB_MODE, Hw4);
    } else {
        TCC_GPSB_BITCLR(h->regs + TCC_GPSB_MODE, Hw4);
    }
	TCC_GPSB_BITSET(h->regs + TCC_GPSB_MODE, Hw3); /* Operation Enable */

	// If GPSB use GDMA
	if(tca_spi_is_use_gdma(h))
	{
#ifndef TCC_DMA_ENGINE
		tca_spi_gdmastart(h);
#endif
	}
	else
	{
	    TCC_GPSB_BITSET(h->regs + TCC_GPSB_DMACTR, Hw31 | Hw30); /* enable DMA Transmit & Receive */
	    TCC_GPSB_BITCLR(h->regs + TCC_GPSB_DMACTR, Hw17 | Hw16 | Hw15 | Hw14); /* set Multiple address mode */

	    TCC_GPSB_BITCLR(h->regs + TCC_GPSB_DMAICR, Hw16); /* disable DMA Packet Interrupt */
	    TCC_GPSB_BITSET(h->regs + TCC_GPSB_DMAICR, Hw17); /* enable DMA Done Interrupt */
	    TCC_GPSB_BITCLR(h->regs + TCC_GPSB_DMAICR, Hw20); /* set rx interrupt */

	    TCC_GPSB_BITSET(h->regs + TCC_GPSB_DMACTR, Hw0); /* DMA enable */
	}

    return 0;
}

static int tca_spi_dmastart_slave(struct tca_spi_handle *h)
{
	// If GPSB use GDMA
	if(tca_spi_is_use_gdma(h))
	{
#ifndef TCC_DMA_ENGINE
		tca_spi_gdmastart_slave(h);
#endif
	}
	else
	{
	    TCC_GPSB_BITSET(h->regs + TCC_GPSB_DMACTR, Hw30); /* enable DMA Transmit */
	    TCC_GPSB_BITCLR(h->regs + TCC_GPSB_DMACTR, Hw17 | Hw16 | Hw15 | Hw14); /* set Multiple address mode */

	    TCC_GPSB_BITSET(h->regs + TCC_GPSB_DMAICR, Hw16); /* enable DMA Packet Interrupt */
	    TCC_GPSB_BITCLR(h->regs + TCC_GPSB_DMAICR, Hw20); /* set rx interrupt */

	    TCC_GPSB_BITSET(h->regs + TCC_GPSB_DMAICR, Hw16); /* enable DMA Packet Interrupt */

	    if (h->dma_mode == 0) {
	        TCC_GPSB_BITCSET(h->regs + TCC_GPSB_DMACTR, Hw5|Hw4, Hw29);	/* Normal mode & Continuous mode*/
	    } else {
	        TCC_GPSB_BITSET(h->regs + TCC_GPSB_DMACTR, Hw4);				/* MPEG2-TS mode */
	    }
		TCC_GPSB_BITSET(h->regs + TCC_GPSB_DMACTR, Hw0); /* DMA enable */
	}

    return 0;
}

static void tca_spi_clearfifopacket(struct tca_spi_handle *h)
{
    /* clear tx/rx FIFO & Packet counter  */
    TCC_GPSB_BITSET(h->regs + TCC_GPSB_MODE, Hw15 | Hw14);
	if(!tca_spi_is_use_gdma(h))
	{
	    TCC_GPSB_BITSET(h->regs + TCC_GPSB_DMACTR, Hw2);
	    TCC_GPSB_BITCLR(h->regs + TCC_GPSB_DMACTR, Hw2);
	}
    TCC_GPSB_BITCLR(h->regs + TCC_GPSB_MODE, Hw15 | Hw14);
}

static void tca_spi_setpacketcnt(struct tca_spi_handle *h, int size)
{
	if(!tca_spi_is_use_gdma(h))
	{
	    /* set packet count & size */
	    tca_spi_writel((size & 0x1FFF), h->regs + TCC_GPSB_PACKET);
	}
}

static void tca_spi_setpacketcnt_slave(struct tca_spi_handle *h, int size)
{
    unsigned int packet_cnt = (h->dma_total_packet_cnt & 0x1FFF) - 1;
#if defined(CONFIG_DAUDIO_KK) 
    unsigned int packet_size = (size & 0x1FFF);
#else
    unsigned int packet_size = (MPEG_PACKET_SIZE & 0x1FFF);
#endif
    unsigned int intr_packet_cnt = (h->dma_intr_packet_cnt & 0x1FFF) - 1;

    if (packet_size != size) {
        size = packet_size;
    }

	if(!tca_spi_is_use_gdma(h))
	{
	    tca_spi_writel(((packet_cnt << 16) | packet_size), h->regs + TCC_GPSB_PACKET);
		TCC_GPSB_BITCSET(h->regs + TCC_GPSB_DMAICR, 0x1FFF, intr_packet_cnt);
	}
}

static void tca_spi_setbitwidth(struct tca_spi_handle *h, int width)
{
    int width_value = (width - 1) & 0x1F;

    /* set bit width */
    TCC_GPSB_BITCLR(h->regs + TCC_GPSB_MODE, Hw12 | Hw11 | Hw10 | Hw9 | Hw8);
	TCC_GPSB_BITSET(h->regs + TCC_GPSB_MODE, (width_value << 8));

	if(!tca_spi_is_use_gdma(h))
	{
	    if (width_value & Hw4) {
	        TCC_GPSB_BITCLR(h->regs + TCC_GPSB_DMACTR, Hw28);
	    } else {
	        TCC_GPSB_BITSET(h->regs + TCC_GPSB_DMACTR, Hw28);
	    }
	}
}

static void tca_spi_setdmaaddr(struct tca_spi_handle *h)
{
	// If GPSB use GDMA
	if(tca_spi_is_use_gdma(h))
	{
#ifndef TCC_DMA_ENGINE
		tca_spi_set_gdma_addr(h);
#endif
	}
	else
	{
	    /* set dma txbase/rxbase & request DMA tx/rx */
	    tca_spi_writel((u32)h->tx_dma.dma_addr, h->regs + TCC_GPSB_TXBASE);
	    tca_spi_writel((u32)h->rx_dma.dma_addr, h->regs + TCC_GPSB_RXBASE);
	}

	if(h->tx_dma.dma_addr)
		TCC_GPSB_BITSET(h->regs + TCC_GPSB_INTEN, Hw31);
	else
		TCC_GPSB_BITCLR(h->regs + TCC_GPSB_INTEN, Hw31);

	if(h->rx_dma.dma_addr)
		TCC_GPSB_BITSET(h->regs + TCC_GPSB_INTEN, Hw30);
	else
		TCC_GPSB_BITCLR(h->regs + TCC_GPSB_INTEN, Hw30);
}

static void tca_spi_setdmaaddr_slave(struct tca_spi_handle *h)
{
	// If GPSB use GDMA
	if(tca_spi_is_use_gdma(h))
	{
#ifndef TCC_DMA_ENGINE
		tca_spi_set_gdma_addr_slave(h);
#endif
	}
	else
	{
		/* set dma txbase/rxbase & request DMA tx/rx */
	    tca_spi_writel(h->rx_dma.dma_addr, h->regs + TCC_GPSB_RXBASE);
	    //Hw18~Hw16 : Revceive FIFO threshold for interrupt/DMA request
	}
    if(h->dma_mode == 0) {
		if(h->rx_dma.dma_addr)
		{
			TCC_GPSB_BITSET(h->regs + TCC_GPSB_INTEN, Hw30|Hw15);
#if defined(CONFIG_DAUDIO_KK)
            TCC_GPSB_BITCLR(h->regs + TCC_GPSB_INTEN, Hw18|Hw17|Hw16);
#else
		#ifndef TCC_DMA_ENGINE
			TCC_GPSB_BITSET(h->regs + TCC_GPSB_INTEN, Hw18|Hw17|Hw16);
		#else
	                TCC_GPSB_BITCLR(h->regs + TCC_GPSB_INTEN, Hw18|Hw17|Hw16);
		#endif
#endif
		}
		else
		{
			TCC_GPSB_BITCLR(h->regs + TCC_GPSB_INTEN, Hw30);
		}
    } else {
		if(h->rx_dma.dma_addr)
		{
			TCC_GPSB_BITSET(h->regs + TCC_GPSB_INTEN, Hw30|Hw15);
#if defined(CONFIG_DAUDIO_KK)
			
			TCC_GPSB_BITCLR(h->regs + TCC_GPSB_INTEN, Hw18|Hw17|Hw16);
#else
			#ifndef TCC_DMA_ENGINE
			TCC_GPSB_BITSET(h->regs + TCC_GPSB_INTEN, Hw18|Hw17|Hw16);
			#else
			TCC_GPSB_BITCLR(h->regs + TCC_GPSB_INTEN, Hw18|Hw17|Hw16);
			#endif
#endif
		}
		else
		{
			TCC_GPSB_BITCLR(h->regs + TCC_GPSB_INTEN, Hw30);
		}
    }
}

static void tca_spi_hwinit(struct tca_spi_handle *h)
{
    /* init => set SPI mode, set Master mode ... */
    memset((void *)(h->regs), 0, sizeof(struct tca_spi_regs));

    h->set_bit_width(h, 32);

    h->set_dma_addr(h);

    /* Clear Fifo must be operated before operation enable */
    h->clear_fifo_packet(h);

    /* [SCK] Tx: risiing edge, Rx: falling edge */
    TCC_GPSB_BITSET(h->regs + TCC_GPSB_MODE, Hw17 | Hw18);
}

static void tca_spi_hwinit_slave(struct tca_spi_handle *h)
{

    /* init => set SPI mode, set Master mode ... */
    memset((void *)(h->regs), 0, sizeof(struct tca_spi_regs));

#ifndef TCC_DMA_ENGINE
    h->set_bit_width(h, 32);
#else
    h->set_bit_width(h, 8);
#endif

    h->set_dma_addr(h);

    /* Clear Fifo must be operated before operation enable */
    h->clear_fifo_packet(h);

#if defined(CONFIG_DAUDIO_KK)
/**
* @author sjpark@cleinsoft
* @date 2013/12/3
* spi setting for daudio - get data bit risiing SCK edge  (spi slave mode3)
* @date 2014/12/15
* There are two points of setting clock porality. This is initial code.
**/
//    TCC_GPSB_BITSET(h->regs + TCC_GPSB_MODE, Hw18 | Hw3);
    TCC_GPSB_BITSET(h->regs + TCC_GPSB_MODE, Hw3);
#else
    /* [SCK] Tx: risiing edge, Rx: falling edge & enable Operation */
   TCC_GPSB_BITSET(h->regs + TCC_GPSB_MODE, Hw17 | Hw18 | Hw3);
#endif
     TCC_GPSB_BITSET(h->regs + TCC_GPSB_MODE, Hw2 | Hw5);
}

static void tca_spi_set_mpegtspidmode(struct tca_spi_handle *h, int is_set)
{
	h->hw_init(h);
	if(!tca_spi_is_use_gdma(h))
	{
	    if (is_set) {
	        TCC_GPSB_BITSET(h->regs + TCC_GPSB_DMACTR, Hw19 | Hw18);
	    } else {
	        TCC_GPSB_BITCLR(h->regs + TCC_GPSB_DMACTR, Hw19 | Hw18);
	    }
	}
}

static void tca_spi_set_port(struct tca_spi_handle *h, struct tca_spi_port_config *port)
{
    static int init_port = 1;
    unsigned int gpsb_channel, gpsb_port;
    void __iomem *gpsb_pcf_regs = h->port_regs;
    int i;

    if(init_port)
    {
  	/* The port map value for each channel should be different.
	 * clear gpsb port config except used port (ch0, ch1)
	 */
	    TCC_GPSB_BITSET(gpsb_pcf_regs + TCC_GPSB_PCFG0, Hw32-Hw0);	// GPS use ch2
		TCC_GPSB_BITSET(gpsb_pcf_regs + TCC_GPSB_PCFG1, Hw16-Hw0);
        init_port = 0;
    }

    gpsb_channel = h->gpsb_channel;
    gpsb_port    = h->gpsb_port;

	printk(KERN_NOTICE "%s: PORT: %d", __func__, port->gpsb_port);

    if(h->gpsb_channel <= 3)
        TCC_GPSB_BITCSET(gpsb_pcf_regs + TCC_GPSB_PCFG0, (Hw8-Hw0)<<(gpsb_channel*8), gpsb_port<<(gpsb_channel*8));
    else
        TCC_GPSB_BITCSET(gpsb_pcf_regs + TCC_GPSB_PCFG1, (Hw8-Hw0)<<((gpsb_channel-4)*8), gpsb_port<<((gpsb_channel-4)*8));

	for(i=0;i<TCC_GPSB_MAX_CH;i++)
	{
		if(i == h->gpsb_channel) continue;

		if(i < 4)
		{
            if (((tca_spi_readl(gpsb_pcf_regs + TCC_GPSB_PCFG0) & (Hw8-Hw0)<<(i*8)) >> (i*8)) == gpsb_port) 
			{
				//printk("\x1b[1;33m[%s:%d] Port Conflict !! [[CH-%d:Port-%d]][CH-%d:Port-%d] : 0%08X\x1b[0m\n", __func__, __LINE__,
				//	h->gpsb_channel,gpsb_port,i,
				//	((tca_spi_readl(gpsb_pcf_regs + TCC_GPSB_PCFG0) & (Hw8-Hw0)<<(i*8)) >> (i*8)),
				//	tca_spi_readl(gpsb_pcf_regs + TCC_GPSB_PCFG0));
                TCC_GPSB_BITCSET(gpsb_pcf_regs + TCC_GPSB_PCFG0, (Hw8-Hw0)<<(i*8), 0xFF<<(i*8));
				//printk("\x1b[1;33m[%s:%d] PCFG0 --> 0x%08X\x1b[0m\n", __func__, __LINE__,tca_spi_readl(gpsb_pcf_regs + TCC_GPSB_PCFG0));
			}
		}
		else
		{
            if (((tca_spi_readl(gpsb_pcf_regs + TCC_GPSB_PCFG1) & (Hw8-Hw0)<<((i-4)*8)) >> ((i-4)*8)) == gpsb_port) 
			{
				//printk("\x1b[1;33m[%s:%d] Port Conflict !! [[CH-%d:Port-%d]][CH-%d:Port-%d] : 0%08X\x1b[0m\n", __func__, __LINE__,
				//	h->gpsb_channel,gpsb_port,i,
				//	((tca_spi_readl(gpsb_pcf_regs + TCC_GPSB_PCFG1) & (Hw8-Hw0)<<((i-4)*8)) >> ((i-4)*8)),
				//	tca_spi_readl(gpsb_pcf_regs + TCC_GPSB_PCFG1));
                TCC_GPSB_BITCSET(gpsb_pcf_regs + TCC_GPSB_PCFG1, (Hw8-Hw0)<<((i-4)*8), 0xFF<<((i-4)*8));
				//printk("\x1b[1;33m[%s:%d] PCFG0 --> 0x%08X\x1b[0m\n", __func__, __LINE__,tca_spi_readl(gpsb_pcf_regs + TCC_GPSB_PCFG1));	
			}
		}
	}
}

static void tca_spi_clear_port(struct tca_spi_handle *h)
{
	unsigned int gpsb_channel;
	void __iomem *gpsb_pcf_regs = h->port_regs;
	gpsb_channel = h->gpsb_channel;
	if(gpsb_channel <= 3)
		TCC_GPSB_BITCSET(gpsb_pcf_regs + TCC_GPSB_PCFG0, (Hw8-Hw0)<<(gpsb_channel*8), 0xFF<<(gpsb_channel*8));
	else
		TCC_GPSB_BITCSET(gpsb_pcf_regs + TCC_GPSB_PCFG1, (Hw8-Hw0)<<((gpsb_channel-4)*8), 0xFF<<((gpsb_channel-4)*8));
}

/******************************
 * return value
 *
 * ret == 0: success
 * ret > 0 or ret < 0: fail
 ******************************/
int tca_spi_init(tca_spi_handle_t *h,
                 void __iomem *regs,
                 unsigned int phy_reg_base,
                 void __iomem *port_regs,
                 void __iomem *pid_regs,
                 int irq,
                 dma_alloc_f tea_dma_alloc,
                 dma_free_f tea_dma_free,
                 int dma_size,
                 int id,
                 int is_slave,
                 struct tca_spi_port_config *port,
                 struct tca_spi_gdma_config *gdma,
                 const char *gpsb_name,
                 struct device *dev)
{
    int ret = -1;
#ifdef TCC_DMA_ENGINE
	struct tcc_spi_dma	dma;
#endif

    if (h) {
#ifdef TCC_DMA_ENGINE
		dma = h->dma;
		if(!dma.chan_rx)
#endif
		memset(h, 0, sizeof(tca_spi_handle_t));
	}
#if !defined(CONFIG_TCC_CODESONAR_BLOCKED)
#else
    else
	    return ret;
#endif

	// Memory Copy of GPSB Port and GDMA Configuration
    memcpy(&h->port_config, port, sizeof(struct tca_spi_port_config));
    memcpy(&h->gdma_config, gdma, sizeof(struct tca_spi_gdma_config));

    if (regs) {
#ifdef TCC_DMA_ENGINE
		h->dma = dma;
#endif
		h->dev = dev;
        h->regs = regs;
		h->phy_reg_base = phy_reg_base;
		h->port_regs = port_regs;
		h->pid_regs = pid_regs;
        h->irq = irq;
        h->id = id;
        h->is_slave = is_slave;
        h->gpsb_port = port->gpsb_port;

        if(!strncmp(gpsb_name,"gpsb0", 5))
            h->gpsb_channel = 0;
        else if(!strncmp(gpsb_name,"gpsb1", 5))
            h->gpsb_channel = 1;
        else if(!strncmp(gpsb_name,"gpsb2", 5))
            h->gpsb_channel = 2;
        else if(!strncmp(gpsb_name,"gpsb3", 5))
            h->gpsb_channel = 3;
        else if(!strncmp(gpsb_name,"gpsb4", 5))
            h->gpsb_channel = 4;
        else if(!strncmp(gpsb_name,"gpsb5", 5))
            h->gpsb_channel = 5;
        else
            return ret;

        h->dma_stop = is_slave ? tca_spi_dmastop_slave : tca_spi_dmastop;
        h->dma_start = is_slave ? tca_spi_dmastart_slave : tca_spi_dmastart;
        h->clear_fifo_packet = tca_spi_clearfifopacket;
        h->set_packet_cnt = is_slave ? tca_spi_setpacketcnt_slave : tca_spi_setpacketcnt;
        h->set_bit_width = tca_spi_setbitwidth;
        h->set_dma_addr = is_slave ? tca_spi_setdmaaddr_slave : tca_spi_setdmaaddr;
        h->hw_init = is_slave ? tca_spi_hwinit_slave : tca_spi_hwinit;
        h->is_enable_dma = tca_spi_isenabledma;
        h->set_mpegts_pidmode = tca_spi_set_mpegtspidmode;

        h->tea_dma_alloc = tea_dma_alloc;
        h->tea_dma_free = tea_dma_free;

        h->dma_total_size = dma_size;
        h->dma_mode = 1;	/* default MPEG2-TS DMA mode */

        h->ctf = 0;
        h->tx_pkt_remain= 0;

        tca_spi_set_port(h, port);

        if (h->tea_dma_alloc) {
            if (h->tea_dma_alloc(&(h->rx_dma), dma_size,h->dev) == 0) {
                if (is_slave) {
                    ret = 0;
                } else if (h->tea_dma_alloc(&(h->tx_dma), dma_size,h->dev) == 0 && h->tea_dma_alloc(&(h->tx_dma_1), dma_size,h->dev) == 0) {
                    ret = 0;
                }
            }
        } else {
            /* Already, tsif has rx_dma buf */
            ret = 0;
        }
    }
    return ret;
}

void tca_spi_clean(tca_spi_handle_t *h)
{
    if (h) {
#ifdef TCC_DMA_ENGINE
		struct tcc_spi_dma	dma;
		dma = h->dma;
#endif
        if (h->tea_dma_free) {
            h->tea_dma_free(&(h->tx_dma), h->dev);
            h->tea_dma_free(&(h->rx_dma), h->dev);
            h->tea_dma_free(&(h->tx_dma_1), h->dev);
        }

        /* Clear PCFG (only one channel being used)*/
        tca_spi_clear_port(h);

        memset(h, 0, sizeof(tca_spi_handle_t));
#ifdef TCC_DMA_ENGINE
		h->dma = dma;
#endif
    }
}

int tca_spi_register_pids(tca_spi_handle_t *h, unsigned int *pids, unsigned int count)
{
    int ret = 0, gpsb_channel = -1;
    gpsb_channel = h->gpsb_channel;

    //supporting pids is 32 
    if (count <= 32) {
#ifdef CONFIG_ARM64
        void __iomem *PIDT;
#else
        volatile unsigned long* PIDT;
#endif
        int i = 0, pid_ch = 0;

        if(gpsb_channel == 0)
            pid_ch = Hw29;//HwGPSB_PIDT_CH0;
        else if(gpsb_channel == 1)
            pid_ch = Hw30;//HwGPSB_PIDT_CH1;
        else if(gpsb_channel == 2)
            pid_ch = Hw31;//HwGPSB_PIDT_CH2;
            
        for (i = 0; i < 32; i++) {
#ifdef CONFIG_ARM64
            PIDT = h->pid_regs + (4*i);
            tca_spi_writel(0x0, PIDT);
#else
            PIDT = (volatile unsigned long *)(h->pid_regs+4*i);//(volatile unsigned long *)tcc_p2v(HwGPSB_PIDT(i));
            *PIDT = 0;
#endif
        }
        if (count > 0) {
            for (i = 0; i < count; i++) {
#ifdef CONFIG_ARM64
                PIDT = h->pid_regs + (4*i);
                tca_spi_writel(pids[i] & 0x1FFFFFFF, PIDT);
                TCC_GPSB_BITSET(PIDT, pid_ch);
                printk("PIDT 0x%p : 0x%08X\n", PIDT, tca_spi_readl(PIDT));
#else
                PIDT = (volatile unsigned long *)(h->pid_regs+4*i);//(volatile unsigned long *)tcc_p2v(HwGPSB_PIDT(i));
                *PIDT = pids[i] & 0x1FFFFFFF;
                BITSET(*PIDT, pid_ch);
                printk("PIDT 0x%08X : 0x%08X\n", (unsigned int)PIDT, (unsigned int)*PIDT);
#endif
            }
            h->set_mpegts_pidmode(h, 1);
        } 
    }
    else {
        printk("tsif: PID TABLE is so big !!!\n");
        ret = -EINVAL;
    }
    return ret;
}

EXPORT_SYMBOL(tca_spi_init);
EXPORT_SYMBOL(tca_spi_clean);
EXPORT_SYMBOL(tca_spi_register_pids);
EXPORT_SYMBOL(tca_spi_print_gdma_config);
EXPORT_SYMBOL(tca_spi_gdma_intr_ch);

#if 0
/****************************************************************************
 * Below is SW Timer for Braodcasting.
 * It is used for calculating STC time for PCR time.
 * Timer should be 1ms resolution.
****************************************************************************/

#if defined(CONFIG_ARCH_TCC897X) || defined(CONFIG_ARCH_TCC570X)
#define	TCCTIMER_VALUE_MAX		(357913)//(0x100000000/12000) // 32bit max for 6000Khz
#define TCC_PA_TIMER		0x74300000

void __iomem *timer_reg = (void __iomem *)io_p2v(TCC_PA_TIMER);
static unsigned long guiRefTime, guiPrevTime, guiOffsetTime; //ms
#else
#include <linux/jiffies.h>
#endif

static unsigned tcc_ref_time_flags = 0;

int TCCREFTIME_Open(void)
{
    if (tcc_ref_time_flags)
        return 1;
    tcc_ref_time_flags = 1;

#if defined(CONFIG_ARCH_TCC897X) || defined(CONFIG_ARCH_TCC570X)
    guiRefTime = 0;
    guiPrevTime = 0;
    guiOffsetTime = 0;
#endif
    return 0;
}

int TCCREFTIME_Close(void)
{
    tcc_ref_time_flags = 0;
    return 0;
}

unsigned int TCCREFTIME_GetTime(void)
{
#if defined(CONFIG_ARCH_TCC897X) || defined(CONFIG_ARCH_TCC570X)
    guiRefTime = __raw_readl(timer_reg+0x94)/12000; //because 12Mhz, divided by 12000 needs

    if(guiRefTime < guiPrevTime )
        guiOffsetTime += TCCTIMER_VALUE_MAX;
    
    guiPrevTime = guiRefTime;

    //PRINTF("%s :: %d",__func__, (guiRefTime+guiOffsetTime));
    return guiRefTime+guiOffsetTime;
#else
    return jiffies_to_msecs(get_jiffies_64());
#endif
}

void TCCREFTIME_TestMain(void)
{
    int start, check, sec;
    TCCREFTIME_Open();		

    //PRINTF("0x%x", pTCCTIMER->TCFG4);	
    //for(i=0;i<50;i++)
    start = TCCREFTIME_GetTime();
    sec = 0;
    while(1)
    {
        //PRINTF("0x%x :: %d\n", pTCCTIMER->TCNT4, pTCCTIMER->TCNT4);
        //PRINTF("REAL :: 0x%x :: %d", TCCREFTIME_GetTime(), TCCREFTIME_GetTime());		
        check = TCCREFTIME_GetTime();
        if( (check - start) == 1000 )
        {
            printk("%d second\n", ++sec);
            start = check;
        }		
        if(sec == 30)
            break;
    }
    TCCREFTIME_Close();
}

EXPORT_SYMBOL(TCCREFTIME_Open);
EXPORT_SYMBOL(TCCREFTIME_Close);
EXPORT_SYMBOL(TCCREFTIME_GetTime);
#endif
