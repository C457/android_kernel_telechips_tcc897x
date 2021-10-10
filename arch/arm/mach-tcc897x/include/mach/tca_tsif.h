/* 
 * arch/arm/mach-tcc893x/include/mach/tca_tsif.h
 *
 * Author:  <linux@telechips.com>
 * Created: 10th Jun, 2013 
 * Description: LINUX TSIF DRIVER FUNCTIONS
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

#ifndef __TCC_TSIF_MODULE_HWSET_H__
#define __TCC_TSIF_MODULE_HWSET_H__

#include <mach/memory.h>
#include <mach/irqs.h>

#define    SUPPORT_PIDFILTER_INTERNAL    //filtering support using internal logic(TSPID reg)
//#define    SUPPORT_PIDFILTER_DMA       //filtering suuport using dma logic
#define    SUPPORT_GDMA 

#define TSIF_BYTE_SIZE			4		// 1, 2, 4byte
#define GDMA_WORD_SIZE			4		// 1, 2, 4byte
#define GDMA_BURST_SIZE			4		// 1, 2, 4, 8 = (TSIF_BYTE_SIZE * TSIF_RXFIFO_THRESHOLD) / GDMA_WORD_SIZE
#define MPEG_PACKET_SIZE 		(188)
#define TSIF_DMA_SIZE           0x100000      //(CONSISTENT_DMA_SIZE / 8)

#ifdef  SUPPORT_GDMA
#define TSIF_RXFIFO_THRESHOLD	4		// 0-7 (0=>8 depth)
#define TSIF_MASK_SIZE			((0x10000000 - TSIF_DMA_SIZE) >> 4)
#define TSIF_DMA_HOPE_CNT(cnt)  (((cnt) * MPEG_PACKET_SIZE) / (TSIF_BYTE_SIZE * TSIF_RXFIFO_THRESHOLD) + 1)
#define TSIF_DMA_INCREASE_SIZE	0x4

#define TSIF_GDMA_CONTROLLER    0
#define TSIF_GDMA_CHANNEL       1
#else
#define TSIF_RXFIFO_THRESHOLD	1		// 0-7 (0=>8 depth)
#endif


#pragma pack(push, 4)
struct tcc_tsif_regs {
    volatile unsigned long TSDI, TSRXCR, TSPID[16], TSIC, TSIS, TSISP, TSTXC;
};
#pragma pack(pop)

struct tea_dma_buf {
    void *v_addr;
    unsigned int dma_addr;
    int buf_size; // total size of DMA
};

typedef struct tcc_tsif_handle tcc_tsif_handle_t;
typedef int (*dma_alloc_f)(struct tea_dma_buf *tdma, unsigned int size);
typedef void (*dma_free_f)(struct tea_dma_buf *tdma);

struct tcc_tsif_handle {
    volatile struct tcc_tsif_regs *regs;
    struct tea_dma_buf rx_dma;
	//int flag;
    int irq;
    void *private_data;
    int id;
	int dma_controller;
	int dma_ch;
	//int is_slave;
	int gpio_port;

    int (*dma_stop)(struct tcc_tsif_handle *h);
    int (*dma_start)(struct tcc_tsif_handle *h);
    void (*clear_fifo_packet)(struct tcc_tsif_handle *h);
	void (*tsif_set)(struct tcc_tsif_handle *h);
	void (*tsif_isr)(struct tcc_tsif_handle *h);
	void (*hw_init)(struct tcc_tsif_handle *h);
    int (*tsif_resync)(struct tcc_tsif_handle *h);
	
    dma_alloc_f tea_dma_alloc; // tea function.
    dma_free_f tea_dma_free; // tea function.

	//int clk;	// Mhz
	
    /* add for slave */
    unsigned int dma_total_packet_cnt, dma_intr_packet_cnt;
    int q_pos, cur_q_pos, prev_q_pos;
    int dma_total_size;

	int dma_phy_maxaddr;
	int dma_phy_rdpos;
	int dma_phy_curpos;
	int dma_virt_maxaddr;
	int dma_virt_rdpos;
	int dma_virt_curpos;
	int dma_recv_size;
	int dma_intr_cnt;
	
	unsigned char msb_first;
	unsigned char big_endian;
	unsigned char serial_mode;
	unsigned char clk_polarity;
	unsigned char valid_polarity;
	unsigned char sync_polarity;
	unsigned char sync_delay;
	unsigned int dma_phy_addr;
	unsigned int dma_phy_size;
	unsigned int mpeg_ts; //0:spi mode, 1(bit0):mpeg_ts format sigal, 2(bit1):mpeg_ts siganl, use pid filtering & ts sync
    unsigned int match_pids[32];
    unsigned int match_pids_count;
};

extern void tca_tsif_set_interface(int mode); // mode(0: serial, 1: parallel)
extern int tca_tsif_get_interface(void);
extern int tca_tsif_init(struct tcc_tsif_handle *h, volatile struct tcc_tsif_regs *regs, 
		dma_alloc_f tea_dma_alloc, dma_free_f tea_dma_free, int dma_size, int tsif_ch, int dma_controller, int dma_ch, int port);
extern void tca_tsif_clean(struct tcc_tsif_handle *h);

extern int tca_tsif_register_pids(struct tcc_tsif_handle *h, unsigned int *pids, unsigned int count);
extern int tca_tsif_can_support(void);
#endif /*__TCC_TSIF_MODULE_HWSET_H__*/
