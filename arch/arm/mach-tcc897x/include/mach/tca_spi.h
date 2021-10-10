/* 
 * arch/arm/mach-tcc893x/include/mach/tca_spi.h
 *
 * Author:  <linux@telechips.com>
 * Created: 10th Jun, 2013 
 * Description: LINUX SPI DRIVER FUNCTIONS
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

#ifndef __TCA_SPI_HWSET_H__
#define __TCA_SPI_HWSET_H__

#define MPEG_PACKET_SIZE 188
#define WAIT_TIME_FOR_DMA_DONE (1000 * 8)

#if defined(CONFIG_MACH_TCC8800) && defined(CONFIG_DRAM_DDR3)
//#define     SUPORT_USE_SRAM
#define     SRAM_PHY_ADDR      0x10001000
#define     SRAM_VIR_ADDR      0xEFF01000
#define     SRAM_TOT_PACKET    410
#define     SRAM_INT_PACKET    1
#endif

#pragma pack(push, 4)
struct tca_spi_regs {
    volatile unsigned long PORT, STAT, INTEN, MODE, CTRL, EVTCTRL, CCV, 
		DUMMY,
        TXBASE, RXBASE, PACKET, DMACTR, DMASTR, DMAICR;
};
#pragma pack(pop)

struct tea_dma_buf {
    void *v_addr;
    unsigned int dma_addr;
    int buf_size; // total size of DMA
};

struct tca_spi_port_config {
    int          gpsb_id;
    unsigned pin0;
    unsigned pin1;
    unsigned pin2;
    unsigned pin3;
    unsigned gpsb_func;
    unsigned gpsb_port;
    const char *name;
};

typedef struct tca_spi_handle tca_spi_handle_t;
typedef int (*dma_alloc_f)(struct tea_dma_buf *tdma, unsigned int size);
typedef void (*dma_free_f)(struct tea_dma_buf *tdma);

struct tca_spi_handle {
    volatile struct tca_spi_regs *regs;
    struct tea_dma_buf tx_dma, rx_dma;
    struct tea_dma_buf tx_dma_1;
    struct tca_spi_port_config port_config;
    int flag;
    int irq;
    void *private_data;
    int id;
    int is_slave;
    int gpsb_port;
    int gpsb_channel;

    int (*is_enable_dma)(tca_spi_handle_t *h);
    int (*dma_stop)(tca_spi_handle_t *h);
    int (*dma_start)(tca_spi_handle_t *h);
    void (*clear_fifo_packet)(tca_spi_handle_t *h);
    void (*set_packet_cnt)(tca_spi_handle_t *h, int cnt);
    void (*set_bit_width)(tca_spi_handle_t *h, int width);
    void (*set_dma_addr)(tca_spi_handle_t *h);
    void (*hw_init)(tca_spi_handle_t *h);
    void (*set_mpegts_pidmode)(tca_spi_handle_t *h, int is_set);

    dma_alloc_f tea_dma_alloc;	// tea function.
    dma_free_f tea_dma_free;	// tea function.

    int clk;	// Mhz
    int ctf;	// continuous transfer mode
    int tx_pkt_remain;
	
    /* add for slave */
    unsigned int dma_total_packet_cnt, dma_intr_packet_cnt;
    int q_pos, cur_q_pos;
    int dma_total_size;
    int dma_mode;

    /* backup gpsb regs */
    unsigned int bak_gpio_port;
    unsigned int bak_gpsb_port;
};


#define tca_spi_setCPOL(R, S) \
    do {\
        if (S) BITSET((R)->MODE, Hw16);\
        else BITCLR((R)->MODE, Hw16);\
    } while (0)

#define tca_spi_setCPHA(R, S) \
    do {\
        if (S) BITSET((R)->MODE, Hw18 | Hw17);\
        else BITCLR((R)->MODE, Hw18 | Hw17);\
    } while (0)

#define tca_spi_setCS_HIGH(R, S) \
    do {\
        if (S) BITSET((R)->MODE, Hw20 | Hw19);\
        else BITCLR((R)->MODE, Hw20 | Hw19);\
    } while (0)
#define tca_spi_setLSB_FIRST(R, S) \
    do {\
        if (S) BITSET((R)->MODE, Hw7);\
        else BITCLR((R)->MODE, Hw7);\
    } while (0) 


#ifdef __cplusplus
extern "C" {
#endif
extern int tca_spi_init(tca_spi_handle_t *h,
                        volatile struct tca_spi_regs *regs,
                        int irq,
                        dma_alloc_f tea_dma_alloc,
                        dma_free_f tea_dma_free,
                        int dma_size,
                        int id,
                        int is_slave,
                        struct tca_spi_port_config *,
                        const char *gpsb_name);

extern void tca_spi_clean(tca_spi_handle_t *h);
extern int tca_spi_register_pids(tca_spi_handle_t *h, unsigned int *pids, unsigned int count);
extern int tca_tsif_can_support(void);
#ifdef __cplusplus
}
#endif

#endif /*__TCA_SPI_H__*/
