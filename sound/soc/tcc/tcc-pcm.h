/*
 * linux/sound/soc/tcc/tcc-pcm.h   
 *
 * Based on:    linux/sound/arm/pxa2xx-pcm.h
 * Author:  <linux@telechips.com>
 * Created:	Nov 30, 2004
 * Modified:    Nov 25, 2008
 * Description: ALSA PCM interface for the Intel PXA2xx chip
 *
 * Copyright:	MontaVista Software, Inc.
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

#ifndef _tcc_PCM_H
#define _tcc_PCM_H

#include "tcc/tca_audio_hw.h"

#define __play_buf_size 65536
#define __play_buf_cnt  8
#define __cap_buf_size 65536
#define __cap_buf_cnt  8

#define min_period_size 256
#define min_period_cnt 2	// Linux recommended value -> 4

#define tcc_i2s_fifo_size 512 //bytes, 128word
#define DMA_CH_OUT 0x0001
#define DMA_CH_IN  0x0002
#define DMA_CH_SPDIF 0x0004
#define DMA_CH_SPDIF_IN 0x0008	// Planet 20150812 S/PDIF_Rx
#define DMA_CH_CDIF 0x0010

#define TCC_INTERRUPT_REQUESTED 0x0001
#define TCC_RUNNING_PLAY        0x0002
#define TCC_RUNNING_CAPTURE     0x0004
#define TCC_RUNNING_SPDIF_PLAY       0x0008
#define TCC_RUNNING_SPDIF_CAPTURE 	0x0100	// Planet 20150812 S/PDIF_Rx
#define TCC_RUNNING_CDIF_CAPTURE 0x0200

#define TCC_TX_INTERRUPT_REQUESTED 0x0010
#define TCC_RX_INTERRUPT_REQUESTED 0x0020
#define NEXT_BUF(_s_,_b_) { \
    (_s_)->_b_##_idx++; \
    (_s_)->_b_##_idx %= (_s_)->nbfrags; \
    (_s_)->_b_ = (_s_)->buffers + (_s_)->_b_##_idx; }

#if defined(CONFIG_SND_SOC_DAUDIO_FIFO_CLEAR_DELAY)
// I2S FIFO clear for 1st pop-noise after boot
#define TCC_DMA_HALF_BUFF_STEREO 64
#define TCC_DMA_HALF_BUFF_MULTI  16
#endif

struct tcc_interrupt_info_x {
    struct snd_pcm_substream *play_ptr;
    struct snd_pcm_substream *cap_ptr;
    struct snd_pcm_substream *spdif_ptr;
	struct snd_pcm_substream *spdif_cap_ptr;	// Planet 20150812 S/PDIF_Rx
	struct snd_pcm_substream *cdif_cap_ptr;
	spinlock_t slock;	//For Access ADMA-ChCtrl register
    //struct mutex mutex;
    unsigned int flag;

    volatile ADMASPDIFTX adma_spdif_tx_base;
    volatile ADMA adma;
    volatile ADMADAI dai;

	bool cap_del_1st;
	unsigned int cap_pos_1st;
	unsigned int cap_intr_byte;
};

struct tcc_pcm_dma_params {
    char *name; /* stream identifier */
    int channel;
    dma_addr_t dma_addr;
    int dma_size; /* Size of the DMA transfer */
};

struct tcc_runtime_data {
    int dma_ch;
    struct tcc_pcm_dma_params *params;
    dma_addr_t dma_buffer;          /* physical address of dma buffer */
    dma_addr_t dma_buffer_end;      /* first address beyond DMA buffer */
    size_t period_size;
    size_t nperiod;     
    dma_addr_t period_ptr;          /* physical address of next period */
};

struct audio_buf_t{
    int size;       /* buffer size */
    char *start;        /* point to actual buffer */
    dma_addr_t dma_addr;    /* physical buffer address */
    struct semaphore sem;    /* down before touching the buffer */
    int master;     /* owner for buffer allocation, contain size when true */
    int dma_size;
};

struct audio_stream_t {
    struct audio_buf_t *buffers;   /* pointer to audio buffer structures */
    struct audio_buf_t *buf;   /* current buffer used by read/write */
    u_int buf_idx;      /* index for the pointer above */
    u_int play_idx;     /* the current buffer that playing */
    u_int fragsize;     /* fragment i.e. buffer size */
    u_int nbfrags;      /* nbr of fragments */
};

struct dma_buf_t {
    char *addr;
    dma_addr_t dma_addr;
    int  buf_size;  // total size of DMA
    int  page_size; // size of each page
    int  dma_half;  // 0 or 1, mark the next available buffer
};

extern int tcc_pcm_platform_register(struct device *dev);
extern void tcc_pcm_platform_unregister(struct device *dev);
extern int tcc_pcm_sub1_platform_register(struct device *dev);
extern void tcc_pcm_sub1_platform_unregister(struct device *dev);
#if defined(CONFIG_SND_TCC_AUDIO_DSP)
extern int tcc_pcm_dsp_platform_register(struct device *dev);
extern void tcc_pcm_dsp_platform_unregister(struct device *dev);
extern int tcc_pcm_ch1_platform_register(struct device *dev);
extern void tcc_pcm_ch1_platform_unregister(struct device *dev);
extern int tcc_pcm_ch2_platform_register(struct device *dev);
extern void tcc_pcm_ch2_platform_unregister(struct device *dev);
#endif
#if !defined(CONFIG_ARCH_TCC898X)
extern int tcc_pcm_sub2_platform_register(struct device *dev);
extern void tcc_pcm_sub2_platform_unregister(struct device *dev);
extern int tcc_pcm_sub3_platform_register(struct device *dev);
extern void tcc_pcm_sub3_platform_unregister(struct device *dev);
#endif

#endif
