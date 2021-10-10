/*
 * linux/sound/soc/tcc/tcc-pcm-v20.c  
 *
 * Based on:    linux/sound/arm/pxa2xx-pcm.c
 * Author:  <linux@telechips.com>
 * Created: Nov 30, 2004
 * Modified:    Feb 24, 2016
 * Description: ALSA PCM interface for the Telechips TCC898x chip
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

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>

#include <linux/delay.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "tcc/tca_audio_hw.h"

#include "tcc-i2s.h"
#include "tcc-pcm.h"

#include "tcc/tca_tcchwcontrol.h"

#undef alsa_dbg
#if 0
#define alsa_dbg(f, a...)  printk("== alsa-debug PCM SUB0 == " f, ##a)
#else
#define alsa_dbg(f, a...)
#endif

#if defined(CONFIG_PM)
static ADMA gADMA;
#endif

static struct tcc_interrupt_info_x tcc_alsa_info;

static struct snd_dma_buffer mono_dma_play,mono_dma_capture;

static const struct snd_pcm_hardware tcc_pcm_hardware_play = {
    .info = (SNDRV_PCM_INFO_MMAP
             | SNDRV_PCM_INFO_MMAP_VALID
             | SNDRV_PCM_INFO_INTERLEAVED
             | SNDRV_PCM_INFO_BLOCK_TRANSFER
             | SNDRV_PCM_INFO_PAUSE
             | SNDRV_PCM_INFO_RESUME),

    .formats = (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_U16_LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_IEC958_SUBFRAME_LE),	// Planet 20120306
#if defined(CONFIG_SND_SOC_TCC_MULTICHANNEL)
    .rates        = SNDRV_PCM_RATE_8000_192000,
    .rate_min     = 8000,
    .rate_max     = 192000,
    .channels_min = 1,
    .channels_max = 8,
#else
    .rates        = SNDRV_PCM_RATE_8000_96000,
    .rate_min     = 8000,
    .rate_max     = 96000,
    .channels_min = 1,
    .channels_max = 2,
#endif
    .period_bytes_min = min_period_size,
    .period_bytes_max = __play_buf_size,
    .periods_min      = min_period_cnt,
#if defined(CONFIG_ANDROID) // Android Style
    .periods_max      = __play_buf_cnt,
    .buffer_bytes_max = __play_buf_cnt * __play_buf_size,
#else  // Linux Style
    .periods_max      = (__play_buf_cnt * __play_buf_size)/min_period_size,
    .buffer_bytes_max = __play_buf_cnt * __play_buf_size,
#endif
    .fifo_size = 16,  //ignored
};

static const struct snd_pcm_hardware tcc_pcm_hardware_capture = {
    .info = (SNDRV_PCM_INFO_MMAP
             | SNDRV_PCM_INFO_MMAP_VALID
             | SNDRV_PCM_INFO_INTERLEAVED
             | SNDRV_PCM_INFO_BLOCK_TRANSFER
             | SNDRV_PCM_INFO_PAUSE
             | SNDRV_PCM_INFO_RESUME),

#if defined(CONFIG_VIDEO_HDMI_IN_SENSOR)
	.formats      = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_U16_LE | SNDRV_PCM_FMTBIT_S24_LE,
#else
	.formats      = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
#endif
#if defined(CONFIG_SND_SOC_TCC_MULTICHANNEL)
	.rates        = SNDRV_PCM_RATE_8000_192000,
	.rate_min     = 8000,
	.rate_max     = 192000,
	.channels_min = 1,
	.channels_max = 8,
#else
	.rates        = SNDRV_PCM_RATE_8000_96000,
    .rate_min     = 8000,
    .rate_max     = 96000,
    .channels_min = 1,
    .channels_max = 2,
#endif
    .period_bytes_min = min_period_size,
    .period_bytes_max = __cap_buf_size,
    .periods_min      = min_period_cnt,
#if defined(CONFIG_ANDROID) // Android Style
    .periods_max      = __cap_buf_cnt,
    .buffer_bytes_max = __cap_buf_cnt * __cap_buf_size,
#else  // Linux Style
    .periods_max      = (__cap_buf_cnt * __cap_buf_size)/min_period_size,
    .buffer_bytes_max = __cap_buf_cnt * __cap_buf_size,
#endif
    .fifo_size = 16, //ignored
};

static int alsa_get_intr_num(struct snd_pcm_substream *substream)
{
    if (substream)
        return INT_ADMA0;

    return -1;
}

static void set_dma_outbuffer(unsigned int addr, unsigned int length,unsigned int period,snd_pcm_format_t format,int chs)
{
    volatile PADMA pADMA = (volatile PADMA)tcc_p2v(BASE_ADDR_ADMA0);
    unsigned long dma_buffer = 0;
	int bit_count;

    BITCLR(pADMA->ChCtrl, Hw28);

//    dma_buffer = 0xFFFFFF00 / ((length >> 4) << 8);
//    dma_buffer = dma_buffer * ((length >> 4) << 8);

	bit_count = 31;
	while(bit_count > 3)
	{
		if((0x00000001 << bit_count) & length) 
			break;

		bit_count--;
	};

	if(bit_count <= 3)
	{
		alsa_dbg("Error : not valid length\n");
		return;
	}
	else
	{
		length = 0x00000001<<bit_count;
		alsa_dbg("[%s], original len[%u] period[%u]\n", __func__, length,period);

	}
    
    dma_buffer = 0xFFFFFF00 & (~((length<<4)-1));

    pADMA->TxDaParam = dma_buffer | 4;
    // generate interrupt when half of buffer was played
    pADMA->TxDaTCnt = (period >> 0x05) - 1;

    alsa_dbg("[%s], addr[0x%08X], len[%u], period[%u]\n", __func__, addr, length, period);
    alsa_dbg("[%s] HwTxDaParam [0x%X]\n", __func__, (int)(dma_buffer | 4));
    alsa_dbg("[%s] HwTxDaTCnt [%d]\n", __func__, ((period) >> 0x05) - 1);

    if(chs == 1){
        alsa_dbg("[%s] addr[0x%08X] mono_dma_play,addr[0x%08X]\n", __func__,addr,mono_dma_play.addr);    
        pADMA->TxDaSarL = addr; 
        pADMA->TxDaSar = mono_dma_play.addr;
        pADMA->TransCtrl &= ~(Hw8);
        pADMA->TransCtrl |= (Hw28 | Hw16 | (Hw9/*4Cycle*/) | (Hw0 | Hw1)); // MONO
        pADMA->ChCtrl |= Hw16;  // MONO
    }else {
        pADMA->TxDaSar = addr;
        pADMA->TransCtrl |= (Hw28 | Hw16 | (Hw9|Hw8) | (Hw0 | Hw1));
        pADMA->ChCtrl &= ~(Hw16);  
    }

    if(format == SNDRV_PCM_FORMAT_S24_LE){ // Dynamic 16/24bit support
        pADMA->ChCtrl &= (~Hw12);
        alsa_dbg("%s SNDRV_PCM_FORMAT_S24_LE foramt %d\n",__func__,format);
    }        
    else{
        pADMA->ChCtrl |= Hw12;
        alsa_dbg("%s SNDRV_PCM_FORMAT_S16_LE foramt %d\n",__func__,format);
    }

}

static void set_dma_spdif_outbuffer(unsigned int addr, unsigned int length, unsigned int period)
{
    volatile PADMA pADMA = (volatile PADMA)tcc_p2v(BASE_ADDR_ADMA0);
    volatile PADMASPDIFTX pADMASPDIFTX = (volatile PADMASPDIFTX)tcc_p2v(BASE_ADDR_SPDIFTX0);
    unsigned long dma_buffer = 0;
	int bit_count;

    alsa_dbg("%s, addr[0x%08X], len[%u], period[%u]\n", __func__, addr, length, period);
    BITCLR(pADMA->ChCtrl, Hw29);

//    dma_buffer = 0xFFFFFF00 / (((length) >> 4) << 8);
//    dma_buffer = dma_buffer * (((length) >> 4) << 8);

	bit_count = 31;
	while(bit_count > 3)
	{
		if((0x00000001 << bit_count) & length) 
			break;

		bit_count--;
	};

	if(bit_count <= 3)
	{
		alsa_dbg("Error : not valid length\n");
		return;
	}
	else
	{
		length = 0x00000001<<bit_count;
		alsa_dbg("[%s], original len[%u]\n", __func__, length);

	}
	
    dma_buffer = 0xFFFFFF00 & (~((length<<4)-1));

    pADMA->TxSpParam = dma_buffer | 4;
    pADMA->TxSpTCnt = (period >> 0x05) - 1;

    alsa_dbg("[%s] HwTxDaParam [0x%X]\n", __func__, (int)(dma_buffer | 4));
    alsa_dbg("[%s] HwTxDaTCnt [%d]\n", __func__, ((period) >> 0x05) - 1);

    pADMA->TxSpSar = addr;
    pADMA->TransCtrl |= (Hw28 | Hw17 | (Hw11|Hw10) | (Hw3 | Hw2));

    memset((void *)pADMASPDIFTX->UserData, 0, 24);
    memset((void *)pADMASPDIFTX->ChStatus, 0, 24);
    memset((void *)pADMASPDIFTX->TxBuffer, 0, 128);

#ifdef SPDIF_24BIT_MODE
	pADMASPDIFTX->TxBuffer[0] = 0;
	pADMASPDIFTX->DMACFG &= ~(Hw12|Hw13);
#else
    pADMASPDIFTX->TxConfig |= HwZERO | Hw8 | Hw1;
	pADMASPDIFTX->TxBuffer[0] = 0;
#endif

    /*
     * Hw2: interrupt output enable
     * Hw1: data being valid
     * Hw0: enable the transmission
     */
#if 0	  // Android Style
    pADMASPDIFTX->TxConfig |= HwZERO | Hw1 | Hw0;
#endif	
    pADMASPDIFTX->TxIntStat = 0x1E; /* Clear Interrupt Status */


    alsa_dbg("%s, spdif current addr[0x%08X] \n", __func__, pADMA->TxSpCsar);
    pADMA->ChCtrl |= (Hw13);
}

static void set_dma_inbuffer(unsigned int addr, unsigned int length, unsigned int period,snd_pcm_format_t format,int chs)
{
    volatile PADMA pADMA = (volatile PADMA)tcc_p2v(BASE_ADDR_ADMA0);
    unsigned long dma_buffer = 0;
	int bit_count;


    BITCLR(pADMA->ChCtrl, Hw30);

//    dma_buffer = 0xFFFFFF00 / ((length >> 4) << 8);
//    dma_buffer = dma_buffer * ((length >> 4) << 8);

	bit_count = 31;
	while(bit_count > 3)
	{
		if((0x00000001 << bit_count) & length) 
			break;

		bit_count--;
	};

	if(bit_count <= 3)
	{
		alsa_dbg("Error : not valid length\n");
		return;
	}
	else
	{
		length = 0x00000001<<bit_count;
		alsa_dbg("[%s], original len[%u]\n", __func__, length);

	}
	
    dma_buffer = 0xFFFFFF00 & (~((length<<4)-1));

    pADMA->RxDaParam = dma_buffer | 4;
    pADMA->RxDaTCnt = (period >> 0x05) - 1;

    alsa_dbg("[%s], addr[0x%08X], len[%d]\n", __func__, addr, length);
    alsa_dbg("[%s] HwRxDaParam [0x%X]\n", __func__, (int)dma_buffer | 4);
    alsa_dbg("[%s] HwRxDaTCnt [%d]\n", __func__, ((period) >> 0x04) - 1);


    if(chs == 1){
        alsa_dbg("[%s] addr[0x%08X] mono_dma_capture,addr[0x%08X]\n", __func__,addr,mono_dma_capture.addr);    
        pADMA->RxDaDarL = addr; 
        pADMA->RxDaDar = mono_dma_capture.addr;
        pADMA->TransCtrl &= ~(Hw12);        
        pADMA->TransCtrl |= (Hw29 | Hw18 | (Hw13) | (Hw4 | Hw5) ); // MONO
        pADMA->ChCtrl |= Hw18;  // MONO
    }else {
        pADMA->RxDaDar = addr;
        pADMA->TransCtrl |= (Hw29 | Hw18 | (Hw13|Hw12) | (Hw4 | Hw5) );
        pADMA->ChCtrl &= ~(Hw18);  
    }

if(format == SNDRV_PCM_FORMAT_S24_LE){ // Dynamic 16/24bit support
    alsa_dbg("%s SNDRV_PCM_FORMAT_S24_LE foramt %d\n",__func__,format);
    pADMA->ChCtrl &= (~Hw14);
}    
else{
    alsa_dbg("%s SNDRV_PCM_FORMAT_S16_LE foramt %d\n",__func__,format);
    pADMA->ChCtrl |= Hw14;
}    
}

static irqreturn_t tcc_dma_done_handler(int irq, void *dev_id)
{
    volatile PADMA pADMA = (volatile PADMA)tcc_p2v(BASE_ADDR_ADMA0);
    struct snd_pcm_runtime *runtime;
    struct tcc_runtime_data *prtd;
    int dmaInterruptSource = 0;

    unsigned long reg_temp = 0;                 

	//printk("%s %d\n", __func__, __LINE__);

    if (pADMA->IntStatus & (Hw4 | Hw0)) {
        dmaInterruptSource |= DMA_CH_OUT;
        reg_temp |= Hw4 | Hw0;
    }
    if (pADMA->IntStatus & (Hw6 | Hw2)) {
        dmaInterruptSource |= DMA_CH_IN;
        reg_temp |= Hw6 | Hw2;
    }
    if (pADMA->IntStatus & (Hw5 | Hw1)) {
        dmaInterruptSource |= DMA_CH_SPDIF;
        reg_temp |= Hw5 | Hw1;
    }

    if (reg_temp) { 
        pADMA->IntStatus |= reg_temp;
    }

    if ((dmaInterruptSource & DMA_CH_SPDIF)
        && (tcc_alsa_info.flag & TCC_RUNNING_SPDIF)) {

        snd_pcm_period_elapsed(tcc_alsa_info.spdif_ptr);

        runtime = tcc_alsa_info.spdif_ptr->runtime;
        prtd = (struct tcc_runtime_data *)runtime->private_data;
    }

    if ((dmaInterruptSource & DMA_CH_OUT)
        && (tcc_alsa_info.flag & TCC_RUNNING_PLAY)) {
        snd_pcm_period_elapsed(tcc_alsa_info.play_ptr);

        runtime = tcc_alsa_info.play_ptr->runtime;
        prtd = (struct tcc_runtime_data *) runtime->private_data;
    }
    if ((dmaInterruptSource & DMA_CH_IN)
        && (tcc_alsa_info.flag & TCC_RUNNING_CAPTURE)) {
        snd_pcm_period_elapsed(tcc_alsa_info.cap_ptr);

        runtime = tcc_alsa_info.cap_ptr->runtime;
        prtd = (struct tcc_runtime_data *)runtime->private_data;
    }

    return IRQ_HANDLED;
}

#if 0
static dma_addr_t get_dma_addr_offset(dma_addr_t start_addr, dma_addr_t cur_addr, unsigned int total_size, unsigned int dma_len)
{
    if ((start_addr <= cur_addr) && ((start_addr + total_size) > cur_addr)) {
        unsigned long remainder = cur_addr - start_addr;
        return (remainder - (remainder % dma_len));
    }
    return 0;
}
#endif

static int tcc_pcm_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params)
{
    struct snd_pcm_runtime     *runtime = substream->runtime;
    struct tcc_runtime_data    *prtd    = runtime->private_data;
    struct snd_soc_pcm_runtime *rtd     = substream->private_data;
    struct tcc_pcm_dma_params  *dma;

    size_t totsize = params_buffer_bytes(params);
    size_t period_bytes = params_period_bytes(params);
    snd_pcm_format_t format = params_format(params);
    int chs = params_channels(params);
    int ret = 0, mod=0, ttemp=0;

	#if defined(CONFIG_VIDEO_HDMI_IN_SENSOR)
	int HDMI_IN_En = 0;
	#endif

    volatile PADMA    pADMA     = (volatile PADMA)tcc_p2v(BASE_ADDR_ADMA0);
    volatile PADMADAI pADMA_DAI = (volatile PADMADAI)tcc_p2v(BASE_ADDR_DAI0);
	volatile PADMASPDIFTX pADMA_SPDIFTX = (volatile PADMASPDIFTX)tcc_p2v(BASE_ADDR_SPDIFTX0);

    dma = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);

    /* return if this is a bufferless transfer e.g.
     * codec <--> BT codec or GSM modem -- lg FIXME */
    if (!dma) {
        return -EACCES;
    }

    /* this may get called several times by oss emulation
     * with different params */
    if (prtd->params == NULL) {
        prtd->params = dma;
        alsa_dbg(" prtd->params = dma\n");

        mutex_lock(&(tcc_alsa_info.mutex));
        if (!(tcc_alsa_info.flag & TCC_INTERRUPT_REQUESTED)) {
            ret = request_irq(alsa_get_intr_num(substream),
                              tcc_dma_done_handler,
                              IRQF_SHARED,
                              "alsa-audio",
                              &tcc_alsa_info);
            if (ret < 0) {
                mutex_unlock(&(tcc_alsa_info.mutex));
                return -EBUSY;
            }
            tcc_alsa_info.flag |= TCC_INTERRUPT_REQUESTED;
        }
        mutex_unlock(&(tcc_alsa_info.mutex));

        if (substream->pcm->device == __SPDIF_DEV_NUM__) {
            prtd->dma_ch = DMA_CH_SPDIF;
        } else {
            prtd->dma_ch = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? DMA_CH_OUT : DMA_CH_IN;
        }
    } else {
        /* should works more? */
    }        

    //DMA Buffer Control
    //DMA transfer buffer pointer initialization
	//DMA size should be 2^n.
	mod = totsize/min_period_size;
	ttemp = 1;	
	while(1){
		if(mod == 1) break;
		else{
			if((ttemp*2) > mod)break;
			else ttemp = ttemp*2;
		}
	}
	if(mod != ttemp){
		printk("== alsa-debug PCM CH0 == Warning!! buffer_size should be 2^n.\n Your buffer[%d bytes] set [%d bytes] in TCC.\n", totsize, ttemp*min_period_size);
	}
	totsize = ttemp * min_period_size;

    snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);        
    runtime->dma_bytes = totsize;

    prtd->dma_buffer_end = runtime->dma_addr + totsize;
    prtd->dma_buffer = runtime->dma_addr;
    prtd->period_size = period_bytes;
    prtd->nperiod = period_bytes;

    alsa_dbg("buffer bytes=0x%X period bytes=0x%X, rate=%d, chs=%d\n",
             totsize, period_bytes, params_rate(params), params_channels(params));
	alsa_dbg("~~~!!! Substream->pcm->device = %d\n",substream->pcm->device);

    if (substream->pcm->device == __SPDIF_DEV_NUM__) {
        if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
            set_dma_spdif_outbuffer(prtd->dma_buffer, totsize, period_bytes);
        } else {
            alsa_dbg("cannot support S/PDIF capture !!!!!\n");
            return -1;
        }
    } else {
        if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
            set_dma_outbuffer(prtd->dma_buffer, totsize, period_bytes,format,chs);
        } else {
            set_dma_inbuffer(prtd->dma_buffer, totsize, period_bytes,format,chs);
        }
    }

#if defined(CONFIG_VIDEO_HDMI_IN_SENSOR)
	if(params_format(params) == SNDRV_PCM_FORMAT_U16) {
		alsa_dbg("~~~!!! HDMI_IN_Enable !!!~~~\n");
		HDMI_IN_En = 1;
	}
#endif
	if ((substream->pcm->device == __I2S_DEV_NUM__)||(substream->pcm->device == __I2S_SUB_DEV_NUM__)){
		BITCLR(pADMA_DAI->MCCR0, Hw31 | Hw30 | Hw29 | Hw28);
		switch(chs){
			case 8:
			case 7:
				{                                 
					pADMA_DAI->DAMR |= (Hw29 | Hw28);
					BITCLR(pADMA->RptCtrl, (Hw28-Hw24));
					pADMA->RptCtrl  |= (7 << 24);
					if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
						BITCLR(pADMA->ChCtrl, (Hw24 | Hw21 | Hw20));
						pADMA->ChCtrl   |= (Hw24 | Hw21 | Hw20);
					}else{
						BITCLR(pADMA->ChCtrl, (Hw25 | Hw23 | Hw22));			
						pADMA->ChCtrl   |= (Hw25 | Hw23 | Hw22);
					}
				}
				break;
			case 6:
			case 5:
				{                                  
					pADMA_DAI->DAMR |= (Hw29 | Hw28);
					BITCLR(pADMA->RptCtrl, (Hw28-Hw24));
					pADMA->RptCtrl  |= (7 << 24);
					if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
						BITCLR(pADMA->ChCtrl, (Hw24 | Hw21 | Hw20));
						pADMA->ChCtrl   |= (Hw24 | Hw21);
					}else{
						BITCLR(pADMA->ChCtrl, (Hw25 | Hw23 | Hw22));			
						pADMA->ChCtrl   |= (Hw25 | Hw22);
					}
				}
				break;
			case 4:
			case 3:
				{                                  
					pADMA_DAI->DAMR |= (Hw29 | Hw28);
					BITCLR(pADMA->RptCtrl, (Hw28-Hw24));
					pADMA->RptCtrl  |= (7 << 24);
					if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
						BITCLR(pADMA->ChCtrl, (Hw24 | Hw21 | Hw20));
					}else{
						BITCLR(pADMA->ChCtrl, (Hw25 | Hw23 | Hw22));			
					}
				}
				break;
			case 2:
			case 1:
				{
					BITSET(pADMA_DAI->DAMR, (Hw29));
					BITCLR(pADMA->RptCtrl, (Hw28-Hw24));
					pADMA->RptCtrl  |= (7 << 24);
					BITCLR(pADMA_DAI->DAMR, (Hw28));
					if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
						BITCLR(pADMA->ChCtrl,(Hw24 | Hw21 | Hw20));
						pADMA->ChCtrl   |= (Hw21 | Hw20);
					}else{
						BITCLR(pADMA->ChCtrl, (Hw25 | Hw23 | Hw22));			
						pADMA->ChCtrl   |= (Hw23 | Hw22);
					}
#if defined(CONFIG_VIDEO_HDMI_IN_SENSOR)
					if(HDMI_IN_En) {
						printk("~~~!!! HDMI In Setting Enable !!!~~~\n");
						BITCLR(pADMA_DAI->DAMR, (Hw11 | Hw10 | Hw9));
					} else {
						printk("~~~!!! HDMI In Setting Disable !!!~~~\n");
						BITSET(pADMA_DAI->DAMR, (Hw11 | Hw10 | Hw9));
					}
#endif
				}
				break;
			default:
				printk("ERROR!! %d channel is not supported!\n", chs);
		}
    }
    else
    {
//        BITCLR(pADMA->RptCtrl, (Hw26 | Hw25 | Hw24));
//        BITCLR(pADMA->ChCtrl,(Hw24 | Hw21 | Hw20));
    }

	if (substream->pcm->device == __SPDIF_DEV_NUM__) {
		//Set Audio or Data Format
		alsa_dbg("%s : runtime->format11 = %d\n", __func__, params_format(params));

		if (params_format(params) == SNDRV_PCM_FORMAT_U16) {
			BITSET(pADMA_SPDIFTX->TxChStat, Hw0);	//Data format
			alsa_dbg("%s : set SPDIF TX Channel STATUS to Data format \n",__func__);
		}
		else {
			BITCLR(pADMA_SPDIFTX->TxChStat, Hw0);	//Audio format
			alsa_dbg("%s : set SPDIF TX Channel STATUS to PCM format \n",__func__);
		}
	}


    return ret;
}

static int tcc_pcm_hw_free(struct snd_pcm_substream *substream)
{
    struct tcc_runtime_data *prtd = substream->runtime->private_data;

    alsa_dbg("[%s] \n", __func__);

    if (prtd->dma_ch) {
        snd_pcm_set_runtime_buffer(substream, NULL);
    }

    return 0;
}

static int tcc_pcm_prepare(struct snd_pcm_substream *substream)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    alsa_dbg("[%s] \n", __func__);

    //DMA initialize
    memset(runtime->dma_area, 0x00, runtime->dma_bytes);
    alsa_dbg(" %s %d dma_bytes -> 0 set Done\n",__func__,runtime->dma_bytes);
    return 0;
}

static int tcc_i2s_tx_enable(int En)
{
	volatile PADMADAI pDAI  = (volatile PADMADAI)tcc_p2v(BASE_ADDR_DAI0);
	volatile PADMA    pADMA = (volatile PADMA)tcc_p2v(BASE_ADDR_ADMA0);
    unsigned int ttemp=0;
    unsigned int ret=0;

    if(En)
    {
        BITSET(pADMA->ChCtrl, Hw0);
        BITSET(pADMA->ChCtrl, Hw28);
        pDAI->DAMR |= Hw14;

        pDAI->DAVC = 0; // Mute OFF
    }
    else
    {
        pDAI->DAVC = 0x10; /* MUTE */
        pDAI->DAMR &= ~Hw14;   /* STOP I2S-Tx */
        ttemp = pADMA->ChCtrl & Hw29;
        if(!ttemp)
            BITCLR(pADMA->ChCtrl, Hw28); /* STOP ADMA */
        BITCLR(pADMA->ChCtrl, Hw0); /* We must have 1 mdelay for DTIEN */
        mdelay(1);
#if 0
        pDAI->DRMR |= Hw20;   /* STOP I2S-Tx */
        mdelay(1);
        pDAI->DRMR &= ~Hw20;   /* STOP I2S-Tx */
        BITSET(pADMA->ADMARST, Hw0);
        mdelay(1);
        BITCLR(pADMA->ADMARST, Hw0);
#else
//        tca_adma_dump((void*)pADMA);
//        tca_dai_dump((void*)pDAI);
#endif
    }
    return ret;
}

static int tcc_i2s_rx_enable(int En)
{
	volatile PADMADAI pDAI  = (volatile PADMADAI)tcc_p2v(BASE_ADDR_DAI0);
	volatile PADMA    pADMA = (volatile PADMA)tcc_p2v(BASE_ADDR_ADMA0);
    unsigned int ttemp=0;
    unsigned int ret=0;

    if(En)
    {
        //pDAI->DAMR |= Hw17;	
		//This is loop-back setting for Debug. Direction: TX->RX
        BITSET(pADMA->ChCtrl, Hw2);
        BITSET(pADMA->ChCtrl, Hw30);

        pDAI->DAMR |= Hw13;
    }
    else
    {

        pDAI->DAMR &= ~Hw13;   /* STOP I2S-Tx */
        ttemp = pADMA->ChCtrl & Hw31;
        if(!ttemp)
            BITCLR(pADMA->ChCtrl, Hw30); /* STOP ADMA */
        BITCLR(pADMA->ChCtrl, Hw2); /* We must have 1 mdelay for RTIEN */
        mdelay(1);
#if 0
        pDAI->DRMR |= Hw21;   /* STOP I2S-Tx */
        mdelay(1);
        pDAI->DRMR &= ~Hw21;   /* STOP I2S-Tx */
        BITSET(pADMA->ADMARST, Hw1);
        mdelay(1);
        BITCLR(pADMA->ADMARST, Hw1);
#else
        //      tca_adma_dump((void*)pADMA);
        //      tca_dai_dump((void*)pDAI);
#endif

    }
    return ret;
}


static int tcc_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	unsigned int ttemp=0;
	volatile PADMASPDIFTX pADMASPDIFTX = (volatile PADMASPDIFTX)tcc_p2v(BASE_ADDR_SPDIFTX0);
	volatile PADMA    pADMA = (volatile PADMA)tcc_p2v(BASE_ADDR_ADMA0);
	int ret = 0;
    alsa_dbg("%s() cmd[%d] frame_bits[%d]\n", __func__, cmd, substream->runtime->frame_bits);

    switch (cmd) {
    case SNDRV_PCM_TRIGGER_START:
    case SNDRV_PCM_TRIGGER_RESUME:        
    case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
        if (substream->pcm->device == __SPDIF_DEV_NUM__) {
// SPDIF_24BIT_MODE Planet_20140408
                alsa_dbg("%s() spdif playback start\n", __func__);
#ifdef SPDIF_24BIT_MODE
			pADMASPDIFTX->TxConfig |= HwZERO | Hw0 | Hw2 | Hw1 | Hw23;
#else
			pADMASPDIFTX->TxConfig |= HwZERO | Hw0 | Hw2 | Hw1;
#endif
			ttemp = pADMA->ChCtrl & Hw28;
			if(!ttemp)
				BITSET(pADMA->ChCtrl, Hw28);
			BITSET(pADMA->ChCtrl, Hw1 | Hw29);
		}  else {

            if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {

					ret = tcc_i2s_tx_enable(1);
					if(ret) alsa_dbg("%s() playback start has some error.\n", __func__);
					else alsa_dbg("%s() playback start\n", __func__);

            } else {

					ret = tcc_i2s_rx_enable(1);
					if(ret) alsa_dbg("%s() capture start has some error.\n", __func__);
					else alsa_dbg("%s() capture start\n", __func__);

            }
        }
        break;

    case SNDRV_PCM_TRIGGER_STOP:
    case SNDRV_PCM_TRIGGER_SUSPEND:
    case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
        if (substream->pcm->device == __SPDIF_DEV_NUM__) {
				alsa_dbg("%s() spdif playback end\n", __func__);
			// SPDIF_24BIT_MODE Planet_20140408
#ifdef SPDIF_24BIT_MODE
			pADMASPDIFTX->TxConfig &= ~(Hw0 | Hw2 | Hw1 | Hw23);
#else
			pADMASPDIFTX->TxConfig &= ~(Hw0 | Hw2 | Hw1);
#endif
			BITCLR(pADMA->ChCtrl, Hw1 | Hw29);
			ttemp = pADMA->ChCtrl & Hw0;
			if(!ttemp)
				BITCLR(pADMA->ChCtrl, Hw28); /* STOP ADMA */
        } else {
            if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {

				ret = tcc_i2s_tx_enable(0);
				if(ret) alsa_dbg("%s() playback end has some error.\n", __func__);
				else alsa_dbg("%s() playback end\n", __func__);

			} else {

				ret = tcc_i2s_rx_enable(0);
				if(ret) alsa_dbg("%s() capture end has some error.\n", __func__);
				else alsa_dbg("%s() capture end\n", __func__);

 			}
        }
        break;

    default:
        ret = -EINVAL;
    }

    return ret;
}


//buffer point update
//current position   range 0-(buffer_size-1)
static snd_pcm_uframes_t tcc_pcm_pointer(struct snd_pcm_substream *substream)
{
    volatile PADMA pADMA = (volatile PADMA)tcc_p2v(BASE_ADDR_ADMA0);
    struct snd_pcm_runtime *runtime = substream->runtime;
    snd_pcm_uframes_t x;
    dma_addr_t ptr = 0;
    int chs = runtime->channels;

    //alsa_dbg(" %s \n", __func__);

    if (substream->pcm->device == __SPDIF_DEV_NUM__) {
        ptr = pADMA->TxSpCsar;
    } else {
        if(chs == 1){
            ptr = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? pADMA->TxDaCsarL : pADMA->RxDaCdarL; // MONO
        }
        else
            ptr = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? pADMA->TxDaCsar : pADMA->RxDaCdar;

    }

    x = bytes_to_frames(runtime, ptr - runtime->dma_addr);

    if (x < runtime->buffer_size) {
        return x;
    }
    return 0;
}

static int tcc_pcm_open(struct snd_pcm_substream *substream)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct tcc_runtime_data *prtd;
    int ret;

    alsa_dbg("[%s] open %s device, %s\n", __func__, 
										substream->pcm->device == __SPDIF_DEV_NUM__ ? "spdif":"pcm", 
										substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? "output" : "input");

    mutex_lock(&(tcc_alsa_info.mutex));
    if (substream->pcm->device == __SPDIF_DEV_NUM__) {
        if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
            tcc_alsa_info.flag |= TCC_RUNNING_SPDIF;
            tcc_alsa_info.spdif_ptr = substream;
            snd_soc_set_runtime_hwparams(substream, &tcc_pcm_hardware_play);
            alsa_dbg("[%s] open spdif device\n", __func__);
        } else {
            alsa_dbg("cannot support S/PDIF capture !!!!!\n");
            return -1;
        }
    } else {
        if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {  
            tcc_alsa_info.flag |= TCC_RUNNING_PLAY;
            tcc_alsa_info.play_ptr = substream;
            snd_soc_set_runtime_hwparams(substream, &tcc_pcm_hardware_play);
        } else {
            tcc_alsa_info.flag |= TCC_RUNNING_CAPTURE;
            tcc_alsa_info.cap_ptr = substream;
            snd_soc_set_runtime_hwparams(substream, &tcc_pcm_hardware_capture);
        }
    }
    mutex_unlock(&(tcc_alsa_info.mutex));

    prtd = kzalloc(sizeof(struct tcc_runtime_data), GFP_KERNEL);

    if (prtd == NULL) {
        ret = -ENOMEM;
        goto out;
    }
    runtime->private_data = prtd;

    return 0;
    kfree(prtd);
out:
    return ret;
}

static int tcc_pcm_close(struct snd_pcm_substream *substream)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct tcc_runtime_data *prtd = runtime->private_data;
    volatile PADMADAI pDAI = (volatile PADMADAI)tcc_p2v(BASE_ADDR_DAI0);
    volatile PADMASPDIFTX pADMASPDIFTX = (volatile PADMASPDIFTX)tcc_p2v(BASE_ADDR_SPDIFTX0);

    alsa_dbg("[%s] close %s device, %s\n", __func__, substream->pcm->device == __SPDIF_DEV_NUM__ ? "spdif":"pcm", 
													substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? "output" : "input");

    mutex_lock(&(tcc_alsa_info.mutex));
    if (substream->pcm->device == __SPDIF_DEV_NUM__) {
        tcc_alsa_info.flag &= ~TCC_RUNNING_SPDIF;
        //tca_i2s_stop(pDAI, pADMASPDIFTX, 0);
        pADMASPDIFTX->TxConfig &= ~Hw0;
        alsa_dbg("[%s] close spdif device\n", __func__);
    } else {
        if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
            tcc_alsa_info.flag &= ~TCC_RUNNING_PLAY;
            //tca_i2s_stop(pDAI, pADMASPDIFTX, 0);
            pDAI->DAMR &= ~Hw14;
        } else {
            tcc_alsa_info.flag &= ~TCC_RUNNING_CAPTURE;
            //tca_i2s_stop(pDAI, pADMASPDIFTX, 1);
            pDAI->DAMR &= ~Hw13;
        }
    }
    // dma_free_writecombine(substream->pcm->card->dev, PAGE_SIZE,); 

    if (prtd) {
        kfree(prtd);
    }

    if (tcc_alsa_info.flag & TCC_INTERRUPT_REQUESTED) {
        if (!(tcc_alsa_info.flag & (TCC_RUNNING_SPDIF | TCC_RUNNING_PLAY | TCC_RUNNING_CAPTURE))) {
            free_irq(alsa_get_intr_num(substream), &tcc_alsa_info);
            tcc_alsa_info.flag &= ~TCC_INTERRUPT_REQUESTED;
        }
    }
    mutex_unlock(&(tcc_alsa_info.mutex));

    return 0;
}

static int tcc_pcm_mmap(struct snd_pcm_substream *substream, struct vm_area_struct *vma)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    return dma_mmap_writecombine(substream->pcm->card->dev, vma,
        runtime->dma_area,
        runtime->dma_addr,
        runtime->dma_bytes);
}

static struct snd_pcm_ops tcc_pcm_ops = {
    .open  = tcc_pcm_open,
    .close  = tcc_pcm_close,
    .ioctl  = snd_pcm_lib_ioctl,
    .hw_params = tcc_pcm_hw_params,
    .hw_free = tcc_pcm_hw_free,
    .prepare = tcc_pcm_prepare,
    .trigger = tcc_pcm_trigger,
    .pointer = tcc_pcm_pointer,
    .mmap = tcc_pcm_mmap,
};

static int tcc_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
    struct snd_pcm_substream *substream = pcm->streams[stream].substream;
    struct snd_dma_buffer *buf = &substream->dma_buffer;
    size_t size = 0;

    buf->dev.type = SNDRV_DMA_TYPE_DEV;
    buf->dev.dev = pcm->card->dev;
    buf->private_data = NULL;

    if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
        mono_dma_play.dev.type = SNDRV_DMA_TYPE_DEV;
        mono_dma_play.dev.dev = 0;// pcm->card->dev;
        mono_dma_play.private_data = NULL;

        size = tcc_pcm_hardware_play.buffer_bytes_max;
        if(tcc_pcm_hardware_play.channels_min == 1){
            mono_dma_play.area = dma_alloc_writecombine(mono_dma_play.dev.dev, size, &mono_dma_play.addr, GFP_KERNEL);
            if (!mono_dma_play.area || !mono_dma_play.addr) {
                alsa_dbg("%s ERROR mono_dma_play dma_alloc_writecombine [%d]\n",__func__,size);
                return -ENOMEM;
            }
            mono_dma_play.bytes = size;
            alsa_dbg("mono_dma_play size [%d]\n", size);
        }

    } else {
        size = tcc_pcm_hardware_capture.buffer_bytes_max;
        if(tcc_pcm_hardware_capture.channels_min == 1){
            mono_dma_capture.dev.type =  SNDRV_DMA_TYPE_DEV;
            mono_dma_capture.dev.dev = 0;// pcm->card->dev;
            mono_dma_capture.private_data =  NULL;
           
            mono_dma_capture.area = dma_alloc_writecombine(mono_dma_capture.dev.dev, size, &mono_dma_capture.addr, GFP_KERNEL);    
            if ( !mono_dma_capture.area || !mono_dma_capture.addr) {
                alsa_dbg("%s ERROR  mono_dma_capture dma_alloc_writecombine [%d]\n",__func__,size);
                return -ENOMEM;
            }
            mono_dma_capture.bytes = size;            
            alsa_dbg("mono_dma_capture size [%d]\n", size);
        }
    }

    buf->area = dma_alloc_writecombine(pcm->card->dev, size, &buf->addr, GFP_KERNEL);
    if (!buf->area || !buf->addr ) {
        alsa_dbg("%s ERROR dma_alloc_writecombine [%d]\n",__func__,size);
        return -ENOMEM;
    }
    else
        alsa_dbg("tcc_pcm_preallocate_dma_buffer size [%d] addr 0x%x, area 0x%x\n", size,buf->area,buf->addr);

    buf->bytes = size;

    return 0;
}


static void tcc_pcm_free_dma_buffers(struct snd_pcm *pcm)
{
    struct snd_pcm_substream *substream;
    struct snd_dma_buffer *buf;
    int stream;

    alsa_dbg(" %s \n", __func__);

    for (stream = 0; stream < 2; stream++) {
        substream = pcm->streams[stream].substream;
        if (!substream) { continue; }

        buf = &substream->dma_buffer;
        if (!buf->area) { continue; }

        dma_free_writecombine(pcm->card->dev, buf->bytes, buf->area, buf->addr);
        buf->area = NULL;
    }

    if(mono_dma_play.area != 0 ){
        dma_free_writecombine(mono_dma_play.dev.dev, mono_dma_play.bytes, mono_dma_play.area, mono_dma_play.addr);
        alsa_dbg("%s mono_dma_play.area Free \n", __func__);
        mono_dma_play.area = 0;
    }

    if(mono_dma_capture.area != 0 ){
        dma_free_writecombine(mono_dma_capture.dev.dev, mono_dma_capture.bytes, mono_dma_capture.area, mono_dma_capture.addr);
        alsa_dbg("%s mono_dma_capture.area Free \n", __func__);
        mono_dma_capture.area = 0;
    }
    
    mutex_init(&(tcc_alsa_info.mutex));
}

static u64 tcc_pcm_dmamask = DMA_BIT_MASK(32);

static int tcc_pcm_new(struct snd_soc_pcm_runtime *rtd) 
{
    struct snd_card *card   = rtd->card->snd_card;
    struct snd_soc_dai *dai = rtd->cpu_dai;
    struct snd_pcm *pcm     = rtd->pcm;
    int ret = 0;

    alsa_dbg("[%s] \n", __func__);

    memset(&tcc_alsa_info, 0, sizeof(struct tcc_interrupt_info_x));
    mutex_init(&(tcc_alsa_info.mutex));

    if (!card->dev->dma_mask) {
        card->dev->dma_mask = &tcc_pcm_dmamask;
    }
    if (!card->dev->coherent_dma_mask) {
        card->dev->coherent_dma_mask = DMA_BIT_MASK(32);
    }
    if (dai->driver->playback.channels_min) {
        ret = tcc_pcm_preallocate_dma_buffer(pcm, SNDRV_PCM_STREAM_PLAYBACK);
        if (ret) { goto out; }
    }

    if (dai->driver->capture.channels_min) {
        ret = tcc_pcm_preallocate_dma_buffer(pcm, SNDRV_PCM_STREAM_CAPTURE);
        if (ret) { goto out; }
    } 
	{
        volatile ADMASPDIFTX *pADMASPDIFTX = (volatile ADMASPDIFTX *)tcc_p2v(BASE_ADDR_SPDIFTX0);

        memset((void *)pADMASPDIFTX->UserData, 0, 24);
        memset((void *)pADMASPDIFTX->ChStatus, 0, 24);
        memset((void *)pADMASPDIFTX->TxBuffer, 0, 128);

        //unsigned int bitSet = 0x0;

        pADMASPDIFTX->TxConfig |= HwZERO
            //16bits 
            //|Hw8 //Clock Divider Ratio is 2(384*fs to default) 		TCC8935 Style
            //| Hw7 | Hw6 //User Data Enalbe Bits is set reserved
            //| Hw5 | Hw4 //Channel Status Enalbe Bits is set reserved
            //| Hw2 //interrupt output enable
            //| Hw1 //data being valid									TCC8935 Style
            //| Hw0 //enable the transmission
            ;
        //Set SPDIF Transmit Channel Status
        pADMASPDIFTX->TxChStat |= HwZERO
            //| 0x00000400 // Store Channel Status bit
            //| 0x00000200 // Store User Data bit
            //| 0x00000100 // Store Validity bit
            //44.1kHz
            //| Hw3 //Original/Commercially Pre-recored data
            //| Hw2 //Pre-emphasis is 50/15us
            //| Hw1 //Copy permitted
            //Audio format
            ;		

        //pADMASPDIFTX->TxIntMask = HwZERO
        //	| Hw2//Higher Data buffer is empty
        //	| Hw1//Lower Data buffer is empty
        //	;

        pADMASPDIFTX->DMACFG = Hw7;
        msleep(100);

        pADMASPDIFTX->DMACFG = HwZERO
            //| Hw14	//Swap Sample Enable
            | Hw13 	//Read Address 16bit Mode
            | Hw11	//DMA Request enable for user data buffer
            | Hw10	//DMA Request enable for sample data buffer
            //| Hw8	//Enable address
            //| Hw3	//FIFO Threshold for DMA request
            | Hw1| Hw0	// [3:0] FIFO Threshold for DMA Request
            ;

        /* Initialize Sample Data Buffer */
        //while(pADMASPDIFTX->DMACFG & 0x00300000) pADMASPDIFTX->TxBuffer[0] = 0;
        pADMASPDIFTX->TxBuffer[0] = 0;

        pADMASPDIFTX->TxConfig |= HwZERO
            //| Hw2 //interrupt output enable
            //| Hw1 //data being valid				TCC8935 Style
            //| Hw0 //enable the transmission		TCC8935 Style
            ;

        pADMASPDIFTX->TxIntStat = 0x1E; /* Clear Interrupt Status */
    }

out:
    return ret;
}


#if defined(CONFIG_PM)
static int tcc_pcm_suspend(struct snd_soc_dai *dai)
{
    volatile PADMA pADMA = (volatile PADMA)tcc_p2v(BASE_ADDR_ADMA0);

    alsa_dbg(" %s \n", __func__);
    gADMA.TransCtrl = pADMA->TransCtrl;
    gADMA.RptCtrl   = pADMA->RptCtrl;
    gADMA.ChCtrl    = pADMA->ChCtrl;
    gADMA.GIntReq   = pADMA->GIntReq;

    return 0;
}

static int tcc_pcm_resume(struct snd_soc_dai *dai)
{
    volatile PADMA pADMA = (volatile PADMA)tcc_p2v(BASE_ADDR_ADMA0);

    alsa_dbg(" %s \n", __func__);
    pADMA->TransCtrl = gADMA.TransCtrl;
    pADMA->RptCtrl   = gADMA.RptCtrl;
    pADMA->ChCtrl    = gADMA.ChCtrl;
    pADMA->GIntReq   = gADMA.GIntReq;

    return 0;
}
#endif

static struct snd_soc_platform_driver tcc_soc_platform = {
    .ops      = &tcc_pcm_ops,
    .pcm_new  = tcc_pcm_new,
    .pcm_free = tcc_pcm_free_dma_buffers,
#if defined(CONFIG_PM)
    .suspend  = tcc_pcm_suspend,
    .resume   = tcc_pcm_resume,
#endif
};

int tcc_pcm_platform_register(struct device *dev)
{
	return snd_soc_register_platform(dev, &tcc_soc_platform);
}
EXPORT_SYMBOL_GPL(tcc_pcm_platform_register);

void tcc_pcm_platform_unregister(struct device *dev)
{
	return snd_soc_unregister_platform(dev);
}
EXPORT_SYMBOL_GPL(tcc_pcm_platform_unregister);

MODULE_DESCRIPTION("Telechips PCM ASoC driver");
MODULE_LICENSE("GPL");

