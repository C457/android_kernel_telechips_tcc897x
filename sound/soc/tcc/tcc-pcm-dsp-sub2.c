/*
 * linux/sound/soc/tcc/tcc-pcm-dsp-ch1.c  
 *
 * Based on:    linux/sound/arm/pxa2xx-pcm.c
 * Author:  <linux@telechips.com>
 * Created: Nov 30, 2004
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

//#include <mach/hardware.h>
//#include <mach/bsp.h>
//#include <mach/tca_i2s.h>

//#include "tcc-i2s.h"
#include "tcc-pcm.h"

#include "tcc/tca_tcchwcontrol.h"

#include <mach/tcc-dsp-api.h>
#include "../../../drivers/misc/tcc/tcc_dsp.h"
#include "../../../drivers/misc/tcc/tcc_dsp_ipc.h"


#undef alsa_dbg
#if 0
#define alsa_dbg(f, a...)  printk("== alsa-debug PCM CH2 == " f, ##a)
#else
#define alsa_dbg(f, a...)
#endif

#if 0	// Android Style
#define __play_buf_size 65536
#define __play_buf_cnt  8
#define __cap_buf_size 8192
#define __cap_buf_cnt  8
#else  // LinuxAVN TCC8935 Style
#define __play_buf_size 16384
#define __play_buf_cnt  4
#define __cap_buf_size 4096
#define __cap_buf_cnt  16
#endif
#define min_period_size 128

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
    .rates        = SNDRV_PCM_RATE_8000_96000,
    .rate_min     = 8000,
    .rate_max     = 96000,
    .channels_min = 1,
    .channels_max = 2,
    .period_bytes_min = min_period_size,
    .period_bytes_max = __play_buf_size,
    .periods_min      = 2,
#if 0 // Android Style	
    .periods_max      = __play_buf_cnt,
    .buffer_bytes_max = __play_buf_cnt * __play_buf_size,
#else  // LinuxAVN TCC8935 Style
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

	.formats      = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
	.rates        = SNDRV_PCM_RATE_8000_96000,
    .rate_min     = 8000,
    .rate_max     = 96000,
    .channels_min = 1,
    .channels_max = 4,

    .period_bytes_min = min_period_size,
    .period_bytes_max = __cap_buf_size,
    .periods_min      = 2,
#if 0 // Android Style		
    .periods_max      = __cap_buf_cnt,
    .buffer_bytes_max = __cap_buf_cnt * __cap_buf_size,
#else  // LinuxAVN TCC8935 Style	
    .periods_max      = (__cap_buf_cnt * __cap_buf_size)/min_period_size,
    .buffer_bytes_max = __cap_buf_cnt * __cap_buf_size,
#endif	
    .fifo_size = 16, //ignored
};

static void set_dma_outbuffer(unsigned int addr, unsigned int length,unsigned int period,snd_pcm_format_t format,int chs)
{

    unsigned long dma_buffer = 0;
	int bit_count;
    unsigned int mod,div;

//    BITCLR(pADMA->ChCtrl, Hw28); Stop Output

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

    mod = length%period;
    if(mod){
        div = length/period;
        if(div < 4){
            period = length/4;
        }
        else{
            period = length/div;
        }
    }
    
    dma_buffer = 0xFFFFFF00 & (~((length<<4)-1));

//    pADMA->TxDaParam = dma_buffer | 4;
//    pADMA->TxDaTCnt = (period >> 0x05) - 1;

    alsa_dbg("[%s], addr[0x%08X], len[%u], period[%u]\n", __func__, addr, length, period);
    alsa_dbg("[%s] HwTxDaParam [0x%X]\n", __func__, (int)(dma_buffer | 4));
    alsa_dbg("[%s] HwTxDaTCnt [%d]\n", __func__, ((period) >> 0x05) - 1);

    if(chs == 1){
        alsa_dbg("[%s] addr[0x%08X] mono_dma_play,addr[0x%08X]\n", __func__,addr,mono_dma_play.addr);    
//        pADMA->TxDaSarL = addr; 
//        pADMA->TxDaSar = mono_dma_play.addr;
    }else {
//        pADMA->TxDaSar = addr;
    }

    if(format == SNDRV_PCM_FORMAT_S24_LE){ //I2S_24BIT_MODE
        alsa_dbg("%s SNDRV_PCM_FORMAT_S24_LE foramt %d\n",__func__,format);
    }        
    else{
        alsa_dbg("%s SNDRV_PCM_FORMAT_S16_LE foramt %d\n",__func__,format);
    }

}

static void set_dma_spdif_outbuffer(unsigned int addr, unsigned int length, unsigned int period)
{
 
}

static void set_dma_inbuffer(unsigned int addr, unsigned int length, unsigned int period,snd_pcm_format_t format,int chs)
{
    unsigned long dma_buffer = 0;
	int bit_count;
    unsigned int mod,div;


//    BITCLR(pADMA->ChCtrl, Hw30); Stop Input


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
	
    mod = length%period;
    if(mod){
        div = length/period;
        if(div < 4){
            period = length/4;
        }
        else{
            period = length/div;
        }
    }
    
    dma_buffer = 0xFFFFFF00 & (~((length<<4)-1));

//    pADMA->RxDaParam = dma_buffer | 4;
//    pADMA->RxDaTCnt = (period >> 0x05) - 1;

    alsa_dbg("[%s], addr[0x%08X], len[%d]\n", __func__, addr, length);
    alsa_dbg("[%s] HwRxDaParam [0x%X]\n", __func__, (int)dma_buffer | 4);
    alsa_dbg("[%s] HwRxDaTCnt [%d]\n", __func__, ((period) >> 0x04) - 1);


    if(chs == 1){
        alsa_dbg("[%s] addr[0x%08X] mono_dma_capture,addr[0x%08X]\n", __func__,addr,mono_dma_capture.addr);    
        // Set mono~~~~~~
    }else {
        // Set Streo~~~~~~
    }

    if(format == SNDRV_PCM_FORMAT_S24_LE){ // Dynamic 16/24bit support
        alsa_dbg("%s SNDRV_PCM_FORMAT_S24_LE foramt %d\n",__func__,format);
//        pADMA->ChCtrl &= (~Hw14);
    }    
    else{
        alsa_dbg("%s SNDRV_PCM_FORMAT_S16_LE foramt %d\n",__func__,format);
//        pADMA->ChCtrl |= Hw14;
    }    
}

//static irqreturn_t tcc_dma_done_handler(int irq, void *dev_id)
void tcc_dma2_done_handler(int playback/*int irq, void *dev_id*/)
{
    struct snd_pcm_runtime *runtime;
    struct tcc_runtime_data *prtd;
    int dmaInterruptSource = 0;

    // Interrupt Done~~~~!

    //
    
    	if(playback)
		dmaInterruptSource |= DMA_CH_OUT;
	else
		dmaInterruptSource |= DMA_CH_IN;


    if ((dmaInterruptSource & DMA_CH_SPDIF)
        && (tcc_alsa_info.flag & TCC_RUNNING_SPDIF_PLAY)) {

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

 //   return IRQ_HANDLED;
}

static int tcc_pcm_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params)
{
    struct snd_pcm_runtime     *runtime = substream->runtime;
    struct tcc_runtime_data    *prtd    = runtime->private_data;
    struct snd_soc_pcm_runtime *rtd     = substream->private_data;
    struct tcc_pcm_dma_params  *dma;

    size_t totsize = params_buffer_bytes(params);
    size_t period = params_period_bytes(params); 
    size_t period_bytes = params_period_bytes(params);
    snd_pcm_format_t format = params_format(params);

    int chs = params_channels(params);
    int ret = 0;
	#if defined(CONFIG_VIDEO_HDMI_IN_SENSOR)
	int HDMI_IN_En = 0;
	#endif

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
            ret = 1;
            if (ret < 0) {
                mutex_unlock(&(tcc_alsa_info.mutex));
                return -EBUSY;
            }
            tcc_alsa_info.flag |= TCC_INTERRUPT_REQUESTED;
        }
        mutex_unlock(&(tcc_alsa_info.mutex));

        prtd->dma_ch = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? DMA_CH_OUT : DMA_CH_IN;

    } else {
        /* should works more? */
    }        

    //DMA ???? ??Ʈ??
    //DMA ???? ???? ?????? ?ʱ?ȭ 

    snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);        
    runtime->dma_bytes = totsize;

    prtd->dma_buffer_end = runtime->dma_addr + totsize;
    prtd->dma_buffer = runtime->dma_addr;
    prtd->period_size = period_bytes;
    prtd->nperiod = period;

    alsa_dbg("totsize=0x%X period=0x%X  period num=%d, rate=%d, chs=%d\n",
             totsize, period_bytes, period, params_rate(params), params_channels(params));


    if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
	set_dma_outbuffer(prtd->dma_buffer, totsize, period_bytes,format,chs);
	tc_dsp_setAudioBuffer(I2S2params,(unsigned long)prtd->dma_buffer,totsize,period_bytes,1);
	tc_dsp_setPCMChannelNum(I2S2params, chs);
    } else {
	set_dma_inbuffer(prtd->dma_buffer, totsize, period_bytes,format,chs);
	tc_dsp_setAudioBuffer(I2S2params,(unsigned long)prtd->dma_buffer,totsize,period_bytes,0);
	tc_dsp_setPCMChannelNum(I2S2params, chs);
    }

    tc_dsp_setI2sMasterMode(I2S2params,0);
    tc_dsp_setI2sFormat(I2S2params, format /*SNDRV_PCM_FORMAT_S24_LE*/);
    tc_dsp_setI2sSamplerate(I2S2params, params_rate(params));
    tc_dsp_setI2sChannelNum(I2S2params,chs );
    
    if (chs > 2) {                           
         // 5.1ch or 7.1ch
    }else {               
        // 2 ch
    }

    return ret;
}

static int tcc_pcm_hw_free(struct snd_pcm_substream *substream)
{
    struct tcc_runtime_data *prtd = substream->runtime->private_data;

    alsa_dbg("[%s] \n", __func__);

    if (prtd->dma_ch) {
        snd_pcm_set_runtime_buffer(substream, NULL);
        //?߰? ?ʿ?  ??
    }

    return 0;
}

static int tcc_pcm_prepare(struct snd_pcm_substream *substream)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    alsa_dbg("[%s] \n", __func__);

    //DMA initialize

    /* Disable Noise */
    //memset((void *)(&HwDADO_L0), 0, 32);
    memset(runtime->dma_area, 0x00, runtime->dma_bytes);
    alsa_dbg(" %s %d dma_bytes -> 0 set Done\n",__func__,runtime->dma_bytes);
    return 0;
}

static int tcc_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
    int ret = 0;

    alsa_dbg("%s() cmd[%d] frame_bits[%d]\n", __func__, cmd, substream->runtime->frame_bits);
    switch (cmd) {
    case SNDRV_PCM_TRIGGER_START:
    case SNDRV_PCM_TRIGGER_RESUME:        
    case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:

        if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
            alsa_dbg("%s() playback start\n", __func__);
            // Start ~~~~~~~~~~~~ ?AA7s
            tc_dsp_DoPlayback(I2S2params,TCC_DSP_IPC_ID_ALSA_SUB02);

        } else {

        	alsa_dbg("%s() recording start\n", __func__);

        }

        break;

    case SNDRV_PCM_TRIGGER_STOP:
    case SNDRV_PCM_TRIGGER_SUSPEND:
    case SNDRV_PCM_TRIGGER_PAUSE_PUSH:

        if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			alsa_dbg("%s() playback end\n", __func__);
            tc_dsp_DoStop(I2S2params,TCC_DSP_IPC_ID_ALSA_SUB02);


        } else {
        	alsa_dbg("%s() recording end\n", __func__);
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
	struct snd_pcm_runtime *runtime = substream->runtime;
	//    struct tcc_runtime_data    *prtd    = runtime->private_data;

	snd_pcm_uframes_t x;
	dma_addr_t ptr = 0;
	int chs = runtime->channels;

	//   alsa_dbg(" %s \n", __func__);

	if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		tc_dsp_getCurrAudioBuffer(I2S2params, (unsigned long*)&ptr,1);
	else
		tc_dsp_getCurrAudioBuffer(I2S2params, (unsigned long*)&ptr,0);
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

    alsa_dbg("[pcm] open %s device, %s\n", __func__, 
										substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? "output" : "input");

    mutex_lock(&(tcc_alsa_info.mutex));
    if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {  
        tcc_alsa_info.flag |= TCC_RUNNING_PLAY;
        tcc_alsa_info.play_ptr = substream;
        snd_soc_set_runtime_hwparams(substream, &tcc_pcm_hardware_play);
    } else {
        tcc_alsa_info.flag |= TCC_RUNNING_CAPTURE;
        tcc_alsa_info.cap_ptr = substream;
        snd_soc_set_runtime_hwparams(substream, &tcc_pcm_hardware_capture);
    }

    mutex_unlock(&(tcc_alsa_info.mutex));

    prtd = kzalloc(sizeof(struct tcc_runtime_data), GFP_KERNEL);

    if (prtd == NULL) {
        ret = -ENOMEM;
        goto out;
    }
    runtime->private_data = prtd;

    tc_dsp_setI2sNum(I2S2params,I2S2params);

    return 0;
    kfree(prtd);
out:
    return ret;
}

static int tcc_pcm_close(struct snd_pcm_substream *substream)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct tcc_runtime_data *prtd = runtime->private_data;

    alsa_dbg("[pcm] close %s device, %s\n", __func__,
        substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? "output" : "input");

   // kthread_example_release();
    mutex_lock(&(tcc_alsa_info.mutex));
    if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
        tcc_alsa_info.flag &= ~TCC_RUNNING_PLAY;
        //tca_i2s_stop(pDAI, pADMASPDIFTX, 0);
        // pDAI->DAMR &= ~Hw14;
    } else {
        tcc_alsa_info.flag &= ~TCC_RUNNING_CAPTURE;
        //tca_i2s_stop(pDAI, pADMASPDIFTX, 1);
        //pDAI->DAMR &= ~Hw13;
    }


    if (prtd) {
        kfree(prtd);
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

out:
    return ret;
}


#if defined(CONFIG_PM)
static int tcc_pcm_suspend(struct snd_soc_dai *dai)
{
    alsa_dbg(" %s \n", __func__);
    return 0;
}

static int tcc_pcm_resume(struct snd_soc_dai *dai)
{
    alsa_dbg(" %s \n", __func__);

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

int tcc_pcm_ch2_platform_register(struct device *dev)
{
	return snd_soc_register_platform(dev, &tcc_soc_platform);
}
EXPORT_SYMBOL_GPL(tcc_pcm_ch2_platform_register);

void tcc_pcm_ch2_platform_unregister(struct device *dev)
{
	return snd_soc_unregister_platform(dev);
}
EXPORT_SYMBOL_GPL(tcc_pcm_ch2_platform_unregister);

MODULE_DESCRIPTION("Telechips PCM DSP driver");
MODULE_LICENSE("GPL");

