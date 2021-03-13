/*
 * linux/sound/soc/tcc/tcc-i2s.c
 *
 * Based on:    linux/sound/soc/pxa/pxa2xx-i2s.h
 * Author: Liam Girdwood<liam.girdwood@wolfsonmicro.com or linux@wolfsonmicro.com>
 * Rewritten by:    <linux@telechips.com>
 * Created:     12th Aug 2005   Initial version.
 * Modified:    Nov 25, 2008
 * Description: ALSA PCM interface for the Intel PXA2xx chip
 *
 * Copyright 2005 Wolfson Microelectronics PLC.
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

#include <linux/clk-private.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <asm/io.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>

#include <sound/initval.h>
#include <sound/soc.h>

#include "tcc/tca_audio_hw.h"

#include "tcc-pcm.h"
#include "tcc-i2s.h"
#include "tcc/tca_tcchwcontrol.h"


#define TCC_I2S_RATES   SNDRV_PCM_RATE_8000_192000

#undef alsa_dbg
#if 0
#define alsa_dbg(f, a...)  printk("== alsa-debug I2S == " f, ##a)
#else
#define alsa_dbg(f, a...)
#endif

#define TAG_ACLK_CTRL "[aclk_ctrl] "

#define	I2S_DADIR0	0x000		// Digital Audio Input Register 0
#define	I2S_DADIR1	0x004		// Digital Audio Input Register 1
#define	I2S_DADIR2	0x008		// Digital Audio Input Register 2
#define	I2S_DADIR3	0x00C		// Digital Audio Input Register 3
#define	I2S_DADIR4	0x010		// Digital Audio Input Register 4
#define	I2S_DADIR5	0x014		// Digital Audio Input Register 5
#define	I2S_DADIR6	0x018		// Digital Audio Input Register 6
#define	I2S_DADIR7	0x01C		// Digital Audio Input Register 7
#define	I2S_DADOR0	0x020		// Digital Audio Output Register 0
#define	I2S_DADOR1	0x024		// Digital Audio Output Register 1
#define	I2S_DADOR2	0x028		// Digital Audio Output Register 2
#define	I2S_DADOR3	0x02C		// Digital Audio Output Register 3
#define	I2S_DADOR4	0x030		// Digital Audio Output Register 4
#define	I2S_DADOR5	0x034		// Digital Audio Output Register 5
#define	I2S_DADOR6	0x038		// Digital Audio Output Register 6
#define	I2S_DADOR7	0x03C		// Digital Audio Output Register 7
#define	I2S_DAMR	0x040		// Digital Audio Mode Register
#define	I2S_DAVC	0x044		// Digital Audio Volume Control Register
#define	I2S_MCCR0	0x048		// Multi Channel Control Register 0
#define	I2S_MCCR1	0x04C		// Multi Channel Control Register 1

#define i2s_writel	__raw_writel
#define i2s_readl	__raw_readl

#define DAI_CLOCK_RATE (256)

#if defined(GLOBAL_DEV_NUM)
int __I2S_DEV_NUM__ = 0;
int __SPDIF_DEV_NUM__ = 1;
int __I2S_SUB_DEV_NUM__ = 2;
int __I2S_SUB2_DEV_NUM__ = 3;
int __SPDIF_SUB_DEV_NUM__ = 4;
int __CDIF_DEV_NUM__ = 5;

#endif

#if defined(CONFIG_SND_SOC_DAUDIO_NOISE_FILTER)
#define MULTIPLE_CLOCK_RATE_NOISE_FILTER (4)
#endif

int tcc_i2s_rate[4][2] = {{0, 0}, }; // to avoid sample rate re-setting.

struct tcc_i2s_data {
	unsigned int	id;

	void __iomem	*dai_reg;
	struct clk	*dai_pclk;
	struct clk	*dai_hclk;
#if defined(CONFIG_SND_SOC_DAUDIO_NOISE_FILTER)
	// Planet20160829 Apply AudioFilter
	struct clk	*dam_pclk;
	unsigned int	dai_slave;
#endif
	unsigned int	dai_irq;
	unsigned long	dai_clk_rate;
#if defined(CONFIG_ARCH_TCC898X)
	int dai_port;
#elif defined(CONFIG_ARCH_TCC802X)
	struct audio_i2s_port *dai_port;
#endif
#if defined(CONFIG_SND_SOC_DAUDIO_CLOCK_CONTROL)
	/* mutex valiable must be seperated for each audio channel */
	struct mutex aclk1_ctrl_lock;
#endif
	ADMADAI		ADMA_DAI;	/* backup_regs */
};

static struct tcc_pcm_dma_params tcc_i2s_pcm_stereo_out = {
	.name       = "I2S PCM Stereo out",
	.dma_addr   = 0,
	.channel    = 0,
};

static struct tcc_pcm_dma_params tcc_i2s_pcm_stereo_in = {
	.name       = "I2S PCM Stereo in",
	.dma_addr   = 0,
	.channel    = 1,
};

static unsigned int io_ckc_get_dai_clock(unsigned int freq)
{
	switch (freq) {
		case 44100:
			return (44100 * DAI_CLOCK_RATE);
		case 22000:
			return (22050 * DAI_CLOCK_RATE);
		case 11000:
			return (11025 * DAI_CLOCK_RATE);
		default:
			break;
	}
	return (freq * DAI_CLOCK_RATE);
}

static int tcc_i2s_startup(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;

	alsa_dbg("tcc_i2s_startup() \n");
	if (!cpu_dai->active) {
	}

	return 0;
}

static int tcc_i2s_set_dai_fmt(struct snd_soc_dai *cpu_dai, unsigned int fmt)
{
	alsa_dbg(" %s \n", __func__);

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
		case SND_SOC_DAIFMT_CBS_CFS:
			break;
		case SND_SOC_DAIFMT_CBM_CFS:
			break;
		default:
			break;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
		case SND_SOC_DAIFMT_I2S:
			break;
		case SND_SOC_DAIFMT_LEFT_J:
			break;
#if 0		// Android Style
		case SND_SOC_DAIFMT_DSP_A:
			printk("Error!! TCC892x/TCC893x support only DSP Early mode!!!\n");
			break;
		case SND_SOC_DAIFMT_DSP_B:
			BITSET(pADMA_DAI->DAMR, Hw26 | Hw25);
			BITSET(pADMA_DAI->MCCR0, Hw14 | Hw11 | Hw10 | (Hw6 - Hw0));
#endif
			break;
	}
	return 0;
}

static int tcc_i2s_set_dai_sysclk(struct snd_soc_dai *cpu_dai,
								  int clk_id, unsigned int freq, int dir)
{
	alsa_dbg(" %s \n", __func__);

	if (clk_id != TCC_I2S_SYSCLK)
		return -ENODEV;

	return 0;
}
static int tcc_i2s_hw_params(struct snd_pcm_substream *substream,
							 struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct tcc_pcm_dma_params *dma_data;
	struct tcc_i2s_data *tcc_i2s = dev_get_drvdata(dai->dev);
	snd_pcm_format_t format = params_format(params);
	unsigned reg_value;
	unsigned int req_clk = 0, pre_clk = 0;

	volatile unsigned long *Schimitt_reg;	// Planet20160829 ApplyAudioFilter

	// Planet20160829 Apply AudioFilter Start
#if defined(CONFIG_SND_SOC_DAUDIO_NOISE_FILTER)
	if(params->reserved[SNDRV_PCM_HW_PARAM_EFLAG0] & SNDRV_PCM_MODE_SLAVE) {
		alsa_dbg("~~~!!! tcc_i2s_hw_params() I2S Slave mode AudioBlock:%d !!!~~~\n", tcc_i2s->id);
		tcc_i2s->dai_slave = 1;
		switch(tcc_i2s->id) {
			case 0:
				// 0x74200180 GPIOG , +0x28 Shimit
				// GPIO_G7 : BCLK, GPIO_G8 : LRCK, GPIO_G10 : DAI = 0x580
				Schimitt_reg = (volatile unsigned long*)tcc_p2v((0x74200180 + 0x28));
				*Schimitt_reg = (*Schimitt_reg) | 0x00000580;
				break;
			case 1:
				// 0x74200180 GPIOG , +0x28 Shimit
				// GPIO_G12 : BCLK, GPIO_G13 : LRCK, GPIO_G15 : DAI = 0xB000
				Schimitt_reg = (volatile unsigned long*)tcc_p2v((0x74200180 + 0x28));
				*Schimitt_reg = (*Schimitt_reg) | 0x0000B000;
				break;
			case 2:
				// 0x74200180 GPIOG , +0x28 Shimit
				// GPIO_E23 : BCLK, GPIO_E24 : LRCK, GPIO_E26 : DAI = 0x5800000
				Schimitt_reg = (volatile unsigned long*)tcc_p2v((0x74200100 + 0x28));
				*Schimitt_reg = (*Schimitt_reg) | 0x05800000;
				break;
			case 3:
				// 0x74200180 GPIOG , +0x28 Shimit
				// GPIO_G1 : BCLK, GPIO_G2 : LRCK, GPIO_G4 : DAI = 0x16
				Schimitt_reg = (volatile unsigned long*)tcc_p2v((0x74200180 + 0x28));
				*Schimitt_reg = (*Schimitt_reg) | 0x00000016;
				break;
		}
		//clk_disable_unprepare(tcc_i2s->dam_pclk);
		clk_set_rate(tcc_i2s->dam_pclk, io_ckc_get_dai_clock(params_rate(params)) * MULTIPLE_CLOCK_RATE_NOISE_FILTER);
		clk_prepare_enable(tcc_i2s->dam_pclk);

		i2s_writel(i2s_readl(tcc_i2s->dai_reg + I2S_DAMR) | Hw27, tcc_i2s->dai_reg + I2S_DAMR);

	}
	else {
		alsa_dbg("~~~!!! tcc_i2s_hw_params() I2S Master mode !!!~~~\n");

	}
#endif
	// Planet20160829 Apply AudioFilter End

	alsa_dbg("%s check device(%d) \n", __func__, substream->pcm->device);
	//It's seem to be useless because both i2s0 and i2s1 use this device with "telechips,i2s".
	//If this blocked code occur a problem, rollback this and do register i2s1 to tcc896x.dtsi with "telechips,i2s-ch1" to use tcc-i2s-ch1.c
	//if (substream->pcm->device == __I2S_DEV_NUM__)
	{
		if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			dma_data = &tcc_i2s_pcm_stereo_out;
		else if(substream->stream == SNDRV_PCM_STREAM_CAPTURE)
			dma_data = &tcc_i2s_pcm_stereo_in;
		snd_soc_dai_set_dma_data(cpu_dai, substream, dma_data);

		reg_value = i2s_readl(tcc_i2s->dai_reg + I2S_DAMR);

		/* sjpark@cleinsoft
		 * date : 2016/07/25
		 * desc : I2S Sample bit control
		 * 16bit mode - TXS ( LSB 16bit mode )
		 *              RXS ( LSB 16bit mode )
		 *              FD Div 32 ( 32fs -> fs )
		 *              BD Div  8 ( 256fs -> 32fs )
		 * 24bit mode - TXS ( LSB 24bit mode )
		 *              RXS ( LSB 24bit mode )
		 *              FD Div 64 ( 64fs -> fs )
		 *              BD Div  4 ( 256fs -> 64fs )
		 * */
		if (format == SNDRV_PCM_FORMAT_S24_LE) {
			reg_value &= ~(Hw20 | Hw18);
			reg_value |= (Hw21 | Hw19);

			reg_value &= (~Hw4);
			reg_value |= (Hw5);
			reg_value &= (~Hw6);
			reg_value &= (~Hw7);

			alsa_dbg("%s %d SNDRV_PCM_FORMAT_S24_LE reg 0x%x\n", __func__, tcc_i2s->id, reg_value);
		}
		else {
			reg_value |= (Hw18 | Hw19);
			reg_value |= (Hw20 | Hw21);

			reg_value &= (~Hw4);
			reg_value &= (~Hw5);
			reg_value &= (~Hw6);
			reg_value |= (Hw7);

			alsa_dbg("%s %d SNDRV_PCM_FORMAT_S16_LE reg 0x%x\n", __func__, tcc_i2s->id, reg_value);
		}

		i2s_writel(reg_value, tcc_i2s->dai_reg + I2S_DAMR);
		// Set DAI clock
		pre_clk = clk_get_rate(tcc_i2s->dai_pclk);
		req_clk = io_ckc_get_dai_clock(params_rate(params));
		if((tcc_i2s_rate[tcc_i2s->id][0] != req_clk)
				|| (tcc_i2s_rate[tcc_i2s->id][1] != pre_clk)) {
			if (tcc_i2s->dai_pclk) {
				clk_disable_unprepare(tcc_i2s->dai_pclk);
				clk_set_rate(tcc_i2s->dai_pclk, io_ckc_get_dai_clock(params_rate(params)));
				clk_prepare_enable(tcc_i2s->dai_pclk);
				tcc_i2s_rate[tcc_i2s->id][0] = req_clk;
				tcc_i2s_rate[tcc_i2s->id][1] = clk_get_rate(tcc_i2s->dai_pclk);
			}
		}
	}

	alsa_dbg("=====================\n");
	alsa_dbg("= rate        : %d\n", params_rate(params));
	alsa_dbg("= channels    : %d\n", params_channels(params));
	alsa_dbg("= period_size : %d\n", params_period_size(params));

	return 0;
}

static int tcc_i2s_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *dai)
{
	int ret = 0;
	//struct tcc_i2s_data *tcc_i2s = dev_get_drvdata(dai->dev);
	//unsigned reg_damr;
	alsa_dbg(" %s \n", __func__);

	//reg_damr = i2s_readl(tcc_i2s->dai_reg+I2S_DAMR);
	switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
		case SNDRV_PCM_TRIGGER_RESUME:
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
			break;
		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_SUSPEND:
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
			break;
		default:
			ret = -EINVAL;
	}
	//i2s_writel(reg_damr, tcc_i2s->dai_reg+I2S_DAMR);
	return ret;
}

static void tcc_i2s_shutdown(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	// Planet20160829 Apply AudioFilter Start
	struct tcc_i2s_data *tcc_i2s = dev_get_drvdata(dai->dev);
	volatile unsigned long *Schimitt_reg;

#if defined(CONFIG_SND_SOC_DAUDIO_NOISE_FILTER)
	if(tcc_i2s->dai_slave == 1) {
		switch(tcc_i2s->id) {
			case 0:
				Schimitt_reg = (volatile unsigned long*)tcc_p2v((0x74200180 + 0x28));
				*Schimitt_reg = (*Schimitt_reg) & ~(0x00000580);
				break;
			case 1:
				Schimitt_reg = (volatile unsigned long*)tcc_p2v((0x74200180 + 0x28));
				*Schimitt_reg = (*Schimitt_reg) & ~(0x0000B000);
				break;
			case 2:
				Schimitt_reg = (volatile unsigned long*)tcc_p2v((0x74200100 + 0x28));
				*Schimitt_reg = (*Schimitt_reg) & ~(0x05800000);
				break;
			case 3:
				Schimitt_reg = (volatile unsigned long*)tcc_p2v((0x74200180 + 0x28));
				*Schimitt_reg = (*Schimitt_reg) & ~(0x00000016);
				break;
		}
		i2s_writel(i2s_readl(tcc_i2s->dai_reg + I2S_DAMR) & ~Hw27, tcc_i2s->dai_reg + I2S_DAMR);

		clk_disable_unprepare(tcc_i2s->dam_pclk);
		tcc_i2s->dai_slave = 0;
	}
#endif
	// Planet20160829 Apply AudioFilter End

	alsa_dbg(" %s \n", __func__);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
	}
	else {
	}
}

#if (1)
static int tcc_i2s_suspend(struct device *dev)
{
	struct tcc_i2s_data *tcc_i2s = dev_get_drvdata(dev);
#else
static int tcc_i2s_suspend(struct snd_soc_dai *dai)
{
	struct tcc_i2s_data *tcc_i2s = dev_get_drvdata(dai->dev);
#endif
	volatile PADMADAI pADMA_DAI = (volatile PADMADAI)(tcc_i2s->dai_reg);

	alsa_dbg(" %s \n", __func__);

	tcc_i2s->ADMA_DAI.DAMR   = pADMA_DAI->DAMR;
	tcc_i2s->ADMA_DAI.DAVC   = pADMA_DAI->DAVC;
	if (tcc_i2s->id == 0) {
		tcc_i2s->ADMA_DAI.MCCR0  = pADMA_DAI->MCCR0;
		tcc_i2s->ADMA_DAI.MCCR1  = pADMA_DAI->MCCR1;
	}

	if((tcc_i2s->dai_pclk) && (tcc_i2s->dai_pclk->enable_count)) {
		clk_disable_unprepare(tcc_i2s->dai_pclk);
	}
	if((tcc_i2s->dai_hclk) && (tcc_i2s->dai_hclk->enable_count)) {
		clk_disable_unprepare(tcc_i2s->dai_hclk);
	}

	return 0;
}

#if (1)
static int tcc_i2s_resume(struct device *dev)
{
	struct tcc_i2s_data *tcc_i2s = dev_get_drvdata(dev);
#else
static int tcc_i2s_resume(struct snd_soc_dai *dai)
{
	struct tcc_i2s_data *tcc_i2s = dev_get_drvdata(dai->dev);
#endif
	volatile PADMADAI pADMA_DAI = (volatile PADMADAI)(tcc_i2s->dai_reg);

	alsa_dbg(" %s \n", __func__);

	if((tcc_i2s->dai_hclk) && (!tcc_i2s->dai_hclk->enable_count))
		clk_prepare_enable(tcc_i2s->dai_hclk);
	if((tcc_i2s->dai_pclk) && (!tcc_i2s->dai_pclk->enable_count))
		clk_prepare_enable(tcc_i2s->dai_pclk);

	pADMA_DAI->DAMR   = tcc_i2s->ADMA_DAI.DAMR;
	pADMA_DAI->DAVC   = tcc_i2s->ADMA_DAI.DAVC;
	if (tcc_i2s->id == 0) {
		pADMA_DAI->MCCR0  = tcc_i2s->ADMA_DAI.MCCR0;
		pADMA_DAI->MCCR1  = tcc_i2s->ADMA_DAI.MCCR1;
	}

	return 0;
}

static unsigned int tcc_i2s_mode(int master)
{
	unsigned int reg_value;
//	reg_value = (1<<31)|(1<<30)|(3<<20)|(3<<18)|  // Android Style
//		(1<<15)|(1<<11)|(1<<10)|(1<<9)|(1<<5);

	reg_value = 0
#if defined(CONFIG_SND_SOC_TCC_MULTICHANNEL )
				| Hw29	//DAI Buffer Threshold Enable
				| Hw28	//Multi-Port Enable
#endif
#if 0 //def I2S_24BIT_MODE
				| Hw21	//DAI Rx shift Bit-pack LSB and 24bit mode
				| Hw19	//DAI Tx shift Bit-pack LSB and 24bit mode
#else
				| Hw21 | Hw20 //DAI Rx shift Bit-pack LSB and 16bit mode
				| Hw19 | Hw18 //DAI Tx shift Bit-pack LSB and 16bit mode
#endif
#if defined(CONFIG_SND_SOC_DAUDIO_CLOCK_CONTROL)
				//	| Hw15	//DAI Enable
#else
				| Hw15	//DAI Enable
#endif
//			| (Hw7|Hw6)	//DAI LRCLK(1024fs->64fs)
				| Hw5	//DAI LRCLK(64fs->fs)
				;

	if(master) {
		reg_value |= ( Hw31 | Hw30 ) ;//BCLK direct at master mode. LRCLK direct at master mode	.
		reg_value |= (Hw11 | Hw10 | Hw9 );   	// DAI System Clock master select
		// DAI Bit Clock master select
		//DAI Frame clock(LRCLK) master selcet

	}
	return reg_value;
#if 0 // We need Audio filter porting

	volatile unsigned long *Shimit_reg;
	volatile unsigned long *DAMR;
	volatile unsigned long *AudioFilterClock;

	/* 0x74200180 GPIOG , +0x28 Shimit
	* GPIO_G1 : BCLK, GPIO_G2 : LRCK, GPIO_G4 : DAI = 0x16 */

	Shimit_reg = (volatile unsigned long*)tcc_p2v((0x74200180 + 0x28));
	*Shimit_reg = (*Shimit_reg) | 0x16;
	AudioFilterClock = (volatile unsigned long*)tcc_p2v(0x740000e8 );
	if((*AudioFilterClock) != 0xa1000009) {
		*AudioFilterClock = 0xa1000009;

	}

	DAMR = (volatile unsigned long*)tcc_p2v(0x76101040);
	if (((*DAMR) & Hw27 ) == 0) {
		*DAMR = (*DAMR) | Hw27;
		//  printk(" ##### NOISE_REMOVE 1 ##### \n");
	}
#endif
}


static int tcc_i2s_probe(struct snd_soc_dai *dai)
{
	struct tcc_i2s_data *tcc_i2s = dev_get_drvdata(dai->dev);
	unsigned reg_value;
	int masterMode = 1;

	alsa_dbg("== alsa-debug == %s() \n", __func__);

#if defined(CONFIG_ARCH_TCC898X)
	tca_i2s_port_mux(tcc_i2s->id, tcc_i2s->dai_port);
#endif
	/* clock enable */
	if (tcc_i2s->dai_hclk)
		clk_prepare_enable(tcc_i2s->dai_hclk);

	/* init dai register */
	i2s_writel(0, tcc_i2s->dai_reg + I2S_DAVC);

	reg_value = tcc_i2s_mode(masterMode);

//    printk("%s === read  reg 0x%x\n",__func__,reg_value);

	i2s_writel(reg_value, tcc_i2s->dai_reg + I2S_DAMR);

	if (tcc_i2s->id == 0) {
		reg_value = i2s_readl(tcc_i2s->dai_reg + I2S_MCCR0) |
					(1 << 31) | (1 << 30) | (1 << 29) | (1 << 28);
		i2s_writel(reg_value, tcc_i2s->dai_reg + I2S_MCCR0);
	}

	/* dai receiver/transmitter disable */
	i2s_writel(i2s_readl(tcc_i2s->dai_reg + I2S_DAMR) & ~((1 << 13) | (1 << 14)),
			   tcc_i2s->dai_reg + I2S_DAMR);

	/* set dai_pclk */
	if (tcc_i2s->dai_pclk) {
		clk_set_rate(tcc_i2s->dai_pclk, tcc_i2s->dai_clk_rate);
		clk_prepare_enable(tcc_i2s->dai_pclk);
		//save defualt rate
		tcc_i2s_rate[tcc_i2s->id][0] = tcc_i2s->dai_clk_rate;
		tcc_i2s_rate[tcc_i2s->id][1] = clk_get_rate(tcc_i2s->dai_pclk);

	}

	return 0;
}

static struct snd_soc_dai_ops tcc_i2s_ops = {
	.set_sysclk = tcc_i2s_set_dai_sysclk,
	.set_fmt    = tcc_i2s_set_dai_fmt,
	.startup    = tcc_i2s_startup,
	.shutdown   = tcc_i2s_shutdown,
	.hw_params  = tcc_i2s_hw_params,
	.trigger    = tcc_i2s_trigger,
};

static struct snd_soc_dai_driver tcc_i2s_dai = {
	.name = "tcc-dai-i2s",
	.probe = tcc_i2s_probe,
#if (0)
	.suspend = tcc_i2s_suspend,
	.resume = tcc_i2s_resume,
#endif
	.playback = {
		.channels_min = 1,
		.channels_max = 8,
		.rates = TCC_I2S_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_IEC958_SUBFRAME_LE,
	},  //should be change? phys:32 width:16
	.capture = {
		.channels_min = 1,
		.channels_max = 8,
		.rates = TCC_I2S_RATES,
#if defined(CONFIG_VIDEO_HDMI_IN_SENSOR)
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_U16_LE | SNDRV_PCM_FMTBIT_S24_LE,
	},
#else
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
	}, //should be change? phys:32 width:16

#endif
	.ops   = &tcc_i2s_ops,
};

static const struct snd_soc_component_driver tcc_i2s_component = {
	.name		= "tcc-i2s",
};
static int soc_tcc_i2s_probe(struct platform_device *pdev)
{
	struct tcc_i2s_data *tcc_i2s;
	u32 clk_rate;
#if defined(CONFIG_ARCH_TCC898X)
	int port_mux = 0;
#endif
	int ret = 0;

	tcc_i2s = devm_kzalloc(&pdev->dev, sizeof(*tcc_i2s), GFP_KERNEL);
	if (!tcc_i2s) {
		dev_err(&pdev->dev, "no memory for state\n");
		goto err;
	}
	dev_set_drvdata(&pdev->dev, tcc_i2s);
	tcc_i2s->id = of_alias_get_id(pdev->dev.of_node, "i2s");

	/* get dai info. */
	tcc_i2s->dai_reg = of_iomap(pdev->dev.of_node, 0);
	if (IS_ERR((void *)tcc_i2s->dai_reg))
		tcc_i2s->dai_reg = NULL;
	tcc_i2s->dai_pclk = of_clk_get(pdev->dev.of_node, 0);
	if (IS_ERR(tcc_i2s->dai_pclk))
		tcc_i2s->dai_pclk = NULL;
	tcc_i2s->dai_hclk = of_clk_get(pdev->dev.of_node, 1);
	if (IS_ERR(tcc_i2s->dai_hclk))
		tcc_i2s->dai_hclk = NULL;
	// Planet20160829 Apply AudioFilter
#if defined(CONFIG_SND_SOC_DAUDIO_NOISE_FILTER)
	tcc_i2s->dam_pclk = of_clk_get(pdev->dev.of_node, 2);
	if (IS_ERR(tcc_i2s->dam_pclk))
		tcc_i2s->dam_pclk = NULL;
#endif
	of_property_read_u32(pdev->dev.of_node, "clock-frequency", &clk_rate);
	tcc_i2s->dai_clk_rate = (unsigned long)clk_rate;
	tcc_i2s->dai_irq = platform_get_irq(pdev, 0);
#if defined(CONFIG_ARCH_TCC898X)
	of_property_read_u32(pdev->dev.of_node, "port-mux", &port_mux);
	tcc_i2s->dai_port = port_mux;
#elif defined(CONFIG_ARCH_TCC802X)
	tcc_i2s->dai_port = (struct audio_i2s_port *)kmalloc(sizeof(struct audio_i2s_port), GFP_KERNEL);
	memset(tcc_i2s->dai_port, 255, sizeof(struct audio_i2s_port));
	of_property_read_u8_array(pdev->dev.of_node, "clk-mux", tcc_i2s->dai_port->clk,
							  of_property_count_elems_of_size(pdev->dev.of_node, "clk-mux", sizeof(char)));
	of_property_read_u8_array(pdev->dev.of_node, "daout-mux", tcc_i2s->dai_port->daout,
							  of_property_count_elems_of_size(pdev->dev.of_node, "daout-mux", sizeof(char)));
	of_property_read_u8_array(pdev->dev.of_node, "dain-mux", tcc_i2s->dai_port->dain,
							  of_property_count_elems_of_size(pdev->dev.of_node, "dain-mux", sizeof(char)));
#endif
#if defined(CONFIG_SND_SOC_DAUDIO_CLOCK_CONTROL)
	mutex_init(&tcc_i2s->aclk1_ctrl_lock);
#endif
	ret = snd_soc_register_component(&pdev->dev, &tcc_i2s_component,
									 &tcc_i2s_dai, 1);
	if (ret)
		goto err_reg_component;

	alsa_dbg("== alsa-debug == %s() tcc_i2s->id:%d \n", __func__, tcc_i2s->id);

#if defined(CONFIG_ARCH_TCC897X)
	if (tcc_i2s->id == 0)
		ret = tcc_pcm_platform_register(&pdev->dev);
	else if (tcc_i2s->id == 1)
		ret = tcc_pcm_sub1_platform_register(&pdev->dev);
	else if (tcc_i2s->id == 2)
		ret = tcc_pcm_sub2_platform_register(&pdev->dev);
	else if (tcc_i2s->id == 3)
		ret = tcc_pcm_sub3_platform_register(&pdev->dev);
	if (ret)
		goto err_reg_pcm;
#else
	if (tcc_i2s->id == 0)
		ret = tcc_pcm_platform_register(&pdev->dev);
	else
		ret = tcc_pcm_sub1_platform_register(&pdev->dev);
#endif

	return 0;

err_reg_pcm:
	snd_soc_unregister_component(&pdev->dev);

err_reg_component:
	if (tcc_i2s->dai_hclk) {
		clk_put(tcc_i2s->dai_hclk);
		tcc_i2s->dai_hclk = NULL;
	}
	if (tcc_i2s->dai_pclk) {
		clk_put(tcc_i2s->dai_pclk);
		tcc_i2s->dai_pclk = NULL;
	}

	devm_kfree(&pdev->dev, tcc_i2s);
err:
	return ret;
}

static int  soc_tcc_i2s_remove(struct platform_device *pdev)
{
	struct tcc_i2s_data *tcc_i2s = platform_get_drvdata(pdev);

#if defined(CONFIG_ARCH_TCC897X)
	if (tcc_i2s->id == 0)
		tcc_pcm_platform_unregister(&pdev->dev);
	else if (tcc_i2s->id == 1)
		tcc_pcm_sub1_platform_unregister(&pdev->dev);
	else if (tcc_i2s->id == 2)
		tcc_pcm_sub2_platform_unregister(&pdev->dev);
	else if (tcc_i2s->id == 3)
		tcc_pcm_sub3_platform_unregister(&pdev->dev);
#else
	if (tcc_i2s->id == 0)
		tcc_pcm_platform_unregister(&pdev->dev);
	else
		tcc_pcm_sub1_platform_unregister(&pdev->dev);
#endif
	snd_soc_unregister_component(&pdev->dev);
	if (tcc_i2s->dai_hclk) {
		clk_put(tcc_i2s->dai_hclk);
		tcc_i2s->dai_hclk = NULL;
	}
	if (tcc_i2s->dai_pclk) {
		clk_put(tcc_i2s->dai_pclk);
		tcc_i2s->dai_pclk = NULL;
	}
	devm_kfree(&pdev->dev, tcc_i2s);

	return 0;
}

static struct of_device_id tcc_i2s_of_match[] = {
	{ .compatible = "telechips,i2s" },
	{ }
};

static SIMPLE_DEV_PM_OPS(tcc_i2s_pm_ops, tcc_i2s_suspend,
						 tcc_i2s_resume);

static struct platform_driver tcc_i2s_driver = {
	.driver = {
		.name = "tcc-dai",
		.owner = THIS_MODULE,
		.pm = &tcc_i2s_pm_ops,
		.of_match_table	= of_match_ptr(tcc_i2s_of_match),
	},
	.probe = soc_tcc_i2s_probe,
	.remove = soc_tcc_i2s_remove,
};

static int __init snd_tcc_i2s_init(void)
{
	return platform_driver_register(&tcc_i2s_driver);
}
module_init(snd_tcc_i2s_init);

static void __exit snd_tcc_i2s_exit(void)
{
	return platform_driver_unregister(&tcc_i2s_driver);
}
module_exit(snd_tcc_i2s_exit);

/* Module information */
MODULE_AUTHOR("linux <linux@telechips.com>");
MODULE_DESCRIPTION("Telechips TCC I2S SoC Interface");
MODULE_LICENSE("GPL");
