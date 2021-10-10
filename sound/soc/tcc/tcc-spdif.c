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
#include <sound/initval.h>
#include <sound/soc.h>

#include "tcc/tca_audio_hw.h"

#include "tcc-pcm.h"
#include "tcc-i2s.h"
#include "tcc/tca_tcchwcontrol.h"

#if defined(CONFIG_SND_SOC_TCC_MULTICHANNEL)
#define TCC_I2S_RATES   SNDRV_PCM_RATE_8000_192000
#else
#define TCC_I2S_RATES   SNDRV_PCM_RATE_8000_96000
#endif

#undef alsa_dbg
#if 0
#define alsa_dbg(f, a...)  printk("== alsa-debug SPDIF == " f, ##a)
#else
#define alsa_dbg(f, a...)
#endif

#define spdif_writel	__raw_writel
#define spdif_readl	__raw_readl

// Planet 20150812 S/PDIF_Rx
#define STREAM_CAPTURE 0
#define STREAM_PLAYBACK 1

#define SPDIF_TX_RATIO (7)

struct tcc_spdif_data {
	int	id;

	void __iomem	*spdif_reg;
	struct clk	*spdif_pclk;
	struct clk	*spdif_hclk;
	int	spdif_irq;
	unsigned long	spdif_clk_rate;
#if defined(CONFIG_ARCH_TCC898X)
	int dai_port;
#elif defined(CONFIG_ARCH_TCC802X)
	char dai_port[2];
#endif


	ADMASPDIFTX	ADMA_SPDIFTX;	/* backup_regs */
	ADMASPDIFRX ADMA_SPDIFRX;   /* backup_regs */	// Planet 20150812 S/PDIF_Rx
};

static struct tcc_pcm_dma_params tcc_spdif_pcm_out = {
	.name       = "I2S PCM S/PDIF out",
	.dma_addr   = 0,
	.channel    = 0,
};

// Planet 20150812 S/PDIF_Rx
static struct tcc_pcm_dma_params tcc_spdif_pcm_in = {
	.name       = "I2S PCM S/PDIF in",
	.dma_addr   = 0,
	.channel    = 1,
};

static unsigned int io_ckc_get_dai_clock(unsigned int freq)
{
	switch (freq) {
		case 44100:
			return (44100 * 256);
		case 22000:
			return (22050 * 256);
		case 11000:
			return (11025 * 256);
		default:
			break;
	}
	return (freq * 256);
}

static void tcc_spdif_set_clock(struct snd_soc_dai *dai, unsigned int clock_rate, bool stream)	// Planet 20150812 S/PDIF_Rx
{
	struct tcc_spdif_data *tcc_spdif = dev_get_drvdata(dai->dev);
	volatile ADMASPDIFTX *p_adma_spdif_tx_base;
	unsigned int clk_rate;
	unsigned tmpCfg, tmpStatus;

	if(tcc_spdif->id == 0)
		p_adma_spdif_tx_base = (volatile ADMASPDIFTX *)tcc_p2v(BASE_ADDR_SPDIFTX0);
	else
#if defined(CONFIG_ARCH_TCC897X) || defined(CONFIG_ARCH_TCC570X)
		p_adma_spdif_tx_base = (volatile ADMASPDIFTX *)tcc_p2v(BASE_ADDR_SPDIFTX3);
#else	// for TCC896X
		p_adma_spdif_tx_base = (volatile ADMASPDIFTX *)tcc_p2v(BASE_ADDR_SPDIFTX1);
#endif

//to fix sound hound rx recording issue.
	clk_rate = io_ckc_get_dai_clock(clock_rate) * 4;
//	clk_rate = io_ckc_get_dai_clock(clock_rate) * 2;   /* set 512fs for HDMI */

	if (tcc_spdif->spdif_pclk) {
		clk_disable_unprepare(tcc_spdif->spdif_pclk);
		clk_set_rate(tcc_spdif->spdif_pclk, clk_rate);
		clk_prepare_enable(tcc_spdif->spdif_pclk);
	}

	alsa_dbg("[%s],%d : clock_rate[%u] [%u] \n", __func__, tcc_spdif->id, clock_rate, clk_rate);

	// Planet 20150812 S/PDIF_Rx Start
	if(stream == STREAM_PLAYBACK) {
		tmpCfg = p_adma_spdif_tx_base->TxConfig;
		tmpStatus = p_adma_spdif_tx_base->TxChStat;

		if (clock_rate == 44100) {          /* 44.1KHz */
			p_adma_spdif_tx_base->TxConfig = ((tmpCfg & 0xFFFF00FF) | (SPDIF_TX_RATIO << 8));
			p_adma_spdif_tx_base->TxChStat = ((tmpStatus & 0xFFFFFF3F) | (0 << 6));
		}
		else if (clock_rate == 48000) {     /* 48KHz */
			p_adma_spdif_tx_base->TxConfig = ((tmpCfg & 0xFFFF00FF) | (SPDIF_TX_RATIO << 8));
			p_adma_spdif_tx_base->TxChStat = ((tmpStatus & 0xFFFFFF3F) | (1 << 6));
		}
		else if (clock_rate == 32000) {     /* 32KHz */
			p_adma_spdif_tx_base->TxConfig = ((tmpCfg & 0xFFFF00FF) | (SPDIF_TX_RATIO << 8));
			p_adma_spdif_tx_base->TxChStat = ((tmpStatus & 0xFFFFFF3F) | (2 << 6));
		}
		else {                              /* Sampling Rate Converter */
			p_adma_spdif_tx_base->TxConfig = ((tmpCfg & 0xFFFF00FF) | (SPDIF_TX_RATIO << 8));
			p_adma_spdif_tx_base->TxChStat = ((tmpStatus & 0xFFFFFF3F) | (3 << 6));
		}
	}	// Planet 20150812 S/PDIF_Rx End
}

static int tcc_spdif_startup(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
#if 0
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;

	alsa_dbg("tcc_spdif_startup() \n");
	if (!cpu_dai->active) {
	}
#endif
	return 0;
}

static int tcc_spdif_set_fmt(struct snd_soc_dai *cpu_dai, unsigned int fmt)
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
	}
	return 0;
}

static int tcc_spdif_set_sysclk(struct snd_soc_dai *cpu_dai,
								int clk_id, unsigned int freq, int dir)
{
	alsa_dbg(" %s \n", __func__);

	if (clk_id != TCC_I2S_SYSCLK)
		return -ENODEV;

	return 0;
}

static int tcc_spdif_hw_params(struct snd_pcm_substream *substream,
							   struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct tcc_pcm_dma_params *dma_data;
	//struct tcc_spdif_data *tcc_spdif = dev_get_drvdata(dai->dev);


	printk("%s check device(%d) \n", __func__, substream->pcm->device);

	// Planet 20150812 S/PDIF_Rx Start
	if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		alsa_dbg(" %s Playback \n", __func__);
		dma_data = &tcc_spdif_pcm_out;
		snd_soc_dai_set_dma_data(cpu_dai, substream, dma_data);
		tcc_spdif_set_clock(dai, params_rate(params), STREAM_PLAYBACK);
	}
	else if(substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		alsa_dbg(" %s Capture \n", __func__);
		dma_data = &tcc_spdif_pcm_in;
		snd_soc_dai_set_dma_data(cpu_dai, substream, dma_data);
		tcc_spdif_set_clock(dai, params_rate(params), STREAM_CAPTURE);
	}
	// Planet 20150812 S/PDIF_Rx End

	alsa_dbg("=====================\n");
	alsa_dbg("%d = rate        : %d\n", substream->pcm->device, params_rate(params));
	alsa_dbg("%d = channels    : %d\n", substream->pcm->device, params_channels(params));
	alsa_dbg("%d= period_size : %d\n", substream->pcm->device, params_period_size(params));

	return 0;

}


static int tcc_spdif_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *dai)
{
	int ret = 0;

	alsa_dbg(" %s \n", __func__);

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
	return ret;
}

static void tcc_spdif_shutdown(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	alsa_dbg(" %s \n", __func__);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
	}
	else {
	}
}

#if (1)
static int tcc_spdif_suspend(struct device *dev)
{
	struct tcc_spdif_data *tcc_spdif = dev_get_drvdata(dev);
#else
static int tcc_spdif_suspend(struct snd_soc_dai *dai)
{
	struct tcc_spdif_data *tcc_spdif = dev_get_drvdata(dai->dev);
#endif
	volatile PADMASPDIFTX pADMA_SPDIFTX = (volatile PADMASPDIFTX)(tcc_spdif->spdif_reg);
	volatile PADMASPDIFRX pADMA_SPDIFRX = (volatile PADMASPDIFRX)((tcc_spdif->spdif_reg) + 0x800);	// Planet 20150812 S/PDIF_Rx

	alsa_dbg(" %s \n", __func__);

	tcc_spdif->ADMA_SPDIFTX.TxConfig  = pADMA_SPDIFTX->TxConfig;
	tcc_spdif->ADMA_SPDIFTX.TxChStat  = pADMA_SPDIFTX->TxChStat;
	tcc_spdif->ADMA_SPDIFTX.TxIntMask = pADMA_SPDIFTX->TxIntMask;
	//tcc_spdif->ADMA_SPDIFTX.TxIntStat = pADMA_SPDIFTX->TxIntStat;
	tcc_spdif->ADMA_SPDIFTX.DMACFG    = pADMA_SPDIFTX->DMACFG;

	// Planet 20150812 S/PDIF_Rx
	tcc_spdif->ADMA_SPDIFRX.RxConfig  = pADMA_SPDIFRX->RxConfig;
	tcc_spdif->ADMA_SPDIFRX.RxStatus  = pADMA_SPDIFRX->RxStatus;
	tcc_spdif->ADMA_SPDIFRX.RxIntMask = pADMA_SPDIFRX->RxIntMask;

	if((tcc_spdif->spdif_pclk) && (tcc_spdif->spdif_pclk->enable_count))
		clk_disable_unprepare(tcc_spdif->spdif_pclk);
	if((tcc_spdif->spdif_hclk) && (tcc_spdif->spdif_hclk->enable_count))
		clk_disable_unprepare(tcc_spdif->spdif_hclk);

	return 0;
}

#if (1)
static int tcc_spdif_resume(struct device *dev)
{
	struct tcc_spdif_data *tcc_spdif = dev_get_drvdata(dev);
#else
static int tcc_spdif_resume(struct snd_soc_dai *dai)
{
	struct tcc_spdif_data *tcc_spdif = dev_get_drvdata(dai->dev);
#endif
	volatile PADMASPDIFTX pADMA_SPDIFTX = (volatile PADMASPDIFTX)(tcc_spdif->spdif_reg);
	volatile PADMASPDIFRX pADMA_SPDIFRX = (volatile PADMASPDIFRX)((tcc_spdif->spdif_reg) + 0x800);	// Planet 20150812 S/PDIF_Rx

	alsa_dbg(" %s \n", __func__);
	if((tcc_spdif->spdif_hclk) && (!tcc_spdif->spdif_hclk->enable_count))
		clk_prepare_enable(tcc_spdif->spdif_hclk);
	if((tcc_spdif->spdif_pclk) && (!tcc_spdif->spdif_pclk->enable_count))
		clk_prepare_enable(tcc_spdif->spdif_pclk);

	pADMA_SPDIFTX->TxConfig  = tcc_spdif->ADMA_SPDIFTX.TxConfig;
	pADMA_SPDIFTX->TxChStat  = tcc_spdif->ADMA_SPDIFTX.TxChStat;
	pADMA_SPDIFTX->TxIntMask = tcc_spdif->ADMA_SPDIFTX.TxIntMask;
	//pADMA_SPDIFTX->TxIntStat = tcc_spdif->ADMA_SPDIFTX.TxIntStat;
	pADMA_SPDIFTX->DMACFG    = tcc_spdif->ADMA_SPDIFTX.DMACFG;

	// Planet 20150812 S/PDIF_Rx
	pADMA_SPDIFRX->RxConfig  = tcc_spdif->ADMA_SPDIFRX.RxConfig;
	pADMA_SPDIFRX->RxStatus  = tcc_spdif->ADMA_SPDIFRX.RxStatus;
	pADMA_SPDIFRX->RxIntMask = tcc_spdif->ADMA_SPDIFRX.RxIntMask;

	return 0;
}

static int tcc_spdif_probe(struct snd_soc_dai *dai)
{
	struct tcc_spdif_data *tcc_spdif = dev_get_drvdata(dai->dev);

	alsa_dbg("%s() \n", __func__);

#if defined(CONFIG_ARCH_TCC898X) || defined(CONFIG_ARCH_TCC802X)
	tca_spdif_port_mux(tcc_spdif->id, tcc_spdif->dai_port);
#endif
#if 1
	/* clock enable */
	if (tcc_spdif->spdif_hclk)
		clk_prepare_enable(tcc_spdif->spdif_hclk);

#endif
	if (tcc_spdif->spdif_pclk) {
		clk_set_rate(tcc_spdif->spdif_pclk, 44100 * 512);
		clk_prepare_enable(tcc_spdif->spdif_pclk);
		tcc_spdif_set_clock(dai, tcc_spdif->spdif_clk_rate, STREAM_PLAYBACK);	// Planet 20150812 S/PDIF_Rx
	}
	return 0;
}

static struct snd_soc_dai_ops tcc_spdif_ops = {
	.set_sysclk = tcc_spdif_set_sysclk,
	.set_fmt    = tcc_spdif_set_fmt,
	.startup    = tcc_spdif_startup,
	.shutdown   = tcc_spdif_shutdown,
	.hw_params  = tcc_spdif_hw_params,
	.trigger    = tcc_spdif_trigger,
};

static struct snd_soc_dai_driver tcc_snd_spdif = {
	.name = "tcc-spdif",
	.probe = tcc_spdif_probe,
#if (0)
	.suspend = tcc_spdif_suspend,
	.resume  = tcc_spdif_resume,
#endif
	.playback = {
		.channels_min = 2,
#if defined(CONFIG_SND_SOC_TCC_MULTICHANNEL)
		.channels_max = 8,
#else
		.channels_max = 2,
#endif
		.rates = TCC_I2S_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_U16_LE | SNDRV_PCM_FMTBIT_S24_LE
		| SNDRV_PCM_FMTBIT_IEC958_SUBFRAME_LE,
	},  //should be change? phys:32 width:16	 // Planet 20120306
	.capture = {	// Planet 20150812 S/PDIF_Rx
		.channels_min = 2,
		.channels_max = 2,
		.rates = TCC_I2S_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_U16_LE | SNDRV_PCM_FMTBIT_S24_LE
		| SNDRV_PCM_FMTBIT_IEC958_SUBFRAME_LE,
	},  //should be change? phys:32 width:16	 // Planet 20120306
	.ops   = &tcc_spdif_ops,
};

static const struct snd_soc_component_driver tcc_spdif_component = {
	.name		= "tcc-spdif",
};
static int soc_tcc_spdif_probe(struct platform_device *pdev)
{
	struct tcc_spdif_data *tcc_spdif;
	u32 clk_rate = 44100;
#if defined(CONFIG_ARCH_TCC898X)
	int port_mux = 0;
#endif
	int ret = 0;


	alsa_dbg("%s() \n", __func__);


	tcc_spdif = devm_kzalloc(&pdev->dev, sizeof(*tcc_spdif), GFP_KERNEL);
	if (!tcc_spdif) {
		dev_err(&pdev->dev, "no memory for state\n");
		goto err;
	}
	dev_set_drvdata(&pdev->dev, tcc_spdif);
	tcc_spdif->id = of_alias_get_id(pdev->dev.of_node, "spdif");

	/* get spdif info. */
	tcc_spdif->spdif_reg = of_iomap(pdev->dev.of_node, 0);
	if (IS_ERR((void *)tcc_spdif->spdif_reg))
		tcc_spdif->spdif_reg = NULL;
	tcc_spdif->spdif_pclk = of_clk_get(pdev->dev.of_node, 0);
	if (IS_ERR(tcc_spdif->spdif_pclk))
		tcc_spdif->spdif_pclk = NULL;
	tcc_spdif->spdif_hclk = of_clk_get(pdev->dev.of_node, 1);
	if (IS_ERR(tcc_spdif->spdif_hclk))
		tcc_spdif->spdif_hclk = NULL;
	of_property_read_u32(pdev->dev.of_node, "clock-frequency", &clk_rate);
	tcc_spdif->spdif_clk_rate = (unsigned long)clk_rate;
	tcc_spdif->spdif_irq = platform_get_irq(pdev, 0);
#if defined(CONFIG_ARCH_TCC898X)
	of_property_read_u32(pdev->dev.of_node, "port-mux", &port_mux);
	tcc_spdif->dai_port = port_mux;
#elif defined(CONFIG_ARCH_TCC802X)
	of_property_read_u8_array(pdev->dev.of_node, "port-mux", tcc_spdif->dai_port,
							  of_property_count_elems_of_size(pdev->dev.of_node, "port-mux", sizeof(char)));
#endif



	ret = snd_soc_register_component(&pdev->dev, &tcc_spdif_component,
									 &tcc_snd_spdif, 1);
	if (ret)
		goto err_reg_component;

	if (tcc_spdif->id == 0)
		ret = tcc_pcm_platform_register(&pdev->dev);
#if defined(CONFIG_ARCH_TCC897X) || defined(CONFIG_ARCH_TCC570X)
	else
		ret = tcc_pcm_sub3_platform_register(&pdev->dev);
#else //TCC898x, TCC802x
	else if (tcc_spdif->id == 1)
		ret = tcc_pcm_sub1_platform_register(&pdev->dev);
#if defined(CONFIG_ARCH_TCC802X)
	else if (tcc_spdif->id == 2)
		ret = tcc_pcm_sub2_platform_register(&pdev->dev);
	else if (tcc_spdif->id == 3)
		ret = tcc_pcm_sub3_platform_register(&pdev->dev);
#endif
#endif

	if (ret)
		goto err_reg_pcm;

	return 0;
#if (1)
err_reg_pcm:
	snd_soc_unregister_component(&pdev->dev);
#endif

err_reg_component:
	if (tcc_spdif->spdif_hclk) {
		clk_put(tcc_spdif->spdif_hclk);
		tcc_spdif->spdif_hclk = NULL;
	}
	if (tcc_spdif->spdif_pclk) {
		clk_put(tcc_spdif->spdif_pclk);
		tcc_spdif->spdif_pclk = NULL;
	}

	devm_kfree(&pdev->dev, tcc_spdif);
err:
	return ret;
}

static int  soc_tcc_spdif_remove(struct platform_device *pdev)
{
	struct tcc_spdif_data *tcc_spdif = platform_get_drvdata(pdev);

#if (0)
	if (tcc_spdif->id == 0)
		tcc_iec958_pcm_platform_unregister(&pdev->dev);
	else
#if defined(CONFIG_ARCH_TCC897X) || defined(CONFIG_ARCH_TCC570X)
		tcc_iec958_pcm_sub3_platform_unregister(&pdev->dev);
#else
		tcc_iec958_pcm_sub1_platform_unregister(&pdev->dev);
#endif
#endif
	snd_soc_unregister_component(&pdev->dev);
	if (tcc_spdif->spdif_hclk) {
		clk_put(tcc_spdif->spdif_hclk);
		tcc_spdif->spdif_hclk = NULL;
	}
	if (tcc_spdif->spdif_pclk) {
		clk_put(tcc_spdif->spdif_pclk);
		tcc_spdif->spdif_pclk = NULL;
	}
	devm_kfree(&pdev->dev, tcc_spdif);

	return 0;
}

static struct of_device_id tcc_spdif_of_match[] = {
	{ .compatible = "telechips,spdif" },
	{ }
};

static SIMPLE_DEV_PM_OPS(tcc_spdif_pm_ops, tcc_spdif_suspend,
						 tcc_spdif_resume);

static struct platform_driver tcc_spdif_driver = {
	.driver = {
		.name = "tcc-spdif",
		.owner = THIS_MODULE,
		.pm = &tcc_spdif_pm_ops,
		.of_match_table	= of_match_ptr(tcc_spdif_of_match),
	},
	.probe = soc_tcc_spdif_probe,
	.remove = soc_tcc_spdif_remove,
};

static int __init snd_tcc_spdif_init(void)
{
	return platform_driver_register(&tcc_spdif_driver);
}
module_init(snd_tcc_spdif_init);

static void __exit snd_tcc_spdif_exit(void)
{
	return platform_driver_unregister(&tcc_spdif_driver);
}
module_exit(snd_tcc_spdif_exit);

/* Module information */
MODULE_AUTHOR("linux <linux@telechips.com>");
MODULE_DESCRIPTION("Telechips TCC SPDIF SoC Interface");
MODULE_LICENSE("GPL");
