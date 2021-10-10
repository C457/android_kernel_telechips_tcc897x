/*
 * linux/sound/soc/tcc/tcc-cdif.c
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
#define alsa_dbg(f, a...)  printk("== alsa-debug CDIF == " f, ##a)
#else
#define alsa_dbg(f, a...)  
#endif

#define spdif_writel	__raw_writel
#define spdif_readl	__raw_readl


struct tcc_cdif_data {
	unsigned int	id;

	void __iomem	*cdif_reg;
	//struct clk	*cdif_pclk;
	struct clk	*cdif_hclk;
	unsigned int	cdif_irq;
	unsigned long	cdif_clk_rate;

	ADMACDIF	ADMA_CDIF;	/* backup_regs */
	
};

static struct tcc_pcm_dma_params tcc_pcm_cdif = {
	.name       = "CDIF in",
	.dma_addr   = 0,
	.channel    = 0,
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

static int tcc_cdif_startup(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;

	alsa_dbg("tcc_cdif_startup() \n");
	if (!cpu_dai->active) {
	}

	return 0;
}

static int tcc_cdif_set_fmt(struct snd_soc_dai *cpu_dai, unsigned int fmt)
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

static int tcc_cdif_set_sysclk(struct snd_soc_dai *cpu_dai,
                                      int clk_id, unsigned int freq, int dir)
{
	alsa_dbg(" %s \n", __func__);

	if (clk_id != TCC_I2S_SYSCLK)
		return -ENODEV;  

	return 0;
}

static int tcc_cdif_hw_params(struct snd_pcm_substream *substream,
                                 struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;

	alsa_dbg(" %s \n", __func__);

	cpu_dai->capture_dma_data = &tcc_pcm_cdif;
	
	return 0;
}


static int tcc_cdif_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *dai)
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

static void tcc_cdif_shutdown(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	alsa_dbg(" %s \n", __func__);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {        
	} 
	else {
	}
}

static int tcc_cdif_suspend(struct snd_soc_dai *dai)
{
	struct tcc_cdif_data *tcc_cdif = dev_get_drvdata(dai->dev);
	volatile PADMACDIF pADMA_CDIF = (volatile PADMACDIF)(tcc_cdif->cdif_reg);

	alsa_dbg(" %s \n", __func__);

	tcc_cdif->ADMA_CDIF.CICR  = pADMA_CDIF->CICR;

	if(tcc_cdif->cdif_hclk)
		clk_disable_unprepare(tcc_cdif->cdif_hclk);

	return 0;
}

static int tcc_cdif_resume(struct snd_soc_dai *dai)
{
	struct tcc_cdif_data *tcc_cdif = dev_get_drvdata(dai->dev);
	volatile PADMACDIF pADMA_CDIF = (volatile PADMACDIF)(tcc_cdif->cdif_reg);

	alsa_dbg(" %s \n", __func__);
	if(tcc_cdif->cdif_hclk)
		clk_prepare_enable(tcc_cdif->cdif_hclk);

	pADMA_CDIF->CICR = tcc_cdif->ADMA_CDIF.CICR;

	return 0;
}

static int tcc_cdif_probe(struct snd_soc_dai *dai)
{
	struct tcc_cdif_data *tcc_cdif = dev_get_drvdata(dai->dev);

	alsa_dbg("%s() \n", __func__);

	/* clock enable */
	if (tcc_cdif->cdif_hclk)
		clk_prepare_enable(tcc_cdif->cdif_hclk);

	//if (tcc_cdif->cdif_pclk) {
	//	clk_set_rate(tcc_cdif->cdif_pclk, 44100*512);
	//	clk_prepare_enable(tcc_cdif->cdif_pclk);
	//	tcc_spdif_set_clock(dai, tcc_cdif->cdif_clk_rate, STREAM_PLAYBACK);	// Planet 20150812 S/PDIF_Rx
	//}

	return 0;
}

static struct snd_soc_dai_ops tcc_cdif_ops = {
	.set_sysclk = tcc_cdif_set_sysclk,
	.set_fmt    = tcc_cdif_set_fmt,
	.startup    = tcc_cdif_startup,
	.shutdown   = tcc_cdif_shutdown,
	.hw_params  = tcc_cdif_hw_params,
	.trigger    = tcc_cdif_trigger,
};

static struct snd_soc_dai_driver tcc_snd_cdif = {
	.name = "tcc-dai-cdif",
	.probe = tcc_cdif_probe,
	.suspend = tcc_cdif_suspend,
	.resume  = tcc_cdif_resume,

	.capture = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = TCC_I2S_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_U16_LE},
	.ops   = &tcc_cdif_ops,
};

static const struct snd_soc_component_driver tcc_cdif_component = {
	.name		= "tcc-cdif",
};
static int soc_tcc_cdif_probe(struct platform_device *pdev)
{
	struct tcc_cdif_data *tcc_cdif;
	u32 clk_rate;
	int ret = 0;

	tcc_cdif = devm_kzalloc(&pdev->dev, sizeof(*tcc_cdif), GFP_KERNEL);
	if (!tcc_cdif) {
		dev_err(&pdev->dev, "no memory for state\n");
		goto err;
	}
	dev_set_drvdata(&pdev->dev, tcc_cdif);
	tcc_cdif->id = of_alias_get_id(pdev->dev.of_node, "cdif");

	/* get cdif info. */
	tcc_cdif->cdif_reg = of_iomap(pdev->dev.of_node, 0);
	if (IS_ERR((void *)tcc_cdif->cdif_reg))
		tcc_cdif->cdif_reg = NULL;
	//tcc_cdif->cdif_pclk = of_clk_get(pdev->dev.of_node, 0);
	//if (IS_ERR(tcc_cdif->cdif_pclk))
	//	tcc_cdif->cdif_pclk = NULL;
	tcc_cdif->cdif_hclk = of_clk_get(pdev->dev.of_node, 1);
	if (IS_ERR(tcc_cdif->cdif_hclk))
		tcc_cdif->cdif_hclk = NULL;
	of_property_read_u32(pdev->dev.of_node, "clock-frequency", &clk_rate);
	tcc_cdif->cdif_clk_rate = (unsigned long)clk_rate;
	tcc_cdif->cdif_irq = platform_get_irq(pdev, 0);

	ret = snd_soc_register_component(&pdev->dev, &tcc_cdif_component,
			&tcc_snd_cdif, 1);
	if (ret)
		goto err_reg_component;

#if (1)
	alsa_dbg("== alsa-debug == %s() tcc_cdif->id:%d \n", __func__, tcc_cdif->id);
	if (tcc_cdif->id == 0)
		ret = tcc_pcm_platform_register(&pdev->dev);
	else
		ret = tcc_pcm_sub3_platform_register(&pdev->dev);
	
	if (ret)
		goto err_reg_pcm;
#endif

	return 0;
#if (1)
err_reg_pcm:
	snd_soc_unregister_component(&pdev->dev);
#endif

err_reg_component:
	if (tcc_cdif->cdif_hclk) {
		clk_put(tcc_cdif->cdif_hclk);
		tcc_cdif->cdif_hclk = NULL;
	}
	//if (tcc_cdif->cdif_pclk) {
	//	clk_put(tcc_cdif->cdif_pclk);
	//	tcc_cdif->cdif_pclk = NULL;
	//}
	
	devm_kfree(&pdev->dev, tcc_cdif);
err:
	return ret;
}

static int  soc_tcc_cdif_remove(struct platform_device *pdev)
{
	struct tcc_cdif_data *tcc_cdif = platform_get_drvdata(pdev);

#if (0)
	if (tcc_spdif->id == 0)
		tcc_iec958_pcm_platform_unregister(&pdev->dev);
	else
		tcc_iec958_pcm_sub3_platform_unregister(&pdev->dev);
#endif
	snd_soc_unregister_component(&pdev->dev);
	if (tcc_cdif->cdif_hclk) {
		clk_put(tcc_cdif->cdif_hclk);
		tcc_cdif->cdif_hclk = NULL;
	}
	//if (tcc_cdif->cdif_pclk) {
	//	clk_put(tcc_cdif->cdif_pclk);
	//	tcc_cdif->cdif_pclk = NULL;
	//}
	devm_kfree(&pdev->dev, tcc_cdif);

	return 0;
}

static struct of_device_id tcc_cdif_of_match[] = {
	{ .compatible = "telechips,cdif" },
	{ }
};

static int tcc_cdif_freeze(struct device *dev)
{
	struct tcc_cdif_data *tcc_cdif = dev_get_drvdata(dev);
	volatile PADMACDIF pADMA_CDIF = (volatile PADMACDIF)(tcc_cdif->cdif_reg);

	alsa_dbg(" %s \n", __func__);

	tcc_cdif->ADMA_CDIF.CICR  = pADMA_CDIF->CICR;

	if(tcc_cdif->cdif_hclk)
		clk_disable_unprepare(tcc_cdif->cdif_hclk);
	
	return 0;
}

static int tcc_cdif_restore(struct device *dev)
{
	struct tcc_cdif_data *tcc_cdif = dev_get_drvdata(dev);
	volatile PADMACDIF pADMA_CDIF = (volatile PADMACDIF)(tcc_cdif->cdif_reg);

	alsa_dbg(" %s \n", __func__);
	if(tcc_cdif->cdif_hclk)
		clk_prepare_enable(tcc_cdif->cdif_hclk);

	pADMA_CDIF->CICR = tcc_cdif->ADMA_CDIF.CICR;
	
	return 0;
}


static const struct dev_pm_ops tcc_cdif_pm_ops = {
    .freeze     = tcc_cdif_freeze,
    .thaw       = tcc_cdif_restore,
    .restore    = tcc_cdif_restore,
};

static struct platform_driver tcc_cdif_driver = {
	.driver = {
		.name = "tcc-cdif",
		.owner = THIS_MODULE,
		.pm = &tcc_cdif_pm_ops,
		.of_match_table	= of_match_ptr(tcc_cdif_of_match),
	},
	.probe = soc_tcc_cdif_probe,
	.remove = soc_tcc_cdif_remove,
};

static int __init snd_tcc_cdif_init(void)
{
    return platform_driver_register(&tcc_cdif_driver);
}
module_init(snd_tcc_cdif_init);

static void __exit snd_tcc_cdif_exit(void)
{
    return platform_driver_unregister(&tcc_cdif_driver);
}
module_exit(snd_tcc_cdif_exit);

/* Module information */
MODULE_AUTHOR("linux <linux@telechips.com>");
MODULE_DESCRIPTION("Telechips TCC CDIF SoC Interface");
MODULE_LICENSE("GPL");
