/*
 * linux/sound/soc/tcc/tcc-i2s-dsp.c
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
#include <sound/pcm_params.h>

#include <sound/initval.h>
#include <sound/soc.h>
//#include <mach/hardware.h>
//#include <mach/bsp.h>
//#include <mach/tca_i2s.h>

#include "tcc-pcm.h"
//#include "tcc-i2s.h"
#include "tcc/tca_tcchwcontrol.h"
#if defined(CONFIG_SND_SOC_TCC_MULTICHANNEL)
#define TCC_I2S_RATES   SNDRV_PCM_RATE_8000_192000
#else
#define TCC_I2S_RATES   SNDRV_PCM_RATE_8000_96000
#endif

#undef alsa_dbg
#if 0 
#define alsa_dbg(f, a...)  printk("== alsa-debug I2S == " f, ##a)
#else
#define alsa_dbg(f, a...)  
#endif

struct tcc_i2s_data {
    unsigned int    id;
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
			break;
	}
	return 0;
}

static int tcc_i2s_set_dai_sysclk(struct snd_soc_dai *cpu_dai,
                                      int clk_id, unsigned int freq, int dir)
{
	alsa_dbg(" %s \n", __func__);

	//if (clk_id != TCC_I2S_SYSCLK)
	//	return -ENODEV;  

	return 0;
}
static int tcc_i2s_hw_params(struct snd_pcm_substream *substream,
                                 struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct tcc_pcm_dma_params *dma_data;
	struct tcc_i2s_data *tcc_i2s = dev_get_drvdata(dai->dev);
    snd_pcm_format_t format=params_format(params);
    unsigned reg_value;

	alsa_dbg("%s check device(%d) \n",__func__,substream->pcm->device);

	if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		dma_data = &tcc_i2s_pcm_stereo_out;
	else if(substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		dma_data = &tcc_i2s_pcm_stereo_in;

	snd_soc_dai_set_dma_data(cpu_dai, substream, dma_data);
    
    if (format == SNDRV_PCM_FORMAT_S24_LE) {
                // ???￤ﾼﾳ￁ﾤ
        alsa_dbg("%s %d SNDRV_PCM_FORMAT_S24_LE \n",__func__);
    }
    else{
                // ???￤ﾼﾳ￁ﾤ
        alsa_dbg("%s %d SNDRV_PCM_FORMAT_S16_LE \n",__func__);
    }

                    // ???￤ﾼﾳ￁ﾤ ?ccommand
		// Set DAI clock
//        DAI ￅﾬ????￁ﾤ


	alsa_dbg("=====================\n");
	alsa_dbg("= rate        : %d\n", params_rate(params));
	alsa_dbg("= channels    : %d\n", params_channels(params));
	alsa_dbg("= period_size : %d\n", params_period_size(params));

	return 0;
}

static int tcc_i2s_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *dai)
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

static void tcc_i2s_shutdown(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	alsa_dbg(" %s \n", __func__);
}

static int tcc_i2s_suspend(struct device *dev)
{
    alsa_dbg(" %s \n", __func__);
	return 0;
}


static int tcc_i2s_resume(struct device *dev)
{
    alsa_dbg(" %s \n", __func__);
	return 0;
}

static int tcc_i2s_probe(struct snd_soc_dai *dai)
{
	struct tcc_i2s_data *tcc_i2s = dev_get_drvdata(dai->dev);
	unsigned reg_value;
	int masterMode =1;

	alsa_dbg("== alsa-debug == %s() \n", __func__);

	/* clock enable */

	/* init dai register */

    /* Select master/slave */

	/* dai receiver/transmitter disable */
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

static struct snd_soc_dai_driver tcc_i2s_dai_dsp = {
	.name = "tcc-dai-dsp",
	.probe = tcc_i2s_probe,
	.playback = {
		.channels_min = 2,
        .channels_min = 1,
        .channels_max = 2,
		.rates = TCC_I2S_RATES,
        .formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_IEC958_SUBFRAME_LE,},  
	.capture = {
            .channels_min = 1,
            .channels_max = 4,
     		.rates = TCC_I2S_RATES,
            .formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,}, 
	.ops   = &tcc_i2s_ops,
};

static const struct snd_soc_component_driver tcc_i2s_component = {
	.name		= "tcc-i2s",
};
static int soc_tcc_i2s_probe(struct platform_device *pdev)
{
    struct tcc_i2s_data *tcc_i2s;
	u32 clk_rate;
	int ret = 0;
	tcc_i2s = devm_kzalloc(&pdev->dev, sizeof(*tcc_i2s), GFP_KERNEL);
	if (!tcc_i2s) {
		dev_err(&pdev->dev, "no memory for state\n");
		goto err;
	}
	dev_set_drvdata(&pdev->dev, tcc_i2s);
	tcc_i2s->id = of_alias_get_id(pdev->dev.of_node, "i2s");

	/* get dai info. */
	//of_property_read_u32(pdev->dev.of_node, "clock-frequency", &clk_rate);

	ret = snd_soc_register_component(&pdev->dev, &tcc_i2s_component,
			&tcc_i2s_dai_dsp, 1);
	if (ret)
	{
		printk("!!!!!!!! error i2s id=%d\r\n", tcc_i2s->id);
		goto err_reg_component;
	}

	printk("i2s id=%d\r\n", tcc_i2s->id);

	if (tcc_i2s->id == 10)
		ret = tcc_pcm_dsp_platform_register(&pdev->dev);
	else if (tcc_i2s->id == 11)
		ret = tcc_pcm_ch1_platform_register(&pdev->dev);
	else
		ret = tcc_pcm_ch2_platform_register(&pdev->dev);
	if (ret)
		goto err_reg_pcm;

	return 0;

err_reg_pcm:
	snd_soc_unregister_component(&pdev->dev);
    
err_reg_component:

    devm_kfree(&pdev->dev, tcc_i2s);

err:
	return ret;
}

static int  soc_tcc_i2s_remove(struct platform_device *pdev)
{
	struct tcc_i2s_data *tcc_i2s = platform_get_drvdata(pdev);

	if (tcc_i2s->id == 10)
		tcc_pcm_dsp_platform_unregister(&pdev->dev);
	else if (tcc_i2s->id == 11)
		tcc_pcm_ch1_platform_unregister(&pdev->dev);
	else
		tcc_pcm_ch2_platform_unregister(&pdev->dev);
	snd_soc_unregister_component(&pdev->dev);


	devm_kfree(&pdev->dev, tcc_i2s);

	return 0;
}

static struct of_device_id tcc_i2s_of_match[] = {
	{ .compatible = "telechips,i2s_dummy" },
	{ }
};

static SIMPLE_DEV_PM_OPS(tcc_i2s_pm_ops, tcc_i2s_suspend,
			 tcc_i2s_resume);

static struct platform_driver tcc_i2s_dsp_driver = {
	.driver = {
		.name = "tcc-dai-dsp",
		.owner = THIS_MODULE,
		.pm = &tcc_i2s_pm_ops,
		.of_match_table	= of_match_ptr(tcc_i2s_of_match),
	},
	.probe = soc_tcc_i2s_probe,
	.remove = soc_tcc_i2s_remove,
};

static int __init snd_tcc_i2s_init(void)
{
    return platform_driver_register(&tcc_i2s_dsp_driver);
}
module_init(snd_tcc_i2s_init);

static void __exit snd_tcc_i2s_exit(void)
{
    return platform_driver_unregister(&tcc_i2s_dsp_driver);
}
module_exit(snd_tcc_i2s_exit);

/* Module information */
MODULE_AUTHOR("linux <linux@telechips.com>");
MODULE_DESCRIPTION("Telechips TCC DSP I2S");
MODULE_LICENSE("GPL");
