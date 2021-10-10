/*
 * ALSA SoC Audio driver
 *
 * Copyright 2010 Telechips
 *
 * Author: 
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <linux/gpio.h>
#include <mach/gpio.h>
#include <linux/slab.h>
#include <asm/mach-types.h>
#include "nxp.h"

#if defined(CONFIG_DAUDIO)
#include "mach/daudio.h"
#endif

#define NXP_VERSION "0.01"

#if defined(CONFIG_DAUDIO_20140220)
	#if defined(CONFIG_SND_SOC_TCC_MULTICHANNEL)
		#define NXP_RATES (SNDRV_PCM_RATE_8000_192000)
	#else
		#define NXP_RATES (SNDRV_PCM_RATE_8000_96000)
	#endif
#else
	#define NXP_RATES (SNDRV_PCM_RATE_8000_96000)
#endif

/** 
* @author sjpark@cleinsoft
* @date 2014/1/24
* Add PCM Format (SNDRV_PCM_FMTBIT_U16_LE) for CDIF
**/
#define NXP_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
					SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_U16_LE) 

#if 0
#define alsa_dbg(f, a...)  printk("== alsa-debug == " f, ##a)
#else
#define alsa_dbg(f, a...)
#endif

#define print_func alsa_dbg("NXP : [%s]\n", __func__);

/* codec private data */
struct nxp_priv {
	unsigned int sysclk;
};

static int nxp_hw_params(struct snd_pcm_substream *substream,
                            struct snd_pcm_hw_params *params, 
                            struct snd_soc_dai *dai)
{
	print_func;
    return 0;
}

static int nxp_mute(struct snd_soc_dai *dai, int mute)
{
	print_func;
	return 0;
}

static int nxp_set_dai_sysclk(struct snd_soc_dai *codec_dai,
                int clk_id, unsigned int freq, int dir)
{
	print_func;
    return 0;
}

static int nxp_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	print_func;
    return 0;
}

static int nxp_pcm_prepare(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	/* set active */
	print_func;
	return 0;
}

static int nxp_startup(struct snd_pcm_substream *substream,
			    struct snd_soc_dai *dai)
{
	print_func;
#ifndef CONFIG_DAUDIO_KK	//NOT USED
/** 
* @author sjpark@cleinsoft
* @date 2013/10/23
* Set NAVI_VOICE_ON  port control.
**/
#if defined(CONFIG_DAUDIO) && (DAUDIO_NAVI_VOC_CTRL == CODEC_NXP)
	gpio_direction_output(GPIO_NAVI_VOC_ON, PORT_HIGH);
#endif
#endif
	return 0;
}

static void nxp_shutdown(struct snd_pcm_substream *substream,
			    struct snd_soc_dai *dai)
{
	print_func;
#ifndef CONFIG_DAUDIO_KK	//NOT USED	
/** 
* @author sjpark@cleinsoft
* @date 2013/10/23
* Set NAVI_VOICE_ON  port control.
**/
#if defined(CONFIG_DAUDIO) && (DAUDIO_NAVI_VOC_CTRL == CODEC_NXP)
	gpio_direction_output(GPIO_NAVI_VOC_ON, PORT_LOW);
#endif
#endif
}

static int nxp_hw_free(struct snd_pcm_substream *substream)
{
	print_func;
	return 0;
}

static struct snd_soc_dai_ops nxp_dai_ops = {
	.hw_params		= nxp_hw_params,
	.startup 		= nxp_startup,
	.shutdown		= nxp_shutdown,
	.digital_mute	= nxp_mute,
	.set_sysclk		= nxp_set_dai_sysclk,
	.set_fmt		= nxp_set_dai_fmt,
	.hw_free		= nxp_hw_free,
};

static struct snd_soc_dai_driver nxp_dai = {
	.name = CODEC_NXP_DAI_NAME,
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
#if (DAUDIO_MULTICHANNEL_USE == CODEC_NXP)
		.channels_max = 8,
#else
		.channels_max = 2,
#endif
		.rates = NXP_RATES,
		.formats = NXP_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
#if (DAUDIO_MULTICHANNEL_USE == CODEC_NXP)
		.channels_max = 8,
#else
		.channels_max = 2,
#endif
		.rates = NXP_RATES,
		.formats = NXP_FORMATS,
	},
	.ops = &nxp_dai_ops,
};

static int nxp_suspend(struct snd_soc_codec *codec, pm_message_t state)
{
	print_func;
	return 0;
}

static int nxp_resume(struct snd_soc_codec *codec)
{
	print_func;
	return 0;
}

static int nxp_probe(struct snd_soc_codec *codec)
{
#ifndef CONFIG_DAUDIO_KK	//NOT USED
/** 
* @author sjpark@cleinsoft
* @date 2013/10/23
* Set NAVI_VOICE_ON  port control.
**/
#if defined(CONFIG_DAUDIO) && (DAUDIO_NAVI_VOC_CTRL == CODEC_NXP)
	tcc_gpio_config(GPIO_NAVI_VOC_ON, GPIO_FN0| GPIO_OUTPUT| GPIO_LOW);
	gpio_request(GPIO_NAVI_VOC_ON, "NAVI_VIOCE_ON");
#endif
#endif
	return 0;
}

/* power down chip */
static int nxp_remove(struct snd_soc_codec *codec)
{
#ifndef CONFIG_DAUDIO_KK	//NOT USED
/** 
* @author sjpark@cleinsoft
* @date 2013/10/23
* Set NAVI_VOICE_ON  port control.
**/
#if defined(CONFIG_DAUDIO) && (DAUDIO_NAVI_VOC_CTRL == CODEC_NXP)
	gpio_direction_output(GPIO_NAVI_VOC_ON, PORT_LOW);
#endif
#endif
	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_nxp = {
	.probe = 	nxp_probe,
	.remove = 	nxp_remove,
	.suspend = 	nxp_suspend,
	.resume =	nxp_resume,
};

static int nxp_codec_probe(struct platform_device *pdev)
{
	int ret;
	print_func;
	ret =  snd_soc_register_codec(&pdev->dev, &soc_codec_dev_nxp, &nxp_dai, 1);
	if (ret) {
		alsa_dbg("Failed to register codec\n");
		return -1;
	}
	return ret;
}

static int nxp_codec_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static struct platform_driver nxp_codec_driver = {
	.driver = {
		.name = CODEC_NXP_NAME,
		.owner = THIS_MODULE,
	},
	.probe = nxp_codec_probe,
	.remove = nxp_codec_remove,
};

static int __init nxp_modinit(void)
{
	return platform_driver_register(&nxp_codec_driver);
}
module_init(nxp_modinit);

static void __exit nxp_exit(void)
{
	platform_driver_unregister(&nxp_codec_driver);
}
module_exit(nxp_exit);

MODULE_DESCRIPTION("ASoC NXP driver");
MODULE_AUTHOR("Richard Purdie");
MODULE_LICENSE("GPL");

