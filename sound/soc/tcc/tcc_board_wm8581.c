/*
 * linux/sound/soc/tcc/tcc_board.c
 *
 * Author:  <linux@telechips.com>
 * Created: Nov 30, 2007
 * Description: SoC audio for TCCxx
 *
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
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/init.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/gpio.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <mach/hardware.h>
#include <mach/bsp.h>
#include <asm/mach-types.h>

#include "tcc/tca_tcchwcontrol.h"
#include "../codecs/wm8581.h"
#include "tcc-pcm.h"
#include "tcc-i2s.h"

#include <linux/of.h>
#include <linux/of_gpio.h>

#define TCC_HP        0
#define TCC_LINE      1
#define TCC_HP_OFF    3
#define TCC_SPK_ON    0
#define TCC_SPK_OFF   1

//#define EXT_CODEC_ON

#undef alsa_dbg
#if 0
#define alsa_dbg(fmt,arg...)    printk("== alsa-debug == "fmt,##arg)
#else
#define alsa_dbg(fmt,arg...)
#endif

static int tcc_jack_func;
static int tcc_spk_func;


static void tcc_ext_control(struct snd_soc_codec *codec)
{
	int spk = 0, mic = 0, line = 0, hp = 0, hs = 0;
    struct snd_soc_dapm_context *dapm = &codec->dapm;

	/* set up jack connection */
	switch (tcc_jack_func) {
		case TCC_HP:
    		hp = 1;
    		snd_soc_dapm_disable_pin(dapm, "Line Jack");
    		snd_soc_dapm_enable_pin(dapm, "Headphone Jack");
    		break;
    	case TCC_LINE:
    		line = 1;
    		snd_soc_dapm_enable_pin(dapm, "Line Jack");
    		snd_soc_dapm_disable_pin(dapm, "Headphone Jack");
    		break;
	}
	snd_soc_dapm_enable_pin(dapm, "Line Jack");
	snd_soc_dapm_enable_pin(dapm, "Headphone Jack");

	/* signal a DAPM event */
	snd_soc_dapm_sync(dapm);
}

static int tcc_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;

	/* check the jack status at stream startup */
	tcc_ext_control(codec);
	return 0;
}

/* we need to unmute the HP at shutdown as the mute burns power on tcc83x */
static void tcc_shutdown(struct snd_pcm_substream *substream)
{
}

static int tcc_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned int clk = 0;
	int ret = 0;

	switch (params_rate(params)) {
	case 8000:
	case 16000:
	case 48000:
	case 96000:
	case 192000:
		clk = 12288000;
		break;
	case 11025:
	case 22050:
    case 32000:
	case 44100:
    default:
		clk = 11289600;
		break;
	}
 
	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0){
        printk("tcc_hw_params()  codec_dai: set_fmt error[%d]\n", ret);
		return ret;
    }

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0){
        printk("tcc_hw_params()  cpu_dai: set_fmt error[%d]\n", ret);
		return ret;
    }

	/* set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_dai, WM8581_CLKSRC_MCLK, clk,
		SND_SOC_CLOCK_IN);
		if (ret < 0) {
        printk("tcc_hw_params()  codec_dai: sysclk error[%d]\n", ret);
		return ret;
    }

	/* set the I2S system clock as input (unused) */
//       ret = cpu_dai->dai_ops.set_sysclk(cpu_dai, PXA2XX_I2S_SYSCLK, 0,SND_SOC_CLOCK_IN);    
    //	ret = cpu_dai->dai_ops.set_sysclk(cpu_dai, 0, 0, SND_SOC_CLOCK_IN);
	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, 0, SND_SOC_CLOCK_IN);
	if (ret < 0) {
        printk("tcc_hw_params()  cpu_dai: sysclk error[%d]\n", ret);
		return ret;
    }
 
	return 0;
}

static struct snd_soc_ops tcc_ops = {
	.startup = tcc_startup,
	.hw_params = tcc_hw_params,
	.shutdown = tcc_shutdown,
};

static int tcc_get_jack(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = tcc_jack_func;
	return 0;
}

static int tcc_set_jack(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	if (tcc_jack_func == ucontrol->value.integer.value[0])
		return 0;

	tcc_jack_func = ucontrol->value.integer.value[0];
	tcc_ext_control(codec);
	return 1;
}

static int tcc_get_spk(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = tcc_spk_func;
	return 0;
}

static int tcc_set_spk(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);

	if (tcc_spk_func == ucontrol->value.integer.value[0])
		return 0;

	tcc_spk_func = ucontrol->value.integer.value[0];
	tcc_ext_control(codec);
	return 1;
}
/* tcc machine dapm widgets */
static const struct snd_soc_dapm_widget wm8581_dapm_widgets[] = {
SND_SOC_DAPM_HP("Headphone Jack", NULL),
SND_SOC_DAPM_LINE("Line Jack", NULL),
};

/* tcc machine audio map (connections to the codec pins) */
static const struct snd_soc_dapm_route audio_map[] = {
	/* headphone connected to LHPOUT1, RHPOUT1 */
	{"Headphone Jack", NULL, "VOUT1L"},
	{"Headphone Jack", NULL, "VOUT1R"},
	/* Same as the above but no mic bias for line signals */
	{"AINL", NULL, "Line Jack"},
	{"AINR", NULL, "Line Jack"},
};

static const char *jack_function[] = {"Headphone", "Line", "Off"};
static const char *spk_function[] = {"On", "Off"};
static const struct soc_enum tcc_enum[] = {
	SOC_ENUM_SINGLE_EXT(3, jack_function),
	SOC_ENUM_SINGLE_EXT(2, spk_function),
};

static const struct snd_kcontrol_new wm8581_tcc_controls[] = {
	SOC_ENUM_EXT("Jack Function", tcc_enum[0], tcc_get_jack,
		tcc_set_jack),
	SOC_ENUM_EXT("Speaker Function", tcc_enum[1], tcc_get_spk,
		tcc_set_spk),
};

/*
 * Logic for a wm8581 as connected on a Sharp SL-C7x0 Device
 */
static int tcc_wm8581_init(struct snd_soc_pcm_runtime *rtd)
{
	int err;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;

    alsa_dbg("%s() \n", __func__);

	/* Add tcc specific controls */
	err = snd_soc_add_codec_controls(codec, wm8581_tcc_controls, ARRAY_SIZE(wm8581_tcc_controls));
	if (err < 0)
		return err;

	/* Add tcc specific widgets */
    snd_soc_dapm_new_controls(dapm, wm8581_dapm_widgets, 
                               ARRAY_SIZE(wm8581_dapm_widgets));

	/* Set up Telechips specific audio path audio_map */
	snd_soc_dapm_add_routes(dapm, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_sync(dapm);
	return 0;
}

static int tcc_iec958_dummy_init(struct snd_soc_pcm_runtime *rtd)
{
    return 0;
}


/* tcc digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link tcc_dai[] = {
    {
        .name = "WM8581",
        .stream_name = "WM8581_PCM",
        .codec_dai_name = "wm8581-hifi",
        .init = tcc_wm8581_init,
        .ops = &tcc_ops,
    },
    {
        .name = "TCC-SPDIF-CH0",
        .stream_name = "IEC958",
        .codec_dai_name = "IEC958",
        .init = tcc_iec958_dummy_init,
        .ops = &tcc_ops,
    },
};


/* tcc audio machine driver */
static struct snd_soc_card tcc_soc_card = {
	.driver_name      = "TCC Audio",
    .long_name = "Telechips Board",
	.dai_link  = tcc_dai,
	.num_links = ARRAY_SIZE(tcc_dai),
};


static int tcc_audio_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &tcc_soc_card;
	int ret, codec_pwr, ext_codec_pwr, line_out_mute, i;

#ifdef EXT_CODEC_ON
	ext_codec_pwr = of_get_named_gpio(pdev->dev.of_node, "ext_codec_pwr-gpios", 0);
	if (gpio_is_valid(ext_codec_pwr)) {
		gpio_request(ext_codec_pwr, "EXT_CODEC_ON");
		gpio_direction_output(ext_codec_pwr, 1);
	}
#else
	codec_pwr = of_get_named_gpio(pdev->dev.of_node, "codec_pwr-gpios", 0);
	if (gpio_is_valid(codec_pwr)) {
		gpio_request(codec_pwr, "CODEC_ON");
		gpio_direction_output(codec_pwr, 1);
	}
#endif
	line_out_mute = of_get_named_gpio(pdev->dev.of_node, "codec_mute-gpios", 0);
	if (gpio_is_valid(line_out_mute)) {
		gpio_request(line_out_mute, "UN_MUTE");
		gpio_direction_output(line_out_mute, 0);
	}

	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);

	ret = snd_soc_of_parse_card_name(card, "telechips,model");
	if (ret)
		return ret;

//	ret = snd_soc_of_parse_audio_routing(card, "telechips,audio-routing");
//	if (ret)
//		return ret;

	for (i=0 ; i<card->num_links ; i++) {
		tcc_dai[i].codec_of_node = of_parse_phandle(pdev->dev.of_node,
				"telechips,audio-codec", i);
		if (!tcc_dai[i].codec_of_node)
			continue;
		tcc_dai[i].cpu_of_node = of_parse_phandle(pdev->dev.of_node,
				"telechips,dai-controller", i);
		if (!tcc_dai[i].cpu_of_node)
			continue;

		tcc_dai[i].platform_of_node = tcc_dai[i].cpu_of_node;
	}
	//tcc_dai[0].platform_of_node = tcc_dai[0].cpu_of_node;

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
			ret);
		return ret;
	}

	return 0;
}

static int tcc_audio_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	snd_soc_unregister_card(card);
	return 0;
}

static const struct of_device_id tcc_audio_of_match[] = {
	{ .compatible = "telechips,snd-wm8581", },
	{},
};

static struct platform_driver tcc_audio_driver = {
	.driver = {
		.name = "tcc-audio-wm8581",
		.owner = THIS_MODULE,
		.of_match_table = tcc_audio_of_match,
	},
	.probe = tcc_audio_probe,
	.remove = tcc_audio_remove,
};
module_platform_driver(tcc_audio_driver);

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:tcc-audio-wm8581");
MODULE_DEVICE_TABLE(of, tcc_audio_of_match);