/*!
 * Author: sjpark@cleinsoft
 * Created: 2013.08.13
 * Description: FM1288 ALSA Driver
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
#include <mach/irqs.h>
#include <mach/bsp.h>
//#include <mach/tca_ckc.h>
#include <asm/mach-types.h>

#include "tcc/tca_tcchwcontrol.h"
#include "../codecs/fm1288.h"
//#include "tcc-pcm-ch1.h"
//#include "tcc-i2s-ch1.h"

#if defined(CONFIG_DAUDIO)
#include "mach/daudio.h"
#endif

#undef alsa_dbg
#if 1
#define alsa_dbg(fmt,arg...)    printk("== alsa-debug == "fmt,##arg)
#else
#define alsa_dbg(fmt,arg...)
#endif

#define print_func alsa_dbg("[%s]\n", __func__);

static struct platform_device *daudio_snd_device;

static int tcc_startup(struct snd_pcm_substream *substream)
{
	print_func;
	return 0;
}

/* we need to unmute the HP at shutdown as the mute burns power on tcc83x */
static void tcc_shutdown(struct snd_pcm_substream *substream)
{
	print_func;
}

static int tcc_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned int clk = 0;
	int ret = 0, rate = 0;

	print_func;

	switch ((rate = params_rate(params))) {
		case 8000:
		case 16000:
		case 48000:
		case 96000:
			clk = 8192000;
			break;
		case 11025:
		case 22050:
		case 32000:
		case 44100:
		default:
			clk = 11289600;
			break;
	}

	alsa_dbg("hw params - rate : %d\n", rate );

	if(params->reserved[SNDRV_PCM_HW_PARAM_EFLAG0] & SNDRV_PCM_MODE_PCMIF) {
		alsa_dbg("DSP mode format set\n");
		/* set codec DAI configuration */
		ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_DSP_A |
								  SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
		if (ret < 0) {
			printk("tcc_hw_params()  codec_dai: set_fmt error[%d]\n", ret);
			return ret;
		}

		/* set cpu DAI configuration */
		ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_DSP_A |
								  SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
		if (ret < 0) {
			printk("tcc_hw_params()  cpu_dai: set_fmt error[%d]\n", ret);
			return ret;
		}
	}
	else {
		/* set codec DAI configuration */
		alsa_dbg("IIS mode format set\n");
		ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
								  SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
		if (ret < 0) {
			printk("tcc_hw_params()  codec_dai: set_fmt error[%d]\n", ret);
			return ret;
		}

		/* set cpu DAI configuration */
		ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
								  SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
		if (ret < 0) {
			printk("tcc_hw_params()  cpu_dai: set_fmt error[%d]\n", ret);
			return ret;
		}
	}

	/* set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_dai, FM1288_SYSCLK_XTAL, clk,
								 SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk("tcc_hw_params()  codec_dai: sysclk error[%d]\n", ret);
		return ret;
	}

	/* set the I2S system clock as input (unused) */
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

static int tcc_iec958_dummy_init(struct snd_soc_pcm_runtime *rtd)
{
	print_func;
	return 0;
}

static int i2s_dummy_init(struct snd_soc_pcm_runtime *rtd)
{
	print_func;
	return 0;
}

/* tcc digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link daudio_dai[] = {

	/*  BT / MODEM  - Sub 2 */
	{
		// Audio0 I2S Capture/Player
		.name = "Audio0",
		.stream_name = "Audio0",
		.codec_dai_name = "i2s-dai-dummy",
		.init = &i2s_dummy_init,
		.ops = &tcc_ops,
	},

	/* NAVI /SOUND HOUND SPDIF */
	{
		// Audio0 SPDIF Capture/player
		.name = "Audio0-spdif",
		.stream_name = "Audio0-spdif",
		.codec_dai_name = "IEC958",
		.init = tcc_iec958_dummy_init,
		.ops = &tcc_ops,
	},

	/* CD / MEDIA - Sub 3 */
	{
		// Audio1 I2S Capture/Player
		.name = "Audio1",
		.stream_name = "Audio1",
		.codec_dai_name = "i2s-dai-dummy",
		.init = &i2s_dummy_init,
		.ops = &tcc_ops,
	},

	/* MIC / SPK OUT - Sub 1 */
	{
		// Audio1 I2S Capture/Player
		.name = "Audio2",
		.stream_name = "Audio2",
		.codec_dai_name = "i2s-dai-dummy",
		.init = &i2s_dummy_init,
		.ops = &tcc_ops,
	}
#if defined(CONFIG_SND_SOC_TCC_CDIF)
	,
	{
		.name = "TCC-CDIF-CH1",
		.stream_name = "cdif-dai-dummy",
		.codec_dai_name = "cdif-dai-dummy",
		.init = tcc_iec958_dummy_init, //tcc_codec_dummy_init,
		.ops = &tcc_ops,
	}
#endif
};

/* tcc audio machine driver */
static struct snd_soc_card daudio_soc_card = {
	.driver_name      = "D-Audio Snd Board",
	.long_name = "D-Audio Snd Board",
	.dai_link  = daudio_dai,
	.num_links = ARRAY_SIZE(daudio_dai),
};

static int tcc_audio_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &daudio_soc_card;
	struct device_node *dnode = NULL;

	int ret = 0, i = 0;

	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);

	ret = snd_soc_of_parse_card_name(card, "telechips,model");
	if (ret)
		return ret;

//	ret = snd_soc_of_parse_audio_routing(card, "telechips,audio-routing");
//	if (ret)
//		return ret;

	i = 0;
	while(1) {
		dnode = of_parse_phandle(pdev->dev.of_node,
								 "telechips,audio-codec", i);
		if(!dnode)break;
		dnode = of_parse_phandle(pdev->dev.of_node,
								 "telechips,dai-controller", i);
		if(!dnode)break;
		i++;
	}

	if(i > card->num_links) {
		printk("==%s== The number of tcc_dai[%d] doesn't match phandle count[%d]\n", __func__, card->num_links, i);
	}
	else {
		card->num_links = i;
	}

	for (i = 0 ; i < card->num_links ; i++) {
		daudio_dai[i].codec_of_node = of_parse_phandle(pdev->dev.of_node,
									  "telechips,audio-codec", i);
		if (!daudio_dai[i].codec_of_node)
			continue;
		//printk("daudio_dai[%d].codec_of_node->name=%s\n", i, daudio_dai[i].codec_of_node->name);
		daudio_dai[i].cpu_of_node = of_parse_phandle(pdev->dev.of_node,
									"telechips,dai-controller", i);
		//printk("daudio_dai[%d].cpu_of_node->name=%s\n", i, daudio_dai[i].cpu_of_node->name);
		if (!daudio_dai[i].cpu_of_node)
			continue;

		daudio_dai[i].ops = &tcc_ops;

		daudio_dai[i].platform_of_node = daudio_dai[i].cpu_of_node;
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
	{ .compatible = "telechips,snd-daudio-dummy", },
	{},
};

static struct platform_driver tcc_audio_driver = {
	.driver = {
		.name = "tcc-audio-dummy",
		.owner = THIS_MODULE,
		.of_match_table = tcc_audio_of_match,
	},
	.probe = tcc_audio_probe,
	.remove = tcc_audio_remove,
};
module_platform_driver(tcc_audio_driver);

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:tcc-audio-dummy");
MODULE_DEVICE_TABLE(of, tcc_audio_of_match);


