/*
 * sound\soc\sunxi\sun50iw10-sndcodec.c
 * (C) Copyright 2014-2018
 * Reuuimlla Technology Co., Ltd. <www.reuuimllatech.com>
 * huangxin <huangxin@Reuuimllatech.com>
 * liushaohua <liushaohua@allwinnertech.com>
 * yumingfengng <yumingfeng@allwinnertech.com>
 * luguofang <luguofang@allwinnertech.com>
 *
 * some simple description for this code
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 */
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/input.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/pcm_params.h>
#include <sound/soc-dapm.h>
#include <linux/extcon.h>

#include "snd_sunxi_log.h"
#include "snd_sunxi_jack.h"
#include "sun50iw10-codec.h"

#define HLOG		"MACH"

struct sunxi_card_priv {
	struct snd_soc_card *card;
	struct snd_soc_component *component;
};

static const struct snd_kcontrol_new sunxi_card_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphone"),
	SOC_DAPM_PIN_SWITCH("HpSpeaker"),
	SOC_DAPM_PIN_SWITCH("LINEOUT"),
};

static const struct snd_soc_dapm_widget sunxi_card_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("HeadphoneMic", NULL),
	SND_SOC_DAPM_MIC("Main Mic", NULL),
};

static const struct snd_soc_dapm_route sunxi_card_routes[] = {
	{"MainMic Bias", NULL, "Main Mic"},
	{"MIC1", NULL, "MainMic Bias"},
	{"MIC2", NULL, "HeadphoneMic"},
};

/*
 * Card initialization
 */
static int sunxi_card_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_component *component = rtd->codec_dai->component;
	struct snd_soc_dapm_context *dapm = &component->dapm;

	struct sunxi_card_priv *priv = snd_soc_card_get_drvdata(rtd->card);

	priv->component = rtd->codec_dai->component;

	snd_soc_dapm_disable_pin(dapm, "HPOUTR");
	snd_soc_dapm_disable_pin(dapm, "HPOUTL");

	snd_soc_dapm_disable_pin(dapm, "LINEOUT");
	snd_soc_dapm_disable_pin(dapm, "HpSpeaker");
	snd_soc_dapm_disable_pin(dapm, "Headphone");

	snd_soc_dapm_sync(dapm);

	return 0;
}

static int sunxi_card_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int freq;
	int ret;
	int stream_flag;

	switch (params_rate(params)) {
	case	8000:
	case	12000:
	case	16000:
	case	24000:
	case	32000:
	case	48000:
	case	96000:
	case	192000:
		freq = 24576000;
		break;
	case	11025:
	case	22050:
	case	44100:
		freq = 22579200;
		break;
	default:
		SND_LOG_ERR(HLOG, "invalid rate setting\n");
		return -EINVAL;
	}

	/* the substream type: 0->playback, 1->capture */
	stream_flag = substream->stream;
	SND_LOG_DEBUG(HLOG, "stream_flag: %d\n", stream_flag);

	/* To surpport playback and capture func in different freq point */
	if (freq == 22579200) {
		if (stream_flag == 0) {
			ret = snd_soc_dai_set_sysclk(codec_dai, 0, freq, 0);
			if (ret < 0) {
				SND_LOG_ERR(HLOG, "set codec dai sysclk faided, freq:%d\n", freq);
				return ret;
			}
		}
	}

	if (freq == 22579200) {
		if (stream_flag == 1) {
			ret = snd_soc_dai_set_sysclk(codec_dai, 1, freq, 0);
			if (ret < 0) {
				SND_LOG_ERR(HLOG, "set codec dai sysclk faided, freq:%d\n", freq);
				return ret;
			}
		}
	}

	if (freq == 24576000) {
		if (stream_flag == 0) {
			ret = snd_soc_dai_set_sysclk(codec_dai, 2, freq, 0);
			if (ret < 0) {
				SND_LOG_ERR(HLOG, "set codec dai sysclk faided, freq:%d\n", freq);
				return ret;
			}
		}
	}

	if (freq == 24576000) {
			if (stream_flag == 1) {
			ret = snd_soc_dai_set_sysclk(codec_dai, 3, freq, 0);
			if (ret < 0) {
				SND_LOG_ERR(HLOG, "set codec dai sysclk faided, freq:%d\n", freq);
				return ret;
			}
		}
	}

	return 0;
}

static struct snd_soc_ops sunxi_card_ops = {
	.hw_params = sunxi_card_hw_params,
};

SND_SOC_DAILINK_DEFS(sun50iw10p1_dai_link,
	DAILINK_COMP_ARRAY(COMP_CPU("sunxi-dummy-cpudai")),
	DAILINK_COMP_ARRAY(COMP_CODEC("sunxi-internal-codec", "sun50iw10codec")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("sunxi-dummy-cpudai")));

static struct snd_soc_dai_link sunxi_card_dai_link[] = {
	{
		.name		= "audiocodec",
		.stream_name	= "SUNXI-CODEC",
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF
				| SND_SOC_DAIFMT_CBM_CFM,
		.init		= sunxi_card_init,
		.ops		= &sunxi_card_ops,
		SND_SOC_DAILINK_REG(sun50iw10p1_dai_link),
	},
};

static int sunxi_card_suspend(struct snd_soc_card *card)
{
	//struct sunxi_card_priv *priv = snd_soc_card_get_drvdata(card);

	SND_LOG_DEBUG(HLOG, "suspend\n");

	return 0;
}

static int sunxi_card_resume(struct snd_soc_card *card)
{
	//struct sunxi_card_priv *priv = snd_soc_card_get_drvdata(card);

	SND_LOG_DEBUG(HLOG, "resume\n");

	return 0;
}

static int simple_soc_probe(struct snd_soc_card *card)
{
	int ret;

	SND_LOG_DEBUG(HLOG, "\n");

	ret = snd_sunxi_jack_register(card);
	if (ret < 0) {
		SND_LOG_ERR(HLOG, "jack init failed\n");
		return ret;
	}

	return 0;
}

static int simple_soc_remove(struct snd_soc_card *card)
{
	SND_LOG_DEBUG(HLOG, "\n");

	snd_sunxi_jack_unregister(card);

	return 0;
}

static struct snd_soc_card snd_soc_sunxi_card = {
	.name			= "audiocodec",
	.owner			= THIS_MODULE,
	.dai_link		= sunxi_card_dai_link,
	.num_links		= ARRAY_SIZE(sunxi_card_dai_link),
	.suspend_post		= sunxi_card_suspend,
	.resume_post		= sunxi_card_resume,
};

static int sunxi_card_dev_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct sunxi_card_priv *priv = NULL;
	struct device_node *np = pdev->dev.of_node;
	struct snd_soc_card *card = &snd_soc_sunxi_card;
	struct snd_soc_dapm_context *dapm = &card->dapm;

	SND_LOG_INFO(HLOG, "register card begin\n");

	if (!np) {
		SND_LOG_ERR(HLOG, "can not get dt node for this device\n");
		return -EINVAL;
	}

	/* dai link */
	sunxi_card_dai_link[0].cpus->dai_name = NULL;
	sunxi_card_dai_link[0].cpus->of_node = of_parse_phandle(np,
					"sunxi,cpudai-controller", 0);
	if (!sunxi_card_dai_link[0].cpus->of_node) {
		SND_LOG_ERR(HLOG, "Property 'sunxi,cpudai-controller' missing or invalid\n");
		ret = -EINVAL;
		goto err_devm_kfree;
	} else {
		sunxi_card_dai_link[0].platforms->name = NULL;
		sunxi_card_dai_link[0].platforms->of_node =
				sunxi_card_dai_link[0].cpus->of_node;
	}
	sunxi_card_dai_link[0].codecs->name = NULL;
	sunxi_card_dai_link[0].codecs->of_node = of_parse_phandle(np,
						"sunxi,audio-codec", 0);
	if (!sunxi_card_dai_link[0].codecs->of_node) {
		SND_LOG_ERR(HLOG, "Property 'sunxi,audio-codec' missing or invalid\n");
		ret = -EINVAL;
		goto err_devm_kfree;
	}

	/* register the soc card */
	card->dev		= &pdev->dev;
	card->probe		= simple_soc_probe;
	card->remove		= simple_soc_remove;

	priv = devm_kzalloc(&pdev->dev,
		sizeof(struct sunxi_card_priv), GFP_KERNEL);
	if (!priv) {
		SND_LOG_ERR(HLOG, "devm_kzalloc failed %d\n", ret);
		return -ENOMEM;
	}
	priv->card = card;

	snd_soc_card_set_drvdata(card, priv);

	ret = snd_soc_register_card(card);
	if (ret) {
		SND_LOG_ERR(HLOG, "snd_soc_register_card failed %d\n", ret);
		goto err_devm_kfree;
	}

	ret = snd_soc_add_card_controls(card, sunxi_card_controls,
					ARRAY_SIZE(sunxi_card_controls));
	if (ret)
		SND_LOG_ERR(HLOG, "failed to register codec controls!\n");

	snd_soc_dapm_new_controls(dapm, sunxi_card_dapm_widgets,
				ARRAY_SIZE(sunxi_card_dapm_widgets));
	snd_soc_dapm_add_routes(dapm, sunxi_card_routes,
				ARRAY_SIZE(sunxi_card_routes));

	SND_LOG_DEBUG(HLOG, "0x310:0x%X,0x314:0x%X,0x318:0x%X,0x1C:0x%X,0x1D:0x%X\n",
			snd_soc_component_read32(priv->component, 0x310),
			snd_soc_component_read32(priv->component, 0x314),
			snd_soc_component_read32(priv->component, 0x318),
			snd_soc_component_read32(priv->component, 0x1C),
			snd_soc_component_read32(priv->component, 0x1D));

	SND_LOG_INFO(HLOG, "register card finished\n");

	return 0;

err_devm_kfree:
	devm_kfree(&pdev->dev, priv);
	return ret;
}

static int __exit sunxi_card_dev_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct sunxi_card_priv *priv = snd_soc_card_get_drvdata(card);

	snd_soc_unregister_card(card);
	devm_kfree(&pdev->dev, priv);

	SND_LOG_INFO(HLOG, "unregister card finished\n");

	return 0;
}

static const struct of_device_id sunxi_card_of_match[] = {
	{ .compatible = "allwinner,sunxi-codec-machine", },
	{},
};

static struct platform_driver sunxi_machine_driver = {
	.driver = {
		.name = "sunxi-codec-machine",
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = sunxi_card_of_match,
	},
	.probe = sunxi_card_dev_probe,
	.remove = __exit_p(sunxi_card_dev_remove),
};

static int __init sunxi_machine_driver_init(void)
{
	return platform_driver_register(&sunxi_machine_driver);
}
late_initcall(sunxi_machine_driver_init);

static void __exit sunxi_machine_driver_exit(void)
{
	platform_driver_unregister(&sunxi_machine_driver);
}
module_exit(sunxi_machine_driver_exit);

MODULE_AUTHOR("luguofang <luguofang@allwinnertech.com>");
MODULE_DESCRIPTION("SUNXI Codec Machine ASoC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:sunxi-codec-machine");
