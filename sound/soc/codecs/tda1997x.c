/*
 * tda1997x.c  --  tda1997x ALSA SoC Audio driver
 *
 * Copyright (C) 2013 Gateworks Corporation
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
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/clk.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/mfd/tda1997x-core.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of_device.h>
#include <linux/pm.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>

/* refine sample-rate based on HDMI source */
static int tda1997x_pcm_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct snd_pcm_runtime *rtd = substream->runtime;
        tda1997x_audout_fmt_t fmt;
	int rate, ret;

        ret = tda1997x_get_audout_fmt(&fmt);
	if (ret) {
		dev_err(codec->dev, "failed to obtain tda1997x info: %d\n",
			ret);
		return -EIO;
	}
	rate = fmt.samplerate;

	ret = snd_pcm_hw_constraint_minmax(rtd, SNDRV_PCM_HW_PARAM_RATE,
					   rate, rate);
	if (ret > 0)
		dev_info(codec->dev, "set samplerate constraint to %dHz\n",
			 rate);
	else
		dev_err(codec->dev, "failed to set samplerate constraint "
			"to %dHz: %d\n", rate, ret);

	return 0;
}

static const struct snd_soc_dai_ops tda1997x_ops = {
	.startup = tda1997x_pcm_startup,
};

static struct snd_soc_dai_driver tda1997x_codec_dai = {
	.name = "tda1997x",
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		/* rate and foramat are dependent on the HDMI source */
		.rates = SNDRV_PCM_RATE_32000 |
			SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |
			SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000 |
			SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_192000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &tda1997x_ops,
};

static int tda1997x_probe(struct snd_soc_codec *codec)
{
	pr_debug("%s\n", __func__);

	return 0;
}

static int tda1997x_remove(struct snd_soc_codec *codec)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static struct snd_soc_codec_driver tda1997x_codec_driver = {
	.probe = tda1997x_probe,
	.remove = tda1997x_remove,
	.reg_word_size = sizeof(u16),
};

static int tda1997x_codec_probe(struct platform_device *pdev)
{
	int ret = 0;

	dev_info(&pdev->dev, "TDA1997x HDMI Audio In\n");

	ret = snd_soc_register_codec(&pdev->dev,
			&tda1997x_codec_driver, &tda1997x_codec_dai, 1);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register codec: %d\n", ret);
		return ret;
	}
	pr_debug("%s registered codec and codec_dai\n", __func__);

	return 0;
}

static int tda1997x_codec_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);

	return 0;
}

static const struct of_device_id tda1997x_dt_ids[] = {
	{ .compatible = "nxp,tda1997x-codec", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, tda1997x_dt_ids);

struct platform_driver tda1997x_driver = {
	.driver = {
		.name = "tda1997x_codec",
		.owner = THIS_MODULE,
		.of_match_table = tda1997x_dt_ids,
	},
	.probe = tda1997x_codec_probe,
	.remove = tda1997x_codec_remove,
};
module_platform_driver(tda1997x_driver);

MODULE_DESCRIPTION("NXP TDA1997x ALSA SoC Codec Driver");
MODULE_AUTHOR("Tim Harvey <tharvey@gateworks.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:tda1997x-audio");
