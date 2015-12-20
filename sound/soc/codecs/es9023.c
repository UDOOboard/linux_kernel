/*
 * Simple codec for the ES9023 DAC.
 *
 * Copyright 2015 Jasbir Matharu
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <sound/soc.h>

static struct snd_soc_dai_driver es9023_dai = {
	.name = "es9023-hifi",
	.playback = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE |
			SNDRV_PCM_FMTBIT_S24_LE
	},
};

static struct snd_soc_codec_driver soc_codec_dev_es9023;

static int es9023_probe(struct platform_device *pdev)
{
	return snd_soc_register_codec(&pdev->dev, &soc_codec_dev_es9023,
			&es9023_dai, 1);
}

static int es9023_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static const struct of_device_id es9023_of_match[] = {
	{ .compatible = "udoo,es9023", },
	{ }
};
MODULE_DEVICE_TABLE(of, es9023_of_match);

static struct platform_driver es9023_codec_driver = {
	.probe 		= es9023_probe,
	.remove 	= es9023_remove,
	.driver		= {
		.name	= "es9023-codec",
		.owner	= THIS_MODULE,
		.of_match_table = es9023_of_match,
	},
};

module_platform_driver(es9023_codec_driver);

MODULE_DESCRIPTION("ASoC ES9023 codec driver");
MODULE_AUTHOR("Jasbir Matharu");
MODULE_LICENSE("GPL v2");
