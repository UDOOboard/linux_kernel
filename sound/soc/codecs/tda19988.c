/*
 * tda19988.c  -- Simple codec for TDA19988 on the UDOO Neo 
 *
 * Copyright 2015 Jasbir Matharu
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>

static struct snd_soc_dai_driver tda19988_dai = {
	.name		= "tda19988-codec",
	.playback	= {
		.stream_name	= "Playback",
		.channels_min	= 2,
		.channels_max	= 2,

		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | 
			SNDRV_PCM_FMTBIT_S24_LE,
	},
};

static struct snd_soc_codec_driver soc_codec_dev_tda998x;

static int tda19988_platform_probe(struct platform_device *pdev)
{
	return snd_soc_register_codec(&pdev->dev, &soc_codec_dev_tda998x,
					  &tda19988_dai, 1);
}

static int tda19988_platform_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

MODULE_ALIAS("platform:tda19988-codec");

static const struct of_device_id tda19988_of_match[] = {
	{ .compatible = "udoo,tda19988-codec", },
	{ }
};
MODULE_DEVICE_TABLE(of, tda19988_of_match);

static struct platform_driver tda19988_platform_driver = {
	.driver		= {
		.name	= "tda19988-codec",
		.owner	= THIS_MODULE,
				.of_match_table = tda19988_of_match,
	},
	.probe		= tda19988_platform_probe,
	.remove		= tda19988_platform_remove,
};
module_platform_driver(tda19988_platform_driver);

MODULE_DESCRIPTION("TDA19988 ALSA SOC Codec driver");
MODULE_AUTHOR("Jasbir Matharu");
MODULE_LICENSE("GPL");
