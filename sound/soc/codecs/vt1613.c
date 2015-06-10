/*
* vt1613.c  --  ALSA SoC VT1613 AC'97 codec support
*
* Copyright 2010-2015 Seco s.r.l.
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
* 02110-1301 USA
*
*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/of_device.h>
#include <sound/soc.h> 
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <sound/ac97_codec.h>

#include "vt1613.h"
#define DRV_NAME "vt1613-codec"

/* TODO: S/PDIF implementation. As this driver was developed for UDOO board's needs,
* we skipped S/PDIF features developing*/ 

static const struct reg_default vt1613_reg_defaults[] = {
  { 0x00, 0x0140 },  /* Reset */ 		
  { 0x02, 0x8000 },  /* Stereo Output Volume */
  { 0x04, 0x8000 },  /* HP Stereo Output Volume */
  { 0x0A, 0x0000 },  /* PC Beep Volume */
  { 0x0C, 0x8008 },  /* Phone Volume */
  { 0x0E, 0x8008 },  /* MIC Volume */
  { 0x10, 0x8808 },  /* Line In Volume */
  { 0x12, 0x8808 },  /* CD Volume */
  { 0x16, 0x8808 },  /* AUX Volume */
  { 0x18, 0x8808 },  /* PCM out Volume */
  { 0x1A, 0x0000 },  /* Record Select */
  { 0x1C, 0x8000 },  /* Record Gain */
  { 0x20, 0x0000 },  /* General Purpose */
  { 0x24, 0x0001 },  /* Audio Interrupt & Paging (AC'97 2.3) */
  { 0x26, 0x0000 },  /* Powerdown control / status */	
  { 0x28, 0x097C },  /* Extended Audio ID */
  { 0x2A, 0x3830 },  /* Extended Audio Status and Control */
  { 0x2C, 0xBB80 },  /* PCM Front DAC Rate */          
  { 0x32, 0xBB80 },  /* PCM LR ADC Rate */         
  { 0x3A, 0x2000 },  /* S/PDIF Control */
  { 0x5A, 0x0000 },  /* Vendor Reserved Register */ 
  { 0x5C, 0x00A9 },  /* Vendor Reserved Register */
  { 0x62, 0xFFFF },  /*  PCI SVID  Page ID = 01h */                            
  { 0x64, 0xFFFF },  /*  PCI SID   Page ID = 01h */                            
  { 0x66, 0x0000 },  /* S/PDIF RX Status    Page ID = 00h */
  { 0x68, 0x0000 },  /* S/PDIF RX Status    Page ID = 00h */
  { 0x6A, 0x0000 },  /* S/PDIF RX Control   Page ID = 00h */
  { 0x6C, 0x376A },  /* DAC Slot Mapping  Page ID = 01h */                     
  { 0x6E, 0x0000 },  /* ADC Slot Mapping  Page ID = 01h */                     	  
  { 0x70, 0x0000 },  /* ADC / SPDIF RX Left Peak */
  { 0x74, 0x0000 },  /* PLL Setting /Debugging */
  { 0x76, 0x1182 },  /* Miscellaneous */
  { 0x78, 0x0070 },  /* GPIO Control */
  { 0x7A, 0x0070 },  /* GPIO Status */
  { 0x7C, 0x5649 },  /* Vendor ID1 */
  { 0x7E, 0x4120 },  /* Vendor ID2 */	
};

static bool vt1613_readable_reg(struct device *dev, unsigned int reg)
{
  switch (reg) {
  case AC97_RESET ... AC97_HEADPHONE:
  case AC97_PC_BEEP ... AC97_CD:
  case AC97_AUX ... AC97_GENERAL_PURPOSE:
  case AC97_INT_PAGING ... AC97_PCM_LR_ADC_RATE:
  case AC97_SPDIF:
	case AC97_AD_TEST:
	case AC97_VT1613_STEREO_MIC:
  case AC97_PCI_SVID ... AC97_SENSE_INFO:
	case AC97_VT1613_DAC_SLOT_MAP:
	case AC97_VT1613_ADC_SLOT_MAP:
  case AC97_AD_CODEC_CFG:
  case AC97_AD_SERIAL_CFG:
	case AC97_AD_MISC:
	case AC97_VT1613_GPIO_CTRL:
  case AC97_VT1613_GPIO_STATUS:
  case AC97_VENDOR_ID1:
  case AC97_VENDOR_ID2:
    return true;
  default:
    return false;
  }
}
  
static bool vt1613_writeable_reg(struct device *dev, unsigned int reg)
{
  switch (reg) {
  case AC97_RESET:
  case AC97_EXTENDED_ID:
  case AC97_PCM_SURR_DAC_RATE:
  case AC97_PCM_LFE_DAC_RATE:
  case AC97_FUNC_SELECT:
  case AC97_FUNC_INFO:
  case AC97_VT1613_ADC_SLOT_MAP:
  case AC97_VENDOR_ID1:
  case AC97_VENDOR_ID2:
    return false;
  default:
    return vt1613_readable_reg(dev, reg);
  }
}
  
static const struct regmap_config vt1613_regmap_config = {
	.name = "vt1613_regmap",
  .reg_bits = 16,
  .reg_stride = 2,
  .val_bits = 16,
  .max_register = 0x7E,
  .cache_type = REGCACHE_RBTREE,
 
  .volatile_reg = regmap_ac97_default_volatile,
  .readable_reg = vt1613_readable_reg,
  .writeable_reg = vt1613_writeable_reg,
 
  .reg_defaults = vt1613_reg_defaults,
  .num_reg_defaults = ARRAY_SIZE(vt1613_reg_defaults),
};

static const char *vt1613_record_mux[] = {"Mic", "CD", "--", "AUX",
			"Line", "Stereo Mix", "Mono Mix", "Phone"};
static SOC_ENUM_DOUBLE_DECL(vt1613_record_enum,
			AC97_REC_SEL, 8, 0, vt1613_record_mux);

static const char *vt1613_mic_mux[] = {"Mic1", "Mic2"};
static SOC_ENUM_SINGLE_DECL(vt1613_mic_enum, 
			AC97_GENERAL_PURPOSE, 8, vt1613_mic_mux); 

static const char *vt1613_boost[] = {"0dB", "20dB"};
static SOC_ENUM_SINGLE_DECL(vt1613_boost_enum, 
			AC97_MIC, 6, vt1613_boost); 

static const char *vt1613_mic_sel[] = {"MonoMic", "StereoMic"};
static SOC_ENUM_SINGLE_DECL(vt1613_mic_sel_enum, 
			AC97_VT1613_STEREO_MIC, 2, vt1613_mic_sel); 

static const DECLARE_TLV_DB_LINEAR(master_tlv, -4650, 0);
static const DECLARE_TLV_DB_LINEAR(record_tlv, 0, 2250); 
static const DECLARE_TLV_DB_LINEAR(beep_tlv, -4500, 0); 
static const DECLARE_TLV_DB_LINEAR(mix_tlv, -3450, 1200);

static const struct snd_kcontrol_new vt1613_snd_ac97_controls[] = {
	SOC_DOUBLE_TLV("Speaker Playback Volume", AC97_MASTER, 8, 0, 31, 1, master_tlv),
	SOC_SINGLE("Speaker Playback Switch", AC97_MASTER, 15, 1, 1), 

	SOC_DOUBLE_TLV("Headphone Playback Volume", AC97_HEADPHONE, 8, 0, 31, 1, master_tlv),
	SOC_SINGLE("Headphone Playback Switch", AC97_HEADPHONE, 15, 1, 1),
	
	SOC_DOUBLE_TLV("PCM Playback Volume", AC97_PCM, 8, 0, 31, 0, mix_tlv),
	SOC_SINGLE("PCM Playback Switch", AC97_PCM, 15, 1, 1), 

	SOC_DOUBLE_TLV("Record Capture Volume", AC97_REC_GAIN, 8, 0, 15, 0, record_tlv),
	SOC_SINGLE("Record Capture Switch", AC97_REC_GAIN, 15, 1, 1),

	SOC_SINGLE_TLV("Beep Volume", AC97_PC_BEEP, 1, 15, 1, beep_tlv),
	SOC_SINGLE("Beep Switch", AC97_PC_BEEP, 15, 1, 1), 
	SOC_SINGLE_TLV("Phone Volume", AC97_PHONE, 0, 31, 0, mix_tlv),
	SOC_SINGLE("Phone Switch", AC97_PHONE, 15, 1, 1),

	/* Mono Mic and Stereo Mic's right channel controls */
	SOC_SINGLE_TLV("Mic/StereoMic_R Volume", AC97_MIC, 0, 31, 0, mix_tlv),
	SOC_SINGLE("Mic/StereoMic_R Switch", AC97_MIC, 15, 1, 1), 

	/* Stereo Mic's left channel controls */
	SOC_SINGLE("StereoMic_L Switch", AC97_MIC, 7, 1, 1), 
	SOC_SINGLE_TLV("StereoMic_L Volume", AC97_MIC, 8, 31, 0, mix_tlv), 

	SOC_DOUBLE_TLV("Line Volume", AC97_LINE, 8, 0, 31, 0, mix_tlv),
	SOC_SINGLE("Line Switch", AC97_LINE, 15, 1, 1), 
	SOC_DOUBLE_TLV("CD Volume", AC97_CD, 8, 0, 31, 0, mix_tlv),
	SOC_SINGLE("CD Switch", AC97_CD, 15, 1, 1), 
	SOC_DOUBLE_TLV("AUX Volume", AC97_AUX, 8, 0, 31, 0, mix_tlv), 
	SOC_SINGLE("AUX Switch", AC97_AUX, 15, 1, 1), 	
	
	SOC_SINGLE("Analog Loopback", AC97_GENERAL_PURPOSE, 7, 1, 0),

	SOC_ENUM("Mic Boost", vt1613_boost_enum),
	SOC_ENUM("Mic1/2 Mux", vt1613_mic_enum),
	SOC_ENUM("Mic Select", vt1613_mic_sel_enum),
	SOC_ENUM("Record Mux", vt1613_record_enum),
};

static const unsigned int vt1613_rates[] = {
	8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000
};

static const struct snd_pcm_hw_constraint_list vt1613_rate_constraints = {
	.count	= ARRAY_SIZE(vt1613_rates),
	.list	= vt1613_rates,
};

static int vt1613_startup(struct snd_pcm_substream *substream,
			struct snd_soc_dai *dai)
{	
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK){
		snd_pcm_hw_constraint_list(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE, &vt1613_rate_constraints);
	} else {
		snd_pcm_hw_constraint_list(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE, &vt1613_rate_constraints);
	}
	
	return 0;
}

static int ac97_analog_prepare(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned short reg; 

	/* enable variable rate audio (VRA) and disable S/PDIF output */
	snd_soc_write(codec, AC97_EXTENDED_STATUS, 
		(snd_soc_read(codec, AC97_EXTENDED_STATUS) | 0x1) & ~0x4);	

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK){
		reg = AC97_PCM_FRONT_DAC_RATE;		
	} else {
		reg = AC97_PCM_LR_ADC_RATE;
	}

	return snd_soc_write(codec, reg, runtime->rate);
}

static int ac97_digital_prepare(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct snd_pcm_runtime *runtime = substream->runtime;

	snd_soc_write(codec, AC97_SPDIF, 0x2002);

	 /* enable VRA and S/PDIF output */
	snd_soc_write(codec, AC97_EXTENDED_STATUS, snd_soc_read(codec, AC97_EXTENDED_STATUS) | 0x5);

	return snd_soc_write(codec, AC97_PCM_FRONT_DAC_RATE, runtime->rate);
}

static int vt1613_set_bias_level(struct snd_soc_codec *codec,
				   enum snd_soc_bias_level level)
{
	switch (level) {
	case SND_SOC_BIAS_ON: /* full On */
	case SND_SOC_BIAS_PREPARE: /* partial On */
	case SND_SOC_BIAS_STANDBY: /* Off, with power */
		snd_soc_write(codec, AC97_POWERDOWN, 0x0000);
		break;
	case SND_SOC_BIAS_OFF: /* Off, without power */
		/* disable everything including AC link */
		snd_soc_write(codec, AC97_POWERDOWN, 0xffff);
		break;
	}
	codec->dapm.bias_level = level;
	return 0;
}

static int vt1613_reset(struct snd_soc_codec *codec, int try_warm)
{
	
	if (try_warm && soc_ac97_ops->warm_reset) {
		soc_ac97_ops->warm_reset(codec->ac97);
		if (snd_soc_read(codec, AC97_RESET) == 0x0140)
			return 1;
	}
	soc_ac97_ops->reset(codec->ac97);
	if (soc_ac97_ops->warm_reset)
		soc_ac97_ops->warm_reset(codec->ac97);
	if (snd_soc_read(codec, AC97_RESET) == 0x0140) 
		return 0;

	return -EIO;
}

static struct snd_soc_dai_ops vt1613_dai_ops_analog = {
	.startup = vt1613_startup,
	.prepare = ac97_analog_prepare,
};

static struct snd_soc_dai_ops vt1613_dai_ops_digital = {
	.prepare = ac97_digital_prepare,
};

struct snd_soc_dai_driver vt1613_dai[] = {
{
	.name = "vt1613-hifi-analog",
	.ac97_control = 1,

	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_KNOT,
		.formats = SND_SOC_STD_AC97_FMTS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_KNOT,
		.formats = SND_SOC_STD_AC97_FMTS,
	},

	.ops = &vt1613_dai_ops_analog,
},
{
	.name = "vt1613-hifi-IEC958",
	.ac97_control = 1,

	.playback = {
		.stream_name = "vt1613 IEC958",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_32000 | 
			SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FORMAT_IEC958_SUBFRAME_BE,
	},

	.ops = &vt1613_dai_ops_digital,
}
};

static int vt1613_codec_suspend(struct snd_soc_codec *codec)
{
	vt1613_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

static int vt1613_codec_resume(struct snd_soc_codec *codec)
{
	u16 id, reset;

	reset = 0;
	/* give the codec an AC97 warm reset to start the link */
reset:
	if (reset > 5) {
		printk(KERN_ERR "vt1613 failed to resume");
		return -EIO;
	}
	codec->ac97->bus->ops->warm_reset(codec->ac97);
	id = soc_ac97_ops->read(codec->ac97, AC97_VENDOR_ID2);
	if (id != 0x4123) {
		vt1613_reset(codec, 0);
		reset++;
		goto reset;
	}
	vt1613_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	return 0;
}

static int vt1613_codec_probe(struct snd_soc_codec *codec)
{
	int ret = 0;
	struct regmap *regmap;

	ret = snd_soc_new_ac97_codec(codec, soc_ac97_ops, 0);
	if (ret){
		dev_err(codec->dev, "Failed to register AC97 codec: %d\n", ret);
		return ret;
	}	

	soc_ac97_dev_register(codec);
	if (ret){
		dev_err(codec->dev, "Failed to register AC97 codec to bus: %d\n", ret);
		goto free_ac97;
	}
	codec->ac97_registered = 1; 

	regmap = regmap_init_ac97(codec->ac97, &vt1613_regmap_config);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		dev_err(codec->dev, "Failed to init register map: %d\n", ret);
		goto free_ac97;
	}
	
	codec->control_data = regmap;
	ret = snd_soc_codec_set_cache_io(codec, 16, 16, SND_SOC_REGMAP);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		goto free_regmap;
	}
	snd_soc_codec_set_drvdata(codec, codec->ac97);

	/* do a cold reset for the controller and then try
	 * a warm reset followed by an optional cold reset for codec */
	
	vt1613_reset(codec, 0);
	ret = vt1613_reset(codec, 1);
	if (ret < 0) {
		printk(KERN_ERR "Failed to reset VT1613: AC97 link error\n");
		goto free_regmap;
	}

	/* Read out vendor IDs */
	printk(KERN_INFO "VT1613 SoC Audio Codec [ID = %04x - %04x]\n", 
		snd_soc_read(codec, AC97_VENDOR_ID1), snd_soc_read(codec, AC97_VENDOR_ID2));

	vt1613_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

 	/* unmute captures and playbacks volume */
	snd_soc_write(codec, AC97_MASTER, 0x0000);
 	snd_soc_write(codec, AC97_PCM, 0x0000);
 	snd_soc_write(codec, AC97_REC_GAIN, 0x0000);

	/* At 3.3V analog supply, for the bits 3:2 should be set 10b for the lowest power instead of default 00b */
	snd_soc_write(codec, AC97_AD_TEST, snd_soc_read(codec, AC97_AD_TEST) | 0x0008);

	/* To maximize recording quality by removing white noise */
	snd_soc_write(codec, AC97_AD_TEST, snd_soc_read(codec, AC97_AD_TEST) | 0x0400);

	snd_soc_add_codec_controls(codec, vt1613_snd_ac97_controls,
			     ARRAY_SIZE(vt1613_snd_ac97_controls));
	
	return 0;

free_regmap:
  regmap_exit(regmap);
free_ac97:
  snd_soc_free_ac97_codec(codec);

 	return ret;
}

static int vt1613_codec_remove(struct snd_soc_codec *codec)
{
	regmap_exit(codec->control_data);
	snd_soc_free_ac97_codec(codec);
	return 0;
}

struct snd_soc_codec_driver vt1613_codec = {
	.probe = vt1613_codec_probe,
	.remove = vt1613_codec_remove,
	.suspend = vt1613_codec_suspend,
	.resume = vt1613_codec_resume,	
	.set_bias_level = vt1613_set_bias_level,
};

static int vt1613_probe(struct platform_device *pdev)
{	
	return snd_soc_register_codec(&pdev->dev,
		&vt1613_codec, vt1613_dai, ARRAY_SIZE(vt1613_dai));
}

static int vt1613_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static const struct of_device_id vt1613_of_match[] = {
	{ .compatible = "via,vt1613", }, 
	{ }
};
MODULE_DEVICE_TABLE(of, vt1613_of_match);

static struct platform_driver vt1613_codec_driver = {
	.driver = {
			.name = DRV_NAME,
			.owner = THIS_MODULE,
			.of_match_table = vt1613_of_match,
	},

	.probe = vt1613_probe,
	.remove = vt1613_remove,
};

module_platform_driver(vt1613_codec_driver);

MODULE_DESCRIPTION("ASoC VT1613 codec driver");
MODULE_AUTHOR("Seco s.r.l.");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
