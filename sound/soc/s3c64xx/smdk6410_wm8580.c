/*
 * smdk6400_wm8580.c
 *
 * Copyright (C) 2009, Samsung Elect. Ltd. - Jaswinder Singh <jassisinghbrar@gmail.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/io.h>
#include <asm/gpio.h> 
#include <plat/gpio-cfg.h> 
#include <plat/map-base.h>

//#define USE_GPR

#include <plat/regs-gpio.h>
#ifdef USE_GPR
#include <plat/gpio-bank-r.h>
#else
#include <plat/gpio-bank-h.h>
#include <plat/gpio-bank-c.h>
#endif

#include "../codecs/wm8580.h"
#include "s3c-pcm.h"

#include "s3c6410-i2s.h"

#define SRC_CLK	s3c6410_i2s_get_clockrate()

/* XXX BLC(bits-per-channel) --> BFS(bit clock shud be >= FS*(Bit-per-channel)*2) XXX */
/* XXX BFS --> RFS(must be a multiple of BFS)                                 XXX */
/* XXX RFS & SRC_CLK --> Prescalar Value(SRC_CLK / RFS_VAL / fs - 1)          XXX */
static int smdk6410_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	int bfs, rfs, psr, ret;

	/* Choose BFS and RFS values combination that is supported by
	 * both the WM8580 codec as well as the S3C6410 AP
	 *
	 * WM8580 codec supports only S16_LE, S20_3LE, S24_LE & S32_LE.
	 * S3C6410 AP supports only S8, S16_LE & S24_LE.
	 * We implement all for completeness but only S16_LE & S24_LE bit-lengths 
	 * are possible for this AP-Codec combination.
	 */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		bfs = 16;
		rfs = 256;		/* Can take any RFS value for AP */
 		break;
 	case SNDRV_PCM_FORMAT_S16_LE:
		bfs = 32;
		rfs = 256;		/* Can take any RFS value for AP */
 		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
 	case SNDRV_PCM_FORMAT_S24_LE:
		bfs = 48;
		rfs = 512;		/* B'coz 48-BFS needs atleast 512-RFS acc to *S5P6440* UserManual */
					/* And S5P6440 uses the same I2S IP as S3C6410 */
 		break;
 	case SNDRV_PCM_FORMAT_S32_LE:	/* Impossible, as the AP doesn't support 64fs or more BFS */
	default:
		return -EINVAL;
 	}
 
	/* Select the AP Sysclk */
	ret = snd_soc_dai_set_sysclk(cpu_dai, S3C6410_CDCLKSRC_INT, params_rate(params), SND_SOC_CLOCK_OUT);
	if (ret < 0)
		return ret;

#ifdef USE_CLKAUDIO
	ret = snd_soc_dai_set_sysclk(cpu_dai, S3C6410_CLKSRC_CLKAUDIO, params_rate(params), SND_SOC_CLOCK_OUT);
#else
	ret = snd_soc_dai_set_sysclk(cpu_dai, S3C6410_CLKSRC_PCLK, 0, SND_SOC_CLOCK_OUT);
#endif
	if (ret < 0)
		return ret;

	/* Set the AP DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	/* Set the AP RFS */
	ret = snd_soc_dai_set_clkdiv(cpu_dai, S3C64XX_DIV_MCLK, rfs);
	if (ret < 0)
		return ret;

	/* Set the AP BFS */
	ret = snd_soc_dai_set_clkdiv(cpu_dai, S3C64XX_DIV_BCLK, bfs);
	if (ret < 0)
		return ret;

	switch (params_rate(params)) {
	case 8000:
	case 11025:
	case 16000:
	case 22050:
	case 32000:
	case 44100: 
	case 48000:
	case 64000:
	case 88200:
	case 96000:
		psr = SRC_CLK / rfs / params_rate(params);
		ret = SRC_CLK / rfs - psr * params_rate(params);
		if(ret >= params_rate(params)/2)	// round off
		   psr += 1;
		psr -= 1;
		break;
	default:
		return -EINVAL;
	}

	//printk("SRC_CLK=%d PSR=%d RFS=%d BFS=%d\n", SRC_CLK, psr, rfs, bfs);

	/* Set the AP Prescalar */
	ret = snd_soc_dai_set_clkdiv(cpu_dai, S3C64XX_DIV_PRESCALER, psr);
	if (ret < 0)
		return ret;

	/* Set the Codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	/* Set the Codec BCLK(no option to set the MCLK) */
	/* See page 2 and 53 of Wm8580 Manual */
	ret = snd_soc_dai_set_clkdiv(codec_dai, WM8580_MCLK, WM8580_CLKSRC_MCLK); /* Use MCLK provided by CPU i/f */
	if (ret < 0)
		return ret;
	ret = snd_soc_dai_set_clkdiv(codec_dai, WM8580_DAC_CLKSEL, WM8580_CLKSRC_MCLK); /* Fig-26 Pg-43 */
	if (ret < 0)
		return ret;
	ret = snd_soc_dai_set_clkdiv(codec_dai, WM8580_CLKOUTSRC, WM8580_CLKSRC_NONE); /* Pg-58 */
	if (ret < 0)
		return ret;

	return 0;
}

/*
 * WM8580 DAI operations.
 */
static struct snd_soc_ops smdk6410_ops = {
	.hw_params = smdk6410_hw_params,
};

static const struct snd_soc_dapm_widget wm8580_dapm_widgets[] = {
	SND_SOC_DAPM_LINE("I2S Front Jack", NULL),
	SND_SOC_DAPM_LINE("I2S Center Jack", NULL),
	SND_SOC_DAPM_LINE("I2S Rear Jack", NULL),
	SND_SOC_DAPM_LINE("Line In Jack", NULL),
};

/* example machine audio_mapnections */
static const struct snd_soc_dapm_route audio_map[] = {

	{ "I2S Front Jack", NULL, "VOUT1L" },
	{ "I2S Front Jack", NULL, "VOUT1R" },

	{ "I2S Center Jack", NULL, "VOUT2L" },
	{ "I2S Center Jack", NULL, "VOUT2R" },

	{ "I2S Rear Jack", NULL, "VOUT3L" },
	{ "I2S Rear Jack", NULL, "VOUT3R" },

	{ "AINL", NULL, "Line In Jack" },
	{ "AINR", NULL, "Line In Jack" },
		
};

static int smdk6410_wm8580_init(struct snd_soc_codec *codec)
{
	int i;

	/* Add smdk6410 specific widgets */
	snd_soc_dapm_new_controls(codec, wm8580_dapm_widgets,ARRAY_SIZE(wm8580_dapm_widgets));

	/* set up smdk6410 specific audio paths */
	snd_soc_dapm_add_routes(codec, audio_map,ARRAY_SIZE(audio_map));

	/* No jack detect - mark all jacks as enabled */
	for (i = 0; i < ARRAY_SIZE(wm8580_dapm_widgets); i++)
		snd_soc_dapm_set_endpoint(codec,
					  wm8580_dapm_widgets[i].name, 1);

	snd_soc_dapm_sync_endpoints(codec);

	return 0;
}

static struct snd_soc_dai_link smdk6410_dai[] = {
{
	.name = "WM8580",
	.stream_name = "WM8580 HiFi Playback",
	.cpu_dai = &s3c6410_i2s_v40_dai,
	.codec_dai = &wm8580_dai[WM8580_DAI_PAIFRX],
	.init = smdk6410_wm8580_init,
	.ops = &smdk6410_ops,
},
};

static struct snd_soc_machine smdk6410 = {
	.name = "smdk6410",
	.dai_link = smdk6410_dai,
	.num_links = ARRAY_SIZE(smdk6410_dai),
};

static struct wm8580_setup_data smdk6410_wm8580_setup = {
	.i2c_address = 0x1b,
};

static struct snd_soc_device smdk6410_snd_devdata = {
	.machine = &smdk6410,
	.platform = &s3c24xx_soc_platform,
	.codec_dev = &soc_codec_dev_wm8580,
	.codec_data = &smdk6410_wm8580_setup,
};

static struct platform_device *smdk6410_snd_device;

static int __init smdk6410_audio_init(void)
{
	int ret;
	u32 val;

#ifdef USE_GPR
	val = __raw_readl(S3C64XX_GPRPUD);
	val &= ~((3<<8) | (3<<10) | (3<<14) | (3<<16) | (3<<18) | (3<<28) | (3<<30));
	val |= ((0<<8) | (0<<10) | (0<<14) | (0<<16) | (0<<18) | (0<<28) | (1<<30));
	__raw_writel(val, S3C64XX_GPRPUD);

	val = __raw_readl(S3C64XX_GPRCON0);
	val &= ~((0xf<<16) | (0xf<<20) | (0xf<<28));
	val |= (5<<16) | (5<<20) | (5<<28);
	__raw_writel(val, S3C64XX_GPRCON0);

	val = __raw_readl(S3C64XX_GPRCON1);
	val &= ~((0xf<<0) | (0xf<<4) | (0xf<<24) | (0xf<<28));
	val |= (5<<0) | (5<<4) | (5<<24) | (5<<28);
	__raw_writel(val, S3C64XX_GPRCON1);

#else
	val = __raw_readl(S3C64XX_GPCPUD);
	val &= ~((3<<8) | (3<<10) | (3<<14));
	val |= ((0<<8) | (0<<10) | (0<<14));
	__raw_writel(val, S3C64XX_GPCPUD);

	val = __raw_readl(S3C64XX_GPCCON);
	val &= ~((0xf<<16) | (0xf<<20) | (0xf<<28));
	val |= (5<<16) | (5<<20) | (5<<28);
	__raw_writel(val, S3C64XX_GPCCON);

	val = __raw_readl(S3C64XX_GPHPUD);
	val &= ~((3<<12) | (3<<14) | (3<<16) | (3<<18));
	val |= ((0<<12) | (1<<14) | (0<<16) | (0<18));
	__raw_writel(val, S3C64XX_GPHPUD);

	val = __raw_readl(S3C64XX_GPHCON0);
	val &= ~((0xf<<24) | (0xf<<28));
	val |= (5<<24) | (5<<28);
	__raw_writel(val, S3C64XX_GPHCON0);

	val = __raw_readl(S3C64XX_GPHCON1);
	val &= ~((0xf<<0) | (0xf<<4));
	val |= (5<<0) | (5<<4);
	__raw_writel(val, S3C64XX_GPHCON1);
#endif

	smdk6410_snd_device = platform_device_alloc("soc-audio", 0);
	if (!smdk6410_snd_device)
		return -ENOMEM;

	platform_set_drvdata(smdk6410_snd_device, &smdk6410_snd_devdata);
	smdk6410_snd_devdata.dev = &smdk6410_snd_device->dev;
	ret = platform_device_add(smdk6410_snd_device);

	if (ret)
		platform_device_put(smdk6410_snd_device);
	
	return ret;
}

static void __exit smdk6410_audio_exit(void)
{
	platform_device_unregister(smdk6410_snd_device);
}

module_init(smdk6410_audio_init);
module_exit(smdk6410_audio_exit);

/* Module information */
MODULE_DESCRIPTION("ALSA SoC SMDK6410 WM8580");
MODULE_LICENSE("GPL");
