/*
 * s5m8751.c  --  S5M8751 Power-Audio IC ALSA Soc Audio driver
 *
 * Copyright 2009 Samsung Electronics.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include <sound/initval.h>
#include <asm/div64.h>

#include "s5m8751.h"

#define S5M8751_VERSION "0.2"

/* S5M8751 is activated only in XXX SLAVE MODE XXX
 * 8kHz ~ 96kHz sample rate. Auto/Manual ???
 * I2S, Lj, Rj, PCM short frame sync or PCM long frame sync formats.
 * Slave addr:- 0xD1-Read, 0xD0-Write
 * XXX This driver supports only SPEAKER-OUT mode, others are easy to implement XXX
 * */

static const u16 s5m8751_reg[S5M8751_NUMREGS];

/*
 * read s5m8751 register cache
 */
static inline unsigned int s5m8751_read_reg_cache(struct snd_soc_codec *codec,
	unsigned int reg)
{
	u16 *cache = codec->reg_cache;
	BUG_ON(reg < S5M8751_DA_PDB1);
	BUG_ON(reg > S5M8751_LINE_CTRL);
	return cache[reg];
}

/* Fills the data in cache and returns only operation status.
 * 1 for success, -EIO for failure.
 */
static int s5m8751_read(struct snd_soc_codec *codec,
				       unsigned int reg)
{
	u8 data;
	u16 *cache = codec->reg_cache;

	data = reg;

	if (codec->hw_write(codec->control_data, &data, 1) != 1){
		return -EIO;
	}

	if (codec->hw_read(codec->control_data, &data, 1) != 1){
		return -EIO;
	}

	cache[reg] = data;	/* Update the cache */

	return 1;
}

/*
 * write s5m8751 register cache
 */
static inline void s5m8751_write_reg_cache(struct snd_soc_codec *codec,
	unsigned int reg, unsigned int value)
{
	u16 *cache = codec->reg_cache;

	if(reg == S5M8751_IN1_CTRL1)
		value &= ~(1<<6);	/* Reset is cleared automatically by the codec */

	cache[reg] = value;
}

/*
 * write to the S5M8751 register space
 */
/* Fills the data in cache and returns only operation status.
 * 1 for success, 0 for failure.
 */
static int s5m8751_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{
	u8 data[2];

	BUG_ON(reg < S5M8751_DA_PDB1);
	BUG_ON(reg > S5M8751_LINE_CTRL);

	data[0] = reg & 0xff;
	data[1] = value & 0xff;

	if (codec->hw_write(codec->control_data, data, 2) == 2){
		s5m8751_write_reg_cache(codec, reg, value);
		return 0;
	}else{
		return -EIO;
	}
}

static const struct snd_kcontrol_new s5m8751_snd_controls[] = {
	SOC_DOUBLE_R("HeadPhone Volume", S5M8751_HP_VOL1, S5M8751_HP_VOL1, 0, 0x7f, 1),
	SOC_DOUBLE_R("Line-In Gain", S5M8751_DA_LGAIN, S5M8751_DA_RGAIN, 0, 0x7f, 1),
	SOC_DOUBLE_R("DAC Volume", S5M8751_DA_VOLL, S5M8751_DA_VOLR, 0, 0xff, 1),
	SOC_SINGLE("Speaker Slope", S5M8751_SPK_SLOPE, 0, 0xf, 1),
	SOC_SINGLE("DAC Mute", S5M8751_DA_DIG2, 7, 1, 0),
	SOC_SINGLE("Mute Dither", S5M8751_DA_DIG2, 6, 1, 0),
	SOC_SINGLE("Dither Level", S5M8751_DA_DIG2, 3, 7, 0),
	SOC_SINGLE("Soft Limit", S5M8751_DA_LIM1, 7, 1, 0),
	SOC_SINGLE("Limit Thrsh", S5M8751_DA_LIM1, 0, 0x7f, 1),
	SOC_SINGLE("Attack", S5M8751_DA_LIM2, 4, 0xf, 0),
	SOC_SINGLE("Release", S5M8751_DA_LIM2, 0, 0xf, 0),
};

/* Add non-DAPM controls */
static int s5m8751_add_controls(struct snd_soc_codec *codec)
{
	int err, i;

	for (i = 0; i < ARRAY_SIZE(s5m8751_snd_controls); i++) {
		err = snd_ctl_add(codec->card,
				  snd_soc_cnew(&s5m8751_snd_controls[i],
					       codec, NULL));
		if (err < 0)
			return err;
	}
	return 0;
}

static const struct snd_soc_dapm_widget s5m8751_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("RL-Mute", "Playback", S5M8751_AMP_MUTE, 0, 1),
	SND_SOC_DAPM_DAC("LL-Mute", "Playback", S5M8751_AMP_MUTE, 1, 1),
	SND_SOC_DAPM_DAC("RH-Mute", "Playback", S5M8751_AMP_MUTE, 2, 1),
	SND_SOC_DAPM_DAC("LH-Mute", "Playback", S5M8751_AMP_MUTE, 3, 1),
	SND_SOC_DAPM_DAC("SPK-Mute", "Playback", S5M8751_AMP_MUTE, 4, 1),
	SND_SOC_DAPM_OUTPUT("RL"),
	SND_SOC_DAPM_OUTPUT("LL"),
	SND_SOC_DAPM_OUTPUT("RH"),
	SND_SOC_DAPM_OUTPUT("LH"),
	SND_SOC_DAPM_OUTPUT("SPK"),
};

static const struct snd_soc_dapm_route audio_map[] = {
	{ "RL", NULL, "RL-Mute" },
	{ "LL", NULL, "LL-Mute" },
	{ "RH", NULL, "RH-Mute" },
	{ "LH", NULL, "LH-Mute" },
	{ "SPK", NULL, "SPK-Mute" },
};

static int s5m8751_add_widgets(struct snd_soc_codec *codec)
{
	snd_soc_dapm_new_controls(codec, s5m8751_dapm_widgets,
				  ARRAY_SIZE(s5m8751_dapm_widgets));
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));
	snd_soc_dapm_new_widgets(codec);

	return 0;
}

/*
 * Set PCM DAI bit size and sample rate.
 */
static int s5m8751_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
	unsigned int in1_ctrl2, in1_ctrl1;

	in1_ctrl1 = s5m8751_read_reg_cache(codec, S5M8751_IN1_CTRL1);
	in1_ctrl1 &= ~(0xf<<2);	/* Sampling Rate field */

	in1_ctrl2 = s5m8751_read_reg_cache(codec, S5M8751_IN1_CTRL2);
	in1_ctrl2 &= ~(0x3<<0);	/* I2S data length field */

	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		in1_ctrl2 |= (0x0<<0); break;
	case SNDRV_PCM_FORMAT_S18_3LE:
		in1_ctrl2 |= (0x1<<0); break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		in1_ctrl2 |= (0x2<<0); break;
	case SNDRV_PCM_FORMAT_S24_LE:
		in1_ctrl2 |= (0x3<<0); break;
	default: /* Doesn't it support SNDRV_PCM_FORMAT_S8 ?! */
		return -EINVAL;
	}

	/* XXX What about bits in DA_DIG1 ? XXX */
	switch (params_rate(params)) {
	case 8000:
		in1_ctrl1 |= (0x0<<2); break;
	case 11025:
		in1_ctrl1 |= (0x1<<2); break;
	case 16000:
		in1_ctrl1 |= (0x2<<2); break;
	case 22050:
		in1_ctrl1 |= (0x3<<2); break;
	case 32000:
		in1_ctrl1 |= (0x4<<2); break;
	case 44100: 
		in1_ctrl1 |= (0x5<<2); break;
	case 48000:
		in1_ctrl1 |= (0x6<<2); break;
	case 64000:
		in1_ctrl1 |= (0x7<<2); break;
	case 88200:
		in1_ctrl1 |= (0x8<<2); break;
	case 96000:
		in1_ctrl1 |= (0x9<<2); break;
	default:
		return -EINVAL;
	}

//	in1_ctrl1 &= ~(1<<7);	/* Power down */
	s5m8751_write(codec, S5M8751_IN1_CTRL1, in1_ctrl1);

	s5m8751_write(codec, S5M8751_IN1_CTRL2, in1_ctrl2);

//	in1_ctrl1 |= (1<<7);	/* Power up */
//	s5m8751_write(codec, S5M8751_IN1_CTRL1, in1_ctrl1);

	return 0;
}

static int s5m8751_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	unsigned int in1_ctrl2, in1_ctrl1;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	/* S5M8751 works only in SLAVE mode */
	case SND_SOC_DAIFMT_CBS_CFS: break;	/* Nothing to do, already setup */
	default:
		printk("Mst-Slv combo(%d) not supported!\n", fmt & SND_SOC_DAIFMT_MASTER_MASK);
		return -EINVAL;
	}

	in1_ctrl2 = s5m8751_read_reg_cache(codec, S5M8751_IN1_CTRL2);
	in1_ctrl2 &= ~(0x3<<2);
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		in1_ctrl2 |= (0x0<<2); break;
	case SND_SOC_DAIFMT_LEFT_J:
		in1_ctrl2 |= (0x1<<2); break;
	case SND_SOC_DAIFMT_RIGHT_J:
		in1_ctrl2 |= (0x2<<2); break;
	default:
		printk("DAIFmt(%d) not supported!\n", fmt & SND_SOC_DAIFMT_FORMAT_MASK);
		return -EINVAL;
	}

	in1_ctrl2 &= ~(0x1<<4); /* LRCLK Polarity */
	in1_ctrl1 = s5m8751_read_reg_cache(codec, S5M8751_IN1_CTRL1);
	in1_ctrl1 &= ~(0x1<<1); /* BCLK Polarity */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		in1_ctrl1 |= (0x0<<1);
		in1_ctrl2 |= (0x0<<4);
		break;

	case SND_SOC_DAIFMT_IB_IF:
		in1_ctrl1 |= (0x1<<1);
		in1_ctrl2 |= (0x1<<4);
		break;

	case SND_SOC_DAIFMT_IB_NF:
		in1_ctrl1 |= (0x1<<1);
		in1_ctrl2 |= (0x0<<4);
		break;

	case SND_SOC_DAIFMT_NB_IF:
		in1_ctrl1 |= (0x0<<1);
		in1_ctrl2 |= (0x1<<4);
		break;

	default:
		printk("Inv-combo(%d) not supported!\n", fmt & SND_SOC_DAIFMT_FORMAT_MASK);
		return -EINVAL;
	}

	s5m8751_write(codec, S5M8751_IN1_CTRL1, in1_ctrl1);
	s5m8751_write(codec, S5M8751_IN1_CTRL2, in1_ctrl2);
	return 0;
}

static int s5m8751_set_clkdiv(struct snd_soc_dai *codec_dai,
				 int div_id, int val)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	unsigned int in1_ctrl1, in1_ctrl2;

	in1_ctrl2 = s5m8751_read_reg_cache(codec, S5M8751_IN1_CTRL2);
	in1_ctrl2 &= ~(0x3<<5);	/* XFS field */
	switch (div_id) {
	case S5M8751_BCLK:
		switch(val){
		case 32: in1_ctrl2 |= (0x0<<5); break;
		case 48: in1_ctrl2 |= (0x1<<5); break;
		case 64: in1_ctrl2 |= (0x2<<5); break;
		default: return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	in1_ctrl1 = s5m8751_read_reg_cache(codec, S5M8751_IN1_CTRL1);
	s5m8751_write(codec, S5M8751_IN1_CTRL1, in1_ctrl1);

	s5m8751_write(codec, S5M8751_IN1_CTRL2, in1_ctrl2);

	return 0;
}

static int s5m8751_set_bias_level(struct snd_soc_codec *codec,
	enum snd_soc_bias_level level)
{
	u16 amp_en, amp_mute_off, in1_ctrl1;

	in1_ctrl1 = s5m8751_read_reg_cache(codec, S5M8751_IN1_CTRL1);
	amp_en = s5m8751_read_reg_cache(codec, S5M8751_AMP_EN);
	amp_mute_off = s5m8751_read_reg_cache(codec, S5M8751_AMP_MUTE);
	
	amp_en &= ~((0x7<<2)|(0x1<<6));
	amp_mute_off &= ~((0x3<<2)|(0x1<<4));
	
	s5m8751_write(codec, S5M8751_AMP_MUTE, amp_mute_off);
	mdelay(30);	
	s5m8751_write(codec, S5M8751_AMP_EN, amp_en);

	switch (level) {
	case SND_SOC_BIAS_ON:
		in1_ctrl1 &= ~(0x1<<7);
		s5m8751_write(codec, S5M8751_IN1_CTRL1, in1_ctrl1);
		in1_ctrl1 |= (0x1<<7);
		s5m8751_write(codec, S5M8751_IN1_CTRL1, in1_ctrl1);
#if defined(CONFIG_SOUND_S5M8751_OUTPUT_STREAM_HP_OUT)
		amp_mute_off |= (0x3<<2);
		amp_en |= (0x7<<2);
#endif

#if defined(CONFIG_SOUND_S5M8751_OUTPUT_STREAM_SPK_OUT)
		amp_mute_off |= (0x1<<4);
		amp_en |= (0x1<<6);
#endif
		s5m8751_write(codec, S5M8751_AMP_EN, amp_en);
		s5m8751_write(codec, S5M8751_AMP_MUTE, amp_mute_off);
		break;
	case SND_SOC_BIAS_PREPARE:
	case SND_SOC_BIAS_STANDBY:
		amp_en &= ~((0x7<<2)|(0x1<<6));
		amp_mute_off &= ~((0x3<<2)|(0x1<<4));
		s5m8751_write(codec, S5M8751_AMP_MUTE, amp_mute_off);
		s5m8751_write(codec, S5M8751_AMP_EN, amp_en);
		in1_ctrl1 &= ~(1<<7);
		in1_ctrl1 &= ~(1<<7);
		s5m8751_write(codec, S5M8751_IN1_CTRL1, in1_ctrl1);
		break;
	case SND_SOC_BIAS_OFF:
		amp_en &= ~((0x7<<2)|(0x1<<6));
		amp_mute_off &= ~((0x3<<2)|(0x1<<4));
		s5m8751_write(codec, S5M8751_AMP_MUTE, amp_mute_off);
		s5m8751_write(codec, S5M8751_AMP_EN, amp_en);
		in1_ctrl1 &= ~(1<<7);
		s5m8751_write(codec, S5M8751_IN1_CTRL1, in1_ctrl1);
		break;
	}

	codec->bias_level = level;
	return 0;
}

/* 
 * S5M8751 can work only in Slave mode.
 * S5M8751 supports only Playback, not Capture!
 * S5M8751 supports two i/f's: I2S and PCM. Only I2S is implemented in this driver.
 */
#define S5M8751_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S18_3LE | \
				SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S24_LE) 
struct snd_soc_dai s5m8751_dai = {
		.name = "S5M8751-I2S",
		.id = 0,
		.playback = {
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_96000,
			.formats = S5M8751_FORMATS,
		},
		.ops = {
			 .hw_params = s5m8751_hw_params,
		},
		.dai_ops = {
			 .set_fmt = s5m8751_set_dai_fmt,
			 .set_clkdiv = s5m8751_set_clkdiv,
		},
};
EXPORT_SYMBOL_GPL(s5m8751_dai);

/*
 * initialise the S5M8751 driver
 * register the mixer and dsp interfaces with the kernel
 */
static int s5m8751_init(struct snd_soc_device *socdev)
{
	int ret = 0, val;
	u16 *cache;
	struct snd_soc_codec *codec = socdev->codec;

	codec->name = "S5M8751";
	codec->owner = THIS_MODULE;
	codec->read = s5m8751_read_reg_cache;
	codec->write = s5m8751_write;
	codec->set_bias_level = s5m8751_set_bias_level;
	codec->dai = &s5m8751_dai;
	codec->num_dai = 1;
	codec->reg_cache_size = ARRAY_SIZE(s5m8751_reg);
	codec->reg_cache = kmemdup(s5m8751_reg, sizeof(s5m8751_reg), GFP_KERNEL);

	if (codec->reg_cache == NULL)
		return -ENOMEM;

	/* Fill the reg cache */
	cache = codec->reg_cache;
	for(val=S5M8751_DA_PDB1; val<=S5M8751_LINE_CTRL; val++){ /* Don't use Power Mngmnt regs here */
		while(s5m8751_read(codec, val) == -EIO)
			printk(KERN_WARNING "Read failed! ");
	}

#if defined(CONFIG_SOUND_S5M8751_OUTPUT_STREAM_HP_OUT)
	val = (0x3d<<0); /* PowerOn */
	s5m8751_write(codec, S5M8751_DA_PDB1, val);
	
	val = (0x21<<0); /* Enable */
	s5m8751_write(codec, S5M8751_DA_AMIX1, val);
	
	val = 0; /* XXX TODO Disable all for the time being TODO XXX */
	s5m8751_write(codec, S5M8751_DA_AMIX2, val);

	val = 0x30;
	s5m8751_write(codec, S5M8751_DA_VOLL, val);
	s5m8751_write(codec, S5M8751_DA_VOLR, val);
	
	val = 0x28;
	s5m8751_write(codec, S5M8751_AMP_CTRL, val);
#endif

#if defined(CONFIG_SOUND_S5M8751_OUTPUT_STREAM_SPK_OUT)
	val = (0x3a<<0); /* PowerOn */
	s5m8751_write(codec, S5M8751_DA_PDB1, val);

	val = (0x12<<0); /* Enable  */
	s5m8751_write(codec, S5M8751_DA_AMIX1, val);

	val = 0; /* XXX TODO Disable all for the time being TODO XXX */
	s5m8751_write(codec, S5M8751_DA_AMIX2, val);

	val = 0x18;
	s5m8751_write(codec, S5M8751_DA_VOLL, val);
	s5m8751_write(codec, S5M8751_DA_VOLR, val);

	val = 0x01;	/* Gradual Slope */
	s5m8751_write(codec, S5M8751_SPK_SLOPE, val);
	val = 0x05;
	s5m8751_write(codec, S5M8751_SPK_DT, val);
	val = 0x00;
	s5m8751_write(codec, S5M8751_SPK_S2D, val);
#endif
	
	/* TODO XXX What about DA_ANA, DA_DWA? TODO XXX */
	codec->bias_level = SND_SOC_BIAS_OFF;

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "s5m8751: failed to create pcms\n");
		goto pcm_err;
	}

	s5m8751_add_controls(codec);
	s5m8751_add_widgets(codec);

	ret = snd_soc_register_card(socdev);
	if (ret < 0) {
		printk(KERN_ERR "s5m8751: failed to register card\n");
		goto card_err;
	}
	return ret;

card_err:
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
pcm_err:
	kfree(codec->reg_cache);
	return ret;
}

/* If the i2c layer weren't so broken, we could pass this kind of data
   around */
static struct snd_soc_device *s5m8751_socdev;

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)

static unsigned short normal_i2c[] = { 0, I2C_CLIENT_END };

/* Magic definition of all other variables and things */
I2C_CLIENT_INSMOD;

static struct i2c_driver s5m8751_i2c_driver;
static struct i2c_client client_template;

static int s5m8751_codec_probe(struct i2c_adapter *adap, int addr, int kind)
{
	struct snd_soc_device *socdev = s5m8751_socdev;
	struct s5m8751_setup_data *setup = socdev->codec_data;
	struct snd_soc_codec *codec = socdev->codec;
	struct i2c_client *i2c;
	int ret;

	if (addr != setup->i2c_address){
		return -ENODEV;
	}

	client_template.adapter = adap;
	client_template.addr = addr;

	i2c =  kmemdup(&client_template, sizeof(client_template), GFP_KERNEL);
	if (i2c == NULL) {
		kfree(codec);
		return -ENOMEM;
	}
	i2c_set_clientdata(i2c, codec);
	codec->control_data = i2c;

	ret = i2c_attach_client(i2c);
	if (ret < 0) {
		dev_err(&i2c->dev, "failed to attach codec at addr %x\n", addr);
		goto err;
	}

	ret = s5m8751_init(socdev);
	if (ret < 0) {
		dev_err(&i2c->dev, "failed to initialise S5M8751\n");
		goto err;
	}

	return ret;

err:
	kfree(codec);
	kfree(i2c);
	return ret;
}

static int s5m8751_i2c_detach(struct i2c_client *client)
{
	struct snd_soc_codec *codec = i2c_get_clientdata(client);
	i2c_detach_client(client);
	kfree(codec->reg_cache);
	kfree(client);
	return 0;
}

static int s5m8751_i2c_attach(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data, s5m8751_codec_probe);
}

static struct i2c_driver s5m8751_i2c_driver = {
	.driver = {
		.name = "S5M8751 I2C Codec",
		.owner = THIS_MODULE,
	},
	.attach_adapter = s5m8751_i2c_attach,
	.detach_client =  s5m8751_i2c_detach,
	.command =        NULL,
};

static struct i2c_client client_template = {
	.name =   "S5M8751",
	.driver = &s5m8751_i2c_driver,
};
#endif

static int s5m8751_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct s5m8751_setup_data *setup;
	struct snd_soc_codec *codec;
	int ret = 0;

	pr_info("S5M8751 Audio Codec %s\n", S5M8751_VERSION);

	setup = socdev->codec_data;
	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;

	socdev->codec = codec;
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);
	s5m8751_socdev = socdev;

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	if (setup->i2c_address) {
		normal_i2c[0] = setup->i2c_address;
		codec->hw_write = (hw_write_t)i2c_master_send;
		codec->hw_read = (hw_read_t)i2c_master_recv;
		ret = i2c_add_driver(&s5m8751_i2c_driver);
		if (ret != 0)
			printk(KERN_ERR "can't add i2c driver");
	}
#else
		/* Add other interfaces here */
#endif

	return ret;
}

/* power down chip */
static int s5m8751_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	if (codec->control_data)
		s5m8751_set_bias_level(codec, SND_SOC_BIAS_OFF);
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	i2c_del_driver(&s5m8751_i2c_driver);
#endif
	kfree(codec);

	return 0;
}

struct snd_soc_codec_device soc_codec_dev_s5m8751 = {
	.probe = 	s5m8751_probe,
	.remove = 	s5m8751_remove,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_s5m8751);

MODULE_DESCRIPTION("ASoC S5M8751 driver");
MODULE_AUTHOR("Jaswinder Singh <jassi.brar@samsung.com>");
