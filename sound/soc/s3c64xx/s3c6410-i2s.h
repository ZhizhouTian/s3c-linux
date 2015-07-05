/*
 * s3c64xx-i2s.c  --  ALSA Soc Audio Layer
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef S3C64XXI2S_H_
#define S3C64XXI2S_H_

#include <plat/regs-iis.h>

#ifndef CONFIG_SND_S3C6410_SOC_I2S_V32
#define IIS_V40	1
#endif

/* clock sources */
#define S3C6410_CLKSRC_PCLK		S3C64XX_IISMOD_MSTPCLK
#define S3C6410_CLKSRC_CLKAUDIO		S3C64XX_IISMOD_MSTCLKAUDIO
#define S3C6410_CLKSRC_SLVPCLK		S3C64XX_IISMOD_SLVPCLK
#define S3C6410_CLKSRC_I2SEXT		S3C64XX_IISMOD_SLVI2SCLK
#define S3C6410_CDCLKSRC_INT		(4<<10)
#define S3C6410_CDCLKSRC_EXT		(5<<10)

/* Clock dividers */
#define S3C64XX_DIV_MCLK	0
#define S3C64XX_DIV_BCLK	1
#define S3C64XX_DIV_PRESCALER	2

#define USE_CLKAUDIO	1

u32 s3c6410_i2s_get_clockrate(void);

#ifdef CONFIG_SND_S3C6410_SOC_I2S_V32
extern struct snd_soc_dai s3c6410_i2s_v32_dai;
#else
extern struct snd_soc_dai s3c6410_i2s_v40_dai;
#endif

#endif /*S3C64XXI2S_H_*/
