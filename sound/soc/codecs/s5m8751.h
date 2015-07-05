/*
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef _S5M8751_H
#define _S5M8751_H

/* S5M8751 register space */
#define S5M8751_IRQB_EVENT1		0x0
#define S5M8751_IRQB_EVENT2		0x1
#define S5M8751_IRQB_MASK1		0x2
#define S5M8751_IRQB_MASK2		0x3
#define S5M8751_ONOFF1			0x4
#define S5M8751_ONOFF2			0x5
#define S5M8751_ONOFF3			0x6
#define S5M8751_SLEEPCNTL1		0x7
#define S5M8751_SLEEPCNTL2		0x8
#define S5M8751_UVLO			0x9
#define S5M8751_LDO_AUDIO_VSET		0x0A
#define S5M8751_LDO1_VSET		0x0B
#define S5M8751_LDO2_VSET		0x0C
#define S5M8751_LDO3_VSET		0x0D
#define S5M8751_LDO4_VSET		0x0E
#define S5M8751_LDO_MEMORY_VSET		0x0F
#define S5M8751_BUCK1_V1_SET		0x10
#define S5M8751_BUCK1_V2_SET		0x11
#define S5M8751_BUCK2_V1_SET		0x12
#define S5M8751_BUCK2_V2_SET		0x13
#define S5M8751_WLED_CNTRL		0x14
#define S5M8751_CHG_IV_SET		0x15
#define S5M8751_CHG_CNTRL		0x16
#define S5M8751_DA_PDB1			0x17
#define S5M8751_DA_AMIX1		0x18
#define S5M8751_DA_AMIX2		0x19
#define S5M8751_DA_ANA			0x1A
#define S5M8751_DA_DWA			0x1B
#define S5M8751_DA_VOLL			0x1C
#define S5M8751_DA_VOLR			0x1D
#define S5M8751_DA_DIG1			0x1E
#define S5M8751_DA_DIG2			0x1F
#define S5M8751_DA_LIM1			0x20
#define S5M8751_DA_LIM2			0x21
#define S5M8751_DA_LOF			0x22
#define S5M8751_DA_ROF			0x23
#define S5M8751_DA_MUX			0x24
#define S5M8751_DA_LGAIN		0x25
#define S5M8751_DA_RGAIN		0x26
#define S5M8751_IN1_CTRL1		0x27
#define S5M8751_IN1_CTRL2		0x28
#define S5M8751_IN1_CTRL3		0x29
#define S5M8751_SLOT_L2			0x2A
#define S5M8751_SLOT_L1			0x2B
#define S5M8751_SLOT_R2			0x2C
#define S5M8751_SLOT_R1			0x2D
#define S5M8751_TSLOT			0x2E
#define S5M8751_TEST			0x2F
#define S5M8751_SPK_SLOPE		0x30
#define S5M8751_SPK_DT			0x31
#define S5M8751_SPK_S2D			0x32
#define S5M8751_SPK_CM			0x33
#define S5M8751_SPK_DUM			0x34
#define S5M8751_HP_VOL1			0x35
#define S5M8751_HP_VOL2			0x36
#define S5M8751_AMP_EN			0x37
#define S5M8751_AMP_MUTE		0x38
#define S5M8751_AMP_CTRL		0x39
#define S5M8751_AMP_VMID		0x3A
#define S5M8751_LINE_CTRL		0x3B
#define S5M8751_NUMREGS			(S5M8751_LINE_CTRL + 1)
	/* Careful, 7regs skipped */
#define S5M8751_CHIP_ID			0x43
#define S5M8751_STATUS			0x44

#define S5M8751_SYSCLK 0
#define S5M8751_MCLK   1
#define S5M8751_BCLK   2

#define MUTE_OFF  0
#define MUTE_ON	  1

struct s5m8751_setup_data {
	unsigned short i2c_address;
};

extern struct snd_soc_dai s5m8751_dai;
extern struct snd_soc_codec_device soc_codec_dev_s5m8751;

#endif
