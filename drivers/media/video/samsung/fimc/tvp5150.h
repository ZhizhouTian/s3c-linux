/* linux/drivers/media/video/samsung/tvp5150.h
 *
 *
 * tvp5150 - Texas Instruments TVP5150A/AM1 video decoder driver
 *
 * Figo Wang, Copyright (c) 2010
 * 	sagres_2004@163.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef _TVP5150_H_
#define _TVP5150_H_

u8 tvp5150_init_reg[][2] =
{
  {0x0a, 0xa0}, //color saturation control		//gavin 09-09-26 设置色饱和初值
  {0x0c, 0xBE},//contrast control				//gavin 09-09-26 设置对比度初值
  {0x03, 0x0D},	

  {0x12, 0x00},
#if defined(CONFIG_AIP1B)
  {0x00, 0x02},		// CVBS-CH2
#else
  {0x00, 0x00},		// CVBS-CH1	
#endif			
  {0x28, 0x00},				
  {0x0F, 0x0A},		// set pin 27 = GPCL for v02.03
  {0x03, 0x6F},		// GPCL HIGH FOR ANALOG SW to CVBS, YUV output enable
  {0x15, 0x05},		// 0x05: ADI RTC mode
  {0xC8, 0x80},		// BuffThresh			set to trigger int when 1 transaction is stored
	{0xCA, 0x8C},		// IntLineNo			enable odd field, set to line 12
	{0xCE, 0x01},		// VidStandard			set 601 sampling
	{0xCF, 0x00},		// Full field enable    disable
	{0xEE, 0xE7},		// Line21 (Field 1)- CC NTSC, was 0xE7/0xC7
	{0xEF, 0xE7},		// Line21 (Field 2)- CC NTSC, was 0xE7/0xC7 
	{0xCB, 0x4E},		// Set Pixel Alignment [7:0] to 4Eh
	{0xCC, 0x00},		// Set pixel Alignment [9:8] to 0
	{0xCD, 0x00},		// Disable Host access VBI FIFO: FIFO outputs to 656 port
	{0xC9, 0x00},		// Reset FIFO
};

#define tvp5150_INIT_REGS	(sizeof(tvp5150_init_reg) / sizeof(tvp5150_init_reg[0]))

#endif
