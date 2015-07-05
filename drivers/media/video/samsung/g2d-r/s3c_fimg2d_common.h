/* linux/drivers/media/video/samsung/g2d/s3c_fimg2d2x.h
 *
 * Driver header file for Samsung 2D Accelerator(FIMG-2D)
 *
 * Jonghun Han, Copyright (c) 2009 Samsung Electronics
 * 	http://www.samsungsemi.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * 2010/8/2 Modified by Patrick Chang
 */

#ifndef _S3C_G2D_DRIVER_COMMON_H_
#define _S3C_G2D_DRIVER_COMMON_H_

// Common
#define G2D_SFR_SIZE	0x1000

#define TRUE		1
#define FALSE		0

#define G2D_MINOR	220	// Just some number

#define G2D_IOCTL_MAGIC 'G'

#define FIFO_NUM				32

#define G2D_TIMEOUT				100
#define ALPHA_VALUE_MAX				255

#define G2D_MAX_WIDTH				(2048)
#define G2D_MAX_HEIGHT				(2048)


#define ABS(v) 	(((v)>=0) ? (v):(-(v)))

typedef enum
{
	ROT_0, ROT_90, ROT_180, ROT_270, ROT_X_FLIP, ROT_Y_FLIP
} ROT_DEG;

typedef enum
{
	ROP_DST_ONLY,
	ROP_SRC_ONLY, 
	ROP_3RD_OPRND_ONLY,
	ROP_SRC_AND_DST,
	ROP_SRC_AND_3RD_OPRND,
	ROP_SRC_OR_DST,
	ROP_SRC_OR_3RD_OPRND,	
	ROP_DST_OR_3RD,
	ROP_SRC_XOR_3RD_OPRND

} G2D_ROP_TYPE;

typedef enum
{
	G2D_NO_ALPHA_MODE,
	G2D_PP_ALPHA_SOURCE_MODE,
	G2D_ALPHA_MODE,
	G2D_FADING_MODE
} G2D_ALPHA_BLENDING_MODE;

typedef enum
{
	G2D_BLACK = 0, G2D_RED = 1, G2D_GREEN = 2, G2D_BLUE = 3, G2D_WHITE = 4, 
	G2D_YELLOW = 5, G2D_CYAN = 6, G2D_MAGENTA = 7
} G2D_COLOR;

/*
#define S3C_G2D_COLOR_RGB_565				(0x0<<0)
#define S3C_G2D_COLOR_RGBA_5551				(0x1<<0)
#define S3C_G2D_COLOR_ARGB_1555				(0x2<<0)
#define S3C_G2D_COLOR_RGBA_8888				(0x3<<0)
#define S3C_G2D_COLOR_ARGB_8888				(0x4<<0)
#define S3C_G2D_COLOR_XRGB_8888				(0x5<<0)
#define S3C_G2D_COLOR_RGBX_8888				(0x6<<0)
*/

typedef enum
{
	PAL1, PAL2, PAL4, PAL8,
	RGB8, ARGB8, RGB16, ARGB16, RGB18, RGB24, RGB30, ARGB24,RGBA16,RGBX24,RGBA24,
	YC420, YC422, // Non-interleave
	CRYCBY, CBYCRY, YCRYCB, YCBYCR, YUV444, // Interleave
	RGBX32, XRGB32,ARGB32,RGBA32,C3RGB24
} G2D_COLOR_SPACE;


#endif /*_S3C_G2D_DRIVER_COMMON_H_*/
