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
 */

#ifndef _S3C_G2D_DRIVER_ANDROID_H_
#define _S3C_G2D_DRIVER_ANDROID_H_
#include "s3c_fimg2d2x_common.h"

#define ANDROID_S3C_G2D_SET_ALPHA			_IO(G2D_IOCTL_MAGIC,6)
#define ANDROID_S3C_G2D_BILT				_IO(G2D_IOCTL_MAGIC,7)
#define ANDROID_S3C_G2D_STRETCH_BILT		_IO(G2D_IOCTL_MAGIC,8)
#define ANDROID_S3C_WAIT_G2D_COMPLETION		_IO(G2D_IOCTL_MAGIC,9)

#define S3C_G2D_NAME "/dev/s3c-g2d"
#define G2D_SUPPORT_ASHMEM

//#define G2D_DEBUG_INFO

#ifdef G2D_SUPPORT_ASHMEM
#define G2D_SRC_IS_ASHMEM 	(1 << 0)
#define G2D_DST_IS_ASHMEM 	(1 << 1)
#endif


// need to flush before use
#define G2D_SRC_IS_PMEM 	(1 << 2)
#define G2D_DST_IS_PMEM 	(1 << 3)

#define G2D_SRC_IS_FB		(1 << 4)
#define G2D_DST_IS_FB		(1 << 5)
 
typedef enum
{
//#define S3C_G2D_COLOR_RGB_565				(0x0<<0)
//#define S3C_G2D_COLOR_RGBA_5551				(0x1<<0)
//#define S3C_G2D_COLOR_ARGB_1555				(0x2<<0)
//#define S3C_G2D_COLOR_RGBA_8888				(0x3<<0)
//#define S3C_G2D_COLOR_ARGB_8888				(0x4<<0)
//#define S3C_G2D_COLOR_XRGB_8888				(0x5<<0)
//#define S3C_G2D_COLOR_RGBX_8888				(0x6<<0)	

  G2D_RGB16,G2D_RGBA16,G2D_ARGB16,G2D_RGBA32,G2D_ARGB32,G2D_XRGB32,G2D_RGBX32
	
} ANDROID_G2D_COLOR_SPACE;



typedef struct
{
	uint32_t 	color_val[8];
	uint32_t	alpha_mode;			//true : enable, false : disable
	uint32_t	alpha_val;
	uint32_t	color_key_mode;			//treu : enable, false : disable
	uint32_t	color_key_val;			//transparent color value
		
} android_s3c_set_alpha_params;


typedef struct
{
	uint32_t	src_base_addr;			//Base address of the source image
	uint32_t	src_full_width;			//source image full width
	uint32_t	src_full_height;		//source image full height
	uint32_t	src_start_x;			//coordinate start x of source image
	uint32_t	src_start_y;			//coordinate start y of source image
	uint32_t	src_work_width;			//source image width for work
	uint32_t 	src_work_height;		//source image height for work
	uint32_t	dst_base_addr;			//Base address of the destination image	
	uint32_t	dst_full_width;			//destination screen full width
	uint32_t	dst_full_height;		//destination screen full width
	uint32_t	dst_start_x;			//coordinate start x of destination screen
	uint32_t	dst_start_y;			//coordinate start y of destination screen
	uint32_t	dst_work_width;			//destination screen width for work
	uint32_t 	dst_work_height;		//destination screen height for work
	// Coordinate (X, Y) of clipping window
	uint32_t cw_x1, cw_y1;
	uint32_t cw_x2, cw_y2;
	ANDROID_G2D_COLOR_SPACE src_bpp;  	// source bpp mode
	ANDROID_G2D_COLOR_SPACE dst_bpp;    // destination bpp mode
	ROT_DEG rot_degree;
	
	
	uint32_t srcdst_mem_flag; 	// bit 0 --> src is ashmem
								// bit 1 --> dst is ashmem
								// bit 2 --> src is pmem
								// bit 3 --> dst is pmem
	uint32_t src_mem_size;	    // for pmem or ashmem 
	uint32_t dst_mem_size;		// for pmem or ashmem
	int src_mem_fd;
	int dst_mem_fd;

#ifdef G2D_DEBUG_INFO
	uint32_t src_base_info;
	uint32_t dst_base_info;
	uint32_t src_offset;
	uint32_t dst_offset;
#endif						  
							  	
} android_s3c_g2d_blit_params;


#endif /*_S3C_G2D_DRIVER_H_*/
