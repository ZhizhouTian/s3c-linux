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

#ifndef _S3C_G2D_DRIVER_ANDROID_H_
#define _S3C_G2D_DRIVER_ANDROID_H_

#include "s3c_fimg2d_common.h"
#define S3C_G2D_NAME "/dev/s3c-g2d"

/*Patrick Added Start*/

#define ANDROID_S3C_G2D_ROTATOR_0			_IO(G2D_IOCTL_MAGIC,6)
#define ANDROID_S3C_G2D_ROTATOR_90			_IO(G2D_IOCTL_MAGIC,7)
#define ANDROID_S3C_G2D_ROTATOR_180			_IO(G2D_IOCTL_MAGIC,8)
#define ANDROID_S3C_G2D_ROTATOR_270			_IO(G2D_IOCTL_MAGIC,9)
#define ANDROID_S3C_G2D_ROTATOR_X_FLIP			_IO(G2D_IOCTL_MAGIC,10)
#define ANDROID_S3C_G2D_ROTATOR_Y_FLIP			_IO(G2D_IOCTL_MAGIC,11)


/*Patrick Added End*/

/* Patrick Added Start*/
typedef struct
{
	uint32_t	src_base_addr;			//Base address of the source image
	uint32_t	src_full_width;			//source image full width
	uint32_t	src_full_height;		//source image full height
	uint32_t	src_start_x;			//coordinate start x of source image
	uint32_t	src_start_y;			//coordinate start y of source image
	uint32_t	src_work_width;			//source image width for work
	uint32_t src_work_height;			//source image height for work
	uint32_t src_offset; 				// Patrick Added (for Android)
	int	src_memory_id;					//Patrick Added (for Android) file descriptor

	uint32_t	dst_base_addr;			//Base address of the destination image	
	uint32_t	dst_full_width;			//destination screen full width
	uint32_t	dst_full_height;		//destination screen full width
	uint32_t	dst_start_x;			//coordinate start x of destination screen
	uint32_t	dst_start_y;			//coordinate start y of destination screen
	uint32_t	dst_work_width;			//destination screen width for work
	uint32_t dst_work_height;			//destination screen height for work
	uint32_t dst_offset; 				// Patrick Added (for Android)
	int dst_memory_id; 					//Patrick Added (for Android) file descriptor
	

	// Coordinate (X, Y) of clipping window
	uint32_t cw_x1, cw_y1;
	uint32_t cw_x2, cw_y2;

	uint32_t color_val[8];
	G2D_COLOR_SPACE src_bpp;
	G2D_COLOR_SPACE dst_bpp;

	uint32_t	alpha_mode;				//true : enable, false : disable
	uint32_t	alpha_val;
	uint32_t	color_key_mode;			//treu : enable, false : disable
	uint32_t	color_key_val;			//transparent color value
	
}android_s3c_g2d_params;
/* Patrick Added End*/


#endif /*_S3C_G2D_DRIVER_ANDROID_H_*/
