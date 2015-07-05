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

#ifndef _S3C_G2D_DRIVER_H_
#define _S3C_G2D_DRIVER_H_
#include <plat/regs-g2d.h>
#include "s3c_fimg2d2x_common.h"
#include "s3c_fimg2d2x_android.h"

#define S3C_G2D_ROTATOR_0			_IO(G2D_IOCTL_MAGIC,0)
#define S3C_G2D_ROTATOR_90			_IO(G2D_IOCTL_MAGIC,1)
#define S3C_G2D_ROTATOR_180			_IO(G2D_IOCTL_MAGIC,2)
#define S3C_G2D_ROTATOR_270			_IO(G2D_IOCTL_MAGIC,3)
#define S3C_G2D_ROTATOR_X_FLIP			_IO(G2D_IOCTL_MAGIC,4)
#define S3C_G2D_ROTATOR_Y_FLIP			_IO(G2D_IOCTL_MAGIC,5)

#define FIFO_NUM				32

#define G2D_TIMEOUT				100


#define ABS(v) 	(((v)>=0) ? (v):(-(v)))

#define G2D_ROP_SRC_ONLY			(0xf0)
#define G2D_ROP_3RD_OPRND_ONLY			(0xaa)
#define G2D_ROP_DST_ONLY			(0xcc)
#define G2D_ROP_SRC_OR_DST			(0xfc)
#define G2D_ROP_SRC_OR_3RD_OPRND		(0xfa)
#define G2D_ROP_SRC_AND_DST			(0xc0) //(pat==1)? src:dst
#define G2D_ROP_SRC_AND_3RD_OPRND		(0xa0)
#define G2D_ROP_SRC_XOR_3RD_OPRND		(0x5a)
#define G2D_ROP_DST_OR_3RD_OPRND		(0xee)

typedef enum
{
	PAL1, PAL2, PAL4, PAL8,
	RGB8, ARGB8, RGB16, ARGB16, RGB18, RGB24, RGB30, ARGB24,RGBA16,RGBX24,RGBA24,
	YC420, YC422, // Non-interleave
	CRYCBY, CBYCRY, YCRYCB, YCBYCR, YUV444 // Interleave
} G2D_COLOR_SPACE;

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

	uint32_t color_val[8];
	G2D_COLOR_SPACE bpp;

	uint32_t	alpha_mode;			//true : enable, false : disable
	uint32_t	alpha_val;
	uint32_t	color_key_mode;			//treu : enable, false : disable
	uint32_t	color_key_val;			//transparent color value
	
}s3c_g2d_params;

/**** function declearation***************************/
static int s3c_g2d_init_regs(s3c_g2d_params *params);
void s3c_g2d_bitblt(uint16_t src_x1, uint16_t src_y1, uint16_t src_x2, uint16_t src_y2,
 	 uint16_t dst_x1, uint16_t dst_y1, uint16_t dst_x2, uint16_t dst_y2);
static void s3c_g2d_rotate_with_bitblt(s3c_g2d_params *params, ROT_DEG rot_degree);
static void s3c_g2d_get_rotation_origin(uint16_t src_x1, uint16_t src_y1, 
					uint16_t src_x2, uint16_t src_y2, 
					uint16_t dst_x1, uint16_t dst_y1, 
					ROT_DEG rot_degree, 
					uint16_t* org_x, uint16_t* org_y);
void s3c_g2d_set_xy_incr_format(uint32_t uDividend, uint32_t uDivisor, uint32_t* uResult);
static void s3c_g2d_rotator_start(s3c_g2d_params *params,ROT_DEG rot_degree);
void s3c_g2d_check_fifo(int empty_fifo);
int s3c_g2d_open(struct inode *inode, struct file *file);
int s3c_g2d_release(struct inode *inode, struct file *file);
int s3c_g2d_mmap(struct file* filp, struct vm_area_struct *vma) ;
static int s3c_g2d_ioctl(struct inode *inode, struct file *file, 
				unsigned int cmd, unsigned long arg);
static unsigned int s3c_g2d_poll(struct file *file, poll_table *wait); 

#endif /*_S3C_G2D_DRIVER_H_*/
