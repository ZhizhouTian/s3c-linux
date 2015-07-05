/*
 * drivers/video/samsung/s3cfb_video.c
 *
 * $Id: s3cfb_video.c,v 1.12 2009/09/13 02:13:24 
 *
 * Copyright (C) 2009 Figo Wang <sagres_2004@163.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 *	S3C Frame Buffer Driver
 *	based on skeletonfb.c, sa1100fb.h, s3c2410fb.c
 */


#ifndef _S3CFB_VIDEO_H_
#define _S3CFB_VIDEO_H_

int ch7026_init(void);

typedef struct s3cfb_video_parameter
{
	u8 *video_type;/*lcd type or vga type*/

	u32 hfp;	/* front porch */
	u32 hsw;	/* hsync width */
	u32 hbp;	/* back porch */

	u32 vfp;	/* front porch */
	u32 vsw;/* vsync width */
	u32 vbp;	/* back porch */

	u32 hres;	/* horizon pixel  x resolition */
	u32 vres;	/* line cnt       y resolution */

	u32 hres_virtual;	/* horizon pixel  x resolition */
	u32 vres_virtual;	/* line cnt       y resolution */

	u32 hres_osd;	/* horizon pixel  x resolition */
	u32 vres_osd;	/* line cnt       y resolution */

	u32 vframe_freq;	/* frame rate freq */

} s3cfb_video_parameter_t;

	



s3cfb_video_parameter_t video_parameter[] = 
{
	{
		"WX4300F",/*lcd type or vga type*/
		2,	/* front porch */
		41,	/* hsync width */
		2,	/* back porch */

		2,	/* front porch */
		10,	/* vsync width */
		2,	/* back porch */

		480,	/* horizon pixel  x resolition */
		272,	/* line cnt       y resolution */

		480,	/* horizon pixel  x resolition */
		272*2,	/* line cnt       y resolution */

		480,	/* horizon pixel  x resolition */
		272,	/* line cnt       y resolution */

		75,	/* frame rate freq */
	},

	{
		"HSD050",/*lcd type or vga type*/
		40,	/* front porch */
		48,	/* hsync width */
		40,	/* back porch */

		13,	/* front porch */
		3,	/* vsync width */
		29,	/* back porch */

		800,	/* horizon pixel  x resolition */
		480,	/* line cnt       y resolution */

	    800,	/* horizon pixel  x resolition */
 		480*2,	/* line cnt       y resolution */

		800,	/* horizon pixel  x resolition */
		480,	/* line cnt       y resolution */

     	60,	/* frame rate freq */
	},

	{
		"AT070TN83",/*lcd type or vga type*/
		40,	/* front porch */
		128,/* hsync width */
		88,	/* back porch */

		1,	/* front porch */
		3,	/* vsync width */
		0,	/* back porch */

		800,	/* horizon pixel  x resolition */
		480,	/* line cnt       y resolution */

	    800,	/* horizon pixel  x resolition */
 		480*2,	/* line cnt       y resolution */

		800,	/* horizon pixel  x resolition */
		480,	/* line cnt       y resolution */

    	60,	/* frame rate freq */
	},

	{
		"VGA800x600",/*lcd type or vga type*/

		10,	/* front porch */
		10,/* hsync width */
		20,	/* back porch */

		10,	/* front porch */
		10,	/* vsync width */
		20,	/* back porch */

		800,	/* horizon pixel  x resolition */
		600,	/* line cnt       y resolution */

	    800,	/* horizon pixel  x resolition */
 		600*2,	/* line cnt       y resolution */

		800,	/* horizon pixel  x resolition */
		600,	/* line cnt       y resolution */

    	60,	/* frame rate freq */
	},
};

#endif
