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

#include <linux/wait.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/platform_device.h>

#include <plat/regs-gpio.h>
#include <plat/regs-lcd.h>

#include <linux/i2c.h>

#include "s3cfb.h"
#include "s3cfb_video.h"

static u8 i;

#define S3CFB_HFP				video_parameter[i].hfp	/* front porch */
#define S3CFB_HSW				video_parameter[i].hsw	/* hsync width */
#define S3CFB_HBP				video_parameter[i].hbp	/* back porch */

#define S3CFB_VFP				video_parameter[i].vfp	/* front porch */
#define S3CFB_VSW				video_parameter[i].vsw	/* vsync width */
#define S3CFB_VBP				video_parameter[i].vbp	/* back porch */

#define S3CFB_HRES				video_parameter[i].hres	/* horizon pixel  x resolition */
#define S3CFB_VRES				video_parameter[i].vres	/* line cnt       y resolution */

#define S3CFB_HRES_VIRTUAL		video_parameter[i].hres_virtual	/* horizon pixel  x resolition */
#define S3CFB_VRES_VIRTUAL		video_parameter[i].vres_virtual	/* line cnt       y resolution */

#define S3CFB_HRES_OSD			video_parameter[i].hres_osd	/* horizon pixel  x resolition */
#define S3CFB_VRES_OSD			video_parameter[i].vres_osd	/* line cnt       y resolution */

#define S3CFB_VFRAME_FREQ     	video_parameter[i].vframe_freq	/* frame rate freq */

#define S3CFB_PIXEL_CLOCK	(S3CFB_VFRAME_FREQ * (S3CFB_HFP + S3CFB_HSW + S3CFB_HBP + S3CFB_HRES) * (S3CFB_VFP + S3CFB_VSW + S3CFB_VBP + S3CFB_VRES))

static void s3cfb_set_fimd_info(void)
{
	s3cfb_fimd.vidcon1 = S3C_VIDCON1_IHSYNC_INVERT | S3C_VIDCON1_IVSYNC_INVERT | S3C_VIDCON1_IVDEN_NORMAL;
	s3cfb_fimd.vidtcon0 = S3C_VIDTCON0_VBPD(S3CFB_VBP - 1) | S3C_VIDTCON0_VFPD(S3CFB_VFP - 1) | S3C_VIDTCON0_VSPW(S3CFB_VSW - 1);
	s3cfb_fimd.vidtcon1 = S3C_VIDTCON1_HBPD(S3CFB_HBP - 1) | S3C_VIDTCON1_HFPD(S3CFB_HFP - 1) | S3C_VIDTCON1_HSPW(S3CFB_HSW - 1);
	s3cfb_fimd.vidtcon2 = S3C_VIDTCON2_LINEVAL(S3CFB_VRES - 1) | S3C_VIDTCON2_HOZVAL(S3CFB_HRES - 1);

	s3cfb_fimd.vidosd0a = S3C_VIDOSDxA_OSD_LTX_F(0) | S3C_VIDOSDxA_OSD_LTY_F(0);
	s3cfb_fimd.vidosd0b = S3C_VIDOSDxB_OSD_RBX_F(S3CFB_HRES - 1) | S3C_VIDOSDxB_OSD_RBY_F(S3CFB_VRES - 1);

	s3cfb_fimd.vidosd1a = S3C_VIDOSDxA_OSD_LTX_F(0) | S3C_VIDOSDxA_OSD_LTY_F(0);
	s3cfb_fimd.vidosd1b = S3C_VIDOSDxB_OSD_RBX_F(S3CFB_HRES_OSD - 1) | S3C_VIDOSDxB_OSD_RBY_F(S3CFB_VRES_OSD - 1);

	s3cfb_fimd.width = S3CFB_HRES;
	s3cfb_fimd.height = S3CFB_VRES;
	s3cfb_fimd.xres = S3CFB_HRES;
	s3cfb_fimd.yres = S3CFB_VRES;

#if defined(CONFIG_FB_S3C_VIRTUAL_SCREEN)
	s3cfb_fimd.xres_virtual = S3CFB_HRES_VIRTUAL;
	s3cfb_fimd.yres_virtual = S3CFB_VRES_VIRTUAL;
#else
	s3cfb_fimd.xres_virtual = S3CFB_HRES;
	s3cfb_fimd.yres_virtual = S3CFB_VRES;
#endif

	s3cfb_fimd.osd_width = S3CFB_HRES_OSD;
	s3cfb_fimd.osd_height = S3CFB_VRES_OSD;
	s3cfb_fimd.osd_xres = S3CFB_HRES_OSD;
	s3cfb_fimd.osd_yres = S3CFB_VRES_OSD;

	s3cfb_fimd.osd_xres_virtual = S3CFB_HRES_OSD;
	s3cfb_fimd.osd_yres_virtual = S3CFB_VRES_OSD;

	s3cfb_fimd.pixclock = S3CFB_PIXEL_CLOCK;

	s3cfb_fimd.hsync_len = S3CFB_HSW;
	s3cfb_fimd.vsync_len = S3CFB_VSW;
	s3cfb_fimd.left_margin = S3CFB_HFP;
	s3cfb_fimd.upper_margin = S3CFB_VFP;
	s3cfb_fimd.right_margin = S3CFB_HBP;
	s3cfb_fimd.lower_margin = S3CFB_VBP;
}

void s3cfb_init_hw(void)
{
	char *option = NULL;

	fb_get_options("fb",&option);
	if(!option || !*option)
	{
		printk("video output type detect error!!\n");
		return;
	}

	for(i=0; i<sizeof(video_parameter)/sizeof(video_parameter[0]); i++)
	{
		if(!strncmp(video_parameter[i].video_type, option, 10))
		{
			break;
		}
	}
	if(i == sizeof(video_parameter)/sizeof(video_parameter[0]))
	{
		printk("video output type detect error!!\n");
		return;
	}

	printk(KERN_INFO "LCD TYPE :: %s will be initialized\n",option);

	s3cfb_set_fimd_info();
	s3cfb_set_gpio();

	if(!strncmp("VGA", option, 3))
	{
		ch7026_init();
	}
}

