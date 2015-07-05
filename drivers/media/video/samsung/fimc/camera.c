/* linux/drivers/media/video/samsung/camera.c
 *
 * real6410 camera driver
 *
 * Figo Wang, Copyright (c) 2010
 * 	sagres_2004@163.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/i2c-id.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/init.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include "s3c_fimc.h"
#include "camera.h"

#define OV965X_I2C_ADDR		0x60
#define OV3640_I2C_ADDR		0x78
#define TVP5150_I2C_ADDR	0xB8		//I2CCSEL下拉:0xB8 上拉:0xBA
const static u16 ignore[] = { I2C_CLIENT_END };
const static u16 *forces[] = { NULL };
const static u16 normal_addr[][2] = 
{
	{(OV965X_I2C_ADDR>>1),I2C_CLIENT_END},
	{(OV3640_I2C_ADDR >> 1),I2C_CLIENT_END},
	{(TVP5150_I2C_ADDR>>1),I2C_CLIENT_END},
};
static struct i2c_driver camera_i2c_driver;
static u32 camera_atach_status = 0;

CAMERA_PARAMETER_t camera_parameter[]=
{
	{//OV965X
		.reset_type = 1,
		.camera_data = 
		{
			.id 		= CONFIG_VIDEO_FIMC_CAM_CH,
			.type		= CAM_TYPE_ITU,
			.mode		= ITU_601_YCBCR422_8BIT,
			.order422	= CAM_ORDER422_8BIT_YCBYCR,
			.clockrate	= 24000000,
			.width		= 640,
			.height		= 480,
			.offset		= {
				.h1 = 0,
				.h2 = 0,
				.v1 = 0,
				.v2 = 0,
			},

			.polarity	= {
				.pclk	= 0,
				.vsync	= 1,
				.href	= 0,
				.hsync	= 0,
			},

			.initialized	= 0,
		},
		.addr_data =
		{
			.normal_i2c	= normal_addr[0],
			.probe		= ignore,
			.ignore		= ignore,
			.forces		= forces,
		},
	},
	{//OV3640
		.reset_type = 0,
		.camera_data = 
		{
			.id 		= CONFIG_VIDEO_FIMC_CAM_CH,
			.type		= CAM_TYPE_ITU,
			.mode		= ITU_601_YCBCR422_8BIT,
			.order422	= CAM_ORDER422_8BIT_YCBYCR,
			.clockrate	= 24000000,
			.width		= 640,
			.height		= 480,
			.offset		= {
				.h1 = 0,
				.h2 = 0,
				.v1 = 0,
				.v2 = 0,
			},

			.polarity	= {
				.pclk	= 0,
				.vsync	= 0,
				.href	= 0,
				.hsync	= 0,
			},

			.initialized	= 0,
		},
		.addr_data =
		{
			.normal_i2c	= normal_addr[1],
			.probe		= ignore,
			.ignore		= ignore,
			.forces		= forces,
		},
	},
	{//TVP5150
		.reset_type = 0,
		.camera_data = 
		{
			.id 		= CONFIG_VIDEO_FIMC_CAM_CH,
			.type		= CAM_TYPE_ITU,
			.mode		= ITU_601_YCBCR422_8BIT,
			.order422	= CAM_ORDER422_8BIT_CBYCRY,
			.clockrate	= 27000000,
			.width		= 720,
			.height		= 288,
			.offset		= {
				.h1 = 0,
				.h2 = 0,
				.v1 = 17,
				.v2 = 0,
			},

			.polarity	= {
				.pclk	= 1,
				.vsync	= 0,
				.href	= 0,
				.hsync	= 0,
			},

			.initialized	= 0,
		},
		.addr_data =
		{
			.normal_i2c	= normal_addr[2],
			.probe		= ignore,
			.ignore		= ignore,
			.forces		= forces,
		},
	},
};


static void camera_start(struct i2c_client *client)
{
	int i;

	if(client->addr == (OV965X_I2C_ADDR>>1))
	{
		for (i = 0; i < OV965X_INIT_REGS; i++) 
		{
			s3c_fimc_i2c_write(client, OV965X_init_reg[i],sizeof(OV965X_init_reg[i]));
		}
	}
	else if(client->addr == (OV3640_I2C_ADDR>>1))
	{
		for (i = 0; i < sizeof(ov3640_setting_15fps_VGA_640_480) / sizeof(ov3640_setting_15fps_VGA_640_480[0]); i++) {
			s3c_fimc_i2c_write(client, ov3640_setting_15fps_VGA_640_480[i],sizeof(ov3640_setting_15fps_VGA_640_480[i]));
		}
	}
	else if(client->addr == (TVP5150_I2C_ADDR>>1))
	{
		for (i = 0; i < tvp5150_INIT_REGS; i++) 
		{
			s3c_fimc_i2c_write(client, tvp5150_init_reg[i],sizeof(tvp5150_init_reg[i]));
		}
	}
}

static int camera_attach(struct i2c_adapter *adap, int addr, int kind)
{
	struct i2c_client *c;

	c = kmalloc(sizeof(*c), GFP_KERNEL);
	if (!c)
		return -ENOMEM;

	memset(c, 0, sizeof(struct i2c_client));	

	strcpy(c->name, "camera");
	c->addr = addr;
	c->adapter = adap;
	c->driver = &camera_i2c_driver;

	if(addr == (OV965X_I2C_ADDR>>1))
	{
		camera_parameter[0].camera_data.client = c;
		printk("CAMERA OV965X attached successfully\n");
	}
	else if(addr == (OV3640_I2C_ADDR>>1))
	{
		camera_parameter[1].camera_data.client = c;
		printk("CAMERA OV3640 attached successfully\n");
	}
	else if(addr == (TVP5150_I2C_ADDR>>1))
	{
		camera_parameter[2].camera_data.client = c;
		printk("CAMERA TVP5150 attached successfully\n");
	}

	camera_atach_status = 1;
	return i2c_attach_client(c);
}

static int camera_attach_adapter(struct i2c_adapter *adap)
{
	int ret = 0;
	int i;
	printk("[CAMERA]camera_attach_adapter.\n");
	for(i=0;i<3;i++)
	{
		if(camera_atach_status == 0)
		{
			s3c_fimc_register_camera(&camera_parameter[i].camera_data);
			s3c_fimc_reset_camera(camera_parameter[i].reset_type);
			ret = i2c_probe(adap, &camera_parameter[i].addr_data, camera_attach);
			if (ret) {
				err("failed to attach camera driver\n");
				ret = -ENODEV;
			}	
		}
	}
	if(camera_atach_status == 0)
	{
		printk("camera is not found!!!!!\n");
	}
	return ret;
}

static int camera_detach(struct i2c_client *client)
{
	i2c_detach_client(client);
	return 0;
}

static int camera_change_resolution(struct i2c_client *client, int res)
{
	switch (res) {
	case CAM_RES_DEFAULT:	/* fall through */
	case CAM_RES_MAX:	/* fall through */
		break;

	default:
		err("unexpect value\n");
	}

	return 0;
}

static int camera_change_whitebalance(struct i2c_client *client, enum s3c_fimc_wb_t type)
{
	//s3c_fimc_i2c_write(client, 0xfc, 0x0);
	//s3c_fimc_i2c_write(client, 0x30, type);

	return 0;
}

static int camera_command(struct i2c_client *client, u32 cmd, void *arg)
{
	switch (cmd) {
	case I2C_CAM_INIT:
		camera_start(client);
		info("external camera initialized\n");
		break;

	case I2C_CAM_RESOLUTION:
		camera_change_resolution(client, (int) arg);
		break;

	case I2C_CAM_WB:
		camera_change_whitebalance(client, (enum s3c_fimc_wb_t) arg);
        break;

	default:
		err("unexpect command\n");
		break;
	}

	return 0;
}

static struct i2c_driver camera_i2c_driver = {
	.driver = {
		.name = "camera",
	},
	.id = I2C_DRIVERID_CAMERA,
	.attach_adapter = camera_attach_adapter,
	.detach_client = camera_detach,
	.command = camera_command,
};

static __init int camera_init(void)
{
	return i2c_add_driver(&camera_i2c_driver);
}

static __init void camera_exit(void)
{
	i2c_del_driver(&camera_i2c_driver);
}

module_init(camera_init)
module_exit(camera_exit)

MODULE_AUTHOR("Figo Wang <sagres_2004@163.com>");
MODULE_DESCRIPTION("camera driver");
MODULE_LICENSE("GPL");
