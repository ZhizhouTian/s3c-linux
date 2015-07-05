/* linux/drivers/media/video/samsung/ov9650.c
 *
 * Omnivision OV9650 CMOS Image Sensor driver
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

#include "s3c_fimc.h"
#include "ov965x.h"

#define OV965X_I2C_ADDR		0x60

const static u16 ignore[] = { I2C_CLIENT_END };
const static u16 normal_addr[] = { (OV965X_I2C_ADDR >> 1), I2C_CLIENT_END };
const static u16 *forces[] = { NULL };
static struct i2c_driver ov965x_i2c_driver;

static struct s3c_fimc_camera ov965x_data = {
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
};

static struct i2c_client_address_data addr_data = {
	.normal_i2c	= normal_addr,
	.probe		= ignore,
	.ignore		= ignore,
	.forces		= forces,
};

static void ov965x_start(struct i2c_client *client)
{
	int i;

	for (i = 0; i < OV965X_INIT_REGS; i++) 
	{
		s3c_fimc_i2c_write(client, OV965X_init_reg[i],sizeof(OV965X_init_reg[i]));
	}

}

static int ov965x_attach(struct i2c_adapter *adap, int addr, int kind)
{
	struct i2c_client *c;

	c = kmalloc(sizeof(*c), GFP_KERNEL);
	if (!c)
		return -ENOMEM;

	memset(c, 0, sizeof(struct i2c_client));	

	strcpy(c->name, "ov965x");
	c->addr = addr;
	c->adapter = adap;
	c->driver = &ov965x_i2c_driver;

	ov965x_data.client = c;

	printk("OV965X attached successfully\n");

	return i2c_attach_client(c);
}

static int ov965x_attach_adapter(struct i2c_adapter *adap)
{
	int ret = 0;
	printk("[OV965X]ov965x_attach_adapter.\n");

	s3c_fimc_register_camera(&ov965x_data);

	ret = i2c_probe(adap, &addr_data, ov965x_attach);
	if (ret) {
		err("failed to attach ov965x driver\n");
		ret = -ENODEV;
	}

	return ret;
}

static int ov965x_detach(struct i2c_client *client)
{
	i2c_detach_client(client);

	return 0;
}

static int ov965x_change_resolution(struct i2c_client *client, int res)
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

static int ov965x_change_whitebalance(struct i2c_client *client, enum s3c_fimc_wb_t type)
{
	//s3c_fimc_i2c_write(client, 0xfc, 0x0);
	//s3c_fimc_i2c_write(client, 0x30, type);

	return 0;
}

static int ov965x_command(struct i2c_client *client, u32 cmd, void *arg)
{
	switch (cmd) {
	case I2C_CAM_INIT:
		ov965x_start(client);
		info("external camera initialized\n");
		break;

	case I2C_CAM_RESOLUTION:
		ov965x_change_resolution(client, (int) arg);
		break;

	case I2C_CAM_WB:
		ov965x_change_whitebalance(client, (enum s3c_fimc_wb_t) arg);
        	break;

	default:
		err("unexpect command\n");
		break;
	}

	return 0;
}

static struct i2c_driver ov965x_i2c_driver = {
	.driver = {
		.name = "ov965x",
	},
	.id = I2C_DRIVERID_OV965X,
	.attach_adapter = ov965x_attach_adapter,
	.detach_client = ov965x_detach,
	.command = ov965x_command,
};

static __init int ov965x_init(void)
{
	return i2c_add_driver(&ov965x_i2c_driver);
}

static __init void ov965x_exit(void)
{
	i2c_del_driver(&ov965x_i2c_driver);
}

module_init(ov965x_init)
module_exit(ov965x_exit)

MODULE_AUTHOR("Figo Wang <sagres_2004@163.com>");
MODULE_DESCRIPTION("Omnivision OV9650 I2C based CMOS Image Sensor driver");
MODULE_LICENSE("GPL");


