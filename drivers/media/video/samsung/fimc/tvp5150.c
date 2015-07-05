/* linux/drivers/media/video/samsung/fimc/tvp5150.c
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/i2c-id.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <asm/io.h>

#include "s3c_fimc.h"
#include "tvp5150.h"
#include "tvp5150_reg.h"

#define TVP5150_I2C_ADDR		0xB8		//I2CCSEL下拉:0xB8 上拉:0xBA

const static u16 ignore[] = { I2C_CLIENT_END };
const static u16 normal_addr[] = { (TVP5150_I2C_ADDR >> 1), I2C_CLIENT_END };
const static u16 *forces[] = { NULL };
static struct i2c_driver tvp5150_i2c_driver;

static struct s3c_fimc_camera tvp5150_data = {
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
};

static struct i2c_client_address_data addr_data = {
	.normal_i2c	= normal_addr,
	.probe		= ignore,
	.ignore		= ignore,
	.forces		= forces,
};

static void tvp5150_start(struct i2c_client *client)
{
	int i;

	for (i = 0; i < tvp5150_INIT_REGS; i++) 
	{
		s3c_fimc_i2c_write(client, tvp5150_init_reg[i],sizeof(tvp5150_init_reg[i]));
	}

}

#define TVP5150_MSB_DEV_ID          0x80 /* MSB of device ID */
#define TVP5150_LSB_DEV_ID          0x81 /* LSB of device ID */
#define TVP5150_ROM_MAJOR_VER       0x82 /* ROM major version */
#define TVP5150_ROM_MINOR_VER       0x83 /* ROM minor version */
static int tvp5150_id_detect(struct i2c_client *client)
{
	u8 msb_id, lsb_id, msb_rom, lsb_rom;

	msb_id=s3c_fimc_i2c_read(client,TVP5150_MSB_DEV_ID);
	lsb_id=s3c_fimc_i2c_read(client,TVP5150_LSB_DEV_ID);
	msb_rom=s3c_fimc_i2c_read(client,TVP5150_ROM_MAJOR_VER);
	lsb_rom=s3c_fimc_i2c_read(client,TVP5150_ROM_MINOR_VER);

	/* Is TVP5150AM1 */
	if ((msb_rom==4)&&(lsb_rom==0)) 
	{ 
		printk("tvp%02x%02xam1 detected.\n",msb_id, lsb_id);
		return 1;
	} 
	else 
	{
		/* Is TVP5150A */
		if ((msb_rom==3)||(lsb_rom==0x21)) 
		{ 
			printk("tvp%02x%02xa detected.\n",msb_id, lsb_id);
		} else 
		{
			printk("*** unknown tvp%02x%02x chip detected.\n",msb_id,lsb_id);
			printk("*** Rom ver is %d.%d\n",msb_rom,lsb_rom);
		}
		return 0;
	}
}

static int tvp5150_detect_client(struct i2c_adapter *adapter,
				 int address, int kind)
{
	struct i2c_client *c;

	/* Check if the adapter supports the needed features */
	if (!i2c_check_functionality
	    (adapter,
	     I2C_FUNC_SMBUS_READ_BYTE | I2C_FUNC_SMBUS_WRITE_BYTE_DATA))
		{
			printk("tvp5150 detect error!\n");
			return 0;
		}


	c = kmalloc(sizeof(*c), GFP_KERNEL);
	if (!c)
		return -ENOMEM;

	memset(c, 0, sizeof(struct i2c_client));	

	strcpy(c->name, "tvp5150");
	c->addr = address;
	c->adapter = adapter;
	c->driver = &tvp5150_i2c_driver;

	tvp5150_data.client = c;

	if (!tvp5150_id_detect(c))
	{
		printk("tvp5150 detect error!\n");
		return 0;
	}

	printk("tvp5150 attached successfully\n");

	return i2c_attach_client(c);
}

static int tvp5150_attach_adapter(struct i2c_adapter *adapter)
{
	int ret = 0;

	printk("[tvp5150]tvp5150_attach_adapter.\n");

	s3c_fimc_register_camera(&tvp5150_data);

	ret = i2c_probe(adapter, &addr_data, tvp5150_detect_client);
	if (ret) {
		err("failed to attach tvp5150 driver\n");
		ret = -ENODEV;
	}

	return ret;
}


static int tvp5150_detach_client(struct i2c_client *client)
{
	i2c_detach_client(client);
	return 0;
}

static int tvp5150_change_resolution(struct i2c_client *client, int res)
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

static int tvp5150_change_whitebalance(struct i2c_client *client, enum s3c_fimc_wb_t type)
{
	//s3c_fimc_i2c_write(client, 0xfc, 0x0);
	//s3c_fimc_i2c_write(client, 0x30, type);

	return 0;
}

static int tvp5150_command(struct i2c_client *client, u32 cmd, void *arg)
{
	switch (cmd) {

	case I2C_CAM_INIT:
		/* Initializes TVP5150 to stream enabled values */
		tvp5150_start(client);
		info("external camera initialized\n");

		break;

	case I2C_CAM_RESOLUTION:
		tvp5150_change_resolution(client, (int) arg);
		break;

	case I2C_CAM_WB:
		tvp5150_change_whitebalance(client, (enum s3c_fimc_wb_t) arg);
        break;

	default:
		err("unexpect command\n");
		break;
	}

	return 0;
}

/* ----------------------------------------------------------------------- */

static struct i2c_driver tvp5150_i2c_driver = {
	.driver = {
		.name = "tvp5150",
	},
	.id = I2C_DRIVERID_TVP5150,
	.attach_adapter = tvp5150_attach_adapter,
	.detach_client = tvp5150_detach_client,
	.command = tvp5150_command,
};

static int __init tvp5150_init(void)
{
	return i2c_add_driver(&tvp5150_i2c_driver);
}

static void __exit tvp5150_exit(void)
{
	i2c_del_driver(&tvp5150_i2c_driver);
}

module_init(tvp5150_init)
module_exit(tvp5150_exit)


MODULE_AUTHOR("Figo Wang <sagres_2004@163.com>");
MODULE_DESCRIPTION("Texas Instruments TVP5150A video decoder driver");
MODULE_LICENSE("GPL");
