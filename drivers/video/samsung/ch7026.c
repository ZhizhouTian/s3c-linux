/*
 * drivers/video/samsung/ch7026.c
 *
 * $Id: ch7026.c,v 1.12 2009/09/13 02:13:24 
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

#include "ch7026.h"

#define CH7026_I2C_ADDR		(0x76<<1)		//AS下拉:0x76 上拉:0x75

const static u16 ignore[] = { I2C_CLIENT_END };
const static u16 normal_addr[] = { (CH7026_I2C_ADDR >> 1), I2C_CLIENT_END };
const static u16 *forces[] = { NULL };
static struct i2c_driver ch7026_i2c_driver;

static struct i2c_client_address_data addr_data = {
	.normal_i2c	= normal_addr,
	.probe		= ignore,
	.ignore		= ignore,
	.forces		= forces,
};


static void ch7026_start(struct i2c_client *client)
{
	int i;

	for (i = 0; i <CH7026_INIT_REGS; i++) 
	{
		i2c_master_send(client, ch7026_init_reg[i],sizeof(ch7026_init_reg[i]));
	}
	printk("ch7026 init ok\n");
}

#define ID_ADDR		0x00
static int ch7026_id_detect(struct i2c_client *client)
{
	unsigned char buffer[1];
	int rc;
	buffer[0] = ID_ADDR;
	if (1 != (rc = i2c_master_send(client, buffer, 1)))
		printk("i2c i/o error: rc == %d (should be 1)\n", rc);

	msleep(10);

	if (1 != (rc = i2c_master_recv(client, buffer, 1)))
		printk("i2c i/o error: rc == %d (should be 1)\n", rc);

	if (buffer[0] != 0x54)
	{
		printk("*** unknown chip detected.\n");
	   return -ENODEV;
	}
	else
	{
		printk("ch7026 detected.\n");
		return 0;
	}
}


static int ch7026_detect_client(struct i2c_adapter *adapter,
				 int address, int kind)
{
	struct i2c_client *c;

	/* Check if the adapter supports the needed features */
	if (!i2c_check_functionality
	    (adapter,
	     I2C_FUNC_SMBUS_READ_BYTE | I2C_FUNC_SMBUS_WRITE_BYTE_DATA))
		{
			printk("ch7026 detect error!\n");
			return 0;
		}

	c = kmalloc(sizeof(*c), GFP_KERNEL);
	if (!c)
		return -ENOMEM;

	memset(c, 0, sizeof(struct i2c_client));	

	strcpy(c->name, "ch7026");
	c->addr = address;
	c->adapter = adapter;
	c->driver = &ch7026_i2c_driver;

	if (ch7026_id_detect(c))
	{
		printk("ch7026 detect error!\n");
		return 0;
	}

	printk("ch7026 attached successfully\n");

	ch7026_start(c);

	return i2c_attach_client(c);
}

static int ch7026_attach_adapter(struct i2c_adapter *adapter)
{
	int ret = 0;

	printk("[ch7026] ch7026_attach_adapter.\n");

	ret = i2c_probe(adapter, &addr_data, ch7026_detect_client);
	if (ret) 
	{
		printk("failed to attach ch7026 driver\n");
		ret = -ENODEV;
	}

	return ret;
}

static int ch7026_detach_client(struct i2c_client *client)
{
	i2c_detach_client(client);
	return 0;
}

static struct i2c_driver ch7026_i2c_driver = {
	.driver = {
		.name = "ch7026",
	},
	.id = I2C_DRIVERID_CH7026,
	.attach_adapter = ch7026_attach_adapter,
	.detach_client = ch7026_detach_client,
};

#if 0
static int __init ch7026_init(void)
{
	return i2c_add_driver(&ch7026_i2c_driver);
}

static void __exit ch7026_exit(void)
{
	i2c_del_driver(&ch7026_i2c_driver);
}

module_init(ch7026_init);
module_exit(ch7026_exit);

MODULE_AUTHOR("Figo Wang <sagres_2004@163.com>");
MODULE_DESCRIPTION("Texas Instruments CH7026 driver");
MODULE_LICENSE("GPL");

#else
int ch7026_init(void)
{
	return i2c_add_driver(&ch7026_i2c_driver);
}
#endif
