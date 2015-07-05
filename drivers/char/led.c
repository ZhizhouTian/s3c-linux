/*
 * =====================================================================================
 *
 *       Filename:  led.c
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  11/30/2010 07:42:29 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Gavin.S (), zdshuwei@gmail.com
 *        Company:  Zhejiang University
 *
 * =====================================================================================
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/device.h>

#include <plat/regs-gpio.h>
#include <plat/s3c6410.h>
#include <plat/gpio-cfg.h>
#include <asm/gpio.h>
#include <plat/gpio-bank-l.h>
#include <plat/gpio-bank-n.h>

#define DEVICE_NAME		"leds"
#define LED_MAJOR		259

#define ON		0
#define OFF		1

/*  注册设备 */
struct leds_dev
{
	struct cdev cdev;
	unsigned char value;
};

/*  指定LED引脚 */
static unsigned long led_table [] =
{
	S3C64XX_GPL(13),
	S3C64XX_GPN(6),
};

/*  GPIO输出功能 */
static unsigned int led_cfg_table [] =
{
	S3C64XX_GPL_OUTPUT(13),
	S3C64XX_GPN_OUTPUT(6),
};

static int leds_open(struct inode *inode, struct file *file)
{
	int i;

	for (i = 0; i < 2; i++)
	{
		s3c_gpio_cfgpin(led_table[i], led_cfg_table[i]);
	}

	return 0;
}

static int leds_ioctl(struct inode *inode, struct file *file,
		unsigned int cmd, unsigned long arg)
{
	if (arg > 4)
	{
		return -EINVAL;
	}

	switch(cmd)
	{
		case ON:
			s3c_gpio_setpin(led_table[arg], 0);
			return 0;
		case OFF:
			s3c_gpio_setpin(led_table[arg], 1);
			return 0;
			
		default:
			return -EINVAL;
	}
}

static struct file_operations s3c6410_leds_fops = 
{
	.owner	=	THIS_MODULE,
	.open	=	leds_open,
	.ioctl	=	leds_ioctl,
};

static void leds_setup_cdev(struct leds_dev *dev)
{
	int err;
	int devno = MKDEV(LED_MAJOR, 0);

	cdev_init(&dev->cdev, &s3c6410_leds_fops);
	dev->cdev.owner = THIS_MODULE;
	dev->cdev.ops = &s3c6410_leds_fops;
	err = cdev_add(&dev->cdev, devno, 1);
	if (err)
		printk(KERN_NOTICE "Error %d adding LED", err);
}

static int __init leds_init(void)
{
	int ret;

	ret = register_chrdev(LED_MAJOR, DEVICE_NAME, &s3c6410_leds_fops);
	if (ret < 0)
	{
		printk(DEVICE_NAME "can't register major number.\n");
		return ret;
	}
	
//	struct leds_dev dev;
//	leds_setup_cdev(&dev);

	struct class *my_class = class_create(THIS_MODULE, "my_device_driver");
	device_create(my_class, NULL, MKDEV(LED_MAJOR, 0), NULL, "leds");

	printk(DEVICE_NAME "initialized.\n");
	return 0;
}

static void __exit leds_exit(void)
{
	unregister_chrdev(LED_MAJOR, DEVICE_NAME);
}


module_init(leds_init);
module_exit(leds_exit);

MODULE_AUTHOR("Gavin.S");
MODULE_DESCRIPTION("s3c6410 led driver test.");
MODULE_LICENSE("GPL");
