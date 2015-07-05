#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/bcd.h>
#include <linux/list.h>
#include <linux/device.h>

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/string.h>
#include <plat/regs-gpio.h>
#include <plat/gpio-cfg.h>

#define DEBUG
#undef DEBUG

#define PEN_IRQ									IRQ_EINT(15)
#define TSC2003_IRQGPIO							S3C64XX_GPN(15)

#define DELTA_X_COORD_VARIANCE  				30
#define DELTA_Y_COORD_VARIANCE  				30

//#define TS_POLL_DELAY							1 /* ms delay between samples */
#define TS_POLL_PERIOD							1 /* ms delay between samples */

#define TSC2003_MEASURE_TEMP0					(0x0 << 4)
#define TSC2003_MEASURE_AUX						(0x2 << 4)
#define TSC2003_MEASURE_TEMP1					(0x4 << 4)
#define TSC2003_ACTIVATE_XN						(0x8 << 4)
#define TSC2003_ACTIVATE_YN						(0x9 << 4)
#define TSC2003_ACTIVATE_YP_XN					(0xa << 4)
#define TSC2003_SETUP							(0xb << 4)
#define TSC2003_MEASURE_X						(0xc << 4)
#define TSC2003_MEASURE_Y						(0xd << 4)
#define TSC2003_MEASURE_Z1						(0xe << 4)
#define TSC2003_MEASURE_Z2						(0xf << 4)

#define TSC2003_POWER_OFF_IRQ_EN				(0x0 << 2)
#define TSC2003_ADC_ON_IRQ_DIS0					(0x1 << 2)
#define TSC2003_ADC_OFF_IRQ_EN					(0x2 << 2)
#define TSC2003_ADC_ON_IRQ_DIS1					(0x3 << 2)

#define TSC2003_12BIT							(0x0 << 1)
#define TSC2003_8BIT							(0x1 << 1)

#define MAX_12BIT								((1 << 12) - 1)

#define ADC_ON_12BIT							(TSC2003_12BIT | TSC2003_ADC_ON_IRQ_DIS0)

#define READ_Y									(ADC_ON_12BIT | TSC2003_MEASURE_Y)
#define READ_Z1									(ADC_ON_12BIT | TSC2003_MEASURE_Z1)
#define READ_Z2									(ADC_ON_12BIT | TSC2003_MEASURE_Z2)
#define READ_X									(ADC_ON_12BIT | TSC2003_MEASURE_X)
#define PWRDOWN									(TSC2003_12BIT | TSC2003_POWER_OFF_IRQ_EN)
#define HIGH_SPEED								(TSC2003_12BIT | TSC2003_ADC_ON_IRQ_DIS1)

struct ts_event
{
	u16	x;
	u16	y;
	u16	z1, z2;
};

struct tsc2003 
{
	struct input_dev	*input;
	char				phys[32];
	struct work_struct	work;
	struct i2c_client	*client;
	bool				pendown;
	int					irq;
};
	

#define TSC2003_I2C_ADDR		0x48
const static u16 ignore[] = { I2C_CLIENT_END };
const static u16 normal_addr[] = { TSC2003_I2C_ADDR, I2C_CLIENT_END };
const static u16 *forces[] = { NULL };

static struct i2c_driver tsc2003_driver;

static struct i2c_client_address_data addr_data = 
{
	.normal_i2c	= normal_addr,
	.probe		= ignore,
	.ignore		= ignore,
	.forces		= forces,
};


static inline void ts_init(void)
{
	s3c_gpio_cfgpin(TSC2003_IRQGPIO,S3C64XX_GPN15_EINT15);	
}

static inline int ts_get_pendown_state(void)
{
	return gpio_get_value(TSC2003_IRQGPIO) ? 0 : 1;
}


static int tsc2003_read(struct tsc2003 * tsc, u8 cmd)
{
	s32 data;
	u16 val;

	data = i2c_smbus_read_word_data(tsc->client, cmd);
	if (data < 0) 
	{
		dev_err(&tsc->client->dev, "i2c io error: %d\n", data);
		return data;
	}

	/* The protocol and raw data format from i2c interface:
	 * S Addr Wr [A] Comm [A] S Addr Rd [A] [DataLow] A [DataHigh] NA P
	 * Where DataLow has [D11-D4], DataHigh has [D3-D0 << 4 | Dummy 4bit].
	 */
	val = swab16(data) >> 4;

	return val;
}

static inline int tsc2003_command(struct tsc2003 *tsc, u8 cmd)
{
	return i2c_master_send(tsc->client, &cmd, 1);
}

//find best point x and y
static void evaluateSample(u16 *val, u16 *sample)
{
	long   diff[3];

	if ((val[0] < MAX_12BIT) && (val[1] < MAX_12BIT) && (val[2] < MAX_12BIT)) 
	{
		// Calculate the absolute value of the differences of the sample
		diff[0] = val[0] - val[1];
		diff[1] = val[1] - val[2];
		diff[2] = val[2] - val[0];
		diff[0] = diff[0] > 0  ? diff[0] : -diff[0];
		diff[1] = diff[1] > 0  ? diff[1] : -diff[1];
		diff[2] = diff[2] > 0  ? diff[2] : -diff[2];

		// Eliminate the one away from other two and add the two
		if (diff[0] < diff[1]) 
			*sample=(unsigned short)(val[0] + ((diff[2] < diff[0]) ? val[2] : val[1]));
		else  
			*sample=(unsigned short)(val[2] + ((diff[2] < diff[1]) ? val[0] : val[1]));

		// Get the average of the two good samples
		*sample>>=1;
	}
}

static void tsc2003_read_values(struct tsc2003 *tsc, struct ts_event *tc)
{	
	u16 x[3], y[3];//,ret;
	x[0] = tsc2003_read(tsc, READ_X);
	y[0] = tsc2003_read(tsc, READ_Y);
		
	x[1] = tsc2003_read(tsc, READ_X);
	y[1] = tsc2003_read(tsc, READ_Y);
		
	x[2] = tsc2003_read(tsc, READ_X);
	y[2] = tsc2003_read(tsc, READ_Y);

	/* Prepare for next touch reading - power down ADC, enable PENIRQ */
	tsc2003_command(tsc, PWRDOWN);

	evaluateSample(x, &tc->x); 
	evaluateSample(y, &tc->y);
}

static void tsc2003_work(struct work_struct *work)
{
	struct tsc2003 *ts =container_of(work, struct tsc2003, work);
	struct ts_event tc;
	
	tsc2003_read_values(ts, &tc);

	while (likely(ts_get_pendown_state())) 
	{
		input_report_abs(ts->input, ABS_X, tc.x);
		input_report_abs(ts->input, ABS_Y, tc.y);
		input_report_key(ts->input, BTN_TOUCH, 1);
		input_report_abs(ts->input, ABS_PRESSURE, 1);
		input_sync(ts->input);
#if defined(DEBUG)		
		printk("point(%4d,%4d)\n",tc.x, tc.y);
#endif		
		if(!ts->pendown)
			ts->pendown = true;
//		schedule_work(&ts->work);	
//		msleep(TS_POLL_PERIOD);
		tsc2003_read_values(ts, &tc);
	}	

	if(ts->pendown)
	{
		input_report_key(ts->input, BTN_TOUCH, 0);
		input_report_abs(ts->input, ABS_PRESSURE, 0);
		input_sync(ts->input);
		ts->pendown = false;
#if defined(DEBUG)			
		printk("UP\n");
#endif
	}
	enable_irq(ts->irq);
	s3c_gpio_cfgpin(TSC2003_IRQGPIO, S3C64XX_GPN15_EINT15); 
}


static irqreturn_t tsc2003_irq(int irq, void *handle)
{
	struct tsc2003 *ts = handle;

	ts->pendown = false;
	s3c_gpio_cfgpin(TSC2003_IRQGPIO, S3C64XX_GPN_INPUT(15));
	disable_irq_nosync(ts->irq);	
	schedule_work(&ts->work);		

	return IRQ_HANDLED;
}


static void tsc2003_free_irq(struct tsc2003 *ts)
{
	free_irq(ts->irq, ts);
	
	if (cancel_work_sync(&ts->work)) 
	{
		enable_irq(ts->irq);
	}
}


static int  tsc2003_probe(struct i2c_client *client)
{
	struct tsc2003 *ts;
	struct input_dev *input_dev;
	int err;

	if (!i2c_check_functionality(client->adapter,I2C_FUNC_SMBUS_READ_WORD_DATA))
		return -EIO;

	ts = kzalloc(sizeof(struct tsc2003), GFP_KERNEL);

	input_dev = input_allocate_device();
	if (!ts || !input_dev) 
	{
		err = -ENOMEM;
		goto err_free_mem;
	}

	ts->client = client;
	ts->input = input_dev;
	INIT_WORK(&ts->work, tsc2003_work);
	
	ts->irq = PEN_IRQ;
	client->irq = ts->irq;

	input_dev->name = "TSC2003 Touchscreen";

	input_dev->id.bustype = BUS_I2C;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	input_set_abs_params(input_dev, ABS_X, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, MAX_12BIT, 0, 0);

	ts_init();
	err = request_irq(ts->irq, tsc2003_irq, IRQF_TRIGGER_FALLING, "tsc2003", ts);
	if (err < 0)
	{
		printk("irq %d busy?\n", ts->irq);
		goto err_free_mem;
	}

	/* Prepare for touch readings - power down ADC and enable PENIRQ */
	err = tsc2003_command(ts, PWRDOWN);
	if (err < 0)
		goto err_free_irq;

	err = input_register_device(input_dev);
	if (err)
		goto err_free_irq;

	i2c_set_clientdata(client, ts);

	return 0;

 err_free_irq:
	tsc2003_free_irq(ts);

 err_free_mem:
	input_free_device(input_dev);
	kfree(ts);
	return err;
}


static int tsc2003_detect_client(struct i2c_adapter *adapter, int address, int kind)
{
	struct i2c_client *client;
	/* Check if the adapter supports the needed features */
	if (!i2c_check_functionality(adapter,  I2C_FUNC_SMBUS_READ_BYTE|I2C_FUNC_SMBUS_WRITE_BYTE_DATA))
	{
		printk("tsc2003 detect error!\n");
		return 0;
	}
	client = kmalloc(sizeof(*client), GFP_KERNEL);
	if (!client)
		return -ENOMEM;
	memset(client, 0, sizeof(struct i2c_client));	

	strcpy(client->name, "tsc2003");
	client->addr = address;
	client->adapter = adapter;
	client->driver = &tsc2003_driver;

	if (tsc2003_probe(client))
	{
		printk("tsc2003 probe error!\n");
		return 0;
	}

	printk("tsc2003 attached successfully\n");

	return i2c_attach_client(client);
}


static int tsc2003_attach_adapter(struct i2c_adapter *adapter)
{
	int ret = 0;

	printk("[tsc2003] tsc2003_attach_adapter.\n");

	ret = i2c_probe(adapter, &addr_data, tsc2003_detect_client);
	if (ret) 
	{
		printk("failed to attach tsc2003 driver\n");
		ret = -ENODEV;
	}

	return ret;
}


static int tsc2003_detach_client(struct i2c_client *client)
{
	struct tsc2003	*ts = i2c_get_clientdata(client);

	tsc2003_free_irq(ts);

	input_unregister_device(ts->input);
	kfree(ts);

	i2c_detach_client(client);
	
	return 0;
}


static struct i2c_driver tsc2003_driver = 
{
	.driver = {.name = "tsc2003",},
	.id = I2C_DRIVERID_TSC2003,
	.attach_adapter = tsc2003_attach_adapter,
	.detach_client = tsc2003_detach_client,
};


static int __init tsc2003_init(void)
{
	return i2c_add_driver(&tsc2003_driver);
}


static void __exit tsc2003_exit(void)
{
	i2c_del_driver(&tsc2003_driver);
}


module_init(tsc2003_init);
module_exit(tsc2003_exit);


MODULE_AUTHOR("Figo Wang <sagres_2004@163.com>");
MODULE_DESCRIPTION("TSC2003 TouchScreen Driver");
MODULE_LICENSE("GPL");
