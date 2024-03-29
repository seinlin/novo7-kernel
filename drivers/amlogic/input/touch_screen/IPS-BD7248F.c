/* 
 * drivers/input/touchscreen/ldwzic_ts.c
 *
 * FocalTech ldwzic TouchScreen driver. 
 *
 * Copyright (c) 2010  Ingenic Semiconductor Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 *	note: only support mulititouch	liaoqizhen 2010-09-01
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/earlysuspend.h>

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/fcntl.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/input-polldev.h>
#include <linux/workqueue.h>



#include <mach/hardware.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/hardware/gic.h>

#include <mach/gpio.h>
#include <mach/irqs.h>                         
#include <media/soc_camera.h>                              
#include <mach/sram.h>

#include "IPS-BD7248F.h"
#define TS_IRQ_GPIO  ((GPIOA_bank_bit0_27(16)<<16) | GPIOA_bit_bit0_27(16))
#define TS_RESET_GPIO  ((GPIOD_bank_bit0_9(5)<<16) | GPIOD_bit_bit0_9(5))


//#undef CONFIG_LINDAWZ_DEBUG
//#define CONFIG_LINDAWZ_DEBUG
//#define dev_dbg(dev, format, arg...)		\
//	dev_printk(KERN_DEBUG , dev , format , ## arg)

 #define CONFIG_LINDAWZ_MULTITOUCH
// #define CONFIG_TOUCH_PANEL_KEY

//#define PENUP_TIMEOUT_TIMER 1 //open by Koffu

#define LDWZIC_IIC_SPEED 100*1000
#define POINT_READ_LEN 9
#define REG_SUM  14
//#define CONFIG_TOUCHSCREEN_X_FLIP
//#define CONFIG_TOUCHSCREEN_Y_FLIP


static struct i2c_client *this_client;
struct i2c_client *infer_ldtp_i2c_client;

struct ts_event {
	u16	x1;
	u16	y1;
	u16	x2;
	u16	y2;
	u16 pen_up;
	u16	pressure;
    u8  touch_point;
};
//struct status {
static	u8	flag;
static	u8	value;
//};
//static struct status icon_status;

struct ldwzic_ts_data {
	struct i2c_client *client;
	struct input_dev	*input_dev;
	struct ts_event		event;
	struct work_struct 	pen_event_work;
	struct workqueue_struct *ts_workqueue;
	struct early_suspend	early_suspend;
	int 		irq;
#ifdef PENUP_TIMEOUT_TIMER
		struct timer_list penup_timeout_timer;
#endif
#ifdef CONFIG_LINDAWZ_DEBUG
	long int count;
#endif
};

/* set  update_status    start*/
static struct class *update_status_class;
extern int update_status;
static ssize_t get_update_status_flag(struct class* class, struct class_attribute* attr, char* buf)
{
    ssize_t ret = 0;

    ret = sprintf(buf, "%d\n", update_status);

    return ret;
}

static ssize_t set_update_status_flag(struct class* class, struct class_attribute* attr, const char* buf, size_t count)
{
    u32 reg;

    switch(buf[0]) {
        case '0':
            update_status=0;
            break;

        case '1':
            update_status=1;
            break;

        case '2':
            update_status=2;
            break;

        default:
            printk("unknow command!\n");
    }

    return count;
}

static struct class_attribute update_status_attrs[]={
  __ATTR(update_status, S_IRUGO | S_IWUSR, get_update_status_flag, set_update_status_flag),
  __ATTR_NULL
};

static void create_update_status_attrs(struct class* class)
{
  int i=0;
  for(i=0; update_status_attrs[i].attr.name; i++){
    class_create_file(class, &update_status_attrs[i]);
  }
}

static void remove_update_status_attrs(struct class* class)
{
  int i=0;
  for(i=0; update_status_attrs[i].attr.name; i++){
    class_remove_file(class, &update_status_attrs[i]);
  }
}
/* set  update_status end */

static int ldwzic_i2c_txdata(char *txdata, int length)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txdata,
		},
	};

   	msleep(1);
	ret = i2c_transfer(this_client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);

	return ret;
}


//write 0 then read 
static int ldwzic_get_event(struct i2c_client *client, char *rxdata, int length)
{
	int ret;
        u8 address = 0;
        
	struct i2c_msg msgs[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= &address,
		},
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxdata,
		},
	};

    //msleep(1);
	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);
	
	return ret;
}

static int ldwzic_set_reg(u8 addr, u8 para)
{
    u8 buf[3];
    int ret = -1;

    buf[0] = addr;
    buf[1] = para;
    ret = ldwzic_i2c_txdata(buf, 2);
    if (ret < 0) {
        pr_err("write reg failed! %#x ret: %d", buf[0], ret);
        return -1;
    }
    
    return 0;
}

static void ldwzic_ts_release(struct ldwzic_ts_data *ldwzic_ts)
{
#ifdef CONFIG_LINDAWZ_MULTITOUCH	
	input_report_key(ldwzic_ts->input_dev, BTN_TOUCH, 0);
	input_report_abs(ldwzic_ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
#else
	input_report_abs(ldwzic_ts->input_dev, ABS_PRESSURE, 0);
	input_report_key(ldwzic_ts->input_dev, BTN_TOUCH, 0);
#endif
	input_sync(ldwzic_ts->input_dev);
#ifdef PENUP_TIMEOUT_TIMER
	del_timer(&(ldwzic_ts->penup_timeout_timer));
#endif
	//printk("=ldwzic_ts_release==\n");
}

static void ldwzic_chip_reset(void)
{
	printk("Ldwzic Chips Reset !\n");
}

static unsigned long transform_to_screen_x(unsigned long x )
{
        //x = TP_MAX_X - x;
/*	if (x < TP_MIN_X) 
		x = TP_MIN_X;
	if (x > TP_MAX_X) 
		x = TP_MAX_X;
         x = TP_MAX_X - x;
	return (x - TP_MIN_X) * SCREEN_MAX_X / (TP_MAX_X - TP_MIN_X);*/
	
	x = 2700 - x;

        if (x < TP_MIN_X) 
		x = TP_MIN_X;
	if (x > TP_MAX_X) 
		x = TP_MAX_X;
	return (x - TP_MIN_X) * SCREEN_MAX_X / (TP_MAX_X - TP_MIN_X);
}

static unsigned long transform_to_screen_y(unsigned long y)
{
   /* 	
	if (y < TP_MIN_Y) 
		y = TP_MIN_Y;
   //             y = TP_MIN_Y - 50;
	if (y > TP_MAX_Y)
	{
	    y = TP_MAX_Y;
 //           y = TP_MAX_Y +50;

		}
	y = TP_MAX_Y -y;
	return (y - TP_MIN_Y) * SCREEN_MAX_Y / (TP_MAX_Y - TP_MIN_Y);*/
	
#ifndef CONFIG_IPS_BD7248F_800x480_TOUCHSCREEN
 	y = 1500 - y;
#endif
	
        if (y < TP_MIN_Y) 
		y = TP_MIN_Y;
	if (y > TP_MAX_Y)
	{
	    y = TP_MAX_Y;
         }
	return (y - TP_MIN_Y) * SCREEN_MAX_Y / (TP_MAX_Y - TP_MIN_Y);	
}

static int ldwzic_read_data(void)
{
	struct ldwzic_ts_data *ldwzic_ts = i2c_get_clientdata(this_client);
	struct ts_event *event = &ldwzic_ts->event;
#ifdef CONFIG_LINDAWZ_DEBUG
	static u16 max_x=0,max_y=0,min_x=TP_MAX_X,min_y=TP_MAX_Y;
#endif
	u8 buf[REG_SUM];
	int ret = -1;

#if 0
	buf[0] = 9;
	ret = ldwzic_i2c_rxdata(buf, 4);
	printk("\nvtp =%02X,%02X,%02X,%02X-",buf[0],buf[0x1],buf[0x2],buf[0x3]);

	buf[0] = 0xd;
	ret = ldwzic_i2c_rxdata(buf, 1);
	printk("\nopmode =%02X-",buf[0]);
#endif
	buf[0] = 0;// read from reg 0
	ret = ldwzic_get_event(this_client, buf, POINT_READ_LEN);

    if (ret < 0) {
		ldwzic_chip_reset();
		printk("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
	}
#ifdef CONFIG_LINDAWZ_DEBUG
	printk("\nstate =%02X-",buf[0]);
	printk("\nreg 1,2,3,4 =%02X,%02X,%02X,%02X-",buf[1],buf[2],buf[3],buf[4]);
	printk("\nreg 5,6,7,8 =%02X,%02X,%02X,%02X-",buf[5],buf[6],buf[7],buf[8]);
	printk("\nreg 9,a,b,c =%02X,%02X,%02X,%02X-",buf[9],buf[0xa],buf[0xb],buf[0xc]);
	printk("\nreg d =%02X-\n",buf[0xd]);
	printk("\nmax x,y=(%d,%d),min x,y=(%d,%d)-\n",max_x,max_y,min_x,min_y);
#endif
	memset(event, 0, sizeof(struct ts_event));
	event->touch_point = buf[0] & REG_STATE_MASK;
//	printk(">>>>>Ldwzic %4d Point In Read Data!\n",event->touch_point);
	if ((event->touch_point == 0)) {
		ldwzic_ts_release(ldwzic_ts);
		return 1; 
	}
#ifdef CONFIG_LINDAWZ_MULTITOUCH
    switch (event->touch_point) {
		case 2:
			event->x1 = ((((u16)buf[5])<<8)&0x0f00) |buf[6];
			event->y1 = ((((u16)buf[7])<<8)&0x0f00) |buf[8];
			event->touch_point = 1;// ONE POINT
//#ifdef CONFIG_LINDAWZ_DEBUG
//			max_y = max(max_y,event->y1);
//			max_x = max(max_x,event->x1);
//			min_x = min(min_x,event->x1);
//			min_y = min(min_y,event->y1);
//#endif
			break;
		case 1:
			event->x1 = ((((u16)buf[1])<<8)&0x0f00) |buf[2];
			event->y1 = ((((u16)buf[3])<<8)&0x0f00) |buf[4];
//#ifdef CONFIG_LINDAWZ_DEBUG
//			max_y = max(max_y,event->y1);
//			max_x = max(max_x,event->x1);
//			min_x = min(min_x,event->x1);
//			min_y = min(min_y,event->y1);
//#endif
            break;
		case 3:
			event->x1 = ((((u16)buf[1])<<8)&0x0f00) |buf[2];
			event->y1 = ((((u16)buf[3])<<8)&0x0f00) |buf[4];
			event->x2 = ((((u16)buf[5])<<8)&0x0f00) |buf[6];
			event->y2 = ((((u16)buf[7])<<8)&0x0f00) |buf[8];
			event->touch_point = 2;// TWO POINT
			break;
		default:
		    return -1;
	}

//	printk(">>>>>>Original Value is ( %6d,%6d )\n",event->x1,event->y1);
	#ifdef CONFIG_TOUCHSCREEN_X_FLIP
	event->x1 = SCREEN_MAX_X - transform_to_screen_x(event->x1);
	#else
//	printk("Convert After event->x1=%d\n",event->x1);
        event->x1 = transform_to_screen_x(event->x1);
	#endif

	#ifdef CONFIG_TOUCHSCREEN_Y_FLIP
	event->y1 = SCREEN_MAX_Y - transform_to_screen_y(event->y1);
	#else
	event->y1 = transform_to_screen_y(event->y1);
	#endif

#ifdef CONFIG_LINDAWZ_DEBUG
	printk("\nstate =%02X-",buf[0]);
	printk("\nx1=%03d,y1=%03d-",event->x1,event->y1);
#endif
	if(event->touch_point == 2)
	{

	#ifdef CONFIG_TOUCHSCREEN_X_FLIP
		event->x2 = SCREEN_MAX_X - transform_to_screen_x(event->x2);
	#else
		event->x2 = transform_to_screen_x(event->x2);
	#endif

	#ifdef CONFIG_TOUCHSCREEN_Y_FLIP
		event->y2 = SCREEN_MAX_Y - transform_to_screen_y(event->y2);
	#else
		event->y2 = transform_to_screen_y(event->y2);
	#endif
#ifdef CONFIG_LINDAWZ_DEBUG
	printk("\t>>>>>>x2=%03d,y2=%03d\n",event->x2,event->y2);
#endif
	}
#else
    switch (event->touch_point) {
		case 2:
			event->x1 = ((((u16)buf[5])<<8)&0x0f00) |buf[6];
			event->y1 = ((((u16)buf[7])<<8)&0x0f00) |buf[8];
		case 1:
			event->x1 = ((((u16)buf[1])<<8)&0x0f00) |buf[2];
			event->y1 = ((((u16)buf[3])<<8)&0x0f00) |buf[4];
            break;
		default:
		    return -1;
	}
	#ifdef CONFIG_TOUCHSCREEN_X_FLIP
	event->x1 = SCREEN_MAX_X - transform_to_screen_x(event->x1);
	#else
	event->x1 = transform_to_screen_x(event->x1);
	#endif

	#ifdef CONFIG_TOUCHSCREEN_Y_FLIP
	event->y1 = SCREEN_MAX_Y - transform_to_screen_y(event->y1);
	#else
	event->y1 = transform_to_screen_y(event->y1);
	#endif
#endif
   	event->pressure = 200;
#ifdef PENUP_TIMEOUT_TIMER
	mod_timer(&(ldwzic_ts->penup_timeout_timer), jiffies+40);
#endif
    return 0;
}

static void ldwzic_report_value(void)
{
	struct ldwzic_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;
//	printk(">>>>>Ldwzic %4d Point In Report Data!\n",event->touch_point);
#ifdef CONFIG_TOUCH_PANEL_KEY
//	flag=0;
#endif

#ifdef CONFIG_LINDAWZ_MULTITOUCH
	switch(event->touch_point) {
		case 2:
//		        input_report_key(data->input_dev, BTN_TOUCH,  !!event->pressure);                              
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x2);	
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y2);
		        input_report_abs(data->input_dev, ABS_MT_PRESSURE, event->pressure);
			input_mt_sync(data->input_dev);
		case 1:
#ifdef CONFIG_TOUCH_PANEL_KEY
//	printk(">>>>>Touch Before Flag=%d,value=%d",flag,value);
	if(flag)
	{   
		if(value == 0)
		{
			input_report_key(data->input_dev,KEY_BACK,1);     	
			input_sync(data->input_dev);
			input_report_key(data->input_dev,KEY_BACK,0);     	
			input_sync(data->input_dev);		  
		}	
		else if(value == 1)		    
		{	
			input_report_key(data->input_dev,KEY_MENU,1);     
			input_sync(data->input_dev);
			input_report_key(data->input_dev,KEY_MENU,0);     
			input_sync(data->input_dev);
		}		
		else  if(value == 2)				     
		{
			input_report_key(data->input_dev,KEY_HOME,1);    	
			input_sync(data->input_dev);
			input_report_key(data->input_dev,KEY_HOME,0);     
			input_sync(data->input_dev);
  		}			
		else
		{
			ldwzic_ts_release(data);
		}
		flag = 0;		
	}
	else	//No Touch Icon Key
	{	                            
		input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
		input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x1);	
		input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y1);
		input_report_abs(data->input_dev, ABS_MT_PRESSURE, event->pressure);
		input_mt_sync(data->input_dev);
	}
#else	// CONFIG_TOUCH_PANEL_KEY                           
		input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
		input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x1);	
		input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y1);
//		input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
		input_report_abs(data->input_dev, ABS_MT_PRESSURE, event->pressure);
		input_mt_sync(data->input_dev);
#endif		
		default:
			break;
	}
#else	/* CONFIG_LINDAWZ_MULTITOUCH*/
	if (event->touch_point == 1) {
		input_report_abs(data->input_dev, ABS_X, event->x1);
		input_report_abs(data->input_dev, ABS_Y, event->y1);
		input_report_abs(data->input_dev, ABS_PRESSURE, event->pressure);
	}
	input_report_key(data->input_dev, BTN_TOUCH, 1);
#endif	/* CONFIG_LINDAWZ_MULTITOUCH*/
	input_sync(data->input_dev);

	dev_dbg(&this_client->dev, "%s: 1:%d %d 2:%d %d \n", __func__,
		event->x1, event->y1, event->x2, event->y2);
}	/*end ldwzic_report_value*/

static void ldwzic_ts_pen_irq_work(struct work_struct *work)
{
	int ret = -1;
	struct ldwzic_ts_data *ts;	
	ts =  container_of(work, struct ldwzic_ts_data, pen_event_work);
#ifdef CONFIG_LINDAWZ_DEBUG
	long int count = 0;
	count = ts->count;
#endif

	ret = ldwzic_read_data();	
	if (ret == 0) {	
		ldwzic_report_value();
	}
     // printk("This_client IRQ=%d\n",ts->irq);
		enable_irq(ts->irq);
}

static irqreturn_t ldwzic_ts_interrupt(int irq, void *dev_id)
{
	struct ldwzic_ts_data *ldwzic_ts = dev_id;
#ifdef CONFIG_LINDAWZ_DEBUG
		static long int count = 0;
#endif
#ifdef CONFIG_LINDAWZ_DEBUG
        if(!count){
            count++;
             return;
         }
 #endif
	disable_irq_nosync(ldwzic_ts->irq);
	

#ifdef CONFIG_LINDAWZ_DEBUG
	ldwzic_ts->count = count;	
	printk("==ctp int(%ld)=\n", count++);	
#endif

	if (!work_pending(&ldwzic_ts->pen_event_work)) {
		queue_work(ldwzic_ts->ts_workqueue, &ldwzic_ts->pen_event_work);
	}
	
	return IRQ_HANDLED;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ldwzic_ts_suspend(struct early_suspend *handler)
{
	struct ldwzic_ts_data *ts;
	ts =  container_of(handler, struct ldwzic_ts_data, early_suspend);
	disable_irq(ts->irq);	
}

static void ldwzic_ts_resume(struct early_suspend *handler)
{
#ifdef CONFIG_LINDAWZ_DEBUG
	printk("==ldwzic_ts_resume=\n");
#endif

/*
	gpio_direction_output(TS_RESET_GPIO, 0 );
	msleep(100);
	gpio_direction_output(TS_RESET_GPIO, 1 );
	msleep(100);
   	ldwzic_set_reg(LDWZIC_REG_CTRL_OPMODE, REG_TOUCH_USED_OPMODE);
	msleep(20);
*/
	struct ldwzic_ts_data *ts;
	printk("==ldwzic_ts_suspend=\n");
	ts =  container_of(handler, struct ldwzic_ts_data, early_suspend);
  	enable_irq(ts->irq); 
}
#endif  //CONFIG_HAS_EARLYSUSPEND

static int 
ldwzic_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ldwzic_ts_data *ldwzic_ts;
	struct input_dev *input_dev;
	int err = 0;
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	ldwzic_ts = kzalloc(sizeof(*ldwzic_ts), GFP_KERNEL);
	if (!ldwzic_ts)	{
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	this_client = client;

	i2c_set_clientdata(client, ldwzic_ts);

	ldwzic_ts->client = client;
	ldwzic_ts->irq	=	client->irq;

	printk("Ldwzic TouchPanel Probe (Koffu)\n");

	if (!ldwzic_ts->irq) {
		dev_dbg(&ldwzic_ts->client->dev, "no IRQ?\n");
		return -ENODEV;
	}else{
		ldwzic_ts->irq = client->irq;
	}
	
	INIT_WORK(&ldwzic_ts->pen_event_work, ldwzic_ts_pen_irq_work);
	ldwzic_ts->ts_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
	if (!ldwzic_ts->ts_workqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}
//	printk("=====client->dev.driver->name:%s=====\n",client->dev.driver->name);
/*
	gpio_direction_output(TS_RESET_GPIO, 0 );
	msleep(100);
	gpio_direction_output(TS_RESET_GPIO, 1 );
	msleep(100);
*/
	gpio_direction_output(TS_RESET_GPIO, 1 );
	msleep(100);

	msleep(100);

	gpio_direction_input(TS_IRQ_GPIO );
	gpio_enable_edge_int(gpio_to_idx(TS_IRQ_GPIO), 1, ldwzic_ts->irq - INT_GPIO_0);

	err = request_irq(ldwzic_ts->irq, ldwzic_ts_interrupt, /*IRQF_DISABLED*/IRQF_TRIGGER_FALLING, "ldwzic_ts", ldwzic_ts);
	printk("Apply IRQ sucess! LDWZIC_IRQ=%d\n",ldwzic_ts->irq);

	if (err < 0) {
		dev_err(&client->dev, "ldwzic_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}
//    printk("This_client IRQ=%d\n",ldwzic_ts->irq);

	disable_irq(ldwzic_ts->irq);

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}
	
	ldwzic_ts->input_dev = input_dev;

#ifdef CONFIG_LINDAWZ_MULTITOUCH
	set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
//	set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_PRESSURE, input_dev->absbit);

#ifdef  CONFIG_TOUCH_PANEL_KEY					//added by Koffu
	set_bit(KEY_HOME, input_dev->keybit);
	set_bit(KEY_MENU, input_dev->keybit);
	set_bit(KEY_BACK, input_dev->keybit);
#endif

	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_PRESSURE, 0, 200, 0, 0);
#else
	set_bit(ABS_X, input_dev->absbit);
	set_bit(ABS_Y, input_dev->absbit);
	set_bit(ABS_PRESSURE, input_dev->absbit);
	set_bit(BTN_TOUCH, input_dev->keybit);

	input_set_abs_params(input_dev, ABS_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_PRESSURE, 0, PRESS_MAX, 0 , 0);
#endif

	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	
#ifdef  CONFIG_TOUCH_PANEL_KEY
	set_bit(EV_SYN, input_dev->evbit);              
#endif	

	input_dev->name		= LDWZIC_NAME;		//dev_name(&client->dev)
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
		"ldwzic_ts_probe: failed to register input device: %s\n",
		dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}
	infer_ldtp_i2c_client = client;
	
#ifdef PENUP_TIMEOUT_TIMER
		init_timer(&(ldwzic_ts->penup_timeout_timer));
		ldwzic_ts->penup_timeout_timer.data = (unsigned long)ldwzic_ts;
		ldwzic_ts->penup_timeout_timer.function  =	(void (*)(unsigned long))ldwzic_ts_release;
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	ldwzic_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ldwzic_ts->early_suspend.suspend = ldwzic_ts_suspend;
	ldwzic_ts->early_suspend.resume	= ldwzic_ts_resume;
	register_early_suspend(&ldwzic_ts->early_suspend);
#endif


//	ldwzic_set_reg(LDWZIC_REG_CTRL_OPMODE, REG_TOUCH_USED_OPMODE); //5, 6,7,8
	msleep(40);

	
    enable_irq(ldwzic_ts->irq);

    return 0;

exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
	free_irq(client->irq, ldwzic_ts);
exit_irq_request_failed:
exit_platform_data_null:
	cancel_work_sync(&ldwzic_ts->pen_event_work);
	destroy_workqueue(ldwzic_ts->ts_workqueue);
exit_create_singlethread:
	i2c_set_clientdata(client, NULL);
	kfree(ldwzic_ts);
exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}

static int __devexit ldwzic_ts_remove(struct i2c_client *client)
{
	struct ldwzic_ts_data *ldwzic_ts = i2c_get_clientdata(client);
	unregister_early_suspend(&ldwzic_ts->early_suspend);
	free_irq(client->irq, ldwzic_ts);
	input_unregister_device(ldwzic_ts->input_dev);
	kfree(ldwzic_ts);
	cancel_work_sync(&ldwzic_ts->pen_event_work);
	destroy_workqueue(ldwzic_ts->ts_workqueue);
	i2c_set_clientdata(client, NULL);
	return 0;
}

static const struct i2c_device_id ldwzic_ts_id[] = {
	{ LDWZIC_NAME, 0 },
};
MODULE_DEVICE_TABLE(i2c, ldwzic_ts_id);

static struct i2c_driver ldwzic_ts_driver = {
	.probe		= ldwzic_ts_probe,
	.remove		= __devexit_p(ldwzic_ts_remove),
	.id_table	= ldwzic_ts_id,
	.driver	= {
		.name	= LDWZIC_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init ldwzic_ts_init(void)
{
	int ret;
	update_status_class = class_create(THIS_MODULE, "update_status_dev");
	if (IS_ERR(update_status_class))
	{
		ret = PTR_ERR(update_status_class);
		class_destroy(update_status_class);
	}
	create_update_status_attrs(update_status_class);
	
	return i2c_add_driver(&ldwzic_ts_driver);
}

static void __exit ldwzic_ts_exit(void)
{
	remove_update_status_attrs(update_status_class);
	
	i2c_del_driver(&ldwzic_ts_driver);
}

module_init(ldwzic_ts_init);
module_exit(ldwzic_ts_exit);

MODULE_AUTHOR("<clivia_cui@163.com>");
MODULE_DESCRIPTION("linda unknown test ic TouchScreen driver");
MODULE_LICENSE("GPL");

