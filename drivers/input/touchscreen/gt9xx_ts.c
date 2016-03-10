/*
 *  driver for Goodix Touchscreens
 *
 *  Copyright (c) 2014 Red Hat Inc.
 *
 *  This code is based on gt9xx.c authored by andrew@goodix.com:
 *
 *  2010 - 2012 Goodix Technology.
 */

/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; version 2 of the License.
 */

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <asm/unaligned.h>

#include <asm/io.h>
#include <linux/gpio.h>
#include <mach/irqs.h>
#include <mach/system.h>
#include <mach/hardware.h>
#include <plat/sys_config.h>

struct goodix_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	int abs_x_max;
	int abs_y_max;
	unsigned int max_touch_num;
	unsigned int int_trigger_type;
	struct work_struct  work;

	 u8  enter_update;
};


//***************************PART1:ON/OFF define*******************************
#define GTP_CUSTOM_CFG        0
#define GTP_DRIVER_SEND_CFG   1 
#define GTP_HAVE_TOUCH_KEY    0
#define GTP_POWER_CTRL_SLEEP  1
#define GTP_AUTO_UPDATE       1
#define GTP_CHANGE_X2Y        0
#define GTP_ESD_PROTECT       0
#define GTP_CREATE_WR_NODE    1
#define GTP_ICS_SLOT_REPORT   0

#define GUP_USE_HEADER_FILE   0

#define GTP_DEBUG_ON          1
#define GTP_DEBUG_ARRAY_ON    0
#define GTP_DEBUG_FUNC_ON     0

#define GTP_MAX_TOUCH         5
#define GTP_ADDR_LENGTH       2
//Register define
#define GTP_READ_COOR_ADDR    0x814E
#define GTP_REG_SLEEP         0x8040
#define GTP_REG_SENSOR_ID     0x804A
#define GTP_REG_CONFIG_DATA   0x8047
#define GTP_REG_VERSION       0x8140


#define GOODIX_CTP_NAME			"gt9xx"

static struct workqueue_struct *goodix_wq;

#define GOODIX_INT_TRIGGER		1
#define GOODIX_CONTACT_SIZE		8
#define GOODIX_MAX_CONTACTS		10

#define GOODIX_CONFIG_MAX_LENGTH	240

/* Register defines */
#define GOODIX_READ_COOR_ADDR		0x814E
#define GOODIX_REG_CONFIG_DATA		0x8047
#define GOODIX_REG_VERSION		0x8140

#define RESOLUTION_LOC		1
#define TRIGGER_LOC		6

/* Base defines for gpio irq */
#define PIO_BASE_ADDRESS	(0x01c20800)
#define PIO_RANGE_SIZE		(0x400)

#define PIO_INT_STAT_OFFSET	(0x214)
#define PIO_INT_CTRL_OFFSET	(0x210)

typedef enum {
     PIO_INT_CFG0_OFFSET = 0x200,
     PIO_INT_CFG1_OFFSET = 0x204,
     PIO_INT_CFG2_OFFSET = 0x208,
     PIO_INT_CFG3_OFFSET = 0x20c,
} int_cfg_offset;

typedef enum {
	POSITIVE_EDGE = 0x0,
	NEGATIVE_EDGE = 0x1,
	HIGH_LEVEL = 0x2,
	LOW_LEVEL = 0x3,
	DOUBLE_EDGE = 0x4
} ext_int_mode;

#define CTP_IRQ_NO		(gpio_int_info[0].port_num)
#define CTP_IRQ_MODE		(NEGATIVE_EDGE)

static void * __iomem gpio_addr = NULL;
static int gpio_int_hdle = 0;
static user_gpio_set_t gpio_int_info[1];
static int int_cfg_addr[] = { PIO_INT_CFG0_OFFSET, \
			      PIO_INT_CFG1_OFFSET, \
			      PIO_INT_CFG2_OFFSET, \
			      PIO_INT_CFG3_OFFSET };

/* Addresses to scan */
static union{
       unsigned short dirty_addr_buf[2];
       const unsigned short normal_i2c[2];
} u_i2c_addr = {{0x00},};
static __u32 twi_id;
static __u32 twi_addr;

static int screen_max_x = 0;
static int screen_max_y = 0;
static int exchange_x_y_flag = 0;

/**
 * goodix_i2c_read - read data from a register of the i2c slave device.
 *
 * @client: i2c device.
 * @reg: the register to read from.
 * @buf: raw write data buffer.
 * @len: length of the buffer to write
 */
static int goodix_i2c_read(struct i2c_client *client,
				u16 reg, u8 *buf, int len)
{
	struct i2c_msg msgs[2];
	u16 wbuf = cpu_to_be16(reg);
	int ret;

	msgs[0].flags = 0;
	msgs[0].addr  = client->addr;
	msgs[0].len   = 2;
	msgs[0].buf   = (u8 *) &wbuf;

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = client->addr;
	msgs[1].len   = len;
	msgs[1].buf   = buf;

	ret = i2c_transfer(client->adapter, msgs, 2);
	return ret < 0 ? ret : (ret != ARRAY_SIZE(msgs) ? -EIO : 0);
}


/*******************************************************	
Function:
	Read data from the i2c slave device.
Input:
	client:	i2c device.
	buf[0]:operate address.
	buf[1]~buf[len]:read data buffer.
	len:operate length.
						
Output:
	numbers of i2c_msgs to transfer
*********************************************************/
s32 gtp_i2c_read(struct i2c_client *client, u8 *buf, s32 len)
{
    struct i2c_msg msgs[2];
    s32 ret=-1;
    s32 retries = 0;
    //GTP_DEBUG_FUNC();

	msgs[0].flags = !I2C_M_RD;
    msgs[0].addr  = client->addr;
    msgs[0].len   = GTP_ADDR_LENGTH;
    msgs[0].buf   = &buf[0];

    msgs[1].flags = I2C_M_RD;
    msgs[1].addr  = client->addr;
    msgs[1].len   = len - GTP_ADDR_LENGTH;
    msgs[1].buf   = &buf[GTP_ADDR_LENGTH];

    while(retries < 5)
    {
        ret = i2c_transfer(client->adapter, msgs, 2);
        if(ret == 2)break;
        retries++;
    }
    if(retries >= 5)
    {
        printk("I2C retry timeout, reset chip.");
        //gtp_reset_guitar(client, 10);
    }
    return ret;
}

/*******************************************************	
Function:
	write data to the i2c slave device.

Input:
	client:	i2c device.
	buf[0]:operate address.
	buf[1]~buf[len]:write data buffer.
	len:operate length.
						
Output:
	numbers of i2c_msgs to transfer.
*********************************************************/
s32 gtp_i2c_write(struct i2c_client *client,u8 *buf,s32 len)
{
    struct i2c_msg msg;
    s32 ret=-1;
    s32 retries = 0;

    msg.flags = !I2C_M_RD;
    msg.addr  = client->addr;
    msg.len   = len;
    msg.buf   = buf;

    while(retries < 5)
    {
        ret = i2c_transfer(client->adapter, &msg, 1);
        if (ret == 1)break;
        retries++;
    }
    if(retries >= 5)
    {
        printk("I2C retry timeout.");
        //gtp_reset_guitar(client, 10);
    }
    return ret;
}

static int goodix_ts_read_input_report(struct goodix_ts_data *ts, u8 *data)
{
	int touch_num;
	int error;

	error = goodix_i2c_read(ts->client, GOODIX_READ_COOR_ADDR, data,
				GOODIX_CONTACT_SIZE + 1);
	if (error) {
		dev_err(&ts->client->dev, "I2C transfer error: %d\n", error);
		return error;
	}

	touch_num = data[0] & 0x0f;
	if (touch_num > GOODIX_MAX_CONTACTS)
		return -EPROTO;

	if (touch_num > 1) {
		data += 1 + GOODIX_CONTACT_SIZE;
		error = goodix_i2c_read(ts->client,
					GOODIX_READ_COOR_ADDR +
						1 + GOODIX_CONTACT_SIZE,
					data,
					GOODIX_CONTACT_SIZE * (touch_num - 1));
		if (error)
			return error;
	}

	return touch_num;
}

static void goodix_ts_report_touch(struct goodix_ts_data *ts, u8 *coor_data)
{
	//int id = coor_data[0] & 0x0F;
	int input_x = get_unaligned_le16(&coor_data[1]);
	int input_y = get_unaligned_le16(&coor_data[3]);
	//int input_w = get_unaligned_le16(&coor_data[5]);

	if (exchange_x_y_flag == 1)
		swap(input_x, input_y);

#if 0
	input_mt_slot(ts->input_dev, id);
	input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);
	input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, input_w);
#else
	input_report_abs(ts->input_dev, ABS_X, input_x);
	input_report_abs(ts->input_dev, ABS_Y, input_y);
	input_report_abs(ts->input_dev, ABS_PRESSURE, 1);
	input_report_key(ts->input_dev, BTN_TOUCH, 1);
	input_sync(ts->input_dev);
#endif
}

/**
 * goodix_process_events - Process incoming events
 *
 * @ts: our goodix_ts_data pointer
 *
 * Called when the IRQ is triggered. Read the current device state, and push
 * the input events to the user space.
 */
static void goodix_process_events(struct goodix_ts_data *ts)
{
	u8  point_data[1 + GOODIX_CONTACT_SIZE * GOODIX_MAX_CONTACTS];
	int touch_num;
	int i;

	touch_num = goodix_ts_read_input_report(ts, point_data);
	if (touch_num < 0)
	{
		return;
	}

	for (i = 0; i < touch_num; i++)
		goodix_ts_report_touch(ts,
				&point_data[1 + GOODIX_CONTACT_SIZE * i]);

	/* input_mt_sync_frame(ts->input_dev); */
	input_sync(ts->input_dev);
}

#if 0
static void goodix_ts_work_func(struct work_struct *work)
{
	static const u8 end_cmd[] = {
		GOODIX_READ_COOR_ADDR >> 8,
		GOODIX_READ_COOR_ADDR & 0xff,
		0
	};

	//printk( "gootix_ts_work_func call\n" );
	//struct goodix_ts_data *ts = dev_id;
	struct goodix_ts_data *ts = container_of(work, struct goodix_ts_data, work);

	goodix_process_events(ts);

	if (i2c_master_send(ts->client, end_cmd, sizeof(end_cmd)) < 0)
	  dev_err(&ts->client->dev, "I2C write end_cmd error\n");
	
}
#endif


/*******************************************************
Function:
	Touch up report function.

Input:
	ts:private data.
			
Output:
	None.
*******************************************************/
static void gtp_touch_up(struct goodix_ts_data* ts, s32 id)
{
#if 0
#if GTP_ICS_SLOT_REPORT
    input_mt_slot(ts->input_dev, id);
    input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, -1);
    printk("Touch id[%2d] release!", id);
#else
    input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
    input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
    input_mt_sync(ts->input_dev);
#endif
#else
	input_report_abs(ts->input_dev, ABS_PRESSURE, 0);
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
	input_sync(ts->input_dev);
#endif
}

/*******************************************************
Function:
	Touch down report function.

Input:
	ts:private data.
	id:tracking id.
	x:input x.
	y:input y.
	w:input weight.
				
Output:
	None.
******************************************************/
static void gtp_touch_down(struct goodix_ts_data* ts,s32 id,s32 x,s32 y,s32 w)
{
#if GTP_CHANGE_X2Y
    GTP_SWAP(x, y);
#endif

	if (exchange_x_y_flag == 1)
		swap( x, y );
#if 0
#if GTP_ICS_SLOT_REPORT
    input_mt_slot(ts->input_dev, id);
    input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, id);
    input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
    input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
    input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
    input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
#else
    input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
    input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
    input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
    input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
    input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, id);
    input_mt_sync(ts->input_dev);
#endif
#else
	input_report_abs(ts->input_dev, ABS_X, x);
	input_report_abs(ts->input_dev, ABS_Y, y);
	input_report_abs(ts->input_dev, ABS_PRESSURE, 1);
	input_report_key(ts->input_dev, BTN_TOUCH, 1);
	input_sync(ts->input_dev);
#endif
    //printk("ID:%d, X:%d, Y:%d, W:%d", id, x, y, w);
}

/*******************************************************
Function:
	Goodix touchscreen work function.

Input:
	work:	work_struct of goodix_wq.
	
Output:
	None.
*******************************************************/
static void goodix_ts_work_func(struct work_struct *work)
{
    u8  end_cmd[3] = {GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF, 0};
    u8  point_data[2 + 1 + 8 * GTP_MAX_TOUCH + 1]={GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF};
    u8  touch_num = 0;
    u8  finger = 0;
    static u16 pre_touch = 0;
    static u8 pre_key = 0;
    u8  key_value = 0;
    u8* coor_data = NULL;
    s32 input_x = 0;
    s32 input_y = 0;
    s32 input_w = 0;
    s32 id = 0;
    s32 i  = 0;
    s32 ret = -1;
    struct goodix_ts_data *ts = NULL;

    ts = container_of(work, struct goodix_ts_data, work);
    if (ts->enter_update)
    {
        return;
    }

    ret = gtp_i2c_read(ts->client, point_data, 12);
    if (ret < 0)
    {
        printk("I2C transfer error. errno:%d\n ", ret);
        goto exit_work_func;
    }

    finger = point_data[GTP_ADDR_LENGTH];    
    if((finger & 0x80) == 0)
    {
        goto exit_work_func;
    }

    touch_num = finger & 0x0f;
    if (touch_num > GTP_MAX_TOUCH)
    {
        goto exit_work_func;
    }

    if (touch_num > 1)
    {
        u8 buf[8 * GTP_MAX_TOUCH] = {(GTP_READ_COOR_ADDR + 10) >> 8, (GTP_READ_COOR_ADDR + 10) & 0xff};

        ret = gtp_i2c_read(ts->client, buf, 2 + 8 * (touch_num - 1)); 
        memcpy(&point_data[12], &buf[2], 8 * (touch_num - 1));
    }

#if GTP_HAVE_TOUCH_KEY
    key_value = point_data[3 + 8 * touch_num];
    
    if(key_value || pre_key)
    {
        for (i = 0; i < GTP_MAX_KEY_NUM; i++)
        {
            input_report_key(ts->input_dev, touch_key_array[i], key_value & (0x01<<i));   
        }
        touch_num = 0;
        pre_touch = 0;
    }
#endif
    pre_key = key_value;

    //printk("pre_touch:%02x, finger:%02x.", pre_touch, finger);

#if GTP_ICS_SLOT_REPORT
    if (pre_touch || touch_num)
    {
        s32 pos = 0;
        u16 touch_index = 0;

        coor_data = &point_data[3];
        if(touch_num)
        {
            id = coor_data[pos] & 0x0F;
            touch_index |= (0x01<<id);
        }

        //printk("id=%d,touch_index=0x%x,pre_touch=0x%x\n",id, touch_index,pre_touch);
        for (i = 0; i < GTP_MAX_TOUCH; i++)
        {
            if (touch_index & (0x01<<i))
            {
                input_x  = coor_data[pos + 1] | coor_data[pos + 2] << 8;
                input_y  = coor_data[pos + 3] | coor_data[pos + 4] << 8;
                input_w  = coor_data[pos + 5] | coor_data[pos + 6] << 8;

                gtp_touch_down(ts, id, input_x, input_y, input_w);
                pre_touch |= 0x01 << i;

                pos += 8;
                id = coor_data[pos] & 0x0F;
                touch_index |= (0x01<<id);
            }
            else// if (pre_touch & (0x01 << i))
            {
                gtp_touch_up(ts, i);
                pre_touch &= ~(0x01 << i);
            }
        }
    }

#else
    if (touch_num )
    {
        for (i = 0; i < touch_num; i++)
        {
            coor_data = &point_data[i * 8 + 3];

            id = coor_data[0] & 0x0F;
            input_x  = coor_data[1] | coor_data[2] << 8;
            input_y  = coor_data[3] | coor_data[4] << 8;
            input_w  = coor_data[5] | coor_data[6] << 8;

            gtp_touch_down(ts, id, input_x, input_y, input_w);
        }
    }
    else if (pre_touch)
    {
        //printk("Touch Release!");
        gtp_touch_up(ts, 0);
    }

    pre_touch = touch_num;
    input_report_key(ts->input_dev, BTN_TOUCH, (touch_num || key_value));
#endif

    input_sync(ts->input_dev);

exit_work_func:
    {
        //ret = gtp_i2c_write(ts->client, end_cmd, 3);
		if (i2c_master_send(ts->client, end_cmd, sizeof(end_cmd)) < 0)
		     dev_err(&ts->client->dev, "I2C write end_cmd error\n");
    }
}


/* goodix_ts_irq_handler - The IRQ handler
 *
 * @irq: interrupt number.
 * @dev_id: private data pointer.
 */
static irqreturn_t goodix_ts_irq_handler(int irq, void *dev_id)
{
	int reg_val;
	struct goodix_ts_data *ts = dev_id;

	/* Clear the IRQ_EINT21 interrupt pending */
	reg_val = readl(gpio_addr + PIO_INT_STAT_OFFSET);

	if (reg_val&(1<<(CTP_IRQ_NO)))
	{
		writel(reg_val&(1<<(CTP_IRQ_NO)), \
			gpio_addr + PIO_INT_STAT_OFFSET);
		queue_work(goodix_wq, &ts->work);
	}
	else
	{
		return IRQ_NONE;
	}

	return IRQ_HANDLED;
}

/**
 * goodix_clear_penirq - Clear int pending
 *
 */
static void goodix_clear_penirq(void)
{
	int reg_val;
	reg_val = readl(gpio_addr + PIO_INT_STAT_OFFSET);
	if ((reg_val = (reg_val&(1<<(CTP_IRQ_NO)))))
		writel(reg_val, gpio_addr + PIO_INT_STAT_OFFSET);
	return;
}

/**
 * goodix_set_irq_mode - Configure irq to int port.
 *
 * @ts: our goodix_ts_data pointer
 * @major_key: section key
 * @subkey: section subkey
 * @int_mode: int mode
 *
 * Must be called during probe
 */
static int goodix_set_irq_mode(struct goodix_ts_data *ts, \
				char *major_key, char *subkey, \
				ext_int_mode int_mode)
{
	__u32 reg_num = 0;
	__u32 reg_addr = 0;
	__u32 reg_val = 0;

	dev_info(&ts->client->dev, "Config gpio to int mode.\n");

	if (gpio_int_hdle)
		gpio_release(gpio_int_hdle, 2);

	gpio_int_hdle = gpio_request_ex(major_key, subkey);
	if (!gpio_int_hdle) {
		dev_err(&ts->client->dev, "Request ctp_int_port failed.\n");
		return -1;
	}
 
	gpio_get_one_pin_status(gpio_int_hdle, gpio_int_info, subkey, 1);

	reg_num = (gpio_int_info[0].port_num)%8;
	reg_addr = (gpio_int_info[0].port_num)/8;
	reg_val = readl(gpio_addr + int_cfg_addr[reg_addr]);
	reg_val &= (~(7 << (reg_num * 4)));
	reg_val |= (int_mode << (reg_num * 4));
	writel(reg_val, gpio_addr + int_cfg_addr[reg_addr]);

	goodix_clear_penirq();

	reg_val = readl(gpio_addr + PIO_INT_CTRL_OFFSET);
	reg_val |= (1 << (gpio_int_info[0].port_num));
	writel(reg_val, gpio_addr + PIO_INT_CTRL_OFFSET);

	return 0;
}

/**
 * goodix_read_config - Read the embedded configuration of the panel
 *
 * @ts: our goodix_ts_data pointer
 *
 * Must be called during probe
 */
static void goodix_read_config(struct goodix_ts_data *ts)
{
	u8 config[GOODIX_CONFIG_MAX_LENGTH];
	int error;

	error = goodix_i2c_read(ts->client, GOODIX_REG_CONFIG_DATA,
			      config,
			   GOODIX_CONFIG_MAX_LENGTH);
	if (error) {
		dev_warn(&ts->client->dev,
			 "Error reading config (%d), using defaults\n",
			 error);
		ts->abs_x_max = screen_max_x;
		ts->abs_y_max = screen_max_y;
		ts->int_trigger_type = GOODIX_INT_TRIGGER;
		return;
	}

	ts->abs_x_max = get_unaligned_le16(&config[RESOLUTION_LOC]);
	ts->abs_y_max = get_unaligned_le16(&config[RESOLUTION_LOC + 2]);
	ts->int_trigger_type = (config[TRIGGER_LOC]) & 0x03;
	if (!ts->abs_x_max || !ts->abs_y_max) {
		dev_err(&ts->client->dev,
			"Invalid config, using defaults\n");
		ts->abs_x_max = screen_max_x;
		ts->abs_y_max = screen_max_y;
	}
}


/**
 * goodix_read_version - Read goodix touchscreen version
 *
 * @client: the i2c client
 * @version: output buffer containing the version on success
 */
static int goodix_read_version(struct i2c_client *client, u16 *version)
{
	int error;
	u8 buf[6];

	error = goodix_i2c_read(client, GOODIX_REG_VERSION, buf, sizeof(buf));
	if (error) {
		dev_err(&client->dev, "read version failed: %d\n", error);
		return error;
	}

	if (version)
		*version = get_unaligned_le16(&buf[4]);

	dev_info(&client->dev, "IC VERSION: %6ph\n", buf);

	return 0;
}

/**
 * goodix_i2c_test - I2C test function to check if the device answers.
 *
 * @client: the i2c client
 */
static int goodix_i2c_test(struct i2c_client *client)
{
	int retry = 0;
	int error;
	u8 test;

	while (retry++ < 2) {
		error = goodix_i2c_read(client, GOODIX_REG_CONFIG_DATA,
					&test, 1);
		if (!error)
			return 0;

		dev_err(&client->dev, "i2c test failed attempt %d: %d\n",
			retry, error);
		msleep(20);
	}

	return error;
}

/**
 * goodix_request_input_dev - Allocate, populate and register the input device
 *
 * @ts: our goodix_ts_data pointer
 *
 * Must be called during probe
 */
static int goodix_request_input_dev(struct goodix_ts_data *ts)
{
	int error;

	ts->input_dev = input_allocate_device();
	if (!ts->input_dev) {
		dev_err(&ts->client->dev, "Failed to allocate input device.");
		return -ENOMEM;
	}

	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) |
				  BIT_MASK(EV_KEY) |
				  BIT_MASK(EV_ABS);

	if (exchange_x_y_flag == 1)
		swap(ts->abs_x_max, ts->abs_y_max);

#if 0
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0,
				ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0,
				ts->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
#else
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	input_set_abs_params(ts->input_dev, ABS_X, 0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0, ts->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 1, 0, 0);
#endif

	input_mt_init_slots(ts->input_dev, GOODIX_MAX_CONTACTS);

	ts->input_dev->name = "Goodix Capacitive TouchScreen";
	ts->input_dev->phys = "input/ts";
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor = 0x0416;
	ts->input_dev->id.product = 0x1001;
	ts->input_dev->id.version = 10427;

	error = input_register_device(ts->input_dev);
	if (error) {
		dev_err(&ts->client->dev,
			"Failed to register input device: %d", error);
		return error;
	}

	return 0;
}

/**
 * goodix_fetch_sysconfig_para - get config info from sysconfig.fex file.
 * return value:
 *	= 0: success;
 *	< 0: err;
 */
static int goodix_fetch_sysconfig_para(void)
{
	int ret = -1;
	int ctp_used = 0;
	char name[I2C_NAME_SIZE];
	script_parser_value_type_t type = SCRIPT_PARSER_VALUE_TYPE_STRING;

	if (SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_used", &ctp_used, 1)) {
		printk(KERN_ERR "*** ctp_used set to 0!\n");
		printk(KERN_ERR "*** If use ctp, please set ctp_used to 1.\n");
		return ret;
	}

	if (SCRIPT_PARSER_OK != script_parser_fetch_ex("ctp_para", "ctp_name", (int *)(&name), &type, sizeof(name)/sizeof(int))) {
		printk(KERN_ERR "Failed to fetch ctp_name.\n");
		return ret;
	}
	printk(KERN_INFO "ctp_name is %s.\n", name);

	if (strcmp(GOODIX_CTP_NAME, name)) {
		printk(KERN_ERR "Name %s does not match GOODIX_CTP_NAME.\n", name);
		return ret;
	}

	if (SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_twi_id", &twi_id, sizeof(twi_id)/sizeof(__u32))) {
		printk(KERN_ERR "Failed to fetch ctp_twi_id.\n");
		return ret;
	}
	printk(KERN_INFO "ctp_twi_id is %d.\n", twi_id);

	if (SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_twi_addr", &twi_addr, sizeof(twi_addr)/sizeof(__u32))) {
		printk(KERN_ERR "Failed to fetch ctp_twi_addr.\n");
		return ret;
	}
	printk(KERN_INFO "ctp_twi_addr is 0x%hx.\n", twi_addr);

	u_i2c_addr.dirty_addr_buf[0] = twi_addr;
	u_i2c_addr.dirty_addr_buf[1] = I2C_CLIENT_END;

	if (SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_screen_max_x", &screen_max_x, 1)) {
		printk(KERN_ERR "Failed to fetch ctp_screen_max_x.\n");
		return ret;
	}
	printk(KERN_INFO "screen_max_x = %d.\n", screen_max_x);

	if (SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_screen_max_y", &screen_max_y, 1)) {
		printk(KERN_ERR "Failed to fetch ctp_screen_max_y.\n");
		return ret;
	}
	printk(KERN_INFO "screen_max_y = %d.\n", screen_max_y);

	if (SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_exchange_x_y_flag", &exchange_x_y_flag, 1)) {
		printk(KERN_ERR "Failed to fetch ctp_exchange_x_y_flag.\n");
		return ret;
	}
	printk(KERN_INFO "exchange_x_y_flag = %d.\n", exchange_x_y_flag);

	return 0;
}


static int goodix_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct goodix_ts_data *ts;
	int error;
	u16 version_info;

	dev_dbg(&client->dev, "I2C Address: 0x%02x\n", client->addr);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "I2C check functionality failed.\n");
		return -ENXIO;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	INIT_WORK(&ts->work, goodix_ts_work_func);

	ts->client = client;
	i2c_set_clientdata(client, ts);

	error = goodix_i2c_test(client);
	if (error) {
		dev_err(&client->dev, "I2C communication failure: %d\n", error);
		return error;
	}

	error = goodix_read_version(client, &version_info);
	if (error) {
		dev_err(&client->dev, "Read version failed.\n");
		return error;
	}

	goodix_read_config(ts);

	error = goodix_request_input_dev(ts);
	if (error)
		return error;

	gpio_addr = ioremap(PIO_BASE_ADDRESS, PIO_RANGE_SIZE);
	if (!gpio_addr)
		return -EIO;

	error = goodix_set_irq_mode(ts, "ctp_para", "ctp_int_port", \
				    CTP_IRQ_MODE);
	if (error < 0) {
		dev_err(&ts->client->dev, "Set irq mode failed.");
		enable_irq(SW_INT_IRQNO_PIO);
	}

	goodix_wq = create_singlethread_workqueue("goodix_wq");
	if (!goodix_wq) {
			printk(KERN_ALERT "Creat goodix_wq workqueue failed.\n");
			return -ENOMEM;
	}
	flush_workqueue(goodix_wq);
	
	error = request_irq(SW_INT_IRQNO_PIO, goodix_ts_irq_handler, IRQF_SHARED, client->name , ts);

	if (error) {
		dev_err(&client->dev, "request IRQ failed: %d.\n", error);
		return error;
	}

	return 0;
}

static int __devexit goodix_ts_remove(struct i2c_client *client)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(client);

	printk(KERN_INFO "Driver gt9xx remove\n" );

	flush_workqueue(goodix_wq);
	if (goodix_wq)
		destroy_workqueue(goodix_wq);

	free_irq(SW_INT_IRQNO_PIO, ts);

	input_unregister_device(ts->input_dev);

	kfree(ts);

	i2c_set_clientdata(client, NULL);

	if (gpio_addr)
		iounmap(gpio_addr);

	if (gpio_int_hdle)
		gpio_release(gpio_int_hdle, 2);

 	return 0;
}

/**
 * goodix_ts_detect - Device detection callback for automatic device creation
 *
 */
static int goodix_ts_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	if (twi_id == adapter->nr) {
		printk(KERN_INFO "Detected chip gt9xx at adapter %d, address 0x%02x\n", i2c_adapter_id(adapter), client->addr);
		strlcpy(info->type, GOODIX_CTP_NAME, I2C_NAME_SIZE);
 		return 0;
	} else
		return -ENODEV;
}

static const struct i2c_device_id goodix_ts_id[] = {
	{ GOODIX_CTP_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, goodix_ts_id);

static struct i2c_driver goodix_ts_driver = {
	.class = I2C_CLASS_HWMON,
	.probe = goodix_ts_probe,
	.remove = goodix_ts_remove,
	.id_table = goodix_ts_id,
	.driver = {
		.name = GOODIX_CTP_NAME,
		.owner = THIS_MODULE,
	},
	.address_list = u_i2c_addr.normal_i2c,
};

/**
 * goodix_ts_init - Driver install function
 *
 */
static int __devinit goodix_ts_init(void)
{
	int ret = -1;

	ret = goodix_fetch_sysconfig_para();
	if (ret != 0)
		return ret;
	
	goodix_ts_driver.detect = goodix_ts_detect;
	ret = i2c_add_driver(&goodix_ts_driver);
	return ret;
}

/**
 * goodix_ts_exit - Driver uninstall function
 *
 */
static void __exit goodix_ts_exit(void)
{
	printk(KERN_INFO "Driver gt9xx unregisteredi\n");
	i2c_del_driver(&goodix_ts_driver);
	return;
}

module_init(goodix_ts_init);
module_exit(goodix_ts_exit);

MODULE_AUTHOR("Benjamin Tissoires <benjamin.tissoires@gmail.com>");
MODULE_AUTHOR("Bastien Nocera <hadess@hadess.net>");
MODULE_DESCRIPTION("Goodix touchscreen driver");
MODULE_LICENSE("GPL v2");
