/*
  *  switch_key.c - Linux kernel modules for ambient light sensor
  *
  *  Copyright (C) 2011
  *
  *  This program is free software; you can redistribute it and/or modify
  *  it under the terms of the GNU General Public License as published by
  *  the Free Software Foundation; either version 2 of the License, or
  *  (at your option) any later version.
  *
  *  This program is distributed in the hope that it will be useful,
  *  but WITHOUT ANY WARRANTY; without even the implied warranty of
  *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  *  GNU General Public License for more details.
  *
  *  You should have received a copy of the GNU General Public License
  *  along with this program; if not, write to the Free Software
  *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
  */
 
#include <linux/module.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/platform_device.h>

#include <linux/irq.h>
#include <linux/interrupt.h>

#include <linux/input.h>
#include <mach/gpio.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/wakelock.h>
#include "../kernel/power/power.h"
#include <linux/of_gpio.h>
#include <mach/gpiomux.h>
#include <linux/slab.h>

#include "switch_key.h"

#define DRIVER_VERSION		"0.1"

#define DEBUG_TAG                  "[switch_key]:"

static int switch_key_debug_mask = 0x00;
module_param_named(debug_mask, switch_key_debug_mask,int, S_IRUGO | S_IWUSR | S_IWGRP);

#define DEBUG_ERR(fmt, args...)   \
	do { \
		if(switch_key_debug_mask & 0x01) \
			printk(KERN_ERR  DEBUG_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args); \
		}while (0)
#define DEBUG_WAR(fmt, args...)   \
	do { \
		if(switch_key_debug_mask & 0x01) \
			printk(KERN_WARNING fmt, ##args); \
		}while (0)
#define DEBUG_LOG(fmt, args...)   \
	do { \
		if(switch_key_debug_mask & 0x01) \
			printk(KERN_INFO DEBUG_TAG fmt, ##args); \
		}while (0)

#define DEBUG_DBG(fmt, args...)   \
	do { \
		if(switch_key_debug_mask & 0x01) \
			printk(KERN_DEBUG fmt, ##args); \
		}while (0)

struct switch_key_data
{
	struct wake_lock wakelock;
	struct delayed_work irq_work;
	struct input_dev *switch_key_input;
	struct switch_key_platform_data *pdata;
	char flag_initialed;
};

struct switch_key_data switch_key_obj;

static irqreturn_t switch_key_irq_handler(int irq, void *dev_id)
{   
	if(true == switch_key_obj.flag_initialed)
	{
		wake_lock_timeout(&switch_key_obj.wakelock, 2*HZ);
		schedule_delayed_work(&switch_key_obj.irq_work,\
					msecs_to_jiffies(20));
	}
	return IRQ_HANDLED;
}

static void switch_key_work_func(struct work_struct *work)
{
	if(switch_key_obj.switch_key_input == NULL)
	{
		DEBUG_ERR("switch_key/gpio_keys input pdev null\n");
		return;
	}

	if(gpio_get_value(switch_key_obj.pdata->irq_gpio)){
		DEBUG_DBG("%s : irq gpio level high\n", __func__);
		input_report_key(switch_key_obj.switch_key_input, KEY_MUTE,KEY_PRESS);
		input_sync(switch_key_obj.switch_key_input);
		msleep(100);
		input_report_key(switch_key_obj.switch_key_input, KEY_MUTE,KEY_RELEASE);
		input_sync(switch_key_obj.switch_key_input);
	}else{
		DEBUG_DBG("%s : irq gpio level low\n", __func__);
		input_report_key(switch_key_obj.switch_key_input, KEY_VOLUMEUP,KEY_PRESS);
		input_sync(switch_key_obj.switch_key_input);
		msleep(100);
		input_report_key(switch_key_obj.switch_key_input, KEY_VOLUMEUP,KEY_RELEASE);
		input_sync(switch_key_obj.switch_key_input);
	}
}

void set_gpio_keys_input_dev(void *ipdev)
{
	if(NULL == ipdev)
		switch_key_obj.switch_key_input = NULL;
	else
		switch_key_obj.switch_key_input = (struct input_dev *)ipdev;
}

EXPORT_SYMBOL(set_gpio_keys_input_dev);

static int switch_key_suspend(struct platform_device *pdev, pm_message_t state)
{
	disable_irq(switch_key_obj.pdata->irq);
	enable_irq_wake(switch_key_obj.pdata->irq);
	return 0;
}

static int switch_key_resume(struct platform_device *pdev)
{
	disable_irq_wake(switch_key_obj.pdata->irq);
	enable_irq(switch_key_obj.pdata->irq);
	return 0;
}

static int __devinit switch_key_probe(struct platform_device *pdev)
{
	int err;
	struct device *dev = &pdev->dev;
	
	printk("switch_key probe! \n");
	switch_key_obj.flag_initialed = false;
	switch_key_obj.pdata = kzalloc(sizeof(struct switch_key_platform_data), GFP_KERNEL);
	if (!switch_key_obj.pdata) {
		DEBUG_ERR("Cannot allocate memory for switch_key_platform_data.\n");
		err = -ENOMEM;
		goto exit;
	}
	switch_key_obj.pdata->irq_gpio = of_get_named_gpio_flags(dev->of_node, "goso,irq-gpio",
				0, NULL);
	if(switch_key_obj.pdata->irq_gpio < 0){
		DEBUG_ERR("get irq gpio failed\n");
		err = -EINVAL;
		goto exit;
	}

	//init irq gpio
	err = gpio_request(switch_key_obj.pdata->irq_gpio, "switch_key irq");
	if(err < 0){
		DEBUG_ERR("request irq_gpio failed, err = %d\n", err);
		goto exit;
	}
	err = gpio_tlmm_config(GPIO_CFG(switch_key_obj.pdata->irq_gpio, 0,
				GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP,
				GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (err < 0) {
		DEBUG_ERR("gpio_tlmm_config irq_gpio failed, err = %d\n", err);
		goto error_gpio_init;
	}
	err = gpio_direction_output(switch_key_obj.pdata->irq_gpio, 1);
	if (err < 0) {
		DEBUG_ERR("gpio_direction_output irq_gpio failed, err = %d\n", err);
		goto error_gpio_init;
	}
	err = gpio_direction_input(switch_key_obj.pdata->irq_gpio);
	if (err < 0) {
		DEBUG_ERR("gpio_direction_input irq_gpio failed, err = %d\n", err);
		goto error_gpio_init;
	}

	//request irq
	switch_key_obj.pdata->irq = gpio_to_irq(switch_key_obj.pdata->irq_gpio);
	err = request_irq(switch_key_obj.pdata->irq, switch_key_irq_handler, \
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "switch_key", &(pdev->dev));
	if(err){
		DEBUG_ERR("Request IRQ for switch_key failed, return:%d\n",err);
		goto error_gpio_init;
	}
	
	INIT_DELAYED_WORK(&switch_key_obj.irq_work, switch_key_work_func);

	disable_irq(switch_key_obj.pdata->irq);
	enable_irq(switch_key_obj.pdata->irq);
	
	wake_lock_init(&switch_key_obj.wakelock, WAKE_LOCK_SUSPEND, "switch_key");

	switch_key_obj.flag_initialed = true;
	printk("switch_key probe sucess! \n");
	error_gpio_init:
		gpio_free(switch_key_obj.pdata->irq_gpio);
	exit:
		kfree(switch_key_obj.pdata);
		return err;
}

static struct of_device_id switch_key_match_table[] = {
	{ .compatible = "switch-key", },
	{ },
};

static struct platform_driver switch_key_driver = {
	.driver = {
		.name	= "switch-key",
		.owner	= THIS_MODULE,
		.of_match_table = switch_key_match_table,
	},
	.probe		= switch_key_probe,
	.resume     = switch_key_resume,
	.suspend    = switch_key_suspend,
};

static int __init switch_key_init(void)
{
	return platform_driver_register(&switch_key_driver);
}

static void __exit switch_key_exit(void)
{
	platform_driver_unregister(&switch_key_driver);
}

MODULE_AUTHOR("xionggh <xiongguanghui@gosomo.cn>");
MODULE_DESCRIPTION("switch_key");
MODULE_LICENSE("GPL");
MODULE_VERSION("ver0.1");

module_init(switch_key_init);
module_exit(switch_key_exit);
