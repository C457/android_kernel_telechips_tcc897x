/*
 * linux/drivers/input/keyboard/tcc-keys.c
 *
 * Based on:     	drivers/input/keyboard/bf54x-keys.c
 * Author: <linux@telechips.com>
 * Created: June 10, 2008
 * Description: Keypad ADC driver on Telechips TCC Series
 *
 * Copyright (C) 2008-2009 Telechips 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
//#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <linux/platform_device.h>
#include <asm/mach-types.h>
#include <linux/of.h>
#include <linux/of_address.h>
//#include <asm/system.h>
#include <mach/bsp.h>
//#include "tca_keypad.h"

/* For id.version */
#define TCCKEYVERSION        0x0001
#define DRV_NAME             "tcc-keypad"


//#define BUTTON_DELAY    ((20 * HZ) / 1000) // 20ms
#define BUTTON_DELAY   msecs_to_jiffies(20) 

#define KEY_RELEASED    0
#define KEY_PRESSED     1
#define KEY_CHANNEL		6

typedef struct 
{
    int             s_scancode;
    int             e_scancode;
    int             vkcode;
}tcc_button;

#if defined(CONFIG_ARCH_TCC897X)
const tcc_button tcc8971_lcn_buttons[] = {
	{0x0, 0x7FF, KEY_MENU},
};
#elif defined(CONFIG_ARCH_TCC8020)
const tcc_button tcc8020_menu_buttons[] = {
	{0x0, 0x7FF, KEY_MENU},
};

const tcc_button tcc8020_FRT_buttons0[] = {
	{0x6B, 0x83, KEY_ENTER},
	{0x266, 0x2EE, KEY_SCAN},
	{0x732, 0x8CC, KEY_SETUP},
	{0x9E5, 0xC18, KEY_MENU},
	{0xD4A, 0xFFF, KEY_DISP},
};

const tcc_button tcc8020_FRT_buttons1[] = {
	{0x6B, 0x83, KEY_SEEKDOWN},
	{0x732, 0x8CC, KEY_SEEKUP},
	{0x9E5, 0xC18, KEY_FOLDERDOWN},
	{0xD4A, 0xFFF, KEY_FOLDERUP},
};

const tcc_button tcc8020_FRT_buttons2[] = {
	{0x6B, 0x83, KEY_ONOFF},
	{0x732, 0x8CC, KEY_RADIO},
	{0x9E5, 0xC18, KEY_MEDIA},
	{0xD4A, 0xFFF, KEY_PHONE},
};

const tcc_button tcc8020_FRT_buttons3[] = {
	{0x6B, 0x83, KEY_1_RPT},
	{0x266, 0x2EE, KEY_2_RDM},
	{0x52B, 0x651, KEY_3},
	{0x732, 0x8CC, KEY_4},
	{0x9E5, 0xC18, KEY_5},
	{0xD4A, 0xFFF, KEY_6},
};

const tcc_button tcc8020_STEERING_buttons[] = {
	{0x6B, 0x83, KEY_VOLUMEUP},
	{0x117, 0x155, KEY_VOLUMEDOWN},
	{0x1E0, 0x24B, KEY_MODE},
	{0x352, 0x40F, KEY_SEEKUP},
	{0x52B, 0x651, KEY_SEEKDOWN},
	{0x732, 0x8CC, KEY_MUTE},
	{0x9E5, 0xC18, KEY_PHONEON},
	{0xD4A, 0xFFF, KEY_PHONEOFF},
};
#endif

struct tcc_private
{
    struct input_dev *input_dev;
    struct input_polled_dev *poll_dev;
   	struct tcc_adc_client *client;
	struct platform_device *pdev;
    int key_pressed;
    int old_key;
    short status;
};

struct tcc_private        *tcc_private;
static struct tcc_adc_client *client;

extern  struct tcc_adc_client *tcc_adc_register(struct device *dev, int ch);
extern unsigned long tcc_adc_getdata(struct tcc_adc_client *client);
extern  void tcc_adc_release(struct tcc_adc_client *client);                   

int tcc_keypad_getkeycodebyscancode(unsigned int adcdata)
{
	int i;
	int key = -1;

	//printk("Out Key Value :: %d\n", adcdata);
	#if defined(CONFIG_ARCH_TCC897X)
	for (i = 0; i < sizeof(tcc8971_lcn_buttons)/sizeof(tcc_button); i++) {
		if ((adcdata >= tcc8971_lcn_buttons[i].s_scancode) && (adcdata <= tcc8971_lcn_buttons[i].e_scancode))
			key = tcc8971_lcn_buttons[i].vkcode;
	}
	#elif defined(CONFIG_ARCH_TCC802X)
	for (i = 0; i < sizeof(tcc8020_menu_buttons)/sizeof(tcc_button); i++) {
		if ((adcdata >= tcc8020_menu_buttons[i].s_scancode) && (adcdata <= tcc8020_menu_buttons[i].e_scancode))
		{
			key = tcc8020_menu_buttons[i].vkcode;
			return key;
		}
	}

	for (i = 0; i < sizeof(tcc8020_FRT_buttons0)/sizeof(tcc_button); i++) {
		if ((adcdata >= tcc8020_FRT_buttons0[i].s_scancode) && (adcdata <= tcc8020_FRT_buttons0[i].e_scancode))
		{
			key = tcc8020_FRT_buttons0[i].vkcode;
			return key;
		}
	}

	for (i = 0; i < sizeof(tcc8020_FRT_buttons1)/sizeof(tcc_button); i++) {
		if ((adcdata >= tcc8020_FRT_buttons1[i].s_scancode) && (adcdata <= tcc8020_FRT_buttons1[i].e_scancode))
		{
			key = tcc8020_FRT_buttons1[i].vkcode;
			return key;
		}
	}
	
	for (i = 0; i < sizeof(tcc8020_FRT_buttons2)/sizeof(tcc_button); i++) {
		if ((adcdata >= tcc8020_FRT_buttons2[i].s_scancode) && (adcdata <= tcc8020_FRT_buttons2[i].e_scancode))
		{
			key = tcc8020_FRT_buttons2[i].vkcode;
			return key;
		}
	}

	for (i = 0; i < sizeof(tcc8020_FRT_buttons3)/sizeof(tcc_button); i++) {
		if ((adcdata >= tcc8020_FRT_buttons3[i].s_scancode) && (adcdata <= tcc8020_FRT_buttons3[i].e_scancode))
		{
			key = tcc8020_FRT_buttons3[i].vkcode;
			return key;
		}
	}
	
	for (i = 0; i < sizeof(tcc8020_STEERING_buttons)/sizeof(tcc_button); i++) {
		if ((adcdata >= tcc8020_STEERING_buttons[i].s_scancode) && (adcdata <= tcc8020_STEERING_buttons[i].e_scancode))
		{
			key = tcc8020_STEERING_buttons[i].vkcode;
			return key;
		}
	}
	#else
	printk("Not support ADC Key!!!\n")
	#endif

	return key;
}

static void tcc_key_poll_callback(struct input_polled_dev *dev)
{
    int     key = -1;
	unsigned int value=0;
	struct tcc_adc_client *client = tcc_private->client;
	
	if (client) {
		value = (unsigned int)tcc_adc_getdata(client);
	}	

	key = tcc_keypad_getkeycodebyscancode(value);

	//printk("key=%d/ status=%d/ old_key=%d, value=%d\n", key, tcc_private->status, tcc_private->old_key, value);

    if(key >= 0){
        if(tcc_private->old_key == key){
            tcc_private->key_pressed = key;
            input_report_key(tcc_private->poll_dev->input, tcc_private->key_pressed, KEY_PRESSED);
            //input_sync(tcc_private->poll_dev->input);
            tcc_private->status = KEY_PRESSED;
        } else {
            input_report_key(tcc_private->poll_dev->input, tcc_private->key_pressed, KEY_RELEASED);
            //input_sync(tcc_private->poll_dev->input);
            tcc_private->status = KEY_RELEASED;
        }
    }else{ 
        if (tcc_private->key_pressed >= 0)
        {
            input_report_key(tcc_private->poll_dev->input, tcc_private->key_pressed, KEY_RELEASED);
            //input_sync(tcc_private->poll_dev->input);
            tcc_private->key_pressed =  -1;
            tcc_private->status = KEY_RELEASED;
        }
    }

    input_sync(tcc_private->poll_dev->input);
    tcc_private->old_key = key;
}

static int tcc_key_probe(struct platform_device *pdev)
{
    struct input_polled_dev *poll_dev; 
    struct input_dev *input_dev;
	struct tcc_adc_client *client=NULL;
	int value=0;
	
    int error;
    int  i;
	int rc;
	
    tcc_private = devm_kzalloc(&pdev->dev, sizeof(struct tcc_private), GFP_KERNEL);

    poll_dev = input_allocate_polled_device();
    
    if (!tcc_private || !poll_dev) {
        error = -ENOMEM;
        goto fail;
    }
        

    platform_set_drvdata(pdev, tcc_private);

	client = tcc_adc_register(&pdev->dev, KEY_CHANNEL);
	if (IS_ERR(client)) {
		rc = PTR_ERR(client);
		goto fail;
	}
    
    poll_dev->private = tcc_private;
    poll_dev->poll = tcc_key_poll_callback;
    poll_dev->poll_interval = BUTTON_DELAY;

    input_dev = poll_dev->input;
    input_dev->evbit[0] = BIT(EV_KEY);
    input_dev->name = "telechips keypad";
    input_dev->phys = "tcc-keypad";
    input_dev->id.version = TCCKEYVERSION;
#if defined(CONFIG_ARCH_TCC897X)
    for (i = 0; i < ARRAY_SIZE(tcc8971_lcn_buttons); i++)
    	set_bit(tcc8971_lcn_buttons[i].vkcode & KEY_MAX, input_dev->keybit);
#elif defined(CONFIG_ARCH_TCC802X)
    for (i = 0; i < ARRAY_SIZE(tcc8020_FRT_buttons0); i++)
    	set_bit(tcc8020_FRT_buttons0[i].vkcode & KEY_MAX, input_dev->keybit);
    for (i = 0; i < ARRAY_SIZE(tcc8020_FRT_buttons1); i++)
    	set_bit(tcc8020_FRT_buttons1[i].vkcode & KEY_MAX, input_dev->keybit);
    for (i = 0; i < ARRAY_SIZE(tcc8020_FRT_buttons2); i++)
    	set_bit(tcc8020_FRT_buttons2[i].vkcode & KEY_MAX, input_dev->keybit);
    for (i = 0; i < ARRAY_SIZE(tcc8020_FRT_buttons3); i++)
    	set_bit(tcc8020_FRT_buttons3[i].vkcode & KEY_MAX, input_dev->keybit);
    for (i = 0; i < ARRAY_SIZE(tcc8020_STEERING_buttons); i++)
    	set_bit(tcc8020_STEERING_buttons[i].vkcode & KEY_MAX, input_dev->keybit);
#endif 

    tcc_private->poll_dev    = poll_dev;
    tcc_private->key_pressed = -1;
    tcc_private->input_dev   = input_dev;
	tcc_private->client      = client;
	tcc_private->pdev        = pdev;
	
   	input_register_polled_device(tcc_private->poll_dev);

	value = tcc_adc_getdata(client);
	printk("\x1b[1;33m[%s:key_adc = %d]\x1b[0m\n", __func__, value);                                                                                                                                                                                                                                                                                                                                                                          
	
    return 0;
    
fail:        
    kfree(tcc_private);
   	tcc_adc_release(client);
    input_free_polled_device(poll_dev);
    return 0;
}

static int tcc_key_remove(struct platform_device *pdev)
{
    input_unregister_polled_device(tcc_private->poll_dev);
    kfree(tcc_private);
   	tcc_adc_release(client);
    return 0;
}

#ifdef CONFIG_PM
static int tcc_key_suspend(struct device *dev)
{
	return 0;
}

static int tcc_key_resume(struct device *dev)
{
	return 0;
}

#else
#define tcc_key_suspend NULL
#define tcc_key_resume  NULL
#endif

static SIMPLE_DEV_PM_OPS(tcc_key_pm_ops, tcc_key_suspend, tcc_key_resume);

#ifdef CONFIG_OF
static const struct of_device_id tcc_key_dt_ids[] = {
	{.compatible = "telechips,tcckey",},
	{}
};
#else
#define tcc_key_dt_ids NULL
#endif

static struct platform_driver tcc_key_driver = {
       .driver         = {
	       .name   = "tcc-keypad",
	       .owner  = THIS_MODULE,
	       .pm	= &tcc_key_pm_ops,
	       .of_match_table	= of_match_ptr(tcc_key_dt_ids),
       },
       .probe          = tcc_key_probe,
       .remove         = tcc_key_remove,

};

module_platform_driver(tcc_key_driver);

MODULE_AUTHOR("linux@telechips.com");
MODULE_DESCRIPTION("Telechips keypad driver");
MODULE_LICENSE("GPL");

