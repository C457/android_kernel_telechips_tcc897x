/****************************************************************************
One line to give the program's name and a brief idea of what it does.
Copyright (C) 2013 Telechips Inc.

This program is free software; you can redistribute it and/or modify it under the terms
of the GNU General Public License as published by the Free Software Foundation;
either version 2 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program; if not, write to the Free Software Foundation, Inc., 59 Temple Place,
Suite 330, Boston, MA 02111-1307 USA
****************************************************************************/
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/videodev2.h>

#include <linux/delay.h> 
#include <mach/hardware.h>
#include <linux/clk.h>
#include <asm/system.h>
#include <linux/regulator/consumer.h>
#include <linux/spinlock.h>
#include <linux/module.h>

#include <asm/io.h>

#include <mach/bsp.h>
#include <linux/gpio.h>
#include <mach/gpio.h>

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>

#include "tcc_hdin_ctrl.h"
#include "tcc_hdin_video.h"
#include "tcc_hdin_main.h"

static int debug	   = 0;
#define dprintk(msg...) if(debug) { printk( "\e[33mhdin_ctrl : \e[0m" msg); }
#ifdef HDIN_DRV_BYPASS_EN
void hdin_ctrl_set_bypass_mode(struct tcc_hdin_device *hdev, int *onoff)
{
	hdev->bypass = *onoff;
}
#endif
void hdin_clock_get(struct tcc_hdin_device *hdev)
{
	if(hdev->hdin_np)
		hdev->hdin_clk = of_clk_get(hdev->hdin_np, 0);
}

void hdin_clock_put(struct tcc_hdin_device *hdev)
{
	clk_put(hdev->hdin_clk);
}

void hdin_clock_enable(struct tcc_hdin_device *hdev)
{
	clk_prepare_enable(hdev->hdin_clk);
}

void hdin_clock_disable(struct tcc_hdin_device *hdev)
{
	clk_disable_unprepare(hdev->hdin_clk);
}

int hdin_ctrl_get_resolution(struct tcc_hdin_device *hdev)
{
	unsigned int ret = 0x00;
	ret = hdev->module_func.GetVideoSize(&hdev->hdmi_in_Interlaced);
	return ret;
}

int hdin_ctrl_get_fps(int *nFrameRate)
{
	*nFrameRate = SENSOR_FRAMERATE;

	if(*nFrameRate)
	{
		return 0;
	}
	else
	{
		printk("Sensor Driver dosen't have frame rate information!!\n");
		return -1;
	}
}

int hdin_ctrl_get_audio_samplerate(struct tcc_hdin_device *hdev, int value)
{
	return hdev->module_func.GetAudioSR();
}

int hdin_ctrl_get_audio_type(struct tcc_hdin_device *hdev, int value)
{
	return hdev->module_func.GetAudioType();
}

void hdin_ctrl_get_gpio(struct tcc_hdin_device *hdev)
{
	if(hdev->hdin_np)
	{
		hdev->gpio.pwr_port	= of_get_named_gpio(hdev->hdin_np,"pwr-gpios",0);
		hdev->gpio.key_port	= of_get_named_gpio(hdev->hdin_np,"key-gpios",0);
		hdev->gpio.rst_port	= of_get_named_gpio(hdev->hdin_np,"rst-gpios",0);
		hdev->gpio.int_port	= of_get_named_gpio(hdev->hdin_np,"int-gpios",0);
		//hdev->gpio.scan_port  = of_get_named_gpio(hdev->hdin_np,"scan-gpios",0);

		gpio_request(hdev->gpio.pwr_port, "HDIN_PWR");
		gpio_direction_output(hdev->gpio.pwr_port, 1);

		gpio_request(hdev->gpio.rst_port, "HDIN_RST");
		gpio_direction_output(hdev->gpio.rst_port, 1);

		gpio_request(hdev->gpio.key_port, "HDIN_KEY");
		gpio_direction_output(hdev->gpio.key_port, 1);

		gpio_request(hdev->gpio.int_port, "HDIN_INT");
		gpio_direction_output(hdev->gpio.int_port, 1);

		//gpio_request(hdev->gpio.scan_port, "HDIN_SCAN");
		//gpio_direction_output(hdev->gpio.scan_port, 1);
	}
}

void hdin_ctrl_pwr_enable(struct tcc_hdin_device *hdev)
{
	gpio_set_value(hdev->gpio.pwr_port,1);
}

void hdin_ctrl_pwr_disable(struct tcc_hdin_device *hdev)
{
	gpio_set_value(hdev->gpio.pwr_port,0);
}

void hdin_ctrl_key_enable(struct tcc_hdin_device *hdev)
{
	gpio_set_value(hdev->gpio.key_port,1);
}

void hdin_ctrl_key_disable(struct tcc_hdin_device *hdev)
{
	gpio_set_value(hdev->gpio.key_port,0);
}

void hdin_ctrl_rst_enable(struct tcc_hdin_device *hdev)
{
	gpio_set_value(hdev->gpio.rst_port,1);
}

void hdin_ctrl_rst_disable(struct tcc_hdin_device *hdev)
{
	gpio_set_value(hdev->gpio.rst_port,0);
}

int hdin_ctrl_init(struct tcc_hdin_device *hdev)
{
	hdin_ctrl_pwr_enable(hdev);
	hdin_ctrl_rst_enable(hdev);
	hdin_ctrl_key_enable(hdev);

	module_init_fnc(&hdev->module_func);

	if(!hdev->enabled) {
		hdev->enabled = 1;
	}

	if(hdev->module_func.Open() < 0) {
		return -1;
	}
	return 0;
}

void hdin_ctrl_cleanup(struct tcc_hdin_device *hdev)
{
	dprintk("hdin_enabled = [%d]\n", hdev->enabled);

	if(hdev->enabled)
	{
		hdev->module_func.Close();
		hdev->enabled = 0;
	}

    return;
}

void hdin_ctrl_delay(int ms)
{
	unsigned int msec;

	msec = ms / 10; //10msec unit

	if(!msec)	msleep(1);
	else		msleep(msec);
}

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
MODULE_ALIAS(DRIVER_NAME);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_VERSION(DRIVER_VERSION);

