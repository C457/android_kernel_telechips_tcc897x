/*
 * drivers/media/video/tccvin/tcc_vin_i2c.c
 *
 * Copyright (C) 2008 Telechips, Inc. 
 * 
 * Video-for-Linux (Version 2) camera capture driver for Telechisp SoC.
 *
 * leverage some code from CEE distribution 
 * Copyright (C) 2003-2004 MontaVista Software, Inc.
 * 
 * This package is free software; you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation. 
 * 
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR 
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED 
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE. 
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <mach/bsp.h>
#include "tcc_vin.h"
#include "tcc_vin_hw.h"
#include <asm/io.h>

#include "sensor/support_sensor.h"
#include <asm/gpio.h>


#ifdef CONFIG_TCC_VIN_DEBUG
#define dprintk(fmt, args...) \
	printk("\e[33m[ vin_i2c]%s(%d): \e[0m" fmt, __func__, __LINE__, ## args);
#else
#define dprintk(fmt, args...)
#endif
#if 0
struct cam_i2c_chip_info {
	unsigned gpio_start;
	uint16_t reg_output;
	uint16_t reg_direction;

	struct i2c_client *client;
	struct gpio_chip gpio_chip;
};
#endif

static int cam_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int cam_i2c_remove(struct i2c_client *client);

#if defined(CONFIG_TCC_V4L2_DEV_VIDEO0)
	static struct i2c_client *cif_i2c_client0 = NULL;
#endif
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO1)
	static struct i2c_client *cif_i2c_client1 = NULL;
#endif
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO2)
	static struct i2c_client *cif_i2c_client2 = NULL;
#endif
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO3)
	static struct i2c_client *cif_i2c_client3 = NULL;
#endif
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO4)
	static struct i2c_client *cif_i2c_client4 = NULL;
#endif

//static struct i2c_client *cif_i2c_client = NULL;

static const struct i2c_device_id cif_i2c_id[] = {
	{"i2c_cif_video0", 0, },
	{"i2c_cif_video1", 1, },
	{"i2c_cif_video2", 2, },
	{"i2c_cif_video3", 3, },
	{"i2c_cif_video4", 4, },
};
MODULE_DEVICE_TABLE(i2c, cif_i2c_id);

static struct of_device_id cam_i2c_of_match[] = {
	{ .compatible = "" },
	{ .compatible = "" },
	{ .compatible = "" },
	{ .compatible = "" },
	{ .compatible = "" },
	{}
};
MODULE_DEVICE_TABLE(of, cam_i2c_of_match);

static struct i2c_driver cif_i2c_driver_video0 = {
	.driver = {
		.name	= "i2c_cif_video0",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(&cam_i2c_of_match[0]),
	},
	.probe		= cam_i2c_probe,
	.remove 	= cam_i2c_remove,
	.id_table	= &cif_i2c_id[0],
};
static struct i2c_driver cif_i2c_driver_video1 = {
	.driver = {
		.name	= "i2c_cif_video1",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(&cam_i2c_of_match[1]),
	},
	.probe		= cam_i2c_probe,
	.remove 	= cam_i2c_remove,
	.id_table	= &cif_i2c_id[1],
};
static struct i2c_driver cif_i2c_driver_video2 = {
	.driver = {
		.name	= "i2c_cif_video2",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(&cam_i2c_of_match[2]),
	},
	.probe		= cam_i2c_probe,
	.remove 	= cam_i2c_remove,
	.id_table	= &cif_i2c_id[2],
};
static struct i2c_driver cif_i2c_driver_video3 = {
	.driver = {
		.name	= "i2c_cif_video3",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(&cam_i2c_of_match[3]),
	},
	.probe		= cam_i2c_probe,
	.remove 	= cam_i2c_remove,
	.id_table	= &cif_i2c_id[3],
};
static struct i2c_driver cif_i2c_driver_video4 = {
	.driver = {
		.name	= "i2c_cif_video4",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(&cam_i2c_of_match[4]),
	},
	.probe		= cam_i2c_probe,
	.remove 	= cam_i2c_remove,
	.id_table	= &cif_i2c_id[4],
};


static int cam_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{	
	dprintk("__ in \n");
	dprintk("__ client-> name : %s \n", client->name);
	dprintk("__ client->dev.id : %d \n", client->dev.id);
	dprintk("__ i2c_device_id address : 0x%x\n", id);
	dprintk("__ i2c_device_id name : %s\n", id->name);
	dprintk("__ client->dev.name : %s \n", client->dev.init_name);
	dprintk("__ client->dev.driver->name : %s \n", client->dev.driver->name);

	if(!strcmp(client->name, "0x42")) {
		dprintk("camera is adv7182 \n");
		client->flags |= I2C_M_NO_STOP;
	}
	else
		client->flags &= ~(I2C_M_NO_STOP);

#if defined(CONFIG_TCC_V4L2_DEV_VIDEO0)
	if(!strcmp(client->dev.driver->name, cif_i2c_driver_video0.driver.name))
	{
		dprintk("__[TCC_V4L2_DEV_VIDEO0] %s's i2c client \n", cif_i2c_driver_video0.driver.name);
		cif_i2c_client0 = client;
	}
#endif
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO1)
	if(!strcmp(client->dev.driver->name, cif_i2c_driver_video1.driver.name))
	{
		dprintk("__[TCC_V4L2_DEV_VIDEO1] %s's i2c client \n", cif_i2c_driver_video1.driver.name);
		cif_i2c_client1 = client;
	}
#endif
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO2)
	if(!strcmp(client->dev.driver->name, cif_i2c_driver_video2.driver.name))
	{
		dprintk("__[TCC_V4L2_DEV_VIDEO2] %s's i2c client \n", cif_i2c_driver_video2.driver.name);
		cif_i2c_client2 = client;
	}
#endif
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO3)
	if(!strcmp(client->dev.driver->name, cif_i2c_driver_video3.driver.name))
	{
		dprintk("__[TCC_V4L2_DEV_VIDEO3] %s's i2c client \n", cif_i2c_driver_video3.driver.name);
		cif_i2c_client3 = client;
	}
#endif
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO4)
	if(!strcmp(client->dev.driver->name, cif_i2c_driver_video4.driver.name))
	{
		dprintk("__[TCC_V4L2_DEV_VIDEO4] %s's i2c client \n", cif_i2c_driver_video4.driver.name);
		cif_i2c_client4 = client;
	}
#endif
	
#if 0
	switch(id->driver_data) {
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO0)
		case 0:
			cif_i2c_client0 = client;
			break;
#endif
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO1)
		case 1:
			cif_i2c_client1 = client;
			break;
#endif
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO2)
		case 2:
			cif_i2c_client2 = client;
			break;
#endif
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO3)
		case 3:
			cif_i2c_client3 = client;
			break;
#endif
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO4)
		case 4:
			cif_i2c_client4 = client;
			break;
#endif
	}
#endif

	printk(KERN_INFO "_______%s() :  addr = 0x%x \n", __func__, (client->addr)<<1);
	dprintk("__ out \n");
	return 0;
}

static int cam_i2c_remove(struct i2c_client *client)
{
//	struct cam_i2c_chip_info		*chip  = i2c_get_clientdata(client);
	
	dprintk("%s\n", client->name);
	
//	kfree(chip);
	
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO0)
		if(!strcmp(client->dev.driver->name, cif_i2c_driver_video0.driver.name))
			cif_i2c_client0 = NULL;
#endif
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO1)
		if(!strcmp(client->dev.driver->name, cif_i2c_driver_video1.driver.name))
			cif_i2c_client1 = NULL;
#endif
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO2)
		if(!strcmp(client->dev.driver->name, cif_i2c_driver_video2.driver.name))
			cif_i2c_client2 = NULL;
#endif
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO3)
		if(!strcmp(client->dev.driver->name, cif_i2c_driver_video3.driver.name))
			cif_i2c_client3 = NULL;
#endif
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO4)
		if(!strcmp(client->dev.driver->name, cif_i2c_driver_video4.driver.name))
			cif_i2c_client4 = NULL;
#endif

	return 0;
}
	

static int cif_i2c_register(struct tcc_video_device *vdev)
{
	unsigned int i2c_core, i2c_addr;
	struct i2c_board_info info;
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	struct tcc_cif_platform_data *pdata = vdev->dev->platform_data;

	i2c_core = pdata->port->i2c_core;
	i2c_addr = vdev->sinfo->i2c_addr;

	memset(&info, 0, sizeof(struct i2c_board_info));
	info.addr = i2c_addr;
	strlcpy(info.type, pdata->i2c_drv_name, I2C_NAME_SIZE);

	adapter = i2c_get_adapter(i2c_core);
	if (!adapter) {
		printk("error: i2c_get_adapter(%s)\n", info.type);
		return -ENODEV;
	}

	client = i2c_new_device(adapter, &info);
	i2c_put_adapter(adapter);
	if (!client) {
		printk("error: i2c_put_adapter(%s)\n", info.type);
		return -ENODEV;
	}

	vdev->cif_i2c_client = client;

	dprintk("%s i2c: name(%s), core(%d), addr(0x%x)\n", 
		vdev->name, client->name, i2c_core, client->addr);
	return 0;
}
#if 0
static const struct i2c_device_id cif_i2c_id[] = {
	{"i2c_cif_video0", 0, },
	{"i2c_cif_video1", 1, },
	{"i2c_cif_video2", 2, },
	{"i2c_cif_video3", 3, },
	{"i2c_cif_video4", 4, },
};
MODULE_DEVICE_TABLE(i2c, cif_i2c_id);

static struct of_device_id cam_i2c_of_match[] = {
	{ .compatible = "" },
	{ .compatible = "" },
	{ .compatible = "" },
	{ .compatible = "" },
	{ .compatible = "" },
	{}
};
MODULE_DEVICE_TABLE(of, cam_i2c_of_match);

static struct i2c_driver cif_i2c_driver_video0 = {
	.driver = {
		.name	= "i2c_cif_video0",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(&cam_i2c_of_match[0]),
	},
	.probe		= cam_i2c_probe,
	.remove		= cam_i2c_remove,
	.id_table	= &cif_i2c_id[0],
};
static struct i2c_driver cif_i2c_driver_video1 = {
	.driver = {
		.name	= "i2c_cif_video1",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(&cam_i2c_of_match[1]),
	},
	.probe		= cam_i2c_probe,
	.remove		= cam_i2c_remove,
	.id_table	= &cif_i2c_id[1],
};
static struct i2c_driver cif_i2c_driver_video2 = {
	.driver = {
		.name	= "i2c_cif_video2",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(&cam_i2c_of_match[2]),
	},
	.probe		= cam_i2c_probe,
	.remove		= cam_i2c_remove,
	.id_table	= &cif_i2c_id[2],
};
static struct i2c_driver cif_i2c_driver_video3 = {
	.driver = {
		.name	= "i2c_cif_video3",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(&cam_i2c_of_match[3]),
	},
	.probe		= cam_i2c_probe,
	.remove		= cam_i2c_remove,
	.id_table	= &cif_i2c_id[3],
};
static struct i2c_driver cif_i2c_driver_video4 = {
	.driver = {
		.name	= "i2c_cif_video4",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(&cam_i2c_of_match[4]),
	},
	.probe		= cam_i2c_probe,
	.remove		= cam_i2c_remove,
	.id_table	= &cif_i2c_id[4],
};
#endif
#if 0
int cif_i2c_init(struct tcc_video_device *vdev)
{
	int ret = 0;
	struct i2c_driver *i2c_drv = NULL;

	dprintk("%s\n", vdev->name);

	switch (vdev->video_nr) {
	case 0: i2c_drv = &cif_i2c_driver_video0; break;
	case 1: i2c_drv = &cif_i2c_driver_video1; break;
	case 2: i2c_drv = &cif_i2c_driver_video2; break;
	case 3: i2c_drv = &cif_i2c_driver_video3; break;
	case 4: i2c_drv = &cif_i2c_driver_video4; break;
	}

	// TODO: you have to work below code after comfirm multi open i2c driver 
	//i2c_drv = kzalloc(sizeof(struct i2c_driver), GFP_KERNEL);
	//i2c_drv->driver.name = pdata->i2c_drv_name;
	//i2c_drv->driver.owner = THIS_MODULE;
	//i2c_drv->probe = cam_i2c_probe;
	//i2c_drv->remove = cam_i2c_remove;
	//i2c_drv->id_table->driver_data = 0;
	//strlcpy(i2c_drv->id_table->name, pdata->i2c_drv_name, I2C_NAME_SIZE);
	//vdev->cif_i2c_driver = i2c_drv;
	//kfree(i2c_drv);	//in the cif_i2c_exit

	/* add i2c driver */
	ret = i2c_add_driver(i2c_drv);
	if (ret < 0) {
		printk("error: i2c_add_driver(i2c_cif_video%d)\n", vdev->video_nr);
		goto err1;
	}

	/* register i2c driver (call probe) */
	ret = cif_i2c_register(vdev);
	if (ret < 0) {
		printk("error: register i2c_client(i2c_cif_video%d)\n", vdev->video_nr);
		goto err2;
	}

	return ret;
err2:
	i2c_del_driver(i2c_drv);
err1:
	return ret;
}
EXPORT_SYMBOL(cif_i2c_init);
#else
static int __init cif_i2c_init(void)
{
	int ret = 0;
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO0)
	ret = i2c_add_driver(&cif_i2c_driver_video0);
	dprintk("_______ [%s] \n", cif_i2c_driver_video0.driver.of_match_table->compatible);
#endif
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO1)
	ret = i2c_add_driver(&cif_i2c_driver_video1);
	dprintk("_______ [%s] \n", cif_i2c_driver_video1.driver.of_match_table->compatible);
#endif
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO2)
	ret = i2c_add_driver(&cif_i2c_driver_video2);
#endif
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO3)
	ret = i2c_add_driver(&cif_i2c_driver_video3);
#endif
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO4)
	ret = i2c_add_driver(&cif_i2c_driver_video4);
#endif
	
	return ret;
}
#endif 

#if 0
void __exit cif_i2c_exit(struct tcc_video_device *vdev)
{
	struct i2c_driver *i2c_drv = NULL;

	dprintk("%s\n", vdev->name);

	switch (vdev->video_nr) {
	case 0: i2c_drv = &cif_i2c_driver_video0; break;
	case 1: i2c_drv = &cif_i2c_driver_video1; break;
	case 2: i2c_drv = &cif_i2c_driver_video2; break;
	case 3: i2c_drv = &cif_i2c_driver_video3; break;
	case 4: i2c_drv = &cif_i2c_driver_video4; break;
	}

	i2c_del_driver(i2c_drv);
}
EXPORT_SYMBOL(cif_i2c_exit);
#else
static void __exit cif_i2c_exit(void)
{
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO0)
	i2c_del_driver(&cif_i2c_driver_video0);
#endif
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO1)
	i2c_del_driver(&cif_i2c_driver_video1);
#endif
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO2)
	i2c_del_driver(&cif_i2c_driver_video2);
#endif
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO3)
	i2c_del_driver(&cif_i2c_driver_video3);
#endif
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO4)
	i2c_del_driver(&cif_i2c_driver_video4);
#endif
}
#endif

/*----------------------------------------------------------
 * i2c send/recv function
 *----------------------------------------------------------
 */

int cif_i2c_send_no_stop(const struct i2c_client *client, char *buf, int count)
{
	int ret;
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg;

	msg.addr = client->addr;
	msg.len = count;
	msg.buf = (char *)buf;
	msg.flags = client->flags & I2C_M_TEN;
	msg.flags |= I2C_M_NO_STOP;

	ret = i2c_transfer(adap, &msg, 1);

	/*
	 * If everything went ok (i.e. 1 msg transmitted), return #bytes
	 * transmitted, else error code.
	 */
	return (ret == 1) ? count : ret;
}

int cif_i2c_send(struct i2c_client *client, unsigned char* data, 
					unsigned short reg_bytes, unsigned short data_bytes)
{
	unsigned short bytes = reg_bytes + data_bytes;

	if (unlikely(client == NULL)) {
		printk("%s: i2c_client is NULL\n", __func__);
		return -EIO;
	}
	dprintk("client.name(%s) .addr(0x%x)\n", client->name, client->addr);

	if (i2c_master_send(client, data, bytes) != bytes)
		return -EIO;

	return 0;
}
EXPORT_SYMBOL(cif_i2c_send);

int cif_i2c_recv(struct i2c_client *client, 
					unsigned short reg, unsigned char reg_bytes, 
					unsigned char *val, unsigned short val_bytes)
{
	unsigned char data[2];

	if (unlikely(client == NULL)) {
		printk("%s: i2c_client is NULL\n", __func__);
		return -EIO;
	}

	if (reg_bytes == 2) {
		data[0] = reg >> 8;
		data[1] = (u8)reg & 0xff;
	} else {
		data[0] = (u8)reg & 0xff;
	}

	if(client->flags & I2C_M_NO_STOP) {
		if(cif_i2c_send_no_stop(client, data, reg_bytes) != reg_bytes)
			return -EIO;
	}
	else
	{
		if(i2c_master_send(client, data, reg_bytes) != reg_bytes)
			return -EIO;
	}
	if (i2c_master_recv(client, val, val_bytes) != val_bytes)
		return -EIO;

    return 0;
}
EXPORT_SYMBOL(cif_i2c_recv);

int cif_set_i2c_name(int id, const char * cam_name)
{	
	switch(id) {
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO0)
		case 0:
			strcpy(cam_i2c_of_match[0].compatible, cam_name);
			dprintk("___[V4L2_DEV_VIDEO0] cam_name : [%s] \n", cam_name);
			dprintk("___[V4L2_DEV_VIDEO0] cam_i2c_of_match[0].compatible [%s] \n", cam_i2c_of_match[0].compatible);
			break;
#endif
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO1)
		case 1:
			strcpy(cam_i2c_of_match[1].compatible, cam_name);
			dprintk("___[V4L2_DEV_VIDEO1] cam_name : [%s] \n", cam_name);
			dprintk("___[V4L2_DEV_VIDEO1] cam_i2c_of_match[1].compatible [%s] \n", cam_i2c_of_match[1].compatible);
			break;
#endif
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO2)
		case 2:
			strcpy(cam_i2c_of_match[2].compatible, cam_name);
			break;
#endif
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO3)
		case 3:
			strcpy(cam_i2c_of_match[3].compatible, cam_name);
			break;
#endif
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO4)
		case 4:
			strcpy(cam_i2c_of_match[4].compatible, cam_name);
			break;
#endif
		default:
			printk("__%s__ set cam_i2c_name is failed!!! \n", __func__);
			return -1;
	}

	return 0;
}
EXPORT_SYMBOL(cif_set_i2c_name);

int cif_get_client(struct tcc_video_device *vdev)
{
	switch(vdev->video_nr) {
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO0)
		case 0:
			vdev->cif_i2c_client = cif_i2c_client0;
			break;
#endif
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO1)
		case 1:
			vdev->cif_i2c_client = cif_i2c_client1;
			break;
#endif
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO2)
		case 2:
			vdev->cif_i2c_client = cif_i2c_client2;
			break;
#endif
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO3)
		case 3:
			vdev->cif_i2c_client = cif_i2c_client3;
			break;
#endif
#if defined(CONFIG_TCC_V4L2_DEV_VIDEO4)
		case 4:
			vdev->cif_i2c_client = cif_i2c_client4;
			break;
#endif
		default:
			return -1;
	}
	return 0;
}
EXPORT_SYMBOL(cif_get_client);

module_init(cif_i2c_init);
module_exit(cif_i2c_exit);
