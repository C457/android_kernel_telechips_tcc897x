/*
 * File:        include/linux/tcc_bt_dev.h
 */
 
#ifndef __TCC_BT_DEV_H__
#define __TCC_BT_DEV_H__

#define BT_DEV_MAJOR_NUM 234
#define BT_DEV_MINOR_NUM 1

#define IOCTL_BT_DEV_POWER          	_IO(BT_DEV_MAJOR_NUM, 100)
#define IOCTL_BT_DEV_SPECIFIC       	_IO(BT_DEV_MAJOR_NUM, 101)
#define IOCTL_BT_DEV_CLOCK_LIMIT    	_IO(BT_DEV_MAJOR_NUM, 105)
#define IOCTL_BT_DEV_CTRL_INIT      	_IO(BT_DEV_MAJOR_NUM, 106)
#define IOCTL_BT_DEV_CTRL_RESET     	_IO(BT_DEV_MAJOR_NUM, 107)

#define BT_DEV_ON   	1
#define BT_DEV_OFF  	0

#define BT_DEV_GPIO_HI 	1
#define BT_DEV_GPIO_LOW	0
#endif

