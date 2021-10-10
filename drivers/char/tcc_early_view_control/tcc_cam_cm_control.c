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

#include <linux/module.h>
#include <linux/delay.h>
#include <mach/io.h>
#include <linux/gpio.h>
#include <mach/bsp.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/cpufreq.h>
#include <linux/err.h>

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>

#include <asm/mach-types.h>

#include <linux/fs.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/platform_device.h>

#include "tcc_cam_cm_control.h"
#include "tcc_avn_proc.h"
#include <linux/of_gpio.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#endif

#define CM_CTRL_DEV_NAME        "tcc_cm_ctrl"
#define CM_CTRL_DEV_MAJOR       221
#define CM_CTRL_DEV_MINOR       0

#define CM_CTRL_IOCTL_OFF	0
#define CM_CTRL_IOCTL_ON	1
#define CM_CTRL_IOCTL_CMD	3
#define CM_CTRL_IOCTL_KNOCK	4

#define DEVICE_NAME             "mailbox"

#define MBOX_IRQ                INT_CMBUS_MBOX0

#define CM_USE_FOR_AVN

static struct device *	pdev_cm	= NULL;
static struct clk *	cm_clk	= NULL;
static int cmbus_irq;

static CM_TSD_CFG	* pTsdCfg   = NULL;

static unsigned int	addrCodeMem = 0;
static unsigned int	addrDataMem = 0;

static MAILBOX		* pMboxMain = NULL;
static MAILBOX		* pMboxSub  = NULL;

DECLARE_WAIT_QUEUE_HEAD(wq_cm_cmd);
static int cm_cmd_status = 0;
static int retData[8];
static void CM_MailBox_Configure(void);
static irqreturn_t CM_MailBox_Handler(int irq, void * dev);

static int debug	= 1;
#define TAG		"tcc_cam_cm_ctrl"
#define dprintk(msg...)	if(debug) { printk(TAG ": " msg); }
#define log(msg...)	if(debug) { printk(TAG ": %s - ", __func__); printk(msg); }
#define FUNCTION_IN	log("In\n");
#define FUNCTION_OUT	log("Out\n");

int tcc_mbox_receive_msg(cm_ctrl_msg_t * msg) {
	volatile MAILBOX	* pMbox	= pMboxMain;//(volatile MAILBOX *)HwCORTEXM4_MAILBOX1_BASE;
	unsigned int		scount	= 0;
	
	// Data received
	// Wait until data received...
	// If "SEMP" is low, the message has been arrived and than "SCOUNT"
	// indicates the total number of received messages in word unit.
//	scount = (unsigned int)pMbox->uMBOX_STATUS.bREG.SCOUNT;
	scount = (unsigned int)((pMbox->uMBOX_CTL_017.nREG >> 20) & 0xF);
//	printk("[KN] %s - STATUS[0x%08x]: 0x%08x, SCOUNT: %d\n", __func__, (unsigned int)&pMbox->uMBOX_STATUS, (unsigned int)pMbox->uMBOX_STATUS.nREG, scount);
//	printk("[KN] %s - STATUS[0x%08x]: 0x%08x, SCOUNT: %d\n", __func__, (unsigned int)&pMbox->uMBOX_CTL_017, (unsigned int)pMbox->uMBOX_CTL_017.nREG, scount);
	if(scount != 8) {
		return -1;
	}
	
	// Read Data from FIFO
	// The sequence of read operation can be replaced to burst read to the receive fifo data region
	// After reading done, the empty status of the counter-part goes to "HIGH" means empty
	memcpy((void *)msg, (const void *)&pMbox->uMBOX_RX0, sizeof(cm_ctrl_msg_t));
	
	return 0;
}

unsigned int tcc_mbox_wait_receive_msg(cm_ctrl_msg_t * msg_to_check) {
	unsigned int	ack_to_check	= msg_to_check->cmd | MAILBOX_MSG_ACK;
	cm_ctrl_msg_t	msg_to_receive;
	int		ret;
	
	while(1) {
		ret = tcc_mbox_receive_msg(&msg_to_receive);
		if(!ret) {
			printk("[KN] Rx: 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x\n",	\
				msg_to_receive.preamble,	\
				msg_to_receive.data[5],		\
				msg_to_receive.data[4],		\
				msg_to_receive.data[3], 	\
				msg_to_receive.data[2],		\
				msg_to_receive.data[1],		\
				msg_to_receive.data[0],		\
				msg_to_receive.cmd);
			if(msg_to_receive.cmd == ack_to_check)
				break;
		}
	}
	return msg_to_receive.data[0];
}

int tcc_mbox_send_msg(/*const */cm_ctrl_msg_t * msg) {
	volatile MAILBOX	* pMbox	 = pMboxMain;//(volatile MAILBOX *)HwCORTEXM4_MAILBOX1_BASE;
	
	// set preamble
	msg->preamble	= MAILBOX_MSG_PREAMBLE;
	
	// Flush transmit FIFO or Wait until empty of transmit FIFO
	// Can be skipped by case
	
	// Set "OEN" to Low
	// To protect the invalid event which can be generated during writing data
	pMbox->uMBOX_CTL_016.bREG.OEN = 0;
	
	// Write Data to FIFO
	// The sequence of writing operation can be replaced to burst writing to the transmit fifo data region
	log("[KN] Tx: 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x\n",      \
		msg->preamble,		\
		msg->data[5],		\
		msg->data[4],		\
		msg->data[3],		\
		msg->data[2],		\
		msg->data[1],		\
		msg->data[0],		\
		msg->cmd);
	memcpy((void *)&pMbox->uMBOX_TX0, (const void *)msg, sizeof(cm_ctrl_msg_t));
	
	// Set "OEN" to HIGH
	// After this, the event(interrupt) for "SendMessage" will occur to counter-part
	pMbox->uMBOX_CTL_016.bREG.OEN = 1;
	
	// Check "MEMPTY" being true
//	while(pMbox->uMBOX_STATUS.bREG.MEMP == 0);
	
	return 0;
}

int CM_SEND_COMMAND(cm_ctrl_msg_t * pTxMsg, cm_ctrl_msg_t * pRxMsg) {
	int		ret = -1;
	
	CM_MailBox_Configure();
	
	ret = request_irq(cmbus_irq, CM_MailBox_Handler, IRQ_TYPE_LEVEL_LOW | IRQF_DISABLED, DEVICE_NAME, NULL);
	if(ret < 0) {
		printk("%s - ERROR : request_irq %d\n", __func__, ret);
		return -1;
	}
	
	log("CM REQ MSG - 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x\n", \
		pTxMsg->preamble, pTxMsg->data[5], pTxMsg->data[4], pTxMsg->data[3], pTxMsg->data[2], pTxMsg->data[1], pTxMsg->data[0], pTxMsg->cmd);
	tcc_mbox_send_msg(pTxMsg);
	
	cm_cmd_status = 1;
	#if 1
	ret = wait_event_interruptible_timeout(wq_cm_cmd, cm_cmd_status == 0, msecs_to_jiffies(1000)) ;
	#else
	ret = wait_event_interruptible(wq_cm_cmd, cm_cmd_status == 0);
	#endif
	if(ret <= 0) {
		printk("%s - wait_event_interruptible_timeout: %d\n", __func__, ret);
	}
	
	pRxMsg->data[0] = retData[1];
	log("CM ACK MSG - 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x\n", \
		pRxMsg->preamble, pRxMsg->data[5], pRxMsg->data[4], pRxMsg->data[3], pRxMsg->data[2], pRxMsg->data[1], pRxMsg->data[0], pRxMsg->cmd);
	
	free_irq(cmbus_irq, NULL);
	
	return 0;
}

static irqreturn_t CM_MailBox_Handler(int irq, void * dev) {
	// get mailbox data
	memcpy(retData, &(pMboxMain->uMBOX_RX0), sizeof(int) * 8);
	
	log("MBOX Rx - 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x\n", \
		retData[7], retData[6], retData[5], retData[4], retData[3], retData[2], retData[1], retData[0]);
	
	cm_cmd_status = 0;
	wake_up_interruptible(&wq_cm_cmd);
	
	return IRQ_HANDLED;
}

/*****************************************************************************
* Function Name : static void CM_UnloadBinary(void);
* Description : CM Code Un-Loading
* Arguments :
******************************************************************************/
static void CM_UnloadBinary(void) {
	FUNCTION_IN
	
	BITSET(pTsdCfg->CM_RESET.nREG, Hw1|Hw2);	// cm no reset
	
	// To clear all code memory.
	#if !defined(CONFIG_TCC_EARLY_CAMERA_CPU) && !defined(CONFIG_TCC_REAR_CAMERA_DRV)
	#if defined(CONFIG_ARCH_TCC802X)
	memset((void *)addrCodeMem, 0, 0x20000);	// CODE_MEM_SIZE == 128KB on CM4
	#else
	memset((void *)addrCodeMem, 0, 0x10000);	// CODE_MEM_SIZE == 64KB on CM4
	#endif
	#endif

	FUNCTION_OUT
}

/*****************************************************************************
* Function Name : static void CM_LoadBinary(void);
* Description : CM Code Loading
* Arguments :
*
* This function is not used, but remained for debug.
* It's disabled not to cause warning during compile.
******************************************************************************/
#if 0
static void CM_LoadBinary(void * binary, unsigned size) {
	FUNCTION_IN
	
	CM_UnloadBinary();
	
	if(binary && size > 0)
		memcpy((void *)addrCodeMem, binary, size);
	else
		log("Using previous loading the firmware\n");

	BITCLR(pTsdCfg->CM_RESET.nREG, Hw1|Hw2);	// cm reset
	
	FUNCTION_OUT
}
#endif

/*****************************************************************************
* Function Name : static void MailBox_Configure(void);
* Description : MailBox register init
* Arguments :
******************************************************************************/
static void CM_MailBox_Configure(void) {
	BITSET(pMboxMain->uMBOX_CTL_016.nREG, Hw0|Hw1|Hw4|Hw5|Hw6);
	BITSET(pMboxSub->uMBOX_CTL_016.nREG, Hw5|Hw6);
	BITSET(pTsdCfg->IRQ_MASK_POL.nREG, Hw16|Hw22); //IRQ_MASK_POL, IRQ, FIQ(CHANGE POLARITY)
}

int tcc_cm_ctrl_off(void) {
	if(IS_ERR_OR_NULL(cm_clk)) {
		printk(KERN_INFO "!@#---- %s - ERROR: cm clock is null\n", __func__);
		return -ENXIO;
	} else {
		// unload image
		CM_UnloadBinary();

		// clock off
		clk_disable_unprepare(cm_clk);
		clk_put(cm_clk);
		cm_clk = NULL;
	}
	return 0;
}

int tcc_cm_ctrl_on(void) {
	if(IS_ERR_OR_NULL(cm_clk)) {
		printk(KERN_INFO "!@#---- %s - ERROR: cm clock is null\n", __func__);
		return -ENXIO;
	} else {
		clk_prepare_enable(cm_clk);
		clk_set_rate(cm_clk, 100 * 1000 * 1000);
		//msleep(100);	// Wait for CM Booting
	}
	return 0;
}
		
int tcc_cm_ctrl_knock(void) {
	int status = !IS_ERR_OR_NULL(cm_clk);//(pTsdCfg->CM_RESET.nREG & (Hw1|Hw2)) ? 0 : 1;
	
	printk(KERN_INFO "!@#---- %s - CM is %s working\n", __func__, (status ? "" : "NOT"));
	
	return status;
}
EXPORT_SYMBOL_GPL(tcc_cm_ctrl_knock);

static int tcc_cm_ctrl_open(struct inode * inode, struct file * filp) {
	return 0;
}

static int tcc_cm_ctrl_release(struct inode * inode, struct file * filp) {
	return 0;
}

static long tcc_cm_ctrl_ioctl(struct file * filp, unsigned int cmd, unsigned long arg) {
	int ret = 0;

	printk(KERN_INFO "%s() - CMD = 0x%x\n", __FUNCTION__, cmd);
	switch(cmd) {
	case CM_CTRL_IOCTL_OFF:
		tcc_cm_ctrl_off();
		break;

	case CM_CTRL_IOCTL_ON:
		tcc_cm_ctrl_on();
		break;

	case CM_CTRL_IOCTL_CMD:
		ret = (* CM_AVN_Proc)(arg);
		break;
		
	case CM_CTRL_IOCTL_KNOCK:
		ret = tcc_cm_ctrl_knock();
		break;
		
	default:
		printk("cm error: unrecognized ioctl (0x%x)\n", cmd); 
		return -EINVAL;
	}

	return ret;
}

struct file_operations tcc_cm_ctrl_fops = {
	.owner          = THIS_MODULE,
	.open           = tcc_cm_ctrl_open,
	.release        = tcc_cm_ctrl_release,
	.unlocked_ioctl = tcc_cm_ctrl_ioctl,
};

static struct class * tcc_cm_ctrl_class;

static int tcc_cm_probe(struct platform_device * pdev) {
	struct device_node * np_phandle;
	int idx = 0;
	int ret = 0;

	printk(KERN_INFO "%s\n", __func__);

	if((ret = register_chrdev(CM_CTRL_DEV_MAJOR, CM_CTRL_DEV_NAME, &tcc_cm_ctrl_fops)) < 0)
		return ret;

	tcc_cm_ctrl_class = class_create(THIS_MODULE, CM_CTRL_DEV_NAME);
	if(NULL == device_create(tcc_cm_ctrl_class, NULL, MKDEV(CM_CTRL_DEV_MAJOR, CM_CTRL_DEV_MINOR), NULL, CM_CTRL_DEV_NAME))
		printk(KERN_INFO "%s device_create failed\n", __FUNCTION__);	

	pdev_cm = &pdev->dev;
	
	cm_clk = of_clk_get(pdev->dev.of_node, 0);
	if(IS_ERR(cm_clk)) {
		dev_err(&pdev->dev, "failed to get cm_clk\n");
		ret = -ENXIO;
		return -1;
	}
	clk_prepare_enable(cm_clk);
	clk_set_rate(cm_clk, 100 * 1000 * 1000);
	
	// parse dtb
	idx = of_property_match_string(pdev->dev.of_node, "reg-names", "tsd_cfg");
	pTsdCfg = (CM_TSD_CFG *)of_iomap(pdev->dev.of_node, idx);
	log("pTsdCfg: 0x%08x\n", (unsigned int)pTsdCfg);
	
	idx = of_property_match_string(pdev->dev.of_node, "reg-names", "code_mem");
	addrCodeMem = (unsigned int)of_iomap(pdev->dev.of_node, idx);
	log("addrCodeMem: 0x%08x\n", (unsigned int)addrCodeMem);
	
	idx = of_property_match_string(pdev->dev.of_node, "reg-names", "data_mem");
	addrDataMem = (unsigned int)of_iomap(pdev->dev.of_node, idx);
	log("addrDataMem: 0x%08x\n", (unsigned int)addrDataMem);
	
	idx = of_property_match_string(pdev->dev.of_node, "reg-names", "mbox_0");
	pMboxMain = (MAILBOX *)of_iomap(pdev->dev.of_node, idx);
	log("pMboxMain: 0x%08x\n", (unsigned int)pMboxMain);
	
	idx = of_property_match_string(pdev->dev.of_node, "reg-names", "mbox_1");
	pMboxSub = (MAILBOX *)of_iomap(pdev->dev.of_node, idx);
	log("pMboxSub: 0x%08x\n", (unsigned int)pMboxSub);
	
	// Reverse Gear
	pinGear = of_get_named_gpio(pdev->dev.of_node, "gear-gpios", 0);
	of_property_read_u32_index(pdev->dev.of_node, "gear-active", 0, &pinActive);

	np_phandle = of_parse_phandle(pdev->dev.of_node, "tcc_cm_bus", 0);
	if(!np_phandle) {
		ret = -ENODEV;
		printk(KERN_ERR "could not find tcc_cm_bus\n");
	}

	cmbus_irq = irq_of_parse_and_map(np_phandle, 0);
	printk(" >> cmbus_irq num = %d\n", cmbus_irq);

	return 0;
}

static int tcc_cm_remove(struct platform_device * pdev) {
	printk(KERN_INFO "%s\n", __func__);

	device_destroy(tcc_cm_ctrl_class, MKDEV(CM_CTRL_DEV_MAJOR, CM_CTRL_DEV_MINOR));
	class_destroy(tcc_cm_ctrl_class);
	unregister_chrdev(CM_CTRL_DEV_MAJOR, CM_CTRL_DEV_NAME);
	
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id tcc_cm_ctrl_of_match[] = {
        {.compatible = "telechips,tcc_cm_ctrl", },
        { },
};
MODULE_DEVICE_TABLE(of, tcc_cm_ctrl_of_match);
#endif

static struct platform_driver tcc_cm_ctrl = {
	.probe	= tcc_cm_probe,
	.remove	= tcc_cm_remove,
	.driver	= {
		.name	= CM_CTRL_DEV_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = tcc_cm_ctrl_of_match,
#endif
	},
};

static int __init tcc_cm_ctrl_init(void) {
	return platform_driver_register(&tcc_cm_ctrl);
}

static void __exit tcc_cm_ctrl_exit(void) {
	platform_driver_unregister(&tcc_cm_ctrl);
}

subsys_initcall(tcc_cm_ctrl_init);
//module_init(tcc_cm_ctrl_init);
module_exit(tcc_cm_ctrl_exit);

MODULE_AUTHOR("Telechips Inc.");
MODULE_DESCRIPTION("TCC Cortex M Control(Power, Reset..)");
MODULE_LICENSE("GPL");

