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

#include <linux/fs.h>
#include <asm/uaccess.h>
#include <asm/io.h>

#include <mach/bsp.h>

#include "tcc_avn_proc.h"
#include "tcc_cam_cm_control.h"

#include <linux/gpio.h>

static int debug	= 1;
#define TAG		"tcc_avn_proc"
#define dprintk(msg...)	if(debug) { printk(TAG ": " msg); }
#define log(msg...)	if(debug) { printk(TAG ": %s - ", __func__); printk(msg); }
#define FUNCTION_IN	log("In\n");
#define FUNCTION_OUT	log("Out\n");

#define	CM_CTRL_PREAMBLE	(unsigned int)0x0043414D	// "CAM" (ascii)

typedef enum {
	DISABLE_RECOVERY	= 0x40,
	ENABLE_RECOVERY		= 0x41,
	GET_GEAR_STATUS		= 0x50,
	STOP_EARLY_CAMERA	= 0x51,
	EXIT_EARLY_CAMERA	= 0x52,
} CM_AVN_CMD;

int	pinGear;
int	pinActive;

int tcc_cm_ctrl_get_gear_status(void) {
	int gear_port = pinGear;
	int gear_value = -1;
	int ret;

	gpio_request(gear_port, NULL);
	gpio_direction_input(gear_port);
	gear_value = !!gpio_get_value(gear_port);

	ret = (gear_value == pinActive) ? 1 : 0;
	
	printk("####################################### GearStatus : %d\n", ret);

	return ret;
}
EXPORT_SYMBOL_GPL(tcc_cm_ctrl_get_gear_status);

int tcc_cm_ctrl_stop_earlycamera(void) {
	cm_ctrl_msg_t	msg;
	
	FUNCTION_IN
	
	// clear msg
	memset((void *)&msg, 0, sizeof(cm_ctrl_msg_t));
	
	// set msg
	msg.preamble	= CM_CTRL_PREAMBLE;
	msg.cmd		= STOP_EARLY_CAMERA;
	
	// send msg
	CM_SEND_COMMAND(&msg, &msg);
	
	printk(KERN_INFO "!@#---- %s - CM Last Gear Status: %d\n", __func__, msg.data[0]);
	
	FUNCTION_OUT
	return msg.data[0];
}
EXPORT_SYMBOL_GPL(tcc_cm_ctrl_stop_earlycamera);

int tcc_cm_ctrl_exit_earlycamera(void) {
	cm_ctrl_msg_t	msg;
	
	FUNCTION_IN
	
	// clear msg
	memset((void *)&msg, 0, sizeof(cm_ctrl_msg_t));
	
	// set msg
	msg.preamble	= CM_CTRL_PREAMBLE;
	msg.cmd		= EXIT_EARLY_CAMERA;
	
	// send msg
	CM_SEND_COMMAND(&msg, &msg);
	
	printk(KERN_INFO "!@#---- %s - CM Last Gear Status: %d\n", __func__, msg.data[0]);
	
	FUNCTION_OUT
	return msg.data[0];
}
EXPORT_SYMBOL_GPL(tcc_cm_ctrl_exit_earlycamera);

int tcc_cm_ctrl_disable_recovery(void) {
	cm_ctrl_msg_t	msg;
	
	FUNCTION_IN
	
	// clear msg
	memset((void *)&msg, 0, sizeof(cm_ctrl_msg_t));
	
	// set msg
	msg.preamble	= CM_CTRL_PREAMBLE;
	msg.cmd		= DISABLE_RECOVERY;
	
	// send msg
	CM_SEND_COMMAND(&msg, &msg);
	
	FUNCTION_OUT
	return msg.data[0];
}
EXPORT_SYMBOL_GPL(tcc_cm_ctrl_disable_recovery);

int tcc_cm_ctrl_enable_recovery(void) {
	cm_ctrl_msg_t	msg;
	
	FUNCTION_IN
	
	// clear msg
	memset((void *)&msg, 0, sizeof(cm_ctrl_msg_t));
	
	// set msg
	msg.preamble	= CM_CTRL_PREAMBLE;
	msg.cmd		= ENABLE_RECOVERY;
	
	// send msg
	CM_SEND_COMMAND(&msg, &msg);
	
	FUNCTION_OUT
	return msg.data[0];
}
EXPORT_SYMBOL_GPL(tcc_cm_ctrl_enable_recovery);

int CM_AVN_Proc(unsigned long arg) {
	int		msg;
	unsigned long	num_to_copy = 0;
	
	num_to_copy = copy_from_user(&msg, (int *)arg, sizeof(int));
	if(0 < num_to_copy) {
		printk("!@#---- %s() - copy_from_user() is failed(%lu)\n", __func__, num_to_copy);
		return -1;
	}
	
	printk(KERN_INFO "!@#---- %s - OP Code: 0x%08x\n", __func__, msg);
	switch(msg) {
	case GET_GEAR_STATUS:
		msg = tcc_cm_ctrl_get_gear_status();
		break;

	case STOP_EARLY_CAMERA:
		msg = tcc_cm_ctrl_stop_earlycamera();
		break;
		
	case EXIT_EARLY_CAMERA:
		msg = tcc_cm_ctrl_exit_earlycamera();
		break;
		
	case DISABLE_RECOVERY:
		msg = tcc_cm_ctrl_disable_recovery();
		break;
		
	case ENABLE_RECOVERY:
		msg = tcc_cm_ctrl_enable_recovery();
		break;
		
	default:
		printk("!@#---- %s - Unsupported Command: 0x%x\n", __func__, msg);
		break;
	}
	
	num_to_copy = copy_to_user((int *)arg, &msg, sizeof(int));
	if(0 < num_to_copy) {
		printk("!@#---- %s() - copy_to_user() is failed(%lu)\n", __func__, num_to_copy);
		return -1;
	}
	return 0;
}

