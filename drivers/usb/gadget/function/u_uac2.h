/*
 * u_uac2.h
 *
 * Utility definitions for UAC2 function
 *
 * Copyright (c) 2014 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Author: Andrzej Pietrasiewicz <andrzej.p@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef U_UAC2_H
#define U_UAC2_H

#include <linux/usb/composite.h>

#define UAC2_DEF_PCHMASK 0x3
#define UAC2_DEF_PSRATE 44100//48000 //changed for iOS10
#define UAC2_DEF_PSSIZE 2
#define UAC2_DEF_CCHMASK 0x3
#define UAC2_DEF_CSRATE 44100 // original value : 64000
#define UAC2_DEF_CSSIZE 2

struct f_uac2_opts {
	struct usb_function_instance	func_inst;
	int				p_chmask;
	int				p_srate;
	int				p_ssize;
	int				c_chmask;
	int				c_srate;
	int				c_ssize;
	bool				bound;

	struct mutex			lock;
	int				refcnt;
};

extern int complete_count;
extern unsigned int ringbuff_offset;
extern int capture_pcm;
extern char dump_path[30];
extern char* pdump_path;


#endif
