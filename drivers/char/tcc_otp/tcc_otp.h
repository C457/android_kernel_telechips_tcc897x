/* linux/drivers/char/tcc_otp.h
 *
 * Copyright (C) 2009, 2010 Telechips, Inc.
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
 */

#ifndef _TCC_OTP_H_
#define _TCC_OTP_H_

#define DEVICE_NAME	"tcc-otp"

enum tcc_otp_ioctl_cmd {
	TCC_OTP_IOCTL_READ,
	TCC_OTP_IOCTL_WRITE,
	TCC_OTP_IOCTL_MAX
};

struct tcc_otp_ioctl_param {
	unsigned int *addr;
	unsigned int *buf;
	unsigned int size;
};

#endif /*_TCC_OTP_H_*/
