/* 
 * linux/drivers/char/tcc_dxb/ctrl/tcc_dxb_control.h
 *
 * Author:  <linux@telechips.com>
 * Created: 10th Jun, 2008 
 * Description: Telechips Linux DxB Control DRIVER
 *
 * Copyright (c) Telechips, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef     _TCC_DXB_CONTROL_H_
#define     _TCC_DXB_CONTROL_H_

typedef enum
{
	BOARD_TDMB_TCC3150, //TCCXXXX_DMB_TCC3150_SV1.2
	BOARD_TDMB_TCC3161, //TCCXXXX_DMB_TCC3161_SV1.0
	BOARD_DVBT_DIB7070,
	BOARD_DVBT_DIB9090, //TCCXXXX_DVBT&H_9080&9090_SV1.0
	BOARD_ISDBT_DIB10096, //tcc89&93XX_ISDBT_DIB1009X_SV0.2
	BOARD_DXB_TCC3510, //TCCXXXX_DXB_TCC3510_SV5.0, OnBoard Module in TCC93XX STB
	BOARD_DVBT_DIB9090M_PA, //TCC89&91&92XX_DVB_DIB90X0_SV0.1 //PARALLEL Interface
	BOARD_ISDBT_MTV818,     //TCCXXXX_ISDB-T_MTV818_SV0.1   //usable only for TCC89xx and TCC92xx
	BOARD_DXB_NMI32X, //ISDBT_NMI325_SV0.1
	BOARD_ISDBT_TOSHIBA,
	BOARD_ISDBT_TCC353X,
	BOARD_ISDBT_TCC353X_FSMA,
	BOARD_DVBT_MXL101SF_YJ,     //Yaojin mxl101sf yaojin board
	BOARD_DVBT2_MN88472_YJ,     //Yaojin mn88472 yaojin board
	BOARD_DVBS2_AVL6211_YJ,     //Yaojin avl6211 yaojin board
	BOARD_DXB_TCC3171,
	BOARD_MAX
}DXB_BOARD_TYPE;

#define DXB_CTRL_DEV_FILE		"/dev/tcc_dxb_ctrl"
#define DXB_CTRL_DEV_NAME		"tcc_dxb_ctrl"
#define DXB_CTRL_DEV_MAJOR		251
#define DXB_CTRL_DEV_MINOR		0

#define IOCTL_DXB_CTRL_OFF		    _IO(DXB_CTRL_DEV_MAJOR, 1)
#define IOCTL_DXB_CTRL_ON			_IO(DXB_CTRL_DEV_MAJOR, 2)
#define IOCTL_DXB_CTRL_RESET    	_IO(DXB_CTRL_DEV_MAJOR, 3)
#define IOCTL_DXB_CTRL_SET_BOARD    _IO(DXB_CTRL_DEV_MAJOR, 4)
#define IOCTL_DXB_CTRL_GET_CTLINFO  _IO(DXB_CTRL_DEV_MAJOR, 5)
#define IOCTL_DXB_CTRL_RF_PATH      _IO(DXB_CTRL_DEV_MAJOR, 6)
#define IOCTL_DXB_CTRL_SET_CTRLMODE _IO(DXB_CTRL_DEV_MAJOR, 7)
#define IOCTL_DXB_CTRL_RESET_LOW	_IO(DXB_CTRL_DEV_MAJOR, 8)
#define IOCTL_DXB_CTRL_RESET_HIGH	_IO(DXB_CTRL_DEV_MAJOR, 9)
#define IOCTL_DXB_CTRL_PURE_ON		_IO(DXB_CTRL_DEV_MAJOR, 10)
#define IOCTL_DXB_CTRL_PURE_OFF		_IO(DXB_CTRL_DEV_MAJOR, 11)
/* add for HWDEMUX cipher */
#define IOCTL_DXB_CTRL_HWDEMUX_CIPHER_SET_ALGORITHM	    _IO(DXB_CTRL_DEV_MAJOR, 12)
#define IOCTL_DXB_CTRL_HWDEMUX_CIPHER_SET_KEY		    _IO(DXB_CTRL_DEV_MAJOR, 13)
#define IOCTL_DXB_CTRL_HWDEMUX_CIPHER_SET_VECTOR	    _IO(DXB_CTRL_DEV_MAJOR, 14)
#define IOCTL_DXB_CTRL_HWDEMUX_CIPHER_ENCRYPT	    _IO(DXB_CTRL_DEV_MAJOR, 15)
#define IOCTL_DXB_CTRL_HWDEMUX_CIPHER_DECRYPT	    _IO(DXB_CTRL_DEV_MAJOR, 16)
#define IOCTL_DXB_CTRL_HWDEMUX_CIPHER_EXECUTE	    _IO(DXB_CTRL_DEV_MAJOR, 17)
#define IOCTL_DXB_CTRL_UNLOCK                       _IO(DXB_CTRL_DEV_MAJOR, 18)
#define IOCTL_DXB_CTRL_LOCK                         _IO(DXB_CTRL_DEV_MAJOR, 19)

#endif
