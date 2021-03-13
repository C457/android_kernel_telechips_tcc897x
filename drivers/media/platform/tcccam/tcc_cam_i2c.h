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


#ifndef __TCC_CAM_I2C_H_char__
#define __TCC_CAM_I2C_H_char__

#include "sensor_if.h"

struct tcc_camera_device;

extern int DDI_I2C_Write(unsigned char* data, unsigned short reg_bytes, unsigned short data_bytes, struct tcc_camera_device * vdev);
extern int DDI_I2C_Read(unsigned short reg, unsigned char reg_bytes, unsigned char *val, unsigned short val_bytes, struct tcc_camera_device * vdev);	
#endif 

