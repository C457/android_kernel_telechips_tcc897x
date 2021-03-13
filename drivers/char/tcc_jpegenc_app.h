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

#ifndef _JPEG_APP_ENC_H_
#define	_JPEG_APP_ENC_H_

#include "tcc_jpeg.h"


extern jpeg_encode_option_type gEncodeOption;

extern void JPEG_Exif_Header_Info_Init(void);
extern int JPEG_ENCODE_Get_Result(int ImageWidth, int ImageHeight, unsigned int *uiBitStreamSize, unsigned int *uiHeaderSize);
extern void EXIF_Set_GPS_Position(exif_gps_position_type *position);
extern int Make_JPEG_Enc_Info(unsigned int jpeg_BufferAddr, unsigned int Width, unsigned int Height, unsigned short factor);
extern int TCCXXX_JPEG_Make_Header(unsigned int jpeg_BufferAddr, jpeg_encode_option_type *EncodeOption, jpeg_header_exif_rsp *jpeg_header_rsp);
extern int JPEG_Make_ExifHeader(TCCXXX_JPEG_ENC_EXIF_DATA *thumb_info, unsigned int virt_baseaddr);

#endif /* _JPEG_APP_ENC_H_ */
