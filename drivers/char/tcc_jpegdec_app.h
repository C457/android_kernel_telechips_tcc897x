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

#ifndef _JPEG_APP_DEC_H_
#define	_JPEG_APP_DEC_H_

#include "tcc_jpeg.h"

extern int JPEG_DECODE_Start(JpegOperationType type);
extern JPEG_PARSER_STATUS JPEG_IMAGE_Parser(unsigned int file_length, unsigned int *BitStreamBuff, unsigned char IsEhiInterface, jpeg_parser_rsp *JpegImageInfo);
extern unsigned int JPEG_Dec_Scale_Set(unsigned short usWidth, unsigned short usHeight, unsigned char ImageFormat);
extern unsigned int JPEG_Dec_IFrame_Width(unsigned char ucScale, unsigned short usJpegHsize);
extern unsigned int JPEG_Dec_IBuf_Height(unsigned char ucScale, unsigned short usJpegVsize);
extern JPEG_PARSER_STATUS ImagePaddingForDecode(unsigned int Chroma, unsigned int ImgWidth, unsigned int ImgHeight,
								unsigned int *NeedsPadding, unsigned int *pad_width, unsigned int *pad_height);
extern int JPEG_SET_Decode_Config (JPEG_DEC_INFO *jpeg_dec_info);
extern int JPEG_RESIZE_For_Thumbnail(unsigned src_width, unsigned src_height, unsigned int src_fmt);
extern int JPEG_DECODE_For_Thumbnail(unsigned int data_size);

#endif /* _JPEG_APP_DEC_H_ */
