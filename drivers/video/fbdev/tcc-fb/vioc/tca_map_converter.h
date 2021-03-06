/****************************************************************************
FileName    : kernel/drivers/video/tcc/vioc/tca_map_converter.h
Description : 

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
 
#ifndef	_TCA_MAP_CONVERTER_H_
#define	_TCA_MAP_CONVERTER_H_  

extern void tca_map_convter_set(unsigned int mc_num, struct tcc_lcdc_image_update *ImageInfo);
extern void tca_map_convter_driver_set(unsigned int mc_num, unsigned int Fwidth, unsigned int Fheight, unsigned int pos_x, unsigned int pos_y, unsigned int y2r, hevc_dec_MapConv_info_t *mapConv_info);
extern void tca_map_convter_onoff(unsigned int mc_num, unsigned int onoff);
extern void tca_map_convter_swreset(unsigned int mc_num);
extern void tca_map_convter_wait_done(unsigned int mc_num);
#endif //_TCA_MAP_CONVERTER_H_
