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

#ifndef TCC_HDIN_VIDEO_H
#define TCC_HDIN_VIDEO_H

struct VIOC_dt_parse{
	unsigned int  index;
};

struct VIOC_Index {
	struct VIOC_dt_parse	rdma;
	struct VIOC_dt_parse	wmixer;
	struct VIOC_dt_parse	vin;
	struct VIOC_dt_parse	wdma;
	struct VIOC_dt_parse	scaler;
	struct VIOC_dt_parse	viqe;
	struct VIOC_dt_parse	config;
};

extern int	hdin_video_buffer_set(struct tcc_hdin_device *hdev, struct v4l2_requestbuffers *req);
extern int  hdin_video_start_stream(struct tcc_hdin_device *hdev);
extern int  hdin_video_stop_stream(struct tcc_hdin_device *hdev);
extern int 	hdin_video_set_resolution(struct tcc_hdin_device *hdev, unsigned int pixel_fmt, unsigned short width, unsigned short height);
extern int 	hdin_video_open(struct tcc_hdin_device *hdev);
extern int 	hdin_video_close(struct tcc_hdin_device *hdev);
extern int 	hdin_video_init(struct tcc_hdin_device *hdev);
extern void hdin_video_set_addr(struct tcc_hdin_device *hdev, int index, unsigned int addr);
extern void hdin_dma_hw_reg(struct TCC_HDIN *h, VIOC_WDMA *pWDMA, unsigned char frame_num);
#endif
