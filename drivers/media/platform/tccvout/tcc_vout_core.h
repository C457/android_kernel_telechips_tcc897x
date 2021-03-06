/*
 * tcc_vout_core.h
 *
 * Copyright (C) 2013 Telechips, Inc. 
 *
 * Video-for-Linux (Version 2) video output driver for Telechips SoC.
 * 
 * This package is free software; you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation. 
 * 
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR 
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED 
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE. 
 *   
 */
#ifndef __TCC_VOUT_CORE_H__
#define __TCC_VOUT_CORE_H__

#ifndef ADDRESS_ALIGNED
#define ADDRESS_ALIGNED
#define ALIGN_BIT (0x8-1)
#define BIT_0 3
#define GET_ADDR_YUV42X_spY(Base_addr) 		(((((unsigned int)Base_addr) + ALIGN_BIT)>> BIT_0)<<BIT_0)
#define GET_ADDR_YUV42X_spU(Yaddr, x, y) 	(((((unsigned int)Yaddr+(x*y)) + ALIGN_BIT)>> BIT_0)<<BIT_0)
#define GET_ADDR_YUV422_spV(Uaddr, x, y) 	(((((unsigned int)Uaddr+(x*y/2)) + ALIGN_BIT) >> BIT_0)<<BIT_0)
#define GET_ADDR_YUV420_spV(Uaddr, x, y) 	(((((unsigned int)Uaddr+(x*y/4)) + ALIGN_BIT) >> BIT_0)<<BIT_0)
#endif

extern int vout_get_pmap(pmap_t *pmap);
extern int vout_set_vout_path(struct tcc_vout_device *vout);
extern int vout_set_deintl_path(int deintl_default, struct tcc_vout_device *vout);
extern int vout_vioc_set_default(struct tcc_vout_device *vout);
extern int vout_vioc_init(struct tcc_vout_device *vout);
extern void vout_deinit(struct tcc_vout_device *vout);
extern void vout_disp_ctrl(struct tcc_vout_vioc *vioc, int enable);
extern void vout_chromakey_enable(struct tcc_vout_vioc *vioc);
extern void vout_rdma_setup(struct tcc_vout_device *vout);
extern void vout_scaler_setup(struct tcc_vout_device *vout);
extern void vout_wmix_setup(struct tcc_vout_device *vout);
extern void vout_wmix_getsize(struct tcc_vout_device *vout, unsigned int *w, unsigned int *h);
extern void vout_path_reset(struct tcc_vout_vioc *vioc);
/* de-interlace */
extern void deintl_path_reset(struct tcc_vout_vioc *vioc);
extern void deintl_rdma_setup(struct vioc_rdma *rdma);
extern int vout_deintl_init(struct tcc_vout_device *vout);
extern void vout_deintl_ctrl(struct tcc_vout_vioc *vioc, int enable);
extern void vout_deintl_deinit(struct tcc_vout_device *vout);
extern void vout_deintl_overlay(struct tcc_vout_device *vout);
/* sub-plane */
extern int vout_subplane_init(struct tcc_vout_device *vout);
extern void vout_subplane_deinit(struct tcc_vout_device *vout);
extern void vout_subplane_ctrl(struct tcc_vout_device *vout, int enable);
extern int vout_subplane_qbuf(struct tcc_vout_device *vout, struct vioc_alpha *alpha);
extern int vout_subplane_onthefly_init(struct tcc_vout_device *vout);
extern int vout_subplane_onthefly_qbuf(struct tcc_vout_device *vout);

/* buffer control */
extern void vout_pop_all_buffer(struct tcc_vout_device *vout);
/* streaming */
extern void vout_onthefly_display_update(struct tcc_vout_device *vout, struct v4l2_buffer *buf);
extern void vout_m2m_display_update(struct tcc_vout_device *vout, struct v4l2_buffer *buf);
#endif //__TCC_VOUT_CORE_H__
