/* tcc_drm_crtc.h
 *
 * Copyright (c) 2016 Telechips Inc.
 *
 * Author:  Telechips Inc.
 * Created: Jan 07, 2016
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 *---------------------------------------------------------------------------
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 * Authors:
 *	Inki Dae <inki.dae@samsung.com>
 *	Joonyoung Shim <jy0922.shim@samsung.com>
 *	Seung-Woo Kim <sw0312.kim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef _TCC_DRM_CRTC_H_
#define _TCC_DRM_CRTC_H_

struct drm_device;
struct drm_crtc;
struct tcc_drm_manager;
struct tcc_drm_overlay;

int tcc_drm_crtc_create(struct tcc_drm_manager *manager);
int tcc_drm_crtc_enable_vblank(struct drm_device *dev, int pipe);
void tcc_drm_crtc_disable_vblank(struct drm_device *dev, int pipe);
void tcc_drm_crtc_finish_pageflip(struct drm_device *dev, int pipe);
void tcc_drm_crtc_complete_scanout(struct drm_framebuffer *fb);

void tcc_drm_crtc_plane_mode_set(struct drm_crtc *crtc,
			struct tcc_drm_overlay *overlay);
void tcc_drm_crtc_plane_commit(struct drm_crtc *crtc, int zpos);
void tcc_drm_crtc_plane_enable(struct drm_crtc *crtc, int zpos);
void tcc_drm_crtc_plane_disable(struct drm_crtc *crtc, int zpos);

/* This function gets pipe value to crtc device matched with out_type. */
int tcc_drm_crtc_get_pipe_from_type(struct drm_device *drm_dev,
					unsigned int out_type);

/*
 * This function calls the crtc device(manager)'s te_handler() callback
 * to trigger to transfer video image at the tearing effect synchronization
 * signal.
 */
void tcc_drm_crtc_te_handler(struct drm_crtc *crtc);

#endif
