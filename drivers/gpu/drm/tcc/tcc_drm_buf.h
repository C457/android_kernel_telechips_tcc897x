/* tcc_drm_buf.h
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
 * Author: Inki Dae <inki.dae@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef _TCC_DRM_BUF_H_
#define _TCC_DRM_BUF_H_

/* create and initialize buffer object. */
struct tcc_drm_gem_buf *tcc_drm_init_buf(struct drm_device *dev,
						unsigned int size);

/* destroy buffer object. */
void tcc_drm_fini_buf(struct drm_device *dev,
				struct tcc_drm_gem_buf *buffer);

/* allocate physical memory region and setup sgt. */
int tcc_drm_alloc_buf(struct drm_device *dev,
				struct tcc_drm_gem_buf *buf,
				unsigned int flags);

/* release physical memory region, and sgt. */
void tcc_drm_free_buf(struct drm_device *dev,
				unsigned int flags,
				struct tcc_drm_gem_buf *buffer);

#endif
