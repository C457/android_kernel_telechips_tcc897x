/* tcc_drm_dmabuf.h
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
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 * Author: Inki Dae <inki.dae@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef _TCC_DRM_DMABUF_H_
#define _TCC_DRM_DMABUF_H_

#ifdef CONFIG_DRM_TCC_DMABUF
struct dma_buf *tcc_dmabuf_prime_export(struct drm_device *drm_dev,
				struct drm_gem_object *obj, int flags);

struct drm_gem_object *tcc_dmabuf_prime_import(struct drm_device *drm_dev,
						struct dma_buf *dma_buf);
#else
#define tcc_dmabuf_prime_export		NULL
#define tcc_dmabuf_prime_import		NULL
#endif
#endif
