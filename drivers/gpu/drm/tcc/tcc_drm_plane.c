/*
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
 * Copyright (C) 2011 Samsung Electronics Co.Ltd
 * Authors: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <drm/drmP.h>

#include <drm/tcc_drm.h>
#include "tcc_drm_drv.h"
#include "tcc_drm_crtc.h"
#include "tcc_drm_fb.h"
#include "tcc_drm_gem.h"
#include "tcc_drm_plane.h"

#define to_tcc_plane(x)	container_of(x, struct tcc_plane, base)

struct tcc_plane {
	struct drm_plane		base;
	struct tcc_drm_overlay	overlay;
	bool				enabled;
};

static const uint32_t formats[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_NV12,
	DRM_FORMAT_NV12MT,
};

/*
 * This function is to get X or Y size shown via screen. This needs length and
 * start position of CRTC.
 *
 *      <--- length --->
 * CRTC ----------------
 *      ^ start        ^ end
 *
 * There are six cases from a to f.
 *
 *             <----- SCREEN ----->
 *             0                 last
 *   ----------|------------------|----------
 * CRTCs
 * a -------
 *        b -------
 *        c --------------------------
 *                 d --------
 *                           e -------
 *                                  f -------
 */
static int tcc_plane_get_size(int start, unsigned length, unsigned last)
{
	int end = start + length;
	int size = 0;

	if (start <= 0) {
		if (end > 0)
			size = min_t(unsigned, end, last);
	} else if (start <= last) {
		size = min_t(unsigned, last - start, length);
	}

	return size;
}

int tcc_plane_mode_set(struct drm_plane *plane, struct drm_crtc *crtc,
			  struct drm_framebuffer *fb, int crtc_x, int crtc_y,
			  unsigned int crtc_w, unsigned int crtc_h,
			  uint32_t src_x, uint32_t src_y,
			  uint32_t src_w, uint32_t src_h)
{
	struct tcc_plane *tcc_plane = to_tcc_plane(plane);
	struct tcc_drm_overlay *overlay = &tcc_plane->overlay;
	unsigned int actual_w;
	unsigned int actual_h;
	int nr;
	int i;

	nr = tcc_drm_fb_get_buf_cnt(fb);
	for (i = 0; i < nr; i++) {
		struct tcc_drm_gem_buf *buffer = tcc_drm_fb_buffer(fb, i);

		if (!buffer) {
			DRM_DEBUG_KMS("buffer is null\n");
			return -EFAULT;
		}

		overlay->dma_addr[i] = buffer->dma_addr;

		DRM_DEBUG_KMS("buffer: %d, dma_addr = 0x%lx\n",
				i, (unsigned long)overlay->dma_addr[i]);
	}

	actual_w = tcc_plane_get_size(crtc_x, crtc_w, crtc->mode.hdisplay);
	actual_h = tcc_plane_get_size(crtc_y, crtc_h, crtc->mode.vdisplay);

	if (crtc_x < 0) {
		if (actual_w)
			src_x -= crtc_x;
		crtc_x = 0;
	}

	if (crtc_y < 0) {
		if (actual_h)
			src_y -= crtc_y;
		crtc_y = 0;
	}

	/* set drm framebuffer data. */
	overlay->fb_x = src_x;
	overlay->fb_y = src_y;
	overlay->fb_width = fb->width;
	overlay->fb_height = fb->height;
	overlay->src_width = src_w;
	overlay->src_height = src_h;
	overlay->bpp = fb->bits_per_pixel;
	overlay->pitch = fb->pitches[0];
	overlay->pixel_format = fb->pixel_format;

	/* set overlay range to be displayed. */
	overlay->crtc_x = crtc_x;
	overlay->crtc_y = crtc_y;
	overlay->crtc_width = actual_w;
	overlay->crtc_height = actual_h;

	/* set drm mode data. */
	overlay->mode_width = crtc->mode.hdisplay;
	overlay->mode_height = crtc->mode.vdisplay;
	overlay->refresh = crtc->mode.vrefresh;
	overlay->scan_flag = crtc->mode.flags;

	DRM_DEBUG_KMS("overlay : offset_x/y(%d,%d), width/height(%d,%d)\n",
			overlay->crtc_x, overlay->crtc_y,
			overlay->crtc_width, overlay->crtc_height);

	plane->crtc = crtc;

	tcc_drm_crtc_plane_mode_set(crtc, overlay);

	return 0;
}

void tcc_plane_commit(struct drm_plane *plane)
{
	struct tcc_plane *tcc_plane = to_tcc_plane(plane);
	struct tcc_drm_overlay *overlay = &tcc_plane->overlay;

	tcc_drm_crtc_plane_commit(plane->crtc, overlay->zpos);
}

void tcc_plane_dpms(struct drm_plane *plane, int mode)
{
	struct tcc_plane *tcc_plane = to_tcc_plane(plane);
	struct tcc_drm_overlay *overlay = &tcc_plane->overlay;

	if (mode == DRM_MODE_DPMS_ON) {
		if (tcc_plane->enabled)
			return;

		tcc_drm_crtc_plane_enable(plane->crtc, overlay->zpos);
		tcc_plane->enabled = true;
	} else {
		if (!tcc_plane->enabled)
			return;

		tcc_drm_crtc_plane_disable(plane->crtc, overlay->zpos);
		tcc_plane->enabled = false;
	}
}

static int
tcc_update_plane(struct drm_plane *plane, struct drm_crtc *crtc,
		     struct drm_framebuffer *fb, int crtc_x, int crtc_y,
		     unsigned int crtc_w, unsigned int crtc_h,
		     uint32_t src_x, uint32_t src_y,
		     uint32_t src_w, uint32_t src_h)
{
	int ret;

	ret = tcc_plane_mode_set(plane, crtc, fb, crtc_x, crtc_y,
			crtc_w, crtc_h, src_x >> 16, src_y >> 16,
			src_w >> 16, src_h >> 16);
	if (ret < 0)
		return ret;

	tcc_plane_commit(plane);
	tcc_plane_dpms(plane, DRM_MODE_DPMS_ON);

	return 0;
}

static int tcc_disable_plane(struct drm_plane *plane)
{
	tcc_plane_dpms(plane, DRM_MODE_DPMS_OFF);

	return 0;
}

static void tcc_plane_destroy(struct drm_plane *plane)
{
	struct tcc_plane *tcc_plane = to_tcc_plane(plane);

	tcc_disable_plane(plane);
	drm_plane_cleanup(plane);
	kfree(tcc_plane);
}

static int tcc_plane_set_property(struct drm_plane *plane,
				     struct drm_property *property,
				     uint64_t val)
{
	struct drm_device *dev = plane->dev;
	struct tcc_plane *tcc_plane = to_tcc_plane(plane);
	struct tcc_drm_private *dev_priv = dev->dev_private;

	if (property == dev_priv->plane_zpos_property) {
		tcc_plane->overlay.zpos = val;
		return 0;
	}

	return -EINVAL;
}

static struct drm_plane_funcs tcc_plane_funcs = {
	.update_plane	= tcc_update_plane,
	.disable_plane	= tcc_disable_plane,
	.destroy	= tcc_plane_destroy,
	.set_property	= tcc_plane_set_property,
};

static void tcc_plane_attach_zpos_property(struct drm_plane *plane, int zpos)
{
	struct drm_device *dev = plane->dev;
	struct tcc_drm_private *dev_priv = dev->dev_private;
	struct drm_property *prop;

	prop = dev_priv->plane_zpos_property;
	if (!prop) {
		prop = drm_property_create_range(dev, 0, "zpos", 0,
						 MAX_PLANE - 1);
		if (!prop)
			return;

		dev_priv->plane_zpos_property = prop;
	}

	drm_object_attach_property(&plane->base, prop, zpos);
}

struct drm_plane *tcc_plane_init(struct drm_device *dev,
				    unsigned long possible_crtcs,
				    enum drm_plane_type type, int zpos)
{
	struct tcc_plane *tcc_plane;
	int err;

	tcc_plane = kzalloc(sizeof(struct tcc_plane), GFP_KERNEL);
	if (!tcc_plane)
		return ERR_PTR(-ENOMEM);

	err = drm_universal_plane_init(dev, &tcc_plane->base, possible_crtcs,
				       &tcc_plane_funcs, formats,
				       ARRAY_SIZE(formats), type);
	if (err) {
		DRM_ERROR("failed to initialize plane\n");
		kfree(tcc_plane);
		return ERR_PTR(err);
	}

	if (type == DRM_PLANE_TYPE_PRIMARY)
		tcc_plane->overlay.zpos = DEFAULT_ZPOS;
	else
	{
		tcc_plane_attach_zpos_property(&tcc_plane->base, zpos);
		tcc_plane->overlay.zpos = zpos;
	}

	return &tcc_plane->base;
}
