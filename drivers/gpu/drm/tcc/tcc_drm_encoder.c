/* tcc_drm_encoder.c
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

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>

#include "tcc_drm_drv.h"
#include "tcc_drm_encoder.h"

#define to_tcc_encoder(x)	container_of(x, struct tcc_drm_encoder,\
				drm_encoder)

/*
 * tcc specific encoder structure.
 *
 * @drm_encoder: encoder object.
 * @display: the display structure that maps to this encoder
 */
struct tcc_drm_encoder {
	struct drm_encoder		drm_encoder;
	struct tcc_drm_display	*display;
};

static void tcc_drm_encoder_dpms(struct drm_encoder *encoder, int mode)
{
	struct tcc_drm_encoder *tcc_encoder = to_tcc_encoder(encoder);
	struct tcc_drm_display *display = tcc_encoder->display;

	DRM_DEBUG_KMS("encoder dpms: %d\n", mode);

	if (display->ops->dpms)
		display->ops->dpms(display, mode);
}

static bool
tcc_drm_encoder_mode_fixup(struct drm_encoder *encoder,
			       const struct drm_display_mode *mode,
			       struct drm_display_mode *adjusted_mode)
{
	struct drm_device *dev = encoder->dev;
	struct tcc_drm_encoder *tcc_encoder = to_tcc_encoder(encoder);
	struct tcc_drm_display *display = tcc_encoder->display;
	struct drm_connector *connector;

	list_for_each_entry(connector, &dev->mode_config.connector_list, head) {
		if (connector->encoder != encoder)
			continue;

		if (display->ops->mode_fixup)
			display->ops->mode_fixup(display, connector, mode,
					adjusted_mode);
	}

	return true;
}

static void tcc_drm_encoder_mode_set(struct drm_encoder *encoder,
					 struct drm_display_mode *mode,
					 struct drm_display_mode *adjusted_mode)
{
	struct tcc_drm_encoder *tcc_encoder = to_tcc_encoder(encoder);
	struct tcc_drm_display *display = tcc_encoder->display;

	if (display->ops->mode_set)
		display->ops->mode_set(display, adjusted_mode);
}

static void tcc_drm_encoder_prepare(struct drm_encoder *encoder)
{
	/* drm framework doesn't check NULL. */
}

static void tcc_drm_encoder_commit(struct drm_encoder *encoder)
{
	struct tcc_drm_encoder *tcc_encoder = to_tcc_encoder(encoder);
	struct tcc_drm_display *display = tcc_encoder->display;

	if (display->ops->dpms)
		display->ops->dpms(display, DRM_MODE_DPMS_ON);

	if (display->ops->commit)
		display->ops->commit(display);
}

static void tcc_drm_encoder_disable(struct drm_encoder *encoder)
{
	struct drm_plane *plane;
	struct drm_device *dev = encoder->dev;

	tcc_drm_encoder_dpms(encoder, DRM_MODE_DPMS_OFF);

	/* all planes connected to this encoder should be also disabled. */
	drm_for_each_legacy_plane(plane, &dev->mode_config.plane_list) {
		if (plane->crtc == encoder->crtc)
			plane->funcs->disable_plane(plane);
	}
}

static struct drm_encoder_helper_funcs tcc_encoder_helper_funcs = {
	.dpms		= tcc_drm_encoder_dpms,
	.mode_fixup	= tcc_drm_encoder_mode_fixup,
	.mode_set	= tcc_drm_encoder_mode_set,
	.prepare	= tcc_drm_encoder_prepare,
	.commit		= tcc_drm_encoder_commit,
	.disable	= tcc_drm_encoder_disable,
};

static void tcc_drm_encoder_destroy(struct drm_encoder *encoder)
{
	struct tcc_drm_encoder *tcc_encoder = to_tcc_encoder(encoder);

	drm_encoder_cleanup(encoder);
	kfree(tcc_encoder);
}

static struct drm_encoder_funcs tcc_encoder_funcs = {
	.destroy = tcc_drm_encoder_destroy,
};

static unsigned int tcc_drm_encoder_clones(struct drm_encoder *encoder)
{
	struct drm_encoder *clone;
	struct drm_device *dev = encoder->dev;
	struct tcc_drm_encoder *tcc_encoder = to_tcc_encoder(encoder);
	struct tcc_drm_display *display = tcc_encoder->display;
	unsigned int clone_mask = 0;
	int cnt = 0;

	list_for_each_entry(clone, &dev->mode_config.encoder_list, head) {
		switch (display->type) {
		case TCC_DISPLAY_TYPE_LCD:
		case TCC_DISPLAY_TYPE_HDMI:
		case TCC_DISPLAY_TYPE_VIDI:
			clone_mask |= (1 << (cnt++));
			break;
		default:
			continue;
		}
	}

	return clone_mask;
}

void tcc_drm_encoder_setup(struct drm_device *dev)
{
	struct drm_encoder *encoder;

	list_for_each_entry(encoder, &dev->mode_config.encoder_list, head)
		encoder->possible_clones = tcc_drm_encoder_clones(encoder);
}

struct drm_encoder *
tcc_drm_encoder_create(struct drm_device *dev,
			   struct tcc_drm_display *display,
			   unsigned long possible_crtcs)
{
	struct drm_encoder *encoder;
	struct tcc_drm_encoder *tcc_encoder;

	if (!possible_crtcs)
		return NULL;

	tcc_encoder = kzalloc(sizeof(*tcc_encoder), GFP_KERNEL);
	if (!tcc_encoder)
		return NULL;

	tcc_encoder->display = display;
	encoder = &tcc_encoder->drm_encoder;
	encoder->possible_crtcs = possible_crtcs;

	DRM_DEBUG_KMS("possible_crtcs = 0x%x\n", encoder->possible_crtcs);

	drm_encoder_init(dev, encoder, &tcc_encoder_funcs,
			DRM_MODE_ENCODER_TMDS);

	drm_encoder_helper_add(encoder, &tcc_encoder_helper_funcs);

	DRM_DEBUG_KMS("encoder has been created\n");

	return encoder;
}

struct tcc_drm_display *tcc_drm_get_display(struct drm_encoder *encoder)
{
	return to_tcc_encoder(encoder)->display;
}
