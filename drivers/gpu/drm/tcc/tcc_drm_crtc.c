/* tcc_drm_crtc.c
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

#include "tcc_drm_crtc.h"
#include "tcc_drm_drv.h"
#include "tcc_drm_encoder.h"
#include "tcc_drm_plane.h"

#define to_tcc_crtc(x)	container_of(x, struct tcc_drm_crtc,\
				drm_crtc)

enum tcc_crtc_mode {
	CRTC_MODE_NORMAL,	/* normal mode */
	CRTC_MODE_BLANK,	/* The private plane of crtc is blank */
};

/*
 * Exynos specific crtc structure.
 *
 * @drm_crtc: crtc object.
 * @manager: the manager associated with this crtc
 * @pipe: a crtc index created at load() with a new crtc object creation
 *	and the crtc object would be set to private->crtc array
 *	to get a crtc object corresponding to this pipe from private->crtc
 *	array when irq interrupt occurred. the reason of using this pipe is that
 *	drm framework doesn't support multiple irq yet.
 *	we can refer to the crtc to current hardware interrupt occurred through
 *	this pipe value.
 * @dpms: store the crtc dpms value
 * @mode: store the crtc mode value
 */
struct tcc_drm_crtc {
	struct drm_crtc			drm_crtc;
	struct tcc_drm_manager	*manager;
	unsigned int			pipe;
	unsigned int			dpms;
	enum tcc_crtc_mode		mode;
	wait_queue_head_t		pending_flip_queue;
	atomic_t			pending_flip;
};

static void tcc_drm_crtc_dpms(struct drm_crtc *crtc, int mode)
{
	struct tcc_drm_crtc *tcc_crtc = to_tcc_crtc(crtc);
	struct tcc_drm_manager *manager = tcc_crtc->manager;

	DRM_DEBUG_KMS("crtc[%d] mode[%d]\n", crtc->base.id, mode);

	if (tcc_crtc->dpms == mode) {
		DRM_DEBUG_KMS("desired dpms mode is same as previous one.\n");
		return;
	}

	if (mode > DRM_MODE_DPMS_ON) {
		/* wait for the completion of page flip. */
		if (!wait_event_timeout(tcc_crtc->pending_flip_queue,
				!atomic_read(&tcc_crtc->pending_flip),
				HZ/20))
			atomic_set(&tcc_crtc->pending_flip, 0);
		drm_crtc_vblank_off(crtc);
	}

	if (manager->ops->dpms)
		manager->ops->dpms(manager, mode);

	tcc_crtc->dpms = mode;

	if (mode == DRM_MODE_DPMS_ON)
		drm_crtc_vblank_on(crtc);
}

static void tcc_drm_crtc_prepare(struct drm_crtc *crtc)
{
	/* drm framework doesn't check NULL. */
}

static void tcc_drm_crtc_commit(struct drm_crtc *crtc)
{
	struct tcc_drm_crtc *tcc_crtc = to_tcc_crtc(crtc);
	struct tcc_drm_manager *manager = tcc_crtc->manager;

	tcc_drm_crtc_dpms(crtc, DRM_MODE_DPMS_ON);

	tcc_plane_commit(crtc->primary);

	if (manager->ops->commit)
		manager->ops->commit(manager);

	tcc_plane_dpms(crtc->primary, DRM_MODE_DPMS_ON);
}

static bool
tcc_drm_crtc_mode_fixup(struct drm_crtc *crtc,
			    const struct drm_display_mode *mode,
			    struct drm_display_mode *adjusted_mode)
{
	struct tcc_drm_crtc *tcc_crtc = to_tcc_crtc(crtc);
	struct tcc_drm_manager *manager = tcc_crtc->manager;

	if (manager->ops->mode_fixup)
		return manager->ops->mode_fixup(manager, mode, adjusted_mode);

	return true;
}

static int
tcc_drm_crtc_mode_set(struct drm_crtc *crtc, struct drm_display_mode *mode,
			  struct drm_display_mode *adjusted_mode, int x, int y,
			  struct drm_framebuffer *old_fb)
{
	struct tcc_drm_crtc *tcc_crtc = to_tcc_crtc(crtc);
	struct tcc_drm_manager *manager = tcc_crtc->manager;
	struct drm_framebuffer *fb = crtc->primary->fb;
	unsigned int crtc_w;
	unsigned int crtc_h;

	/*
	 * copy the mode data adjusted by mode_fixup() into crtc->mode
	 * so that hardware can be seet to proper mode.
	 */
	memcpy(&crtc->mode, adjusted_mode, sizeof(*adjusted_mode));

	crtc_w = fb->width - x;
	crtc_h = fb->height - y;

	if (manager->ops->mode_set)
		manager->ops->mode_set(manager, &crtc->mode);

	return tcc_plane_mode_set(crtc->primary, crtc, fb, 0, 0,
				     crtc_w, crtc_h, x, y, crtc_w, crtc_h);
}

static int tcc_drm_crtc_mode_set_commit(struct drm_crtc *crtc, int x, int y,
					  struct drm_framebuffer *old_fb)
{
	struct tcc_drm_crtc *tcc_crtc = to_tcc_crtc(crtc);
	struct drm_framebuffer *fb = crtc->primary->fb;
	unsigned int crtc_w;
	unsigned int crtc_h;
	int ret;

	/* when framebuffer changing is requested, crtc's dpms should be on */
	if (tcc_crtc->dpms > DRM_MODE_DPMS_ON) {
		DRM_ERROR("failed framebuffer changing request.\n");
		return -EPERM;
	}

	crtc_w = fb->width - x;
	crtc_h = fb->height - y;

	ret = tcc_plane_mode_set(crtc->primary, crtc, fb, 0, 0,
				    crtc_w, crtc_h, x, y, crtc_w, crtc_h);
	if (ret)
		return ret;

	tcc_drm_crtc_commit(crtc);

	return 0;
}

static int tcc_drm_crtc_mode_set_base(struct drm_crtc *crtc, int x, int y,
					  struct drm_framebuffer *old_fb)
{
	return tcc_drm_crtc_mode_set_commit(crtc, x, y, old_fb);
}

static void tcc_drm_crtc_disable(struct drm_crtc *crtc)
{
	struct drm_plane *plane;
	int ret;

	tcc_drm_crtc_dpms(crtc, DRM_MODE_DPMS_OFF);

	drm_for_each_legacy_plane(plane, &crtc->dev->mode_config.plane_list) {
		if (plane->crtc != crtc)
			continue;

		ret = plane->funcs->disable_plane(plane);
		if (ret)
			DRM_ERROR("Failed to disable plane %d\n", ret);
	}
}

static struct drm_crtc_helper_funcs tcc_crtc_helper_funcs = {
	.dpms		= tcc_drm_crtc_dpms,
	.prepare	= tcc_drm_crtc_prepare,
	.commit		= tcc_drm_crtc_commit,
	.mode_fixup	= tcc_drm_crtc_mode_fixup,
	.mode_set	= tcc_drm_crtc_mode_set,
	.mode_set_base	= tcc_drm_crtc_mode_set_base,
	.disable	= tcc_drm_crtc_disable,
};

static int tcc_drm_crtc_page_flip(struct drm_crtc *crtc,
				     struct drm_framebuffer *fb,
				     struct drm_pending_vblank_event *event,
				     uint32_t page_flip_flags)
{
	struct drm_device *dev = crtc->dev;
	struct tcc_drm_private *dev_priv = dev->dev_private;
	struct tcc_drm_crtc *tcc_crtc = to_tcc_crtc(crtc);
	struct drm_framebuffer *old_fb = crtc->primary->fb;
	int ret = -EINVAL;

	/* when the page flip is requested, crtc's dpms should be on */
	if (tcc_crtc->dpms > DRM_MODE_DPMS_ON) {
		DRM_ERROR("failed page flip request.\n");
		return -EINVAL;
	}

	DRM_DEBUG("[%s]:%d\n", __func__, __LINE__);
	mutex_lock(&dev->struct_mutex);

	if (event) {
		/*
		 * the pipe from user always is 0 so we can set pipe number
		 * of current owner to event.
		 */
		event->pipe = tcc_crtc->pipe;

		ret = drm_vblank_get(dev, tcc_crtc->pipe);
		if (ret) {
			DRM_DEBUG("failed to acquire vblank counter\n");

			goto out;
		}

		spin_lock_irq(&dev->event_lock);
		list_add_tail(&event->base.link,
				&dev_priv->pageflip_event_list);
		atomic_set(&tcc_crtc->pending_flip, 1);
		spin_unlock_irq(&dev->event_lock);

		crtc->primary->fb = fb;
		ret = tcc_drm_crtc_mode_set_commit(crtc, crtc->x, crtc->y,
						    NULL);
		if (ret) {
			crtc->primary->fb = old_fb;

			spin_lock_irq(&dev->event_lock);
			drm_vblank_put(dev, tcc_crtc->pipe);
			list_del(&event->base.link);
			atomic_set(&tcc_crtc->pending_flip, 0);
			spin_unlock_irq(&dev->event_lock);

			goto out;
		}
	}
out:
	mutex_unlock(&dev->struct_mutex);
	return ret;
}

static void tcc_drm_crtc_destroy(struct drm_crtc *crtc)
{
	struct tcc_drm_crtc *tcc_crtc = to_tcc_crtc(crtc);
	struct tcc_drm_private *private = crtc->dev->dev_private;

	private->crtc[tcc_crtc->pipe] = NULL;

	drm_crtc_cleanup(crtc);
	kfree(tcc_crtc);
}

static int tcc_drm_crtc_set_property(struct drm_crtc *crtc,
					struct drm_property *property,
					uint64_t val)
{
	struct drm_device *dev = crtc->dev;
	struct tcc_drm_private *dev_priv = dev->dev_private;
	struct tcc_drm_crtc *tcc_crtc = to_tcc_crtc(crtc);

	if (property == dev_priv->crtc_mode_property) {
		enum tcc_crtc_mode mode = val;

		if (mode == tcc_crtc->mode)
			return 0;

		tcc_crtc->mode = mode;

		switch (mode) {
		case CRTC_MODE_NORMAL:
			tcc_drm_crtc_commit(crtc);
			break;
		case CRTC_MODE_BLANK:
			tcc_plane_dpms(crtc->primary, DRM_MODE_DPMS_OFF);
			break;
		default:
			break;
		}

		return 0;
	}

	return -EINVAL;
}

static struct drm_crtc_funcs tcc_crtc_funcs = {
	.set_config	= drm_crtc_helper_set_config,
	.page_flip	= tcc_drm_crtc_page_flip,
	.destroy	= tcc_drm_crtc_destroy,
	.set_property	= tcc_drm_crtc_set_property,
};

static const struct drm_prop_enum_list mode_names[] = {
	{ CRTC_MODE_NORMAL, "normal" },
	{ CRTC_MODE_BLANK, "blank" },
};

static void tcc_drm_crtc_attach_mode_property(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct tcc_drm_private *dev_priv = dev->dev_private;
	struct drm_property *prop;

	prop = dev_priv->crtc_mode_property;
	if (!prop) {
		prop = drm_property_create_enum(dev, 0, "mode", mode_names,
						ARRAY_SIZE(mode_names));
		if (!prop)
			return;

		dev_priv->crtc_mode_property = prop;
	}

	drm_object_attach_property(&crtc->base, prop, 0);
}

int tcc_drm_crtc_create(struct tcc_drm_manager *manager)
{
	struct tcc_drm_crtc *tcc_crtc;
	struct drm_plane *plane;
	struct tcc_drm_private *private = manager->drm_dev->dev_private;
	struct drm_crtc *crtc;
	int ret;

	tcc_crtc = kzalloc(sizeof(*tcc_crtc), GFP_KERNEL);
	if (!tcc_crtc)
		return -ENOMEM;

	init_waitqueue_head(&tcc_crtc->pending_flip_queue);
	atomic_set(&tcc_crtc->pending_flip, 0);

	tcc_crtc->dpms = DRM_MODE_DPMS_OFF;
	tcc_crtc->manager = manager;
	tcc_crtc->pipe = manager->pipe;
	plane = tcc_plane_init(manager->drm_dev, 1 << manager->pipe,
				  DRM_PLANE_TYPE_PRIMARY, DEFAULT_ZPOS);
	if (IS_ERR(plane)) {
		ret = PTR_ERR(plane);
		goto err_plane;
	}

	manager->crtc = &tcc_crtc->drm_crtc;
	crtc = &tcc_crtc->drm_crtc;

	private->crtc[manager->pipe] = crtc;

	ret = drm_crtc_init_with_planes(manager->drm_dev, crtc, plane, NULL,
					&tcc_crtc_funcs);
	if (ret < 0)
		goto err_crtc;

	drm_crtc_helper_add(crtc, &tcc_crtc_helper_funcs);

	tcc_drm_crtc_attach_mode_property(crtc);

	return 0;

err_crtc:
	plane->funcs->destroy(plane);
err_plane:
	kfree(tcc_crtc);
	return ret;
}

int tcc_drm_crtc_enable_vblank(struct drm_device *dev, int pipe)
{
	struct tcc_drm_private *private = dev->dev_private;
	struct tcc_drm_crtc *tcc_crtc =
		to_tcc_crtc(private->crtc[pipe]);
	struct tcc_drm_manager *manager = tcc_crtc->manager;

	if (tcc_crtc->dpms != DRM_MODE_DPMS_ON)
		return -EPERM;

	if (manager->ops->enable_vblank)
		manager->ops->enable_vblank(manager);

	return 0;
}

void tcc_drm_crtc_disable_vblank(struct drm_device *dev, int pipe)
{
	struct tcc_drm_private *private = dev->dev_private;
	struct tcc_drm_crtc *tcc_crtc =
		to_tcc_crtc(private->crtc[pipe]);
	struct tcc_drm_manager *manager = tcc_crtc->manager;

	if (tcc_crtc->dpms != DRM_MODE_DPMS_ON)
		return;

	if (manager->ops->disable_vblank)
		manager->ops->disable_vblank(manager);
}

void tcc_drm_crtc_finish_pageflip(struct drm_device *dev, int pipe)
{
	struct tcc_drm_private *dev_priv = dev->dev_private;
	struct drm_pending_vblank_event *e, *t;
	struct drm_crtc *drm_crtc = dev_priv->crtc[pipe];
	struct tcc_drm_crtc *tcc_crtc = to_tcc_crtc(drm_crtc);
	unsigned long flags;

	spin_lock_irqsave(&dev->event_lock, flags);

	list_for_each_entry_safe(e, t, &dev_priv->pageflip_event_list,
			base.link) {
		/* if event's pipe isn't same as crtc then ignore it. */
		if (pipe != e->pipe)
			continue;

		list_del(&e->base.link);
		drm_send_vblank_event(dev, -1, e);
		drm_vblank_put(dev, pipe);
		atomic_set(&tcc_crtc->pending_flip, 0);
		wake_up(&tcc_crtc->pending_flip_queue);
	}

	spin_unlock_irqrestore(&dev->event_lock, flags);
}

void tcc_drm_crtc_plane_mode_set(struct drm_crtc *crtc,
			struct tcc_drm_overlay *overlay)
{
	struct tcc_drm_manager *manager = to_tcc_crtc(crtc)->manager;

	if (manager->ops->win_mode_set)
		manager->ops->win_mode_set(manager, overlay);
}

void tcc_drm_crtc_plane_commit(struct drm_crtc *crtc, int zpos)
{
	struct tcc_drm_manager *manager = to_tcc_crtc(crtc)->manager;

	if (manager->ops->win_commit)
		manager->ops->win_commit(manager, zpos);
}

void tcc_drm_crtc_plane_enable(struct drm_crtc *crtc, int zpos)
{
	struct tcc_drm_manager *manager = to_tcc_crtc(crtc)->manager;

	if (manager->ops->win_enable)
		manager->ops->win_enable(manager, zpos);
}

void tcc_drm_crtc_plane_disable(struct drm_crtc *crtc, int zpos)
{
	struct tcc_drm_manager *manager = to_tcc_crtc(crtc)->manager;

	if (manager->ops->win_disable)
		manager->ops->win_disable(manager, zpos);
}

void tcc_drm_crtc_complete_scanout(struct drm_framebuffer *fb)
{
	struct tcc_drm_manager *manager;
	struct drm_device *dev = fb->dev;
	struct drm_crtc *crtc;

	/*
	 * make sure that overlay data are updated to real hardware
	 * for all encoders.
	 */
	list_for_each_entry(crtc, &dev->mode_config.crtc_list, head) {
		manager = to_tcc_crtc(crtc)->manager;

		/*
		 * wait for vblank interrupt
		 * - this makes sure that overlay data are updated to
		 *	real hardware.
		 */
		if (manager->ops->wait_for_vblank)
			manager->ops->wait_for_vblank(manager);
	}
}

int tcc_drm_crtc_get_pipe_from_type(struct drm_device *drm_dev,
					unsigned int out_type)
{
	struct drm_crtc *crtc;

	list_for_each_entry(crtc, &drm_dev->mode_config.crtc_list, head) {
		struct tcc_drm_crtc *tcc_crtc;

		tcc_crtc = to_tcc_crtc(crtc);
		if (tcc_crtc->manager->type == out_type)
			return tcc_crtc->manager->pipe;
	}

	return -EPERM;
}

void tcc_drm_crtc_te_handler(struct drm_crtc *crtc)
{
	struct tcc_drm_manager *manager = to_tcc_crtc(crtc)->manager;

	if (manager->ops->te_handler)
		manager->ops->te_handler(manager);
}
