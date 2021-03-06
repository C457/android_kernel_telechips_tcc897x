/* tcc_drm_fb.c
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
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_helper.h>
#include <uapi/drm/tcc_drm.h>

#include "tcc_drm_drv.h"
#include "tcc_drm_fb.h"
#include "tcc_drm_fbdev.h"
#include "tcc_drm_gem.h"
#include "tcc_drm_iommu.h"
#include "tcc_drm_crtc.h"

#ifdef CONFIG_HIBERNATION
extern unsigned int *drm_fb_virt_addr;
#endif

#define to_tcc_fb(x)	container_of(x, struct tcc_drm_fb, fb)

/*
 * tcc specific framebuffer structure.
 *
 * @fb: drm framebuffer obejct.
 * @buf_cnt: a buffer count to drm framebuffer.
 * @tcc_gem_obj: array of tcc specific gem object containing a gem object.
 */
struct tcc_drm_fb {
	struct drm_framebuffer		fb;
	unsigned int			buf_cnt;
	struct tcc_drm_gem_obj	*tcc_gem_obj[MAX_FB_BUFFER];
};

static int check_fb_gem_memory_type(struct drm_device *drm_dev,
				struct tcc_drm_gem_obj *tcc_gem_obj)
{
	unsigned int flags;

	/*
	 * if tcc drm driver supports iommu then framebuffer can use
	 * all the buffer types.
	 */
	if (is_drm_iommu_supported(drm_dev))
		return 0;

	flags = tcc_gem_obj->flags;

	/*
	 * without iommu support, not support physically non-continuous memory
	 * for framebuffer.
	 */
	if (IS_NONCONTIG_BUFFER(flags)) {
		DRM_ERROR("cannot use this gem memory type for fb.\n");
		return -EINVAL;
	}

	return 0;
}

static void tcc_drm_fb_destroy(struct drm_framebuffer *fb)
{
	struct tcc_drm_fb *tcc_fb = to_tcc_fb(fb);
	unsigned int i;

	/* make sure that overlay data are updated before relesing fb. */
	tcc_drm_crtc_complete_scanout(fb);

	drm_framebuffer_cleanup(fb);

	for (i = 0; i < ARRAY_SIZE(tcc_fb->tcc_gem_obj); i++) {
		struct drm_gem_object *obj;

		if (tcc_fb->tcc_gem_obj[i] == NULL)
			continue;

		obj = &tcc_fb->tcc_gem_obj[i]->base;
		drm_gem_object_unreference_unlocked(obj);
	}

	kfree(tcc_fb);
	tcc_fb = NULL;
}

static int tcc_drm_fb_create_handle(struct drm_framebuffer *fb,
					struct drm_file *file_priv,
					unsigned int *handle)
{
	struct tcc_drm_fb *tcc_fb = to_tcc_fb(fb);

	/* This fb should have only one gem object. */
	if (WARN_ON(tcc_fb->buf_cnt != 1))
		return -EINVAL;

	return drm_gem_handle_create(file_priv,
			&tcc_fb->tcc_gem_obj[0]->base, handle);
}

static int tcc_drm_fb_dirty(struct drm_framebuffer *fb,
				struct drm_file *file_priv, unsigned flags,
				unsigned color, struct drm_clip_rect *clips,
				unsigned num_clips)
{
	/* TODO */

	return 0;
}

static struct drm_framebuffer_funcs tcc_drm_fb_funcs = {
	.destroy	= tcc_drm_fb_destroy,
	.create_handle	= tcc_drm_fb_create_handle,
	.dirty		= tcc_drm_fb_dirty,
};

void tcc_drm_fb_set_buf_cnt(struct drm_framebuffer *fb,
						unsigned int cnt)
{
	struct tcc_drm_fb *tcc_fb;

	tcc_fb = to_tcc_fb(fb);

	tcc_fb->buf_cnt = cnt;
}

unsigned int tcc_drm_fb_get_buf_cnt(struct drm_framebuffer *fb)
{
	struct tcc_drm_fb *tcc_fb;

	tcc_fb = to_tcc_fb(fb);

	return tcc_fb->buf_cnt;
}

struct drm_framebuffer *
tcc_drm_framebuffer_init(struct drm_device *dev,
			    struct drm_mode_fb_cmd2 *mode_cmd,
			    struct drm_gem_object *obj)
{
	struct tcc_drm_fb *tcc_fb;
	struct tcc_drm_gem_obj *tcc_gem_obj;
	int ret;

	tcc_gem_obj = to_tcc_gem_obj(obj);

	ret = check_fb_gem_memory_type(dev, tcc_gem_obj);
	if (ret < 0) {
		DRM_ERROR("cannot use this gem memory type for fb.\n");
		return ERR_PTR(-EINVAL);
	}

	tcc_fb = kzalloc(sizeof(*tcc_fb), GFP_KERNEL);
	if (!tcc_fb)
		return ERR_PTR(-ENOMEM);

	drm_helper_mode_fill_fb_struct(&tcc_fb->fb, mode_cmd);
	tcc_fb->tcc_gem_obj[0] = tcc_gem_obj;

	ret = drm_framebuffer_init(dev, &tcc_fb->fb, &tcc_drm_fb_funcs);
	if (ret) {
		kfree(tcc_fb);
		DRM_ERROR("failed to initialize framebuffer\n");
		return ERR_PTR(ret);
	}

	return &tcc_fb->fb;
}

static u32 tcc_drm_format_num_buffers(struct drm_mode_fb_cmd2 *mode_cmd)
{
	unsigned int cnt = 0;

	if (mode_cmd->pixel_format != DRM_FORMAT_NV12)
		return drm_format_num_planes(mode_cmd->pixel_format);

	while (cnt != MAX_FB_BUFFER) {
		if (!mode_cmd->handles[cnt])
			break;
		cnt++;
	}

	/*
	 * check if NV12 or NV12M.
	 *
	 * NV12
	 * handles[0] = base1, offsets[0] = 0
	 * handles[1] = base1, offsets[1] = Y_size
	 *
	 * NV12M
	 * handles[0] = base1, offsets[0] = 0
	 * handles[1] = base2, offsets[1] = 0
	 */
	if (cnt == 2) {
		/*
		 * in case of NV12 format, offsets[1] is not 0 and
		 * handles[0] is same as handles[1].
		 */
		if (mode_cmd->offsets[1] &&
			mode_cmd->handles[0] == mode_cmd->handles[1])
			cnt = 1;
	}

	return cnt;
}

static struct drm_framebuffer *
tcc_user_fb_create(struct drm_device *dev, struct drm_file *file_priv,
		      struct drm_mode_fb_cmd2 *mode_cmd)
{
	struct drm_gem_object *obj;
	struct tcc_drm_gem_obj *tcc_gem_obj;
	struct tcc_drm_fb *tcc_fb;
	int i, ret;

	tcc_fb = kzalloc(sizeof(*tcc_fb), GFP_KERNEL);
	if (!tcc_fb)
		return ERR_PTR(-ENOMEM);

	obj = drm_gem_object_lookup(dev, file_priv, mode_cmd->handles[0]);
	if (!obj) {
		DRM_ERROR("failed to lookup gem object\n");
		ret = -ENOENT;
		goto err_free;
	}

	drm_helper_mode_fill_fb_struct(&tcc_fb->fb, mode_cmd);
	tcc_fb->tcc_gem_obj[0] = to_tcc_gem_obj(obj);
	tcc_fb->buf_cnt = tcc_drm_format_num_buffers(mode_cmd);

	DRM_DEBUG_KMS("buf_cnt = %d\n", tcc_fb->buf_cnt);

	for (i = 1; i < tcc_fb->buf_cnt; i++) {
		obj = drm_gem_object_lookup(dev, file_priv,
				mode_cmd->handles[i]);
		if (!obj) {
			DRM_ERROR("failed to lookup gem object\n");
			ret = -ENOENT;
			tcc_fb->buf_cnt = i;
			goto err_unreference;
		}

		tcc_gem_obj = to_tcc_gem_obj(obj);
		tcc_fb->tcc_gem_obj[i] = tcc_gem_obj;

		ret = check_fb_gem_memory_type(dev, tcc_gem_obj);
		if (ret < 0) {
			DRM_ERROR("cannot use this gem memory type for fb.\n");
			goto err_unreference;
		}
	}

	ret = drm_framebuffer_init(dev, &tcc_fb->fb, &tcc_drm_fb_funcs);
	if (ret) {
		DRM_ERROR("failed to init framebuffer.\n");
		goto err_unreference;
	}
#ifdef CONFIG_HIBERNATION
	/*drm_fb_virt_addr = tcc_gem_obj->buffer->kvaddr;*/
	drm_fb_virt_addr = tcc_fb->tcc_gem_obj[0]->buffer->kvaddr;
	DRM_DEBUG("[%s] drm_fb_virt_addr : %p \n", __FILE__, drm_fb_virt_addr);
#endif
	return &tcc_fb->fb;

err_unreference:
	for (i = 0; i < tcc_fb->buf_cnt; i++) {
		struct drm_gem_object *obj;

		obj = &tcc_fb->tcc_gem_obj[i]->base;
		if (obj)
			drm_gem_object_unreference_unlocked(obj);
	}
err_free:
	kfree(tcc_fb);
	return ERR_PTR(ret);
}

struct tcc_drm_gem_buf *tcc_drm_fb_buffer(struct drm_framebuffer *fb,
						int index)
{
	struct tcc_drm_fb *tcc_fb = to_tcc_fb(fb);
	struct tcc_drm_gem_buf *buffer;

	if (index >= MAX_FB_BUFFER)
		return NULL;

	buffer = tcc_fb->tcc_gem_obj[index]->buffer;
	if (!buffer)
		return NULL;

	DRM_DEBUG_KMS("dma_addr = 0x%lx\n", (unsigned long)buffer->dma_addr);

	return buffer;
}

static void tcc_drm_output_poll_changed(struct drm_device *dev)
{
	struct tcc_drm_private *private = dev->dev_private;
	struct drm_fb_helper *fb_helper = private->fb_helper;

	if (fb_helper)
		drm_fb_helper_hotplug_event(fb_helper);
	else
		tcc_drm_fbdev_init(dev);
}

static const struct drm_mode_config_funcs tcc_drm_mode_config_funcs = {
	.fb_create = tcc_user_fb_create,
	.output_poll_changed = tcc_drm_output_poll_changed,
};

void tcc_drm_mode_config_init(struct drm_device *dev)
{
	dev->mode_config.min_width = 0;
	dev->mode_config.min_height = 0;

	/*
	 * set max width and height as default value(4096x4096).
	 * this value would be used to check framebuffer size limitation
	 * at drm_mode_addfb().
	 */
	dev->mode_config.max_width = 4096;
	dev->mode_config.max_height = 4096;

	dev->mode_config.funcs = &tcc_drm_mode_config_funcs;
}
