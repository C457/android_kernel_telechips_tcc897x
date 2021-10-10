/* tcc_drm_dpi.c
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
 * Copyright (c) 2014 Samsung Electronics Co., Ltd
 *
 * Contacts: Andrzej Hajda <a.hajda@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_panel.h>

#include <linux/regulator/consumer.h>

#include <video/of_videomode.h>
#include <video/videomode.h>
#ifdef CONFIG_ARCH_TCC897X
#include <mach/tccfb.h>
#else
#include <video/tcc/tccfb.h>
#endif
#include "tcc_drm_drv.h"

struct tcc_dpi {
	struct device *dev;
	struct device_node *panel_node;

	struct drm_panel *panel;
	struct drm_connector connector;
	struct drm_encoder *encoder;

	struct videomode *vm;
	int dpms_mode;
};

#define connector_to_dpi(c) container_of(c, struct tcc_dpi, connector)

static enum drm_connector_status
tcc_dpi_detect(struct drm_connector *connector, bool force)
{
	struct tcc_dpi *ctx = connector_to_dpi(connector);

	if (ctx->panel && !ctx->panel->connector)
		drm_panel_attach(ctx->panel, &ctx->connector);

	return connector_status_connected;
}

static void tcc_dpi_connector_destroy(struct drm_connector *connector)
{
	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
}

static struct drm_connector_funcs tcc_dpi_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.detect = tcc_dpi_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = tcc_dpi_connector_destroy,
};

static int tcc_dpi_get_modes(struct drm_connector *connector)
{
	struct tcc_dpi *ctx = connector_to_dpi(connector);

	/* lcd timings gets precedence over panel modes */
	if (ctx->vm) {
		struct drm_display_mode *mode;

		mode = drm_mode_create(connector->dev);
		if (!mode) {
			DRM_ERROR("failed to create a new display mode\n");
			return 0;
		}
		drm_display_mode_from_videomode(ctx->vm, mode);
		mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
		drm_mode_probed_add(connector, mode);
		return 1;
	}

	if (ctx->panel)
		return ctx->panel->funcs->get_modes(ctx->panel);

	return 0;
}

static struct drm_encoder *
tcc_dpi_best_encoder(struct drm_connector *connector)
{
	struct tcc_dpi *ctx = connector_to_dpi(connector);

	return ctx->encoder;
}

static struct drm_connector_helper_funcs tcc_dpi_connector_helper_funcs = {
	.get_modes = tcc_dpi_get_modes,
	.best_encoder = tcc_dpi_best_encoder,
};

static int tcc_dpi_create_connector(struct tcc_drm_display *display,
				       struct drm_encoder *encoder)
{
	struct tcc_dpi *ctx = display->ctx;
	struct drm_connector *connector = &ctx->connector;
	int ret;

	ctx->encoder = encoder;

	connector->polled = DRM_CONNECTOR_POLL_HPD;

	ret = drm_connector_init(encoder->dev, connector,
				 &tcc_dpi_connector_funcs,
				 DRM_MODE_CONNECTOR_LVDS);
	if (ret) {
		DRM_ERROR("failed to initialize connector with drm\n");
		return ret;
	}

	drm_connector_helper_add(connector, &tcc_dpi_connector_helper_funcs);
	drm_connector_register(connector);
	drm_mode_connector_attach_encoder(connector, encoder);

	return 0;
}

static void tcc_dpi_poweron(struct tcc_dpi *ctx)
{
	if (ctx->panel) {
		drm_panel_prepare(ctx->panel);
		drm_panel_enable(ctx->panel);
	}
}

static void tcc_dpi_poweroff(struct tcc_dpi *ctx)
{
	if (ctx->panel) {
		drm_panel_disable(ctx->panel);
		drm_panel_unprepare(ctx->panel);
	}
}

static void tcc_dpi_dpms(struct tcc_drm_display *display, int mode)
{
	struct tcc_dpi *ctx = display->ctx;

	switch (mode) {
	case DRM_MODE_DPMS_ON:
		if (ctx->dpms_mode != DRM_MODE_DPMS_ON)
				tcc_dpi_poweron(ctx);
			break;
	case DRM_MODE_DPMS_STANDBY:
	case DRM_MODE_DPMS_SUSPEND:
	case DRM_MODE_DPMS_OFF:
		if (ctx->dpms_mode == DRM_MODE_DPMS_ON)
			tcc_dpi_poweroff(ctx);
		break;
	default:
		break;
	}
	ctx->dpms_mode = mode;
}

static struct tcc_drm_display_ops tcc_dpi_display_ops = {
	.create_connector = tcc_dpi_create_connector,
	.dpms = tcc_dpi_dpms
};

static struct tcc_drm_display tcc_dpi_display = {
	.type = TCC_DISPLAY_TYPE_LCD,
	.ops = &tcc_dpi_display_ops,
};

static int tcc_dpi_parse_dt(struct tcc_dpi *ctx)
{
	struct device *dev = ctx->dev;
	struct device_node *dn, *np;

	dn = of_find_node_by_name(NULL, "fld0800");
	if (!dn) {
		dn = of_find_node_by_name(NULL, "at070tn93");
		if (!dn) {
			DRM_ERROR("cannot find lvds, lcd device node\n");
			return -ENODEV;
		}
	}

	np = of_get_child_by_name(dn, "display-timings");
	if (np) {
		struct videomode *vm;
		int ret;

		of_node_put(np);

		vm = devm_kzalloc(dev, sizeof(*ctx->vm), GFP_KERNEL);
		if (!vm)
		{
			DRM_ERROR("%s vm alloc fail\n", __FILE__);
			return -ENOMEM;
		}
		ret = of_get_videomode(dn, vm, 0);
		if (ret < 0) {
			DRM_ERROR("%s of_get_videomode\n", __FILE__);
			devm_kfree(dev, vm);
			return ret;
		}

		ctx->vm = vm;
	}
	else {
		DRM_ERROR("cannot find display-timings\n");
		return -ENODEV;
	}

	return 0;
}

struct tcc_drm_display *tcc_dpi_probe(struct device *dev)
{
	struct tcc_dpi *ctx;
	int ret;

	ret = tcc_drm_component_add(dev,
					TCC_DEVICE_TYPE_CONNECTOR,
					tcc_dpi_display.type);
	if (ret)
		return ERR_PTR(ret);

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		goto err_del_component;

	ctx->dev = dev;
	tcc_dpi_display.ctx = ctx;
	ctx->dpms_mode = DRM_MODE_DPMS_OFF;

	ret = tcc_dpi_parse_dt(ctx);
	if (ret < 0) {
		devm_kfree(dev, ctx);
		goto err_del_component;
	}

	if (ctx->panel_node) {
		ctx->panel = of_drm_find_panel(ctx->panel_node);
		if (!ctx->panel) {
			tcc_drm_component_del(dev,
						TCC_DEVICE_TYPE_CONNECTOR);
			return ERR_PTR(-EPROBE_DEFER);
		}
	}

	return &tcc_dpi_display;

err_del_component:
	tcc_drm_component_del(dev, TCC_DEVICE_TYPE_CONNECTOR);

	return NULL;
}

int tcc_dpi_remove(struct device *dev)
{
	struct tcc_dpi *ctx = tcc_dpi_display.ctx;

	tcc_dpi_dpms(&tcc_dpi_display, DRM_MODE_DPMS_OFF);

	if (ctx->panel)
		drm_panel_detach(ctx->panel);

	tcc_drm_component_del(dev, TCC_DEVICE_TYPE_CONNECTOR);

	return 0;
}
