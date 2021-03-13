/* tcc_drm_lcd.c
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
 * Copyright (C) 2011 Samsung Electronics Co.Ltd
 * Authors:
 *	Joonyoung Shim <jy0922.shim@samsung.com>
 *	Inki Dae <inki.dae@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#include <drm/drmP.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/pm_runtime.h>
#include <linux/component.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

#include <video/of_display_timing.h>
#include <video/of_videomode.h>
#include <drm/tcc_drm.h>

#include "tcc_drm_drv.h"
#include "tcc_drm_fbdev.h"
#include "tcc_drm_crtc.h"
#include "tcc_drm_iommu.h"

#include <linux/irq.h>
#ifdef CONFIG_ARCH_TCC897X
#include <mach/tccfb.h>
#include <mach/vioc_rdma.h>
#include <mach/vioc_wmix.h>
#include <mach/tccfb_ioctrl.h>
#else
#include <video/tcc/tccfb.h>
#include <video/tcc/vioc_rdma.h>
#include <video/tcc/vioc_wmix.h>
#include <video/tcc/tccfb_ioctrl.h>
#endif

/*
 * LCD is stand for Fully Interactive Display and
 * as a display controller, it transfers contents drawn on memory
 * to a LCD Panel through Display Interfaces such as RGB or
 * CPU Interface.
 */

#define LCD_DEFAULT_FRAMERATE 60

/* TCC LCD has totally four hardware windows. */
#define WINDOWS_NR MAX_PLANE

#define get_lcd_manager(mgr)	platform_get_drvdata(to_platform_device(dev))

struct lcd_driver_data {
	unsigned int timing_base;
};

struct lcd_win_data {
	unsigned int		offset_x;
	unsigned int		offset_y;
	unsigned int		ovl_width;
	unsigned int		ovl_height;
	unsigned int		fb_width;
	unsigned int		fb_height;
	unsigned int		bpp;
	unsigned int		pixel_format;
	dma_addr_t		dma_addr;
	void __iomem		*virt_addr;	/* TCC rdma node address */
	unsigned int		buf_offsize;
	unsigned int		line_size;	/* bytes */
	bool			enabled;
	bool			resume;
};

struct lcd_context {
	struct device		*dev;
	struct drm_device	*drm_dev;
	int				irq;
	unsigned int	nr_disp;	/* TCC display path number */
	void __iomem	*virt_addr;	/* TCC wmixer node address */
	struct clk			*bus_clk;
	struct clk			*lcd_clk;
	void __iomem			*regs;
	struct regmap		*sysreg;
	struct drm_display_mode	mode;
	struct lcd_win_data		win_data[WINDOWS_NR];
	unsigned long			irq_flags;
	bool				suspended;
	int				pipe;
	wait_queue_head_t		wait_vsync_queue;
	atomic_t			wait_vsync_event;
	atomic_t			win_updated;
	atomic_t			triggering;

	struct tcc_drm_panel_info panel;
	struct lcd_driver_data	*driver_data;
	struct tcc_drm_display	*display;
};

static const struct of_device_id lcd_driver_dt_match[] = {
	{ .compatible = "telechips,lvds-fld0800" },
	{},
};
MODULE_DEVICE_TABLE(of, lcd_driver_dt_match);

struct tcc_vioc_fb_info {
    struct tcc_dp_device *dp;
    struct fb_var_screeninfo *var;
};

static inline struct lcd_driver_data *drm_lcd_get_driver_data(
	struct platform_device *pdev)
{
	const struct of_device_id *of_id =
			of_match_device(lcd_driver_dt_match, &pdev->dev);

	return (struct lcd_driver_data *)of_id->data;
}

static void lcd_wait_for_vblank(struct tcc_drm_manager *mgr)
{
	struct lcd_context *ctx = mgr->ctx;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	if (ctx->suspended)
		return;

	atomic_set(&ctx->wait_vsync_event, 1);

	/*
	 * wait for LCD to signal VSYNC interrupt or return after
	 * timeout which is set to 50ms (refresh rate of 20).
	 */
	if (!wait_event_timeout(ctx->wait_vsync_queue,
				!atomic_read(&ctx->wait_vsync_event),
				HZ/20))
		DRM_DEBUG_KMS("vblank wait timed out.\n");
}

static void lcd_clear_channel(struct tcc_drm_manager *mgr)
{
	/*
	 *struct lcd_context *ctx = mgr->ctx;
	 *int win, ch_enabled = 0;
	 */

	DRM_DEBUG_KMS("%s\n", __FILE__);

	/* TODO */
}

static int lcd_mgr_initialize(struct tcc_drm_manager *mgr,
		struct drm_device *drm_dev)
{
	struct lcd_context *ctx = mgr->ctx;
	struct tcc_drm_private *priv;
	priv = drm_dev->dev_private;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	mgr->drm_dev = ctx->drm_dev = drm_dev;
	mgr->pipe = ctx->pipe = priv->pipe++;

	/* attach this sub driver to iommu mapping if supported. */
	if (is_drm_iommu_supported(ctx->drm_dev)) {
		/*
		 * If any channel is already active, iommu will throw
		 * a PAGE FAULT when enabled. So clear any channel if enabled.
		 */
		lcd_clear_channel(mgr);
		drm_iommu_attach_device(ctx->drm_dev, ctx->dev);
	}

	return 0;
}

static void lcd_mgr_remove(struct tcc_drm_manager *mgr)
{
	struct lcd_context *ctx = mgr->ctx;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	/* detach this sub driver from iommu mapping if supported. */
	if (is_drm_iommu_supported(ctx->drm_dev))
		drm_iommu_detach_device(ctx->drm_dev, ctx->dev);
}

static bool lcd_mode_fixup(struct tcc_drm_manager *mgr,
		const struct drm_display_mode *mode,
		struct drm_display_mode *adjusted_mode)
{
	DRM_DEBUG_KMS("%s\n", __FILE__);

	if (adjusted_mode->vrefresh == 0)
		adjusted_mode->vrefresh = LCD_DEFAULT_FRAMERATE;

	return true;
}

static void lcd_mode_set(struct tcc_drm_manager *mgr,
		const struct drm_display_mode *in_mode)
{
	struct lcd_context *ctx = mgr->ctx;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	drm_mode_copy(&ctx->mode, in_mode);
}

static void lcd_commit(struct tcc_drm_manager *mgr)
{
	struct lcd_context *ctx = mgr->ctx;
	DRM_DEBUG_KMS("%s\n", __FILE__);


	if (ctx->suspended)
		return;

	/* TODO */
}

static int lcd_enable_vblank(struct tcc_drm_manager *mgr)
{
	struct lcd_context *ctx = mgr->ctx;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	if (ctx->suspended)
		return -EPERM;

	if (!test_and_set_bit(0, &ctx->irq_flags)) {
		vioc_intr_enable(ctx->nr_disp, VIOC_DISP_INTR_DISPLAY);
		//tca_lcdc_interrupt_onoff(1, ctx->nr_disp);
	}

	return 0;
}

static void lcd_disable_vblank(struct tcc_drm_manager *mgr)
{
	struct lcd_context *ctx = mgr->ctx;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	if (ctx->suspended)
		return;

	if (test_and_clear_bit(0, &ctx->irq_flags)) {
		vioc_intr_disable(ctx->nr_disp, VIOC_DISP_INTR_DISPLAY);
		//tca_lcdc_interrupt_onoff(0, ctx->nr_disp);
	}
}

static void lcd_win_mode_set(struct tcc_drm_manager *mgr,
			      struct tcc_drm_overlay *overlay)
{
	struct lcd_context *ctx = mgr->ctx;
	struct lcd_win_data *win_data;
	int win;
	unsigned long offset;

	if (!overlay) {
		DRM_ERROR("overlay is NULL\n");
		return;
	}

	win = overlay->zpos;

	if (win < 0 || win >= WINDOWS_NR)
		return;

	offset = overlay->fb_x * (overlay->bpp >> 3);
	offset += overlay->fb_y * overlay->pitch;

	DRM_DEBUG_KMS("offset = 0x%lx, pitch = %x\n", offset, overlay->pitch);

	win_data = &ctx->win_data[win];

	win_data->offset_x = overlay->crtc_x;
	win_data->offset_y = overlay->crtc_y;
	win_data->ovl_width = overlay->crtc_width;
	win_data->ovl_height = overlay->crtc_height;
	win_data->fb_width = overlay->fb_width;
	win_data->fb_height = overlay->fb_height;
	win_data->dma_addr = overlay->dma_addr[0] + offset;
	win_data->bpp = overlay->bpp;
	win_data->pixel_format = overlay->pixel_format;
	win_data->buf_offsize = (overlay->fb_width - overlay->crtc_width) *
				(overlay->bpp >> 3);
	win_data->line_size = overlay->crtc_width * (overlay->bpp >> 3);

	DRM_DEBUG_KMS("offset_x = %d, offset_y = %d\n",
			win_data->offset_x, win_data->offset_y);
	DRM_DEBUG_KMS("ovl_width = %d, ovl_height = %d\n",
			win_data->ovl_width, win_data->ovl_height);
	DRM_DEBUG_KMS("paddr = 0x%lx\n", (unsigned long)win_data->dma_addr);
	DRM_DEBUG_KMS("fb_width = %d, crtc_width = %d\n",
			overlay->fb_width, overlay->crtc_width);

}

static void lcd_win_commit(struct tcc_drm_manager *mgr, int zpos)
{
	struct lcd_context *ctx = mgr->ctx;
	struct lcd_win_data *win_data;
	int win = zpos;
	unsigned int val;

	/* TCC specific structure */
	VIOC_WMIX *pWMIX;
	VIOC_RDMA *pRDMA;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	if (ctx->suspended)
		return;

	if (win < 0 || win >= WINDOWS_NR)
		return;

	win_data = &ctx->win_data[win];

	/* If suspended, enable this on resume */
	if (ctx->suspended) {
		win_data->resume = true;
		return;
	}

	pWMIX = ctx->virt_addr;
	pRDMA = win_data->virt_addr;

	VIOC_WMIX_SetPosition(pWMIX, win, win_data->offset_x, win_data->offset_y);

	if ( win == 1 ) { /* Test for H/W Cursor */
		VIOC_WMIX_SetOverlayPriority(pWMIX, 16); /* 1 - 0 - 2 - 3 */
	}

	/* Select & enable pixel alpha
	 * Warning : this will be ignore global alpha
	 */
	VIOC_RDMA_SetImageAlphaSelect(pRDMA, 1);
	VIOC_RDMA_SetImageAlphaEnable(pRDMA, 1);

	/* buffer start address */
	val = (unsigned int)win_data->dma_addr;
	VIOC_RDMA_SetImageBase(pRDMA, val, 0, 0);
	VIOC_RDMA_SetImageSize(pRDMA, win_data->ovl_width, win_data->ovl_height);

	if (win_data->bpp == 16) {
		VIOC_RDMA_SetImageOffset(pRDMA, TCC_LCDC_IMG_FMT_RGB565, win_data->fb_width);
		VIOC_RDMA_SetImageFormat(pRDMA, TCC_LCDC_IMG_FMT_RGB565);
	}
	else {
		VIOC_RDMA_SetImageOffset(pRDMA, TCC_LCDC_IMG_FMT_RGB888, win_data->fb_width);
		VIOC_RDMA_SetImageFormat(pRDMA, TCC_LCDC_IMG_FMT_RGB888);
	}

	DRM_DEBUG_KMS("start addr = 0x%x, zpos = %d\n",
			(unsigned int)win_data->dma_addr, win);
	DRM_DEBUG_KMS("ovl_width = %d, ovl_height = %d\n",
			win_data->ovl_width, win_data->ovl_height);

	DRM_DEBUG_KMS("osd pos: tx = %d, ty = %d\n",
			win_data->offset_x, win_data->offset_y);

	VIOC_RDMA_SetImageEnable(pRDMA);
	VIOC_WMIX_SetUpdate(pWMIX);

	win_data->enabled = true;
}

static void lcd_win_disable(struct tcc_drm_manager *mgr, int zpos)
{
	struct lcd_context *ctx = mgr->ctx;
	struct lcd_win_data *win_data;
	int win = zpos;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	if (win < 0 || win >= WINDOWS_NR)
		return;

	win_data = &ctx->win_data[win];

	if (ctx->suspended) {
		/* do not resume this window*/
		win_data->resume = false;
		return;
	}

	/* TODO */

	win_data->enabled = false;
}

static void lcd_window_suspend(struct tcc_drm_manager *mgr)
{
	struct lcd_context *ctx = mgr->ctx;
	struct lcd_win_data *win_data;
	int i;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	for (i = 0; i < WINDOWS_NR; i++) {
		win_data = &ctx->win_data[i];
		win_data->resume = win_data->enabled;
		if (win_data->enabled)
			lcd_win_disable(mgr, i);
	}
	lcd_wait_for_vblank(mgr);
}

static void lcd_window_resume(struct tcc_drm_manager *mgr)
{
	struct lcd_context *ctx = mgr->ctx;
	struct lcd_win_data *win_data;
	int i;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	for (i = 0; i < WINDOWS_NR; i++) {
		win_data = &ctx->win_data[i];
		win_data->enabled = win_data->resume;
		win_data->resume = false;
	}
}

static void lcd_apply(struct tcc_drm_manager *mgr)
{
	struct lcd_context *ctx = mgr->ctx;
	struct lcd_win_data *win_data;
	int i;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	for (i = 0; i < WINDOWS_NR; i++) {
		win_data = &ctx->win_data[i];
		if (win_data->enabled)
			lcd_win_commit(mgr, i);
		else
			lcd_win_disable(mgr, i);
	}

	lcd_commit(mgr);
}

static int lcd_poweron(struct tcc_drm_manager *mgr)
{
	struct lcd_context *ctx = mgr->ctx;
	int ret;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	if (!ctx->suspended)
		return 0;

	ctx->suspended = false;

	pm_runtime_get_sync(ctx->dev);

	ret = clk_prepare_enable(ctx->bus_clk);
	if (ret < 0) {
		DRM_ERROR("Failed to prepare_enable the bus clk [%d]\n", ret);
		goto bus_clk_err;
	}

	ret = clk_prepare_enable(ctx->lcd_clk);
	if  (ret < 0) {
		DRM_ERROR("Failed to prepare_enable the lcd clk [%d]\n", ret);
		goto lcd_clk_err;
	}

	/* if vblank was enabled status, enable it again. */
	if (test_and_clear_bit(0, &ctx->irq_flags)) {
		ret = lcd_enable_vblank(mgr);
		if (ret) {
			DRM_ERROR("Failed to re-enable vblank [%d]\n", ret);
			goto enable_vblank_err;
		}
	}

	lcd_window_resume(mgr);

	lcd_apply(mgr);

	return 0;

enable_vblank_err:
	clk_disable_unprepare(ctx->lcd_clk);
lcd_clk_err:
	clk_disable_unprepare(ctx->bus_clk);
bus_clk_err:
	ctx->suspended = true;
	return ret;
}

static int lcd_poweroff(struct tcc_drm_manager *mgr)
{
	struct lcd_context *ctx = mgr->ctx;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	if (ctx->suspended)
		return 0;

	/*
	 * We need to make sure that all windows are disabled before we
	 * suspend that connector. Otherwise we might try to scan from
	 * a destroyed buffer later.
	 */
	lcd_window_suspend(mgr);

	clk_disable_unprepare(ctx->lcd_clk);
	clk_disable_unprepare(ctx->bus_clk);

	pm_runtime_put_sync(ctx->dev);

	ctx->suspended = true;
	return 0;
}

static void lcd_dpms(struct tcc_drm_manager *mgr, int mode)
{
	DRM_DEBUG_KMS("%s, %d\n", __FILE__, mode);

	switch (mode) {
	case DRM_MODE_DPMS_ON:
		lcd_poweron(mgr);
		break;
	case DRM_MODE_DPMS_STANDBY:
	case DRM_MODE_DPMS_SUSPEND:
	case DRM_MODE_DPMS_OFF:
		lcd_poweroff(mgr);
		break;
	default:
		DRM_DEBUG_KMS("unspecified mode %d\n", mode);
		break;
	}
}

static void lcd_trigger(struct device *dev)
{
	/* TODO */
	DRM_DEBUG_KMS("%s\n", __FILE__);
}

static void lcd_te_handler(struct tcc_drm_manager *mgr)
{
	struct lcd_context *ctx = mgr->ctx;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	/* Checks the crtc is detached already from encoder */
	if (ctx->pipe < 0 || !ctx->drm_dev)
		return;

	/*
	 * Skips to trigger if in triggering state, because multiple triggering
	 * requests can cause panel reset.
	 */
	if (atomic_read(&ctx->triggering))
		return;

	/*
	 * If there is a page flip request, triggers and handles the page flip
	 * event so that current fb can be updated into panel GRAM.
	 */
	if (atomic_add_unless(&ctx->win_updated, -1, 0))
		lcd_trigger(ctx->dev);

	/* Wakes up vsync event queue */
	if (atomic_read(&ctx->wait_vsync_event)) {
		atomic_set(&ctx->wait_vsync_event, 0);
		wake_up(&ctx->wait_vsync_queue);

		if (!atomic_read(&ctx->triggering))
			drm_handle_vblank(ctx->drm_dev, ctx->pipe);
	}
}

static struct tcc_drm_manager_ops lcd_manager_ops = {
    .dpms = lcd_dpms,
	.mode_fixup = lcd_mode_fixup,
	.mode_set = lcd_mode_set,
	.commit = lcd_commit,
    .enable_vblank = lcd_enable_vblank,
    .disable_vblank = lcd_disable_vblank,
    .wait_for_vblank = lcd_wait_for_vblank,
	.win_mode_set = lcd_win_mode_set,
	.win_commit = lcd_win_commit,
	.win_disable = lcd_win_disable,
	.te_handler = lcd_te_handler,
};

static struct tcc_drm_manager lcd_manager = {
	.type = TCC_DISPLAY_TYPE_LCD,
	.ops = &lcd_manager_ops,
};

static irqreturn_t lcd_irq_handler(int irq, void *dev_id)
{
	struct lcd_context *ctx = (struct lcd_context *)dev_id;

	if (!is_vioc_intr_activatied(ctx->nr_disp, VIOC_DISP_INTR_RU)) {
		DRM_DEBUG("interupt but not RU\n");
		return IRQ_NONE;
	}

	vioc_intr_clear(ctx->nr_disp, (1 << VIOC_DISP_INTR_RU));

	/* check the crtc is detached already from encoder */
	if (ctx->pipe < 0 || !ctx->drm_dev)
		goto out;

	drm_handle_vblank(ctx->drm_dev, ctx->pipe);
	tcc_drm_crtc_finish_pageflip(ctx->drm_dev, ctx->pipe);

	/* set wait vsync event to zero and wake up queue. */
	if (atomic_read(&ctx->wait_vsync_event)) {
		atomic_set(&ctx->wait_vsync_event, 0);
		wake_up(&ctx->wait_vsync_queue);
	}

out:
	return IRQ_HANDLED;
}

static int lcd_bind(struct device *dev, struct device *master, void *data)
{
	struct lcd_context *ctx = lcd_manager.ctx;
	struct drm_device *drm_dev = data;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	lcd_mgr_initialize(&lcd_manager, drm_dev);
	tcc_drm_crtc_create(&lcd_manager);
	if (ctx->display)
		tcc_drm_create_enc_conn(drm_dev, ctx->display);

	return 0;
}

static void lcd_unbind(struct device *dev, struct device *master,
		void *data)
{
	struct tcc_drm_manager *mgr = dev_get_drvdata(dev);
	struct lcd_context *ctx = lcd_manager.ctx;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	lcd_dpms(mgr, DRM_MODE_DPMS_OFF);

	if (ctx->display)
		tcc_dpi_remove(dev);

	lcd_mgr_remove(mgr);
}

static const struct component_ops lcd_component_ops = {
	.bind	= lcd_bind,
	.unbind = lcd_unbind,
};

static int lcd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct lcd_context *ctx;
	int ret = -EINVAL;
	unsigned int index = 0, i = 0;
	/* TCC specific device node from device tree */
	struct device_node *np_worker, *np_main, *np_ddc, *np_rdma, *np_wmixer;

	ret = tcc_drm_component_add(&pdev->dev, TCC_DEVICE_TYPE_CRTC,
				lcd_manager.type);
	if (ret)
		return ret;

	/* TODO
	 * Current No device tree for TCC DRM,
	 * so disable check of_node function 2016-04-08
	 */

	/*
	 *if (!dev->of_node) {
	 *    ret = -ENODEV;
	 *    goto err_del_component;
	 *}
	 */

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		ret = -ENOMEM;
		goto err_del_component;
	}

	ctx->dev = dev;
	ctx->suspended = true;

	/* TODO */
	/*ctx->driver_data = drm_lcd_get_driver_data(pdev);*/

	/* Get TCC vioc display path number */
	np_worker = of_find_compatible_node(NULL, NULL, "telechips,vioc-fb");
	if (of_property_read_u32(np_worker, "telechips,fbdisplay_num", &ctx->nr_disp)) {
		DRM_ERROR("could not find tcc fbdisplay_num\n");
		ret = -ENODEV;
		goto err_del_component;
	}

	/* Get TCC specific vioc data */
	if ( ctx->nr_disp == 0 ) {
		np_main = of_find_node_by_name(np_worker, "fbdisplay0");
		if (!np_main) {
			DRM_ERROR("could not find tcc fb node number %d\n", ctx->nr_disp);
			ret = -ENODEV;
			goto err_del_component;
		}

		np_ddc = of_parse_phandle(np_main, "telechips,disp", 0);
		if (!np_ddc) {
			DRM_ERROR("could not find telechips,disp node\n");
			ret = -ENODEV;
			goto err_del_component;
		}

		ctx->regs = of_iomap(np_ddc, 0);
		if (IS_ERR(ctx->regs)) {
			ret = PTR_ERR(ctx->regs);
			goto err_del_component;
		}

		ctx->irq = irq_of_parse_and_map(np_ddc, 0);
		synchronize_irq(ctx->irq);
		ret = request_irq(ctx->irq, lcd_irq_handler,
				IRQF_SHARED/*|IRQ_TYPE_EDGE_FALLING*/,
				"drm_lcd", ctx);
		if (ret) {
			DRM_ERROR("irq request failed.\n");
			goto err_del_component;
		}

		ctx->bus_clk = of_clk_get(np_ddc, 1);
		if (IS_ERR(ctx->bus_clk)) {
			DRM_ERROR("failed to get bus clock\n");
			ret = PTR_ERR(ctx->bus_clk);
			goto err_del_component;
		}

		ctx->lcd_clk = of_clk_get(np_ddc, 0);
		if (IS_ERR(ctx->lcd_clk)) {
			DRM_ERROR("failed to get lcd clock\n");
			ret = PTR_ERR(ctx->lcd_clk);
			goto err_del_component;
		}

		/* Get WMIXER, RDMA */
		np_wmixer = of_parse_phandle(np_main, "telechips,wmixer", 0);
		if (!np_wmixer) {
			DRM_ERROR("could not find telechips,wmixer node\n");
			ret = -ENODEV;
			goto err_del_component;
		}

		/* Get WMIXER address */
		of_property_read_u32_index(np_main, "telechips,wmixer", 1, &index);
		ctx->virt_addr = of_iomap(np_wmixer, index);
		DRM_DEBUG_KMS("wmixer %d 0x%p\n", index, ctx->virt_addr);

		/* Get RDMA address */
		np_rdma = of_parse_phandle(np_main, "telechips,rdma", 0);
		if (!np_rdma) {
			DRM_ERROR("could not find telechips,rdma node\n");
			ret = -ENODEV;
			goto err_del_component;
		}
		for (i = 0; i < WINDOWS_NR; i++) {
			of_property_read_u32_index(np_main, "telechips,rdma", i + 1, &index);
			ctx->win_data[i].virt_addr = of_iomap(np_rdma, index);
			DRM_DEBUG_KMS("ctx->win_data[%d]->virt_addr:0x%p\n", i, ctx->win_data[i].virt_addr);
		}
		DRM_DEBUG_KMS("DispNum:%d, virt_addr:0x%p\n", ctx->nr_disp, ctx->regs);
	}
	else {
		DRM_ERROR("TCC-DRM do not support display path 1\n");
		ret = -ENODEV;
		goto err_del_component;
	}

	init_waitqueue_head(&ctx->wait_vsync_queue);
	atomic_set(&ctx->wait_vsync_event, 0);

	platform_set_drvdata(pdev, &lcd_manager);

	lcd_manager.ctx = ctx;

	ctx->display = tcc_dpi_probe(dev);
	if (IS_ERR(ctx->display))
		return PTR_ERR(ctx->display);

	pm_runtime_enable(&pdev->dev);

	ret = component_add(&pdev->dev, &lcd_component_ops);
	if (ret)
		goto err_disable_pm_runtime;

	return ret;

err_disable_pm_runtime:
	pm_runtime_disable(&pdev->dev);

err_del_component:
	tcc_drm_component_del(&pdev->dev, TCC_DEVICE_TYPE_CRTC);
	return ret;
}

static int lcd_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);
	component_del(&pdev->dev, &lcd_component_ops);
	tcc_drm_component_del(&pdev->dev, TCC_DEVICE_TYPE_CRTC);

	return 0;
}

struct platform_driver lcd_driver = {
	.probe		= lcd_probe,
	.remove		= lcd_remove,
	.driver		= {
		.name			= "tcc-drm-lcd",
		.owner			= THIS_MODULE,
		.of_match_table = lcd_driver_dt_match,
	},
};

MODULE_AUTHOR("Telechips Inc. <linux@telechips.com>");
MODULE_DESCRIPTION("Telechips TCC DRM LCD driver");
MODULE_LICENSE("GPL");
