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

#include <linux/pm_runtime.h>
#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>

#include <linux/component.h>

#include <drm/tcc_drm.h>

#include "tcc_drm_drv.h"
#include "tcc_drm_crtc.h"
#include "tcc_drm_encoder.h"
#include "tcc_drm_fbdev.h"
#include "tcc_drm_fb.h"
#include "tcc_drm_gem.h"
#include "tcc_drm_plane.h"
#include "tcc_drm_dmabuf.h"
#include "tcc_drm_iommu.h"

#define DRIVER_NAME	"tcc-drm"
#define DRIVER_DESC	"Telechips SoC DRM"
#define DRIVER_DATE	"20160107"
#define DRIVER_MAJOR	1
#define DRIVER_MINOR	0

static struct platform_device *tcc_drm_pdev;

#ifdef CONFIG_DRM_UT_CORE
unsigned int drm_ut_core = DRM_UT_CORE;
#else
unsigned int drm_ut_core = 0;
#endif
#ifdef CONFIG_DRM_UT_DRIVER
unsigned int drm_ut_driver = DRM_UT_DRIVER;
#else
unsigned int drm_ut_driver = 0;
#endif
#ifdef CONFIG_DRM_UT_KMS
unsigned int drm_ut_kms = DRM_UT_KMS;
#else
unsigned int drm_ut_kms = 0;
#endif
#ifdef CONFIG_DRM_UT_PRIME
unsigned int drm_ut_prime = DRM_UT_PRIME;
#else
unsigned int drm_ut_prime = 0;
#endif

void tcc_drm_debug_set(void)
{
	drm_debug = 0 \
				| drm_ut_core \
				| drm_ut_driver \
				| drm_ut_kms \
				| drm_ut_prime \
				;
}

static DEFINE_MUTEX(drm_component_lock);
static LIST_HEAD(drm_component_list);

struct component_dev {
	struct list_head list;
	struct device *crtc_dev;
	struct device *conn_dev;
	enum tcc_drm_output_type out_type;
	unsigned int dev_type_flag;
};

static int tcc_drm_load(struct drm_device *dev, unsigned long flags)
{
	struct tcc_drm_private *private;
	int ret;
	int nr;

	private = kzalloc(sizeof(struct tcc_drm_private), GFP_KERNEL);
	if (!private)
		return -ENOMEM;

	INIT_LIST_HEAD(&private->pageflip_event_list);
	dev_set_drvdata(dev->dev, dev);
	dev->dev_private = (void *)private;

	/*
	 * create mapping to manage iommu table and set a pointer to iommu
	 * mapping structure to iommu_mapping of private data.
	 * also this iommu_mapping can be used to check if iommu is supported
	 * or not.
	 */
	ret = drm_create_iommu_mapping(dev);
	if (ret < 0) {
		DRM_ERROR("failed to create iommu mapping.\n");
		goto err_free_private;
	}

	drm_mode_config_init(dev);

	tcc_drm_mode_config_init(dev);

	for (nr = DEFAULT_ZPOS + 1; nr < MAX_PLANE; nr++) {
		struct drm_plane *plane;
		unsigned long possible_crtcs = (1 << MAX_CRTC) - 1;

		plane = tcc_plane_init(dev, possible_crtcs,
					  DRM_PLANE_TYPE_OVERLAY, nr);
		if (!IS_ERR(plane))
			continue;

		ret = PTR_ERR(plane);
		goto err_mode_config_cleanup;
	}

	/* setup possible_clones. */
	tcc_drm_encoder_setup(dev);

	platform_set_drvdata(dev->platformdev, dev);

	/* Try to bind all sub drivers. */
	ret = component_bind_all(dev->dev, dev);
	if (ret)
		goto err_mode_config_cleanup;

	ret = drm_vblank_init(dev, dev->mode_config.num_crtc);
	if (ret)
		goto err_unbind_all;

	/* Probe non kms sub drivers and virtual display driver. */
	ret = tcc_drm_device_subdrv_probe(dev);
	if (ret)
		goto err_cleanup_vblank;

	/*
	 * enable drm irq mode.
	 * - with irq_enabled = true, we can use the vblank feature.
	 *
	 * P.S. note that we wouldn't use drm irq handler but
	 *	just specific driver own one instead because
	 *	drm framework supports only one irq handler.
	 */
	dev->irq_enabled = true;

	/*
	 * with vblank_disable_allowed = true, vblank interrupt will be disabled
	 * by drm timer once a current process gives up ownership of
	 * vblank event.(after drm_vblank_put function is called)
	 */
	dev->vblank_disable_allowed = true;

	/* init kms poll for handling hpd */
	drm_kms_helper_poll_init(dev);

	/* force connectors detection */
	drm_helper_hpd_irq_event(dev);

	return 0;

err_cleanup_vblank:
	drm_vblank_cleanup(dev);
err_unbind_all:
	component_unbind_all(dev->dev, dev);
err_mode_config_cleanup:
	drm_mode_config_cleanup(dev);
	drm_release_iommu_mapping(dev);
err_free_private:
	kfree(private);

	return ret;
}

static int tcc_drm_unload(struct drm_device *dev)
{
	tcc_drm_device_subdrv_remove(dev);

	tcc_drm_fbdev_fini(dev);
	drm_kms_helper_poll_fini(dev);

	drm_vblank_cleanup(dev);
	component_unbind_all(dev->dev, dev);
	drm_mode_config_cleanup(dev);
	drm_release_iommu_mapping(dev);

	kfree(dev->dev_private);
	dev->dev_private = NULL;

	return 0;
}

static int tcc_drm_suspend(struct drm_device *dev, pm_message_t state)
{
	struct drm_connector *connector;

	drm_modeset_lock_all(dev);
	list_for_each_entry(connector, &dev->mode_config.connector_list, head) {
		int old_dpms = connector->dpms;

		if (connector->funcs->dpms)
			connector->funcs->dpms(connector, DRM_MODE_DPMS_OFF);

		/* Set the old mode back to the connector for resume */
		connector->dpms = old_dpms;
	}
	drm_modeset_unlock_all(dev);

	return 0;
}

static int tcc_drm_resume(struct drm_device *dev)
{
	struct drm_connector *connector;

	drm_modeset_lock_all(dev);
	list_for_each_entry(connector, &dev->mode_config.connector_list, head) {
		if (connector->funcs->dpms) {
			int dpms = connector->dpms;

			connector->dpms = DRM_MODE_DPMS_OFF;
			connector->funcs->dpms(connector, dpms);
		}
	}
	drm_modeset_unlock_all(dev);

	drm_helper_resume_force_mode(dev);

	return 0;
}

static int tcc_drm_open(struct drm_device *dev, struct drm_file *file)
{
	struct drm_tcc_file_private *file_priv;
	int ret;

	file_priv = kzalloc(sizeof(*file_priv), GFP_KERNEL);
	if (!file_priv)
		return -ENOMEM;

	file->driver_priv = file_priv;

	ret = tcc_drm_subdrv_open(dev, file);
	if (ret)
		goto err_file_priv_free;

	return ret;

err_file_priv_free:
	kfree(file_priv);
	file->driver_priv = NULL;
	return ret;
}

static void tcc_drm_preclose(struct drm_device *dev,
					struct drm_file *file)
{
	tcc_drm_subdrv_close(dev, file);
}

static void tcc_drm_postclose(struct drm_device *dev, struct drm_file *file)
{
	struct tcc_drm_private *private = dev->dev_private;
	struct drm_pending_vblank_event *v, *vt;
	struct drm_pending_event *e, *et;
	unsigned long flags;

	if (!file->driver_priv)
		return;

	/* Release all events not unhandled by page flip handler. */
	spin_lock_irqsave(&dev->event_lock, flags);
	list_for_each_entry_safe(v, vt, &private->pageflip_event_list,
			base.link) {
		if (v->base.file_priv == file) {
			list_del(&v->base.link);
			drm_vblank_put(dev, v->pipe);
			v->base.destroy(&v->base);
		}
	}

	/* Release all events handled by page flip handler but not freed. */
	list_for_each_entry_safe(e, et, &file->event_list, link) {
		list_del(&e->link);
		e->destroy(e);
	}
	spin_unlock_irqrestore(&dev->event_lock, flags);

	kfree(file->driver_priv);
	file->driver_priv = NULL;
}

static void tcc_drm_lastclose(struct drm_device *dev)
{
	tcc_drm_fbdev_restore_mode(dev);
}

static const struct vm_operations_struct tcc_drm_gem_vm_ops = {
	.fault = tcc_drm_gem_fault,
	.open = drm_gem_vm_open,
	.close = drm_gem_vm_close,
};

static const struct drm_ioctl_desc tcc_ioctls[] = {
	DRM_IOCTL_DEF_DRV(TCC_GEM_CREATE, tcc_drm_gem_create_ioctl,
			DRM_UNLOCKED | DRM_AUTH),
	DRM_IOCTL_DEF_DRV(TCC_GEM_GET,
			tcc_drm_gem_get_ioctl, DRM_UNLOCKED),
};

static const struct file_operations tcc_drm_driver_fops = {
	.owner		= THIS_MODULE,
	.open		= drm_open,
	.mmap		= tcc_drm_gem_mmap,
	.poll		= drm_poll,
	.read		= drm_read,
	.unlocked_ioctl	= drm_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = drm_compat_ioctl,
#endif
	.release	= drm_release,
};

static struct drm_driver tcc_drm_driver = {
	.driver_features	= DRIVER_MODESET | DRIVER_GEM | DRIVER_PRIME,
	.load			= tcc_drm_load,
	.unload			= tcc_drm_unload,
	.suspend		= tcc_drm_suspend,
	.resume			= tcc_drm_resume,
	.open			= tcc_drm_open,
	.preclose		= tcc_drm_preclose,
	.lastclose		= tcc_drm_lastclose,
	.postclose		= tcc_drm_postclose,
	.set_busid		= drm_platform_set_busid,
	.get_vblank_counter	= drm_vblank_count,
	.enable_vblank		= tcc_drm_crtc_enable_vblank,
	.disable_vblank		= tcc_drm_crtc_disable_vblank,
	.gem_free_object	= tcc_drm_gem_free_object,
	.gem_vm_ops		= &tcc_drm_gem_vm_ops,
	.dumb_create		= tcc_drm_gem_dumb_create,
	.dumb_map_offset	= tcc_drm_gem_dumb_map_offset,
	.dumb_destroy		= drm_gem_dumb_destroy,
	.prime_handle_to_fd	= drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle	= drm_gem_prime_fd_to_handle,
	.gem_prime_export	= tcc_dmabuf_prime_export,
	.gem_prime_import	= tcc_dmabuf_prime_import,
	.ioctls			= tcc_ioctls,
	.num_ioctls		= ARRAY_SIZE(tcc_ioctls),
	.fops			= &tcc_drm_driver_fops,
	.name	= DRIVER_NAME,
	.desc	= DRIVER_DESC,
	.date	= DRIVER_DATE,
	.major	= DRIVER_MAJOR,
	.minor	= DRIVER_MINOR,
};

#ifdef CONFIG_PM_SLEEP
static int tcc_drm_sys_suspend(struct device *dev)
{
	struct drm_device *drm_dev = dev_get_drvdata(dev);
	pm_message_t message;

	if (pm_runtime_suspended(dev) || !drm_dev)
		return 0;

	message.event = PM_EVENT_SUSPEND;
	return tcc_drm_suspend(drm_dev, message);
}

static int tcc_drm_sys_resume(struct device *dev)
{
	struct drm_device *drm_dev = dev_get_drvdata(dev);

	if (pm_runtime_suspended(dev) || !drm_dev)
		return 0;

	return tcc_drm_resume(drm_dev);
}
#endif

static const struct dev_pm_ops tcc_drm_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(tcc_drm_sys_suspend, tcc_drm_sys_resume)
};

int tcc_drm_component_add(struct device *dev,
				enum tcc_drm_device_type dev_type,
				enum tcc_drm_output_type out_type)
{
	struct component_dev *cdev;

	if (dev_type != TCC_DEVICE_TYPE_CRTC &&
			dev_type != TCC_DEVICE_TYPE_CONNECTOR) {
		DRM_ERROR("invalid device type.\n");
		return -EINVAL;
	}

	mutex_lock(&drm_component_lock);

	/*
	 * Make sure to check if there is a component which has two device
	 * objects, for connector and for encoder/connector.
	 * It should make sure that crtc and encoder/connector drivers are
	 * ready before tcc drm core binds them.
	 */
	list_for_each_entry(cdev, &drm_component_list, list) {
		if (cdev->out_type == out_type) {
			/*
			 * If crtc and encoder/connector device objects are
			 * added already just return.
			 */
			if (cdev->dev_type_flag == (TCC_DEVICE_TYPE_CRTC |
						TCC_DEVICE_TYPE_CONNECTOR)) {
				mutex_unlock(&drm_component_lock);
				return 0;
			}

			if (dev_type == TCC_DEVICE_TYPE_CRTC) {
				cdev->crtc_dev = dev;
				cdev->dev_type_flag |= dev_type;
			}

			if (dev_type == TCC_DEVICE_TYPE_CONNECTOR) {
				cdev->conn_dev = dev;
				cdev->dev_type_flag |= dev_type;
			}

			mutex_unlock(&drm_component_lock);
			return 0;
		}
	}

	mutex_unlock(&drm_component_lock);

	cdev = kzalloc(sizeof(*cdev), GFP_KERNEL);
	if (!cdev)
		return -ENOMEM;

	if (dev_type == TCC_DEVICE_TYPE_CRTC)
		cdev->crtc_dev = dev;
	if (dev_type == TCC_DEVICE_TYPE_CONNECTOR)
		cdev->conn_dev = dev;

	cdev->out_type = out_type;
	cdev->dev_type_flag = dev_type;

	mutex_lock(&drm_component_lock);
	list_add_tail(&cdev->list, &drm_component_list);
	mutex_unlock(&drm_component_lock);

	return 0;
}

void tcc_drm_component_del(struct device *dev,
				enum tcc_drm_device_type dev_type)
{
	struct component_dev *cdev, *next;

	mutex_lock(&drm_component_lock);

	list_for_each_entry_safe(cdev, next, &drm_component_list, list) {
		if (dev_type == TCC_DEVICE_TYPE_CRTC) {
			if (cdev->crtc_dev == dev) {
				cdev->crtc_dev = NULL;
				cdev->dev_type_flag &= ~dev_type;
			}
		}

		if (dev_type == TCC_DEVICE_TYPE_CONNECTOR) {
			if (cdev->conn_dev == dev) {
				cdev->conn_dev = NULL;
				cdev->dev_type_flag &= ~dev_type;
			}
		}

		/*
		 * Release cdev object only in case that both of crtc and
		 * encoder/connector device objects are NULL.
		 */
		if (!cdev->crtc_dev && !cdev->conn_dev) {
			list_del(&cdev->list);
			kfree(cdev);
		}

		break;
	}

	mutex_unlock(&drm_component_lock);
}

static int compare_dev(struct device *dev, void *data)
{
	return dev == (struct device *)data;
}

static struct component_match *tcc_drm_match_add(struct device *dev)
{
	struct component_match *match = NULL;
	struct component_dev *cdev;
	unsigned int attach_cnt = 0;

	mutex_lock(&drm_component_lock);

	/* Do not retry to probe if there is no any kms driver regitered. */
	if (list_empty(&drm_component_list)) {
		mutex_unlock(&drm_component_lock);
		return ERR_PTR(-ENODEV);
	}

	list_for_each_entry(cdev, &drm_component_list, list) {
		/*
		 * Add components to master only in case that crtc and
		 * encoder/connector device objects exist.
		 */
		if (!cdev->crtc_dev || !cdev->conn_dev)
			continue;

		attach_cnt++;

		mutex_unlock(&drm_component_lock);

		/*
		 * lcd and dpi modules have same device object so add
		 * only crtc device object in this case.
		 */
		if (cdev->crtc_dev == cdev->conn_dev) {
			component_match_add(dev, &match, compare_dev,
						cdev->crtc_dev);
			goto out_lock;
		}

		/*
		 * Do not chage below call order.
		 * crtc device first should be added to master because
		 * connector/encoder need pipe number of crtc when they
		 * are created.
		 */
		component_match_add(dev, &match, compare_dev, cdev->crtc_dev);
		component_match_add(dev, &match, compare_dev, cdev->conn_dev);

out_lock:
		mutex_lock(&drm_component_lock);
	}

	mutex_unlock(&drm_component_lock);

	return attach_cnt ? match : ERR_PTR(-EPROBE_DEFER);
}

static int tcc_drm_bind(struct device *dev)
{
	return drm_platform_init(&tcc_drm_driver, to_platform_device(dev));
}

static void tcc_drm_unbind(struct device *dev)
{
	drm_put_dev(dev_get_drvdata(dev));
}

static const struct component_master_ops tcc_drm_ops = {
	.bind		= tcc_drm_bind,
	.unbind		= tcc_drm_unbind,
};

static int tcc_drm_platform_probe(struct platform_device *pdev)
{
	struct component_match *match;
	int ret;

	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	tcc_drm_driver.num_ioctls = ARRAY_SIZE(tcc_ioctls);

	match = tcc_drm_match_add(&pdev->dev);
	if (IS_ERR(match)) {
		ret = PTR_ERR(match);
		goto err_unregister_hdmi_drv;
	}

	ret = component_master_add_with_match(&pdev->dev, &tcc_drm_ops,
						match);
	if (ret < 0) {
		goto err_unregister_hdmi_drv;
	}

	return ret;

err_unregister_hdmi_drv:
	platform_driver_unregister(&lcd_driver);

	return ret;
}

static int tcc_drm_platform_remove(struct platform_device *pdev)
{
	platform_driver_unregister(&lcd_driver);

	component_master_del(&pdev->dev, &tcc_drm_ops);
	return 0;
}

static struct platform_driver tcc_drm_platform_driver = {
	.probe	= tcc_drm_platform_probe,
	.remove	= tcc_drm_platform_remove,
	.driver	= {
		.owner	= THIS_MODULE,
		.name	= "tcc-drm",
		.pm	= &tcc_drm_pm_ops,
	},
};

static int tcc_drm_init(void)
{
	struct platform_device *pdev;
	int ret;

	DRM_DEBUG_DRIVER("%s\n", __FILE__);
	tcc_drm_debug_set();

	/*
	 * Register device object only in case of Telechips SoC.
	 *
	 * Below codes resolves temporarily infinite loop issue incurred
	 * by Telechips drm driver when using multi-platform kernel.
	 * So these codes will be replaced with more generic way later.
	 */
	if (!of_machine_is_compatible("telechips,tcc896x") &&
			!of_machine_is_compatible("telechips,tcc897x") &&
			!of_machine_is_compatible("telechips,tcc898x"))
		return -ENODEV;

	tcc_drm_pdev = platform_device_register_simple("tcc-drm", -1,
								NULL, 0);

	if (IS_ERR(tcc_drm_pdev))
		return PTR_ERR(tcc_drm_pdev);

#ifdef CONFIG_DRM_TCC_LCD
	pdev = platform_device_register_simple("tcc-drm-lcd", -1,
			NULL, 0);
	if (IS_ERR(pdev)) {
		ret = PTR_ERR(pdev);
		goto err_remove_lcd;
	}

	ret = platform_driver_register(&lcd_driver);
	if (ret < 0)
		goto err_remove_lcd;
#endif

	ret = platform_driver_register(&tcc_drm_platform_driver);
	if (ret)
		goto err_remove;

	return 0;

err_remove:
	platform_device_unregister(tcc_drm_pdev);
#ifdef CONFIG_DRM_TCC_LCD
err_remove_lcd:
	platform_device_unregister(pdev);
#endif
	return ret;
}

static void tcc_drm_exit(void)
{
#ifdef CONFIG_DRM_TCC_LCD
	platform_driver_unregister(&lcd_driver);
#endif
	platform_driver_unregister(&tcc_drm_platform_driver);
	platform_device_unregister(tcc_drm_pdev);
}

module_init(tcc_drm_init);
module_exit(tcc_drm_exit);

MODULE_AUTHOR("Telechips Inc. <linux@telechips.com>");
MODULE_DESCRIPTION("Telechips SoC DRM Driver");
MODULE_LICENSE("GPL");
