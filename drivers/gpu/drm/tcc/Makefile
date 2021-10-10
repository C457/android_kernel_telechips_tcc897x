#
# Makefile for the drm device driver.  This driver provides support for the
# Direct Rendering Infrastructure (DRI) in XFree86 4.1.0 and higher.

ccflags-y := -Iinclude/drm -Idrivers/gpu/drm/tcc
tccdrm-y := tcc_drm_drv.o \
			tcc_drm_encoder.o \
			tcc_drm_crtc.o \
			tcc_drm_fbdev.o \
			tcc_drm_fb.o \
			tcc_drm_buf.o \
			tcc_drm_gem.o \
			tcc_drm_core.o \
			tcc_drm_plane.o \
			tcc_drm_dpi.o

tccdrm-$(CONFIG_DRM_TCC_IOMMU) += tcc_drm_iommu.o
tccdrm-$(CONFIG_DRM_TCC_DMABUF) += tcc_drm_dmabuf.o
tccdrm-$(CONFIG_DRM_TCC_LCD)	+= tcc_drm_lcd.o

obj-$(CONFIG_DRM_TCC)		+= tccdrm.o
