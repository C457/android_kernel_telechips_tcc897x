# Makefile for TCCxxx v4l device driver

obj-$(CONFIG_VIDEO_V4L_SENSOR) += sensor_if.o tcc_cam_i2c.o cam_clock.o
obj-$(CONFIG_VIDEO_TCCXXX_V4L_DEVICE) += camera_core.o tcc_cam.o tcc_cam_direct_display_if.o
obj-$(CONFIG_TCC_REAR_CAMERA_DRV)	+= tcc_cam_switchmanager.o

obj-$(CONFIG_VIDEO_DUAL_CAMERA_SUPPORT) += camera/
obj-$(CONFIG_VIDEO_SINGLE_CAMERA_SUPPORT) += camera/
obj-$(CONFIG_VIDEO_TCCXX_ATV) += atv/

EXTRA_CFLAGS = -I$(src)/.. -I$(srctree)/drivers/char/tcc_early_view_control

