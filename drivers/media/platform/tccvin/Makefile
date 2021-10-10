# 
# Makefile for Telechips VIN drivers.
#
EXTRA_CFLAGS = -I$(src)/..

obj-$(CONFIG_VIDEO_TCC_VIN) += tcc_vin_v4l2.o \
								tcc_vin_core.o \

obj-$(CONFIG_VIDEO_SENSOR) += tcc_vin_hw.o \
								tcc_vin_i2c.o

#
# Sensor module driver.
#
obj-$(CONFIG_VIDEO_SENSOR) += sensor/

