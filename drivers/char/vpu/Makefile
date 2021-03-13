obj-$(CONFIG_TCC_VPU_DRV) += hevc_lib/
obj-$(CONFIG_TCC_VPU_DRV) += vpu_lib/
obj-$(CONFIG_TCC_VPU_DRV) += jpu_lib/
obj-$(CONFIG_TCC_VPU_DRV) += vp9_lib/

obj-$(CONFIG_TCC_VPU_DRV) += vpu.o

#ifeq ($(CONFIG_SUPPORT_TCC_HEVC),y)
vpu-y += hevc_mgr.o
vpu-y += hevc_mgr_sys.o
#endif

#ifeq ($(CONFIG_SUPPORT_TCC_VP9),y)
vpu-y += vp9_mgr.o
vpu-y += vp9_mgr_sys.o
#endif

#ifeq ($(CONFIG_SUPPORT_TCC_JPU),y)
vpu-y += jpu_mgr.o
vpu-y += jpu_mgr_sys.o
#endif

vpu-y += vpu_mgr.o vpu_buffer.o vpu_mem.o vpu_etc.o
vpu-y += vpu_mgr_sys.o
vpu-y += vpu_devices.o
vpu-y += vpu_dec.o
vpu-y += vpu_enc.o

obj-$(CONFIG_TCC_VPU_DRV) += hevc_dev.o
hevc_dev-y += hevc_devices.o

obj-$(CONFIG_TCC_VPU_DRV) += vp9_dev.o
vp9_dev-y += vp9_devices.o

obj-$(CONFIG_TCC_VPU_DRV) += jpu_dev.o
jpu_dev-y += jpu_devices.o
