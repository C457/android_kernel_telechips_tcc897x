#
# Telechips SOC drivers
#
obj-$(CONFIG_SOC_TCC)	+= irq.o 
obj-$(CONFIG_ARCH_TCC898X)	+= timer.o pmu_wakeup.o 
ifeq ($(CONFIG_ARM64),y)
else
obj-$(CONFIG_ARCH_TCC898X)	+= pm-tcc898x.o suspend_asm.o suspend.o tcc_sram.o
endif
#obj-$(CONFIG_ARCH_TCC897X)	+= pm-tcc897x.o 
obj-$(CONFIG_PMAP)	+= pmap.o
obj-$(CONFIG_PMAP_TCC897X)	+= pmap-tcc897x.o
