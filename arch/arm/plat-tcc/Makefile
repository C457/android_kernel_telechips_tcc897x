# "Telechips Platform Common Modules"

ifneq ($(CONFIG_ARCH_TCC898X),y)
obj-$(CONFIG_ARCH_TCC)	+= cpu.o io.o nand.o proc_rotation.o timer.o
obj-$(CONFIG_ARCH_TCC)	+= poweroff.o suspend.o pmu_wakeup.o suspend_asm.o
obj-$(CONFIG_ARCH_TCC)	+= lcd_panel.o
obj-$(CONFIG_CPU_IDLE)	+= cpuidle.o
endif
