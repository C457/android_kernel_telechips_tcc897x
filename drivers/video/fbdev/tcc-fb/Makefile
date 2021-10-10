obj-$(CONFIG_LCD_SELECT)  += lcd_select.o

#obj-$(CONFIG_LCD_FLD0800)  += lcd_FLD0800.o
#obj-$(CONFIG_LCD_ED090NA) 	+= lcd_ED090NA.o
#obj-$(CONFIG_LCD_AT070TN93) += lcd_at070tn93.o
obj-$(CONFIG_FB_TCC_OVERLAY)	+= tcc_overlay.o
ifdef CONFIG_ANDROID
EXTRA_CFLAGS += -Idrivers/staging/android
obj-$(CONFIG_TCC_VIOC_CONTROLLER)	 += tcc_displayblock.o
endif
obj-$(CONFIG_TCC_VIDEO_DISPLAY_BY_VSYNC_INT) += tcc_vsync.o
obj-$(CONFIG_TCC_VIOC_CONTROLLER)	 += viqe.o tcc_ccfb.o tcc_vioc_fb.o tcc_vioc_interface.o tca_display_config.o vioc/
#obj-$(CONFIG_LCD_HDMI1920X720)	+= hdmi_1280x720.o hdmi_1920x720.o
obj-$(CONFIG_LCD_HDMI1920X1080)	+= hdmi_1920x1080.o
obj-$(CONFIG_LCD_HDMI4K)		+= hdmi_4k.o
obj-$(CONFIG_TCC_VIDEO_DISPLAY_DEINTERLACE_MODE)	+= viqe/
EXTRA_CFLAGS += -Idrivers/video/fbdev/tcc-fb/viqe
obj-$(CONFIG_HIBERNATION)	+= tcc_qb_fb.o

obj-$(CONFIG_TCC_MIPI)	      += tcc_mipi_dsi.o
