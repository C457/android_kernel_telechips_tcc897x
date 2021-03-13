# TCC Platform Support
ifdef CONFIG_SND_SOC_TCC_PCM_V10
snd-soc-tcc-pcm-objs := tcc-pcm-v10.o tcc-pcm-v10-sub1.o tcc-pcm-v10-sub2.o tcc-pcm-v10-sub3.o tcc/tca_tcchwcontrol_v10.o
else #for TCC898x, TCC802x
ifdef CONFIG_ARCH_TCC898X
snd-soc-tcc-pcm-objs := tcc-pcm-v20.o tcc-pcm-v20-sub1.o tcc/tca_tcchwcontrol_v20.o
else #for TCC802x
snd-soc-tcc-pcm-objs := tcc-pcm-v20.o tcc-pcm-v20-sub1.o tcc-pcm-v20-sub2.o tcc-pcm-v20-sub3.o tcc/tca_tcchwcontrol_v20.o
endif
endif

ifdef CONFIG_ARCH_TCC897X
snd-soc-tcc-i2s-objs := tcc-i2s.o tcc-spdif.o tcc-cdif.o
else #for TCC898x
snd-soc-tcc-i2s-objs := tcc-i2s.o tcc-spdif.o
endif
snd-soc-tcc-loopback-objs := tcc-loopback.o
obj-$(CONFIG_SND_TCC_AUDIO_DSP) += tcc_board_dsp.o tcc-dsp-api.o tcc-i2s-dsp.o tcc-pcm-dsp.o tcc-pcm-dsp-sub1.o tcc-pcm-dsp-sub2.o
obj-$(CONFIG_SND_SOC_TCC) += snd-soc-tcc-pcm.o
obj-$(CONFIG_SND_SOC_TCC_I2S) += snd-soc-tcc-i2s.o
obj-$(CONFIG_SND_TCC_LOOPBACK) += snd-soc-tcc-loopback.o

# TCC Machine Support
snd-soc-tcc-board-wm8731-objs := tcc_board_wm8731.o
snd-soc-tcc-board-wm8581-objs := tcc_board_wm8581.o
snd-soc-tcc-board-wm8524-objs := tcc_board_wm8524.o
snd-soc-daudio-board-fm1288-objs := daudio_board_fm1288.o

obj-$(CONFIG_SND_SOC_TCC_BOARD_LCN2) += tcc_board_lcn2.o
obj-$(CONFIG_SND_SOC_TCC5700_BOARD_HD_DMB) += tcc5700_board_hd-dmb.o
obj-$(CONFIG_SND_SOC_TCC8021_BOARD_EVM) += tcc8021_board_evm.o

obj-$(CONFIG_SND_SOC_TCC_BOARD_WM8524) += snd-soc-tcc-board-wm8524.o
obj-$(CONFIG_SND_SOC_TCC_BOARD_WM8731) += snd-soc-tcc-board-wm8731.o
obj-$(CONFIG_SND_SOC_TCC_BOARD_WM8581) += snd-soc-tcc-board-wm8581.o
obj-$(CONFIG_SND_SOC_DAUDIO_BOARD) += snd-soc-daudio-board-fm1288.o
