#ifndef __LINUX_IBOX_API__IOCTL_PIN_ID_H__
#define __LINUX_IBOX_API__IOCTL_PIN_ID_H__

/*
 * WARN WARN WARN WARN
 *
 *    DO NOT REMOVE AND CHANGE ANY ONE OF THESE after first SDK release.
 *    For backward/forward binary compability.
 *
 *    Just add at end of this list for new control !!
 */
#define __i_FN(x) IOPIN_##x
typedef enum {
	_____________________________MARK_START_OF_BASE = 0,

	IOPIN_DET_PREMIUM_AVN,     /* iBox, detect Perimium or Standard */
	IOPIN_DET_DH_AVN,          /* iBox, detect Monitor is seperated */

	IOPIN_DET_ETHERNET_DET,    /* from Ethernet, detect existing or not */
	IOPIN_CTL_DBGETH_RESET,    /* to debugboard, FEC reset control */
	IOPIN_DET_DBGETH_INT,      /* from debugboard, FEC interrupt ? */

	IOPIN_CTL_FAN_ON,          /* not used ? */

	IOPIN_DET_BATT_DET_4_5V,   /* from unknown, unknown */

	IOPIN_CTL_CPU_BOOT_OK,     /* to Micom, CPU booting complete */
	IOPIN_DET_CPU_BOOT_MODE,   /* from Micom, normal or update booting */

	IOPIN_CTL_UPDATE_IND,      /* to LED, LED control */

	IOPIN_CTL_SUB_MCU_UPDATE,  /* to Micom, start to update */
	IOPIN_DET_USB_UPDATE_DET,  /* from USB, detect update mode or not */

	IOPIN_DET_SUBMICOM_INT,    /* from SubMicom, SPI data interrupt */

	IOPIN_CTL_LVDS_SSEN,       /* to LVDS IC, SSEN control */
	IOPIN_DET_LVDS_LINE_FAULT, /* from LVDS, detect line fault */

	_____________________________MARK_END_OF_BASE = 20,

	IOPIN_CTL_DMB_RESET,       /* to DMB?, reset control */
	IOPIN_DET_DMB_INT,         /* from DMB?, interrupt sense */

	IOPIN_DET_XM_ANT_OPEN,     /* from XM, antenna open sense */
	IOPIN_DET_XM_ANT_SHORT,    /* from XM, antenna short sense */

	IOPIN_CTL_HFK_POWER,	   /* to Handsfree DSP, Vcc control */
	IOPIN_CTL_HFK_POWERDOWN,   /* to Handsfree DSP, PowerDown control */
	IOPIN_CTL_HFK_RESET,       /* to Handsfree DSP, reset control */

	IOPIN_CTL_NAVI_VOICE_ON,   /* to Micom, start up Navigation sound */
	IOPIN_CTL_VOICE_IN_MIC,    /* AK5701 Mic or FM1188 HFK */

	IOPIN_CTL_TOUCH_RESET,     /* to touchscreen, reset control */
	IOPIN_DET_LVDS_TOUCH_INT,  /* from LVDS, multi-touch irq */
	IOPIN_DET_OLD_TOUCH_INT,   /* from old touchscreen, pen interrupt */

	IOPIN_CTL_DRM_RESET,       /* to iPod DRM, reset control */

	IOPIN_CTL_SDCARD_POWER,    /* to SDCARD slot, power control */
	IOPIN_DET_SDCARD_CD,       /* from SDCARD slot, card detect sense */
	IOPIN_DET_SDCARD_WP,       /* from SDCARD slot, write-protect sense */

	IOPIN_CTL_BT_WIFI_POWER,   /* to Bluetooth & WIFI, Vcc control */
	IOPIN_CTL_BT_WIFI_RESET,   /* to Bluetooth & WIFI, reset control */

	IOPIN_CTL_USBOTG_POWER,    /* to USBOTG vbus s/w, power control */
	IOPIN_DET_USBOTG_OC,       /* from USBOTG IC, over-current sense */

	IOPIN_DET_CDMA_OC,         /* from HSDPA, detect over-current */
	IOPIN_DET_CDMA_BOOT_OK,    /* from HSDPA, detect booting completion */

	IOPIN_CTL_GPS_RESET,       /* to GPS, reset control */
	IOPIN_CTL_ANT_POWER,       /* to antenna, Vcc control */
	IOPIN_DET_GPS_ANT_OPEN,    /* from GPS, antenna sense */
	IOPIN_DET_GPS_ANT_SHORT,   /* from GPS, antenna sense */

	IOPIN_CTL_AUDIOADC_POWER,  /* to AK5701 ADC, power control */
	IOPIN_CTL_VIDEOADC_RESET,  /* to TW9900 ADC, reset control */
	IOPIN_DET_VIDEOADC_INT,    /* from TW9900 ADC, sense interrupt */

	IOPIN_CTL_WIBRO_RESET,     /* to Wibro modem, reset control */
	IOPIN_CTL_WIBRO_POWER,     /* to Wibro modem, power control */
	IOPIN_DET_WIBRO_BOOT_OK,   /* from Wibro modem, boot status */

	IOPIN_DET_CDMA_AUDIO_ON,   /* from CDMA modem, detect audio */
	IOPIN_DET_CDMA_AIRBAG,     /* from CDMA mode, detect AirBag */

	IOPIN_CTL_GPS_BOOT_LOADER_MODE, /* to GPS ? */
	IOPIN_CTL_MIC_SW2,         /* unknown */

	IOPIN_CTL_AIRBAG_DIG_PULSE, /* unknown */
	IOPIN_DET_AIRBAG_DIG_KEY,   /* unknown */

	IOPIN_CTL_INIC_MLB_CONTROL, /* to INIC, unknown */

	/* rev.1 */

	IOPIN_DET_REGION1_AVN,      /* >= rev.1 board, [region2:region1] */
	IOPIN_DET_REGION2_AVN,      /* 00:Korea 01:NA 10:China 11:Europe */

	/* standard rev.1. [MIC_SW2:MIC_SW1]
	 *  00: CODEC MIC, 01: FM1188_MIC, 10: BT_Module, 11: Open
	 */
	IOPIN_CTL_VOICE_IN_MIC2,

	IOPIN_CTL_WIFI_RESET,      /* to Wifi, reset control */
	IOPIN_CTL_BT_RESET,        /* to BT, reset control */
	IOPIN_DET_GPS_UPDATE_MODE, /* from GPS, running mode */

	IOPIN_DET_ACC_CHECK,	/* unknown */
	IOPIN_CTL_ADC_CLOCK_CONTROL,	/* control AK5701's MCLK, High : Enable*/

	/* Standard Rev.4 */
	IOPIN_CTL_WIFI_EEPROM_WP,
	IOPIN_CTL_BT_EEPROM_WP,
	IOPIN_CTL_CPI_I2S_SEL,
	IOPIN_CTL_BT_MODEM_SEL,
	IOPIN_CTL_SW_HANDSFREE_ON,	
	IOPIN_CTL_ADC_MCLK_ON,

	/* Standard Rev.5 */
	IOPIN_CTL_USB_OTG_ID, /* to XX, L:host H:device */
	IOPIN_CTL_CPU_UART_CTL1, /* to DAB: see document */
	IOPIN_CTL_ECSPI1_XM_MOSI, /* to DAB: see document */
	IOPIN_CTL_ECSPI1_XM_MISO, /* to DAB, L:normal H:debug */
	IOPIN_DET_TOUCH_JIG, /* from XX, L:JIG H:normal */

	IOPIN_CTL_SUBMICOM_TRS,
	IOPIN_CTL_PCM_MODE_SEL0,
	IOPIN_CTL_PCM_MODE_SEL1,

	/*  standard ibox:  L: GPS, H: RTC
	 *  premier ibox:   L: 60Hz LCD,  H: 30Hz LCD
	 */
	IOPIN_CPU_BD_VER_4,

	/* North America SXM gpio control */
	IOPIN_CTL_SXM_RESET,
	IOPIN_CTL_SXM_SHUTDOWN,

	/* add new controls above this line */

	IOPIN_NR_KNOWN,
} ibox_iopin_id_t;

#endif /* __LINUX_IBOX_API__IOCTL_PIN_ID_H__ */
