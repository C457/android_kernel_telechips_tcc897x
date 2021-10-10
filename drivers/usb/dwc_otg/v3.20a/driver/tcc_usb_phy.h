/*
 *
 */
#ifndef __TCC_USB_PHY_H__
#define __TCC_USB_PHY_H__

#include "tcc_usb_def.h"
#include "dwc_otg_cil.h"
#include "dwc_otg_core_if.h"

typedef enum {
	USBPHY_MODE_RESET = 0,
	USBPHY_MODE_OTG,
	USBPHY_MODE_ON,
	USBPHY_MODE_OFF,
	USBPHY_MODE_START,
	USBPHY_MODE_STOP,
	USBPHY_MODE_DEVICE_DETACH
} USBPHY_MODE_T;

extern const unsigned char * USBPHY_GetVersion(void);
extern void USBPHY_EnableClock(void);
extern USBPHY_MODE_T USBPHY_SetMode(dwc_otg_core_if_t *_core_if, USBPHY_MODE_T mode);
void dwc_otg_mux_host_phy_init(dwc_otg_core_if_t *_core_if);

extern int USBPHY_IsActive(dwc_otg_core_if_t *_core_if);
void USBPHY_DEVICE_VBUS(unsigned int valid);
#if defined(CONFIG_MACH_TCC8900) || defined(CONFIG_MACH_TCC9200S) || defined(CONFIG_MACH_TCC9201)
extern void USBPHY_SetID(unsigned int ID);
#endif

#if defined(CONFIG_MACH_TCC9300) || defined(CONFIG_MACH_TCC9300CM) || defined(CONFIG_MACH_TCC9300ST)
extern void USBPHY_PWR_Control(int port_index, int on);
#endif

extern int dwc_otg_phy_get_dc_level(dwc_otg_core_if_t * _core_if);
extern int dwc_otg_phy_set_dc_level(dwc_otg_core_if_t * _core_if, unsigned int level);
#endif /*__TCC_USB_PHY_H__*/
