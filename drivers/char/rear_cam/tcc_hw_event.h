#ifndef TCC_HW_EVENT_H
#define TCC_HW_EVENT_H 


#define EXT_INT_GPIO_SET()                      tcc_gpio_config(TCC_GPC(22), GPIO_FN0|GPIO_INPUT|GPIO_PULLDOWN)
#define EXT_INT_GPIO_READ()                     gpio_get_value(TCC_GPC(22))

#define EXT_INT_WHICH_PIN_NUM                   146
#define EXT_INT_GPIO_ACTIVE                     0
#define EXT_INT_WHICH_PIN_REG                   0x74200280

#define INT_REARCAM                             INT_EINT0

#define CIF_INTERFACE_I2C_NUM                   I2C_CH_MASTER2

#define CIF_INTERFACE_RESET_ACTIVE              0
#define CIF_INTERFACE_RESET_ON()                tcc_gpio_config(TCC_GPEXT2(20), GPIO_OUTPUT|(CIF_INTERFACE_RESET_ACTIVE?GPIO_HIGH:GPIO_LOW)) // RESET ON
#define CIF_INTERFACE_RESET_OFF()               tcc_gpio_config(TCC_GPEXT2(20), GPIO_OUTPUT|(CIF_INTERFACE_RESET_ACTIVE?GPIO_LOW:GPIO_HIGH)) // RESET Release
#define CIF_INTERFACE_RESET(ONOFF)              gpio_set(TCC_GPEXT2(20), ONOFF?(CIF_INTERFACE_RESET_ACTIVE?1:0):(CIF_INTERFACE_RESET_ACTIVE?0:1))
#define CIF_INTERFACE_CKI_GPIO_SET()            tcc_gpio_config(TCC_GPF(0), GPIO_FN1|GPIO_INPUT) // CKI
#define CIF_INTERFACE_HS_GPIO_SET()             tcc_gpio_config(TCC_GPF(1), GPIO_FN1|GPIO_INPUT) // HS
#define CIF_INTERFACE_VS_GPIO_SET()             tcc_gpio_config(TCC_GPF(2), GPIO_FN1|GPIO_INPUT) // VS
#define CIF_INTERFACE_DE_GPIO_SET()             tcc_gpio_config(TCC_GPF(12), GPIO_FN1|GPIO_INPUT) // DE     

#define	EXT_INT_RGEAR_NUM			0x00000008 
#if defined(CONFIG_TCC_VIOCMG)
extern void RCAM_ExternalINT_POL_change(void);
#else
extern void RCAM_ExternalINT_POL_change(unsigned long* pShow);
#endif
#endif
