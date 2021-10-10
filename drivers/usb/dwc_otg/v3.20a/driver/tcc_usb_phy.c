/*
 *
 */
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <asm/io.h>
#include <linux/platform_data/pca953x.h>

#include "tcc_usb_def.h"
#include "tcc_usb_phy.h"

#include <linux/delay.h>

/** TCC897X USB OTG PHY Configuration Register(USBOTG_PCFG) */

/**
 * TCC8975 Stick board test results.
 * The voltage drop is 0.2 V.
 * USBOTG_PCFG1_OTGT = 0x00 (-12 %): Disconnection threshold = 4.76 V (Power supply)
 */
//==============================================================================================================
// Caution : USBPHY_IGNORE_VBUS_COMPARATOR will be ignore overcurrent change event.(valid always)
//           As the result, OTG core cannot disconnect when overcurrent occurred.
//           And, VBUS remains "on" if current role of OTG is "host".
//==============================================================================================================
#ifndef CONFIG_TCC_DWC_HS_ELECT_TST
#define USBPHY_IGNORE_VBUS_COMPARATOR
#endif

/*** FSEL: Clock table 	*****/
//	0b000		9.6		MHz	//
//	0b001		10		MHz	//
//	0b010		12		MHz	//
//	0b011		19.2	MHz	//
//	0b100		20		MHz	//
//	0b101		24		MHz	//
//	0b111		50		MHz	//
/****************************/
#define USBOTG_PCFG0_PHY_POR					Hw31					// PHY Power-on Reset; (1:Reset)(0:Normal)
#define USBOTG_PCFG0_PR							Hw30					// Port Reset; (1:enable)(0:disable)
#define USBOTG_PCFG0_CM							Hw27					// COMMONONN; (1:power down)(0:don't power down)
#define USBOTG_PCFG0_SDI						Hw24					// IDDQ Test Enable; (1:enable)(0:disable)
#define USBOTG_PCFG0_IRQ_PHYVLDEN 				Hw20					// PHY Valid Enable; (1:enable)(0:disable)
#define USBOTG_PCFG0_VBDS						Hw16					// External VBUS Valid Select; (1:external)(0:internal)
#define USBOTG_PCFG0_VBDSRC						Hw15					// Select External VBUS Valid Source; (1:PCFG0.VBD)(0:GPIO)
#define USBOTG_PCFG0_VBD						Hw14					// External VBUS Valid Indicator; (1:valid)(0:not valid)
#define USBOTG_PCFG0_IDPU						Hw8						// Analog ID Input Sample Enable; (1:enable)(0:disable)
#define USBOTG_PCFG0_RCS(x)						((x&0x3)<<4)			// Internal Clock source is used for PHY reference clock.(Only avail. 0b01)(others: forbidden)
#define USBOTG_PCFG0_FSEL(x)					(x&0x7)					// Reference clock frequency;

#define USBOTG_PCFG1_TXPPT						Hw28					// HS Transmitter Pre-Emphasis Duration Control; (1:short)(0:Default)
#define USBOTG_PCFG1_TXFSLST(x)					((x&0xF)<<24)			// FS/LS Pull-Up Resistance Adjustment;
#define USBOTG_PCFG1_SQRXT(x)					((x&0x7)<<20)			// Squelch Threshold Tune;
#define USBOTG_PCFG1_OTGT(x)					((x&0x7)<<16)			// VBUS Valid Threshold Adjustment;
#define USBOTG_PCFG1_TXHSXVT(x)					((x&0x3)<<14)			// Transmitter High Speed Crossover Adjustment;
#define USBOTG_PCFG1_TXREST(x)					((x&0x3)<<12)			// USB Source Impedance Adjustment;
#define USBOTG_PCFG1_TXRT(x)					((x&0x3)<<10)			// HS Transmitter Rise/Fall Time Adjustment;
#define USBOTG_PCFG1_TPT(x)						((x&0x3)<<8)			// HS Transmitter Pre-Emphasis Current Control;
#define USBOTG_PCFG1_CDT(x)						((x&0x7)<<4)			// Disconnect Threshold Adjustment;
#define USBOTG_PCFG1_TXVRT(x)					(x&0xF)					// HS DC Voltage Level Adjustment;

#define USBOTG_PCFG2_CHGDET						Hw22					// Battery Charger Detection Output;
#define USBOTG_PCFG2_ADPPRB						Hw21					// ADP Probe Indicator; (1:above)(0:below)
#define USBOTG_PCFG2_ADPSNS						Hw20					// ADP Sense Indicator; (1:above)(0:below)
#define USBOTG_PCFG2_ACAENB						Hw13					// ACA ID_OTG Pin Resistance Detection Enable; (1:enable)(0:disable)
#define USBOTG_PCFG2_CHRGSEL					Hw10					// Battery Charging Source Select;
#define USBOTG_PCFG2_VDATSRCENB					Hw9						// Battery Charging Sourcing Select; (1:enable)(0:disable)
#define USBOTG_PCFG2_VDATDETENB					Hw8						// Battery Charging Attach/Connect Detection Enable; (1:enable)(0:disable)
#define USBOTG_PCFG2_DCDENB						Hw7						// Data Contact Detection Enable; (1:enable)(0:disable)
#define USBOTG_PCFG2_UART_TX_DP					Hw1						// Connect UART TX to DP; (1:TX to DP)(0:RX to DP)
#define USBOTG_PCFG2_UART_EN					Hw0						// UART via USB2.0 PHY Enable; (1:enable)(0:disable)

#define USBOTG_PCFG3_TDOSEL						Hw29					// Test Data Out Select;
#define USBOTG_PCFG3_TCK						Hw28					// Clocking Signal for TESTDATAIN[7:0];
#define USBOTG_PCFG3_TDA(x)						((x&0xF)<<24)			// Test Interface Register Address;
#define USBOTG_PCFG3_TDI(x)						((x&0xFF)<<16)			// TESTDATAIN[7:0] Test Data Write Bus;
#define USBOTG_PCFG3_TDO(x)						((x&0xF)<<12)			// USB PHY Test Data Out;

#define USBOTG_PCFG4_IRQ_CLR					Hw31					// Clear PHY IRQ;
#define USBOTG_PCFG4_IRQ_VBUSEN 				Hw29					// VBUS Detect IRQ; (1:enable)(0:disable)

#define USBOTG_LSTS_BC_CHIRP_ON					Hw5						// Chirp_ON;

#define USBOTG_LCFG0_PRSTN	 					Hw29					//
#define USBOTG_LCFG0_ADPRSTN					Hw28

#define TCC_MUX_H_SWRST				(1<<4)		/* Host Controller in OTG MUX S/W Reset */
#define TCC_MUX_H_CLKMSK			(1<<3)		/* Host Controller in OTG MUX Clock Enable */
#define TCC_MUX_O_SWRST				(1<<2)		/* OTG Controller in OTG MUX S/W Reset */
#define TCC_MUX_O_CLKMSK			(1<<1)		/* OTG Controller in OTG MUX Clock Enable */
#define TCC_MUX_OPSEL				(1<<0)		/* OTG MUX Controller Select */
#define TCC_MUX_O_SELECT			(TCC_MUX_O_SWRST|TCC_MUX_O_CLKMSK)
#define TCC_MUX_H_SELECT			(TCC_MUX_H_SWRST|TCC_MUX_H_CLKMSK)

/** TCC897X USB OTG PHY Configuration Register(USBOTG_PCFG) End */

#if defined (CONFIG_DYNAMIC_DC_LEVEL_ADJUSTMENT)
#define USBOTG_PCFG1_TXVRT_MASK	0xF
#define USBOTG_PCFG1_TXVRT_SHIFT	0x0
int dwc_otg_phy_get_dc_level(dwc_otg_core_if_t * _core_if)
{
	tcc_dwc_otg_phy_t* tcc_phy;

	if(_core_if == NULL)
		return -1;

	tcc_phy = _core_if->tcc_phy_config;

	return ((tcc_phy->pcfg1 & USBOTG_PCFG1_TXVRT_MASK) >> USBOTG_PCFG1_TXVRT_SHIFT);
}
int dwc_otg_phy_set_dc_level(dwc_otg_core_if_t *_core_if, unsigned int level)
{
	tcc_dwc_otg_phy_t* tcc_phy;

	if(_core_if == NULL)
		return -1;

	tcc_phy = _core_if->tcc_phy_config;

	BITCSET(tcc_phy->pcfg1, USBOTG_PCFG1_TXVRT_MASK, level << USBOTG_PCFG1_TXVRT_SHIFT);

	return 0;
}
#endif

USBPHY_MODE_T USBPHY_SetMode(dwc_otg_core_if_t *_core_if, USBPHY_MODE_T mode)
{
#if !defined(CONFIG_TCC_CODESONAR_BLOCKED)
	if(!_core_if) {
		printk(KERN_ERR "%s error", __func__);
		return mode;
	}
#else
#endif
	switch (mode) {
		case USBPHY_MODE_RESET:
		{
			printk("dwc_otg PHY reset\n");
#ifdef CONFIG_TCC_DWC_OTG_HOST_MUX
			BITSET(_core_if->tcc_phy_config->otgmux,(TCC_MUX_O_SELECT| TCC_MUX_H_SELECT));
			if(_core_if->mux_own == MUX_MODE_DEVICE) {
				BITSET(_core_if->tcc_phy_config->otgmux,TCC_MUX_OPSEL);
			}
#endif /* CONFIG_TCC_DWC_OTG_HOST_MUX */

			_core_if->tcc_phy_config->pcfg0 =
				(USBOTG_PCFG0_PHY_POR
				| USBOTG_PCFG0_SDI
				| USBOTG_PCFG0_RCS(1)
				| USBOTG_PCFG0_FSEL(5)
				| Hw25)  // check PCFG0.b25!!!;
				& ~USBOTG_PCFG0_PR
				& ~USBOTG_PCFG0_CM
				& ~USBOTG_PCFG0_IRQ_PHYVLDEN
				& ~USBOTG_PCFG0_VBDS
				& ~USBOTG_PCFG0_VBDSRC
				& ~USBOTG_PCFG0_VBD
				& ~USBOTG_PCFG0_IDPU
				;
			_core_if->tcc_phy_config->pcfg1 =
				(USBOTG_PCFG1_TXFSLST(3)
				| USBOTG_PCFG1_SQRXT(3)
				| USBOTG_PCFG1_OTGT(0)	// -12 %
				| USBOTG_PCFG1_TXHSXVT(1)
				| USBOTG_PCFG1_TXREST(3)
				| USBOTG_PCFG1_TXRT(1)
				| USBOTG_PCFG1_TPT(3)
				| USBOTG_PCFG1_CDT(7)
#if defined(CONFIG_ARCH_TCC897X)
				| USBOTG_PCFG1_TXVRT(9))
#else
				| USBOTG_PCFG1_TXVRT(5))
#endif
				& ~USBOTG_PCFG1_TXPPT
				;

#if defined (CONFIG_DYNAMIC_DC_LEVEL_ADJUSTMENT)
			dwc_otg_phy_set_dc_level(_core_if, CONFIG_USB_HS_DC_VOLTAGE_LEVEL);
			printk("[dwc_otg] pcfg1: 0x%x txvrt: 0x%x\n",_core_if->tcc_phy_config->pcfg1,CONFIG_USB_HS_DC_VOLTAGE_LEVEL);
#endif

#ifdef CONFIG_TCC_DWC_HS_ELECT_TST
			_core_if->tcc_phy_config->pcfg2    	= 0x4 | USBOTG_PCFG2_ACAENB;
#else
			_core_if->tcc_phy_config->pcfg2  	= 0x4;
#endif /* CONFIG_TCC_DWC_HS_ELECT_TST */
			_core_if->tcc_phy_config->pcfg3 	= 0x0;
			_core_if->tcc_phy_config->pcfg4 	= 0x0;
			_core_if->tcc_phy_config->lcfg0  	= 0x30000000;
			_core_if->tcc_phy_config->lcfg0 	= 0x0;							// assert prstn, adp_reset_n
			#if defined(USBPHY_IGNORE_VBUS_COMPARATOR)
			_core_if->tcc_phy_config->lcfg0 	|= (0x3 << 21);					// forced that VBUS status is valid always.
			#endif

			BITCLR(_core_if->tcc_phy_config->pcfg0,
				(
				USBOTG_PCFG0_PHY_POR|
				Hw25|
				USBOTG_PCFG0_SDI|
				USBOTG_PCFG0_IRQ_PHYVLDEN
				)
				);		//disable PHYVALID_EN -> no irq, SIDDQ, POR
			//BITCLR(pUSBPHYCFG->PCFG0,(Hw20|Hw24|Hw25|Hw31));		//disable PHYVALID_EN -> no irq, SIDDQ, POR
			msleep(10);
			BITSET(_core_if->tcc_phy_config->lcfg0, USBOTG_LCFG0_PRSTN);					// prstn; Hw29
			#if 0
			printk("USBOTG_PCFG0: 0x%08x\nUSBOTG_PCFG1: 0x%08x\nUSBOTG_PCFG2: 0x%08x\nUSBOTG_PCFG3: 0x%08x\nUSBOTG_PCFG4: 0x%08x\nUSBOTG_LCFG0: 0x%08x\n",
				pUSBPHYCFG->PCFG0,
				pUSBPHYCFG->PCFG1,
				pUSBPHYCFG->PCFG2,
				pUSBPHYCFG->PCFG3,
				pUSBPHYCFG->PCFG4,
				pUSBPHYCFG->LCFG0);
			#endif
#if !defined(CONFIG_TCC_CODESONAR_BLOCKED)
#else
			if(_core_if != NULL)
#endif
				_core_if->phy_mode = USBPHY_MODE_RESET;
			break;
		}
		case USBPHY_MODE_OTG:
		{
			printk("dwc_otg PHY : OTG\n");
			break;
		}
		case USBPHY_MODE_ON:
		{
			printk("dwc_otg PHY ON\n");
			break;
		}
		case USBPHY_MODE_OFF:
		{
			printk("dwc_otg PHY OFF\n");
			break;
		}
		case USBPHY_MODE_START:
		{
			printk("dwc_otg PHY start\n");
			BITCLR(_core_if->tcc_phy_config->pcfg0, (USBOTG_PCFG0_PHY_POR|USBOTG_PCFG0_SDI));
#if !defined(CONFIG_TCC_CODESONAR_BLOCKED)
#else
			if(_core_if != NULL)
#endif
				_core_if->phy_mode = USBPHY_MODE_ON;
			break;
		}
		case USBPHY_MODE_STOP:
		{
			printk("dwc_otg PHY stop\n");
			BITSET(_core_if->tcc_phy_config->pcfg0, (USBOTG_PCFG0_PHY_POR|USBOTG_PCFG0_SDI));
#if !defined(CONFIG_TCC_CODESONAR_BLOCKED)
#else
			if(_core_if != NULL)
#endif
				_core_if->phy_mode = USBPHY_MODE_OFF;
			break;
		}
		default:
		{
			printk(KERN_ERR "%s error", __func__);
			break;
		}
	}

	return mode;
}

int USBPHY_IsActive(dwc_otg_core_if_t *_core_if)
{
	return ((_core_if->tcc_phy_config->pcfg0 & USBOTG_PCFG0_PHY_POR) == 0) ? TRUE : FALSE;
}


#if defined(CONFIG_TCC_DWC_OTG_V281A)
extern int dwc_otg_driver_force_init(dwc_otg_core_if_t *_core_if);
#endif
int USBPHY_Power(dwc_otg_core_if_t *_core_if, int power)
{
	#if defined(CONFIG_TCC_USB_IAP2)
	gusbcfg_data_t gusbcfg;
	gusbcfg.d32 = DWC_READ_REG32(&_core_if->core_global_regs->gusbcfg);
	#endif

	if (power) {
#if defined(CONFIG_ARCH_TCC88XX) || defined(CONFIG_ARCH_TCC93XX) || defined(CONFIG_ARCH_TCC892X) 
|| defined(CONFIG_CHIP_TCC8935S) || defined(CONFIG_CHIP_TCC8933S) || defined(CONFIG_CHIP_TCC8937S)
		#if defined(CONFIG_TCC_USB_IAP2)
		if(gusbcfg.b.force_dev_mode == 1)
			return 0;
		#endif

		if (dwc_otg_is_device_mode(_core_if)) 
			return dwc_otg_driver_force_init(_core_if);
		else
			return 0;
#else
		return USBPHY_SetMode(_core_if, USBPHY_MODE_ON);
#endif
	}else{
#if defined(CONFIG_ARCH_TCC88XX) || defined(CONFIG_ARCH_TCC93XX) || defined(CONFIG_ARCH_TCC892X) || defined(CONFIG_CHIP_TCC8935S) || defined(CONFIG_CHIP_TCC8933S) || defined(CONFIG_CHIP_TCC8937S)
		return 0;
#else
		return USBPHY_SetMode(_core_if, USBPHY_MODE_OFF);
#endif
	}
}

#ifdef CONFIG_TCC_DWC_OTG_HOST_MUX
#define mux_host_readl(r)			readl(r)
#define mux_host_writel(v,r)		writel(v,r)

#define USB_OTGH_PHY_BCFG			0x00
#define USB_OTGH_PHY_PCFG0			0x04
#define USB_OTGH_PHY_PCFG1			0x08
#define USB_OTGH_PHY_PCFG2			0x0C
#define USB_OTGH_PHY_PCFG3			0x10
#define USB_OTGH_PHY_PCFG4			0x14
#define USB_OTGH_PHY_STS			0x18
#define USB_OTGH_PHY_LCFG0			0x1C
#define USB_OTGH_PHY_LCFG1			0x20

void dwc_otg_mux_host_phy_init(dwc_otg_core_if_t *_core_if)
{
	int i;

#if defined(CONFIG_ARCH_TCC898X) || defined(CONFIG_ARCH_TCC802X)
	BITSET(_core_if->tcc_phy_config->otgmux, TCC_MUX_H_SELECT);
	if(_core_if->mux_own == MUX_MODE_HOST) {
		BITCLR(_core_if->tcc_phy_config->otgmux, TCC_MUX_OPSEL);
	}

	// Reset PHY Registers
	mux_host_writel(0x03000115, _core_if->ehci_phy_regs+USB_OTGH_PHY_PCFG0);
	mux_host_writel(0x0334D145, _core_if->ehci_phy_regs+USB_OTGH_PHY_PCFG1);
	mux_host_writel(0x4, _core_if->ehci_phy_regs+USB_OTGH_PHY_PCFG2);
	mux_host_writel(0x0, _core_if->ehci_phy_regs+USB_OTGH_PHY_PCFG3);
	mux_host_writel(0x0, _core_if->ehci_phy_regs+USB_OTGH_PHY_PCFG4);
	mux_host_writel(0x30048020, _core_if->ehci_phy_regs+USB_OTGH_PHY_LCFG0);

	// Set the POR
	mux_host_writel(mux_host_readl(_core_if->ehci_phy_regs+USB_OTGH_PHY_PCFG0) | Hw31, _core_if->ehci_phy_regs+USB_OTGH_PHY_PCFG0);
	// Set the Core Reset
	mux_host_writel(mux_host_readl(_core_if->ehci_phy_regs+USB_OTGH_PHY_LCFG0) & 0xCFFFFFFF, _core_if->ehci_phy_regs+USB_OTGH_PHY_LCFG0);

	// Wait 10 usec
	udelay(10);

	// Release POR
	mux_host_writel(mux_host_readl(_core_if->ehci_phy_regs+USB_OTGH_PHY_PCFG0) & ~(Hw31), _core_if->ehci_phy_regs+USB_OTGH_PHY_PCFG0);
	// Clear SIDDQ
	mux_host_writel(mux_host_readl(_core_if->ehci_phy_regs+USB_OTGH_PHY_PCFG0) & ~(Hw24), _core_if->ehci_phy_regs+USB_OTGH_PHY_PCFG0);
	// Set Phyvalid en
	mux_host_writel(mux_host_readl(_core_if->ehci_phy_regs+USB_OTGH_PHY_PCFG4) | Hw30, _core_if->ehci_phy_regs+USB_OTGH_PHY_PCFG4);
	// Set DP/DM (pull down)
	mux_host_writel(mux_host_readl(_core_if->ehci_phy_regs+USB_OTGH_PHY_PCFG4) | 0x1400, _core_if->ehci_phy_regs+USB_OTGH_PHY_PCFG4);

	// Wait Phy Valid Interrupt
	i = 0;
	while (i < 10000) {
		if ((mux_host_readl(_core_if->ehci_phy_regs+USB_OTGH_PHY_PCFG0) & Hw21)) break;
		i++;
		udelay(5);
	}
	printk("PHY valid check %s\x1b[0m\n",i>=9999?"fail!":"pass.");

	// Release Core Reset
	mux_host_writel(mux_host_readl(_core_if->ehci_phy_regs+USB_OTGH_PHY_LCFG0) | 0x30000000, _core_if->ehci_phy_regs+USB_OTGH_PHY_LCFG0);
#endif
}
#endif

/* end of file */
