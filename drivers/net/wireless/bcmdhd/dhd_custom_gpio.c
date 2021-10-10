/*
 * Customer code to add GPIO control during WLAN start/stop
 *
 * Portions of this code are copyright (c) 2018 Cypress Semiconductor Corporation
 * 
 * Copyright (C) 1999-2018, Broadcom Corporation
 * 
 *      Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 * 
 *      As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy and
 * distribute the resulting executable under terms of your choice, provided that
 * you also meet, for each linked independent module, the terms and conditions of
 * the license of that module.  An independent module is a module which is not
 * derived from this software.  The special exception does not apply to any
 * modifications of the software.
 * 
 *      Notwithstanding the above, under no circumstances may you combine this
 * software in any way with any other Broadcom software provided under a license
 * other than the GPL, without Broadcom's express prior written consent.
 *
 *
 * <<Broadcom-WL-IPTag/Open:>>
 *
 * $Id: dhd_custom_gpio.c 591129 2015-10-07 05:22:14Z $
 */

#include <typedefs.h>
#include <linuxver.h>
#include <osl.h>
#include <bcmutils.h>
#include <dngl_stats.h>
#include <dhd.h>
#include <dhd_linux.h>

#include <wlioctl.h>
#if defined(WL_WIRELESS_EXT)
#include <wl_iw.h>
#endif

#define WL_ERROR(x) printf x
#define WL_TRACE(x)

#if defined(OOB_INTR_ONLY)

#if defined(BCMLXSDMMC)
extern int sdioh_mmc_irq(int irq);
#endif /* (BCMLXSDMMC)  */

/* Customer specific Host GPIO defintion  */
static int dhd_oob_gpio_num = -1;

module_param(dhd_oob_gpio_num, int, 0644);
MODULE_PARM_DESC(dhd_oob_gpio_num, "DHD oob gpio number");

/* This function will return:
 *  1) return :  Host gpio interrupt number per customer platform
 *  2) irq_flags_ptr : Type of Host interrupt as Level or Edge
 *
 *  NOTE :
 *  Customer should check his platform definitions
 *  and his Host Interrupt spec
 *  to figure out the proper setting for his platform.
 *  Broadcom provides just reference settings as example.
 *
 */
int dhd_customer_oob_irq_map(void *adapter, unsigned long *irq_flags_ptr)
{
	int  host_oob_irq = 0;

#if defined(CUSTOMER_HW2)
	host_oob_irq = wifi_platform_get_irq_number(adapter, irq_flags_ptr);

#else
#if defined(CUSTOM_OOB_GPIO_NUM)
	if (dhd_oob_gpio_num < 0) {
		dhd_oob_gpio_num = CUSTOM_OOB_GPIO_NUM;
	}
#endif /* CUSTOMER_OOB_GPIO_NUM */

	if (dhd_oob_gpio_num < 0) {
		WL_ERROR(("%s: ERROR customer specific Host GPIO is NOT defined \n",
		__FUNCTION__));
		return (dhd_oob_gpio_num);
	}

	WL_ERROR(("%s: customer specific Host GPIO number is (%d)\n",
	         __FUNCTION__, dhd_oob_gpio_num));

#endif 

	return (host_oob_irq);
}
#endif 

/* Customer function to control hw specific wlan gpios */
int
dhd_customer_gpio_wlan_ctrl(void *adapter, int onoff)
{
	int err = 0;

	return err;
}

#ifdef GET_CUSTOM_MAC_ENABLE
//==========================================================
//FEATURE_MOBIS_WIFI
//jaehyungkim@mobis.co.kr 2017.05.12
//==========================================================
static char mac_address[50]; 
int mac_address_change(void)
{
	struct file *fp = NULL;
	char buf[100]={0x00,};
	int len =0;

    //=============================================================  
    //FEATURE_MOBIS_WIFI
    //jaehyungkim@mobis.co.kr 2013.12.19
    //change mac file path
    //=============================================================
    fp = filp_open("/data/data/com.hkmc.system.app.engineering/files/mac.txt", O_RDONLY, 0);
	//fp = filp_open("/data/misc/wifi/config_mac", O_RDONLY, 0);
    //=============================================================

	if (IS_ERR(fp))
	     fp = NULL;
	 
	if (fp == NULL)
		return 0;	

       len=kernel_read(fp, 0, buf, 49);

	if ((len == 0) ||(len >12))
	{
		filp_close((struct file *)fp, NULL);
		return 0;
	}

	strncpy(mac_address,buf,len);
//	printk("********** mac_address=%s\n", mac_address);
	
	filp_close((struct file *)fp, NULL);

	return 1;
}

int atox(char * num)
{
	if(!strncmp(num,"A",1) || !strncmp(num,"a",1))
		return 0x0a;

	if(!strncmp(num,"B",1) || !strncmp(num,"b",1))
		return 0x0b;

	if(!strncmp(num,"C",1) || !strncmp(num,"c",1))
		return 0x0c;

	if(!strncmp(num,"D",1) || !strncmp(num,"d",1))
		return 0x0d;

	if(!strncmp(num,"E",1) || !strncmp(num,"e",1))
		return 0x0e;

	if(!strncmp(num,"F",1) || !strncmp(num,"f",1))
		return 0x0f;

	if(!strncmp(num,"0",1))
		return 0x0;
	if(!strncmp(num,"1",1))
		return 0x01;
	if(!strncmp(num,"2",1))
		return 0x02;
	if(!strncmp(num,"3",1))
		return 0x03;
	if(!strncmp(num,"4",1))
		return 0x04;
	if(!strncmp(num,"5",1))
		return 0x05;
	if(!strncmp(num,"6",1))
		return 0x06;
	if(!strncmp(num,"7",1))
		return 0x07;
	if(!strncmp(num,"8",1))
		return 0x08;
	if(!strncmp(num,"9",1))
		return 0x09;

	return 0;
}

int wifi_get_mac_addr(unsigned char *buf) 
{ 
	int ret = 0;
	char buf_mac[6] = {0x00,};

	ret = mac_address_change();

//	ret = 1;
//	strcpy(mac_address, "0011223344ff");
	
	if(ret)
	{
	//	00904cc51238	
		buf_mac[0]=(atox(&mac_address[0]) * 0x10) + atox(&mac_address[1]);
		buf_mac[1]=(atox(&mac_address[2]) * 0x10) + atox(&mac_address[3]);
		buf_mac[2]=(atox(&mac_address[4]) * 0x10) + atox(&mac_address[5]);
		buf_mac[3]=(atox(&mac_address[6]) * 0x10) + atox(&mac_address[7]);
		buf_mac[4]=(atox(&mac_address[8]) * 0x10) + atox(&mac_address[9]);
		buf_mac[5]=(atox(&mac_address[10]) * 0x10) + atox(&mac_address[11]);

		memcpy(buf,buf_mac,6);

		printk("customer mac address %02x:%02x:%02x:%02x:%02x:%02x\n",buf_mac[0],buf_mac[1],buf_mac[2],buf_mac[3],buf_mac[4],buf_mac[5]);
		
		return 0;
	}
	else
		return -1; 

}
//==========================================================


/* Function to get custom MAC address */
int
dhd_custom_get_mac_address(void *adapter, unsigned char *buf)
{
	int ret = 0;

	WL_TRACE(("%s Enter\n", __FUNCTION__));
	if (!buf)
		return -EINVAL;

	/* Customer access to MAC address stored outside of DHD driver */
#if defined(CUSTOMER_HW2) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35))
	ret = wifi_platform_get_mac_addr(adapter, buf);
#endif

#ifdef EXAMPLE_GET_MAC
	/* EXAMPLE code */
	{
		struct ether_addr ea_example = {{0x00, 0x11, 0x22, 0x33, 0x44, 0xFF}};
		bcopy((char *)&ea_example, buf, sizeof(struct ether_addr));
	}
#endif /* EXAMPLE_GET_MAC */

	return ret;
}
#endif /* GET_CUSTOM_MAC_ENABLE */

#if !defined(WL_WIRELESS_EXT)
struct cntry_locales_custom {
	char iso_abbrev[WLC_CNTRY_BUF_SZ];	/* ISO 3166-1 country abbreviation */
	char custom_locale[WLC_CNTRY_BUF_SZ];	/* Custom firmware locale */
	int32 custom_locale_rev;		/* Custom local revisin default -1 */
};
#endif /* WL_WIRELESS_EXT */

/* Customized Locale table : OPTIONAL feature */
const struct cntry_locales_custom translate_custom_table[] = {
/* Table should be filled out based on custom platform regulatory requirement */
#ifdef EXAMPLE_TABLE
	{"",   "XY", 4},  /* Universal if Country code is unknown or empty */
	{"US", "US", 69}, /* input ISO "US" to : US regrev 69 */
	{"CA", "US", 69}, /* input ISO "CA" to : US regrev 69 */
	{"EU", "EU", 5},  /* European union countries to : EU regrev 05 */
	{"AT", "EU", 5},
	{"BE", "EU", 5},
	{"BG", "EU", 5},
	{"CY", "EU", 5},
	{"CZ", "EU", 5},
	{"DK", "EU", 5},
	{"EE", "EU", 5},
	{"FI", "EU", 5},
	{"FR", "EU", 5},
	{"DE", "EU", 5},
	{"GR", "EU", 5},
	{"HU", "EU", 5},
	{"IE", "EU", 5},
	{"IT", "EU", 5},
	{"LV", "EU", 5},
	{"LI", "EU", 5},
	{"LT", "EU", 5},
	{"LU", "EU", 5},
	{"MT", "EU", 5},
	{"NL", "EU", 5},
	{"PL", "EU", 5},
	{"PT", "EU", 5},
	{"RO", "EU", 5},
	{"SK", "EU", 5},
	{"SI", "EU", 5},
	{"ES", "EU", 5},
	{"SE", "EU", 5},
	{"GB", "EU", 5},
	{"KR", "XY", 3},
	{"AU", "XY", 3},
	{"CN", "XY", 3}, /* input ISO "CN" to : XY regrev 03 */
	{"TW", "XY", 3},
	{"AR", "XY", 3},
	{"MX", "XY", 3},
	{"IL", "IL", 0},
	{"CH", "CH", 0},
	{"TR", "TR", 0},
	{"NO", "NO", 0},
#endif /* EXMAPLE_TABLE */
#if defined(CUSTOMER_HW2)
#if defined(BCM4335_CHIP)
	{"",   "XZ", 11},  /* Universal if Country code is unknown or empty */
#endif
	{"AE", "AE", 1},
	{"AR", "AR", 1},
	{"AT", "AT", 1},
	{"AU", "AU", 2},
	{"BE", "BE", 1},
	{"BG", "BG", 1},
	{"BN", "BN", 1},
	{"CA", "CA", 2},
	{"CH", "CH", 1},
	{"CY", "CY", 1},
	{"CZ", "CZ", 1},
	{"DE", "DE", 3},
	{"DK", "DK", 1},
	{"EE", "EE", 1},
	{"ES", "ES", 1},
	{"FI", "FI", 1},
	{"FR", "FR", 1},
	{"GB", "GB", 1},
	{"GR", "GR", 1},
	{"HR", "HR", 1},
	{"HU", "HU", 1},
	{"IE", "IE", 1},
	{"IS", "IS", 1},
	{"IT", "IT", 1},
	{"ID", "ID", 1},
	{"JP", "JP", 8},
	{"KR", "KR", 24},
	{"KW", "KW", 1},
	{"LI", "LI", 1},
	{"LT", "LT", 1},
	{"LU", "LU", 1},
	{"LV", "LV", 1},
	{"MA", "MA", 1},
	{"MT", "MT", 1},
	{"MX", "MX", 1},
	{"NL", "NL", 1},
	{"NO", "NO", 1},
	{"PL", "PL", 1},
	{"PT", "PT", 1},
	{"PY", "PY", 1},
	{"RO", "RO", 1},
	{"SE", "SE", 1},
	{"SI", "SI", 1},
	{"SK", "SK", 1},
	{"TR", "TR", 7},
	{"TW", "TW", 1},
	{"IR", "XZ", 11},	/* Universal if Country code is IRAN, (ISLAMIC REPUBLIC OF) */
	{"SD", "XZ", 11},	/* Universal if Country code is SUDAN */
	{"SY", "XZ", 11},	/* Universal if Country code is SYRIAN ARAB REPUBLIC */
	{"GL", "XZ", 11},	/* Universal if Country code is GREENLAND */
	{"PS", "XZ", 11},	/* Universal if Country code is PALESTINIAN TERRITORY, OCCUPIED */
	{"TL", "XZ", 11},	/* Universal if Country code is TIMOR-LESTE (EAST TIMOR) */
	{"MH", "XZ", 11},	/* Universal if Country code is MARSHALL ISLANDS */
#ifdef BCM4330_CHIP
	{"RU", "RU", 1},
	{"US", "US", 5}
#endif
#endif 
};


/* Customized Locale convertor
*  input : ISO 3166-1 country abbreviation
*  output: customized cspec
*/
#ifdef CUSTOM_COUNTRY_CODE
void get_customized_country_code(void *adapter, char *country_iso_code,
  wl_country_t *cspec, u32 flags)
#else
void get_customized_country_code(void *adapter, char *country_iso_code, wl_country_t *cspec)
#endif /* CUSTOM_COUNTRY_CODE */
{
#if defined(CUSTOMER_HW2) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 39))

	struct cntry_locales_custom *cloc_ptr;

	if (!cspec)
		return;
#ifdef CUSTOM_COUNTRY_CODE
	cloc_ptr = wifi_platform_get_country_code(adapter, country_iso_code,
	           flags);
#else
	cloc_ptr = wifi_platform_get_country_code(adapter, country_iso_code);
#endif /* CUSTOM_COUNTRY_CODE */
	if (cloc_ptr) {
		strlcpy(cspec->ccode, cloc_ptr->custom_locale, WLC_CNTRY_BUF_SZ);
		cspec->rev = cloc_ptr->custom_locale_rev;
	}
	return;
#else
	int size, i;

	size = ARRAYSIZE(translate_custom_table);

	if (cspec == 0)
		 return;

	if (size == 0)
		 return;

	for (i = 0; i < size; i++) {
		if (strcmp(country_iso_code, translate_custom_table[i].iso_abbrev) == 0) {
			memcpy(cspec->ccode,
				translate_custom_table[i].custom_locale, WLC_CNTRY_BUF_SZ);
			cspec->rev = translate_custom_table[i].custom_locale_rev;
			return;
		}
	}
#ifdef EXAMPLE_TABLE
	/* if no country code matched return first universal code from translate_custom_table */
	memcpy(cspec->ccode, translate_custom_table[0].custom_locale, WLC_CNTRY_BUF_SZ);
	cspec->rev = translate_custom_table[0].custom_locale_rev;
#endif /* EXMAPLE_TABLE */
	return;
#endif /* defined(CUSTOMER_HW2) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36)) */
}
