#include <linux/kernel.h>
#include <linux/device.h>
#include <mach/daudio.h>
#include <mach/daudio_info.h>
#if defined(CONFIG_ARCH_TCC897X)
#include <mach/tcc_adc.h>
#include <mach/gpio.h>
#endif

#define ADC_READ_COUNT      5
#define ADC_READ_RETRY      10
#if defined(CONFIG_ARCH_TCC897X)
#define VIN                 1800            //Input Voltage : 1.8V
#define DIV                 (tcc_adc_is_12bit_res() ? 0xFFF: 0x3FF)
#else
#define VIN                 3300            //Input Voltage : 3.3V
#define DIV                 0x0FFF          //ADC Maximum Data
#endif

#define ADC_PORT_NUM        4
#define ADCCONV(X)          (X + 1) * VIN / DIV

typedef enum daudio_ver_data
{
    DAUDIO_VER_LCD,
    DAUDIO_VER_MAIN,
    DAUDIO_VER_HW,
    DAUDIO_VER_BT,
}daudio_ver_data_t;

typedef struct ADC_Range
{
    unsigned int ver;
    unsigned int min;
    unsigned int max;
}ADC_Range_t;

static unsigned long adcs[ADC_PORT_NUM];
static unsigned long vers[ADC_PORT_NUM];

extern D_Audio_bsp_version bsp_version;
extern D_Audio_bsp_adc bsp_adc;
#if !defined(CONFIG_ARCH_TCC897X)
extern unsigned int tca_adc_read(unsigned int channel);
#endif
extern int get_lcd_version(int ver);
extern int get_gps_version(int ver);

static void init_daudio_rev(void);
static int init_version(int what, unsigned long adc);
static int adc_valid_check(unsigned long adc, int what, int ver);
static int sizeof_adc_range_t(int what);
static ADC_Range_t* get_range(int what);
static unsigned char parse_ver(unsigned long adc, int what);
int SerDes_Conn_Try = 0; //serdes conn check 20180622 mhjung

static ADC_Range_t hw_ranges[] =
{
        {DAUDIOKK_HW_1ST, 0, 199},              //HW Ver.01 - 0.15V
        {DAUDIOKK_HW_2ND, 200, 399},            //HW Ver.02 - 0.32V
        {DAUDIOKK_HW_3RD, 400, 599},            //HW Ver.03 - 0.51V
        {DAUDIOKK_HW_4TH, 600, 799},            //HW Ver.04 - 0.73V
        {DAUDIOKK_HW_5TH, 800, 999},            //HW Ver.05 - 0.9V
        {DAUDIOKK_HW_6TH, 1000, 1199},          //HW Ver.06 - 1.11V
        {DAUDIOKK_HW_7TH, 1200, 1399},          //HW Ver.07 - 1.31V
        {DAUDIOKK_HW_8TH, 1400, 1599},          //HW Ver.08 - 1.48V
        {DAUDIOKK_HW_9TH, 1600, 1800},          //HW Ver.09 - 1.69V
};


static ADC_Range_t main_ranges[] =
{
	{DAUDIOKK_PLATFORM_WS4, 0, 199},           //Platform Ver.01 - 0.15V
	{DAUDIOKK_PLATFORM_WS5, 200, 399},         //Platform Ver.02 - 0.32V
	{DAUDIOKK_PLATFORM_WS3, 400, 599},        //Platform Ver.03 - 0.51V
	{DAUDIOKK_PLATFORM_WS6, 600, 799},        //Platform Ver.04 - 0.73V
	{DAUDIOKK_PLATFORM_WS7, 800, 999},       //Platform Ver.05 - 0.9V
	{DAUDIOKK_PLATFORM_WS8, 1000, 1199},     //Platform Ver.06 - 1.11V
	{DAUDIOKK_PLATFORM_WS9, 1200, 1399},     //Platform Ver.07 - 1.31V
	{DAUDIOKK_PLATFORM_WS10, 1400, 1599},     //Platform Ver.08 - 1.48V
	{DAUDIOKK_PLATFORM_WS11, 1600, 1800},     //Platform Ver.09 - 1.69V
};
static ADC_Range_t bt_ranges[] =
{
       {DAUDIOKK_BT_VER_1,0, 199},
       {DAUDIOKK_BT_VER_2,200, 399},
       {DAUDIOKK_BT_VER_3,400, 599},
       {DAUDIOKK_BT_VER_4,600, 799},
       {DAUDIOKK_BT_VER_5,800, 999},
       {DAUDIOKK_BT_VER_6,1000, 1199},
       {DAUDIOKK_BT_VER_7,1200, 1399},
       {DAUDIOKK_BT_VER_8,1400, 1599},
       {DAUDIOKK_BT_VER_9,1600, 1800},
};

static ADC_Range_t lcd_ranges[11];
static ADC_Range_t lcd_versions_oe_int[11] =
{
	{DAUDIOKK_LCD_OI_10_25_1920_720_INCELL_Si, 0, 0},
	{DAUDIOKK_LCD_OI_10_25_1920_720_OGS_TEMP, 100, 270},
	{DAUDIOKK_LCD_OI_RESERVED1, 280, 470},
	{DAUDIOKK_LCD_OI_RESERVED2, 480, 630},
	{DAUDIOKK_LCD_OI_RESERVED3, 640, 810},
	{DAUDIOKK_LCD_OI_RESERVED4, 820, 980},
	{DAUDIOKK_LCD_OI_RESERVED5, 990, 1140},
	{DAUDIOKK_LCD_OI_10_25_1920_720_INCELL_LTPS, 1150, 1340},
	{DAUDIOKK_LCD_OI_08_00_1280_720_OGS_Si, 1350, 1520},
	{DAUDIOKK_LCD_OI_RESERVED6, 1530, 1710},
	{DAUDIOKK_LCD_OI_DISCONNECTED, 1710, 1890},
};

static ADC_Range_t lcd_versions_pio_int[11] =
{
	{DAUDIOKK_LCD_PI_RESERVED1, 0, 0},
	{DAUDIOKK_LCD_PI_RESERVED2, 100, 270},
	{DAUDIOKK_LCD_PI_RESERVED3, 280, 470},
	{DAUDIOKK_LCD_PI_RESERVED4, 480, 630},
	{DAUDIOKK_LCD_PI_RESERVED5, 640, 810},
	{DAUDIOKK_LCD_PI_10_25_1920_720_PIO_AUO,820, 980},
	{DAUDIOKK_LCD_PI_08_00_800_400_PIO_TRULY, 990, 1140},
	{DAUDIOKK_LCD_PI_RESERVED6, 1150, 1340},
	{DAUDIOKK_LCD_PI_RESERVED7, 1350, 1520},
	{DAUDIOKK_LCD_PI_RESERVED8, 1530, 1710},
	{DAUDIOKK_LCD_PI_DISCONNECTED, 1710, 1890},
};


static ADC_Range_t lcd_versions_oe_de[11] =
{
	{DAUDIOKK_LCD_OD_RESERVED1, 0, 0},
	{DAUDIOKK_LCD_OD_RESERVED2, 100, 270},
	{DAUDIOKK_LCD_OD_RESERVED3, 280, 470},
	{DAUDIOKK_LCD_OD_10_25_1920_720_INCELL_Si, 480, 630},
	{DAUDIOKK_LCD_OD_12_30_1920_720_INCELL_Si, 640, 810},
	{DAUDIOKK_LCD_OD_10_25_1920_720_INCELL_LTPS, 820, 980},
	{DAUDIOKK_LCD_OD_08_00_1280_720_OGS_Si, 990, 1140},
	{DAUDIOKK_LCD_OD_RESERVED4, 1150, 1340},
	{DAUDIOKK_LCD_OD_RESERVED5, 1350, 1520},
	{DAUDIOKK_LCD_OD_RESERVED6, 1530, 1710},
	{DAUDIOKK_LCD_OD_DISCONNECTED, 1710, 1890},
};

static ADC_Range_t lcd_versions_pio_de[11] =
{
	{DAUDIOKK_LCD_PD_RESERVED1, 0, 0},
	{DAUDIOKK_LCD_PD_RESERVED2, 100, 270},
	{DAUDIOKK_LCD_PD_RESERVED3, 280, 470},
	{DAUDIOKK_LCD_PD_RESERVED4, 480, 630},
	{DAUDIOKK_LCD_PD_RESERVED5, 640, 810},
	{DAUDIOKK_LCD_PD_RESERVED6, 820, 980},
	{DAUDIOKK_LCD_PD_RESERVED7, 990, 1140},
	{DAUDIOKK_LCD_PD_RESERVED8, 1150, 1340},
	{DAUDIOKK_LCD_PD_RESERVED9, 1350, 1520},
	{DAUDIOKK_LCD_PD_RESERVED10, 1530, 1710},
	{DAUDIOKK_LCD_PD_DISCONNECTED, 1710, 1890},
};

static int lcd_version_match(int oem, int mon)
{
	if((oem == 1) && (mon == 1))
		memcpy(lcd_ranges, lcd_versions_oe_int, sizeof(lcd_versions_oe_int));
	else if((oem == 0) && (mon == 1))
		memcpy(lcd_ranges, lcd_versions_pio_int, sizeof(lcd_versions_pio_int));	
	else if ((oem == 1) && (mon == 0))
		memcpy(lcd_ranges, lcd_versions_oe_de, sizeof(lcd_versions_oe_de));	
	else if ((oem == 0) && (mon == 0))
		memcpy(lcd_ranges, lcd_versions_pio_de, sizeof(lcd_versions_pio_de));
	else
		return -1;
	return 0;
}


#if defined(CONFIG_ARCH_TCC897X)

struct daudio_adc
{
	int ch;
	struct tcc_adc_client *client;
};

static struct daudio_adc daudio_info_adc[] = {
		{ ADC_CH2, NULL },
		{ ADC_CH3, NULL },
		{ ADC_CH4, NULL },
		{ ADC_CH5, NULL },
		{ ADC_CH6, NULL },
		{ ADC_CH7, NULL },
		{ ADC_CH8, NULL },
		{ ADC_CH9, NULL },
		{ 0xFF, NULL }
};

#endif // CONFIG_DAUDIO_KK

int daudio_info_adc_init(struct device *dev)
{
#if defined(CONFIG_ARCH_TCC897X)
	struct daudio_adc *da = daudio_info_adc;
	int adc;

	while (da->ch != 0xFF) {
		da->client = tcc_adc_register(dev, da->ch);
		if (!da->client) {
			dev_err(dev, "%s: tcc_adc_register on ch%d failed\n", __func__, da->ch);
		}
		adc = tcc_adc_getdata(da->client);
		//dev_err(dev, "%s: ADC %d=%d %d\n", __func__, da->ch, adc, ADCCONV(adc));
		da++;
	}
#endif

	return 0;
}

static unsigned long daudio_info_adc_read(int ch)
{
	unsigned long adc = 0;
#if defined(CONFIG_ARCH_TCC897X)
	struct daudio_adc *da = daudio_info_adc;

	while (da->ch != 0xFF) {
		if (da->ch == ch) {
			BUG_ON(!da->client);
			adc = tcc_adc_getdata(da->client);
			break;
		}
		da++;
	}
#else
	adc = tcc_adc_read(ch);
#endif

	return adc;
}

void get_daudio_rev(void)
{
    int oem, mon;
    init_daudio_rev();
    oem = gpio_get_value(TCC_GPB(24));
    mon = gpio_get_value(TCC_GPB(13));

    bsp_version.hw_version = vers[DAUDIO_VER_HW];
    bsp_version.main_version = vers[DAUDIO_VER_MAIN];
    bsp_version.bt_version = vers[DAUDIO_VER_BT];
#if defined(CONFIG_DAUDIO_ECO) || defined(CONFIG_DAUDIO_KK)
//    bsp_version.gps_version = 0;
      bsp_version.lcd_version = vers[DAUDIO_VER_LCD];
#else
    bsp_version.gps_version = get_gps_version(vers[DAUDIO_VER_GPS_LCD]);
    bsp_version.lcd_version = get_lcd_version(vers[DAUDIO_VER_GPS_LCD]);
#endif
    bsp_adc.hw_adc = adcs[DAUDIO_VER_HW];
    bsp_adc.main_adc = adcs[DAUDIO_VER_MAIN];
    bsp_adc.bt_adc = adcs[DAUDIO_VER_BT];
      bsp_adc.lcd_adc = adcs[DAUDIO_VER_LCD];

    printk(KERN_INFO "========== %s ==========\n", __func__);
     printk(KERN_INFO "[Version] HW: %d, Main : %d, LCD[OEM(%d), MON(%d)] : %d, BT : %d\n", bsp_version.hw_version, bsp_version.main_version,oem, mon, bsp_version.lcd_version, bsp_version.bt_version);  // GPS Version : 0 at Daudio_Eco, --> GPS is not used at Daudio_Eco
    printk(KERN_INFO "[ADC] HW: %d, Main : %d, LCD : %d, BT : %d\n", bsp_adc.hw_adc, bsp_adc.main_adc, bsp_adc.lcd_adc, bsp_adc.bt_adc);
#if defined(CONFIG_DAUDIO_ECO) || defined(CONFIG_DAUDIO_KK)
	printk("ADC_AIN[02]: %ld, ADC_AIN[06]: %ld, ADC_AIN[08]: %ld\n", get_ant_diag_adc(2),get_ant_diag_adc(6), get_ant_diag_adc(8));
#else
	printk("ADC_AIN[06]: %ld\n", get_ant_diag_adc(6));
#endif
    printk(KERN_INFO "======================================\n");
}
/*
int get_gps_rtc_det() // GPS - 0 : RTC - 1
{
       unsigned long adc = get_ant_diag_adc(6); //ADC_CHANNEL6
       int retval = -1;

       if((adc >= 440) && (adc < 800)) retval = 1;
       else retval = 0;

       return retval;
}
*/
#if defined(CONFIG_DAUDIO_ECO) || defined(CONFIG_DAUDIO_KK)
int get_dmb_diag_short() //normal - 1, short - 0
{
	unsigned long adc = get_ant_diag_adc(8); //ADC_CHANNEL8
	int retval = -1;

	if(adc > 1100) retval = 0;
	else retval = 1;

	return retval;
}

int get_dmb_diag_open() //normal - 1, open - 0
{
	unsigned long adc = get_ant_diag_adc(8); //ADC_CHANNEL8
	int retval = -1;

	if(adc < 110) retval = 0;
	else retval = 1;

	return retval;
}

int get_gps_diag_short() //normal - 1, short - 0
{
	unsigned long adc = get_ant_diag_adc(2); //ADC_CHANNEL2
	int retval = -1;

	if(adc > 1100) retval = 0;
	else retval = 1;

	return retval;
}

int get_gps_diag_open() //normal - 1, open - 0
{
	unsigned long adc = get_ant_diag_adc(2); //ADC_CHANNEL2
	int retval = -1;

	if(adc < 110) retval = 0;
	else retval = 1;

	return retval;
}

int set_serdes_conn_check(int vlaue) //serdes conn check 20180622 mhjung
{
	SerDes_Conn_Try = vlaue;
}

EXPORT_SYMBOL(set_serdes_conn_check);

int get_serdes_conn() //serdes conn check 20180622 mhjung
{
	int count = SerDes_Conn_Try;
	int retval = -1;

	if(count == 2 ) retval = 0;
	else retval = 1;

	return retval;
}
#endif

unsigned long get_ant_diag_adc(unsigned int ch)
{
	int i = 0, sum = 0;
	unsigned long adc, min, max;
	unsigned long voltage[ADC_READ_COUNT];

	sum = min = max = voltage[0] = ADCCONV(daudio_info_adc_read(ch));

	for(i = 1 ; i < ADC_READ_COUNT ; i++)
	{
		adc = daudio_info_adc_read(ch);
		voltage[i] = ADCCONV(adc);

		if(voltage[i] < min)
			min = voltage[i];
		else if(voltage[i] > max)
			max = voltage[i];
		sum += voltage[i];
	}

	return (sum - min - max)/(ADC_READ_COUNT-2);
}


static void init_daudio_rev(void)
{
    int i;
    int valid;
    unsigned long daudio_board_revs[ADC_PORT_NUM] = {0, };
    unsigned long adc = 0;
    int init_ver;

    for(i = 0 ; i < ADC_PORT_NUM ; i++)
    {
        int retry = ADC_READ_RETRY;
        valid = 0;

        while(retry-- > 0)
        {
            int count = ADC_READ_COUNT;
	    if(i == DAUDIO_VER_BT)
		    init_ver = init_version(i, daudio_info_adc_read(i+4));
	    else
	            init_ver = init_version(i, daudio_info_adc_read(3*(i+1)));

            if(init_ver < 0) continue;

            while(count-- > 0)
            {
		if( i == DAUDIO_VER_BT)
			adc = daudio_info_adc_read(i+4);
		else
	                adc = daudio_info_adc_read(3*(i+1));
                valid = adc_valid_check(adc, i, init_ver);
                if(!valid)  break;
            }

            if(valid)
                break;
        }
        daudio_board_revs[i] = valid ? adc : -1;
    }

    for(i = 0 ; i < ADC_PORT_NUM ; i++)
    {
        adc = ADCCONV(daudio_board_revs[i]);
        adcs[i] = (daudio_board_revs[i] >= 0) ? adc : -1;
        vers[i] = parse_ver(adc, i);
    }
}

static int init_version(int what, unsigned long adc)
{
    int i ;
    int ret = -1;
    ADC_Range_t* adc_range = get_range(what);
    int size = sizeof_adc_range_t(what); 
    if(adc_range)
    {
        adc = ADCCONV(adc);

        for(i = 0 ; i < size ; i++)
        {
	
            if(adc_range[i].min <= adc && adc <= adc_range[i].max)
            { 
                ret = i;
                break;
            }
        }
    }

    return ret;
}

static int adc_valid_check(unsigned long adc, int what, int ver)
{
    int ret = 0;
    ADC_Range_t* adc_range = get_range(what);
    unsigned long max, min, margin;

    if(adc_range)
    {
        adc = ADCCONV(adc);

        max = adc_range[ver].max;
        min = adc_range[ver].min;
        margin = (max - min)/10;

        if((min + margin) <= adc && adc <= (max - margin))
            ret = 1;
        else
            ret = 0;
    }

    return ret;
}

static unsigned char parse_ver(unsigned long adc, int what)
{
    int i ;
    ADC_Range_t* temp_range = get_range(what);
    int size = sizeof_adc_range_t(what);
#if defined(CONFIG_DAUDIO_ECO) || defined(CONFIG_DAUDIO_KK)
    unsigned char default_ver[ADC_PORT_NUM] = {DAUDIOKK_LCD_OI_10_25_1920_720_INCELL_Si,DAUDIOKK_PLATFORM_WS4,DAUDIOKK_HW_1ST, DAUDIOKK_BT_VER_1};
#endif
    unsigned char ret = default_ver[what];
    unsigned long min, max;

    if(temp_range)
    {
        for(i = 0 ; i < size ; i++)
        {
            min = temp_range[i].min;
            max = temp_range[i].max;

            if(adc >= min && adc <= max)
            {
                ret = temp_range[i].ver;
                break;
            }
        }
	#if defined(CONFIG_DAUDIO_ECO)
	if ((ret < 0) || (ret >(size-1))) default_ver[what];
	#endif
    }

    return ret;
}

static int sizeof_adc_range_t(int what)
{
    int ret = 0;

    switch(what)
    {
        case DAUDIO_VER_HW:
            ret = sizeof(hw_ranges)/sizeof(hw_ranges[0]);
            break;
        case DAUDIO_VER_MAIN:
            ret = sizeof(main_ranges)/sizeof(hw_ranges[0]);
            break;
        case DAUDIO_VER_BT:
            ret = sizeof(bt_ranges)/sizeof(hw_ranges[0]);
            break;
        case DAUDIO_VER_LCD:
            ret = sizeof(lcd_ranges)/sizeof(lcd_ranges[0]);
            break;
        default:
            ret = 0;
            break;
    }

    return ret;
}

static ADC_Range_t* get_range(int what)
{
    ADC_Range_t* ret = 0;
    int oem, mon;
    int retval;

    switch(what)
    {
        case DAUDIO_VER_HW:
            ret = hw_ranges;
            break;
        case DAUDIO_VER_MAIN:
            ret = main_ranges;
            break;
        case DAUDIO_VER_BT:
            ret = bt_ranges;
            break;             
        case DAUDIO_VER_LCD:
	{
	   oem = gpio_get_value(TCC_GPB(24));
	   mon = gpio_get_value(TCC_GPB(13));
	   retval = lcd_version_match(oem, mon);
	   if(retval)
		   printk("%s, The table is out of boundary[HW Probe]\n",__func__);
            ret = lcd_ranges;
            break;
	}
        default:
            ret = 0;
            break;
    }
    return ret;
}

