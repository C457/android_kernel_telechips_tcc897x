/*
 ** When you modify this source code, 
 ** please notify this change to the Framework Group Members
 **                                & Engineering Mode Developers.
 */

#ifndef __DAUDIO_BOARD_VER__
#define __DAUDIO_BOARD_VER__

typedef enum gps_ver
{
    KET_GPS,
    TRIMBLE,
}gps_ver_t;

typedef enum lcd_ver
{
    LCD_7,
    LCD_8_LG,
    LCD_8_AUO,
}lcd_ver_t;

typedef struct GPS_LCD_Version
{
    int gps_ver;
    int lcd_ver;
} GPS_LCD_Version_t;

static char *daudio_lcd_versions_oe_int_label[11] =
{
	"DAUDIOKK_LCD_OI_10_25_1920_720_INCELL_Si",
	"DAUDIOKK_LCD_OI_10_25_1920_720_OGS_TEMP",
	"DAUDIOKK_LCD_OI_RESERVED1",
	"DAUDIOKK_LCD_OI_RESERVED2",
	"DAUDIOKK_LCD_OI_RESERVED3",
	"DAUDIOKK_LCD_OI_RESERVED4",
	"DAUDIOKK_LCD_OI_RESERVED5",
	"DAUDIOKK_LCD_OI_10_25_1920_720_INCELL_LTPS",
	"DAUDIOKK_LCD_OI_08_00_1280_720_OGS_Si",
	"DAUDIOKK_LCD_OI_RESERVED6",
	"DAUDIOKK_LCD_OI_DISCONNECTED",
};

static char *daudio_lcd_versions_pio_int_label[11] =
{
	"DAUDIOKK_LCD_PI_RESERVED1",
	"DAUDIOKK_LCD_PI_RESERVED2",
	"DAUDIOKK_LCD_PI_RESERVED3",
	"DAUDIOKK_LCD_PI_RESERVED4",
	"DAUDIOKK_LCD_PI_RESERVED5",
	"DAUDIOKK_LCD_PI_10_25_1920_720_PIO_AUO",
	"DAUDIOKK_LCD_PI_08_00_800_400_PIO_TRULY",
	"DAUDIOKK_LCD_PI_RESERVED6",
	"DAUDIOKK_LCD_PI_RESERVED7",
	"DAUDIOKK_LCD_PI_RESERVED8",
	"DAUDIOKK_LCD_PI_DISCONNECTED",
};


static char *daudio_lcd_versions_oe_de_label[11] =
{
	"DAUDIOKK_LCD_OD_RESERVED1",
	"DAUDIOKK_LCD_OD_RESERVED2",
	"DAUDIOKK_LCD_OD_RESERVED3",
	"DAUDIOKK_LCD_OD_10_25_1920_720_INCELL_Si",
	"DAUDIOKK_LCD_OD_12_30_1920_720_INCELL_Si",
	"DAUDIOKK_LCD_OD_10_25_1920_720_INCELL_LTPS",
	"DAUDIOKK_LCD_OD_08_00_1280_720_OGS_Si",
	"DAUDIOKK_LCD_OD_RESERVED4",
	"DAUDIOKK_LCD_OD_RESERVED5",
	"DAUDIOKK_LCD_OD_RESERVED6",
	"DAUDIOKK_LCD_OD_DISCONNECTED",
};

static char *daudio_lcd_versions_pio_de_label[11] =
{
	"DAUDIOKK_LCD_PD_RESERVED1",
	"DAUDIOKK_LCD_PD_RESERVED2",
	"DAUDIOKK_LCD_PD_RESERVED3",
	"DAUDIOKK_LCD_PD_RESERVED4",
	"DAUDIOKK_LCD_PD_RESERVED5",
	"DAUDIOKK_LCD_PD_RESERVED6",
	"DAUDIOKK_LCD_PD_RESERVED7",
	"DAUDIOKK_LCD_PD_RESERVED8",
	"DAUDIOKK_LCD_PD_RESERVED9",
	"DAUDIOKK_LCD_PD_RESERVED10",
	"DAUDIOKK_LCD_PD_DISCONNECTED",
};

int daudio_hw_version(void);
int daudio_main_version(void);
//int daudio_bt_version(void);
//int daudio_gps_version(void);
int daudio_lcd_version(void);
//int get_gps_rtc_det(void);
unsigned long get_ant_diag_adc(unsigned int ch);
void get_daudio_rev(void);
int get_dmb_diag_short(void);
int get_dmb_diag_open(void);
int get_gps_diag_short(void);
int get_gps_diag_open(void);
int get_serdes_conn(void); //serdes conn check 20180622 mhjung
extern int set_serdes_conn_check(int value); //serdes conn check 20180622 mhjung

#endif
