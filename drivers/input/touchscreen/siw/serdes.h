#ifndef SERDES_INFO
#define SERDES_INFO

#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/gpio.h>

#ifdef CONFIG_WIDE_PE_COMMON
#include <mobis/daudio.h>
#include <mobis/daudio_info.h>
#include <mobis/daudio_pinctl.h>
#endif

#define SER_ADDRESS 0x40
#define DES_ADDRESS 0x48
#define DES_DELAY 0x00

typedef struct
{
	u8 chip_addr;
	u16 reg_addr;
	u8 value;
	u8 right_value;
}serdes_config_5;

static u16 serdes_link_lock_config [1][2] = {
        {SER_ADDRESS, 0x0013},
};
static u16 des_set_check [2][3] = {
        {DES_ADDRESS, 0x0458, 0x28},
        {DES_ADDRESS, 0x0206, 0x85},
};
static u16 ser_bitrate_check [1][2] = {
        {SER_ADDRESS, 0x0001},
};
static u16 des_bitrate_check [1][2] = {
        {DES_ADDRESS, 0x0001},
};

static u16 ser_bitrate_reset [1][3] = {
        {SER_ADDRESS, 0x0010, 0x31},
};
static u16 des_bitrate_reset [1][3] = {
        {DES_ADDRESS, 0x0010, 0x31},
};

static serdes_config_5 serdes_1920_720_12_3_config_Si_LG[]={
        {SER_ADDRESS, 0x0001, 0x88, 0x88},

	{SER_ADDRESS, 0x0004, 0x0A, 0x0A},

	{DES_ADDRESS, 0x0002, 0x03, 0x03},

        {DES_ADDRESS, 0x0458, 0x28, 0x28},
        {DES_ADDRESS, 0x0558, 0x28, 0x28},
        {DES_ADDRESS, 0x0459, 0x68, 0x68},
        {DES_ADDRESS, 0x0559, 0x68, 0x68},

        {DES_ADDRESS, 0x01CE, 0x47, 0x47},
        {DES_ADDRESS, 0x01D2, 0x24, 0x24},
        {DES_ADDRESS, 0x01D4, 0x42, 0x42},
        {DES_ADDRESS, 0x01CF, 0x01, 0x01},

        {SER_ADDRESS, 0x0140, 0x20, 0x20},
        {SER_ADDRESS, 0x0002, 0x03, 0x03},
        {DES_ADDRESS, 0x0140, 0x20, 0x20},
        {DES_DELAY, 0x0000, 0x01, 0x01},
        {DES_ADDRESS, 0x0002, 0x43, 0x43},
#ifdef CONFIG_WIDE_PE_COMMON
	{DES_ADDRESS, 0x0221, 0x80, 0x80},
#endif
        {DES_ADDRESS, 0x0218, 0x01, 0x09},
        {SER_ADDRESS, 0x020C, 0x01, 0x09},
        {SER_ADDRESS, 0x020D, 0x04, 0x04},
        {SER_ADDRESS, 0x020E, 0x04, 0x44},
        {DES_ADDRESS, 0x0219, 0x04, 0x04},
        {DES_ADDRESS, 0x021A, 0x04, 0x44},

        {DES_ADDRESS, 0x020C, 0x01, 0x01},
        {SER_ADDRESS, 0x020F, 0x01, 0x01},
        {SER_ADDRESS, 0x0210, 0x05, 0x05},
        {SER_ADDRESS, 0x0211, 0x05, 0x05},
        {DES_ADDRESS, 0x020D, 0x05, 0x05},
        {DES_ADDRESS, 0x020E, 0x05, 0x45},

        {SER_ADDRESS, 0x021B, 0x01, 0x09},
        {DES_ADDRESS, 0x0224, 0x01, 0x01},
        {SER_ADDRESS, 0x021C, 0x09, 0x09},
        {SER_ADDRESS, 0x021D, 0x09, 0x49},
        {DES_ADDRESS, 0x0225, 0x09, 0x09},
        {DES_ADDRESS, 0x0226, 0x09, 0x49},

        {SER_ADDRESS, 0x0206, 0x01, 0x09},
        {DES_ADDRESS, 0x0212, 0x01, 0x01},
        {SER_ADDRESS, 0x0207, 0x02, 0x02},
        {SER_ADDRESS, 0x0208, 0x02, 0x42},
        {DES_ADDRESS, 0x0213, 0x02, 0x02},
        {DES_ADDRESS, 0x0214, 0x02, 0x42},

        {SER_ADDRESS, 0x0209, 0x01, 0x09},
        {DES_ADDRESS, 0x0221, 0x01, 0x01},
        {SER_ADDRESS, 0x020A, 0x03, 0x03},
        {SER_ADDRESS, 0x020B, 0x03, 0x43},
        {DES_ADDRESS, 0x0222, 0x03, 0x03},
        {DES_ADDRESS, 0x0223, 0x03, 0x43},

        {SER_ADDRESS, 0x0215, 0x01, 0x01},
        {DES_ADDRESS, 0x0215, 0x01, 0x01},
        {SER_ADDRESS, 0x0216, 0x07, 0x07},
        {SER_ADDRESS, 0x0217, 0x07, 0x47},
        {DES_ADDRESS, 0x0216, 0x07, 0x07},
        {DES_ADDRESS, 0x0217, 0x07, 0x07},

        {SER_ADDRESS, 0x0212, 0x01, 0x01},
        {DES_ADDRESS, 0x0206, 0x01, 0x01},
        {SER_ADDRESS, 0x0213, 0x06, 0x06},
        {SER_ADDRESS, 0x0214, 0x06, 0x46},
        {DES_ADDRESS, 0x0207, 0x06, 0x06},
        {DES_ADDRESS, 0x0208, 0x06, 0x06},

	{SER_ADDRESS, 0x020D, 0xA4, 0xa4},
        {SER_ADDRESS, 0x020C, 0x05, 0x0c},
        {DES_ADDRESS, 0x0218, 0x83, 0x8b},
	{SER_ADDRESS, 0x020C, 0x04, 0x0c},

	{SER_ADDRESS, 0x0210, 0xA5, 0xa5},
        {SER_ADDRESS, 0x020F, 0x05, 0x04},
        {DES_ADDRESS, 0x020C, 0x83, 0x83},
	{SER_ADDRESS, 0x020F, 0x04, 0x04},

	{DES_ADDRESS, 0x0225, 0xA9, 0xa9},
        {DES_ADDRESS, 0x0224, 0x85, 0x84},
        {SER_ADDRESS, 0x021B, 0x83, 0x8b},
	{DES_ADDRESS, 0x0224, 0x84, 0x84},

	{DES_ADDRESS, 0x0213, 0xA6, 0xa6},
	{DES_ADDRESS, 0x0212, 0x85, 0x84},
        {SER_ADDRESS, 0x0206, 0x83, 0x8b},
	{DES_ADDRESS, 0x0212, 0x84, 0x84},

	{DES_ADDRESS, 0x0222, 0xA3, 0xa3},
        {DES_ADDRESS, 0x0221, 0x85, 0x84},
        {SER_ADDRESS, 0x0209, 0x83, 0x8b},
	{DES_ADDRESS, 0x0221, 0x84, 0x84},

	{DES_ADDRESS, 0x0216, 0xA7, 0xa7},
        {DES_ADDRESS, 0x0215, 0x05, 0x04},
        {SER_ADDRESS, 0x0215, 0x83, 0x8b},
	{DES_ADDRESS, 0x0215, 0x04, 0x04},

        {DES_ADDRESS, 0x0206, 0x85, 0x85},
        {SER_ADDRESS, 0x0212, 0x83, 0x83},
	{DES_ADDRESS, 0x0207, 0xA2, 0xa2},

        {DES_ADDRESS, 0x0227, 0x80, 0x80},
#ifdef CONFIG_WIDE_PE_COMMON
	{DES_ADDRESS, 0x0221, 0x90, 0x90},
#endif

        {SER_ADDRESS, 0x0003, 0x10, 0x10},
};

static serdes_config_5 serdes_1920_720_10_25_config_LTPS_LG[] = {
        {SER_ADDRESS, 0x0001, 0x88, 0x88},

	{SER_ADDRESS, 0x0004, 0x0A, 0x0A},

	{DES_ADDRESS, 0x0002, 0x03, 0x03},

        {DES_ADDRESS, 0x0458, 0x28, 0x28},
        {DES_ADDRESS, 0x0558, 0x28, 0x28},
        {DES_ADDRESS, 0x0459, 0x68, 0x68},
        {DES_ADDRESS, 0x0559, 0x68, 0x68},

	{DES_ADDRESS, 0x01C8, 0x38, 0x38},
        {DES_ADDRESS, 0x01C9, 0x39, 0x39},
        {DES_ADDRESS, 0x01CA, 0x3A, 0x3A},

        {DES_ADDRESS, 0x01CE, 0x5E, 0x5E},
        {DES_DELAY, 0x0000, 0x02, 0x02},
        {DES_ADDRESS, 0x01D0, 0xBB, 0xBB},
        {DES_ADDRESS, 0x01D1, 0xCC, 0xCC},
        {DES_ADDRESS, 0x01D2, 0x99, 0x99},
        {DES_ADDRESS, 0x01D3, 0x88, 0x88},
        {DES_ADDRESS, 0x01D4, 0xAA, 0xAA},
        {DES_ADDRESS, 0x01CF, 0x01, 0x01},

        {SER_ADDRESS, 0x0140, 0x20, 0x20},
        {SER_ADDRESS, 0x0002, 0x03, 0x03},
        {DES_ADDRESS, 0x0140, 0x20, 0x20},
        {DES_DELAY, 0x0000, 0x01, 0x01},
#ifdef CONFIG_WIDE_PE_COMMON
	{DES_ADDRESS, 0x0221, 0x80, 0x80},
#endif

        {DES_ADDRESS, 0x0218, 0x01, 0x09},
        {SER_ADDRESS, 0x020C, 0x01, 0x09},
        {SER_ADDRESS, 0x020D, 0x04, 0x04},
        {SER_ADDRESS, 0x020E, 0x04, 0x44},
        {DES_ADDRESS, 0x0219, 0x04, 0x04},
        {DES_ADDRESS, 0x021A, 0x04, 0x44},

        {DES_ADDRESS, 0x020C, 0x01, 0x01},
        {SER_ADDRESS, 0x020F, 0x01, 0x09},
        {SER_ADDRESS, 0x0210, 0x05, 0x05},
        {SER_ADDRESS, 0x0211, 0x05, 0x05},
        {DES_ADDRESS, 0x020D, 0x05, 0x05},
        {DES_ADDRESS, 0x020E, 0x05, 0x45},

        {SER_ADDRESS, 0x021B, 0x01, 0x09},
        {DES_ADDRESS, 0x0224, 0x01, 0x01},
        {SER_ADDRESS, 0x021C, 0x09, 0x09},
        {SER_ADDRESS, 0x021D, 0x09, 0x49},
        {DES_ADDRESS, 0x0225, 0x09, 0x09},
        {DES_ADDRESS, 0x0226, 0x09, 0x49},

        {SER_ADDRESS, 0x0206, 0x01, 0x09},
        {DES_ADDRESS, 0x0212, 0x01, 0x01},
        {SER_ADDRESS, 0x0207, 0x02, 0x02},
        {SER_ADDRESS, 0x0208, 0x02, 0x42},
        {DES_ADDRESS, 0x0213, 0x02, 0x02},
        {DES_ADDRESS, 0x0214, 0x02, 0x42},

        {SER_ADDRESS, 0x0209, 0x01, 0x09},
        {DES_ADDRESS, 0x0221, 0x01, 0x01},
        {SER_ADDRESS, 0x020A, 0x03, 0x03},
        {SER_ADDRESS, 0x020B, 0x03, 0x43},
        {DES_ADDRESS, 0x0222, 0x03, 0x03},
        {DES_ADDRESS, 0x0223, 0x03, 0x43},

        {SER_ADDRESS, 0x0215, 0x01, 0x09},
        {DES_ADDRESS, 0x0215, 0x01, 0x01},
        {SER_ADDRESS, 0x0216, 0x07, 0x07},
        {SER_ADDRESS, 0x0217, 0x07, 0x47},
        {DES_ADDRESS, 0x0216, 0x07, 0x07},
        {DES_ADDRESS, 0x0217, 0x07, 0x47},

        {SER_ADDRESS, 0x0212, 0x01, 0x09},
        {DES_ADDRESS, 0x0206, 0x01, 0x01},
        {SER_ADDRESS, 0x0213, 0x06, 0x06},
        {SER_ADDRESS, 0x0214, 0x06, 0x46},
        {DES_ADDRESS, 0x0207, 0x06, 0x06},
        {DES_ADDRESS, 0x0208, 0x06, 0x46},
#ifdef CONFIG_WIDE_PE_COMMON
	{DES_DELAY, 0x0000, 0xFF, 0xFF},
	{DES_DELAY, 0x0000, 0x64, 0x64},
#endif
	{SER_ADDRESS, 0x020D, 0xA4, 0xa4},
        {SER_ADDRESS, 0x020C, 0x05, 0x0c},
        {DES_ADDRESS, 0x0218, 0x83, 0x8b},
	{SER_ADDRESS, 0x020C, 0x04, 0x0c},

	{SER_ADDRESS, 0x0210, 0xA5, 0xa5},
        {SER_ADDRESS, 0x020F, 0x05, 0x04},
        {DES_ADDRESS, 0x020C, 0x83, 0x83},
	{SER_ADDRESS, 0x020F, 0x04, 0x04},

	{DES_ADDRESS, 0x0225, 0xA9, 0xa9},
        {DES_ADDRESS, 0x0224, 0x85, 0x84},
        {SER_ADDRESS, 0x021B, 0x83, 0x8B},
	{DES_ADDRESS, 0x0224, 0x84, 0x84},

	{DES_ADDRESS, 0x0213, 0xA6, 0xa6},
        {DES_ADDRESS, 0x0212, 0x85, 0x84},
        {SER_ADDRESS, 0x0206, 0x83, 0x8b},
	{DES_ADDRESS, 0x0212, 0x84, 0x84},

	{DES_ADDRESS, 0x0222, 0xA3, 0xa3},
        {DES_ADDRESS, 0x0221, 0x85, 0x84},
        {SER_ADDRESS, 0x0209, 0x83, 0x8b},
	{DES_ADDRESS, 0x0221, 0x84, 0x84},

	{DES_ADDRESS, 0x0216, 0xA7, 0xa7},
        {DES_ADDRESS, 0x0215, 0x05, 0x04},
        {SER_ADDRESS, 0x0215, 0x83, 0x8b},

        {DES_ADDRESS, 0x0206, 0x85, 0x85},
	{SER_ADDRESS, 0x0212, 0x83, 0x8b},
	{DES_ADDRESS, 0x0207, 0xA2, 0xa2},

        {DES_ADDRESS, 0x0227, 0x80, 0x80},
#ifdef CONFIG_WIDE_PE_COMMON
        {DES_ADDRESS, 0x0221, 0x90, 0x90},
#endif

	{DES_ADDRESS, 0x0002, 0x43, 0x43},
	{DES_DELAY, 0x0000, 0x01, 0x01},
	{DES_ADDRESS, 0x0003, 0xFF, 0xFF},
	{DES_ADDRESS, 0x0002, 0x03, 0x03},
	{DES_ADDRESS, 0x0D00, 0xF4, 0xF4},
	{DES_ADDRESS, 0x0002, 0x43, 0x43},
	{DES_DELAY, 0x0000, 0x02, 0x02},
	{DES_ADDRESS, 0x0108, 0xFF, 0xFF},
	{DES_ADDRESS, 0x0D00, 0xF5, 0xF5},

	{DES_ADDRESS, 0x01C8, 0x18, 0x18},
	{DES_ADDRESS, 0x01C9, 0x19, 0x19},
	{DES_ADDRESS, 0x01CA, 0x1A, 0x1A},

	{DES_ADDRESS, 0x0D03, 0x8B, 0x8B},

	{DES_ADDRESS, 0x0215, 0x04, 0x04},

        {SER_ADDRESS, 0x0003, 0x10, 0x10},
};

static serdes_config_5 serdes_1920_720_12_3_config_CURVED_AUO[] = {
        {SER_ADDRESS, 0x0001, 0x88, 0x88},

	{SER_ADDRESS, 0x0004, 0x0A, 0x0A},

	{DES_ADDRESS, 0x0002, 0x03, 0x03},

        {DES_ADDRESS, 0x0458, 0x28, 0x28},
        {DES_ADDRESS, 0x0558, 0x28, 0x28},
        {DES_ADDRESS, 0x0459, 0x68, 0x68},
        {DES_ADDRESS, 0x0559, 0x68, 0x68},

        {DES_ADDRESS, 0x01CE, 0x5E, 0x5E},
        {DES_DELAY, 0x0000, 0x02, 0x02},
        {DES_ADDRESS, 0x01CF, 0x01, 0x01},

        {SER_ADDRESS, 0x0140, 0x20, 0x20},
        {SER_ADDRESS, 0x0002, 0x03, 0x03},
        {DES_ADDRESS, 0x0140, 0x20, 0x20},
        {DES_DELAY, 0x0000, 0x01, 0x01},
        {DES_ADDRESS, 0x0002, 0x43, 0x43},
#ifdef CONFIG_WIDE_PE_COMMON
	{DES_ADDRESS, 0x0221, 0x80, 0x80},
#endif

        {DES_ADDRESS, 0x0218, 0x01, 0x09},
        {SER_ADDRESS, 0x020C, 0x01, 0x09},
        {SER_ADDRESS, 0x020D, 0x04, 0x04},
        {SER_ADDRESS, 0x020E, 0x04, 0x44},
        {DES_ADDRESS, 0x0219, 0x04, 0x04},
        {DES_ADDRESS, 0x021A, 0x04, 0x44},

        {DES_ADDRESS, 0x020C, 0x01, 0x01},
        {SER_ADDRESS, 0x020F, 0x01, 0x09},
        {SER_ADDRESS, 0x0210, 0x05, 0x05},
        {SER_ADDRESS, 0x0211, 0x05, 0x05},
        {DES_ADDRESS, 0x020D, 0x05, 0x05},
        {DES_ADDRESS, 0x020E, 0x05, 0x45},

        {SER_ADDRESS, 0x021B, 0x01, 0x09},
        {DES_ADDRESS, 0x0224, 0x01, 0x01},
        {SER_ADDRESS, 0x021C, 0x09, 0x09},
        {SER_ADDRESS, 0x021D, 0x09, 0x49},
        {DES_ADDRESS, 0x0225, 0x09, 0x09},
        {DES_ADDRESS, 0x0226, 0x09, 0x49},

        {SER_ADDRESS, 0x0206, 0x01, 0x09},
        {DES_ADDRESS, 0x0212, 0x01, 0x01},
        {SER_ADDRESS, 0x0207, 0x02, 0x02},
        {SER_ADDRESS, 0x0208, 0x02, 0x42},
        {DES_ADDRESS, 0x0213, 0x02, 0x02},
        {DES_ADDRESS, 0x0214, 0x02, 0x42},

        {SER_ADDRESS, 0x0209, 0x01, 0x09},
        {DES_ADDRESS, 0x0221, 0x01, 0x01},
        {SER_ADDRESS, 0x020A, 0x03, 0x03},
        {SER_ADDRESS, 0x020B, 0x03, 0x43},
        {DES_ADDRESS, 0x0222, 0x03, 0x03},
        {DES_ADDRESS, 0x0223, 0x03, 0x43},

        {SER_ADDRESS, 0x0215, 0x01, 0x09},
        {DES_ADDRESS, 0x0215, 0x01, 0x01},
        {SER_ADDRESS, 0x0216, 0x07, 0x07},
        {SER_ADDRESS, 0x0217, 0x07, 0x47},
        {DES_ADDRESS, 0x0216, 0x07, 0x07},
        {DES_ADDRESS, 0x0217, 0x07, 0x47},

        {SER_ADDRESS, 0x0212, 0x01, 0x09},
        {DES_ADDRESS, 0x0206, 0x01, 0x01},
        {SER_ADDRESS, 0x0213, 0x06, 0x06},
        {SER_ADDRESS, 0x0214, 0x06, 0x46},
        {DES_ADDRESS, 0x0207, 0x06, 0x06},
        {DES_ADDRESS, 0x0208, 0x06, 0x46},

	{SER_ADDRESS, 0x020D, 0xA4, 0xa4},
        {SER_ADDRESS, 0x020C, 0x05, 0x0c},
        {DES_ADDRESS, 0x0218, 0x83, 0x8b},
	{SER_ADDRESS, 0x020C, 0x04, 0x0c},

	{SER_ADDRESS, 0x0210, 0xA5, 0xa5},
        {SER_ADDRESS, 0x020F, 0x05, 0x04},
        {DES_ADDRESS, 0x020C, 0x83, 0x83},
	{SER_ADDRESS, 0x020F, 0x04, 0x04},

	{DES_ADDRESS, 0x0225, 0xA9, 0xa9},
        {DES_ADDRESS, 0x0224, 0x85, 0x84},
        {SER_ADDRESS, 0x021B, 0x83, 0x8B},
	{DES_ADDRESS, 0x0224, 0x84, 0x84},

	{DES_ADDRESS, 0x0213, 0xA6, 0xa6},
        {DES_ADDRESS, 0x0212, 0x85, 0x84},
        {SER_ADDRESS, 0x0206, 0x83, 0x8b},
	{DES_ADDRESS, 0x0212, 0x84, 0x84},

	{DES_ADDRESS, 0x0222, 0xA3, 0xa3},
        {DES_ADDRESS, 0x0221, 0x85, 0x84},
        {SER_ADDRESS, 0x0209, 0x83, 0x8b},
	{DES_ADDRESS, 0x0221, 0x84, 0x84},

	{DES_ADDRESS, 0x0216, 0xA7, 0xa7},
        {DES_ADDRESS, 0x0215, 0x05, 0x04},
        {SER_ADDRESS, 0x0215, 0x83, 0x8b},
	{DES_ADDRESS, 0x0215, 0x04, 0x04},

        {DES_ADDRESS, 0x0206, 0x85, 0x85},
	{SER_ADDRESS, 0x0212, 0x83, 0x8b},
	{DES_ADDRESS, 0x0207, 0xA2, 0xa2},

        {DES_ADDRESS, 0x0227, 0x80, 0x80},
#ifdef CONFIG_WIDE_PE_COMMON
	{DES_ADDRESS, 0x0221, 0x90, 0x90},
#endif
        {SER_ADDRESS, 0x0003, 0x10, 0x10},
};


static u16 lcd_type_check_config [2][2] = {
	{DES_ADDRESS, 0x0209},
	{DES_ADDRESS, 0x020F},
};

static u16 ser_6gbps_config[3][3] = {
        {SER_ADDRESS, 0x0001, 0x88},
        {SER_ADDRESS, 0x0010, 0x31},
        {DES_DELAY, 0x0000, 0x64},
};

static u16 des_6gbps_config[3][3] = {
        {DES_ADDRESS, 0x0001, 0x02},
        {DES_ADDRESS, 0x0010, 0x31},
        {DES_DELAY, 0x0000, 0x64},
};

static u16 serdes_3gbps_config[4][3] = {
        {DES_ADDRESS, 0x0001, 0x01},
        {DES_ADDRESS, 0x0010, 0x31},
        {SER_ADDRESS, 0x0001, 0x84},
        {SER_ADDRESS, 0x0010, 0x31},
};

static u16 serdes_line_fault_config [3][3] = {
        {SER_ADDRESS, 0x0004, 0x20},
        {SER_ADDRESS, 0x0004, 0x10},
        {DES_ADDRESS, 0x0022, 0x00},
};

static u16 serdes_read[9][3]={
        {SER_ADDRESS, 0x01C9, 0x88},
        {SER_ADDRESS, 0x0013, 0x47},
        {DES_ADDRESS, 0x0108, 0x24},
	{DES_ADDRESS, 0x0050, 0x24},
	{SER_ADDRESS, 0x0102, 0x00},
	{SER_ADDRESS, 0x0053, 0x00},
        {SER_ADDRESS, 0x0004, 0x00},
        {SER_ADDRESS, 0x0005, 0x00},
	{DES_ADDRESS, 0x0022, 0x00},
};

#endif
