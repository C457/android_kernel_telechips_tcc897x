#ifndef _TCC_EP9553T_CEC_H_
#define _TCC_EP9553T_CEC_H_

#define REG_LEN		(2)

#define REG_EDID_BASE				(0x0000)
#define REG_EDID_SIZE				(256)

//Control
#define REG_SYSTEM_0				(0x0100)
#define REG_SYSTEM_1				(0x0101)
#define REG_SYSTEM_2				(0x0102)
#define REG_FEATURE_ENABLE_0		(0x0103)
#define REG_FEATURE_ENABLE_1		(0x0104)
#define REG_INTERRUPT_FLAGS			(0x0105)

#define SYSTEM_1_DEV_SEL			(1 << 0)
#define SYSTEM_2_CCI_EN				(1 << 0)
#define SYSTEM_2_ECI_EN				(1 << 1)
#define SYSTEM_2_HIGH_LEVEL			(1 << 4)
#define SYSTEM_2_LISTEN				(1 << 5)
#define SYSTEM_2_AUTO_STB			(1 << 7)

#define INTERRUPT_FLAG_CCF			(1 << 0)
#define INTERRUPT_FLAG_ECF			(1 << 1)
                                    
//Send Command                      
#define REG_SEND_COMMAND			(0x0106)

enum EP9553T_CEC_SEND_CODE {
	SEND_CODE_TRANSMIT_CEC_MESSAGE	= 1,
	SEND_CODE_SET_LOGICAL_ADDRESS	= 2,
};


//Device Topo                       
#define REG_PHYSICAL_ADDR_HIGH		(0x0107)
#define REG_PHYSICAL_ADDR_LOW		(0x0108)
#define REG_DEVICE_TYPE_1			(0x0109)
#define REG_DEVICE_TYPE_2			(0x010A)
                                    
//Device Power Status               
#define REG_POWER_STATUS			(0x010B)
                                    
//Device info Fixed                 
#define REG_LANGUAGE_0				(0x010C)
#define REG_LANGUAGE_1				(0x010D)
#define REG_LANGUAGE_2				(0x010E)
#define REG_VENDOR_ID_0				(0x010F)
#define REG_VENDOR_ID_1				(0x0110)
#define REG_VENDOR_ID_2				(0x0111)
#define REG_OSD_NAME_0				(0x0112)
#define REG_OSD_NAME_1				(0x0113)
#define REG_OSD_NAME_2				(0x0114)
#define REG_OSD_NAME_3				(0x0115)
#define REG_OSD_NAME_4				(0x0116)
#define REG_OSD_NAME_5				(0x0117)
#define REG_OSD_NAME_6				(0x0118)
#define REG_OSD_NAME_7				(0x0119)
#define REG_OSD_NAME_8				(0x011A)
#define REG_OSD_NAME_9				(0x011B)
#define REG_OSD_NAME_10				(0x011C)
#define REG_OSD_NAME_11				(0x011D)
#define REG_OSD_NAME_12				(0x011E)
#define REG_OSD_NAME_13				(0x011F)
                                    
//Device info Update                
#define REG_CURRENT_SOURCE_0		(0x0120)
#define REG_CURRENT_SOURCE_1		(0x0121)
#define REG_CURRENT_SOURCE_2		(0x0122)
#define REG_CURRENT_SOURCE_3		(0x0123)
#define REG_CURRENT_SOURCE_4		(0x0124)
#define REG_CURRENT_SOURCE_5		(0x0125)
#define REG_CURRENT_SOURCE_6		(0x0126)
#define REG_CURRENT_SOURCE_7		(0x0127)

//Collected info
#define REG_COLLECTED_INFO_TV		(0x200)
#define REG_COLLECTED_INFO_REC_1	(0x210)
#define REG_COLLECTED_INFO_REC_2	(0x220)
#define REG_COLLECTED_INFO_TUNER_1	(0x230)
#define REG_COLLECTED_INFO_PLAY_1	(0x240)
#define REG_COLLECTED_INFO_AUDIO	(0x250)
#define REG_COLLECTED_INFO_TUNER_2	(0x260)
#define REG_COLLECTED_INFO_TUNER_3	(0x270)
#define REG_COLLECTED_INFO_PLAY_2	(0x280)
#define REG_COLLECTED_INFO_REC_3	(0x290)
#define REG_COLLECTED_INFO_TUNER_4	(0x2A0)
#define REG_COLLECTED_INFO_PLAY_3	(0x2B0)
#define REG_COLLECTED_INFO_FREE_USE	(0x2E0)

#define REG_COLLECTED_OSD_TV		(0x300)
#define REG_COLLECTED_OSD_REC_1		(0x310)
#define REG_COLLECTED_OSD_REC_2		(0x320)
#define REG_COLLECTED_OSD_TUNER_1	(0x330)
#define REG_COLLECTED_OSD_PLAY_1	(0x340)
#define REG_COLLECTED_OSD_AUDIO		(0x350)
#define REG_COLLECTED_OSD_TUNER_2	(0x360)
#define REG_COLLECTED_OSD_TUNER_3	(0x370)
#define REG_COLLECTED_OSD_PLAY_2	(0x380)
#define REG_COLLECTED_OSD_REC_3		(0x390)
#define REG_COLLECTED_OSD_TUNER_4	(0x3A0)
#define REG_COLLECTED_OSD_PLAY_3	(0x3B0)
#define REG_COLLECTED_OSD_FREE_USE	(0x3E0)


//Firmware Revision
#define REG_REVISION_0				(0x0400)
#define REG_REVISION_1				(0x0401)

//CEC-Command
#define REG_COMMAND_PARAMETER_00	(0x0500)
#define REG_COMMAND_PARAMETER_01	(0x0501)
#define REG_COMMAND_PARAMETER_02	(0x0502)
#define REG_COMMAND_PARAMETER_03	(0x0503)
#define REG_COMMAND_PARAMETER_04	(0x0504)
#define REG_COMMAND_PARAMETER_05	(0x0505)
#define REG_COMMAND_PARAMETER_06	(0x0506)
#define REG_COMMAND_PARAMETER_07	(0x0507)
#define REG_COMMAND_PARAMETER_08	(0x0508)
#define REG_COMMAND_PARAMETER_09	(0x0509)
#define REG_COMMAND_PARAMETER_0A	(0x050A)
#define REG_COMMAND_PARAMETER_0B	(0x050B)
#define REG_COMMAND_PARAMETER_0C	(0x050C)
#define REG_COMMAND_PARAMETER_0D	(0x050D)
#define REG_COMMAND_PARAMETER_0E	(0x050E)
#define REG_COMMAND_PARAMETER_0F	(0x050F)
#define REG_COMMAND_PARAMETER_10	(0x0510)
#define REG_COMMAND_PARAMETER_11	(0x0511)
#define REG_COMMAND_PARAMETER_12	(0x0512)
#define REG_COMMAND_PARAMETER_13	(0x0513)
#define REG_COMMAND_PARAMETER_14	(0x0514)
#define REG_COMMAND_PARAMETER_15	(0x0515)
#define REG_COMMAND_PARAMETER_16	(0x0516)
#define REG_COMMAND_PARAMETER_17	(0x0517)
#define REG_COMMAND_PARAMETER_18	(0x0518)
#define REG_COMMAND_PARAMETER_19	(0x0519)
#define REG_COMMAND_PARAMETER_1A	(0x051A)
#define REG_COMMAND_PARAMETER_1B	(0x051B)
#define REG_COMMAND_PARAMETER_1C	(0x051C)
#define REG_COMMAND_PARAMETER_1D	(0x051D)
#define REG_COMMAND_PARAMETER_1E	(0x051E)
#define REG_COMMAND_STATUS			(0x0600)

//CEC-Event
#define REG_EVENT_STATUS_BUFFER_00	(0x0700)
#define REG_EVENT_STATUS_BUFFER_01	(0x0701)
#define REG_EVENT_STATUS_BUFFER_02	(0x0702)
#define REG_EVENT_STATUS_BUFFER_03	(0x0703)
#define REG_EVENT_STATUS_BUFFER_04	(0x0704)
#define REG_EVENT_STATUS_BUFFER_05	(0x0705)
#define REG_EVENT_STATUS_BUFFER_06	(0x0706)
#define REG_EVENT_STATUS_BUFFER_07	(0x0707)
#define REG_EVENT_STATUS_BUFFER_08	(0x0708)
#define REG_EVENT_STATUS_BUFFER_09	(0x0709)
#define REG_EVENT_STATUS_BUFFER_0A	(0x070A)
#define REG_EVENT_STATUS_BUFFER_0B	(0x070B)
#define REG_EVENT_STATUS_BUFFER_0C	(0x070C)
#define REG_EVENT_STATUS_BUFFER_0D	(0x070D)
#define REG_EVENT_STATUS_BUFFER_0E	(0x070E)
#define REG_EVENT_STATUS_BUFFER_0F	(0x070F)
#define REG_EVENT_STATUS_BUFFER_10	(0x0710)
#define REG_EVENT_STATUS_BUFFER_11	(0x0711)
#define REG_EVENT_STATUS_BUFFER_12	(0x0712)

//System
#define REG_SYS_CONTROL				(0x800)
#define REG_SYS_STATUS				(0x900)

#define SYS_CONTROL_CEC_AUTO		(1<<0)
#define SYS_STATUS_CBUS_CON			(1<<1)
#define SYS_STATUS_CDS				(1<<0)

#endif /*_TCC_EP9553T_CEC_H_*/