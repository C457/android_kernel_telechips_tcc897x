#ifndef __DAUDIO_ENG__
#define __DAUDIO_ENG__

#include <linux/major.h>

#define DAUDIO_ENG_MAJOR		232

#define ENGMODE_CTRL_DEV		"daudio_eng"
#define ENGMODE_CTRL_DRIVER 	(ENGMODE_CTRL_DEV)
#define ENGMODE_IOC_MAGIC		(DAUDIO_ENG_MAJOR)

/**  
 * @author sjpark@cleinsoft
 * @date 2014/07/29
 * Engineering mode IOCTL Commands
 **/
// ********* IOCTL COMMAND LIST ********** //
typedef enum {
#if !defined(CONFIG_DAUDIO_KK)
	// FM1288 UART GPIO Pin control command
	ENGMODE_CTRL_FM1288_UART_SET = 0,
	ENGMODE_CTRL_FM1288_UART_GET,
#endif
	ENGMODE_CTRL_SETTING_MODE_SET,
	ENGMODE_CTRL_SETTING_MODE_GET,
	ENGMODE_IOC_MAX
} daudio_eng_ioc;
// ****************************************//

typedef enum {
	INPUT,
	FUNCTION,
	INVALID
} daudio_eng_uart_status;

#if !defined(CONFIG_DAUDIO_KK)
#define ENG_IOC_FM1288_UART_SET      _IOWR(DAUDIO_ENG_MAJOR, ENGMODE_CTRL_FM1288_UART_SET, int) 
#define ENG_IOC_FM1288_UART_GET      _IOWR(DAUDIO_ENG_MAJOR, ENGMODE_CTRL_FM1288_UART_GET, int) 
#endif

typedef struct {
	unsigned int name;
	unsigned int value;
} __attribute__((packed))ioc_info;

#define EM_SETTING_ID		0x45444F4D5F4D45
#define EM_MODE_DISABLE		0x0
#define EM_MODE_ENABLE		0x1

typedef struct _em_setting_info_t {
	unsigned long long id;
	unsigned int mode;
	int temp;
} em_setting_info;

#define ENG_IOC_SETTING_SET      _IOWR(DAUDIO_ENG_MAJOR, ENGMODE_CTRL_SETTING_MODE_SET, int)
#define ENG_IOC_SETTING_GET      _IOWR(DAUDIO_ENG_MAJOR, ENGMODE_CTRL_SETTING_MODE_GET, int)

#endif
