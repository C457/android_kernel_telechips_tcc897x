#ifndef __LINUX_IBOX_API__IOCTL_PIN_H__
#define __LINUX_IBOX_API__IOCTL_PIN_H__

#include "ioctl_pin_id.h"

typedef enum {
	IBOX_IOPIN_STATE_ACTIVE = 0x10,
	IBOX_IOPIN_STATE_INACTIVE,
	IBOX_IOPIN_STATE_NOEXIST,
	IBOX_IOPIN_STATE_ERROR,
	IBOX_IOPIN_STATE_OK,
} ibox_iopin_state_t;

typedef enum {
	IBOX_IOPIN_CMD_FORCE_0 = 0, /* for compatibility */
	IBOX_IOPIN_CMD_FORCE_1 = 1, /* for compatibility */
	IBOX_IOPIN_CMD_ACTIVATE = 0x100,
	IBOX_IOPIN_CMD_DEACTIVATE,
	IBOX_IOPIN_CMD_GETSTATE,

	IBOX_IOPIN_CMD_SCRIPT_DELAY_MS,
} ibox_iopin_cmd_t;


#define IBOX_PIN_IOC_MAGIC_V1	'P'
#define __IBOX_PIN_IOC_CUSTOM_NR (10)
	
/*
   unsigned int pin_maxnr;
   ret = ioctl(fd, IBOX_PINIOC_GET_MAXNR, &pin_maxnr);
   if (ret == 0)
	  printf("%d\n", pin_nr);
 */
#define IBOX_PINIOC_GET_MAXNR	\
		_IOWR(IBOX_PIN_IOC_MAGIC_V1, 0, unsigned int)

/*
   struct ibox_pinioc_pindata data;
   for (i = 0; i < pin_maxnr; i++) {
      data.pin_id = (ibox_iopin_id_t)i;
      ret = ioctl(ds, IBOX_PINIOC_GET_PINDATA, &data);
      if (ret == 0) {
	  printf("%s\n", data.name);
      }
   }
*/
#define PINIOC_NAME_MAX	(32)
struct ibox_pinioc_pindata {
	ibox_iopin_id_t		pin_id;
	ibox_iopin_state_t	state;
	char			name[PINIOC_NAME_MAX];
};
#define IBOX_PINIOC_GET_PINDATA	\
		_IOWR(IBOX_PIN_IOC_MAGIC_V1, 1, struct ibox_pinioc_pindata)


/*
 * struct ibox_pinioc_cmddata my_data[] = {
 *  { CTL_DRM_RESET,          IBOX_IOPIN_CMD_ACTIVE, 0, },
 *  {             1, IBOX_IOPIN_CMD_SCRIPT_DELAY_MS, 0, },
 *  { CTL_DRM_RESET,        IBOX_IOPIN_CMD_INACTIVE, 0, },
 * };
 *
 * #define count_of(x) sizeof(x)/sizeof(x[0])
 * ret = ioctl(fd, IBOX_PINIOC_CMDS(count_of(my_data)), &mydata[0]);
 */
struct ibox_pinioc_cmddata {
	ibox_iopin_id_t		pin_id;
	ibox_iopin_cmd_t	ctl_cmd;
	ibox_iopin_state_t	cmd_ret;
};

#define __PIN_CMDSIZE(N) \
     ((((N)*(sizeof(struct ibox_pinioc_cmddata))) < (1 << _IOC_SIZEBITS)) ? ((N)*(sizeof(struct ibox_pinioc_cmddata))) : 0)

#define IBOX_PINIOC_CMDS(N) \
	_IOWR(IBOX_PIN_IOC_MAGIC_V1, ((N) + __IBOX_PIN_IOC_CUSTOM_NR), char[__PIN_CMDSIZE(N)])

#endif /* __LINUX_IBOX_API__IOCTL_PIN_H__ */
