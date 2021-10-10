#include <linux/gpio.h>
#include <linux/irq.h>

#include <mach/io.h>
#include <mach/bsp.h>

#include "tcc_hw_event.h"
#include "rear_cam_drv.h"

#include <linux/delay.h>

#define REAR_CAM_DEBUG                  0
#define dprintk(format, msg...)         if(REAR_CAM_DEBUG) { printk("[REAR_CAM] "format"", ##msg); }
#define dfnprintk(format, msg...)       if(REAR_CAM_DEBUG) { printk("[REAR_CAM] %s: "format"", __func__,  ##msg); }

void RCAM_ExternalINT_POL_change(void)
{
        unsigned long pin_status = gpio_get_value(rear_cam_soc->gear_port);
        if(pin_status) {
                dprintk("RCAM_ExternalINT_POL_change LOW\r\n");
                 irq_set_irq_type(rear_cam_soc->gear_port_irq, IRQ_TYPE_LEVEL_LOW);
        }
        else {
                dprintk("RCAM_ExternalINT_POL_change HIGH\r\n");
                irq_set_irq_type(rear_cam_soc->gear_port_irq, IRQ_TYPE_LEVEL_HIGH);
        }
}
