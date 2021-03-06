/*
 *  tcc_dxb_kernel.c
 *
 *  Written by C2-G1-3T
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.=
 */

#include <linux/platform_device.h>

#include "dvb_frontend.h"
#include "dvb_demux.h"
#include "dmxdev.h"

#include "tcc_fe.h"
#include "tcc_dmx.h"
#include "tcc_tsif_interface.h"

#include "tcc_dxb_kernel.h"


/*****************************************************************************
 * Log Message
 ******************************************************************************/
#define LOG_TAG    "[TCC_DXB_DRV]"

static int dev_debug = 0;

module_param(dev_debug, int, 0644);
MODULE_PARM_DESC(dev_debug, "Turn on/off device debugging (default:off).");

#define dprintk(msg...)                                \
{                                                      \
	if (dev_debug)                                     \
		printk(KERN_INFO LOG_TAG "(D)" msg);           \
}

#define eprintk(msg...)                                \
{                                                      \
	printk(KERN_INFO LOG_TAG " (E) " msg);             \
}


/*****************************************************************************
 * Defines
 ******************************************************************************/
#define DRV_NAME      "tcc_dxb_drv"

#define FE_DEV_COUNT   1
#define TSIF_DEV_COUNT 1 // the number of tsif/hwdmx
#define DMX_DEV_COUNT  2 // the number of linuxtv demux


/*****************************************************************************
 * structures
 ******************************************************************************/


/*****************************************************************************
 * Variables
 ******************************************************************************/
DVB_DEFINE_MOD_OPT_ADAPTER_NR(adapter_nr);


/*****************************************************************************
 * External Functions
 ******************************************************************************/


/*****************************************************************************
 * Functions
 ******************************************************************************/


/*****************************************************************************
 * tcc_dxb_drv Init/Deinit
 ******************************************************************************/
static void tcc_dxb_drv_init(tcc_dxb_inst_t *inst)
{
	inst->fe.dev_num = FE_DEV_COUNT;
	inst->fe.adapter = &inst->adapter;
	inst->fe.of_node = inst->dev->of_node;
	tcc_fe_init(&inst->fe);

	inst->tsif.dev_num = TSIF_DEV_COUNT;
	inst->tsif.adapter = &inst->adapter;
	tcc_tsif_init(&inst->tsif);

	inst->dmx.dev_num = DMX_DEV_COUNT;
	inst->dmx.adapter = &inst->adapter;
	tcc_dmx_init(&inst->dmx);

	dprintk("%s\n", __FUNCTION__);
}

static void tcc_dxb_drv_deinit(tcc_dxb_inst_t *inst)
{
	tcc_dmx_deinit(&inst->dmx);

	tcc_tsif_deinit(&inst->tsif);

	tcc_fe_deinit(&inst->fe);

	dprintk("%s\n", __FUNCTION__);
}


/*****************************************************************************
 * tcc_dxb_drv Prove/Remove
 ******************************************************************************/
static int tcc_dxb_drv_probe(struct platform_device *pdev)
{
	tcc_dxb_inst_t *inst;

	inst = kzalloc(sizeof(tcc_dxb_inst_t), GFP_KERNEL);
	if (inst == NULL)
	{
		eprintk("%s(kzalloc fail)\n", __FUNCTION__);
		return -ENOMEM;
	}

	if (dvb_register_adapter(&inst->adapter, pdev->name, THIS_MODULE, &pdev->dev, adapter_nr) < 0)
	{
		eprintk("%s(Failed to register dvb adapter)\n", __FUNCTION__);
		kfree(inst);
		return -ENOMEM;
	}

	inst->dev = &pdev->dev;

	tcc_dxb_drv_init(inst);

	dev_set_drvdata(&pdev->dev, inst);

	printk("%s\n", __FUNCTION__);

	return 0;
}

static int tcc_dxb_drv_remove(struct platform_device *pdev)
{
	tcc_dxb_inst_t *inst;

	inst = dev_get_drvdata(&pdev->dev);
	if (inst == NULL)
	{
		return 0;
	}

	tcc_dxb_drv_deinit(inst);

	dvb_unregister_adapter(&inst->adapter);

	kfree(inst);

	dprintk("%s\n", __FUNCTION__);

    return 0;
}


/*****************************************************************************
 * tcc_dxb_drv Module Init/Exit
 ******************************************************************************/
#ifdef CONFIG_OF
static const struct of_device_id tcc_dxb_drv_of_match[] = {
	{.compatible = "telechips,tcc_dxb_drv", },
	{.compatible = "telechips,tcc_dxb_drv,TS0", },
	{.compatible = "telechips,tcc_dxb_drv,TS1", },
	{.compatible = "telechips,tcc_dxb_drv,TS2", },
	{.compatible = "telechips,tcc_dxb_drv,TS3", },
	{.compatible = "telechips,tcc_dxb_drv,TS4", },
	{.compatible = "telechips,tcc_dxb_drv,TS5", },
	{.compatible = "telechips,tcc_dxb_drv,TS6", },
	{.compatible = "telechips,tcc_dxb_drv,TS7", },
	{ },
};
MODULE_DEVICE_TABLE(of, tcc_dxb_drv_of_match);
#endif

static struct platform_driver tcc_dxb_drv = {
	.probe	= tcc_dxb_drv_probe,
	.remove	= tcc_dxb_drv_remove,
	.driver	= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = tcc_dxb_drv_of_match,
#endif
	},
};

static int __init tcc_dxb_drv_module_init(void)
{
	dprintk("%s\n", __FUNCTION__);

	platform_driver_register(&tcc_dxb_drv);

	return 0;
}

static void __exit tcc_dxb_drv_module_exit(void)
{
	platform_driver_unregister(&tcc_dxb_drv);

	dprintk("%s\n", __FUNCTION__);
}

module_init(tcc_dxb_drv_module_init);
module_exit(tcc_dxb_drv_module_exit);

MODULE_AUTHOR("Telechips Inc.");
MODULE_DESCRIPTION("TCCxxx dxb drv module");
MODULE_LICENSE("GPL");
