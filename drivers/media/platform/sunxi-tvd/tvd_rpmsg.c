/*
* Allwinner rpmsg-tvd_rpmsg driver.
*
* Copyright(c) 2022-2027 Allwinnertech Co., Ltd.
*
* This file is licensed under the terms of the GNU General Public
* License version 2.  This program is licensed "as is" without any
* warranty of any kind, whether express or implied.
*/
#include <linux/rpmsg.h>
#include <linux/workqueue.h>
#include <linux/remoteproc.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/device.h>
#include "tvd.h"


struct rpmsg_tvd_private *tvd_rpmsg;

static int rpmsg_tvd_cb(struct rpmsg_device *rpdev, void *data, int len,
						void *priv, u32 src)
{
	struct tvd_packet *pack;
	tvd_dbg("rpmsg_tvd__cb \n");

	pack = (struct tvd_packet *)data;

	if (pack->magic != TVD_PACKET_MAGIC || len != sizeof(*pack)) {
		tvd_wrn("packet invalid magic or size %d %d %x\n",
				len, (int)sizeof(*pack), pack->magic);
		return 0;
	}

	mutex_lock(&tvd_rpmsg->lock);

	if (tvd_rpmsg->tvd_ops == NULL) {
		tvd_dbg("please pass tvd_ops to rpmsg \n");
		mutex_unlock(&tvd_rpmsg->lock);
		return 0;
	}

	if (pack->type == RV_TVD_START && tvd_rpmsg->control != CONTROL_BY_RTOS) {
		tvd_rpmsg->tvd_ops->rv_start(tvd_rpmsg->tvd);
		amp_tvd_rpmsg_send(RV_TVD_START_ACK);
		tvd_rpmsg->control = CONTROL_BY_RTOS;
	} else if (pack->type == RV_TVD_STOP && tvd_rpmsg->control != CONTROL_BY_ARM) {
		tvd_rpmsg->tvd_ops->rv_stop(tvd_rpmsg->tvd);
		amp_tvd_rpmsg_send(RV_TVD_STOP_ACK);
		tvd_rpmsg->control = CONTROL_BY_NONE;
	} else if (pack->type == ARM_TVD_START_ACK) {
		tvd_rpmsg->control = CONTROL_BY_ARM;
	} else if (pack->type == ARM_TVD_STOP_ACK) {
		tvd_rpmsg->control = CONTROL_BY_NONE;
	}

	tvd_wrn("pack_type %d control by:%d\n",
		pack->type, tvd_rpmsg->control);

	mutex_unlock(&tvd_rpmsg->lock);

	return 0;
}

int amp_tvd_rpmsg_send(enum tvd_packet_type type)
{
	struct tvd_packet packet;
	int ret = 0;

	if ((CONTROL_BY_ARM == tvd_rpmsg->control && ARM_TVD_START == type) ||
			(CONTROL_BY_RTOS == tvd_rpmsg->control && ARM_TVD_STOP == type))
		return -1;

	if (!tvd_rpmsg->ept)
		ret = -1;

	memset(&packet, 0, sizeof(packet));
	packet.magic = TVD_PACKET_MAGIC;
	packet.type = type;

	ret = rpmsg_send(tvd_rpmsg->ept, &packet, sizeof(packet));

	return ret;
}

static int rpmsg_tvd_probe(struct rpmsg_device *rpdev)
{
	mutex_lock(&tvd_rpmsg->lock);
	tvd_rpmsg->ept = rpdev->ept;
	tvd_rpmsg->rpdev = rpdev;
	rpdev->announce = rpdev->src != RPMSG_ADDR_ANY;
	mutex_unlock(&tvd_rpmsg->lock);
	return 0;
}

static void rpmsg_tvd_remove(struct rpmsg_device *rpdev)
{
	dev_info(&rpdev->dev, "%s is removed\n", dev_name(&rpdev->dev));
	mutex_lock(&tvd_rpmsg->lock);
	tvd_rpmsg->ept = NULL;
	mutex_unlock(&tvd_rpmsg->lock);
}


static struct rpmsg_device_id rpmsg_driver_tvd_id_table[] = {
	{ .name	= "sunxi,rpmsg_tvd" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, rpmsg_driver_tvd_id_table);

static struct rpmsg_driver rpmsg_tvd_client = {
	.drv = {
		.name	= KBUILD_MODNAME,
	},
	.id_table	= rpmsg_driver_tvd_id_table,
	.probe		= rpmsg_tvd_probe,
	.callback	= rpmsg_tvd_cb,
	.remove		= rpmsg_tvd_remove,
};

void amp_tvd_init(void *ops, struct tvd_dev *tvd)
{
	struct rpmsg_tvd_private *p;
	p = kzalloc(sizeof(*tvd_rpmsg), GFP_KERNEL);
	mutex_init(&p->lock);
	tvd_rpmsg = p;
	tvd_rpmsg->tvd_ops = (struct tvd_amp_ctrl *) ops;
	tvd_rpmsg->tvd = tvd;
	tvd->rpmsg_tvd = tvd_rpmsg;
	register_rpmsg_driver(&rpmsg_tvd_client);
	tvd_rpmsg->control = CONTROL_BY_NONE;
}

void amp_tvd_exit(void)
{
	unregister_rpmsg_driver(&rpmsg_tvd_client);
	kfree(tvd_rpmsg);
}