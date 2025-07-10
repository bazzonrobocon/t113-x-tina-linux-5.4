/*
* Allwinner rpmsg-heartbeat driver.
*
* Copyright(c) 2022-2027 Allwinnertech Co., Ltd.
*
* This file is licensed under the terms of the GNU General Public
* License version 2.  This program is licensed "as is" without any
* warranty of any kind, whether express or implied.
*/

/* #define DEBUG */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/rpmsg.h>
#include <linux/workqueue.h>
#include <linux/remoteproc.h>
#include <linux/types.h>
#include <linux/io.h>

struct rpmsg_openamp_test {
	struct rpmsg_device *rpdev;
	void __iomem *time_reglow;
};

#define OPENAMP_JITTER_PACKET_MAGIC	(0x12344321)
#define OPENAMP_JITTER_TIMER_INIT	(0)
#define OPENAMP_JITTER_TEST_START	(1)
#define OPENAMP_JITTER_TIMER_INIT_ACK	(2)
#define OPENAMP_JITTER_TIMER_START_ACK	(3)

struct openamp_jitter_packet {
	int magic;
	int type;
	unsigned int timer_reglow;
	unsigned int rt2linux_cnt;
	unsigned int linux2rt_cnt;
	int timer_freq;
};

static int rpmsg_openamp_test_cb(struct rpmsg_device *rpdev, void *data, int len,
						void *priv, u32 src)
{
	struct rpmsg_openamp_test *chip = dev_get_drvdata(&rpdev->dev);
	struct openamp_jitter_packet *pack = data;
	struct openamp_jitter_packet ack_pack;

	if (pack->magic != OPENAMP_JITTER_PACKET_MAGIC) {
		dev_err(&rpdev->dev, "magic invalid, err value %d\n", pack->magic);
		return 0;
	}
	switch (pack->type) {
	case OPENAMP_JITTER_TIMER_INIT:
		chip->time_reglow = ioremap(pack->timer_reglow, 4);
		ack_pack.magic = OPENAMP_JITTER_PACKET_MAGIC;
		ack_pack.type = OPENAMP_JITTER_TIMER_INIT_ACK;
		rpmsg_send(chip->rpdev->ept, &ack_pack, sizeof(struct openamp_jitter_packet));
		break;
	case OPENAMP_JITTER_TEST_START:
		ack_pack.rt2linux_cnt = readl(chip->time_reglow);
		ack_pack.magic = OPENAMP_JITTER_PACKET_MAGIC;
		ack_pack.type = OPENAMP_JITTER_TIMER_START_ACK;
		rpmsg_send(chip->rpdev->ept, &ack_pack, sizeof(struct openamp_jitter_packet));
		iounmap(chip->time_reglow);
		break;
	}

	return 0;
}

static int rpmsg_openamp_test_probe(struct rpmsg_device *rpdev)
{
	struct rpmsg_openamp_test *chip;

	chip = devm_kzalloc(&rpdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->rpdev = rpdev;

	dev_set_drvdata(&rpdev->dev, chip);
	/* wo need to announce the new ept to remote */
	rpdev->announce = rpdev->src != RPMSG_ADDR_ANY;

	return 0;
}

static void rpmsg_openamp_test_remove(struct rpmsg_device *rpdev)
{
	dev_info(&rpdev->dev, "%s is removed\n", dev_name(&rpdev->dev));
}

static struct rpmsg_device_id rpmsg_driver_openamp_test_id_table[] = {
	{ .name	= "sunxi,rpmsg_openamp_test" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, rpmsg_driver_openamp_test_id_table);

static struct rpmsg_driver rpmsg_sample_client = {
	.drv = {
		.name	= KBUILD_MODNAME,
	},
	.id_table	= rpmsg_driver_openamp_test_id_table,
	.probe		= rpmsg_openamp_test_probe,
	.callback	= rpmsg_openamp_test_cb,
	.remove		= rpmsg_openamp_test_remove,
};
module_rpmsg_driver(rpmsg_sample_client);

MODULE_DESCRIPTION("Remote Processor OpenAmp Test Receive Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("wujiayi <wujiayi@allwinnertech.com>");
MODULE_VERSION("1.0.0");
