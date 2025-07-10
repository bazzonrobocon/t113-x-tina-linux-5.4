/*
* SPDX-License-Identifier: GPL-2.0-or-later
*
* Allwinner MAILBOX-HEARTBEAT driver.
*
* Copyright(c) 2022-2027 Allwinnertech Co., Ltd.
*
* This file is licensed under the terms of the GNU General Public
* License version 2.  This program is licensed "as is" without any
* warranty of any kind, whether express or implied.
*/

/* #define DEBUG */
#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/sched/signal.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/remoteproc.h>

#define MBOX_HEARTBEAT_TIMEOUT	(3 * HZ)  /* 3s */

struct mbox_heartbeat {
	struct device		*dev;
	struct device_node	*rproc_np;
	struct timer_list	timer;
	struct mbox_chan	*tx_channel;
	struct mbox_chan	*rx_channel;
	u64			ticks;
};

static const char *rproc_name;  /* global rproc_name to report remote processor carsh */
extern int sunxi_rproc_report_crash(char *name, enum rproc_crash_type type);

static void mbox_heartbeat_timeout_handler(struct timer_list *unused);
static DEFINE_TIMER(mbox_heartbeat_timer, mbox_heartbeat_timeout_handler);

static void mbox_heartbeat_timeout_handler(struct timer_list *unused)
{
	sunxi_rproc_report_crash(rproc_name, RPROC_WATCHDOG);
}

static void mbox_heartbeat_receive_message(struct mbox_client *client, void *message)
{
	struct mbox_heartbeat *chip = dev_get_drvdata(client->dev);
	unsigned long flags;

	chip->ticks++;

	/* when dsp mailbox-heartbeat-thread start, linux mbox_heartbeat_timer start */
	mod_timer(&mbox_heartbeat_timer, jiffies + MBOX_HEARTBEAT_TIMEOUT);
}

static void mbox_heartbeat_message_sent(struct mbox_client *client,
				   void *message, int timeout)
{
	if (timeout)
		dev_warn(client->dev,
			 "Client: Message could not be sent: %d\n", r);
	else
		dev_info(client->dev,
			 "Client: Message sent\n");
}

static struct mbox_chan *
mbox_heartbeat_request_channel(struct platform_device *pdev, const char *name)
{
	struct mbox_client *client;
	struct mbox_chan *channel;

	client = devm_kzalloc(&pdev->dev, sizeof(*client), GFP_KERNEL);
	if (!client)
		return ERR_PTR(-ENOMEM);

	client->dev		= &pdev->dev;
	client->rx_callback	= mbox_heartbeat_receive_message;
	client->tx_done		= mbox_heartbeat_message_sent;
	client->tx_block	= true;
	client->knows_txdone	= false;
	client->tx_tout		= 500;

	channel = mbox_request_channel_byname(client, name);
	if (IS_ERR(channel)) {
		dev_warn(&pdev->dev, "Failed to request %s channel\n", name);
		return NULL;
	}

	return channel;
}

static ssize_t mbox_heartbeat_ticks_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mbox_heartbeat *chip = dev_get_drvdata(dev);

	return sprintf(buf, "DSP heartbeat ticks: %lld\n", chip->ticks);
}

static DEVICE_ATTR(heartbeat_ticks, 0444, mbox_heartbeat_ticks_show, NULL);

static void mbox_heartbeat_sysfs_create(struct device *dev)
{
	device_create_file(dev, &dev_attr_heartbeat_ticks);
}

static void mbox_heartbeat_sysfs_destroy(struct device *dev)
{
	device_remove_file(dev, &dev_attr_heartbeat_ticks);
}

static int mbox_heartbeat_probe(struct platform_device *pdev)
{
	struct mbox_heartbeat *chip;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->rproc_np = of_parse_phandle(np, "rproc-np", 0);
	if (!chip->rproc_np) {
		dev_err(&pdev->dev, "no rproc-np in mailbox-heartbeat node");
		return -EINVAL;
	}

	/* Save rproc_np name to global rproc_name, to use it in timer handler */
	rproc_name = chip->rproc_np->name;
	chip->dev = &pdev->dev;
	platform_set_drvdata(pdev, chip);

	chip->tx_channel = mbox_heartbeat_request_channel(pdev, "tx");
	chip->rx_channel = mbox_heartbeat_request_channel(pdev, "rx");

	if (!chip->tx_channel && !chip->rx_channel)
		return -EPROBE_DEFER;

	mbox_heartbeat_sysfs_create(chip->dev);

	return 0;
}

static int mbox_heartbeat_remove(struct platform_device *pdev)
{
	struct mbox_heartbeat *chip = platform_get_drvdata(pdev);

	if (chip->tx_channel)
		mbox_free_channel(chip->tx_channel);
	if (chip->rx_channel)
		mbox_free_channel(chip->rx_channel);

	mbox_heartbeat_sysfs_destroy(chip->dev);

	return 0;
}

static const struct of_device_id mbox_heartbeat_match[] = {
	{ .compatible = "mailbox-heartbeat" },
	{},
};
MODULE_DEVICE_TABLE(of, mbox_heartbeat_match);

static struct platform_driver mbox_heartbeat_driver = {
	.driver = {
		.name = "mailbox-heartbeat",
		.of_match_table = mbox_heartbeat_match,
	},
	.probe  = mbox_heartbeat_probe,
	.remove = mbox_heartbeat_remove,
};
module_platform_driver(mbox_heartbeat_driver);

MODULE_DESCRIPTION("Mailbox heartbeat for single remote processor");
MODULE_AUTHOR("xuminghui <xuminghui@allwinnertech.com");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0.0");
