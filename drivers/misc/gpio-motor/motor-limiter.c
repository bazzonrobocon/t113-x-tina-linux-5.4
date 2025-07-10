// SPDX-License-Identifier: GPL-2.0-only
/*
 * drivers/misc/gpio-motor/motor-limiter.c
 *
 * Copyright (C) 2022 Allwinner.
 * Jingyan Liang <jingyanliang@allwinnertech.com>
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/of_platform.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

struct motor_limiter {
	struct platform_device *pdev;
	struct device *dev;

	int gpio;
};

static ssize_t motor_limiter_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct motor_limiter *limiter = (struct motor_limiter *)platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", gpio_get_value(limiter->gpio));
}

static struct device_attribute motor_limiter_attr = {
	.attr = {
		.name = "motor_limiter",
		.mode = 0444,
	},
	.show = motor_limiter_show,
	.store = NULL,
};

static int motor_limiter_fdt_parse(struct motor_limiter *limiter, struct device_node *np)
{
	int err;

	limiter->gpio = of_get_named_gpio_flags(np, "limiter-gpio", 0, NULL);
	if (limiter->gpio < 0) {
		dev_err(limiter->dev, "failed to get property limiter-gpio\n");
		return -EINVAL;
	} else {
		err = gpio_request(limiter->gpio, dev_name(limiter->dev));
		if (err < 0) {
			dev_err(limiter->dev, "failed to request gpio_%d as %s\n", limiter->gpio, dev_name(limiter->dev));
			return err;
		} else {
			gpio_direction_input(limiter->gpio);
			dev_dbg(limiter->dev, "gpio_%d intput", limiter->gpio);
		}
	}

	return 0;
}

static int motor_limiter_probe(struct platform_device *pdev)
{
	struct motor_limiter *limiter = NULL;
	int err;

	limiter = devm_kzalloc(&pdev->dev, sizeof(*limiter), GFP_KERNEL);
	if (IS_ERR_OR_NULL(limiter))
		return -ENOMEM;  /* Do not print prompts after kzalloc errors */

	limiter->pdev = pdev;
	limiter->dev = &pdev->dev;

	err = motor_limiter_fdt_parse(limiter, pdev->dev.of_node);
	if (err < 0) {
		dev_err(limiter->dev, "failed to get fdt resource\n");
		return err;
	}

	platform_set_drvdata(pdev, limiter);

	err = device_create_file(limiter->dev, &motor_limiter_attr);
	if (err < 0) {
		dev_err(limiter->dev, "failed to create device file motor_limiter\n");
		return err;
	}

	dev_info(limiter->dev, "probe success\n");
	return 0;
}

static int motor_limiter_remove(struct platform_device *pdev)
{
	struct motor_limiter *limiter = (struct motor_limiter *)platform_get_drvdata(pdev);

	device_remove_file(limiter->dev, &motor_limiter_attr);

	gpio_free(limiter->gpio);

	return 0;
}

static const struct of_device_id of_motor_limiter_match[] = {
	{ .compatible = "motor-limiter", },
	{},
};

static struct platform_driver motor_limiter_driver = {
	.probe = motor_limiter_probe,
	.remove = motor_limiter_remove,
	.driver = {
		.name = "motor-limiter",
		.of_match_table = of_motor_limiter_match,
	},
};

module_platform_driver(motor_limiter_driver);

MODULE_AUTHOR("JingyanLiang <jingyanliang@allwinnertech.com>");
MODULE_DESCRIPTION("Motor Limiter Driver");
MODULE_VERSION("1.0.0");
MODULE_LICENSE("GPL");
