// SPDX-License-Identifier: GPL-2.0-only
/*
 * drivers/misc/gpio-motor/motor-control.c
 *
 * Copyright (C) 2022 Allwinner.
 * Jingyan Liang <jingyanliang@allwinnertech.com>
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/gpio/consumer.h>
#include <linux/of_platform.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

enum motor_dir {
	MITOR_DIR_UNKNOWN = 0,
	MITOR_DIR_CW,
	MITOR_DIR_CCW,
};

struct motor_workdata {
	struct list_head entry;
	enum motor_dir dir;
	int cycle;
};

struct motor_control {
	struct platform_device *pdev;
	struct device *dev;

	int phase_num;
	int *phase_gpios;
	int step_num;
	u8 *cw_table;
	u8 *ccw_table;
	int phase_udelay;
	int step_mdelay;

	struct workqueue_struct *workqueue;
	struct delayed_work delaywork;
	struct list_head data_list;
	spinlock_t lock;
};

static void motor_set_phase(struct device *dev, int *gpios, int num, int phases, int delay)
{
	int i;
	int value;

	for (i = 0; i < num; i++) {
		value = (phases >> i) & 0x01;
		dev_dbg(dev, "set gpio %d value %d\n", gpios[i], value);
		gpio_set_value(gpios[i], value);
		udelay(delay);
	}
}

static void motor_set_stop(struct device *dev, int *gpios, int num)
{
	int i;

	for (i = 0; i < num; i++)
		gpio_set_value(gpios[i], 0);
}

static int motor_run_mstep(struct motor_control *motor, struct motor_workdata *data)
{
	int i, j;
	char *phase_table = NULL;

	switch (data->dir) {
	case MITOR_DIR_CW:
		phase_table = motor->cw_table;
		break;
	case MITOR_DIR_CCW:
		phase_table = motor->ccw_table;
		break;
	default:
		dev_err(motor->dev, "motor run step dir_%d error\n", data->dir);
		return -EINVAL;
	}

	for (i = 0; i < data->cycle; i++) {
		for (j = 0; j < motor->step_num; j++) {
			dev_dbg(motor->dev, "cycle_%d set motor phase 0x%x\n", i, phase_table[j]);
			motor_set_phase(motor->dev, motor->phase_gpios, \
							motor->phase_num, phase_table[j], motor->phase_udelay);
			mdelay(motor->step_mdelay);
		}
	}
	motor_set_stop(motor->dev, motor->phase_gpios, motor->phase_num);

	return 0;
}

static void motor_mstep_work_handler(struct work_struct *work)
{
	struct motor_control *motor = container_of(work, struct motor_control, delaywork.work);
	struct motor_workdata *data;

	dev_dbg(motor->dev, "%s enter\n", __FUNCTION__);

	if (motor == NULL)
		return;

	if (!list_empty(&motor->data_list)) {
		data = list_entry(motor->data_list.next, struct motor_workdata, entry);
		dev_dbg(motor->dev, "handler get workdata dir_%d cycle_%d\n", data->dir, data->cycle);

		motor_run_mstep(motor, data);

		spin_lock(&motor->lock);
		list_del(&data->entry);
		spin_unlock(&motor->lock);

		devm_kfree(motor->dev, data);

		if (!list_empty(&motor->data_list))
			queue_delayed_work(motor->workqueue, &motor->delaywork, msecs_to_jiffies(1));
	}
}

static ssize_t motor_ctrl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE,
		"show input help:\n"
		"\techo dir > motor_ctrl\n"
		"\t-----dir: motor direction (1-cw, 2-ccw, other-unknown)\n\n"
		"\techo dir,cycle > motor_ctrl\n"
		"\t-----dir: motor direction (1-cw, 2-ccw, other-unknown)\n"
		"\t---cycle: motor step loop cycle\n"
		);
}

static ssize_t motor_ctrl_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct motor_control *motor = (struct motor_control *)platform_get_drvdata(pdev);

	int err = 0;
	char *ptr = NULL;
	char *p[2];
	int cycle = 1;
	enum motor_dir dir = MITOR_DIR_UNKNOWN;
	struct motor_workdata *data;

	ptr = (char *)buf;
	if (strchr(ptr, ',') == NULL) {
		err = kstrtouint(ptr, 10, &dir);
		if (err) {
			dev_err(motor->dev, "failed to parse str %s\n", ptr);
			dir = MITOR_DIR_UNKNOWN;
		}
	} else {
		p[0] = strsep(&ptr, ",");
		p[1] = strsep(&ptr, ",");

		err = kstrtouint(p[0], 10, &dir);
		if (err) {
			dev_err(motor->dev, "failed to parse str %s\n", ptr);
			dir = MITOR_DIR_UNKNOWN;
		}
		err = kstrtouint(p[1], 10, &cycle);
		if (err) {
			dev_err(motor->dev, "failed to parse str %s\n", ptr);
			cycle = 0;
		}
	}

	data = devm_kzalloc(motor->dev, sizeof(*data), GFP_KERNEL);
	INIT_LIST_HEAD(&data->entry);
	data->dir = dir & 0xff;
	data->cycle = cycle;
	spin_lock(&motor->lock);
	list_add_tail(&data->entry, &motor->data_list);
	spin_unlock(&motor->lock);
	dev_dbg(motor->dev, "motor dir_%d cycle_%d\n", data->dir, data->cycle);

	queue_delayed_work(motor->workqueue, &motor->delaywork, msecs_to_jiffies(1));

	return count;
}

static struct device_attribute motor_ctrl_attr = {
	.attr = {
		.name = "motor_ctrl",
		.mode = 0664,
	},
	.show = motor_ctrl_show,
	.store = motor_ctrl_store,
};

static int motor_control_fdt_parse(struct motor_control *motor, struct device_node *np)
{
	int err;
	int i;
	char motor_gpio_name[32] = {0};
	char str[256];
	char *ptr = NULL;

	err = of_property_read_u32(np, "motor-phase-num", &motor->phase_num);
	if (err) {
		dev_err(motor->dev, "failed to get gpio num\n");
		return -EINVAL;
	}

	err = of_property_read_u32(np, "motor-step-num", &motor->step_num);
	if (err) {
		dev_err(motor->dev, "failed to get phase num\n");
		return -EINVAL;
	}

	err = of_property_read_u32(np, "motor-phase-udelay", &motor->phase_udelay);
	if (err) {
		dev_err(motor->dev, "failed to get phase udelay time\n");
		return -EINVAL;
	}

	err = of_property_read_u32(np, "motor-step-mdelay", &motor->step_mdelay);
	if (err) {
		dev_err(motor->dev, "failed to get step mdelay time\n");
		return -EINVAL;
	}

	motor->phase_gpios = devm_kzalloc(motor->dev, sizeof(int) * motor->phase_num, GFP_KERNEL);
	if (IS_ERR_OR_NULL(motor->phase_gpios)) {
		dev_err(motor->dev, "failed to alloc motor phase_gpios mem\n");
		return -ENOMEM;
	}

	motor->cw_table = devm_kzalloc(motor->dev, motor->step_num, GFP_KERNEL);
	if (IS_ERR_OR_NULL(motor->cw_table)) {
		dev_err(motor->dev, "failed to alloc motor cw_table mem\n");
		return -ENOMEM;
	}

	motor->ccw_table = devm_kzalloc(motor->dev, motor->step_num, GFP_KERNEL);
	if (IS_ERR_OR_NULL(motor->ccw_table)) {
		dev_err(motor->dev, "failed to alloc motor ccw_table mem\n");
		return -ENOMEM;
	}

	for (i = 0; i < motor->phase_num; i++) {
		sprintf(motor_gpio_name, "motor-gpio-phase%d", i);
		motor->phase_gpios[i] = of_get_named_gpio_flags(np, motor_gpio_name, 0, NULL);
		if (motor->phase_gpios[i] < 0) {
			dev_err(motor->dev, "failed to get property %s\n", motor_gpio_name);
			return -EINVAL;
		} else {
			err = gpio_request(motor->phase_gpios[i], motor_gpio_name);
			if (err < 0) {
				dev_err(motor->dev, "failed to request gpio_%d %s\n", \
									motor->phase_gpios[i], motor_gpio_name);
				return err;
			} else {
				gpio_direction_output(motor->phase_gpios[i], 0);
				dev_dbg(motor->dev, "gpio_%d %s output 0", \
									motor->phase_gpios[i], motor_gpio_name);
			}
		}
	}

	err = of_property_read_u8_array(np, "motor-cw-table", motor->cw_table, motor->step_num);
	if (err < 0)
		dev_err(motor->dev, "failed to get motor-cw-table size %d\n", motor->step_num);

	err = of_property_read_u8_array(np, "motor-ccw-table", motor->ccw_table, motor->step_num);
	if (err < 0)
		dev_err(motor->dev, "failed to get motor-ccw-table index %d\n", motor->step_num);

	dev_info(motor->dev, "motor-phase-num %d\n", motor->phase_num);
	dev_info(motor->dev, "motor-step-num %d\n", motor->step_num);

	memset(str, 0, sizeof(str));
	ptr = str;
	ptr += sprintf(ptr, "motor-cw-table < ");
	for (i = 0; i < motor->step_num; i++)
		ptr += sprintf(ptr, "0x%02x ", motor->cw_table[i]);
	sprintf(ptr, ">");
	dev_info(motor->dev, "%s\n", str);

	memset(str, 0, sizeof(str));
	ptr = str;
	ptr += sprintf(ptr, "motor-ccw-table < ");
	for (i = 0; i < motor->step_num; i++)
		ptr += sprintf(ptr, "0x%02x ", motor->ccw_table[i]);
	sprintf(ptr, ">");
	dev_info(motor->dev, "%s\n", str);

	return 0;
}

static int motor_control_probe(struct platform_device *pdev)
{
	struct motor_control *motor = NULL;
	int err;

	motor = devm_kzalloc(&pdev->dev, sizeof(*motor), GFP_KERNEL);
	if (IS_ERR_OR_NULL(motor))
		return -ENOMEM;  /* Do not print prompts after kzalloc errors */

	motor->pdev = pdev;
	motor->dev = &pdev->dev;

	err = motor_control_fdt_parse(motor, pdev->dev.of_node);
	if (err < 0) {
		dev_err(motor->dev, "failed to get fdt resource\n");
		return err;
	}

	spin_lock_init(&motor->lock);

	INIT_LIST_HEAD(&motor->data_list);

	motor->workqueue = create_singlethread_workqueue(dev_name(motor->dev));
	INIT_DELAYED_WORK(&motor->delaywork, motor_mstep_work_handler);

	platform_set_drvdata(pdev, motor);

	err = device_create_file(motor->dev, &motor_ctrl_attr);
	if (err < 0) {
		dev_err(motor->dev, "failed to create device file motor_ctrl\n");
		return err;
	}

	dev_info(motor->dev, "probe success\n");
	return 0;
}

static int motor_control_remove(struct platform_device *pdev)
{
	struct motor_control *motor = (struct motor_control *)platform_get_drvdata(pdev);
	int i;

	device_remove_file(motor->dev, &motor_ctrl_attr);

	cancel_delayed_work(&motor->delaywork);
	destroy_workqueue(motor->workqueue);

	for (i = 0; i < motor->phase_num; i++)
		gpio_free(motor->phase_gpios[i]);

	return 0;
}

static const struct of_device_id of_motor_control_match[] = {
	{ .compatible = "motor-control", },
	{},
};

static struct platform_driver motor_control_driver = {
	.probe = motor_control_probe,
	.remove = motor_control_remove,
	.driver = {
		.name = "motor-control",
		.of_match_table = of_motor_control_match,
	},
};

module_platform_driver(motor_control_driver);

MODULE_AUTHOR("JingyanLiang <jingyanliang@allwinnertech.com>");
MODULE_DESCRIPTION("Motor Control Driver");
MODULE_VERSION("1.0.0");
MODULE_LICENSE("GPL");
