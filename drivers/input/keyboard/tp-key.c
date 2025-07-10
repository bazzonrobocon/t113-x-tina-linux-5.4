// SPDX-License-Identifier: SimPL-2.0
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Copyright (c) 2014
 *
 * ChangeLog
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/keyboard.h>
#include <linux/ioport.h>
#include <asm/irq.h>
#include <linux/io.h>
#include <linux/timer.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/pm.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/reset.h>
#include <linux/clk/sunxi.h>
#include <linux/kthread.h>

#define REG_TP_KEY_CTL0			0X00
#define REG_TP_KEY_CTL1			0X04
#define REG_TP_KEY_CTL2			0X08
#define REG_TP_KEY_CTL3			0X0C
#define REG_TP_KEY_INT_CTL		0X10
#define REG_TP_KEY_INT_STS		0X14
#define REG_TP_KEY_COM_DAT		0X1C
#define REG_TP_KEY_ADC_DAT		0X24

#define POLL_TIMEOUT	(0xffffff)

#define TP_KEY_CHANNEL_CNT	1		/* the min key_channel is 1 */
#define TP_KEY_CHANNEL_LAST		(TP_KEY_CHANNEL_CNT - 1)

/* set the initial value from key_channel to TP_KEY_CHANNEL_INITIAL_VALUE
 * and do not set it to be greater than or equal to 0
 */
#define TP_KEY_CHANNEL_INITIAL_VALUE	-1		//Do not set it to be greater than or equal to 0
int prev_key_value[TP_KEY_CHANNEL_CNT] = {TP_KEY_CHANNEL_INITIAL_VALUE};

/* voltage range 0~2.3v, unit is uv */
#define VOL_RANGE				(1800000UL)

struct sunxi_tpadc {
	struct platform_device	*pdev;
	struct input_dev *input_dev;
	struct device *dev;
	void __iomem	*reg_base;
	int				channel_compare_lowdata;
	int				channel_compare_higdata;
	int				key_channel_config;
	int				key_num;
	unsigned int	*key_val;
	int				*key_code;
	int				*key_channel;
	int				task_flag;			/* the task need stop when set this value to 1 */
	int				key_select;			/* the down key get its actual value after key_select times */
	int				key_state_for_down;	/* the key down when set this value to 1 */
	struct task_struct *task;
	struct clk *bus_clk;
	struct clk *mod_clk;
	struct reset_control *reset;
};

struct sunxi_tpadc *sunxi_tpadc;
static int tp_key_clear_fifo(void __iomem *reg_base);

static ssize_t
tpadc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u32 data, vol_data, reg_val;

	reg_val = readl((void __iomem *)
		(sunxi_tpadc->reg_base + REG_TP_KEY_INT_STS));
	data = readl((void __iomem *)
		(sunxi_tpadc->reg_base + REG_TP_KEY_ADC_DAT));
	writel(reg_val, (void __iomem *)
		(sunxi_tpadc->reg_base + REG_TP_KEY_INT_STS));

	data = ((VOL_RANGE / 4096) * data);	/* 12bits sample rate */
	vol_data = data / 1000;				/* data to val_data */

	return sprintf(buf, "the key vol_data is %d\n", vol_data);
}

static ssize_t
tpadc_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	return 0;
}

static DEVICE_ATTR(tpadc, 0444, tpadc_show, tpadc_store);

static int sunxi_tp_clk_enable(struct sunxi_tpadc *ts)
{
	if (ts->reset) {
		reset_control_deassert(ts->reset);
	}

	if (ts->mod_clk) {
		clk_prepare_enable(ts->mod_clk);
	}

	if (ts->bus_clk) {
		clk_prepare_enable(ts->bus_clk);
	}

	return 0;
}

static int sunxi_tp_clk_disable(struct sunxi_tpadc *ts)
{
	if (ts->reset) {
		reset_control_deassert(ts->reset);
	}

	if (ts->mod_clk) {
		clk_disable_unprepare(ts->mod_clk);
	}

	if (ts->bus_clk) {
		clk_disable_unprepare(ts->bus_clk);
	}

	return 0;
}

/* get the current vol_data and determine whether it is a valid key */
static int tp_key_get_vol(void *data)
{
	struct sunxi_tpadc *sunxi_tpadc = (struct sunxi_tpadc *)data;
	u32  reg_val = 0, reg_data = 0, vol_data = 0, i = 0;
	static int key_channel_current;

	if (IS_ERR_OR_NULL(data)) {
		pr_err("%s:invalid tp data pointer\n", __func__);
		return -EINVAL;
	}

	reg_val = readl((void __iomem *)
		(sunxi_tpadc->reg_base + REG_TP_KEY_INT_STS));
	reg_data = readl((void __iomem *)
		(sunxi_tpadc->reg_base + REG_TP_KEY_ADC_DAT));
	writel(reg_val, (void __iomem *)
		(sunxi_tpadc->reg_base + REG_TP_KEY_INT_STS));

	/* 12bits sample rate,adc to vol_data,
	 * which is the voltage before voltage division
	 */
	vol_data = ((VOL_RANGE / 4096) * reg_data) / 1000;

	for (i = 0; i < TP_KEY_CHANNEL_CNT; i++) {
		if (key_channel_current == TP_KEY_CHANNEL_LAST) {
			key_channel_current = 0;
			sunxi_tpadc->key_select = 0;
			tp_key_clear_fifo(sunxi_tpadc->reg_base);
			/* the vol_data of the valid key must less than lowdata and greater than highdata */
			if (vol_data > sunxi_tpadc->channel_compare_lowdata ||
				vol_data < sunxi_tpadc->channel_compare_higdata) {
				return 1;
			} else {
				return vol_data;
			}
		} else if (key_channel_current == i && TP_KEY_CHANNEL_CNT != 1) {
			key_channel_current++;
			sunxi_tpadc->key_select = TP_KEY_CHANNEL_LAST - i;
			if (vol_data > sunxi_tpadc->channel_compare_lowdata ||
				vol_data < sunxi_tpadc->channel_compare_higdata) {
				return 1;
			} else {
				return vol_data;
			}
		}
	}

	return vol_data;

}

/* determine the key from which channel */
static int tp_key_which_key(struct sunxi_tpadc *sunxi_tpadc, u32 key_data, u32 channel_flag)
{
	int i = 0;
	u32 vol_data = 0;

	if (!sunxi_tpadc) {
		pr_info("%s:invalid key data\n", __func__);
		return -EINVAL;
	}

	if (key_data == 1) {
		return -EINVAL;
	} else {
		vol_data = key_data;
	}

	for (i = 0; i < sunxi_tpadc->key_num; i++) {
		if ((vol_data < sunxi_tpadc->key_val[i]) &&
			(channel_flag == sunxi_tpadc->key_channel[i]))
			break;
	}

	if (i >= sunxi_tpadc->key_num) {
		return -EINVAL;
	}

	return i;
}

/* when a key is pressed, need to deal it */
static int tp_key_handle(void *dummy)
{
	struct sunxi_tpadc *sunxi_tpadc = (struct sunxi_tpadc *)dummy;
	u32  vol_data = 0;
	int index = 0, i = 0;
	static int key_down_flag, key_channel_current;

	if (IS_ERR_OR_NULL(dummy)) {
		pr_err("%s:invalid tp dummy pointer\n", __func__);
		return -EINVAL;
	}

	msleep(10);

	vol_data = tp_key_get_vol(dummy);
	index = tp_key_which_key(sunxi_tpadc, vol_data, key_channel_current);

	if (index < 0) {
		for (i = 0; i < TP_KEY_CHANNEL_CNT; i++) {
			if (prev_key_value[i] != TP_KEY_CHANNEL_INITIAL_VALUE) {
				break;
			}
		}
		if (i == TP_KEY_CHANNEL_CNT) {
			key_down_flag = 0;
			for (i = 0; i < TP_KEY_CHANNEL_CNT; i++) {
				if (key_channel_current == TP_KEY_CHANNEL_LAST) {
					key_channel_current = 0;
					tp_key_clear_fifo(sunxi_tpadc->reg_base);
					sunxi_tpadc->key_state_for_down = 0;
					return 0;
				} else if (key_channel_current == i && TP_KEY_CHANNEL_CNT != 1) {
					key_channel_current++;
					return 0;
				}
			}
		} else {
			for (i = 0; i < TP_KEY_CHANNEL_CNT; i++) {
				if (key_channel_current == TP_KEY_CHANNEL_LAST && TP_KEY_CHANNEL_CNT == 1) {
					goto deal_the_key;
				} else if (key_channel_current == TP_KEY_CHANNEL_LAST &&
							prev_key_value[TP_KEY_CHANNEL_LAST] == TP_KEY_CHANNEL_INITIAL_VALUE) {
					key_channel_current = 0;
					return 0;
				} else if (key_channel_current == i &&
							prev_key_value[i] == TP_KEY_CHANNEL_INITIAL_VALUE) {
					key_channel_current++;
					return 0;
				} else {
					goto deal_the_key;
				}
			}
		}
	}

deal_the_key:

	for (i = 0; i < TP_KEY_CHANNEL_CNT; i++) {
		if (key_channel_current == TP_KEY_CHANNEL_LAST) {
			if (key_down_flag && prev_key_value[TP_KEY_CHANNEL_LAST] > TP_KEY_CHANNEL_INITIAL_VALUE &&
				prev_key_value[TP_KEY_CHANNEL_LAST] != index) {
				pr_info("\n%d %s : the key %d up now !\n",
					__LINE__, __func__, prev_key_value[TP_KEY_CHANNEL_LAST]);
				/*if key down and current key is not equal to prev key, then the key up */
				input_report_key(sunxi_tpadc->input_dev,
								sunxi_tpadc->key_code[prev_key_value[TP_KEY_CHANNEL_LAST]], 0);
				/* report key up event */
				input_sync(sunxi_tpadc->input_dev);
				key_down_flag = 0;
				prev_key_value[TP_KEY_CHANNEL_LAST] = TP_KEY_CHANNEL_INITIAL_VALUE;
				key_channel_current = 0;
				tp_key_clear_fifo(sunxi_tpadc->reg_base);
				sunxi_tpadc->key_state_for_down = 0;
				return 0;
			}

			if (prev_key_value[TP_KEY_CHANNEL_LAST] == TP_KEY_CHANNEL_INITIAL_VALUE && index >= 0) {
				/* save the last key index */
				prev_key_value[TP_KEY_CHANNEL_LAST] = index;
			}

			if (prev_key_value[TP_KEY_CHANNEL_LAST] > TP_KEY_CHANNEL_INITIAL_VALUE && index >= 0 &&
				prev_key_value[TP_KEY_CHANNEL_LAST] != index) {
				pr_info("%d %s : fatal: must clear all flag!\n", __LINE__, __func__);
				/* it is an error and clear all */
				prev_key_value[TP_KEY_CHANNEL_LAST] = TP_KEY_CHANNEL_INITIAL_VALUE;
				if (key_down_flag) {
					input_report_key(sunxi_tpadc->input_dev, sunxi_tpadc->key_code[index], 0);
					input_sync(sunxi_tpadc->input_dev);
				}
				key_down_flag = 0;
				key_channel_current = 0;
				tp_key_clear_fifo(sunxi_tpadc->reg_base);
				sunxi_tpadc->key_state_for_down = 0;
				return 0;
			}
		} else if (key_channel_current == i && TP_KEY_CHANNEL_CNT != 1) {
			if (key_down_flag && prev_key_value[i] > TP_KEY_CHANNEL_INITIAL_VALUE
				&& prev_key_value[i] != index) {
				pr_info("\n%d %s : the key %d up now !\n", __LINE__, __func__, prev_key_value[i]);
				/*if key down and current key is not equal to prev key, then the key up */
				input_report_key(sunxi_tpadc->input_dev, sunxi_tpadc->key_code[prev_key_value[i]], 0);
				/* report key up event */
				input_sync(sunxi_tpadc->input_dev);
				key_down_flag = 0;
				prev_key_value[i] = TP_KEY_CHANNEL_INITIAL_VALUE;
				key_channel_current++;
				return 0;
			}

			if (prev_key_value[i] == TP_KEY_CHANNEL_INITIAL_VALUE && index >= 0) {
				/* save the i key index */
				prev_key_value[i] = index;
			}

			if (prev_key_value[i] > TP_KEY_CHANNEL_INITIAL_VALUE && index >= 0 &&
				prev_key_value[i] != index) {
				pr_info("%d %s : fatal: must clear all flag!\n", __LINE__, __func__);
				/* it is an error and clear all */
				prev_key_value[i] = TP_KEY_CHANNEL_INITIAL_VALUE;
				if (key_down_flag) {
					input_report_key(sunxi_tpadc->input_dev, sunxi_tpadc->key_code[index], 0);
					input_sync(sunxi_tpadc->input_dev);
				}
				key_down_flag = 0;
				key_channel_current++;
				return 0;
			}
		}
	}

	/* if the key has been continuously pressed */
	if (index >= 0) {
		if (key_down_flag) {
			for (i = 0; i < TP_KEY_CHANNEL_CNT; i++) {
				if (key_channel_current == TP_KEY_CHANNEL_LAST) {
					key_channel_current = 0;
				} else if (key_channel_current == i && TP_KEY_CHANNEL_CNT != 1) {
					key_channel_current++;
					break;
				}
			}
			return 0;
		}

	}

	for (i = 0; i < TP_KEY_CHANNEL_CNT; i++) {
		if (key_channel_current == TP_KEY_CHANNEL_LAST) {
			if (!key_down_flag && prev_key_value[TP_KEY_CHANNEL_LAST] == index) {
				pr_info("\n%d %s : the key %d down now !\n", __LINE__, __func__, index);
				/* if the current key equal prev key index, then the key down */
				input_report_key(sunxi_tpadc->input_dev,
								sunxi_tpadc->key_code[prev_key_value[TP_KEY_CHANNEL_LAST]], 1);
				/* report key down event */
				input_sync(sunxi_tpadc->input_dev);
				key_down_flag = 1;
				key_channel_current = 0;
			}
		} else if (key_channel_current == i && TP_KEY_CHANNEL_CNT != 1) {
			if (!key_down_flag && prev_key_value[i] == index) {
				pr_info("\n%d %s : the key %d down now !\n", __LINE__, __func__, index);
				/* if the current key equal prev key index, then the key down */
				input_report_key(sunxi_tpadc->input_dev, sunxi_tpadc->key_code[prev_key_value[i]], 1);
				/* report key down event */
				input_sync(sunxi_tpadc->input_dev);
				key_down_flag = 1;
				key_channel_current++;
				break;
			}
		}
	}

	return 0;
}

static int tp_key_dts_parse(struct device *pdev, struct sunxi_tpadc *sunxi_tpadc)
{
	struct device_node *np = pdev->of_node;
	u32 val[3] = {0, 0, 0};
	int i = 0, ret = 0;
	char key_name[16];

	if (IS_ERR_OR_NULL(pdev)) {
		pr_err("%s:invalid device pointer\n", __func__);
		ret = -EINVAL;
	}

	if (IS_ERR_OR_NULL(sunxi_tpadc)) {
		pr_err("%s:invalid tp sunxi_tpadc data pointer\n", __func__);
		ret = -EINVAL;
	}

	if (!of_device_is_available(np)) {
		pr_err("%s:invalid device node\n", __func__);
		ret = -EPERM;
	}

	sunxi_tpadc->reg_base = of_iomap(np, 0);
	if (IS_ERR_OR_NULL(sunxi_tpadc->reg_base)) {
		pr_err("%s:Failed to iomap io memory region\n", __func__);
		ret = -EBUSY;
		goto err0;
	} else {
		pr_info("key base: %p\n", sunxi_tpadc->reg_base);
	}

	sunxi_tpadc->reset = devm_reset_control_get(sunxi_tpadc->dev, NULL);
	if (sunxi_tpadc->reset) {
		reset_control_assert(sunxi_tpadc->reset);
		reset_control_deassert(sunxi_tpadc->reset);
	} else {
		pr_err("get tpadc reset failed\n");
		goto err1;
	}

	sunxi_tpadc->mod_clk = devm_clk_get(sunxi_tpadc->dev, "mod");
	if (sunxi_tpadc->mod_clk) {
		clk_prepare_enable(sunxi_tpadc->mod_clk);
	} else {
		pr_err("get tpadc mode clock failed.\n");
		goto err1;
	}

	sunxi_tpadc->bus_clk = devm_clk_get(sunxi_tpadc->dev, "bus");
	if (sunxi_tpadc->bus_clk) {
		clk_prepare_enable(sunxi_tpadc->bus_clk);
	} else {
		pr_err("get tpadc bus clock failed\n");
		goto err1;
	}

	if (of_property_read_u32(np, "channel_compare_lowdata",
		&sunxi_tpadc->channel_compare_lowdata)) {
		pr_err("%s: get key channel_compare_lowdata failed\n", __func__);
		ret = -ENODEV;
		goto err1;
	}

	if (of_property_read_u32(np, "channel_compare_higdata",
		&sunxi_tpadc->channel_compare_higdata)) {
		pr_err("%s: get key channel_compare_higdata failed\n", __func__);
		ret = -ENODEV;
		goto err1;
	}

	if (of_property_read_u32(np, "key_channel_config",
		&sunxi_tpadc->key_channel_config)) {
		pr_err("%s: get key key_channel_config failed\n", __func__);
		ret = -ENODEV;
		goto err1;
	}

	if (of_property_read_u32(np, "key_cnt", &sunxi_tpadc->key_num)) {
		pr_err("%s: get key count failed\n", __func__);
		ret = -ENODEV;
		goto err1;
	}

	if (sunxi_tpadc->key_num <= 0) {
		pr_err("key num is not right\n");
		ret = -ENODEV;
		goto err1;
	}

	sunxi_tpadc->key_code = (int *)devm_kzalloc(pdev,
		sizeof(int) * sunxi_tpadc->key_num, GFP_KERNEL);
	if (IS_ERR_OR_NULL(sunxi_tpadc->key_code)) {
		pr_err("sunxi_tpadc: not enough memory for key code array\n");
		ret = -ENOMEM;
		goto err1;
	}

	sunxi_tpadc->key_val = (int *)devm_kzalloc(pdev,
		sizeof(int) * sunxi_tpadc->key_num, GFP_KERNEL);
	if (IS_ERR_OR_NULL(sunxi_tpadc->key_val)) {
		pr_err("sunxi_tpadc: not enough memory for key adc value array\n");
		ret = -ENOMEM;
		goto err2;
	}

	sunxi_tpadc->key_channel = (int *)devm_kzalloc(pdev,
		sizeof(int) * sunxi_tpadc->key_num, GFP_KERNEL);
	if (IS_ERR_OR_NULL(sunxi_tpadc->key_channel)) {
		pr_err("sunxi_tpadc: not enough memory for key adc hannel array\n");
		ret = -ENOMEM;
		goto err3;
	}

	for (i = 1; i <= sunxi_tpadc->key_num; i++) {
		sprintf(key_name, "key%d", i);
		if (of_property_read_u32_array(np, key_name, val, ARRAY_SIZE(val))) {
			pr_err("%s: get %s err!\n", __func__, key_name);
			ret = -EBUSY;
			goto err4;
		}
		sunxi_tpadc->key_val[i - 1] = val[0];
		sunxi_tpadc->key_code[i - 1] = val[1];
		sunxi_tpadc->key_channel[i - 1] = val[2];
		pr_info("key adc value:%d key code:%d key channel:%d\n",
			sunxi_tpadc->key_val[i - 1], sunxi_tpadc->key_code[i - 1],
			sunxi_tpadc->key_channel[i - 1]);
	}

	return 0;

err4:
	kfree(sunxi_tpadc->key_channel);
err3:
	kfree(sunxi_tpadc->key_val);
err2:
	kfree(sunxi_tpadc->key_code);
err1:
	iounmap(sunxi_tpadc->reg_base);
err0:
	kfree(sunxi_tpadc);

	return ret;
}

static int tp_key_clear_fifo(void __iomem *reg_base)
{
	u32 val = 0;

	val = readl((void __iomem *)
		(reg_base + REG_TP_KEY_INT_CTL));
	val |= 1 << 4;	/* clear fifo */
	writel(val, (void __iomem *)
		(reg_base + REG_TP_KEY_INT_CTL));

	return 0;
}

static int tp_key_channel_select(struct sunxi_tpadc *sunxi_tpadc)
{
	u32 val = 0;

	if (IS_ERR_OR_NULL(sunxi_tpadc)) {
		pr_err("%s:invalid tp sunxi_tpadc data pointer\n", __func__);
		return -EINVAL;
	}

	val = readl((void __iomem *)
		(sunxi_tpadc->reg_base + REG_TP_KEY_CTL1));
	val &= ~(0x0f);
	val |= sunxi_tpadc->key_channel_config;	/* channel select */
	writel(val, (void __iomem *)
		(sunxi_tpadc->reg_base + REG_TP_KEY_CTL1));

	return 0;
}

static int tp_key_channel_deselect(struct sunxi_tpadc *sunxi_tpadc)
{
	u32 val = 0;

	if (IS_ERR_OR_NULL(sunxi_tpadc)) {
		pr_err("%s:invalid tp sunxi_tpadc data pointer\n", __func__);
		return -EINVAL;
	}

	val = readl((void __iomem *)
		(sunxi_tpadc->reg_base + REG_TP_KEY_CTL1));
	val &= ~(sunxi_tpadc->key_channel_config);	/* channel deselect */
	writel(val, (void __iomem *)
		(sunxi_tpadc->reg_base + REG_TP_KEY_CTL1));

	return 0;
}

static int tp_key_hw_init(struct sunxi_tpadc *sunxi_tpadc)
{
	u32 val = 0;

	if (IS_ERR_OR_NULL(sunxi_tpadc)) {
		pr_err("%s:invalid tp sunxi_tpadc data pointer\n", __func__);
		return -EINVAL;
	}

	val = 0XF << 24;
	val |= 1 << 23;
	val &= ~(1 << 22);	/*sellect HOSC(24MHz)*/
	val |= 0x3 << 20;	/*00:CLK_IN/2,01:CLK_IN/3,10:CLK_IN/6,11:CLK_IN/1*/
	val |= 0x7f << 0;	/*FS DIV*/
	writel(val, (void __iomem *)
		(sunxi_tpadc->reg_base + REG_TP_KEY_CTL0));

	val = 1 << 4 | 1 << 5; /* select adc mode and enable tp */
	writel(val, (void __iomem *)
		(sunxi_tpadc->reg_base + REG_TP_KEY_CTL1));

	tp_key_channel_select(sunxi_tpadc);

	return 0;
}

static int tp_key_hw_exit(struct sunxi_tpadc *sunxi_tpadc)
{
	u32 val = 0;

	if (IS_ERR_OR_NULL(sunxi_tpadc)) {
		pr_err("%s:invalid tp sunxi_tpadc data pointer\n", __func__);
		return -EINVAL;
	}

	val = readl((void __iomem *)
		(sunxi_tpadc->reg_base + REG_TP_KEY_CTL1));
	val &= ~(1 << 5); /* disable tp */
	writel(val, (void __iomem *)
		(sunxi_tpadc->reg_base + REG_TP_KEY_CTL1));

	tp_key_channel_deselect(sunxi_tpadc);

	return 0;
}

static int sunxi_tp_key_task(void *data)
{
	u32 poll_time = POLL_TIMEOUT, ret = -1;
	int key_short_cnt;
	struct sunxi_tpadc *sunxi_tpadc = (struct sunxi_tpadc *)data;

	if (IS_ERR_OR_NULL(data)) {
		pr_err("%s:invalid tp data pointer\n", __func__);
		ret = -EINVAL;
	}

	tp_key_clear_fifo(sunxi_tpadc->reg_base);

	msleep(500);

	while (!kthread_should_stop()) {
		while ((tp_key_get_vol(data) == 1) && (--poll_time > 0) &&
			(sunxi_tpadc->task_flag == 0)) {
			msleep(10);
		}

		if (sunxi_tpadc->task_flag == 1) {
			pr_info("%d %s:tp_key stop the kthread now !\n", __LINE__, __func__);
			do_exit(0);
			if (IS_ERR(sunxi_tpadc->task)) {
				ret = PTR_ERR(sunxi_tpadc->task);
				sunxi_tpadc->task = NULL;
			}
		}

		if (poll_time <= 0) {
			pr_info("%d %s:tp_key get effective key time out\n", __LINE__, __func__);
			continue;
		} else {
			sunxi_tpadc->key_state_for_down = 1;
		}

		for (key_short_cnt = sunxi_tpadc->key_select; key_short_cnt > 0; --key_short_cnt) {
			ret = tp_key_get_vol(data);
		}

		while (sunxi_tpadc->key_state_for_down == 1) {
			ret = tp_key_handle(data);
		}

	}

	return 0;
}

static int tp_key_probe(struct platform_device *pdev)
{
	int i;
	int ret = -1;
	struct input_dev *input_dev = NULL;

	pr_info("%s:tp_key probe begin\n", __func__);

	sunxi_tpadc = (struct sunxi_tpadc *)devm_kzalloc(&pdev->dev,
			sizeof(struct sunxi_tpadc), GFP_KERNEL);
	if (IS_ERR_OR_NULL(sunxi_tpadc)) {
		pr_info("sunxi_tpadc: not enough memory for key data\n");
		return -ENOMEM;
	}

	sunxi_tpadc->pdev = pdev;
	sunxi_tpadc->dev = &pdev->dev;

	ret = tp_key_dts_parse(&pdev->dev, sunxi_tpadc);
	if (ret != 0) {
		goto err0;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		pr_info("%s:allocate input device fail\n", __func__);
		ret = -ENOMEM;
		goto err1;
	}

	input_dev->name = "sunxi-tpadc";
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0100;

	__set_bit(EV_REP, input_dev->evbit); /* support key repeat */
	__set_bit(EV_KEY, input_dev->evbit); /* support key repeat */

	for (i = 0; i < sunxi_tpadc->key_num; i++) {
		__set_bit(sunxi_tpadc->key_code[i], input_dev->keybit);
	}

	sunxi_tpadc->input_dev = input_dev;

	/* hardware setting */
	if (sunxi_tp_clk_enable(sunxi_tpadc)) {
		pr_err("%s:request tpadc clk failed\n", __func__);
		ret = -EPROBE_DEFER;
		goto err2;
	}

	tp_key_hw_init(sunxi_tpadc);

	platform_set_drvdata(pdev, sunxi_tpadc);

	dev_set_drvdata(&sunxi_tpadc->input_dev->dev, sunxi_tpadc);

	sunxi_tpadc->task = kthread_run(sunxi_tp_key_task, sunxi_tpadc, "sunxi-tp-key");
	if (IS_ERR(sunxi_tpadc->task)) {
		pr_err("%s:unable to start kernel thread sunxi_tpadc\n", __func__);
		ret = PTR_ERR(sunxi_tpadc->task);
		sunxi_tpadc->task = NULL;
		ret = -EINVAL;
		goto err3;
	}

	ret = input_register_device(sunxi_tpadc->input_dev);
	if (ret) {
		pr_err("%s:register input device error\n", __func__);
		goto err4;
	}

	ret = device_create_file(&sunxi_tpadc->input_dev->dev, &dev_attr_tpadc);
	if (ret) {
		pr_err("%s: couldn't create device file for status\n", __func__);
		goto err5;
	}

	return 0;

err5:
	input_unregister_device(sunxi_tpadc->input_dev);
err4:
	sunxi_tpadc->task_flag = 1;
	if (IS_ERR(sunxi_tpadc->task)) {
		ret = PTR_ERR(sunxi_tpadc->task);
		sunxi_tpadc->task = NULL;
	}
err3:
	platform_set_drvdata(pdev, NULL);
	tp_key_hw_exit(sunxi_tpadc);
	sunxi_tp_clk_disable(sunxi_tpadc);
err2:
	input_free_device(sunxi_tpadc->input_dev);
err1:
	kfree(sunxi_tpadc->key_channel);
	kfree(sunxi_tpadc->key_val);
	kfree(sunxi_tpadc->key_code);
	iounmap(sunxi_tpadc->reg_base);
err0:
	kfree(sunxi_tpadc);

	return ret;
}

static int tp_key_remove(struct platform_device *pdev)
{
	struct sunxi_tpadc *sunxi_tpadc = platform_get_drvdata(pdev);

	device_remove_file(&sunxi_tpadc->input_dev->dev, &dev_attr_tpadc);

	input_unregister_device(sunxi_tpadc->input_dev);

	sunxi_tpadc->task_flag = 1;

	platform_set_drvdata(pdev, NULL);

	tp_key_hw_exit(sunxi_tpadc);

	sunxi_tp_clk_disable(sunxi_tpadc);

	input_free_device(sunxi_tpadc->input_dev);

	kfree(sunxi_tpadc->key_channel);
	kfree(sunxi_tpadc->key_val);
	kfree(sunxi_tpadc->key_code);
	iounmap(sunxi_tpadc->reg_base);
	kfree(sunxi_tpadc);

	pr_err("%d %s: remove device success\n", __LINE__, __func__);
	return 0;
}

static struct of_device_id tp_key_of_match[] = {
	{ .compatible = "allwinner,tp_key",},
	{ },
};
MODULE_DEVICE_TABLE(of, tp_key_of_match);

static struct platform_driver tp_key_driver = {
	.probe  = tp_key_probe,
	.remove = tp_key_remove,
	.driver = {
		.name   = "sunxi-tpadc",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(tp_key_of_match),
	},
};

static int __init sunxi_tp_key_init(void)
{
	pr_info("insmod the tp-key.ko now !\n");

	return platform_driver_register(&tp_key_driver);
}

static void __exit sunxi_tp_key_exit(void)
{
	pr_info("rmmod the tp-key.ko now !\n");

	platform_driver_unregister(&tp_key_driver);
}

module_init(sunxi_tp_key_init);
module_exit(sunxi_tp_key_exit);

MODULE_AUTHOR("Edwin");
MODULE_DESCRIPTION("sunxi-tpadc AW1859 driver");
MODULE_LICENSE("GPL");
