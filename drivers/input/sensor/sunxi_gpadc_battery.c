/* SPDX-License-Identifier: GPL-2.0-only
 * Based on drivers/input/sensor/sunxi_gpadc_battery.c
 *
 * Copyright (C) 2022 Allwinner.
 * zhengweilong <zhengweilong@allwinnertech.com>
 *
 * SUNXI GPADC BATTERY Controller Driver
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <linux/timer.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/pm.h>
#include <linux/pm_wakeirq.h>
#include <linux/pm_runtime.h>
#include <linux/workqueue.h>
#include "sunxi_gpadc_battery.h"
#include <linux/sched.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/reset.h>
#include <linux/clk/sunxi.h>

static u32 debug_mask = 1;
#define dprintk(level_mask, fmt, ...)                                          \
	do {                                                                   \
		if (unlikely(debug_mask & level_mask))                         \
			pr_info(fmt, ##__VA_ARGS__);                           \
	} while (0)

static struct sunxi_gpadc_battery *sunxi_gpadc_battery;
u8 channel;
void __iomem *vaddr;

static inline void
sunxi_gpadc_battery_save_regs(struct sunxi_gpadc_battery *sunxi_gpadc_battery)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(sunxi_gpadc_battery_regs_offset); i++)
		sunxi_gpadc_battery->regs_backup[i] =
			readl(sunxi_gpadc_battery->reg_base +
			      sunxi_gpadc_battery_regs_offset[i]);
}

static inline void sunxi_gpadc_battery_restore_regs(
	struct sunxi_gpadc_battery *sunxi_gpadc_battery)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(sunxi_gpadc_battery_regs_offset); i++)
		writel(sunxi_gpadc_battery->regs_backup[i],
		       sunxi_gpadc_battery->reg_base +
			       sunxi_gpadc_battery_regs_offset[i]);
}

u32 sunxi_gpadc_battery_sample_rate_read(void __iomem *reg_base, u32 clk_in)
{
	u32 div, round_clk;

	div = readl(reg_base + GP_SR_REG);
	div = div >> 16;
	round_clk = clk_in / (div + 1);
	//	round_clk = (div + 1) / clk_in;
	return round_clk;
}

/* clk_in: source clock, round_clk: sample rate */
void sunxi_gpadc_battery_sample_rate_set(void __iomem *reg_base, u32 clk_in,
					 u32 round_clk)
{
	u32 div, reg_val;

	if (round_clk > clk_in)
		pr_err("%s, invalid round clk!\n", __func__);
	div = clk_in / round_clk - 1;
	reg_val = readl(reg_base + GP_SR_REG);
	reg_val &= ~GP_SR_CON;
	reg_val |= (div << 16);
	writel(reg_val, reg_base + GP_SR_REG);
}

void sunxi_gpadc_battery_ctrl_set(void __iomem *reg_base, u32 ctrl_para)
{
	u32 reg_val;

	reg_val = readl(reg_base + GP_CTRL_REG);
	reg_val |= ctrl_para;
	writel(reg_val, reg_base + GP_CTRL_REG);
}

static u32 sunxi_gpadc_battery_ch_select(void __iomem *reg_base,
					 enum gp_channel_id id)
{
	u32 reg_val;

	reg_val = readl(reg_base + GP_CS_EN_REG);
	switch (id) {
	case GP_CH_0:
		reg_val |= GP_CH0_SELECT;
		break;
	case GP_CH_1:
		reg_val |= GP_CH1_SELECT;
		break;
	case GP_CH_2:
		reg_val |= GP_CH2_SELECT;
		break;
	case GP_CH_3:
		reg_val |= GP_CH3_SELECT;
		break;
	case GP_CH_4:
		reg_val |= GP_CH4_SELECT;
		break;
	case GP_CH_5:
		reg_val |= GP_CH5_SELECT;
		break;
	case GP_CH_6:
		reg_val |= GP_CH6_SELECT;
		break;
	case GP_CH_7:
		reg_val |= GP_CH7_SELECT;
		break;
	default:
		pr_err("%s, invalid channel id!", __func__);
		return -EINVAL;
	}
	writel(reg_val, reg_base + GP_CS_EN_REG);

	return 0;
}

static u32 sunxi_gpadc_battery_ch_deselect(void __iomem *reg_base,
					   enum gp_channel_id id)
{
	u32 reg_val;

	reg_val = readl(reg_base + GP_CS_EN_REG);
	switch (id) {
	case GP_CH_0:
		reg_val &= ~GP_CH0_SELECT;
		break;
	case GP_CH_1:
		reg_val &= ~GP_CH1_SELECT;
		break;
	case GP_CH_2:
		reg_val &= ~GP_CH2_SELECT;
		break;
	case GP_CH_3:
		reg_val &= ~GP_CH3_SELECT;
		break;
	case GP_CH_4:
		reg_val &= ~GP_CH4_SELECT;
		break;
	case GP_CH_5:
		reg_val &= ~GP_CH5_SELECT;
		break;
	case GP_CH_6:
		reg_val &= ~GP_CH6_SELECT;
		break;
	case GP_CH_7:
		reg_val &= ~GP_CH7_SELECT;
		break;
	default:
		pr_err("%s, invalid channel id!", __func__);
		return -EINVAL;
	}
	writel(reg_val, reg_base + GP_CS_EN_REG);

	return 0;
}

static u32 sunxi_gpadc_battery_cmp_select(void __iomem *reg_base,
					  enum gp_channel_id id)
{
	u32 reg_val;

	reg_val = readl(reg_base + GP_CS_EN_REG);
	switch (id) {
	case GP_CH_0:
		reg_val |= GP_CH0_CMP_EN;
		break;
	case GP_CH_1:
		reg_val |= GP_CH1_CMP_EN;
		break;
	case GP_CH_2:
		reg_val |= GP_CH2_CMP_EN;
		break;
	case GP_CH_3:
		reg_val |= GP_CH3_CMP_EN;
		break;
	case GP_CH_4:
		reg_val |= GP_CH4_CMP_EN;
		break;
	case GP_CH_5:
		reg_val |= GP_CH5_CMP_EN;
		break;
	case GP_CH_6:
		reg_val |= GP_CH6_CMP_EN;
		break;
	case GP_CH_7:
		reg_val |= GP_CH7_CMP_EN;
		break;
	default:
		pr_err("%s, invalid value!", __func__);
		return -EINVAL;
	}
	writel(reg_val, reg_base + GP_CS_EN_REG);

	return 0;
}

static u32 sunxi_gpadc_battery_cmp_deselect(void __iomem *reg_base,
					    enum gp_channel_id id)
{
	u32 reg_val;

	reg_val = readl(reg_base + GP_CS_EN_REG);
	switch (id) {
	case GP_CH_0:
		reg_val &= ~GP_CH0_CMP_EN;
		break;
	case GP_CH_1:
		reg_val &= ~GP_CH1_CMP_EN;
		break;
	case GP_CH_2:
		reg_val &= ~GP_CH2_CMP_EN;
		break;
	case GP_CH_3:
		reg_val &= ~GP_CH3_CMP_EN;
		break;
	case GP_CH_4:
		reg_val &= ~GP_CH4_CMP_EN;
		break;
	case GP_CH_5:
		reg_val &= ~GP_CH5_CMP_EN;
		break;
	case GP_CH_6:
		reg_val &= ~GP_CH6_CMP_EN;
		break;
	case GP_CH_7:
		reg_val &= ~GP_CH7_CMP_EN;
		break;

	default:
		pr_err("%s, invalid value!", __func__);
		return -EINVAL;
	}
	writel(reg_val, reg_base + GP_CS_EN_REG);

	return 0;
}

static u32 sunxi_gpadc_battery_mode_select(void __iomem *reg_base,
					   enum gp_select_mode mode)
{
	u32 reg_val;

	reg_val = readl(reg_base + GP_CTRL_REG);
	reg_val &= ~GP_MODE_SELECT;
	reg_val |= (mode << 18);
	writel(reg_val, reg_base + GP_CTRL_REG);

	return 0;
}

/* enable gpadc function, true:enable, false:disable */
static void sunxi_gpadc_battery_enable(void __iomem *reg_base, bool onoff)
{
	u32 reg_val = 0;

	reg_val = readl(reg_base + GP_CTRL_REG);
	if (true == onoff)
		reg_val |= GP_ADC_EN;
	else
		reg_val &= ~GP_ADC_EN;
	writel(reg_val, reg_base + GP_CTRL_REG);
}

static void sunxi_gpadc_battery_calibration_enable(void __iomem *reg_base)
{
	u32 reg_val;

	reg_val = readl(reg_base + GP_CTRL_REG);
	reg_val |= GP_CALIBRATION_ENABLE;
	writel(reg_val, reg_base + GP_CTRL_REG);
}

static u32 sunxi_gpadc_battery_channel_id(enum gp_channel_id id)
{
	u32 channel_id;

	switch (id) {
	case GP_CH_0:
		channel_id = CHANNEL_0_SELECT;
		break;
	case GP_CH_1:
		channel_id = CHANNEL_1_SELECT;
		break;
	case GP_CH_2:
		channel_id = CHANNEL_2_SELECT;
		break;
	case GP_CH_3:
		channel_id = CHANNEL_3_SELECT;
		break;
	case GP_CH_4:
		channel_id = CHANNEL_4_SELECT;
		break;
	case GP_CH_5:
		channel_id = CHANNEL_5_SELECT;
		break;
	case GP_CH_6:
		channel_id = CHANNEL_6_SELECT;
		break;
	case GP_CH_7:
		channel_id = CHANNEL_7_SELECT;
		break;
	default:
		pr_err("%s,channel id select fail", __func__);
		return -EINVAL;
	}

	return channel_id;
}

static void sunxi_gpadc_battery_enable_irq(void __iomem *reg_base)
{
	u32 reg_val;

	reg_val = readl(reg_base + GP_FIFO_INTC_REG);
	reg_val |= FIFO_DATA_IRQ_EN;
	writel(reg_val, reg_base + GP_FIFO_INTC_REG);
}

static u32 sunxi_battery_enable_irq_ch_select(void __iomem *reg_base,
					      enum gp_channel_id id)
{
	u32 reg_val;

	reg_val = readl(reg_base + GP_DATA_INTC_REG);
	switch (id) {
	case GP_CH_0:
		reg_val |= GP_CH0_SELECT;
		break;
	case GP_CH_1:
		reg_val |= GP_CH1_SELECT;
		break;
	case GP_CH_2:
		reg_val |= GP_CH2_SELECT;
		break;
	case GP_CH_3:
		reg_val |= GP_CH3_SELECT;
		break;
	case GP_CH_4:
		reg_val |= GP_CH4_SELECT;
		break;
	case GP_CH_5:
		reg_val |= GP_CH5_SELECT;
		break;
	case GP_CH_6:
		reg_val |= GP_CH6_SELECT;
		break;
	case GP_CH_7:
		reg_val |= GP_CH7_SELECT;
		break;
	default:
		pr_err("%s, invalid channel id!", __func__);
		return -EINVAL;
	}
	writel(reg_val, reg_base + GP_DATA_INTC_REG);

	return 0;
}

static u32 sunxi_battery_disable_irq_ch_select(void __iomem *reg_base,
					       enum gp_channel_id id)
{
	u32 reg_val;

	reg_val = readl(reg_base + GP_DATA_INTC_REG);
	switch (id) {
	case GP_CH_0:
		reg_val &= ~GP_CH0_SELECT;
		break;
	case GP_CH_1:
		reg_val &= ~GP_CH1_SELECT;
		break;
	case GP_CH_2:
		reg_val &= ~GP_CH2_SELECT;
		break;
	case GP_CH_3:
		reg_val &= ~GP_CH3_SELECT;
		break;
	case GP_CH_4:
		reg_val &= ~GP_CH4_SELECT;
		break;
	case GP_CH_5:
		reg_val &= ~GP_CH5_SELECT;
		break;
	case GP_CH_6:
		reg_val &= ~GP_CH6_SELECT;
		break;
	case GP_CH_7:
		reg_val &= ~GP_CH7_SELECT;
		break;
	default:
		pr_err("%s, invalid channel id!", __func__);
		return -EINVAL;
	}
	writel(reg_val, reg_base + GP_DATA_INTC_REG);

	return 0;
}

static u32 sunxi_battery_ch_irq_status(void __iomem *reg_base)
{
	return readl(reg_base + GP_DATA_INTS_REG);
}

static void sunxi_battery_ch_irq_clear_flags(void __iomem *reg_base, u32 flags)
{
	writel(flags, reg_base + GP_DATA_INTS_REG);
}

static u32 sunxi_gpadc_battery_read_ch_irq_enable(void __iomem *reg_base)
{
	return readl(reg_base + GP_DATA_INTC_REG);
}

static u32 sunxi_gpadc_battery_read_data(void __iomem *reg_base,
					 enum gp_channel_id id)
{
	switch (id) {
	case GP_CH_0:
		return readl(reg_base + GP_CH0_DATA_REG) & GP_CH0_DATA_MASK;
	case GP_CH_1:
		return readl(reg_base + GP_CH1_DATA_REG) & GP_CH1_DATA_MASK;
	case GP_CH_2:
		return readl(reg_base + GP_CH2_DATA_REG) & GP_CH2_DATA_MASK;
	case GP_CH_3:
		return readl(reg_base + GP_CH3_DATA_REG) & GP_CH3_DATA_MASK;
	case GP_CH_4:
		return readl(reg_base + GP_CH4_DATA_REG) & GP_CH4_DATA_MASK;
	case GP_CH_5:
		return readl(reg_base + GP_CH5_DATA_REG) & GP_CH5_DATA_MASK;
	case GP_CH_6:
		return readl(reg_base + GP_CH6_DATA_REG) & GP_CH6_DATA_MASK;
	case GP_CH_7:
		return readl(reg_base + GP_CH7_DATA_REG) & GP_CH7_DATA_MASK;
	default:
		pr_err("%s, invalid channel id!", __func__);
		return -EINVAL;
	}
}

static int sunxi_gpadc_battery_input_event_set(struct input_dev *input_dev,
					       enum gp_channel_id id)
{
	if (!input_dev) {
		pr_err("%s:input_dev: not enough memory for input device\n",
		       __func__);
		return -ENOMEM;
	}

	switch (id) {
	case GP_CH_0:

		input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_REP);

	case GP_CH_1:
	case GP_CH_2:
#ifndef CONFIG_ARCH_SUN8IW18
	case GP_CH_3:
#endif
	case GP_CH_4:
	case GP_CH_5:
	case GP_CH_6:
	case GP_CH_7:
		input_dev->evbit[0] = BIT_MASK(EV_MSC);
		set_bit(EV_MSC, input_dev->evbit);
		set_bit(MSC_SCAN, input_dev->mscbit);
		break;
	default:
		pr_err("%s, invalid channel id!", __func__);
		return -ENOMEM;
	}

	return 0;
}

static int sunxi_gpadc_battery_input_register(
	struct sunxi_gpadc_battery *sunxi_gpadc_battery, int id)
{
	static struct input_dev *input_dev;
	int ret;

	input_dev = input_allocate_device();
	if (!input_dev) {
		pr_err("input_dev: not enough memory for input device\n");
		return -ENOMEM;
	}

	switch (id) {
	case GP_CH_0:
		input_dev->name = "sunxi-gpadc-battery0";
		input_dev->phys = "sunxigpadcbattery0/input0";
		break;
	case GP_CH_1:
		input_dev->name = "sunxi-gpadc-battery1";
		input_dev->phys = "sunxigpadcbattery1/input0";
		break;
	case GP_CH_2:
		input_dev->name = "sunxi-gpadc-battery2";
		input_dev->phys = "sunxigpadcbattery2/input0";
		break;
	case GP_CH_3:
		input_dev->name = "sunxi-gpadc-battery3";
		input_dev->phys = "sunxigpadcbattery3/input0";
		break;
	case GP_CH_4:
		input_dev->name = "sunxi-gpadc-battery4";
		input_dev->phys = "sunxigpadcbattery4/input0";
		break;
	case GP_CH_5:
		input_dev->name = "sunxi-gpadc-battery5";
		input_dev->phys = "sunxigpadcbattery5/input0";
		break;
	case GP_CH_6:
		input_dev->name = "sunxi-gpadc-battery6";
		input_dev->phys = "sunxigpadcbattery6/input0";
		break;
	case GP_CH_7:
		input_dev->name = "sunxi-gpadc-battery7";
		input_dev->phys = "sunxigpadcbattery7/input0";
		break;
	default:
		pr_err("%s, invalid channel id!", __func__);
		goto fail;
	}

	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x0101;
	input_dev->id.product = 0x0011;
	input_dev->id.version = 0x0110;

	sunxi_gpadc_battery_input_event_set(input_dev, id);
	sunxi_gpadc_battery->input_gpadc_battery[id] = input_dev;
	ret = input_register_device(
		sunxi_gpadc_battery->input_gpadc_battery[id]);
	if (ret) {
		pr_err("input register fail\n");
		goto fail1;
	}

	return 0;

fail1:
	input_unregister_device(sunxi_gpadc_battery->input_gpadc_battery[id]);
fail:
	input_free_device(sunxi_gpadc_battery->input_gpadc_battery[id]);
	return ret;
}

static int sunxi_gpadc_battery_input_register_setup(
	struct sunxi_gpadc_battery *sunxi_gpadc_battery)
{
	struct sunxi_config *config = NULL;
	int i;

	config = &sunxi_gpadc_battery->gpadc_battery_config;
	for (i = 0; i < sunxi_gpadc_battery->channel_num; i++) {
		if (config->channel_select & sunxi_gpadc_battery_channel_id(i))
			sunxi_gpadc_battery_input_register(sunxi_gpadc_battery,
							   i);
	}

	return 0;
}

static int
sunxi_gpadc_battery_setup(struct platform_device *pdev,
			  struct sunxi_gpadc_battery *sunxi_gpadc_battery)
{
	struct device_node *np = NULL;
	struct sunxi_config *gpadc_battery_config =
		&sunxi_gpadc_battery->gpadc_battery_config;

	np = pdev->dev.of_node;
	if (of_property_read_u32(
		    np, "gpadc_battery_sample_rate",
		    &sunxi_gpadc_battery->gpadc_battery_sample_rate)) {
		pr_debug("%s: get sample rate failed\n", __func__);
		sunxi_gpadc_battery->gpadc_battery_sample_rate = DEFAULT_SR;
	} else {
		if (sunxi_gpadc_battery->gpadc_battery_sample_rate < MIN_SR ||
		    sunxi_gpadc_battery->gpadc_battery_sample_rate > MAX_SR)
			sunxi_gpadc_battery->gpadc_battery_sample_rate =
				DEFAULT_SR;
	}

	if (of_property_read_u32(np, "channel_num",
				 &sunxi_gpadc_battery->channel_num)) {
		pr_err("%s: get channel num failed\n", __func__);
		return -EBUSY;
	}

	if (of_property_read_u32(np, "channel_select",
				 &gpadc_battery_config->channel_select)) {
		pr_err("%s: get channel set select failed\n", __func__);
		gpadc_battery_config->channel_select = 0;
	}

	if (of_property_read_u32(np, "channel_data_select",
				 &gpadc_battery_config->channel_data_select)) {
		pr_err("%s: get channel data setlect failed\n", __func__);
		gpadc_battery_config->channel_data_select = 0;
	}

	if (of_property_read_u32(
		    np, "channel_compare_select",
		    &gpadc_battery_config->channel_compare_select)) {
		pr_err("%s: get channel compare select failed\n", __func__);
		gpadc_battery_config->channel_compare_select = 0;
	}

	if (of_property_read_u32(
			np, "resistance_before",
			&gpadc_battery_config->resistance_before)) {
		pr_err("%s: get resistance_before failed\n", __func__);
		gpadc_battery_config->resistance_before = 0;
	}

	if (of_property_read_u32(
			np, "resistance_later",
			&gpadc_battery_config->resistance_later)) {
		pr_err("%s: get resistance_later failed\n", __func__);
		gpadc_battery_config->resistance_later = 0;
	}

	sunxi_gpadc_battery->wakeup_en = 0;
	if (of_property_read_bool(np, "wakeup-source")) {
		device_init_wakeup(&pdev->dev, true);
		dev_pm_set_wake_irq(&pdev->dev, sunxi_gpadc_battery->irq_num);
		sunxi_gpadc_battery->wakeup_en = 1;
	}

	return 0;
}

static int
sunxi_gpadc_battery_request_clk(struct sunxi_gpadc_battery *sunxi_gpadc_battery)
{
	sunxi_gpadc_battery->bus_clk =
		devm_clk_get(sunxi_gpadc_battery->dev, "bus");
	if (IS_ERR_OR_NULL(sunxi_gpadc_battery->bus_clk)) {
		pr_err("[gpadc_battery%d] request GPADC_BATTERY clock failed\n",
		       sunxi_gpadc_battery->bus_num);
		return -1;
	}

	sunxi_gpadc_battery->reset =
		devm_reset_control_get(sunxi_gpadc_battery->dev, NULL);
	if (IS_ERR_OR_NULL(sunxi_gpadc_battery->reset)) {
		pr_err("[gpadc_battery%d] request GPADC_BATTERY reset failed\n",
		       sunxi_gpadc_battery->bus_num);
		return -1;
	}

	return 0;
}

static int
sunxi_gpadc_battery_clk_init(struct sunxi_gpadc_battery *sunxi_gpadc_battery)
{
	if (reset_control_deassert(sunxi_gpadc_battery->reset)) {
		pr_err("[gpadc_battery%d] reset control deassert failed!\n",
		       sunxi_gpadc_battery->bus_num);
		return -1;
	}

	if (clk_prepare_enable(sunxi_gpadc_battery->bus_clk)) {
		pr_err("[gpadc_battery%d] enable clock failed!\n",
		       sunxi_gpadc_battery->bus_num);
		return -1;
	}

	return 0;
}

static void
sunxi_gpadc_battery_clk_exit(struct sunxi_gpadc_battery *sunxi_gpadc_battery)
{
	if (!IS_ERR_OR_NULL(sunxi_gpadc_battery->bus_clk) &&
	    __clk_is_enabled(sunxi_gpadc_battery->bus_clk))
		clk_disable_unprepare(sunxi_gpadc_battery->bus_clk);
}

static int
sunxi_gpadc_battery_hw_init(struct sunxi_gpadc_battery *sunxi_gpadc_battery)
{
	struct sunxi_config *gpadc_battery_config =
		&sunxi_gpadc_battery->gpadc_battery_config;
	int i;

	if (sunxi_gpadc_battery_request_clk(sunxi_gpadc_battery)) {
		pr_err("[gpadc_battery%d] request gpadc_battery clk failed\n",
		       sunxi_gpadc_battery->bus_num);
		return -EPROBE_DEFER;
	}

	if (sunxi_gpadc_battery_clk_init(sunxi_gpadc_battery)) {
		pr_err("[gpadc_battery%d] init gpadc_battery clk failed\n",
		       sunxi_gpadc_battery->bus_num);
		return -EPROBE_DEFER;
	}

	sunxi_gpadc_battery_sample_rate_set(
		sunxi_gpadc_battery->reg_base, OSC_24MHZ,
		sunxi_gpadc_battery->gpadc_battery_sample_rate);
	for (i = 0; i < sunxi_gpadc_battery->channel_num; i++) {
		if (gpadc_battery_config->channel_select &
		    sunxi_gpadc_battery_channel_id(i)) {
			sunxi_gpadc_battery_ch_select(
				sunxi_gpadc_battery->reg_base, i);
			if (gpadc_battery_config->channel_data_select &
			    sunxi_gpadc_battery_channel_id(i)) {
				sunxi_battery_enable_irq_ch_select(
					sunxi_gpadc_battery->reg_base, i);
			}
			if (gpadc_battery_config->channel_compare_select &
			    sunxi_gpadc_battery_channel_id(i)) {
				sunxi_gpadc_battery_cmp_select(
					sunxi_gpadc_battery->reg_base, i);
				sunxi_battery_enable_irq_ch_select(
					sunxi_gpadc_battery->reg_base, i);
			}
		}
	}
	sunxi_gpadc_battery_calibration_enable(sunxi_gpadc_battery->reg_base);
	sunxi_gpadc_battery_mode_select(sunxi_gpadc_battery->reg_base,
					GP_CONTINUOUS_MODE);
	sunxi_gpadc_battery_enable(sunxi_gpadc_battery->reg_base, true);
	sunxi_gpadc_battery_enable_irq(sunxi_gpadc_battery->reg_base);

	return 0;
}

static int
sunxi_gpadc_battery_hw_exit(struct sunxi_gpadc_battery *sunxi_gpadc_battery)
{
	struct sunxi_config *gpadc_battery_config =
		&sunxi_gpadc_battery->gpadc_battery_config;
	int i;

	for (i = 0; i < sunxi_gpadc_battery->channel_num; i++) {
		if (gpadc_battery_config->channel_select &
		    sunxi_gpadc_battery_channel_id(i)) {
			sunxi_gpadc_battery_ch_deselect(
				sunxi_gpadc_battery->reg_base, i);
			if (gpadc_battery_config->channel_data_select &
			    sunxi_gpadc_battery_channel_id(i))
				sunxi_battery_disable_irq_ch_select(
					sunxi_gpadc_battery->reg_base, i);
			if (gpadc_battery_config->channel_compare_select &
			    sunxi_gpadc_battery_channel_id(i)) {
				sunxi_gpadc_battery_cmp_deselect(
					sunxi_gpadc_battery->reg_base, i);
				sunxi_battery_disable_irq_ch_select(
					sunxi_gpadc_battery->reg_base, i);
			}
		}
	}
	sunxi_gpadc_battery_enable(sunxi_gpadc_battery->reg_base, false);
	sunxi_gpadc_battery_clk_exit(sunxi_gpadc_battery);

	return 0;
}

static irqreturn_t sunxi_gpadc_battery_interrupt(int irqno, void *dev_id)
{
	struct sunxi_gpadc_battery *sunxi_gpadc_battery =
		(struct sunxi_gpadc_battery *)dev_id;
	u32 irq_data_set;
	u32 irq_data_val;
	u32 data;

#ifndef CONFIG_ARCH_SUN8IW18
	u32 i;
#endif
	irq_data_val = sunxi_gpadc_battery_read_ch_irq_enable(
		sunxi_gpadc_battery->reg_base);
	irq_data_set =
		sunxi_battery_ch_irq_status(sunxi_gpadc_battery->reg_base);
	sunxi_battery_ch_irq_clear_flags(sunxi_gpadc_battery->reg_base,
					 irq_data_set);

	if (irq_data_set & GP_CH0_DATA) {
		data = sunxi_gpadc_battery_read_data(
			sunxi_gpadc_battery->reg_base, GP_CH_0);
	}

	for (i = 1; i < sunxi_gpadc_battery->channel_num; i++) {
		if (irq_data_set & irq_data_val & (1 << i)) {
			data = sunxi_gpadc_battery_read_data(
				sunxi_gpadc_battery->reg_base, i);
			input_event(sunxi_gpadc_battery->input_gpadc_battery[i],
				    EV_MSC, MSC_SCAN, data);
			input_sync(sunxi_gpadc_battery->input_gpadc_battery[i]);
			pr_debug("channel %d data pend\n", i);
		}
	}

	return IRQ_HANDLED;
}

u32 sunxi_gpadc_battery_read_channel_data(u8 channel,
			  struct sunxi_gpadc_battery *sunxi_gpadc_battery)
{
	struct sunxi_config *gpadc_battery_config =
		&sunxi_gpadc_battery->gpadc_battery_config;
	u32 data, vol_data, vol_vcc;

	data = readl(sunxi_gpadc_battery->reg_base + GP_CS_EN_REG);

	if ((data & (0x01 << channel)) == 0) {
		return VOL_RANGE + 1;
	}

	data = sunxi_gpadc_battery_read_data(sunxi_gpadc_battery->reg_base,
					     channel);
	/* 12bits sample rate,data to val_data
	 *,which is the voltage before voltage division
	 */
	vol_data = (((data * VOL_RANGE) / 4096) / 1000);
	printk("%s, data: (%d / 4096), vol_data: %dmV\n", __func__, data,
	       vol_data);

	/* vol_data to vol_vcc
	 *,which is the voltage before voltage division
	 */
	vol_vcc = (vol_data * (gpadc_battery_config->resistance_before
				+ gpadc_battery_config->resistance_later)
				/ gpadc_battery_config->resistance_later);
	printk("%s, vol_vcc: %dmV\n", __func__, vol_vcc);

	return vol_vcc;
}
EXPORT_SYMBOL_GPL(sunxi_gpadc_battery_read_channel_data);

static ssize_t data_show(struct class *class, struct class_attribute *attr,
			 char *buf)
{
	unsigned int data;

	data = sunxi_gpadc_battery_read_channel_data(channel, sunxi_gpadc_battery);

	return sprintf(buf, "%d\n", data);
}

static struct class_attribute gpadc_battery_class_attrs[] = {
	__ATTR(data, 0644, data_show, NULL),
};

static struct class gpadc_battery_class = {
	.name = "gpadc_battery",
	.owner = THIS_MODULE,
};

int sunxi_gpadc_battery_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int ret = 0;

	if (!of_device_is_available(np)) {
		pr_err("%s: sunxi gpadc battery is disable\n", __func__);
		return -EPERM;
	}

	sunxi_gpadc_battery =
		kzalloc(sizeof(struct sunxi_gpadc_battery), GFP_KERNEL);
	if (IS_ERR_OR_NULL(sunxi_gpadc_battery)) {
		pr_err("not enough memory for sunxi_gpadc_battery\n");
		return -ENOMEM;
	}

	sunxi_gpadc_battery->reg_base = of_iomap(np, 0);
	if (NULL == sunxi_gpadc_battery->reg_base) {
		pr_err("sunxi_gpadc_battery of_iomap fail\n");
		ret = -EBUSY;
		goto fail1;
	}

	vaddr = ioremap(LDOA_EFUSE_REG, 1);
	if (!vaddr) {
		pr_err("sunxi_gpadc_battery iomap fail\n");
		ret = -EBUSY;
		goto fail1;
	}

	sunxi_gpadc_battery->irq_num = irq_of_parse_and_map(np, 0);
	if (0 == sunxi_gpadc_battery->irq_num) {
		pr_err("sunxi_gpadc_battery fail to map irq\n");
		ret = -EBUSY;
		goto fail2;
	}

	sunxi_gpadc_battery->dev = &pdev->dev;
	sunxi_gpadc_battery_setup(pdev, sunxi_gpadc_battery);
	sunxi_gpadc_battery_hw_init(sunxi_gpadc_battery);
	sunxi_gpadc_battery_input_register_setup(sunxi_gpadc_battery);
	sunxi_gpadc_battery_read_channel_data(channel, sunxi_gpadc_battery);

	platform_set_drvdata(pdev, sunxi_gpadc_battery);

	if (request_irq(sunxi_gpadc_battery->irq_num,
			sunxi_gpadc_battery_interrupt, IRQF_TRIGGER_NONE,
			"sunxi-gpadc-battery", sunxi_gpadc_battery)) {
		pr_err("sunxi_gpadc_battery request irq failure\n");
		return -1;
	}

	return 0;

fail2:
	iounmap(vaddr);
	iounmap(sunxi_gpadc_battery->reg_base);
fail1:
	kfree(sunxi_gpadc_battery);

	return ret;
}

int sunxi_gpadc_battery_remove(struct platform_device *pdev)
{
	struct sunxi_gpadc_battery *sunxi_gpadc_battery =
		platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&sunxi_gpadc_battery->gpadc_battery_work);

	sunxi_gpadc_battery_hw_exit(sunxi_gpadc_battery);
	free_irq(sunxi_gpadc_battery->irq_num, sunxi_gpadc_battery);
	iounmap(sunxi_gpadc_battery->reg_base);
	kfree(sunxi_gpadc_battery);

	return 0;
}

#define SUNXI_GPADC_BATTERY_DEV_PM_OPS NULL
static const struct of_device_id sunxi_gpadc_battery_of_match[] = {
	{.compatible = "allwinner,sunxi-gpadc-battery" },
	{},
};
MODULE_DEVICE_TABLE(of, sunxi_gpadc_battery_of_match);

static struct platform_driver sunxi_gpadc_battery_driver = {
	.probe = sunxi_gpadc_battery_probe,
	.remove = sunxi_gpadc_battery_remove,
	.driver = {
			.name = "sunxi-gpadc-battery",
			.owner = THIS_MODULE,
			.pm = SUNXI_GPADC_BATTERY_DEV_PM_OPS,
			.of_match_table =
				of_match_ptr(sunxi_gpadc_battery_of_match),
		},
};
module_param_named(debug_mask, debug_mask, int, 0644);

static int __init sunxi_gpadc_battery_init(void)
{
	int ret;
	int i;
	int err;

	ret = class_register(&gpadc_battery_class);
	if (ret < 0)
		pr_err("%s,%d err, ret:%d\n", __func__, __LINE__, ret);
	else
		pr_info("%s,%d, success\n", __func__, __LINE__);

	for (i = 0; i < ARRAY_SIZE(gpadc_battery_class_attrs); i++) {
		err = class_create_file(&gpadc_battery_class,
					&gpadc_battery_class_attrs[i]);
		if (err) {
			pr_err("%s(): class_create_file() failed. err=%d\n",
			       __func__, err);
			while (i--)
				class_remove_file(
					&gpadc_battery_class,
					&gpadc_battery_class_attrs[i]);
			class_unregister(&gpadc_battery_class);
			return err;
		}
	}

	ret = platform_driver_register(&sunxi_gpadc_battery_driver);
	if (ret != 0) {
		class_unregister(&gpadc_battery_class);
	}

	return ret;
}
module_init(sunxi_gpadc_battery_init);

static void __exit sunxi_gpadc_battery_exit(void)
{
	platform_driver_unregister(&sunxi_gpadc_battery_driver);
	class_unregister(&gpadc_battery_class);
}
module_exit(sunxi_gpadc_battery_exit);

MODULE_AUTHOR("Zhengweilong");
MODULE_DESCRIPTION("sunxi-gpadc-battery driver");
MODULE_LICENSE("GPL");
