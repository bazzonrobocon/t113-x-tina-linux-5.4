/*
 * sound\soc\sunxi\snd_sunxi_common.c
 * (C) Copyright 2021-2025
 * AllWinner Technology Co., Ltd. <www.allwinnertech.com>
 * Dby <dby@allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/list.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/device.h>
#include <linux/ioport.h>
#include <linux/regmap.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include "snd_sunxi_log.h"
#include "snd_sunxi_common.h"

#define HLOG		"COMMON"

/******* reg label *******/
int snd_sunxi_save_reg(struct regmap *regmap, struct audio_reg_label *reg_labels)
{
	int i = 0;

	SND_LOG_DEBUG(HLOG, "\n");

	while (reg_labels[i].name != NULL) {
		regmap_read(regmap, reg_labels[i].address, &(reg_labels[i].value));
		i++;
	}

	return i;
}
EXPORT_SYMBOL_GPL(snd_sunxi_save_reg);

int snd_sunxi_echo_reg(struct regmap *regmap, struct audio_reg_label *reg_labels)
{
	int i = 0;

	SND_LOG_DEBUG(HLOG, "\n");

	while (reg_labels[i].name != NULL) {
		regmap_write(regmap, reg_labels[i].address, reg_labels[i].value);
		i++;
	}

	return i;
}
EXPORT_SYMBOL_GPL(snd_sunxi_echo_reg);

/******* regulator config *******/
struct snd_sunxi_rglt *snd_sunxi_regulator_init(struct platform_device *pdev)
{
	int ret, i, j;
	struct device *dev = NULL;
	struct device_node *np = NULL;
	struct snd_sunxi_rglt *rglt = NULL;
	struct snd_sunxi_rglt_unit *unit = NULL;
	struct snd_sunxi_rglt_unit *rglt_unit = NULL;
	u32 temp_val;
	char str[32] = {0};
	const char *out_string;
	struct {
		char *name;
		enum SND_SUNXI_RGLT_MODE mode;
	} of_mode_table[] = {
		{ "PMU",	SND_SUNXI_RGLT_PMU },
		{ "AUDIO",	SND_SUNXI_RGLT_AUDIO },
	};

	SND_LOG_DEBUG(HLOG, "\n");

	if (!pdev) {
		SND_LOG_ERR(HLOG, "platform_device invailed\n");
		return NULL;
	}
	dev = &pdev->dev;
	np = pdev->dev.of_node;

	rglt = kzalloc(sizeof(*rglt), GFP_KERNEL);
	if (!rglt) {
		SND_LOGDEV_ERR(dev, HLOG, "can't allocate snd_sunxi_rglt memory\n");
		return NULL;
	}

	ret = of_property_read_u32(np, "rglt-max", &temp_val);
	if (ret < 0) {
		SND_LOGDEV_DEBUG(dev, HLOG, "rglt-max get failed\n");
		rglt->unit_cnt = 0;
		return rglt;
	} else {
		rglt->unit_cnt = temp_val;
	}

	rglt_unit = kzalloc(sizeof(*rglt_unit) * rglt->unit_cnt, GFP_KERNEL);
	if (!rglt) {
		SND_LOGDEV_ERR(dev, HLOG, "can't allocate rglt_unit memory\n");
		kfree(rglt_unit);
		return NULL;
	}

	for (i = 0; i < rglt->unit_cnt; ++i) {
		unit = &rglt_unit[i];
		snprintf(str, sizeof(str), "rglt%d-mode", i);
		ret = of_property_read_string(np, str, &out_string);
		if (ret < 0) {
			SND_LOGDEV_ERR(dev, HLOG, "get %s failed\n", str);
			goto err;
		} else {
			for (j = 0; i < ARRAY_SIZE(of_mode_table); ++j) {
				if (strcmp(out_string, of_mode_table[i].name) == 0) {
					unit->mode = of_mode_table[i].mode;
					break;
				}
			}
		}
		switch (unit->mode) {
		case SND_SUNXI_RGLT_PMU:
			snprintf(str, sizeof(str), "rglt%d-voltage", i);
			ret = of_property_read_u32(np, str, &temp_val);
			if (ret < 0) {
				SND_LOGDEV_ERR(dev, HLOG, "get %s failed\n", str);
				goto err;
			} else {
				unit->vcc_vol = temp_val;
			}
			snprintf(str, sizeof(str), "rglt%d", i);
			unit->vcc = regulator_get(dev, str);
			if (IS_ERR_OR_NULL(unit->vcc)) {
				SND_LOGDEV_ERR(dev, HLOG, "get %s failed\n", str);
				goto err;
			}
			ret = regulator_set_voltage(unit->vcc, unit->vcc_vol, unit->vcc_vol);
			if (ret < 0) {
				SND_LOGDEV_ERR(dev, HLOG, "set %s voltage failed\n", str);
				goto err;
			}
			ret = regulator_enable(unit->vcc);
			if (ret < 0) {
				SND_LOGDEV_ERR(dev, HLOG, "enable %s failed\n", str);
				goto err;
			}
			break;
		default:
			SND_LOGDEV_DEBUG(dev, HLOG, "%u mode no need to procees\n", unit->mode);
			break;
		}
	}

	rglt->unit = rglt_unit;
	rglt->priv = pdev;
	return rglt;
err:
	kfree(rglt_unit);
	kfree(rglt);
	return NULL;

}
EXPORT_SYMBOL_GPL(snd_sunxi_regulator_init);

void snd_sunxi_regulator_exit(struct snd_sunxi_rglt *rglt)
{
	int i;
	struct platform_device *pdev = NULL;
	struct device *dev = NULL;
	struct snd_sunxi_rglt_unit *unit = NULL;

	SND_LOG_DEBUG(HLOG, "\n");

	if (!rglt) {
		SND_LOG_ERR(HLOG, "snd_sunxi_rglt invailed\n");
		return;
	}
	pdev = (struct platform_device *)rglt->priv;
	dev = &pdev->dev;

	for (i = 0; i < rglt->unit_cnt; ++i) {
		unit = &rglt->unit[i];
		switch (unit->mode) {
		case SND_SUNXI_RGLT_PMU:
			if (!IS_ERR_OR_NULL(unit->vcc)) {
				regulator_disable(unit->vcc);
				regulator_put(unit->vcc);
			}
			break;
		default:
			break;
		}
	}

	if (rglt->unit)
		kfree(rglt->unit);
	kfree(rglt);
}
EXPORT_SYMBOL_GPL(snd_sunxi_regulator_exit);

int snd_sunxi_regulator_enable(struct snd_sunxi_rglt *rglt)
{
	int ret, i;
	struct platform_device *pdev = NULL;
	struct device *dev = NULL;
	struct snd_sunxi_rglt_unit *unit = NULL;

	SND_LOG_DEBUG(HLOG, "\n");

	if (!rglt) {
		SND_LOG_ERR(HLOG, "snd_sunxi_rglt invailed\n");
		return -1;
	}
	pdev = (struct platform_device *)rglt->priv;
	dev = &pdev->dev;

	for (i = 0; i < rglt->unit_cnt; ++i) {
		unit = &rglt->unit[i];
		switch (unit->mode) {
		case SND_SUNXI_RGLT_PMU:
			if (!IS_ERR_OR_NULL(unit->vcc)) {
				ret = regulator_enable(unit->vcc);
				if (ret) {
					SND_LOGDEV_ERR(dev, HLOG, "enable vcc failed\n");
					return -1;
				}
			}
			break;
		default:
			break;
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(snd_sunxi_regulator_enable);

void snd_sunxi_regulator_disable(struct snd_sunxi_rglt *rglt)
{
	int i;
	struct platform_device *pdev = NULL;
	struct device *dev = NULL;
	struct snd_sunxi_rglt_unit *unit = NULL;

	SND_LOG_DEBUG(HLOG, "\n");

	if (!rglt) {
		SND_LOG_ERR(HLOG, "snd_sunxi_rglt invailed\n");
		return;
	}
	pdev = (struct platform_device *)rglt->priv;
	dev = &pdev->dev;

	for (i = 0; i < rglt->unit_cnt; ++i) {
		unit = &rglt->unit[i];
		switch (unit->mode) {
		case SND_SUNXI_RGLT_PMU:
			if (!IS_ERR_OR_NULL(unit->vcc))
				regulator_disable(unit->vcc);
			break;
		default:
			break;
		}
	}
}
EXPORT_SYMBOL_GPL(snd_sunxi_regulator_disable);

/******* pa config *******/
struct pa_config *snd_sunxi_pa_pin_init(struct platform_device *pdev, u32 *pa_pin_max)
{
	int ret, i;
	u32 pin_max;
	u32 gpio_tmp;
	u32 temp_val;
	char str[32] = {0};
	struct pa_config *pa_cfg;
	struct device_node *np = pdev->dev.of_node;

	SND_LOG_DEBUG(HLOG, "\n");

	*pa_pin_max = 0;
	ret = of_property_read_u32(np, "pa-pin-max", &temp_val);
	if (ret < 0) {
		SND_LOG_DEBUG(HLOG, "pa-pin-max get failed, default 0\n");
		return NULL;
	} else {
		pin_max = temp_val;
	}

	pa_cfg = kzalloc(sizeof(*pa_cfg) * pin_max, GFP_KERNEL);
	if (!pa_cfg) {
		SND_LOG_ERR(HLOG, "can't pa_config memory\n");
		return NULL;
	}

	for (i = 0; i < pin_max; i++) {
		snprintf(str, sizeof(str), "pa-pin-%d", i);
		ret = of_get_named_gpio(np, str, 0);
		if (ret < 0) {
			SND_LOG_ERR(HLOG, "%s get failed\n", str);
			pa_cfg[i].used = 0;
			continue;
		}
		gpio_tmp = ret;
		if (!gpio_is_valid(gpio_tmp)) {
			SND_LOG_ERR(HLOG, "%s (%u) is invalid\n", str, gpio_tmp);
			pa_cfg[i].used = 0;
			continue;
		}
		ret = devm_gpio_request(&pdev->dev, gpio_tmp, str);
		if (ret) {
			SND_LOG_ERR(HLOG, "%s (%u) request failed\n", str, gpio_tmp);
			pa_cfg[i].used = 0;
			continue;
		}
		pa_cfg[i].used = 1;
		pa_cfg[i].pin = gpio_tmp;

		snprintf(str, sizeof(str), "pa-pin-level-%d", i);
		ret = of_property_read_u32(np, str, &temp_val);
		if (ret < 0) {
			SND_LOG_WARN(HLOG, "%s get failed, default low\n", str);
			pa_cfg[i].level = 0;
		} else {
			if (temp_val > 0)
				pa_cfg[i].level = 1;
		}
		snprintf(str, sizeof(str), "pa-pin-msleep-%d", i);
		ret = of_property_read_u32(np, str, &temp_val);
		if (ret < 0) {
			SND_LOG_WARN(HLOG, "%s get failed, default 0\n", str);
			pa_cfg[i].msleep = 0;
		} else {
			pa_cfg[i].msleep = temp_val;
		}
	}

	*pa_pin_max = pin_max;
	snd_sunxi_pa_pin_disable(pa_cfg, pin_max);

	return pa_cfg;
}
EXPORT_SYMBOL_GPL(snd_sunxi_pa_pin_init);

void snd_sunxi_pa_pin_exit(struct platform_device *pdev,
			   struct pa_config *pa_cfg, u32 pa_pin_max)
{
	SND_LOG_DEBUG(HLOG, "\n");

	snd_sunxi_pa_pin_disable(pa_cfg, pa_pin_max);
	kfree(pa_cfg);
}
EXPORT_SYMBOL_GPL(snd_sunxi_pa_pin_exit);

int snd_sunxi_pa_pin_enable(struct pa_config *pa_cfg, u32 pa_pin_max)
{
	int i;

	SND_LOG_DEBUG(HLOG, "\n");

	if (pa_pin_max < 1) {
		SND_LOG_DEBUG(HLOG, "no pa pin config\n");
		return 0;
	}

	for (i = 0; i < pa_pin_max; i++) {
		if (!pa_cfg[i].used)
			continue;

		gpio_direction_output(pa_cfg[i].pin, 1);
		gpio_set_value(pa_cfg[i].pin, pa_cfg[i].level);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(snd_sunxi_pa_pin_enable);

void snd_sunxi_pa_pin_disable(struct pa_config *pa_cfg, u32 pa_pin_max)
{
	int i;

	SND_LOG_DEBUG(HLOG, "\n");

	if (pa_pin_max < 1) {
		SND_LOG_DEBUG(HLOG, "no pa pin config\n");
		return;
	}

	for (i = 0; i < pa_pin_max; i++) {
		if (!pa_cfg[i].used)
			continue;

		gpio_direction_output(pa_cfg[i].pin, 1);
		gpio_set_value(pa_cfg[i].pin, !pa_cfg[i].level);
	}
}
EXPORT_SYMBOL_GPL(snd_sunxi_pa_pin_disable);

/******* hdmi format config *******/
DEFINE_SPINLOCK(hdmi_fmt_lock);

static enum HDMI_FORMAT g_hdmi_fmt;

enum HDMI_FORMAT snd_sunxi_hdmi_get_fmt(void)
{
	enum HDMI_FORMAT tmp_hdmi_fmt;
	unsigned long flags;

	spin_lock_irqsave(&hdmi_fmt_lock, flags);
	tmp_hdmi_fmt = g_hdmi_fmt;
	spin_unlock_irqrestore(&hdmi_fmt_lock, flags);

	return tmp_hdmi_fmt;
}
EXPORT_SYMBOL_GPL(snd_sunxi_hdmi_get_fmt);

int snd_sunxi_hdmi_set_fmt(int hdmi_fmt)
{
	unsigned long flags;

	spin_lock_irqsave(&hdmi_fmt_lock, flags);
	g_hdmi_fmt = hdmi_fmt;
	spin_unlock_irqrestore(&hdmi_fmt_lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(snd_sunxi_hdmi_set_fmt);

int snd_sunxi_hdmi_get_dai_type(struct device_node *np, unsigned int *dai_type)
{
	int ret;
	const char *str;

	SND_LOG_DEBUG(HLOG, "\n");

	if (!np) {
		SND_LOG_ERR(HLOG, "np is err\n");
		return -1;
	}

	ret = of_property_read_string(np, "dai-type", &str);
	if (ret < 0) {
		*dai_type = SUNXI_DAI_I2S_TYPE;
	} else {
		if (strcmp(str, "hdmi") == 0) {
			*dai_type = SUNXI_DAI_HDMI_TYPE;
		} else {
			*dai_type = SUNXI_DAI_I2S_TYPE;
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(snd_sunxi_hdmi_get_dai_type);

/******* sysfs dump *******/
struct snd_sunxi_dev {
	dev_t snd_dev;
	struct class *snd_class;
	char *snd_dev_name;
	char *snd_class_name;
};

static LIST_HEAD(dump_list);

int snd_sunxi_dump_register(struct snd_sunxi_dump *dump)
{
	struct snd_sunxi_dump *dump_tmp, *c;

	SND_LOG_DEBUG(HLOG, "\n");

	if (!dump) {
		SND_LOG_ERR(HLOG, "snd sunxi dump invailed\n");
		return -1;
	}
	if (!dump->name) {
		SND_LOG_ERR(HLOG, "snd sunxi dump name null\n");
		return -1;
	}

	list_for_each_entry_safe(dump_tmp, c, &dump_list, list) {
		if (!strcmp(dump_tmp->name, dump->name)) {
			SND_LOG_ERR(HLOG, "snd dump(%s) already exist\n", dump->name);
			return -1;
		}
	}

	dump->use = false;
	list_add_tail(&dump->list, &dump_list);
	SND_LOG_DEBUG(HLOG, "snd dump(%s) add\n", dump->name);

	return 0;
}
EXPORT_SYMBOL_GPL(snd_sunxi_dump_register);

void snd_sunxi_dump_unregister(struct snd_sunxi_dump *dump)
{
	struct snd_sunxi_dump *dump_del, *c;

	SND_LOG_DEBUG(HLOG, "\n");

	if (!dump) {
		SND_LOG_ERR(HLOG, "snd sunxi dump invailed\n");
		return;
	}

	list_for_each_entry_safe(dump_del, c, &dump_list, list) {
		if (!strcmp(dump_del->name, dump->name)) {
			SND_LOG_DEBUG(HLOG, "snd dump(%s) del\n", dump_del->name);
			list_del(&dump_del->list);
		}
	}
}
EXPORT_SYMBOL_GPL(snd_sunxi_dump_unregister);

static ssize_t snd_sunxi_version_show(struct class *class, struct class_attribute *attr, char *buf)
{
	size_t count = 0, cound_tmp = 0;
	struct snd_sunxi_dump *dump_tmp, *c;
	struct snd_sunxi_dump *dump = NULL;

	list_for_each_entry_safe(dump_tmp, c, &dump_list, list) {
		dump = dump_tmp;
		if (dump && dump->dump_version) {
			count += sprintf(buf + count, "module(%s) version: ", dump->name);
			dump->dump_version(dump->priv, buf + count, &cound_tmp);
			count += cound_tmp;
		}
	}

	return count;
}

static ssize_t snd_sunxi_help_show(struct class *class, struct class_attribute *attr, char *buf)
{
	size_t count = 0, cound_tmp = 0;
	struct snd_sunxi_dump *dump_tmp, *c;
	struct snd_sunxi_dump *dump = NULL;

	list_for_each_entry_safe(dump_tmp, c, &dump_list, list)
		if (dump_tmp->use)
			dump = dump_tmp;

	count += sprintf(buf + count, "== module help ==\n");
	count += sprintf(buf + count, "1. get optional modules: cat module\n");
	count += sprintf(buf + count, "2. set current module  : echo {module name} > module\n");

	if (dump && dump->dump_help) {
		count += sprintf(buf + count, "== current module(%s) help ==\n", dump->name);
		dump->dump_help(dump->priv, buf + count, &cound_tmp);
		count += cound_tmp;
	} else if (dump && !dump->dump_help) {
		count += sprintf(buf + count, "== current module(%s), but not help ==\n", dump->name);
	} else {
		count += sprintf(buf + count, "== current module(NULL) ==\n");
	}

	return count;
}

static ssize_t snd_sunxi_module_show(struct class *class, struct class_attribute *attr, char *buf)
{
	size_t count = 0;
	struct snd_sunxi_dump *dump_tmp, *c;
	struct snd_sunxi_dump *dump = NULL;
	unsigned int module_num = 0;

	count += sprintf(buf + count, "optional modules:\n");
	list_for_each_entry_safe(dump_tmp, c, &dump_list, list) {
		count += sprintf(buf + count, "%u. %s\n", ++module_num, dump_tmp->name);
		if (dump_tmp->use)
			dump = dump_tmp;
	}

	if (dump)
		count += sprintf(buf + count, "current module(%s)\n", dump->name);
	else
		count += sprintf(buf + count, "current module(NULL)\n");

	return count;
}

static ssize_t snd_sunxi_module_store(struct class *class, struct class_attribute *attr,
				      const char *buf, size_t count)
{
	struct snd_sunxi_dump *dump, *c;
	int scanf_cnt = 0;
	char arg1[32] = {0};

	scanf_cnt = sscanf(buf, "%31s", arg1);
	if (scanf_cnt != 1)
		return count;

	list_for_each_entry_safe(dump, c, &dump_list, list) {
		if (!strcmp(arg1, dump->name))
			dump->use = true;
		else
			dump->use = false;
	}

	return count;
}

static ssize_t snd_sunxi_dump_show(struct class *class, struct class_attribute *attr, char *buf)
{
	int ret;
	size_t count = 0, cound_tmp = 0;
	struct snd_sunxi_dump *dump_tmp, *c;
	struct snd_sunxi_dump *dump = NULL;

	list_for_each_entry_safe(dump_tmp, c, &dump_list, list)
		if (dump_tmp->use)
			dump = dump_tmp;

	if (dump && dump->dump_show) {
		count += sprintf(buf + count, "module(%s)\n", dump->name);
		ret = dump->dump_show(dump->priv, buf + count, &cound_tmp);
		if (ret)
			pr_err("module(%s) show failed\n", dump->name);
		count += cound_tmp;
	} else if (dump && !dump->dump_show) {
		count += sprintf(buf + count, "current module(%s), but not show\n", dump->name);
	} else {
		count += sprintf(buf + count, "current module(NULL)\n");
	}

	return count;
}

static ssize_t snd_sunxi_dump_store(struct class *class, struct class_attribute *attr,
				    const char *buf, size_t count)
{
	int ret;
	struct snd_sunxi_dump *dump_tmp, *c;
	struct snd_sunxi_dump *dump = NULL;

	list_for_each_entry_safe(dump_tmp, c, &dump_list, list)
		if (dump_tmp->use)
			dump = dump_tmp;

	if (dump && dump->dump_store) {
		ret = dump->dump_store(dump->priv, buf, count);
		if (ret)
			pr_err("module(%s) store failed\n", dump->name);
	}

	return count;
}

static struct class_attribute snd_class_attrs[] = {
	__ATTR(version, 0644, snd_sunxi_version_show, NULL),
	__ATTR(help, 0644, snd_sunxi_help_show, NULL),
	__ATTR(module, 0644, snd_sunxi_module_show, snd_sunxi_module_store),
	__ATTR(dump, 0644, snd_sunxi_dump_show, snd_sunxi_dump_store),
};

#if IS_ENABLED(CONFIG_SND_SOC_SUNXI_DEBUG)
static int snd_sunxi_debug_create(struct snd_sunxi_dev *sunxi_dev)
{
	int ret, i;
	unsigned int debug_node_cnt;

	SND_LOG_DEBUG(HLOG, "\n");

	debug_node_cnt = ARRAY_SIZE(snd_class_attrs);
	for (i = 0; i < debug_node_cnt; i++) {
		ret = class_create_file(sunxi_dev->snd_class, &snd_class_attrs[i]);
		if (ret) {
			SND_LOG_ERR(HLOG, "class_create_file %s failed\n",
				    snd_class_attrs[i].attr.name);
			return -1;
		}
	}

	return 0;
}

static void snd_sunxi_debug_remove(struct snd_sunxi_dev *sunxi_dev)
{
	int i;
	unsigned int debug_node_cnt;

	SND_LOG_DEBUG(HLOG, "\n");

	debug_node_cnt = ARRAY_SIZE(snd_class_attrs);
	for (i = 0; i < debug_node_cnt; i++)
		class_remove_file(sunxi_dev->snd_class, &snd_class_attrs[i]);
}
#else
static int snd_sunxi_debug_create(struct snd_sunxi_dev *sunxi_dev)
{
	(void)snd_class_attrs;

	SND_LOG_DEBUG(HLOG, "unsupport debug\n");
	(void)sunxi_dev;
	return 0;
}

static void snd_sunxi_debug_remove(struct snd_sunxi_dev *sunxi_dev)
{
	SND_LOG_DEBUG(HLOG, "unsupport debug\n");
	(void)sunxi_dev;
}
#endif

static int _snd_sunxi_dev_init(struct snd_sunxi_dev *sunxi_dev)
{
	int ret;

	SND_LOG_DEBUG(HLOG, "\n");

	if (IS_ERR_OR_NULL(sunxi_dev)) {
		SND_LOG_ERR(HLOG, "snd_sunxi_dev is NULL\n");
		return -1;
	}
	if (IS_ERR_OR_NULL(sunxi_dev->snd_dev_name) ||
	    IS_ERR_OR_NULL(sunxi_dev->snd_class_name)) {
		SND_LOG_ERR(HLOG, "snd_sunxi_dev name member is NULL\n");
		return -1;
	}

	ret = alloc_chrdev_region(&sunxi_dev->snd_dev, 0, 1, sunxi_dev->snd_dev_name);
	if (ret) {
		SND_LOG_ERR(HLOG, "alloc_chrdev_region failed\n");
		goto err_alloc_chrdev;
	}
	SND_LOG_DEBUG(HLOG, "sunxi_dev major = %u, sunxi_dev minor = %u\n",
		      MAJOR(sunxi_dev->snd_dev), MINOR(sunxi_dev->snd_dev));

	sunxi_dev->snd_class = class_create(THIS_MODULE, sunxi_dev->snd_class_name);
	if (IS_ERR_OR_NULL(sunxi_dev->snd_class)) {
		SND_LOG_ERR(HLOG, "class_create failed\n");
		goto err_class_create;
	}

	ret = snd_sunxi_debug_create(sunxi_dev);
	if (ret) {
		SND_LOG_ERR(HLOG, "snd_sunxi_debug_create failed\n");
		goto err_class_create_file;
	}

	return 0;

err_class_create_file:
	class_destroy(sunxi_dev->snd_class);
err_class_create:
	unregister_chrdev_region(sunxi_dev->snd_dev, 1);
err_alloc_chrdev:
	return -1;
}

static void _snd_sunxi_dev_exit(struct snd_sunxi_dev *sunxi_dev)
{
	SND_LOG_DEBUG(HLOG, "\n");

	if (IS_ERR_OR_NULL(sunxi_dev)) {
		SND_LOG_ERR(HLOG, "snd_sunxi_dev is NULL\n");
		return;
	}
	if (IS_ERR_OR_NULL(sunxi_dev->snd_class)) {
		SND_LOG_ERR(HLOG, "snd_sunxi_dev class is NULL\n");
		return;
	}

	snd_sunxi_debug_remove(sunxi_dev);

	class_destroy(sunxi_dev->snd_class);
	unregister_chrdev_region(sunxi_dev->snd_dev, 1);
}

static struct snd_sunxi_dev sunxi_dev = {
	.snd_dev_name = "snd_sunxi_dev",
	.snd_class_name = "snd_sunxi",
};

int __init snd_sunxi_dev_init(void)
{
	SND_LOG_DEBUG(HLOG, "\n");
	return _snd_sunxi_dev_init(&sunxi_dev);
}

void __exit snd_sunxi_dev_exit(void)
{
	SND_LOG_DEBUG(HLOG, "\n");
	_snd_sunxi_dev_exit(&sunxi_dev);
}

module_init(snd_sunxi_dev_init);
module_exit(snd_sunxi_dev_exit);

MODULE_AUTHOR("Dby@allwinnertech.com");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.2");
MODULE_DESCRIPTION("sunxi common interface");
