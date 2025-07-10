/*
 * Copyright (C) 2019 Allwinner Technology Limited. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * Author: Albert Yu <yuxyun@allwinnertech.com>
 */

#include <linux/ioport.h>
#include <mali_kbase.h>
#include <mali_kbase_defs.h>
#include <mali_kbase_config.h>
#include "mali_kbase_config_platform.h"

#include "platform.h"
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>

#include <linux/pm_opp.h>
#include <linux/pm_runtime.h>

#define SUNXI_BAK_CLK_RATE 600000000

static struct sunxi_data *sunxi_data;
static struct kbase_device *s_kbdev;
static struct dentry *sunxi_debugfs;

#ifdef CONFIG_DEBUG_FS
static int sunxi_create_debugfs(void);
#endif
void kbase_pm_get_dvfs_metrics(struct kbase_device *kbdev,
			       struct kbasep_pm_metrics *last,
			       struct kbasep_pm_metrics *diff);

static inline void ioremap_regs(void)
{
#if defined(GPU_SUPPORT_DRM)
	struct reg *p_drm;

	p_drm = &sunxi_data->regs.drm;
	p_drm->phys = SMC_GPU_DRM_REG;
	p_drm->ioaddr = ioremap(p_drm->phys, 4);
#endif
}

static inline void iounmap_regs(void)
{
#if defined(GPU_SUPPORT_DRM)
	iounmap(sunxi_data->regs.drm.ioaddr);
#endif
}

static int sunxi_protected_mode_enable(struct protected_mode_device *pdev)
{
#if defined(GPU_SUPPORT_DRM)
	u32 val;
	val = readl(sunxi_data->regs.drm.ioaddr);
	val |= 1;
	writel(val, sunxi_data->regs.drm.ioaddr);
#endif
	return 0;
}

static int sunxi_protected_mode_disable(struct protected_mode_device *pdev)
{
#if defined(GPU_SUPPORT_DRM)
	u32 val;
	val = readl(sunxi_data->regs.drm.ioaddr);
	val &= ~1;
	writel(val, sunxi_data->regs.drm.ioaddr);
#endif
	return 0;
}

struct protected_mode_ops sunxi_protected_ops = {
	.protected_mode_enable = sunxi_protected_mode_enable,
	.protected_mode_disable = sunxi_protected_mode_disable
};

static int parse_dts_and_fex(struct kbase_device *kbdev, struct sunxi_data *sunxi_data)
{
#ifdef CONFIG_OF
	u32 val;
	int err;
	err = of_property_read_u32(kbdev->dev->of_node, "gpu_idle", &val);
	if (!err)
		sunxi_data->idle_ctrl = val ? true : false;
	err = of_property_read_u32(kbdev->dev->of_node, "independent_power", &val);
	if (!err)
		sunxi_data->independent_power = val ? true : false;
	err = of_property_read_u32(kbdev->dev->of_node, "dvfs_status", &val);
	if (!err)
		sunxi_data->dvfs_ctrl = val ? true : false;

	sunxi_data->reset = devm_reset_control_get(kbdev->dev, "rst_bus_gpu");
	if (IS_ERR_OR_NULL(sunxi_data->reset)) {
		dev_info(kbdev->dev, "sunxi init gpu Failed to get reset ctrl\n");
	}
#endif /* CONFIG_OF */

	return 0;
}

static ssize_t scene_ctrl_cmd_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return sprintf(buf, "%d\n", sunxi_data->sence_ctrl);
}

static ssize_t scene_ctrl_cmd_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int err;
	unsigned long val;
	unsigned long volt;
	enum scene_ctrl_cmd cmd;
#if defined(CONFIG_PM_OPP)
	struct dev_pm_opp *opp;
#endif
	err = kstrtoul(buf, 10, &val);
	if (err) {
		dev_err(dev, "scene_ctrl_cmd_store gets a invalid parameter!\n");
		return count;
	}

	cmd = (enum scene_ctrl_cmd)val;
	switch (cmd) {
	case SCENE_CTRL_NORMAL_MODE:
		sunxi_data->sence_ctrl = 0;
		break;

	case SCENE_CTRL_PERFORMANCE_MODE:
		sunxi_data->sence_ctrl = 1;
		val = sunxi_data->max_freq;
		volt = sunxi_data->max_u_volt;
#if defined(CONFIG_PM_OPP)
		rcu_read_lock();
		opp = dev_pm_opp_find_freq_floor(dev, &val);
		if (!IS_ERR_OR_NULL(opp)) {
			volt = dev_pm_opp_get_voltage(opp);
		} else {
			val = sunxi_data->max_freq;
			volt = sunxi_data->max_u_volt;
		}
		rcu_read_unlock();
#endif

#ifdef CONFIG_MALI_DEVFREQ
		s_kbdev->current_nominal_freq = val;
#endif
		break;

	default:
		dev_err(dev, "invalid scene control command %d!\n", cmd);
		return count;
	}

	return count;
}

static DEVICE_ATTR(command, 0660,
		scene_ctrl_cmd_show, scene_ctrl_cmd_store);

static struct attribute *scene_ctrl_attributes[] = {
	&dev_attr_command.attr,
	NULL
};

static struct attribute_group scene_ctrl_attribute_group = {
	.name = "scenectrl",
	.attrs = scene_ctrl_attributes
};

int sunxi_platform_init(struct kbase_device *kbdev)
{
	dev_err(kbdev->dev, "%s start!\n", __func__);

//(1)allocate sunxi private data and initialize it
	sunxi_data = (struct sunxi_data *)kzalloc(sizeof(struct sunxi_data), GFP_KERNEL);
	if (IS_ERR_OR_NULL(sunxi_data)) {
		dev_err(kbdev->dev, "sunxi init gpu Failed to malloc memory.\n");
		return -ENOMEM;
	}
	sunxi_data->dvfs_ctrl = true;
	sunxi_data->idle_ctrl = true;
	sunxi_data->independent_power = false;
	sunxi_data->power_on = false;
	sunxi_data->clk_on = false;
	mutex_init(&sunxi_data->sunxi_lock);

//(2)ioremap other repling devices' registers
	ioremap_regs();

//(3)paraw dts for device-speciffic property
	parse_dts_and_fex(kbdev, sunxi_data);
#ifdef CONFIG_REGULATOR
	if (IS_ERR_OR_NULL(kbdev->regulators[0])) {
		sunxi_data->independent_power = 0;
	}
#else
	sunxi_data->independent_power = 0;
#endif

//(4)enable clk
	reset_control_deassert(sunxi_data->reset);
	clk_set_parent(kbdev->clocks[1], kbdev->clocks[0]);

	pm_runtime_enable(kbdev->dev);

#ifdef CONFIG_DEBUG_FS
	sunxi_create_debugfs();
#endif

	if (sysfs_create_group(&kbdev->dev->kobj, &scene_ctrl_attribute_group)) {
		dev_err(kbdev->dev, "sunxi sysfs group creation failed!\n");
	}
	s_kbdev = kbdev;

	dev_err(kbdev->dev, "%s end!\n", __func__);
	return 0;
}

void sunxi_platform_term(struct kbase_device *kbdev)
{
	sysfs_remove_group(&kbdev->dev->kobj, &scene_ctrl_attribute_group);
	debugfs_remove_recursive(sunxi_debugfs);

	pm_runtime_disable(kbdev->dev);
	reset_control_deassert(sunxi_data->reset);

	iounmap_regs();
	mutex_destroy(&sunxi_data->sunxi_lock);
	kfree(sunxi_data);

	sunxi_data = NULL;
}

struct kbase_platform_funcs_conf sunxi_platform_conf = {
	.platform_init_func = sunxi_platform_init,
	.platform_term_func = sunxi_platform_term,
};

static int sunxi_gpu_power_on(struct kbase_device *kbdev)
{
	unsigned int i;

	dev_info(kbdev->dev, "sunxi_pm_callback_runtime_on\n");

	reset_control_deassert(sunxi_data->reset);
	clk_set_parent(kbdev->clocks[1], kbdev->clocks[0]);

#if defined(CONFIG_REGULATOR)
	for (i = 0; i < kbdev->nr_regulators; i++) {
		if (WARN_ON(kbdev->regulators[i] == NULL))
			;
		else if (!regulator_is_enabled(kbdev->regulators[i]))
			WARN_ON(regulator_enable(kbdev->regulators[i]));
	}
#endif

	for (i = 0; i < kbdev->nr_clocks; i++) {
		if (WARN_ON(kbdev->clocks[i] == NULL))
			;
		else if (!__clk_is_enabled(kbdev->clocks[i]))
			WARN_ON(clk_prepare_enable(kbdev->clocks[i]));
	}

	return 0;
}

static void sunxi_gpu_power_off(struct kbase_device *kbdev)
{
	unsigned int i;

	dev_info(kbdev->dev, "sunxi_pm_callback_runtime_off\n");

	for (i = 0; i < kbdev->nr_clocks; i++) {
		if (WARN_ON(kbdev->clocks[i] == NULL))
			;
		else if (__clk_is_enabled(kbdev->clocks[i])) {
			clk_disable_unprepare(kbdev->clocks[i]);
			WARN_ON(__clk_is_enabled(kbdev->clocks[i]));
		}

	}

#if defined(CONFIG_REGULATOR)
	for (i = 0; i < kbdev->nr_regulators; i++) {
		if (WARN_ON(kbdev->regulators[i] == NULL))
			;
		else if (regulator_is_enabled(kbdev->regulators[i]))
			WARN_ON(regulator_disable(kbdev->regulators[i]));
	}
#endif
}

static int sunxi_pm_callback_power_on(struct kbase_device *kbdev)
{
	int ret = 1; /* Assume GPU has been powered off */
	int error;

	dev_info(kbdev->dev, "sunxi_pm_callback_power_on %p\n",
			(void *)kbdev->dev->pm_domain);

	sunxi_gpu_power_on(kbdev);

	error = pm_runtime_get_sync(kbdev->dev);
	if (error == 1) {
		/*
		 * Let core know that the chip has not been
		 * powered off, so we can save on re-initialization.
		 */
		ret = 0;
	}

	dev_info(kbdev->dev, "pm_runtime_get_sync returned %d\n", error);

	return ret;
}

static void sunxi_pm_callback_power_off(struct kbase_device *kbdev)
{
	dev_info(kbdev->dev, "sunxi_pm_callback_power_off\n");

	pm_runtime_mark_last_busy(kbdev->dev);
	pm_runtime_put_autosuspend(kbdev->dev);

#ifndef KBASE_PM_RUNTIME
	sunxi_gpu_power_off(kbdev);
#endif
}

static void sunxi_pm_callback_resume(struct kbase_device *kbdev)
{
	int ret = sunxi_gpu_power_on(kbdev);

	WARN_ON(ret);
}

static void sunxi_pm_callback_suspend(struct kbase_device *kbdev)
{
	sunxi_gpu_power_off(kbdev);
}

#ifdef KBASE_PM_RUNTIME
static int sunxi_kbase_device_runtime_init(struct kbase_device *kbdev)
{
	int ret = 0;

	dev_info(kbdev->dev, "sunxi_kbase_device_runtime_init\n");

	pm_runtime_set_autosuspend_delay(kbdev->dev, AUTO_SUSPEND_DELAY);
	pm_runtime_use_autosuspend(kbdev->dev);

	pm_runtime_set_active(kbdev->dev);
	pm_runtime_enable(kbdev->dev);

	if (!pm_runtime_enabled(kbdev->dev)) {
		dev_warn(kbdev->dev, "pm_runtime not enabled");
		ret = -ENOSYS;
	}

	return ret;
}

static void sunxi_kbase_device_runtime_disable(struct kbase_device *kbdev)
{
	dev_info(kbdev->dev, "sunxi_kbase_device_runtime_disable\n");
	pm_runtime_disable(kbdev->dev);
}

static int sunxi_pm_callback_runtime_on(struct kbase_device *kbdev)
{
	dev_info(kbdev->dev, "sunxi_pm_callback_runtime_on\n");

	sunxi_gpu_power_on(kbdev);
	return 0;
}

static void sunxi_pm_callback_runtime_off(struct kbase_device *kbdev)
{
	dev_info(kbdev->dev, "sunxi_pm_callback_runtime_off\n");

	sunxi_gpu_power_off(kbdev);
}
#endif

struct kbase_pm_callback_conf sunxi_pm_callbacks = {
	.power_on_callback = sunxi_pm_callback_power_on,
	.power_off_callback = sunxi_pm_callback_power_off,
	.power_suspend_callback  = sunxi_pm_callback_resume,
	.power_resume_callback = sunxi_pm_callback_suspend,
#ifdef KBASE_PM_RUNTIME
	.power_runtime_init_callback = sunxi_kbase_device_runtime_init,
	.power_runtime_term_callback = sunxi_kbase_device_runtime_disable,
	.power_runtime_on_callback = sunxi_pm_callback_runtime_on,
	.power_runtime_off_callback = sunxi_pm_callback_runtime_off,
#else				/* KBASE_PM_RUNTIME */
	.power_runtime_init_callback = NULL,
	.power_runtime_term_callback = NULL,
	.power_runtime_on_callback = NULL,
	.power_runtime_off_callback = NULL,
#endif

};

#ifdef CONFIG_DEBUG_FS
static ssize_t write_write(struct file *filp, const char __user *buf,
	size_t count, loff_t *offp)
{
	int i, err;
	unsigned long val;
	bool semicolon = false;
	bool update_man = false;
	char buffer[50], data[32];
	int head_size, data_size;
	static unsigned long man_freq;
	static unsigned long man_u_volt;

	if (count >= sizeof(buffer))
		goto err_out;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	/* The command format is '<head>:<data>' or '<head>:<data>;' */
	for (i = 0; i < count; i++) {
		if (*(buffer+i) == ':')
			head_size = i;
		if (*(buffer+i) == ';' && head_size) {
			data_size = count - head_size - 3;
			semicolon = true;
			break;
		}
	}

	if (!head_size)
		goto err_out;

	if (!semicolon)
		data_size = count - head_size - 2;

	if (data_size > 32)
		goto err_out;

	memcpy(data, buffer + head_size + 1, data_size);
	data[data_size] = '\0';

	err = kstrtoul(data, 10, &val);
	if (err)
		goto err_out;

	if (!strncmp("frequency", buffer, head_size)) {
		if (val == 0 || val == 1) {
			sunxi_data->man_ctrl = val ? true : false;
		} else {
			man_freq = val * 1000 * 1000;
			update_man = 1;
		}
	} else if (!strncmp("voltage", buffer, head_size)) {
		if (val == 0 || val == 1) {
			sunxi_data->man_ctrl = val ? true : false;
		} else {
			update_man = 1;
			man_u_volt = val * 1000;
		}
	} else if (!strncmp("idle", buffer, head_size)) {
		if (val == 0 || val == 1)
			sunxi_data->idle_ctrl = val ? true : false;
		else
			goto err_out;
	} else if (!strncmp("dvfs", buffer, head_size)) {
		if (val == 0 || val == 1)
			sunxi_data->dvfs_ctrl = val ? true : false;
		else
			goto err_out;
	} else {
		goto err_out;
	}

	return count;

err_out:
	dev_err(s_kbdev->dev, "sunxi gpu invalid parameter:%s!\n", buffer);
	return -EINVAL;
}

static const struct file_operations write_fops = {
	.owner = THIS_MODULE,
	.write = write_write,
};

static int dump_debugfs_show(struct seq_file *s, void *data)
{

	struct kbasep_pm_metrics diff;
	kbase_pm_get_dvfs_metrics(s_kbdev, &sunxi_data->sunxi_last, &diff);
#ifdef CONFIG_REGULATOR
	if (!IS_ERR_OR_NULL(s_kbdev->regulators[0])) {
		int vol = regulator_get_voltage(s_kbdev->regulators[0]);
		seq_printf(s, "voltage:%dmV;\n", vol);
	}
#endif /* CONFIG_REGULATOR */

	seq_printf(s, "idle:%s;\n", sunxi_data->idle_ctrl ? "on" : "off");
	seq_printf(s, "scenectrl:%s;\n",
			sunxi_data->sence_ctrl ? "on" : "off");
	seq_printf(s, "dvfs:%s;\n",
			sunxi_data->dvfs_ctrl ? "on" : "off");
	seq_printf(s, "independent_power:%s;\n",
			sunxi_data->independent_power ? "yes" : "no");
	seq_printf(s, "Frequency:%luMHz;\n", sunxi_data->current_freq/1000/1000);
	seq_printf(s, "Utilisation from last show:%u%%;\n",
		diff.time_busy * 100 / (diff.time_busy + diff.time_idle));

	seq_puts(s, "\n");

	return 0;
}

static int dump_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, dump_debugfs_show, inode->i_private);
}

static const struct file_operations dump_fops = {
	.owner = THIS_MODULE,
	.open = dump_debugfs_open,
	.read  = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int sunxi_create_debugfs(void)
{
	struct dentry *node;

	sunxi_debugfs = debugfs_create_dir("sunxi_gpu", NULL);
	if (IS_ERR_OR_NULL(sunxi_debugfs))
		return -ENOMEM;

	node = debugfs_create_file("write", 0644,
				sunxi_debugfs, NULL, &write_fops);
	if (IS_ERR_OR_NULL(node))
		return -ENOMEM;

	node = debugfs_create_file("dump", 0644,
				sunxi_debugfs, NULL, &dump_fops);
	if (IS_ERR_OR_NULL(node))
		return -ENOMEM;

	return 0;
}
#endif /* CONFIG_DEBUG_FS */
