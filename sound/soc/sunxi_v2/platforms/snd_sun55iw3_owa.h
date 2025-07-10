/* sound\soc\sunxi\snd_sunxi_owa.h
 * (C) Copyright 2021-2025
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * Dby <dby@allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#ifndef __SND_SUN55IW3_OWA_H
#define __SND_SUN55IW3_OWA_H

#define HLOG		"OWA"

struct sunxi_owa_clk {
	/* dependent clk */
	struct clk *clk_peri0_2x;
	struct clk *clk_dsp_src;
	/* parent clk */
	struct clk *clk_pll_audio0_4x;
	struct clk *clk_pll_audio1_div2;
	struct clk *clk_pll_audio1_div5;
	struct clk *clk_pll_peri0_300;
	/* module clk */
	struct clk *clk_owa_tx;
	struct clk *clk_owa_rx;

	unsigned int pll_fs_tx;
	unsigned int pll_fs_rx;

	struct clk *clk_bus;
	struct reset_control *clk_rst;
};

static int snd_sunxi_clk_init(struct platform_device *pdev, struct sunxi_owa_clk *clk);
static void snd_sunxi_clk_exit(struct sunxi_owa_clk *clk);
static int snd_sunxi_clk_enable(struct sunxi_owa_clk *clk);
static void snd_sunxi_clk_disable(struct sunxi_owa_clk *clk);
static int snd_sunxi_clk_rate(struct sunxi_owa_clk *clk, unsigned int freq_in,
			      unsigned int freq_out);

static inline int snd_sunxi_clk_init(struct platform_device *pdev, struct sunxi_owa_clk *clk)
{
	int ret = 0;
	struct device_node *np = pdev->dev.of_node;

	SND_LOG_DEBUG(HLOG, "\n");

	/* get rst clk */
	clk->clk_rst = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR_OR_NULL(clk->clk_rst)) {
		SND_LOG_ERR(HLOG, "clk rst get failed\n");
		ret =  PTR_ERR(clk->clk_rst);
		goto err_get_clk_rst;
	}

	/* get dependent clock */
	clk->clk_peri0_2x = of_clk_get_by_name(np, "clk_pll_peri0_2x");
	if (IS_ERR_OR_NULL(clk->clk_peri0_2x)) {
		SND_LOG_ERR(HLOG, "clk_peri0_2x get failed\n");
		ret = PTR_ERR(clk->clk_peri0_2x);
		goto err_get_clk_peri0_2x;
	}
	clk->clk_dsp_src = of_clk_get_by_name(np, "clk_dsp_src");
	if (IS_ERR_OR_NULL(clk->clk_dsp_src)) {
		SND_LOG_ERR(HLOG, "clk_dsp_src get failed\n");
		ret = PTR_ERR(clk->clk_dsp_src);
		goto err_get_clk_dsp_src;
	}

	/* get bus clk */
	clk->clk_bus = of_clk_get_by_name(np, "clk_bus_owa");
	if (IS_ERR_OR_NULL(clk->clk_bus)) {
		SND_LOG_ERR(HLOG, "clk bus get failed\n");
		ret = PTR_ERR(clk->clk_bus);
		goto err_get_clk_bus;
	}

	/* get parent clk */
	clk->clk_pll_audio0_4x = of_clk_get_by_name(np, "clk_pll_audio0_4x");
	if (IS_ERR_OR_NULL(clk->clk_pll_audio0_4x)) {
		SND_LOG_ERR(HLOG, "clk_pll_audio0_4x get failed\n");
		ret = PTR_ERR(clk->clk_pll_audio0_4x);
		goto err_get_clk_pll_audio0_4x;
	}
	clk->clk_pll_audio1_div2 = of_clk_get_by_name(np, "clk_pll_audio1_div2");
	if (IS_ERR_OR_NULL(clk->clk_pll_audio1_div2)) {
		SND_LOG_ERR(HLOG, "clk_pll_audio1_div2 get failed\n");
		ret = PTR_ERR(clk->clk_pll_audio1_div2);
		goto err_get_clk_pll_audio1_div2;
	}
	clk->clk_pll_audio1_div5 = of_clk_get_by_name(np, "clk_pll_audio1_div5");
	if (IS_ERR_OR_NULL(clk->clk_pll_audio1_div5)) {
		SND_LOG_ERR(HLOG, "clk_pll_audio1_div5 get failed\n");
		ret = PTR_ERR(clk->clk_pll_audio1_div5);
		goto err_get_clk_pll_audio1_div5;
	}
	clk->clk_pll_peri0_300 = of_clk_get_by_name(np, "clk_pll_peri0_300");
	if (IS_ERR_OR_NULL(clk->clk_pll_peri0_300)) {
		SND_LOG_ERR(HLOG, "clk_pll_peri0_300 get failed\n");
		ret = PTR_ERR(clk->clk_pll_peri0_300);
		goto err_get_clk_pll_peri0_300;
	}

	clk->pll_fs_tx = 1;
	clk->pll_fs_rx = 1;

	/* get owa clk */
	clk->clk_owa_tx = of_clk_get_by_name(np, "clk_owa_tx");
	if (IS_ERR_OR_NULL(clk->clk_owa_tx)) {
		SND_LOG_ERR(HLOG, "clk owa tx get failed\n");
		ret = PTR_ERR(clk->clk_owa_tx);
		goto err_get_clk_owa_tx;
	}
	clk->clk_owa_rx = of_clk_get_by_name(np, "clk_owa_rx");
	if (IS_ERR_OR_NULL(clk->clk_owa_rx)) {
		SND_LOG_ERR(HLOG, "clk owa rx get failed\n");
		ret = PTR_ERR(clk->clk_owa_rx);
		goto err_get_clk_owa_rx;
	}

	ret = snd_sunxi_clk_enable(clk);
	if (ret) {
		SND_LOG_ERR(HLOG, "clk enable failed\n");
		ret = -EINVAL;
		goto err_clk_enable;
	}

	return 0;

err_clk_enable:
	clk_put(clk->clk_owa_rx);
err_get_clk_owa_rx:
	clk_put(clk->clk_owa_tx);
err_get_clk_owa_tx:
	clk_put(clk->clk_pll_peri0_300);
err_get_clk_pll_peri0_300:
	clk_put(clk->clk_pll_audio1_div5);
err_get_clk_pll_audio1_div5:
	clk_put(clk->clk_pll_audio1_div2);
err_get_clk_pll_audio1_div2:
	clk_put(clk->clk_pll_audio0_4x);
err_get_clk_pll_audio0_4x:
	clk_put(clk->clk_bus);
err_get_clk_bus:
	clk_put(clk->clk_dsp_src);
err_get_clk_dsp_src:
	clk_put(clk->clk_peri0_2x);
err_get_clk_peri0_2x:
err_get_clk_rst:
	return ret;
}

static inline void snd_sunxi_clk_exit(struct sunxi_owa_clk *clk)
{
	SND_LOG_DEBUG(HLOG, "\n");

	snd_sunxi_clk_disable(clk);
	clk_put(clk->clk_owa_rx);
	clk_put(clk->clk_owa_tx);
	clk_put(clk->clk_pll_peri0_300);
	clk_put(clk->clk_pll_audio1_div5);
	clk_put(clk->clk_pll_audio1_div2);
	clk_put(clk->clk_pll_audio0_4x);
	clk_put(clk->clk_bus);
	clk_put(clk->clk_dsp_src);
	clk_put(clk->clk_peri0_2x);
}

static inline int snd_sunxi_clk_enable(struct sunxi_owa_clk *clk)
{
	int ret = 0;

	SND_LOG_DEBUG(HLOG, "\n");

	if (reset_control_deassert(clk->clk_rst)) {
		SND_LOG_ERR(HLOG, "clk_rst deassert failed\n");
		goto err_deassert_rst;
	}

	if (clk_prepare_enable(clk->clk_peri0_2x)) {
		SND_LOG_ERR(HLOG, "clk_peri0_2x enable failed\n");
		goto err_enable_clk_peri0_2x;
	}
	if (clk_prepare_enable(clk->clk_dsp_src)) {
		SND_LOG_ERR(HLOG, "clk_dsp_src enable failed\n");
		goto err_enable_clk_dsp_src;
	}

	if (clk_prepare_enable(clk->clk_bus)) {
		SND_LOG_ERR(HLOG, "clk_bus enable failed\n");
		goto err_enable_clk_bus;
	}

	if (clk_prepare_enable(clk->clk_pll_audio0_4x)) {
		SND_LOG_ERR(HLOG, "clk_pll_audio0_4x enable failed\n");
		goto err_enable_clk_pll_audio0_4x;
	}
	if (clk_prepare_enable(clk->clk_pll_audio1_div2)) {
		SND_LOG_ERR(HLOG, "clk_pll_audio1_div2 enable failed\n");
		goto err_enable_clk_pll_audio1_div2;
	}
	if (clk_prepare_enable(clk->clk_pll_audio1_div5)) {
		SND_LOG_ERR(HLOG, "clk_pll_audio1_div5 enable failed\n");
		goto err_enable_clk_pll_audio1_div5;
	}
	if (clk_prepare_enable(clk->clk_pll_peri0_300)) {
		SND_LOG_ERR(HLOG, "clk_pll_peri0_300 enable failed\n");
		goto err_enable_clk_pll_peri0_300;
	}

	if (clk_prepare_enable(clk->clk_owa_tx)) {
		SND_LOG_ERR(HLOG, "clk_owa_tx enable failed\n");
		goto err_enable_clk_owa_tx;
	}

	if (clk_prepare_enable(clk->clk_owa_rx)) {
		SND_LOG_ERR(HLOG, "clk_owa_rx enable failed\n");
		goto err_enable_clk_owa_rx;
	}

	return 0;

err_enable_clk_owa_rx:
	clk_disable_unprepare(clk->clk_owa_tx);
err_enable_clk_owa_tx:
	clk_disable_unprepare(clk->clk_pll_peri0_300);
err_enable_clk_pll_peri0_300:
	clk_disable_unprepare(clk->clk_pll_audio1_div5);
err_enable_clk_pll_audio1_div5:
	clk_disable_unprepare(clk->clk_pll_audio1_div2);
err_enable_clk_pll_audio1_div2:
	clk_disable_unprepare(clk->clk_pll_audio0_4x);
err_enable_clk_pll_audio0_4x:
	clk_disable_unprepare(clk->clk_bus);
err_enable_clk_bus:
	clk_disable_unprepare(clk->clk_dsp_src);
err_enable_clk_dsp_src:
	clk_disable_unprepare(clk->clk_peri0_2x);
err_enable_clk_peri0_2x:
	reset_control_assert(clk->clk_rst);
err_deassert_rst:
	return ret;
}

static inline void snd_sunxi_clk_disable(struct sunxi_owa_clk *clk)
{
	SND_LOG_DEBUG(HLOG, "\n");

	clk_disable_unprepare(clk->clk_owa_rx);
	clk_disable_unprepare(clk->clk_owa_tx);
	clk_disable_unprepare(clk->clk_pll_peri0_300);
	clk_disable_unprepare(clk->clk_pll_audio1_div5);
	clk_disable_unprepare(clk->clk_pll_audio1_div2);
	clk_disable_unprepare(clk->clk_pll_audio0_4x);
	clk_disable_unprepare(clk->clk_bus);
	clk_disable_unprepare(clk->clk_dsp_src);
	clk_disable_unprepare(clk->clk_peri0_2x);
	reset_control_assert(clk->clk_rst);
}

static inline int snd_sunxi_clk_rate(struct sunxi_owa_clk *clk, unsigned int freq_in,
				     unsigned int freq_out)
{
	SND_LOG_DEBUG(HLOG, "\n");

	/* set dependent clk */
	if (clk_set_parent(clk->clk_dsp_src, clk->clk_peri0_2x)) {
		SND_LOG_ERR(HLOG, "set clk_dsp_src parent clk failed\n");
		return -EINVAL;
	}
	if (clk_set_rate(clk->clk_dsp_src, 600000000)) {
		SND_LOG_ERR(HLOG, "set clk_dsp_src rate failed, rate\n");
		return -EINVAL;
	}

	if (freq_in % 24576000 == 0) {
		/* If you want to use clk_pll_audio0_4x, must set it 1083801600Hz */
		if (clk_set_parent(clk->clk_owa_tx, clk->clk_pll_audio0_4x)) {
			SND_LOG_ERR(HLOG, "set owa tx parent clk failed\n");
			return -EINVAL;
		}
		if (clk_set_rate(clk->clk_pll_audio0_4x, 1083801600)) {
			SND_LOG_ERR(HLOG, "set clk_pll_audio0_4x rate failed\n");
			return -EINVAL;
		}
	} else {
		if (clk_set_parent(clk->clk_owa_tx, clk->clk_pll_audio1_div5)) {
			SND_LOG_ERR(HLOG, "set owa tx parent clk failed\n");
			return -EINVAL;
		}
	}

	if (clk_set_rate(clk->clk_owa_tx, freq_out)) {
		SND_LOG_ERR(HLOG, "clk owa tx set rate failed\n");
		return -EINVAL;
	}

	if (clk_set_parent(clk->clk_owa_rx, clk->clk_pll_peri0_300)) {
		SND_LOG_ERR(HLOG, "set clk_owa_rx parent clk failed\n");
		return -EINVAL;
	}

	return 0;
}

#endif /* __SND_SUN55IW3_OWA_H */