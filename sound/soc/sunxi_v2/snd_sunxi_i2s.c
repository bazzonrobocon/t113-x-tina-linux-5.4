/*
 * sound\soc\sunxi\snd_sunxi_i2s.c
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
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/device.h>
#include <linux/ioport.h>
#include <linux/regmap.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>

#include "snd_sunxi_log.h"
#include "snd_sunxi_pcm.h"
#include "snd_sunxi_rxsync.h"
#include "snd_sunxi_common.h"
#include "snd_sunxi_i2s.h"

#define HLOG		"I2S"
#define DRV_NAME	"sunxi-snd-plat-i2s"

/* for reg debug */
static struct audio_reg_label sunxi_reg_labels[] = {
	REG_LABEL(SUNXI_I2S_CTL),
	REG_LABEL(SUNXI_I2S_FMT0),
	REG_LABEL(SUNXI_I2S_FMT1),
	REG_LABEL(SUNXI_I2S_INTSTA),
	/* REG_LABEL(SUNXI_I2S_RXFIFO), */
	REG_LABEL(SUNXI_I2S_FIFOCTL),
	REG_LABEL(SUNXI_I2S_FIFOSTA),
	REG_LABEL(SUNXI_I2S_INTCTL),
	/* REG_LABEL(SUNXI_I2S_TXFIFO), */
	REG_LABEL(SUNXI_I2S_CLKDIV),
	REG_LABEL(SUNXI_I2S_TXCNT),
	REG_LABEL(SUNXI_I2S_RXCNT),

	REG_LABEL(SUNXI_I2S_CHCFG),
	REG_LABEL(SUNXI_I2S_TX0CHSEL),
	REG_LABEL(SUNXI_I2S_TX1CHSEL),
	REG_LABEL(SUNXI_I2S_TX2CHSEL),
	REG_LABEL(SUNXI_I2S_TX3CHSEL),
	REG_LABEL(SUNXI_I2S_TX0CHMAP0),
	REG_LABEL(SUNXI_I2S_TX0CHMAP1),
	REG_LABEL(SUNXI_I2S_TX1CHMAP0),
	REG_LABEL(SUNXI_I2S_TX1CHMAP1),
	REG_LABEL(SUNXI_I2S_TX2CHMAP0),
	REG_LABEL(SUNXI_I2S_TX2CHMAP1),
	REG_LABEL(SUNXI_I2S_TX3CHMAP0),
	REG_LABEL(SUNXI_I2S_TX3CHMAP1),
	REG_LABEL(SUNXI_I2S_RXCHSEL),
	REG_LABEL(SUNXI_I2S_RXCHMAP0),
	REG_LABEL(SUNXI_I2S_RXCHMAP1),
	REG_LABEL(SUNXI_I2S_RXCHMAP2),
	REG_LABEL(SUNXI_I2S_RXCHMAP3),

	REG_LABEL(SUNXI_I2S_DEBUG),
	REG_LABEL(SUNXI_I2S_REV),
	REG_LABEL_END,
};

static struct audio_reg_label sun8iw11_reg_labels[] = {
	REG_LABEL(SUNXI_I2S_CTL),
	REG_LABEL(SUNXI_I2S_FMT0),
	REG_LABEL(SUNXI_I2S_FMT1),
	REG_LABEL(SUNXI_I2S_INTSTA),
	/* REG_LABEL(SUNXI_I2S_RXFIFO), */
	REG_LABEL(SUNXI_I2S_FIFOCTL),
	REG_LABEL(SUNXI_I2S_FIFOSTA),
	REG_LABEL(SUNXI_I2S_INTCTL),
	/* REG_LABEL(SUNXI_I2S_TXFIFO), */
	REG_LABEL(SUNXI_I2S_CLKDIV),
	REG_LABEL(SUNXI_I2S_TXCNT),
	REG_LABEL(SUNXI_I2S_RXCNT),

	REG_LABEL(SUNXI_I2S_8SLOT_CHCFG),
	REG_LABEL(SUNXI_I2S_8SLOT_TX0CHSEL),
	REG_LABEL(SUNXI_I2S_8SLOT_TX1CHSEL),
	REG_LABEL(SUNXI_I2S_8SLOT_TX2CHSEL),
	REG_LABEL(SUNXI_I2S_8SLOT_TX3CHSEL),
	REG_LABEL(SUNXI_I2S_8SLOT_TX0CHMAP),
	REG_LABEL(SUNXI_I2S_8SLOT_TX1CHMAP),
	REG_LABEL(SUNXI_I2S_8SLOT_TX2CHMAP),
	REG_LABEL(SUNXI_I2S_8SLOT_TX3CHMAP),
	REG_LABEL(SUNXI_I2S_8SLOT_RXCHSEL),
	REG_LABEL(SUNXI_I2S_8SLOT_RXCHMAP),
	REG_LABEL_END,
};

static struct regmap_config g_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = SUNXI_I2S_MAX_REG,
	.cache_type = REGCACHE_NONE,
};

static int sunxi_get_i2s_dai_fmt(struct sunxi_i2s_dai_fmt *i2s_dai_fmt,
				 enum SUNXI_I2S_DAI_FMT_SEL dai_fmt_sel, unsigned int *val);
static int sunxi_set_i2s_dai_fmt(struct sunxi_i2s_dai_fmt *i2s_dai_fmt,
				 enum SUNXI_I2S_DAI_FMT_SEL dai_fmt_sel, unsigned int val);

static void sunxi_rx_sync_enable(void *data, bool enable);

static int sunxi_i2s_set_ch_en(struct sunxi_i2s *i2s, int stream, unsigned int channels)
{
	struct regmap *regmap = i2s->mem.regmap;
	struct sunxi_i2s_dts *dts = &i2s->dts;
	struct sunxi_i2s_dai_fmt *i2s_dai_fmt = &i2s->i2s_dai_fmt;
	unsigned int slot_en_num;
	uint32_t channels_en_slot[16] = {
		0x0001, 0x0003, 0x0007, 0x000f, 0x001f, 0x003f, 0x007f, 0x00ff,
		0x01ff, 0x03ff, 0x07ff, 0x0fff, 0x1fff, 0x3fff, 0x7fff, 0xffff
	};
	int ret;

	if (IS_ERR_OR_NULL(i2s) || channels < 1 || channels > 16)
		return -EINVAL;

	ret = sunxi_get_i2s_dai_fmt(i2s_dai_fmt, SUNXI_I2S_DAI_SLOT_NUM, &slot_en_num);
	if (ret < 0)
		return -EINVAL;

	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		regmap_update_bits(regmap, SUNXI_I2S_CHCFG, 0xF << TX_SLOT_NUM,
				   (channels - 1) << TX_SLOT_NUM);

		if (dts->tx_pin[0]) {
			regmap_update_bits(regmap, SUNXI_I2S_TX0CHSEL, 0xF << TX_CHSEL,
					   (slot_en_num - 1) << TX_CHSEL);
			regmap_update_bits(regmap, SUNXI_I2S_TX0CHSEL, 0xFFFF << TX_CHEN,
					   channels_en_slot[slot_en_num - 1] << TX_CHEN);
		}
		if (dts->tx_pin[1]) {
			regmap_update_bits(regmap, SUNXI_I2S_TX1CHSEL, 0xF << TX_CHSEL,
					   (slot_en_num - 1) << TX_CHSEL);
			regmap_update_bits(regmap, SUNXI_I2S_TX1CHSEL, 0xFFFF << TX_CHEN,
					   channels_en_slot[slot_en_num - 1] << TX_CHEN);
		}
		if (dts->tx_pin[2]) {
			regmap_update_bits(regmap, SUNXI_I2S_TX2CHSEL, 0xF << TX_CHSEL,
					   (slot_en_num - 1) << TX_CHSEL);
			regmap_update_bits(regmap, SUNXI_I2S_TX2CHSEL, 0xFFFF << TX_CHEN,
					   channels_en_slot[slot_en_num - 1] << TX_CHEN);
		}
		if (dts->tx_pin[3]) {
			regmap_update_bits(regmap, SUNXI_I2S_TX3CHSEL, 0xF << TX_CHSEL,
					   (slot_en_num - 1) << TX_CHSEL);
			regmap_update_bits(regmap, SUNXI_I2S_TX3CHSEL, 0xFFFF << TX_CHEN,
					   channels_en_slot[slot_en_num - 1] << TX_CHEN);
		}
	} else {
		regmap_update_bits(regmap, SUNXI_I2S_CHCFG, 0xF << RX_SLOT_NUM,
				   (channels - 1) << RX_SLOT_NUM);
		regmap_update_bits(regmap, SUNXI_I2S_RXCHSEL, 0xF << RX_CHSEL,
				   (channels - 1) << RX_CHSEL);
	}

	return 0;
}

static int sun8iw11_i2s_set_ch_en(struct sunxi_i2s *i2s, int stream, unsigned int channels)
{
	struct regmap *regmap = i2s->mem.regmap;
	struct sunxi_i2s_dts *dts = &i2s->dts;
	struct sunxi_i2s_dai_fmt *i2s_dai_fmt = &i2s->i2s_dai_fmt;
	unsigned int slot_en_num;
	uint32_t channels_en_slot[8] = {
		0x01, 0x03, 0x07, 0x0f, 0x1f, 0x3f, 0x7f, 0xff,
	};
	int ret;

	if (IS_ERR_OR_NULL(i2s) || channels < 1 || channels > 8)
		return -EINVAL;

	ret = sunxi_get_i2s_dai_fmt(i2s_dai_fmt, SUNXI_I2S_DAI_SLOT_NUM, &slot_en_num);
	if (ret < 0)
		return -EINVAL;

	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		regmap_update_bits(regmap, SUNXI_I2S_CHCFG, 0x7 << TX_SLOT_NUM,
				   (channels - 1) << TX_SLOT_NUM);

		if (dts->tx_pin[0]) {
			regmap_update_bits(regmap, SUNXI_I2S_8SLOT_TX0CHSEL,
					   0x7 << TX_CHSEL_8SLOT,
					   (slot_en_num - 1) << TX_CHSEL_8SLOT);
			regmap_update_bits(regmap, SUNXI_I2S_8SLOT_TX0CHSEL,
					   0xFF << TX_CHEN_8SLOT,
					   channels_en_slot[slot_en_num - 1] << TX_CHEN_8SLOT);
		}
		if (dts->tx_pin[1]) {
			regmap_update_bits(regmap, SUNXI_I2S_8SLOT_TX1CHSEL,
					   0x7 << TX_CHSEL_8SLOT,
					   (slot_en_num - 1) << TX_CHSEL_8SLOT);
			regmap_update_bits(regmap, SUNXI_I2S_8SLOT_TX1CHSEL,
					   0xFF << TX_CHEN_8SLOT,
					   channels_en_slot[slot_en_num - 1] << TX_CHEN_8SLOT);
		}
		if (dts->tx_pin[2]) {
			regmap_update_bits(regmap, SUNXI_I2S_8SLOT_TX2CHSEL,
					   0x7 << TX_CHSEL_8SLOT,
					   (slot_en_num - 1) << TX_CHSEL_8SLOT);
			regmap_update_bits(regmap, SUNXI_I2S_8SLOT_TX2CHSEL,
					   0xFF << TX_CHEN_8SLOT,
					   channels_en_slot[slot_en_num - 1] << TX_CHEN_8SLOT);
		}
		if (dts->tx_pin[3]) {
			regmap_update_bits(regmap, SUNXI_I2S_8SLOT_TX3CHSEL,
					   0x7 << TX_CHSEL_8SLOT,
					   (slot_en_num - 1) << TX_CHSEL_8SLOT);
			regmap_update_bits(regmap, SUNXI_I2S_8SLOT_TX3CHSEL,
					   0xFF << TX_CHEN_8SLOT,
					   channels_en_slot[slot_en_num - 1] << TX_CHEN_8SLOT);
		}
	} else {
		regmap_update_bits(regmap, SUNXI_I2S_8SLOT_CHCFG, 0x7 << RX_SLOT_NUM_8SLOT,
				   (channels - 1) << RX_SLOT_NUM_8SLOT);
		regmap_update_bits(regmap, SUNXI_I2S_8SLOT_RXCHSEL, 0x7 << RX_CHSEL_8SLOT,
				   (channels - 1) << RX_CHSEL_8SLOT);
	}

	return 0;
}

static int sunxi_i2s_set_daifmt_fmt(struct sunxi_i2s *i2s, unsigned int format)
{
	struct regmap *regmap = i2s->mem.regmap;
	unsigned int mode, offset;

	if (IS_ERR_OR_NULL(i2s))
		return -EINVAL;

	switch (format) {
	case SND_SOC_DAIFMT_I2S:
		mode = 1;
		offset = 1;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		mode = 2;
		offset = 0;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		mode = 1;
		offset = 0;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		mode = 0;
		offset = 1;
		/* L data MSB after FRM LRC (short frame) */
		regmap_update_bits(regmap, SUNXI_I2S_FMT0, 1 << LRCK_WIDTH, 0 << LRCK_WIDTH);
		break;
	case SND_SOC_DAIFMT_DSP_B:
		mode = 0;
		offset = 0;
		/* L data MSB during FRM LRC (long frame) */
		regmap_update_bits(regmap, SUNXI_I2S_FMT0, 1 << LRCK_WIDTH, 1 << LRCK_WIDTH);
		break;
	default:
		SND_LOG_ERR(HLOG, "format setting failed\n");
		return -EINVAL;
	}

	regmap_update_bits(regmap, SUNXI_I2S_CTL, 3 << MODE_SEL, mode << MODE_SEL);

	regmap_update_bits(regmap, SUNXI_I2S_TX0CHSEL,
			   3 << TX_OFFSET, offset << TX_OFFSET);
	regmap_update_bits(regmap, SUNXI_I2S_TX1CHSEL,
			   3 << TX_OFFSET, offset << TX_OFFSET);
	regmap_update_bits(regmap, SUNXI_I2S_TX2CHSEL,
			   3 << TX_OFFSET, offset << TX_OFFSET);
	regmap_update_bits(regmap, SUNXI_I2S_TX3CHSEL,
			   3 << TX_OFFSET, offset << TX_OFFSET);
	regmap_update_bits(regmap, SUNXI_I2S_RXCHSEL,
			   3 << RX_OFFSET, offset << RX_OFFSET);

	return 0;
}

static int sun8iw11_i2s_set_daifmt_fmt(struct sunxi_i2s *i2s, unsigned int format)
{
	struct regmap *regmap = i2s->mem.regmap;
	unsigned int mode, offset;

	if (IS_ERR_OR_NULL(i2s))
		return -EINVAL;

	switch (format) {
	case SND_SOC_DAIFMT_I2S:
		mode = 1;
		offset = 1;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		mode = 2;
		offset = 0;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		mode = 1;
		offset = 0;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		mode = 0;
		offset = 1;
		/* L data MSB after FRM LRC (short frame) */
		regmap_update_bits(regmap, SUNXI_I2S_FMT0, 1 << LRCK_WIDTH, 0 << LRCK_WIDTH);
		break;
	case SND_SOC_DAIFMT_DSP_B:
		mode = 0;
		offset = 0;
		/* L data MSB during FRM LRC (long frame) */
		regmap_update_bits(regmap, SUNXI_I2S_FMT0, 1 << LRCK_WIDTH, 1 << LRCK_WIDTH);
		break;
	default:
		SND_LOG_ERR(HLOG, "format setting failed\n");
		return -EINVAL;
	}

	regmap_update_bits(regmap, SUNXI_I2S_CTL, 3 << MODE_SEL, mode << MODE_SEL);

	regmap_update_bits(regmap, SUNXI_I2S_8SLOT_TX0CHSEL, 3 << 12, offset << 12);
	regmap_update_bits(regmap, SUNXI_I2S_8SLOT_TX1CHSEL, 3 << 12, offset << 12);
	regmap_update_bits(regmap, SUNXI_I2S_8SLOT_TX2CHSEL, 3 << 12, offset << 12);
	regmap_update_bits(regmap, SUNXI_I2S_8SLOT_TX3CHSEL, 3 << 12, offset << 12);
	regmap_update_bits(regmap, SUNXI_I2S_8SLOT_RXCHSEL, 3 << 12, offset << 12);

	return 0;
}

static int sunxi_i2s_set_ch_map(struct sunxi_i2s *i2s, unsigned int channels)
{
	struct regmap *regmap = i2s->mem.regmap;
	struct sunxi_i2s_dts *dts = &i2s->dts;
	(void)channels;

	if (IS_ERR_OR_NULL(i2s))
		return -EINVAL;

	/* tx_pin_map[n][0] -> SUNXI_I2S_TXnCHMAP1
	 * tx_pin_map[n][1] -> SUNXI_I2S_TXnCHMAP0
	 */
	regmap_write(regmap, SUNXI_I2S_TX0CHMAP0, dts->tx_pin_map[0][1]);
	regmap_write(regmap, SUNXI_I2S_TX0CHMAP1, dts->tx_pin_map[0][0]);
	regmap_write(regmap, SUNXI_I2S_TX1CHMAP0, dts->tx_pin_map[1][1]);
	regmap_write(regmap, SUNXI_I2S_TX1CHMAP1, dts->tx_pin_map[1][0]);
	regmap_write(regmap, SUNXI_I2S_TX2CHMAP0, dts->tx_pin_map[2][1]);
	regmap_write(regmap, SUNXI_I2S_TX2CHMAP1, dts->tx_pin_map[2][0]);
	regmap_write(regmap, SUNXI_I2S_TX3CHMAP0, dts->tx_pin_map[3][1]);
	regmap_write(regmap, SUNXI_I2S_TX3CHMAP1, dts->tx_pin_map[3][0]);

	regmap_write(regmap, SUNXI_I2S_RXCHMAP0, 0x0F0E0D0C);
	regmap_write(regmap, SUNXI_I2S_RXCHMAP1, 0x0B0A0908);
	regmap_write(regmap, SUNXI_I2S_RXCHMAP2, 0x07060504);
	regmap_write(regmap, SUNXI_I2S_RXCHMAP3, 0x03020100);

	return 0;
}

static int sun8iw11_i2s_set_ch_map(struct sunxi_i2s *i2s, unsigned int channels)
{
	struct regmap *regmap = i2s->mem.regmap;
	struct sunxi_i2s_dts *dts = &i2s->dts;

	if (IS_ERR_OR_NULL(i2s))
		return -EINVAL;

	regmap_write(regmap, SUNXI_I2S_8SLOT_TX0CHMAP, dts->tx_pin_map[0][0]);
	regmap_write(regmap, SUNXI_I2S_8SLOT_TX1CHMAP, dts->tx_pin_map[1][0]);
	regmap_write(regmap, SUNXI_I2S_8SLOT_TX2CHMAP, dts->tx_pin_map[2][0]);
	regmap_write(regmap, SUNXI_I2S_8SLOT_TX3CHMAP, dts->tx_pin_map[3][0]);

	regmap_write(regmap, SUNXI_I2S_8SLOT_RXCHMAP, 0x76543210);

	return 0;
}

static int sunxi_get_i2s_dai_fmt(struct sunxi_i2s_dai_fmt *i2s_dai_fmt,
				 enum SUNXI_I2S_DAI_FMT_SEL dai_fmt_sel,
				 unsigned int *val)
{
	switch (dai_fmt_sel) {
	case SUNXI_I2S_DAI_PLL:
		*val = i2s_dai_fmt->pllclk_freq;
		break;
	case SUNXI_I2S_DAI_MCLK:
		*val = i2s_dai_fmt->moduleclk_freq;
		break;
	case SUNXI_I2S_DAI_FMT:
		*val = i2s_dai_fmt->fmt & SND_SOC_DAIFMT_FORMAT_MASK;
		break;
	case SUNXI_I2S_DAI_MASTER:
		*val = i2s_dai_fmt->fmt & SND_SOC_DAIFMT_MASTER_MASK;
		break;
	case SUNXI_I2S_DAI_INVERT:
		*val = i2s_dai_fmt->fmt & SND_SOC_DAIFMT_INV_MASK;
		break;
	case SUNXI_I2S_DAI_SLOT_NUM:
		*val = i2s_dai_fmt->slots;
		break;
	case SUNXI_I2S_DAI_SLOT_WIDTH:
		*val = i2s_dai_fmt->slot_width;
		break;
	default:
		SND_LOG_ERR(HLOG, "unsupport dai fmt sel %d\n", dai_fmt_sel);
		return -EINVAL;
	}

	return 0;
}

static int sunxi_set_i2s_dai_fmt(struct sunxi_i2s_dai_fmt *i2s_dai_fmt,
				 enum SUNXI_I2S_DAI_FMT_SEL dai_fmt_sel,
				 unsigned int val)
{
	switch (dai_fmt_sel) {
	case SUNXI_I2S_DAI_PLL:
		i2s_dai_fmt->pllclk_freq = val;
		break;
	case SUNXI_I2S_DAI_MCLK:
		i2s_dai_fmt->moduleclk_freq = val;
		break;
	case SUNXI_I2S_DAI_FMT:
		i2s_dai_fmt->fmt &= ~SND_SOC_DAIFMT_FORMAT_MASK;
		i2s_dai_fmt->fmt |= SND_SOC_DAIFMT_FORMAT_MASK & val;
		break;
	case SUNXI_I2S_DAI_MASTER:
		i2s_dai_fmt->fmt &= ~SND_SOC_DAIFMT_MASTER_MASK;
		i2s_dai_fmt->fmt |= SND_SOC_DAIFMT_MASTER_MASK & val;
		break;
	case SUNXI_I2S_DAI_INVERT:
		i2s_dai_fmt->fmt &= ~SND_SOC_DAIFMT_INV_MASK;
		i2s_dai_fmt->fmt |= SND_SOC_DAIFMT_INV_MASK & val;
		break;
	case SUNXI_I2S_DAI_SLOT_NUM:
		i2s_dai_fmt->slots = val;
		break;
	case SUNXI_I2S_DAI_SLOT_WIDTH:
		i2s_dai_fmt->slot_width = val;
		break;
	default:
		SND_LOG_ERR(HLOG, "unsupport dai fmt sel %d\n", dai_fmt_sel);
		return -EINVAL;
	}

	return 0;
}

static void sunxi_sdout_enable(struct regmap *regmap, bool *tx_pin)
{
	/* tx_pin[x] -- x < 4 */
	regmap_update_bits(regmap, SUNXI_I2S_CTL, 1 << (SDO0_EN + 0),
			   tx_pin[0] << (SDO0_EN + 0));
	regmap_update_bits(regmap, SUNXI_I2S_CTL, 1 << (SDO0_EN + 1),
			   tx_pin[1] << (SDO0_EN + 1));
	regmap_update_bits(regmap, SUNXI_I2S_CTL, 1 << (SDO0_EN + 2),
			   tx_pin[2] << (SDO0_EN + 2));
	regmap_update_bits(regmap, SUNXI_I2S_CTL, 1 << (SDO0_EN + 3),
			   tx_pin[3] << (SDO0_EN + 3));
}

static void sunxi_sdout_disable(struct regmap *regmap)
{
	regmap_update_bits(regmap, SUNXI_I2S_CTL, 1 << (SDO0_EN + 0), 0 << (SDO0_EN + 0));
	regmap_update_bits(regmap, SUNXI_I2S_CTL, 1 << (SDO0_EN + 1), 0 << (SDO0_EN + 0));
	regmap_update_bits(regmap, SUNXI_I2S_CTL, 1 << (SDO0_EN + 2), 0 << (SDO0_EN + 0));
	regmap_update_bits(regmap, SUNXI_I2S_CTL, 1 << (SDO0_EN + 3), 0 << (SDO0_EN + 0));
}

static int sunxi_i2s_dai_set_pll(struct snd_soc_dai *dai, int pll_id, int source,
				 unsigned int freq_in, unsigned int freq_out)
{
	struct sunxi_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	struct sunxi_i2s_clk *clk = &i2s->clk;
	struct sunxi_i2s_dai_fmt *i2s_dai_fmt = &i2s->i2s_dai_fmt;
	int ret;

	SND_LOG_DEBUG(HLOG, "\n");

	if (snd_sunxi_clk_rate(clk, freq_in, freq_out)) {
		SND_LOG_ERR(HLOG, "clk set rate failed\n");
		return -EINVAL;
	}

	ret = sunxi_set_i2s_dai_fmt(i2s_dai_fmt, SUNXI_I2S_DAI_PLL, freq_in);
	if (ret < 0)
		return -EINVAL;
	ret = sunxi_set_i2s_dai_fmt(i2s_dai_fmt, SUNXI_I2S_DAI_MCLK, freq_out);
	if (ret < 0)
		return -EINVAL;

	return 0;
}

static int sunxi_i2s_dai_set_sysclk(struct snd_soc_dai *dai, int clk_id, unsigned int freq, int dir)
{
	struct sunxi_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	struct regmap *regmap = i2s->mem.regmap;
	struct sunxi_i2s_dai_fmt *i2s_dai_fmt = &i2s->i2s_dai_fmt;
	unsigned int pllclk_freq, mclk_ratio, mclk_ratio_map;
	int ret;

	SND_LOG_DEBUG(HLOG, "\n");

	if (freq == 0) {
		regmap_update_bits(regmap, SUNXI_I2S_CLKDIV, 1 << MCLKOUT_EN, 0 << MCLKOUT_EN);
		return 0;
	}

	ret = sunxi_get_i2s_dai_fmt(i2s_dai_fmt, SUNXI_I2S_DAI_PLL, &pllclk_freq);
	if (ret < 0)
		return -EINVAL;

	if (pllclk_freq == 0) {
		SND_LOG_ERR(HLOG, "pllclk freq is invalid\n");
		return -ENOMEM;
	}
	mclk_ratio = pllclk_freq / freq;

	switch (mclk_ratio) {
	case 1:
		mclk_ratio_map = 1;
		break;
	case 2:
		mclk_ratio_map = 2;
		break;
	case 4:
		mclk_ratio_map = 3;
		break;
	case 6:
		mclk_ratio_map = 4;
		break;
	case 8:
		mclk_ratio_map = 5;
		break;
	case 12:
		mclk_ratio_map = 6;
		break;
	case 16:
		mclk_ratio_map = 7;
		break;
	case 24:
		mclk_ratio_map = 8;
		break;
	case 32:
		mclk_ratio_map = 9;
		break;
	case 48:
		mclk_ratio_map = 10;
		break;
	case 64:
		mclk_ratio_map = 11;
		break;
	case 96:
		mclk_ratio_map = 12;
		break;
	case 128:
		mclk_ratio_map = 13;
		break;
	case 176:
		mclk_ratio_map = 14;
		break;
	case 192:
		mclk_ratio_map = 15;
		break;
	default:
		regmap_update_bits(regmap, SUNXI_I2S_CLKDIV, 1 << MCLKOUT_EN, 0 << MCLKOUT_EN);
		SND_LOG_ERR(HLOG, "mclk freq div unsupport\n");
		return -EINVAL;
	}

	regmap_update_bits(regmap, SUNXI_I2S_CLKDIV,
			   0xf << MCLK_DIV, mclk_ratio_map << MCLK_DIV);
	regmap_update_bits(regmap, SUNXI_I2S_CLKDIV, 1 << MCLKOUT_EN, 1 << MCLKOUT_EN);

	return 0;
}

static int sunxi_i2s_dai_set_bclk_ratio(struct snd_soc_dai *dai, unsigned int ratio)
{
	struct sunxi_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	struct regmap *regmap = i2s->mem.regmap;
	unsigned int bclk_ratio;

	SND_LOG_DEBUG(HLOG, "\n");

	/* ratio -> cpudai pllclk / pcm rate */
	switch (ratio) {
	case 1:
		bclk_ratio = 1;
		break;
	case 2:
		bclk_ratio = 2;
		break;
	case 4:
		bclk_ratio = 3;
		break;
	case 6:
		bclk_ratio = 4;
		break;
	case 8:
		bclk_ratio = 5;
		break;
	case 12:
		bclk_ratio = 6;
		break;
	case 16:
		bclk_ratio = 7;
		break;
	case 24:
		bclk_ratio = 8;
		break;
	case 32:
		bclk_ratio = 9;
		break;
	case 48:
		bclk_ratio = 10;
		break;
	case 64:
		bclk_ratio = 11;
		break;
	case 96:
		bclk_ratio = 12;
		break;
	case 128:
		bclk_ratio = 13;
		break;
	case 176:
		bclk_ratio = 14;
		break;
	case 192:
		bclk_ratio = 15;
		break;
	default:
		SND_LOG_ERR(HLOG, "bclk freq div unsupport\n");
		return -EINVAL;
	}

	regmap_update_bits(regmap, SUNXI_I2S_CLKDIV, 0xf << BCLK_DIV, bclk_ratio << BCLK_DIV);

	return 0;
}

static int sunxi_i2s_dai_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct sunxi_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	const struct sunxi_i2s_quirks *quirks = i2s->quirks;
	struct regmap *regmap = i2s->mem.regmap;
	struct sunxi_i2s_dai_fmt *i2s_dai_fmt = &i2s->i2s_dai_fmt;
	unsigned int lrck_polarity, bclk_polarity;
	/* dai mode of i2s format
	 * I2S/RIGHT_J/LEFT_J	-> 0
	 * DSP_A/DSP_B		-> 1
	 */
	unsigned int dai_mode;
	int ret;

	SND_LOG_DEBUG(HLOG, "dai fmt -> 0x%x\n", fmt);

	ret = sunxi_set_i2s_dai_fmt(i2s_dai_fmt, SUNXI_I2S_DAI_FMT, fmt);
	if (ret < 0)
		return -EINVAL;
	ret = sunxi_set_i2s_dai_fmt(i2s_dai_fmt, SUNXI_I2S_DAI_MASTER, fmt);
	if (ret < 0)
		return -EINVAL;
	ret = sunxi_set_i2s_dai_fmt(i2s_dai_fmt, SUNXI_I2S_DAI_INVERT, fmt);
	if (ret < 0)
		return -EINVAL;

	/* get dai mode */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
	case SND_SOC_DAIFMT_RIGHT_J:
	case SND_SOC_DAIFMT_LEFT_J:
		dai_mode = 0;
		break;
	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_DSP_B:
		dai_mode = 1;
		break;
	default:
		SND_LOG_ERR(HLOG, "dai_mode setting failed\n");
		return -EINVAL;
	}

	/* set TDM format */
	ret = quirks->set_daifmt_format(i2s, fmt & SND_SOC_DAIFMT_FORMAT_MASK);
	if (ret < 0) {
		SND_LOG_ERR(HLOG, "set daifmt format failed\n");
		return -EINVAL;
	}

	/* set lrck & bclk polarity */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		lrck_polarity = 0;
		bclk_polarity = 0;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		lrck_polarity = 1;
		bclk_polarity = 0;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		lrck_polarity = 0;
		bclk_polarity = 1;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		lrck_polarity = 1;
		bclk_polarity = 1;
		break;
	default:
		SND_LOG_ERR(HLOG, "invert clk setting failed\n");
		return -EINVAL;
	}

	/* lrck polarity of i2s format
	 * LRCK_POLARITY	-> 0
	 * Left channel when LRCK is low(I2S/RIGHT_J/LEFT_J);
	 * PCM LRCK asserted at the negative edge(DSP_A/DSP_B);
	 * LRCK_POLARITY	-> 1
	 * Left channel when LRCK is high(I2S/RIGHT_J/LEFT_J);
	 * PCM LRCK asserted at the positive edge(DSP_A/DSP_B);
	 */
	lrck_polarity ^= dai_mode;

	regmap_update_bits(regmap, SUNXI_I2S_FMT0,
			   1 << LRCK_POLARITY,
			   lrck_polarity << LRCK_POLARITY);
	regmap_update_bits(regmap, SUNXI_I2S_FMT0,
			   1 << BCLK_POLARITY,
			   bclk_polarity << BCLK_POLARITY);

	/* set master/slave */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	/* bclk & lrck dir input */
	case SND_SOC_DAIFMT_CBM_CFM:
		regmap_update_bits(regmap, SUNXI_I2S_CTL, 1 << BCLK_OUT, 0 << BCLK_OUT);
		regmap_update_bits(regmap, SUNXI_I2S_CTL, 1 << LRCK_OUT, 0 << LRCK_OUT);
		break;
	case SND_SOC_DAIFMT_CBS_CFM:
		regmap_update_bits(regmap, SUNXI_I2S_CTL, 1 << BCLK_OUT, 1 << BCLK_OUT);
		regmap_update_bits(regmap, SUNXI_I2S_CTL, 1 << LRCK_OUT, 0 << LRCK_OUT);
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
		regmap_update_bits(regmap, SUNXI_I2S_CTL, 1 << BCLK_OUT, 0 << BCLK_OUT);
		regmap_update_bits(regmap, SUNXI_I2S_CTL, 1 << LRCK_OUT, 1 << LRCK_OUT);
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		regmap_update_bits(regmap, SUNXI_I2S_CTL, 1 << BCLK_OUT, 1 << BCLK_OUT);
		regmap_update_bits(regmap, SUNXI_I2S_CTL, 1 << LRCK_OUT, 1 << LRCK_OUT);
		break;
	default:
		SND_LOG_ERR(HLOG, "unknown master/slave format\n");
		return -EINVAL;
	}

	return 0;
}

static int sunxi_i2s_dai_set_tdm_slot(struct snd_soc_dai *dai,
				      unsigned int tx_mask, unsigned int rx_mask,
				      int slots, int slot_width)
{
	struct sunxi_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	struct regmap *regmap = i2s->mem.regmap;
	struct sunxi_i2s_dai_fmt *i2s_dai_fmt = &i2s->i2s_dai_fmt;
	unsigned int slot_width_map, lrck_width_map;
	unsigned int dai_fmt_get;
	int ret;

	SND_LOG_DEBUG(HLOG, "\n");

	switch (slot_width) {
	case 8:
		slot_width_map = 1;
		break;
	case 12:
		slot_width_map = 2;
		break;
	case 16:
		slot_width_map = 3;
		break;
	case 20:
		slot_width_map = 4;
		break;
	case 24:
		slot_width_map = 5;
		break;
	case 28:
		slot_width_map = 6;
		break;
	case 32:
		slot_width_map = 7;
		break;
	default:
		SND_LOG_ERR(HLOG, "unknown slot width\n");
		return -EINVAL;
	}
	regmap_update_bits(regmap, SUNXI_I2S_FMT0,
			   7 << SLOT_WIDTH, slot_width_map << SLOT_WIDTH);

	/* bclk num of per channel
	 * I2S/RIGHT_J/LEFT_J	-> lrck long total is lrck_width_map * 2
	 * DSP_A/DSP_B		-> lrck long total is lrck_width_map * 1
	 */
	ret = sunxi_get_i2s_dai_fmt(i2s_dai_fmt, SUNXI_I2S_DAI_FMT, &dai_fmt_get);
	if (ret < 0)
		return -EINVAL;
	switch (dai_fmt_get) {
	case SND_SOC_DAIFMT_I2S:
	case SND_SOC_DAIFMT_RIGHT_J:
	case SND_SOC_DAIFMT_LEFT_J:
		lrck_width_map = (slots / 2) * slot_width - 1;
		break;
	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_DSP_B:
		lrck_width_map = slots * slot_width - 1;
		break;
	default:
		SND_LOG_ERR(HLOG, "unsupoort format\n");
		return -EINVAL;
	}
	regmap_update_bits(regmap, SUNXI_I2S_FMT0,
			   0x3ff << LRCK_PERIOD, lrck_width_map << LRCK_PERIOD);

	ret = sunxi_set_i2s_dai_fmt(i2s_dai_fmt, SUNXI_I2S_DAI_SLOT_NUM, slots);
	if (ret < 0)
		return -EINVAL;
	ret = sunxi_set_i2s_dai_fmt(i2s_dai_fmt, SUNXI_I2S_DAI_SLOT_WIDTH, slot_width);
	if (ret < 0)
		return -EINVAL;

	return 0;
}

static int sunxi_i2s_dai_startup(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct sunxi_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	const struct sunxi_i2s_quirks *quirks = i2s->quirks;
	struct sunxi_i2s_dts *dts = &i2s->dts;

	SND_LOG_DEBUG(HLOG, "\n");

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		snd_soc_dai_set_dma_data(dai, substream, &i2s->playback_dma_param);
	} else {
		snd_soc_dai_set_dma_data(dai, substream, &i2s->capture_dma_param);
		if (quirks->rx_sync_en && dts->rx_sync_en && dts->rx_sync_ctl)
			sunxi_rx_sync_startup(dts->rx_sync_domain, dts->rx_sync_id);
	}

	return 0;
}

static void sunxi_i2s_dai_shutdown(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct sunxi_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	const struct sunxi_i2s_quirks *quirks = i2s->quirks;
	struct sunxi_i2s_dts *dts = &i2s->dts;

	SND_LOG_DEBUG(HLOG, "\n");

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		if (quirks->rx_sync_en && dts->rx_sync_en && dts->rx_sync_ctl)
			sunxi_rx_sync_shutdown(dts->rx_sync_domain, dts->rx_sync_id);
	}

	return;
}

static int sunxi_i2s_dai_hw_params(struct snd_pcm_substream *substream,
				   struct snd_pcm_hw_params *params,
				   struct snd_soc_dai *dai)
{
	struct sunxi_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	const struct sunxi_i2s_quirks *quirks = i2s->quirks;
	struct regmap *regmap = i2s->mem.regmap;
	int ret;

	SND_LOG_DEBUG(HLOG, "\n");

	if (i2s->dts.dai_type == SUNXI_DAI_HDMI_TYPE) {
		i2s->hdmi_fmt = snd_sunxi_hdmi_get_fmt();
		SND_LOG_DEBUG(HLOG, "hdmi fmt -> %d\n", i2s->hdmi_fmt);
	}

	/* set bits */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			if (i2s->dts.dai_type == SUNXI_DAI_HDMI_TYPE &&
			    i2s->hdmi_fmt > HDMI_FMT_PCM) {
				regmap_update_bits(regmap, SUNXI_I2S_FMT0,
						   0x7 << I2S_SAMPLE_RESOLUTION,
						   0x5 << I2S_SAMPLE_RESOLUTION);
				regmap_update_bits(regmap, SUNXI_I2S_FIFOCTL,
						   0x1 << TXIM, 0x0 << TXIM);
			} else {
				regmap_update_bits(regmap, SUNXI_I2S_FMT0,
						   0x7 << I2S_SAMPLE_RESOLUTION,
						   0x3 << I2S_SAMPLE_RESOLUTION);
				regmap_update_bits(regmap, SUNXI_I2S_FIFOCTL,
						   0x1 << TXIM, 0x1 << TXIM);
			}
		} else {
			regmap_update_bits(regmap, SUNXI_I2S_FMT0,
					   0x7 << I2S_SAMPLE_RESOLUTION,
					   0x3 << I2S_SAMPLE_RESOLUTION);
			regmap_update_bits(regmap, SUNXI_I2S_FIFOCTL, 0x3 << RXOM, 0x1 << RXOM);
		}
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
	case SNDRV_PCM_FORMAT_S24_3LE:
		regmap_update_bits(regmap, SUNXI_I2S_FMT0,
				   0x7 << I2S_SAMPLE_RESOLUTION,
				   0x5 << I2S_SAMPLE_RESOLUTION);

		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			regmap_update_bits(regmap, SUNXI_I2S_FIFOCTL, 0x1 << TXIM, 0x1 << TXIM);
		} else {
			regmap_update_bits(regmap, SUNXI_I2S_FIFOCTL, 0x3 << RXOM, 0x1 << RXOM);
		}
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_S32_LE:
		regmap_update_bits(regmap, SUNXI_I2S_FMT0,
				   0x7 << I2S_SAMPLE_RESOLUTION,
				   0x7 << I2S_SAMPLE_RESOLUTION);

		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			regmap_update_bits(regmap, SUNXI_I2S_FIFOCTL, 0x1 << TXIM, 0x1 << TXIM);
		} else {
			regmap_update_bits(regmap, SUNXI_I2S_FIFOCTL, 0x3 << RXOM, 0x1 << RXOM);
		}
		break;
	default:
		SND_LOG_ERR(HLOG, "unrecognized format\n");
		return -EINVAL;
	}

	/* set channels map */
	ret = quirks->set_channels_map(i2s, params_channels(params));

	/* set channels */
	ret = quirks->set_channel_enable(i2s, substream->stream, params_channels(params));
	if (ret < 0) {
		SND_LOG_ERR(HLOG, "set channel enable failed\n");
		return -EINVAL;
	}

	return 0;
}

static int sunxi_i2s_dai_prepare(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct sunxi_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	struct regmap *regmap = i2s->mem.regmap;
	unsigned int i;

	SND_LOG_DEBUG(HLOG, "\n");

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		for (i = 0 ; i < 10 ; i++) {
			regmap_update_bits(regmap, SUNXI_I2S_FIFOCTL,
					   1 << FIFO_CTL_FTX, 1 << FIFO_CTL_FTX);
			mdelay(1);
		}
		regmap_write(regmap, SUNXI_I2S_TXCNT, 0);
	} else {
		regmap_update_bits(regmap, SUNXI_I2S_FIFOCTL,
				   1 << FIFO_CTL_FRX, 1 << FIFO_CTL_FRX);
		regmap_write(regmap, SUNXI_I2S_RXCNT, 0);
	}

	return 0;
}

static void sunxi_i2s_dai_tx_route(struct sunxi_i2s *i2s, bool enable)
{
	struct sunxi_i2s_dts *dts = &i2s->dts;
	struct regmap *regmap = i2s->mem.regmap;
	unsigned int reg_val;

	if (enable) {
		regmap_update_bits(regmap, SUNXI_I2S_INTCTL, 1 << TXDRQEN, 1 << TXDRQEN);
		sunxi_sdout_enable(regmap, dts->tx_pin);
		regmap_update_bits(regmap, SUNXI_I2S_CTL, 1 << CTL_TXEN, 1 << CTL_TXEN);
	} else {
		regmap_update_bits(regmap, SUNXI_I2S_INTCTL, 1 << TXDRQEN, 0 << TXDRQEN);

		/* add this to avoid the i2s pop */
		while (1) {
			regmap_update_bits(regmap, SUNXI_I2S_FIFOCTL,
					   1 << FIFO_CTL_FTX, 1 << FIFO_CTL_FTX);
			regmap_write(regmap, SUNXI_I2S_TXCNT, 0);
			regmap_read(regmap, SUNXI_I2S_FIFOSTA, &reg_val);
			reg_val = ((reg_val & 0xFF0000) >> 16);
			if (reg_val == 0x80)
				break;
		}
		udelay(250);

		regmap_update_bits(regmap, SUNXI_I2S_CTL, 1 << CTL_TXEN, 0 << CTL_TXEN);
		sunxi_sdout_disable(regmap);
	}
}

static void sunxi_i2s_dai_rx_route(struct sunxi_i2s *i2s, bool enable)
{
	struct regmap *regmap = i2s->mem.regmap;

	if (enable) {
		regmap_update_bits(regmap, SUNXI_I2S_CTL, 1 << CTL_RXEN, 1 << CTL_RXEN);
		regmap_update_bits(regmap, SUNXI_I2S_INTCTL, 1 << RXDRQEN, 1 << RXDRQEN);
	} else {
		regmap_update_bits(regmap, SUNXI_I2S_INTCTL, 1 << RXDRQEN, 0 << RXDRQEN);
		regmap_update_bits(regmap, SUNXI_I2S_CTL, 1 << CTL_RXEN, 0 << CTL_RXEN);
	}
}

static int sunxi_i2s_dai_trigger(struct snd_pcm_substream *substream,
				 int cmd, struct snd_soc_dai *dai)
{
	struct sunxi_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	const struct sunxi_i2s_quirks *quirks = i2s->quirks;
	struct sunxi_i2s_dts *dts = &i2s->dts;

	SND_LOG_DEBUG(HLOG, "\n");

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			sunxi_i2s_dai_tx_route(i2s, true);
		} else {
			sunxi_i2s_dai_rx_route(i2s, true);
			if (dts->rx_sync_en && dts->rx_sync_ctl && quirks->rx_sync_en)
				sunxi_rx_sync_control(dts->rx_sync_domain, dts->rx_sync_id, true);
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			sunxi_i2s_dai_tx_route(i2s, false);
		} else {
			sunxi_i2s_dai_rx_route(i2s, false);
			if (dts->rx_sync_en && dts->rx_sync_ctl && quirks->rx_sync_en)
				sunxi_rx_sync_control(dts->rx_sync_domain, dts->rx_sync_id, false);
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct snd_soc_dai_ops sunxi_i2s_dai_ops = {
	/* call by machine */
	.set_pll	= sunxi_i2s_dai_set_pll,		/* set pllclk */
	.set_sysclk	= sunxi_i2s_dai_set_sysclk,		/* set mclk */
	.set_bclk_ratio	= sunxi_i2s_dai_set_bclk_ratio,	/* set bclk freq */
	.set_fmt	= sunxi_i2s_dai_set_fmt,		/* set tdm fmt */
	.set_tdm_slot	= sunxi_i2s_dai_set_tdm_slot,	/* set slot num and width */
	/* call by asoc */
	.startup	= sunxi_i2s_dai_startup,
	.hw_params	= sunxi_i2s_dai_hw_params,
	.prepare	= sunxi_i2s_dai_prepare,
	.trigger	= sunxi_i2s_dai_trigger,
	.shutdown	= sunxi_i2s_dai_shutdown,
};

static int sunxi_i2s_init(struct sunxi_i2s *i2s)
{
	const struct sunxi_i2s_quirks *quirks = i2s->quirks;
	struct sunxi_i2s_dts *dts = &i2s->dts;
	struct regmap *regmap = i2s->mem.regmap;

	regmap_update_bits(regmap, SUNXI_I2S_FMT1, 1 << TX_MLS, dts->tx_lsb_first << TX_MLS);
	regmap_update_bits(regmap, SUNXI_I2S_FMT1, 1 << RX_MLS, dts->rx_lsb_first << RX_MLS);
	regmap_update_bits(regmap, SUNXI_I2S_FMT1, 3 << SEXT, 0 << SEXT);
	regmap_update_bits(regmap, SUNXI_I2S_FMT1, 3 << TX_PDM, 0 << TX_PDM);
	regmap_update_bits(regmap, SUNXI_I2S_FMT1, 3 << RX_PDM, 0 << RX_PDM);

	if (quirks->rx_sync_en)
		regmap_update_bits(regmap, SUNXI_I2S_CTL, 1 << RX_SYNC_EN, 0 << RX_SYNC_EN);

	regmap_update_bits(regmap, SUNXI_I2S_CTL, 1 << GLOBAL_EN, 1 << GLOBAL_EN);

	return 0;
}

static int sunxi_i2s_dai_probe(struct snd_soc_dai *dai)
{
	struct sunxi_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	SND_LOG_DEBUG(HLOG, "\n");

	/* pcm_new will using the dma_param about the cma and fifo params. */
	snd_soc_dai_init_dma_data(dai,
				  &i2s->playback_dma_param,
				  &i2s->capture_dma_param);

	sunxi_i2s_init(i2s);

	return 0;
}

static int sunxi_i2s_dai_remove(struct snd_soc_dai *dai)
{
	struct sunxi_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	struct regmap *regmap = i2s->mem.regmap;

	SND_LOG_DEBUG(HLOG, "\n");

	regmap_update_bits(regmap, SUNXI_I2S_CTL, 0x1 << GLOBAL_EN, 0x0 << GLOBAL_EN);

	return 0;
}

static struct snd_soc_dai_driver sunxi_i2s_dai = {
	.name = DRV_NAME,
	.probe		= sunxi_i2s_dai_probe,
	.remove		= sunxi_i2s_dai_remove,
	.playback = {
		.stream_name	= "Playback",
		.channels_min	= 1,
		.channels_max	= 16,
		.rates		= SNDRV_PCM_RATE_8000_192000
				| SNDRV_PCM_RATE_KNOT,
		.formats	= SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S20_3LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S24_3LE
				| SNDRV_PCM_FMTBIT_S32_LE,
	},
	.capture = {
		.stream_name	= "Capture",
		.channels_min	= 1,
		.channels_max	= 16,
		.rates		= SNDRV_PCM_RATE_8000_192000
				| SNDRV_PCM_RATE_KNOT,
		.formats	= SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S20_3LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S24_3LE
				| SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops = &sunxi_i2s_dai_ops,
};

/*******************************************************************************
 * *** sound card & component function source ***
 * @0 sound card probe
 * @1 component function kcontrol register
 ******************************************************************************/
static void sunxi_rx_sync_enable(void *data, bool enable)
{
	struct regmap *regmap = data;

	SND_LOG_DEBUG(HLOG, "%s\n", enable ? "on" : "off");

	if (enable) {
		regmap_update_bits(regmap, SUNXI_I2S_CTL,
				   0x1 << RX_SYNC_EN_START, 0x1 << RX_SYNC_EN_START);
	} else {
		regmap_update_bits(regmap, SUNXI_I2S_CTL,
				   0x1 << RX_SYNC_EN_START, 0x0 << RX_SYNC_EN_START);
	}

	return;
}

static int sunxi_get_tx_hub_mode(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct sunxi_i2s *i2s = snd_soc_component_get_drvdata(component);
	struct regmap *regmap = i2s->mem.regmap;

	unsigned int reg_val;

	regmap_read(regmap, SUNXI_I2S_FIFOCTL, &reg_val);

	ucontrol->value.integer.value[0] = ((reg_val & (0x1 << HUB_EN)) ? 1 : 0);

	return 0;
}

static int sunxi_set_tx_hub_mode(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct sunxi_i2s *i2s = snd_soc_component_get_drvdata(component);
	struct sunxi_i2s_dts *dts = &i2s->dts;
	struct regmap *regmap = i2s->mem.regmap;

	switch (ucontrol->value.integer.value[0]) {
	case 0:
		regmap_update_bits(regmap, SUNXI_I2S_CTL, 1 << CTL_TXEN, 0 << CTL_TXEN);
		sunxi_sdout_disable(regmap);
		regmap_update_bits(regmap, SUNXI_I2S_FIFOCTL, 1 << HUB_EN, 0 << HUB_EN);
		break;
	case 1:
		regmap_update_bits(regmap, SUNXI_I2S_FIFOCTL, 1 << HUB_EN, 1 << HUB_EN);
		regmap_update_bits(regmap, SUNXI_I2S_CTL, 1 << CTL_TXEN, 1 << CTL_TXEN);
		sunxi_sdout_enable(regmap, dts->tx_pin);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int sunxi_get_rx_sync_mode(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct sunxi_i2s *i2s = snd_soc_component_get_drvdata(component);
	struct sunxi_i2s_dts *dts = &i2s->dts;

	ucontrol->value.integer.value[0] = dts->rx_sync_ctl;

	return 0;
}

static int sunxi_set_rx_sync_mode(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct sunxi_i2s *i2s = snd_soc_component_get_drvdata(component);
	struct sunxi_i2s_dts *dts = &i2s->dts;
	struct regmap *regmap = i2s->mem.regmap;

	switch (ucontrol->value.integer.value[0]) {
	case 0:
		dts->rx_sync_ctl = 0;
		regmap_update_bits(regmap, SUNXI_I2S_CTL, 1 << RX_SYNC_EN, 0 << RX_SYNC_EN);
		break;
	case 1:
		regmap_update_bits(regmap, SUNXI_I2S_CTL, 1 << RX_SYNC_EN, 1 << RX_SYNC_EN);
		dts->rx_sync_ctl = 1;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static const char *sunxi_switch_text[] = {"Off", "On"};

static SOC_ENUM_SINGLE_EXT_DECL(sunxi_tx_hub_mode_enum, sunxi_switch_text);
static SOC_ENUM_SINGLE_EXT_DECL(sunxi_rx_sync_mode_enum, sunxi_switch_text);
static const struct snd_kcontrol_new sunxi_tx_hub_controls[] = {
	SOC_ENUM_EXT("tx hub mode", sunxi_tx_hub_mode_enum,
		     sunxi_get_tx_hub_mode, sunxi_set_tx_hub_mode),
};
static const struct snd_kcontrol_new sunxi_rx_sync_controls[] = {
	SOC_ENUM_EXT("rx sync mode", sunxi_rx_sync_mode_enum,
		     sunxi_get_rx_sync_mode, sunxi_set_rx_sync_mode),
};
static const struct snd_kcontrol_new sunxi_i2s_controls[] = {
	SOC_SINGLE("loopback debug", SUNXI_I2S_CTL, LOOP_EN, 1, 0),
};

static int sunxi_i2s_component_probe(struct snd_soc_component *component)
{
	struct sunxi_i2s *i2s = snd_soc_component_get_drvdata(component);
	const struct sunxi_i2s_quirks *quirks = i2s->quirks;
	struct sunxi_i2s_dts *dts = &i2s->dts;
	struct regmap *regmap = i2s->mem.regmap;
	int ret;

	SND_LOG_DEBUG(HLOG, "\n");

	/* component kcontrols -> tx_hub */
	if (dts->tx_hub_en) {
		ret = snd_soc_add_component_controls(component, sunxi_tx_hub_controls,
						     ARRAY_SIZE(sunxi_tx_hub_controls));
		if (ret)
			SND_LOG_ERR(HLOG, "add tx_hub kcontrols failed\n");
	}

	/* component kcontrols -> rx_sync */
	if (quirks->rx_sync_en && dts->rx_sync_en) {
		ret = snd_soc_add_component_controls(component, sunxi_rx_sync_controls,
						     ARRAY_SIZE(sunxi_rx_sync_controls));
		if (ret)
			SND_LOG_ERR(HLOG, "add rx_sync kcontrols failed\n");

		dts->rx_sync_ctl = false;
		dts->rx_sync_domain = RX_SYNC_SYS_DOMAIN;
		dts->rx_sync_id = sunxi_rx_sync_probe(dts->rx_sync_domain);
		if (dts->rx_sync_id < 0) {
			SND_LOG_ERR(HLOG, "sunxi_rx_sync_probe failed\n");
		} else {
			SND_LOG_DEBUG(HLOG, "sunxi_rx_sync_probe successful. domain=%d, id=%d\n",
				      dts->rx_sync_domain, dts->rx_sync_id);
			ret = sunxi_rx_sync_register_cb(dts->rx_sync_domain, dts->rx_sync_id,
							(void *)regmap, sunxi_rx_sync_enable);
			if (ret)
				SND_LOG_ERR(HLOG, "callback register failed\n");
		}
	}

	return 0;
}

static void sunxi_i2s_component_remove(struct snd_soc_component *component)
{
	struct sunxi_i2s *i2s = snd_soc_component_get_drvdata(component);
	const struct sunxi_i2s_quirks *quirks = i2s->quirks;
	struct sunxi_i2s_dts *dts = &i2s->dts;
	SND_LOG_DEBUG(HLOG, "\n");

	if (quirks->rx_sync_en && dts->rx_sync_en)
		sunxi_rx_sync_unregister_cb(dts->rx_sync_domain, dts->rx_sync_id);
}

static int sunxi_i2s_component_suspend(struct snd_soc_component *component)
{
	struct sunxi_i2s *i2s = snd_soc_component_get_drvdata(component);
	const struct sunxi_i2s_quirks *quirks = i2s->quirks;
	struct sunxi_i2s_clk *clk = &i2s->clk;
	struct regmap *regmap = i2s->mem.regmap;

	SND_LOG_DEBUG(HLOG, "\n");

	/* save reg value */
	snd_sunxi_save_reg(regmap, quirks->reg_labels);

	/* disable clk & regulator */
	snd_sunxi_regulator_disable(i2s->rglt);
	snd_sunxi_clk_disable(clk);

	return 0;
}

static int sunxi_i2s_component_resume(struct snd_soc_component *component)
{
	struct sunxi_i2s *i2s = snd_soc_component_get_drvdata(component);
	const struct sunxi_i2s_quirks *quirks = i2s->quirks;
	struct sunxi_i2s_clk *clk = &i2s->clk;
	struct regmap *regmap = i2s->mem.regmap;
	int ret;
	int i;

	SND_LOG_DEBUG(HLOG, "\n");

	ret = snd_sunxi_clk_enable(clk);
	if (ret) {
		SND_LOG_ERR(HLOG, "clk enable failed\n");
		return ret;
	}
	ret = snd_sunxi_regulator_enable(i2s->rglt);
	if (ret) {
		SND_LOG_ERR(HLOG, "regulator enable failed\n");
		return ret;
	}

	/* for i2s init */
	sunxi_i2s_init(i2s);

	/* resume reg value */
	snd_sunxi_echo_reg(regmap, quirks->reg_labels);

	/* for clear TX fifo */
	for (i = 0 ; i < 10 ; i++) {
		regmap_update_bits(regmap, SUNXI_I2S_FIFOCTL,
				   1 << FIFO_CTL_FTX, 1 << FIFO_CTL_FTX);
		mdelay(1);
	}
	regmap_write(regmap, SUNXI_I2S_TXCNT, 0);

	/* for clear RX fifo */
	regmap_update_bits(regmap, SUNXI_I2S_FIFOCTL, 1 << FIFO_CTL_FRX, 1 << FIFO_CTL_FRX);
	regmap_write(regmap, SUNXI_I2S_RXCNT, 0);

	return 0;
}

static struct snd_soc_component_driver sunxi_i2s_dev = {
	.name		= DRV_NAME,
	.probe		= sunxi_i2s_component_probe,
	.remove		= sunxi_i2s_component_remove,
	.suspend	= sunxi_i2s_component_suspend,
	.resume		= sunxi_i2s_component_resume,
	.controls	= sunxi_i2s_controls,
	.num_controls	= ARRAY_SIZE(sunxi_i2s_controls),
};

/*******************************************************************************
 * *** kernel source ***
 * @1 regmap
 * @2 clk
 * @3 regulator
 * @4 dts params
 * @5 dma params
 * @6 pinctrl
 * @7 reg debug
 ******************************************************************************/
static int snd_sunxi_mem_init(struct platform_device *pdev, struct sunxi_i2s_mem *mem)
{
	int ret = 0;
	struct device_node *np = pdev->dev.of_node;

	SND_LOG_DEBUG(HLOG, "\n");

	ret = of_address_to_resource(np, 0, &mem->res);
	if (ret) {
		SND_LOG_ERR(HLOG, "parse device node resource failed\n");
		ret = -EINVAL;
		goto err_of_addr_to_resource;
	}

	mem->memregion = devm_request_mem_region(&pdev->dev, mem->res.start,
						 resource_size(&mem->res),
						 DRV_NAME);
	if (IS_ERR_OR_NULL(mem->memregion)) {
		SND_LOG_ERR(HLOG, "memory region already claimed\n");
		ret = -EBUSY;
		goto err_devm_request_region;
	}

	mem->membase = devm_ioremap(&pdev->dev, mem->memregion->start,
				    resource_size(mem->memregion));
	if (IS_ERR_OR_NULL(mem->membase)) {
		SND_LOG_ERR(HLOG, "ioremap failed\n");
		ret = -EBUSY;
		goto err_devm_ioremap;
	}

	mem->regmap = devm_regmap_init_mmio(&pdev->dev, mem->membase, &g_regmap_config);
	if (IS_ERR_OR_NULL(mem->regmap)) {
		SND_LOG_ERR(HLOG, "regmap init failed\n");
		ret = -EINVAL;
		goto err_devm_regmap_init;
	}

	return 0;

err_devm_regmap_init:
	devm_iounmap(&pdev->dev, mem->membase);
err_devm_ioremap:
	devm_release_mem_region(&pdev->dev, mem->memregion->start, resource_size(mem->memregion));
err_devm_request_region:
err_of_addr_to_resource:
	return ret;
}

static void snd_sunxi_mem_exit(struct platform_device *pdev, struct sunxi_i2s_mem *mem)
{
	SND_LOG_DEBUG(HLOG, "\n");

	devm_iounmap(&pdev->dev, mem->membase);
	devm_release_mem_region(&pdev->dev, mem->memregion->start, resource_size(mem->memregion));
}

static void snd_sunxi_dts_params_init(struct platform_device *pdev, struct sunxi_i2s_dts *dts)
{
	int i;
	int ret = 0;
	unsigned int temp_val;
	unsigned int tx_pin_size, rx_pin_size;
	char tx_pin_map0_str[16] = "", tx_pin_map1_str[16] = "";
	struct device_node *np = pdev->dev.of_node;

	SND_LOG_DEBUG(HLOG, "\n");

	/* get dma params */
	ret = of_property_read_u32(np, "playback-cma", &temp_val);
	if (ret < 0) {
		dts->playback_cma = SUNXI_AUDIO_CMA_MAX_KBYTES;
		SND_LOG_WARN(HLOG, "playback-cma missing, using default value\n");
	} else {
		if (temp_val		> SUNXI_AUDIO_CMA_MAX_KBYTES)
			temp_val	= SUNXI_AUDIO_CMA_MAX_KBYTES;
		else if (temp_val	< SUNXI_AUDIO_CMA_MIN_KBYTES)
			temp_val	= SUNXI_AUDIO_CMA_MIN_KBYTES;

		dts->playback_cma = temp_val;
	}
	ret = of_property_read_u32(np, "capture-cma", &temp_val);
	if (ret != 0) {
		dts->capture_cma = SUNXI_AUDIO_CMA_MAX_KBYTES;
		SND_LOG_WARN(HLOG, "capture-cma missing, using default value\n");
	} else {
		if (temp_val		> SUNXI_AUDIO_CMA_MAX_KBYTES)
			temp_val	= SUNXI_AUDIO_CMA_MAX_KBYTES;
		else if (temp_val	< SUNXI_AUDIO_CMA_MIN_KBYTES)
			temp_val	= SUNXI_AUDIO_CMA_MIN_KBYTES;

		dts->capture_cma = temp_val;
	}
	ret = of_property_read_u32(np, "tx-fifo-size", &temp_val);
	if (ret != 0) {
		dts->playback_fifo_size = SUNXI_AUDIO_FIFO_SIZE;
		SND_LOG_WARN(HLOG, "tx-fifo-size miss, using default value\n");
	} else {
		dts->playback_fifo_size = temp_val;
	}
	ret = of_property_read_u32(np, "rx-fifo-size", &temp_val);
	if (ret != 0) {
		dts->capture_fifo_size = SUNXI_AUDIO_FIFO_SIZE;
		SND_LOG_WARN(HLOG, "rx-fifo-size miss,using default value\n");
	} else {
		dts->capture_fifo_size = temp_val;
	}

	ret = of_property_read_u32(np, "tdm-num", &temp_val);
	if (ret < 0) {
		SND_LOG_WARN(HLOG, "tdm-num config missing\n");
		dts->tdm_num = 0;
	} else {
		dts->tdm_num = temp_val;
	}

	/* default value: tx-msb, rx-msb */
	dts->tx_lsb_first = of_property_read_bool(np, "tx-lsb-first");
	dts->rx_lsb_first = of_property_read_bool(np, "rx-lsb-first");

	tx_pin_size = of_property_count_elems_of_size(np, "tx-pin", 4);
	for (i = 0; i < tx_pin_size; i++) {
		ret = of_property_read_u32_index(np, "tx-pin", i, &temp_val);
		if (temp_val > 3) {
			SND_LOG_WARN(HLOG, "tx-pin[%d] config invalid\n", i);
			continue;
		}
		if (ret < 0) {
			dts->tx_pin[temp_val] = false;
			SND_LOG_WARN(HLOG, "tx-pin[%d] config missing\n", i);
		} else {
			dts->tx_pin[temp_val] = true;
		}

		/* temp_val length shounld be 1 char */
		snprintf(tx_pin_map0_str, 13, "tx-pin%u-map0", temp_val);
		snprintf(tx_pin_map1_str, 13, "tx-pin%u-map1", temp_val);
		ret = of_property_read_u32(np, tx_pin_map0_str, &(dts->tx_pin_map[temp_val][0]));
		if (ret < 0) {
			SND_LOG_DEBUG(HLOG, "%s get failed!\n", tx_pin_map0_str);
			dts->tx_pin_map[temp_val][0] = 0x76543210;
		}
		ret = of_property_read_u32(np, tx_pin_map1_str, &(dts->tx_pin_map[temp_val][1]));
		if (ret < 0) {
			SND_LOG_DEBUG(HLOG, "%s get failed!\n", tx_pin_map1_str);
			dts->tx_pin_map[temp_val][1] = 0xFEDCBA98;
		}
	}

	rx_pin_size = of_property_count_elems_of_size(np, "rx-pin", 4);
	for (i = 0; i < rx_pin_size; i++) {
		ret = of_property_read_u32_index(np, "rx-pin", i, &temp_val);
		if (temp_val > 3) {
			SND_LOG_WARN(HLOG, "rx-pin[%d] config invalid\n", i);
			continue;
		}
		if (ret < 0) {
			dts->rx_pin[temp_val] = false;
			SND_LOG_WARN(HLOG, "rx-pin[%d] config missing\n", i);
		} else {
			dts->rx_pin[temp_val] = true;
		}
	}

	SND_LOG_DEBUG(HLOG, "playback-cma : %zu\n", dts->playback_cma);
	SND_LOG_DEBUG(HLOG, "capture-cma  : %zu\n", dts->capture_cma);
	SND_LOG_DEBUG(HLOG, "tx-fifo-size : %zu\n", dts->playback_fifo_size);
	SND_LOG_DEBUG(HLOG, "rx-fifo-size : %zu\n", dts->capture_fifo_size);
	SND_LOG_DEBUG(HLOG, "tx-pin       : %u\n", dts->tx_pin[0]);
	SND_LOG_DEBUG(HLOG, "rx-pin       : %u\n", dts->rx_pin[0]);

	/* tx_hub */
	dts->tx_hub_en = of_property_read_bool(np, "tx-hub-en");

	/* components func -> rx_sync */
	dts->rx_sync_en = of_property_read_bool(np, "rx-sync-en");

	/* dai-type */
	ret = snd_sunxi_hdmi_get_dai_type(np, &dts->dai_type);
	if (ret)
		dts->dai_type = SUNXI_DAI_I2S_TYPE;
}

static int snd_sunxi_pin_init(struct platform_device *pdev, struct sunxi_i2s_pinctl *pin)
{
	int ret = 0;
	u32 temp_val;
	u32 offset = 0;
	u32 __iomem *pad = NULL;
	bool is_sys_pad = true;
	struct device_node *np = pdev->dev.of_node;

	SND_LOG_DEBUG(HLOG, "\n");

	if (of_property_read_bool(np, "pinctrl-used")) {
		pin->pinctrl_used = 1;
	} else {
		pin->pinctrl_used = 0;
		SND_LOG_DEBUG(HLOG, "unused pinctrl\n");
		return 0;
	}

	pin->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR_OR_NULL(pin->pinctrl)) {
		SND_LOG_ERR(HLOG, "pinctrl get failed\n");
		ret = -EINVAL;
		return ret;
	}
	pin->pinstate = pinctrl_lookup_state(pin->pinctrl, PINCTRL_STATE_DEFAULT);
	if (IS_ERR_OR_NULL(pin->pinstate)) {
		SND_LOG_ERR(HLOG, "pinctrl default state get fail\n");
		ret = -EINVAL;
		goto err_loopup_pinstate;
	}
	pin->pinstate_sleep = pinctrl_lookup_state(pin->pinctrl, PINCTRL_STATE_SLEEP);
	if (IS_ERR_OR_NULL(pin->pinstate_sleep)) {
		SND_LOG_ERR(HLOG, "pinctrl sleep state get failed\n");
		ret = -EINVAL;
		goto err_loopup_pin_sleep;
	}
	ret = pinctrl_select_state(pin->pinctrl, pin->pinstate);
	if (ret < 0) {
		SND_LOG_ERR(HLOG, "i2s set pinctrl default state fail\n");
		ret = -EBUSY;
		goto err_pinctrl_select_default;
	}

	/* TODO: select pin pad, should not be implemented here */
	ret = of_property_read_u32(np, "pin-pad-addr", &temp_val);
	if (ret < 0) {
		SND_LOG_DEBUG(HLOG, "pin-pad-addr get failed\n");
		return 0;
	}

	pad = ioremap(temp_val, 4);
	if (IS_ERR_OR_NULL(pad)) {
		SND_LOG_ERR(HLOG, "pin-pad-addr ioremap failed\n");
		return -EINVAL;
	}

	if (is_sys_pad)
		*pad = *pad | BIT(offset);
	else
		*pad = *pad & ~BIT(offset);

	iounmap(pad);

	return 0;

err_pinctrl_select_default:
err_loopup_pin_sleep:
err_loopup_pinstate:
	devm_pinctrl_put(pin->pinctrl);
	return ret;
}

static void snd_sunxi_dma_params_init(struct sunxi_i2s *i2s)
{
	struct resource *res = &i2s->mem.res;
	struct sunxi_i2s_dts *dts = &i2s->dts;

	SND_LOG_DEBUG(HLOG, "\n");

	i2s->playback_dma_param.src_maxburst = 8;
	i2s->playback_dma_param.dst_maxburst = 8;
	i2s->playback_dma_param.dma_addr = res->start + SUNXI_I2S_TXFIFO;
	i2s->playback_dma_param.cma_kbytes = dts->playback_cma;
	i2s->playback_dma_param.fifo_size = dts->playback_fifo_size;

	i2s->capture_dma_param.src_maxburst = 8;
	i2s->capture_dma_param.dst_maxburst = 8;
	i2s->capture_dma_param.dma_addr = res->start + SUNXI_I2S_RXFIFO;
	i2s->capture_dma_param.cma_kbytes = dts->capture_cma;
	i2s->capture_dma_param.fifo_size = dts->capture_fifo_size;
};

static void snd_sunxi_pin_exit(struct platform_device *pdev, struct sunxi_i2s_pinctl *pin)
{
	SND_LOG_DEBUG(HLOG, "\n");

	if (pin->pinctrl_used)
		devm_pinctrl_put(pin->pinctrl);
}

/* sysfs debug */
static void snd_sunxi_dump_version(void *priv, char *buf, size_t *count)
{
	size_t count_tmp = 0;
	struct sunxi_i2s *i2s = (struct sunxi_i2s *)priv;

	if (!i2s) {
		SND_LOG_ERR(HLOG, "priv to i2s failed\n");
		return;
	}
	if (i2s->pdev)
		if (i2s->pdev->dev.driver)
			if (i2s->pdev->dev.driver->owner)
				goto module_version;
	return;

module_version:
	i2s->module_version = i2s->pdev->dev.driver->owner->version;
	count_tmp += sprintf(buf + count_tmp, "%s\n", i2s->module_version);

	*count = count_tmp;
}

static void snd_sunxi_dump_help(void *priv, char *buf, size_t *count)
{
	size_t count_tmp = 0;

	count_tmp += sprintf(buf + count_tmp, "1. reg read : echo {num} > dump && cat dump\n");
	count_tmp += sprintf(buf + count_tmp, "num: 0(all)\n");
	count_tmp += sprintf(buf + count_tmp, "2. reg write: echo {reg} {value} > dump\n");
	count_tmp += sprintf(buf + count_tmp, "eg. echo 0x00 0xaa > dump\n");

	*count = count_tmp;
}

static int snd_sunxi_dump_show(void *priv, char *buf, size_t *count)
{
	size_t count_tmp = 0;
	struct sunxi_i2s *i2s = (struct sunxi_i2s *)priv;
	int i = 0;
	unsigned int reg_cnt;
	unsigned int output_reg_val;
	struct regmap *regmap;

	if (!i2s) {
		SND_LOG_ERR(HLOG, "priv to i2s failed\n");
		return -1;
	}
	if (!i2s->show_reg_all)
		return 0;
	else
		i2s->show_reg_all = false;

	regmap = i2s->mem.regmap;
	reg_cnt = ARRAY_SIZE(sunxi_reg_labels);
	while ((i < reg_cnt) && sunxi_reg_labels[i].name) {
		regmap_read(regmap, sunxi_reg_labels[i].address, &output_reg_val);
		count_tmp += sprintf(buf + count_tmp, "[0x%03x]: 0x%8x\n",
				     sunxi_reg_labels[i].address, output_reg_val);
		i++;
	}

	*count = count_tmp;

	return 0;
}

static int snd_sunxi_dump_store(void *priv, const char *buf, size_t count)
{
	struct sunxi_i2s *i2s = (struct sunxi_i2s *)priv;
	int scanf_cnt;
	unsigned int input_reg_offset, input_reg_val, output_reg_val;
	struct regmap *regmap;

	if (count <= 1)	/* null or only "\n" */
		return 0;
	if (!i2s) {
		SND_LOG_ERR(HLOG, "priv to i2s failed\n");
		return -1;
	}
	regmap = i2s->mem.regmap;

	if (!strcmp(buf, "0\n")) {
		i2s->show_reg_all = true;
		return 0;
	}

	scanf_cnt = sscanf(buf, "0x%x 0x%x", &input_reg_offset, &input_reg_val);
	if (scanf_cnt != 2) {
		pr_err("wrong format: %s\n", buf);
		return -1;
	}
	if (input_reg_offset > SUNXI_I2S_MAX_REG) {
		pr_err("reg offset > audio max reg[0x%x]\n", SUNXI_I2S_MAX_REG);
		return -1;
	}
	regmap_read(regmap, input_reg_offset, &output_reg_val);
	pr_info("reg[0x%03x]: 0x%x (old)\n", input_reg_offset, output_reg_val);
	regmap_write(regmap, input_reg_offset, input_reg_val);
	regmap_read(regmap, input_reg_offset, &output_reg_val);
	pr_info("reg[0x%03x]: 0x%x (new)\n", input_reg_offset, output_reg_val);

	return 0;
}

static int sunxi_i2s_dev_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	struct sunxi_i2s *i2s;
	struct sunxi_i2s_mem *mem;
	struct sunxi_i2s_clk *clk;
	struct sunxi_i2s_pinctl *pin;
	struct sunxi_i2s_dts *dts;
	struct snd_sunxi_dump *dump;
	const struct sunxi_i2s_quirks *quirks;

	SND_LOG_DEBUG(HLOG, "\n");

	/* sunxi i2s */
	i2s = devm_kzalloc(dev, sizeof(*i2s), GFP_KERNEL);
	if (IS_ERR_OR_NULL(i2s)) {
		SND_LOG_ERR(HLOG, "alloc sunxi_i2s failed\n");
		ret = -ENOMEM;
		goto err_devm_kzalloc;
	}
	dev_set_drvdata(dev, i2s);

	mem = &i2s->mem;
	clk = &i2s->clk;
	pin = &i2s->pin;
	dts = &i2s->dts;
	dump = &i2s->dump;
	i2s->pdev = pdev;

	ret = snd_sunxi_mem_init(pdev, mem);
	if (ret) {
		SND_LOG_ERR(HLOG, "remap init failed\n");
		ret = -EINVAL;
		goto err_snd_sunxi_mem_init;
	}

	ret = snd_sunxi_clk_init(pdev, clk);
	if (ret) {
		SND_LOG_ERR(HLOG, "clk init failed\n");
		ret = -EINVAL;
		goto err_snd_sunxi_clk_init;
	}

	i2s->rglt = snd_sunxi_regulator_init(pdev);
	if (!i2s->rglt) {
		SND_LOG_ERR(HLOG, "rglt init failed\n");
		ret = -EINVAL;
		goto err_snd_sunxi_rglt_init;
	}

	snd_sunxi_dts_params_init(pdev, dts);
	snd_sunxi_dma_params_init(i2s);

	ret = snd_sunxi_pin_init(pdev, pin);
	if (ret) {
		SND_LOG_ERR(HLOG, "pinctrl init failed\n");
		ret = -EINVAL;
		goto err_snd_sunxi_pin_init;
	}

	quirks = of_device_get_match_data(&pdev->dev);
	if (quirks == NULL) {
		SND_LOG_ERR(HLOG, "quirks get failed\n");
		return -ENODEV;
	}
	i2s->quirks = quirks;

	ret = snd_soc_register_component(&pdev->dev, &sunxi_i2s_dev, &sunxi_i2s_dai, 1);
	if (ret) {
		SND_LOG_ERR(HLOG, "component register failed\n");
		ret = -ENOMEM;
		goto err_snd_soc_register_component;
	}

	if (dts->dai_type == SUNXI_DAI_HDMI_TYPE)
		ret = snd_sunxi_hdmi_platform_register(&pdev->dev);
	else
		ret = snd_sunxi_dma_platform_register(&pdev->dev);
	if (ret) {
		SND_LOG_ERR(HLOG, "register ASoC platform failed\n");
		ret = -ENOMEM;
		goto err_snd_sunxi_platform_register;
	}

	snprintf(i2s->module_name, 32, "%s%u", "I2S", dts->tdm_num);
	dump->name = i2s->module_name;
	dump->priv = i2s;
	dump->dump_version = snd_sunxi_dump_version;
	dump->dump_help = snd_sunxi_dump_help;
	dump->dump_show = snd_sunxi_dump_show;
	dump->dump_store = snd_sunxi_dump_store;
	ret = snd_sunxi_dump_register(dump);
	if (ret)
		SND_LOG_WARN(HLOG, "snd_sunxi_dump_register failed\n");

	SND_LOG_DEBUG(HLOG, "register i2s platform success\n");

	return 0;

err_snd_sunxi_platform_register:
	snd_soc_unregister_component(&pdev->dev);
err_snd_soc_register_component:
	snd_sunxi_pin_exit(pdev, pin);
err_snd_sunxi_pin_init:
	snd_sunxi_regulator_exit(i2s->rglt);
err_snd_sunxi_rglt_init:
	snd_sunxi_clk_exit(clk);
err_snd_sunxi_clk_init:
	snd_sunxi_mem_exit(pdev, mem);
err_snd_sunxi_mem_init:
	devm_kfree(dev, i2s);
err_devm_kzalloc:
	of_node_put(np);

	return ret;
}

static int sunxi_i2s_dev_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	struct sunxi_i2s *i2s = dev_get_drvdata(dev);
	struct sunxi_i2s_mem *mem = &i2s->mem;
	struct sunxi_i2s_clk *clk = &i2s->clk;
	struct sunxi_i2s_pinctl *pin = &i2s->pin;
	struct sunxi_i2s_dts *dts = &i2s->dts;
	struct snd_sunxi_dump *dump = &i2s->dump;

	SND_LOG_DEBUG(HLOG, "\n");

	/* remove components */
	snd_sunxi_dump_unregister(dump);
	if (dts->rx_sync_en)
		sunxi_rx_sync_remove(dts->rx_sync_domain);

	if (dts->dai_type == SUNXI_DAI_HDMI_TYPE)
		snd_sunxi_hdmi_platform_unregister(dev);
	else
		snd_sunxi_dma_platform_unregister(&pdev->dev);

	snd_soc_unregister_component(&pdev->dev);

	snd_sunxi_pin_exit(pdev, pin);
	snd_sunxi_clk_exit(clk);
	snd_sunxi_mem_exit(pdev, mem);
	snd_sunxi_regulator_exit(i2s->rglt);

	devm_kfree(dev, i2s);
	of_node_put(np);

	SND_LOG_DEBUG(HLOG, "unregister i2s platform success\n");

	return 0;
}

static const struct sunxi_i2s_quirks sunxi_i2s_quirks = {
	.reg_labels		= sunxi_reg_labels,
	.reg_labels_size	= ARRAY_SIZE(sunxi_reg_labels),
	.reg_max		= SUNXI_I2S_MAX_REG,
	.rx_sync_en = true,
	.set_channel_enable	= sunxi_i2s_set_ch_en,
	.set_daifmt_format	= sunxi_i2s_set_daifmt_fmt,
	.set_channels_map	= sunxi_i2s_set_ch_map,
};

static const struct sunxi_i2s_quirks sun8iw11_i2s_quirks = {
	.reg_labels		= sun8iw11_reg_labels,
	.reg_labels_size	= ARRAY_SIZE(sun8iw11_reg_labels),
	.reg_max		= SUN8IW11_I2S_MAX_REG,
	.rx_sync_en = false,
	.set_channel_enable	= sun8iw11_i2s_set_ch_en,
	.set_daifmt_format	= sun8iw11_i2s_set_daifmt_fmt,
	.set_channels_map	= sun8iw11_i2s_set_ch_map,
};

static const struct of_device_id sunxi_i2s_of_match[] = {
	{
		.compatible = "allwinner," DRV_NAME,
		.data = &sunxi_i2s_quirks,
	},
	{
		.compatible = "allwinner,sun8iw11-i2s",
		.data = &sun8iw11_i2s_quirks,
	},
	{},
};
MODULE_DEVICE_TABLE(of, sunxi_i2s_of_match);

static struct platform_driver sunxi_i2s_driver = {
	.driver	= {
		.name		= DRV_NAME,
		.owner		= THIS_MODULE,
		.of_match_table	= sunxi_i2s_of_match,
	},
	.probe	= sunxi_i2s_dev_probe,
	.remove	= sunxi_i2s_dev_remove,
};

int __init sunxi_i2s_dev_init(void)
{
	int ret;

	ret = platform_driver_register(&sunxi_i2s_driver);
	if (ret != 0) {
		SND_LOG_ERR(HLOG, "platform driver register failed\n");
		return -EINVAL;
	}

	return ret;
}

void __exit sunxi_i2s_dev_exit(void)
{
	platform_driver_unregister(&sunxi_i2s_driver);
}

late_initcall(sunxi_i2s_dev_init);
module_exit(sunxi_i2s_dev_exit);

MODULE_AUTHOR("Dby@allwinnertech.com");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.4");
MODULE_DESCRIPTION("sunxi soundcard platform of i2s");
