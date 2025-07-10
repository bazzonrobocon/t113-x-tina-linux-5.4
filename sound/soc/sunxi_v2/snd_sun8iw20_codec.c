/*
 * sound\soc\sunxi\sun8iw20-codec.c
 * (C) Copyright 2021-2026
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * Dby <Dby@allwinnertech.com>
 * lijingpsw <lijingpsw@allwinnertech.com>
 *
 * some simple description for this code
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/pm.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/pinctrl-sunxi.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/reset.h>
#include <asm/dma.h>
#include <sound/pcm.h>
#include <sound/dmaengine_pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/core.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <sound/jack.h>
#include <linux/sunxi-gpio.h>
#include <linux/sunxi-sid.h>

#include "snd_sunxi_log.h"
#include "snd_sunxi_pcm.h"
#include "snd_sunxi_common.h"
#include "snd_sunxi_jack.h"
#include "snd_sunxi_rxsync.h"
#include "snd_sun8iw20_codec.h"

#define HLOG		"CODEC"
#define DRV_NAME	"sunxi-snd-codec"

struct sample_rate {
	unsigned int samplerate;
	unsigned int rate_bit;
};

static const struct sample_rate sample_rate_conv[] = {
	{8000,   5},
	{11025,  4},
	{12000,  4},
	{16000,  3},
	{22050,  2},
	{24000,  2},
	{32000,  1},
	{44100,  0},
	{48000,  0},
	{96000,  7},
	{192000, 6},
};
static struct audio_reg_label sunxi_reg_labels[] = {
	REG_LABEL(SUNXI_DAC_DPC),
	REG_LABEL(SUNXI_DAC_VOL_CTRL),
	REG_LABEL(SUNXI_DAC_FIFOC),
	REG_LABEL(SUNXI_DAC_FIFOS),
	REG_LABEL(SUNXI_DAC_TXDATA),
	REG_LABEL(SUNXI_DAC_CNT),
	REG_LABEL(SUNXI_DAC_DG),
	REG_LABEL(SUNXI_ADC_FIFOC),
	REG_LABEL(SUNXI_ADC_VOL_CTRL),
	REG_LABEL(SUNXI_ADC_FIFOS),
	REG_LABEL(SUNXI_ADC_RXDATA),
	REG_LABEL(SUNXI_ADC_CNT),
	REG_LABEL(SUNXI_ADC_DEBUG),
	REG_LABEL(SUNXI_ADC_DIG_CTRL),
	REG_LABEL(SUNXI_VRA1SPEEDUP_DOWN_CTRL),
	REG_LABEL(SUNXI_DAC_DAP_CTL),
	REG_LABEL(SUNXI_ADC_DAP_CTL),
	REG_LABEL(SUNXI_ADC1_REG),
	REG_LABEL(SUNXI_ADC2_REG),
	REG_LABEL(SUNXI_ADC3_REG),
	REG_LABEL(SUNXI_DAC_REG),
	REG_LABEL(SUNXI_MICBIAS_AN_CTL),
	REG_LABEL(SUNXI_RAMP_REG),
	REG_LABEL(SUNXI_BIAS_REG),
	REG_LABEL(SUNXI_HMIC_CTRL),
	REG_LABEL(SUNXI_HMIC_STS),
	REG_LABEL(SUNXI_HP2_REG),
	REG_LABEL(SUNXI_POWER_REG),
	REG_LABEL(SUNXI_ADC_CUR_REG),
	REG_LABEL_END,
};

static int snd_sunxi_clk_init(struct platform_device *pdev, struct sunxi_codec_clk *clk);
static void snd_sunxi_clk_exit(struct sunxi_codec_clk *clk);
static int snd_sunxi_clk_enable(struct sunxi_codec_clk *clk);
static void snd_sunxi_clk_disable(struct sunxi_codec_clk *clk);
static int snd_sunxi_rglt_init(struct platform_device *pdev, struct sunxi_codec_rglt *rglt);
static void snd_sunxi_rglt_exit(struct sunxi_codec_rglt *rglt);
static int snd_sunxi_rglt_enable(struct sunxi_codec_rglt *rglt);
static void snd_sunxi_rglt_disable(struct sunxi_codec_rglt *rglt);
static int snd_sunxi_clk_rate(struct sunxi_codec_clk *clk, int stream,
			      unsigned int freq_in, unsigned int freq_out);

/* DAC&ADC - DRC&HPF FUNC */
static int sunxi_codec_get_dap_status(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(component);
	struct regmap *regmap = codec->mem.regmap;
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int shift = e->shift_l;
	unsigned int reg_val;

	switch (shift) {
	case DACDRC_SHIFT:
		regmap_read(regmap, SUNXI_DAC_DAP_CTL, &reg_val);
		ucontrol->value.integer.value[0] =
			(reg_val & 0x1 << DDAP_EN) && (reg_val & 0x1 << DDAP_DRC_EN) ? 1 : 0;
		break;
	case DACHPF_SHIFT:
		regmap_read(regmap, SUNXI_DAC_DAP_CTL, &reg_val);
		ucontrol->value.integer.value[0] =
			(reg_val & 0x1 << DDAP_EN) && (reg_val & 0x1 << DDAP_HPF_EN) ? 1 : 0;
		break;
	case ADCDRC0_SHIFT:
		regmap_read(regmap, SUNXI_ADC_DAP_CTL, &reg_val);
		ucontrol->value.integer.value[0] =
			(reg_val & 0x1 << ADAP0_EN) && (reg_val & 0x1 << ADAP0_DRC_EN) ? 1 : 0;
		break;
	case ADCHPF0_SHIFT:
		regmap_read(regmap, SUNXI_ADC_DAP_CTL, &reg_val);
		ucontrol->value.integer.value[0] =
			(reg_val & 0x1 << ADAP0_EN) && (reg_val & 0x1 << ADAP0_HPF_EN) ? 1 : 0;
		break;
	case ADCDRC1_SHIFT:
		regmap_read(regmap, SUNXI_ADC_DAP_CTL, &reg_val);
		ucontrol->value.integer.value[0] =
			(reg_val & 0x1 << ADAP1_EN) && (reg_val & 0x1 << ADAP1_DRC_EN) ? 1 : 0;
		break;

	case ADCHPF1_SHIFT:
		regmap_read(regmap, SUNXI_ADC_DAP_CTL, &reg_val);
		ucontrol->value.integer.value[0] =
			(reg_val & 0x1 << ADAP1_EN) && (reg_val & 0x1 << ADAP1_HPF_EN) ? 1 : 0;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int sunxi_codec_set_dap_status(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(component);
	struct regmap *regmap = codec->mem.regmap;
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int shift = e->shift_l;

	switch (shift) {
	case DACDRC_SHIFT:
		if (ucontrol->value.integer.value[0]) {
			regmap_update_bits(regmap, SUNXI_DAC_DAP_CTL,
					   0x1 << DDAP_EN | 0x1 << DDAP_DRC_EN,
					   0x1 << DDAP_EN | 0x1 << DDAP_DRC_EN);
		} else {
			regmap_update_bits(regmap, SUNXI_DAC_DAP_CTL,
					   0x1 << DDAP_DRC_EN, 0x0 << DDAP_DRC_EN);
		}
		break;
	case DACHPF_SHIFT:
		if (ucontrol->value.integer.value[0]) {
			regmap_update_bits(regmap, SUNXI_DAC_DAP_CTL,
					   0x1 << DDAP_EN | 0x1 << DDAP_HPF_EN,
					   0x1 << DDAP_EN | 0x1 << DDAP_HPF_EN);
		} else {
			regmap_update_bits(regmap, SUNXI_DAC_DAP_CTL,
					   0x1 << DDAP_HPF_EN, 0x0 << DDAP_HPF_EN);
		}
		break;
	case ADCDRC0_SHIFT:
		if (ucontrol->value.integer.value[0]) {
			regmap_update_bits(regmap, SUNXI_ADC_DAP_CTL,
					   0x1 << ADAP0_EN | 0x1 << ADAP0_DRC_EN,
					   0x1 << ADAP0_EN | 0x1 << ADAP0_DRC_EN);
		} else {
			regmap_update_bits(regmap, SUNXI_ADC_DAP_CTL,
					   0x1 << ADAP0_DRC_EN, 0x0 << ADAP0_DRC_EN);
		}
		break;
	case ADCHPF0_SHIFT:
		if (ucontrol->value.integer.value[0]) {
			regmap_update_bits(regmap, SUNXI_ADC_DAP_CTL,
					   0x1 << ADAP0_EN| 0x1 << ADAP0_HPF_EN,
					   0x1 << ADAP0_EN | 0x1 << ADAP0_HPF_EN);
		} else {
			regmap_update_bits(regmap, SUNXI_ADC_DAP_CTL,
					   0x1 << ADAP0_HPF_EN, 0x0 << ADAP0_HPF_EN);
		}
		break;
	case ADCDRC1_SHIFT:
		if (ucontrol->value.integer.value[0]) {
			regmap_update_bits(regmap, SUNXI_ADC_DAP_CTL,
					   0x1 << ADAP1_EN | 0x1 << ADAP1_DRC_EN,
					   0x1 << ADAP1_EN | 0x1 << ADAP1_DRC_EN);
		} else {
			regmap_update_bits(regmap, SUNXI_ADC_DAP_CTL,
					   0x1 << ADAP1_DRC_EN, 0x0 << ADAP1_DRC_EN);
		}
		break;
	case ADCHPF1_SHIFT:
		if (ucontrol->value.integer.value[0]) {
			regmap_update_bits(regmap, SUNXI_ADC_DAP_CTL,
					   0x1 << ADAP1_EN | 0x1 << ADAP1_HPF_EN,
					   0x1 << ADAP1_EN | 0x1 << ADAP1_HPF_EN);
		} else {
			regmap_update_bits(regmap, SUNXI_ADC_DAP_CTL,
					   0x1 << ADAP1_HPF_EN, 0x0 << ADAP1_HPF_EN);
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const char *sunxi_switch_text[] = {"Off", "On"};
static const char *sunxi_differ_text[] = {"single", "differ"};
static const char *sunxi_input1_mux_text[] = {"MIC1", "FMINL", "LINEINL"};
static const char *sunxi_input2_mux_text[] = {"MIC2", "FMINR", "LINEINR"};
static const char *sunxi_input3_mux_text[] = {"MIC3"};
static SOC_ENUM_SINGLE_EXT_DECL(sunxi_tx_hub_mode_enum, sunxi_switch_text);
static SOC_ENUM_SINGLE_EXT_DECL(sunxi_rx_sync_mode_enum, sunxi_switch_text);

static SOC_ENUM_SINGLE_DECL(sunxi_lineoutl_enum, SUNXI_DAC_REG, LINEOUTLDIFFEN, sunxi_differ_text);
static SOC_ENUM_SINGLE_DECL(sunxi_lineoutr_enum, SUNXI_DAC_REG, LINEOUTRDIFFEN, sunxi_differ_text);
static SOC_ENUM_SINGLE_DECL(sunxi_mic1_enum, SUNXI_ADC1_REG, MIC1_SIN_EN, sunxi_differ_text);
static SOC_ENUM_SINGLE_DECL(sunxi_mic2_enum, SUNXI_ADC2_REG, MIC2_SIN_EN, sunxi_differ_text);
static SOC_ENUM_SINGLE_DECL(sunxi_mic3_enum, SUNXI_ADC3_REG, MIC3_SIN_EN, sunxi_differ_text);

static SOC_ENUM_SINGLE_DECL(sunxi_dacdrc_sta_enum, SND_SOC_NOPM, DACDRC_SHIFT, sunxi_switch_text);
static SOC_ENUM_SINGLE_DECL(sunxi_dachpf_sta_enum, SND_SOC_NOPM, DACHPF_SHIFT, sunxi_switch_text);
static SOC_ENUM_SINGLE_DECL(sunxi_adcdrc0_sta_enum, SND_SOC_NOPM, ADCDRC0_SHIFT, sunxi_switch_text);
static SOC_ENUM_SINGLE_DECL(sunxi_adchpf0_sta_enum, SND_SOC_NOPM, ADCHPF0_SHIFT, sunxi_switch_text);
static SOC_ENUM_SINGLE_DECL(sunxi_adcdrc1_sta_enum, SND_SOC_NOPM, ADCDRC1_SHIFT, sunxi_switch_text);
static SOC_ENUM_SINGLE_DECL(sunxi_adchpf1_sta_enum, SND_SOC_NOPM, ADCHPF1_SHIFT, sunxi_switch_text);

static SOC_ENUM_SINGLE_DECL(sunxi_adc12_swap_enum, SUNXI_ADC_DEBUG, ADC_SWP1, sunxi_switch_text);
static SOC_ENUM_SINGLE_DECL(sunxi_adc34_swap_enum, SUNXI_ADC_DEBUG, ADC_SWP2, sunxi_switch_text);

static const DECLARE_TLV_DB_SCALE(digital_tlv, -7424, 116, 0);
static const DECLARE_TLV_DB_SCALE(dac_vol_tlv, -11925, 75, 0);
static const DECLARE_TLV_DB_SCALE(adc_vol_tlv, -11925, 75, 0);
static const DECLARE_TLV_DB_SCALE(mic_vol_tlv, 0, 100, 0);

static const DECLARE_TLV_DB_SCALE(fmin_gain_tlv, 0, 600, 0);
static const DECLARE_TLV_DB_SCALE(linein_gain_tlv, 0, 600, 0);
static const DECLARE_TLV_DB_SCALE(hp_vol_tlv, -4200, 600, 0);
static const unsigned int lineout_tlv[] = {
	TLV_DB_RANGE_HEAD(2),
	0, 1, TLV_DB_SCALE_ITEM(0, 0, 1),
	2, 31, TLV_DB_SCALE_ITEM(-4350, 150, 1),
};

static int sunxi_get_tx_hub_mode(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(component);
	struct regmap *regmap = codec->mem.regmap;
	unsigned int reg_val;

	regmap_read(regmap, SUNXI_DAC_DPC, &reg_val);

	ucontrol->value.integer.value[0] = ((reg_val & (0x1 << DAC_HUB_EN)) ? 1 : 0);

	return 0;
}

static int sunxi_set_tx_hub_mode(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(component);
	struct regmap *regmap = codec->mem.regmap;

	switch (ucontrol->value.integer.value[0]) {
	case 0:
		regmap_update_bits(regmap, SUNXI_DAC_DPC, 0x1 << DAC_HUB_EN, 0x0 << DAC_HUB_EN);
		break;
	case 1:
		regmap_update_bits(regmap, SUNXI_DAC_DPC, 0x1 << DAC_HUB_EN, 0x1 << DAC_HUB_EN);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static void sunxi_rx_sync_enable(void *data, bool enable)
{
	struct regmap *regmap = data;

	SND_LOG_DEBUG(HLOG, "%s\n", enable ? "on" : "off");

	if (enable)
		regmap_update_bits(regmap, SUNXI_ADC_FIFOC,
				   1 << RX_SYNC_EN_STA, 1 << RX_SYNC_EN_STA);
	else
		regmap_update_bits(regmap, SUNXI_ADC_FIFOC,
				   1 << RX_SYNC_EN_STA, 0 << RX_SYNC_EN_STA);
}

static int sunxi_get_rx_sync_mode(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(component);
	struct sunxi_codec_dts *dts = &codec->dts;

	ucontrol->value.integer.value[0] = dts->rx_sync_ctl;

	return 0;
}

static int sunxi_set_rx_sync_mode(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(component);
	struct sunxi_codec_dts *dts = &codec->dts;
	struct regmap *regmap = codec->mem.regmap;

	SND_LOG_DEBUG(HLOG, "\n");

	switch (ucontrol->value.integer.value[0]) {
	case 0:
		dts->rx_sync_ctl = 0;
		regmap_update_bits(regmap, SUNXI_ADC_FIFOC, 0x1 << ADCDFEN, 0x1 << ADCDFEN);
		regmap_update_bits(regmap, SUNXI_ADC_FIFOC, 0x1 << RX_SYNC_EN_STA, 0x0 << RX_SYNC_EN_STA);
		break;
	case 1:
		regmap_update_bits(regmap, SUNXI_ADC_FIFOC, 0x1 << ADCDFEN, 0x0 << ADCDFEN);
		regmap_update_bits(regmap, SUNXI_ADC_FIFOC, 0x1 << RX_SYNC_EN, 0x1 << RX_SYNC_EN);
		regmap_update_bits(regmap, SUNXI_ADC_FIFOC, 0x1 << RX_SYNC_EN_STA, 0x1 << RX_SYNC_EN_STA);
		dts->rx_sync_ctl = 1;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int sunxi_get_input_mux(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_dapm_kcontrol_component(kcontrol);
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(component);
	struct regmap *regmap = codec->mem.regmap;
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int shift = e->shift_l;
	unsigned int reg_val;
	bool mic = false;
	bool fmin = false;
	bool linein = false;

	/* default: micin */
	ucontrol->value.integer.value[0] = 0;

	switch (shift) {
	case INPUT1_SHIFT:
		regmap_read(regmap, SUNXI_ADC1_REG, &reg_val);
		if (reg_val & 0x1 << MIC1_PGA_EN) {
			mic = true;
			ucontrol->value.integer.value[0] = 0;
		}
		if (reg_val & 0x1 << FMINLEN) {
			fmin = true;
			ucontrol->value.integer.value[0] = 1;
		}
		if (reg_val & 0x1 << LINEINLEN) {
			linein = true;
			ucontrol->value.integer.value[0] = 2;
		}
		if ((mic + fmin + linein) > 1) {
			ucontrol->value.integer.value[0] = 0;
			SND_LOG_ERR(HLOG, "input1 mux has multiple input sources\n");
			return -EINVAL;
		}
		break;
	case INPUT2_SHIFT:
		regmap_read(regmap, SUNXI_ADC2_REG, &reg_val);
		if (reg_val & 0x1 << MIC2_PGA_EN) {
			mic = true;
			ucontrol->value.integer.value[0] = 0;
		}
		if (reg_val & 0x1 << FMINREN) {
			fmin = true;
			ucontrol->value.integer.value[0] = 1;
		}
		if (reg_val & 0x1 << LINEINREN) {
			linein = true;
			ucontrol->value.integer.value[0] = 2;
		}
		if ((mic + fmin + linein) > 1) {
			ucontrol->value.integer.value[0] = 0;
			SND_LOG_ERR(HLOG, "input2 mux has multiple input sources\n");
			return -EINVAL;
		}
		break;
	case INPUT3_SHIFT:
		/* only one input source, default micin. */
		/* regmap_read(regmap, SUNXI_ADC3_REG, &reg_val); */
		ucontrol->value.integer.value[0] = 0;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int sunxi_set_input_mux(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_dapm_kcontrol_component(kcontrol);
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(component);
	struct regmap *regmap = codec->mem.regmap;
	struct snd_soc_dapm_context *dapm = snd_soc_dapm_kcontrol_dapm(kcontrol);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int shift = e->shift_l;

	switch (shift) {
	case INPUT1_SHIFT:
		if (ucontrol->value.integer.value[0] == 0) {
			regmap_update_bits(regmap, SUNXI_ADC1_REG, 1 << MIC1_PGA_EN, 1 << MIC1_PGA_EN);
			regmap_update_bits(regmap, SUNXI_ADC1_REG, 1 << FMINLEN, 0 << FMINLEN);
			regmap_update_bits(regmap, SUNXI_ADC1_REG, 1 << LINEINLEN, 0 << LINEINLEN);
		} else if (ucontrol->value.integer.value[0] == 1) {
			regmap_update_bits(regmap, SUNXI_ADC1_REG, 1 << MIC1_PGA_EN, 0 << MIC1_PGA_EN);
			regmap_update_bits(regmap, SUNXI_ADC1_REG, 1 << FMINLEN, 1 << FMINLEN);
			regmap_update_bits(regmap, SUNXI_ADC1_REG, 1 << LINEINLEN, 0 << LINEINLEN);
		} else if (ucontrol->value.integer.value[0] == 2) {
			regmap_update_bits(regmap, SUNXI_ADC1_REG, 1 << MIC1_PGA_EN, 0 << MIC1_PGA_EN);
			regmap_update_bits(regmap, SUNXI_ADC1_REG, 1 << FMINLEN, 0 << FMINLEN);
			regmap_update_bits(regmap, SUNXI_ADC1_REG, 1 << LINEINLEN, 1 << LINEINLEN);
		} else {
			SND_LOG_ERR(HLOG, "input1 mux val(%ld) invailed\n",
				    ucontrol->value.integer.value[0]);
			return -EINVAL;
		}
		break;
	case INPUT2_SHIFT:
		if (ucontrol->value.integer.value[0] == 0) {
			regmap_update_bits(regmap, SUNXI_ADC2_REG, 1 << MIC2_PGA_EN, 1 << MIC2_PGA_EN);
			regmap_update_bits(regmap, SUNXI_ADC2_REG, 1 << FMINREN, 0 << FMINREN);
			regmap_update_bits(regmap, SUNXI_ADC2_REG, 1 << LINEINREN, 0 << LINEINREN);
		} else if (ucontrol->value.integer.value[0] == 1) {
			regmap_update_bits(regmap, SUNXI_ADC2_REG, 1 << MIC2_PGA_EN, 0 << MIC2_PGA_EN);
			regmap_update_bits(regmap, SUNXI_ADC2_REG, 1 << FMINREN, 1 << FMINREN);
			regmap_update_bits(regmap, SUNXI_ADC2_REG, 1 << LINEINREN, 0 << LINEINREN);
		} else if (ucontrol->value.integer.value[0] == 2) {
			regmap_update_bits(regmap, SUNXI_ADC2_REG, 1 << MIC2_PGA_EN, 0 << MIC2_PGA_EN);
			regmap_update_bits(regmap, SUNXI_ADC2_REG, 1 << FMINREN, 0 << FMINREN);
			regmap_update_bits(regmap, SUNXI_ADC2_REG, 1 << LINEINREN, 1 << LINEINREN);
		} else {
			SND_LOG_ERR(HLOG, "input2 mux val(%ld) invailed\n",
				    ucontrol->value.integer.value[0]);
			return -EINVAL;
		}
		break;
	case INPUT3_SHIFT:
		regmap_update_bits(regmap, SUNXI_ADC3_REG, 1 << MIC3_PGA_EN, 1 << MIC3_PGA_EN);
		break;
	default:
		return -EINVAL;
	}

	snd_soc_dapm_mux_update_power(dapm, kcontrol, ucontrol->value.enumerated.item[0], e, NULL);

	return 0;
}

static int sunxi_playback_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *k, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(component);
	struct regmap *regmap = codec->mem.regmap;

	SND_LOG_DEBUG(HLOG, "\n");

	switch (event) {
	case	SND_SOC_DAPM_PRE_PMU:
		/* fix signal interference lineout -> hpout
		 * power on:
		 * hpldo enable
		 * dac enable
		 *
		 * power off:
		 * dac disable
		 * hpldo disable
		 *
		 * note: hpldo enable when lineout or hpout use.
		 */
		regmap_update_bits(regmap, SUNXI_POWER_REG, 0x1 << HPLDO_EN, 0x1 << HPLDO_EN);

		msleep(30);
		/* DACL to left channel LINEOUT Mute control 0:mute 1: not mute */
		regmap_update_bits(regmap, SUNXI_DAC_REG,
				   0x1 << DACLMUTE | 0x1 << DACRMUTE,
				   0x1 << DACLMUTE | 0x1 << DACRMUTE);
		regmap_update_bits(regmap, SUNXI_DAC_DPC, 0x1 << EN_DAC, 0x1 << EN_DAC);
		break;
	case	SND_SOC_DAPM_POST_PMD:
		regmap_update_bits(regmap, SUNXI_DAC_DPC, 0x1 << EN_DAC, 0x0<<EN_DAC);
		/* DACL to left channel LINEOUT Mute control 0:mute 1: not mute */
		regmap_update_bits(regmap, SUNXI_DAC_REG,
				   0x1 << DACLMUTE | 0x1 << DACRMUTE,
				   0x0 << DACLMUTE | 0x0 << DACRMUTE);

		regmap_update_bits(regmap, SUNXI_POWER_REG, 0x1 << HPLDO_EN, 0x0 << HPLDO_EN);
		break;
	default:
		break;
	}

	return 0;
}

static int sunxi_lineoutl_event(struct snd_soc_dapm_widget *w, struct snd_kcontrol *k, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(component);
	struct regmap *regmap = codec->mem.regmap;
	unsigned int reg_val;

	SND_LOG_DEBUG(HLOG, "\n");

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		/* In order to ensure that the hpout pop for B chip SOC,
		* the B chip SOC needs to enable RMC_EN when RD_EN disable (HPOUT no useing).
		*/
		if (sunxi_get_soc_ver() & 0x7) {
			regmap_read(regmap, SUNXI_RAMP_REG, &reg_val);
			if (!(reg_val & (0x1 << RD_EN)))
				regmap_update_bits(regmap, SUNXI_RAMP_REG, 0x1 << RMC_EN, 0x1 << RMC_EN);
		}

		regmap_update_bits(regmap, SUNXI_DAC_REG, 0x1 << LINEOUTLEN, 0x1 << LINEOUTLEN);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		regmap_update_bits(regmap, SUNXI_DAC_REG, 0x1 << LINEOUTLEN, 0x0 << LINEOUTLEN);

		/* In order to ensure that the hpout pop for B chip SOC,
		* the B chip SOC needs to disable RMC_EN when RD_EN disable and disable lineout.
		*/
		if (sunxi_get_soc_ver() & 0x7) {
			regmap_read(regmap, SUNXI_RAMP_REG, &reg_val);
			if (!(reg_val & (0x1 << RD_EN)))
				regmap_update_bits(regmap, SUNXI_RAMP_REG, 0x1 << RMC_EN, 0x0 << RMC_EN);
		}
		break;
	default:
		break;
	}

	return 0;
}

static int sunxi_lineoutr_event(struct snd_soc_dapm_widget *w, struct snd_kcontrol *k, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(component);
	struct regmap *regmap = codec->mem.regmap;
	unsigned int reg_val;

	SND_LOG_DEBUG(HLOG, "\n");

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		/* In order to ensure that the hpout pop for B chip SOC,
		* the B chip SOC needs to enable RMC_EN when RD_EN disable (HPOUT no useing).
		*/
		if (sunxi_get_soc_ver() & 0x7) {
			regmap_read(regmap, SUNXI_RAMP_REG, &reg_val);
			if (!(reg_val & (0x1 << RD_EN)))
				regmap_update_bits(regmap, SUNXI_RAMP_REG,
						   0x1 << RMC_EN, 0x1 << RMC_EN);
		}

		regmap_update_bits(regmap, SUNXI_DAC_REG, 0x1 << LINEOUTREN, 0x1 << LINEOUTREN);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		regmap_update_bits(regmap, SUNXI_DAC_REG, 0x1 << LINEOUTREN, 0x0 << LINEOUTREN);

		/* In order to ensure that the hpout pop for B chip SOC,
		* the B chip SOC needs to disable RMC_EN when RD_EN disable and disable lineout.
		*/
		if (sunxi_get_soc_ver() & 0x7) {
			regmap_read(regmap, SUNXI_RAMP_REG, &reg_val);
			if (!(reg_val & (0x1 << RD_EN)))
				regmap_update_bits(regmap, SUNXI_RAMP_REG,
						   0x1 << RMC_EN, 0x0 << RMC_EN);
		}
		break;
	default:
		break;
	}

	return 0;
}

static int sunxi_spk_event(struct snd_soc_dapm_widget *w, struct snd_kcontrol *k, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(component);

	SND_LOG_DEBUG(HLOG, "\n");

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_sunxi_pa_pin_enable(codec->pa_cfg, codec->pa_pin_max);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_sunxi_pa_pin_disable(codec->pa_cfg, codec->pa_pin_max);
		break;
	default:
		break;
	}

	return 0;
}

static int sunxi_hpout_event(struct snd_soc_dapm_widget *w, struct snd_kcontrol *k, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(component);
	struct regmap *regmap = codec->mem.regmap;
	unsigned int reg_val;

	SND_LOG_DEBUG(HLOG, "\n");

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		/* In order to ensure that the hpout pop for B chip SOC,
		* the B chip SOC needs to disable RMC_EN and enable RD_EN
		* when up HPOUT (whether LINEOUT use).
		*/
		if (sunxi_get_soc_ver() & 0x7) {
			regmap_update_bits(regmap, SUNXI_RAMP_REG, 0x1 << RMC_EN, 0x0 << RMC_EN);
			regmap_update_bits(regmap, SUNXI_RAMP_REG, 0x1 << RAMP_SRST, 0x1 << RAMP_SRST);
			regmap_update_bits(regmap, SUNXI_RAMP_REG, 0x1 << RD_EN, 0x1 << RD_EN);
			msleep(100);
		} else {
			regmap_update_bits(regmap, SUNXI_HP2_REG, 0x1 << HPFB_BUF_EN, 0x1 << HPFB_BUF_EN);
			regmap_update_bits(regmap, SUNXI_HP2_REG, 0x1 << HP_DRVEN, 0x1 << HP_DRVEN);
			regmap_update_bits(regmap, SUNXI_HP2_REG, 0x1 << HP_DRVOUTEN, 0x1 << HP_DRVOUTEN);
			regmap_update_bits(regmap, SUNXI_HP2_REG, 0x1 << RSWITCH, 0x1 << RSWITCH);
			regmap_update_bits(regmap, SUNXI_HP2_REG, 0x1 << HPFB_IN_EN, 0x1 << HPFB_IN_EN);
			regmap_update_bits(regmap, SUNXI_HP2_REG, 0x1 << RAMP_OUT_EN, 0x1 << RAMP_OUT_EN);
			/* fix signal interference lineout -> hpout
			regmap_update_bits(regmap, SUNXI_POWER_REG,
					(0x1 << HPLDO_EN), (0x1 << HPLDO_EN));
			*/
		}
		break;
	case SND_SOC_DAPM_PRE_PMD:
		/* In order to ensure that the hpout pop for B chip SOC,
		* the B chip SOC needs to enable RMC_EN and disable RD_EN
		* when down HPOUT and LINEOUT useing.
		*/
		if (sunxi_get_soc_ver() & 0x7) {
			/* if lineout playing, enable RMC_EN */
			regmap_read(regmap, SUNXI_DAC_REG, &reg_val);
			if ((reg_val & (0x1 << LINEOUTLEN)) || (reg_val & (0x1 << LINEOUTREN)))
				regmap_update_bits(regmap, SUNXI_RAMP_REG, 0x1 << RMC_EN, 0x1 << RMC_EN);

			regmap_update_bits(regmap, SUNXI_RAMP_REG, 0x1 << RD_EN, 0x0 << RD_EN);
		} else {
			/* fix signal interference lineout -> hpout
			regmap_update_bits(regmap, SUNXI_POWER_REG,
					(0x1 << HPLDO_EN), (0x0 << HPLDO_EN));
			*/
			regmap_update_bits(regmap, SUNXI_HP2_REG, 0x1 << HPFB_BUF_EN, 0x0 << HPFB_BUF_EN);
			regmap_update_bits(regmap, SUNXI_HP2_REG, 0x1 << HP_DRVEN, 0x0 << HP_DRVEN);
			regmap_update_bits(regmap, SUNXI_HP2_REG, 0x1 << HP_DRVOUTEN, 0x0 << HP_DRVOUTEN);
			regmap_update_bits(regmap, SUNXI_HP2_REG, 0x1 << RSWITCH, 0x0 << RSWITCH);
			regmap_update_bits(regmap, SUNXI_HP2_REG, 0x1 << HPFB_IN_EN, 0x0 << HPFB_IN_EN);
			regmap_update_bits(regmap, SUNXI_HP2_REG, 0x1 << RAMP_OUT_EN, 0x0 << RAMP_OUT_EN);
		}
		break;
	}

	return 0;
}

static int sunxi_adc1_event(struct snd_soc_dapm_widget *w, struct snd_kcontrol *k, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(component);
	struct regmap *regmap = codec->mem.regmap;

	SND_LOG_DEBUG(HLOG, "\n");

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		//mdelay(80);
		regmap_update_bits(regmap, SUNXI_ADC_DIG_CTRL,
				   0x1 << ADC1_CHANNEL_EN, 0x1 << ADC1_CHANNEL_EN);
		regmap_update_bits(regmap, SUNXI_ADC1_REG, 0x1 << ADC1_EN, 0x1 << ADC1_EN);
		regmap_update_bits(regmap, SUNXI_ADC_FIFOC, 0x1 << EN_AD, 0x1 << EN_AD);
		break;
	case SND_SOC_DAPM_POST_PMD:
		regmap_update_bits(regmap, SUNXI_ADC_FIFOC, 0x1 << EN_AD, 0x0 << EN_AD);
		regmap_update_bits(regmap, SUNXI_ADC1_REG, 0x1 << ADC1_EN, 0x0 << ADC1_EN);
		regmap_update_bits(regmap, SUNXI_ADC_DIG_CTRL,
				   0x1 << ADC1_CHANNEL_EN, 0x0 << ADC1_CHANNEL_EN);
		break;
	default:
		break;
	}

	return 0;
}

static int sunxi_adc2_event(struct snd_soc_dapm_widget *w, struct snd_kcontrol *k, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(component);
	struct regmap *regmap = codec->mem.regmap;

	SND_LOG_DEBUG(HLOG, "\n");

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		//mdelay(80);
		regmap_update_bits(regmap, SUNXI_ADC_DIG_CTRL,
				   0x1 << ADC2_CHANNEL_EN, 0x1 << ADC2_CHANNEL_EN);
		regmap_update_bits(regmap, SUNXI_ADC2_REG,
				   0x1 << ADC2_EN, 0x1 << ADC2_EN);
			regmap_update_bits(regmap, SUNXI_ADC_FIFOC, 0x1 << EN_AD, 0x1 << EN_AD);
		break;
	case SND_SOC_DAPM_POST_PMD:
		regmap_update_bits(regmap, SUNXI_ADC_FIFOC, 0x1 << EN_AD, 0x0 << EN_AD);
		regmap_update_bits(regmap, SUNXI_ADC2_REG,
				   0x1 << ADC2_EN, 0x0 << ADC2_EN);
		regmap_update_bits(regmap, SUNXI_ADC_DIG_CTRL,
				   0x1 << ADC2_CHANNEL_EN, 0x0 << ADC2_CHANNEL_EN);
		break;
	default:
		break;
	}

	return 0;
}

static int sunxi_adc3_event(struct snd_soc_dapm_widget *w, struct snd_kcontrol *k, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(component);
	struct regmap *regmap = codec->mem.regmap;

	SND_LOG_DEBUG(HLOG, "\n");

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		//mdelay(80);
		regmap_update_bits(regmap, SUNXI_ADC_DIG_CTRL,
				   0x1 << ADC3_CHANNEL_EN, 0x1 << ADC3_CHANNEL_EN);
		regmap_update_bits(regmap, SUNXI_ADC3_REG,
				   0x1 << ADC3_EN, 0x1 << ADC3_EN);
		regmap_update_bits(regmap, SUNXI_ADC_FIFOC, 0x1 << EN_AD, 0x1 << EN_AD);
		break;
	case SND_SOC_DAPM_POST_PMD:
		regmap_update_bits(regmap, SUNXI_ADC_FIFOC, 0x1 << EN_AD, 0x0 << EN_AD);
		regmap_update_bits(regmap, SUNXI_ADC3_REG,
				   0x1 << ADC3_EN, 0x0 << ADC3_EN);
		regmap_update_bits(regmap, SUNXI_ADC_DIG_CTRL,
				   0x1 << ADC3_CHANNEL_EN, 0x0 << ADC3_CHANNEL_EN);
		break;
	default:
		break;
	}

	return 0;
}

static int sunxi_mic1_event(struct snd_soc_dapm_widget *w, struct snd_kcontrol *k, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(component);
	struct regmap *regmap = codec->mem.regmap;

	SND_LOG_DEBUG(HLOG, "\n");

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		mutex_lock(&codec->mic_sta.mic_sta_mutex);
		codec->mic_sta.mic1 = true;
		if (codec->mic_sta.mic2 == false && codec->mic_sta.mic3 == false) {
			regmap_update_bits(regmap, SUNXI_MICBIAS_AN_CTL,
					   0x1 << MMICBIASEN, 0x1 << MMICBIASEN);
			regmap_update_bits(regmap, SUNXI_ADC_FIFOC, 0x1 << EN_AD, 0x1 << EN_AD);
		}
		mutex_unlock(&codec->mic_sta.mic_sta_mutex);
		break;
	case SND_SOC_DAPM_POST_PMD:
		mutex_lock(&codec->mic_sta.mic_sta_mutex);
		codec->mic_sta.mic1 = false;
		if (codec->mic_sta.mic2 == false && codec->mic_sta.mic3 == false) {
			regmap_update_bits(regmap, SUNXI_ADC_FIFOC, 0x1 << EN_AD, 0x0 << EN_AD);
			regmap_update_bits(regmap, SUNXI_MICBIAS_AN_CTL,
					   0x1 << MMICBIASEN, 0x0 << MMICBIASEN);
		}
		mutex_unlock(&codec->mic_sta.mic_sta_mutex);
		break;
	default:
		break;
	}

	return 0;
}

static int sunxi_mic2_event(struct snd_soc_dapm_widget *w, struct snd_kcontrol *k, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(component);
	struct regmap *regmap = codec->mem.regmap;

	SND_LOG_DEBUG(HLOG, "\n");

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		mutex_lock(&codec->mic_sta.mic_sta_mutex);
		codec->mic_sta.mic2 = true;
		if (codec->mic_sta.mic1 == false && codec->mic_sta.mic3 == false) {
			regmap_update_bits(regmap, SUNXI_MICBIAS_AN_CTL,
					   0x1 << MMICBIASEN, 0x1 << MMICBIASEN);
			regmap_update_bits(regmap, SUNXI_ADC_FIFOC, 0x1 << EN_AD, 0x1 << EN_AD);
		}
		mutex_unlock(&codec->mic_sta.mic_sta_mutex);
		break;
	case SND_SOC_DAPM_POST_PMD:
		mutex_lock(&codec->mic_sta.mic_sta_mutex);
		codec->mic_sta.mic2 = false;
		if (codec->mic_sta.mic1 == false && codec->mic_sta.mic3 == false) {
			regmap_update_bits(regmap, SUNXI_ADC_FIFOC, 0x1 << EN_AD, 0x0 << EN_AD);
			regmap_update_bits(regmap, SUNXI_MICBIAS_AN_CTL,
					   0x1 << MMICBIASEN, 0x0 << MMICBIASEN);
		}
		mutex_unlock(&codec->mic_sta.mic_sta_mutex);
		break;
	default:
		break;
	}

	return 0;
}

static int sunxi_mic3_event(struct snd_soc_dapm_widget *w, struct snd_kcontrol *k, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(component);
	struct regmap *regmap = codec->mem.regmap;

	SND_LOG_DEBUG(HLOG, "\n");

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		mutex_lock(&codec->mic_sta.mic_sta_mutex);
		codec->mic_sta.mic3 = true;
		if (codec->mic_sta.mic1 == false && codec->mic_sta.mic2 == false) {
			regmap_update_bits(regmap, SUNXI_MICBIAS_AN_CTL,
					   0x1 << MMICBIASEN, 0x1 << MMICBIASEN);
			regmap_update_bits(regmap, SUNXI_ADC_FIFOC, 0x1 << EN_AD, 0x1 << EN_AD);
		}
		mutex_unlock(&codec->mic_sta.mic_sta_mutex);
		break;
	case SND_SOC_DAPM_POST_PMD:
		mutex_lock(&codec->mic_sta.mic_sta_mutex);
		codec->mic_sta.mic3 = false;
		if (codec->mic_sta.mic1 == false && codec->mic_sta.mic2 == false) {
			regmap_update_bits(regmap, SUNXI_ADC_FIFOC, 0x1 << EN_AD, 0x0 << EN_AD);
			regmap_update_bits(regmap, SUNXI_MICBIAS_AN_CTL,
					   0x1 << MMICBIASEN, 0x0 << MMICBIASEN);
		}
		mutex_unlock(&codec->mic_sta.mic_sta_mutex);
		break;
	default:
		break;
	}

	return 0;
}

static const struct snd_kcontrol_new sunxi_tx_hub_controls[] = {
	SOC_ENUM_EXT("tx hub mode", sunxi_tx_hub_mode_enum,
		     sunxi_get_tx_hub_mode, sunxi_set_tx_hub_mode),
};

static const struct snd_kcontrol_new sunxi_rx_sync_controls[] = {
	SOC_ENUM_EXT("rx sync mode", sunxi_rx_sync_mode_enum,
		     sunxi_get_rx_sync_mode, sunxi_set_rx_sync_mode),
};

static const struct snd_kcontrol_new sunxi_codec_controls[] = {
	/* DAP Func */
	SOC_ENUM_EXT("DAC DRC Mode", sunxi_dacdrc_sta_enum,
		     sunxi_codec_get_dap_status, sunxi_codec_set_dap_status),
	SOC_ENUM_EXT("DAC HPF Mode", sunxi_dachpf_sta_enum,
		     sunxi_codec_get_dap_status, sunxi_codec_set_dap_status),
	SOC_ENUM_EXT("ADC DRC0 Mode", sunxi_adcdrc0_sta_enum,
		     sunxi_codec_get_dap_status, sunxi_codec_set_dap_status),
	SOC_ENUM_EXT("ADC HPF0 Mode", sunxi_adchpf0_sta_enum,
		     sunxi_codec_get_dap_status, sunxi_codec_set_dap_status),
	SOC_ENUM_EXT("ADC DRC1 Mode", sunxi_adcdrc1_sta_enum,
		     sunxi_codec_get_dap_status, sunxi_codec_set_dap_status),
	SOC_ENUM_EXT("ADC HPF1 Mode", sunxi_adchpf1_sta_enum,
		     sunxi_codec_get_dap_status, sunxi_codec_set_dap_status),

	SOC_ENUM("ADC1 ADC2 Swap", sunxi_adc12_swap_enum),
	SOC_ENUM("ADC3 ADC4 Swap", sunxi_adc34_swap_enum),

	SOC_ENUM("LINEOUTL Output Select", sunxi_lineoutl_enum),
	SOC_ENUM("LINEOUTR Output Select", sunxi_lineoutr_enum),
	SOC_ENUM("MIC1 Input Select", sunxi_mic1_enum),
	SOC_ENUM("MIC2 Input Select", sunxi_mic2_enum),
	SOC_ENUM("MIC3 Input Select", sunxi_mic3_enum),

	/* Volume */
	SOC_SINGLE_TLV("DAC Digital Volume", SUNXI_DAC_DPC, DVOL, 0x3F, 1, digital_tlv),
	SOC_SINGLE_TLV("DACL Volume", SUNXI_DAC_VOL_CTRL, DAC_VOL_L, 0xFF, 0, dac_vol_tlv),
	SOC_SINGLE_TLV("DACR Volume", SUNXI_DAC_VOL_CTRL,  DAC_VOL_R, 0xFF, 0, dac_vol_tlv),
	SOC_SINGLE_TLV("ADC1 Volume", SUNXI_ADC_VOL_CTRL, ADC1_VOL, 0xFF, 0, adc_vol_tlv),
	SOC_SINGLE_TLV("ADC2 Volume", SUNXI_ADC_VOL_CTRL, ADC2_VOL, 0xFF, 0, adc_vol_tlv),
	SOC_SINGLE_TLV("ADC3 Volume", SUNXI_ADC_VOL_CTRL, ADC3_VOL, 0xFF, 0, adc_vol_tlv),
	SOC_SINGLE_TLV("ADC1 Gain", SUNXI_ADC1_REG, ADC1_PGA_GAIN_CTRL, 0x1F, 0, mic_vol_tlv),
	SOC_SINGLE_TLV("ADC2 Gain", SUNXI_ADC2_REG, ADC2_PGA_GAIN_CTRL, 0x1F, 0, mic_vol_tlv),
	SOC_SINGLE_TLV("ADC3 Gain", SUNXI_ADC3_REG, ADC3_PGA_GAIN_CTRL, 0x1F, 0, mic_vol_tlv),
	SOC_SINGLE_TLV("LINEOUT Volume", SUNXI_DAC_REG, LINEOUT_VOL, 0x1F, 1, lineout_tlv),
	SOC_SINGLE_TLV("HPOUT Gain", SUNXI_HP2_REG, HEADPHONE_GAIN, 0x7, 1, hp_vol_tlv),
	SOC_SINGLE_TLV("FMINL Gain", SUNXI_ADC1_REG, FMINLG, 0x1, 0, fmin_gain_tlv),
	SOC_SINGLE_TLV("FMINR Gain", SUNXI_ADC2_REG, FMINRG, 0x1, 0, fmin_gain_tlv),
	SOC_SINGLE_TLV("LINEINL Gain", SUNXI_ADC1_REG, LINEINLG, 0x1, 0, linein_gain_tlv),
	SOC_SINGLE_TLV("LINEINR Gain", SUNXI_ADC2_REG, LINEINRG, 0x1, 0, linein_gain_tlv),
};

/* Define Mux Controls */
static const struct soc_enum input1_mux_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, INPUT1_SHIFT, 3, sunxi_input1_mux_text);
static const struct soc_enum input2_mux_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, INPUT2_SHIFT, 3, sunxi_input2_mux_text);
static const struct soc_enum input3_mux_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, INPUT3_SHIFT, 1, sunxi_input3_mux_text);

static const struct snd_kcontrol_new input1_mux =
	SOC_DAPM_ENUM_EXT("Input1 Mux", input1_mux_enum, sunxi_get_input_mux, sunxi_set_input_mux);

static const struct snd_kcontrol_new input2_mux =
	SOC_DAPM_ENUM_EXT("Input2 Mux", input2_mux_enum, sunxi_get_input_mux, sunxi_set_input_mux);

static const struct snd_kcontrol_new input3_mux =
	SOC_DAPM_ENUM_EXT("Input3 Mux", input3_mux_enum, sunxi_get_input_mux, sunxi_set_input_mux);

/*audio dapm widget */
static const struct snd_soc_dapm_widget sunxi_codec_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN_E("DACL", "Playback", 0, SUNXI_DAC_REG,
			      DACLEN, 0,
			      sunxi_playback_event,
			      SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_IN_E("DACR", "Playback", 0, SUNXI_DAC_REG,
			      DACREN, 0,
			      sunxi_playback_event,
			      SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_OUT_E("ADC1", "Capture", 0, SND_SOC_NOPM, 0, 0,
			       sunxi_adc1_event,
			       SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("ADC2", "Capture", 0, SND_SOC_NOPM, 0, 0,
			       sunxi_adc2_event,
			       SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("ADC3", "Capture", 0, SND_SOC_NOPM, 0, 0,
			       sunxi_adc3_event,
			       SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX("Input1 Mux", SND_SOC_NOPM, 0, 0, &input1_mux),
	SND_SOC_DAPM_MUX("Input2 Mux", SND_SOC_NOPM, 0, 0, &input2_mux),
	SND_SOC_DAPM_MUX("Input3 Mux", SND_SOC_NOPM, 0, 0, &input3_mux),

	SND_SOC_DAPM_OUTPUT("LINEOUTL_PIN"),
	SND_SOC_DAPM_OUTPUT("LINEOUTR_PIN"),
	SND_SOC_DAPM_OUTPUT("HPOUTL_PIN"),
	SND_SOC_DAPM_OUTPUT("HPOUTR_PIN"),

	SND_SOC_DAPM_INPUT("MIC1_PIN"),
	SND_SOC_DAPM_INPUT("MIC2_PIN"),
	SND_SOC_DAPM_INPUT("MIC3_PIN"),
	SND_SOC_DAPM_INPUT("FMINL_PIN"),
	SND_SOC_DAPM_INPUT("FMINR_PIN"),
	SND_SOC_DAPM_INPUT("LINEINL_PIN"),
	SND_SOC_DAPM_INPUT("LINEINR_PIN"),

	SND_SOC_DAPM_MIC("MIC1", sunxi_mic1_event),
	SND_SOC_DAPM_MIC("MIC2", sunxi_mic2_event),
	SND_SOC_DAPM_MIC("MIC3", sunxi_mic3_event),
	SND_SOC_DAPM_LINE("LINEOUTL", sunxi_lineoutl_event),
	SND_SOC_DAPM_LINE("LINEOUTR", sunxi_lineoutr_event),
	SND_SOC_DAPM_LINE("FMINL", NULL),
	SND_SOC_DAPM_LINE("FMINR", NULL),
	SND_SOC_DAPM_LINE("LINEINL", NULL),
	SND_SOC_DAPM_LINE("LINEINR", NULL),
	SND_SOC_DAPM_HP("HPOUT", sunxi_hpout_event),
	SND_SOC_DAPM_SPK("SPK", sunxi_spk_event),
};

static const struct snd_soc_dapm_route sunxi_codec_dapm_routes[] = {
	/* DAC --> HPOUT_PIN*/
	{"HPOUTL_PIN", NULL, "DACL"},
	{"HPOUTR_PIN", NULL, "DACR"},

	/* DAC --> LINEOUT_PIN*/
	{"LINEOUTL_PIN", NULL, "DACL"},
	{"LINEOUTR_PIN", NULL, "DACR"},

	/*xxx_PIN --> ADC*/
	{"Input1 Mux", "MIC1", "MIC1_PIN"},
	{"Input1 Mux", "FMINL", "FMINL_PIN"},
	{"Input1 Mux", "LINEINL", "LINEINL_PIN"},

	{"Input2 Mux", "MIC2", "MIC2_PIN"},
	{"Input2 Mux", "FMINR", "FMINR_PIN"},
	{"Input2 Mux", "LINEINR", "LINEINR_PIN"},

	{"Input3 Mux", "MIC3", "MIC3_PIN"},

	{"ADC1", NULL, "Input1 Mux"},
	{"ADC2", NULL, "Input2 Mux"},
	{"ADC3", NULL, "Input3 Mux"},
};

#if IS_ENABLED(CONFIG_SND_SOC_SUNXI_JACK_GPIO)
static int sunxi_jack_status_sync(void *data, enum snd_jack_types jack_type);

struct sunxi_jack_gpio sunxi_jack_gpio = {
	.jack_status_sync	= sunxi_jack_status_sync,
};

static int sunxi_jack_status_sync(void *data, enum snd_jack_types jack_type)
{
	(void)data;

	SND_LOG_DEBUG(HLOG, "jack_type %d\n", jack_type);

	return 0;
}
#endif

static void sunxi_codec_init(struct snd_soc_component *component)
{
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(component);
	struct sunxi_codec_dts *dts = &codec->dts;
	struct regmap *regmap = codec->mem.regmap;

	SND_LOG_DEBUG(HLOG, "\n");

	/* In order to ensure that the ADC sampling is normal,
	 * the A chip SOC needs to always open HPLDO and RMC_EN
	 */
	if (sunxi_get_soc_ver() & 0x7) {
		regmap_update_bits(regmap, SUNXI_RAMP_REG, 0x1 << RMC_EN, 0x0 << RMC_EN);
	} else {
		/* A chip */
		regmap_update_bits(regmap, SUNXI_POWER_REG, 0x1 << HPLDO_EN, 0x1 << HPLDO_EN);
		regmap_update_bits(regmap, SUNXI_RAMP_REG, 0x1 << RMC_EN, 0x1 << RMC_EN);
	}

	/* enable ADC volume control */
	regmap_update_bits(regmap, SUNXI_ADC_DIG_CTRL, 0x1 << ADC1_2_VOL_EN, 0x1 << ADC1_2_VOL_EN);
	regmap_update_bits(regmap, SUNXI_ADC_DIG_CTRL, 0x1 << ADC3_VOL_EN, 0x1 << ADC3_VOL_EN);

	regmap_update_bits(regmap, SUNXI_DAC_VOL_CTRL, 0x1 << DAC_VOL_SEL, 0x1 << DAC_VOL_SEL);

	regmap_update_bits(regmap, SUNXI_ADC_FIFOC, 0x1 << ADCDFEN, 0x1 << ADCDFEN);
	regmap_update_bits(regmap, SUNXI_ADC_FIFOC, 0x3 << ADCFDT, 0x2 << ADCFDT);

	regmap_update_bits(regmap, SUNXI_DAC_DPC, 0x3F << DVOL, dts->dac_vol << DVOL);
	regmap_update_bits(regmap, SUNXI_DAC_VOL_CTRL,
			   0xFF << DAC_VOL_L, dts->dacl_vol << DAC_VOL_L);
	regmap_update_bits(regmap, SUNXI_DAC_VOL_CTRL,
			   0xFF << DAC_VOL_R, dts->dacr_vol << DAC_VOL_R);
	regmap_update_bits(regmap, SUNXI_DAC_REG,
			   0x1F << LINEOUT_VOL, dts->lineout_vol << LINEOUT_VOL);
	regmap_update_bits(regmap, SUNXI_HP2_REG,
			   0x7 << HEADPHONE_GAIN, dts->hpout_gain << HEADPHONE_GAIN);

	SND_LOG_DEBUG(HLOG, "adc1_vol:%d, adc2_vol:%d, adc3_vol:%d\n",
			dts->adc1_vol, dts->adc2_vol, dts->adc3_vol);

	regmap_update_bits(regmap, SUNXI_ADC_VOL_CTRL,
			   0x1F << ADC1_VOL, dts->adc1_vol << ADC1_VOL);
	regmap_update_bits(regmap, SUNXI_ADC_VOL_CTRL,
			   0x1F << ADC2_VOL, dts->adc2_vol << ADC2_VOL);
	regmap_update_bits(regmap, SUNXI_ADC_VOL_CTRL,
			   0x1F << ADC3_VOL, dts->adc3_vol << ADC3_VOL);

	SND_LOG_DEBUG(HLOG, "adc1_gain:%d, adc2_gain:%d, adc3_gain:%d\n",
			dts->adc1_gain, dts->adc2_gain, dts->adc3_gain);

	regmap_update_bits(regmap, SUNXI_ADC1_REG,
			   0x1F << ADC1_PGA_GAIN_CTRL, dts->adc1_gain << ADC1_PGA_GAIN_CTRL);
	regmap_update_bits(regmap, SUNXI_ADC2_REG,
			   0x1F << ADC2_PGA_GAIN_CTRL, dts->adc2_gain << ADC2_PGA_GAIN_CTRL);
	regmap_update_bits(regmap, SUNXI_ADC3_REG,
			   0x1F << ADC3_PGA_GAIN_CTRL, dts->adc3_gain << ADC3_PGA_GAIN_CTRL);

	regmap_update_bits(regmap, SUNXI_ADC1_REG,
			   0xFF << ADC1_IOPMIC, 0x55 << ADC1_IOPMIC);
	regmap_update_bits(regmap, SUNXI_ADC2_REG,
			   0xFF << ADC2_IOPMIC, 0x55 << ADC2_IOPMIC);
	regmap_update_bits(regmap, SUNXI_ADC3_REG,
			   0xFF << ADC3_IOPMIC, 0x55 << ADC3_IOPMIC);

	/* input mux default mic */
	regmap_update_bits(regmap, SUNXI_ADC1_REG, 1 << MIC1_PGA_EN, 1 << MIC1_PGA_EN);
	regmap_update_bits(regmap, SUNXI_ADC1_REG, 1 << FMINLEN, 0 << FMINLEN);
	regmap_update_bits(regmap, SUNXI_ADC1_REG, 1 << LINEINLEN, 0 << LINEINLEN);

	regmap_update_bits(regmap, SUNXI_ADC2_REG, 1 << MIC2_PGA_EN, 1 << MIC2_PGA_EN);
	regmap_update_bits(regmap, SUNXI_ADC2_REG, 1 << FMINREN, 0 << FMINREN);
	regmap_update_bits(regmap, SUNXI_ADC2_REG, 1 << LINEINREN, 0 << LINEINREN);

	regmap_update_bits(regmap, SUNXI_ADC3_REG, 1 << MIC3_PGA_EN, 1 << MIC3_PGA_EN);

}

static int sunxi_codec_dai_startup(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(component);
	struct sunxi_codec_dts *dts = &codec->dts;

	SND_LOG_DEBUG(HLOG, "\n");

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		if (dts->rx_sync_en && dts->rx_sync_ctl)
			sunxi_rx_sync_startup(dts->rx_sync_domain, dts->rx_sync_id);
	}

	return 0;
}

static void sunxi_codec_dai_shutdown(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(component);
	struct sunxi_codec_dts *dts = &codec->dts;

	SND_LOG_DEBUG(HLOG, "\n");

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		if (dts->rx_sync_en && dts->rx_sync_ctl)
			sunxi_rx_sync_shutdown(dts->rx_sync_domain, dts->rx_sync_id);
	}
}

static int sunxi_codec_dai_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params,
				     struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(component);
	struct regmap *regmap = codec->mem.regmap;
	int i = 0;

	SND_LOG_DEBUG(HLOG, "\n");

	switch (params_format(params)) {
	case	SNDRV_PCM_FORMAT_S16_LE:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			regmap_update_bits(regmap, SUNXI_DAC_FIFOC,
					   3 << FIFO_MODE, 3 << FIFO_MODE);
			regmap_update_bits(regmap, SUNXI_DAC_FIFOC,
					   1 << TX_SAMPLE_BITS, 0 << TX_SAMPLE_BITS);
		} else {
			regmap_update_bits(regmap, SUNXI_ADC_FIFOC,
					   1 << RX_FIFO_MODE, 1 << RX_FIFO_MODE);
			regmap_update_bits(regmap, SUNXI_ADC_FIFOC,
					   1 << RX_SAMPLE_BITS, 0 << RX_SAMPLE_BITS);
		}
		break;
	case	SNDRV_PCM_FORMAT_S32_LE:
	case	SNDRV_PCM_FORMAT_S24_LE:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			regmap_update_bits(regmap, SUNXI_DAC_FIFOC,
					   3 << FIFO_MODE, 0 << FIFO_MODE);
			regmap_update_bits(regmap, SUNXI_DAC_FIFOC,
					   1 << TX_SAMPLE_BITS, 1 << TX_SAMPLE_BITS);
		} else {
			regmap_update_bits(regmap, SUNXI_ADC_FIFOC,
					   1 << RX_FIFO_MODE, 0 << RX_FIFO_MODE);
			regmap_update_bits(regmap, SUNXI_ADC_FIFOC,
					   1 << RX_SAMPLE_BITS, 1 << RX_SAMPLE_BITS);
		}
		break;
	default:
		break;
	}

	for (i = 0; i < ARRAY_SIZE(sample_rate_conv); i++) {
		if (sample_rate_conv[i].samplerate == params_rate(params)) {
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
				regmap_update_bits(regmap, SUNXI_DAC_FIFOC,
						   0x7 << DAC_FS,
						   sample_rate_conv[i].rate_bit << DAC_FS);
			} else {
				if (sample_rate_conv[i].samplerate > 48000)
					return -EINVAL;
				regmap_update_bits(regmap, SUNXI_ADC_FIFOC,
						   0x7 << ADC_FS,
						   sample_rate_conv[i].rate_bit<<ADC_FS);
			}
		}
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		switch (params_channels(params)) {
		case 1:
			/* DACL & DACR send same data */
			regmap_update_bits(regmap, SUNXI_DAC_FIFOC,
					   0x1 << DAC_MONO_EN, 0x1 << DAC_MONO_EN);
			break;
		case 2:
			regmap_update_bits(regmap, SUNXI_DAC_FIFOC,
					   0x1 << DAC_MONO_EN, 0x0 << DAC_MONO_EN);
			break;
		default:
			 SND_LOG_WARN(HLOG, "not support channels:%u\n", params_channels(params));
			return -EINVAL;
		}
	}

	return 0;
}

static int sunxi_codec_dai_set_pll(struct snd_soc_dai *dai, int pll_id, int source,
				   unsigned int freq_in, unsigned int freq_out)
{
	struct snd_soc_component *component = dai->component;
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(component);
	struct sunxi_codec_clk *clk = &codec->clk;

	SND_LOG_DEBUG(HLOG, "stream -> %s, freq_in ->%u, freq_out ->%u\n",
		      pll_id ? "IN" : "OUT", freq_in, freq_out);

	return snd_sunxi_clk_rate(clk, pll_id, freq_in, freq_out);
}

static int sunxi_codec_dai_prepare(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(component);
	struct regmap *regmap = codec->mem.regmap;

	SND_LOG_DEBUG(HLOG, "\n");

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		regmap_update_bits(regmap, SUNXI_DAC_FIFOC, 1 << FIFO_FLUSH, 1 << FIFO_FLUSH);
		regmap_write(regmap, SUNXI_DAC_FIFOS,
			     1 << DAC_TXE_INT | 1 << DAC_TXU_INT | 1 << DAC_TXO_INT);
		regmap_write(regmap, SUNXI_DAC_CNT, 0);
	} else {
		regmap_update_bits(regmap, SUNXI_ADC_FIFOC,
				   1 << ADC_FIFO_FLUSH, 1 << ADC_FIFO_FLUSH);
		regmap_write(regmap, SUNXI_ADC_FIFOS, 1 << ADC_RXA_INT | 1 << ADC_RXO_INT);
		regmap_write(regmap, SUNXI_ADC_CNT, 0);
	}

	return 0;
}

static int sunxi_codec_dai_trigger(struct snd_pcm_substream *substream,
				   int cmd, struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(component);
	struct regmap *regmap = codec->mem.regmap;
	struct sunxi_codec_dts *dts = &codec->dts;

	SND_LOG_DEBUG(HLOG, "\n");

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			regmap_update_bits(regmap, SUNXI_DAC_FIFOC,
					   1 << DAC_DRQ_EN, 1 << DAC_DRQ_EN);
		} else {
			regmap_update_bits(regmap, SUNXI_ADC_FIFOC,
					   1 << ADC_DRQ_EN, 1 << ADC_DRQ_EN);
			if (dts->rx_sync_en)
				sunxi_rx_sync_control(dts->rx_sync_domain, dts->rx_sync_id, 1);
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			regmap_update_bits(regmap, SUNXI_DAC_FIFOC,
					   1 << DAC_DRQ_EN, 0 << DAC_DRQ_EN);
		} else {
			regmap_update_bits(regmap, SUNXI_ADC_FIFOC,
					   1 << ADC_DRQ_EN, 0 << ADC_DRQ_EN);
			if (dts->rx_sync_en)
				sunxi_rx_sync_control(dts->rx_sync_domain, dts->rx_sync_id, 0);
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct snd_soc_dai_ops sunxi_codec_dai_ops = {
	.startup	= sunxi_codec_dai_startup,
	.shutdown	= sunxi_codec_dai_shutdown,
	.hw_params	= sunxi_codec_dai_hw_params,
	// .set_sysclk	= sunxi_codec_dai_set_sysclk,
	.set_pll	= sunxi_codec_dai_set_pll,
	.trigger	= sunxi_codec_dai_trigger,
	.prepare	= sunxi_codec_dai_prepare,
};

static struct snd_soc_dai_driver sunxi_codec_dai = {
	.name = DRV_NAME,
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates	= SNDRV_PCM_RATE_8000_192000
			| SNDRV_PCM_RATE_KNOT,
		.formats = SNDRV_PCM_FMTBIT_S16_LE
			| SNDRV_PCM_FMTBIT_S24_LE
			| SNDRV_PCM_FMTBIT_S32_LE,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 3,
		.rates = SNDRV_PCM_RATE_8000_48000
			| SNDRV_PCM_RATE_KNOT,
		.formats = SNDRV_PCM_FMTBIT_S16_LE
			| SNDRV_PCM_FMTBIT_S24_LE
			| SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops = &sunxi_codec_dai_ops,
};

static void snd_sunxi_dts_params_init(struct platform_device *pdev, struct sunxi_codec_dts *dts)

{
	int ret = 0;
	unsigned int temp_val;
	struct device_node *np = pdev->dev.of_node;

	SND_LOG_DEBUG(HLOG, "\n");

	dts->rx_sync_en = of_property_read_bool(np, "rx-sync-en");
	dts->tx_hub_en = of_property_read_bool(np, "tx-hub-en");

	ret = of_property_read_u32(np, "dac-vol", &temp_val);
	if (ret < 0) {
		SND_LOG_WARN(HLOG, "digital volume get failed, use default vol");
		dts->dac_vol = 0;
	} else {
		dts->dac_vol = 63 - temp_val;
	}

	ret = of_property_read_u32(np, "dacl-vol", &temp_val);
	if (ret < 0) {
		SND_LOG_WARN(HLOG, "dacl volume get failed, use default vol");
		dts->dacl_vol = 160;
	} else {
		dts->dacl_vol = temp_val;
	}

	ret = of_property_read_u32(np, "dacr-vol", &temp_val);
	if (ret < 0) {
		SND_LOG_WARN(HLOG, "dacr volume get failed, use default vol");
		dts->dacr_vol = 160;
	} else {
		dts->dacr_vol = temp_val;
	}

	ret = of_property_read_u32(np, "lineout-vol", &temp_val);
	if (ret < 0) {
		SND_LOG_WARN(HLOG, "lineout volume get failed, use default vol");
		dts->lineout_vol = 0;
	} else {
		dts->lineout_vol = 31 - temp_val;
	}

	ret = of_property_read_u32(np, "hpout-gain", &temp_val);
	if (ret < 0) {
		SND_LOG_WARN(HLOG, "hpout_gain volume get failed, use default vol");
		dts->hpout_gain = 0;
	} else {
		dts->hpout_gain = 7 - temp_val;
	}

	ret = of_property_read_u32(np, "adc1-vol", &temp_val);
	if (ret < 0) {
		SND_LOG_WARN(HLOG, "adc1_vol get failed, use default vol");
		dts->adc1_vol = 160;
	} else {
		dts->adc1_vol = temp_val;
	}
	ret = of_property_read_u32(np, "adc2-vol", &temp_val);
	if (ret < 0) {
		SND_LOG_WARN(HLOG, "adc2_vol get failed, use default vol");
		dts->adc2_vol = 160;
	} else {
		dts->adc2_vol = temp_val;
	}
	ret = of_property_read_u32(np, "adc3-vol", &temp_val);
	if (ret < 0) {
		dts->adc3_vol = 160;
		SND_LOG_WARN(HLOG, "mic3_vol get failed, use default vol");
	} else {
		dts->adc3_vol = temp_val;
	}

	ret = of_property_read_u32(np, "adc1-gain", &temp_val);
	if (ret < 0) {
		SND_LOG_WARN(HLOG, "adc1_gain get failed, use default vol");
		dts->adc1_gain = 31;
	} else {
		dts->adc1_gain = temp_val;
	}
	ret = of_property_read_u32(np, "adc2-gain", &temp_val);
	if (ret < 0) {
		SND_LOG_WARN(HLOG, "adc2_gain get failed, use default vol");
		dts->adc2_gain = 31;
	} else {
		dts->adc2_gain = temp_val;
	}
	ret = of_property_read_u32(np, "adc3-gain", &temp_val);
	if (ret < 0) {
		dts->adc3_gain = 31;
		SND_LOG_WARN(HLOG, "mic3_gain get failed, use default vol");
	} else {
		dts->adc3_gain = temp_val;
	}

	ret = of_property_read_u32(np, "fminl-gain", &temp_val);
	if (ret < 0) {
		dts->fminl_gain = 0;
		SND_LOG_WARN(HLOG, "fminl_gain get failed, use default vol");
	} else {
		dts->fminl_gain = temp_val;
	}

	ret = of_property_read_u32(np, "fminr-gain", &temp_val);
	if (ret < 0) {
		dts->fminr_gain = 0;
		SND_LOG_WARN(HLOG, "fminr_gain get failed, use default vol");
	} else {
		dts->fminr_gain = temp_val;
	}

	ret = of_property_read_u32(np, "lineinl-gain", &temp_val);
	if (ret < 0) {
		dts->lineinl_gain = 0;
		SND_LOG_WARN(HLOG, "lineinl_gain get failed, use default vol");
	} else {
		dts->lineinl_gain = temp_val;
	}

	ret = of_property_read_u32(np, "lineinr-gain", &temp_val);
	if (ret < 0) {
		dts->lineinr_gain = 0;
		SND_LOG_WARN(HLOG, "lineinr_gain get failed, use default vol");
	} else {
		dts->lineinr_gain = temp_val;
	}

	SND_LOG_DEBUG(HLOG, "dac_vol:%d, lineout_vol:%d, mic1_gain:%d, mic2_gain:%d,"
		      "mic3_gain:%d, rx_sync_en:%d, tx-hub-en:%d\n",
		      dts->dac_vol, dts->lineout_vol, dts->adc1_gain, dts->adc2_gain,
		      dts->adc3_gain, dts->rx_sync_en, dts->tx_hub_en);

	return;
}

static int sunxi_codec_component_probe(struct snd_soc_component *component)
{
	int ret;
	struct snd_soc_dapm_context *dapm = &component->dapm;
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(component);
	struct sunxi_codec_dts *dts = &codec->dts;
	struct regmap *regmap = codec->mem.regmap;

	SND_LOG_DEBUG(HLOG, "\n");

	/* component kcontrols -> rx_sync */
	if (dts->rx_sync_en) {
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

	if (dts->tx_hub_en) {
		ret = snd_soc_add_component_controls(component, sunxi_tx_hub_controls, ARRAY_SIZE(sunxi_tx_hub_controls));

		if (ret)
			SND_LOG_ERR(HLOG, "add tx_hub kcontrols failed\n");

	}

	/* component kcontrols -> codec */
	ret = snd_soc_add_component_controls(component, sunxi_codec_controls,
					     ARRAY_SIZE(sunxi_codec_controls));
	if (ret)
		SND_LOG_ERR(HLOG, "register codec kcontrols failed\n");

	/* dapm-widget */
	ret = snd_soc_dapm_new_controls(dapm, sunxi_codec_dapm_widgets,
					ARRAY_SIZE(sunxi_codec_dapm_widgets));
	if (ret)
		SND_LOG_ERR(HLOG, "register codec dapm_widgets failed\n");

	/* dapm-routes */
	ret = snd_soc_dapm_add_routes(dapm, sunxi_codec_dapm_routes,
				      ARRAY_SIZE(sunxi_codec_dapm_routes));
	if (ret)
		SND_LOG_ERR(HLOG, "register codec dapm_routes failed\n");

	sunxi_codec_init(component);

#if IS_ENABLED(CONFIG_SND_SOC_SUNXI_JACK_GPIO)
	sunxi_jack_gpio.pdev = codec->pdev;
	snd_sunxi_jack_gpio_init((void *)(&sunxi_jack_gpio));
#endif

	return 0;
}

static void sunxi_codec_component_remove(struct snd_soc_component *component)
{
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(component);
	struct sunxi_codec_dts *dts = &codec->dts;

	SND_LOG_DEBUG(HLOG, "\n");

	if (dts->rx_sync_en)
		sunxi_rx_sync_unregister_cb(dts->rx_sync_domain, dts->rx_sync_id);

#if IS_ENABLED(CONFIG_SND_SOC_SUNXI_JACK_GPIO)
	snd_sunxi_jack_gpio_exit((void *)(&sunxi_jack_gpio));
#endif
}

static int sunxi_codec_component_suspend(struct snd_soc_component *component)
{
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(component);
	struct regmap *regmap = codec->mem.regmap;

	SND_LOG_DEBUG(HLOG, "\n");

	snd_sunxi_save_reg(regmap, sunxi_reg_labels);
	snd_sunxi_pa_pin_disable(codec->pa_cfg, codec->pa_pin_max);
	snd_sunxi_rglt_disable(&codec->rglt);
	snd_sunxi_clk_disable(&codec->clk);

	return 0;
}

static int sunxi_codec_component_resume(struct snd_soc_component *component)
{
	int ret;
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(component);
	struct regmap *regmap = codec->mem.regmap;

	SND_LOG_DEBUG(HLOG, "\n");

	snd_sunxi_pa_pin_disable(codec->pa_cfg, codec->pa_pin_max);
	ret = snd_sunxi_clk_enable(&codec->clk);
	if (ret) {
		SND_LOG_ERR(HLOG, "clk enable failed\n");
		return ret;
	}
	ret = snd_sunxi_rglt_enable(&codec->rglt);
	if (ret) {
		SND_LOG_ERR(HLOG, "regulator enable failed\n");
		return ret;
	}
	sunxi_codec_init(component);
	snd_sunxi_echo_reg(regmap, sunxi_reg_labels);

	return 0;
}

static struct snd_soc_component_driver sunxi_codec_component_dev = {
	.name		= DRV_NAME,
	.probe		= sunxi_codec_component_probe,
	.remove		= sunxi_codec_component_remove,
	.suspend	= sunxi_codec_component_suspend,
	.resume		= sunxi_codec_component_resume,
};

/* regmap configuration */
static const struct regmap_config sunxi_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = SUNXI_AUDIO_MAX_REG,
	.cache_type = REGCACHE_NONE,
};

static int snd_sunxi_mem_init(struct platform_device *pdev, struct sunxi_codec_mem *mem)
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

	mem->regmap = devm_regmap_init_mmio(&pdev->dev, mem->membase, &sunxi_regmap_config);
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

static void snd_sunxi_mem_exit(struct platform_device *pdev, struct sunxi_codec_mem *mem)
{
	SND_LOG_DEBUG(HLOG, "\n");

	devm_iounmap(&pdev->dev, mem->membase);
	devm_release_mem_region(&pdev->dev, mem->memregion->start, resource_size(mem->memregion));
}

static int snd_sunxi_clk_init(struct platform_device *pdev, struct sunxi_codec_clk *clk)
{
	int ret = 0;
	struct device_node *np = pdev->dev.of_node;

	SND_LOG_DEBUG(HLOG, "\n");

	/* get rst clk */
	clk->codec_clk_rst = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR_OR_NULL(clk->codec_clk_rst)) {
		SND_LOG_ERR(HLOG, "clk rst get failed\n");
		ret =  PTR_ERR(clk->codec_clk_rst);
		goto err_get_clk_rst;
	}

	/* get bus clk */
	clk->codec_clk_bus = of_clk_get_by_name(np, "audio_clk_bus");
	if (IS_ERR_OR_NULL(clk->codec_clk_bus)) {
		SND_LOG_ERR(HLOG, "audio_clk_bus get failed\n");
		ret = PTR_ERR(clk->codec_clk_bus);
		goto err_get_clk_bus;
	}

	/* get parent clk */
	clk->pllaudio0 = of_clk_get_by_name(np, "pll_audio0");
	if (IS_ERR_OR_NULL(clk->codec_clk_bus)) {
		SND_LOG_ERR(HLOG, "pll_audio0 get failed\n");
		ret = PTR_ERR(clk->codec_clk_bus);
		goto err_get_pll_audio0;
	}

	clk->pllaudio1_div5 = of_clk_get_by_name(np, "pll_audio1_div5");
	if (IS_ERR_OR_NULL(clk->codec_clk_bus)) {
		SND_LOG_ERR(HLOG, "pll_audio1_div5 get failed\n");
		ret = PTR_ERR(clk->codec_clk_bus);
		goto err_get_pll_audio1_div5;
	}

	/* get module clk */
	clk->dacclk = of_clk_get_by_name(np, "audio_clk_dac");
	if (IS_ERR_OR_NULL(clk->codec_clk_bus)) {
		SND_LOG_ERR(HLOG, "audio_clk_dac get failed\n");
		ret = PTR_ERR(clk->codec_clk_bus);
		goto err_get_clk_audio_dac;
	}

	clk->adcclk = of_clk_get_by_name(np, "audio_clk_adc");
	if (IS_ERR_OR_NULL(clk->codec_clk_bus)) {
		SND_LOG_ERR(HLOG, "audio_clk_adc get failed\n");
		ret = PTR_ERR(clk->codec_clk_bus);
		goto err_get_clk_audio_adc;
	}

	/* set clk audio parent of pllaudio */
	if (clk_set_parent(clk->dacclk, clk->pllaudio0)) {
		SND_LOG_ERR(HLOG, "set parent of dacclk to pllaudio0 failed");
		goto err_set_parent;
	}

	if (clk_set_parent(clk->adcclk, clk->pllaudio0)) {
		SND_LOG_ERR(HLOG, "set parent of adcclk to pllaudio0 failed");
		goto err_set_parent;
	}

	/* 22579200 * n */
	if (clk_set_rate(clk->pllaudio0, 22579200)) {
		SND_LOG_ERR(HLOG, "pllaudio0 set rate failed");
		goto err_set_rate;
	}

	/* 24576000 * n */
	if (clk_set_rate(clk->pllaudio1_div5, 614400000)) {
		SND_LOG_ERR(HLOG, "pllaudio1_div5 set rate failed");
		goto err_set_rate;
	}

	ret = snd_sunxi_clk_enable(clk);
	if (ret) {
		SND_LOG_ERR(HLOG, "clk enable failed\n");
		ret = -EINVAL;
		goto err_clk_enable;
	}

	return 0;

err_get_clk_bus:
	clk_put(clk->codec_clk_bus);
err_get_pll_audio0:
	clk_put(clk->pllaudio0);
err_get_pll_audio1_div5:
	clk_put(clk->pllaudio1_div5);
err_get_clk_audio_dac:
	clk_put(clk->dacclk);
err_get_clk_audio_adc:
	clk_put(clk->adcclk);
err_clk_enable:
	snd_sunxi_clk_disable(clk);
err_get_clk_rst:
err_set_rate:
err_set_parent:
	return ret;
}

static void snd_sunxi_clk_exit(struct sunxi_codec_clk *clk)
{
	SND_LOG_DEBUG(HLOG, "\n");

	snd_sunxi_clk_disable(clk);
	clk_put(clk->adcclk);
	clk_put(clk->dacclk);
	clk_put(clk->pllaudio1_div5);
	clk_put(clk->pllaudio0);
	clk_put(clk->codec_clk_bus);
}

static int snd_sunxi_clk_enable(struct sunxi_codec_clk *clk)
{
	int ret = 0;

	SND_LOG_DEBUG(HLOG, "\n");

	if (reset_control_deassert(clk->codec_clk_rst)) {
		SND_LOG_ERR(HLOG, "clk rst deassert failed\n");
		ret = -EINVAL;
		goto err_deassert_rst;
	}

	if (clk_prepare_enable(clk->codec_clk_bus)) {

		SND_LOG_ERR(HLOG, "clk bus enable failed\n");
		goto err_enable_clk_bus;
	}

	if (clk_prepare_enable(clk->pllaudio0)) {
		SND_LOG_ERR(HLOG, "pllaudio0 enable failed\n");
		goto err_enable_clk_pll_audio0;
	}

	if (clk_prepare_enable(clk->pllaudio1_div5)) {
		SND_LOG_ERR(HLOG, "pllaudio1_div5 enable failed\n");
		goto err_enable_clk_pllaudio1_div5;
	}

	if (clk_prepare_enable(clk->dacclk)) {
		SND_LOG_ERR(HLOG, "dacclk enable failed\n");
		goto err_enable_clk_dac_clk;
	}

	if (clk_prepare_enable(clk->adcclk)) {
		SND_LOG_ERR(HLOG, "adcclk enable failed\n");
		goto err_enable_clk_adc_clk;
	}

	return 0;

err_enable_clk_adc_clk:
	clk_disable_unprepare(clk->adcclk);
err_enable_clk_dac_clk:
	clk_disable_unprepare(clk->dacclk);
err_enable_clk_pllaudio1_div5:
	clk_disable_unprepare(clk->pllaudio1_div5);
err_enable_clk_pll_audio0:
	clk_disable_unprepare(clk->pllaudio0);
err_enable_clk_bus:
	clk_disable_unprepare(clk->codec_clk_bus);
err_deassert_rst:
	reset_control_assert(clk->codec_clk_rst);
	return ret;
}

static void snd_sunxi_clk_disable(struct sunxi_codec_clk *clk)
{
	SND_LOG_DEBUG(HLOG, "\n");

	clk_disable_unprepare(clk->adcclk);
	clk_disable_unprepare(clk->dacclk);
	clk_disable_unprepare(clk->pllaudio1_div5);
	clk_disable_unprepare(clk->pllaudio0);
	clk_disable_unprepare(clk->codec_clk_bus);
	reset_control_assert(clk->codec_clk_rst);
}

static int snd_sunxi_clk_rate(struct sunxi_codec_clk *clk, int stream,
			      unsigned int freq_in, unsigned int freq_out)
{
	SND_LOG_DEBUG(HLOG, "\n");

	if (stream  == SNDRV_PCM_STREAM_PLAYBACK) {
		if (freq_in % 24576000 == 0) {
			/* set clk source [24.576MHz * n] to surpport playback */
			if (clk_set_parent(clk->dacclk, clk->pllaudio1_div5)) {
				SND_LOG_ERR(HLOG, "set parent of dacclk to pllaudio1_div5 failed");
				return -EINVAL;
			}

			if (clk_set_rate(clk->dacclk, freq_out)) {
				SND_LOG_ERR(HLOG, "codec set dac clk rate failed");
				return -EINVAL;
			}
		} else {
			/* set clk source [22.5792MHz * n] to surpport playback */
			if (clk_set_parent(clk->dacclk, clk->pllaudio0)) {
				SND_LOG_ERR(HLOG, "set parent of dacclk to pllaudio0 failed");
				return -EINVAL;
			}

			if (clk_set_rate(clk->dacclk, freq_out)) {
				SND_LOG_ERR(HLOG, "codec set dac clk rate failed");
				return -EINVAL;
			}
		}
	} else {
		if (freq_in % 24576000 == 0) {
			/* set clk source [24.576MHz * n] to surpport capture */
			if (clk_set_parent(clk->adcclk, clk->pllaudio1_div5)) {
				SND_LOG_ERR(HLOG, "set parent of adcclk to pllaudio1_div5 failed");
				return -EINVAL;
			}

			if (clk_set_rate(clk->adcclk, freq_out)) {
				SND_LOG_ERR(HLOG, "codec set adc clk rate failed");
				return -EINVAL;
			}

		} else {
			/* set clk source [22.5792MHz * n] to surpport capture */
			if (clk_set_parent(clk->adcclk, clk->pllaudio0)) {
				SND_LOG_ERR(HLOG, "set parent of adcclk to pllaudio0 failed");
				return -EINVAL;
			}

			if (clk_set_rate(clk->adcclk, freq_out)) {
				SND_LOG_ERR(HLOG, "codec set adc clk rate failed");
				return -EINVAL;
			}
		}
	}

	return 0;
}

static int snd_sunxi_rglt_init(struct platform_device *pdev, struct sunxi_codec_rglt *rglt)
{
	int ret = 0;
	unsigned int temp_val;
	struct device_node *np = pdev->dev.of_node;

	SND_LOG_DEBUG(HLOG, "\n");

	rglt->avcc_external = of_property_read_bool(np, "avcc-external");
	ret = of_property_read_u32(np, "avcc-vol", &temp_val);
	if (ret < 0) {
		rglt->avcc_vol = 1800000;	/* default avcc voltage: 1.8v */
	} else {
		rglt->avcc_vol = temp_val;
	}

	if (rglt->avcc_external) {
		SND_LOG_DEBUG(HLOG, "use external avcc\n");
		rglt->avcc = regulator_get(&pdev->dev, "avcc");
		if (IS_ERR_OR_NULL(rglt->avcc)) {
			SND_LOG_DEBUG(HLOG, "unused external pmu\n");
		} else {
			ret = regulator_set_voltage(rglt->avcc, rglt->avcc_vol, rglt->avcc_vol);
			if (ret < 0) {
				SND_LOG_ERR(HLOG, "set avcc voltage failed\n");
				ret = -EFAULT;
				goto err_rglt;
			}
			ret = regulator_enable(rglt->avcc);
			if (ret < 0) {
				SND_LOG_ERR(HLOG, "enable avcc failed\n");
				ret = -EFAULT;
				goto err_rglt;
			}
		}
	} else {
		SND_LOG_DEBUG(HLOG, "unset external avcc\n");
		return 0;
	}

	return 0;

err_rglt:
	snd_sunxi_rglt_exit(rglt);
	return ret;
}

static void snd_sunxi_rglt_exit(struct sunxi_codec_rglt *rglt)
{
	SND_LOG_DEBUG(HLOG, "\n");

	if (rglt->avcc)
		if (!IS_ERR_OR_NULL(rglt->avcc)) {
			regulator_disable(rglt->avcc);
			regulator_put(rglt->avcc);
		}
}

static int snd_sunxi_rglt_enable(struct sunxi_codec_rglt *rglt)
{
	int ret;

	SND_LOG_DEBUG(HLOG, "\n");

	if (rglt->avcc)
		if (!IS_ERR_OR_NULL(rglt->avcc)) {
			ret = regulator_enable(rglt->avcc);
			if (ret) {
				SND_LOG_ERR(HLOG, "enable avcc failed\n");
				return -1;
			}
		}

	return 0;
}

static void snd_sunxi_rglt_disable(struct sunxi_codec_rglt *rglt)
{
	SND_LOG_DEBUG(HLOG, "\n");

	if (rglt->avcc)
		if (!IS_ERR_OR_NULL(rglt->avcc)) {
			regulator_disable(rglt->avcc);
		}
}

/* sysfs debug */
static ssize_t snd_sunxi_debug_show_reg(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	size_t count = 0;

	count += sprintf(buf + count, "usage->read : echo [num] > audio_reg\n");
	count += sprintf(buf + count, "usage->write: echo [reg] [value] > audio_reg\n");

	count += sprintf(buf + count, "num: 0.all\n");

	return count;
}

static ssize_t snd_sunxi_debug_store_reg(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct sunxi_codec *codec = dev_get_drvdata(dev);
	struct regmap *regmap = codec->mem.regmap;
	int scanf_cnt;
	unsigned int num = 0, i = 0;
	unsigned int output_reg_val;
	unsigned int input_reg_val;
	unsigned int input_reg_offset;
	unsigned int size = ARRAY_SIZE(sunxi_reg_labels);

	if (buf[1] == 'x')
		scanf_cnt = sscanf(buf, "0x%x 0x%x", &input_reg_offset, &input_reg_val);
	else
		scanf_cnt = sscanf(buf, "%x", &num);

	if (scanf_cnt <= 0 || num != 0) {
		pr_err("please get the usage by\"cat audio_reg\"\n");
		return count;
	}

	if (scanf_cnt == 1) {
		while ((i < size) && (sunxi_reg_labels[i].name != NULL)) {
			regmap_read(regmap, sunxi_reg_labels[i].address, &output_reg_val);
			pr_info("%-32s [0x%03x]: 0x%8x :0x%x\n",
				sunxi_reg_labels[i].name,
				sunxi_reg_labels[i].address, output_reg_val,
				sunxi_reg_labels[i].value);
			i++;
		}
		return count;
	} else if (scanf_cnt == 2) {
		if (input_reg_offset > SUNXI_AUDIO_MAX_REG) {
			pr_err("reg offset > audio max reg[0x%x]\n", SUNXI_AUDIO_MAX_REG);
			return count;
		}

		regmap_read(regmap, input_reg_offset, &output_reg_val);
		pr_info("reg[0x%03x]: 0x%x (old)\n", input_reg_offset, output_reg_val);
		regmap_write(regmap, input_reg_offset, input_reg_val);
		regmap_read(regmap, input_reg_offset, &output_reg_val);
		pr_info("reg[0x%03x]: 0x%x (new)\n", input_reg_offset, output_reg_val);
	}

	return count;
}

static DEVICE_ATTR(audio_reg, 0644, snd_sunxi_debug_show_reg, snd_sunxi_debug_store_reg);

static struct attribute *audio_debug_attrs[] = {
	&dev_attr_audio_reg.attr,
	NULL,
};

static struct attribute_group debug_attr = {
	.name	= "audio_debug",
	.attrs	= audio_debug_attrs,
};

static int sunxi_codec_dev_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	struct sunxi_codec *codec;
	struct sunxi_codec_mem *mem;
	struct sunxi_codec_clk *clk;
	struct sunxi_codec_rglt *rglt;
	struct sunxi_codec_dts *dts;

	SND_LOG_DEBUG(HLOG, "\n");

	/* sunxi codec info */
	codec = devm_kzalloc(dev, sizeof(struct sunxi_codec), GFP_KERNEL);
	if (!codec) {
		SND_LOG_ERR(HLOG, "can't allocate sunxi codec memory\n");
		ret = -ENOMEM;
		goto err_devm_kzalloc;
	}
	dev_set_drvdata(dev, codec);
	mem = &codec->mem;
	clk = &codec->clk;
	rglt = &codec->rglt;
	dts = &codec->dts;
	codec->pdev = pdev;

	/* memio init */
	ret = snd_sunxi_mem_init(pdev, mem);
	if (ret) {
		SND_LOG_ERR(HLOG, "mem init failed\n");
		ret = -ENOMEM;
		goto err_mem_init;
	}

	/* clk init */
	ret = snd_sunxi_clk_init(pdev, clk);
	if (ret) {
		SND_LOG_ERR(HLOG, "clk init failed\n");
		ret = -ENOMEM;
		goto err_clk_init;
	}

	/* regulator init */
	ret = snd_sunxi_rglt_init(pdev, rglt);
	if (ret) {
		SND_LOG_ERR(HLOG, "regulator init failed\n");
		ret = -ENOMEM;
		goto err_regulator_init;
	}

	/* dts_params init */
	snd_sunxi_dts_params_init(pdev, dts);

	/* pa_pin init */
	codec->pa_cfg = snd_sunxi_pa_pin_init(pdev, &codec->pa_pin_max);

	/* alsa component register */
	ret = snd_soc_register_component(dev,
					 &sunxi_codec_component_dev,
					 &sunxi_codec_dai, 1);
	if (ret) {
		SND_LOG_ERR(HLOG, "internal-codec component register failed\n");
		ret = -ENOMEM;
		goto err_register_component;
	}

	ret = snd_sunxi_sysfs_create_group(pdev, &debug_attr);
	if (ret)
		SND_LOG_WARN(HLOG, "sysfs debug create failed\n");

	SND_LOG_DEBUG(HLOG, "register internal-codec codec success\n");

	return 0;

err_register_component:
	snd_sunxi_rglt_exit(rglt);
err_regulator_init:
	snd_sunxi_clk_exit(clk);
err_clk_init:
	snd_sunxi_mem_exit(pdev, mem);
err_mem_init:
	devm_kfree(dev, codec);
err_devm_kzalloc:
	of_node_put(np);

	return ret;
}

static int sunxi_codec_dev_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sunxi_codec *codec = dev_get_drvdata(dev);
	struct sunxi_codec_mem *mem = &codec->mem;
	struct sunxi_codec_clk *clk = &codec->clk;
	struct sunxi_codec_rglt *rglt = &codec->rglt;

	SND_LOG_DEBUG(HLOG, "\n");

	/* alsa component unregister */
	snd_sunxi_sysfs_remove_group(pdev, &debug_attr);
	snd_soc_unregister_component(dev);

	snd_sunxi_mem_exit(pdev, mem);
	snd_sunxi_clk_exit(clk);
	snd_sunxi_rglt_exit(rglt);
	snd_sunxi_pa_pin_exit(pdev, codec->pa_cfg, codec->pa_pin_max);

	/* sunxi codec custom info free */
	devm_kfree(dev, codec);
	of_node_put(pdev->dev.of_node);

	SND_LOG_DEBUG(HLOG, "unregister internal-codec codec success\n");

	return 0;
}

static const struct of_device_id sunxi_codec_of_match[] = {
	{ .compatible = "allwinner," DRV_NAME, },
	{},
};
MODULE_DEVICE_TABLE(of, sunxi_codec_of_match);

static struct platform_driver sunxi_codec_driver = {
	.driver	= {
		.name		= DRV_NAME,
		.owner		= THIS_MODULE,
		.of_match_table	= sunxi_codec_of_match,
	},
	.probe	= sunxi_codec_dev_probe,
	.remove	= sunxi_codec_dev_remove,
};

int __init sunxi_codec_dev_init(void)
{
	int ret;

	ret = platform_driver_register(&sunxi_codec_driver);
	if (ret != 0) {
		SND_LOG_ERR(HLOG, "platform driver register failed\n");
		return -EINVAL;
	}

	return ret;
}

void __exit sunxi_codec_dev_exit(void)
{
	platform_driver_unregister(&sunxi_codec_driver);
}
late_initcall(sunxi_codec_dev_init);
module_exit(sunxi_codec_dev_exit);

MODULE_DESCRIPTION("SUNXI Codec ASoC driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.4");
MODULE_ALIAS("sunxi soundcard codec of internal-codec");
