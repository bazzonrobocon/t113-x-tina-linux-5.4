/*
 * sound\soc\sunxi\snd_sunxi_dap.c
 * (C) Copyright 2023-2028
 * AllWinner Technology Co., Ltd. <www.allwinnertech.com>
 * Dby <dby@allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#include <linux/regmap.h>

#include "snd_sunxi_log.h"
#include "snd_sunxi_dap.h"

#define HLOG		"DAP"

void snd_sunxi_dap_dacdrc(struct regmap *regmap, bool enable)
{
	SND_LOG_DEBUG(HLOG, "%s\n", enable ? "enable" : "disable");

	if (!regmap) {
		SND_LOG_ERR(HLOG, "regmap is invailed\n");
		return;
	}

	if (enable) {
		regmap_update_bits(regmap, SUNXI_DAC_DAP_CTL,
				   0x1 << DAC_DAP_EN | 0x1 << DDAP_DRC_EN,
				   0x1 << DAC_DAP_EN | 0x1 << DDAP_DRC_EN);
	} else {
		regmap_update_bits(regmap, SUNXI_DAC_DAP_CTL,
				   0x1 << DDAP_DRC_EN, 0x0 << DDAP_DRC_EN);
		return;
	}

	regmap_write(regmap, SUNXI_DAC_DRC_CTRL, 0x00BB);

	/* left peak filter attack time */
	regmap_write(regmap, SUNXI_DAC_DRC_LPFHAT, 0x000B);
	regmap_write(regmap, SUNXI_DAC_DRC_LPFLAT, 0x77F0);

	/* right peak filter attack time */
	regmap_write(regmap, SUNXI_DAC_DRC_RPFHAT, 0x000B);
	regmap_write(regmap, SUNXI_DAC_DRC_RPFLAT, 0x77F0);

	/* Left peak filter release time */
	regmap_write(regmap, SUNXI_DAC_DRC_LPFHRT, 0x00FF);
	regmap_write(regmap, SUNXI_DAC_DRC_LPFLRT, 0xE1F8);

	/* Right peak filter release time */
	regmap_write(regmap, SUNXI_DAC_DRC_RPFHRT, 0x00FF);
	regmap_write(regmap, SUNXI_DAC_DRC_RPFLRT, 0xE1F8);

	/* Left RMS filter attack time */
	regmap_write(regmap, SUNXI_DAC_DRC_LRMSHAT, 0x0001);
	regmap_write(regmap, SUNXI_DAC_DRC_LRMSLAT, 0x2BB0);

	/* Right RMS filter attack time */
	regmap_write(regmap, SUNXI_DAC_DRC_RRMSHAT, 0x0001);
	regmap_write(regmap, SUNXI_DAC_DRC_RRMSLAT, 0x2BB0);

	/* CT */
	regmap_write(regmap, SUNXI_DAC_DRC_HCT, 0x0B92);
	regmap_write(regmap, SUNXI_DAC_DRC_LCT, 0x461C);

	/* Kc */
	regmap_write(regmap, SUNXI_DAC_DRC_HKC, 0x0087);
	regmap_write(regmap, SUNXI_DAC_DRC_LKC, 0x3ECB);

	/* OPC */
	regmap_write(regmap, SUNXI_DAC_DRC_HOPC, 0xFBAE);
	regmap_write(regmap, SUNXI_DAC_DRC_LOPC, 0x765C);

	/* LT */
	regmap_write(regmap, SUNXI_DAC_DRC_HLT, 0x05B3);
	regmap_write(regmap, SUNXI_DAC_DRC_LLT, 0xE068);

	/* Ki */
	regmap_write(regmap, SUNXI_DAC_DRC_HKI, 0x0018);
	regmap_write(regmap, SUNXI_DAC_DRC_LKI, 0xDAB8);

	/* OPL */
	regmap_write(regmap, SUNXI_DAC_DRC_HOPL, 0xFEC8);
	regmap_write(regmap, SUNXI_DAC_DRC_LOPL, 0x2E83);

	/* ET */
	regmap_write(regmap, SUNXI_DAC_DRC_HET, 0x0DF3);
	regmap_write(regmap, SUNXI_DAC_DRC_LET, 0xBCAE);

	/* Ke */
	regmap_write(regmap, SUNXI_DAC_DRC_HKE, 0x0345);
	regmap_write(regmap, SUNXI_DAC_DRC_LKE, 0x5554);

	/* OPE */
	regmap_write(regmap, SUNXI_DAC_DRC_HOPE, 0xF815);
	regmap_write(regmap, SUNXI_DAC_DRC_LOPE, 0x2E50);

	/* Kn */
	regmap_write(regmap, SUNXI_DAC_DRC_HKN, 0x0182);
	regmap_write(regmap, SUNXI_DAC_DRC_LKN, 0xFA08);

	/* smooth filter attack time */
	regmap_write(regmap, SUNXI_DAC_DRC_SFHAT, 0x0001);
	regmap_write(regmap, SUNXI_DAC_DRC_SFLAT, 0x7665);

	/* gain smooth filter release time */
	regmap_write(regmap, SUNXI_DAC_DRC_SFHRT, 0x0000);
	regmap_write(regmap, SUNXI_DAC_DRC_SFLRT, 0x0F04);

	/* MXG */
	regmap_write(regmap, SUNXI_DAC_DRC_MXGHS, 0x0352);
	regmap_write(regmap, SUNXI_DAC_DRC_MXGLS, 0x69E0);

	/* MNG */
	regmap_write(regmap, SUNXI_DAC_DRC_MNGHS, 0xF95B);
	regmap_write(regmap, SUNXI_DAC_DRC_MNGLS, 0x2C3F);

	/* EPS */
	regmap_write(regmap, SUNXI_DAC_DRC_EPSHC, 0x0002);
	regmap_write(regmap, SUNXI_DAC_DRC_EPSLC, 0x5600);

	regmap_write(regmap, SUNXI_DAC_DRC_OPT, 0x0000);
	regmap_write(regmap, SUNXI_DAC_DRC_HPFHGAIN, 0x0100);
	regmap_write(regmap, SUNXI_DAC_DRC_HPFLGAIN, 0x0000);
}

void snd_sunxi_dap_dachpf(struct regmap *regmap, bool enable)
{
	SND_LOG_DEBUG(HLOG, "%s\n", enable ? "enable" : "disable");

	if (!regmap) {
		SND_LOG_ERR(HLOG, "regmap is invailed\n");
		return;
	}

	if (enable) {
		regmap_update_bits(regmap, SUNXI_DAC_DAP_CTL,
				   0x1 << DAC_DAP_EN | 0x1 << DDAP_HPF_EN,
				   0x1 << DAC_DAP_EN | 0x1 << DDAP_HPF_EN);
	} else {
		regmap_update_bits(regmap, SUNXI_DAC_DAP_CTL,
				   0x1 << DDAP_HPF_EN, 0x0 << DDAP_HPF_EN);
		return;
	}

	regmap_write(regmap, SUNXI_DAC_DRC_HHPFC, (0xFFFAC1 >> 16) & 0xFFFF);
	regmap_write(regmap, SUNXI_DAC_DRC_LHPFC, 0xFFFAC1 & 0xFFFF);
}

void snd_sunxi_dap_adcdrc(struct regmap *regmap, bool enable)
{
	SND_LOG_DEBUG(HLOG, "%s\n", enable ? "enable" : "disable");

	if (!regmap) {
		SND_LOG_ERR(HLOG, "regmap is invailed\n");
		return;
	}

	if (enable) {
		regmap_update_bits(regmap, SUNXI_ADC_DAP_CTL,
				   0x1 << ADC_DAP_EN | 0x1 << ADAP_DRC_EN,
				   0x1 << ADC_DAP_EN | 0x1 << ADAP_DRC_EN);
	} else {
		regmap_update_bits(regmap, SUNXI_ADC_DAP_CTL,
				   0x1 << ADAP_DRC_EN, 0x0 << ADAP_DRC_EN);
		return;
	}

	regmap_write(regmap, SUNXI_ADC_DRC_CTRL, 0x00BB);

	/* left peak filter attack time */
	regmap_write(regmap, SUNXI_ADC_DRC_LPFHAT, 0x000B);
	regmap_write(regmap, SUNXI_ADC_DRC_LPFLAT, 0x77F0);

	/* right peak filter attack time */
	regmap_write(regmap, SUNXI_ADC_DRC_RPFHAT, 0x000B);
	regmap_write(regmap, SUNXI_ADC_DRC_RPFLAT, 0x77F0);

	/* Left peak filter release time */
	regmap_write(regmap, SUNXI_ADC_DRC_LPFHRT, 0x00FF);
	regmap_write(regmap, SUNXI_ADC_DRC_LPFLRT, 0xE1F8);

	/* Right peak filter release time */
	regmap_write(regmap, SUNXI_ADC_DRC_RPFHRT, 0x00FF);
	regmap_write(regmap, SUNXI_ADC_DRC_RPFLRT, 0xE1F8);

	/* Left RMS filter attack time */
	regmap_write(regmap, SUNXI_ADC_DRC_LRMSHAT, 0x0001);
	regmap_write(regmap, SUNXI_ADC_DRC_LRMSLAT, 0x2BB0);

	/* Right RMS filter attack time */
	regmap_write(regmap, SUNXI_ADC_DRC_RRMSHAT, 0x0001);
	regmap_write(regmap, SUNXI_ADC_DRC_RRMSLAT, 0x2BB0);

	/* CT */
	regmap_write(regmap, SUNXI_ADC_DRC_HCT, 0x0B92);
	regmap_write(regmap, SUNXI_ADC_DRC_LCT, 0x461C);

	/* Kc */
	regmap_write(regmap, SUNXI_ADC_DRC_HKC, 0x0087);
	regmap_write(regmap, SUNXI_ADC_DRC_LKC, 0x3ECB);

	/* OPC */
	regmap_write(regmap, SUNXI_ADC_DRC_HOPC, 0xFBAE);
	regmap_write(regmap, SUNXI_ADC_DRC_LOPC, 0x765C);

	/* LT */
	regmap_write(regmap, SUNXI_ADC_DRC_HLT, 0x05B3);
	regmap_write(regmap, SUNXI_ADC_DRC_LLT, 0xE068);

	/* Ki */
	regmap_write(regmap, SUNXI_ADC_DRC_HKI, 0x0018);
	regmap_write(regmap, SUNXI_ADC_DRC_LKI, 0xDAB8);

	/* OPL */
	regmap_write(regmap, SUNXI_ADC_DRC_HOPL, 0xFEC8);
	regmap_write(regmap, SUNXI_ADC_DRC_LOPL, 0x2E83);

	/* ET */
	regmap_write(regmap, SUNXI_ADC_DRC_HET, 0x0DF3);
	regmap_write(regmap, SUNXI_ADC_DRC_LET, 0xBCAE);

	/* Ke */
	regmap_write(regmap, SUNXI_ADC_DRC_HKE, 0x0345);
	regmap_write(regmap, SUNXI_ADC_DRC_LKE, 0x5554);

	/* OPE */
	regmap_write(regmap, SUNXI_ADC_DRC_HOPE, 0xF815);
	regmap_write(regmap, SUNXI_ADC_DRC_LOPE, 0x2E50);

	/* Kn */
	regmap_write(regmap, SUNXI_ADC_DRC_HKN, 0x0182);
	regmap_write(regmap, SUNXI_ADC_DRC_LKN, 0xFA08);

	/* smooth filter attack time */
	regmap_write(regmap, SUNXI_ADC_DRC_SFHAT, 0x0001);
	regmap_write(regmap, SUNXI_ADC_DRC_SFLAT, 0x7665);

	/* gain smooth filter release time */
	regmap_write(regmap, SUNXI_ADC_DRC_SFHRT, 0x0000);
	regmap_write(regmap, SUNXI_ADC_DRC_SFLRT, 0x0F04);

	/* MXG */
	regmap_write(regmap, SUNXI_ADC_DRC_MXGHS, 0x0352);
	regmap_write(regmap, SUNXI_ADC_DRC_MXGLS, 0x69E0);

	/* MNG */
	regmap_write(regmap, SUNXI_ADC_DRC_MNGHS, 0xF95B);
	regmap_write(regmap, SUNXI_ADC_DRC_MNGLS, 0x2C3F);

	/* EPS */
	regmap_write(regmap, SUNXI_ADC_DRC_EPSHC, 0x0002);
	regmap_write(regmap, SUNXI_ADC_DRC_EPSLC, 0x5600);

	regmap_write(regmap, SUNXI_ADC_DRC_OPT, 0x0000);
	regmap_write(regmap, SUNXI_ADC_DRC_HPFHGAIN, 0x0100);
	regmap_write(regmap, SUNXI_ADC_DRC_HPFLGAIN, 0x0000);
}

void snd_sunxi_dap_adchpf(struct regmap *regmap, bool enable)
{
	SND_LOG_DEBUG(HLOG, "%s\n", enable ? "enable" : "disable");

	if (!regmap) {
		SND_LOG_ERR(HLOG, "regmap is invailed\n");
		return;
	}

	if (enable) {
		regmap_update_bits(regmap, SUNXI_ADC_DAP_CTL,
				   0x1 << ADC_DAP_EN | 0x1 << ADAP_HPF_EN,
				   0x1 << ADC_DAP_EN | 0x1 << ADAP_HPF_EN);
	} else {
		regmap_update_bits(regmap, SUNXI_ADC_DAP_CTL,
				   0x1 << ADAP_HPF_EN, 0x0 << ADAP_HPF_EN);
		return;
	}

	regmap_write(regmap, SUNXI_ADC_DRC_HHPFC, (0xFFFAC1 >> 16) & 0xFFFF);
	regmap_write(regmap, SUNXI_ADC_DRC_LHPFC, 0xFFFAC1 & 0xFFFF);
}
