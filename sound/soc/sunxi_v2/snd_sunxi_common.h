/* sound\soc\sunxi\snd_sunxi_common.h
 * (C) Copyright 2021-2025
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * Dby <dby@allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#ifndef __SND_SUNXI_COMMON_H
#define __SND_SUNXI_COMMON_H

/******* reg label *******/
#define REG_LABEL(constant)	{#constant, constant, 0}
#define REG_LABEL_END		{NULL, 0, 0}

struct audio_reg_label {
	const char *name;
	const unsigned int address;
	unsigned int value;
};

/* EX:
 * static struct audio_reg_label reg_labels[] = {
 * 	REG_LABEL(SUNXI_REG_0),
 * 	REG_LABEL(SUNXI_REG_1),
 * 	REG_LABEL(SUNXI_REG_n),
 * 	REG_LABEL_END,
 * };
 */
int snd_sunxi_save_reg(struct regmap *regmap, struct audio_reg_label *reg_labels);
int snd_sunxi_echo_reg(struct regmap *regmap, struct audio_reg_label *reg_labels);

/******* regulator config *******/
/* board.dts format
 * &xxx {
 *	rglt-max	= <n>;		// n: rglt cnt
 *	rglt0-mode	= "xxx";	// PMU; AUDIO;
 *	rglt0-voltage	= <n>;		// n: vcc voltage
 *	rglt0-supply	= <&pmu_node>;
 *	rglt1-xxx
 *	...
 * };
 *
 * EX: rglt cnt = 2
 * &xxx {
 *	rglt-max	= <2>;
 *	rglt0-mode	= "AUDIO";
 *	rglt0-voltage	= <1800000>;	// 1.8v
 *	//rglt0-supply	= <>;		AUDIO mode, unnecessary.
 *	rglt1-mode	= PMU;
 *	rglt1-voltage	= <3300000>;	// 3.3v
 *	rglt1-supply	= <&reg_aldo1>;	// pmu node
 * };
 */
enum SND_SUNXI_RGLT_MODE {
	SND_SUNXI_RGLT_NULL = -1,
	SND_SUNXI_RGLT_PMU = 0,
	SND_SUNXI_RGLT_AUDIO,
	SND_SUNXI_RGLT_USER,
};

struct snd_sunxi_rglt_unit {
	enum SND_SUNXI_RGLT_MODE mode;
	u32 vcc_vol;
	struct regulator *vcc;
};

struct snd_sunxi_rglt {
	u32 unit_cnt;
	struct snd_sunxi_rglt_unit *unit;
	void *priv;
};

struct snd_sunxi_rglt *snd_sunxi_regulator_init(struct platform_device *pdev);
void snd_sunxi_regulator_exit(struct snd_sunxi_rglt *rglt);
int snd_sunxi_regulator_enable(struct snd_sunxi_rglt *rglt);
void snd_sunxi_regulator_disable(struct snd_sunxi_rglt *rglt);

/******* pa config *******/
struct pa_config {
	u32 pin;
	u32 msleep;
	bool used;
	bool level;
};

struct pa_config *snd_sunxi_pa_pin_init(struct platform_device *pdev, u32 *pa_pin_max);
void snd_sunxi_pa_pin_exit(struct platform_device *pdev,
			   struct pa_config *pa_cfg, u32 pa_pin_max);
int snd_sunxi_pa_pin_enable(struct pa_config *pa_cfg, u32 pa_pin_max);
void snd_sunxi_pa_pin_disable(struct pa_config *pa_cfg, u32 pa_pin_max);

/******* hdmi format config *******/
#define	SUNXI_DAI_I2S_TYPE	0
#define	SUNXI_DAI_HDMI_TYPE	1

enum HDMI_FORMAT {
	HDMI_FMT_NULL = 0,
	HDMI_FMT_PCM = 1,
	HDMI_FMT_AC3,
	HDMI_FMT_MPEG1,
	HDMI_FMT_MP3,
	HDMI_FMT_MPEG2,
	HDMI_FMT_AAC,
	HDMI_FMT_DTS,
	HDMI_FMT_ATRAC,
	HDMI_FMT_ONE_BIT_AUDIO,
	HDMI_FMT_DOLBY_DIGITAL_PLUS,
	HDMI_FMT_DTS_HD,
	HDMI_FMT_MAT,
	HDMI_FMT_DST,
	HDMI_FMT_WMAPRO,
};

enum HDMI_FORMAT snd_sunxi_hdmi_get_fmt(void);
int snd_sunxi_hdmi_set_fmt(int hdmi_fmt);
int snd_sunxi_hdmi_get_dai_type(struct device_node *np, unsigned int *dai_type);

/******* sysfs dump *******/
struct snd_sunxi_dump {
	struct list_head list;

	void (*dump_version)(void *priv, char *buf, size_t *count);
	void (*dump_help)(void *priv, char *buf, size_t *count);
	int (*dump_show)(void *priv, char *buf, size_t *count);
	int (*dump_store)(void *priv, const char *buf, size_t count);

	const char *name;
	void *priv;
	bool use;
};

int snd_sunxi_dump_register(struct snd_sunxi_dump *dump);
void snd_sunxi_dump_unregister(struct snd_sunxi_dump *dump);

#endif /* __SND_SUNXI_COMMON_H */
