/* sound\soc\sunxi\snd_sunxi_jack_utils.h
 * (C) Copyright 2021-2025
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * Dby <dby@allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */
#ifndef __SND_SUNXI_JACK_UTILS_H
#define __SND_SUNXI_JACK_UTILS_H

int snd_sunxi_card_jack_new(struct snd_soc_card *card, const char *id, int type,
			    struct snd_soc_jack *jack);

#endif /* __SND_SUNXI_JACK_UTILS_H */