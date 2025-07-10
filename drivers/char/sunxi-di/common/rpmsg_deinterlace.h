/*
 * Copyright (c) 2007-2018 Allwinnertech Co., Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef _DI_RPMSG_H_
#define _DI_RPMSG_H_

typedef int amp_ctrl(void);

struct di_amp_ctrl {
	amp_ctrl *rv_start;
	amp_ctrl *rv_stop;
	bool pm_state;
	bool rv_irq_state;
	bool iommu_need;
};

void amp_deinterlace_init(void *amp_ops);
int amp_deinterlace_start_ack(void);
int amp_deinterlace_stop_ack(void);
void amp_deinterlace_exit(void);

#endif