/*
 * drivers/media/platform/sunxi-tvd/tvd/tvd.h
 *
 * Copyright (c) 2007-2023 Allwinnertech Co., Ltd.
 * Author: zhengxiaobin <zhengxiaobin@allwinnertech.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef __TVD__RPMSG__H__
#define __TVD__RPMSG__H__

#define TVD_PACKET_MAGIC 0x10244023

int amp_tvd_start_ack(void);
int amp_tvd_stop_ack(void);

enum control_status {
	CONTROL_BY_RTOS,
	CONTROL_BY_ARM,
	CONTROL_BY_NONE,
};

enum tvd_packet_type {
	RV_TVD_START,
	RV_TVD_START_ACK,
	RV_TVD_STOP,
	RV_TVD_STOP_ACK,

	ARM_TVD_START,
	ARM_TVD_START_ACK,
	ARM_TVD_STOP,
	ARM_TVD_STOP_ACK,
};

struct tvd_packet {
	u32 magic;
	u32 type;
};

struct rpmsg_tvd_private {
	struct tvd_amp_ctrl *tvd_ops;
	struct rpmsg_endpoint *ept;
	struct rpmsg_device *rpdev;
	struct mutex lock;
	enum control_status control;
	struct tvd_dev *tvd;
};
#endif