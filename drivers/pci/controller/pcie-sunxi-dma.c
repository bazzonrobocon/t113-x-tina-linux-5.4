// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2022 Allwinner Co., Ltd.
 *
 * The pcie_dma_chnl_request() is used to apply for pcie DMA channels;
 * The pcie_dma_mem_xxx() is to initiate DMA read and write operations;
 *
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_pci.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/reset.h>
#include <linux/resource.h>
#include <linux/signal.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include "pcie-sunxi-dma.h"

dma_channel_t dma_wr_chn[PCIE_DMA_WR_CHN_CNT];
dma_channel_t dma_rd_chn[PCIE_DMA_RD_CHN_CNT];

struct dma_trx_obj *obj_global;

static inline bool is_rc(struct dma_trx_obj *obj)
{
	return (obj->busno == 0);
}

dma_hdl_t pcie_dma_chan_request(enum dma_dir dma_trx)
{
	int i = 0;
	dma_channel_t *pchan = NULL;

	if (dma_trx == PCIE_DMA_WRITE) {
		for (i = 0; i < PCIE_DMA_WR_CHN_CNT; i++) {
			pchan = &dma_wr_chn[i];

			if (!pchan->dma_used) {
				pchan->dma_used = 1;
				pchan->chnl_num = i;
				spin_lock_init(&pchan->lock);
				return (dma_hdl_t)pchan;
			}
		}
	} else if (dma_trx == PCIE_DMA_READ) {
		for (i = 0; i < PCIE_DMA_RD_CHN_CNT; i++) {
			pchan = &dma_rd_chn[i];

			if (pchan->dma_used == 0) {
				pchan->dma_used = 1;
				pchan->chnl_num = i;
				spin_lock_init(&pchan->lock);
				return (dma_hdl_t)pchan;
			}
		}
	}

	return (dma_hdl_t)NULL;
}
EXPORT_SYMBOL_GPL(pcie_dma_chan_request);

int pcie_dma_chan_release(u32 channel, enum dma_dir dma_trx)
{
	if ((channel > PCIE_DMA_WR_CHN_CNT) || (channel > PCIE_DMA_RD_CHN_CNT)) {
		pr_err("ERR: the channel num:%d is error\n", channel);
		return -1;
	}

	if (PCIE_DMA_WRITE == dma_trx) {
		dma_wr_chn[channel].dma_used = 0;
		dma_wr_chn[channel].chnl_num = 0;
	} else if (PCIE_DMA_READ == dma_trx) {
		dma_rd_chn[channel].dma_used = 0;
		dma_rd_chn[channel].chnl_num = 0;
	}

	return 0;
}

int pcie_dma_mem_read(phys_addr_t sar_addr, phys_addr_t dar_addr, unsigned int size)
{
	struct dma_table read_table = {0};
	int ret;

	if (likely(obj_global->config_dma_trx_func)) {
		ret = obj_global->config_dma_trx_func(&read_table, sar_addr, dar_addr, size, PCIE_DMA_READ);

		if (ret < 0) {
			pr_err("pcie dma mem read error ! \n");
			return -EINVAL;
		}
	} else {
		pr_err("config_dma_trx_func is NULL ! \n");
		return -EINVAL;
	}

	obj_global->start_dma_trx_func(&read_table, obj_global);

	return 0;
}
EXPORT_SYMBOL_GPL(pcie_dma_mem_read);


int pcie_dma_mem_write(phys_addr_t sar_addr, phys_addr_t dar_addr, unsigned int size)
{
	struct dma_table write_table = {0};
	int ret;

	if (likely(obj_global->config_dma_trx_func)) {
		ret = obj_global->config_dma_trx_func(&write_table, sar_addr, dar_addr, size, PCIE_DMA_WRITE);

		if (ret < 0) {
			pr_err("pcie dma mem write error ! \n");
			return -EINVAL;
		}
	} else {
		pr_err("config_dma_trx_func is NULL ! \n");
		return -EINVAL;
	}

	obj_global->start_dma_trx_func(&write_table, obj_global);

	return 0;
}
EXPORT_SYMBOL_GPL(pcie_dma_mem_write);

struct dma_trx_obj *sunxi_pcie_dma_obj_probe(struct device *dev)
{
	int ret;
	int busno;
	struct device_node *np = dev->of_node;
	struct dma_trx_obj *obj;

	obj = devm_kzalloc(dev, sizeof(struct dma_trx_obj), GFP_KERNEL);
	if (!obj)
		return ERR_PTR(-ENOMEM);

	obj_global = obj;
	obj->dev = dev;

	ret = of_property_read_u32(np, "busno", &busno);
	if (ret < 0) {
		dev_err(dev, "pcie edma missing \"busno\" property\n");
		return ERR_PTR(ret);
	}

	obj->busno = busno;

	INIT_LIST_HEAD(&obj->dma_list);
	spin_lock_init(&obj->dma_list_lock);

	mutex_init(&obj->count_mutex);

	return obj;
}
