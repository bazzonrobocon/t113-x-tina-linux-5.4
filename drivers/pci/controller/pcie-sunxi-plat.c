// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe RC driver for Allwinner Core
 *
 * Copyright (C) 2022 Allwinner Co., Ltd.
 *
 * Author: songjundong <songjundong@allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/pci.h>
#include "linux/mfd/pcie-usb-phy.h"
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/resource.h>
#include <linux/signal.h>
#include <linux/types.h>
#include <linux/reset.h>
#include <linux/sunxi-gpio.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include "pcie-sunxi-dma.h"
#include "pcie-sunxi.h"

static const struct sunxi_pcie_of_data sunxi_pcie_rc_v100_of_data = {
	.mode = SUNXI_PCIE_RC_TYPE,
};

static const struct sunxi_pcie_of_data sunxi_pcie_ep_of_data = {
	.mode = SUNXI_PCIE_EP_TYPE,
};
static const struct of_device_id sunxi_plat_pcie_of_match[] = {
	{
		.compatible = "allwinner,sunxi-pcie-v100",
		.data = &sunxi_pcie_rc_v100_of_data,
	},
	{},
};
MODULE_DEVICE_TABLE(of, sunxi_plat_pcie_of_match);

static inline void sunxi_pcie_writel_phy(struct pcie_port *pp, u32 val, u32 reg)
{
	writel(val, pp->phy_base + reg);
}

static inline u32 sunxi_pcie_readl_phy(struct pcie_port *pp, u32 reg)
{
	return readl(pp->phy_base + reg);
}

static inline void sunxi_pcie_writel_rcl(struct pcie_port *pp, u32 val, u32 reg)
{
	writel(val, pp->dbi_base + reg);
}

static inline u32 sunxi_pcie_readl_rcl(struct pcie_port *pp, u32 reg)
{
	return readl(pp->dbi_base + reg);
}

static void sunxi_pcie_ltssm_enable(struct pcie_port *pp)
{
	struct sunxi_pcie *pcie = to_sunxi_pcie(pp);
	u32 val;

	val = sunxi_pcie_readl(pcie, PCIE_LTSSM_ENABLE);
	val |= PCIE_LINK_TRAINING;
	sunxi_pcie_writel(val, pcie, PCIE_LTSSM_ENABLE);
}

static void sunxi_pcie_irqpending(struct pcie_port *pp)
{
	struct sunxi_pcie *pcie = to_sunxi_pcie(pp);
	u32 val;

	val = sunxi_pcie_readl(pcie, PCIE_INT_ENABLE_CLR);
	val &= ~(0x3<<0);
	sunxi_pcie_writel(val, pcie, PCIE_INT_ENABLE_CLR);
}

static void sunxi_pcie_irqmask(struct pcie_port *pp)
{
	struct sunxi_pcie *pcie = to_sunxi_pcie(pp);
	u32 val;

	val = sunxi_pcie_readl(pcie, PCIE_INT_ENABLE_CLR);
	val |= 0x3<<0;
	sunxi_pcie_writel(val, pcie, PCIE_INT_ENABLE_CLR);
}

static void sunxi_pcie_ltssm_disable(struct pcie_port *pp)
{
	struct sunxi_pcie *pcie = to_sunxi_pcie(pp);
	u32 val;

	val = sunxi_pcie_readl(pcie, PCIE_LTSSM_ENABLE);
	val &= ~PCIE_LINK_TRAINING;
	sunxi_pcie_writel(val, pcie, PCIE_LTSSM_ENABLE);
}

static int sunxi_pcie_wait_for_speed_change(struct pcie_port *pp)
{
	u32 tmp;
	unsigned int retries;

	for (retries = 0; retries < 200; retries++) {
		tmp = sunxi_pcie_readl_rcl(pp, PCIE_LINK_WIDTH_SPEED_CONTROL);
		if (!(tmp & PORT_LOGIC_SPEED_CHANGE))
			return 0;
		usleep_range(SPEED_CHANGE_USLEEP_MIN, SPEED_CHANGE_USLEEP_MAX);
	}

	dev_err(pp->dev, "Speed change timeout\n");
	return -ETIMEDOUT;
}

int sunxi_pcie_wait_for_link(struct pcie_port *pp)
{
	int retries;

	for (retries = 0; retries < LINK_WAIT_MAX_RETRIE; retries++) {
		if (sunxi_pcie_link_up(pp)) {
			dev_info(pp->dev, "sunxi pcie link up success\n");
			return 0;
		}
		usleep_range(LINK_WAIT_USLEEP_MIN, LINK_WAIT_USLEEP_MAX);
	}

	return -ETIMEDOUT;
}

static int sunxi_pcie_establish_link(struct pcie_port *pp)
{
	if (sunxi_pcie_link_up(pp)) {
		dev_info(pp->dev, "PCIe link is already up\n");
		return 0;
	}

	sunxi_pcie_ltssm_enable(pp);
	sunxi_pcie_wait_for_link(pp);

	return 1;
}

static int sunxi_pcie_link_up_status(struct pcie_port *pp)
{
	u32 val;
	int ret;
	struct sunxi_pcie *pcie = to_sunxi_pcie(pp);

	val = sunxi_pcie_readl(pcie, PCIE_LINK_STAT);

	if ((val & RDLH_LINK_UP) && (val & SMLH_LINK_UP))
		ret = 1;
	else
		ret = 0;

	return ret;
}

static int sunxi_pcie_speed_chang(struct pcie_port *pp, int gen)
{
	int val;
	int ret;

	val = sunxi_pcie_readl_rcl(pp, LINK_CONTROL2_LINK_STATUS2);
	val &=  ~0xf;
	val |= gen;
	sunxi_pcie_writel_rcl(pp, val, LINK_CONTROL2_LINK_STATUS2);

	val = sunxi_pcie_readl_rcl(pp, PCIE_LINK_WIDTH_SPEED_CONTROL);
	val &= ~PORT_LOGIC_SPEED_CHANGE;
	sunxi_pcie_writel_rcl(pp, val, PCIE_LINK_WIDTH_SPEED_CONTROL);

	val = sunxi_pcie_readl_rcl(pp, PCIE_LINK_WIDTH_SPEED_CONTROL);
	val |= PORT_LOGIC_SPEED_CHANGE;
	sunxi_pcie_writel_rcl(pp, val, PCIE_LINK_WIDTH_SPEED_CONTROL);

	ret = sunxi_pcie_wait_for_speed_change(pp);
	if (!ret)
		dev_info(pp->dev, "PCI-e speed of Gen%d\n", gen);
	else
		dev_info(pp->dev, "PCI-e speed of Gen1\n");

	return 0;
}

static int sunxi_pcie_clk_setup(struct pcie_port *pp)
{
	struct sunxi_pcie *sunxi_pcie = to_sunxi_pcie(pp);
	int ret;

	ret = clk_prepare_enable(sunxi_pcie->pcie_ref);
	if (ret) {
		dev_err(pp->dev, "PCIE: cannot prepare/enable ref clock\n");
		goto err_clk_ref;
	}

	ret = clk_prepare_enable(sunxi_pcie->pcie_aux);
	if (ret) {
		dev_err(pp->dev, "PCIE: cannot prepare/enable aux clock\n");
		goto err_clk_aux;
	}

	ret = reset_control_deassert(sunxi_pcie->pcie_rst);
	if (ret) {
		dev_err(pp->dev, "PCIE: cannot reset pcie\n");
		goto err_clk_rst;
	}

	return 0;

err_clk_rst:
	clk_disable_unprepare(sunxi_pcie->pcie_aux);

err_clk_aux:
	clk_disable_unprepare(sunxi_pcie->pcie_ref);

err_clk_ref:

	return ret;
}

static void sunxi_pcie_clk_exit(struct pcie_port *pp)
{
	struct sunxi_pcie *sunxi_pcie = to_sunxi_pcie(pp);

	reset_control_assert(sunxi_pcie->pcie_rst);
	clk_disable_unprepare(sunxi_pcie->pcie_aux);
	clk_disable_unprepare(sunxi_pcie->pcie_ref);
}

static int sunxi_pcie_combo_phy_init(struct pcie_port *pp)
{
	u32 val;
	int ret = 0;

	/* select phy mode  */
	val = sunxi_pcie_readl_phy(pp, PCIE_COMBO_PHY_CTL);
	val &= (~PHY_USE_SEL);
	val &= (~(0x03<<8));
	val &= (~PHY_FPGA_SYS_RSTN);
	val &= (~PHY_RSTN);
	sunxi_pcie_writel_phy(pp, val, PCIE_COMBO_PHY_CTL);

	/* phy reset  */
	val = sunxi_pcie_readl_phy(pp, PCIE_COMBO_PHY_CTL);
	val &= (~PHY_CLK_SEL);
	val &= (~(0x03<<8));
	val &= (~PHY_FPGA_SYS_RSTN);
	val &= (~PHY_RSTN);
	val |= PHY_RSTN ;
	sunxi_pcie_writel_phy(pp, val, PCIE_COMBO_PHY_CTL);

	/* sys_rstn dessert after phy reset */
	val = sunxi_pcie_readl_phy(pp, PCIE_COMBO_PHY_CTL);
	val &= (~PHY_CLK_SEL);
	val &= (~(0x03<<8));
	val &= (~PHY_FPGA_SYS_RSTN);
	val &= (~PHY_RSTN);
	val |= PHY_RSTN ;
	val |= PHY_FPGA_SYS_RSTN;
	sunxi_pcie_writel_phy(pp, val, PCIE_COMBO_PHY_CTL);

	/* set the phy,then work  */
	val = sunxi_pcie_readl_phy(pp, PCIE_COMBO_PHY_BGR);
	val &= (~(0x03<<0));
	val &= (~(0x03<<16));
	val |= (0x03<<0);
	val |= (0x03<<16);
	sunxi_pcie_writel_phy(pp, val, PCIE_COMBO_PHY_BGR);

	return 0;
}

static irqreturn_t sunxi_plat_pcie_msi_irq_handler(int irq, void *arg)
{
	struct pcie_port *pp = (struct pcie_port *)arg;

	return sunxi_handle_msi_irq(pp);
}

static irqreturn_t sunxi_plat_pcie_linkup_handler(int irq, void *arg)
{
	struct pcie_port *pp = (struct pcie_port *)arg;

	sunxi_pcie_irqpending(pp);

	return IRQ_HANDLED;
}

static inline void
sunxi_pcie_dma_handle_interrupt(u32 chnl, enum dma_dir dma_trx)
{
	int ret = 0;

	ret = pcie_dma_chan_release(chnl, dma_trx);
	if (unlikely(ret < 0)) {
		pr_err("%s is error release chnl%d !\n", __func__, chnl);
	}
}

#define SUNXI_PCIE_DMA_IRQ_HANDLER(name, chn, dir)				\
static irqreturn_t sunxi_pcie_##name##_irq_handler				\
						(int irq, void *arg)		\
{										\
	struct sunxi_pcie *sunxi_pcie = arg;					\
	struct pcie_port *pp = &sunxi_pcie->pp;                                 \
	union int_status sta = {0};						\
	union int_clear  clr = {0};                                             \
												  \
	sta.dword = sunxi_pcie_readl_rcl(pp, PCIE_DMA_OFFSET +					  \
					(dir ? PCIE_DMA_RD_INT_STATUS : PCIE_DMA_WR_INT_STATUS)); \
												  \
	if (sta.done & BIT(chn)) {							          \
		clr.doneclr = BIT(chn);								  \
		sunxi_pcie_writel_rcl(pp, clr.dword, PCIE_DMA_OFFSET +				  \
					(dir ? PCIE_DMA_RD_INT_CLEAR : PCIE_DMA_WR_INT_CLEAR));   \
		sunxi_pcie_dma_handle_interrupt(chn, dir);					  \
	}											  \
												  \
	if (sta.abort & BIT(chn)) {								  \
		clr.abortclr = BIT(chn);							  \
		sunxi_pcie_writel_rcl(pp, clr.dword, PCIE_DMA_OFFSET +				  \
					(dir ? PCIE_DMA_RD_INT_CLEAR : PCIE_DMA_WR_INT_CLEAR));   \
		dev_err(pp->dev, "DMA %s channel %d is abort failed handle\n",			  \
							dir ? "read":"write", chn);		  \
	}											  \
												  \
	return IRQ_HANDLED;									  \
}

SUNXI_PCIE_DMA_IRQ_HANDLER(dma_w0, 0, PCIE_DMA_WRITE)
SUNXI_PCIE_DMA_IRQ_HANDLER(dma_w1, 1, PCIE_DMA_WRITE)
SUNXI_PCIE_DMA_IRQ_HANDLER(dma_w2, 2, PCIE_DMA_WRITE)
SUNXI_PCIE_DMA_IRQ_HANDLER(dma_w3, 3, PCIE_DMA_WRITE)

SUNXI_PCIE_DMA_IRQ_HANDLER(dma_r0, 0, PCIE_DMA_READ)
SUNXI_PCIE_DMA_IRQ_HANDLER(dma_r1, 1, PCIE_DMA_READ)
SUNXI_PCIE_DMA_IRQ_HANDLER(dma_r2, 2, PCIE_DMA_READ)
SUNXI_PCIE_DMA_IRQ_HANDLER(dma_r3, 3, PCIE_DMA_READ)

static void sunxi_pcie_dma_read(struct pcie_port *pp, struct dma_table *table, int PCIE_OFFSET)
{
	sunxi_pcie_writel_rcl(pp, table->enb.dword,
							PCIE_DMA_OFFSET + PCIE_DMA_RD_ENB);
	sunxi_pcie_writel_rcl(pp, table->ctx_reg.ctrllo.dword,
							PCIE_OFFSET + PCIE_DMA_RD_CTRL_LO);
	sunxi_pcie_writel_rcl(pp, table->ctx_reg.ctrlhi.dword,
							PCIE_OFFSET + PCIE_DMA_RD_CTRL_HI);
	sunxi_pcie_writel_rcl(pp, table->ctx_reg.xfersize,
							PCIE_OFFSET + PCIE_DMA_RD_XFERSIZE);
	sunxi_pcie_writel_rcl(pp, table->ctx_reg.sarptrlo,
							PCIE_OFFSET + PCIE_DMA_RD_SAR_LO);
	sunxi_pcie_writel_rcl(pp, table->ctx_reg.sarptrhi,
							PCIE_OFFSET + PCIE_DMA_RD_SAR_HI);
	sunxi_pcie_writel_rcl(pp, table->ctx_reg.darptrlo,
							PCIE_OFFSET + PCIE_DMA_RD_DAR_LO);
	sunxi_pcie_writel_rcl(pp, table->ctx_reg.darptrhi,
							PCIE_OFFSET + PCIE_DMA_RD_DAR_HI);
	sunxi_pcie_writel_rcl(pp, table->weilo.dword,
							PCIE_OFFSET + PCIE_DMA_RD_WEILO);
	sunxi_pcie_writel_rcl(pp, table->start.dword,
							PCIE_DMA_OFFSET + PCIE_DMA_RD_DOORBELL);

}

static void sunxi_pcie_dma_write(struct pcie_port *pp, struct dma_table *table, int PCIE_OFFSET)
{
	sunxi_pcie_writel_rcl(pp, table->enb.dword,
							PCIE_DMA_OFFSET + PCIE_DMA_WR_ENB);
	sunxi_pcie_writel_rcl(pp, table->ctx_reg.ctrllo.dword,
							PCIE_OFFSET + PCIE_DMA_WR_CTRL_LO);
	sunxi_pcie_writel_rcl(pp, table->ctx_reg.ctrlhi.dword,
							PCIE_OFFSET + PCIE_DMA_WR_CTRL_HI);
	sunxi_pcie_writel_rcl(pp, table->ctx_reg.xfersize,
							PCIE_OFFSET + PCIE_DMA_WR_XFERSIZE);
	sunxi_pcie_writel_rcl(pp, table->ctx_reg.sarptrlo,
							PCIE_OFFSET + PCIE_DMA_WR_SAR_LO);
	sunxi_pcie_writel_rcl(pp, table->ctx_reg.sarptrhi,
							PCIE_OFFSET + PCIE_DMA_WR_SAR_HI);
	sunxi_pcie_writel_rcl(pp, table->ctx_reg.darptrlo,
							PCIE_OFFSET + PCIE_DMA_WR_DAR_LO);
	sunxi_pcie_writel_rcl(pp, table->ctx_reg.darptrhi,
							PCIE_OFFSET + PCIE_DMA_WR_DAR_HI);
	sunxi_pcie_writel_rcl(pp, table->weilo.dword,
							PCIE_OFFSET + PCIE_DMA_WR_WEILO);
	sunxi_pcie_writel_rcl(pp, table->start.dword,
							PCIE_DMA_OFFSET + PCIE_DMA_WR_DOORBELL);
}

static void sunxi_pcie_dma_start(struct dma_table *table, struct dma_trx_obj *obj)
{
	struct sunxi_pcie *sunxi_pcie = dev_get_drvdata(obj->dev);
	struct pcie_port *pp = &sunxi_pcie->pp;
	u32 val;
	int offset = PCIE_DMA_OFFSET + table->start.chnl * 0x200;

	val = sunxi_pcie_readl_rcl(pp, PCIE_ATU_CR2_OUTBOUND(0));
	val |= PCIE_ATU_DMA_BYPASS;
	sunxi_pcie_writel_rcl(pp, val, PCIE_ATU_CR2_OUTBOUND(0));

	val = sunxi_pcie_readl_rcl(pp, PCIE_ATU_CR2_OUTBOUND(1));
	val |= PCIE_ATU_DMA_BYPASS;
	sunxi_pcie_writel_rcl(pp, val, PCIE_ATU_CR2_OUTBOUND(1));

	if (table->dir == PCIE_DMA_READ) {
		sunxi_pcie_dma_read(pp, table, offset);
	} else if (table->dir == PCIE_DMA_WRITE) {
		sunxi_pcie_dma_write(pp, table, offset);
	}

	val = sunxi_pcie_readl_rcl(pp, PCIE_ATU_CR2_OUTBOUND(0));
	val &= ~PCIE_ATU_DMA_BYPASS;
	sunxi_pcie_writel_rcl(pp, val, PCIE_ATU_CR2_OUTBOUND(0));

	val = sunxi_pcie_readl_rcl(pp, PCIE_ATU_CR2_OUTBOUND(1));
	val &= ~PCIE_ATU_DMA_BYPASS;
	sunxi_pcie_writel_rcl(pp, val, PCIE_ATU_CR2_OUTBOUND(1));
}

static int sunxi_pcie_dma_config(struct dma_table *table, phys_addr_t sar_addr, phys_addr_t dar_addr, unsigned int size, enum dma_dir dma_trx)
{
	dma_channel_t *chn = NULL;

	table->ctx_reg.ctrllo.lie   = 0x1;
	table->ctx_reg.ctrllo.rie   = 0x0;
	table->ctx_reg.ctrllo.td    = 0x1;
	table->ctx_reg.ctrlhi.dword = 0x0;
	table->ctx_reg.xfersize = size;
	table->ctx_reg.sarptrlo = (u32)(sar_addr & 0xffffffff);
	table->ctx_reg.sarptrhi = (u32)(sar_addr >> 32);
	table->ctx_reg.darptrlo = (u32)(dar_addr & 0xffffffff);
	table->ctx_reg.darptrhi = (u32)(dar_addr >> 32);
	table->start.stop = 0x0;
	table->dir = dma_trx;

	chn = (dma_channel_t *)pcie_dma_chan_request(dma_trx);
	if (!chn) {
		pr_err("pcie request %s channel error! \n", (dma_trx ? "DMA_READ":"DMA_WRITE"));
		return -ENOMEM;
	}

	table->start.chnl = chn->chnl_num;

	if (dma_trx == PCIE_DMA_WRITE) {
		table->weilo.dword = (PCIE_WEIGHT << (5*chn->chnl_num));
		table->enb.enb = 0x1;
	} else {
		table->weilo.dword = (PCIE_WEIGHT << (5*chn->chnl_num));
		table->enb.enb = 0x1;
	}

	return 0;
}

static int sunxi_pcie_request_irq(struct sunxi_pcie *sunxi_pcie, struct platform_device *pdev)
{
	int irq, ret, num;

	irq  = platform_get_irq_byname(pdev, "sii");
	if (irq < 0) {
		dev_err(&pdev->dev, "missing sys IRQ resource\n");
		return -EINVAL;
	}

	ret = devm_request_irq(&pdev->dev, irq,
				 sunxi_plat_pcie_linkup_handler,
					IRQF_SHARED, "pcie-linkup", &sunxi_pcie->pp);
	if (ret) {
		dev_err(&pdev->dev, "PCIe failed to request linkup IRQ\n");
		return ret;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "num-edma", &num);
	if (ret) {
		dev_info(&pdev->dev, "PCIe get num-edma from DT failed\n");
		return ret;
	}

	irq = platform_get_irq_byname(pdev, "edma-w0");
	if (irq < 0) {
		dev_err(&pdev->dev, "missing edma-write IRQ resource\n");
	}
	ret = devm_request_irq(&sunxi_pcie->dev, irq, sunxi_pcie_dma_w0_irq_handler,
			       IRQF_SHARED, "pcie-dma-w0", sunxi_pcie);
	if (ret) {
		dev_err(&sunxi_pcie->dev, "failed to request PCIe DMA IRQ\n");
		return ret;
	}

	irq = platform_get_irq_byname(pdev, "edma-w1");
	if (irq < 0) {
		dev_err(&pdev->dev, "missing edma-write IRQ resource\n");
	}
	ret = devm_request_irq(&sunxi_pcie->dev, irq, sunxi_pcie_dma_w1_irq_handler,
			       IRQF_SHARED, "pcie-dma-w1", sunxi_pcie);
	if (ret) {
		dev_err(&sunxi_pcie->dev, "failed to request PCIe DMA IRQ\n");
		return ret;
	}

	irq = platform_get_irq_byname(pdev, "edma-w2");
	if (irq < 0) {
		dev_err(&pdev->dev, "missing edma-write IRQ resource\n");
	}
	ret = devm_request_irq(&sunxi_pcie->dev, irq, sunxi_pcie_dma_w2_irq_handler,
			       IRQF_SHARED, "pcie-dma-w2", sunxi_pcie);
	if (ret) {
		dev_err(&sunxi_pcie->dev, "failed to request PCIe DMA IRQ\n");
		return ret;
	}

	irq = platform_get_irq_byname(pdev, "edma-w3");
	if (irq < 0) {
		dev_err(&pdev->dev, "missing edma-write IRQ resource\n");
	}
	ret = devm_request_irq(&sunxi_pcie->dev, irq, sunxi_pcie_dma_w3_irq_handler,
			       IRQF_SHARED, "pcie-dma-w3", sunxi_pcie);
	if (ret) {
		dev_err(&sunxi_pcie->dev, "failed to request PCIe DMA IRQ\n");
		return ret;
	}

	irq = platform_get_irq_byname(pdev, "edma-r0");
	if (irq < 0) {
		dev_err(&pdev->dev, "missing edma-write IRQ resource\n");
	}
	ret = devm_request_irq(&sunxi_pcie->dev, irq, sunxi_pcie_dma_r0_irq_handler,
			       IRQF_SHARED, "pcie-dma-r0", sunxi_pcie);
	if (ret) {
		dev_err(&sunxi_pcie->dev, "failed to request PCIe DMA IRQ\n");
		return ret;
	}

	irq = platform_get_irq_byname(pdev, "edma-r1");
	if (irq < 0) {
		dev_err(&pdev->dev, "missing edma-write IRQ resource\n");
	}
	ret = devm_request_irq(&sunxi_pcie->dev, irq, sunxi_pcie_dma_r1_irq_handler,
			       IRQF_SHARED, "pcie-dma-r1", sunxi_pcie);
	if (ret) {
		dev_err(&sunxi_pcie->dev, "failed to request PCIe DMA IRQ\n");
		return ret;
	}

	irq = platform_get_irq_byname(pdev, "edma-r2");
	if (irq < 0) {
		dev_err(&pdev->dev, "missing edma-write IRQ resource\n");
	}
	ret = devm_request_irq(&sunxi_pcie->dev, irq, sunxi_pcie_dma_r2_irq_handler,
			       IRQF_SHARED, "pcie-dma-r2", sunxi_pcie);
	if (ret) {
		dev_err(&sunxi_pcie->dev, "failed to request PCIe DMA IRQ\n");
		return ret;
	}

	irq = platform_get_irq_byname(pdev, "edma-r3");
	if (irq < 0) {
		dev_err(&pdev->dev, "missing edma-write IRQ resource\n");
	}
	ret = devm_request_irq(&sunxi_pcie->dev, irq, sunxi_pcie_dma_r3_irq_handler,
			       IRQF_SHARED, "pcie-dma-r3", sunxi_pcie);
	if (ret) {
		dev_err(&sunxi_pcie->dev, "failed to request PCIe DMA IRQ\n");
		return ret;
	}

	return 0;
}

static int sunxi_pcie_get_clk(struct platform_device *pdev, struct sunxi_pcie *sunxi_pcie)
{
	int ret;

	sunxi_pcie->pcie_ref = devm_clk_get(&pdev->dev, "pclk_ref");
	if (IS_ERR_OR_NULL(sunxi_pcie->pcie_ref))
		return PTR_ERR(sunxi_pcie->pcie_ref);

	sunxi_pcie->pcie_per = devm_clk_get(&pdev->dev, "pclk_per");
	if (IS_ERR_OR_NULL(sunxi_pcie->pcie_per))
		return PTR_ERR(sunxi_pcie->pcie_per);

	ret = clk_set_parent(sunxi_pcie->pcie_ref, sunxi_pcie->pcie_per);
	if (ret != 0) {
		return -1;
	}

	sunxi_pcie->pcie_aux = devm_clk_get(&pdev->dev, "pclk_aux");
	if (IS_ERR_OR_NULL(sunxi_pcie->pcie_aux))
		return PTR_ERR(sunxi_pcie->pcie_aux);

	sunxi_pcie->pcie_rst = devm_reset_control_get(&pdev->dev, "pclk_rst");
	if (IS_ERR_OR_NULL(sunxi_pcie->pcie_rst)) {
		return PTR_ERR(sunxi_pcie->pcie_rst);
	}

	return 0;
}

static int sunxi_pcie_init_dma_trx(struct pcie_port *pp)
{
	pp->dma_obj = sunxi_pcie_dma_obj_probe(pp->dev);

	if (IS_ERR(pp->dma_obj)) {
		dev_err(pp->dev, "failed to prepare dma obj probe\n");
		return -EINVAL;
	}

	sunxi_pcie_writel_rcl(pp, 0x0,	PCIE_DMA_OFFSET + PCIE_DMA_WR_INT_MASK);
	sunxi_pcie_writel_rcl(pp, 0x0,	PCIE_DMA_OFFSET + PCIE_DMA_RD_INT_MASK);

	return 0;
}

static void sunxi_plat_pcie_host_init(struct pcie_port *pp)
{
	sunxi_pcie_ltssm_disable(pp);

	sunxi_pcie_setup_rc(pp);

	sunxi_pcie_establish_link(pp);

	//sunxi_pcie_speed_chang(pp, pcie->speed_gen);
}

static struct pcie_host_ops sunxi_plat_pcie_host_ops = {
	.link_up = sunxi_pcie_link_up_status,
	.host_init = sunxi_plat_pcie_host_init,
};

static int sunxi_plat_add_pcie_port(struct pcie_port *pp,
				 struct platform_device *pdev)
{
	int ret;

	pp->msi_ext = false;

	if (device_property_read_bool(&pdev->dev, "msi-map")) {
		pp->msi_ext = true;
		printk("PCIe get msi-map OK\n");
	}

	if (IS_ENABLED(CONFIG_PCI_MSI) && !pp->msi_ext) {
		pp->msi_irq = platform_get_irq_byname(pdev, "msi");
		if (pp->msi_irq < 0)
			return pp->msi_irq;

		ret = devm_request_irq(&pdev->dev, pp->msi_irq,
					sunxi_plat_pcie_msi_irq_handler,
					IRQF_SHARED, "pcie-msi", pp);
		if (ret) {
			dev_err(&pdev->dev, "failed to request MSI IRQ\n");
			return ret;
		}
	}

	pp->root_bus_nr = -1;
	pp->ops = &sunxi_plat_pcie_host_ops;

	ret = sunxi_pcie_host_init(pp);
	if (ret) {
		dev_err(&pdev->dev, "failed to initialize host\n");
		return ret;
	}

	ret = sunxi_pcie_init_dma_trx(pp);
	if (ret) {
		dev_err(&pdev->dev, "failed to init host dma trx\n");
		return ret;
	}

	return 0;
}

static int sunxi_plat_pcie_probe(struct platform_device *pdev)
{
	struct sunxi_pcie *sunxi_pcie;
	struct pcie_port *pp;
	struct resource *dbi_res;
	struct resource *phy_res;
	struct resource *clk_res;
	int ret;
	const struct of_device_id *match;
	const struct sunxi_pcie_of_data *data;
	enum sunxi_pcie_device_mode	mode;

	match = of_match_device(sunxi_plat_pcie_of_match, &pdev->dev);
	if (!match) {
		dev_err(&pdev->dev, "ERROR:Pcie probe match device id failed\n");
		return -EINVAL;
	}

	data = (struct sunxi_pcie_of_data *)match->data;
	mode = (enum sunxi_pcie_device_mode)data->mode;

	sunxi_pcie = devm_kzalloc(&pdev->dev, sizeof(*sunxi_pcie),
					GFP_KERNEL);
	if (!sunxi_pcie)
		return -ENOMEM;

	pp = &sunxi_pcie->pp;
	pp->dev = &pdev->dev;
	sunxi_pcie->dev = pdev->dev;
	sunxi_pcie->mode = mode;

	dbi_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi");
	if (!dbi_res) {
		dev_err(&pdev->dev, "get pcie dbi failed\n");
		return -ENODEV;
	}

	sunxi_pcie->dbi_base = devm_ioremap_resource(&pdev->dev, dbi_res);
	if (IS_ERR(sunxi_pcie->dbi_base)) {
		dev_err(&pdev->dev, "ioremap pcie dbi failed\n");
		return PTR_ERR(sunxi_pcie->dbi_base);
	}

	pp->dbi_base = sunxi_pcie->dbi_base;
	sunxi_pcie->app_base = sunxi_pcie->dbi_base + USER_DEFINED_REGISTER_LIST;

	phy_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "pcie-phy");
	if (!phy_res) {
		dev_err(&pdev->dev, "get pcie-phy failed\n");
		return -ENODEV;
	}

	sunxi_pcie->phy_base = devm_ioremap_resource(&pdev->dev, phy_res);
	if (IS_ERR_OR_NULL(sunxi_pcie->phy_base)) {
		dev_err(&pdev->dev, "ioremap pcie phy_res failed\n");
		return PTR_ERR(sunxi_pcie->phy_base);
	}
	pp->phy_base = sunxi_pcie->phy_base;

	clk_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "clk_add");
	if (!clk_res) {
		dev_err(&pdev->dev, "get clk_add failed\n");
		return -ENODEV;
	}

	sunxi_pcie->clk_add = devm_ioremap_resource(&pdev->dev, clk_res);

	ret = sunxi_pcie_get_clk(pdev, sunxi_pcie);
	if (ret) {
		dev_err(pp->dev, "pcie get clk init failed\n");
		return -ENODEV;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "pcie_speed_gen", &sunxi_pcie->speed_gen);
	if (ret) {
		dev_info(&pdev->dev, "PCIe get speed Gen failed\n");
		sunxi_pcie->speed_gen = 0x1;
	}

	if (sunxi_pcie_clk_setup(pp)) {
		dev_err(pp->dev, "error: PCIe clk setup failed. \n");
		return -1;
	}

	ret = sunxi_pcie_combo_phy_init(pp);
	if (ret)
		return -EPROBE_DEFER;

	sunxi_pcie_irqmask(pp);

	platform_set_drvdata(pdev, sunxi_pcie);

	ret = sunxi_pcie_request_irq(sunxi_pcie, pdev);
	if (ret) {
		dev_err(&pdev->dev, "pcie link irq init failed\n");
	}

	ret = sunxi_plat_add_pcie_port(pp, pdev);
	if (ret < 0)
		return ret;

	if (pp->dma_obj) {
		pp->dma_obj->start_dma_trx_func  = sunxi_pcie_dma_start;
		pp->dma_obj->config_dma_trx_func = sunxi_pcie_dma_config;
	}

	return 0;
}

static int sunxi_plat_pcie_remove(struct platform_device *pdev)
{
	struct sunxi_pcie *sunxi_pcie = platform_get_drvdata(pdev);
	struct pcie_port *pp = &sunxi_pcie->pp;

	sunxi_pcie_clk_exit(pp);

	return 0;
}

#ifdef CONFIG_PM
static int sunxi_pcie_hw_init(struct sunxi_pcie *sunxi_pcie)
{
	struct pcie_port *pp = &sunxi_pcie->pp;
	int val;

	if (sunxi_pcie_clk_setup(pp))
		return -1;

	if (sunxi_pcie_regulator_bypass(pp))
		return -1;

	usleep_range(100, 1000);
	sunxi_pcie_ltssm_disable(pp);

	sunxi_pcie_setup_rc(pp);
	sunxi_pcie_ltssm_enable(pp);

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		u64 msi_target;

		msi_target = virt_to_phys((void *)pp->msi_data);

		sunxi_pcie_wr_own_conf(pp, PCIE_MSI_ADDR_LO, 4, (u32)(msi_target & 0xffffffff));
		sunxi_pcie_wr_own_conf(pp, PCIE_MSI_ADDR_HI, 4, (u32)(msi_target >> 32 & 0xffffffff));
	}

	val = sunxi_pcie_readl_rcl(pp, LINK_CONTROL2_LINK_STATUS2);
	val &= ~0xf;
	val |= sunxi_pcie->speed_gen;
	sunxi_pcie_writel_rcl(pp, val, LINK_CONTROL2_LINK_STATUS2);

	val = sunxi_pcie_readl_rcl(pp, PCIE_LINK_WIDTH_SPEED_CONTROL);
	val &= ~PORT_LOGIC_SPEED_CHANGE;
	sunxi_pcie_writel_rcl(pp, val, PCIE_LINK_WIDTH_SPEED_CONTROL);

	val = sunxi_pcie_readl_rcl(pp, PCIE_LINK_WIDTH_SPEED_CONTROL);
	val |= PORT_LOGIC_SPEED_CHANGE;
	sunxi_pcie_writel_rcl(pp, val, PCIE_LINK_WIDTH_SPEED_CONTROL);

	sunxi_pcie_wr_own_conf(pp, PCI_CACHE_LINE_SIZE, 4, sunxi_pcie->cache_lime_size);
	sunxi_pcie_wr_own_conf(pp, PCI_IO_BASE, 4, sunxi_pcie->io_base);
	sunxi_pcie_wr_own_conf(pp, PCI_INTERRUPT_LINE, 4, sunxi_pcie->interrupt_control);
	sunxi_pcie_wr_own_conf(pp, PCIE_MSI_INTR0_ENABLE(0), 4, sunxi_pcie->msi_enable);

	return 0;
}

static int sunxi_pcie_hw_exit(struct sunxi_pcie *sunxi_pcie)
{
	struct pcie_port *pp = &sunxi_pcie->pp;

	sunxi_pcie_rd_own_conf(pp, PCI_CACHE_LINE_SIZE, 4, &sunxi_pcie->cache_lime_size);
	sunxi_pcie_rd_own_conf(pp, PCI_IO_BASE, 4, &sunxi_pcie->io_base);
	sunxi_pcie_rd_own_conf(pp, PCI_INTERRUPT_LINE, 4, &sunxi_pcie->interrupt_control);
	sunxi_pcie_rd_own_conf(pp, PCIE_MSI_INTR0_ENABLE(0), 4, &sunxi_pcie->msi_enable);
	sunxi_pcie_clk_exit(pp);

	return 0;
}

static int sunxi_pcie_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sunxi_pcie *sunxi_pcie = platform_get_drvdata(pdev);

	sunxi_pcie_hw_exit(sunxi_pcie);

	return 0;
}

static int sunxi_pcie_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sunxi_pcie *sunxi_pcie = platform_get_drvdata(pdev);

	sunxi_pcie_hw_init(sunxi_pcie);

	return 0;
}
static struct dev_pm_ops sunxi_pcie_pm_ops = {
	.suspend_noirq = sunxi_pcie_suspend,
	.resume_noirq = sunxi_pcie_resume,
};
#define SUNXI_PCIE_PM_OPS (&sunxi_pcie_pm_ops)
#else
#define SUNXI_PCIE_PM_OPS NULL
#endif /* CONFIG_PM */

static struct platform_driver sunxi_plat_pcie_driver = {
	.driver = {
		.name	= "sunxi-pcie",
		.owner	= THIS_MODULE,
		.of_match_table = sunxi_plat_pcie_of_match,
		.pm = SUNXI_PCIE_PM_OPS,
	},
	.probe  = sunxi_plat_pcie_probe,
	.remove = sunxi_plat_pcie_remove,
};

static int __init sunxi_pcie_init(void)
{
	return platform_driver_register(&sunxi_plat_pcie_driver);
}
late_initcall_sync(sunxi_pcie_init);

static void __exit sunxi_pcie_exit(void)
{
	platform_driver_unregister(&sunxi_plat_pcie_driver);
}
module_exit(sunxi_pcie_exit);

MODULE_AUTHOR("songjundong <songjundong@allwinnertech.com>");
MODULE_DESCRIPTION("Allwinner PCIe host controller platform driver");
MODULE_VERSION("1.0.0");
MODULE_LICENSE("GPL v2");
