// SPDX_License-Identifier: GPL-2.0
/*
 * allwinner PCIe host controller driver
 *
 * Copyright (c) 2007-2022 Allwinnertech Co., Ltd.
 *
 * Author: songjundong <songjundong@allwinnertech.com>
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
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/msi.h>
#include <linux/of_address.h>
#include <linux/of_pci.h>
#include <linux/pci.h>
#include <linux/pci_regs.h>
#include <linux/types.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include "pcie-sunxi.h"
#include "../pci.h"
#include "pcie-sunxi-dma.h"

static inline struct pcie_port *sys_to_pcie(struct pci_sys_data *sys)
{
	return sys->private_data;
}

static int sunxi_pcie_cfg_read(void __iomem *addr, int size, u32 *val)
{
	if ((uintptr_t)addr & (size - 1)) {
		*val = 0;
		return PCIBIOS_BAD_REGISTER_NUMBER;
	}

	if (size == 4) {
		*val = readl(addr);
	} else if (size == 2) {
		*val = readw(addr);
	} else if (size == 1) {
		*val = readb(addr);
	} else {
		*val = 0;
		return PCIBIOS_BAD_REGISTER_NUMBER;
	}

	return PCIBIOS_SUCCESSFUL;
}

static int sunxi_pcie_cfg_write(void __iomem *addr, int size, u32 val)
{
	if ((uintptr_t)addr & (size - 1))
		return PCIBIOS_BAD_REGISTER_NUMBER;

	if (size == 4)
		writel(val, addr);
	else if (size == 2)
		writew(val, addr);
	else if (size == 1)
		writeb(val, addr);
	else
		return PCIBIOS_BAD_REGISTER_NUMBER;

	return PCIBIOS_SUCCESSFUL;
}

static inline void sunxi_pcie_readl_rc(struct pcie_port *pp, u32 reg, u32 *val)
{
	if (pp->ops->readl_rc)
		pp->ops->readl_rc(pp, pp->dbi_base + reg, val);
	else
		*val = readl(pp->dbi_base + reg);
}

static inline void sunxi_pcie_writel_rc(struct pcie_port *pp, u32 val, u32 reg)
{
	if (pp->ops->writel_rc)
		pp->ops->writel_rc(pp, val, pp->dbi_base + reg);
	else
		writel(val, pp->dbi_base + reg);
}

static int sunxi_pcie_rd_own_conf(struct pcie_port *pp, int where, int size,
			       u32 *val)
{
	int ret;

	if (pp->ops->rd_own_conf)
		ret = pp->ops->rd_own_conf(pp, where, size, val);
	else
		ret = sunxi_pcie_cfg_read(pp->dbi_base + where, size, val);

	return ret;
}

static int sunxi_pcie_wr_own_conf(struct pcie_port *pp, int where, int size,
			       u32 val)
{
	int ret;

	if (pp->ops->wr_own_conf)
		ret = pp->ops->wr_own_conf(pp, where, size, val);
	else
		ret = sunxi_pcie_cfg_write(pp->dbi_base + where, size, val);

	return ret;
}

static struct irq_chip sunxi_msi_irq_chip = {
	.name = "SUNXI-PCIe-MSI",
	.irq_enable  = pci_msi_unmask_irq,
	.irq_disable = pci_msi_mask_irq,
	.irq_mask    = pci_msi_mask_irq,
	.irq_unmask  = pci_msi_unmask_irq,
};

irqreturn_t sunxi_handle_msi_irq(struct pcie_port *pp)
{
	unsigned long val;
	int i, pos, irq;
	u32 status;
	irqreturn_t ret = IRQ_NONE;

	for (i = 0; i < MAX_MSI_CTRLS; i++) {
		sunxi_pcie_rd_own_conf(pp, PCIE_MSI_INTR_STATUS + (i * 12), 4, &status);

		if (!status)
			continue;

		ret = IRQ_HANDLED;
		pos = 0;
		val = status;
		while ((pos = find_next_bit(&val, 32, pos)) != 32) {
			irq = irq_find_mapping(pp->msi_domain,
					i * 32 + pos);
			sunxi_pcie_wr_own_conf(pp,
					PCIE_MSI_INTR_STATUS + (i * 12),
					4, 1 << pos);
			generic_handle_irq(irq);
			pos++;
		}
	}

	return ret;
}

static int sunxi_pcie_msi_enable(struct pcie_port *pp)
{
	u64 msi_target;
	u32 i;

	pp->msi_pages = __get_free_pages(GFP_KERNEL, 0);
	if (!pp->msi_pages)
		return -ENOMEM;

	msi_target = virt_to_phys((void *)pp->msi_pages);

	sunxi_pcie_wr_own_conf(pp, PCIE_MSI_ADDR_LO, 4,	(u32)(msi_target & 0xffffffff));
	sunxi_pcie_wr_own_conf(pp, PCIE_MSI_ADDR_HI, 4, (u32)(msi_target >> 32 & 0xffffffff));

	for (i = 0; i < 8; i++) {
		sunxi_pcie_wr_own_conf(pp, PCIE_MSI_INTR_ENABLE(i), 4, ~0);
	}

	return 0;
}

static int sunxi_pcie_assign_msi(void)
{
	int pos;

	pos = find_first_zero_bit(msi_irq_in_use, INT_PCI_MSI_NR);
	if (pos < INT_PCI_MSI_NR)
		set_bit(pos, msi_irq_in_use);
	else
		return -ENOSPC;

	return pos;
}

static void sunxi_pcie_destroy_msi(unsigned int irq)
{
	clear_bit(irq, msi_irq_in_use);
}

static int sunxi_msi_setup_irq(struct msi_controller *chip, struct pci_dev *pdev,
			struct msi_desc *desc)
{
	unsigned int irq;
	int hwirq;
	struct msi_msg msg;
	phys_addr_t msg_addr;
	struct pcie_port *pp = pdev->bus->sysdata;

	hwirq = sunxi_pcie_assign_msi();

	if (hwirq < 0)
		return hwirq;

	irq = irq_find_mapping(pp->msi_domain, hwirq);
	if (!irq) {
		sunxi_pcie_destroy_msi(hwirq);
		return -EINVAL;
	}

	irq_set_msi_desc(irq, desc);

	msg_addr = virt_to_phys((void *)pp->msi_pages);
	msg.address_lo = (u32)(msg_addr & 0xffffffff);
	msg.address_hi = (u32)(msg_addr >> 32 & 0xffffffff);
	msg.data = hwirq;

	pci_write_msi_msg(irq, &msg);

	return 0;
}

static void sunxi_msi_teardown_irq(struct msi_controller *chip, unsigned int irq)
{
	sunxi_pcie_destroy_msi(irq);
	irq_dispose_mapping(irq);
}

static struct msi_controller sunxi_pcie_msi_chip = {
	.setup_irq    = sunxi_msi_setup_irq,
	.teardown_irq = sunxi_msi_teardown_irq,
};

int sunxi_pcie_link_up(struct pcie_port *pp)
{
	if (pp->ops->link_up)
		return pp->ops->link_up(pp);
	else
		return 0;
}

static int sunxi_pcie_msi_map(struct irq_domain *domain, unsigned int irq,
			irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &sunxi_msi_irq_chip, handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);

	return 0;
}

static const struct irq_domain_ops msi_domain_ops = {
	.map = sunxi_pcie_msi_map,
};

void sunxi_pcie_prog_outbound_atu(struct pcie_port *pp, int index, int type,
					u64 cpu_addr, u64 pci_addr, u32 size)
{
	sunxi_pcie_writel_rc(pp, lower_32_bits(cpu_addr), PCIE_ATU_LOWER_BASE_OUTBOUND(index));
	sunxi_pcie_writel_rc(pp, upper_32_bits(cpu_addr), PCIE_ATU_UPPER_BASE_OUTBOUND(index));
	sunxi_pcie_writel_rc(pp, lower_32_bits(cpu_addr + size - 1), PCIE_ATU_LIMIT_OUTBOUND(index));
	sunxi_pcie_writel_rc(pp, lower_32_bits(pci_addr), PCIE_ATU_LOWER_TARGET_OUTBOUND(index));
	sunxi_pcie_writel_rc(pp, upper_32_bits(pci_addr), PCIE_ATU_UPPER_TARGET_OUTBOUND(index));
	sunxi_pcie_writel_rc(pp, type, PCIE_ATU_CR1_OUTBOUND(index));
	sunxi_pcie_writel_rc(pp, PCIE_ATU_ENABLE, PCIE_ATU_CR2_OUTBOUND(index));
}

static int sunxi_pcie_rd_other_conf(struct pcie_port *pp, struct pci_bus *bus,
		u32 devfn, int where, int size, u32 *val)
{
	int ret = PCIBIOS_SUCCESSFUL, type;
	u32 busdev, cfg_size;
	u64 cpu_addr;
	void __iomem *va_cfg_base;

	busdev = PCIE_ATU_BUS(bus->number) | PCIE_ATU_DEV(PCI_SLOT(devfn)) |
		 PCIE_ATU_FUNC(PCI_FUNC(devfn));

	if (bus->parent->number == pp->root_bus_nr) {
		type = PCIE_ATU_TYPE_CFG0;
		cpu_addr = pp->cfg0_base - PCIE_CPU_BASE;
		cfg_size = pp->cfg0_size;
		va_cfg_base = pp->va_cfg0_base;

	} else {
		type = PCIE_ATU_TYPE_CFG1;
		cpu_addr = pp->cfg1_base - PCIE_CPU_BASE;
		cfg_size = pp->cfg1_size;
		va_cfg_base = pp->va_cfg1_base;
	}

	sunxi_pcie_prog_outbound_atu(pp, PCIE_ATU_REGION_INDEX1, type, cpu_addr, busdev, cfg_size);

	ret = sunxi_pcie_cfg_read(va_cfg_base + where, size, val);

	if (pp->num_viewport <= 2)
		sunxi_pcie_prog_outbound_atu(pp, PCIE_ATU_REGION_INDEX1, PCIE_ATU_TYPE_IO,
						pp->io_base - PCIE_CPU_BASE, pp->io_bus_addr, pp->io_size);

	return ret;
}

static int sunxi_pcie_wr_other_conf(struct pcie_port *pp, struct pci_bus *bus,
		u32 devfn, int where, int size, u32 val)
{
	int ret = PCIBIOS_SUCCESSFUL, type;
	u32 busdev, cfg_size;
	u64 cpu_addr;
	void __iomem *va_cfg_base;

	busdev = PCIE_ATU_BUS(bus->number) | PCIE_ATU_DEV(PCI_SLOT(devfn)) |
		 PCIE_ATU_FUNC(PCI_FUNC(devfn));

	if (bus->parent->number == pp->root_bus_nr) {
		type = PCIE_ATU_TYPE_CFG0;
		cpu_addr = pp->cfg0_base - PCIE_CPU_BASE;
		cfg_size = pp->cfg0_size;
		va_cfg_base = pp->va_cfg0_base;

	} else {
		type = PCIE_ATU_TYPE_CFG1;
		cpu_addr = pp->cfg1_base - PCIE_CPU_BASE;
		cfg_size = pp->cfg1_size;
		va_cfg_base = pp->va_cfg1_base;
	}

	sunxi_pcie_prog_outbound_atu(pp, PCIE_ATU_REGION_INDEX1, type, cpu_addr, busdev, cfg_size);

	ret = sunxi_pcie_cfg_write(va_cfg_base + where, size, val);

	if (pp->num_viewport <= 2)
		sunxi_pcie_prog_outbound_atu(pp, PCIE_ATU_REGION_INDEX1, PCIE_ATU_TYPE_IO,
						pp->io_base - PCIE_CPU_BASE, pp->io_bus_addr, pp->io_size);

	return ret;
}

static int sunxi_pcie_valid_config(struct pcie_port *pp,
				struct pci_bus *bus, int dev)
{
	if (bus->number != pp->root_bus_nr) {
		if (!sunxi_pcie_link_up(pp))
			return 0;
	}

	if (bus->number == pp->root_bus_nr && dev > 0)
		return 0;

	return 1;
}

static int sunxi_pcie_rd_conf(struct pci_bus *bus, u32 devfn, int where,
			int size, u32 *val)
{
	struct pcie_port *pp = (bus->sysdata);
	int ret;

	if (!pp) {
		BUG();
		return -EINVAL;
	}

	if (!sunxi_pcie_valid_config(pp, bus, PCI_SLOT(devfn))) {
		*val = 0xffffffff;
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	if (bus->number != pp->root_bus_nr)
		ret = sunxi_pcie_rd_other_conf(pp, bus, devfn,
						where, size, val);
	else
		ret = sunxi_pcie_rd_own_conf(pp, where, size, val);

	return ret;
}

static int sunxi_pcie_wr_conf(struct pci_bus *bus, u32 devfn,
			int where, int size, u32 val)
{
	struct pcie_port *pp = (bus->sysdata);
	int ret;

	if (!pp) {
		BUG();
		return -EINVAL;
	}
	if (sunxi_pcie_valid_config(pp, bus, PCI_SLOT(devfn)) == 0)
		return PCIBIOS_DEVICE_NOT_FOUND;

	if (bus->number != pp->root_bus_nr)
		ret = sunxi_pcie_wr_other_conf(pp, bus, devfn,
						where, size, val);
	else
		ret = sunxi_pcie_wr_own_conf(pp, where, size, val);

	return ret;
}

static struct pci_ops sunxi_pcie_ops = {
	.read = sunxi_pcie_rd_conf,
	.write = sunxi_pcie_wr_conf,
};

int sunxi_pcie_host_init(struct pcie_port *pp)
{
	struct device_node *np = pp->dev->of_node;
	struct device *dev = pp->dev;
	struct resource_entry *win, *tmp;
	struct pci_bus *bus, *child;
	struct pci_host_bridge *bridge;
	int ret, i;

	bridge = devm_pci_alloc_host_bridge(dev, 0);
	if (!bridge)
		return -ENOMEM;

	ret = devm_of_pci_get_host_bridge_resources(dev, 0, 0xff,
					&bridge->windows, &pp->io_base);
	if (ret) {
		dev_err(dev, "pci_get_host_bridge_resources failed\n");
		return ret;
	}

	ret = devm_request_pci_bus_resources(dev, &bridge->windows);
	if (ret) {
		dev_err(dev, "request_pci_bus_resources failed\n");
		return ret;
	}

	/* Get the I/O and memory ranges from DTS */
	resource_list_for_each_entry_safe(win, tmp, &bridge->windows) {
		switch (resource_type(win->res)) {
		case IORESOURCE_IO:
			ret = devm_pci_remap_iospace(dev, win->res,
						     pp->io_base);
			if (ret) {
				dev_warn(dev, "Error %d: failed to map resource %pR\n",
					 ret, win->res);
				resource_list_destroy_entry(win);
			} else {
				pp->io = win->res;
				pp->io->name = "I/O";
				pp->io_size = resource_size(pp->io);
				pp->io_bus_addr = pp->io->start - win->offset;
			}
			break;
		case IORESOURCE_MEM:
			pp->mem = win->res;
			pp->mem->name = "MEM";
			pp->mem_size = resource_size(pp->mem);
			pp->mem_bus_addr = pp->mem->start - win->offset;
			break;
		case 0:
			pp->cfg = win->res;
			pp->cfg0_size = resource_size(pp->cfg) >> 1;
			pp->cfg1_size = resource_size(pp->cfg) >> 1;
			pp->cfg0_base = pp->cfg->start;
			pp->cfg1_base = pp->cfg->start + pp->cfg0_size;
			break;
		case IORESOURCE_BUS:
			pp->busn = win->res;
			break;
		}
	}

	if (!pp->dbi_base) {
		pp->dbi_base = devm_pci_remap_cfgspace(dev,
						pp->cfg->start,
						resource_size(pp->cfg));
		if (!pp->dbi_base) {
			dev_err(dev, "Error with ioremap\n");
			return -ENOMEM;
		}
	}

	pp->mem_base = pp->mem->start - PCIE_CPU_BASE;

	if (!pp->va_cfg0_base) {
		pp->va_cfg0_base = devm_pci_remap_cfgspace(dev,
					pp->cfg0_base, pp->cfg0_size);
		if (!pp->va_cfg0_base) {
			dev_err(dev, "Error with ioremap in function\n");
			return -ENOMEM;
		}
	}

	if (!pp->va_cfg1_base) {
		pp->va_cfg1_base = devm_pci_remap_cfgspace(dev,
					pp->cfg1_base, pp->cfg1_size);
		if (!pp->va_cfg1_base) {
			dev_err(dev, "Error with ioremap\n");
			return -ENOMEM;
		}
	}

	ret = of_property_read_u32(np, "num-viewport", &pp->num_viewport);
	if (ret)
		pp->num_viewport = 2;

	if (of_property_read_u32(np, "num-lanes", &pp->lanes)) {
		dev_err(pp->dev, "Failed to parse the number of lanes\n");
		return -EINVAL;
	}

	if (IS_ENABLED(CONFIG_PCI_MSI) && !pp->msi_ext) {
		pp->msi_domain = irq_domain_add_linear(pp->dev->of_node,
					INT_PCI_MSI_NR, &msi_domain_ops,
					&sunxi_pcie_msi_chip);
		if (!pp->msi_domain) {
			dev_err(pp->dev, "msi domain init failed\n");
			return -ENXIO;
		}


		for (i = 0; i < INT_PCI_MSI_NR; i++)
			irq_create_mapping(pp->msi_domain, i);

		ret = sunxi_pcie_msi_enable(pp);
		if (ret)
			return ret;
	}

	if (pp->ops->host_init)
		pp->ops->host_init(pp);

	pp->root_bus_nr = pp->busn->start;

	bridge->dev.parent = dev;
	bridge->sysdata = pp;
	bridge->busnr = pp->root_bus_nr;
	bridge->ops = &sunxi_pcie_ops;
	bridge->map_irq = of_irq_parse_and_map_pci;
	bridge->swizzle_irq = pci_common_swizzle;

	if (IS_ENABLED(CONFIG_PCI_MSI) && !pp->msi_ext) {
		bridge->msi = &sunxi_pcie_msi_chip;
	}

	ret = pci_scan_root_bus_bridge(bridge);
	if (ret)
		return ret;

	bus = bridge->bus;

	if (pp->ops->scan_bus)
		pp->ops->scan_bus(pp);

	pci_bus_size_bridges(bus);
	pci_bus_assign_resources(bus);

	list_for_each_entry(child, &bus->children, node)
		pcie_bus_configure_settings(child);

	pci_bus_add_devices(bus);

	return 0;
}
void sunxi_pcie_setup_rc(struct pcie_port *pp)
{
	u32 val;

	/* set the number of lanes */
	sunxi_pcie_readl_rc(pp, PCIE_PORT_LINK_CONTROL, &val);
	val &= ~PORT_LINK_MODE_MASK;
	switch (pp->lanes) {
	case 1:
		val |= PORT_LINK_MODE_1_LANES;
		break;
	case 2:
		val |= PORT_LINK_MODE_2_LANES;
		break;
	case 4:
		val |= PORT_LINK_MODE_4_LANES;
		break;
	default:
		dev_err(pp->dev, "num-lanes %u: invalid value\n", pp->lanes);
		return;
	}
	sunxi_pcie_writel_rc(pp, val, PCIE_PORT_LINK_CONTROL);

	/* set link width speed control register */
	sunxi_pcie_readl_rc(pp, PCIE_LINK_WIDTH_SPEED_CONTROL, &val);
	val &= ~PORT_LOGIC_LINK_WIDTH_MASK;
	switch (pp->lanes) {
	case 1:
		val |= PORT_LOGIC_LINK_WIDTH_1_LANES;
		break;
	case 2:
		val |= PORT_LOGIC_LINK_WIDTH_2_LANES;
		break;
	case 4:
		val |= PORT_LOGIC_LINK_WIDTH_4_LANES;
		break;
	}
	sunxi_pcie_writel_rc(pp, val, PCIE_LINK_WIDTH_SPEED_CONTROL);

	/* set mode gen1 for FPGA */
	sunxi_pcie_readl_rc(pp, 0xA0, &val);
	val &= ~(0xf<<0);
	val |= (0x1<<0);
	sunxi_pcie_writel_rc(pp, val, 0xA0);

	/* setup RC BARs */
	sunxi_pcie_writel_rc(pp, 0x00000004, PCI_BASE_ADDRESS_0);
	sunxi_pcie_writel_rc(pp, 0x00000000, PCI_BASE_ADDRESS_1);

	/* setup interrupt pins */
	sunxi_pcie_readl_rc(pp, PCI_INTERRUPT_LINE, &val);
	val &= PCIE_INTERRUPT_LINE_MASK;
	val |= PCIE_INTERRUPT_LINE_ENABLE;
	sunxi_pcie_writel_rc(pp, val, PCI_INTERRUPT_LINE);

	/* setup bus numbers */
	sunxi_pcie_readl_rc(pp, PCI_PRIMARY_BUS, &val);
	val &= 0xff000000;
	val |= 0x00ff0100;
	sunxi_pcie_writel_rc(pp, val, PCI_PRIMARY_BUS);

	/* setup command register */
	sunxi_pcie_readl_rc(pp, PCI_COMMAND, &val);

	val &= PCIE_HIGH16_MASK;
	val |= PCI_COMMAND_IO | PCI_COMMAND_MEMORY |
		PCI_COMMAND_MASTER | PCI_COMMAND_SERR;

	sunxi_pcie_writel_rc(pp, val, PCI_COMMAND);

	sunxi_pcie_prog_outbound_atu(pp, PCIE_ATU_REGION_INDEX0, PCIE_ATU_TYPE_MEM, pp->mem_base, pp->mem_bus_addr, pp->mem_size);

	if (pp->num_viewport > 2)
		sunxi_pcie_prog_outbound_atu(pp, PCIE_ATU_REGION_INDEX2, PCIE_ATU_TYPE_IO, pp->io_base - PCIE_CPU_BASE,
						pp->io_bus_addr,  pp->io_size);

	sunxi_pcie_wr_own_conf(pp, PCI_BASE_ADDRESS_0, 4, 0);

	sunxi_pcie_readl_rc(pp, PCIE_MISC_CONTROL_1_CFG, &val);
	val |= 0x1;
	sunxi_pcie_writel_rc(pp, val, PCIE_MISC_CONTROL_1_CFG);


	sunxi_pcie_wr_own_conf(pp, PCI_CLASS_DEVICE, 2, PCI_CLASS_BRIDGE_PCI);

	sunxi_pcie_readl_rc(pp, PCIE_MISC_CONTROL_1_CFG, &val);
	val &= ~(0x1<<0);
	sunxi_pcie_writel_rc(pp, val, PCIE_MISC_CONTROL_1_CFG);

	sunxi_pcie_rd_own_conf(pp, PCIE_LINK_WIDTH_SPEED_CONTROL, 4, &val);
	val |= PORT_LOGIC_SPEED_CHANGE;
	sunxi_pcie_wr_own_conf(pp, PCIE_LINK_WIDTH_SPEED_CONTROL, 4, val);
}

MODULE_AUTHOR("songjundong <songjundong@allwinnertech.com>");
MODULE_DESCRIPTION("sunxi PCIe host controller driver");
MODULE_VERSION("1.0.0");
MODULE_LICENSE("GPL");
