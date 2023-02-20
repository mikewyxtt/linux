// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe host bridge driver for Apple system-on-chips.
 *
 * The HW is ECAM compliant, so once the controller is initialized,
 * the driver mostly deals MSI mapping and handling of per-port
 * interrupts (INTx, management and error signals).
 *
 * Initialization requires enabling power and clocks, along with a
 * number of register pokes.
 *
 * Copyright (C) 2021 Alyssa Rosenzweig <alyssa@rosenzweig.io>
 * Copyright (C) 2021 Google LLC
 * Copyright (C) 2021 Corellium LLC
 * Copyright (C) 2021 Mark Kettenis <kettenis@openbsd.org>
 *
 * Author: Alyssa Rosenzweig <alyssa@rosenzweig.io>
 * Author: Marc Zyngier <maz@kernel.org>
 */

#include <linux/clk-provider.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/iopoll.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/msi.h>
#include <linux/notifier.h>
#include <linux/of_irq.h>
#include <linux/pci-ecam.h>
#include <linux/pm_domain.h>
#include <linux/reset.h>
#include <linux/thunderbolt.h>

/* for pci_dev_set_disconnected */
#include "../pci.h"

#define CORE_RC_PHYIF_CTL		0x00024
#define   CORE_RC_PHYIF_CTL_RUN		BIT(0)
#define CORE_RC_PHYIF_STAT		0x00028
#define   CORE_RC_PHYIF_STAT_REFCLK	BIT(4)
#define CORE_RC_CTL			0x00050
#define   CORE_RC_CTL_RUN		BIT(0)
#define CORE_RC_STAT			0x00058
#define   CORE_RC_STAT_READY		BIT(0)
#define CORE_FABRIC_STAT		0x04000
#define   CORE_FABRIC_STAT_MASK		0x001F001F
#define CORE_LANE_CFG(port)		(0x84000 + 0x4000 * (port))
#define   CORE_LANE_CFG_REFCLK0REQ	BIT(0)
#define   CORE_LANE_CFG_REFCLK1REQ	BIT(1)
#define   CORE_LANE_CFG_REFCLK0ACK	BIT(2)
#define   CORE_LANE_CFG_REFCLK1ACK	BIT(3)
#define   CORE_LANE_CFG_REFCLKEN	(BIT(9) | BIT(10))
#define CORE_LANE_CTL(port)		(0x84004 + 0x4000 * (port))
#define   CORE_LANE_CTL_CFGACC		BIT(15)

#define PORT_LTSSMCTL			0x00080
#define   PORT_LTSSMCTL_START		BIT(0)
#define PORT_INTSTAT			0x00100
#define   PORT_INT_TUNNEL_ERR		31
#define   PORT_INT_CPL_TIMEOUT		23
#define   PORT_INT_RID2SID_MAPERR	22
#define   PORT_INT_CPL_ABORT		21
#define   PORT_INT_MSI_BAD_DATA		19
#define   PORT_INT_MSI_ERR		18
#define   PORT_INT_REQADDR_GT32		17
#define   PORT_INT_AF_TIMEOUT		15
#define   PORT_INT_LINK_DOWN		14
#define   PORT_INT_LINK_UP		12
#define   PORT_INT_LINK_BWMGMT		11
#define   PORT_INT_AER_MASK		(15 << 4)
#define   PORT_INT_PORT_ERR		4
#define   PORT_INT_INTx(i)		i
#define   PORT_INT_INTx_MASK		15
#define PORT_INTMSK			0x00104
#define PORT_INTMSKSET			0x00108
#define PORT_INTMSKCLR			0x0010c
#define PORT_MSICFG			0x00124
#define   PORT_MSICFG_EN		BIT(0)
#define   PORT_MSICFG_L2MSINUM_SHIFT	4
#define PORT_MSIBASE			0x00128
#define   PORT_MSIBASE_1_SHIFT		16
#define PORT_MSIADDR			0x00168
#define PORT_LINKSTS			0x00208
#define   PORT_LINKSTS_UP		BIT(0)
#define   PORT_LINKSTS_BUSY		BIT(2)
#define PORT_LINKCMDSTS			0x00210
#define PORT_OUTS_NPREQS		0x00284
#define   PORT_OUTS_NPREQS_REQ		BIT(24)
#define   PORT_OUTS_NPREQS_CPL		BIT(16)
#define PORT_RXWR_FIFO			0x00288
#define   PORT_RXWR_FIFO_HDR		GENMASK(15, 10)
#define   PORT_RXWR_FIFO_DATA		GENMASK(9, 0)
#define PORT_RXRD_FIFO			0x0028C
#define   PORT_RXRD_FIFO_REQ		GENMASK(6, 0)
#define PORT_OUTS_CPLS			0x00290
#define   PORT_OUTS_CPLS_SHRD		GENMASK(14, 8)
#define   PORT_OUTS_CPLS_WAIT		GENMASK(6, 0)
#define PORT_APPCLK			0x00800
#define   PORT_APPCLK_EN		BIT(0)
#define   PORT_APPCLK_CGDIS		BIT(8)
#define PORT_STATUS			0x00804
#define   PORT_STATUS_READY		BIT(0)
#define PORT_REFCLK			0x00810
#define   PORT_REFCLK_EN		BIT(0)
#define   PORT_REFCLK_CGDIS		BIT(8)
#define PORT_PERST			0x00814
#define   PORT_PERST_OFF		BIT(0)
#define PORT_RID2SID(i16)		(0x00828 + 4 * (i16))
#define   PORT_RID2SID_VALID		BIT(31)
#define   PORT_RID2SID_SID_SHIFT	16
#define   PORT_RID2SID_BUS_SHIFT	8
#define   PORT_RID2SID_DEV_SHIFT	3
#define   PORT_RID2SID_FUNC_SHIFT	0
#define PORT_OUTS_PREQS_HDR		0x00980
#define   PORT_OUTS_PREQS_HDR_MASK	GENMASK(9, 0)
#define PORT_OUTS_PREQS_DATA		0x00984
#define   PORT_OUTS_PREQS_DATA_MASK	GENMASK(15, 0)
#define PORT_TUNCTRL			0x00988
#define   PORT_TUNCTRL_PERST_ON		BIT(0)
#define   PORT_TUNCTRL_PERST_ACK_REQ	BIT(1)
#define PORT_TUNSTAT			0x0098c
#define   PORT_TUNSTAT_PERST_ON		BIT(0)
#define   PORT_TUNSTAT_PERST_ACK_PEND	BIT(1)
#define PORT_PREFMEM_ENABLE		0x00994

#define MAX_RID2SID			64

#define USB4_TUNNEL_RETRIES		5

/*
 * The doorbell address is set to 0xfffff000, which by convention
 * matches what MacOS does, and it is possible to use any other
 * address (in the bottom 4GB, as the base register is only 32bit).
 * However, it has to be excluded from the IOVA range, and the DART
 * driver has to know about it.
 */
#define DOORBELL_ADDR		CONFIG_PCIE_APPLE_MSI_DOORBELL_ADDR

enum apple_pcie_type {
	APPLE_PCIE,
	APPLE_PCIE_USB4,
};

struct apple_pcie {
	struct mutex		lock;
	struct device		*dev;
	void __iomem            *base;
	struct irq_domain	*domain;
	unsigned long		*bitmap;
	struct list_head	ports;
	struct irq_fwspec	fwspec;
	u32			nvecs;
	struct device		**pd_dev;
	struct device_link	**pd_link;
	int			pd_count;
	enum apple_pcie_type	type;
	struct reset_control	*reset;
};

struct apple_pcie_port {
	struct apple_pcie	*pcie;
	struct device_node	*np;
	void __iomem		*base;
	struct irq_domain	*domain;
	struct list_head	entry;
	struct completion	event;
	bool			link_up;
	DECLARE_BITMAP(sid_map, MAX_RID2SID);
	int			sid_map_sz;
	int			idx;
	struct clk_hw		clk;
};

static void rmw_set(u32 set, void __iomem *addr)
{
	writel_relaxed(readl_relaxed(addr) | set, addr);
}

static void rmw_clear(u32 clr, void __iomem *addr)
{
	writel_relaxed(readl_relaxed(addr) & ~clr, addr);
}

static void apple_msi_top_irq_mask(struct irq_data *d)
{
	pci_msi_mask_irq(d);
	irq_chip_mask_parent(d);
}

static void apple_msi_top_irq_unmask(struct irq_data *d)
{
	pci_msi_unmask_irq(d);
	irq_chip_unmask_parent(d);
}

static struct irq_chip apple_msi_top_chip = {
	.name			= "PCIe MSI",
	.irq_mask		= apple_msi_top_irq_mask,
	.irq_unmask		= apple_msi_top_irq_unmask,
	.irq_eoi		= irq_chip_eoi_parent,
	.irq_set_affinity	= irq_chip_set_affinity_parent,
	.irq_set_type		= irq_chip_set_type_parent,
};

static void apple_msi_compose_msg(struct irq_data *data, struct msi_msg *msg)
{
	msg->address_hi = upper_32_bits(DOORBELL_ADDR);
	msg->address_lo = lower_32_bits(DOORBELL_ADDR);
	msg->data = data->hwirq;
}

static struct irq_chip apple_msi_bottom_chip = {
	.name			= "MSI",
	.irq_mask		= irq_chip_mask_parent,
	.irq_unmask		= irq_chip_unmask_parent,
	.irq_eoi		= irq_chip_eoi_parent,
	.irq_set_affinity	= irq_chip_set_affinity_parent,
	.irq_set_type		= irq_chip_set_type_parent,
	.irq_compose_msi_msg	= apple_msi_compose_msg,
};

static int apple_msi_domain_alloc(struct irq_domain *domain, unsigned int virq,
				  unsigned int nr_irqs, void *args)
{
	struct apple_pcie *pcie = domain->host_data;
	struct irq_fwspec fwspec = pcie->fwspec;
	unsigned int i;
	int ret, hwirq;

	mutex_lock(&pcie->lock);

	hwirq = bitmap_find_free_region(pcie->bitmap, pcie->nvecs,
					order_base_2(nr_irqs));

	mutex_unlock(&pcie->lock);

	if (hwirq < 0)
		return -ENOSPC;

	fwspec.param[fwspec.param_count - 2] += hwirq;

	ret = irq_domain_alloc_irqs_parent(domain, virq, nr_irqs, &fwspec);
	if (ret)
		return ret;

	for (i = 0; i < nr_irqs; i++) {
		irq_domain_set_hwirq_and_chip(domain, virq + i, hwirq + i,
					      &apple_msi_bottom_chip,
					      domain->host_data);
	}

	return 0;
}

static void apple_msi_domain_free(struct irq_domain *domain, unsigned int virq,
				  unsigned int nr_irqs)
{
	struct irq_data *d = irq_domain_get_irq_data(domain, virq);
	struct apple_pcie *pcie = domain->host_data;

	mutex_lock(&pcie->lock);

	bitmap_release_region(pcie->bitmap, d->hwirq, order_base_2(nr_irqs));

	mutex_unlock(&pcie->lock);
}

static const struct irq_domain_ops apple_msi_domain_ops = {
	.alloc	= apple_msi_domain_alloc,
	.free	= apple_msi_domain_free,
};

static struct msi_domain_info apple_msi_info = {
	.flags	= (MSI_FLAG_USE_DEF_DOM_OPS | MSI_FLAG_USE_DEF_CHIP_OPS |
		   MSI_FLAG_MULTI_PCI_MSI | MSI_FLAG_PCI_MSIX),
	.chip	= &apple_msi_top_chip,
};

static void apple_port_irq_mask(struct irq_data *data)
{
	struct apple_pcie_port *port = irq_data_get_irq_chip_data(data);

	writel_relaxed(BIT(data->hwirq), port->base + PORT_INTMSKSET);
}

static void apple_port_irq_unmask(struct irq_data *data)
{
	struct apple_pcie_port *port = irq_data_get_irq_chip_data(data);

	writel_relaxed(BIT(data->hwirq), port->base + PORT_INTMSKCLR);
}

static bool hwirq_is_intx(unsigned int hwirq)
{
	return BIT(hwirq) & PORT_INT_INTx_MASK;
}

static void apple_port_irq_ack(struct irq_data *data)
{
	struct apple_pcie_port *port = irq_data_get_irq_chip_data(data);

	if (!hwirq_is_intx(data->hwirq))
		writel_relaxed(BIT(data->hwirq), port->base + PORT_INTSTAT);
}

static int apple_port_irq_set_type(struct irq_data *data, unsigned int type)
{
	/*
	 * It doesn't seem that there is any way to configure the
	 * trigger, so assume INTx have to be level (as per the spec),
	 * and the rest is edge (which looks likely).
	 */
	if (hwirq_is_intx(data->hwirq) ^ !!(type & IRQ_TYPE_LEVEL_MASK))
		return -EINVAL;

	irqd_set_trigger_type(data, type);
	return 0;
}

static struct irq_chip apple_port_irqchip = {
	.name		= "PCIe",
	.irq_ack	= apple_port_irq_ack,
	.irq_mask	= apple_port_irq_mask,
	.irq_unmask	= apple_port_irq_unmask,
	.irq_set_type	= apple_port_irq_set_type,
};

static int apple_port_irq_domain_alloc(struct irq_domain *domain,
				       unsigned int virq, unsigned int nr_irqs,
				       void *args)
{
	struct apple_pcie_port *port = domain->host_data;
	struct irq_fwspec *fwspec = args;
	int i;

	for (i = 0; i < nr_irqs; i++) {
		irq_flow_handler_t flow = handle_edge_irq;
		unsigned int type = IRQ_TYPE_EDGE_RISING;

		if (hwirq_is_intx(fwspec->param[0] + i)) {
			flow = handle_level_irq;
			type = IRQ_TYPE_LEVEL_HIGH;
		}

		irq_domain_set_info(domain, virq + i, fwspec->param[0] + i,
				    &apple_port_irqchip, port, flow,
				    NULL, NULL);

		irq_set_irq_type(virq + i, type);
	}

	return 0;
}

static void apple_port_irq_domain_free(struct irq_domain *domain,
				       unsigned int virq, unsigned int nr_irqs)
{
	int i;

	for (i = 0; i < nr_irqs; i++) {
		struct irq_data *d = irq_domain_get_irq_data(domain, virq + i);

		irq_set_handler(virq + i, NULL);
		irq_domain_reset_irq_data(d);
	}
}

static const struct irq_domain_ops apple_port_irq_domain_ops = {
	.translate	= irq_domain_translate_onecell,
	.alloc		= apple_port_irq_domain_alloc,
	.free		= apple_port_irq_domain_free,
};

static void apple_port_irq_handler(struct irq_desc *desc)
{
	struct apple_pcie_port *port = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned long stat;
	int i;

	chained_irq_enter(chip, desc);

	stat = readl_relaxed(port->base + PORT_INTSTAT);

	for_each_set_bit(i, &stat, 32)
		generic_handle_domain_irq(port->domain, i);

	chained_irq_exit(chip, desc);
}

static int apple_pcie_port_setup_irq(struct apple_pcie_port *port)
{
	struct fwnode_handle *fwnode = &port->np->fwnode;
	unsigned int irq;

	/* FIXME: consider moving each interrupt under each port */
	irq = irq_of_parse_and_map(to_of_node(dev_fwnode(port->pcie->dev)),
				   port->idx);
	if (!irq)
		return -ENXIO;

	port->domain = irq_domain_create_linear(fwnode, 32,
						&apple_port_irq_domain_ops,
						port);
	if (!port->domain)
		return -ENOMEM;

	/* Disable all interrupts */
	writel_relaxed(~0, port->base + PORT_INTMSKSET);
	writel_relaxed(~0, port->base + PORT_INTSTAT);

	irq_set_chained_handler_and_data(irq, apple_port_irq_handler, port);

	/* Configure MSI base address */
	BUILD_BUG_ON(upper_32_bits(DOORBELL_ADDR));
	writel_relaxed(lower_32_bits(DOORBELL_ADDR), port->base + PORT_MSIADDR);

	/* Enable MSIs, shared between all ports */
	writel_relaxed(0, port->base + PORT_MSIBASE);
	writel_relaxed((ilog2(port->pcie->nvecs) << PORT_MSICFG_L2MSINUM_SHIFT) |
		       PORT_MSICFG_EN, port->base + PORT_MSICFG);

	return 0;
}

static irqreturn_t apple_pcie_port_irq(int irq, void *data)
{
	struct apple_pcie_port *port = data;
	unsigned int hwirq = irq_domain_get_irq_data(port->domain, irq)->hwirq;

	switch (hwirq) {
	case PORT_INT_LINK_UP:
		dev_info_ratelimited(port->pcie->dev, "Link up on %pOF\n",
				     port->np);
		port->link_up = true;
		complete(&port->event);
		break;
	case PORT_INT_LINK_DOWN:
		dev_info_ratelimited(port->pcie->dev, "Link down on %pOF\n",
				     port->np);
		port->link_up = false;
		complete(&port->event);
		break;
	case PORT_INT_TUNNEL_ERR:
		dev_err_ratelimited(port->pcie->dev, "Tunnel error on %pOF\n",
				    port->np);
		port->link_up = false;
		complete(&port->event);
		break;
	default:
		return IRQ_NONE;
	}

	return IRQ_HANDLED;
}

static int apple_pcie_port_register_irqs(struct apple_pcie_port *port)
{
	static struct {
		unsigned int	hwirq;
		const char	*name;
	} port_irqs[] = {
		{ PORT_INT_LINK_UP,	"Link up",	},
		{ PORT_INT_LINK_DOWN,	"Link down",	},
		{ PORT_INT_TUNNEL_ERR,	"Tunnel error",	},
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(port_irqs); i++) {
		struct irq_fwspec fwspec = {
			.fwnode		= &port->np->fwnode,
			.param_count	= 1,
			.param		= {
				[0]	= port_irqs[i].hwirq,
			},
		};
		unsigned int irq;
		int ret;

		irq = irq_domain_alloc_irqs(port->domain, 1, NUMA_NO_NODE,
					    &fwspec);
		if (WARN_ON(!irq))
			continue;

		ret = request_irq(irq, apple_pcie_port_irq, 0,
				  port_irqs[i].name, port);
		WARN_ON(ret);
	}

	return 0;
}

static int apple_pcie_setup_refclk(struct apple_pcie *pcie,
				   struct apple_pcie_port *port)
{
	u32 stat;
	int res;

	res = readl_relaxed_poll_timeout(pcie->base + CORE_RC_PHYIF_STAT, stat,
					 stat & CORE_RC_PHYIF_STAT_REFCLK,
					 100, 50000);
	if (res < 0)
		return res;

	rmw_set(CORE_LANE_CTL_CFGACC, pcie->base + CORE_LANE_CTL(port->idx));
	rmw_set(CORE_LANE_CFG_REFCLK0REQ, pcie->base + CORE_LANE_CFG(port->idx));

	res = readl_relaxed_poll_timeout(pcie->base + CORE_LANE_CFG(port->idx),
					 stat, stat & CORE_LANE_CFG_REFCLK0ACK,
					 100, 50000);
	if (res < 0)
		return res;

	rmw_set(CORE_LANE_CFG_REFCLK1REQ, pcie->base + CORE_LANE_CFG(port->idx));
	res = readl_relaxed_poll_timeout(pcie->base + CORE_LANE_CFG(port->idx),
					 stat, stat & CORE_LANE_CFG_REFCLK1ACK,
					 100, 50000);

	if (res < 0)
		return res;

	rmw_clear(CORE_LANE_CTL_CFGACC, pcie->base + CORE_LANE_CTL(port->idx));

	rmw_set(CORE_LANE_CFG_REFCLKEN, pcie->base + CORE_LANE_CFG(port->idx));
	rmw_set(PORT_REFCLK_EN, port->base + PORT_REFCLK);

	return 0;
}

static u32 apple_pcie_rid2sid_write(struct apple_pcie_port *port,
				    int idx, u32 val)
{
	writel_relaxed(val, port->base + PORT_RID2SID(idx));
	/* Read back to ensure completion of the write */
	return readl_relaxed(port->base + PORT_RID2SID(idx));
}

static void apple_pcie_stop_ltssm(struct apple_pcie_port *port)
{
	static struct {
		unsigned int	offset;
		unsigned int	mask;
		const char	*name;
	} fifo_levels[] = {
		{ PORT_OUTS_NPREQS,	PORT_OUTS_NPREQS_REQ,		"OUTS_NPREQS_REQ" },
		{ PORT_OUTS_NPREQS,	PORT_OUTS_NPREQS_CPL,		"OUTS_NPREQS_CPL" },
		{ PORT_OUTS_PREQS_HDR,	PORT_OUTS_PREQS_HDR_MASK,	"OUTS_PREQS_HDR"  },
		{ PORT_OUTS_PREQS_DATA,	PORT_OUTS_PREQS_DATA_MASK,	"OUTS_PREQS_DATA" },
		{ PORT_RXWR_FIFO,	PORT_RXWR_FIFO_DATA,		"RXWR_FIFO_DATA"  },
		{ PORT_RXWR_FIFO,	PORT_RXWR_FIFO_HDR,		"RXWR_FIFO_HDR"   },
		{ PORT_RXRD_FIFO,	PORT_RXRD_FIFO_REQ,		"RXRD_FIFO_REQ"   },
		{ PORT_OUTS_CPLS,	PORT_OUTS_CPLS_SHRD,		"OUTS_CPLS_SHRD"  },
	};
	u32 val;
	int ret;

	writel_relaxed(0, port->base + PORT_LTSSMCTL);

	for (int i = 0; i < ARRAY_SIZE(fifo_levels); i++) {
		ret = readl_relaxed_poll_timeout(port->base + fifo_levels[i].offset,
						val,
						!(val & fifo_levels[i].mask),
						100, USEC_PER_SEC / 100);
		if (ret)
			dev_warn(port->pcie->dev,
				"port %pOF waiting for %s to empty timed out\n",
				port->np,
				fifo_levels[i].name);
	}
}

static void apple_pcie_reset_tunnel(struct apple_pcie_port *port)
{
	struct apple_pcie *pcie = port->pcie;
	int ret;
	u32 stat;

	rmw_set(PORT_TUNCTRL_PERST_ON, port->base + PORT_TUNCTRL);
	ret = readl_relaxed_poll_timeout(port->base + PORT_TUNSTAT,
					stat,
					(stat & PORT_TUNSTAT_PERST_ON),
					100, USEC_PER_SEC / 100);
	if (ret < 0)
		dev_warn(pcie->dev,
			"port %pOF tunnel PERST set timeout\n", port->np);

	rmw_set(PORT_TUNCTRL_PERST_ACK_REQ, port->base + PORT_TUNCTRL);
	ret = readl_relaxed_poll_timeout(port->base + PORT_TUNSTAT,
					stat,
					!(stat & PORT_TUNSTAT_PERST_ACK_PEND),
					100, USEC_PER_SEC / 100);
	if (ret < 0)
		dev_warn(pcie->dev,
			"port %pOF tunnel PERST REQ ack clear timeout\n", port->np);

	rmw_clear(PORT_TUNCTRL_PERST_ACK_REQ, port->base + PORT_TUNCTRL);
}

static int apple_pcie_start_tunnel(struct tb_protocol_adapter *adapter)
{
	struct apple_pcie_port *port = tb_protocol_adapter_get_drvdata(adapter);
	struct apple_pcie *pcie = port->pcie;
	struct pci_host_bridge *bridge =
		platform_get_drvdata(to_platform_device(pcie->dev));
	struct pci_dev *dev;
	struct pci_bus *bus;
	int ret;
	u32 stat;

	/*
	 * Make sure nothing else tries to rescan the bus while we're bringing
	 * the tunnel up.
	 */
	pci_lock_rescan_remove();

	/*
	 * The root port device might alread exist here with an empty
	 * subordinate bus. Remove it first such that we don't leave the
	 * dangling bus behind when calling pci_hp_add_bridge.
	 */
	dev = pci_get_slot(bridge->bus, PCI_DEVFN(port->idx, 0));
	if (dev) {
		pci_stop_and_remove_bus_device(dev);
		pci_dev_put(dev);
	}

	port->link_up = false;
	ret = readl_relaxed_poll_timeout(port->base + PORT_STATUS, stat,
					 stat & PORT_STATUS_READY, 100, 250000);
	if (ret < 0) {
		dev_err(pcie->dev, "%pOF: ready wait timeout\n", port->np);
		ret = -ETIMEDOUT;
		goto out;
	}

	for (int attempt = 0; attempt < USB4_TUNNEL_RETRIES; ++attempt) {
		dev_dbg(pcie->dev, "%pOF: starting tunnel attempt %d\n",
			port->np, attempt);

		/* Deassert tunnel reset and wait until the tunnel is ready */
		rmw_clear(PORT_TUNCTRL_PERST_ON, port->base + PORT_TUNCTRL);
		ret = readl_relaxed_poll_timeout(port->base + PORT_TUNSTAT,
						stat,
						!(stat & PORT_TUNSTAT_PERST_ON),
						100, USEC_PER_SEC / 100);
		if (ret < 0) {
			dev_warn(port->pcie->dev,
				"%pOF: tunnel PERST clear timeout\n", port->np);
			apple_pcie_reset_tunnel(port);
			continue;
		}

		/* Start link training and wait for the next interrupt */
		reinit_completion(&port->event);
		writel_relaxed(PORT_LTSSMCTL_START, port->base + PORT_LTSSMCTL);
		wait_for_completion_timeout(&port->event, HZ / 10);
		if (port->link_up)
			break;

		/* On tunnel error/link down reset the tunnel and try again */
		dev_warn(pcie->dev, "%pOF: link didn't come up\n", port->np);
		apple_pcie_stop_ltssm(port);
		apple_pcie_reset_tunnel(port);
	}

	if (!port->link_up) {
		dev_warn(pcie->dev,
			"%pOF: tunnel didn't come up after %d attempts; giving up.\n",
			port->np, USB4_TUNNEL_RETRIES);
		ret = -ETIMEDOUT;
		goto out;
	}

	/* Recreate the root port device */
	pci_scan_slot(bridge->bus, PCI_DEVFN(port->idx, 0));
	dev = pci_get_slot(bridge->bus, PCI_DEVFN(port->idx, 0));
	if (!dev) {
		dev_err(pcie->dev, "%pOF: root port device not found after rescan.\n", port->np);
		ret = -ENODEV;
		goto out;
	}

	/*
	 * Add the root port as a hot plug bridge to create its bus and assign
	 * the required resources.
	 */
	pci_hp_add_bridge(dev);
	pci_assign_unassigned_root_bus_resources(bridge->bus);
	list_for_each_entry(bus, &bridge->bus->children, node) {
		if (bus->self == dev) {
			pcie_bus_configure_settings(bus);
			break;
		}
	}
	pci_dev_put(dev);

	/*
	 * Re-enable prefetchable memory which was automatically disabled by
	 * the hardware when the resources were reconfigured.
	 */
	writel_relaxed(1, port->base + PORT_PREFMEM_ENABLE);

	/* And finally scan the new bus for devices */
	pci_bus_add_devices(bridge->bus);

out:
	pci_unlock_rescan_remove();
	return ret;
}

static void apple_usb4_pcie_reset(struct apple_pcie_port *port)
{
	int ret;
	u32 intmsk, stat;

	/*
	 * This is quite a hack to bring the controller back to a working
	 * state without having to re-setup everything inside Linux.
	 * As long as the following is true nothing bad should happen:
	 * - This is a USB4 controller with only a single port
	 * - There are no devices attached and the root port itself has been
	 *   removed
	 * - pci_lock_rescan_remove is held
	 */

	/* Disable all interrupts just in case we get one during reset */
	intmsk = readl_relaxed(port->base + PORT_INTMSK);
	writel_relaxed(0xffffffff, port->base + PORT_INTMSK);

	/* Trigger the PMGR reset */
	reset_control_reset(port->pcie->reset);

	/* Restore the port */
	rmw_set(PORT_REFCLK_EN, port->base + PORT_REFCLK);
	rmw_set(PORT_APPCLK_EN, port->base + PORT_APPCLK);
	rmw_clear(PORT_APPCLK_CGDIS, port->base + PORT_APPCLK);

	/* The minimal Tperst-clk value is 100us (PCIe CEM r5.0, 2.9.2) */
	usleep_range(100, 200);

	/* Deassert PERST# */
	rmw_set(PORT_PERST_OFF, port->base + PORT_PERST);

	/* Wait for 100ms after PERST# deassertion (PCIe r5.0, 6.6.1) */
	msleep(100);

	ret = readl_relaxed_poll_timeout(port->base + PORT_STATUS, stat,
					 stat & PORT_STATUS_READY, 100, 250000);
	if (ret < 0)
		dev_err(port->pcie->dev, "port %pOF ready wait timeout\n", port->np);

	rmw_clear(PORT_REFCLK_CGDIS, port->base + PORT_REFCLK);

	/* Re-enable MSI and restore the original interrupt mask */
	writel_relaxed(lower_32_bits(DOORBELL_ADDR), port->base + PORT_MSIADDR);
	writel_relaxed(0, port->base + PORT_MSIBASE);
	writel_relaxed((ilog2(port->pcie->nvecs) << PORT_MSICFG_L2MSINUM_SHIFT) |
		       PORT_MSICFG_EN, port->base + PORT_MSICFG);
	writel_relaxed(intmsk, port->base + PORT_INTMSK);
}

static void apple_pcie_stop_tunnel(struct tb_protocol_adapter *adapter)
{
	struct apple_pcie_port *port = tb_protocol_adapter_get_drvdata(adapter);
	struct apple_pcie *pcie = port->pcie;
	struct pci_host_bridge *bridge =
		platform_get_drvdata(to_platform_device(port->pcie->dev));
	struct pci_dev *dev;

	dev_dbg(pcie->dev, "%pOF: stopping tunnel\n", port->np);
	pci_lock_rescan_remove();

	dev = pci_get_slot(bridge->bus, PCI_DEVFN(port->idx, 0));
	if (dev) {
		/* The tunnel is already gone, mark all devices as dead */
		if (dev->subordinate)
			pci_walk_bus(dev->subordinate, pci_dev_set_disconnected, NULL);

		/* Remove the root port device and the corresponding bus */
		pci_stop_and_remove_bus_device(dev);
		pci_dev_put(dev);
	}

	/* Disable link training and assert tunnel reset */
	apple_pcie_stop_ltssm(port);
	apple_pcie_reset_tunnel(port);

	/*
	 * Reset and reinitialize the entire controller. This is a bit of a hack
	 * and assumes we really only have this single port which is always
	 * true for the USB4 ports.
	 */
	apple_usb4_pcie_reset(port);

	/*
	 * Rescan the root port device since it still exists. This technically
	 * isn't required but the port initially comes up with no subordinate
	 * devices after probing and would also show up again when a manual
	 * rescan is triggered. Just already restore it here for consistency.
	 */
	pci_scan_slot(bridge->bus, PCI_DEVFN(port->idx, 0));

	pci_unlock_rescan_remove();
}

#define clkhw_to_port(_hw) container_of(_hw, struct apple_pcie_port, clk)

static int apple_pcie_appclk_enable(struct clk_hw *hw)
{
	struct apple_pcie_port *port = clkhw_to_port(hw);

	rmw_set(PORT_APPCLK_EN, port->base + PORT_APPCLK);
	rmw_clear(PORT_APPCLK_CGDIS, port->base + PORT_APPCLK);

	return 0;
}

static int apple_pcie_appclk_is_enabled(struct clk_hw *hw)
{
	struct apple_pcie_port *port = clkhw_to_port(hw);

	return !!(readl_relaxed(port->base + PORT_APPCLK) & PORT_APPCLK_EN);
}

static void apple_pcie_appclk_disable(struct clk_hw *hw)
{
	struct apple_pcie_port *port = clkhw_to_port(hw);

	rmw_set(PORT_APPCLK_CGDIS, port->base + PORT_APPCLK);
	rmw_clear(PORT_APPCLK_EN, port->base + PORT_APPCLK);
}

static const struct clk_ops apple_pcie_appclk_ops = {
	.enable = apple_pcie_appclk_enable,
	.disable = apple_pcie_appclk_disable,
	.is_enabled = apple_pcie_appclk_is_enabled,
};

static int apple_pcie_setup_port(struct apple_pcie *pcie,
				 struct device_node *np)
{
	struct platform_device *platform = to_platform_device(pcie->dev);
	struct apple_pcie_port *port;
	struct gpio_desc *reset;
	struct tb_protocol_adapter *adapter;
	u32 stat, idx;
	int ret, i;
	struct clk_init_data clk_init = {
		.ops = &apple_pcie_appclk_ops,
		.num_parents = 0,
	};
	struct tb_protocol_adapter_desc adapter_desc = {
		.activate = apple_pcie_start_tunnel,
		.deactivate = apple_pcie_stop_tunnel,
	};

	if (pcie->type == APPLE_PCIE) {
		reset = devm_fwnode_gpiod_get(pcie->dev, of_fwnode_handle(np),
					"reset", GPIOD_OUT_LOW, "PERST#");
		if (IS_ERR(reset))
			return PTR_ERR(reset);
	}

	if (pcie->type == APPLE_PCIE_USB4) {
		pcie->reset = devm_reset_control_array_get_exclusive(pcie->dev);
		if (IS_ERR(pcie->reset))
			return PTR_ERR(pcie->reset);

		ret = reset_control_deassert(pcie->reset);
		if (ret)
			return ret;
	}

	port = devm_kzalloc(pcie->dev, sizeof(*port), GFP_KERNEL);
	if (!port)
		return -ENOMEM;

	ret = of_property_read_u32_index(np, "reg", 0, &idx);
	if (ret)
		return ret;

	/* Use the first reg entry to work out the port index */
	port->idx = idx >> 11;
	port->pcie = pcie;
	port->np = np;

	port->base = devm_platform_ioremap_resource(platform, port->idx + 2);
	if (IS_ERR(port->base))
		return PTR_ERR(port->base);

	/*
	 * The clock must be enabled here and is never disabled afterwards.
	 * Instead of adding more code to manually call clk_prepare_enable
	 * it's just marked as critical for now.
	 */
	clk_init.flags = CLK_IS_CRITICAL;
	clk_init.name = devm_kasprintf(pcie->dev, GFP_KERNEL, "%pOF", np);
	port->clk.init = &clk_init;
	ret = of_clk_hw_register(port->np, &port->clk);
	if (ret)
		return ret;

	ret = of_clk_add_hw_provider(port->np, of_clk_hw_simple_get, &port->clk);
	if (ret)
		return ret;

	if (pcie->type == APPLE_PCIE) {
		/* Assert PERST# before setting up the clock */
		gpiod_set_value(reset, 1);

		ret = apple_pcie_setup_refclk(pcie, port);
		if (ret < 0)
			return ret;
	}

	/* The minimal Tperst-clk value is 100us (PCIe CEM r5.0, 2.9.2) */
	usleep_range(100, 200);

	/* Deassert PERST# */
	rmw_set(PORT_PERST_OFF, port->base + PORT_PERST);
	if (pcie->type == APPLE_PCIE)
		gpiod_set_value(reset, 0);

	/* Wait for 100ms after PERST# deassertion (PCIe r5.0, 6.6.1) */
	msleep(100);

	ret = readl_relaxed_poll_timeout(port->base + PORT_STATUS, stat,
					 stat & PORT_STATUS_READY, 100, 250000);
	if (ret < 0) {
		dev_err(pcie->dev, "port %pOF ready wait timeout\n", np);
		return ret;
	}

	rmw_clear(PORT_REFCLK_CGDIS, port->base + PORT_REFCLK);

	ret = apple_pcie_port_setup_irq(port);
	if (ret)
		return ret;

	/* Reset all RID/SID mappings, and check for RAZ/WI registers */
	for (i = 0; i < MAX_RID2SID; i++) {
		if (apple_pcie_rid2sid_write(port, i, 0xbad1d) != 0xbad1d)
			break;
		apple_pcie_rid2sid_write(port, i, 0);
	}

	dev_dbg(pcie->dev, "%pOF: %d RID/SID mapping entries\n", np, i);

	port->sid_map_sz = i;

	list_add_tail(&port->entry, &pcie->ports);
	init_completion(&port->event);

	ret = apple_pcie_port_register_irqs(port);
	WARN_ON(ret);

	switch (pcie->type) {
	case APPLE_PCIE:
		/* Start link training directly for internal PCIe */
		writel_relaxed(PORT_LTSSMCTL_START, port->base + PORT_LTSSMCTL);
		wait_for_completion_timeout(&port->event, HZ / 10);
		if (!port->link_up)
			dev_warn(pcie->dev, "%pOF link didn't come up\n", np);
		break;
	case APPLE_PCIE_USB4:
		/* Link training will be started once a tunnel is established */
		adapter_desc.driver_data = port;
		adapter_desc.fwnode = of_fwnode_handle(np);
		adapter = tb_protocol_adapter_register(pcie->dev, &adapter_desc);
		if (IS_ERR(adapter))
			return dev_err_probe(pcie->dev, PTR_ERR(adapter),
				"%pOF: could not register USB4 protocol adapter.\n",
				np);
		break;
	}

	return 0;
}

static int apple_msi_init(struct apple_pcie *pcie)
{
	struct fwnode_handle *fwnode = dev_fwnode(pcie->dev);
	struct of_phandle_args args = {};
	struct irq_domain *parent;
	int ret;

	ret = of_parse_phandle_with_args(to_of_node(fwnode), "msi-ranges",
					 "#interrupt-cells", 0, &args);
	if (ret)
		return ret;

	ret = of_property_read_u32_index(to_of_node(fwnode), "msi-ranges",
					 args.args_count + 1, &pcie->nvecs);
	if (ret)
		return ret;

	of_phandle_args_to_fwspec(args.np, args.args, args.args_count,
				  &pcie->fwspec);

	pcie->bitmap = devm_bitmap_zalloc(pcie->dev, pcie->nvecs, GFP_KERNEL);
	if (!pcie->bitmap)
		return -ENOMEM;

	parent = irq_find_matching_fwspec(&pcie->fwspec, DOMAIN_BUS_WIRED);
	if (!parent) {
		dev_err(pcie->dev, "failed to find parent domain\n");
		return -ENXIO;
	}

	parent = irq_domain_create_hierarchy(parent, 0, pcie->nvecs, fwnode,
					     &apple_msi_domain_ops, pcie);
	if (!parent) {
		dev_err(pcie->dev, "failed to create IRQ domain\n");
		return -ENOMEM;
	}
	irq_domain_update_bus_token(parent, DOMAIN_BUS_NEXUS);

	pcie->domain = pci_msi_create_irq_domain(fwnode, &apple_msi_info,
						 parent);
	if (!pcie->domain) {
		dev_err(pcie->dev, "failed to create MSI domain\n");
		irq_domain_remove(parent);
		return -ENOMEM;
	}

	return 0;
}

static struct apple_pcie_port *apple_pcie_get_port(struct pci_dev *pdev)
{
	struct pci_config_window *cfg = pdev->sysdata;
	struct apple_pcie *pcie = cfg->priv;
	struct pci_dev *port_pdev;
	struct apple_pcie_port *port;

	/* Find the root port this device is on */
	port_pdev = pcie_find_root_port(pdev);

	/* If finding the port itself, nothing to do */
	if (WARN_ON(!port_pdev) || pdev == port_pdev)
		return NULL;

	list_for_each_entry(port, &pcie->ports, entry) {
		if (port->idx == PCI_SLOT(port_pdev->devfn))
			return port;
	}

	return NULL;
}

static int apple_pcie_add_device(struct apple_pcie_port *port,
				 struct pci_dev *pdev)
{
	u32 sid, rid = PCI_DEVID(pdev->bus->number, pdev->devfn);
	int idx, err;

	dev_dbg(&pdev->dev, "added to bus %s, index %d\n",
		pci_name(pdev->bus->self), port->idx);

	err = of_map_id(port->pcie->dev->of_node, rid, "iommu-map",
			"iommu-map-mask", NULL, &sid);
	if (err)
		return err;

	mutex_lock(&port->pcie->lock);

	idx = bitmap_find_free_region(port->sid_map, port->sid_map_sz, 0);
	if (idx >= 0) {
		apple_pcie_rid2sid_write(port, idx,
					 PORT_RID2SID_VALID |
					 (sid << PORT_RID2SID_SID_SHIFT) | rid);

		dev_dbg(&pdev->dev, "mapping RID%x to SID%x (index %d)\n",
			rid, sid, idx);
	}

	mutex_unlock(&port->pcie->lock);

	return idx >= 0 ? 0 : -ENOSPC;
}

static void apple_pcie_release_device(struct apple_pcie_port *port,
				      struct pci_dev *pdev)
{
	u32 rid = PCI_DEVID(pdev->bus->number, pdev->devfn);
	int idx;

	mutex_lock(&port->pcie->lock);

	for_each_set_bit(idx, port->sid_map, port->sid_map_sz) {
		u32 val;

		val = readl_relaxed(port->base + PORT_RID2SID(idx));
		if ((val & 0xffff) == rid) {
			apple_pcie_rid2sid_write(port, idx, 0);
			bitmap_release_region(port->sid_map, idx, 0);
			dev_dbg(&pdev->dev, "Released %x (%d)\n", val, idx);
			break;
		}
	}

	mutex_unlock(&port->pcie->lock);
}

static int apple_pcie_bus_notifier(struct notifier_block *nb,
				   unsigned long action,
				   void *data)
{
	struct device *dev = data;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct apple_pcie_port *port;
	int err;

	/*
	 * This is a bit ugly. We assume that if we get notified for
	 * any PCI device, we must be in charge of it, and that there
	 * is no other PCI controller in the whole system. It probably
	 * holds for now, but who knows for how long?
	 */
	port = apple_pcie_get_port(pdev);
	if (!port)
		return NOTIFY_DONE;

	switch (action) {
	case BUS_NOTIFY_ADD_DEVICE:
		err = apple_pcie_add_device(port, pdev);
		if (err)
			return notifier_from_errno(err);
		break;
	case BUS_NOTIFY_DEL_DEVICE:
		apple_pcie_release_device(port, pdev);
		break;
	default:
		return NOTIFY_DONE;
	}

	return NOTIFY_OK;
}

static struct notifier_block apple_pcie_nb = {
	.notifier_call = apple_pcie_bus_notifier,
};

static DEFINE_MUTEX(apple_pcie_bus_notifier_lock);
static unsigned int apple_pcie_bus_notifier_count = 0;

static int apple_pcie_register_bus_notifier(void)
{
	int ret = 0;

	mutex_lock(&apple_pcie_bus_notifier_lock);
	if (apple_pcie_bus_notifier_count == 0)
		ret = bus_register_notifier(&pci_bus_type, &apple_pcie_nb);
	if (!ret)
		apple_pcie_bus_notifier_count++;
	mutex_unlock(&apple_pcie_bus_notifier_lock);

	return ret;
}

static void apple_pcie_unregister_bus_notifier(void)
{
	mutex_lock(&apple_pcie_bus_notifier_lock);
	apple_pcie_bus_notifier_count--;
	if (!apple_pcie_bus_notifier_count)
		bus_unregister_notifier(&pci_bus_type, &apple_pcie_nb);
	mutex_unlock(&apple_pcie_bus_notifier_lock);
}

static void apple_pcie_detach_genpd(void *data)
{
	struct apple_pcie *pcie = data;
	int i;

	if (pcie->pd_count <= 1)
		return;

	for (i = pcie->pd_count - 1; i >= 0; i--) {
		if (pcie->pd_link[i])
			device_link_del(pcie->pd_link[i]);
		if (!IS_ERR_OR_NULL(pcie->pd_dev[i]))
			dev_pm_domain_detach(pcie->pd_dev[i], true);
	}
}

static int apple_pcie_attach_genpd(struct apple_pcie *pcie)
{
	struct device *dev = pcie->dev;
	int i;

	pcie->pd_count = of_count_phandle_with_args(
		dev->of_node, "power-domains", "#power-domain-cells");
	if (pcie->pd_count <= 1)
		return 0;

	pcie->pd_dev = devm_kcalloc(dev, pcie->pd_count, sizeof(*pcie->pd_dev),
				   GFP_KERNEL);
	if (!pcie->pd_dev)
		return -ENOMEM;

	pcie->pd_link = devm_kcalloc(dev, pcie->pd_count, sizeof(*pcie->pd_link),
				    GFP_KERNEL);
	if (!pcie->pd_link)
		return -ENOMEM;

	for (i = 0; i < pcie->pd_count; i++) {
		pcie->pd_dev[i] = dev_pm_domain_attach_by_id(dev, i);
		if (IS_ERR(pcie->pd_dev[i])) {
			apple_pcie_detach_genpd(pcie);
			return PTR_ERR(pcie->pd_dev[i]);
		}

		pcie->pd_link[i] = device_link_add(dev, pcie->pd_dev[i],
						  DL_FLAG_STATELESS |
						  DL_FLAG_PM_RUNTIME |
						  DL_FLAG_RPM_ACTIVE);
		if (!pcie->pd_link[i]) {
			apple_pcie_detach_genpd(pcie);
			return -EINVAL;
		}
	}

	return 0;
}

static int apple_pcie_init_common(struct pci_config_window *cfg,
				  enum apple_pcie_type type)
{
	struct device *dev = cfg->parent;
	struct platform_device *platform = to_platform_device(dev);
	struct device_node *of_port;
	struct apple_pcie *pcie;
	int ret;

	pcie = devm_kzalloc(dev, sizeof(*pcie), GFP_KERNEL);
	if (!pcie)
		return -ENOMEM;

	pcie->dev = dev;
	pcie->type = type;

	mutex_init(&pcie->lock);

	ret = apple_pcie_attach_genpd(pcie);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, apple_pcie_detach_genpd, pcie);
	if (ret)
		return ret;

	pcie->base = devm_platform_ioremap_resource(platform, 1);
	if (IS_ERR(pcie->base))
		return PTR_ERR(pcie->base);

	cfg->priv = pcie;
	INIT_LIST_HEAD(&pcie->ports);

	for_each_child_of_node(dev->of_node, of_port) {
		ret = apple_pcie_setup_port(pcie, of_port);
		if (ret) {
			dev_err(pcie->dev, "Port %pOF setup fail: %d\n", of_port, ret);
			of_node_put(of_port);
			return ret;
		}
	}

	return apple_msi_init(pcie);
}

static int apple_pcie_init(struct pci_config_window *cfg)
{
	return apple_pcie_init_common(cfg, APPLE_PCIE);
}

static int apple_pcie_init_usb4(struct pci_config_window *cfg)
{
	return apple_pcie_init_common(cfg, APPLE_PCIE_USB4);
}

static int apple_pcie_probe(struct platform_device *pdev)
{
	int ret;

	ret = apple_pcie_register_bus_notifier();
	if (ret)
		return ret;

	ret = pci_host_common_probe(pdev);
	if (ret)
		apple_pcie_unregister_bus_notifier();

	return ret;
}

static const struct pci_ecam_ops apple_pcie_cfg_ecam_ops = {
	.init		= apple_pcie_init,
	.pci_ops	= {
		.map_bus	= pci_ecam_map_bus,
		.read		= pci_generic_config_read,
		.write		= pci_generic_config_write,
	}
};

static const struct pci_ecam_ops apple_pcie_usb4_cfg_ecam_ops = {
	.init		= apple_pcie_init_usb4,
	.pci_ops	= {
		.map_bus	= pci_ecam_map_bus,
		.read		= pci_generic_config_read,
		.write		= pci_generic_config_write,
	}
};

static const struct of_device_id apple_pcie_of_match[] = {
	{ .compatible = "apple,pcie", .data = &apple_pcie_cfg_ecam_ops },
	{ .compatible = "apple,usb4-pcie", .data = &apple_pcie_usb4_cfg_ecam_ops },
	{ }
};
MODULE_DEVICE_TABLE(of, apple_pcie_of_match);

static struct platform_driver apple_pcie_driver = {
	.probe	= apple_pcie_probe,
	.driver	= {
		.name			= "pcie-apple",
		.of_match_table		= apple_pcie_of_match,
		.suppress_bind_attrs	= true,
	},
};
module_platform_driver(apple_pcie_driver);

MODULE_LICENSE("GPL v2");
