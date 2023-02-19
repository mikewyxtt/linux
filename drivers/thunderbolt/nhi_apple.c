// SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause
/*
 * Apple Converged IO ("CIO") NHI driver
 *
 * Copyright (C) The Asahi Linux Contributors
 * Author: Sven Peter <sven@svenpeter.dev>
 */
#define DEBUG

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/iommu.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/phy/phy.h>
#include <linux/soc/apple/rtkit.h>
#include <linux/types.h>
#include <linux/usb/typec_mux.h>
#include <linux/usb/typec_tbt.h>
#include <linux/usb/pd.h>

#include "nhi.h"
#include "nhi_regs.h"
#include "tb.h"
#include "tunnel.h"

#define MAX_ROOT_SWITCH_PORTS 10

#define APPLE_CIO_M3_CTRL 0x0c
#define APPLE_CIO_M3_CTRL_START BIT(1)
#define APPLE_CIO_M3_STAT 0xa8
#define APPLE_CIO_M3_STAT_STATE GENMASK(30, 24) // (0x7F << 24)

#define APPLE_CIO_NHI_HOP_COUNT 0x0

#define APPLE_CIO_NHI_TXRING_DESC_BASE 0x10000
#define APPLE_CIO_NHI_RXRING_DESC_BASE 0x80000
#define APPLE_CIO_NHI_RING_MMIO_SIZE 0x20

#define APPLE_CIO_NHI_IRQ_STATUS 0xD0000
#define APPLE_CIO_NHI_IRQ_ENABLE 0xD0010

#define TB_VSE_CAP_APPLE 0x00
#define TB_VSE_CAP_APPLE_CABLE_INFO 0x01
#define TB_VSE_CAP_APPLE_CABLE_INFO_PRESENT BIT(0)
#define TB_VSE_CAP_APPLE_CABLE_INFO_ORIENTATION BIT(1)
#define TB_VSE_CAP_APPLE_CABLE_INFO_ACTIVE BIT(2)
#define TB_VSE_CAP_APPLE_CABLE_INFO_LINK_TRAINING BIT(3)
#define TB_VSE_CAP_APPLE_CABLE_INFO_20_GBPS BIT(4)
#define TB_VSE_CAP_APPLE_CABLE_INFO_LEGACY_ADAPTER BIT(9)
#define TB_VSE_CAP_APPLE_CABLE_INFO_TBT2_3 BIT(10)

struct apple_cio_tunable {
	size_t sz;
	struct {
		u32 offset;
		u32 mask;
		u32 value;
	} * values;
};

struct apple_cio {
	struct device *dev;
	struct platform_device *pdev;
	struct device_node *np;
	struct apple_rtkit *rtk;
	struct tb *tb;
	struct tb_nhi nhi;

	void __iomem *regs_nhi;
	void __iomem *regs_m3;
	void __iomem *regs_pcie;

	struct apple_cio_tunable tunable_nhi;
	struct apple_cio_tunable tunable_m3;
	struct apple_cio_tunable tunable_pcie;

	int *tx_irqs;
	int *rx_irqs;
	size_t n_rings;

	struct phy *phy;

	struct clk_bulk_data *clks;
	int num_clks;

	u32 current_cable_info;
	u32 target_cable_info;
	enum typec_orientation orientation;

	struct work_struct mux_set_work;

	struct resource *sram;
	void __iomem *sram_base;
	u64 sram_dva_base;

	struct tb_protocol_adapter *adapters[MAX_ROOT_SWITCH_PORTS];
	struct mutex lock;

	struct platform_device *iommu_pdev;
	struct iommu_domain *iommu_dom;
	struct platform_device *nhi_pdev;

	bool online;

	struct device		**pd_dev;
	struct device_link	**pd_link;
	int			pd_count;
};

#define nhi_to_acio(nhi_) container_of((nhi_), struct apple_cio, nhi)

static void apple_cio_rtkit_crashed(void *cookie)
{
	struct apple_cio *acio = cookie;

	dev_err(acio->dev, "RTKit crashed; unable to recover without a reboot");
}

static int apple_cio_rtkit_shmem_setup(void *cookie,
				       struct apple_rtkit_shmem *bfr)
{
	struct apple_cio *acio = cookie;
	struct resource res = {
		.start = bfr->iova - acio->sram_dva_base + acio->sram->start,
		.end = bfr->iova - acio->sram_dva_base + acio->sram->start +
		       bfr->size - 1,
		.name = "acio_rtkit_buffer",
		.flags = acio->sram->flags,
	};

	if (!bfr->iova)
		return -EIO;

	if (res.end < res.start) {
		dev_err(acio->dev, "firmware requested invalid buffer %pR",
			&res);
		return -EFAULT;
	}

	if (!resource_contains(acio->sram, &res)) {
		dev_err(acio->dev,
			"firmware requested buffer %pR outside SRAM %pR", &res,
			acio->sram);
		return -EFAULT;
	}

	bfr->iomem = acio->sram_base + (res.start - acio->sram->start);
	bfr->is_mapped = true;

	return 0;
}

static void apple_cio_rtkit_shmem_destroy(void *cookie,
					  struct apple_rtkit_shmem *bfr)
{
	// no-op
}

static const struct apple_rtkit_ops apple_cio_rtkit_ops = {
	.crashed = apple_cio_rtkit_crashed,
	.shmem_destroy = apple_cio_rtkit_shmem_destroy,
	.shmem_setup = apple_cio_rtkit_shmem_setup,
};

static int apple_cio_parse_tunable(struct apple_cio *acio,
				   struct apple_cio_tunable *tunable,
				   const char *name)
{
	struct property *prop;
	const __le32 *p = NULL;
	int i;

	prop = of_find_property(acio->np, name, NULL);
	if (!prop) {
		dev_err(acio->dev, "tunable %s not found\n", name);
		return -ENOENT;
	}

	if (prop->length % (3 * sizeof(u32))) {
		dev_err(acio->dev, "tunable %s has invalid length\n", name);
		return -EINVAL;
	}

	tunable->sz = prop->length / (3 * sizeof(u32));
	tunable->values = devm_kcalloc(acio->dev, tunable->sz,
				       sizeof(*tunable->values), GFP_KERNEL);
	if (!tunable->values)
		return -ENOMEM;

	for (i = 0; i < tunable->sz; ++i) {
		p = of_prop_next_u32(prop, p, &tunable->values[i].offset);
		p = of_prop_next_u32(prop, p, &tunable->values[i].mask);
		p = of_prop_next_u32(prop, p, &tunable->values[i].value);
	}

	return 0;
}

static void apple_cio_apply_tunable(struct apple_cio_tunable *tunable,
				    void __iomem *regs)
{
	for (size_t i = 0; i < tunable->sz; ++i) {
		u32 reg = readl(regs + tunable->values[i].offset);
		reg &= ~tunable->values[i].mask;
		reg |= tunable->values[i].value;
		writel(reg, regs + tunable->values[i].offset);
	}
}

static int apple_cio_probe_irqs(struct apple_cio *acio)
{
	int ret;
	int n_irqs;
	char name[64];

	n_irqs = platform_irq_count(acio->pdev);
	if (n_irqs < 0)
		return dev_err_probe(acio->dev, n_irqs,
				     "platform_irq_count failed\n");
	if (n_irqs == 0) {
		dev_err(acio->dev, "No interrupts found\n");
		return -EINVAL;
	}
	if (n_irqs % 2) {
		dev_err(acio->dev,
			"Invalid number of irqs: %d should be even\n", n_irqs);
		return -EINVAL;
	}
	acio->n_rings = n_irqs / 2;

	acio->rx_irqs = devm_kcalloc(acio->dev, acio->n_rings,
				     sizeof(*acio->rx_irqs), GFP_KERNEL);
	if (!acio->rx_irqs)
		return -ENOMEM;
	acio->tx_irqs = devm_kcalloc(acio->dev, acio->n_rings,
				     sizeof(*acio->tx_irqs), GFP_KERNEL);
	if (!acio->tx_irqs)
		return -ENOMEM;

	for (int i = 0; i < acio->n_rings; ++i) {
		snprintf(name, sizeof(name), "rxring%d", i);
		ret = platform_get_irq_byname(acio->pdev, name);
		if (ret == 0) {
			dev_err(acio->dev, "IRQ for %s is zero\n", name);
			return -EINVAL;
		}
		if (ret < 0)
			return dev_err_probe(acio->dev, ret,
					     "Failed to get IRQ %s\n", name);
		acio->rx_irqs[i] = ret;

		snprintf(name, sizeof(name), "txring%d", i);
		ret = platform_get_irq_byname(acio->pdev, name);
		if (ret == 0) {
			dev_err(acio->dev, "IRQ for %s is zero\n", name);
			return -EINVAL;
		}
		if (ret < 0)
			return dev_err_probe(acio->dev, ret,
					     "Failed to get IRQ %s\n", name);
		acio->tx_irqs[i] = ret;
	}

	return 0;
}

static int apple_cio_probe_adapters(struct apple_cio *acio)
{
	struct fwnode_handle *ep;

	fwnode_graph_for_each_endpoint (of_fwnode_handle(acio->np), ep) {
		struct fwnode_endpoint fwnode_ep = { 0 };
		struct fwnode_handle *remote;
		struct tb_protocol_adapter *adapter;
		int ret;

		ret = fwnode_graph_parse_endpoint(ep, &fwnode_ep);
		if (ret < 0)
			continue;

		if (fwnode_ep.port >= MAX_ROOT_SWITCH_PORTS) {
			dev_warn(acio->dev, "port %d > max %d\n",
				 fwnode_ep.port, MAX_ROOT_SWITCH_PORTS);
			continue;
		}

		remote = fwnode_graph_get_remote_port_parent(ep);
		if (!remote)
			continue;

		adapter = devm_tb_protocol_adapter_get(acio->dev, remote);
		fwnode_handle_put(remote);

		if (!adapter)
			continue;
		if (IS_ERR(adapter))
			return PTR_ERR(adapter);

		if (acio->adapters[fwnode_ep.port]) {
			dev_err(acio->dev,
				"duplicate protocol adapter for port %d\n",
				fwnode_ep.port);
			return -EINVAL;
		}

		acio->adapters[fwnode_ep.port] = adapter;
	}

	return 0;
}

static unsigned int apple_cio_ring_index(struct tb_ring *ring)
{
	struct apple_cio *acio = nhi_to_acio(ring->nhi);

	if (ring->is_tx)
		return ring->hop;
	else
		return ring->hop + acio->n_rings;
}

static void apple_cio_ring_interrupt(struct tb_ring *ring)
{
	if (!ring->running)
		return;

	if (ring->start_poll) {
		ring->nhi->ops->ring_interrupt_mask(ring, true);
		ring->start_poll(ring->poll_data);
	} else {
		schedule_work(&ring->work);
	}
}

static irqreturn_t apple_cio_ring_irq(int irq, void *data)
{
	struct tb_ring *ring = data;
	struct apple_cio *acio = nhi_to_acio(ring->nhi);
	unsigned int idx = apple_cio_ring_index(ring);

	spin_lock(&ring->nhi->lock);
	writel(BIT(idx), acio->regs_nhi + APPLE_CIO_NHI_IRQ_STATUS);
	spin_lock(&ring->lock);
	apple_cio_ring_interrupt(ring);
	spin_unlock(&ring->lock);
	spin_unlock(&ring->nhi->lock);

	return IRQ_HANDLED;
}

static void apple_cio_ring_interrupt_active(struct tb_ring *ring, bool active)
{
	struct apple_cio *acio = nhi_to_acio(ring->nhi);
	unsigned int idx = apple_cio_ring_index(ring);
	u32 reg;

	reg = readl(acio->regs_nhi + APPLE_CIO_NHI_IRQ_ENABLE);

	if (active)
		reg |= BIT(idx);
	else
		reg &= ~BIT(idx);

	writel(reg, acio->regs_nhi + APPLE_CIO_NHI_IRQ_ENABLE);
}

static void apple_cio_ring_interrupt_mask(struct tb_ring *ring, bool mask)
{
	apple_cio_ring_interrupt_active(ring, !mask);
}

static int apple_cio_request_irq(struct tb_ring *ring, bool no_suspend)
{
	struct apple_cio *acio = nhi_to_acio(ring->nhi);
	int ret;

	printk("REQUEST IRQ\n");
	printk("REQUEST IRQ\n");
	if (ring->is_tx)
		ring->irq = acio->tx_irqs[ring->hop];
	else
		ring->irq = acio->rx_irqs[ring->hop];

	printk("REQUEST IRQ NOW\n");
	ret = request_irq(ring->irq, apple_cio_ring_irq,
			   no_suspend ? IRQF_NO_SUSPEND : 0, "thunderbolt",
			   ring);
	printk("REQUEST IRQ DONE\n");
	return ret;
}

static void apple_cio_release_irq(struct tb_ring *ring)
{
	if (ring->irq <= 0)
		return;

	free_irq(ring->irq, ring);
	ring->irq = 0;
}

static void __iomem *apple_cio_ring_desc_base(struct tb_ring *ring)
{
	struct apple_cio *acio = nhi_to_acio(ring->nhi);
	void __iomem *io = acio->regs_nhi;

	io += ring->hop * APPLE_CIO_NHI_RING_MMIO_SIZE;

	if (ring->is_tx)
		io += APPLE_CIO_NHI_TXRING_DESC_BASE;
	else
		io += APPLE_CIO_NHI_RXRING_DESC_BASE;

	return io;
}

static void __iomem *apple_cio_ring_options_base(struct tb_ring *ring)
{
	return apple_cio_ring_desc_base(ring) + 0x10;
}

static void apple_cio_oob_notify_tunnel_state(struct tb_tunnel *tunnel,
					      bool active)
{
	struct apple_cio *acio = nhi_to_acio(tunnel->tb->nhi);
	unsigned port = tunnel->src_port->port;

	dev_warn(acio->dev, "OOB tunnel notify for port %d: %d\n", port,
		 active);

	if (!acio->adapters[port])
		return;

	if (active)
		tb_protocol_adapter_activate(acio->adapters[port]);
	else
		tb_protocol_adapter_deactivate(acio->adapters[port]);
}

static const struct tb_nhi_ops apple_nhi_ops = {
	.ring_request_irq = apple_cio_request_irq,
	.ring_release_irq = apple_cio_release_irq,
	.ring_interrupt_active = apple_cio_ring_interrupt_active,
	.ring_interrupt_mask = apple_cio_ring_interrupt_mask,
	.ring_desc_base = apple_cio_ring_desc_base,
	.ring_options_base = apple_cio_ring_options_base,
	.oob_notify_tunnel_state = apple_cio_oob_notify_tunnel_state,
};

static int apple_cio_mux_set(struct typec_mux_dev *mux,
			     struct typec_mux_state *state)
{
	struct apple_cio *acio = typec_mux_get_drvdata(mux);
	u32 cable_info;

	if (state->alt && state->alt->svid == USB_TYPEC_TBT_SID) {
		struct typec_thunderbolt_data *data = state->data;

		cable_info = TB_VSE_CAP_APPLE_CABLE_INFO_PRESENT;
		cable_info |= TB_VSE_CAP_APPLE_CABLE_INFO_TBT2_3;

		//if (data->cable_mode & TBT_CABLE_OPTICAL)
		//	req.mode_data |= TODO;
		if (data->cable_mode & TBT_CABLE_LINK_TRAINING)
			cable_info |= TB_VSE_CAP_APPLE_CABLE_INFO_LINK_TRAINING;
		if (data->enter_vdo & TBT_ENTER_MODE_ACTIVE_CABLE)
			cable_info |= TB_VSE_CAP_APPLE_CABLE_INFO_ACTIVE;
		if (TBT_ADAPTER(data->device_mode) == TBT_ADAPTER_LEGACY)
			cable_info |=
				TB_VSE_CAP_APPLE_CABLE_INFO_LEGACY_ADAPTER;
		if (TBT_CABLE_SPEED(data->cable_mode) ==
		    TBT_CABLE_10_AND_20GBPS)
			cable_info |= TB_VSE_CAP_APPLE_CABLE_INFO_20_GBPS;
	} else if (!state->alt && state->mode == TYPEC_MODE_USB4) {
		struct enter_usb_data *data = state->data;

		cable_info = TB_VSE_CAP_APPLE_CABLE_INFO_PRESENT;
		cable_info |= TB_VSE_CAP_APPLE_CABLE_INFO_TBT2_3;
		cable_info |= TB_VSE_CAP_APPLE_CABLE_INFO_LEGACY_ADAPTER;
		//cable_info |= TB_VSE_CAP_APPLE_CABLE_INFO_ACTIVE;
	} else {
		cable_info = 0;
	}

	if (cable_info && acio->orientation == TYPEC_ORIENTATION_REVERSE)
		cable_info |= TB_VSE_CAP_APPLE_CABLE_INFO_ORIENTATION;

	mutex_lock(&acio->lock);
	acio->target_cable_info = cable_info;
	if (acio->current_cable_info != acio->target_cable_info)
		schedule_work(&acio->mux_set_work);
	mutex_unlock(&acio->lock);

	return 0;
}

static int apple_cio_create_nhi_dev(struct apple_cio *acio)
{
	struct device_node *child_node;
	int ret;

	child_node = of_get_child_by_name(acio->np, "nhi");
	if (!child_node)
		return dev_err_probe(acio->dev, -ENODEV,
				     "Failed to get NHI child DT node\n");

	acio->nhi_pdev = of_platform_device_create(child_node, NULL, acio->dev);
	if (!acio->nhi_pdev)
		return dev_err_probe(acio->dev, -ENODEV,
				     "Failed to create NHI child\n");

	ret = of_dma_configure(&acio->nhi_pdev->dev, child_node, true);
	if (ret) {
		ret = dev_err_probe(acio->dev, ret,
				    "Failed to configure NHI child DMA\n");
		goto err_destroy_pdev;
	}

	return 0;

err_destroy_pdev:
	of_platform_device_destroy(&acio->nhi_pdev->dev, NULL);

	return ret;
}

static int apple_cio_create_fake_iommu_dev(struct apple_cio *acio)
{
	struct device_node *child_node;
	int ret;

	child_node = of_get_child_by_name(acio->np, "dummy");
	if (!child_node)
		return dev_err_probe(acio->dev, -ENODEV,
				     "Failed to get IOMMU child DT node\n");

	acio->iommu_pdev =
		of_platform_device_create(child_node, NULL, acio->dev);
	if (!acio->iommu_pdev)
		return dev_err_probe(acio->dev, -ENODEV,
				     "Failed to create IOMMU child\n");

	ret = of_dma_configure(&acio->iommu_pdev->dev, child_node, true);
	if (ret) {
		ret = dev_err_probe(acio->dev, ret,
				    "Failed to configure IOMMU child DMA\n");
		goto err_destroy_pdev;
	}

	acio->iommu_dom = iommu_domain_alloc(&platform_bus_type);
	if (!acio->iommu_dom) {
		ret = -ENOMEM;
		goto err_destroy_pdev;
	}

	ret = iommu_attach_device(acio->iommu_dom, &acio->iommu_pdev->dev);
	if (ret) {
		ret = dev_err_probe(acio->dev, ret,
				    "Failed to attach IOMMU child domain\n");
		goto err_free_domain;
	}

#if 0
	struct page *lol = alloc_pages(GFP_KERNEL, 1);
	if (!lol)
		return -ENOMEM;

	memset(page_to_virt(lol), 0xff, 0x4000);
#endif

	ret = iommu_map(acio->iommu_dom, 0x100000, 0x100000, 0x4000,
			IOMMU_READ);
	if (ret) {
		ret = dev_err_probe(acio->dev, ret, "Failed to map page\n");
		goto err_detach_dev;
	}

	return 0;

err_detach_dev:
	iommu_detach_device(acio->iommu_dom, &acio->iommu_pdev->dev);
err_free_domain:
	iommu_domain_free(acio->iommu_dom);
err_destroy_pdev:
	of_platform_device_destroy(&acio->iommu_pdev->dev, NULL);

	return ret;
}

static void apple_cio_start(struct apple_cio *acio)
{
	int ret, cap_apple;
	u32 state;

	dev_dbg(acio->dev, "bringing NHI up\n");
	ret = pm_runtime_resume_and_get(acio->dev);
	if (ret) {
		dev_err_probe(acio->dev, ret,
			      "Failed to resume power domains\n");
		return;
	}
	pm_runtime_forbid(acio->dev);

	ret = phy_power_on(acio->phy);
	if (ret)
		return;
	dev_dbg(acio->dev, "PHY is powered\n");

	// Create iommu
	ret = of_platform_populate(acio->np, NULL, NULL, acio->dev);
	if (ret)
		return;
	dev_dbg(acio->dev, "IOMMU is probed\n");

	// Create dev for unknown iommu stream
	ret = apple_cio_create_fake_iommu_dev(acio);
	if (ret)
		return;
	dev_dbg(acio->dev, "dummy IOMMU dev is ready\n");

	// Create main NHI dev
	ret = apple_cio_create_nhi_dev(acio);
	if (ret)
		return;
	dev_dbg(acio->dev, "main NHI dev is ready\n");
	platform_set_drvdata(acio->nhi_pdev, acio);

	writel(0, acio->regs_nhi + APPLE_CIO_NHI_IRQ_ENABLE);
	writel(-1, acio->regs_nhi + APPLE_CIO_NHI_IRQ_STATUS);
	writel(APPLE_CIO_M3_CTRL_START, acio->regs_m3 + APPLE_CIO_M3_CTRL);

	ret = readl_poll_timeout(acio->regs_m3 + APPLE_CIO_M3_STAT, state,
				 state & APPLE_CIO_M3_STAT_STATE, 100, 500000);
	if (ret < 0) {
		dev_err_probe(acio->dev, ret, "M3 firmware failed to boot\n");
		return;
	}
	dev_dbg(acio->dev, "M3 firmware is ready\n");

	acio->rtk = apple_rtkit_init(acio->dev, acio, NULL, 0,
				     &apple_cio_rtkit_ops);
	if (IS_ERR(acio->rtk)) {
		dev_err_probe(acio->dev, PTR_ERR(acio->rtk),
			      "Failed to initialize RTKit");
		return;
	}
	dev_dbg(acio->dev, "M3 rtkit is initialized\n");

	ret = apple_rtkit_boot(acio->rtk);
	if (ret) {
		dev_err_probe(acio->dev, ret, "M3 RTKit failed to boot");
		return;
	}
	dev_dbg(acio->dev, "M3 rtkit has booted\n");

	apple_cio_apply_tunable(&acio->tunable_m3, acio->regs_m3);
	apple_cio_apply_tunable(&acio->tunable_nhi, acio->regs_nhi);
	apple_cio_apply_tunable(&acio->tunable_pcie, acio->regs_pcie);
	dev_dbg(acio->dev, "tunables have been applied\n");

	acio->nhi.hop_count = readl(acio->regs_nhi + APPLE_CIO_NHI_HOP_COUNT) &
			      0x3ff;
	if (acio->nhi.hop_count != acio->n_rings) {
		dev_err_probe(acio->dev, -EINVAL,
			      "Ring IRQs (%zd) != HOP_COUNT (%d)",
			      acio->n_rings, acio->nhi.hop_count);
		return;
	}

	acio->nhi.dev = &acio->nhi_pdev->dev;
	acio->tb = tb_probe(&acio->nhi);
	if (!acio->tb) {
		dev_err_probe(acio->dev, -ENODEV,
			      "failed to init software connection manager\n");
		return;
	}
	dev_dbg(acio->dev, "tb_probe is done\n");

	ret = tb_domain_add(acio->tb);
	if (ret) {
		dev_err_probe(acio->dev, ret, "failed to add TB domain\n");
		return;
	}
	dev_dbg(acio->dev, "tb domain is added\n");

	mutex_lock(&acio->tb->lock);
	cap_apple =
		tb_switch_find_vse_cap(acio->tb->root_switch, TB_VSE_CAP_APPLE);

	if (!cap_apple) {
		dev_err(acio->dev, "blah");
		mutex_unlock(&acio->tb->lock);
		return;
	}

	dev_warn(acio->dev, "VSE Apple offset: %x\n", cap_apple);

	ret = tb_sw_write(acio->tb->root_switch, &acio->target_cable_info,
			  TB_CFG_SWITCH,
			  cap_apple + TB_VSE_CAP_APPLE_CABLE_INFO, 1);
	if (ret)
		dev_warn(acio->dev, "setting VSE Apple cable info failed: %d\n",
			 ret);
	acio->current_cable_info = acio->target_cable_info;

	dev_warn(acio->dev, "setting VSE Apple cable info done: 0x%x\n",
		 acio->target_cable_info);
	mutex_unlock(&acio->tb->lock);

	dev_dbg(acio->dev, "NHI is up\n");
}

static void apple_cio_stop(struct apple_cio *acio)
{
	int ret;

	dev_dbg(acio->dev, "shutting NHI down\n");

	tb_domain_remove(acio->tb);

	ret = apple_rtkit_full_shutdown(acio->rtk);
	if (ret)
		dev_warn(acio->dev, "Failed to shutdown M3 RTKit\n");
	apple_rtkit_free(acio->rtk);
	dev_dbg(acio->dev, "RTKit is freed\n");

	iommu_detach_device(acio->iommu_dom, &acio->iommu_pdev->dev);
	iommu_domain_free(acio->iommu_dom);
	of_platform_device_destroy(&acio->iommu_pdev->dev, NULL);
	dev_dbg(acio->dev, "fake IOMMU dev has been destroyed\n");

	of_platform_device_destroy(&acio->nhi_pdev->dev, NULL);
	dev_dbg(acio->dev, "main NHI dev has been destroyed\n");

	of_platform_depopulate(acio->dev);
	dev_dbg(acio->dev, "IOMMU has been destroyed\n");

	phy_power_off(acio->phy);
	dev_dbg(acio->dev, "PHY is powered off\n");

	pm_runtime_allow(acio->dev);
	pm_runtime_put_sync_suspend(acio->dev);

	dev_dbg(acio->dev, "NHI has been stopped\n");

	acio->current_cable_info = 0;
}

static void apple_cio_mux_set_work(struct work_struct *work)
{
	struct apple_cio *acio =
		container_of(work, struct apple_cio, mux_set_work);

	mutex_lock(&acio->lock);

	if (acio->online) {
		apple_cio_stop(acio);
		acio->online = false;
	}

	if (acio->target_cable_info) {
		apple_cio_start(acio);
		acio->online = true;
	}

	mutex_unlock(&acio->lock);
}

static int apple_cio_sw_set(struct typec_switch_dev *sw,
			    enum typec_orientation orientation)
{
	struct apple_cio *acio = typec_switch_get_drvdata(sw);

	/*
	 * This is a bit of a hack since we should technically also update
	 * the cable info here. We however know that apple_cio_sw_set will
	 * always be called before apple_cio_mux_set since all Apple Silicon
	 * platforms use a TI TPS6598x variant as their USB PD controller.
	 */
	acio->orientation = orientation;
	return 0;
}

static void apple_cio_mux_unregister(void *data)
{
	struct typec_mux_dev *mux_dev = data;

	typec_mux_unregister(mux_dev);
}

static void apple_cio_switch_unregister(void *data)
{
	struct typec_switch_dev *sw_dev = data;

	typec_switch_unregister(sw_dev);
}

static void apple_cio_phy_exit(void *data)
{
	struct apple_cio *acio = data;

	phy_exit(acio->phy);
}

static void apple_cio_detach_genpd(void *data)
{
	struct apple_cio *acio = data;
	int i;

	if (acio->pd_count <= 1)
		return;

	for (i = acio->pd_count - 1; i >= 0; i--) {
		if (acio->pd_link[i])
			device_link_del(acio->pd_link[i]);
		if (!IS_ERR_OR_NULL(acio->pd_dev[i]))
			dev_pm_domain_detach(acio->pd_dev[i], true);
	}
}

static int apple_cio_attach_genpd(struct apple_cio *acio)
{
	struct device *dev = acio->dev;
	int i;

	acio->pd_count = of_count_phandle_with_args(
		dev->of_node, "power-domains", "#power-domain-cells");
	if (acio->pd_count <= 1)
		return 0;

	acio->pd_dev = devm_kcalloc(dev, acio->pd_count, sizeof(*acio->pd_dev),
				   GFP_KERNEL);
	if (!acio->pd_dev)
		return -ENOMEM;

	acio->pd_link = devm_kcalloc(dev, acio->pd_count, sizeof(*acio->pd_link),
				    GFP_KERNEL);
	if (!acio->pd_link)
		return -ENOMEM;

	for (i = 0; i < acio->pd_count; i++) {
		acio->pd_dev[i] = dev_pm_domain_attach_by_id(dev, i);
		if (IS_ERR(acio->pd_dev[i])) {
			apple_cio_detach_genpd(acio);
			return PTR_ERR(acio->pd_dev[i]);
		}

		acio->pd_link[i] = device_link_add(dev, acio->pd_dev[i],
						  DL_FLAG_STATELESS |
						  DL_FLAG_PM_RUNTIME |
						  DL_FLAG_RPM_ACTIVE);
		if (!acio->pd_link[i]) {
			apple_cio_detach_genpd(acio);
			return -EINVAL;
		}
	}

	return 0;
}

static int apple_cio_probe(struct platform_device *pdev)
{
	struct typec_mux_desc mux_desc = {
		.fwnode = pdev->dev.fwnode,
		.set = apple_cio_mux_set,
	};
	struct typec_switch_desc sw_desc = {
		.fwnode = pdev->dev.fwnode,
		.set = apple_cio_sw_set,
	};
	struct device *dev = &pdev->dev;
	struct typec_mux_dev *mux_dev;
	struct typec_switch_dev *sw_dev;
	struct apple_cio *acio;
	int ret;

	acio = devm_kzalloc(dev, sizeof(*acio), GFP_KERNEL);
	if (!acio)
		return -ENOMEM;

	mutex_init(&acio->lock);
	acio->pdev = pdev;
	acio->dev = dev;
	acio->np = dev->of_node;
	platform_set_drvdata(pdev, acio);
	mux_desc.drvdata = acio;
	sw_desc.drvdata = acio;
	INIT_WORK(&acio->mux_set_work, apple_cio_mux_set_work);

	apple_cio_attach_genpd(acio);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	ret = of_property_read_u64(acio->np, "apple,sram-dva-base",
				   &acio->sram_dva_base);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to read apple,sram-dva-base");
	acio->sram = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sram");
	if (!acio->sram)
		return dev_err_probe(dev, -EIO, "Failed to get SRAM resource");
	acio->sram_base = devm_ioremap_resource(dev, acio->sram);
	if (IS_ERR(acio->sram_base))
		return dev_err_probe(dev, PTR_ERR(acio->sram_base),
				     "Failed to map SRAM");

	ret = devm_clk_bulk_get_all(dev, &acio->clks);
	if (ret < 0)
		return ret;
	acio->num_clks = ret;

	ret = clk_bulk_prepare_enable(acio->num_clks, acio->clks);
	if (ret)
		return ret;

	acio->regs_nhi = devm_platform_ioremap_resource_byname(pdev, "nhi");
	if (IS_ERR(acio->regs_nhi))
		return PTR_ERR(acio->regs_nhi);
	acio->regs_m3 = devm_platform_ioremap_resource_byname(pdev, "m3");
	if (IS_ERR(acio->regs_m3))
		return PTR_ERR(acio->regs_m3);
	acio->regs_pcie =
		devm_platform_ioremap_resource_byname(pdev, "pcie-adapter");
	if (IS_ERR(acio->regs_pcie))
		return PTR_ERR(acio->regs_pcie);

	acio->phy = devm_phy_get(dev, "usb4-phy");
	if (IS_ERR(acio->phy))
		return dev_err_probe(acio->dev, PTR_ERR(acio->phy),
				     "Couldn't get USB4 PHY.\n");

	ret = apple_cio_probe_irqs(acio);
	if (ret)
		return ret;
	ret = apple_cio_probe_adapters(acio);
	if (ret)
		return ret;

	ret = apple_cio_parse_tunable(acio, &acio->tunable_nhi,
				      "apple,tunable-nhi");
	if (ret)
		return ret;
	ret = apple_cio_parse_tunable(acio, &acio->tunable_m3,
				      "apple,tunable-m3");
	if (ret)
		return ret;
	ret = apple_cio_parse_tunable(acio, &acio->tunable_pcie,
				      "apple,tunable-pcie-adapter");
	if (ret)
		return ret;

	spin_lock_init(&acio->nhi.lock);
	acio->nhi.iommu_dma_protection = true;
	acio->nhi.ops = &apple_nhi_ops;
	acio->nhi.iobase = acio->regs_nhi;
	acio->nhi.quirks = QUIRK_RING_START_APPLE | QUIRK_NO_DMA_PORT;
	acio->nhi.hop_count = acio->n_rings;
	acio->nhi.tx_rings = devm_kcalloc(dev, acio->nhi.hop_count,
					  sizeof(*acio->nhi.tx_rings),
					  GFP_KERNEL);
	acio->nhi.rx_rings = devm_kcalloc(dev, acio->nhi.hop_count,
					  sizeof(*acio->nhi.rx_rings),
					  GFP_KERNEL);
	if (!acio->nhi.tx_rings || !acio->nhi.rx_rings)
		return -ENOMEM;

	ret = phy_init(acio->phy);
	if (ret)
		return dev_err_probe(acio->dev, ret, "Failed to init PHY.\n");
	ret = devm_add_action_or_reset(dev, apple_cio_phy_exit, acio);
	if (ret)
		return ret;

	sw_dev = typec_switch_register(dev, &sw_desc);
	ret = PTR_ERR_OR_ZERO(sw_dev);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, apple_cio_switch_unregister,
				       sw_dev);
	if (ret)
		return ret;

	mux_dev = typec_mux_register(dev, &mux_desc);
	ret = PTR_ERR_OR_ZERO(mux_dev);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, apple_cio_mux_unregister, mux_dev);
	if (ret)
		return ret;

	return 0;
}

static const struct of_device_id apple_cio_match[] = {
	{
		.compatible = "apple,usb4-nhi",
	},
	{},
};
MODULE_DEVICE_TABLE(of, apple_cio_match);

static struct platform_driver apple_cio_driver = {
	.driver = {
		.name = "thunderbolt-apple-nhi",
		.of_match_table = apple_cio_match,
	},
	.probe = apple_cio_probe,
};

module_platform_driver(apple_cio_driver);
