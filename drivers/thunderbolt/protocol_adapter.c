// SPDX-License-Identifier: GPL-2.0-only
/*
 * Thunderbolt driver - Out-of-band Protocol Adapter notification support
 *
 * Copyright (C) The Asahi Linux Contributors 
 * Author: Sven Peter <sven@svenpeter.dev>
 * 
 * based on USB Role Switch Support (drivers/usb/roles/class.c)
 * Copyright (C) 2018 Intel Corporation
 * Author: Heikki Krogerus <heikki.krogerus@linux.intel.com>
 *         Hans de Goede <hdegoede@redhat.com>
 */

#include <linux/mutex.h>
#include <linux/thunderbolt.h>
#include <linux/property.h>

static struct class *tb_protocol_adapter_class;

struct tb_protocol_adapter {
	struct device dev;
	struct mutex lock;

	tb_protocol_adapter_activate_t activate;
	tb_protocol_adapter_deactivate_t deactivate;
};

#define to_tb_protocol_adapter(d) \
	container_of(d, struct tb_protocol_adapter, dev)

static void tb_protocol_adapter_switch_release(struct device *dev)
{
	struct tb_protocol_adapter *adapter = to_tb_protocol_adapter(dev);

	kfree(adapter);
}

static const struct device_type tb_protocol_adapter_dev_type = {
	.name = "tb_protocol_adapter",
	.release = tb_protocol_adapter_switch_release,
};

struct tb_protocol_adapter *
tb_protocol_adapter_register(struct device *parent,
			     struct tb_protocol_adapter_desc *desc)
{
	struct tb_protocol_adapter *adapter;
	int ret;

	if (!desc->fwnode)
		return ERR_PTR(-EINVAL);

	adapter = kzalloc(sizeof(*adapter), GFP_KERNEL);
	if (!adapter)
		return ERR_PTR(-ENOMEM);

	mutex_init(&adapter->lock);

	adapter->activate = desc->activate;
	adapter->deactivate = desc->deactivate;
	adapter->dev.parent = parent;
	adapter->dev.fwnode = desc->fwnode;
	adapter->dev.class = tb_protocol_adapter_class;
	adapter->dev.type = &tb_protocol_adapter_dev_type;
	dev_set_drvdata(&adapter->dev, desc->driver_data);
	dev_set_name(&adapter->dev, "%s-tbt-protocol-adapter",
		     dev_name(parent));

	ret = device_register(&adapter->dev);
	if (ret) {
		put_device(&adapter->dev);
		return ERR_PTR(ret);
	}

	return adapter;
}
EXPORT_SYMBOL_GPL(tb_protocol_adapter_register);

void tb_protocol_adapter_unregister(struct tb_protocol_adapter *adapter)
{
	device_unregister(&adapter->dev);
}
EXPORT_SYMBOL_GPL(tb_protocol_adapter_unregister);

struct tb_protocol_adapter *
tb_protocol_adapter_get(struct fwnode_handle *fwnode)
{
	struct device *dev;
	struct tb_protocol_adapter *adapter;

	if (!fwnode_property_present(fwnode, "thunderbolt-protocol-adapter"))
		return NULL;

	dev = class_find_device_by_fwnode(tb_protocol_adapter_class, fwnode);
	if (!dev)
		return ERR_PTR(-EPROBE_DEFER);

	adapter = to_tb_protocol_adapter(dev);
	WARN_ON(!try_module_get(adapter->dev.parent->driver->owner));
	return adapter;
}
EXPORT_SYMBOL_GPL(tb_protocol_adapter_get);

static void tb_protocol_adapter_put_devm_action(void *adapter)
{
	tb_protocol_adapter_put(adapter);
}

struct tb_protocol_adapter *
devm_tb_protocol_adapter_get(struct device *dev, struct fwnode_handle *fwnode)
{
	struct tb_protocol_adapter *adapter;
	int ret;

	adapter = tb_protocol_adapter_get(fwnode);
	if (IS_ERR(adapter))
		return adapter;

	ret = devm_add_action_or_reset(dev, tb_protocol_adapter_put_devm_action,
				       adapter);
	if (ret)
		return ERR_PTR(ret);
	return adapter;
}
EXPORT_SYMBOL_GPL(devm_tb_protocol_adapter_get);

void tb_protocol_adapter_put(struct tb_protocol_adapter *adapter)
{
	module_put(adapter->dev.parent->driver->owner);
	put_device(&adapter->dev);
}
EXPORT_SYMBOL_GPL(tb_protocol_adapter_put);

void tb_protocol_adapter_set_drvdata(struct tb_protocol_adapter *adapter,
				     void *data)
{
	dev_set_drvdata(&adapter->dev, data);
}
EXPORT_SYMBOL_GPL(tb_protocol_adapter_set_drvdata);

void *tb_protocol_adapter_get_drvdata(struct tb_protocol_adapter *adapter)
{
	return dev_get_drvdata(&adapter->dev);
}
EXPORT_SYMBOL_GPL(tb_protocol_adapter_get_drvdata);

int tb_protocol_adapter_activate(struct tb_protocol_adapter *adapter)
{
	int ret = 0;

	mutex_lock(&adapter->lock);
	if (adapter->activate)
		ret = adapter->activate(adapter);
	mutex_unlock(&adapter->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(tb_protocol_adapter_activate);

void tb_protocol_adapter_deactivate(struct tb_protocol_adapter *adapter)
{
	mutex_lock(&adapter->lock);
	if (adapter->deactivate)
		adapter->deactivate(adapter);
	mutex_unlock(&adapter->lock);
}
EXPORT_SYMBOL_GPL(tb_protocol_adapter_deactivate);

static int __init tb_protocol_adapter_init(void)
{
	tb_protocol_adapter_class =
		class_create(THIS_MODULE, "thunderbolt_protocol_adapter");
	return PTR_ERR_OR_ZERO(tb_protocol_adapter_class);
}
subsys_initcall(tb_protocol_adapter_init);

static void __exit tb_protocol_adapter_exit(void)
{
	class_destroy(tb_protocol_adapter_class);
}
module_exit(tb_protocol_adapter_exit);
