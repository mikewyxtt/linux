// SPDX-License-Identifier: GPL-2.0

#include <linux/of.h>

bool rust_helper_of_node_is_root(const struct device_node *np)
{
	return of_node_is_root(np);
}
EXPORT_SYMBOL_GPL(rust_helper_of_node_is_root);

struct device_node *rust_helper_of_parse_phandle(const struct device_node *np,
               const char *phandle_name,
               int index)
{
	return of_parse_phandle(np, phandle_name, index);
}
EXPORT_SYMBOL_GPL(rust_helper_of_parse_phandle);
