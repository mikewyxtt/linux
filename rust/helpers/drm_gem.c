// SPDX-License-Identifier: GPL-2.0

#include <linux/export.h>
#include <linux/drm_gem.h>

#ifdef CONFIG_DRM

void rust_helper_drm_gem_object_get(struct drm_gem_object *obj)
{
	drm_gem_object_get(obj);
}
EXPORT_SYMBOL_GPL(rust_helper_drm_gem_object_get);

void rust_helper_drm_gem_object_put(struct drm_gem_object *obj)
{
	drm_gem_object_put(obj);
}
EXPORT_SYMBOL_GPL(rust_helper_drm_gem_object_put);

__u64 rust_helper_drm_vma_node_offset_addr(struct drm_vma_offset_node *node)
{
	return drm_vma_node_offset_addr(node);
}
EXPORT_SYMBOL_GPL(rust_helper_drm_vma_node_offset_addr);

#endif
