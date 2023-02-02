/* SPDX-License-Identifier: MIT */
/*
 * Copyright (C) The Asahi Linux Contributors
 *
 * Based on asahi_drm.h which is
 *
 * Copyright © 2014-2018 Broadcom
 * Copyright © 2019 Collabora ltd.
 */
#ifndef _ASAHI_DRM_H_
#define _ASAHI_DRM_H_

#include "drm.h"

#if defined(__cplusplus)
extern "C" {
#endif

#define DRM_ASAHI_UNSTABLE_UABI_VERSION	4

#define DRM_ASAHI_GET_PARAM			0x00
#define DRM_ASAHI_VM_CREATE			0x01
#define DRM_ASAHI_VM_DESTROY			0x02
#define DRM_ASAHI_GEM_CREATE			0x03
#define DRM_ASAHI_GEM_MMAP_OFFSET		0x04
#define DRM_ASAHI_GEM_BIND			0x05
#define DRM_ASAHI_QUEUE_CREATE			0x06
#define DRM_ASAHI_QUEUE_DESTROY			0x07
#define DRM_ASAHI_SUBMIT			0x08

enum drm_asahi_param {
	/* UAPI related */
	DRM_ASAHI_PARAM_UNSTABLE_UABI_VERSION,

	/* GPU identification */
	DRM_ASAHI_PARAM_GPU_GENERATION,
	DRM_ASAHI_PARAM_GPU_VARIANT,
	DRM_ASAHI_PARAM_GPU_REVISION,
	DRM_ASAHI_PARAM_CHIP_ID,

	/* GPU features */
	DRM_ASAHI_PARAM_FEAT_COMPAT,
	DRM_ASAHI_PARAM_FEAT_INCOMPAT,

	/* VM info */
	DRM_ASAHI_PARAM_VM_PAGE_SIZE,
	DRM_ASAHI_PARAM_VM_USER_START,
	DRM_ASAHI_PARAM_VM_USER_END,
	DRM_ASAHI_PARAM_VM_SHADER_START,
	DRM_ASAHI_PARAM_VM_SHADER_END,
};

/*
enum drm_asahi_feat_compat {
};
*/

enum drm_asahi_feat_incompat {
	DRM_ASAHI_FEAT_MANDATORY_ZS_COMPRESSION = (1UL) << 0,
};

struct drm_asahi_get_param {
	/** @param: Parameter ID to fetch */
	__u32 param;

	/** @pad: MBZ */
	__u32 pad;

	/** @value: Returned parameter value */
	__u64 value;
};

struct drm_asahi_vm_create {
	/** @value: Returned VM ID */
	__u32 vm_id;

	/** @pad: MBZ */
	__u32 pad;
};

struct drm_asahi_vm_destroy {
	/** @value: VM ID to be destroyed */
	__u32 vm_id;

	/** @pad: MBZ */
	__u32 pad;
};

#define ASAHI_GEM_WRITEBACK	(1L << 0)

struct drm_asahi_gem_create {
	/** @size: Size of the BO */
	__u64 size;

	/** @flags: BO creation flags */
	__u32 flags;

	/** @handle: Returned GEM handle for the BO. */
	__u32 handle;
};

struct drm_asahi_gem_mmap_offset {
	/** @handle: Handle for the object being mapped. */
	__u32 handle;

	/** @flags: Must be zero */
	__u32 flags;

	/** @offset: The fake offset to use for subsequent mmap call */
	__u64 offset;
};

#define ASAHI_BIND_READ		(1L << 0)
#define ASAHI_BIND_WRITE	(1L << 1)

struct drm_asahi_gem_bind {
	/** @obj: GEM object to bind */
	__u32 handle;

	/** @vm_id: The ID of the VM to bind to */
	__u32 vm_id;

	/** @offset: Offset into the object */
	__u64 offset;

	/** @range: Number of bytes from the object to bind to addr */
	__u64 range;

	/** @addr: Address to bind to */
	__u64 addr;

	/** @flags: One or more of ASAHI_BO_* */
	__u32 flags;
};

enum drm_asahi_queue_type {
	DRM_ASAHI_QUEUE_RENDER = 0,
	DRM_ASAHI_QUEUE_COMPUTE = 1,
};

struct drm_asahi_queue_create {
	/** @vm_id: The ID of the VM this queue is bound to */
	__u32 vm_id;

	/** @type: One of enum drm_asahi_queue_type */
	__u32 queue_type;

	/** @priority: Queue priority, 0-3 */
	__u32 priority;

	/** @flags: MBZ */
	__u32 flags;

	/** @queue_id: The returned queue ID */
	__u32 queue_id;
};

struct drm_asahi_queue_destroy {
	/** @queue_id: The queue ID to be destroyed */
	__u32 queue_id;
};

enum drm_asahi_cmd_type {
	DRM_ASAHI_CMD_RENDER = 0,
	DRM_ASAHI_CMD_BLIT = 1,
	DRM_ASAHI_CMD_COMPUTE = 2,
};

struct drm_asahi_submit {
	/** @queue_id: The queue ID to be submitted to */
	__u32 queue_id;

	/** @type: One of drm_asahi_cmd_type */
	__u32 cmd_type;

	/** @cmdbuf: Pointer to the appropriate command buffer structure */
	__u64 cmd_buffer;

	/** @flags: MBZ */
	__u32 flags;

	/** @in_sync_count: Number of sync objects to wait on before starting this job. */
	__u32 in_sync_count;

	/** @in_syncs: An optional array of sync objects to wait on before starting this job. */
	__u64 in_syncs;

	/** @out_sync: An optional sync object to place the completion fence in. */
	__u32 out_sync;
};

#define ASAHI_MAX_ATTACHMENTS 16

#define ASAHI_ATTACHMENT_C    0
#define ASAHI_ATTACHMENT_Z    1
#define ASAHI_ATTACHMENT_S    2

struct drm_asahi_attachment {
	__u32 type;
	__u32 size;
	__u64 pointer;
};

#define ASAHI_CMDBUF_NO_CLEAR_PIPELINE_TEXTURES (1UL << 0)
#define ASAHI_CMDBUF_SET_WHEN_RELOADING_Z_OR_S (1UL << 1)
#define ASAHI_CMDBUF_MEMORYLESS_RTS_USED (1UL << 2)
#define ASAHI_CMDBUF_PROCESS_EMPTY_TILES (1UL << 3)

struct drm_asahi_cmd_render {
	__u64 flags;

	__u64 encoder_ptr;

	__u64 depth_buffer_1;
	__u64 depth_buffer_2;
	__u64 depth_buffer_3;
	__u64 depth_meta_buffer_1;
	__u64 depth_meta_buffer_2;
	__u64 depth_meta_buffer_3;

	__u64 stencil_buffer_1;
	__u64 stencil_buffer_2;
	__u64 stencil_buffer_3;
	__u64 stencil_meta_buffer_1;
	__u64 stencil_meta_buffer_2;
	__u64 stencil_meta_buffer_3;

	__u64 scissor_array;
	__u64 depth_bias_array;
	__u64 visibility_result_buffer;

	__u64 zls_ctrl;
	__u64 ppp_multisamplectl;
	__u32 ppp_ctrl;

	__u32 fb_width;
	__u32 fb_height;

	__u32 utile_width;
	__u32 utile_height;

	__u32 samples;
	__u32 layers;

	__u32 encoder_id;
	__u32 cmd_ta_id;
	__u32 cmd_3d_id;

	__u32 iogpu_unk_49;
	__u32 iogpu_unk_212;
	__u32 iogpu_unk_214;

	__u32 merge_upper_x;
	__u32 merge_upper_y;

	__u32 load_pipeline;
	__u32 load_pipeline_bind;

	__u32 store_pipeline;
	__u32 store_pipeline_bind;

	__u32 partial_reload_pipeline;
	__u32 partial_reload_pipeline_bind;

	__u32 partial_store_pipeline;
	__u32 partial_store_pipeline_bind;

	__u32 depth_dimensions;
	__u32 isp_bgobjdepth;
	__u32 isp_bgobjvals;

	struct drm_asahi_attachment attachments[ASAHI_MAX_ATTACHMENTS];
	__u32 attachment_count;
};

struct drm_asahi_cmd_compute {
	__u64 flags;

	__u64 encoder_ptr;
	__u64 encoder_end;

	__u64 buffer_descriptor;

	__u32 buffer_descriptor_size; /* ? */
	__u32 ctx_switch_prog;

	__u32 encoder_id;
	__u32 cmd_id;

	__u32 iogpu_unk_40;
	__u32 iogpu_unk_44;
};

/* Note: this is an enum so that it can be resolved by Rust bindgen. */
enum {
   DRM_IOCTL_ASAHI_GET_PARAM        = DRM_IOWR(DRM_COMMAND_BASE + DRM_ASAHI_GET_PARAM, struct drm_asahi_get_param),
   DRM_IOCTL_ASAHI_VM_CREATE        = DRM_IOWR(DRM_COMMAND_BASE + DRM_ASAHI_VM_CREATE, struct drm_asahi_vm_create),
   DRM_IOCTL_ASAHI_VM_DESTROY       = DRM_IOW(DRM_COMMAND_BASE + DRM_ASAHI_VM_DESTROY, struct drm_asahi_vm_destroy),
   DRM_IOCTL_ASAHI_GEM_CREATE       = DRM_IOWR(DRM_COMMAND_BASE + DRM_ASAHI_GEM_CREATE, struct drm_asahi_gem_create),
   DRM_IOCTL_ASAHI_GEM_MMAP_OFFSET  = DRM_IOWR(DRM_COMMAND_BASE + DRM_ASAHI_GEM_MMAP_OFFSET, struct drm_asahi_gem_mmap_offset),
   DRM_IOCTL_ASAHI_GEM_BIND         = DRM_IOW(DRM_COMMAND_BASE + DRM_ASAHI_GEM_BIND, struct drm_asahi_gem_bind),
   DRM_IOCTL_ASAHI_QUEUE_CREATE     = DRM_IOWR(DRM_COMMAND_BASE + DRM_ASAHI_QUEUE_CREATE, struct drm_asahi_queue_create),
   DRM_IOCTL_ASAHI_QUEUE_DESTROY    = DRM_IOW(DRM_COMMAND_BASE + DRM_ASAHI_QUEUE_DESTROY, struct drm_asahi_queue_destroy),
   DRM_IOCTL_ASAHI_SUBMIT           = DRM_IOW(DRM_COMMAND_BASE + DRM_ASAHI_SUBMIT, struct drm_asahi_submit),
};

#if defined(__cplusplus)
}
#endif

#endif /* _ASAHI_DRM_H_ */
