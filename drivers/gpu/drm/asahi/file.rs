// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(clippy::unusual_byte_groupings)]

//! File implementation, which represents a single DRM client.
//!
//! This is in charge of managing the resources associated with one GPU client, including an
//! arbitrary number of submission queues and Vm objects, and reporting hardware/driver
//! information to userspace and accepting submissions.

use crate::debug::*;
use crate::driver::AsahiDevice;
use crate::{alloc, buffer, driver, gem, mmu};
use kernel::drm::gem::BaseObject;
use kernel::io_buffer::IoBufferWriter;
use kernel::prelude::*;
use kernel::sync::{smutex::Mutex, Arc};
use kernel::user_ptr::UserSlicePtr;
use kernel::{bindings, drm, xarray};

const DEBUG_CLASS: DebugFlags = DebugFlags::File;

/// A client instance of an `mmu::Vm` address space.
struct Vm {
    ualloc: Arc<Mutex<alloc::DefaultAllocator>>,
    ualloc_priv: Arc<Mutex<alloc::DefaultAllocator>>,
    vm: mmu::Vm,
    _dummy_obj: gem::ObjectRef,
}

/// Trait implemented by queue implementations (Render and Compute).
pub(crate) trait Queue: Send + Sync {
    fn submit(&self, cmd: &bindings::drm_asahi_command, id: u64) -> Result;
}

/// State associated with a client.
pub(crate) struct File {
    id: u64,
    vms: xarray::XArray<Box<Vm>>,
    queues: xarray::XArray<Arc<Box<dyn Queue>>>,
}

/// Convenience type alias for our DRM `File` type.
pub(crate) type DrmFile = drm::file::File<File>;

/// Start address of the 32-bit USC address space.
const VM_SHADER_START: u64 = 0x11_00000000;
/// End address of the 32-bit USC address space.
const VM_SHADER_END: u64 = 0x11_ffffffff;
/// Start address of the general user mapping region.
const VM_USER_START: u64 = 0x20_00000000;
/// End address of the general user mapping region.
const VM_USER_END: u64 = 0x5f_ffffffff;

/// Start address of the kernel-managed GPU-only mapping region.
const VM_DRV_GPU_START: u64 = 0x60_00000000;
/// End address of the kernel-managed GPU-only mapping region.
const VM_DRV_GPU_END: u64 = 0x60_ffffffff;
/// Start address of the kernel-managed GPU/FW shared mapping region.
const VM_DRV_GPUFW_START: u64 = 0x61_00000000;
/// End address of the kernel-managed GPU/FW shared mapping region.
const VM_DRV_GPUFW_END: u64 = 0x61_ffffffff;
/// Address of a special dummy page?
const VM_UNK_PAGE: u64 = 0x6f_ffff8000;

impl drm::file::DriverFile for File {
    type Driver = driver::AsahiDriver;

    /// Create a new `File` instance for a fresh client.
    fn open(device: &AsahiDevice) -> Result<Box<Self>> {
        debug::update_debug_flags();

        let gpu = &device.data().gpu;
        let id = gpu.ids().file.next();

        mod_dev_dbg!(device, "[File {}]: DRM device opened", id);
        Ok(Box::try_new(Self {
            id,
            vms: xarray::XArray::new(xarray::flags::ALLOC1)?,
            queues: xarray::XArray::new(xarray::flags::ALLOC1)?,
        })?)
    }
}

impl File {
    /// IOCTL: get_param: Get a driver parameter value.
    pub(crate) fn get_params(
        device: &AsahiDevice,
        data: &mut bindings::drm_asahi_get_params,
        file: &DrmFile,
    ) -> Result<u32> {
        mod_dev_dbg!(device, "[File {}]: IOCTL: get_params", file.id);

        let gpu = &device.data().gpu;

        if data.extensions != 0 || data.param_group != 0 || data.pad != 0 {
            return Err(EINVAL);
        }

        let params = bindings::drm_asahi_params_global {
            unstable_uabi_version: bindings::DRM_ASAHI_UNSTABLE_UABI_VERSION,
            pad0: 0,
            feat_compat: gpu.get_cfg().gpu_feat_compat,
            feat_incompat: gpu.get_cfg().gpu_feat_incompat,
            gpu_generation: gpu.get_dyncfg().id.gpu_gen as u32,
            gpu_variant: gpu.get_dyncfg().id.gpu_variant as u32,
            gpu_revision: gpu.get_dyncfg().id.gpu_rev as u32,
            chip_id: gpu.get_cfg().chip_id,
            vm_page_size: mmu::UAT_PGSZ as u32,
            pad1: 0,
            vm_user_start: VM_USER_START,
            vm_user_end: VM_USER_END,
            vm_shader_start: VM_SHADER_START,
            vm_shader_end: VM_SHADER_END,
            max_commands_per_submission: 1024,
            max_pending_commands: 3072,
            max_attachments: 16,
        };

        let size =
            core::mem::size_of::<bindings::drm_asahi_params_global>().min(data.size.try_into()?);

        // SAFETY: We only write to this userptr once, so there are no TOCTOU issues.
        let mut params_writer =
            unsafe { UserSlicePtr::new(data.pointer as usize as *mut _, size).writer() };

        // SAFETY: `size` is at most the sizeof of `params`
        unsafe { params_writer.write_raw(&params as *const _ as *const u8, size)? };

        Ok(0)
    }

    /// IOCTL: vm_create: Create a new `Vm`.
    pub(crate) fn vm_create(
        device: &AsahiDevice,
        data: &mut bindings::drm_asahi_vm_create,
        file: &DrmFile,
    ) -> Result<u32> {
        if data.extensions != 0 {
            return Err(EINVAL);
        }

        let gpu = &device.data().gpu;
        let file_id = file.id;
        let vm = gpu.new_vm(file_id)?;

        let resv = file.vms.reserve()?;
        let id: u32 = resv.index().try_into()?;

        mod_dev_dbg!(device, "[File {} VM {}]: VM Create", file_id, id);
        mod_dev_dbg!(device, "[File {} VM {}]: Creating allocators", file_id, id);
        let ualloc = Arc::try_new(Mutex::new(alloc::DefaultAllocator::new(
            device,
            &vm,
            VM_DRV_GPU_START,
            VM_DRV_GPU_END,
            buffer::PAGE_SIZE,
            mmu::PROT_GPU_SHARED_RW,
            512 * 1024,
            true,
            fmt!("File {} VM {} GPU Shared", file_id, id),
            false,
        )?))?;
        let ualloc_priv = Arc::try_new(Mutex::new(alloc::DefaultAllocator::new(
            device,
            &vm,
            VM_DRV_GPUFW_START,
            VM_DRV_GPUFW_END,
            buffer::PAGE_SIZE,
            mmu::PROT_GPU_FW_PRIV_RW,
            64 * 1024,
            true,
            fmt!("File {} VM {} GPU FW Private", file_id, id),
            false,
        )?))?;

        mod_dev_dbg!(
            device,
            "[File {} VM {}]: Creating dummy object",
            file_id,
            id
        );
        let mut dummy_obj = gem::new_kernel_object(device, 0x4000)?;
        dummy_obj.vmap()?.as_mut_slice().fill(0);
        dummy_obj.map_at(&vm, VM_UNK_PAGE, mmu::PROT_GPU_SHARED_RW, true)?;

        mod_dev_dbg!(device, "[File {} VM {}]: VM created", file_id, id);
        resv.store(Box::try_new(Vm {
            ualloc,
            ualloc_priv,
            vm,
            _dummy_obj: dummy_obj,
        })?)?;

        data.vm_id = id;

        Ok(0)
    }

    /// IOCTL: vm_destroy: Destroy a `Vm`.
    pub(crate) fn vm_destroy(
        _device: &AsahiDevice,
        data: &mut bindings::drm_asahi_vm_destroy,
        file: &DrmFile,
    ) -> Result<u32> {
        if data.extensions != 0 {
            return Err(EINVAL);
        }

        if file.vms.remove(data.vm_id as usize).is_none() {
            Err(ENOENT)
        } else {
            Ok(0)
        }
    }

    /// IOCTL: gem_create: Create a new GEM object.
    pub(crate) fn gem_create(
        device: &AsahiDevice,
        data: &mut bindings::drm_asahi_gem_create,
        file: &DrmFile,
    ) -> Result<u32> {
        mod_dev_dbg!(
            device,
            "[File {}]: IOCTL: gem_create size={:#x?}",
            file.id,
            data.size
        );

        if data.extensions != 0 {
            return Err(EINVAL);
        }

        if (data.flags & !bindings::ASAHI_GEM_WRITEBACK) != 0 {
            return Err(EINVAL);
        }

        let bo = gem::new_object(device, data.size.try_into()?, data.flags)?;

        let handle = bo.gem.create_handle(file)?;
        data.handle = handle;

        mod_dev_dbg!(
            device,
            "[File {}]: IOCTL: gem_create size={:#x} handle={:#x?}",
            file.id,
            data.size,
            data.handle
        );

        Ok(0)
    }

    /// IOCTL: gem_mmap_offset: Assign an mmap offset to a GEM object.
    pub(crate) fn gem_mmap_offset(
        device: &AsahiDevice,
        data: &mut bindings::drm_asahi_gem_mmap_offset,
        file: &DrmFile,
    ) -> Result<u32> {
        mod_dev_dbg!(
            device,
            "[File {}]: IOCTL: gem_mmap_offset handle={:#x?}",
            file.id,
            data.handle
        );

        if data.extensions != 0 {
            return Err(EINVAL);
        }

        if data.flags != 0 {
            return Err(EINVAL);
        }

        let bo = gem::lookup_handle(file, data.handle)?;
        data.offset = bo.gem.create_mmap_offset()?;
        Ok(0)
    }

    /// IOCTL: gem_bind: Map a GEM object into a Vm.
    pub(crate) fn gem_bind(
        device: &AsahiDevice,
        data: &mut bindings::drm_asahi_gem_bind,
        file: &DrmFile,
    ) -> Result<u32> {
        mod_dev_dbg!(
            device,
            "[File {} VM {}]: IOCTL: gem_bind handle={:#x?} flags={:#x?} {:#x?}:{:#x?} -> {:#x?}",
            file.id,
            data.vm_id,
            data.handle,
            data.flags,
            data.offset,
            data.range,
            data.addr
        );

        if data.extensions != 0 {
            return Err(EINVAL);
        }

        if data.offset != 0 {
            return Err(EINVAL); // Not supported yet
        }

        if (data.addr | data.range) as usize & mmu::UAT_PGMSK != 0 {
            return Err(EINVAL); // Must be page aligned
        }

        if (data.flags & !(bindings::ASAHI_BIND_READ | bindings::ASAHI_BIND_WRITE)) != 0 {
            return Err(EINVAL);
        }

        let mut bo = gem::lookup_handle(file, data.handle)?;

        if data.range != bo.size().try_into()? {
            return Err(EINVAL); // Not supported yet
        }

        let start = data.addr;
        let end = data.addr + data.range - 1;

        if (VM_SHADER_START..=VM_SHADER_END).contains(&start) {
            if !(VM_SHADER_START..=VM_SHADER_END).contains(&end) {
                return Err(EINVAL); // Invalid map range
            }
        } else if (VM_USER_START..=VM_USER_END).contains(&start) {
            if !(VM_USER_START..=VM_USER_END).contains(&end) {
                return Err(EINVAL); // Invalid map range
            }
        } else {
            return Err(EINVAL); // Invalid map range
        }

        // Just in case
        if end >= VM_DRV_GPU_START {
            return Err(EINVAL);
        }

        let prot = if data.flags & bindings::ASAHI_BIND_READ != 0 {
            if data.flags & bindings::ASAHI_BIND_WRITE != 0 {
                mmu::PROT_GPU_SHARED_RW
            } else {
                mmu::PROT_GPU_SHARED_RO
            }
        } else if data.flags & bindings::ASAHI_BIND_WRITE != 0 {
            mmu::PROT_GPU_SHARED_WO
        } else {
            return Err(EINVAL); // Must specify one of ASAHI_BIND_{READ,WRITE}
        };

        // Clone it immediately so we aren't holding the XArray lock
        let vm = file
            .vms
            .get(data.vm_id.try_into()?)
            .ok_or(ENOENT)?
            .vm
            .clone();

        bo.map_at(&vm, start, prot, true)?;

        Ok(0)
    }

    /// IOCTL: queue_create: Create a new command submission queue of a given type.
    pub(crate) fn queue_create(
        device: &AsahiDevice,
        data: &mut bindings::drm_asahi_queue_create,
        file: &DrmFile,
    ) -> Result<u32> {
        let file_id = file.id;

        mod_dev_dbg!(
            device,
            "[File {} VM {}]: Creating queue caps={:?} prio={:?} flags={:#x?}",
            file_id,
            data.vm_id,
            data.queue_caps,
            data.priority,
            data.flags,
        );

        if data.extensions != 0 {
            return Err(EINVAL);
        }

        if data.flags != 0 {
            return Err(EINVAL);
        }

        if data.queue_caps == 0
            || (data.queue_caps
                & !(bindings::drm_asahi_queue_cap_DRM_ASAHI_QUEUE_CAP_RENDER
                    | bindings::drm_asahi_queue_cap_DRM_ASAHI_QUEUE_CAP_BLIT
                    | bindings::drm_asahi_queue_cap_DRM_ASAHI_QUEUE_CAP_COMPUTE))
                != 0
        {
            return Err(EINVAL);
        }

        if data.priority > 3 {
            return Err(EINVAL);
        }

        let resv = file.queues.reserve()?;
        let file_vm = file.vms.get(data.vm_id.try_into()?).ok_or(ENOENT)?;
        let vm = file_vm.vm.clone();
        let ualloc = file_vm.ualloc.clone();
        let ualloc_priv = file_vm.ualloc_priv.clone();
        // Drop the vms lock eagerly
        core::mem::drop(file_vm);

        todo!();

        /*
        let queue = match data.queue_type {
            bindings::drm_asahi_queue_type_DRM_ASAHI_QUEUE_RENDER => device
                .data()
                .gpu
                .new_render_queue(vm, ualloc, ualloc_priv, data.priority)?,
            bindings::drm_asahi_queue_type_DRM_ASAHI_QUEUE_COMPUTE => device
                .data()
                .gpu
                .new_compute_queue(vm, ualloc, data.priority)?,
            _ => return Err(EINVAL),
        };

        data.queue_id = resv.index().try_into()?;
        resv.store(Arc::try_new(queue)?)?;

        Ok(0)
        */
    }

    /// IOCTL: queue_destroy: Destroy a command submission queue.
    pub(crate) fn queue_destroy(
        _device: &AsahiDevice,
        data: &mut bindings::drm_asahi_queue_destroy,
        file: &DrmFile,
    ) -> Result<u32> {
        if data.extensions != 0 {
            return Err(EINVAL);
        }

        if file.queues.remove(data.queue_id as usize).is_none() {
            Err(ENOENT)
        } else {
            Ok(0)
        }
    }

    /// IOCTL: submit: Submit GPU work to a command submission queue.
    pub(crate) fn submit(
        device: &AsahiDevice,
        data: &mut bindings::drm_asahi_submit,
        file: &DrmFile,
    ) -> Result<u32> {
        if data.extensions != 0 {
            return Err(EINVAL);
        }

        debug::update_debug_flags();

        let gpu = &device.data().gpu;
        gpu.update_globals();

        todo!();

        /* Upgrade to Arc<T> to drop the XArray lock early */

        /*
        let queue: Arc<Box<dyn Queue>> = file
            .queues
            .get(data.queue_id.try_into()?)
            .ok_or(ENOENT)?
            .borrow()
            .into();

        let id = gpu.ids().submission.next();
        mod_dev_dbg!(
            device,
            "[File {} Queue {}]: IOCTL: submit (submission ID: {})",
            file.id,
            data.queue_id,
            id
        );
        let ret = queue.submit(data, id);
        if let Err(e) = ret {
            dev_info!(
                device,
                "[File {} Queue {}]: IOCTL: submit failed! (submission ID: {} err: {:?})",
                file.id,
                data.queue_id,
                id,
                e
            );
            Err(e)
        } else {
            Ok(0)
        }
        */
    }

    /// Returns the unique file ID for this `File`.
    pub(crate) fn file_id(&self) -> u64 {
        self.id
    }
}

impl Drop for File {
    fn drop(&mut self) {
        mod_pr_debug!("[File {}]: Closing...", self.id);
    }
}
