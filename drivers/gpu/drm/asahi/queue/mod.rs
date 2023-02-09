// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(unused_imports)]

//! Submission queue management
//!
//! This module implements the userspace view of submission queues and the logic to map userspace
//! submissions to firmware queues.

use kernel::dma_fence::*;
use kernel::io_buffer::IoBufferReader;
use kernel::prelude::*;
use kernel::user_ptr::UserSlicePtr;
use kernel::{
    bindings, c_str, dma_fence,
    drm::sched,
    macros::versions,
    prelude::*,
    sync::{smutex::Mutex, Arc, Guard, UniqueArc},
};

use crate::alloc::Allocator;
use crate::debug::*;
use crate::driver::AsahiDevice;
use crate::fw::types::*;
use crate::gpu::GpuManager;
use crate::util::*;
use crate::{alloc, buffer, channel, event, file, fw, gem, gpu, microseq, mmu, object, workqueue};
use crate::{box_in_place, inner_ptr, inner_weak_ptr, place};

use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicU64, Ordering};

const DEBUG_CLASS: DebugFlags = DebugFlags::Queue;

const WQ_SIZE: u32 = file::MAX_COMMANDS_IN_FLIGHT + 128;

mod common;
mod compute;
mod render;

/// Trait implemented by all versioned queues.
pub(crate) trait Queue: Send + Sync {
    fn submit(
        &mut self,
        id: u64,
        in_syncs: Vec<file::SyncItem>,
        out_syncs: Vec<file::SyncItem>,
        result_buf: Option<gem::ObjectRef>,
        commands: Vec<bindings::drm_asahi_command>,
    ) -> Result;
}

#[versions(AGX)]
struct SubQueue {
    wq: Arc<workqueue::WorkQueue::ver>,
    cmd_count: u64,
}

#[versions(AGX)]
impl SubQueue::ver {
    fn new_job(&mut self) -> SubQueueJob::ver {
        SubQueueJob::ver {
            wq: self.wq.clone(),
            job: None,
        }
    }
}

#[versions(AGX)]
struct SubQueueJob {
    wq: Arc<workqueue::WorkQueue::ver>,
    job: Option<workqueue::Job::ver>,
}

#[versions(AGX)]
impl SubQueueJob::ver {
    fn get(&mut self) -> Result<&mut workqueue::Job::ver> {
        if self.job.is_none() {
            self.job.replace(self.wq.new_job()?);
        }
        Ok(self.job.as_mut().expect("expected a Job"))
    }

    fn can_submit(&self) -> bool {
        match self.job.as_ref() {
            None => true,
            Some(job) => job.can_submit(),
        }
    }
}

#[versions(AGX)]
pub(crate) struct Queue {
    dev: AsahiDevice,
    sched: sched::Scheduler<QueueJob::ver>,
    entity: sched::Entity<QueueJob::ver>,
    vm: mmu::Vm,
    ualloc: Arc<Mutex<alloc::DefaultAllocator>>,
    q_vtx: Option<SubQueue::ver>,
    q_frag: Option<SubQueue::ver>,
    q_comp: Option<SubQueue::ver>,
    buffer: Option<Mutex<buffer::Buffer::ver>>,
    gpu_context: Arc<GpuObject<fw::workqueue::GpuContextData>>,
    notifier_list: Arc<GpuObject<fw::event::NotifierList>>,
    id: u64,
    fence_ctx: FenceContexts,
    #[ver(V >= V13_0B4)]
    counter: u64,
}

#[versions(AGX)]
#[derive(Default)]
pub(crate) struct JobFence {
    pending: AtomicU64,
}

#[versions(AGX)]
impl JobFence::ver {
    fn add_command(self: &FenceObject<Self>) {
        self.pending.fetch_add(1, Ordering::Relaxed);
    }

    fn command_complete(self: &FenceObject<Self>) {
        if self.pending.fetch_sub(1, Ordering::Relaxed) == 1 && self.signal().is_err() {
            pr_err!("Fence signal failed\n");
        }
    }
}

#[versions(AGX)]
#[vtable]
impl dma_fence::FenceOps for JobFence::ver {
    const USE_64BIT_SEQNO: bool = true;

    fn get_driver_name<'a>(self: &'a FenceObject<Self>) -> &'a CStr {
        c_str!("asahi")
    }
    fn get_timeline_name<'a>(self: &'a FenceObject<Self>) -> &'a CStr {
        c_str!("queue")
    }
}

#[versions(AGX)]
pub(crate) struct QueueJob {
    dev: AsahiDevice,
    vm_bind: mmu::VmBind,
    sj_vtx: Option<SubQueueJob::ver>,
    sj_frag: Option<SubQueueJob::ver>,
    sj_comp: Option<SubQueueJob::ver>,
    fence: UserFence<JobFence::ver>,
    notifier: Arc<GpuObject<fw::event::Notifier::ver>>,
}

#[versions(AGX)]
impl QueueJob::ver {
    fn get_vtx(&mut self) -> Result<&mut workqueue::Job::ver> {
        self.sj_vtx.as_mut().ok_or(EINVAL)?.get()
    }
    fn get_frag(&mut self) -> Result<&mut workqueue::Job::ver> {
        self.sj_frag.as_mut().ok_or(EINVAL)?.get()
    }
    fn get_comp(&mut self) -> Result<&mut workqueue::Job::ver> {
        self.sj_comp.as_mut().ok_or(EINVAL)?.get()
    }
}

#[versions(AGX)]
impl sched::JobImpl for QueueJob::ver {
    fn can_run(job: &mut sched::Job<Self>) -> bool {
        if let Some(sj) = job.sj_vtx.as_ref() {
            if !sj.can_submit() {
                mod_dev_dbg!(job.dev, "QueueJob: Blocking due to vertex queue full\n");
                return false;
            }
        }
        if let Some(sj) = job.sj_frag.as_ref() {
            if !sj.can_submit() {
                mod_dev_dbg!(job.dev, "QueueJob: Blocking due to fragment queue full\n");
                return false;
            }
        }
        if let Some(sj) = job.sj_comp.as_ref() {
            if !sj.can_submit() {
                mod_dev_dbg!(job.dev, "QueueJob: Blocking due to compute queue full\n");
                return false;
            }
        }
        true
    }

    #[allow(unused_assignments)]
    fn run(job: &mut sched::Job<Self>) -> Result<Option<dma_fence::Fence>> {
        mod_dev_dbg!(job.dev, "QueueJob: Running Job\n");

        // First submit all the commands for each queue. This can fail.

        let mut vtx_job = None;
        let mut vtx_sub = None;
        if let Some(sj) = job.sj_vtx.as_mut() {
            vtx_job = sj.job.take();
            if let Some(job) = vtx_job.as_mut() {
                vtx_sub = Some(job.submit()?);
            }
        }

        let mut frag_job = None;
        let mut frag_sub = None;
        if let Some(sj) = job.sj_frag.as_mut() {
            frag_job = sj.job.take();
            if let Some(job) = frag_job.as_mut() {
                frag_sub = Some(job.submit()?);
            }
        }

        let mut comp_job = None;
        let mut comp_sub = None;
        if let Some(sj) = job.sj_comp.as_mut() {
            comp_job = sj.job.take();
            if let Some(job) = comp_job.as_mut() {
                comp_sub = Some(job.submit()?);
            }
        }

        let dev = job.dev.data();
        let gpu = match dev.gpu.as_any().downcast_ref::<gpu::GpuManager::ver>() {
            Some(gpu) => gpu,
            None => panic!("GpuManager mismatched with JobImpl!"),
        };

        // Now we fully commit to running the job
        vtx_sub.map(|a| gpu.run_job(a)).transpose()?;
        frag_sub.map(|a| gpu.run_job(a)).transpose()?;
        comp_sub.map(|a| gpu.run_job(a)).transpose()?;

        Ok(Some(Fence::from_fence(&job.fence)))
    }

    fn timed_out(job: &mut sched::Job<Self>) -> sched::Status {
        // FIXME: Handle timeouts properly
        dev_err!(
            job.dev,
            "Job timed out on the DRM scheduler, things will probably break\n"
        );
        sched::Status::NoDevice
    }
}

static QUEUE_NAME: &CStr = c_str!("asahi_fence");
static QUEUE_CLASS_KEY: kernel::sync::LockClassKey = kernel::sync::LockClassKey::new();

#[versions(AGX)]
impl Queue::ver {
    /// Create a new user queue.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new(
        dev: &AsahiDevice,
        vm: mmu::Vm,
        alloc: &mut gpu::KernelAllocators,
        ualloc: Arc<Mutex<alloc::DefaultAllocator>>,
        ualloc_priv: Arc<Mutex<alloc::DefaultAllocator>>,
        event_manager: Arc<event::EventManager>,
        mgr: &buffer::BufferManager,
        id: u64,
        priority: u32,
        caps: u32,
    ) -> Result<Queue::ver> {
        mod_dev_dbg!(dev, "[Queue {}] Creating renderer\n", id);

        let data = dev.data();

        let gpu_context: Arc<GpuObject<fw::workqueue::GpuContextData>> = Arc::try_new(
            alloc
                .shared
                .new_object(Default::default(), |_inner| Default::default())?,
        )?;

        let mut notifier_list = alloc.private.new_default::<fw::event::NotifierList>()?;

        let self_ptr = notifier_list.weak_pointer();
        notifier_list.with_mut(|raw, _inner| {
            raw.list_head.next = Some(inner_weak_ptr!(self_ptr, list_head));
        });

        let sched = sched::Scheduler::new(dev, WQ_SIZE, 0, 100000, c_str!("asahi_sched"))?;
        // Priorities are handled by the AGX scheduler, there is no meaning within a
        // per-queue scheduler.
        let entity = sched::Entity::new(&sched, sched::Priority::Normal)?;

        let mut ret = Queue::ver {
            dev: dev.clone(),
            sched,
            entity,
            vm,
            ualloc,
            q_vtx: None,
            q_frag: None,
            q_comp: None,
            buffer: None,
            gpu_context,
            notifier_list: Arc::try_new(notifier_list)?,
            id,
            fence_ctx: FenceContexts::new(1, QUEUE_NAME, &QUEUE_CLASS_KEY)?,
            #[ver(V >= V13_0B4)]
            counter: AtomicU64::new(0),
        };

        // Rendering structures
        if caps & bindings::drm_asahi_queue_cap_DRM_ASAHI_QUEUE_CAP_RENDER != 0 {
            let buffer =
                buffer::Buffer::ver::new(&*data.gpu, alloc, ret.ualloc.clone(), ualloc_priv, mgr)?;
            let tvb_blocks = {
                let lock = crate::THIS_MODULE.kernel_param_lock();
                *crate::initial_tvb_size.read(&lock)
            };

            buffer.ensure_blocks(tvb_blocks)?;

            ret.buffer = Some(Mutex::new(buffer));
            ret.q_vtx = Some(SubQueue::ver {
                wq: workqueue::WorkQueue::ver::new(
                    alloc,
                    event_manager.clone(),
                    ret.gpu_context.clone(),
                    ret.notifier_list.clone(),
                    channel::PipeType::Vertex,
                    id,
                    priority,
                    WQ_SIZE,
                )?,
                cmd_count: 0,
            });
        }

        // Rendering & blit structures
        if caps
            & (bindings::drm_asahi_queue_cap_DRM_ASAHI_QUEUE_CAP_RENDER
                | bindings::drm_asahi_queue_cap_DRM_ASAHI_QUEUE_CAP_BLIT)
            != 0
        {
            ret.q_frag = Some(SubQueue::ver {
                wq: workqueue::WorkQueue::ver::new(
                    alloc,
                    event_manager.clone(),
                    ret.gpu_context.clone(),
                    ret.notifier_list.clone(),
                    channel::PipeType::Fragment,
                    id,
                    priority,
                    WQ_SIZE,
                )?,
                cmd_count: 0,
            });
        }

        // Compute structures
        if caps & bindings::drm_asahi_queue_cap_DRM_ASAHI_QUEUE_CAP_COMPUTE != 0 {
            ret.q_comp = Some(SubQueue::ver {
                wq: workqueue::WorkQueue::ver::new(
                    alloc,
                    event_manager,
                    ret.gpu_context.clone(),
                    ret.notifier_list.clone(),
                    channel::PipeType::Compute,
                    id,
                    priority,
                    WQ_SIZE,
                )?,
                cmd_count: 0,
            });
        }

        mod_dev_dbg!(dev, "[Queue {}] Queue created\n", id);
        Ok(ret)
    }
}

#[versions(AGX)]
impl Queue for Queue::ver {
    fn submit(
        &mut self,
        id: u64,
        in_syncs: Vec<file::SyncItem>,
        out_syncs: Vec<file::SyncItem>,
        result_buf: Option<gem::ObjectRef>,
        commands: Vec<bindings::drm_asahi_command>,
    ) -> Result {
        let dev = self.dev.data();
        let gpu = match dev.gpu.as_any().downcast_ref::<gpu::GpuManager::ver>() {
            Some(gpu) => gpu,
            None => panic!("GpuManager mismatched with Queue!"),
        };

        let vm_bind = gpu.bind_vm(&self.vm)?;

        let mut alloc = gpu.alloc();

        let threshold = alloc.shared.new_default::<fw::event::Threshold>()?;

        let notifier: Arc<GpuObject<fw::event::Notifier::ver>> =
            Arc::try_new(alloc.private.new_inplace(
                fw::event::Notifier::ver { threshold },
                |inner, ptr: &mut MaybeUninit<fw::event::raw::Notifier::ver<'_>>| {
                    Ok(place!(
                        ptr,
                        fw::event::raw::Notifier::ver {
                            threshold: inner.threshold.gpu_pointer(),
                            generation: AtomicU32::new(id as u32),
                            cur_count: AtomicU32::new(0),
                            unk_10: AtomicU32::new(0x50),
                            state: Default::default()
                        }
                    ))
                },
            )?)?;

        core::mem::drop(alloc);

        let mut job = self.entity.new_job(QueueJob::ver {
            dev: self.dev.clone(),
            vm_bind,
            sj_vtx: self.q_vtx.as_mut().map(|a| a.new_job()),
            sj_frag: self.q_frag.as_mut().map(|a| a.new_job()),
            sj_comp: self.q_comp.as_mut().map(|a| a.new_job()),
            fence: self
                .fence_ctx
                .new_fence::<JobFence::ver>(0, Default::default())?
                .into(),
            notifier,
        })?;

        for sync in in_syncs {
            job.add_dependency(sync.fence.expect("in_sync missing fence"))?;
        }

        for cmd in commands {
            match cmd.cmd_type {
                bindings::drm_asahi_cmd_type_DRM_ASAHI_CMD_RENDER => {
                    self.submit_render(&mut job, &cmd, id)?
                }
                bindings::drm_asahi_cmd_type_DRM_ASAHI_CMD_COMPUTE => {
                    self.submit_compute(&mut job, &cmd, id)?
                }
                _ => return Err(EINVAL),
            }
        }

        let job = job.arm();
        let out_fence = job.fences().finished();
        job.push();

        for sync in out_syncs {
            sync.syncobj.replace_fence(Some(&out_fence));
        }

        todo!();
    }
}

#[versions(AGX)]
impl Drop for Queue::ver {
    fn drop(&mut self) {
        let dev = self.dev.data();
        if dev.gpu.invalidate_context(&self.gpu_context).is_err() {
            dev_err!(self.dev, "Queue::drop: Failed to invalidate GPU context!\n");
        }
    }
}
