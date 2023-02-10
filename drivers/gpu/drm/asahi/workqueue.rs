// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! GPU command execution queues
//!
//! The AGX GPU firmware schedules GPU work commands out of work queues, which are ring buffers of
//! pointers to work commands. There can be an arbitrary number of work queues. Work queues have an
//! associated type (vertex, fragment, or compute) and may only contain generic commands or commands
//! specific to that type.
//!
//! This module manages queueing work commands into a work queue and submitting them for execution
//! by the firmware. An active work queue needs an event to signal completion of its work, which is
//! owned by what we call a batch. This event then notifies the work queue when work is completed,
//! and that triggers freeing of all resources associated with that work. An idle work queue gives
//! up its associated event.

use crate::debug::*;
use crate::fw::channels::PipeType;
use crate::fw::types::*;
use crate::fw::workqueue::*;
use crate::object::OpaqueGpuObject;
use crate::{box_in_place, place};
use crate::{channel, event, fw, gpu, object, regs};
use core::num::NonZeroU64;
use core::sync::atomic::Ordering;
use kernel::{
    bindings,
    prelude::*,
    sync::{smutex::Mutex, Arc, Guard, UniqueArc},
    Opaque,
};

const DEBUG_CLASS: DebugFlags = DebugFlags::WorkQueue;

/// An enum of possible errors that might cause a piece of work to fail execution.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum WorkError {
    /// GPU timeout (command execution took too long).
    Timeout,
    /// GPU MMU fault (invalid access).
    Fault(regs::FaultInfo),
    /// Unknown reason.
    Unknown,
    /// Work failed due to an error caused by other concurrent GPU work.
    Killed,
}

impl From<WorkError> for kernel::error::Error {
    fn from(err: WorkError) -> Self {
        match err {
            WorkError::Timeout => ETIMEDOUT,
            // Not EFAULT because that's for userspace faults
            WorkError::Fault(_) => EIO,
            WorkError::Unknown => ENODATA,
            WorkError::Killed => ECANCELED,
        }
    }
}

struct SubmittedWork<O, C>
where
    O: OpaqueGpuObject,
    C: FnOnce(O, Option<WorkError>) + Send + Sync + 'static,
{
    object: O,
    value: EventValue,
    error: Option<WorkError>,
    wptr: u32,
    vm_slot: u32,
    callback: C,
}

trait GenSubmittedWork: Send + Sync {
    fn gpu_va(&self) -> NonZeroU64;
    fn value(&self) -> event::EventValue;
    fn wptr(&self) -> u32;
    fn mark_error(&mut self, value: event::EventValue, error: WorkError) -> bool;
    fn complete(self: Box<Self>);
}

impl<O: OpaqueGpuObject, C: FnOnce(O, Option<WorkError>) + Send + Sync> GenSubmittedWork
    for SubmittedWork<O, C>
{
    fn gpu_va(&self) -> NonZeroU64 {
        self.object.gpu_va()
    }

    fn value(&self) -> event::EventValue {
        self.value
    }

    fn wptr(&self) -> u32 {
        self.wptr
    }

    fn complete(self: Box<Self>) {
        let SubmittedWork {
            object,
            value,
            error,
            wptr,
            vm_slot,
            callback,
        } = *self;

        callback(object, error);
    }

    fn mark_error(&mut self, value: event::EventValue, error: WorkError) -> bool {
        if self.value <= value {
            mod_pr_debug!("WorkQueue: Command at value {:#x?} failed", self.value,);
            self.error = Some(match error {
                WorkError::Fault(info) if info.vm_slot != self.vm_slot => WorkError::Killed,
                err => err,
            });
            true
        } else {
            false
        }
    }
}

/// Inner data for managing a single work queue.
#[versions(AGX)]
struct WorkQueueInner {
    event_manager: Arc<event::EventManager>,
    info: GpuObject<QueueInfo::ver>,
    new: bool,
    pipe_type: PipeType,
    size: u32,
    wptr: u32,
    pending: Vec<Box<dyn GenSubmittedWork>>,
    last_token: Option<event::Token>,
    pending_jobs: usize,
    event: Option<(event::Event, event::EventValue)>,
    priority: u32,
    commit_seq: u64,
    submit_seq: u64,
}

/// An instance of a work queue.
#[versions(AGX)]
pub(crate) struct WorkQueue {
    info_pointer: GpuWeakPointer<QueueInfo::ver>,
    inner: Mutex<WorkQueueInner::ver>,
}

#[versions(AGX)]
impl WorkQueueInner::ver {
    /// Return the GPU done pointer, representing how many work items have been completed by the
    /// GPU.
    fn doneptr(&self) -> u32 {
        self.info
            .state
            .with(|raw, _inner| raw.gpu_doneptr.load(Ordering::Acquire))
    }
}

#[versions(AGX)]
#[derive(Copy, Clone)]
pub(crate) struct QueueEventInfo {
    pub(crate) stamp_pointer: GpuWeakPointer<Stamp>,
    pub(crate) fw_stamp_pointer: GpuWeakPointer<FwStamp>,
    pub(crate) slot: u32,
    pub(crate) value: event::EventValue,
    pub(crate) cmd_seq: u64,
    pub(crate) info_ptr: GpuWeakPointer<QueueInfo::ver>,
}

#[versions(AGX)]
pub(crate) struct Job {
    wq: Arc<WorkQueue::ver>,
    wq_size: u32,
    event_info: QueueEventInfo::ver,
    wptr: u32,
    cmd_wptr: u32,
    start_value: EventValue,
    pending: Vec<Box<dyn GenSubmittedWork>>,
    committed: bool,
    submitted: bool,
    event_count: usize,
}

#[versions(AGX)]
pub(crate) struct JobSubmission<'a> {
    inner: Guard<'a, Mutex<WorkQueueInner::ver>>,
    wptr: u32,
    event_count: usize,
    command_count: usize,
}

#[versions(AGX)]
impl Job::ver {
    pub(crate) fn event_info(&self) -> QueueEventInfo::ver {
        let mut info = self.event_info;
        info.cmd_seq += self.event_count as u64;

        info
    }

    pub(crate) fn next_seq(&mut self) {
        self.event_count += 1;
        self.event_info.value.increment();
    }

    pub(crate) fn add<O: object::OpaqueGpuObject + 'static>(
        &mut self,
        command: O,
        vm_slot: u32,
    ) -> Result {
        self.add_cb(command, vm_slot, |_, _| {})
    }

    pub(crate) fn add_cb<O: object::OpaqueGpuObject + 'static>(
        &mut self,
        command: O,
        vm_slot: u32,
        callback: impl FnOnce(O, Option<WorkError>) + Sync + Send + 'static,
    ) -> Result {
        if self.committed {
            pr_err!("WorkQueue: Tried to mutate committed Job");
            return Err(EINVAL);
        }

        self.pending.try_push(Box::try_new(SubmittedWork::<_, _> {
            object: command,
            value: self.event_info.value.next(),
            error: None,
            callback,
            wptr: self.cmd_wptr,
            vm_slot,
        })?)?;

        self.cmd_wptr = (self.cmd_wptr + 1) % self.wq_size;

        Ok(())
    }

    pub(crate) fn commit(&mut self) -> Result {
        if self.committed {
            pr_err!("WorkQueue: Tried to commit committed Job");
            return Err(EINVAL);
        }

        if self.pending.is_empty() {
            pr_err!("WorkQueue: Job::commit() with no commands");
            return Err(EINVAL);
        }

        let mut inner = self.wq.inner.lock();

        let ev = inner.event.as_mut().expect("WorkQueue: Job lost its event");

        if ev.1 != self.start_value {
            pr_err!("WorkQueue: Job::commit() out of order (event)");
            return Err(EINVAL);
        }

        ev.1 = self.event_info.value;
        inner.commit_seq += self.pending.len() as u64;
        self.committed = true;

        Ok(())
    }

    pub(crate) fn can_submit(&self) -> bool {
        self.wq.free_space() > self.pending.len()
    }

    pub(crate) fn submit(&mut self) -> Result<JobSubmission::ver<'_>> {
        if !self.committed {
            pr_err!("WorkQueue: Tried to submit uncommitted Job");
            return Err(EINVAL);
        }

        if self.submitted {
            pr_err!("WorkQueue: Tried to submit Job twice");
            return Err(EINVAL);
        }

        if self.pending.is_empty() {
            pr_err!("WorkQueue: Job::submit() with no commands");
            return Err(EINVAL);
        }

        let mut inner = self.wq.inner.lock();

        if inner.submit_seq != self.event_info.cmd_seq {
            pr_err!("WorkQueue: Job::submit() out of order (start_seq)");
            return Err(EINVAL);
        }

        if inner.commit_seq < (self.event_info.cmd_seq + self.pending.len() as u64) {
            pr_err!("WorkQueue: Job::submit() out of order (commit_seq)");
            return Err(EINVAL);
        }

        if self.wptr != inner.wptr {
            pr_err!("WorkQueue: Job::submit() out of order (wptr)");
            return Err(EINVAL);
        }

        let mut wptr = self.wptr;
        let command_count = self.pending.len();

        if inner.free_space() <= command_count {
            pr_err!("WorkQueue: Job does not fit in ring buffer");
            return Err(EBUSY);
        }

        inner.pending.try_reserve(command_count)?;

        for command in self.pending.drain(..) {
            let next_wptr = (wptr + 1) % inner.size;
            assert!(inner.doneptr() != next_wptr);
            inner.info.ring[self.wptr as usize] = command.gpu_va().get();
            wptr = next_wptr;

            // Cannot fail, since we did a try_reserve(1) above
            inner
                .pending
                .try_push(command)
                .expect("try_push() failed after try_reserve()");
        }

        self.submitted = true;

        Ok(JobSubmission::ver {
            inner,
            wptr,
            command_count,
            event_count: self.event_count,
        })
    }
}

#[versions(AGX)]
impl<'a> JobSubmission::ver<'a> {
    pub(crate) fn run(mut self, channel: &mut channel::PipeChannel::ver) {
        self.inner
            .info
            .state
            .with(|raw, _inner| raw.cpu_wptr.store(self.wptr, Ordering::Release));

        self.inner.wptr = self.wptr;

        let event = self
            .inner
            .event
            .as_mut()
            .expect("JobSubmission lost its event");

        let event_slot = event.0.slot();

        let msg = fw::channels::RunWorkQueueMsg::ver {
            pipe_type: self.inner.pipe_type,
            work_queue: Some(self.inner.info.weak_pointer()),
            wptr: self.inner.wptr,
            event_slot,
            is_new: self.inner.new,
            __pad: Default::default(),
        };
        channel.send(&msg);
        self.inner.new = false;

        self.inner.submit_seq += self.command_count as u64;

        core::mem::forget(self);
    }

    pub(crate) fn pipe_type(&self) -> PipeType {
        self.inner.pipe_type
    }

    pub(crate) fn priority(&self) -> u32 {
        self.inner.priority
    }
}

#[versions(AGX)]
impl Drop for Job::ver {
    fn drop(&mut self) {
        let mut inner = self.wq.inner.lock();

        if self.committed && !self.submitted {
            let event = inner.event.as_mut().expect("Job lost its event");
            event.1.sub(self.event_count as u32);
            inner.commit_seq -= self.pending.len() as u64;
        }

        inner.pending_jobs -= 1;

        if inner.pending.is_empty() && inner.pending_jobs == 0 {
            inner.event = None;
        }
    }
}

#[versions(AGX)]
impl<'a> Drop for JobSubmission::ver<'a> {
    fn drop(&mut self) {
        let inner = &mut self.inner;
        let new_len = inner.pending.len() - self.command_count;
        inner.pending.truncate(new_len);

        let event = inner.event.as_mut().expect("JobSubmission lost its event");
        event.1.sub(self.event_count as u32);
        inner.commit_seq -= self.command_count as u64;
    }
}

#[versions(AGX)]
impl WorkQueueInner::ver {
    /// Return the number of free entries in the workqueue
    pub(crate) fn free_space(&self) -> usize {
        self.size as usize - self.pending.len() - 1
    }
}

#[versions(AGX)]
impl WorkQueue::ver {
    /// Create a new WorkQueue of a given type and priority.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new(
        alloc: &mut gpu::KernelAllocators,
        event_manager: Arc<event::EventManager>,
        gpu_context: Arc<GpuObject<fw::workqueue::GpuContextData>>,
        notifier_list: Arc<GpuObject<fw::event::NotifierList>>,
        pipe_type: PipeType,
        id: u64,
        priority: u32,
        size: u32,
    ) -> Result<Arc<WorkQueue::ver>> {
        let mut info = box_in_place!(QueueInfo::ver {
            state: alloc.shared.new_default::<RingState>()?,
            ring: alloc.shared.array_empty(size as usize)?,
            gpu_buf: alloc.private.array_empty(0x2c18)?,
            notifier_list: notifier_list,
            gpu_context: gpu_context,
        })?;

        info.state.with_mut(|raw, _inner| {
            raw.rb_size = size;
        });

        let inner = WorkQueueInner::ver {
            event_manager,
            info: alloc.private.new_boxed(info, |inner, ptr| {
                Ok(place!(
                    ptr,
                    raw::QueueInfo::ver {
                        state: inner.state.gpu_pointer(),
                        ring: inner.ring.gpu_pointer(),
                        notifier_list: inner.notifier_list.gpu_pointer(),
                        gpu_buf: inner.gpu_buf.gpu_pointer(),
                        gpu_rptr1: Default::default(),
                        gpu_rptr2: Default::default(),
                        gpu_rptr3: Default::default(),
                        event_id: AtomicI32::new(-1),
                        priority: *raw::PRIORITY.get(priority as usize).ok_or(EINVAL)?,
                        unk_4c: -1,
                        uuid: id as u32,
                        unk_54: -1,
                        unk_58: Default::default(),
                        busy: Default::default(),
                        __pad: Default::default(),
                        unk_84_state: Default::default(),
                        unk_88: 0,
                        unk_8c: 0,
                        unk_90: 0,
                        unk_94: 0,
                        pending: Default::default(),
                        unk_9c: 0,
                        #[ver(V >= V13_2)]
                        unk_a0_0: 0,
                        gpu_context: inner.gpu_context.gpu_pointer(),
                        unk_a8: Default::default(),
                        #[ver(V >= V13_2)]
                        unk_b0: 0,
                    }
                ))
            })?,
            new: true,
            pipe_type,
            size,
            wptr: 0,
            pending: Vec::new(),
            last_token: None,
            event: None,
            priority,
            pending_jobs: 0,
            commit_seq: 0,
            submit_seq: 0,
        };

        Arc::try_new(Self {
            info_pointer: inner.info.weak_pointer(),
            inner: Mutex::new(inner),
        })
    }

    /// Returns the QueueInfo pointer for this workqueue, as a weak pointer.
    pub(crate) fn info_pointer(&self) -> GpuWeakPointer<QueueInfo::ver> {
        self.info_pointer
    }

    pub(crate) fn event_info(&self) -> Option<QueueEventInfo::ver> {
        let inner = self.inner.lock();

        inner.event.as_ref().map(|ev| QueueEventInfo::ver {
            stamp_pointer: ev.0.stamp_pointer(),
            fw_stamp_pointer: ev.0.fw_stamp_pointer(),
            slot: ev.0.slot(),
            value: ev.1,
            cmd_seq: inner.commit_seq,
            info_ptr: self.info_pointer,
        })
    }

    pub(crate) fn new_job(self: &Arc<Self>) -> Result<Job::ver> {
        let mut inner = self.inner.lock();

        if inner.event.is_none() {
            let event = inner.event_manager.get(inner.last_token, self.clone())?;
            let cur = event.current();
            inner.last_token = Some(event.token());
            inner.event = Some((event, cur));
        }

        inner.pending_jobs += 1;

        let ev = &inner.event.as_ref().unwrap();

        Ok(Job::ver {
            wq: self.clone(),
            wq_size: inner.size,
            event_info: QueueEventInfo::ver {
                stamp_pointer: ev.0.stamp_pointer(),
                fw_stamp_pointer: ev.0.fw_stamp_pointer(),
                slot: ev.0.slot(),
                value: ev.1,
                cmd_seq: inner.commit_seq,
                info_ptr: self.info_pointer,
            },
            start_value: ev.1,
            pending: Vec::new(),
            event_count: 0,
            committed: false,
            submitted: false,
            wptr: inner.wptr,
            cmd_wptr: inner.wptr,
        })
    }

    /// Return the number of free entries in the workqueue
    pub(crate) fn free_space(&self) -> usize {
        self.inner.lock().free_space()
    }

    pub(crate) fn pipe_type(&self) -> PipeType {
        self.inner.lock().pipe_type
    }
}

/// Trait used to erase the version-specific type of WorkQueues, to avoid leaking
/// version-specificity into the event module.
pub(crate) trait WorkQueue {
    fn signal(&self) -> bool;
    fn mark_error(&self, value: event::EventValue, error: WorkError);
}

#[versions(AGX)]
impl WorkQueue for WorkQueue::ver {
    /// Signal a workqueue that some work was completed.
    ///
    /// This will check the event stamp value to find out exactly how many commands were processed.
    fn signal(&self) -> bool {
        let mut inner = self.inner.lock();
        let event = inner.event.as_ref();
        let value = match event {
            None => {
                pr_err!("WorkQueue: signal() called but no event?");
                return true;
            }
            Some(event) => event.0.current(),
        };

        mod_pr_debug!(
            "WorkQueue({:?}): Signaling event {:?} value {:#x?}",
            inner.pipe_type,
            inner.last_token,
            value
        );

        let mut completed_commands: usize = 0;

        for cmd in inner.pending.iter() {
            if cmd.value() <= value {
                mod_pr_debug!(
                    "WorkQueue({:?}): Command at value {:#x?} complete",
                    inner.pipe_type,
                    cmd.value()
                );
                completed_commands += 1;
            } else {
                break;
            }
        }

        if completed_commands == 0 {
            return inner.pending.is_empty();
        }

        let mut completed = Vec::new();

        if completed.try_reserve(completed_commands).is_err() {
            pr_crit!(
                "WorkQueue({:?}): Failed to allocated space for {} completed commands",
                inner.pipe_type,
                completed_commands
            );
        }

        let last_wptr = 0;

        let pipe_type = inner.pipe_type;

        for cmd in inner.pending.drain(..completed_commands) {
            if completed.try_push(cmd).is_err() {
                pr_crit!(
                    "WorkQueue({:?}): Failed to signal a completed command",
                    pipe_type,
                );
            }
        }

        mod_pr_debug!(
            "WorkQueue({:?}): Completed {} commands",
            inner.pipe_type,
            completed_commands
        );

        if let Some(i) = completed.last() {
            inner
                .info
                .state
                .with(|raw, _inner| raw.cpu_freeptr.store(i.wptr(), Ordering::Release));
        }

        let empty = inner.pending.is_empty();
        if empty && inner.pending_jobs == 0 {
            inner.event = None;
        }

        core::mem::drop(inner);

        for cmd in completed {
            cmd.complete();
        }

        empty
    }

    /// Mark this queue's work up to a certain stamp value as having failed.
    fn mark_error(&self, value: event::EventValue, error: WorkError) {
        // If anything is marked completed, we can consider it successful
        // at this point, even if we didn't get the signal event yet.
        self.signal();

        let mut inner = self.inner.lock();

        if inner.event.is_none() {
            pr_err!("WorkQueue: signal_fault() called but no event?");
            return;
        }

        mod_pr_debug!(
            "WorkQueue({:?}): Signaling fault for event {:?} at value {:#x?}",
            inner.pipe_type,
            inner.last_token,
            value
        );

        for cmd in inner.pending.iter_mut() {
            if !cmd.mark_error(value, error) {
                break;
            }
        }
    }
}
