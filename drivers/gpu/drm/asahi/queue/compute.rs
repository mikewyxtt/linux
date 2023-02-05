// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(clippy::unusual_byte_groupings)]
#![allow(unused_imports)]

//! Compute work queue.
//!
//! A compute queue consists of one underlying WorkQueue.
//! This module is in charge of creating all of the firmware structures required to submit compute
//! work to the GPU, based on the userspace command buffer.

use super::common;
use crate::alloc::Allocator;
use crate::debug::*;
use crate::driver::AsahiDevice;
use crate::fw::types::*;
use crate::gpu::GpuManager;
use crate::{alloc, channel, event, file, fw, gpu, microseq, mmu, workqueue};
use crate::{box_in_place, inner_ptr, inner_weak_ptr, place};
use core::mem::MaybeUninit;
use core::sync::atomic::Ordering;
use kernel::bindings;
use kernel::io_buffer::IoBufferReader;
use kernel::prelude::*;
use kernel::sync::{smutex::Mutex, Arc};
use kernel::user_ptr::UserSlicePtr;

const DEBUG_CLASS: DebugFlags = DebugFlags::Compute;

#[versions(AGX)]
impl super::Queue::ver {
    /// Submit work to a compute queue.
    fn submit_compute(&self, cmd: &bindings::drm_asahi_command, id: u64) -> Result {
        if cmd.cmd_type != bindings::drm_asahi_cmd_type_DRM_ASAHI_CMD_COMPUTE {
            return Err(EINVAL);
        }

        let dev = self.dev.data();
        let gpu = match dev.gpu.as_any().downcast_ref::<gpu::GpuManager::ver>() {
            Some(gpu) => gpu,
            None => panic!("GpuManager mismatched with ComputeQueue!"),
        };

        // Wake up the firmware early while we get everything ready.
        let _op_guard = gpu.start_op()?;

        let notifier = &self.notifier;

        let mut alloc = gpu.alloc();
        let kalloc = &mut *alloc;

        mod_dev_dbg!(self.dev, "[Submission {}] Compute!\n", id);

        let mut cmdbuf_reader = unsafe {
            UserSlicePtr::new(
                cmd.cmd_buffer as usize as *mut _,
                core::mem::size_of::<bindings::drm_asahi_cmd_compute>(),
            )
            .reader()
        };

        let mut cmdbuf: MaybeUninit<bindings::drm_asahi_cmd_compute> = MaybeUninit::uninit();
        unsafe {
            cmdbuf_reader.read_raw(
                cmdbuf.as_mut_ptr() as *mut u8,
                core::mem::size_of::<bindings::drm_asahi_cmd_compute>(),
            )?;
        }
        let cmdbuf = unsafe { cmdbuf.assume_init() };

        // CHECKS HERE

        // This sequence number increases per new client/VM? assigned to some slot,
        // but it's unclear *which* slot...
        let slot_client_seq: u8 = (self.id & 0xff) as u8;

        let vm_bind = gpu.bind_vm(&self.vm)?;

        mod_dev_dbg!(
            self.dev,
            "[Submission {}] VM slot = {}\n",
            id,
            vm_bind.slot()
        );

        let mut batches = workqueue::WorkQueue::ver::begin_batch(&self.wq, vm_bind.slot())?;

        // TODO: Is this the same on all GPUs? Is this really for preemption?
        let preempt_size = 0x7fa0;
        let preempt2_off = 0x7f80;
        let preempt3_off = 0x7f88;
        let preempt4_off = 0x7f90;
        let preempt5_off = 0x7f98;

        let preempt_buf = self.ualloc.lock().array_empty(preempt_size)?;

        let mut seq_buf = self.ualloc.lock().array_empty(0x800)?;
        for i in 1..0x400 {
            seq_buf[i] = (i + 1) as u64;
        }

        let next_stamp = batches.event_value().next();

        mod_dev_dbg!(
            self.dev,
            "[Submission {}] Event #{} {:#x?} -> {:#x?}\n",
            id,
            batches.event().slot(),
            batches.event_value(),
            next_stamp
        );

        let timestamps = Arc::try_new(kalloc.shared.new_default::<fw::job::JobTimestamps>()?)?;

        let uuid = cmdbuf.cmd_id;

        let unk3 = debug_enabled(debug::DebugFlags::Debug3);

        mod_dev_dbg!(self.dev, "[Submission {}] UUID = {:#x?}\n", id, uuid);

        let cmd_seq = self.command_count.fetch_add(1, Ordering::Relaxed);

        mod_dev_dbg!(
            self.dev,
            "[Submission {}] Command seq = {:#x?}\n",
            id,
            cmd_seq
        );

        #[ver(V >= V13_0B4)]
        let count = self.counter.fetch_add(1, Ordering::Relaxed);

        let comp = GpuObject::new_prealloc(
            kalloc.private.alloc_object()?,
            |ptr: GpuWeakPointer<fw::compute::RunCompute::ver>| {
                let mut builder = microseq::Builder::new();

                let stats = gpu.initdata.runtime_pointers.stats.comp.weak_pointer();

                let start_comp = builder.add(microseq::StartCompute::ver {
                    header: microseq::op::StartCompute::HEADER,
                    unk_pointer: inner_weak_ptr!(ptr, unk_pointee),
                    job_params1: inner_weak_ptr!(ptr, job_params1),
                    stats,
                    work_queue: self.wq.info_pointer(),
                    vm_slot: vm_bind.slot(),
                    unk_28: 0x1,
                    event_generation: self.id as u32,
                    cmd_seq,
                    unk_34: 0x0,
                    unk_38: 0x0,
                    job_params2: inner_weak_ptr!(ptr, job_params2),
                    unk_44: 0x0,
                    uuid,
                    attachments: common::build_attachments(
                        cmdbuf.attachments,
                        cmdbuf.attachment_count,
                    )?,
                    padding: Default::default(),
                    #[ver(V >= V13_0B4)]
                    unk_flag: inner_weak_ptr!(ptr, unk_flag),
                    #[ver(V >= V13_0B4)]
                    counter: U64(count),
                    #[ver(V >= V13_0B4)]
                    notifier_buf: inner_weak_ptr!(notifier.weak_pointer(), state.unk_buf),
                })?;

                /*
                builder.add(microseq::Timestamp::ver {
                    header: microseq::op::Timestamp::new(true),
                    cur_ts: inner_weak_ptr!(ptr, cur_ts),
                    start_ts: inner_weak_ptr!(ptr, start_ts),
                    update_ts: inner_weak_ptr!(ptr, start_ts),
                    work_queue: self.wq.info_pointer(),
                    unk_24: U64(0),
                    #[ver(V >= V13_0B4)]
                    unk_ts: inner_weak_ptr!(ptr, unk_ts),
                    uuid,
                    unk_30_padding: 0,
                })?;
                */

                builder.add(microseq::WaitForIdle {
                    header: microseq::op::WaitForIdle::new(microseq::Pipe::Compute),
                })?;

                /*
                builder.add(microseq::Timestamp::ver {
                    header: microseq::op::Timestamp::new(false),
                    cur_ts: inner_weak_ptr!(ptr, cur_ts),
                    start_ts: inner_weak_ptr!(ptr, start_ts),
                    update_ts: inner_weak_ptr!(ptr, end_ts),
                    work_queue: self.wq.info_pointer(),
                    unk_24: U64(0),
                    #[ver(V >= V13_0B4)]
                    unk_ts: inner_weak_ptr!(ptr, unk_ts),
                    uuid,
                    unk_30_padding: 0,
                })?;
                */

                let off = builder.offset_to(start_comp);
                builder.add(microseq::FinalizeCompute::ver {
                    header: microseq::op::FinalizeCompute::HEADER,
                    stats,
                    work_queue: self.wq.info_pointer(),
                    vm_slot: vm_bind.slot(),
                    #[ver(V < V13_0B4)]
                    unk_18: 0,
                    job_params2: inner_weak_ptr!(ptr, job_params2),
                    unk_24: 0,
                    uuid,
                    fw_stamp: batches.event().fw_stamp_pointer(),
                    stamp_value: next_stamp,
                    unk_38: 0,
                    unk_3c: 0,
                    unk_40: 0,
                    unk_44: 0,
                    unk_48: 0,
                    unk_4c: 0,
                    unk_50: 0,
                    unk_54: 0,
                    unk_58: 0,
                    #[ver(G == G14 && V < V13_0B4)]
                    unk_5c_g14: U64(0),
                    restart_branch_offset: off,
                    unk_60: unk3.into(),
                    #[ver(V >= V13_0B4)]
                    unk_64: Default::default(),
                    #[ver(V >= V13_0B4)]
                    unk_flag: inner_weak_ptr!(ptr, unk_flag),
                    #[ver(V >= V13_0B4)]
                    unk_79: Default::default(),
                })?;

                builder.add(microseq::RetireStamp {
                    header: microseq::op::RetireStamp::HEADER,
                })?;

                Ok(box_in_place!(fw::compute::RunCompute::ver {
                    notifier: notifier.clone(),
                    preempt_buf: preempt_buf,
                    seq_buf: seq_buf,
                    micro_seq: builder.build(&mut kalloc.private)?,
                    vm_bind: vm_bind.clone(),
                    timestamps: timestamps.clone(),
                })?)
            },
            |inner, ptr| {
                Ok(place!(
                    ptr,
                    fw::compute::raw::RunCompute::ver {
                        tag: fw::workqueue::CommandType::RunCompute,
                        #[ver(V >= V13_0B4)]
                        counter: U64(count),
                        unk_4: 0,
                        vm_slot: vm_bind.slot(),
                        notifier: inner.notifier.gpu_pointer(),
                        unk_pointee: Default::default(),
                        job_params1: fw::compute::raw::JobParameters1 {
                            preempt_buf1: inner.preempt_buf.gpu_pointer(),
                            encoder: U64(cmdbuf.encoder_ptr),
                            // buf2-5 Only if internal program is used
                            preempt_buf2: inner.preempt_buf.gpu_offset_pointer(preempt2_off),
                            preempt_buf3: inner.preempt_buf.gpu_offset_pointer(preempt3_off),
                            preempt_buf4: inner.preempt_buf.gpu_offset_pointer(preempt4_off),
                            preempt_buf5: inner.preempt_buf.gpu_offset_pointer(preempt5_off),
                            pipeline_base: U64(0x11_00000000),
                            unk_38: U64(0x8c60),
                            unk_40: cmdbuf.ctx_switch_prog, // Internal program addr | 1
                            unk_44: 0,
                            compute_layout_addr: U64(cmdbuf.buffer_descriptor), // Only if internal program used
                            unk_50: cmdbuf.buffer_descriptor_size, // 0x40 if internal program used
                            unk_54: 0,
                            unk_58: 1,
                            unk_5c: 0,
                            iogpu_unk_40: cmdbuf.iogpu_unk_40, // 0x1c if internal program used
                        },
                        unk_b8: Default::default(),
                        microsequence: inner.micro_seq.gpu_pointer(),
                        microsequence_size: inner.micro_seq.len() as u32,
                        job_params2: fw::compute::raw::JobParameters2::ver {
                            #[ver(V >= V13_0B4)]
                            unk_0_0: 0,
                            unk_0: Default::default(),
                            preempt_buf1: inner.preempt_buf.gpu_pointer(),
                            encoder_end: U64(cmdbuf.encoder_end),
                            unk_34: Default::default(),
                            #[ver(V < V13_0B4)]
                            unk_5c: 0,
                        },
                        encoder_params: fw::job::raw::EncoderParams {
                            unk_8: 0x0,  // fixed
                            unk_c: 0x0,  // fixed
                            unk_10: 0x0, // fixed
                            encoder_id: cmdbuf.encoder_id,
                            unk_18: 0x0, // fixed
                            iogpu_compute_unk44: cmdbuf.iogpu_unk_44,
                            seq_buffer: inner.seq_buf.gpu_pointer(),
                            unk_28: U64(0x0), // fixed
                        },
                        meta: fw::job::raw::JobMeta {
                            unk_4: 0,
                            stamp: batches.event().stamp_pointer(),
                            fw_stamp: batches.event().fw_stamp_pointer(),
                            stamp_value: next_stamp,
                            stamp_slot: batches.event().slot(),
                            evctl_index: 0, // fixed
                            flush_stamps: 0,
                            uuid: uuid,
                            queue_cmd_count: 0, // TODO: fix
                        },
                        cur_ts: U64(0),
                        start_ts: Some(inner_ptr!(inner.timestamps.gpu_pointer(), start)),
                        end_ts: Some(inner_ptr!(inner.timestamps.gpu_pointer(), end)),
                        unk_2c0: 0,
                        unk_2c4: 0,
                        unk_2c8: 0,
                        unk_2cc: 0,
                        client_sequence: slot_client_seq,
                        pad_2d1: Default::default(),
                        unk_2d4: 0,
                        unk_2d8: 0,
                        #[ver(V >= V13_0B4)]
                        unk_ts: U64(0),
                        #[ver(V >= V13_0B4)]
                        unk_2e1: Default::default(),
                        #[ver(V >= V13_0B4)]
                        unk_flag: U32(0),
                        #[ver(V >= V13_0B4)]
                        unk_pad: Default::default(),
                    }
                ))
            },
        )?;

        core::mem::drop(alloc);

        notifier.threshold.with(|raw, _inner| {
            raw.increment();
        });
        batches.add(Box::try_new(comp)?)?;
        let batch = batches.commit()?;

        mod_dev_dbg!(self.dev, "[Submission {}] Submit compute!\n", id);
        gpu.submit_batch(batches)?;

        mod_dev_dbg!(
            self.dev,
            "[Submission {}] Waiting for compute batch...\n",
            id
        );

        let mut ret = Ok(());

        match batch.wait() {
            Ok(()) => {
                mod_dev_dbg!(self.dev, "[Submission {}] Compute batch completed!\n", id);
            }
            Err(err) => {
                dev_err!(
                    self.dev,
                    "[Submission {}] Compute batch failed: {:?}\n",
                    id,
                    err
                );
                ret = Err(err.into());
            }
        }

        let (ts_start, ts_end) = timestamps.with(|raw, _inner| {
            (
                raw.start.load(Ordering::Relaxed),
                raw.end.load(Ordering::Relaxed),
            )
        });

        mod_dev_dbg!(
            self.dev,
            "[Submission {}] Timestamps: {} {} (delta: {})\n",
            id,
            ts_start,
            ts_end,
            ts_end.wrapping_sub(ts_start)
        );

        if debug_enabled(debug::DebugFlags::WaitForPowerOff) {
            mod_dev_dbg!(self.dev, "[Submission {}] Waiting for GPU power-off\n", id);
            if gpu.wait_for_poweroff(100).is_err() {
                dev_warn!(self.dev, "[Submission {}] GPU failed to power off\n", id);
            }
            mod_dev_dbg!(self.dev, "[Submission {}] GPU powered off\n", id);
        }

        ret
    }
}
