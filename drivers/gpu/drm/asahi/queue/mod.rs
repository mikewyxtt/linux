// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Submission queue management
//!
//! This module implements the userspace view of submission queues and the logic to map userspace
//! submissions to firmware queues.

mod render;
mod compute;
mod common;
