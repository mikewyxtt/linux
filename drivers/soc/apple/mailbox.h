/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * Apple mailbox message format
 *
 * Copyright The Asahi Linux Contributors
 */

#ifndef _APPLE_MAILBOX_H_
#define _APPLE_MAILBOX_H_

#include <linux/device.h>
#include <linux/types.h>

/* encodes a single 96bit message sent over the single channel */
struct apple_mbox_msg {
	u64 msg0;
	u32 msg1;
};

struct apple_mbox {
	void __iomem *regs;
	const struct apple_mbox_hw *hw;

	int irq_recv_not_empty;
	int irq_send_empty;

	spinlock_t rx_lock;
	spinlock_t tx_lock;

	struct completion tx_empty;

	/** Receive callback for incoming messages */
	void (*rx)(struct apple_mbox *mbox, struct apple_mbox_msg msg);
};

int apple_mbox_init_byname(struct device *dev, struct apple_mbox *mbox,
			   const char *name);
int apple_mbox_init(struct device *dev, struct apple_mbox *mbox, int index);

void apple_mbox_shutdown(struct apple_mbox *mbox);
void apple_mbox_start(struct apple_mbox *mbox);
void apple_mbox_stop(struct apple_mbox *mbox);
int apple_mbox_poll(struct apple_mbox *mbox);
int apple_mbox_send(struct apple_mbox *mbox, struct apple_mbox_msg msg,
		    bool atomic);

#endif
