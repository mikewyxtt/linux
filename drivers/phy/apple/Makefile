# SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause
CFLAGS_trace.o			:= -I$(src)

obj-$(CONFIG_PHY_APPLE_ATC)		+= phy-apple-atc.o
phy-apple-atc-y			:= atc.o
phy-apple-atc-$(CONFIG_TRACING)	+= trace.o

obj-$(CONFIG_PHY_APPLE_ATC_CROSSBAR)	+= phy-apple-atc-display-crossbar.o
phy-apple-atc-display-crossbar-y	:= atc-display-crossbar.o
