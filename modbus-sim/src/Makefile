# Plain Makefile: native build, or cross build with CC=<cross-gcc>.
#   make
#   make CC=mipsel-openwrt-linux-musl-gcc LDFLAGS=-static
# The OpenWrt package Makefile (../Makefile) calls this with the SDK toolchain.

CC      ?= cc
CFLAGS  ?= -Os
override CFLAGS += -std=gnu99 -Wall -Wextra
override LDLIBS += -lm

modbus_sim: modbus_sim.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $< $(LDLIBS)

clean:
	rm -f modbus_sim

.PHONY: clean
