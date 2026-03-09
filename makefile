#!/usr/bin/make

NPROCS := $(shell nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 1)
MAKEFLAGS += -j$(NPROCS)

DIRS = mbed src
DIRSCLEAN = $(addsuffix .clean,$(DIRS))

all:
	@ $(MAKE) -C mbed
	@echo Building Smoothie
	@ $(MAKE) -C src

clean: $(DIRSCLEAN)

$(DIRSCLEAN): %.clean:
	@echo Cleaning $*
	@ $(MAKE) -C $*  clean

debug-store:
	@ $(MAKE) -C src debug-store

flash:
	@ $(MAKE) -C src flash

stm32-pad:
	python3 tools/pad-firmware.py STM32F407xG/main.bin

stm32-flash: stm32-pad
	st-flash write STM32F407xG/main-padded.bin 0x08000000

stm32-backup:
	st-flash read firmware-backup-$$(date +%Y%m%d).bin 0x08000000 0x80000

stm32-verify:
	st-flash read /tmp/stm32-verify.bin 0x08000000 0x80000
	@md5sum /tmp/stm32-verify.bin

dfu:
	@ $(MAKE) -C src dfu

upload:
	@ $(MAKE) -C src upload

debug:
	@ $(MAKE) -C src debug

console:
	@ $(MAKE) -C src console

.PHONY: all $(DIRS) $(DIRSCLEAN) debug-store flash stm32-pad stm32-flash stm32-backup stm32-verify upload debug console dfu
