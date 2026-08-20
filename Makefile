# Project Name
TARGET = DualWavetableOSCsm

USE_DAISYSP_LGPL = 1

# Enable float formatting for PrintLine("%.5f")
LDFLAGS += -u _printf_float

# Run the app from QSPI (8 MB) via the Daisy bootloader instead of the 128 KB
# internal flash. The menu UI had filled internal flash to ~98%; QSPI removes the
# ceiling. The board's micro-USB is blocked by the menu header, so flashing is
# done over JTAG/SWD with OpenOCD (see `program` below) — never DFU.
APP_TYPE = BOOT_QSPI

# No-wait bootloader (10 ms window) so it jumps straight to the QSPI app at boot.
BOOT_BIN = $(SYSTEM_FILES_DIR)/dsy_bootloader_v6_4-intdfu-10ms.bin

# OpenOCD probe: ST-Link in DAP (non-HLA) mode — required for QSPI flashing via
# daisy_qspi.cfg. Set before the libDaisy include so it wins over the core's
# `PGM_DEVICE ?= interface/stlink.cfg`, while still allowing a CLI override.
PGM_DEVICE = interface/stlink-dap.cfg

# Optimise for size. Not strictly required from QSPI, but keeps the image small.
OPT = -Os

# Sources
CPP_SOURCES = \
main.cpp \
src/calibration/calibration_data.cpp \
src/calibration/calibration_processor.cpp \
src/calibration/calibration_store.cpp \
src/calibration/calibration_runtime.cpp

# Library Locations
LIBDAISY_DIR = ./lib/libDaisy
DAISYSP_DIR  = ./lib/DaisySP

# Core location, and generic makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile

# Build all dependencies then the project
all: libs
	$(MAKE) build/$(TARGET).elf build/$(TARGET).hex build/$(TARGET).bin

libs:
	$(MAKE) -C $(LIBDAISY_DIR)
	$(MAKE) -C $(DAISYSP_DIR)
	$(MAKE) -C $(DAISYSP_DIR)/DaisySP-LGPL

lint:
	clang-format --dry-run --Werror $(CPP_SOURCES)

# ---------------------------------------------------------------------------
# JTAG / SWD flashing via OpenOCD. The app runs from QSPI, so we use the ST-Link
# *DAP* driver (non-HLA) plus daisy_qspi.cfg's unlock sequence so OpenOCD's stmqspi
# flash driver can write the external IS25LP064A. Override PGM_DEVICE for another
# probe (e.g. a CMSIS-DAP adapter) — see PGM_DEVICE above.
# ---------------------------------------------------------------------------

# Per build: flash the app into QSPI (overrides libDaisy's DFU-only stub).
program:
	$(OCD) -s $(OCD_DIR) -f $(PGM_DEVICE) -f ./daisy_qspi.cfg \
		-c "program build/$(TARGET).elf verify reset exit"

# One-time: flash the Daisy bootloader into internal flash (0x08000000).
program-bootloader:
	$(OCD) -s $(OCD_DIR) -f $(PGM_DEVICE) -f ./daisy_qspi.cfg \
		-c "program $(BOOT_BIN) 0x08000000 verify reset exit"

.PHONY: all libs lint program program-bootloader