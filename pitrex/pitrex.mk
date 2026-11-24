# pitrex.mk — build an SDK game as a PiTrex kernel image (Raspberry Pi Zero 2 W).
#
# A game's Makefile sets the same variables it sets for uvm2.mk and includes this:
#
#     UVM2_NAME   = mygame
#     UVM2_SRCS   = src/main.c
#     PITREX_CFLAGS = -Isrc            (the game's own includes and defines)
#     include <sdk>/pitrex/pitrex.mk
#
#     make pitrex        -> $(PITREX_BUILD)/kernel7l.img, the file the card boots
#
# WHAT IS THE SAME AS ON THE CARTRIDGES: uvm2_draw.c, the beam model (vectrex-draw), text,
# input, sound, the config and the calibration wizard. WHAT IS THIS BOARD'S: the bus layer
# (uvm2_bus_pitrex.c), start-up and the SD card, in this folder.
#
# It needs, outside the SDK:
#   PITREX_BM         Malban's baremetal tree (boot, MMU, FAT, the GPIO bus primitives).
#                     ITS lib7, built for the Zero 2: a library built for the Zero 1 carries
#                     the other peripheral base, links clean and does not run.
#   PITREX_TOOLCHAIN  GCC 10.3 (gnu-rm). The baremetal libraries are built with it.

PITREX_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
UVM2_SDK   ?= $(abspath $(PITREX_DIR)/../uvm2-sdk)
VECTREX_DRAW_CABI := $(abspath $(PITREX_DIR)/../vectrex-draw/cabi)

PITREX_BM        ?= $(HOME)/projects/pitrex-baremetal
PITREX_TOOLCHAIN ?= /opt/arm-toolchain/bin
# THE BOARD: zero2 (Raspberry Pi Zero 2 W, the default) or zero1 (the first Zero). One card
# takes both: the Pi's firmware boots its own kernel by model (config.txt's [pi0] and [pi02]).
PITREX_BOARD     ?= zero2
ifeq ($(PITREX_BOARD),zero1)
PITREX_BUILD     ?= build_pitrex1
else
PITREX_BUILD     ?= build_pitrex
endif

PITREX_CC      := $(PITREX_TOOLCHAIN)/arm-none-eabi-gcc
PITREX_OBJCOPY := $(PITREX_TOOLCHAIN)/arm-none-eabi-objcopy
PITREX_NM      := $(PITREX_TOOLCHAIN)/arm-none-eabi-nm
ifeq ($(PITREX_BOARD),zero1)
# The Zero's ARM1176JZF-S, ARMv6 with VFP: the flags of the baremetal tree's BUILD_FOR=zero1,
# whose libraries are in its lib/ (built with `make BUILD_FOR=zero1` in pitrex/baremetal,
# pitrex/vectrex). One core: the list is replayed in series (PITREX_DUAL_CORE below).
PITREX_LIB     := $(PITREX_BM)/pitrex/lib
PITREX_ARCH    := -mhard-float -mfloat-abi=hard -mfpu=vfp -march=armv6zk -mtune=arm1176jzf-s
PITREX_RASPPI  := 1
PITREX_RUST_TARGET := arm-unknown-linux-gnueabihf
PITREX_DUAL_CORE   := 0
else
PITREX_LIB     := $(PITREX_BM)/pitrex/lib7

# The Zero 2's Cortex-A53, in AArch32 with the hard-float ABI: the same flags the baremetal
# libraries were built with (its Rules.mk, BUILD_FOR=zero2).
PITREX_ARCH := -mhard-float -mfloat-abi=hard -mfpu=neon-fp-armv8 -march=armv8-a -mtune=cortex-a53
PITREX_RASPPI  := 3

# The beam model, for the same CPU and ABI. rustc 1.91 ships no core for a bare-metal
# ARMv7-A hard-float target, and the crate is no_std: the Linux target of that CPU gives the
# same code and the same ABI, and nothing of Linux is linked. (The Zero's: its ARMv6 Linux
# target, the same way.)
PITREX_RUST_TARGET := armv7-unknown-linux-gnueabihf
endif
PITREX_DRAW_LIB    := $(VECTREX_DRAW_CABI)/target/$(PITREX_RUST_TARGET)/release/libvectrex_draw_cabi.a

# UVM2_PITREX          this board's branches in shared code
# UVM2_CONFIG_NO_FLASH the card is the only store (uvm2_config.c)
# UVM2_SUBUNITS        geometry in 1/16 of a unit, as every other build (uvm2.mk)
# RASPPI/USE_PL011_UART what the baremetal headers key on
PITREX_DEFS := -DUVM2_PITREX -DUVM2_CONFIG_NO_FLASH -DUVM2_SUBUNITS \
               -DUVM2_HZ=$(or $(UVM2_HZ),50) -DRASPPI=$(PITREX_RASPPI) -DUSE_PL011_UART=1
# DUAL CORE IS THE BUILD: core 1 replays the lists with the cartridges' own loop
# (uvm2_core1.c). PITREX_DUAL_CORE=0 is there to bisect, like UVM2_BENCH_NO_CORE1 on the
# cartridges: the same kernel with the list replayed in series on core 0.
PITREX_DUAL_CORE ?= 1
ifeq ($(PITREX_DUAL_CORE),1)
PITREX_DEFS += -DUVM2_DUAL_CORE
# ...and while the program has no new list, core 1 draws the last one again instead of leaving
# the beam off: a report written to the card (the one at frame 600 takes about half a second)
# or a game that reads a file blanked the screen for as long as it took (2026-11-24, the menu
# going black once a few seconds after it came up). The cartridges need a guard around their
# BIOS calls for this (uvm2_core1.c: their card shares the bus); here the card is the Pi's own.
PITREX_REPLAY_LAST ?= 1
ifeq ($(PITREX_REPLAY_LAST),1)
PITREX_DEFS += -DUVM2_REPLAY_LAST
endif
PITREX_CORE_SRCS := $(UVM2_SDK)/uvm2_core1.c
else
PITREX_DEFS += -DUVM2_BENCH_NO_CORE1
PITREX_CORE_SRCS :=
endif

# THE CARD DRIVER, ENDED PROPERLY. The baremetal tree's sd.c reads and writes several blocks
# with CMD18 / CMD25 and never ends them: no CMD12, and the controller is not told to send one.
# The card stays in its receive state; the next command finds it there — a state the driver's
# check before each command does not handle — fails, and the driver starts the card again from
# nothing, half-second waits included. Every write paid that: installing a package wrote at
# 16 KB/s, 95% of its time in writing (the records of 2026-11-25: 54 s of 57). gen_sd.sh makes
# the controller send the CMD12 (auto CMD12) and the check stop a card left receiving. The
# driver is compiled here, from the tree, and takes the place of the library's.
PITREX_SD_GEN := $(PITREX_BUILD)/gen/pitrex_sd.c
PITREX_SD_OK  := $(shell sh $(PITREX_DIR)/gen_sd.sh $(PITREX_BM)/pitrex/baremetal/sd.c $(PITREX_SD_GEN))
ifneq ($(PITREX_SD_OK),ok)
$(error pitrex: the baremetal tree's sd.c is not as gen_sd.sh expects: look at what changed there)
endif
# (added to PITREX_BOARD_SRCS below, once that is defined)
# The romset's zip is read whole into a buffer before the game starts (uvm2_romzip.c). On the
# cartridges that buffer is sized from the zip, to the KB, because it is their SRAM; here it
# is ordinary memory, so it is sized once for the largest set a card may carry.
#
# A RESIDENT IMAGE HAS NO ROMSET AND MUST STAY SMALL: it lives at LOADER_START (64 MB), and
# with this buffer in it the image ended past 80 MB, where the baremetal tree's programs put
# things (heap_low, linkerHeapDefBoot.ld): the menu crashed when a game came back (2026-11-23).
# (What was written here then — "the heap of every program starts at heap_low" — is NOT how
# the library's sbrk works: it starts at the image's own end. See uvm2_pitrex_loader.c.) The buffer is then the size of
# the search's scratch, which is all a program without a romset uses; and the end of the image
# is checked against heap_low when it is linked (loader.pit, below).
ifeq ($(PITREX_LOADER),1)
PITREX_ROMZIP_MAX ?= 65536
else
PITREX_ROMZIP_MAX ?= 16777216
endif
PITREX_DEFS += -DUVM2_ROMZIP_MAX=$(PITREX_ROMZIP_MAX)u
# THE SAMPLE DAC IS WRITTEN 8000 TIMES A SECOND HERE, not the cartridges' 16000 (uvm2_smp.h,
# UVM2_SMP_HZ). Each DAC write is four commands in the draw list, and on this board a command
# costs two bus cycles: at 16000 that is a tenth of a 50 Hz frame taken from a picture that
# is already at its limit; at 8000, a twentieth. 8000 is also the rate that header says was
# measured working on the console. A project may ask for another (UVM2_SMP_HZ=...).
PITREX_DEFS += -DUVM2_SMP_HZ=$(or $(UVM2_SMP_HZ),8000)u
# A COMMAND COSTS TWO BUS CYCLES HERE, never one: the board leaves an idle period after every
# access (uvm2_bus_pitrex.c measures it and reports it as write_cycles). The list builder
# counts with it (uvm2_draw.c, UVM2_CMD_MIN_CYCLES).
PITREX_DEFS += -DUVM2_CMD_MIN_CYCLES=2u
# PITREX_SPEAKER: the jack's sound out of the console's speaker (uvm2_jack_pitrex.c).
# PITREX_SPEAKER=0 builds without it.
PITREX_SPEAKER ?= 1
ifeq ($(PITREX_SPEAKER),1)
PITREX_DEFS += -DUVM2_PITREX_SPEAKER
endif
# PITREX_REPORTS=1: every program writes its reports to the card's logs/ (start-up, frame 600,
# exit). Off by default: only a fault is written (uvm2_pitrex_main.c, report_wanted).
PITREX_REPORTS ?= 0
# PITREX_LAUNCH_LOG=1 (a resident menu): each launch leaves a line in logs/launch.txt
ifeq ($(PITREX_LAUNCH_LOG),1)
PITREX_DEFS += -DUVM2_PITREX_LAUNCH_LOG
endif
ifeq ($(PITREX_REPORTS),1)
PITREX_DEFS += -DUVM2_PITREX_REPORTS
endif
# the image's name, for its reports on the card (logs/<name>_<n>.log)
PITREX_DEFS += -DUVM2_PITREX_NAME='"$(UVM2_NAME)"'
# The program's own folder on the card, from the top ("/games/zblast"): where it starts, when
# the card has it (uvm2_pitrex_main.c, game_folder). Empty: it starts at the top.
ifneq ($(PITREX_GAME_DIR),)
PITREX_DEFS += -DUVM2_PITREX_GAME_DIR='"$(PITREX_GAME_DIR)"'
endif
ifneq ($(UVM2_CMD_CAPACITY),)
PITREX_DEFS += -DUVM2_CMD_CAPACITY=$(UVM2_CMD_CAPACITY)u
endif

# The baremetal tree's headers go LAST, after the game's own folders: a port includes
# <vectrex/vectrexInterface.h> meaning the SDK's v_* declarations (pitrex-sim/include), and
# the tree has a header of that same name for its own drawing library.
# compat/: the few pico-sdk headers a .um2 game's source asks for, answered by this board
PITREX_INCS    := -I$(UVM2_SDK) -I$(PITREX_DIR) -I$(PITREX_DIR)/compat
PITREX_BM_INCS := -I$(PITREX_BM)/pitrex

PITREX_ALL_CFLAGS := -O2 -ffreestanding -ffunction-sections -fdata-sections -fno-common \
                     -fno-math-errno -w $(PITREX_ARCH) $(PITREX_DEFS) $(PITREX_INCS)
# The SDK's own files see the baremetal tree before the game's folders (they include its real
# headers); the game's files see their own folders first (see PITREX_BM_INCS).
PITREX_SDK_CFLAGS  := $(PITREX_ALL_CFLAGS) $(PITREX_BM_INCS) $(PITREX_CFLAGS)
PITREX_GAME_CFLAGS := $(PITREX_ALL_CFLAGS) $(PITREX_CFLAGS) $(PITREX_BM_INCS)

# The SDK, shared: the files a .um2 compiles, minus the ones that are the RP2350's.
PITREX_SDK_SRCS := $(UVM2_SDK)/uvm2_draw.c $(UVM2_SDK)/uvm2_hud.c $(UVM2_SDK)/uvm2_text.c \
                   $(UVM2_SDK)/uvm2_input.c $(UVM2_SDK)/uvm2_audio.c \
                   $(UVM2_SDK)/uvm2_config.c $(UVM2_SDK)/uvm2_wizard.c \
                   $(UVM2_SDK)/uvm2_svc.c $(UVM2_SDK)/uvm2_romzip.c \
                   $(UVM2_SDK)/uvm2_smp.c $(UVM2_SDK)/uvm2_options.c
# This board's.
PITREX_BOARD_SRCS := $(PITREX_DIR)/uvm2_bus_pitrex.c $(PITREX_DIR)/uvm2_pitrex_main.c \
                     $(PITREX_DIR)/uvm2_sd_pitrex.c $(PITREX_DIR)/uvm2_pitrex_stubs.c \
                     $(PITREX_DIR)/uvm2_pitrex_probe.c $(PITREX_DIR)/uvm2_jack_pitrex.c

# A RESIDENT IMAGE (PITREX_LOADER = 1): the program the card's bootstrap reads to LOADER_START
# and that starts every other one — a menu. Same SDK, linked 64 MB up: its entry object goes
# FIRST in the link and pushes the rest there (uvm2_pitrex_loader_entry.S says why), and the
# result is loader.pit, the image with the empty 64 MB cut off.
ifeq ($(PITREX_LOADER),1)
PITREX_BOARD_SRCS += $(PITREX_DIR)/uvm2_pitrex_loader.c
# ...and the menu installs Universal Vectrex Packages (uvm2_uvp.c, tools/uvp.py)
PITREX_BOARD_SRCS += $(UVM2_SDK)/uvm2_uvp.c $(UVM2_SDK)/uvm2_toml.c
PITREX_BOARD_SRCS += $(PITREX_SD_GEN)
# it includes its own folder's headers, as when the tree builds it
$(PITREX_BUILD)/sdk/pitrex_sd.o: PITREX_SDK_CFLAGS += -I$(PITREX_BM)/pitrex/baremetal
PITREX_ENTRY_OBJ  := $(PITREX_BUILD)/sdk/uvm2_pitrex_loader_entry.o
# The C heap of a resident image starts past what it keeps beyond its end, not at its end
# (uvm2_pitrex_loader.c, "THE HEAP").
PITREX_LDFLAGS    += -Wl,--wrap=sbrk -Wl,--wrap=_sbrk
else
PITREX_ENTRY_OBJ  :=
endif

PITREX_SDK_OBJS   := $(patsubst %.c,$(PITREX_BUILD)/sdk/%.o,$(notdir $(PITREX_SDK_SRCS) $(PITREX_CORE_SRCS) $(PITREX_BOARD_SRCS)))
# A PORT WRITTEN AGAINST THE v_* CALLS (v_directDraw32, v_WaitRecal, ...) sets
# PITREX_V_API = 1: the same bridge the .um2 links (rp2350-sdk/sdk_rp2350.c), which on this
# board reaches the SDK's dispatcher (uvm2_svc.c) by a call instead of `svc`.
# UVM2_PICO_RUNTIME is how the bridge and the ports name "the SDK is linked into this image".
# Game sources may sit outside the game's folder; their objects are named by file name.
PITREX_RP2350_SDK ?= $(abspath $(PITREX_DIR)/../rp2350-sdk)
# PITREX_SRCS: the game's sources when they are not simply UVM2_SRCS (uvm2.mk passes the list
# with the cartridge-only files dropped).
PITREX_SRCS ?= $(UVM2_SRCS)
ifeq ($(PITREX_V_API),1)
# (a project that lists the bridge among its own sources gets it once)
PITREX_GAME_SRCS := $(filter-out %/sdk_rp2350.c,$(PITREX_SRCS)) $(PITREX_RP2350_SDK)/sdk_rp2350.c
PITREX_GAME_DEFS := -DUVM2_PICO_RUNTIME=1
else
PITREX_GAME_SRCS := $(PITREX_SRCS)
PITREX_GAME_DEFS :=
endif
# Objects are named by file name alone, so two sources with one name would be one object.
PITREX_GAME_DUPES := $(strip $(foreach n,$(sort $(notdir $(PITREX_GAME_SRCS))),\
                       $(if $(filter-out 1,$(words $(filter %/$(n) $(n),$(PITREX_GAME_SRCS)))),$(n))))
ifneq ($(PITREX_GAME_DUPES),)
$(error pitrex: two game sources share a file name ($(PITREX_GAME_DUPES)); their objects would collide)
endif
PITREX_GAME_OBJS  := $(patsubst %.c,$(PITREX_BUILD)/game/%.o,$(notdir $(PITREX_GAME_SRCS)))
vpath %.c $(sort $(dir $(PITREX_GAME_SRCS)))

# C++ SOURCES (PITREX_CXX_SRCS): compiled with g++ whatever their extension — a program may
# compile a .c file as C++ — with the game's flags less its C standard, and linked with the
# C++ support library. Objects are named by file name, like the C ones.
#
# `main` is the one name the two languages have to agree on: the start-up calls the game's
# main as uvm2_game_main, a C name, and a main renamed in a C++ file would get a C++ name. A
# header put in front of every C++ source declares it with C linkage (compat/cxx_main.h).
PITREX_CXX        := $(PITREX_TOOLCHAIN)/arm-none-eabi-g++
PITREX_CXX_SRCS   ?=
PITREX_CXX_OBJS   := $(addprefix $(PITREX_BUILD)/game/,$(addsuffix .o,$(basename $(notdir $(PITREX_CXX_SRCS)))))
ifneq ($(PITREX_CXX_SRCS),)
PITREX_CXX_LIBS   := -lsupc++
endif
PITREX_ALL_GAME_NAMES := $(basename $(notdir $(PITREX_GAME_SRCS) $(PITREX_CXX_SRCS)))
ifneq ($(words $(PITREX_ALL_GAME_NAMES)),$(words $(sort $(PITREX_ALL_GAME_NAMES))))
$(error pitrex: a C and a C++ source share a file name; their objects would collide)
endif

vpath %.c $(UVM2_SDK) $(PITREX_DIR) $(dir $(PITREX_SD_GEN))
vpath %.S $(PITREX_DIR)

# THE FLAGS ARE A DEPENDENCY. make does not see a define that changed (a bigger command list,
# another draw option): the objects stay as they were and the image links without a word,
# built with two different sets. The flags are written to a file only when they differ, and
# every object depends on it.
PITREX_FLAGS_FILE := $(PITREX_BUILD)/flags.txt
.PHONY: pitrex-flags-force
pitrex-flags-force:
$(PITREX_FLAGS_FILE): pitrex-flags-force
	@mkdir -p $(PITREX_BUILD)
	@printf '%s\n' '$(subst ','\'',$(PITREX_GAME_CFLAGS) $(PITREX_GAME_DEFS))' | cmp -s - $(PITREX_FLAGS_FILE) || \
	    printf '%s\n' '$(subst ','\'',$(PITREX_GAME_CFLAGS) $(PITREX_GAME_DEFS))' > $(PITREX_FLAGS_FILE)

$(PITREX_BUILD)/sdk/%.o: %.S $(PITREX_FLAGS_FILE)
	@mkdir -p $(dir $@)
	$(PITREX_CC) $(PITREX_ARCH) $(PITREX_DEFS) $(PITREX_INCS) $(PITREX_BM_INCS) -D__ASSEMBLY__ -c $< -o $@

$(PITREX_BUILD)/sdk/%.o: %.c $(PITREX_FLAGS_FILE)
	@mkdir -p $(dir $@)
	$(PITREX_CC) $(PITREX_SDK_CFLAGS) -MMD -MP -c $< -o $@

# The game keeps its own `main`; start-up (uvm2_pitrex_main.c) wraps it, as on the .um2.
$(PITREX_BUILD)/game/%.o: %.c $(PITREX_FLAGS_FILE)
	@mkdir -p $(dir $@)
	$(PITREX_CC) $(PITREX_GAME_CFLAGS) $(PITREX_GAME_DEFS) -Dmain=uvm2_game_main -MMD -MP -c $< -o $@

define PITREX_CXX_RULE
$(PITREX_BUILD)/game/$(basename $(notdir $(1))).o: $(1) $(PITREX_FLAGS_FILE)
	@mkdir -p $$(dir $$@)
	$(PITREX_CXX) $(filter-out -std=%,$(PITREX_GAME_CFLAGS)) $(PITREX_GAME_DEFS) -include $(PITREX_DIR)/compat/cxx_main.h \
	    -Dmain=uvm2_game_main -MMD -MP -c $$< -o $$@
endef
$(foreach s,$(PITREX_CXX_SRCS),$(eval $(call PITREX_CXX_RULE,$(s))))

# THE HEADERS ARE DEPENDENCIES TOO (the compiler writes them next to each object): a header
# that changed and an object that did not is an image that links without a word and runs the
# old code.
-include $(PITREX_SDK_OBJS:.o=.d) $(PITREX_GAME_OBJS:.o=.d) $(PITREX_CXX_OBJS:.o=.d)

# cargo decides by itself whether anything changed; it runs in the crate's folder so rustup
# reads the SDK's rust-toolchain.toml.
.PHONY: pitrex-draw-lib
pitrex-draw-lib:
	cd $(VECTREX_DRAW_CABI) && cargo build --release -q --target $(PITREX_RUST_TARGET)
$(PITREX_DRAW_LIB): pitrex-draw-lib

# PITREX_LDFLAGS: a project's own linker options (a port that wraps one of the bridge's
# calls, say).
# Link order is the baremetal tree's own (Rules.mk): the heap script is a positional
# argument and goes before -lbaremetal. No libvectrexInterface: its drawing layer is the
# one this SDK replaces.
$(PITREX_BUILD)/$(UVM2_NAME).elf: $(PITREX_ENTRY_OBJ) $(PITREX_SDK_OBJS) $(PITREX_GAME_OBJS) $(PITREX_CXX_OBJS) $(PITREX_DRAW_LIB)
	$(PITREX_CC) -fuse-ld=bfd -nostartfiles $(PITREX_ARCH) -Wl,--gc-sections $(PITREX_LDFLAGS) \
	    -Wl,-Map,$(PITREX_BUILD)/$(UVM2_NAME).map -L$(PITREX_LIB) -o $@ \
	    $(PITREX_ENTRY_OBJ) $(PITREX_SDK_OBJS) $(PITREX_GAME_OBJS) $(PITREX_CXX_OBJS) $(PITREX_DRAW_LIB) \
	    $(PITREX_CXX_LIBS) -lm -lc $(PITREX_LIB)/linkerHeapDefBoot.ld -lbaremetal -lgcc

# The Pi's firmware jumps to the LOAD address, not to the ELF's entry point: the baremetal
# tree's entry code has to be the first thing in the image. Checked here, because an image that
# fails this boots into whatever came first and draws nothing. The Zero 2's entry starts with
# _start; the Zero's with a few instructions before it (baremetalEntry_6.S, _preStart), so what
# is checked is where its section starts: the entry object's .init, first in the link.
PITREX_LOAD_ADDR := 00008000
$(PITREX_BUILD)/kernel7l.img: $(PITREX_BUILD)/$(UVM2_NAME).elf
	@start=$$($(PITREX_TOOLCHAIN)/arm-none-eabi-objdump -h $< | awk '$$2 == ".init" { print substr($$4, length($$4) - 7) }'); \
	 entry=$$($(PITREX_NM) $< | awk '$$3 == "_start" { print $$1 }'); \
	 if [ "$$start" != "$(PITREX_LOAD_ADDR)" ] || [ -z "$$entry" ]; then \
	     echo "pitrex: the entry code (.init) is at 0x$$start, not at the load address 0x$(PITREX_LOAD_ADDR)" >&2; exit 1; fi
	$(PITREX_OBJCOPY) $< -O binary $@
	@echo "pitrex: $@ ($$(wc -c < $@ | tr -d ' ') bytes)"

# THE SAME IMAGE UNDER ITS OWN NAME, for a card with the PiTrex menu: the loader reads a
# program from piZero2/<name>.img into the address this image is linked at and starts it.
# kernel7l.img is for a card that boots straight into one game.
$(PITREX_BUILD)/$(UVM2_NAME).img: $(PITREX_BUILD)/kernel7l.img
	cp $< $@

# loader.pit: the resident image as the bootstrap reads it. The ELF starts at the usual
# 0x8000 with nothing in it up to LOADER_START; that stretch is cut off. Checked: the entry
# must be exactly at LOADER_START, and the settings block the programs read must be inside
# the 1.5 MB above it that the baremetal tree's getLoadParameter accepts.
PITREX_LOADER_ADDR  := 04000000
# what the image keeps just past its end (uvm2_pitrex_loader.h: UVM2_LOADER_KEEP_BYTES), and
# after that a copy of its .data
PITREX_LOADER_KEEP  := $(shell awk '$$2 == "UVM2_LOADER_KEEP_BYTES" { print $$3 }' $(PITREX_DIR)/uvm2_pitrex_loader.h)
PITREX_LOADER_SKIP  := $(shell printf '%d' 0x3FF8000)
PITREX_SETTINGS_MAX := $(shell printf '%d' 0x416E360)
$(PITREX_BUILD)/loader.pit: $(PITREX_BUILD)/$(UVM2_NAME).elf
	@at=$$($(PITREX_NM) $< | awk '$$3 == "uvm2_loader_entry" { print $$1 }'); \
	 if [ "$$at" != "$(PITREX_LOADER_ADDR)" ]; then \
	     echo "pitrex: the resident entry is at 0x$$at, not at 0x$(PITREX_LOADER_ADDR)" >&2; exit 1; fi
	@st=$$($(PITREX_NM) $< | awk '$$3 == "s_settings" { print $$1 }'); \
	 if [ $$((0x$$st)) -ge $(PITREX_SETTINGS_MAX) ]; then \
	     echo "pitrex: the settings block is at 0x$$st, past what the programs accept" >&2; exit 1; fi
	@end=$$($(PITREX_NM) $< | awk '$$3 == "__end__" { print $$1 }'); \
	 heap=$$($(PITREX_NM) $< | awk '$$3 == "heap_low" { print $$1 }'); \
	 data=$$(( 0x$$($(PITREX_NM) $< | awk '$$3 == "__bss_start__" { print $$1 }') - 0x$$($(PITREX_NM) $< | awk '$$3 == "__data_start" { print $$1 }') )); \
	 last=$$(( 0x$$end + 16 + $(PITREX_LOADER_KEEP) + data )); \
	 if [ $$last -ge $$((0x$$heap)) ]; then \
	     echo "pitrex: the resident image, with what it keeps past its end, reaches $$(printf '0x%X' $$last): the programs' heap starts at 0x$$heap" >&2; exit 1; fi
	$(PITREX_OBJCOPY) $< -O binary $(PITREX_BUILD)/loader.full
	dd if=$(PITREX_BUILD)/loader.full of=$@ bs=4096 skip=$$(( $(PITREX_LOADER_SKIP) / 4096 )) 2>/dev/null
	@echo "pitrex: $@ ($$(wc -c < $@ | tr -d ' ') bytes)"

.PHONY: pitrex
ifeq ($(PITREX_LOADER),1)
pitrex: $(PITREX_BUILD)/loader.pit
else
pitrex: $(PITREX_BUILD)/kernel7l.img $(PITREX_BUILD)/$(UVM2_NAME).img
endif
