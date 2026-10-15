#!/bin/sh
# build.sh — the UVMC2 SD bridge (bridge.c), built as a UVMC2 game through the SDK's own
# recipe (../pico, dual core) so it can draw, plus TinyUSB. Loaded through BOOTSEL into SRAM
# (picotool load -x), never written to the flash. The IDE bundles the stripped ELF as
# ide/electron/resources/uvmc2/sdbridge.elf.
#
#   PICO_SDK_PATH=... PICO_TINYUSB_PATH=... UVM2_ARM_TOOLCHAIN=.../arm-none-eabi ./build.sh <build dir>
#   -> <build dir>/sdbridge.elf
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
SDK=$(cd "$HERE/.." && pwd)
B=${1:-"$HERE/build"}
DEFS="UVM2_DUAL_CORE;UVM2_REPLAY_LAST;UVM2_RESET_BUTTON;UVM2_RESET_WATCH_ONLY;UVM2_SD_STREAM;USB_PRODUCT=\"UVMC2 SD bridge\""
cmake -S "$SDK/pico" -B "$B" -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_TOOLCHAIN_FILE="$PICO_SDK_PATH/cmake/preload/toolchains/pico_arm_cortex_m33_gcc.cmake" \
    -DPICO_SDK_PATH="$PICO_SDK_PATH" -DPICO_TINYUSB_PATH="$PICO_TINYUSB_PATH" \
    -DPICO_TOOLCHAIN_PATH="$UVM2_ARM_TOOLCHAIN" -DUVM2_SDK_DIR="$SDK" -DUVM2_NAME=sdbridge \
    -DUVM2_GAME_SRCS="$HERE/bridge.c;$SDK/monitor/usb_descriptors.c" \
    -DUVM2_GAME_INCS="$SDK/monitor;$SDK" \
    -DUVM2_GAME_DEFS="$DEFS" \
    -DUVM2_GAME_OPTS="-I$SDK/monitor" \
    -DUVM2_GAME_LIBS="tinyusb_device" > "$B.log"
cmake --build "$B" -j8
