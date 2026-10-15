# MONITOR.BIN — the UVMC2 debug monitor (monitor.c). Built on its own:
#   cmake -S . -B build -DPICO_SDK_PATH=... -DPICO_TINYUSB_PATH=... -DPICO_PLATFORM=rp2350 \
#         -DCMAKE_TOOLCHAIN_FILE=$PICO_SDK_PATH/cmake/preload/toolchains/pico_arm_cortex_m33_gcc.cmake
#   cmake --build build      (-> build/MONITOR.BIN, copied to the SD card's root)
cmake_minimum_required(VERSION 3.13)
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)
project(uvm2_monitor C CXX ASM)
pico_sdk_init()
add_executable(monitor monitor.c usb_descriptors.c)
target_include_directories(monitor PRIVATE ${CMAKE_CURRENT_LIST_DIR})
# HARDWARE spin locks, not the RP2350 default software ones: those are LDAEX/STREX on a
# variable in the image's .bss, i.e. in the PSRAM, where an exclusive store never succeeds --
# the first lock taken (inside tud_init) spun for ever. The game keeps the software ones in
# its SRAM, so the two never share a lock.
target_compile_definitions(monitor PRIVATE PICO_USE_SW_SPIN_LOCKS=0)
target_link_libraries(monitor tinyusb_device hardware_irq hardware_resets hardware_timer pico_bootrom)
# Our own script, given straight to the linker (the pico-sdk's own one is not used at all)
target_link_options(monitor PRIVATE -nostartfiles -Wl,--gc-sections -Wl,--wrap=panic -Wl,--wrap=irq_add_shared_handler -Wl,--wrap=irq_remove_handler -Wl,--wrap=irq_set_enabled -T${CMAKE_CURRENT_LIST_DIR}/monitor.ld)
add_custom_command(TARGET monitor POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:monitor> ${CMAKE_BINARY_DIR}/MONITOR.BIN)
