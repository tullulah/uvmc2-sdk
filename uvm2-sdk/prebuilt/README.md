# Prebuilt UVMC2 USB programs

Built from this SDK so nobody needs TinyUSB to use them (the pico-sdk ships its TinyUSB
submodule empty). `tools/uvmc2.py` and the Vectrex Studio IDE use these.

| File | What | Source, how to build |
|------|------|----------------------|
| `MONITOR.BIN` | The debug monitor: a game built with `UVM2_USB_MONITOR=1` loads it off the card's root into the PSRAM. Its first USB port takes uploads and the IDE's debugger; its second speaks gdb's remote protocol. | `../monitor/` (`cmake` there; see its CMakeLists.txt) |
| `sdbridge.elf` | The SD bridge: run from SRAM through BOOTSEL when the cartridge is in the multicart menu, it writes files to the card and shows the upload on the console. | `../sdbridge/build.sh` (then `arm-none-eabi-strip`) |

Rebuild both whenever `../monitor`, `../sdbridge` or `../uvm2_sd.c` change, and copy them to
the IDE (`ide/electron/resources/uvmc2/`).
