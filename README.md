# uvmc2-sdk

The SDK for Vectrex games that run on a **UVMC2** (an RP2350 cartridge): the
runtime that lives inside a `.um2` image, the game-facing API, the beam model and
the bus stream. It is its own repository so that every project uses **one**
copy of it, as a git submodule:

| project | mounted at |
|---|---|
| [uvmc2-starter-kit](https://github.com/tullulah/uvmc2-starter-kit) | `sdk/` |
| Vectrex Studio (the IDE) | `ide/electron/resources/uvmc2-sdk/` |
| other game repositories | wherever they like; point `UVM2_SDK` at `<mount>/uvm2-sdk` |

Start with the starter kit: it has the setup script, the documentation
(`docs/`), a minimal example and a complete arcade port built on this SDK.

```
uvm2-sdk/        the runtime inside the .um2 image; uvm2.mk is what a game includes
rp2350-sdk/      the game-facing API (v_directDraw32 &c.) and linker scripts
vectrex-draw/    the beam model (Rust, no_std)
vectrex-bus/     the PIO + DMA bus stream (Rust, no_std)
vpy-c/           libvpy: shapes, sprites, text, input, sound, the stroke buffer, 3D
pitrex-sim/      the host-side contract, for desktop harnesses
tools/           package_um2.py (the .um2 header)
third_party/     FatFs (the SD card's file system; see its README for local changes)
```

## Using it

A game's Makefile includes `uvm2-sdk/uvm2.mk`; every other piece is found from
there, relative to it. The pico-sdk (2.2.0) is **not** included: set
`PICO_SDK_PATH`, or mount this SDK next to a `third_party/pico-sdk` (the starter
kit's `setup.sh` does that). A missing one stops `make uvm2` with the path it
tried.

Requirements: the Arm GNU Toolchain (`arm-none-eabi-gcc`), CMake, Python 3, and
Rust (cargo 1.78 or newer — the beam model and the bus stream are linked into
every build).

## On the cartridge, over USB

The UVMC2's USB port is enough to put games on its SD card, run them and debug them — no card
reader, no probe, no IDE. A game built with `make uvm2 UVM2_USB_MONITOR=1` carries the debug
monitor (`MONITOR.BIN` on the card, loaded into the PSRAM); from the multicart's menu, an SD
bridge is run from SRAM through BOOTSEL (the flash is never written). Both are prebuilt in
`uvm2-sdk/prebuilt/`. Needs `picotool`; macOS and Linux.

```sh
tools/uvmc2.py deploy build_uvm2/game.um2      # onto the card (with MONITOR.BIN), and running
tools/uvmc2.py log                             # the game's DEBUG_PRINT / vpy_log text
tools/uvmc2.py gdb build_uvm2/pico/game.elf    # the gdb port, and the command:
arm-none-eabi-gdb build_uvm2/pico/game.elf -ex "target remote /dev/cu.usbmodemXXX3"
```

gdb attaches with no server in between (the monitor's second USB port speaks its remote
protocol): break, step, next, finish, bt, print, set var, info registers, Ctrl-C, detach. A
held game stays on screen with a pause sign. Breakpoints work from the game's first frame on.
`uvmc2.py --help` lists the rest (`upload`, `run`, `ports`); the protocol is in its docstring
and in `uvm2-sdk/monitor/`.

## Checking it

```sh
(cd vectrex-draw && cargo test)          # one test is ignored on purpose; see the kit's CLAUDE.md
uvm2-sdk/tools/uvm2_sd_test.sh           # FAT16/FAT32/exFAT images, macOS
```

and build a game: the starter kit's `examples/hello_uvmc2` (fast) and
`game/tacscan` (exercises much more).

Everything in this repository is in English (US), including comments and commit
messages.
