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

## Checking it

```sh
(cd vectrex-draw && cargo test)          # one test is ignored on purpose; see the kit's CLAUDE.md
uvm2-sdk/tools/uvm2_sd_test.sh           # FAT16/FAT32/exFAT images, macOS
```

and build a game: the starter kit's `examples/hello_uvmc2` (fast) and
`game/tacscan` (exercises much more).

Everything in this repository is in English (US), including comments and commit
messages.
