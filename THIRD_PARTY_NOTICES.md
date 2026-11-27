# Third-party notices

The code in this repository is under the MIT License ([LICENSE](LICENSE)),
**except** for the components below, which keep their own licenses. Each one's
license text travels with it, as its license requires.

## Code included in this repository

| component | where | copyright | license |
|---|---|---|---|
| **FatFs R0.16** | `third_party/fatfs/` | © 2022–2025 ChaN | FatFs license (BSD-style, 1 clause): [`third_party/fatfs/LICENSE.txt`](third_party/fatfs/LICENSE.txt). Only `ffconf.h` is modified; see [`third_party/fatfs/README.md`](third_party/fatfs/README.md). |
| **`pico_sdk_import.cmake`** | `uvm2-sdk/pico/` | © 2020 Raspberry Pi (Trading) Ltd. | BSD-3-Clause (the text is in the file's header) |
| **pico-sdk linker script layout** | `uvm2-sdk/memmap_psram.ld` | © 2020 Raspberry Pi (Trading) Ltd. | BSD-3-Clause. The file is derived from the pico-sdk's `memmap_*.ld` scripts; the PSRAM layout is ours. |
| **PiTrex vector font** | the glyph table in `vpy-c/vpy.c` | the PiTrex project (Malban) | ported from `pitrex/vectrex/vectorFont.i` in [pitrex-baremetal](https://github.com/malbanGit/pitrex-baremetal), which publishes no license. Credited here; not covered by this repository's MIT grant. |

## Linked at build time, not included

| component | license | notes |
|---|---|---|
| [Raspberry Pi pico-sdk](https://github.com/raspberrypi/pico-sdk) 2.2.0 | BSD-3-Clause | Every `.um2` image links it, so a distributed image must carry its notice. |
| Rust crates [`pio`](https://crates.io/crates/pio) / `pio-proc` 0.2 and their dependencies | MIT, MIT OR Apache-2.0, Unlicense OR MIT, BSD-3-Clause OR MIT OR Apache-2.0, CC0-1.0, Unicode-3.0 | Fetched by cargo. Almost all of the tree is build-time only (the `pio-proc` macro); `pio` and `num_enum` reach the binary. `cargo tree` in `vectrex-bus/` lists them. |

## Designs and values learned from others

No code from these is included, but this SDK would not exist without them:

* **Ultimate Vectrex Multicart 2** (the UVMC2) and its firmware, by Ralf & Jason.
  `uvm2-sdk/uvm2_draw.c` reproduces the command sequences of the multicart's
  `VectrexHaltCommandWriter`, the only version of them proven on real hardware.
* **Malban** (Vectorblade, VIDE, PiTrex): the calibration screens and the factory
  zero-reference value (`0x23`, `calibrationValue16`), and the idea behind the
  bus stream's timing.
* **PiTrex** (Malban, Kevin Koster, Graham Toal): `pitrex-sim/` reproduces the
  PiTrex SDK's function names and contract (`v_directDraw32`, `v_WaitRecal`, …)
  so that code written for it builds here. Declarations only; no PiTrex code.
* **MAME**: the AY-3-8910 output ladder in `uvm2-sdk/uvm2_smp.c` is derived from
  the values in MAME's `ay8910.cpp`.
* The **Vectrex BIOS** (GCE, 1982), as documented in Bruce Tomlin's disassembly,
  for the VIA programming the SDK mirrors.

## Trademarks

Vectrex is a trademark of its owner. This project is not affiliated with or
endorsed by any of the companies or people named above.
