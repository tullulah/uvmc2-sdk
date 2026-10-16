# Changelog

Releases are the repository's tags (`vX.Y.Z`); Vectrex Studio's IDE installs them by tag. Each
entry says what changes for a game built with that version.

## v1.4.4 — uvm2 builds on Windows

* The Rust drawing layer builds on a Windows PC. `pio-proc`'s build tools pulled the newest
  `termcolor`, `is-terminal` and `parking_lot`, which link Windows' DLLs through `windows-sys`
  0.61 (raw-dylib): on the `x86_64-pc-windows-gnu` host that needs `dlltool` and a PC
  assembler, which a Windows machine without MinGW does not have. The lockfiles now hold
  `winapi-util` 0.1.9, `is-terminal` 0.4.13 and `parking_lot` 0.12.3, on `windows-sys`
  0.52/0.59 with import libraries prebuilt. Host tools only: the image is byte for byte the
  same as v1.4.3's.
* Vectrex Studio (Windows) sets the rest up: make with busybox, CMake + Ninja, Raspberry Pi's
  prebuilt picotool, and Rust on the gnu host.

## v1.4.3 — the same image on every machine: Rust pinned, its paths remapped

* `rust-toolchain.toml` pins Rust 1.91.1 (with the `thumbv8m.main-none-eabi` target): rustup
  installs it on first use. A newer rustc linked ~3 KB more of `core` into the drawing layer,
  and return of the jedi, a few hundred bytes from full, linked on one machine and not on a
  fresh one. Raise it on purpose, with a release.
* The paths Rust embeds (panic locations) are remapped (`~`, `uvmc2-sdk`), so where the SDK
  and `~/.cargo` live does not change the image's size. Not when the build sets its own
  `RUSTFLAGS`.
* Vectrex Studio pairs it with Arm GNU Toolchain 15.2.rel1 for uvm2 builds (downloaded by the
  IDE when missing): the C side is one compiler everywhere too.

## v1.4.2 — the romset buffer in the PSRAM for every .um2

* `uvm2_pico_main.c` loads a romset in every image, so the 23 KB buffer of `uvm2_romzip.c`
  sat in the SRAM of every `make`-built `.um2`, a game with a romset or not, unless the build
  passed `UVM2_ROMZIP_IN_PSRAM=1`. return of the jedi linked where an IDE passed it and came
  out 20 KB over where one did not. It is now the PSRAM by default (`uvm2_pico.cmake`);
  `UVM2_ROMZIP_IN_PSRAM=0` keeps it in SRAM.

## v1.4.1 — work that had stayed on one machine, now in the SDK

Everything here was in use (return of the jedi, ScummVM) but only in a local checkout and in
the SDK copies installed on that machine: a game using it built there and nowhere else.

* **vpy3d scenes and scenery.** A version-2 `.vmesh` carries scenery lines and occluders,
  drawn after the faces with optional depth fog (`vpy3d_set_scenery_fog`); a `.vscene`
  places `.vmesh` pieces in a set (`vpy_scene`, `vpy3d_load_scene`, `vpy3d_draw_scene`).
  `VPY3D_MESH_MAXE` sizes the per-edge scratch to the biggest mesh, not the whole pool.
  Checked by `vpy-c/tools/scenery_check.c` and `scene_check.c`.
* **Float on the FPU.** Both builds compile for the M33's FPU (`UVM2_FPU_FLAGS`,
  `-fno-math-errno`, `-Wdouble-promotion`); README "Float". Cloth springs (`vpysoft.c`) moved
  from 64-bit integers to float: tens of milliseconds a frame to a few.
* **The jack's ring at the top of SRAM** (`UVM2_JACK_RING_TOP`): its 8 KB alignment no longer
  pads `.bss`. The refill target never asks for more than the ring holds.
* **SD:** `uvm2_sd_seek`; `uvm2_sd_dir_open` / `_next` / `_close` to list a folder;
  `uvm2_sd_sim.c`, the card as a folder in the WASM simulator.
* **Images in the PSRAM:** the Rust crate's `.bss` and the pico-sdk's spin locks stay in
  SRAM (exclusive accesses never succeed over XIP: a black screen); an image a loader started
  measures and records the clocks (`clock_get_hz()` read 0, the SD came up at the wrong rate).
* `UVM2_TARGET_INCS`: include folders for the whole target.
* **libvpy's note engine is linked only by a game that plays a note** (~420 bytes of SRAM back
  for every other game).

## v1.4.0 — over USB: upload, run, debug; the reset button

**The UVMC2's USB port uploads, runs and debugs games** — no card reader, no SWD probe.

* **Debug monitor** (`UVM2_USB_MONITOR=1`): a game built with it loads `MONITOR.BIN` off the
  card into the PSRAM and answers on USB while it runs: files onto the card, the game's
  `DEBUG_PRINT` / `vpy_log` text, and a debugger (monitor-mode debug: breakpoints, steps,
  pause, registers) — on its own port, and on a second one that speaks **gdb's remote
  protocol**: `arm-none-eabi-gdb game.elf -ex "target remote <port>"`. A held game shows its
  last picture with a pause sign. About 2 KB of SRAM; nothing without the flag.
* **SD bridge**: from the multicart's menu, run from SRAM through BOOTSEL (the flash is never
  written), it writes files to the card and shows the upload with the Vectrex Studio logo.
* **`tools/uvmc2.py`** (Python stdlib + picotool, macOS/Linux): `deploy` (onto the card and
  running), `upload`, `run`, `log`, `gdb`, `ports`. The monitor and the bridge come built in
  `uvm2-sdk/prebuilt/`.
* **The console's reset button**, on by default in dual-core games: one press starts the game
  again, two within a second go back to the menu. It reads DDRB (Ralf's test: a reset puts it
  back to 0, every frame sets it); the old T1 test misfired and "held 3 s" could never work —
  the button gives the VIA one pulse, however long it is held. `UVM2_NO_RESET_BUTTON` turns it
  off. The restart's copy of `.data` is kept in the PSRAM, so it costs no SRAM.
* **`UVM2_DEBUG_CFLAGS` reaches uvm2 builds** (it only reached the rp2350 recipe): a debug
  build is really `-Og`.
* RTT for the Vectrex Studio cartridge's debugger (`vpy_log`, `DEBUG_PRINT`, never waiting);
  `-g` in the cartridge recipe; one SD sector buffer shared by the SDK (1 KB of SRAM back);
  `uvm2_list_rewind` / `uvm2_list_clear_frame`.
* libvpy: `DRAW_VECTOR_ROT`, `PLAY_NOTE`, `DRAW_RECORDING`, `PLAY_SAMPLE` / `SAMPLE_POS`;
  core 1 can run the game's work (`UVM2_CORE0_WORKER`, a BIOS's).

Docs: the starter kit's `docs/13-usb-upload-and-debug.md`.

## Earlier

v1.1 … v1.3.4: see each tag's message (`git tag -n1`).
