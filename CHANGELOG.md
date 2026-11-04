# Changelog

Releases are the repository's tags (`vX.Y.Z`); Vectrex Studio's IDE installs them by tag. Each
entry says what changes for a game built with that version.

## v1.4.12 — a game with a fixed refresh reaches it

* Fixed: a dual-core game with a fixed refresh (`UVM2_HZ`, `uvm2_set_refresh`) ran below it.
  Between frames the SDK pads the list with a filler, and it padded to the whole period at a
  nominal 1.5 MHz; but the console's bus is a little slower than that, and core 1 also reads
  the controllers between lists, so no frame ever fitted and the game ran free. Measured with
  Boxing Bugs at 38 Hz: 36.3 fps, every frame counted in `uvm2_stats.overrun`. Now core 1
  measures each frame how much list really fits in the period (the lowest over the last
  second) and the filler pads to that: the same game holds 38.0. Nothing to change in a game;
  rebuilding is enough. `uvm2_filler_cap` holds the measured figure (0 = not measured yet, or
  single core, where the period rules as before).

## v1.4.11 — the buttons can be remapped and saved; the picture fits the tube upright and turned

* New: the OPTIONS screen has a CONTROLS line. It lists the game's four buttons by name and
  the console button that does each; pick a line with the stick and press the button you want
  for it (the line that had that button takes this one's, so the four stay four). DEFAULTS
  puts them back as wired. Saved per game in `config/<NAME>.CFG` as `buttons`, four digits:
  `1234` is as wired, `2134` swaps 1 and 2. Every game with `UVM2_SETTING_OPTIONS` has it
  (`UVM2_SETTING_BUTTONS`).
* `uvm2_config_buttons("FIRE", "THRUST", ...)` names a game's four buttons for that screen;
  without it they read BUTTON 1..4. `uvm2_buttons_set()` / `uvm2_buttons_remap()` for a game
  that wants to do it itself.
* The map is applied where a game reads its buttons (`SYS_READ_BUTTONS`, `SYS_READ_BTN_RAW`),
  so combinations a game reads (button 2 + 3) follow it, and the SDK's own screens (OPTIONS,
  calibration, the HUD combo) keep the console's buttons as wired. UVMC2 only: on the debug
  cartridge the BIOS answers the buttons and does not know the map.
* New: `uvm2_view_field(x0, x1, y0, y1, aw, ah)`. A game declares the box it draws in (the
  units `v_directDraw32` takes) and the shape that box had on its own monitor (4:3; 0:0 for
  the box's own); from then on its picture is scaled and centred to be as large as the tube
  shows without changing shape: upright with its width across the tube, with SCREEN
  HORIZONTAL with its width along it. The tube is the console's calibration (WIN X, WIN Y,
  ASPECT); a console still on the stock 15500 square gets the photographed one, +-18000 x
  +-20500 (`UVM2_VIEW_REF_WIN_X/Y`). `UVM2_HAS_VIEW_FIT` says the SDK has it. UVMC2 only.

## v1.4.10 — the Vectrex Studio intro card

* New: a game with the OPTIONS screen (`UVM2_SETTING_OPTIONS`) shows the Vectrex Studio card
  before it (`uvm2_intro.c`): the logo turns twice as it fades in, holds over the wordmark with
  STUDIO and GAMES, and fades out, to the Vectrex Studio jingle on the jack; ~4.2 s, as long as
  the jingle. Any button skips to the fade. It follows the SCREEN setting. The jingle is inside
  the image (16 kHz IMA ADPCM, `uvm2_intro_data.h`, made by `tools/gen_intro.py`): ~36 KB of
  SRAM in all.
* **Optional:** `make uvm2 UVM2_INTRO=0` (or `UVM2_INTRO = 0` in the game's Makefile) compiles
  it out (`UVM2_NO_INTRO`) and gives the 36 KB back. Games without the OPTIONS screen never
  show it.
* `uvm2_jack_init()` can be called more than once: a second call restarts what the first set
  up (a cleared ring, the same state machine and DMA channel) instead of claiming new ones. The
  card starts the jack and leaves it stopped; the game's own init starts it again.

## v1.4.9 — the refresh starts at the game's own rate

* `uvm2_setting_hz` starts at the game's `UVM2_HZ` (50 when that is 0 or unset, as before)
  and holds 0 (flat out) or any rate from 20 to 100; it held only 50, 60 or 0, so a game whose
  arcade ran at another rate could not keep it once the OPTIONS screen applied the setting
  (Armor Attack, 38 Hz, played 32% fast). The OPTIONS screen's REFRESH line offers the game's
  own rate first, then 50, 60 and MAX ("REFRESH 38 HZ"). `uvm2_config_native_hz()` says which
  it is. Games built at 50 behave exactly as before.

## v1.4.8 — a standard OPTIONS screen, a brightness that reaches every vector, straighter first strokes

* New, opt-in: the SDK's OPTIONS screen (`uvm2_options.c`). A game declares
  `UVM2_SETTING_OPTIONS` in `uvm2_config_game()` (from `uvm2_game_settings`, so it is loaded
  before the game) and the runtime shows it before the game's `main`: PLAY, SCREEN
  VERTICAL/HORIZONTAL (if the game also declares `UVM2_SETTING_ROTATE`), SOUND JACK/VECTREX (if
  it declares `UVM2_SETTING_AUDIO`), REFRESH 50/60/MAX, BRIGHTNESS, CALIBRATE and START IN
  MENU/IN GAME. With START IN GAME it stays closed unless button 4 is held for a second at
  launch. Changes are saved to `config/<NAME>.CFG` as they are made; a failed save is shown.
  `uvm2_config_title()` sets the name it shows. SCREEN names how the console stands:
  HORIZONTAL is the console on its side, the drawing turned 90 degrees.
* New setting `UVM2_SETTING_BRIGHT` (`bright_pct`, percent, default 100): a gain applied in
  `uvm2_draw_intensity` to every intensity a game asks for. The calibration's BRIGHT is only the
  last intensity set, which a game that sets its own per vector overrides at once.
  `uvm2_draw_intensity_current()` still returns what the game asked for.
* `uvm2_setting_rotate` is declared in `uvm2_config.h`. The REFRESH setting is applied by the
  OPTIONS screen on its way out; games that do not use the screen apply it as before (or not).
* A jump's own rounding no longer bends the first stroke after it: the soft start subtracts
  what its priming unit really travels, and the debt a jump leaves is parked until the next jump
  instead of being paid by the first lit strokes of the object it lands on. Measured on
  kuroishi's S: deformation 290 -> 4. `JUMP_DEBT_PARKS` (default 1) turns it off.
* Debug cartridge: `memset`, `memcpy` and `memmove` (`sdk_rp2350.c`) work a word at a time
  where the pointers allow it, instead of a byte at a time; they were 4.3% of a heavy game's
  frame clearing a large buffer. UVMC2 images are unchanged (they link the pico-sdk's).

## v1.4.7 — the reset's second of waiting shows RESETTING

* After one press of the console's reset the SDK waits up to `UVM2_RESET_DOUBLE_US` (1 s) for a
  second (two presses go back to the UVMC2's menu). The game went on being drawn meanwhile, so
  a reset looked like a flicker, the game carrying on, and then a restart. Now the game's frame
  is dropped while the press is pending and the screen says RESETTING
  (`uvm2_restart_pending`, `uvm2_frame_end`). The game's code still runs; only what it draws
  is replaced.

## v1.4.6 — one press of reset restarts the game, and not into the calibration wizard

* One press of the console's reset restarted the game into the calibration wizard instead of
  the game. The reset clears the VIA (DDRB back to 0) and nothing programs it until the first
  frame's list; core 1's button refresh before that frame read through it with port B going
  nowhere, the PSG never selected and port A reading 0x00 — all buttons pressed — and
  `uvm2_config_boot_combo` took that for buttons 2+3 held. Before the first frame a DDRB of 0
  is now set to 0x9F before the read; after it, DDRB at 0 stays the reset watch's signal. From
  the UVMC2's menu it never showed: its firmware leaves the VIA set up.
* The simulator's `v_simSaveData` declares its dependency on `UTF8ToString` (`EM_JS_DEPS`).
  Without it the build linked and the first save threw "UTF8ToString is not defined", which
  in the simulator stops `main()` while the jack audio, made by the IDE, plays on. Games that
  exported it themselves as a workaround can stop.
* Opt-in, nothing changes without them: `UVM2_WORKER_STREAM`, a core-1 worker's pass fed to
  the PIO in chunks by two chained DMA channels (`uvm2_stream_finish`,
  `uvm2_stream_running`, `uvm2_frame_can_begin`), and `UVM2_HEAP_SIZE`, the C heap's
  reservation for an image that links no malloc.

## v1.4.5 — a VPy game's USB debug monitor starts

* A VPy game built with `UVM2_USB_MONITOR` (Vectrex Studio's Upload / Debug on Cartridge)
  linked the monitor and never started it. VPy images boot through `uvm2_runtime_init`
  (`UVM2_STEP_OWNS_INIT`), whose copy of `main()`'s bring-up had no PSRAM and no
  `uvm2_monitor_load()`: `MONITOR.BIN` was never read off the card, the cartridge never showed
  up on USB, and the IDE could not reach the game. That path now brings the PSRAM up under the
  same condition as `main()` and loads the monitor, in the same order as the C path.
* With dual core (every VPy game) the reset watch is on, so the PSRAM now comes up in every VPy
  image, as it already did in every C one since v1.4.0. An image built without the monitor
  gets nothing else new.
* Still missing on the VPy path, and left for their own release: the reset button's restart
  copy (`uvm2_restart_snapshot` / `uvm2_restart_keep`), the romset load and the stack paint.

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
