/* uvm2_uvp.h — Universal Vectrex Packages on a board's card: installed, listed, uninstalled.
 * The package's format is tools/uvp.py's. One implementation for every board, over the SDK's
 * card layer (uvm2_sd.h); what differs is where a package's files go:
 *
 *                              PiTrex (both Pi Zeros at once)     UVMC2
 *   uvmc2/<name>.um2           -                                  <name>.um2 at the top
 *   pitrex/zero2/<name>.img    games/<name>.img                   -
 *   pitrex/zero1/<name>.img    piZero1/<name>.img                 -
 *   gamedata/<path>            gamedata/<path>                    <path> at the top
 *   roms/<path>                roms/<path>                        roms/<path>
 *                              (a package made for private testing only: tools/uvp.py)
 *
 * (On a UVMC2 card a game's folders are at the top, where its games open them; on a PiTrex
 * card the SDK looks for them under gamedata/ too.)
 *
 * WHAT IS KEPT: .uvp/<id>.toml, the package's manifest followed by an [installed] table with
 * every file this put on the card. Uninstalling removes exactly those — what the game wrote
 * beside them (scores, saves) stays — and then the record. A package file is deleted once
 * installed; one that failed is left where it was, and what it had copied is taken back. */
#ifndef UVM2_UVP_H
#define UVM2_UVP_H
#include <stdint.h>

/* while a package installs: its title, bytes done and bytes in all */
typedef void (*uvm2_uvp_progress)(const char *title, uint32_t done, uint32_t total);
/* a line for a menu, for every installed package that has an executable for this board: the
 * menu group, the title, the package's id, the executable's file name, its parameter ("" for
 * none) */
typedef void (*uvm2_uvp_entry)(void *user, const char *menu, const char *title, const char *id,
                               const char *image, const char *param);

/* Every *.uvp at the card's top, installed and deleted. How many were; the last one that
 * failed is named in `failed`. */
int  uvm2_uvp_install_pending(uvm2_uvp_progress progress, char *failed, uint32_t failed_max);
void uvm2_uvp_list(uvm2_uvp_entry entry, void *user);
/* 1 if the package was installed and is now removed */
int  uvm2_uvp_uninstall(const char *id);

#endif
