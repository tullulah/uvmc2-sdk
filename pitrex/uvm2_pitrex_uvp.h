/* uvm2_pitrex_uvp.h — Universal Vectrex Packages on a PiTrex card: installed when the menu
 * starts, listed in its menu, uninstalled from it. The package's format is tools/uvp.py's.
 *
 * WHERE THINGS GO, for both Pi Zeros at once (one card takes both; a card installed on one
 * works on the other):
 *
 *   pitrex/zero2/<name>.img   ->  games/<name>.img
 *   pitrex/zero1/<name>.img   ->  piZero1/<name>.img
 *   gamedata/...              ->  gamedata/...
 *   uvmc2/...                     not here (the UVMC2's card)
 *
 * WHAT IS KEPT: .uvp/<id>.toml, the package's manifest followed by an [installed] table with
 * every file this put on the card. Uninstalling removes exactly those — what the game wrote
 * beside them (scores, saves) stays — and then the record. The package file itself is deleted
 * once installed. */
#ifndef UVM2_PITREX_UVP_H
#define UVM2_PITREX_UVP_H
#include <stdint.h>

#define UVP_RECORDS "/.uvp"

/* Called while a package installs: its title, bytes done and bytes in all. */
typedef void (*uvm2_uvp_progress)(const char *title, uint32_t done, uint32_t total);
/* Called with a line for the menu of every installed package that has an image for this
 * board: the menu group, the title, the package's id, the image's file name, and its
 * parameter ("" for none). */
typedef void (*uvm2_uvp_entry)(void *user, const char *menu, const char *title, const char *id,
                               const char *image, const char *param);

/* Every *.uvp at the card's top, installed and deleted. How many were; a package that failed
 * is left where it was and named in *failed (the last one). */
int  uvm2_uvp_install_pending(uvm2_uvp_progress progress, char *failed, uint32_t failed_max);
void uvm2_uvp_menu(uvm2_uvp_entry entry, void *user);
/* 1 if the package was installed and is now removed. */
int  uvm2_uvp_uninstall(const char *id);

#endif
