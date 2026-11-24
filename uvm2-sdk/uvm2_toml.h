/* uvm2_toml.h — the part of TOML a package manifest uses (tools/uvp.py writes them), read on
 * the boards.
 *
 *   # comments            [table]            [[array.of.tables]]
 *   key = "string"        key = 123          key = ["a", "b"]
 *
 * Nothing else: no inline tables, no dates, no multi-line strings, no dotted keys. A line this
 * reader does not understand is skipped, not an error: the manifest is ours and a newer one may
 * carry keys an older board does not know.
 *
 * The reader calls back once per value, with the table it is in ("" at the top), how many
 * [[table]] of that name came before it (0 for a plain [table]), the key, and the value as
 * text — a string unescaped, a number as written. An array calls back once per element, the
 * same key each time. Return 0 from the callback to stop. */
#ifndef UVM2_TOML_H
#define UVM2_TOML_H
#include <stdint.h>

#define UVM2_TOML_NAME_MAX  48
#define UVM2_TOML_VALUE_MAX 160

typedef int (*uvm2_toml_cb)(void *user, const char *table, int index, const char *key,
                            const char *value);

/* 0 if the whole text was read, 1 if the callback stopped it. */
int uvm2_toml_read(const char *text, uint32_t len, uvm2_toml_cb cb, void *user);

#endif
