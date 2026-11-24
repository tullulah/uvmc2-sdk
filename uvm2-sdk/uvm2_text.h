/*
 * uvm2_text.h — stroke-font text on the halted bus (SYS_PRINT_TEXT).
 */
#ifndef UVM2_TEXT_H
#define UVM2_TEXT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Draw `str` with its top-left at (x, y) in VPy units.
 * `scale` is the half-unit size the VPy stub resolves (TEXT_SIZE, default 3 →
 * x1.5); `intensity` is 0..127.  Stops at NUL or the Vectrex 0x80 terminator,
 * and after 256 characters. */
void uvm2_print_text(int x, int y, const char *str, int scale, int intensity);

/* The same text drawn the way a GAME draws it: three calls per stroke, and NO re-zero between
 * glyphs. Slower to settle and less forgiving — that is the point: it is the calibration
 * pattern for the zero reference, where uvm2_print_text's per-glyph reset would hide the
 * error. */
void uvm2_print_text_chained(int x, int y, const char *str, int scale, int intensity);

/* The same text at ANY size: positions and the font's step in 1/16 of a unit. `step_q4` is
 * one step of the glyph grid (a glyph is 6 steps tall and 7 apart), so uvm2_print_text's
 * `scale` s is step_q4 = 8 * s. For a layout made for another font, whose sizes do not fall
 * on the half units `scale` counts in. At most `max_chars` characters (0 = no limit beyond
 * uvm2_print_text's own). */
void uvm2_print_text_q4(int x_q4, int y_q4, const char *str, int step_q4, int intensity,
                        int max_chars);

#ifdef __cplusplus
}
#endif

#endif /* UVM2_TEXT_H */
