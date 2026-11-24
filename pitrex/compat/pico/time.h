/* pico/time.h — on a PiTrex.
 *
 * Code written for the .um2 asks the pico-sdk for the time. There is no pico-sdk on this board;
 * what such code uses of that header is the microsecond clock, and the SDK has its own
 * (uvm2_now_us, pitrex/uvm2_pitrex_main.c). A game's source then builds here as it is.
 */
#ifndef UVM2_PITREX_COMPAT_PICO_TIME_H
#define UVM2_PITREX_COMPAT_PICO_TIME_H

#include <stdint.h>

uint32_t uvm2_now_us(void);
static inline uint32_t time_us_32(void) { return uvm2_now_us(); }

#endif
