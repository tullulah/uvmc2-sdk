/* uvm2_pitrex_core.h — what uvm2_core1.c takes from the pico-sdk, on a PiTrex.
 *
 * The second core's loop is the cartridges' own (uvm2_core1.c). It needs two things from
 * its platform: microseconds, and a way to start itself on core 1.
 */
#ifndef UVM2_PITREX_CORE_H
#define UVM2_PITREX_CORE_H

#include <stdint.h>
#include "uvm2_bus.h"

static inline uint32_t time_us_32(void) { return uvm2_now_us(); }

/* The Pi's core 1 is parked by the baremetal start-up with its stacks, FPU, MMU and cycle
 * counter already set, waiting for an address (baremetalEntry_7.S). This hands it one.
 * It reports on uvm2_pitrex_bus.core1 whether the core took it. */
void multicore_launch_core1(void (*entry)(void));

/* Nothing: a core that is running the loop stays in it. There is no flash to erase here,
 * which is what the cartridges stop it for. */
static inline void multicore_reset_core1(void) { }

#endif /* UVM2_PITREX_CORE_H */
