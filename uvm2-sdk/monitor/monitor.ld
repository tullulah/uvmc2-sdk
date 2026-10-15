/* MONITOR.BIN: everything in the PSRAM window, at the address the game loads it to
 * (uvm2_monitor.c, UVM2_MONITOR_ADDR). The table first; no vectors, no start-up code. */
ENTRY(uvm2_monitor_entry)
/* hardware_irq is compiled in but its calls are wrapped (monitor.c): never reached */
__unhandled_user_irq = 0;
MEMORY { MON (rwx) : ORIGIN = 0x117C0000, LENGTH = 256K }
SECTIONS {
  .mon_entry : { KEEP(*(.mon_entry)) } > MON
  .text : { *(.text*) *(.time_critical*) *(.rodata*) *(.srodata*) } > MON
  .data : { *(.data*) *(.sdata*) . = ALIGN(4); __mutex_array_start = .; __mutex_array_end = .; } > MON
  .bss (NOLOAD) : { __mon_bss_start = .; *(.bss*) *(.sbss*) *(COMMON) . = ALIGN(4); __mon_bss_end = .; } > MON
  /DISCARD/ : { *(.ARM.exidx*) *(.ARM.extab*) }
}
