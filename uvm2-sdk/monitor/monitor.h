/* monitor.h — what monitor.c and debug.c share: the port's output, the hex parser, and the
 * debugger's hooks (debug.c). */
#ifndef UVM2_MONITOR_H
#define UVM2_MONITOR_H
#include <stdint.h>

void mon_put(const char *s);
void mon_put_hex(uint32_t v, int digits);
uint32_t mon_parse_hex(const char **p);
void mon_wake_usb(void);          /* run the USB interrupt (poll) soon: an event to send */
void mon_paused(int on);          /* the game held / running: its pause sign (uvm2_monitor.c) */

int mon_readable(uint32_t a, uint32_t n);    /* SRAM, scratch, flash and PSRAM windows */
int mon_writable(uint32_t a, uint32_t n);    /* SRAM and scratch */

void dbg_init(void);
void dbg_events(void);            /* from poll: report a stop */
int dbg_command(const char *l);   /* 1 = a debugger command, answered */

/* The held context and its control, for gdb.c. */
enum { DBG_RUN, DBG_BP, DBG_STEP, DBG_PAUSE, DBG_BKPT };
int dbg_halted(void);
int dbg_reason(void);
uint32_t dbg_reg(int n);                    /* 0-12 r, 13 sp, 14 lr, 15 pc, 16 xpsr */
int dbg_set_reg(int n, uint32_t v);
int dbg_bp(int set, uint32_t addr);
void dbg_pause(void);
void dbg_resume(int step);                  /* step: one instruction */
void dbg_detach(void);                      /* every breakpoint out, running */

void gdb_poll(void);              /* from poll: the gdb port (gdb.c) */
void gdb_stopped(int why);        /* from dbg_events */
#endif
