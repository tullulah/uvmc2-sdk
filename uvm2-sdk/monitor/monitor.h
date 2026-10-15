/* monitor.h — what monitor.c and debug.c share: the port's output, the hex parser, and the
 * debugger's hooks (debug.c). */
#ifndef UVM2_MONITOR_H
#define UVM2_MONITOR_H
#include <stdint.h>

void mon_put(const char *s);
void mon_put_hex(uint32_t v, int digits);
uint32_t mon_parse_hex(const char **p);
void mon_wake_usb(void);          /* run the USB interrupt (poll) soon: an event to send */

void dbg_init(void);
void dbg_events(void);            /* from poll: report a stop */
int dbg_command(const char *l);   /* 1 = a debugger command, answered */
#endif
