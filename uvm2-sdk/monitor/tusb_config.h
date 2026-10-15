/* TinyUSB for the UVMC2 debug monitor: its serial port, and a second one for gdb (gdb.c) when
 * UVM2_GDB_PORT (the monitor; the SD bridge, which shares these files, has one). No OS. */
#ifndef TUSB_CONFIG_H
#define TUSB_CONFIG_H
#define CFG_TUSB_RHPORT0_MODE   (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)
#define CFG_TUD_ENABLED         1
#define CFG_TUD_ENDPOINT0_SIZE  64
#ifdef UVM2_GDB_PORT
#define CFG_TUD_CDC             2
#else
#define CFG_TUD_CDC             1
#endif
#define CFG_TUD_CDC_RX_BUFSIZE  1024
#define CFG_TUD_CDC_TX_BUFSIZE  1024
#define CFG_TUD_CDC_EP_BUFSIZE  64
#endif
