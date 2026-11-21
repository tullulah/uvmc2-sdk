/*
 * uvm2_usbaudio.h — the game's sound out of USB headphones plugged into the cartridge.
 *
 * Both cartridges wire their USB-C the same way (hardware/uvm2/uvm2.jpeg; the debug cart's
 * J1 in its netlist): VBUS on the +5V rail, so the port is powered while the console is on,
 * and D+/D- on the RP2350's own USB PHY, which can be a HOST. Apple's USB-C EarPods power up
 * on it and enumerate at full speed as a USB Audio Class 2 device (hardware/uvm2/usbprobe).
 * On the debug cart, which has no DAC, this is the only audio out there is.
 *
 * It is a minimal host of its own, not TinyUSB: TinyUSB's RP2040 host does no isochronous
 * transfers, its interrupt handler panics on an endpoint it did not open, and it is ~18 KB a
 * big game has not got in SRAM. This one enumerates, picks the device's playback alternate
 * (16-bit stereo if it has one, else 24-bit), sets the sample rate, and streams one packet a
 * millisecond as isochronous transactions on the controller's EPX.
 *
 * IT SITS UNDER uvm2_jack, chosen on the OPTIONS screen (SOUND USB, uvm2_setting_audio_usb):
 * then whatever a game writes to the jack (uvm2_jack_write/_lr, which the VPy jukebox calls)
 * comes here instead of the DAC, resampled from UVM2_JACK_RATE to the device's rate. A game
 * does not call any of this; uvm2_jack does. Headphones can come and go at any time.
 *
 * Not with UVM2_USB_MONITOR (the debug monitor owns the same port, as a device).
 */
#ifndef UVM2_USBAUDIO_H
#define UVM2_USBAUDIO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void uvm2_usbaudio_start(void);              /* the port as a host, waiting for a device */
void uvm2_usbaudio_poll(void);               /* once a frame: attach, enumerate, detach */
void uvm2_usbaudio_write_lr(const int16_t *l, const int16_t *r, int n);   /* UVM2_JACK_RATE */
int  uvm2_usbaudio_space(void);              /* samples to write now to stay ahead */
int  uvm2_usbaudio_playing(void);            /* 1 while a device takes the sound */

/* WHAT IT SEES, for a game or a test to put on the screen:
 *   0 off   1 waiting for a device   2 enumerating   3 playing   4 a device it cannot play to
 *   (not audio, no playback endpoint, a format it does not do)   5 a transfer failed */
int         uvm2_usbaudio_state(void);
const char *uvm2_usbaudio_report(void);      /* one line: state, ids, format, rate, packets */

#ifdef __cplusplus
}
#endif

#endif /* UVM2_USBAUDIO_H */
