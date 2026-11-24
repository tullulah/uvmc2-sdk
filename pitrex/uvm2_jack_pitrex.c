/* uvm2_jack_pitrex.c — the SDK's line jack (uvm2_jack.h), on a board that has none.
 *
 * On the UVMC2 the arcade ports mix their sound — the recordings of the arcade's sound
 * circuits, the POKEYs — into 16-bit PCM for the cartridge's DAC. A PiTrex has no such output,
 * and until this file its answer was "no jack here": about forty ports had nowhere to send
 * their sound and were silent.
 *
 * THE CONSOLE'S OWN SPEAKER IS THE OUTPUT HERE. The same PCM goes to the SDK's sample player
 * as a stream (uvm2_smp.h), which plays it through the PSG's volume register with the writes
 * put into the draw list. It is four bits and the console's speaker, not sixteen and a line
 * output; it is also every one of those games sounding, with none of them changed.
 *
 * WHAT A GAME SEES is the jack's contract unchanged: init says 1, space says how many samples
 * to render now, write takes them at UVM2_JACK_RATE.
 *
 * A SWITCH, AND ITS HISTORY (PITREX_SPEAKER, on by default; 0 builds without it and the board
 * answers "no jack here" as before). The first night it was on (2026-11-24) the picture of
 * every game that opened the jack broke: letters with their strokes out of place, a square
 * with a notch in its side and its edges drawn as dots, while the same calibration screen in
 * the menu, which opens no jack, was clean. The cause was not the samples but the executor,
 * which took the cycles their commands ran long by out of the waits that followed — the ramp
 * finishing (uvm2_bus_pitrex.c). It gives every wait whole now.
 */
#include <stdint.h>
#include "uvm2_jack.h"
#include "uvm2_smp.h"

/* HOW FAR AHEAD THE GAME RENDERS. The list a frame builds is replayed during the NEXT frame,
 * and the stream is read while the list is built: what is waiting has to cover the frame
 * being built, and one more against a frame that takes long. More than that is only delay
 * between what is on the screen and what is heard. */
#define JACK_AHEAD_FRAMES 2u
/* the slowest refresh a game is paced at here: the Cinematronics ports, at 38 (a frame of
 * theirs at 50 would have been cut a third short, which is a click every frame) */
#define JACK_FRAME_HZ     38u
#define JACK_AHEAD        (JACK_AHEAD_FRAMES * UVM2_JACK_RATE / JACK_FRAME_HZ)
#define JACK_CHUNK        64               /* stereo is folded to mono this many at a time */

static int s_on;

int uvm2_jack_init(void)
{
#ifdef UVM2_PITREX_SPEAKER
    if (!s_on) { uvm2_smp_stream_open(UVM2_JACK_RATE); s_on = 1; }
    return 1;
#else
    return 0;
#endif
}

int uvm2_jack_ok(void) { return s_on; }

int uvm2_jack_set(int on)
{
    if (on) return uvm2_jack_init();
    if (s_on) { uvm2_smp_stream_close(); s_on = 0; }
    return 0;
}

int uvm2_jack_space(void)
{
    if (!s_on) return 0;
    const uint32_t fill = uvm2_smp_stream_fill();
    return fill >= JACK_AHEAD ? 0 : (int)(JACK_AHEAD - fill);
}

void uvm2_jack_write(const int16_t *s, int n)
{
    if (s_on) uvm2_smp_stream_write(s, n);
}

/* One speaker: the two channels' mean. */
void uvm2_jack_write_lr(const int16_t *l, const int16_t *r, int n)
{
    int16_t mono[JACK_CHUNK];
    while (s_on && n > 0) {
        const int take = n < JACK_CHUNK ? n : JACK_CHUNK;
        for (int i = 0; i < take; i++) mono[i] = (int16_t)(((int32_t)l[i] + r[i]) / 2);
        uvm2_smp_stream_write(mono, take);
        l += take; r += take; n -= take;
    }
}

/* What the cartridge's driver reports about its PIO, DMA and clock: there are none here. */
int      uvm2_jack_dma(void)     { return -1; }
int      uvm2_jack_sm(void)      { return -1; }
uint32_t uvm2_jack_clk(void)     { return 0; }
uint32_t uvm2_jack_div_q8(void)  { return 0; }
uint32_t uvm2_jack_pinctrl(void) { return 0; }
uint32_t uvm2_jack_base(void)    { return 0; }
/* The debug cart's USB headphones (UVM2_USB_JACK): not this board's. */
int      uvm2_jack_usb_playing(void) { return 0; }
int      uvm2_jack_usb_wait(void)    { return 0; }
