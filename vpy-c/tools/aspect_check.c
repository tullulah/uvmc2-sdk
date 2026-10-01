/* aspect_check — does the renderer draw a square as a square ON THE GLASS?
 *
 *   cc -O2 -w -Iinclude -I../pitrex-sim/include \
 *      tools/aspect_check.c vpy.c vpy3d.c -o /tmp/aspect_check && /tmp/aspect_check
 *
 * WHY THIS EXISTS AND WHY IT IS NOT A STROKE COUNT. A change of aspect moves
 * where a stroke goes, not how many there are, so every host counter reports
 * the same numbers before and after it. The shape needs its own witness.
 *
 * WHAT IS RIGHT, AND HOW WE KNOW. One unit is one unit on both axes: a 16000
 * square in device units is square on the glass (photographed 2026-10-01, one
 * console, examples/geometry_card in the starter kit). So a 1000 x 1000 world
 * square, face on at 3 m, must project to a deflection box with w/h = 1.000
 * under the DEFAULTS. (It said 1.333 from 2026-09-30, when the default was a
 * 4:3 stretch derived from the service manual; the photograph refuted it.)
 *
 * And the half angles must follow each axis's own clip, or a game that opens up
 * the portrait window with set_clip_xy culls what is on screen. */
#include <stdio.h>
#include <stdint.h>
#include "vpy.h"
#include "vpy3d.h"
uint8_t currentButtonState; int8_t currentJoy1X, currentJoy1Y;
static int32_t xmin,xmax,ymin,ymax;
void vectrexinit(int m){} void v_init(void){} void v_setRefresh(int h){}
void v_WaitRecal(void){}
void v_directDraw32(int32_t a,int32_t b,int32_t c,int32_t d,uint8_t e){
    if(a<xmin)xmin=a; if(a>xmax)xmax=a; if(c<xmin)xmin=c; if(c>xmax)xmax=c;
    if(b<ymin)ymin=b; if(b>ymax)ymax=b; if(d<ymin)ymin=d; if(d>ymax)ymax=d;
}
void v_setColour(uint32_t r){} uint8_t v_readButtons(void){return 0;}
void v_readJoystick1Analog(void){} uint32_t v_millis(void){return 0;}
void v_setSoundAY(uint8_t r,uint8_t v){} void v_writePSG(uint8_t r,uint8_t v){}
void v_playSample(int a,int b,int c){} void v_stopSample(int v){}
int v_samplePlaying(int v){return 0;}
static void square(void){
    xmin=ymin=1<<30; xmax=ymax=-(1<<30);
    vpy3d_look_at(0,0,0, 0,0,1000, 0,1,0);
    vpy3d_line_world(-500,-500,3000,  500,-500,3000, 100);
    vpy3d_line_world( 500,-500,3000,  500, 500,3000, 100);
    vpy3d_line_world( 500, 500,3000, -500, 500,3000, 100);
    vpy3d_line_world(-500, 500,3000, -500,-500,3000, 100);
    vpy_flush();
}
int main(void){
    int bad = 0;
    int n, d;
    vpy3d_aspect(&n, &d);
    square();
    const double w = xmax - xmin, h = ymax - ymin;
    printf("default aspect %d/%d  screen box %5.0f x %5.0f  w/h = %.3f  %s\n",
           n, d, w, h, w / h, (w / h > 0.99 && w / h < 1.01) ? "OK" : "WRONG: not square");
    bad |= !(w / h > 0.99 && w / h < 1.01);

    /* square clip: the two half angles agree; a tall window: v opens, h does not */
    const int h0 = vpy3d_h_half_angle(), v0 = vpy3d_v_half_angle();
    vpy3d_set_clip_xy(15500, 20500);
    const int h1 = vpy3d_h_half_angle(), v1 = vpy3d_v_half_angle();
    vpy3d_set_clip(15500);
    printf("half angles, square clip  h %d  v %d   tall clip  h %d  v %d  %s\n",
           h0, v0, h1, v1, (h0 == v0 && h1 == h0 && v1 > v0) ? "OK" : "WRONG");
    bad |= !(h0 == v0 && h1 == h0 && v1 > v0);
    return bad;   /* nonzero = the default is not the glass: a test, not a report */
}
