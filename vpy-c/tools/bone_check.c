/* bone_check — vpybone against geometry with known answers.
 *
 *   cc -O2 -w -Iinclude -I../pitrex-sim/include tools/bone_check.c vpybone.c vpyik.c vpy3d.c vpy.c \
 *      -lm -o /tmp/bone_check && /tmp/bone_check
 *
 * Exit status is the number of failures. <math.h> only for the reference values.
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "vpy.h"
#include "vpy3d.h"
#include "vpybone.h"

uint8_t currentButtonState; int8_t currentJoy1X, currentJoy1Y;
static int NS;
void vectrexinit(int m) { (void)m; } void v_init(void) {} void v_setRefresh(int h) { (void)h; }
void v_WaitRecal(void) {}
void v_directDraw32(int32_t a, int32_t b, int32_t c, int32_t d, uint8_t e) { (void)a; (void)b; (void)c; (void)d; (void)e; NS++; }
void v_setColour(uint32_t r) { (void)r; } uint8_t v_readButtons(void) { return 0; }
void v_readJoystick1Analog(void) {} uint32_t v_millis(void) { return 0; }
void v_setSoundAY(uint8_t r, uint8_t v) { (void)r; (void)v; } void v_writePSG(uint8_t r, uint8_t v) { (void)r; (void)v; }
void v_playSample(int a, int b, int c) { (void)a; (void)b; (void)c; } void v_stopSample(int v) { (void)v; }
int v_samplePlaying(int v) { (void)v; return 0; }

static int fails;
#define CHECK(cond, ...) do { if (cond) printf("  ok   "); else { printf("  FAIL "); fails++; } \
                              printf(__VA_ARGS__); printf("\n"); } while (0)
static double dist(const int32_t a[3], const int32_t b[3])
{ return sqrt((double)(a[0]-b[0])*(a[0]-b[0]) + (double)(a[1]-b[1])*(a[1]-b[1]) + (double)(a[2]-b[2])*(a[2]-b[2])); }
static double qlen(vpyb_quat q) { return sqrt((double)q.w*q.w + (double)q.x*q.x + (double)q.y*q.y + (double)q.z*q.z) / 16384.0; }
static double qangle(vpyb_quat q) { return 2 * acos(fabs(q.w) / 16384.0 > 1 ? 1 : fabs(q.w) / 16384.0) * 180 / M_PI; }

static vpyb_skeleton S;   /* static: a skeleton is ~2 KB */

int main(void)
{
    const int32_t O[3] = { 0, 0, 0 };
    const vpyb_quat I = vpyb_quat_identity();

    /* 1. an arm along x: shoulder at the origin, elbow 300 out, hand 200 further */
    vpyb_init(&S);
    int sh = vpyb_add(&S, -1, 0, 0, 0, 0);
    int el = vpyb_add(&S, sh, 300, 0, 0, 0);
    int ha = vpyb_add(&S, el, 200, 0, 0, 0);
    vpyb_pose(&S, O, I);
    CHECK(S.pos[ha][0] == 500 && S.pos[ha][1] == 0, "rest: the hand at (%d,%d), expected (500,0)", S.pos[ha][0], S.pos[ha][1]);
    /* shoulder up 90 degrees about z, elbow a further 90: the hand comes back over */
    S.rot[sh] = vpyb_quat_axis(0, 0, 1, 1024);
    S.rot[el] = vpyb_quat_axis(0, 0, 1, 1024);
    vpyb_pose(&S, O, I);
    CHECK(abs(S.pos[el][0]) <= 1 && abs(S.pos[el][1] - 300) <= 1 && abs(S.pos[ha][0] + 200) <= 1 && abs(S.pos[ha][1] - 300) <= 1,
          "shoulder 90 + elbow 90: elbow (%d,%d) expected (0,300), hand (%d,%d) expected (-200,300)",
          S.pos[el][0], S.pos[el][1], S.pos[ha][0], S.pos[ha][1]);
    /* 30 and 45 degrees, against the trigonometry */
    S.rot[sh] = vpyb_quat_axis(0, 0, 1, 4096 * 30 / 360);
    S.rot[el] = vpyb_quat_axis(0, 0, 1, 512);
    vpyb_pose(&S, O, I);
    const double ex = 300 * cos(30 * M_PI / 180) + 200 * cos(75 * M_PI / 180), ey = 300 * sin(30 * M_PI / 180) + 200 * sin(75 * M_PI / 180);
    CHECK(fabs(S.pos[ha][0] - ex) <= 2 && fabs(S.pos[ha][1] - ey) <= 2, "30 + 45 degrees: hand (%d,%d), trigonometry (%.1f,%.1f)", S.pos[ha][0], S.pos[ha][1], ex, ey);

    /* 2. a child follows its parent: turn the whole skeleton and the hand turns with it */
    vpyb_pose(&S, O, vpyb_quat_axis(0, 1, 0, 1024));
    CHECK(fabs(S.pos[ha][2] + ex) <= 2 && fabs(S.pos[ha][1] - ey) <= 2, "the skeleton turned 90 about y: the hand goes to z=%d (%.0f)", S.pos[ha][2], -ex);
    const vpy_xf hx = vpyb_xf(&S, ha);
    CHECK(hx.t[2] == S.pos[ha][2] && abs(hx.m[0] * hx.m[0] + hx.m[3] * hx.m[3] + hx.m[6] * hx.m[6] - (1 << 28)) < (1 << 20),
          "the hand's vpy_xf is at its joint and its matrix is a rotation");

    /* 3. a clip: elbow 0 at frame 0, 90 degrees at 10, back to 0 at 20, looping over 20 */
    static const vpyb_key EL_KEYS[] = { { 0, { 16384, 0, 0, 0 } }, { 10, { 11585, 0, 0, 11585 } }, { 20, { 16384, 0, 0, 0 } } };
    static vpyb_track TR[3];
    TR[el].keys = EL_KEYS; TR[el].n = 3;
    const vpyb_clip bend = { TR, 3, 20, 1 };
    vpyb_quat q10 = vpyb_sample(&TR[el], 10 << 8, 20, 1), q5 = vpyb_sample(&TR[el], 5 << 8, 20, 1);
    CHECK(q10.w == 11585 && q10.z == 11585, "at a key the clip gives the key (w=%d z=%d)", q10.w, q10.z);
    CHECK(fabs(qangle(q5) - 45) < 0.5 && fabs(qlen(q5) - 1) < 0.001, "halfway it is between: %.2f degrees (45), length %.4f", qangle(q5), qlen(q5));
    vpyb_quat q25 = vpyb_sample(&TR[el], 25 << 8, 20, 1);
    CHECK(fabs(qangle(q25) - 45) < 0.5, "looping: frame 25 is frame 5 again (%.2f degrees)", qangle(q25));
    vpyb_quat qh = vpyb_sample(&TR[el], 30 << 8, 20, 0);
    CHECK(qh.w == 16384, "not looping: past the end it holds the last key");
    vpyb_init(&S); sh = vpyb_add(&S, -1, 0, 0, 0, 0); el = vpyb_add(&S, sh, 300, 0, 0, 0); ha = vpyb_add(&S, el, 200, 0, 0, 0);
    vpyb_apply(&S, &bend, 10 << 8); vpyb_pose(&S, O, I);
    CHECK(abs(S.pos[ha][0] - 300) <= 1 && abs(S.pos[ha][1] - 200) <= 1, "applied at frame 10 the hand points up: (%d,%d)", S.pos[ha][0], S.pos[ha][1]);

    /* 4. blending: weight 0 is clip A, 256 is clip B, 128 between */
    static const vpyb_key B_KEYS[] = { { 0, { 0, 0, 0, 16384 } } };     /* 180 degrees, held */
    static vpyb_track TB[3];
    TB[el].keys = B_KEYS; TB[el].n = 1;
    const vpyb_clip flip = { TB, 3, 1, 1 };
    vpyb_apply_blend(&S, &bend, 10 << 8, &flip, 0, 0);
    const vpyb_quat w0 = S.rot[el];
    vpyb_apply_blend(&S, &bend, 10 << 8, &flip, 0, 256);
    const vpyb_quat w1 = S.rot[el];
    vpyb_apply_blend(&S, &bend, 10 << 8, &flip, 0, 128);
    const double a5 = qangle(S.rot[el]);
    CHECK(w0.w == 11585 && w1.z == 16384 && a5 > 90 && a5 < 180, "blend: 0 is A (90), 256 is B (180), 128 between (%.1f)", a5);

    /* 5. a leg by IK: hip, knee, ankle; the ankle onto a reachable target */
    vpyb_init(&S);
    int hip = vpyb_add(&S, -1, 0, 1000, 0, 0);
    int knee = vpyb_add(&S, hip, 0, -450, 0, 0);
    int ankle = vpyb_add(&S, knee, 0, -400, 0, 0);
    vpyb_pose(&S, O, I);
    const int32_t target[3] = { 150, 300, 80 }, pole[3] = { 0, 600, 500 };
    const int r = vpyb_ik(&S, hip, target, pole);
    CHECK(r == 1 && dist(S.pos[ankle], target) <= 3, "IK: the ankle onto (150,300,80): at (%d,%d,%d), %.1f off", S.pos[ankle][0], S.pos[ankle][1], S.pos[ankle][2], dist(S.pos[ankle], target));
    CHECK(fabs(dist(S.pos[hip], S.pos[knee]) - 450) <= 2 && fabs(dist(S.pos[knee], S.pos[ankle]) - 400) <= 2 && S.pos[knee][2] > 0,
          "the bones keep their lengths (%.0f, %.0f) and the knee bends towards the pole (z=%d)", dist(S.pos[hip], S.pos[knee]), dist(S.pos[knee], S.pos[ankle]), S.pos[knee][2]);
    const int32_t far_t[3] = { 0, -2000, 0 };
    const int r2 = vpyb_ik(&S, hip, far_t, pole);
    CHECK(r2 == 0 && vpyb_stats()->ik_stretched == 1 && fabs(dist(S.pos[hip], S.pos[ankle]) - 850) <= 3, "out of reach: stretched straight (%.0f of 850), counted", dist(S.pos[hip], S.pos[ankle]));
    CHECK(vpyb_ik(&S, knee, target, pole) == -1 && vpyb_stats()->ik_refused == 1, "IK on a bone without two below it is refused and counted");

    /* 6. refusals, drawing, determinism */
    vpyb_init(&S); vpyb_reset_stats();
    int made = 0; for (int i = 0; i < VPYB_MAX_BONES + 2; i++) made += vpyb_add(&S, i ? 0 : -1, 10, 0, 0, 0) >= 0;
    CHECK(made == VPYB_MAX_BONES && vpyb_stats()->refused == 2 && vpyb_add(&S, 99, 0, 0, 0, 0) < 0,
          "%d bones of %d asked, the rest refused and counted (%u); a missing parent refused too", made, VPYB_MAX_BONES + 2, vpyb_stats()->refused);
    vpyb_init(&S); sh = vpyb_add(&S, -1, 0, 0, 0, 0); el = vpyb_add(&S, sh, 300, 0, 0, 0); ha = vpyb_add(&S, el, 200, 0, 0, 0);
    vpyb_pose(&S, O, I);
    vpy3d_look_at(250, 0, -2000, 250, 0, 0, 0, 1, 0); NS = 0;
    const int nd = vpyb_draw(&S, 100, VPYB_DRAW_LINES);
    vpy_flush();
    CHECK(nd == 2 && NS >= 2, "lines: 2 bones with a parent drawn, %d strokes out", NS);
    int32_t snap[2];
    for (int run = 0; run < 2; run++) {
        vpyb_apply(&S, &bend, 7 * 256 + 100); vpyb_pose(&S, O, vpyb_quat_axis(1, 2, 3, 700));
        snap[run] = S.pos[ha][0] * 31 + S.pos[ha][1] * 7 + S.pos[ha][2];
    }
    CHECK(snap[0] == snap[1], "deterministic: the same pose twice");

    printf("%s (%d failed)\n", fails ? "FAILED" : "ALL OK", fails);
    return fails;
}
