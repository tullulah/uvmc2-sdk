/* phys_check — what vpyphys gets right, as tests with known answers.
 *
 *   cc -O2 -Iinclude tools/phys_check.c vpyphys.c -lm -o /tmp/phys_check && /tmp/phys_check
 *
 * Every claim in vpyphys.h's "WHAT IT GETS RIGHT" is a line here, against a
 * formula (free fall, braking distance, momentum) or a behaviour (a stack
 * holds and sleeps, a sleeper wakes when its support goes). Exit status is the
 * number of failures, so a change to the solver that breaks one is loud.
 *
 * <math.h> is used HERE only, for the reference formulas; vpyphys.c is integer.
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "vpyphys.h"
/* vpyphys turns bodies with vpy_sin_q14 from vpy.c; the host has <math.h> */
int vpy_sin_q14(int a) { return (int)lround(16384.0 * sin(a * 2 * M_PI / 4096)); }
int vpy_cos_q14(int a) { return (int)lround(16384.0 * cos(a * 2 * M_PI / 4096)); }
static int fails;
#define CHECK(cond, ...) do { if (cond) printf("  ok   "); else { printf("  FAIL "); fails++; } printf(__VA_ARGS__); printf("\n"); } while (0)

int main(void){
  int32_t x,y,z,vx,vy,vz;
  /* 1. free fall vs y = h - g t^2/2, before it reaches the floor */
  vpyp_reset(); vpyp_set_gravity(0,-9800,0);
  int s = vpyp_add_sphere(0,100000,0,50,1);
  for (int i=0;i<25;i++) vpyp_step();
  vpyp_position(s,&x,&y,&z); vpyp_velocity(s,&vx,&vy,&vz);
  double t=25/50.0, ya=100000-0.5*9800*t*t, yd=100000-9800.0/2500*25*26/2;
  CHECK(fabs(y-yd) <= 2, "free fall 0.5 s: y=%d, semi-implicit Euler exact %.0f (continuous %.0f: half a step ahead, by design)", y, yd, ya);
  CHECK(abs(vy+4900) <= 2, "velocity exact: vy=%d (-4900)", vy);

  /* 2. dropped ball bounces, comes to rest on the floor, sleeps */
  vpyp_reset(); vpyp_set_gravity(0,-9800,0); vpyp_set_floor(1,0,128,128);
  s = vpyp_add_sphere(0,1000,0,50,1); vpyp_set_material(s,160,128);
  int peak2=0, landed=0; int32_t prevvy=0;
  for (int i=0;i<500;i++){ vpyp_step(); vpyp_velocity(s,&vx,&vy,&vz); vpyp_position(s,&x,&y,&z);
    if (prevvy<0 && vy>0) { landed++; }
    if (landed==1 && y>peak2) peak2=y; prevvy=vy; }
  vpyp_position(s,&x,&y,&z);
  CHECK(landed>=2, "ball bounced %d times", landed);
  CHECK(peak2 > 150 && peak2 < 1000, "first bounce peak %d (from 1000, e=160/256)", peak2);
  CHECK(abs(y-50) <= 2, "rests on the floor: y=%d (radius 50)", y);
  CHECK(vpyp_sleeping(s), "asleep after 10 s");

  /* 3. a stack of 5 boxes holds and sleeps */
  vpyp_reset(); vpyp_set_gravity(0,-9800,0); vpyp_set_floor(1,0,0,200);
  int bx[5]; for (int i=0;i<5;i++) bx[i]=vpyp_add_box(0,100+i*200+i*5,0,100,100,100,1);
  for (int i=0;i<500;i++) vpyp_step();
  /* boxes turn now, so the solver's order leaves a little sideways drift: up
   * to 20 units on a 1000-unit column, every box still upright */
  int ok=1, slept=1, drift=0; int32_t m[9];
  for (int i=0;i<5;i++){ vpyp_position(bx[i],&x,&y,&z); vpyp_rotation(bx[i],m);
    if (abs(y-(100+i*200))>4 || m[4] < 16300) ok=0;
    if (abs(x)>drift) drift=abs(x); if (abs(z)>drift) drift=abs(z);
    if(!vpyp_sleeping(bx[i])) slept=0; }
  vpyp_position(bx[4],&x,&y,&z);
  CHECK(ok && drift <= 20, "stack of 5 holds upright: top at y=%d (expected 900), drift %d", y, drift);
  CHECK(slept, "whole stack asleep; awake=%u", vpyp_stats()->awake);

  /* 4. pull the bottom box out: everything above wakes and falls */
  vpyp_remove(bx[0]);
  for (int i=0;i<200;i++) vpyp_step();
  vpyp_position(bx[1],&x,&y,&z);
  CHECK(abs(y-100) <= 3, "bottom removed: next box fell to the floor, y=%d", y);

  /* 5. sliding box stops at v^2/(2 mu g) */
  vpyp_reset(); vpyp_set_gravity(0,-9800,0); vpyp_set_floor(1,0,0,128);
  int b = vpyp_add_box(0,100,0,100,100,100,1); vpyp_set_material(b,0,128);
  for (int i=0;i<10;i++) vpyp_step();
  vpyp_set_velocity(b,2000,0,0);
  for (int i=0;i<300;i++) vpyp_step();
  vpyp_position(b,&x,&y,&z);
  double mu=128/256.0, da=2000.0*2000/(2*mu*9800);
  CHECK(fabs(x-da) < 0.1*da, "slide with mu=0.5 from 2000 u/s: stopped at x=%d, theory %.0f", x, da);

  /* 6. head-on elastic collision of equal spheres swaps velocities */
  vpyp_reset();
  int a1=vpyp_add_sphere(-500,0,0,50,1), a2=vpyp_add_sphere(500,0,0,50,1);
  vpyp_set_material(a1,256,0); vpyp_set_material(a2,256,0);
  vpyp_set_velocity(a1,1000,0,0); vpyp_set_velocity(a2,-1000,0,0);
  for (int i=0;i<60;i++) vpyp_step();
  int32_t v1,v2; vpyp_velocity(a1,&v1,&vy,&vz); vpyp_velocity(a2,&v2,&vy,&vz);
  CHECK(abs(v1+1000)<30 && abs(v2-1000)<30, "elastic head-on: v1=%d v2=%d (expected -1000, 1000)", v1, v2);

  /* 7. heavy vs light: momentum conserved */
  vpyp_reset();
  a1=vpyp_add_sphere(-500,0,0,50,4); a2=vpyp_add_sphere(500,0,0,50,1);
  vpyp_set_material(a1,0,0); vpyp_set_material(a2,0,0);
  vpyp_set_velocity(a1,1000,0,0);
  for (int i=0;i<60;i++) vpyp_step();
  vpyp_velocity(a1,&v1,&vy,&vz); vpyp_velocity(a2,&v2,&vy,&vz);
  CHECK(abs(4*v1+v2-4000)<40, "momentum 4*1000: after %d (v1=%d v2=%d)", 4*v1+v2, v1, v2);

  /* 8. determinism: same scene twice, same result */
  int32_t r[2][3];
  for (int run=0;run<2;run++){ vpyp_reset(); vpyp_set_gravity(0,-9800,0); vpyp_set_floor(1,0,100,150);
    for (int i=0;i<20;i++){ int id=(i&1)?vpyp_add_box(i*37-300,300+i*150,(i*53)%200-100,60,60,60,1+i%3):vpyp_add_sphere(i*41-350,300+i*150,(i*29)%200-100,60,1+i%2); vpyp_set_velocity(id,(i*97)%400-200,0,(i*61)%400-200);}
    for (int i=0;i<400;i++) vpyp_step();
    vpyp_position(13,&r[run][0],&r[run][1],&r[run][2]); }
  CHECK(r[0][0]==r[1][0]&&r[0][1]==r[1][1]&&r[0][2]==r[1][2], "deterministic: body 13 at (%d,%d,%d) both runs", r[0][0],r[0][1],r[0][2]);
  printf("        20-body pile: pairs/step %u, contacts %u, awake %u after 8 s\n", vpyp_stats()->pairs, vpyp_stats()->contacts, vpyp_stats()->awake);

  /* 9. rays */
  vpyp_reset(); vpyp_set_floor(1,0,0,0);
  int tgt=vpyp_add_box(0,500,2000,200,200,200,1), sph=vpyp_add_sphere(0,500,1000,100,1);
  vpyp_hit h; int id=vpyp_raycast(0,500,0, 0,0,1, 10000, 0xFF, &h);
  CHECK(id==sph && abs(h.z-900)<=1 && h.nz< -16000, "ray hits the near sphere at z=%d (900), normal z %d", h.z, h.nz);
  vpyp_remove(sph); id=vpyp_raycast(0,500,0, 0,0,1, 10000, 0xFF, &h);
  CHECK(id==tgt && abs(h.z-1800)<=1, "then the box at z=%d (1800)", h.z);
  id=vpyp_raycast(0,500,0, 0,-1,1, 10000, 0xFF, &h);
  CHECK(id==VPYP_FLOOR && abs(h.z-500)<=1, "down at 45 deg: floor at z=%d (500)", h.z);

  /* 10. the table full is counted */
  vpyp_reset(); int last=0; for (int i=0;i<VPYP_MAX_BODIES+3;i++) last=vpyp_add_sphere(i*300,0,0,10,1);
  CHECK(last==VPYP_NONE && vpyp_stats()->refused==3, "table full: refused=%u", vpyp_stats()->refused);

  /* 11. contacts report how hard: a drop hits harder than resting */
  vpyp_reset(); vpyp_set_gravity(0,-9800,0); vpyp_set_floor(1,0,0,128);
  s=vpyp_add_sphere(0,2000,0,50,1); int32_t hit=0, rest=0;
  for (int i=0;i<200;i++){ vpyp_step(); for (int k=0;k<vpyp_contact_count();k++){ const vpyp_contact*c=vpyp_contact_get(k); if (i<60 && c->impulse>hit) hit=c->impulse; if (i>150) rest=c->impulse; } }
  CHECK(hit > 10*rest && hit > 5000, "impact impulse %d vs resting %d (fall speed ~%d u/s)", hit, rest, (int)sqrt(2*9800*1950.0));

  /* 12. a ball thrown at a sleeping stack wakes it and moves it */
  vpyp_reset(); vpyp_set_gravity(0,-9800,0); vpyp_set_floor(1,0,0,200);
  for (int i=0;i<3;i++) bx[i]=vpyp_add_box(0,100+i*200,0,100,100,100,1);
  for (int i=0;i<200;i++) vpyp_step();
  int slept3 = vpyp_sleeping(bx[0])&&vpyp_sleeping(bx[1])&&vpyp_sleeping(bx[2]);
  int ball = vpyp_add_sphere(-1000,300,0,60,2); vpyp_set_velocity(ball,4000,0,0);
  for (int i=0;i<100;i++) vpyp_step();
  /* the ball drops on its way and hits the BOTTOM box: that one goes +x, and
   * the two above, their support gone, come down */
  int32_t x0,y0,z0, y2; vpyp_position(bx[0],&x0,&y0,&z0); vpyp_position(bx[2],&x,&y2,&z);
  CHECK(slept3 && x0 > 50 && y2 < 300, "thrown ball knocks the bottom box out (x=%d) and the stack falls (top at y=%d)", x0, y2);

  /* 15. a box dropped on an edge ends on a face */
  vpyp_reset(); vpyp_set_gravity(0,-9800,0); vpyp_set_floor(1,0,0,200);
  b = vpyp_add_box(0,400,0,100,100,100,1); vpyp_set_rotation(b,0,0,1,4096*30/360);
  for (int i=0;i<400;i++) vpyp_step();
  vpyp_position(b,&x,&y,&z); vpyp_rotation(b,m);
  int flat = abs(m[1]) < 300 || abs(m[1]) > 16080;
  CHECK(flat && abs(y-100) <= 4 && vpyp_sleeping(b), "box dropped on an edge lands on a face: y=%d, m01=%d, asleep %d", y, m[1], vpyp_sleeping(b));

  /* 16. a kick off-centre turns a box the right way */
  vpyp_reset();
  b = vpyp_add_box(0,0,0,100,100,100,1);
  vpyp_apply_impulse_at(b, 1000,0,0, 0,100,0);   /* +x at the top: spins about -z */
  vpyp_step(); int32_t wx,wy,wz; vpyp_spin(b,&wx,&wy,&wz);
  CHECK(wz < -100 && wx == 0 && wy == 0, "push +x at the top spins it about -z: spin z %d", wz);

  /* 17. a ball sliding onto rough floor starts to roll, w = -v/r, then stops */
  vpyp_reset(); vpyp_set_gravity(0,-9800,0); vpyp_set_floor(1,0,0,200);
  s = vpyp_add_sphere(0,100,0,100,1); vpyp_set_material(s,0,200);
  for (int i=0;i<5;i++) vpyp_step();
  vpyp_set_velocity(s,1000,0,0);
  for (int i=0;i<20;i++) vpyp_step();
  vpyp_velocity(s,&vx,&vy,&vz); vpyp_spin(s,&wx,&wy,&wz);
  const double roll = -vx / 100.0 * 4096 / (2*M_PI);
  CHECK(fabs(wz - roll) < 0.1*fabs(roll) && vx < 1000 && vx > 500, "rolling: v=%d, spin %d, -v/r = %.0f (4096ths/s)", vx, wz, roll);
  for (int i=0;i<1500;i++) vpyp_step();
  CHECK(vpyp_sleeping(s), "the rolling ball stops and sleeps within 30 s");

  /* 18. a tall box pushed at the top tips over */
  vpyp_reset(); vpyp_set_gravity(0,-9800,0); vpyp_set_floor(1,0,0,300);
  b = vpyp_add_box(0,300,0,50,300,50,1);
  for (int i=0;i<10;i++) vpyp_step();
  vpyp_apply_impulse_at(b, 400,0,0, 0,580,0);
  for (int i=0;i<300;i++) vpyp_step();
  vpyp_position(b,&x,&y,&z);
  CHECK(y < 80, "a tall box pushed at the top tips over: centre now at y=%d (was 300)", y);

  /* 13b. take out one of two supports from under a sleeping box: it tips */
  vpyp_reset(); vpyp_set_gravity(0,-9800,0); vpyp_set_floor(1,0,0,300);
  int l1=vpyp_add_box(-101,100,0,100,100,100,1), l2=vpyp_add_box(101,100,0,100,100,100,1);
  int topb=vpyp_add_box(0,300,0,100,100,100,1); (void)l1;
  for (int i=0;i<200;i++) vpyp_step();
  int slept_top = vpyp_sleeping(topb);
  vpyp_remove(l2);
  for (int i=0;i<200;i++) vpyp_step();
  vpyp_position(topb,&x,&y,&z); vpyp_rotation(topb,m);
  CHECK(slept_top && (y < 250 || m[4] < 15000), "a sleeping box on two supports, one removed: it comes down (y=%d, m11=%d)", y, m[4]);

  /* 13c. a blast throws everything near it away, light and heavy alike, and
   * leaves what is out of reach alone */
  vpyp_reset();
  int near1=vpyp_add_box(300,0,0,50,50,50,1), near2=vpyp_add_box(-300,0,0,50,50,50,8), far1=vpyp_add_sphere(0,0,2000,50,1);
  const int moved = vpyp_blast(0,0,0, 1000, 2000, 0xFF);
  int32_t v1x,v2x,dummy,v3x;
  vpyp_velocity(near1,&v1x,&dummy,&dummy); vpyp_velocity(near2,&v2x,&dummy,&dummy); vpyp_velocity(far1,&v3x,&dummy,&dummy);
  CHECK(moved==2 && v1x > 1300 && v1x < 1500 && v2x < -1300 && v2x > -1500 && v3x == 0,
        "blast r=1000 at 2000 u/s: bodies at 300 fly off at %d and %d (expected +-1400, mass 1 and 8), the one at 2000 stays", v1x, v2x);

  /* 13. tunnelling: 600 u/s per step through a 20-unit wall, then with 8 substeps */
  for (int sub=1; sub<=8; sub*=8) {
    vpyp_reset(); vpyp_set_substeps(sub);
    vpyp_add_box(0,0,0,10,1000,1000,0);
    int f=vpyp_add_sphere(-400,0,0,20,1); vpyp_set_velocity(f,30000,0,0);
    for (int i=0;i<10;i++) vpyp_step();
    vpyp_position(f,&x,&y,&z);
    if (sub==1) printf("        substeps 1: fast sphere ends at x=%d (%s)\n", x, x>0?"went through":"stopped");
    else CHECK(x < 0, "substeps 8: fast sphere stopped by the wall at x=%d", x);
  }

  /* 14. masks: a shot in another bit passes through its shooter */
  vpyp_reset();
  int shooter=vpyp_add_box(0,0,0,100,100,100,0), shot=vpyp_add_sphere(0,0,0,20,1);
  vpyp_set_mask(shooter,1); vpyp_set_mask(shot,2); vpyp_set_velocity(shot,1000,0,0);
  for (int i=0;i<5;i++) vpyp_step();
  vpyp_position(shot,&x,&y,&z);
  CHECK(x==100 && vpyp_contact_count()==0, "shot inside its shooter, other mask: flew to x=%d untouched", x);

  /* 13. edge across edge: a box dropped crosswise onto another's ridge, both
   *     turned 45 degrees, rests on the ridge (edges touch at y = 2 sqrt2 h)
   *     instead of sinking into it */
  { vpyp_reset(); vpyp_set_gravity(0,-9800,0);
    const int h = 100; const double rest = 2*sqrt(2.0)*h;
    int A = vpyp_add_box(0,0,0,h,h,h,0); vpyp_set_rotation(A,0,0,1,512);
    int B = vpyp_add_box(0,400,0,h,h,h,1); vpyp_set_rotation(B,1,0,0,512);
    int worst = 0;
    for (int i=0;i<40;i++){ vpyp_step(); vpyp_position(B,&x,&y,&z);
      const int sink = (int)lround(rest - y); if (sink > worst) worst = sink; }
    CHECK(worst <= 3, "box crosswise on a ridge: sinks at most %d units (rest at y=%.0f)", worst, rest);
    vpyp_position(B,&x,&y,&z);
    CHECK(y > rest - 5, "and is held there: y=%d after 0.8 s", y);
  }

  /* 14. convex hulls */
  { static const int16_t CUBE[8*3] = { -100,-100,-100, 100,-100,-100, -100,100,-100, 100,100,-100,
                                       -100,-100,100, 100,-100,100, -100,100,100, 100,100,100 };
    static const uint8_t CUBE_F[] = { 4,0,2,6,4, 4,1,3,7,5, 4,0,1,5,4, 4,2,3,7,6, 4,0,1,3,2, 4,4,5,7,6, 0 };
    /* a hull made as a cube does what the box does */
    int32_t yb[2];
    for (int kind=0; kind<2; kind++) {
      vpyp_reset(); vpyp_set_gravity(0,-9800,0); vpyp_set_floor(1,0,64,128);
      const int sh = vpyp_hull_shape(CUBE, 8, CUBE_F);
      int top=-1;
      for (int i=0;i<4;i++) top = kind ? vpyp_add_hull(0,100+i*200,0,sh,1) : vpyp_add_box(0,100+i*200,0,100,100,100,1);
      for (int i=0;i<400;i++) vpyp_step();
      vpyp_position(top,&x,&y,&z); yb[kind]=y;
      if (kind) CHECK(abs(y-700)<=6 && abs(x)<=6 && vpyp_stats()->awake==0, "4 hull cubes stack and sleep: top at (%d,%d), expected (0,700); awake %u", x, y, vpyp_stats()->awake);
    }
    CHECK(abs(yb[0]-yb[1])<=3, "the same stack of boxes: top at y=%d against the hulls' %d", yb[0], yb[1]);

    /* a pyramid (square base 200, apex 200 above the base) dropped tipped ends on its base */
    vpyp_reset(); vpyp_set_gravity(0,-9800,0); vpyp_set_floor(1,0,64,128);
    static const int16_t PYR[5*3] = { -100,-50,-100, 100,-50,-100, 100,-50,100, -100,-50,100, 0,150,0 };
    static const uint8_t PYR_F[] = { 4,0,1,2,3, 3,0,1,4, 3,1,2,4, 3,2,3,4, 3,3,0,4, 0 };
    int ps = vpyp_hull_shape(PYR, 5, PYR_F);
    int p = vpyp_add_hull(0,400,0,ps,1); vpyp_set_rotation(p,1,0,1,700);
    for (int i=0;i<500;i++) vpyp_step();
    /* it ends FLAT on a face — base or side, both are stable for this pyramid —
     * with its centre that face's distance above the floor */
    int32_t m[9]; vpyp_rotation(p,m); vpyp_position(p,&x,&y,&z);
    const double fn[5][3] = { {0,-1,0}, {0,1,-2}, {2,1,0}, {0,1,2}, {-2,1,0} };   /* outward, unnormalised */
    const double fd[5] = { 50, 150, 150, 150, 150 };                              /* n·x = d, unnormalised */
    int flat = -1; double want = 0;
    for (int f=0; f<5; f++) {
      const double l = sqrt(fn[f][0]*fn[f][0]+fn[f][1]*fn[f][1]+fn[f][2]*fn[f][2]);
      const double wy = (m[3]*fn[f][0] + m[4]*fn[f][1] + m[5]*fn[f][2]) / 16384.0 / l;   /* world y of the normal */
      if (wy < -0.999) { flat = f; want = fd[f] / l; }
    }
    CHECK(flat >= 0 && fabs(y-want)<=3 && vpyp_sleeping(p), "a pyramid dropped tipped lands flat on face %d: centre y=%d (%.0f), asleep %d", flat, y, want, vpyp_sleeping(p));

    /* a ball dropped on a hull comes to rest on top of it */
    vpyp_reset(); vpyp_set_gravity(0,-9800,0); vpyp_set_floor(1,0,64,128);
    int cs = vpyp_hull_shape(CUBE, 8, CUBE_F);
    vpyp_add_hull(0,100,0,cs,0);
    int ball = vpyp_add_sphere(0,500,0,50,1);
    for (int i=0;i<300;i++) vpyp_step();
    vpyp_position(ball,&x,&y,&z);
    CHECK(abs(y-250)<=3, "a ball rests on a static hull cube: y=%d (250)", y);

    /* and on its corner it rolls off rather than sinking in */
    vpyp_reset(); vpyp_set_gravity(0,-9800,0); vpyp_set_floor(1,0,64,128);
    cs = vpyp_hull_shape(CUBE, 8, CUBE_F);
    vpyp_add_hull(0,100,0,cs,0);
    ball = vpyp_add_sphere(130,500,130,50,1);
    int worst = 0;
    for (int i=0;i<300;i++){ vpyp_step(); vpyp_position(ball,&x,&y,&z);
      const double dx=x-100.0, dy=y-200.0, dz=z-100.0; const double d=sqrt(dx*dx+dy*dy+dz*dz);
      if (x<100 && z<100 && y>200) { /* over the top face */ if (250-y>worst) worst=250-y; }
      else if (d < 50-worst) worst=(int)(50-d); }
    vpyp_position(ball,&x,&y,&z);
    CHECK(worst<=3 && y<=60, "a ball on the corner of a hull rolls off it: sank %d at most, now at y=%d", worst, y);

    /* hull edge across box edge, as test 13 */
    vpyp_reset(); vpyp_set_gravity(0,-9800,0);
    cs = vpyp_hull_shape(CUBE, 8, CUBE_F);
    int A = vpyp_add_box(0,0,0,100,100,100,0); vpyp_set_rotation(A,0,0,1,512);
    int B = vpyp_add_hull(0,400,0,cs,1); vpyp_set_rotation(B,1,0,0,512);
    worst = 0;
    for (int i=0;i<40;i++){ vpyp_step(); vpyp_position(B,&x,&y,&z);
      const int sink = (int)lround(2*sqrt(2.0)*100 - y); if (sink > worst) worst = sink; }
    CHECK(worst <= 3, "hull crosswise on a box's ridge: sinks at most %d units", worst);

    /* a ray hits a hull's face */
    vpyp_reset();
    cs = vpyp_hull_shape(PYR, 5, PYR_F);
    vpyp_add_hull(0,0,0,cs,0);
    vpyp_hit h;
    int id = vpyp_raycast(0,1000,0, 0,-1,0, 5000, 0xFF, &h);
    CHECK(id==0 && h.y==150 && h.dist==850, "ray down onto the pyramid's apex: y=%d (150), dist %d", h.y, h.dist);
    id = vpyp_raycast(-1000,-20,0, 1,0,0, 5000, 0xFF, &h);
    const double sx = -(100.0 * (150+20) / 200.0);       /* the slope at y=-20 */
    CHECK(id==0 && abs(h.x-(int)lround(sx))<=1 && h.nx<0 && h.ny>0, "ray along x into its side: x=%d (%.0f), normal (%d,%d)", h.x, sx, h.nx, h.ny);
    /* x + y = 0 meets the -x side (-2x + y = 150) at (-50, 50) */
    id = vpyp_raycast(-500,500,0, 3,-3,0, 5000, 0xFF, &h);
    CHECK(id==0 && abs(h.x+50)<=1 && abs(h.y-50)<=1 && abs(h.dist-636)<=1, "a short direction (3,-3,0) is followed exactly: hit at (%d,%d) dist %d, expected (-50,50) 636", h.x, h.y, h.dist);

    /* refusals are loud */
    vpyp_reset();
    static const int16_t SUNK[8*3] = { -100,-100,-100, 100,-100,-100, -100,100,-100, 100,100,-100,
                                       -100,-100,100, 100,-100,100, -100,100,100, 30,30,30 };
    int bad = vpyp_hull_shape(SUNK, 8, CUBE_F);
    CHECK(bad==VPYP_NONE && vpyp_hull_error()==VPYP_HULL_NOT_FLAT && vpyp_stats()->shapes_refused==1, "a cube with a corner pushed in is refused: error %d (NOT_FLAT)", vpyp_hull_error());
    static const int16_t SPIKE[6*3] = { -100,-50,-100, 100,-50,-100, 100,-50,100, -100,-50,100, 0,150,0, 0,300,0 };
    bad = vpyp_hull_shape(SPIKE, 6, PYR_F);
    CHECK(bad==VPYP_NONE && vpyp_hull_error()==VPYP_HULL_NOT_CONVEX, "a corner above the apex, outside every face: error %d (NOT_CONVEX)", vpyp_hull_error());
    static const int16_t OFF[5*3] = { 400,-50,-100, 600,-50,-100, 600,-50,100, 400,-50,100, 500,150,0 };
    bad = vpyp_hull_shape(OFF, 5, PYR_F);
    CHECK(bad==VPYP_NONE && vpyp_hull_error()==VPYP_HULL_ORIGIN_OUTSIDE, "a shape around (500,0,0) is refused: error %d (ORIGIN_OUTSIDE)", vpyp_hull_error());
    static const uint8_t BADF[] = { 4,0,1,2,9, 0 };
    bad = vpyp_hull_shape(PYR, 5, BADF);
    CHECK(bad==VPYP_NONE && vpyp_hull_error()==VPYP_HULL_BAD_FACE, "a face naming corner 9 of 5 is refused: error %d (BAD_FACE)", vpyp_hull_error());
    int made=0; for (int i=0;i<VPYP_MAX_HULLS+1;i++) made += vpyp_hull_shape(PYR,5,PYR_F)>=0;
    CHECK(made==VPYP_MAX_HULLS && vpyp_hull_error()==VPYP_HULL_TABLE_FULL, "%d shapes made of %d asked, the last refused: TABLE_FULL", made, VPYP_MAX_HULLS+1);
  }

  /* 15. joints */
  { /* a ball on a joint is a physical pendulum: T = 2 pi sqrt((2/5 r^2 + L^2) / (g L)) */
    vpyp_reset(); vpyp_set_gravity(0,-9800,0);
    const int L = 500, r = 50; const double th = 0.15;
    int b = vpyp_add_sphere((int)lround(L*sin(th)), 1000-(int)lround(L*cos(th)), 0, r, 1);
    vpyp_set_material(b,0,0);
    vpyp_ball_joint(b, VPYP_NONE, 0,1000,0);
    int cross_n=0, first=-1, last=-1; int32_t px=1; int stretch=0;
    for (int i=0;i<500;i++){ vpyp_step(); vpyp_position(b,&x,&y,&z);
      if ((px>0)!=(x>0)) { cross_n++; if (first<0) first=i; last=i; } px=x;
      if (vpyp_stats()->joint_stretch>stretch) stretch=vpyp_stats()->joint_stretch; }
    const double T = 2.0*(last-first)/(cross_n-1)/50.0, Tth = 2*M_PI*sqrt((0.4*r*r+(double)L*L)/(9800.0*L));
    CHECK(fabs(T-Tth) < 0.03*Tth, "pendulum on a ball joint: period %.3f s, theory %.3f s", T, Tth);
    CHECK(stretch <= 2, "and the joint holds: stretch at most %d units", stretch);

    /* hanging still it sleeps: the joint holds it up */
    vpyp_reset(); vpyp_set_gravity(0,-9800,0);
    b = vpyp_add_sphere(0,500,0,50,1); vpyp_ball_joint(b, VPYP_NONE, 0,1000,0);
    for (int i=0;i<150;i++) vpyp_step();
    vpyp_position(b,&x,&y,&z);
    CHECK(vpyp_sleeping(b) && abs(y-500)<=2, "a ball hanging still on a joint sleeps: asleep %d, y=%d (500)", vpyp_sleeping(b), y);

    /* a chain of five boxes from a pin, let go sideways: the links hold */
    vpyp_reset(); vpyp_set_gravity(0,-9800,0);
    int link[5]; stretch=0;
    for (int i=0;i<5;i++) { link[i] = vpyp_add_box(50+i*100,1000,0,50,15,15,1);
      vpyp_ball_joint(link[i], i ? link[i-1] : VPYP_NONE, i*100,1000,0); }
    int lowest=1000;
    for (int i=0;i<200;i++){ vpyp_step();
      if (vpyp_stats()->joint_stretch>stretch) stretch=vpyp_stats()->joint_stretch;
      vpyp_position(link[4],&x,&y,&z); if (y<lowest) lowest=y; }
    CHECK(stretch <= 6 && lowest < 600, "a chain of 5 boxes swings down (end reached y=%d) and stretches at most %d units", lowest, stretch);

    /* a door on a hinge about y: it swings, and it does not sag or tilt */
    vpyp_reset(); vpyp_set_gravity(0,-9800,0);
    int door = vpyp_add_box(200,0,0,200,300,20,4);
    vpyp_hinge(door, VPYP_NONE, 0,0,0, 0,1,0);
    vpyp_apply_impulse_at(door, 0,0,4*1500, 400,0,0);
    int sag=0, tilt=16384; stretch=0;
    for (int i=0;i<100;i++){ vpyp_step(); vpyp_position(door,&x,&y,&z); int32_t m[9]; vpyp_rotation(door,m);
      if (-y>sag) sag=-y; if (m[4]<tilt) tilt=m[4];
      if (vpyp_stats()->joint_stretch>stretch) stretch=vpyp_stats()->joint_stretch; }
    int32_t wx,wy,wz; vpyp_spin(door,&wx,&wy,&wz);
    const double dr = sqrt((double)x*x+(double)z*z);
    CHECK(sag<=3 && tilt>16300 && fabs(dr-200)<=3 && abs(wy)>0, "a hinged door swings about y (spin %d) without sagging (%d) or tilting (up %d/16384), centre %.0f from the hinge (200)", wy, sag, tilt, dr);
    CHECK(stretch<=3, "and its hinge holds: stretch at most %d", stretch);
    /* the same door on a ball joint is not held upright: the axis is what the hinge adds */
    vpyp_reset(); vpyp_set_gravity(0,-9800,0);
    door = vpyp_add_box(200,0,0,200,300,20,4);
    vpyp_ball_joint(door, VPYP_NONE, 0,0,0);
    int low = 0;
    for (int i=0;i<100;i++) { vpyp_step(); vpyp_position(door,&x,&y,&z); if (y<low) low=y; }
    CHECK(low < -150, "the same door on a ball joint swings down instead: centre reached y=%d", low);

    /* two halves of a hinge overlap and do not push each other apart */
    vpyp_reset();
    int h1 = vpyp_add_box(0,0,0,100,100,100,1), h2 = vpyp_add_box(150,0,0,100,100,100,1);
    vpyp_hinge(h1,h2, 75,0,0, 0,0,1);
    for (int i=0;i<50;i++) vpyp_step();
    vpyp_position(h2,&x,&y,&z);
    CHECK(x==150 && vpyp_contact_count()==0, "joined bodies overlapping by 50 are left alone: x=%d, contacts %d", x, vpyp_contact_count());
    vpyp_remove(h1);
    CHECK(vpyp_stats()->joints==0 || (vpyp_step(), vpyp_stats()->joints==0), "removing a body removes its joint (%u left)", vpyp_stats()->joints);

    /* refusals */
    vpyp_reset();
    int s0 = vpyp_add_box(0,0,0,10,10,10,0), m1 = vpyp_add_box(0,0,0,10,10,10,1);
    int bad = (vpyp_ball_joint(m1,m1,0,0,0)==VPYP_NONE) + (vpyp_ball_joint(s0,VPYP_NONE,0,0,0)==VPYP_NONE)
            + (vpyp_hinge(m1,s0,0,0,0,0,0,0)==VPYP_NONE) + (vpyp_ball_joint(m1,40,0,0,0)==VPYP_NONE);
    int madej=0; for (int i=0;i<VPYP_MAX_JOINTS+1;i++) madej += vpyp_ball_joint(m1,VPYP_NONE,0,0,0)>=0;
    CHECK(bad==4 && madej==VPYP_MAX_JOINTS && vpyp_stats()->joints_refused==5, "joints refused: self, static to world, no axis, no body, table full (%u counted)", vpyp_stats()->joints_refused);
  }

  printf("%s (%d failed)\n", fails?"FAILED":"ALL OK", fails);
  return fails;
}
