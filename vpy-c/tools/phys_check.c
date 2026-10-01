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
  int ok=1, slept=1; for (int i=0;i<5;i++){ vpyp_position(bx[i],&x,&y,&z); if (abs(y-(100+i*200))>3 || x||z) ok=0; if(!vpyp_sleeping(bx[i])) slept=0; }
  vpyp_position(bx[4],&x,&y,&z);
  CHECK(ok, "stack of 5 holds: top at y=%d (expected 900)", y);
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
  vpyp_position(bx[1],&x,&y,&z);
  CHECK(slept3 && x > 20, "thrown ball hits the sleeping stack: middle box pushed to x=%d", x);

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

  printf("%s (%d failed)\n", fails?"FAILED":"ALL OK", fails);
  return fails;
}
