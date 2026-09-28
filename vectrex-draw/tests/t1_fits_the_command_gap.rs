//! THE CEILING BOUNDS t1; RELEASING IT UNBOUNDS IT.
//!
//! `ceiling` is capped by T1_TRANSPORT (110 in the cartridge BIOS). `t1_vcap` has no upper
//! bound — its own comment says that at VCAP = 8 it asks for 2540. While the ceiling always
//! ruled, t1 could not exceed 110; the branch that releases it only existed under
//! CEILING_RULES = 0, which is not the default. CEILING_PAYS releases it where it pays.
//!
//! What t1 feeds is the command's gap, a TWELVE-BIT field. A saturated gap blows the frame
//! up — 788,661 cycles measured once with an ORA+4096 — and a half-second frame leaves the
//! console drawing at 2 fps: input is read once per frame, so the controller looks dead even
//! though the drawing is still there.
//!
//! The emulator does not see it: it reproduces the GEOMETRY, not the timing, so two builds
//! with the same segments and different gaps look identical to it.
use std::sync::atomic::Ordering;
use vectrex_draw::ramp::{ramp_params_chain_qn, CEILING_PAYS, MIN_T1, T1_TRANSPORT};

#[test]
fn t1_fits_the_command_gap() {
    /* The cartridge's configuration, ALL of it: these are process statics, and a bench that
     * only sets what it looks at inherits the rest from the previous test. */
    T1_TRANSPORT.store(110, Ordering::Relaxed);
    MIN_T1.store(8, Ordering::Relaxed);

    /* BOTH VARIANTS, and what matters is not only the maximum but the SUM: t1 is ramp time,
     * so summing it over a scene's deltas is what the frame lasts. */
    let sweep = || {
        let (mut worst, mut at, mut sum, mut n) = (0u16, (0, 0), 0u64, 0u32);
        for dx in (-2048..=2048).step_by(17) {
            for dy in (-2048..=2048).step_by(17) {
                if dx == 0 && dy == 0 { continue; }
                let (_vx, _vy, t1) = ramp_params_chain_qn(dx, dy, 4);
                n += 1; sum += t1 as u64;
                if t1 > worst { worst = t1; at = (dx, dy); }
            }
        }
        (n, worst, at, sum)
    };
    CEILING_PAYS.store(0, Ordering::Relaxed);
    let (n, worst0, at0, sum0) = sweep();
    CEILING_PAYS.store(1, Ordering::Relaxed);
    let (_, worst1, at1, sum1) = sweep();
    println!("  {n} deltas");
    println!("  the ceiling ALWAYS rules (as before): t1 max {worst0} at {at0:?}, sum {sum0}");
    println!("  the ceiling rules WHILE IT PAYS     : t1 max {worst1} at {at1:?}, sum {sum1}");
    println!("  ramp time changes by {:+.1}%",
             100.0 * (sum1 as f64 - sum0 as f64) / sum0 as f64);
    assert!(worst1 <= 4095, "t1 = {worst1} at {at1:?} does not fit the 12-bit gap");
}
