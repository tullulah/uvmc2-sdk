/* uvm2_pitrex_cpu.h — the few instructions the PiTrex layer needs that differ between the two
 * Pis a PiTrex takes.
 *
 *   Zero 2 W   Cortex-A53, ARMv8 in AArch32: dmb/dsb/isb, the cycle counter PMCCNTR (enabled
 *              by the baremetal start-up), cache and TLB maintenance broadcast to every core.
 *   Zero (1)   ARM1176JZF-S, ARMv6: the barriers are CP15 operations, the cycle counter is the
 *              ARM11's own (CCNT in c15, enabled here: that start-up does not), and there is
 *              one core, so nothing is broadcast.
 *
 * Chosen by the compiler's architecture (pitrex.mk, PITREX_BOARD), not by a define of ours. */
#ifndef UVM2_PITREX_CPU_H
#define UVM2_PITREX_CPU_H
#include <stdint.h>

#if defined(__ARM_ARCH) && __ARM_ARCH < 7
#define PITREX_ARMV6 1
#define PITREX_DMB() __asm__ volatile("mcr p15, 0, %0, c7, c10, 5" : : "r"(0) : "memory")
#define PITREX_DSB() __asm__ volatile("mcr p15, 0, %0, c7, c10, 4" : : "r"(0) : "memory")
#define PITREX_ISB() __asm__ volatile("mcr p15, 0, %0, c7, c5, 4"  : : "r"(0) : "memory")
/* the ARM11's cycle counter: PMNC bit 0 enables the counters, bit 2 resets the cycle count */
#define ARM11_PMNC_ENABLE 0x1u
static inline void pitrex_ccnt_enable(void)
{
    uint32_t v;
    __asm__ volatile("mrc p15, 0, %0, c15, c12, 0" : "=r"(v));
    v |= ARM11_PMNC_ENABLE;
    __asm__ volatile("mcr p15, 0, %0, c15, c12, 0" : : "r"(v));
}
static inline uint32_t pitrex_ccnt(void)
{
    uint32_t v;
    __asm__ volatile("mrc p15, 0, %0, c15, c12, 1" : "=r"(v));
    return v;
}
/* this core's instruction cache, branch predictor and translations: the only core there is */
#define PITREX_FORGET_CODE_ALL_CORES() \
    __asm__ volatile("mcr p15, 0, %0, c7, c5, 0\n mcr p15, 0, %0, c7, c5, 6\n mcr p15, 0, %0, c8, c7, 0" \
                     : : "r"(0) : "memory")
#else
#define PITREX_DMB() __asm__ volatile("dmb" ::: "memory")
#define PITREX_DSB() __asm__ volatile("dsb" ::: "memory")
#define PITREX_ISB() __asm__ volatile("isb" ::: "memory")
static inline void pitrex_ccnt_enable(void) { }   /* the baremetal start-up has (PMCR) */
static inline uint32_t pitrex_ccnt(void)
{
    uint32_t v;
    __asm__ volatile("mrc p15, 0, %0, c9, c13, 0" : "=r"(v));
    return v;
}
/* ICIALLUIS, BPIALLIS, TLBIALLIS: every core's, broadcast in the inner shareable domain */
#define PITREX_FORGET_CODE_ALL_CORES() \
    __asm__ volatile("mcr p15, 0, %0, c7, c1, 0\n mcr p15, 0, %0, c7, c1, 6\n mcr p15, 0, %0, c8, c3, 0" \
                     : : "r"(0) : "memory")
#endif
#endif
