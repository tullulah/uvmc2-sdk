#!/bin/sh
# build_host_tools.sh — every host tool that links uvm2_draw.c, built the one right way.
#
#   tools/build_host_tools.sh [outdir]        (default /tmp/uvm2-tools)
#
# WHY ONE SCRIPT. Each tool carried its own build line and its own stubs, and
# they drifted: on 2026-10-01 six of the eight no longer linked (uvm2_draw.c had
# gained neighbours — the sample player, the config loader, the microsecond
# clock), and one still read the command list as 4-byte words. Building them all
# here means a tool that breaks is seen the day it breaks.
#
# -DUVM2_HZ=0 IS NOT OPTIONAL. With the 50 Hz lock on, frame_end pads every list
# to 30000 cycles and a tool that measures a frame measures the padding:
# uvm2_anatomy said 937 cycles per operation with it and 48 without.
set -e
cd "$(dirname "$0")/.."
OUT=${1:-/tmp/uvm2-tools}
mkdir -p "$OUT"
( cd ../vectrex-draw/cabi && cargo build --release -q )
LIB=../vectrex-draw/cabi/target/release/libvectrex_draw_cabi.a
FLAGS="-O2 -w -DUVM2_HOST -DUVM2_BENCH_NO_CORE1 -DUVM2_SUBUNITS -DUVM2_HZ=0 -DUVM2_CMD_CAPACITY=65536u -I."
fail=0
for t in uvm2_anatomy uvm2_budget uvm2_gapped_budget uvm2_list_count uvm2_op_cost uvm2_order uvm2_ramp_cost uvm2_smp_test; do
    extra=""
    grep -q "uvm2_smp.c" "tools/$t.c" && extra="uvm2_smp.c"
    if cc $FLAGS -o "$OUT/$t" "tools/$t.c" uvm2_draw.c $extra tools/uvm2_host_stubs.c "$LIB"; then
        echo "  ok    $t"
    else
        echo "  FAIL  $t"; fail=$((fail + 1))
    fi
done
echo "$fail failed -> $OUT"
exit $fail
