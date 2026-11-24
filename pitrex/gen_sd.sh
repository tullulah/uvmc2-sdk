#!/bin/sh
# gen_sd.sh IN OUT — the baremetal tree's SD card driver (pitrex/baremetal/sd.c) with its
# multi-block transfers ended by the controller (pitrex.mk says why). Prints "ok"; refuses,
# printing nothing, if IN does not have the lines this changes exactly once each: a driver that
# changed is looked at, not patched blind.
in=$1; out=$2
r='SD_CMD_INDEX(18) | SD_RESP_R1 | SD_DATA_READ | SD_CMD_MULTI_BLOCK | SD_CMD_BLKCNT_EN,'
w='SD_CMD_INDEX(25) | SD_RESP_R1 | SD_DATA_WRITE | SD_CMD_MULTI_BLOCK | SD_CMD_BLKCNT_EN,'
s='} else if (cur_state == 5) {'
for p in "$r" "$w" "$s"; do [ "$(grep -c -F "$p" "$in")" = 1 ] || exit 1; done
mkdir -p "$(dirname "$out")"
sed -e 's/SD_CMD_INDEX(18) | SD_RESP_R1 | SD_DATA_READ | SD_CMD_MULTI_BLOCK | SD_CMD_BLKCNT_EN,/SD_CMD_INDEX(18) | SD_RESP_R1 | SD_DATA_READ | SD_CMD_MULTI_BLOCK | SD_CMD_BLKCNT_EN | SD_CMD_AUTO_CMD_EN_CMD12,/' \
    -e 's/SD_CMD_INDEX(25) | SD_RESP_R1 | SD_DATA_WRITE | SD_CMD_MULTI_BLOCK | SD_CMD_BLKCNT_EN,/SD_CMD_INDEX(25) | SD_RESP_R1 | SD_DATA_WRITE | SD_CMD_MULTI_BLOCK | SD_CMD_BLKCNT_EN | SD_CMD_AUTO_CMD_EN_CMD12,/' \
    -e 's/} else if (cur_state == 5) {/} else if (cur_state == 5 || cur_state == 6) {/' "$in" > "$out.new" || exit 1
cmp -s "$out.new" "$out" || cp "$out.new" "$out"
echo ok
