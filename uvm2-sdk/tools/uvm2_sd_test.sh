#!/bin/sh
# uvm2_sd_test.sh — uvm2_sd.c against real FAT16, FAT32 and exFAT images, MBR and GPT.
#
# macOS only: the images are made by hdiutil/newfs, filled from the Mac side, then read and
# written through uvm2_sd.c (tools/uvm2_sd_test.c), and finally checked twice — `fsck -n` must
# come out clean, and the Mac must read back exactly what the harness wrote.
#
#     uvm2-sdk/tools/uvm2_sd_test.sh
#
# Each image is also read and written through a SIMULATED card (tools/uvm2_sd_card.c), which
# runs the driver's own SPI protocol — CMD18 runs, their CMD12 stop, the fallback to CMD17 —
# with faults injected through UVM2_CARD.
#
# Exit status 0 = every case passed. Nothing here touches a real card.
set -eu
cd "$(dirname "$0")/.."
FATFS=../third_party/fatfs
W=$(mktemp -d)
MNT="$W/mnt"
DEV=""
trap '[ -n "$DEV" ] && hdiutil detach -quiet "$DEV" 2>/dev/null; rm -rf "$W"' EXIT

cc -O2 -Wall -DUVM2_SD_HOST -I. -I$FATFS -o "$W/t" \
   tools/uvm2_sd_test.c uvm2_sd.c $FATFS/ff.c $FATFS/ffunicode.c
# The same harness with the driver's OWN protocol (start-up, CMD17/18/12/24) against a
# simulated card (tools/uvm2_sd_card.c): what UVM2_SD_HOST cannot see.
cc -O2 -Wall -DUVM2_SD_FAKECARD -I. -I$FATFS -o "$W/c" \
   tools/uvm2_sd_test.c tools/uvm2_sd_card.c uvm2_sd.c $FATFS/ff.c $FATFS/ffunicode.c

fails=0
check() {   # check <label> <expected ok=/err= fragment> <output>
    case "$3" in *"$2"*) echo "  pass  $1" ;;
                 *)     echo "  FAIL  $1: wanted '$2', got '$3'"; fails=$((fails + 1)) ;; esac
}
match() {   # match <label> <glob> <output>: like check, but the pattern may hold * and [..]
    case "$3" in $2) echo "  pass  $1" ;;
                 *)  echo "  FAIL  $1: wanted '$2', got '$3'"; fails=$((fails + 1)) ;; esac
}
same() {    # same <label> <a> <b>
    if cmp -s "$2" "$3"; then echo "  pass  $1"
    else echo "  FAIL  $1: contents differ"; fails=$((fails + 1)); fi
}

head -c 30000   /dev/urandom > "$W/rom.zip"
head -c 300000  /dev/urandom > "$W/big.bin"

# fs  size  layout   hdiutil -fs   fsck      expected fs_type (FatFs FS_*)   newfs args
#
# THE CLUSTER SIZE DECIDES WHETHER CMD18 CAN HAPPEN AT ALL: FatFs never asks for a run past
# the end of a cluster, and macOS makes 256 MB of FAT32 with 512-byte clusters. That image
# is kept as the case where every read must stay single-block; the 4 KB one is closer to a
# real card (the SD Association's formatter uses 32 KB on FAT32).
for spec in "FAT16 64m MBRSPUD MS-DOS+FAT16 fsck_msdos 2 -" \
            "FAT32 256m MBRSPUD MS-DOS+FAT32 fsck_msdos 3 -" \
            "FAT32 512m MBRSPUD MS-DOS+FAT32 fsck_msdos 3 -c+8" \
            "exFAT 256m MBRSPUD ExFAT fsck_exfat 4 -" \
            "exFAT 256m GPTSPUD ExFAT fsck_exfat 4 -"; do
    set -- $spec
    name=$1; size=$2; layout=$3; fs=$(echo "$4" | tr + ' '); fsck=$5; want_fs=$6
    fsargs=$(echo "$7" | tr + ' ')
    if [ "$fsargs" = "-" ]; then echo "$name $size ($layout)"; else echo "$name $size ($layout, $fsargs)"; fi
    img="$W/$name-$size-$layout.dmg"
    rm -f "$img"
    if [ "$fsargs" = "-" ]; then
        hdiutil create -quiet -size "$size" -fs "$fs" -volname UVMC2 -layout "$layout" "$img"
    else
        hdiutil create -quiet -size "$size" -fs "$fs" -fsargs "$fsargs" -volname UVMC2 \
                -layout "$layout" "$img"
    fi
    DEV=$(hdiutil attach -nobrowse -mountpoint "$MNT" "$img" | awk 'NR==1{print $1}')
    mkdir -p "$MNT/roms" "$MNT/config"
    cp "$W/rom.zip" "$MNT/roms/tacscan.zip"
    cp "$W/rom.zip" "$MNT/roms/A Long Romset Name.zip"
    cp "$W/big.bin" "$MNT/roms/big.bin"
    printf 'old\n' > "$MNT/config/short.cfg"                 # shorter than 512 bytes
    head -c 1024 /dev/zero | tr '\0' x > "$MNT/config/uvm2.cfg"
    hdiutil detach -quiet "$DEV"; DEV=""

    T="$W/t $img"
    check "mount sees $name"    "fs=$want_fs"  "$($T read roms/tacscan.zip "$W/out")"
    same  "romset reads back"   "$W/rom.zip" "$W/out"
    check "case-insensitive"    "ok=1"         "$($T read ROMS/TACSCAN.ZIP "$W/out")"
    check "long file name"      "ok=1"         "$($T read "roms/a long romset name.zip" "$W/out")"
    same  "long name contents"  "$W/rom.zip" "$W/out"
    check "missing -> MISSING"  "ok=0 err=4"   "$($T read roms/nothere.zip "$W/out")"
    check "chunk"               "ok=1 err=0 n=1000" "$($T readfrom roms/tacscan.zip 20000 1000 "$W/out")"
    dd if="$W/rom.zip" of="$W/ref" bs=1 skip=20000 count=1000 2>/dev/null
    same  "chunk contents"      "$W/ref" "$W/out"
    check "chunk past the end"  "ok=1 err=0 n=0" "$($T readfrom roms/tacscan.zip 40000 10 "$W/out")"
    check "stream in slices"    "ok=1 err=0 n=30000" "$($T stream roms/tacscan.zip 4096 "roms/a long romset name.zip" "$W/out")"
    same  "stream contents"     "$W/rom.zip" "$W/out"
    # THROUGH THE SIMULATED CARD. Matching data is not enough: a CMD18 run that fails is read
    # again with CMD17 and comes out right, so multi_fail=0 and viol=0 are what prove CMD18
    # itself worked. The `card` line is printed at exit, after the harness's own line.
    C="$W/c $img"
    csize=$($C read roms/tacscan.zip "$W/out" 2>&1 | sed -n 's/.* csize=\([0-9]*\).*/\1/p')
    if [ "$csize" -gt 1 ]; then clean="*multi=[1-9]* multi_fail=0*viol=0"
    else                        clean="*multi=0 multi_fail=0*cmd18=0*viol=0"; fi
    echo "        (clusters of $csize sectors)"
    match "card: read"   "ok=1 err=0 n=300000 $clean" "$($C read roms/big.bin "$W/out" 2>&1)"
    same  "card: contents"      "$W/big.bin" "$W/out"
    match "card: stream 1 KB"   "ok=1 err=0 n=300000 $clean" "$($C stream roms/big.bin 1024 roms/tacscan.zip "$W/out" 2>&1)"
    same  "card: stream contents" "$W/big.bin" "$W/out"
    match "card: stream 4 KB"   "ok=1 err=0 n=300000 $clean" "$($C stream roms/big.bin 4096 roms/tacscan.zip "$W/out" 2>&1)"
    match "card: SDSC addressing" "ok=1 err=0 n=300000 $clean" "$(UVM2_CARD=sdsc $C read roms/big.bin "$W/out" 2>&1)"
    same  "card: SDSC contents" "$W/big.bin" "$W/out"
    if [ "$csize" -gt 1 ]; then
    match "card: no CMD18 -> CMD17" "ok=1 err=0 n=300000 *multi=0 multi_fail=[1-9]*cmd18=0*viol=0" \
          "$(UVM2_CARD=nocmd18 $C read roms/big.bin "$W/out" 2>&1)"
    same  "card: fallback contents" "$W/big.bin" "$W/out"
    match "card: error token"   "ok=1 err=0 n=300000 *multi_fail=1*viol=0" \
          "$(UVM2_CARD=errtoken $C read roms/big.bin "$W/out" 2>&1)"
    same  "card: error token contents" "$W/big.bin" "$W/out"
    fi
    match "card: write"         "ok=1 err=0*cmd24=[1-9]*viol=0" "$($C write card/w.bin "$W/rom.zip" 2>&1)"
    match "card: read the write" "ok=1 err=0 n=30000 *viol=0" "$($C read card/w.bin "$W/out" 2>&1)"
    same  "card: write contents" "$W/rom.zip" "$W/out"

    check "write, nested dirs"  "ok=1 err=0"   "$($T write traces/run1/big.bin "$W/big.bin")"
    check "write replaces"      "ok=1 err=0"   "$($T write traces/run1/big.bin "$W/rom.zip")"
    check "create"              "ok=1 err=0"   "$($T create newcfg/game.cfg "a=1")"
    check "overwrite existing"  "ok=1 err=0"   "$($T overwrite config/uvm2.cfg "b=2")"
    check "overwrite missing"   "ok=0 err=4"   "$($T overwrite config/none.cfg "c=3")"
    check "overwrite too short" "ok=0 err=5"   "$($T overwrite config/short.cfg "d=4")"

    # Attached without mounting: the first line is the disk, the last one the volume.
    hdiutil attach -nobrowse -nomount "$img" > "$W/att"
    DEV=$(awk 'NR==1{print $1}' "$W/att"); part=$(awk 'END{print $1}' "$W/att")
    if $fsck -n "$part" > "$W/fsck.log" 2>&1; then echo "  pass  $fsck -n clean"
    else echo "  FAIL  $fsck -n:"; sed 's/^/        /' "$W/fsck.log"; fails=$((fails + 1)); fi
    hdiutil detach -quiet "$DEV"
    DEV=$(hdiutil attach -nobrowse -mountpoint "$MNT" "$img" | awk 'NR==1{print $1}')
    same  "Mac reads the write" "$W/rom.zip" "$MNT/traces/run1/big.bin"
    same  "Mac reads the card's write" "$W/rom.zip" "$MNT/card/w.bin"
    printf 'a=1' > "$W/ref"; head -c 3 "$MNT/newcfg/game.cfg" > "$W/out"
    same  "Mac reads create"    "$W/ref" "$W/out"
    check "create is 512 bytes" "512"          "$(wc -c < "$MNT/newcfg/game.cfg" | tr -d ' ')"
    printf 'b=2' > "$W/ref"; head -c 3 "$MNT/config/uvm2.cfg" > "$W/out"
    same  "Mac reads overwrite" "$W/ref" "$W/out"
    check "overwrite keeps size" "1024"        "$(wc -c < "$MNT/config/uvm2.cfg" | tr -d ' ')"
    hdiutil detach -quiet "$DEV"; DEV=""
done

echo
if [ "$fails" -eq 0 ]; then echo "all passed"; else echo "$fails FAILED"; exit 1; fi
