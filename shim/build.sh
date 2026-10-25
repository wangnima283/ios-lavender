#!/bin/bash
# Build the freestanding shim and wrap it in an Android boot.img for SDM660 (lavender).
set -euo pipefail

PAYLOAD="${1:-}"
if [ -z "$PAYLOAD" ]; then
	if [ -f "$(dirname "$0")/../xnu/out/kernel.development.lavender" ]; then
		PAYLOAD="$(dirname "$0")/../xnu/out/kernel.development.lavender"
	elif [ -f "$(dirname "$0")/../xnu/out/kernel.development.santoni" ]; then
		PAYLOAD="$(dirname "$0")/../xnu/out/kernel.development.santoni"
	fi
fi
[ -n "$PAYLOAD" ] && [ -f "$PAYLOAD" ] && PAYLOAD="$(realpath "$PAYLOAD")"

cd "$(dirname "$0")"

CC="${CC:-clang}"
OBJCOPY="${OBJCOPY:-llvm-objcopy}"
READELF="${READELF:-llvm-readelf}"
NM="${NM:-llvm-nm}"
LD="${LD:-ld.lld}"
HOST_CC="${HOST_CC:-clang}"

CFLAGS="-target aarch64-linux-gnu -O2 -ffreestanding -nostdlib -fno-pic -fno-jump-tables -mgeneral-regs-only -mstrict-align -fno-stack-protector -Wall -Wextra"

mkdir -p out

for f in start.S shim.c console.c mmu.c fdt.c loader.c; do
	$CC $CFLAGS -c $f -o out/${f%.*}.o
done

abs=$($READELF -r out/shim.o out/console.o out/mmu.o out/fdt.o out/loader.o | grep -c ABS64 || true)
[ "$abs" = 0 ] || { echo "error: $abs absolute relocations in C objects" >&2; exit 1; }

$LD -T shim.ld --build-id=none -N \
	out/start.o out/shim.o out/console.o out/mmu.o out/fdt.o out/loader.o -o out/shim.elf

$OBJCOPY -O binary out/shim.elf out/Image

size=$((16#$($NM out/shim.elf | awk '$3=="_end"{print $1}')))
truncate -s "$size" out/Image

# Build static Mach-O PID 1 init test program
echo "Building static Mach-O PID 1 init..."
$CC -target arm64-apple-ios16.0 -c ../tools/santoni-init.S -o out/lavender-init.o
python3 ../tools/mkinit.py out/lavender-init.o out/init.macho

# Ramdisk / Payload packaging
RAMDISK="out/ramdisk.bin"
CMDLINE="${CMDLINE:-debug=0x14e serial=3 cs_enforcement_disable=1 amfi_get_out_of_my_way=1 amfi_allow_any_signature=1 disable_kprintf_output=0 -enable_kprintf_spam sweetfb_native=1 sweetcpufreq=0 sweetbattery=0 rd=emmcroot0 emmcroot=userdata emmcrw=1 rootrw=1 unentitled_ios_sim_launch=1 ignition_level=1 wdt_test=no_user_checkin launchd_unsecure_cache=1 wdt=-1 -v keepsyms=1 sweetscan=backboardd,SpringBoard lavender}"

# Ensure a minimal ramdisk.bin is always present
if command -v cpio >/dev/null 2>&1; then
	(cd out && rm -rf rd && mkdir rd && cd rd && find . | cpio -o -H newc --quiet | gzip -9 > ../ramdisk.bin)
else
	python3 -c 'import gzip; h=b"07070100000000000000000000000000000000000000010000000000000000000000000000000000000000000000000000000b00000000TRAILER!!!\0\0\0\0"; open("out/ramdisk.bin","wb").write(gzip.compress(h + b"\0"*(512-len(h)), compresslevel=9))'
fi
RAMDISK="out/ramdisk.bin"

if [ -n "$PAYLOAD" ]; then
	magic=$(python3 -c "import sys; f=open(sys.argv[1],'rb'); b=f.read(4); print(b.hex())" "$PAYLOAD" 2>/dev/null || true)
	if [ "$magic" = "cffaedfe" ] || [ "$magic" = "feedfacf" ]; then
		echo "Detected raw Mach-O kernel ($PAYLOAD). Generating ADT and packing with mkpayload.py..."
		python3 ../tools/mkadt.py out/lavender.adt
		python3 ../tools/mkpayload.py "$PAYLOAD" out/lavender.adt out/payload.bin --cmdline "$CMDLINE" --ramdisk out/init.macho
	fi
fi

# Append payload to Image after _end (allows ABL to load payload regardless of ramdisk handling)
if [ -n "$PAYLOAD" ] && [ -f "out/payload.bin" ]; then
	align_size=$(( (size + 0xfff) & ~0xfff ))
	truncate -s "$align_size" out/Image
	cat out/payload.bin >> out/Image
	python3 -c 'import os,struct,sys; f=open(sys.argv[1],"r+b"); f.seek(16); f.write(struct.pack("<Q", os.path.getsize(sys.argv[1])))' out/Image
fi

# Package as Image.gz-dtb (standard SDM660 kernel format for Qualcomm ABL)
gzip -9 -c out/Image > out/Image.gz
if [ -f ../resources/lavender_dtb.bin ]; then
	cat ../resources/lavender_dtb.bin >> out/Image.gz
fi

# Packaging: wrap in boot.img using tools/mkbootimg.py with lavender parameters
python3 ../tools/mkbootimg.py \
	--kernel out/Image.gz \
	--ramdisk "$RAMDISK" \
	--output out/shim-boot.img \
	--base 0x00000000 \
	--kernel_offset 0x00008000 \
	--ramdisk_offset 0x02000000 \
	--tags_offset 0x00000100 \
	--pagesize 4096 \
	--header_version 0 \
	--cmdline "$CMDLINE"

# Host render of the console layout
$HOST_CC -O2 -Wall -I. -o out/host_test console.c host_test.c && ./out/host_test

# Host unit test for Mach-O segment unroller and boot_args
$HOST_CC -O2 -Wall -I. -o out/loader_test loader_test.c loader.c && ./out/loader_test

# Unit tests and patch checks
HOST_CC="$HOST_CC" OUT_DIR="$PWD/out" bash ../tests/run_tests.sh

ls -l out/Image out/shim-boot.img
