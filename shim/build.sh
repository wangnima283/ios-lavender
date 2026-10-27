#!/bin/bash
# Build the freestanding shim and wrap it in an Android boot.img for an
# SDM660/SDM636 board (BOARD=whyred by default, BOARD=lavender for the
# Redmi Note 7).
set -euo pipefail

BOARD="${BOARD:-whyred}"
case "$BOARD" in
	whyred|WHYRED)   BOARD_LC=whyred;  BOARD_DEFINE=-DBOARD_WHYRED ;;
	lavender|LAVENDER) BOARD_LC=lavender; BOARD_DEFINE=-DBOARD_LAVENDER ;;
	*) echo "error: BOARD must be whyred or lavender (got '$BOARD')" >&2; exit 2 ;;
esac

PAYLOAD="${1:-}"
if [ -z "$PAYLOAD" ]; then
	if [ -f "$(dirname "$0")/../xnu/out/kernel.development.$BOARD_LC" ]; then
		PAYLOAD="$(dirname "$0")/../xnu/out/kernel.development.$BOARD_LC"
	elif [ -f "$(dirname "$0")/../xnu/out/kernel.development.santoni" ]; then
		PAYLOAD="$(dirname "$0")/../xnu/out/kernel.development.santoni"
	fi
fi
[ -n "$PAYLOAD" ] && [ -f "$PAYLOAD" ] && PAYLOAD="$(realpath "$PAYLOAD")"
# Resolve IMAGE_GZ before the cd below: callers pass it relative to the repo
# root ("shim/out/Image.z.gz"), and after `cd shim` that path points at
# shim/shim/out/... - the file is silently missed and the build packs a gzip
# Image instead of the pre-compressed one.
[ -n "${IMAGE_GZ:-}" ] && [ -f "$IMAGE_GZ" ] && IMAGE_GZ="$(realpath "$IMAGE_GZ")"

cd "$(dirname "$0")"

CC="${CC:-clang}"
OBJCOPY="${OBJCOPY:-llvm-objcopy}"
READELF="${READELF:-llvm-readelf}"
NM="${NM:-llvm-nm}"
LD="${LD:-ld.lld}"
HOST_CC="${HOST_CC:-clang}"

CFLAGS="-target aarch64-linux-gnu -O2 -ffreestanding -nostdlib -fno-pic -fno-jump-tables -mgeneral-regs-only -mstrict-align -fno-stack-protector -Wall -Wextra $BOARD_DEFINE"

mkdir -p out

# Fail on the host, not on the phone: a kernel built for another machine, or a
# DTB whose qcom,msm-id/board-id the ABL does not accept, only shows up as
# "Failed to load/authenticate boot image" in the bootloader.
if [ -n "$PAYLOAD" ] && [ -f "$PAYLOAD" ]; then
	python3 ../tools/check_payload.py --board "$BOARD_LC" --kernel "$PAYLOAD"
else
	echo "warning: no kernel payload; the boot image will contain only the shim" >&2
fi
if [ -f "../resources/${BOARD_LC}_dtb.bin" ]; then
	python3 ../tools/check_payload.py --board "$BOARD_LC" --dtb "../resources/${BOARD_LC}_dtb.bin"
else
	echo "warning: resources/${BOARD_LC}_dtb.bin is missing; the stock ABL may reject the image" >&2
	echo "         extract it with: python3 tools/extract_appended_dtb.py <boot.img> --board $BOARD_LC" >&2
fi

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
$CC -target arm64-apple-ios16.0 -c ../tools/santoni-init.S -o out/$BOARD_LC-init.o
python3 ../tools/mkinit.py out/$BOARD_LC-init.o out/init.macho

# Ramdisk / Payload packaging
RAMDISK="out/ramdisk.bin"
# Root device: the ram disk by default (safe fallback), or emmcroot0 once an
# iOS root is staged in userdata. EXTRA_CMDLINE carries e.g. emmcrw=1.
ROOTDEV="${ROOTDEV:-md0}"
EXTRA_CMDLINE="${EXTRA_CMDLINE:-}"
# Verbosity is a build knob, not a source change. The diagnostic image keeps
# -enable_kprintf_spam; the fast one sets KPRINTF_SPAM= (empty) because every
# extra line costs 115200-baud UART time and, with cs_debug=2, megabytes of it.
# Unset means "spam", so a hand build keeps the old command line.
KPRINTF_SPAM="${KPRINTF_SPAM--enable_kprintf_spam}"
[ -n "$KPRINTF_SPAM" ] && KPRINTF_SPAM="$KPRINTF_SPAM "
# lavender: match the command line of the image that last booted to the
# lockscreen. disable_kprintf_output=1 keeps the boot log off the panel
# (the SYSTEM ring still captures it: 0143 appends before the mute check),
# and sweetfb_stayon=1 keeps the display from blanking mid-boot.
if [ "$BOARD_LC" = lavender ]; then
	CMDLINE="debug=0x14e serial=1 disable_kprintf_output=1 cpus=4 sweetfb_native=1 sweetfb_stayon=1 sweetfb_hz=60 sweetsample=- sweetcpufreq=0 sweetbattery=0 rd=$ROOTDEV emmcroot=userdata rootrw=1 amfi_get_out_of_my_way=1 amfi_allow_any_signature=1 ignition_level=1 wdt_test=no_user_checkin wdt=-1 -v keepsyms=1 $BOARD_LC $EXTRA_CMDLINE"
else
	CMDLINE="debug=0x14e serial=3 disable_kprintf_output=0 ${KPRINTF_SPAM}sweetfb_native=1 sweetcpufreq=0 sweetbattery=0 rd=$ROOTDEV -v keepsyms=1 $BOARD_LC $EXTRA_CMDLINE"
fi

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
		python3 ../tools/mkadt.py out/$BOARD_LC.adt --board "$BOARD_LC"
		python3 ../tools/mkpayload.py "$PAYLOAD" out/$BOARD_LC.adt out/payload.bin --cmdline "$CMDLINE" --ramdisk out/init.macho
	fi
fi

# Append payload to Image after _end (allows ABL to load payload regardless of ramdisk handling)
if [ -n "$PAYLOAD" ] && [ -f "out/payload.bin" ]; then
	align_size=$(( (size + 0xfff) & ~0xfff ))
	truncate -s "$align_size" out/Image
	cat out/payload.bin >> out/Image
	python3 -c 'import os,struct,sys; f=open(sys.argv[1],"r+b"); f.seek(16); f.write(struct.pack("<Q", os.path.getsize(sys.argv[1])))' out/Image
	# The shim reads this line from the payload, not from the boot.img header:
	# print it so a build that silently boots the ramdisk probe cannot pass as
	# an iOS-root build.
	python3 ../tools/payload_cmdline.py --payload out/payload.bin
fi

# Package as Image.gz-dtb (standard SDM660 kernel format for Qualcomm ABL)
if [ -n "${IMAGE_GZ:-}" ] && [ -f "$IMAGE_GZ" ]; then
	# Pre-compressed kernel, e.g. a zopfli stream when the ABL download limit is
	# smaller than gzip -9 produces (whyred's rejects more than ~6 MiB).
	cp "$IMAGE_GZ" out/Image.gz
else
	gzip -9 -c out/Image > out/Image.gz
fi
if [ -f "../resources/${BOARD_LC}_dtb.bin" ]; then
	cat "../resources/${BOARD_LC}_dtb.bin" >> out/Image.gz
fi

# Packaging: wrap in boot.img using tools/mkbootimg.py with SDM660/SDM636 parameters
# The dummy cpio only fills the ramdisk field; the real ramdisk rides inside the
# payload. Dropping it saves one whole 4 KiB page, which matters for whyred: the
# ABL rejects a single download over 6 MiB.
if [ "${NO_DUMMY_RAMDISK:-0}" = "1" ]; then
	RD_ARG=""
else
	RD_ARG="--ramdisk $RAMDISK"
fi

python3 ../tools/mkbootimg.py \
	--kernel out/Image.gz \
	$RD_ARG \
	--output out/shim-boot.img \
	--base 0x00000000 \
	--kernel_offset 0x00008000 \
	--ramdisk_offset 0x02000000 \
	--tags_offset 0x00000100 \
	--pagesize 4096 \
	--header_version 0 \
	--cmdline "$CMDLINE"

# Hand-over check. whyred's ABL refuses a single download over 6 MiB, and the
# command line that decides which root is booted lives in the payload, not in
# this header, so verify the image that will actually be flashed. 0173 was
# handed over as an "iOS root" image that booted the ramdisk probe because the
# payload still said rd=md0.
img_size=$(wc -c < out/shim-boot.img)
img_limit=6291456
if [ "$BOARD_LC" = lavender ]; then
	# lavender's ABL accepted a 6705152-byte image; keep a margin below that
	img_limit=7340032
fi
if [ "$img_size" -gt "$img_limit" ]; then
	echo "error: out/shim-boot.img is $img_size bytes; $BOARD_LC limit is $img_limit" >&2
	exit 1
fi
python3 ../tools/payload_cmdline.py --image out/shim-boot.img \
	--require "rd=$ROOTDEV"
python3 -c 'import hashlib,sys; d=open(sys.argv[1],"rb").read(); print("%s  %d bytes  sha256 %s" % (sys.argv[1], len(d), hashlib.sha256(d).hexdigest()))' out/shim-boot.img

# Host render of the console layout
$HOST_CC -O2 -Wall -I. $BOARD_DEFINE -o out/host_test console.c host_test.c && ./out/host_test

# Host unit test for Mach-O segment unroller and boot_args
$HOST_CC -O2 -Wall -I. -o out/loader_test loader_test.c loader.c && ./out/loader_test

# Unit tests and patch checks
HOST_CC="$HOST_CC" OUT_DIR="$PWD/out" bash ../tests/run_tests.sh

ls -l out/Image out/shim-boot.img
