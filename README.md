# ios-lavender

**Experimental iOS 16 bring-up on the Redmi Note 7.**

This is a port of Apple's open-source XNU kernel (xnu-8796) to the Xiaomi **Redmi Note 7** (`lavender`, Qualcomm **SDM660** / Snapdragon 660), using the iPhone X's iOS 16.7.16 (20H392) userland. Hardware photos verify XNU, launchd and usermanagerd. Reaching backboardd and SpringBoard on Lavender is still bring-up work; see the active handoff in [NOTES.md](NOTES.md).

> Experimental bring-up. It installs into the `userdata` partition and so **erases Android's user data**. Nothing is flashed to the boot partitions; the kernel is started from RAM with `fastboot boot`.

## What works

| Area | State |
| --- | --- |
| Boot | Stock Xiaomi ABL → `fastboot boot` of our shim (boot image v0/v1) → XNU. Boot partitions are not touched |
| CPU | Single Cortex-A73 Gold boot baseline (`cpus=1`), 16K pages, GICv3 and timers. The ADT describes four Gold cores (`0x100`–`0x103`); SMP needs its own hardware verification |
| Memory | DRAM physical base `0x80000000`, safe kernel budget `0xa0000000` + 1 GiB |
| Storage | eMMC 5.1 host controller (Qualcomm SDHC v5 at `0x0c0c4000`, GIC SPI 110) |
| iOS userland | iPhone X 16.7.16 root filesystem; launchd and system daemons |
| Display | Verified framebuffer console at `0x9d400000` (1080×2340, 32-bit `a8r8g8b8`); IOSurface/IOMobileFramebuffer drivers are included, GUI presentation is pending |
| Device identity | ADT advertises iPhone X (`iPhone10,3` / `D22AP`) and copies its real `/product` properties |

## Hardware architecture & differences

| Component | Redmi Note 7 (lavender / SDM660) |
| --- | --- |
| CPU | Octa-core Kryo 260 (4x Cortex-A73 Gold @ 2.2 GHz + 4x Cortex-A53 Silver @ 1.84 GHz) |
| 16K Granule | Supported on Gold cluster (`0x100`–`0x103`); Silver cluster is parked via PSCI `CPU_ON` |
| Interrupt Controller | ARM GICv3: GICD `0x17a00000`, GICR `0x17b00000` |
| Timers | VTIMER INTID 19 (GIC_PPI 3), PTIMER INTID 18 (GIC_PPI 2) |
| Storage | eMMC 5.1 (`sdhc_1` at `0x0c0c4000`, Qualcomm SDCC v5) |
| Framebuffer | `0x9d400000`, 1080x2340, 32-bit `a8r8g8b8`, stride 4320 bytes |
| Power / Reset | PS_HOLD `0x010ac000`, IMEM restart reason `0x146bf65c` |

## How it boots

```
ABL (stock Xiaomi bootloader, unlocked)
  -> fastboot boot shim-boot.img        (Android boot image, RAM only)
     -> shim on Silver CPU0: prints MMFR0/MIDR, wakes Gold CPU 0x100 via PSCI, parks
        -> shim on Gold: GICv3 setup, timer unmask, locates payload
           -> XNU Mach-O + Apple Device Tree (ADT) + boot-args
              -> XNU -> SDHCI eMMC driver -> HFS+ root in userdata
                  -> dyld + shared cache -> launchd -> usermanagerd (observed)
                     -> backboardd + SpringBoard (goal)
```

There is no serial console. The kernel draws its console into ABL's framebuffer; with `sweetfb_noconsole=1` it stops after the first iOS frame (panics are still drawn). The kernel also keeps sweet's persistent log ring at the end of `userdata` and its user stack sampler (`SMP name tN pc .. bt ..`).

## How to do it yourself

You need a Redmi Note 7 (`lavender`, SDM660) with an **unlocked bootloader**, a host with `fastboot`, and a GitHub account for the build. Creating a new installation erases `userdata`; an existing iOS root should receive its preparation tweaks in place.

### 1. Build the kernel

The kernel builds on GitHub's macOS runners (macos-14, Xcode 15) with [blacktop/darwin-xnu-build](https://github.com/blacktop/darwin-xnu-build); newer Xcode's `iig` does not match the xnu-8796 IORPC ABI. The workflow then compiles the Lavender shim on Ubuntu and fetches only the matching iPhone X device-tree member for `/product`. Fork the repo and run on `lavender`:

```sh
gh workflow run xnu.yml -R YOU/ios-lavender --ref lavender -f machine_config=LAVENDER
gh run download RUN_ID -R YOU/ios-lavender -n lavender-shim -D out/lavender-shim
gh run download RUN_ID -R YOU/ios-lavender -n xnu-kernel -D out/lavender-kernel
```

A build takes about 8 minutes. If the artifact upload times out on GitHub's side, rerun the job. Check that the kernel has no LSE left (`llvm-objdump -d --mattr=+v8.5a ... | grep -P '\t(cas|ldadd|ldset|ldclr|swp|ldapr)'`; the only hit should be the `K256` SHA table).

### 2. Build the iOS root image

Download the iPhone X (iPhone10,3) iOS 16.7.16 (20H392) IPSW from Apple. On Linux, with `apfs-fuse` (build it from [sgan81/apfs-fuse](https://github.com/sgan81/apfs-fuse) if your distribution has no package), `hfsprogs` and sudo:

```sh
# OS, SystemOS and AppOS disk images (see BuildManifest.plist, D22AP Erase identity)
sudo bash tools/mkiosroot.sh OS.dmg SystemOS.dmg AppOS.dmg ios16-root.img 16G
sudo mount -t hfsplus -o loop,force ios16-root.img /mnt/iosroot
sudo python3 tools/sweet-root-tweaks.py /mnt/iosroot
sudo umount /mnt/iosroot
img2simg ios16-root.img ios16-root.simg
```

The device tree builder copies `/product` from the real iPhone X device tree. The Actions shim job downloads and decompresses `Firmware/all_flash/DeviceTree.d22ap.im4p` automatically. For an existing installation, run `sweet-root-tweaks.py` against the live mounted root after backing up the affected files; a previously flashed image is no longer an exact copy after iOS has written session state.

### 3. Build the boot image

Wrap the checked Actions bundle with the existing Lavender board DTB at `resources/lavender_dtb.bin`. This step uses Python, without local compilation. The output name must be new:

```sh
python3 tools/pack-lavender-boot.py out/lavender-shim resources/lavender_dtb.bin out/shim-boot-lavender.img
```

| Boot-arg | Effect |
| --- | --- |
| `rd=emmcroot0 emmcroot=userdata emmcrw=1 rootrw=1` | HFS+ root on `userdata`, writable |
| `cpus=1` | Current single-Gold-core boot baseline |
| `sweetsample=launchd,usermanagerd,UserEventAgent,backboardd,SpringBoard` | Sample the early-boot services and GUI processes |
| `lavender_suspend_daemons=1` | Explicit diagnostic suspension of mediaserverd/thermalmonitord; disabled by default. UserEventAgent is never suspended by this workaround |
| `sweetfb_noconsole=1` | Stop drawing kernel text over the GUI after the first iOS frame. Leave it out to debug (much slower: every console line is drawn and scrolled by the CPU) |
| `sweetfb_hz=30` | Panel refresh rate of the CPU frame copy |
| `sweettouch=0` | Disable the touch driver |
| `sweethw=1` | Also start sweet's SM7150 key and USB drivers (**unsafe on topaz**, not ported) |
| `launchd_unsecure_cache=1`, `wdt=-1 wdt_test=no_user_checkin` | launchd without AMFI; no hardware watchdog |

### 4. Flash the root and boot

```sh
fastboot -S 512M flash userdata ios16-root.simg   # ~5 minutes; do not unplug
fastboot boot out/shim-boot-lavender.img
```

The shim shows its boot logo, performs the Gold hand-off and starts XNU. Keep the text console enabled during bring-up and capture `exec`, `CRASH` and `SMP` lines. A Lavender lock-screen boot has not yet been verified. To get back to fastboot, hold Power + Volume down.

### Debugging tips

- Stack samples (`SMP name[pid] slide S mh M` then `SMP name tN pc .. bt ..`): symbolicate shared-cache addresses with `ipsw dyld a2s dyld_shared_cache_arm64 <addr - S>`.
- `SWEET TOUCH: ...` lines show each touch bring-up step (clocks, SE2 firmware check, chip id `54:52`).

## Roadmap

The list below is inherited **Topaz reference material**, including its hardware addresses. Current Lavender priorities are prepared-userland boot and hardware verification of the display, SMP, buttons and touch; the Topaz peripheral drivers need board-specific porting.

1. **Touch** (FT5452 at I²C `0x38` on QUP0 SE2 `0x4a88000`, IRQ GPIO80, reset GPIO86, AVDD GPIO36): patch 0083, needs confirmation on the phone.
2. **Buttons**: power and volume down on the PM6125 PON block, volume up on PM6125 GPIO5.
3. **Smoother display**: higher `sweetfb_hz`; check whether the Gold cores run at a low clock without cpufreq (the shim's speed probe) and raise it if so.
4. **Narrow the GIC fix** to the register that actually matters, then move it into XNU.
5. **Real-time clock** from the PMIC RTC or passed in by the shim.
6. **All 8 cores (experiment, later):** build XNU with 4K pages (drop `__ARM_16K_PG__` from `SWEET.h`, as XNU's own `BCM2837` Raspberry Pi 3 config does on Cortex-A53) and describe two clusters, Gold as `P` and Silver as `E`, so the AMP scheduler uses both. Risk: the iOS 16 userland was only ever run on 16K pages (hard-coded 16K in malloc, JIT guard pages, IOSurface, commpage layout), and the 16K assumptions in the shim and sweet's patches have to be checked too. The 16K Gold-only kernel stays as the fallback.
7. **Silver cores as a coprocessor** (idea): run bare-metal shim code on them for the panel frame copy instead of XNU.
8. **6 GB model** (requested): the RAM layout is hard-coded for the 4 GB phone (`mkadt.py --highmem`, 4 GiB at `0x1_40000000`). Have the shim read the DDR ranges from the `/memory` node of the DTB that ABL passes and write them into the ADT and boot args, so one image boots both the 4 GB and the 6 GB versions. Everything else (SoC, panel, touch, battery) is the same hardware.
**GPU plan (2026-10-19):** a kernel Adreno 610 driver (topaz-woa `TopazGpuW` sequences) with an msm DRM style user client for Mesa freedreno, so QuartzCore's OpenGL ES renderer draws the UI on the GPU; Metal for apps after that.

9. **Wi-Fi** (port from [MakrSas/topaz-woa](https://github.com/MakrSas/topaz-woa) `TopazWifi`, which has scan, WPA2 and working internet on Windows): bring up the modem/WLAN firmware through TrustZone, SMEM/QMI and the WCN3988 WLAN commands exactly as `TopazWifi` does, in an XNU driver. Expose it to iOS first as a network interface (IONetworkingFamily, network set by config), then emulate the IO80211 interface that `wifid` talks to, so Settings → Wi-Fi shows the scan list and joins networks.
10. **Speaker** (port from topaz-woa `TopazAudio` (ADSP / AudioReach) + `TopazSpeaker`, see its `docs/P9_audio.md`): ADSP firmware through TrustZone, AudioReach graph to the speaker, and an XNU audio driver that CoreAudio can output to.
11. **Also ready in topaz-woa to port**: brightness (display adapter), USB host/OTG (`TopazOtgDxe` + XHCI), RPM requests (`TopazRpm`: rails, clocks, GPU CX level), SIM/LTE registration (`docs/P9_sim.md`). The GPU work follows `TopazGpuW`.

## Repository layout

| Path | What |
| --- | --- |
| `shim/` | Bare-metal boot shim (C + asm), boot image v4, Gold hand-off, GIC setup, payload loader |
| `xnu/patches/` | XNU patch series (applied in order by CI): sweet's 0001–0079, topaz's 0080+ |
| `.github/workflows/xnu.yml` | Kernel CI build |
| `tools/mkadt.py` | Apple Device Tree for topaz (Gold cores, memory map, MMIO) and the iPhone X identity |
| `tools/mkpayload.py` | Packs kernel + ADT + boot-args into the shim payload |
| `tools/mkiosroot.sh`, `tools/sweet-root-tweaks.py` | Build and adjust the HFS+ iOS root |
| `tools/sweet-crash.py`, `sweet-log.py` | Symbolicate crash reports, read the persistent log |
| `NOTES.md` | sweet's engineering log (history of the original port) |

Not in git: partition backups, live device trees, Apple firmware and root images, build trees.

## topaz kernel patches

| # | What |
| --- | --- |
| 0080 | CPU topology for SM6225; sweet's SM7150 key/touch/USB drivers gated behind `sweethw=1` |
| 0081 | ARMv8.0: `-mcpu=cortex-a53`, no PAN, PFZ without CASA, userland LSE/LDAPR emulation |
| 0082 | Boot CPU is Gold MPIDR `0x100` |
| 0083 | Touch: FocalTech FT5452 on QUP0 SE2 (values from [topaz-woa](https://github.com/MakrSas/topaz-woa) TopazTouch) |

## Key hardware facts (topaz)

- SoC SM6225 "khaje": 4× Kryo 260 Silver (A53 class, MPIDR `0x0`–`0x3`) + 4× Kryo 260 Gold (A73 class, `0x100`–`0x103`). Only Gold has the 16K granule.
- DDR banks `0x40000000`+`0x3bb00000`, `0x80000000`+3 GiB, `0x1_40000000`+4 GiB; all fixed carve-outs are below `0x63900000`.
- GICv3 at `0xf200000` (redistributors `0xf300000`, stride `0x20000`), virtual timer INTID 19, 19.2 MHz.
- Panel: 1080×2400 DSI video mode with DSC, continuous splash at `0x5c000000`.
- Touch: FocalTech FT5452 at `0x38` on QUP0 SE2, raw range 1350×3000 (1.25× the panel).
- More hardware notes for this phone: [MakrSas/topaz-woa](https://github.com/MakrSas/topaz-woa) (`docs/AGENT_BRIEF_drivers.md`).

## Credits

Based on [dipa-studio/ios-sweet](https://github.com/dipa-studio/ios-sweet), which did the hard part: XNU on a Qualcomm phone with the iPhone X userland. Built on Apple's open-source releases (xnu, IOHIDFamily, libpthread, hfs), [BearSSL](https://bearssl.org), [blacktop/darwin-xnu-build](https://github.com/blacktop/darwin-xnu-build), [ipsw](https://github.com/blacktop/ipsw), Qualcomm/Xiaomi's published Linux kernel sources, and the topaz hardware work in [MakrSas/topaz-woa](https://github.com/MakrSas/topaz-woa). iOS itself is Apple's and is not distributed here.
