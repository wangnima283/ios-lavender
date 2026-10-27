/*
 * WHYRED internal eMMC 5.1 root device (polled SDHCI MSM v4, SDCC1).
 *
 * The Qualcomm LK bootloader already powered, clocked and identified the
 * internal eMMC through SDCC1 and hands it over in Transfer State (TRAN) on
 * an 8-bit bus. This driver reuses that state instead of reimplementing the
 * GCC clock tree and SPMI regulators: it maps the single 0x1000 MMIO window
 * at 0x07824000 (Qualcomm vendor core registers at +0x000, standard SDHCI
 * host controller registers at +0x900) and issues polled commands, one at a
 * time, with PIO transfers through the SDHCI buffer port.
 *
 * SDHCI_RESET_ALL (0x01) is issued exactly once, during takeover and before
 * the card is used: it clears the standard SDHCI register file while the GCC
 * clocks and the vendor window (HC_MODE, VENDOR_SPEC, DLL) survive. Takeover
 * then rebuilds HOST_CONTROL/HOST_CONTROL2/TIMEOUT_CONTROL from the hand-off
 * values and switches the vendor core from the bootloader's legacy MCI mode
 * into SDHCI HC mode (CORE_HC_MODE, core + 0x78). It selects the free-running
 * MCLK the HC datapath runs on (CORE_VENDOR_SPEC, HC + 0x10c = 0x07824a0c),
 * keeps the divider at 0 because GCC owns the card rate and sdhci-msm has no
 * usable internal divider, and opens the SDHCI card-clock gate. After
 * takeover only the command and data paths (0x06) are ever reset. All
 * DMA-free, so the only cache maintenance is the (conservative)
 * clean/invalidate of the bounce buffer.
 *
 * One GPT partition is found by name (boot-arg emmcroot=, default
 * "userdata") and exposed as the BSD disk emmcroot0 (remmcroot0 character
 * device), 512-byte logical sectors. Root on it with rd=emmcroot0. Writes
 * are refused unless boot-arg emmcrw=1 is given, and always stay inside the
 * selected partition. The primary GPT header and its partition entry array
 * are verified with their IEEE 802.3 CRC32 before any entry is trusted.
 *
 * The "system" partition is parsed as well, but only to give the persistent
 * console log (xnu/patches/0143) a private 8193-sector area at its start:
 * santoni_sdhci_write_log_blocks() is the one write path that needs neither
 * emmcrw=1 nor the userdata fence, and it refuses anything outside that area.
 */
#if defined(ARM64_BOARD_CONFIG_SANTONI) || defined(ARM64_BOARD_CONFIG_LAVENDER)

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/buf.h>
#include <sys/conf.h>
#include <sys/disk.h>
#include <sys/errno.h>
#include <sys/proc.h>
#include <sys/uio.h>
#include <mach/vm_param.h>
#include <libkern/copyio.h>
#include <miscfs/devfs/devfs.h>
#include <kern/locks.h>
#include <kern/clock.h>
#include <machine/machine_routines.h>
#include <arm/caches_internal.h>
#include <vm/memory_types.h>
#include <vm/pmap.h>
#include <vm/vm_kern.h>
#include <pexpert/pexpert.h>
#include <pexpert/device_tree.h>
#include <libkern/libkern.h>
#include <sys/kernel.h>
#include <os/atomic_private.h>

static void santoni_log_start(void);
static void santoni_fb_test(void);
extern void sweet_touch_init(void);

extern ppnum_t pmap_find_phys(pmap_t pmap, addr64_t va);
/* osfmk/arm/machine_routines.h, not exported to the BSD layer */
extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);

/* ------------------------------------------------- SDHCI standard registers */
#define SDHCI_SDMA_ADDR         0x00
#define SDHCI_BLOCK_SIZE        0x04
#define SDHCI_BLOCK_COUNT       0x06
#define SDHCI_ARGUMENT          0x08
#define SDHCI_TRANSFER_MODE     0x0c
#define SDHCI_COMMAND           0x0e
#define SDHCI_RESPONSE          0x10
#define SDHCI_BUFFER            0x20
#define SDHCI_PRESENT_STATE     0x24
#define SDHCI_HOST_CONTROL      0x28
#define SDHCI_CLOCK_CONTROL     0x2c
#define SDHCI_CLOCK_INT_EN      0x0001
#define SDHCI_CLOCK_INT_STABLE  0x0002
#define SDHCI_CLOCK_CARD_EN     0x0004
/*
 * The SDHCI divider field (CLOCK_CONTROL 15:8, plus 7:6 for SDHCI 3.0)
 * is ignored by this MSM v4 core; the card rate is the GCC
 * sdcc1_apps_clk_src rate, so the field stays 0 (see
 * sdhci_gcc_set_apps()).
 */
#define SDHCI_POWER_CONTROL     0x29
#define SDHCI_SOFTWARE_RESET    0x2f
#define SDHCI_INT_STATUS        0x30
#define SDHCI_INT_ENABLE        0x34
#define SDHCI_SIGNAL_ENABLE     0x38

#define SDHCI_MAKE_BLKSZ(dma, blksz) (((dma) << 12) | ((blksz) & 0xfff))

#define SDHCI_TRNS_DMA          0x0001
#define SDHCI_TRNS_BLK_CNT_EN   0x0002
#define SDHCI_TRNS_AUTO_CMD12   0x0004
#define SDHCI_TRNS_READ         0x0010
#define SDHCI_TRNS_MULTI        0x0020

#define SDHCI_CMD_RESP_LONG     0x01
#define SDHCI_CMD_RESP_SHORT    0x02
#define SDHCI_CMD_RESP_SHORT_BUSY 0x03
#define SDHCI_CMD_CRC            0x08
#define SDHCI_CMD_INDEX          0x10
#define SDHCI_CMD_DATA           0x20
#define SDHCI_MAKE_CMD(c, f)     ((((c) & 0xff) << 8) | ((f) & 0xff))

#define SDHCI_CMD_INHIBIT       0x00000001u
#define SDHCI_DATA_INHIBIT      0x00000002u
/* PRESENT_STATE[20]: DAT0 line level, used for R1b/CMD6 busy polling. */
#define SDHCI_DATA_0_LVL_MASK   0x00100000u
/*
 * PRESENT_STATE[11:8], as defined by the SD Host Controller spec (and
 * sdhci.h in Linux): 8 = write transfer active, 9 = read transfer active,
 * 10 = buffer write enable (space in the FIFO), 11 = buffer read enable
 * (data in the FIFO). This core does not always latch the corresponding
 * interrupt bits into INT_STATUS: the 0152 boot ended CMD8 with
 * INT_STATUS 0x00000000 while PRESENT_STATE was 0x01f8aaf6 - bit 9 (read
 * transfer active) and bit 11 (buffer read enable) set, i.e. the 512-byte
 * block already in the FIFO. Waiting only on INT_STATUS therefore reports a
 * timeout for a transfer that ran. The waits below sample these bits as
 * well; the INT_STATUS bit is still cleared on the way out so a late edge
 * cannot be consumed by the next wait.
 */
#define SDHCI_PSTATE_WR_ACTIVE    0x00000100u
#define SDHCI_PSTATE_RD_ACTIVE    0x00000200u
#define SDHCI_PSTATE_BUF_WR_READY 0x00000400u
#define SDHCI_PSTATE_BUF_RD_READY 0x00000800u
#define SDHCI_PSTATE_DATA_ACTIVE \
	(SDHCI_PSTATE_RD_ACTIVE | SDHCI_PSTATE_WR_ACTIVE)

#define SDHCI_CTRL_DMA_MASK     0x18u
#define SDHCI_CTRL_4BITBUS      0x02u
#define SDHCI_CTRL_8BITBUS      0x20u
#define SDHCI_CTRL_HISPD        0x04u
#define SDHCI_CTRL_BUS_MASK     (SDHCI_CTRL_4BITBUS | SDHCI_CTRL_8BITBUS)
/*
 * HOST_CONTROL2[2:0] is the UHS mode select. sdhci_msm_set_uhs_signaling()
 * clears it for HS200/HS400/SDR104 when the card clock is <= 100 MHz: modes
 * >= 3 expect the DLL/tuned receive path and do not select the fixed feedback
 * clock, so CMD responses cannot be sampled. Preserve VDD_180 (bit 3) while
 * clearing the mode.
 */
#define SDHCI_CTRL_UHS_MASK     0x0007u
/*
 * HOST_CONTROL2[7:6] drive the tuning engine. ABL leaves TUNED_CLK set after
 * its HS400 boot; a legacy 400 kHz host must clear it, or the core keeps
 * sampling through the tuned path the powered-down DLL no longer drives.
 */
#define SDHCI_CTRL_EXEC_TUNING  0x0040u
#define SDHCI_CTRL_TUNED_CLK    0x0080u

#define SDHCI_RESET_ALL         0x01
#define SDHCI_RESET_CMD         0x02
#define SDHCI_RESET_DATA        0x04

#define SDHCI_INT_CC            0x00000001u
#define SDHCI_INT_TC            0x00000002u
#define SDHCI_INT_BWR           0x00000010u
#define SDHCI_INT_BRR           0x00000020u
#define SDHCI_INT_ERR           0x00008000u
/*
 * The specific error bits behind SDHCI_INT_ERR (SDHCI 3.0, one dword at
 * 0x30). They separate "the card never answered" (CMD_TIMEOUT) from "the
 * controller never shifted the command at all" (nothing but the software
 * deadline), which is exactly the ambiguity a bare timeout leaves.
 */
#define SDHCI_INT_CMD_TIMEOUT   0x00010000u     /* no response from the card */
#define SDHCI_INT_CMD_CRC       0x00020000u
#define SDHCI_INT_CMD_END_BIT   0x00040000u
#define SDHCI_INT_CMD_INDEX     0x00080000u
/*
 * The data-side errors. Without these the PIO waits used to return a bare
 * ETIMEDOUT, which cannot distinguish "the card never sent a block" from
 * "the block arrived corrupted" or "the host sampled the wrong bus width".
 */
#define SDHCI_INT_DATA_TIMEOUT  0x00100000u
#define SDHCI_INT_DATA_CRC      0x00200000u
#define SDHCI_INT_DATA_END_BIT  0x00400000u
#define SDHCI_INT_ACMD12ERR     0x01000000u
#define SDHCI_INT_ADMAERR       0x02000000u

/* ------------------------------------------ Qualcomm MSM v4 core registers */
#if defined(ARM64_BOARD_CONFIG_LAVENDER)
#define SDHCI_HC_OFFSET         0x000           /* SDCC v5: native Host Controller at 0 */
#define SDHCI_HOST_VERSION      0xfe
#define CORE_MCI_VERSION        0x318           /* SDCC v5 read-only core ID */
#define CORE_GENERICS           0x320
#else
#define SDHCI_HC_OFFSET         0x900           /* HC base - core base */
#define SDHCI_HOST_VERSION      0xfe
#define CORE_MCI_VERSION        0x50            /* read-only core ID */
#define CORE_GENERICS           0x70
#endif
#define CORE_HC_MODE            0x78
#define CORE_HC_MODE_EN         0x00000001u
/*
 * Vendor-only reset request in CORE_VENDOR_SPEC_FUNC2. sdhci-msm's
 * sdhci_msm_reset_workaround() uses it when the SDHCI SOFTWARE_RESET cannot
 * complete because pending data transfers never drain: bit 21 asks the core
 * to reset, and bit 20 tells it to stop waiting for the AXI side to go idle.
 * The SDHCI reset bits never self-clear on this board, so this is the reset
 * path that is left.
 */
#if defined(ARM64_BOARD_CONFIG_LAVENDER)
#define CORE_VENDOR_SPEC_FUNC2      0x210
#else
#define CORE_VENDOR_SPEC_FUNC2      0x110
#endif
#define CORE_HC_SW_RST_REQ          (1u << 21)
#define CORE_HC_SW_RST_WAIT_IDLE_DIS (1u << 20)
#define CORE_FF_CLK_SW_RST_DIS  0x00002000u
#if defined(ARM64_BOARD_CONFIG_LAVENDER)
#define CORE_PWRCTL_STATUS      0x240
#define CORE_PWRCTL_MASK        0x244
#define CORE_PWRCTL_CLEAR       0x248
#define CORE_PWRCTL_CTL         0x24c
#else
#define CORE_PWRCTL_STATUS      0xdc
#define CORE_PWRCTL_MASK        0xe0
#define CORE_PWRCTL_CLEAR       0xe4
#define CORE_PWRCTL_CTL         0xe8
#endif
#define CORE_PWRCTL_BUS_OFF     0x00000001u
#define CORE_PWRCTL_BUS_ON      0x00000002u
#define CORE_PWRCTL_IO_LOW      0x00000004u
#define CORE_PWRCTL_IO_HIGH     0x00000008u
#define CORE_PWRCTL_INT_MASK    0x0000000fu
#define CORE_PWRCTL_BUS_SUCCESS 0x00000001u
#define CORE_PWRCTL_IO_SUCCESS  0x00000004u
#define CORE_IO_PAD_PWR_SWITCH_EN 0x00008000u    /* BIT(15) */
#define CORE_IO_PAD_PWR_SWITCH    0x00010000u    /* BIT(16) */

#define SDHCI_TIMEOUT_CONTROL   0x2e
#define SDHCI_TIMEOUT_MAX       0x0eu
#define SDHCI_HOST_CONTROL2     0x3e
#define SDHCI_CTRL_VDD_180      0x0008u
#define SDHCI_POWER_ON          0x01u
#define SDHCI_POWER_180         0x0au
#define SDHCI_POWER_CONTROL_VAL (SDHCI_POWER_180 | SDHCI_POWER_ON) /* 0x0b */

#define PWRCTL_TIMEOUT_NS       (100ull * NSEC_PER_MSEC)
/* 1 s: SDCC v5 needs the MCLK running, and the flag can stay set
 * while the core is still settling after ABL handed it over. */
#define RESET_TIMEOUT_NS        (1000ull * NSEC_PER_MSEC)
#define CLOCK_STABLE_TIMEOUT_NS (150ull * NSEC_PER_MSEC)
/*
 * Qualcomm vendor-specific registers CORE_DLL_CONFIG (0x100), CORE_DLL_STATUS
 * (0x108), CORE_VENDOR_SPEC (0x10c), CORE_VENDOR_SPEC_FUNC2 (0x110), and
 * CORE_VENDOR_SPEC_CAPABILITIES0 (0x11c) reside at offsets within hc_mem
 * (0x07824900), mapping physically to 0x07824a00..0x07824a1c (HC + 0x100..0x11c),
 * while core_mem (0x07824000) hosts only legacy MCI registers (MCI_VERSION 0x50,
 * GENERICS 0x70, HC_MODE 0x78, PWRCTL 0xdc..0xe8).
 * HC + 0x10c: vendor clock/DLL control (physically 0x07824a0c). Linux resets
 * it to CORE_VENDOR_SPEC_POR_VAL before HC mode; bits 9:8 select HC datapath.
 * POR value is MCLK sel 2 (0xa9c); LK's board value adds the 1.8V pad power
 * switch (0x18000), which no reset value carries.
 */
#if defined(ARM64_BOARD_CONFIG_LAVENDER)
#define CORE_VENDOR_SPEC        0x20c
#else
#define CORE_VENDOR_SPEC        0x10c
#endif
/*
 * HC + 0x250: CORE_VENDOR_SPEC3. Bit 10 is CORE_FIFO_ALT_EN: with it set the
 * controller routes data through the command-queue "alternative FIFO" and the
 * standard SDHCI buffer (PIO) and SDMA engines never see a byte. Linux's
 * sdhci_msm_init() clears it after every reset ("Ensure SDHCI FIFO is enabled
 * by disabling alternative FIFO"), and this board is handed over in CMDQ mode,
 * which is exactly what leaves it set.
 */
#if defined(ARM64_BOARD_CONFIG_LAVENDER)
#define CORE_VENDOR_SPEC3       0x250
#else
#define CORE_VENDOR_SPEC3       0x1b0
#endif
#define CORE_FIFO_ALT_EN        0x00000400u     /* BIT(10) */
#define CORE_VENDOR_SPEC_POR_VAL 0x00000a9cu
#define CORE_VENDOR_SPEC_BOARD_VAL (CORE_VENDOR_SPEC_POR_VAL | \
	CORE_IO_PAD_PWR_SWITCH_EN | CORE_IO_PAD_PWR_SWITCH)
#define CORE_VENDOR_SPEC_HS400_DIV2_VAL (CORE_VENDOR_SPEC_BOARD_VAL | (1u << 8))
#define CORE_HC_MCLK_SEL_MASK   (3u << 8)
#define CORE_HC_MCLK_SEL_DFLT   (2u << 8)
#define CORE_HC_SELECT_IN_EN    0x00040000u     /* BIT(18) */
#define CORE_HC_SELECT_IN_MASK  (7u << 19)
/*
 * HC + 0x100: DLL configuration (physically 0x07824a00; sdhci-msm.c).
 * CORE_CK_OUT_EN hands the card clock to DLL output; CORE_DLL_EN arms DLL.
 * Selecting a DLL that is not running drives no card clock; output source
 * must follow DLL state: bypass if DLL is in reset/off, DLL output if locked.
 */
#if defined(ARM64_BOARD_CONFIG_LAVENDER)
#define CORE_DLL_CONFIG         0x200
#define CORE_DLL_STATUS         0x208
#else
#define CORE_DLL_CONFIG         0x100
#define CORE_DLL_STATUS         0x108
#endif
#define CORE_DLL_LOCK           0x00000080u     /* BIT(7) */
#define CORE_DLL_EN             0x00010000u     /* BIT(16) */
#define CORE_CK_OUT_EN          0x00040000u     /* BIT(18) */
#define CORE_DLL_PDN            0x20000000u     /* BIT(29) */
#define CORE_DLL_RST            0x40000000u     /* BIT(30) */
#define CORE_DLL_OFF            (CORE_DLL_RST | CORE_DLL_PDN)
/* Legacy MCI window the bootloader drove (LK mmc.c MMC_BOOT_MCI_POWER/CLK). */
#if defined(ARM64_BOARD_CONFIG_LAVENDER)
#define CORE_MCI_POWER          0x324
#define CORE_MCI_CLK            0x328
#else
#define CORE_MCI_POWER          0x000
#define CORE_MCI_CLK            0x004
#endif

/* ---------------------------------------------------- GCC SDCC1 apps clock */
/*
 * sdcc1_apps_clk_src is a Qualcomm RCG in the GCC block (DT gcc@1800000,
 * reg 0x01800000+0x80000; msm8937.dtsi). Only the 4 KiB page holding this
 * RCG is mapped: CMD_RCGR 0x01842004 -> 0x004, CFG_RCGR 0x008, M 0x00c,
 * N 0x010, D 0x014, CBCR 0x018.
 *
 * The apps branch (CBCR bit 0, halt bit 31) and the AHB branch at 0x4201c
 * were left enabled by LK; only the RCG rate is written. Register layout:
 *   CMD_RCGR: bit0 UPDATE, bit1 ROOT_EN, bits4-7 DIRTY, bit31 ROOT_OFF
 *   CFG_RCGR: [4:0] SRC_DIV, [10:8] SRC_SEL, [13:12] MODE
 *   parents:  XO=0 (19.2 MHz), GPLL0=1 (800 MHz), GPLL4=2 (1152 MHz)
 */
#if defined(ARM64_BOARD_CONFIG_LAVENDER)
#define GCC_SDCC1_APPS_PHYS      0x00116000u
#define GCC_SDCC1_APPS_SIZE      0x1000u
#define GCC_SDCC1_APPS_CMD_RCGR  0x02cu
#define GCC_SDCC1_APPS_CFG_RCGR  0x030u
#define GCC_SDCC1_APPS_M         0x034u
#define GCC_SDCC1_APPS_N         0x038u
#define GCC_SDCC1_APPS_D         0x03cu
#define GCC_SDCC1_APPS_CBCR      0x004u
#else
#define GCC_SDCC1_APPS_PHYS      0x01842000u
#define GCC_SDCC1_APPS_SIZE      0x1000u
#define GCC_SDCC1_APPS_CMD_RCGR  0x004u
#define GCC_SDCC1_APPS_CFG_RCGR  0x008u
#define GCC_SDCC1_APPS_M         0x00cu
#define GCC_SDCC1_APPS_N         0x010u
#define GCC_SDCC1_APPS_D         0x014u
#define GCC_SDCC1_APPS_CBCR      0x018u
#endif

#define GCC_CMD_UPDATE           0x00000001u
#define GCC_CMD_ROOT_OFF         0x80000000u
#define GCC_CFG_DIV_MASK         0x0000001fu
#define GCC_CFG_SEL_MASK         0x00000700u
#define GCC_CFG_MODE_MASK        0x00003000u
#define GCC_CBCR_CLK_OFF         0x80000000u

/*
 * RCG divider programs, exactly as clk_rcg2_set_rate() encodes them. The
 * M/N/D registers hold inverted values and SRC_DIV is (2*h - 1):
 *   400 kHz: XO 19.2 MHz, h=12 -> pre_div 23, m=1, n=4, dual edge
 *            M=1, N=~(4-1)=0xfc, D=~clamp(4,1,6)=0xfb, CFG=0x2017
 *   25 MHz:  GPLL0 800 MHz, h=16 -> pre_div 31, m=1, n=2, dual edge
 *            M=1, N=~(2-1)=0xfe, D=~clamp(2,1,2)=0xfd, CFG=0x211f
 */
struct santoni_gcc_rate {
	const char *name;
	uint32_t m, n, d;
	uint32_t cfg;           /* SRC_DIV | SRC_SEL << 8 | MODE << 12 */
};

static const struct santoni_gcc_rate santoni_gcc_rates[] = {
	{ "400 kHz", 0x01u, 0xfcu, 0xfbu, 0x2017u },
	{ "25 MHz",  0x01u, 0xfeu, 0xfdu, 0x211fu },
};
#define SANTONI_GCC_RATE_400K   0
#define SANTONI_GCC_RATE_25M    1

/*
 * The relative card address the bootloader assigned in CMD3. Qualcomm LK/ABL
 * use MMC_RCA 2 (bootloader_lk include/mmc.h), but it is probed, never
 * assumed: CMD13/CMD9/CMD10 are addressed commands and must carry the RCA in
 * bits 31:16, unlike CMD8 (SEND_EXT_CSD), whose argument is all stuff bits.
 */
#define SANTONI_CARD_RCA        2u

/* eMMC R1 status word (JESD84-B51): [12:9] state, bit8 ready, bit7 switch. */
#define SDHCI_R1_STATE_MASK      0x00001e00u
#define SDHCI_R1_STATE_TRAN      4u
#define SDHCI_R1_READY_FOR_DATA  0x00000100u
#define SDHCI_R1_SWITCH_ERROR    0x00000080u
#define SDHCI_R1_ILLEGAL_COMMAND 0x00400000u
#define SDHCI_R1_ERROR           0x00080000u

/* CMD6 SWITCH fields (JESD84-B51): write-byte, EXT_CSD index, value, set. */
#define EXT_CSD_CMD_SET_NORMAL  0u
#define EXT_CSD_WRITE_BYTE      3u
#define EXT_CSD_BUS_WIDTH       183u
#define EXT_CSD_BUS_WIDTH_1     0u
#define EXT_CSD_BUS_WIDTH_4     1u
#define EXT_CSD_BUS_WIDTH_8     2u
#define EXT_CSD_HS_TIMING       185u
#define EXT_CSD_TIMING_HS       1u

/* --------------------------------------------------------------- transfers */
#define SDHCI_BLOCK             512u
#define BOUNCE_SIZE             (1u << 20)
/*
 * SYSTEM partition log area used by the persistent console log
 * (bsd/dev/santoni_sdhci.c + xnu/patches/0143): sector 0 carries the
 * santoni_log_header, sectors 1..8192 the 4 MiB circular ring.
 */
#define SANTONI_LOG_SYSTEM_BLOCKS 8193u
#define DMA_BOUNCE              0x0000
#define CMD_TIMEOUT_NS          (100ull * NSEC_PER_MSEC)
#define DATA_TIMEOUT_NS         (2ull * NSEC_PER_SEC)
#define CMD6_BUSY_TIMEOUT_NS    (1ull * NSEC_PER_SEC)

static struct {
	vm_offset_t regs;                       /* core base, HC at +0x900 */
	vm_offset_t gcc;                        /* GCC sdcc1_apps RCG page */
	vm_offset_t dma;                        /* kernel VA of the bounce buffer */
	uint64_t dma_pa;
	uint64_t last_cmd_ns;                   /* duration of the last command */
	char part[64];                          /* GPT partition name */
	uint64_t first_block, nblocks;          /* root partition, 512-byte sectors */
	uint64_t sys_first_block, sys_nblocks;  /* "system" partition, 512-byte sectors */
	uint32_t sec_count;                     /* EXT_CSD SEC_COUNT, 512-byte units */
	uint32_t rca;                           /* RCA the card answered with */
	int cmd1_error;                         /* last CMD1 result (probe) */
	uint32_t cmd1_ocr;                      /* OCR of the last CMD1 poll */
	uint32_t secsize;                       /* logical sector size for BSD */
	uint8_t saved_ctrl;                     /* HOST_CONTROL at hand-off */
	uint16_t saved_hc2;                     /* HOST_CONTROL2 at hand-off */
	uint8_t saved_timeout;                  /* TIMEOUT_CONTROL at hand-off */
	bool enumerated;                        /* card is 25 MHz, 8-bit, HS */
	bool log_ok;                            /* SYSTEM log area is safe */
	bool writable;                          /* boot-arg emmcrw=1 */
	int bmajor, cmajor;
	dev_t bdev;
	bool ready;
} se;

/* One-word verdict of the LBA0 data self-test, kept for the failure line so a
 * screen that scrolled cannot hide it. */
static char santoni_lba0_note[64] = "not-run";
/* Result of the last data command, so the one summary line that survives a
 * scrolled console can say what the card actually answered. */
static uint32_t santoni_data_r1;
static int santoni_data_error;
/* Engine that actually moved LBA 0 (true = PIO). The 0164 boot proved the
 * data path works, then lost it again on the very next SDMA command. */
static bool santoni_lba0_pio = true;

static LCK_GRP_DECLARE(santoni_sdhci_grp, "santoni_sdhci");
static LCK_MTX_DECLARE(santoni_sdhci_lock, &santoni_sdhci_grp);

static inline uint32_t
hc_rd(uint32_t off)
{
	return *(volatile uint32_t *)(se.regs + SDHCI_HC_OFFSET + off);
}

static inline void
hc_wr(uint32_t off, uint32_t val)
{
	*(volatile uint32_t *)(se.regs + SDHCI_HC_OFFSET + off) = val;
}

static inline uint16_t
hc_rd16(uint32_t off)
{
	return *(volatile uint16_t *)(se.regs + SDHCI_HC_OFFSET + off);
}

static inline void
hc_wr16(uint32_t off, uint16_t val)
{
	*(volatile uint16_t *)(se.regs + SDHCI_HC_OFFSET + off) = val;
}

static inline uint8_t
hc_rd8(uint32_t off)
{
	return *(volatile uint8_t *)(se.regs + SDHCI_HC_OFFSET + off);
}

static inline void
hc_wr8(uint32_t off, uint8_t val)
{
	*(volatile uint8_t *)(se.regs + SDHCI_HC_OFFSET + off) = val;
}

/*
 * BLOCK_SIZE (0x04) and BLOCK_COUNT (0x06) are the two halves of one 32-bit
 * register in SDHCI 3.0: size in 15:0, count in 31:16. The v5 core latches
 * the pair on a 32-bit access; two 16-bit writes can leave the count at zero,
 * after which the transfer never starts - no DATA_TIMEOUT, no DAT activity and
 * CMD_INHIBIT stays set, which is exactly the wedge every boot shows.
 */
static inline void
sdhci_set_block(uint32_t count)
{
	hc_wr(SDHCI_BLOCK_SIZE,
	    (count << 16) | SDHCI_MAKE_BLKSZ(0, SDHCI_BLOCK));
}

/* Linux sdhci_msm_init(): keep the standard FIFO selected. */
static inline void
sdhci_v5_standard_fifo(void)
{
	uint32_t vs3 = hc_rd(CORE_VENDOR_SPEC3);

	if (vs3 & CORE_FIFO_ALT_EN) {
		hc_wr(CORE_VENDOR_SPEC3, vs3 & ~CORE_FIFO_ALT_EN);
	}
}

static inline uint32_t
core_rd(uint32_t off)
{
	return *(volatile uint32_t *)(se.regs + off);
}

static inline void
core_wr(uint32_t off, uint32_t val)
{
	*(volatile uint32_t *)(se.regs + off) = val;
}

/*
 * GCC registers are mapped read/write, uncached and outside the SDHCI reset
 * domain. They are not touched until sdhci_gcc_set_apps() validates the
 * branch/root state.
 */
static inline uint32_t
gcc_rd(uint32_t off)
{
	return *(volatile uint32_t *)(se.gcc + off);
}

static inline void
gcc_wr(uint32_t off, uint32_t val)
{
	*(volatile uint32_t *)(se.gcc + off) = val;
}

/* PIO is CPU-driven, but the bounce buffer is shared with every later user of
 * this driver; keep it clean/invalidated like the UFS driver does for DMA. */
static void
dma_sync(uint32_t len, bool is_read)
{
	flush_dcache64((addr64_t)(se.dma_pa + DMA_BOUNCE), len, is_read);
}

static int
sdhci_poll_state(uint32_t mask, uint64_t timeout_ns)
{
	uint64_t deadline;

	nanoseconds_to_absolutetime(timeout_ns, &deadline);
	deadline += mach_absolute_time();
	while (hc_rd(SDHCI_PRESENT_STATE) & mask) {
		if (mach_absolute_time() > deadline) {
			return ETIMEDOUT;
		}
		delay(1);
	}
	return 0;
}

/*
 * Record how long the command frame took. Probe step 2 uses it to prove the
 * GCC RCG write reached the card: CMD0 takes ~0.5 us at the inherited
 * 384 MHz and ~130 us at 400 kHz, independent of whether the card answers.
 */
static void
sdhci_cmd_stamp(uint64_t start)
{
	uint64_t ns = 0;

	absolutetime_to_nanoseconds(mach_absolute_time() - start, &ns);
	se.last_cmd_ns = ns;
}

/*
 * Issue one command and wait for Command Complete.
 *
 * The status register is cleared before the command is written: the
 * bootloader leaves Command Complete (and possibly an error) set, and a stale
 * bit would otherwise be read as this command's result. Every failure prints
 * the argument, the raw status (with the specific error bits named) and
 * PRESENT_STATE, because "no response", "CRC error" and "the command never
 * shifted" are one indistinguishable timeout without them.
 */
static int
sdhci_cmd(uint8_t index, uint32_t arg, uint16_t tr_mode, uint16_t flags,
    uint32_t *resp)
{
	uint64_t deadline, start;
	int error;

	se.last_cmd_ns = 0;
	error = sdhci_poll_state(SDHCI_CMD_INHIBIT | SDHCI_DATA_INHIBIT,
	    CMD_TIMEOUT_NS);
	if (error) {
		{
			uint32_t ps = hc_rd(SDHCI_PRESENT_STATE);

			printf("WHYRED SDHCI: command %u blocked (arg "
			    "0x%08x, INT_STATUS 0x%08x, PRESENT_STATE "
			    "0x%08x:%s%s%s DAT0 %s)\n", index, arg,
			    hc_rd(SDHCI_INT_STATUS), ps,
			    (ps & SDHCI_CMD_INHIBIT) ? " CMD-inhibit" : "",
			    (ps & SDHCI_DATA_INHIBIT) ? " DAT-inhibit" : "",
			    (ps & SDHCI_PSTATE_DATA_ACTIVE) ? " data-active" : "",
			    (ps & SDHCI_DATA_0_LVL_MASK) ? "high" : "low");
		}
		return EBUSY;
	}

	hc_wr(SDHCI_INT_STATUS, hc_rd(SDHCI_INT_STATUS));
	hc_wr(SDHCI_ARGUMENT, arg);
	/*
	 * SDHCI 3.0 defines the 32-bit view at 0x0C too: command in bits 31:16,
	 * transfer mode in 15:0. The v5 core samples the pair from one access,
	 * so a separate 16-bit write can lose READ/DMA/BLK_CNT_EN and leave the
	 * command waiting for a data phase that never begins.
	 */
	start = mach_absolute_time();
	hc_wr(SDHCI_TRANSFER_MODE,
	    ((uint32_t)SDHCI_MAKE_CMD(index, flags) << 16) | tr_mode);

	nanoseconds_to_absolutetime(CMD_TIMEOUT_NS, &deadline);
	deadline += mach_absolute_time();
	for (;;) {
		uint32_t st = hc_rd(SDHCI_INT_STATUS);

		if (st & SDHCI_INT_ERR) {
			sdhci_cmd_stamp(start);
			hc_wr(SDHCI_INT_STATUS, st);
			printf("WHYRED SDHCI: command %u failed (arg 0x%08x, "
			    "INT_STATUS 0x%08x%s%s%s%s, PRESENT_STATE "
			    "0x%08x, %llu ns)\n", index, arg, st,
			    (st & SDHCI_INT_CMD_TIMEOUT) ? " cmd-timeout" : "",
			    (st & SDHCI_INT_CMD_CRC) ? " cmd-crc" : "",
			    (st & SDHCI_INT_CMD_END_BIT) ? " cmd-end-bit" : "",
			    (st & SDHCI_INT_CMD_INDEX) ? " cmd-index" : "",
			    hc_rd(SDHCI_PRESENT_STATE),
			    (unsigned long long)se.last_cmd_ns);
			return EIO;
		}
		if (st & SDHCI_INT_CC) {
			sdhci_cmd_stamp(start);
			hc_wr(SDHCI_INT_STATUS, SDHCI_INT_CC);
			if (resp) {
				*resp = hc_rd(SDHCI_RESPONSE);
			}
			return 0;
		}
		if (mach_absolute_time() > deadline) {
			sdhci_cmd_stamp(start);
			printf("WHYRED SDHCI: command %u timed out (arg "
			    "0x%08x, INT_STATUS 0x%08x, PRESENT_STATE 0x%08x, "
			    "CMD_INHIBIT %u, %llu ns)\n", index, arg, st,
			    hc_rd(SDHCI_PRESENT_STATE),
			    hc_rd(SDHCI_PRESENT_STATE) & SDHCI_CMD_INHIBIT,
			    (unsigned long long)se.last_cmd_ns);
			return ETIMEDOUT;
		}
		delay(1);
	}
}

/* Name the error bits a data-phase wait saw, plus the state it saw them in. */
static void
sdhci_data_failure(const char *what, uint32_t st)
{
	printf("WHYRED SDHCI: %s: INT_STATUS 0x%08x%s%s%s%s%s%s%s, "
	    "PRESENT_STATE 0x%08x\n", what, st,
	    (st & SDHCI_INT_ERR) ? " err" : "",
	    (st & SDHCI_INT_CMD_TIMEOUT) ? " cmd-timeout" : "",
	    (st & SDHCI_INT_CMD_CRC) ? " cmd-crc" : "",
	    (st & SDHCI_INT_DATA_TIMEOUT) ? " data-timeout" : "",
	    (st & SDHCI_INT_DATA_CRC) ? " data-crc" : "",
	    (st & SDHCI_INT_DATA_END_BIT) ? " data-end-bit" : "",
	    (st & (SDHCI_INT_ACMD12ERR | SDHCI_INT_ADMAERR)) ?
	    " acmd12/adma-err" : "",
	    hc_rd(SDHCI_PRESENT_STATE));
	/*
	 * A zero status with DAT line active is not enough to tell a wrong bus
	 * width from a transfer that never started. Print what the transfer
	 * registers actually hold, so the next log says whether the block
	 * size, the transfer mode (DMA/read bits) and the host width latched.
	 */
	printf("WHYRED SDHCI: %s: BLOCK_SIZE 0x%04x BLOCK_COUNT 0x%04x "
	    "TRANSFER_MODE 0x%04x HOST_CONTROL 0x%02x CLOCK_CONTROL 0x%04x "
	    "TIMEOUT_CONTROL 0x%02x\n", what, hc_rd16(SDHCI_BLOCK_SIZE),
	    hc_rd16(SDHCI_BLOCK_COUNT), hc_rd16(SDHCI_TRANSFER_MODE),
	    hc_rd8(SDHCI_HOST_CONTROL), hc_rd16(SDHCI_CLOCK_CONTROL),
	    hc_rd8(SDHCI_TIMEOUT_CONTROL));
}

/*
 * Data-phase bookkeeping. The PRESENT_STATE fallbacks may only relax the
 * INT_STATUS test when bytes really moved, otherwise a dead transfer would be
 * reported as a finished one. santoni_data_moved is set by the FIFO handshake
 * (one dword at a time) or by an SDMA burst that advanced the system address.
 */
static bool santoni_data_moved;
static bool santoni_sdma_active;
static uint32_t santoni_sdma_start;
static bool santoni_pstate_buf_logged;
static bool santoni_pstate_tc_logged;

static int sdhci_reset_paths(void);
static int sdhci_enable_card_clock(void);
static bool sdhci_ext_csd_sane(void);
static void sdhci_ext_csd_clear(void);

/* Wait for one data interrupt (BRR for reads, BWR for writes); clear it. */
static int
sdhci_wait_data(uint32_t bit, const char *what)
{
	const uint32_t ps_bit = (bit == SDHCI_INT_BRR) ?
	    SDHCI_PSTATE_BUF_RD_READY : SDHCI_PSTATE_BUF_WR_READY;
	uint64_t deadline;

	nanoseconds_to_absolutetime(DATA_TIMEOUT_NS, &deadline);
	deadline += mach_absolute_time();
	for (;;) {
		uint32_t st = hc_rd(SDHCI_INT_STATUS);
		uint32_t ps;

		if (st & SDHCI_INT_ERR) {
			sdhci_data_failure(what, st);
			hc_wr(SDHCI_INT_STATUS, st);
			return EIO;
		}
		if (st & bit) {
			hc_wr(SDHCI_INT_STATUS, bit);
			santoni_data_moved = true;
			return 0;
		}
		ps = hc_rd(SDHCI_PRESENT_STATE);
		if (ps & ps_bit) {
			if (!santoni_pstate_buf_logged) {
				santoni_pstate_buf_logged = true;
				printf("WHYRED SDHCI: %s: %s buffer ready "
				    "in PRESENT_STATE 0x%08x while INT_STATUS "
				    "0x%08x never latched it; polling "
				    "PRESENT_STATE from now on\n", what,
				    (bit == SDHCI_INT_BRR) ? "read" : "write",
				    ps, st);
			}
			hc_wr(SDHCI_INT_STATUS, bit);
			santoni_data_moved = true;
			return 0;
		}
		if (mach_absolute_time() > deadline) {
			sdhci_data_failure(what, st);
			return ETIMEDOUT;
		}
		delay(1);
	}
}

/*
 * Wait for the end of the transfer. all_bytes says the caller's own loop has
 * already moved every byte it asked for (PIO read/write): in that case the
 * data is in the buffer even if the engine state or TC never catches up, and
 * dropping it would only cost another boot. open_ended marks a multi-block
 * transfer with AUTO_CMD12, whose data path must not be reset behind the
 * card's back.
 */
static int
sdhci_wait_transfer_complete(const char *what, bool all_bytes,
    bool open_ended)
{
	uint64_t deadline;
	uint32_t ps;

	nanoseconds_to_absolutetime(DATA_TIMEOUT_NS, &deadline);
	deadline += mach_absolute_time();
	for (;;) {
		uint32_t st = hc_rd(SDHCI_INT_STATUS);

		if (st & SDHCI_INT_ERR) {
			sdhci_data_failure(what, st);
			hc_wr(SDHCI_INT_STATUS, st);
			return EIO;
		}
		if (st & SDHCI_INT_TC) {
			hc_wr(SDHCI_INT_STATUS, SDHCI_INT_TC);
			return 0;
		}
		ps = hc_rd(SDHCI_PRESENT_STATE);
		if (santoni_sdma_active &&
		    hc_rd(SDHCI_SDMA_ADDR) != santoni_sdma_start) {
			santoni_data_moved = true;
		}
		/*
		 * Same latching problem as BRR/BWR: INT_STATUS can stay 0x0
		 * after the last block. Accept the transfer only if bytes
		 * moved and either the caller consumed everything it asked
		 * for or the engine itself reports idle, so a dead transfer
		 * still times out instead of looking successful.
		 */
		if (santoni_data_moved &&
		    (all_bytes || !(ps & SDHCI_PSTATE_DATA_ACTIVE))) {
			if (!santoni_pstate_tc_logged) {
				santoni_pstate_tc_logged = true;
				printf("WHYRED SDHCI: %s: no TC in INT_STATUS "
				    "0x%08x (PRESENT_STATE 0x%08x) but the "
				    "block(s) are already in the buffer; "
				    "treating the transfer as complete\n",
				    what, st, ps);
			}
			/*
			 * Let the data machine retire the transfer. If it is
			 * still inhibiting commands after a generous margin,
			 * drop the path: every byte this request asked for is
			 * already in the buffer. An open-ended transfer keeps
			 * its path so AUTO_CMD12 can still end it on the card.
			 */
			if (!open_ended) {
				uint64_t drain;

				nanoseconds_to_absolutetime(
				    50ull * NSEC_PER_MSEC, &drain);
				drain += mach_absolute_time();
				for (;;) {
					ps = hc_rd(SDHCI_PRESENT_STATE);
					if (!(ps & (SDHCI_PSTATE_DATA_ACTIVE |
					    SDHCI_DATA_INHIBIT))) {
						break;
					}
					if (mach_absolute_time() > drain) {
						printf("WHYRED SDHCI: %s: "
						    "data machine still busy "
						    "(PRESENT_STATE 0x%08x) "
						    "after the last byte; "
						    "resetting the data "
						    "path\n", what, ps);
						(void)sdhci_reset_paths();
						break;
					}
					delay(1);
				}
			}
			hc_wr(SDHCI_INT_STATUS, SDHCI_INT_TC);
			return 0;
		}
		if (mach_absolute_time() > deadline) {
			sdhci_data_failure(what, st);
			return ETIMEDOUT;
		}
		delay(1);
	}
}

static int
sdhci_stop_transmission(void)
{
	return sdhci_cmd(12, 0, 0,
	    SDHCI_CMD_RESP_SHORT_BUSY | SDHCI_CMD_CRC | SDHCI_CMD_INDEX, NULL);
}

/*
 * Vendor reset: the one sequence sdhci-msm uses when the standard reset cannot
 * complete. Returns 0 when the core accepted it and dropped the request.
 */
static int
sdhci_vendor_core_reset(void)
{
	uint32_t f2 = hc_rd(CORE_VENDOR_SPEC_FUNC2);
	int i;

	printf("WHYRED SDHCI: vendor reset: FUNCTION2 0x%08x -> 0x%08x "
	    "(HC_SW_RST_REQ)\n", f2, f2 | CORE_HC_SW_RST_REQ);
	hc_wr(CORE_VENDOR_SPEC_FUNC2, f2 | CORE_HC_SW_RST_REQ);
	for (i = 0; i < 100; i++) {
		if (!(hc_rd(CORE_VENDOR_SPEC_FUNC2) & CORE_HC_SW_RST_REQ)) {
			printf("WHYRED SDHCI: vendor reset: request accepted "
			    "after %d ms\n", i);
			return 0;
		}
		delay(1);
	}

	/*
	 * The core is waiting for AXI to drain, which is exactly what a stuck
	 * transfer never does. Disabling that wait is the documented
	 * workaround; the reset then completes in the background.
	 */
	f2 = hc_rd(CORE_VENDOR_SPEC_FUNC2);
	printf("WHYRED SDHCI: vendor reset: request did not clear in 100 ms; "
	    "disabling the wait for idle (FUNCTION2 0x%08x -> 0x%08x)\n",
	    f2, f2 | CORE_HC_SW_RST_WAIT_IDLE_DIS);
	hc_wr(CORE_VENDOR_SPEC_FUNC2, f2 | CORE_HC_SW_RST_WAIT_IDLE_DIS);
	delay(10);
	if (!(hc_rd(CORE_VENDOR_SPEC_FUNC2) & CORE_HC_SW_RST_REQ)) {
		printf("WHYRED SDHCI: vendor reset: request accepted after "
		    "the wait-idle workaround\n");
		return 0;
	}
	printf("WHYRED SDHCI: vendor reset: request still pending "
	    "(FUNCTION2 0x%08x)\n", hc_rd(CORE_VENDOR_SPEC_FUNC2));
	return ETIMEDOUT;
}

/* Reset the command and data paths and wait for the core to self-clear. */
static int
sdhci_reset_paths(void)
{
	uint64_t deadline;

	hc_wr8(SDHCI_SOFTWARE_RESET, SDHCI_RESET_CMD | SDHCI_RESET_DATA);
	nanoseconds_to_absolutetime(CMD_TIMEOUT_NS, &deadline);
	deadline += mach_absolute_time();
	while (hc_rd8(SDHCI_SOFTWARE_RESET) &
	    (SDHCI_RESET_CMD | SDHCI_RESET_DATA)) {
		if (mach_absolute_time() > deadline) {
			printf("WHYRED SDHCI: path reset did not complete "
			    "(SOFTWARE_RESET 0x%02x)\n",
			    hc_rd8(SDHCI_SOFTWARE_RESET));
			return ETIMEDOUT;
		}
		delay(1);
	}
	hc_wr(SDHCI_INT_STATUS, hc_rd(SDHCI_INT_STATUS));
	/*
	 * SOFTWARE_RESET is not enough on this core. The 0159 boot shows
	 * CMD_INHIBIT and DAT_INHIBIT latched with every DAT line and CMD
	 * high - the card is idle, the core is not. Ask it to resample the
	 * lines by stopping and restarting the card clock while nothing can
	 * be in flight.
	 */
	if (!(hc_rd(SDHCI_PRESENT_STATE) &
	    (SDHCI_CMD_INHIBIT | SDHCI_DATA_INHIBIT))) {
		return 0;
	}
	printf("WHYRED SDHCI: path reset left PRESENT_STATE 0x%08x: "
	    "CMD_INHIBIT/DAT_INHIBIT still set\n",
	    hc_rd(SDHCI_PRESENT_STATE));
	(void)sdhci_vendor_core_reset();
	(void)hc_rd(SDHCI_INT_STATUS);
	sdhci_v5_standard_fifo();
	if (!(hc_rd(SDHCI_PRESENT_STATE) &
	    (SDHCI_CMD_INHIBIT | SDHCI_DATA_INHIBIT))) {
		printf("WHYRED SDHCI: vendor reset cleared the inhibit\n");
		return 0;
	}
	printf("WHYRED SDHCI: cycling the card clock to resample the "
	    "lines\n");
	if (sdhci_enable_card_clock() != 0) {
		return ETIMEDOUT;
	}
	if (hc_rd(SDHCI_PRESENT_STATE) &
	    (SDHCI_CMD_INHIBIT | SDHCI_DATA_INHIBIT)) {
		static bool reported;
		if (!reported) {
			reported = true;
			printf("WHYRED SDHCI: core still inhibited after a "
			    "clock cycle (PRESENT_STATE 0x%08x): this core "
			    "cannot be reset in software; later commands will "
			    "report EBUSY\n", hc_rd(SDHCI_PRESENT_STATE));
		}
		return ETIMEDOUT;
	}
	printf("WHYRED SDHCI: stale inhibit cleared by the clock cycle\n");
	return 0;
}

/*
 * Recover the controller after a failed command or data transfer: reset the
 * command and data paths and drop every pending interrupt. CMD12 is only
 * meaningful while an open-ended (multi-block) transfer is running; after a
 * single-block read or write the card has already returned to Transfer, so
 * sending it there just produced a misleading "command 12 failed" line while
 * the real failure (the 512-byte EXT_CSD read) stayed anonymous.
 */
static void
sdhci_recover(bool open_ended)
{
	(void)sdhci_reset_paths();
	if (!open_ended) {
		return;
	}
	printf("WHYRED SDHCI: recover: stopping an open-ended transfer\n");
	(void)sdhci_stop_transmission();
	(void)sdhci_reset_paths();
}

/*
 * Polled PIO read of count 512-byte blocks. index is 18 (multi-block) or
 * 8/17 (single block). Data is assembled from the FIFO port, 128 words per
 * block.
 *
 * For single-block reads (count == 1), CMD17 is used without AUTO_CMD12 or
 * BLK_CNT_EN; the card returns to TRAN automatically after 1 block.
 * For multi-block reads (count > 1), CMD18 is used with AUTO_CMD12 so the
 * host controller automatically terminates the streaming data transfer.
 */
static int
sdhci_read_pio_once(uint8_t index, uint32_t arg, uint32_t count, void *buf)
{
	const char *tag = (index == 18) ? "CMD18 block" :
	    ((index == 17) ? "CMD17 block" : "CMD8 EXT_CSD");
	uint16_t tr_mode;
	uint32_t *dst = (uint32_t *)buf;
	uint32_t r1 = 0;
	uint32_t zero = 0;
	int error;

	if (count == 0 || count > BOUNCE_SIZE / SDHCI_BLOCK) {
		return EINVAL;
	}

	santoni_data_moved = false;
	santoni_sdma_active = false;

	if (index == 18 && count == 1) {
		index = 17;
	} else if (index == 17 && count > 1) {
		index = 18;
	}

	sdhci_set_block(count);

	if (index == 18) {
		tr_mode = SDHCI_TRNS_READ | SDHCI_TRNS_BLK_CNT_EN |
		    SDHCI_TRNS_MULTI | SDHCI_TRNS_AUTO_CMD12;
	} else {
		tr_mode = SDHCI_TRNS_READ | SDHCI_TRNS_BLK_CNT_EN;
	}
	error = sdhci_cmd(index, arg, tr_mode,
	    SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX |
	    SDHCI_CMD_DATA, &r1);
	if (error) {
		sdhci_recover(index == 18);
		return error;
	}
	/*
	 * CMD8 carries a response even though it starts a data phase, and the
	 * card rejects it with ILLEGAL_COMMAND/ERROR when it is not in the
	 * state the driver assumes. Printing R1 turns a silent "no data ever
	 * arrived" into the reason the card gave. Other reads are left quiet:
	 * this path runs once per block below.
	 */
	if (index == 8) {
		printf("WHYRED SDHCI: %s: R1 0x%08x (state %u)%s%s\n", tag,
		    r1, (r1 & SDHCI_R1_STATE_MASK) >> 9,
		    (r1 & SDHCI_R1_ILLEGAL_COMMAND) ? " illegal-command" : "",
		    (r1 & SDHCI_R1_ERROR) ? " error" : "");
	}

	/*
	 * One dword per handshake. BUFFER_READ_ENABLE is a FIFO level, not a
	 * block counter: it can clear before the whole 512-byte block has
	 * arrived, so a single 128-word burst was only safe while BRR latched
	 * reliably (it does not on this core). Per-word waiting costs a few
	 * microseconds at 400 kHz and is what the SDHCI spec describes.
	 */
	for (uint32_t w = 0; w < count * (SDHCI_BLOCK / 4); w++) {
		error = sdhci_wait_data(SDHCI_INT_BRR, tag);
		if (error) {
			sdhci_recover(index == 18);
			return error;
		}
		dst[w] = hc_rd(SDHCI_BUFFER);
		if (dst[w] == 0) {
			zero++;
		}
	}
	/*
	 * An all-zero EXT_CSD is impossible: byte 192 is the card's own
	 * revision, 1..8. The 0156 boot read 128 zero words here and kept the
	 * setup because "a byte moved". Treat that as a failed read so the
	 * caller can try the DMA engine instead.
	 */
	if (index == 8 && zero == count * (SDHCI_BLOCK / 4)) {
		printf("WHYRED SDHCI: %s: PIO returned %u zero words; no data "
		    "arrived (PRESENT_STATE 0x%08x)\n", tag, zero,
		    hc_rd(SDHCI_PRESENT_STATE));
		sdhci_recover(false);
		return EIO;
	}
	/* The word loop above moved every byte this read asked for. */
	error = sdhci_wait_transfer_complete(tag, true, index == 18);
	if (error) {
		sdhci_recover(index == 18);
		return error;
	}
	return 0;
}

/*
 * SDMA variant of the same block read. The ABL loaded this kernel through
 * the controller's DMA engine, so it is the path the hardware is known to
 * complete even if this core never raises Buffer Read Ready for PIO.
 */
static int
sdhci_read_sdma(uint8_t index, uint32_t arg, uint32_t count, uint64_t pa)
{
	uint16_t tr_mode = (uint16_t)(SDHCI_TRNS_DMA | SDHCI_TRNS_READ |
	    SDHCI_TRNS_BLK_CNT_EN);
	int error;

	if (count == 0 || count > BOUNCE_SIZE / SDHCI_BLOCK ||
	    pa + (uint64_t)count * SDHCI_BLOCK > 0x100000000ull) {
		return EINVAL;
	}
	if (index == 18 && count == 1) {
		index = 17;
	} else if (index == 17 && count > 1) {
		index = 18;
	}
	if (index == 18) {
		tr_mode |= SDHCI_TRNS_BLK_CNT_EN | SDHCI_TRNS_MULTI |
		    SDHCI_TRNS_AUTO_CMD12;
	}

	santoni_data_moved = false;
	santoni_sdma_active = true;
	santoni_sdma_start = (uint32_t)pa;
	hc_wr(SDHCI_SDMA_ADDR, (uint32_t)pa);
	sdhci_set_block(count);
	error = sdhci_cmd(index, arg, tr_mode,
	    SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX |
	    SDHCI_CMD_DATA, &santoni_data_r1);
	if (error) {
		santoni_sdma_active = false;
		sdhci_recover(index == 18);
		return error;
	}
	/* SDMA: only the engine's own state proves the burst landed. */
	error = sdhci_wait_transfer_complete("SDMA block", false,
	    index == 18);
	santoni_sdma_active = false;
	if (error) {
		sdhci_recover(index == 18);
		return error;
	}
	return 0;
}

/*
 * Read count 512-byte blocks: PIO first (no cache or address setup), then
 * the same transfer through SDMA if the controller never raises the buffer
 * interrupt. Both engines end at the same bounce buffer, so the caller's
 * dma_sync() contract is unchanged.
 */
static int
sdhci_read_pio(uint8_t index, uint32_t arg, uint32_t count, void *buf)
{
	const char *tag = (index == 8) ? "CMD8 EXT_CSD" :
	    ((count == 1) ? "CMD17 block" : "CMD18 block");
	uint64_t pa;
	int error;

	error = sdhci_read_pio_once(index, arg, count, buf);
	if (!error || buf != (void *)(se.dma + DMA_BOUNCE)) {
		return error;
	}

	pa = se.dma_pa + DMA_BOUNCE;
	printf("WHYRED SDHCI: %s: PIO failed (%d), retrying the same block "
	    "with SDMA\n", tag, error);
	dma_sync(count * SDHCI_BLOCK, TRUE);
	error = sdhci_read_sdma(index, arg, count, pa);
	if (!error) {
		dma_sync(count * SDHCI_BLOCK, TRUE);
	}
	return error;
}

/* ------------------------------------------- first data phase probe matrix */
/*
 * whyred's first data phase has never moved a byte: CMD6 accepts the width
 * (R1 0x00000900), CMD8 answers with the same R1, and then INT_STATUS stays
 * 0x00000000 for the whole timeout with DAT_LINE_ACTIVE clear and an empty
 * FIFO. Every wrong hypothesis costs one boot and one screen, so the failed
 * EXT_CSD ladder ends in a compact matrix over the setups that could
 * plausibly differ: bus width (1/4/8), PIO vs SDMA, block-count enable, and
 * HOST_CONTROL2 bit 11 - preset value enable - which the bootloader may have
 * left set. With that bit set the core takes its clock/timing from the preset
 * registers instead of the ones this driver writes for its own 400 kHz mode.
 *
 * Each attempt prints exactly one line, and that line carries the card's own
 * R1 state and PRESENT_STATE: state 4 (TRAN) with DATA_AVAILABLE clear means
 * the card never started the block, DATA_AVAILABLE set means the block is
 * sitting in the FIFO and only the status path is wrong. A successful attempt
 * is kept and the probe continues, so the matrix can be the fix, not just a
 * report.
 */
#define SDHCI_PROBE_TIMEOUT_NS  (250ull * NSEC_PER_MSEC)
#define SDHCI_CTRL_PRESET_VAL_EN 0x0800u

/* One line that names every register the transfer path depends on. */
static void
sdhci_probe_dump(const char *what)
{
	printf("WHYRED SDHCI: %s: HS 0x%02x HC2 0x%04x CLK 0x%04x TMO 0x%02x "
	    "VENDOR 0x%08x DLL 0x%08x/0x%08x\n", what,
	    hc_rd8(SDHCI_HOST_CONTROL), hc_rd16(SDHCI_HOST_CONTROL2),
	    hc_rd16(SDHCI_CLOCK_CONTROL), hc_rd8(SDHCI_TIMEOUT_CONTROL),
	    hc_rd(CORE_VENDOR_SPEC), hc_rd(CORE_DLL_CONFIG),
	    hc_rd(CORE_DLL_STATUS));
}

/*
 * CMD6 the width in, then wait for DAT0 release - without the per-command
 * prints sdhci_switch_ext_csd() makes, because the matrix prints its own.
 */
static int
sdhci_probe_set_width(uint8_t value, uint8_t ctrl_set, uint8_t ctrl_clear)
{
	uint32_t arg = (EXT_CSD_WRITE_BYTE << 24) |
	    ((uint32_t)EXT_CSD_BUS_WIDTH << 16) | ((uint32_t)value << 8) |
	    EXT_CSD_CMD_SET_NORMAL;
	uint32_t r1 = 0;
	uint64_t deadline;
	uint8_t ctrl;
	int error;

	error = sdhci_cmd(6, arg, 0, SDHCI_CMD_RESP_SHORT_BUSY |
	    SDHCI_CMD_CRC | SDHCI_CMD_INDEX, &r1);
	if (error || (r1 & SDHCI_R1_SWITCH_ERROR)) {
		return EIO;
	}
	ctrl = hc_rd8(SDHCI_HOST_CONTROL);
	hc_wr8(SDHCI_HOST_CONTROL, (uint8_t)((ctrl & ~ctrl_clear) | ctrl_set));

	nanoseconds_to_absolutetime(CMD6_BUSY_TIMEOUT_NS, &deadline);
	deadline += mach_absolute_time();
	for (;;) {
		uint32_t st = hc_rd(SDHCI_PRESENT_STATE);

		if (!(st & (SDHCI_CMD_INHIBIT | SDHCI_DATA_INHIBIT)) &&
		    (st & SDHCI_DATA_0_LVL_MASK)) {
			return 0;
		}
		if (mach_absolute_time() > deadline) {
			return ETIMEDOUT;
		}
		delay(1000);
	}
}

/*
 * One CMD8 attempt with the transfer registers already programmed and read
 * back, a short deadline, and no recovery print. On success the block lands
 * in the bounce buffer, so the caller can use it directly.
 */
static int
sdhci_probe_attempt(uint16_t tr_mode, uint32_t *tm_rb, uint32_t *ps_out,
    uint32_t *r1_out)
{
	uint64_t deadline;
	uint32_t st = 0, ps = 0, r1 = 0;
	unsigned int i;
	int error;

	santoni_data_moved = false;
	santoni_sdma_active = false;
	if (tr_mode & SDHCI_TRNS_DMA) {
		santoni_sdma_active = true;
		santoni_sdma_start = (uint32_t)(se.dma_pa + DMA_BOUNCE);
	}
	sdhci_set_block(1);
	hc_wr16(SDHCI_TRANSFER_MODE, tr_mode);
	*tm_rb = hc_rd16(SDHCI_TRANSFER_MODE);

	error = sdhci_cmd(8, 0, tr_mode,
	    SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX |
	    SDHCI_CMD_DATA, &r1);
	*r1_out = r1;
	if (error) {
		*ps_out = hc_rd(SDHCI_PRESENT_STATE);
		return error;
	}

	nanoseconds_to_absolutetime(SDHCI_PROBE_TIMEOUT_NS, &deadline);
	deadline += mach_absolute_time();
	for (;;) {
		st = hc_rd(SDHCI_INT_STATUS);
		ps = hc_rd(SDHCI_PRESENT_STATE);
		if (st & (SDHCI_INT_ERR | SDHCI_INT_BRR | SDHCI_INT_TC) ||
		    (ps & SDHCI_PSTATE_BUF_RD_READY)) {
			break;
		}
		if (mach_absolute_time() > deadline) {
			break;
		}
		delay(1);
	}
	hc_wr(SDHCI_INT_STATUS, st);

	if (st & SDHCI_INT_ERR) {
		*ps_out = ps;
		return EIO;
	}
	if (santoni_sdma_active &&
	    hc_rd(SDHCI_SDMA_ADDR) != santoni_sdma_start) {
		santoni_data_moved = true;
	}
	/*
	 * The matrix exists to find a transfer setup that works, so it must
	 * accept the same two weak signals the real path does: a FIFO level
	 * instead of BRR, and an idle engine instead of TC.
	 */
	if ((st & SDHCI_INT_BRR) || (ps & SDHCI_PSTATE_BUF_RD_READY)) {
		uint32_t zero = 0;

		for (i = 0; i < SDHCI_BLOCK / 4; i++) {
			uint32_t v = hc_rd(SDHCI_BUFFER);

			((uint32_t *)(void *)(se.dma + DMA_BOUNCE))[i] = v;
			if (v == 0) {
				zero++;
			}
		}
		/*
		 * BUFFER_READ_ENABLE can be set before a single byte has
		 * arrived: the 0156 boot read 128 zero words and then called
		 * the transfer complete. An all-zero EXT_CSD block is never
		 * real (REV is 1..8), so treat it as "no data" and let the
		 * matrix try the next engine instead of keeping this setup.
		 */
		if (zero == SDHCI_BLOCK / 4) {
			printf("WHYRED SDHCI: data probe: FIFO returned %u "
			    "zero words; no data arrived (PRESENT_STATE "
			    "0x%08x DAT 0x%x)\n", zero,
			    hc_rd(SDHCI_PRESENT_STATE),
			    hc_rd(SDHCI_PRESENT_STATE) & 0x00f00000u);
			*ps_out = hc_rd(SDHCI_PRESENT_STATE);
			return EIO;
		}
		santoni_data_moved = true;
		*ps_out = hc_rd(SDHCI_PRESENT_STATE);
		return 0;
	}
	if ((st & SDHCI_INT_TC) ||
	    (santoni_data_moved && !(ps & SDHCI_PSTATE_DATA_ACTIVE))) {
		santoni_data_moved = true;
		*ps_out = ps;
		return 0;
	}
	*ps_out = ps;
	return ETIMEDOUT;
}

static int
sdhci_data_matrix(uint32_t *width)
{
	static const struct {
		uint8_t lanes;
		uint8_t ctrl_set;
		uint8_t engine;         /* 0 = PIO, 1 = SDMA */
		uint8_t blk_cnt;
		uint8_t preset_off;
	} cfg[] = {
		/*
		 * SDMA first. The 0156 boot proved the PIO FIFO handshake is
		 * a false positive on this core: BUFFER_READ_ENABLE was set,
		 * 128 words were read, and every one of them was zero while
		 * DAT_LINE_ACTIVE never cleared. ABL loaded this kernel
		 * through the same controller, so the DMA engine is the path
		 * the hardware is known to complete.
		 */
		{ 1, 0, 1, 1, 1 },
		{ 8, SDHCI_CTRL_8BITBUS, 1, 1, 1 },
		{ 1, 0, 1, 0, 1 },
		{ 1, 0, 0, 1, 1 },
		{ 1, 0, 0, 0, 1 },
		{ 1, 0, 0, 1, 0 },
		{ 4, SDHCI_CTRL_4BITBUS, 0, 1, 1 },
		{ 8, SDHCI_CTRL_8BITBUS, 0, 1, 1 },
	};
	uint16_t saved_hc2 = hc_rd16(SDHCI_HOST_CONTROL2);
	unsigned int i;
	int last = EIO;

	sdhci_probe_dump("data probe: start");
	/*
	 * CMD16 first: the block length for CMD8 is fixed at 512 bytes, but a
	 * bootloader that left BLOCKLEN at another value would make the card
	 * reject or mis-frame the read, and the command is free.
	 */
	(void)sdhci_cmd(16, SDHCI_BLOCK, 0,
	    SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX, NULL);
	for (i = 0; i < sizeof(cfg) / sizeof(cfg[0]); i++) {
		const char *engine = cfg[i].engine ? "SDMA" : "PIO";
		uint16_t hc2 = saved_hc2;
		uint16_t tr_mode = SDHCI_TRNS_READ;
		uint32_t tm = 0, ps = 0, r1 = 0;
		int error;

		if (cfg[i].blk_cnt) {
			tr_mode |= SDHCI_TRNS_BLK_CNT_EN;
		}
		if (cfg[i].engine) {
			tr_mode |= SDHCI_TRNS_DMA;
		}
		if (cfg[i].preset_off) {
			hc2 = (uint16_t)(hc2 & ~SDHCI_CTRL_PRESET_VAL_EN);
		}
		hc_wr16(SDHCI_HOST_CONTROL2, hc2);

		error = sdhci_probe_set_width(
		    cfg[i].lanes == 8 ? EXT_CSD_BUS_WIDTH_8 :
		    (cfg[i].lanes == 4 ? EXT_CSD_BUS_WIDTH_4 :
		    EXT_CSD_BUS_WIDTH_1),
		    cfg[i].ctrl_set, SDHCI_CTRL_BUS_MASK);
		if (error) {
			printf("WHYRED SDHCI: data probe #%u %u-bit %s bc=%u "
			    "pre=%u: width switch failed (%d)\n", i + 1,
			    (unsigned)cfg[i].lanes, engine,
			    (unsigned)cfg[i].blk_cnt,
			    (unsigned)!cfg[i].preset_off, error);
			last = error;
			continue;
		}
		if (cfg[i].engine) {
			hc_wr(SDHCI_SDMA_ADDR,
			    (uint32_t)(se.dma_pa + DMA_BOUNCE));
		}
		sdhci_ext_csd_clear();

		error = sdhci_probe_attempt(tr_mode, &tm, &ps, &r1);
		printf("WHYRED SDHCI: data probe #%u %u-bit %s bc=%u pre=%u: "
		    "%s (tm 0x%04x ps 0x%08x r1 0x%08x)\n", i + 1,
		    (unsigned)cfg[i].lanes, engine, (unsigned)cfg[i].blk_cnt,
		    (unsigned)!cfg[i].preset_off,
		    error ? (error == ETIMEDOUT ? "timeout" : "error") : "OK",
		    tm, ps, r1);
		dma_sync(SDHCI_BLOCK, TRUE);
		/*
		 * "A byte moved" is not the same as "the block is the
		 * EXT_CSD". The 0156 boot kept a PIO setup whose payload was
		 * 128 zero words. Only a block that names its own revision,
		 * structure and capacity may end the search.
		 */
		if (sdhci_ext_csd_sane()) {
			*width = cfg[i].lanes;
			printf("WHYRED SDHCI: data probe: EXT_CSD moved at "
			    "%u-bit %s bc=%u pre=%u%s - keeping it\n",
			    (unsigned)cfg[i].lanes, engine,
			    (unsigned)cfg[i].blk_cnt,
			    (unsigned)!cfg[i].preset_off,
			    error ? " (status error ignored)" : "");
			return 0;
		}
		if (!error) {
			{
				const uint8_t *ext =
				    (const void *)(se.dma + DMA_BOUNCE);
				const uint32_t *w =
				    (const void *)(se.dma + DMA_BOUNCE);
				uint32_t sec = 0;

				memcpy(&sec, ext + 212, sizeof(sec));
				printf("WHYRED SDHCI: data probe #%u %u-bit "
				    "%s bc=%u pre=%u: block is not EXT_CSD "
				    "(REV %u STRUCTURE %u SEC_COUNT %u, words "
				    "%08x %08x %08x %08x); trying the next "
				    "setup\n", i + 1, (unsigned)cfg[i].lanes,
				    engine, (unsigned)cfg[i].blk_cnt,
				    (unsigned)!cfg[i].preset_off, ext[192],
				    ext[193], sec, w[0], w[1], w[2], w[3]);
			}
			error = EIO;
		}
		last = error;
		sdhci_recover(false);
	}
	hc_wr16(SDHCI_HOST_CONTROL2, saved_hc2);
	(void)sdhci_probe_set_width(EXT_CSD_BUS_WIDTH_1, 0,
	    SDHCI_CTRL_BUS_MASK);
	sdhci_probe_dump("data probe: no configuration moved a byte");
	return last;
}

/*
 * Polled PIO write of count 512-byte blocks. For count == 1, CMD24 is used.
 * For count > 1, CMD25 is used with AUTO_CMD12.
 */
static int
sdhci_write_pio(uint32_t lba, uint32_t count, const void *buf)
{
	const char *tag = (count == 1) ? "CMD24 block" : "CMD25 block";
	const uint32_t *src = (const uint32_t *)buf;
	uint16_t tr_mode;
	uint8_t index = (count == 1) ? 24 : 25;
	int error;

	if (count == 0 || count > BOUNCE_SIZE / SDHCI_BLOCK) {
		return EINVAL;
	}
	sdhci_set_block(count);

	if (index == 25) {
		tr_mode = SDHCI_TRNS_MULTI | SDHCI_TRNS_BLK_CNT_EN |
		    SDHCI_TRNS_AUTO_CMD12;
	} else {
		tr_mode = SDHCI_TRNS_BLK_CNT_EN;
	}

	error = sdhci_cmd(index, lba, tr_mode,
	    SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX |
	    SDHCI_CMD_DATA, NULL);
	if (error) {
		sdhci_recover(index == 25);
		return error;
	}

	for (uint32_t b = 0; b < count; b++) {
		error = sdhci_wait_data(SDHCI_INT_BWR, tag);
		if (error) {
			sdhci_recover(index == 25);
			return error;
		}
		for (uint32_t i = 0; i < SDHCI_BLOCK / 4; i++) {
			hc_wr(SDHCI_BUFFER, src[b * (SDHCI_BLOCK / 4) + i]);
		}
	}
	error = sdhci_wait_transfer_complete(tag, true, index == 25);
	if (error) {
		sdhci_recover(index == 25);
		return error;
	}
	return 0;
}

/*
 * CMD2 / CMD9 / CMD10: 136-bit R2 response into out[0] = MSB word.
 *
 * CMD2 is broadcast (arg 0); CMD9/CMD10 are addressed, so arg carries the RCA
 * in bits 31:16 (Linux include/linux/mmc/mmc.h: "ac [31:16] RCA R2"). R2 has
 * no opcode byte, so SDHCI_CMD_INDEX must not be set: the controller would
 * compare a nonexistent index and report SDHCI_INT_CMD_INDEX. CRC is part of
 * Linux's MMC_RSP_R2. The MSM v4 core strips the 8-bit CRC and right-aligns
 * the remaining 120 bits, so the payload of the first word sits in bits 23:0;
 * Linux (sdhci_read_rsp_136()) shifts each word left by 8 and folds in the top
 * byte of the next. Without that shift the CID decodes as MID 0x00, a blank
 * PNM and a nonsense serial number.
 */
static int
sdhci_read_response128(uint8_t index, uint32_t arg, uint32_t out[4])
{
	int error = sdhci_cmd(index, arg, 0,
	    SDHCI_CMD_RESP_LONG | SDHCI_CMD_CRC, NULL);
	if (error) {
		sdhci_recover(false);
		return error;
	}
	for (int i = 0; i < 4; i++) {
		uint32_t lo = hc_rd(SDHCI_RESPONSE + 4 * (3 - i));
		uint32_t hi = (i == 3) ? 0u :
		    hc_rd(SDHCI_RESPONSE + 4 * (2 - i));

		out[i] = (lo << 8) | (hi >> 24);
	}
	return 0;
}

/*
 * The vendor core answers only while its GCC clock branch is on.
 * CORE_MCI_VERSION is a read-only ID register that reads 0x100000xx on a live
 * SDCC v4 core; 0 or all-ones means the block is unclocked (or the access
 * never reached it), and then no register in either window can be trusted.
 */
static int
sdhci_core_check(void)
{
#if defined(ARM64_BOARD_CONFIG_LAVENDER)
	uint16_t host_version = hc_rd16(SDHCI_HOST_VERSION);
	if (host_version == 0 || host_version == 0xffffu) {
		printf("WHYRED SDHCI: SDCC v5 HOST_VERSION invalid: 0x%04x\n", host_version);
		return ENXIO;
	}
	printf("WHYRED SDHCI: SDCC v5 core present (HOST_VERSION 0x%04x)\n", host_version);
	return 0;
#else
	if (mci_version == 0 || mci_version == 0xffffffffu) {
		printf("WHYRED SDHCI: vendor core does not answer "
		    "(MCI_VERSION 0x%08x, GENERICS 0x%08x): GCC SDC1 clock "
		    "branch off or block held in reset\n",
		    mci_version, generics);
		return ENXIO;
	}
	printf("WHYRED SDHCI: core MCI_VERSION 0x%08x GENERICS 0x%08x\n",
	    mci_version, generics);
	return 0;
#endif
}

/*
 * Probe step 0: read-only snapshot. This runs before the driver writes any
 * register. The legacy MCI window, vendor window and GCC RCG are readable
 * while LK's HC_MODE is still 0; the standard +0x900 command/clock registers
 * are not decoded until sdhci_enter_hc_mode(), so they are dumped separately
 * after the switch and before RESET_ALL.
 *
 * Commands cannot be sent here: the card clock gate is still LK's
 * (CLOCK_CONTROL 0x0003, CARD_EN off), the inherited card rate is 384 MHz and
 * HOST_CONTROL2 still says UHS=3 with a powered-down DLL. Opening the gate in
 * that state would only reproduce the b6a8788 timeout at an illegal rate. The
 * functional CMD13/CMD5 test therefore belongs to step 2, after the GCC clock
 * and the host receive path are made safe.
 */
static void
sdhci_snapshot_untouched(void)
{
	uint32_t vendor = hc_rd(CORE_VENDOR_SPEC);
	uint32_t dll = hc_rd(CORE_DLL_CONFIG);
	uint32_t status = hc_rd(CORE_DLL_STATUS);

#if defined(ARM64_BOARD_CONFIG_LAVENDER)
	printf("WHYRED SDHCI: probe step 0: VENDOR_SPEC 0x%08x "
	    "(MCLK sel %u, SELECT_IN 0x%x), DLL_CONFIG 0x%08x "
	    "DLL_STATUS 0x%08x\n", vendor,
	    (vendor & CORE_HC_MCLK_SEL_MASK) >> 8,
	    (vendor & CORE_HC_SELECT_IN_MASK) >> 19, dll, status);
#else
	printf("WHYRED SDHCI: probe step 0: HC_MODE 0x%08x, "
	    "MCI_POWER 0x%08x, MCI_CLK 0x%08x, VENDOR_SPEC 0x%08x "
	    "(MCLK sel %u, SELECT_IN 0x%x), DLL_CONFIG 0x%08x "
	    "DLL_STATUS 0x%08x\n", core_rd(CORE_HC_MODE),
	    core_rd(CORE_MCI_POWER), core_rd(CORE_MCI_CLK), vendor,
	    (vendor & CORE_HC_MCLK_SEL_MASK) >> 8,
	    (vendor & CORE_HC_SELECT_IN_MASK) >> 19, dll, status);
#endif
	printf("WHYRED SDHCI: probe step 0: GCC sdcc1_apps CMD 0x%08x "
	    "CFG 0x%08x M 0x%02x N 0x%02x D 0x%02x CBCR 0x%08x\n",
	    gcc_rd(GCC_SDCC1_APPS_CMD_RCGR),
	    gcc_rd(GCC_SDCC1_APPS_CFG_RCGR),
	    gcc_rd(GCC_SDCC1_APPS_M) & 0xffu,
	    gcc_rd(GCC_SDCC1_APPS_N) & 0xffu,
	    gcc_rd(GCC_SDCC1_APPS_D) & 0xffu,
	    gcc_rd(GCC_SDCC1_APPS_CBCR));
}

/* The standard window is only valid after HC mode; this is still pre-reset. */
static void
sdhci_snapshot_standard(const char *tag)
{
	printf("WHYRED SDHCI: probe step 0 %s: POWER_CONTROL 0x%02x, "
	    "CLOCK_CONTROL 0x%04x, HOST_CONTROL 0x%02x, HOST_CONTROL2 0x%04x, "
	    "TIMEOUT_CONTROL 0x%02x, PRESENT_STATE 0x%08x, INT_STATUS 0x%08x, "
	    "SOFTWARE_RESET 0x%02x, HOST_VERSION 0x%04x\n", tag,
	    hc_rd8(SDHCI_POWER_CONTROL), hc_rd16(SDHCI_CLOCK_CONTROL),
	    hc_rd8(SDHCI_HOST_CONTROL), hc_rd16(SDHCI_HOST_CONTROL2),
	    hc_rd8(SDHCI_TIMEOUT_CONTROL), hc_rd(SDHCI_PRESENT_STATE),
	    hc_rd(SDHCI_INT_STATUS), hc_rd8(SDHCI_SOFTWARE_RESET),
	    hc_rd16(SDHCI_HOST_VERSION));
}

/*
 * LK drives SDCC1 through the legacy MCI register set and leaves HC_MODE
 * (core + 0x78) at 0, so the standard SDHCI window at core + 0x900 is not
 * decoded yet. Do not trust the bootloader for this: Linux
 * (sdhci_msm_init(), drivers/mmc/host/sdhci-msm.c) performs the switch itself,
 * writing HC_MODE_EN and then FF_CLK_SW_RST_DIS.
 *
 * CORE_HC_MODE is a plain flop in the core's always-on register block: the
 * write has landed by the time the bus write is acknowledged, so the read-back
 * both flushes the posted write and proves the bit. No settling delay is
 * required before touching the +0x900 window. FF_CLK_SW_RST_DIS is written in
 * a second access, only after HC_MODE_EN has latched, exactly as Linux does
 * it: the hardware log from commit df4395e shows HC_MODE 0x00000001 after a
 * single combined write, i.e. bit 13 was dropped. Bit 13 is what stops the
 * core from resetting itself whenever the SDCLK feedback is not toggling,
 * which is the state the controller is in until a card clock is out - so a
 * dropped bit 13 can hold the command state machine in reset forever.
 */
static int
sdhci_enter_hc_mode(void)
{
#if defined(ARM64_BOARD_CONFIG_LAVENDER)
	uint16_t host_version = hc_rd16(SDHCI_HOST_VERSION);
	if (host_version == 0 || host_version == 0xffffu) {
		printf("WHYRED SDHCI: SDCC v5 HOST_VERSION invalid: 0x%04x\n", host_version);
		return ENXIO;
	}
	printf("WHYRED SDHCI: SDCC v5 in HC mode (HOST_VERSION 0x%04x)\n", host_version);
	return 0;
#else
	uint32_t hc_mode = core_rd(CORE_HC_MODE);
	uint16_t host_version;

	if ((hc_mode & (CORE_HC_MODE_EN | CORE_FF_CLK_SW_RST_DIS)) ==
	    (CORE_HC_MODE_EN | CORE_FF_CLK_SW_RST_DIS)) {
		return 0;
	}

	printf("WHYRED SDHCI: HC_MODE 0x%08x, enabling HC mode\n", hc_mode);
	/* Step 1: HC_MODE_EN, keeping any other vendor bits LK left. */
	core_wr(CORE_HC_MODE, (hc_mode & ~CORE_HC_MODE_EN) | CORE_HC_MODE_EN);
	hc_mode = core_rd(CORE_HC_MODE);

	if (!(hc_mode & CORE_HC_MODE_EN)) {
		/*
		 * Some SDCC revisions do not make bit 0 readable. The switch
		 * happened if the standard window at +0x900 now answers.
		 */
		host_version = hc_rd16(SDHCI_HOST_VERSION);
		if (host_version == 0 || host_version == 0xffffu) {
			printf("WHYRED SDHCI: HC mode did not latch "
			    "(HC_MODE 0x%08x, HOST_VERSION 0x%04x)\n",
			    hc_mode, host_version);
			return ENXIO;
		}
		printf("WHYRED SDHCI: HC_MODE_EN reads back 0 but the HC "
		    "window answers (HOST_VERSION 0x%04x)\n", host_version);
	}
	/* Step 2: the free-running-clock reset guard, after bit 0 is in. */
	core_wr(CORE_HC_MODE, hc_mode | CORE_HC_MODE_EN | CORE_FF_CLK_SW_RST_DIS);
	hc_mode = core_rd(CORE_HC_MODE);
	if (!(hc_mode & CORE_FF_CLK_SW_RST_DIS)) {
		printf("WHYRED SDHCI: FF_CLK_SW_RST_DIS did not stick "
		    "(HC_MODE 0x%08x): the core may self-reset while SDCLK "
		    "feedback is stopped\n", hc_mode);
	}
	printf("WHYRED SDHCI: HC mode enabled (HC_MODE 0x%08x, "
	    "HOST_VERSION 0x%04x)\n", hc_mode, hc_rd16(SDHCI_HOST_VERSION));
	return 0;
#endif
}

/*
 * HC block clock source. CORE_VENDOR_SPEC[9:8] selects the clock the HC
 * datapath runs on; LK leaves it at 0 because it only ever drove the legacy
 * MCI block. With selection 0 the register file still answers (bus clock) and
 * CLOCK_CONTROL can report INT_EN|INT_STABLE, but a command written to 0x0e
 * never leaves the shifter: PRESENT_STATE keeps CMD_INHIBIT set and no
 * completion is ever posted - the exact failure in the df4395e boot log,
 * including CMD0, which needs no card at all.
 *
 * Linux writes CORE_VENDOR_SPEC_POR_VAL in sdhci_msm_probe(), then re-selects
 * free-running MCLK and disables HC_SELECT_IN from msm_hc_select_default()
 * before programming any rate. Takeover does the POR write before HC mode and
 * this selection right after it, so both happen before the first command.
 */
static void
sdhci_select_hc_clock(void)
{
	uint32_t vs = hc_rd(CORE_VENDOR_SPEC);

	vs &= ~CORE_HC_MCLK_SEL_MASK;
	vs |= CORE_HC_MCLK_SEL_DFLT;
	vs &= ~(CORE_HC_SELECT_IN_EN | CORE_HC_SELECT_IN_MASK);
	hc_wr(CORE_VENDOR_SPEC, vs);
	vs = hc_rd(CORE_VENDOR_SPEC);
	printf("WHYRED SDHCI: HC clock source MCLK sel %u (VENDOR_SPEC "
	    "0x%08x)\n", (vs & CORE_HC_MCLK_SEL_MASK) >> 8, vs);
}

/*
 * Card-clock output stage (HC + 0x100). CORE_CK_OUT_EN selects the DLL as
 * the source of SDCLK; a DLL held in reset, powered down or unlocked cannot
 * drive it, so the output must fall back to bypass. Linux never sets
 * CK_OUT_EN outside msm_init_cm_dll() (HS200/HS400 tuning), so bypass is the
 * correct state for the 400 kHz identification and 52 MHz high-speed path this
 * driver uses - unless LK left a locked DLL running. Log the register either
 * way: it is the one clock register the previous boot log never showed.
 */
static void
sdhci_clock_output(void)
{
	uint32_t dll = hc_rd(CORE_DLL_CONFIG);
	uint32_t status = hc_rd(CORE_DLL_STATUS);
	bool dll_live = (dll & CORE_DLL_EN) && !(dll & CORE_DLL_OFF) &&
	    (status & CORE_DLL_LOCK);
	uint32_t want = dll_live ? (dll | CORE_CK_OUT_EN) :
	    ((dll | CORE_DLL_RST | CORE_DLL_PDN) & ~CORE_CK_OUT_EN);

	printf("WHYRED SDHCI: CORE_DLL_CONFIG 0x%08x CORE_DLL_STATUS 0x%08x "
	    "(DLL_EN %u, PDN %u, RST %u, LOCK %u, CK_OUT_EN %u)\n", dll, status,
	    (dll & CORE_DLL_EN) ? 1u : 0u, (dll & CORE_DLL_PDN) ? 1u : 0u,
	    (dll & CORE_DLL_RST) ? 1u : 0u, (status & CORE_DLL_LOCK) ? 1u : 0u,
	    (dll & CORE_CK_OUT_EN) ? 1u : 0u);
	if (want != dll) {
		hc_wr(CORE_DLL_CONFIG, want);
		printf("WHYRED SDHCI: card clock output %s (CORE_DLL_CONFIG "
		    "0x%08x)\n", dll_live ? "selected from the locked DLL" :
		    "bypasses the powered-down DLL", hc_rd(CORE_DLL_CONFIG));
	}
}

/*
 * Put the host receive path into the mode sdhci-msm uses below 100 MHz:
 * HOST_CONTROL2 UHS=0 (fixed feedback clock, no DLL), one data bit, and the
 * DLL reset/powered down with CK_OUT_EN off. The card clock must already be
 * gated; callers close CLOCK_CONTROL first.
 *
 * This is the b6a8788 fix. That boot proved the GCC RCG reached 400 kHz
 * (CMD0 took 146.5 us) but CMD1 still timed out: LK had left UHS=3 (SDR104),
 * so the controller expected the DLL receive path instead of the feedback
 * clock and never sampled the card's response. VDD_180 is preserved because
 * this board's I/O supply is fixed at 1.8 V; clearing it would request a 3.3 V
 * switch that does not exist.
 */
static void
sdhci_set_legacy_host(const char *why)
{
	uint16_t hc2 = hc_rd16(SDHCI_HOST_CONTROL2);
	uint8_t ctrl = hc_rd8(SDHCI_HOST_CONTROL);
	uint32_t dll = hc_rd(CORE_DLL_CONFIG);
	uint16_t new_hc2 = (uint16_t)(hc2 & ~(SDHCI_CTRL_UHS_MASK |
	    SDHCI_CTRL_EXEC_TUNING | SDHCI_CTRL_TUNED_CLK));
	uint8_t new_ctrl = (uint8_t)(ctrl & ~(SDHCI_CTRL_DMA_MASK |
	    SDHCI_CTRL_HISPD | SDHCI_CTRL_BUS_MASK));
	uint32_t new_dll = (dll | CORE_DLL_RST | CORE_DLL_PDN) &
	    ~CORE_CK_OUT_EN;

	printf("WHYRED SDHCI: %s: HOST_CONTROL 0x%02x -> 0x%02x, "
	    "HOST_CONTROL2 0x%04x -> 0x%04x (UHS 0, 1-bit), DLL_CONFIG "
	    "0x%08x -> 0x%08x (RST|PDN, CK_OUT_EN off)\n", why, ctrl,
	    new_ctrl, hc2, new_hc2, dll, new_dll);
	hc_wr16(SDHCI_HOST_CONTROL2, new_hc2);
	hc_wr8(SDHCI_HOST_CONTROL, new_ctrl);
	hc_wr(CORE_DLL_CONFIG, new_dll);
	/* Flush the posted writes before the card clock is re-enabled. */
	(void)hc_rd16(SDHCI_HOST_CONTROL2);
	(void)hc_rd8(SDHCI_HOST_CONTROL);
	(void)hc_rd(CORE_DLL_CONFIG);
}

/*
 * Answer pending CORE_PWRCTL requests. No regulator lives in this driver:
 * LK brought VMMC/VQMMC up and the pad switch is already at 1.8 V, so every
 * request is answered SUCCESS (Linux sdhci_msm_handle_pwr_irq() does the same
 * for the voltages the core switches itself). A BUS_ON request re-asserts
 * LK's pad bits, as Linux's pwr-irq handler does.
 *
 * wait == false: the requested state is already current, so no request will
 * be raised; do not wait 100 ms for one that cannot come.
 *
 * Only a request that was raised and cannot be cleared is an error.
 */
static int
sdhci_pwrctl_ack(const char *what, bool wait)
{
#if defined(ARM64_BOARD_CONFIG_LAVENDER)
	(void)what;
	(void)wait;
	return 0;
#else
	uint64_t deadline;
	uint32_t pending, ack;
	int retry;

	nanoseconds_to_absolutetime(PWRCTL_TIMEOUT_NS, &deadline);
	deadline += mach_absolute_time();

	if (wait) {
		do {
			pending = core_rd(CORE_PWRCTL_STATUS) &
			    CORE_PWRCTL_INT_MASK;
			if (pending) {
				break;
			}
			delay(1);
		} while (mach_absolute_time() <= deadline);
	} else {
		pending = core_rd(CORE_PWRCTL_STATUS) & CORE_PWRCTL_INT_MASK;
	}
	if (!pending) {
		printf("WHYRED SDHCI: PWRCTL %s: no request (STATUS 0x%08x), "
		    "already in that state\n", what,
		    core_rd(CORE_PWRCTL_STATUS));
		return 0;
	}

	for (retry = 0; retry < 10; retry++) {
		pending = core_rd(CORE_PWRCTL_STATUS) & CORE_PWRCTL_INT_MASK;
		if (!pending) {
			printf("WHYRED SDHCI: PWRCTL %s acked (CTL 0x%08x)\n",
			    what, core_rd(CORE_PWRCTL_CTL));
			return 0;
		}
		ack = 0;
		if (pending & (CORE_PWRCTL_BUS_ON | CORE_PWRCTL_BUS_OFF)) {
			ack |= CORE_PWRCTL_BUS_SUCCESS;
		}
		if (pending & (CORE_PWRCTL_IO_LOW | CORE_PWRCTL_IO_HIGH)) {
			ack |= CORE_PWRCTL_IO_SUCCESS;
		}
		if (pending & CORE_PWRCTL_BUS_ON) {
			hc_wr(CORE_VENDOR_SPEC, hc_rd(CORE_VENDOR_SPEC) |
			    CORE_IO_PAD_PWR_SWITCH_EN | CORE_IO_PAD_PWR_SWITCH);
		}
		core_wr(CORE_PWRCTL_CLEAR, pending);
		core_wr(CORE_PWRCTL_CTL, ack);
		delay(1);
	}
	printf("WHYRED SDHCI: PWRCTL %s did not clear (STATUS 0x%08x): "
	    "power handshake failed\n", what, core_rd(CORE_PWRCTL_STATUS));
	return ETIMEDOUT;
#endif
}

static int
sdhci_set_bus_power(void)
{
	const uint8_t want = SDHCI_POWER_CONTROL_VAL;
	uint8_t cur = hc_rd8(SDHCI_POWER_CONTROL);

	if (cur == want) {
		return sdhci_pwrctl_ack("BUS_ON", false);
	}

	if (cur & SDHCI_POWER_ON) {
		hc_wr8(SDHCI_POWER_CONTROL, 0);
		(void)sdhci_pwrctl_ack("BUS_OFF", true);
	}

	hc_wr8(SDHCI_POWER_CONTROL, want);
	if (hc_rd8(SDHCI_POWER_CONTROL) != want) {
		printf("WHYRED SDHCI: POWER_CONTROL did not latch 0x%02x "
		    "(reads 0x%02x)\n", want, hc_rd8(SDHCI_POWER_CONTROL));
		return ENXIO;
	}
	return sdhci_pwrctl_ack("BUS_ON", true);
}

/*
 * The MSM SDCC v4 core ignores the SDHCI internal divider. The field-500
 * boot log timed out CMD1 exactly like divider 0, and probe step 2 now
 * timestamps CMD0 to measure the real rate. The card clock rate is the GCC
 * sdcc1_apps_clk rate (sdhci-msm programs it through
 * msm_set_clock_rate_for_bus_mode()/dev_pm_opp_set_rate()), so the SDHCI
 * divider field stays 0 and sdhci_gcc_set_apps() programs the RCG.
 *
 * LK left CLOCK_CONTROL 0x0003: INT_EN | INT_STABLE, divider 0. That is also
 * the only divider Linux ever writes on this IP, so 0 is preserved.
 */
static int
sdhci_enable_card_clock(void)
{
	uint16_t clk;
	uint64_t deadline;

	hc_wr16(SDHCI_CLOCK_CONTROL, 0);
	hc_wr16(SDHCI_CLOCK_CONTROL, SDHCI_CLOCK_INT_EN);

	nanoseconds_to_absolutetime(CLOCK_STABLE_TIMEOUT_NS, &deadline);
	deadline += mach_absolute_time();
	for (;;) {
		clk = hc_rd16(SDHCI_CLOCK_CONTROL);
		if (clk & SDHCI_CLOCK_INT_STABLE) {
			break;
		}
		if (mach_absolute_time() > deadline) {
			printf("WHYRED SDHCI: SDHCI clock not stable after "
			    "150 ms (CLOCK_CONTROL 0x%04x): GCC SDC1 branch "
			    "dead\n", clk);
			return ETIMEDOUT;
		}
		delay(1);
	}

	/* Read-modify-write: INT_STABLE is a status bit, never written back. */
	hc_wr16(SDHCI_CLOCK_CONTROL, (uint16_t)(clk | SDHCI_CLOCK_CARD_EN));
	clk = hc_rd16(SDHCI_CLOCK_CONTROL);
	printf("WHYRED SDHCI: card clock gate was closed, enabled "
	    "(CLOCK_CONTROL 0x%04x, divider 0: GCC owns the rate)\n", clk);
	if (!(clk & SDHCI_CLOCK_CARD_EN)) {
		printf("WHYRED SDHCI: card clock gate did not latch "
		    "(CLOCK_CONTROL 0x%04x)\n", clk);
		return ENXIO;
	}
	return 0;
}

/*
 * Program sdcc1_apps_clk_src. This mirrors clk_rcg2_set_rate() on the table
 * entry in santoni_gcc_rates: stage M/N/D, write CFG, set CMD_UPDATE and wait
 * for the hardware to clear it.
 *
 * Ordering and safety:
 *  - the HC card clock gate is closed first, so the transition cannot reach
 *    the card mid-frame;
 *  - the GCC branch (CBCR bit 0, halt bit 31) and the AHB branch at 0x4201c
 *    are never written; LK left them running and only the RCG rate changes;
 *  - CFG is read-modify-written so bit 20 (HW clk control) and anything else
 *    LK set is preserved;
 *  - the RCG is not touched when its root is parked or the apps branch is
 *    halted;
 *  - the card clock is re-enabled at divider 0 afterwards.
 */
static int
sdhci_gcc_set_apps(const struct santoni_gcc_rate *r)
{
	uint32_t cmd, cfg, cbcr;
	uint64_t deadline;

	if (se.gcc == 0) {
		return ENXIO;
	}

	hc_wr16(SDHCI_CLOCK_CONTROL, 0);
	delay(1);

	cmd = gcc_rd(GCC_SDCC1_APPS_CMD_RCGR);
	cfg = gcc_rd(GCC_SDCC1_APPS_CFG_RCGR);
	cbcr = gcc_rd(GCC_SDCC1_APPS_CBCR);
	printf("WHYRED SDHCI: GCC sdcc1_apps before: CMD 0x%08x CFG 0x%08x "
	    "M 0x%02x N 0x%02x D 0x%02x CBCR 0x%08x\n", cmd, cfg,
	    gcc_rd(GCC_SDCC1_APPS_M) & 0xffu, gcc_rd(GCC_SDCC1_APPS_N) & 0xffu,
	    gcc_rd(GCC_SDCC1_APPS_D) & 0xffu, cbcr);

	if ((cmd & GCC_CMD_ROOT_OFF) || (cbcr & GCC_CBCR_CLK_OFF)) {
		printf("WHYRED SDHCI: GCC sdcc1_apps not running "
		    "(CMD 0x%08x CBCR 0x%08x), not touching it\n", cmd, cbcr);
		return ENXIO;
	}

	gcc_wr(GCC_SDCC1_APPS_M, r->m);
	gcc_wr(GCC_SDCC1_APPS_N, r->n);
	gcc_wr(GCC_SDCC1_APPS_D, r->d);

	cfg &= ~(GCC_CFG_DIV_MASK | GCC_CFG_SEL_MASK | GCC_CFG_MODE_MASK);
	cfg |= r->cfg;
	gcc_wr(GCC_SDCC1_APPS_CFG_RCGR, cfg);
	(void)gcc_rd(GCC_SDCC1_APPS_CFG_RCGR);  /* posted before UPDATE */

	gcc_wr(GCC_SDCC1_APPS_CMD_RCGR, cmd | GCC_CMD_UPDATE);
	nanoseconds_to_absolutetime(500ull * NSEC_PER_USEC, &deadline);
	deadline += mach_absolute_time();
	while (gcc_rd(GCC_SDCC1_APPS_CMD_RCGR) & GCC_CMD_UPDATE) {
		if (mach_absolute_time() > deadline) {
			printf("WHYRED SDHCI: GCC sdcc1_apps update stuck "
			    "(CMD 0x%08x)\n",
			    gcc_rd(GCC_SDCC1_APPS_CMD_RCGR));
			return ETIMEDOUT;
		}
		delay(1);
	}

	printf("WHYRED SDHCI: GCC sdcc1_apps now: CMD 0x%08x CFG 0x%08x "
	    "M 0x%02x N 0x%02x D 0x%02x (%s)\n",
	    gcc_rd(GCC_SDCC1_APPS_CMD_RCGR),
	    gcc_rd(GCC_SDCC1_APPS_CFG_RCGR),
	    gcc_rd(GCC_SDCC1_APPS_M) & 0xffu,
	    gcc_rd(GCC_SDCC1_APPS_N) & 0xffu,
	    gcc_rd(GCC_SDCC1_APPS_D) & 0xffu, r->name);

	return sdhci_enable_card_clock();
}

static int sdhci_send_status(uint32_t rca, uint32_t *r1);
static int sdhci_sleep_awake(uint32_t rca, uint32_t *r1);
static int sdhci_enumerate_card(void);

/*
 * CMD1 SEND_OP_COND. OCR bit31 is the card's power-up status: 1 means ready
 * (Linux calls the same bit MMC_CARD_BUSY, which reads backwards). Poll only
 * while the card answers with bit31 clear. A command timeout is not "busy":
 * no response was sampled, so repeating the same frame cannot help and would
 * hide the first failure behind 100 identical lines.
 */
static int
sdhci_cmd1_poll(uint32_t *ocr)
{
	uint32_t arg = 0x40FF8080u;
	uint64_t deadline;
	int error;

	nanoseconds_to_absolutetime(1ull * NSEC_PER_SEC, &deadline);
	deadline += mach_absolute_time();

	for (int attempt = 0; attempt < 100; attempt++) {
		*ocr = 0;
		error = sdhci_cmd(1, arg, 0, SDHCI_CMD_RESP_SHORT, ocr);
		if (error) {
			printf("WHYRED SDHCI: probe step 2: CMD1 attempt %d "
			    "got no usable response (%d), OCR 0x%08x, "
			    "%llu ns; stop\n", attempt + 1, error, *ocr,
			    (unsigned long long)se.last_cmd_ns);
			sdhci_recover(false);
			return error;
		}
		printf("WHYRED SDHCI: probe step 2: CMD1 attempt %d OCR "
		    "0x%08x (%llu ns)\n", attempt + 1, *ocr,
		    (unsigned long long)se.last_cmd_ns);
		if (*ocr & 0x80000000u) {
			printf("WHYRED SDHCI: probe step 2: CMD1 ready: OCR "
			    "bit31 set (card power-up complete)\n");
			return 0;
		}
		/* Linux mmc_send_op_cond(): echo the card's OCR and set the
		 * sector-mode request bit for the next attempt. */
		arg = *ocr | 0x40000000u;
		if (mach_absolute_time() > deadline) {
			break;
		}
		delay(10000);   /* 10 ms between polls, 1 s total */
	}
	printf("WHYRED SDHCI: probe step 2: CMD1 still busy after 1 s\n");
	return ETIMEDOUT;
}

static int
sdhci_take_over(void)
{
	uint16_t handoff_hc2;
	uint8_t handoff_ctrl, handoff_timeout;
	uint32_t mci_clk = 0, mci_power = 0, vendor, probe_r1 = 0, probe_ocr = 0;
	uint64_t deadline;
	int error, err0, err_cmd1;
	bool reset_done, card_in_transfer = false;

	/*
	 * Name the build in the log. Matching a boot log to a patch by
	 * guessing from its messages has already cost flash cycles.
	 */
	printf("WHYRED SDHCI: driver 0179 (root \"%s\", writes %s; boot "
	    "args decide whether the HFS+ root comes from eMMC)\n",
	    se.part, se.writable ? "enabled" : "read-only");

	error = sdhci_core_check();
	if (error) {
		return error;
	}

	/* Probe step 0: the only true pre-write snapshot. */
	sdhci_snapshot_untouched();

#if !defined(ARM64_BOARD_CONFIG_LAVENDER)
	/* Disable power IRQ generation on GIC while polled */
	core_wr(CORE_PWRCTL_MASK, 0);

	/*
	 * Sample the legacy MCI window while it is still the active layout:
	 * LK drove SDCC1 entirely through it, and once HC mode is on these
	 * flops no longer describe the card clock.
	 */
	mci_power = core_rd(CORE_MCI_POWER);
	mci_clk = core_rd(CORE_MCI_CLK);
#else
	(void)mci_power;
	(void)mci_clk;
#endif

	/*
	 * Step 1: vendor clock and pad supply before HC mode, in Linux's order
	 * (sdhci_msm_probe: board value with 1.8V pad switches, then HC_MODE_EN,
	 * then FF_CLK_SW_RST_DIS).
	 */
	vendor = hc_rd(CORE_VENDOR_SPEC);
	printf("WHYRED SDHCI: CORE_VENDOR_SPEC 0x%08x (MCLK sel %u, "
	    "PWRSAVE %u) -> board 0x%08x\n", vendor,
	    (vendor & CORE_HC_MCLK_SEL_MASK) >> 8, (vendor >> 1) & 1u,
	    CORE_VENDOR_SPEC_BOARD_VAL);
	hc_wr(CORE_VENDOR_SPEC, CORE_VENDOR_SPEC_BOARD_VAL);

	/* Step 2: switch the core to SDHCI host controller mode */
	error = sdhci_enter_hc_mode();
	if (error) {
		return error;
	}

	/* Step 0b: standard window is decoded now, but RESET_ALL is not run. */
	sdhci_snapshot_standard("(post-HC-mode, pre-reset)");

	/* Drain any stale PWRCTL request firmware or reset left behind */
	(void)sdhci_pwrctl_ack("stale", false);

	/* Save pre-reset host control, host control 2, timeout control */
	handoff_ctrl = hc_rd8(SDHCI_HOST_CONTROL);
	handoff_hc2 = hc_rd16(SDHCI_HOST_CONTROL2);
	handoff_timeout = hc_rd8(SDHCI_TIMEOUT_CONTROL);
	if (handoff_timeout == 0) {
		handoff_timeout = SDHCI_TIMEOUT_MAX;
	}

	/*
	 * Step 3: full reset of the HC block (LK sdhci_msm_init and Linux sdhci_init
	 * do SOFT_RESET_ALL here). GCC clocks are outside the SDHCI reset domain.
	 */
	hc_wr8(SDHCI_SOFTWARE_RESET, SDHCI_RESET_ALL);
	nanoseconds_to_absolutetime(RESET_TIMEOUT_NS, &deadline);
	deadline += mach_absolute_time();
	reset_done = false;
	while (hc_rd8(SDHCI_SOFTWARE_RESET) & SDHCI_RESET_ALL) {
		if (mach_absolute_time() > deadline) {
			break;
		}
		delay(1);
	}
	reset_done = (hc_rd8(SDHCI_SOFTWARE_RESET) & SDHCI_RESET_ALL) == 0;
	if (!reset_done) {
		printf("WHYRED SDHCI: RESET_ALL did not self-clear "
		    "(SOFTWARE_RESET 0x%02x, PRESENT_STATE 0x%08x, "
		    "CLOCK_CONTROL 0x%04x, VENDOR_SPEC 0x%08x)\n",
		    hc_rd8(SDHCI_SOFTWARE_RESET), hc_rd(SDHCI_PRESENT_STATE),
		    hc_rd16(SDHCI_CLOCK_CONTROL), hc_rd(CORE_VENDOR_SPEC));
#if defined(ARM64_BOARD_CONFIG_LAVENDER)
		/*
		 * SDCC v5 leaves RESET_ALL asserted when the HC has no MCLK to
		 * clock the reset through. Panicking the probe here throws away
		 * a controller that ABL left working: the card is in TRAN, and
		 * step 2 starts with an addressed CMD13 that proves the
		 * inherited state. Continue and let that command decide.
		 */
		printf("WHYRED SDHCI: continuing without a completed reset\n");
#else
		return ETIMEDOUT;
#endif
	}
	/* RESET_ALL turns off bus power; acknowledge the BUS_OFF request */
	(void)sdhci_pwrctl_ack("BUS_OFF", reset_done);

	/* Re-assert vendor clock selection in case core reset affected it */
	sdhci_select_hc_clock();

	/* Restore host control registers saved before RESET_ALL */
	hc_wr8(SDHCI_HOST_CONTROL, (uint8_t)(handoff_ctrl & ~SDHCI_CTRL_DMA_MASK));
	hc_wr16(SDHCI_HOST_CONTROL2, handoff_hc2);
	hc_wr8(SDHCI_TIMEOUT_CONTROL, handoff_timeout);
	if (!(handoff_ctrl & SDHCI_CTRL_8BITBUS)) {
		printf("WHYRED SDHCI: host bus width not 8-bit "
		    "(HOST_CONTROL 0x%02x)\n", handoff_ctrl);
	}

	/* Step 4: turn on 1.8V bus power and acknowledge BUS_ON request */
	error = sdhci_set_bus_power();
	if (error) {
		printf("WHYRED SDHCI: bus power not established (%d)\n", error);
		return error;
	}

	/* Step 5: card clock, proved live */
	error = sdhci_enable_card_clock();
	if (error) {
		return error;
	}

	se.saved_ctrl = handoff_ctrl;
	se.saved_hc2 = handoff_hc2;
	se.saved_timeout = handoff_timeout;

	/* Route card clock out (DLL or bypass) */
	sdhci_clock_output();

	/*
	 * Polled operation: status enable all ones, signal enable zero.
	 */
	hc_wr(SDHCI_INT_ENABLE, 0xffffffffu);
	hc_wr(SDHCI_SIGNAL_ENABLE, 0);
	hc_wr(SDHCI_INT_STATUS, hc_rd(SDHCI_INT_STATUS));

	/*
	 * The one state bit that explains "no data, no timeout, no DAT
	 * activity": the command-queue FIFO. Print what ABL left, then select
	 * the standard FIFO the PIO and SDMA engines feed from.
	 */
	{
		uint32_t vs3 = hc_rd(CORE_VENDOR_SPEC3);

		sdhci_v5_standard_fifo();
		printf("WHYRED SDHCI: alt FIFO VENDOR_SPEC3 0x%08x -> "
		    "0x%08x (bit 10 %s)\n", vs3,
		    hc_rd(CORE_VENDOR_SPEC3),
		    (vs3 & CORE_FIFO_ALT_EN) ?
		    "was set: data went to the CMDQ FIFO, not SDHCI" :
		    "already clear");
	}

#if defined(ARM64_BOARD_CONFIG_LAVENDER)
	printf("WHYRED SDHCI: takeover complete (VENDOR_SPEC 0x%08x, "
	    "POWER_CONTROL 0x%02x, CLOCK_CONTROL 0x%04x, HOST_CONTROL 0x%02x, "
	    "HOST_CONTROL2 0x%04x, TIMEOUT_CONTROL 0x%02x)\n",
	    hc_rd(CORE_VENDOR_SPEC), hc_rd8(SDHCI_POWER_CONTROL),
	    hc_rd16(SDHCI_CLOCK_CONTROL), hc_rd8(SDHCI_HOST_CONTROL),
	    hc_rd16(SDHCI_HOST_CONTROL2), hc_rd8(SDHCI_TIMEOUT_CONTROL));
#else
	printf("WHYRED SDHCI: takeover complete (HC_MODE 0x%08x, "
	    "VENDOR_SPEC 0x%08x, legacy MCI_CLK 0x%08x, MCI_POWER 0x%08x, "
	    "POWER_CONTROL 0x%02x, CLOCK_CONTROL 0x%04x, HOST_CONTROL 0x%02x, "
	    "HOST_CONTROL2 0x%04x, TIMEOUT_CONTROL 0x%02x)\n",
	    core_rd(CORE_HC_MODE), hc_rd(CORE_VENDOR_SPEC), mci_clk,
	    mci_power, hc_rd8(SDHCI_POWER_CONTROL),
	    hc_rd16(SDHCI_CLOCK_CONTROL), hc_rd8(SDHCI_HOST_CONTROL),
	    hc_rd16(SDHCI_HOST_CONTROL2), hc_rd8(SDHCI_TIMEOUT_CONTROL));
#endif

	/*
	 * Probe step 2: close the card clock, force the host back to the
	 * low-speed receive path, then program sdcc1_apps_clk_src to 400 kHz.
	 * The order matters: UHS=3 with the DLL powered down is why b6a8788
	 * completed CMD0 (no response) but never sampled CMD1's response.
	 */
	printf("WHYRED SDHCI: probe step 2: gate card clock, UHS=0, "
	    "GCC 400 kHz\n");
	hc_wr16(SDHCI_CLOCK_CONTROL, 0);
	hc_wr(CORE_VENDOR_SPEC, CORE_VENDOR_SPEC_BOARD_VAL);
	sdhci_set_legacy_host("probe step 2 (400 kHz)");
	/*
	 * Keep the bus width ABL used for its last data transfer. Every command
	 * up to CMD7 runs on the CMD line, so the card and host disagreeing on
	 * DAT[7:0] is harmless until the first data phase. That phase is the
	 * EXT_CSD read, and sdhci_read_ext_csd() negotiates the width again
	 * there (CMD6), since the CMD0 below may have reset the card's data
	 * interface.
	 */
	hc_wr8(SDHCI_HOST_CONTROL,
	    (uint8_t)(handoff_ctrl & SDHCI_CTRL_BUS_MASK));
	printf("WHYRED SDHCI: probe step 2: bus width from ABL kept until "
	    "CMD0 (HOST_CONTROL 0x%02x, %u-bit); CMD0 returns the card to "
	    "1-bit and the host follows\n", hc_rd8(SDHCI_HOST_CONTROL),
	    (handoff_ctrl & SDHCI_CTRL_8BITBUS) ? 8 :
	    ((handoff_ctrl & SDHCI_CTRL_4BITBUS) ? 4 : 1));

	error = sdhci_gcc_set_apps(&santoni_gcc_rates[SANTONI_GCC_RATE_400K]);
	if (error) {
		printf("WHYRED SDHCI: probe step 2: GCC 400 kHz failed (%d)\n",
		    error);
		return error;
	}
	printf("WHYRED SDHCI: probe step 2: CLOCK_CONTROL 0x%04x "
	    "HOST_CONTROL 0x%02x HOST_CONTROL2 0x%04x DLL_CONFIG 0x%08x "
	    "(SDHCI divider ignored; UHS 0; DLL off)\n",
	    hc_rd16(SDHCI_CLOCK_CONTROL), hc_rd8(SDHCI_HOST_CONTROL),
	    hc_rd16(SDHCI_HOST_CONTROL2), hc_rd(CORE_DLL_CONFIG));

	/*
	 * At 400 kHz, 1 ms is 400 clocks: far more than the 74 clocks the
	 * eMMC needs after power-up/reset before CMD1.
	 */
	delay(1000);

	/*
	 * The card clock is live now, which is what RESET_ALL needed and did
	 * not have during takeover (CLOCK_CONTROL 0x0003 there, SD clock off).
	 * Reset the command and data paths here, with the clock running, so
	 * the first real data transfer does not inherit ABL's last transfer.
	 */
	printf("WHYRED SDHCI: probe step 2: reset paths with the clock "
	    "live (PRESENT_STATE 0x%08x)\n", hc_rd(SDHCI_PRESENT_STATE));
	if (sdhci_reset_paths() == 0) {
		printf("WHYRED SDHCI: probe step 2: PRESENT_STATE 0x%08x "
		    "after path reset\n", hc_rd(SDHCI_PRESENT_STATE));
	}

	/*
	 * First prove the inherited hand-off without resetting the card. If
	 * CMD13 answers with state 4, the feedback-clock fix is validated and
	 * the card is already in Transfer. CMD5 is only legal in Standby or
	 * Sleep: in TRAN the card ignores it, and the 0155 boot log shows the
	 * two resulting command timeouts (and their path resets) right
	 * before CMD0. Only ask when CMD13 did not prove TRAN.
	 */
	error = sdhci_send_status(SANTONI_CARD_RCA, &probe_r1);
	if (!error) {
		card_in_transfer =
		    (probe_r1 & SDHCI_R1_STATE_MASK) ==
		    (SDHCI_R1_STATE_TRAN << 9);
		printf("WHYRED SDHCI: probe step 2: CMD13 RCA 2 answered "
		    "before reset: R1 0x%08x (state %u%s)\n", probe_r1,
		    (probe_r1 >> 9) & 0xf,
		    card_in_transfer ? ", TRAN; CMD5 not needed" : "");
	} else {
		printf("WHYRED SDHCI: probe step 2: CMD13 RCA 2 failed "
		    "before reset (%d)\n", error);
	}

	if (!card_in_transfer) {
		error = sdhci_sleep_awake(SANTONI_CARD_RCA, &probe_r1);
		if (!error) {
			printf("WHYRED SDHCI: probe step 2: CMD5 awake RCA 2 "
			    "answered: R1 0x%08x\n", probe_r1);
		} else {
			(void)sdhci_reset_paths();
			error = sdhci_sleep_awake(1, &probe_r1);
			if (!error) {
				printf("WHYRED SDHCI: probe step 2: CMD5 awake "
				    "RCA 1 answered: R1 0x%08x\n", probe_r1);
			} else {
				printf("WHYRED SDHCI: probe step 2: CMD5 "
				    "awake got no answer on RCA 2 or 1\n");
				(void)sdhci_reset_paths();
			}
		}
	}

	/*
	 * Reset the card to Idle and identify. CMD0 needs no response, so its
	 * 146.5 us duration is the clock proof; CMD1 is the first command that
	 * needs the receive path to work.
	 */
	err0 = sdhci_cmd(0, 0, 0, 0, NULL);
	printf("WHYRED SDHCI: probe step 2: CMD0 -> error %d, %llu ns\n",
	    err0, (unsigned long long)se.last_cmd_ns);
	if (err0) {
		sdhci_recover(false);
	} else {
		/*
		 * GO_IDLE_STATE resets the card's data interface to 1-bit,
		 * whatever width ABL used for its last transfer. Keeping the
		 * host at 8-bit does not hurt the command line, which is why
		 * enumeration still worked, but it breaks the first data
		 * command: the card drives DAT0 only while the controller waits
		 * for DAT[7:0] to finish the block. DAT_LINE_ACTIVE then stays
		 * set forever and no error bit is raised - the 0160 wedge.
		 * Follow the card now; 8-bit is renegotiated with CMD6 only
		 * after a read has completed.
		 */
		uint8_t now = hc_rd8(SDHCI_HOST_CONTROL);
		hc_wr8(SDHCI_HOST_CONTROL,
		    (uint8_t)(now & ~SDHCI_CTRL_BUS_MASK));
		printf("WHYRED SDHCI: probe step 2: CMD0 reset the card to "
		    "1-bit; host width 0x%02x -> 0x%02x\n", now,
		    hc_rd8(SDHCI_HOST_CONTROL));
	}

	err_cmd1 = sdhci_cmd1_poll(&probe_ocr);
	if (err_cmd1) {
		/*
		 * JEDEC GO_PRE_IDLE_STATE. Linux's mmc_go_idle() uses arg 0
		 * for a sleeping eMMC, so this is only a fallback, never the
		 * primary reset; if CMD1 still fails the next suspect is the
		 * host receive path, not another reset.
		 */
		printf("WHYRED SDHCI: probe step 2: CMD1 failed (%d), one "
		    "GO_PRE_IDLE retry (CMD0 0xf0f0f0f0)\n", err_cmd1);
		err0 = sdhci_cmd(0, 0xf0f0f0f0u, 0, 0, NULL);
		printf("WHYRED SDHCI: probe step 2: CMD0 pre-idle -> "
		    "error %d, %llu ns\n", err0,
		    (unsigned long long)se.last_cmd_ns);
		if (err0) {
			sdhci_recover(false);
		}
		err_cmd1 = sdhci_cmd1_poll(&probe_ocr);
	}
	printf("WHYRED SDHCI: probe step 2: CMD1 final -> error %d, "
	    "OCR 0x%08x\n", err_cmd1, probe_ocr);
	se.cmd1_error = err_cmd1;
	se.cmd1_ocr = probe_ocr;

	if (err_cmd1 == 0) {
		error = sdhci_enumerate_card();
		if (error) {
			printf("WHYRED SDHCI: enumeration failed (%d); "
			    "leaving the card alone\n", error);
			return error;
		}
	}
	return 0;
}

static int
sdhci_restore_controller(void)
{
	/*
	 * After enumeration the card may be in 8-bit high-speed mode. Dropping
	 * the host to the 400 kHz 1-bit path would make CMD13 work but corrupt
	 * every data read, because the card would still drive eight data bits.
	 * Fail closed instead; this legacy restore is only for the pre-CMD1
	 * hand-off ladder.
	 */
	if (se.enumerated) {
		printf("WHYRED SDHCI: rung 3: refusing the legacy 1-bit "
		    "restore after enumeration\n");
		return ENXIO;
	}
	hc_wr(CORE_VENDOR_SPEC, CORE_VENDOR_SPEC_BOARD_VAL);
	sdhci_select_hc_clock();
	hc_wr8(SDHCI_HOST_CONTROL, se.saved_ctrl);
	hc_wr16(SDHCI_HOST_CONTROL2, se.saved_hc2);
	hc_wr8(SDHCI_TIMEOUT_CONTROL,
	    se.saved_timeout ? se.saved_timeout : SDHCI_TIMEOUT_MAX);
	/*
	 * A RESET_ALL wiped HOST_CONTROL2 back to 0, and the saved hand-off
	 * value is UHS=3 (SDR104) with an 8-bit bus. Do not put the receive
	 * path back into that state: rebuild the 400 kHz legacy mode instead.
	 */
	sdhci_set_legacy_host("rung 3 controller restore");
	if (sdhci_set_bus_power() != 0) {
		printf("WHYRED SDHCI: rung 3: bus power not restored\n");
		return ENXIO;
	}
	hc_wr(SDHCI_INT_ENABLE, 0xffffffffu);
	hc_wr(SDHCI_SIGNAL_ENABLE, 0);
	hc_wr(SDHCI_INT_STATUS, hc_rd(SDHCI_INT_STATUS));
	if (sdhci_gcc_set_apps(&santoni_gcc_rates[SANTONI_GCC_RATE_400K]) != 0) {
		printf("WHYRED SDHCI: rung 3: 400 kHz clock not restored\n");
		return ENXIO;
	}
	sdhci_clock_output();
	return sdhci_reset_paths();
}

/*
 * CMD13 SEND_STATUS: the cheap proof that a card is selected and answering.
 * arg is rca << 16 (Linux include/linux/mmc/mmc.h: "ac [31:16] RCA R1"). A
 * timed-out command leaves CMD_INHIBIT set on this core, so the command path
 * is reset before the next probe rung: writing INT_STATUS alone is not enough.
 */
static int
sdhci_send_status(uint32_t rca, uint32_t *r1)
{
	int error;

	error = sdhci_cmd(13, rca << 16, 0,
	    SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX, r1);
	if (error) {
		sdhci_recover(false);
	}
	return error;
}

/* CMD7: select one card by RCA (Standby -> Transfer). */
static int
sdhci_select_card(uint32_t rca)
{
	return sdhci_cmd(7, rca << 16, 0,
	    SDHCI_CMD_RESP_SHORT_BUSY | SDHCI_CMD_CRC | SDHCI_CMD_INDEX, NULL);
}

/*
 * CMD5 SLEEP_AWAKE with SLEEP_AWAKE=0: wake a card LK may have put to sleep.
 * LK's mmc_put_card_to_sleep() deselects with CMD7 arg 0 and then sends CMD5
 * with bit 15 set; a sleeping card ignores CMD13 but answers CMD5, and the
 * R1b response repeats the RCA in bits 31:16.
 */
static int
sdhci_sleep_awake(uint32_t rca, uint32_t *r1)
{
	return sdhci_cmd(5, rca << 16, 0,
	    SDHCI_CMD_RESP_SHORT_BUSY | SDHCI_CMD_CRC | SDHCI_CMD_INDEX, r1);
}

/*
 * CMD6 is R1b: the card holds DAT0 low while it programs EXT_CSD. The SDHCI
 * core does not wait for that busy phase itself, and sending CMD13 into it
 * would time out and leave CMD_INHIBIT set. Poll PRESENT_STATE until the
 * command/data inhibitors are clear and DAT0 is high, then confirm TRAN with
 * one CMD13. The 1 ms pre-delay lets the card actually enter the busy phase;
 * eMMC CMD6 busy can otherwise be missed by an immediate read.
 */
static int
sdhci_wait_card_ready(uint32_t rca, const char *what)
{
	uint32_t st, r1 = 0;
	uint64_t deadline;
	int error;

	nanoseconds_to_absolutetime(CMD6_BUSY_TIMEOUT_NS, &deadline);
	deadline += mach_absolute_time();

	/*
	 * CMD6 is sent without its R1b busy flag (see sdhci_switch_ext_csd),
	 * so Command Complete arrives while the card may still be
	 * programming. CMD13 is the defined way to ask: the card stays in
	 * TRAN and clears READY_FOR_DATA until the switch is finished. Poll
	 * it instead of waiting for DAT0, which is the handshake that wedged
	 * this controller.
	 */
	delay(1000);
	for (;;) {
		error = sdhci_cmd(13, rca << 16, 0,
		    SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX,
		    &r1);
		if (error) {
			printf("WHYRED SDHCI: enumeration: %s: CMD13 status "
			    "poll failed (%d)\n", what, error);
			return error;
		}
		if ((r1 & SDHCI_R1_STATE_MASK) ==
		    (SDHCI_R1_STATE_TRAN << 9) &&
		    (r1 & SDHCI_R1_READY_FOR_DATA)) {
			break;
		}
		if (mach_absolute_time() > deadline) {
			st = hc_rd(SDHCI_PRESENT_STATE);
			printf("WHYRED SDHCI: enumeration: %s: card did not "
			    "become ready within 1 s (R1 0x%08x, "
			    "PRESENT_STATE 0x%08x)\n", what, r1, st);
			return ETIMEDOUT;
		}
		delay(1000);
	}
	printf("WHYRED SDHCI: enumeration: %s: card ready "
	    "(R1 0x%08x, TRAN)\n", what, r1);
	return 0;
}

/*
 * CMD6 SWITCH write-byte. The host bus/timing change is applied immediately
 * after the response, before the busy phase ends: the card has already been
 * told the new mode, and the next CMD13/data phase must use it. ctrl_clear is
 * used for the mutually exclusive 4/8-bit field; ctrl_set is 8BITBUS or HISPD.
 */
static int
sdhci_switch_ext_csd(uint8_t index, uint8_t value, uint8_t ctrl_set,
    uint8_t ctrl_clear, const char *what)
{
	uint32_t arg = (EXT_CSD_WRITE_BYTE << 24) |
	    ((uint32_t)index << 16) | ((uint32_t)value << 8) |
	    EXT_CSD_CMD_SET_NORMAL;
	uint32_t r1 = 0;
	uint8_t ctrl;
	int error;

	printf("WHYRED SDHCI: enumeration: CMD6 %s (arg 0x%08x)\n",
	    what, arg);
	/*
	 * No R1b (busy) flag. On this core the R1b handshake is what latched
	 * CMD_INHIBIT+DAT_INHIBIT for good in the 0159 boot: DAT0 was high,
	 * INT_STATUS was clean, and SOFTWARE_RESET never cleared it. The card
	 * still has to finish programming, so sdhci_wait_card_ready() polls
	 * CMD13 for READY_FOR_DATA afterwards - the state the driver actually
	 * needs, without asking the controller to watch DAT0 for it.
	 */
	error = sdhci_cmd(6, arg, 0,
	    SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX,
	    &r1);
	if (error) {
		printf("WHYRED SDHCI: enumeration: CMD6 %s failed (%d)\n",
		    what, error);
		return error;
	}
	printf("WHYRED SDHCI: enumeration: CMD6 %s -> R1 0x%08x\n",
	    what, r1);
	if (r1 & SDHCI_R1_SWITCH_ERROR) {
		/*
		 * The card rejected the EXT_CSD byte. Do not touch
		 * HOST_CONTROL: the card is still in its old bus/timing mode.
		 */
		printf("WHYRED SDHCI: enumeration: CMD6 %s reported "
		    "SWITCH_ERROR, host left unchanged\n", what);
		return EIO;
	}

	ctrl = hc_rd8(SDHCI_HOST_CONTROL);
	ctrl = (uint8_t)((ctrl & ~ctrl_clear) | ctrl_set);
	hc_wr8(SDHCI_HOST_CONTROL, ctrl);
	ctrl = hc_rd8(SDHCI_HOST_CONTROL);
	printf("WHYRED SDHCI: enumeration: HOST_CONTROL 0x%02x after "
	    "%s\n", ctrl, what);
	/*
	 * ctrl_set and ctrl_clear overlap on purpose (8-bit sets 0x20 and
	 * clears 0x22), so check the two parts separately. The old test only
	 * looked at ctrl_set, which made the 1-bit case (set == 0) vacuous: a
	 * HOST_CONTROL that stayed at 8-bit looked like a success.
	 */
	if ((ctrl & ctrl_set) != ctrl_set ||
	    ((ctrl & ~ctrl_set) & ctrl_clear) != 0) {
		printf("WHYRED SDHCI: enumeration: HOST_CONTROL did not "
		    "latch %s (0x%02x, set 0x%02x, clear 0x%02x)\n", what,
		    ctrl, ctrl_set, ctrl_clear);
		return ENXIO;
	}

	return sdhci_wait_card_ready(se.rca, what);
}

/*
 * A PRESENT_STATE fallback can hand back an empty FIFO, and a constant burst
 * (all 0x00 or all 0xff) still has a non-zero word at the SEC_COUNT offset.
 * A real EXT_CSD names its own revision (byte 192), CSD structure (193) and a
 * plausible capacity, so check all three before a bus width is switched on
 * the strength of this read.
 */
static bool
sdhci_ext_csd_sane(void)
{
	const uint8_t *ext = (const void *)(se.dma + DMA_BOUNCE);
	uint32_t sec_count;

	if (ext[192] == 0 || ext[192] > 8 || ext[193] > 3) {
		return false;
	}
	memcpy(&sec_count, (const void *)(ext + 212), 4);
	return sec_count != 0 && sec_count <= 0x0fffffffu;
}

/*
 * Report a block that came back but is not the EXT_CSD, including its first
 * words: on this board "the transfer finished" has already meant "128 zero
 * words" once, and a REV/STRUCTURE pair alone would not show that.
 */
static bool
sdhci_ext_csd_check(uint32_t lanes, const char *how)
{
	const uint8_t *ext = (const void *)(se.dma + DMA_BOUNCE);
	const uint32_t *w = (const void *)(se.dma + DMA_BOUNCE);
	uint32_t sec_count;

	if (sdhci_ext_csd_sane()) {
		return true;
	}
	memcpy(&sec_count, (const void *)(ext + 212), 4);
	printf("WHYRED SDHCI: enumeration: EXT_CSD at %u-bit %s is not real "
	    "(REV %u STRUCTURE %u SEC_COUNT %u, words %08x %08x %08x %08x); "
	    "trying the next setup\n", lanes, how, ext[192], ext[193],
	    sec_count, w[0], w[1], w[2], w[3]);
	return false;
}

/*
 * Every attempt starts from a cleared buffer: a block left by an earlier
 * setup must never be mistaken for the answer to this one.
 */
static void
sdhci_ext_csd_clear(void)
{
	memset((void *)(se.dma + DMA_BOUNCE), 0, SDHCI_BLOCK);
	dma_sync(SDHCI_BLOCK, TRUE);
}

/*
 * Read one sector and report exactly what came back.
 *
 * LBA 0 of the user area is MIUI's protective MBR, so bytes 510/511 must be
 * 0x55/0xaa. That is a real block, not "a byte moved": the 0156/0157 logs
 * showed the FIFO handshake declaring success with 128 zero words.
 */
static bool
sdhci_lba0_attempt(const char *width, const char *engine, bool use_sdma)
{
	const uint8_t *blk = (const void *)(se.dma + DMA_BOUNCE);
	const uint32_t *w = (const void *)(se.dma + DMA_BOUNCE);
	bool mbr;
	int error;

	sdhci_ext_csd_clear();
	santoni_data_r1 = 0;
	if (use_sdma) {
		error = sdhci_read_sdma(17, 0, 1, se.dma_pa + DMA_BOUNCE);
	} else {
		error = sdhci_read_pio_once(17, 0, 1,
		    (void *)(se.dma + DMA_BOUNCE));
	}
	santoni_data_error = error;
	dma_sync(SDHCI_BLOCK, TRUE);
	mbr = blk[510] == 0x55 && blk[511] == 0xaa;
	if (mbr) {
		santoni_lba0_pio = !use_sdma;
	}
	printf("WHYRED SDHCI: data self-test LBA0 %s %s: %s (%d), bytes %02x "
	    "%02x, MBR %s, R1 0x%08x, words %08x %08x %08x %08x "
	    "(PRESENT_STATE 0x%08x)\n",
	    width, engine, error ? (error == ETIMEDOUT ? "timeout" : "error") :
	    "OK", error, blk[510], blk[511], mbr ? "55aa" : "absent",
	    santoni_data_r1, w[0], w[1], w[2], w[3],
	    hc_rd(SDHCI_PRESENT_STATE));
	return mbr;
}

/*
 * Read LBA 0 as a data-path self-test, in the state the card is actually in
 * after enumeration.
 *
 * 0160 kept ABL's 8-bit host width; 0161 followed CMD0 back to 1-bit. Both
 * wedged the same way (CMD_INHIBIT latched, INT_STATUS 0, DAT lines high), so
 * the width alone does not explain it. What both boots shared is
 * TIMEOUT_CONTROL 0x0f: the core would wait ~11 minutes for a missing block,
 * long after the driver gave up, so the data phase never ended and no error
 * was ever reported.
 *
 * This probe therefore shortens that timeout and sweeps only the host width
 * (1, 4, 8), with no CMD6 at all: the card never learns the host's width, so
 * one of the three must return a full block. Whichever does is the width the
 * card is really using, and EXT_CSD is then read at exactly that width.
 */
static bool
sdhci_probe_lba0(const char *phase, uint32_t *lanes)
{
	static const struct {
		uint8_t ctrl;
		const char *what;
	} widths[] = {
		{ 0, "1-bit" },
		{ SDHCI_CTRL_4BITBUS, "4-bit" },
		{ SDHCI_CTRL_8BITBUS, "8-bit" },
	};
	uint8_t saved_timeout = hc_rd8(SDHCI_TIMEOUT_CONTROL);
	unsigned int i;

	/*
	 * The 0161 boot proved the width was not the whole story: the host was
	 * already at 1-bit (HOST_CONTROL 0x00) and the first data command still
	 * never finished. The reason nothing ever recovers is TIMEOUT_CONTROL
	 * 0x0f: the core waits 2^28 card clocks, which at 400 kHz is about
	 * eleven minutes, so no data-timeout ever fires, no error bit appears
	 * and CMD_INHIBIT stays set for the rest of the boot. Drop it to 0x03
	 * (2^16 clocks, ~164 ms at 400 kHz): a block that never arrives now
	 * fails loudly and the core releases the path by itself.
	 */
	printf("WHYRED SDHCI: data self-test: phase %s, TIMEOUT_CONTROL "
	    "0x%02x -> 0x03 (~164 ms at 400 kHz)\n", phase, saved_timeout);

	/*
	 * Sweep the host side only, with no CMD6. The card never learns the
	 * host's width, so if it is still at ABL's 8-bit (CMD0 does not reset
	 * every part), the 8-bit pass is the one that returns a full 512-byte
	 * block; if it really went back to 1-bit, the first pass does. Either
	 * way a failed pass now costs 164 ms instead of wedging the core.
	 */
	for (i = 0; i < sizeof(widths) / sizeof(widths[0]); i++) {
		uint8_t ctrl = hc_rd8(SDHCI_HOST_CONTROL);
		uint8_t want = (uint8_t)((ctrl & ~SDHCI_CTRL_BUS_MASK) |
		    widths[i].ctrl);

		uint32_t r1 = 0;

		hc_wr8(SDHCI_TIMEOUT_CONTROL, 0x03);
		hc_wr8(SDHCI_HOST_CONTROL, want);
		printf("WHYRED SDHCI: data self-test: host width 0x%02x -> "
		    "0x%02x (%s, no CMD6)\n", ctrl,
		    hc_rd8(SDHCI_HOST_CONTROL), widths[i].what);
		(void)sdhci_cmd(16, SDHCI_BLOCK, 0,
		    SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX,
		    &r1);
		printf("WHYRED SDHCI: data self-test: CMD16 %s -> R1 "
		    "0x%08x\n", widths[i].what, r1);
		if (sdhci_lba0_attempt(widths[i].what, "PIO", false) ||
		    sdhci_lba0_attempt(widths[i].what, "SDMA", true)) {
			printf("WHYRED SDHCI: data self-test: the data path "
			    "works at %s (phase %s)\n", widths[i].what, phase);
			snprintf(santoni_lba0_note, sizeof(santoni_lba0_note),
			    "ok-%s", widths[i].what);
			*lanes = (widths[i].ctrl == SDHCI_CTRL_8BITBUS) ? 8 :
			    ((widths[i].ctrl == SDHCI_CTRL_4BITBUS) ? 4 : 1);
			hc_wr8(SDHCI_TIMEOUT_CONTROL, saved_timeout);
			return true;
		}
	}
	hc_wr8(SDHCI_TIMEOUT_CONTROL, saved_timeout);
	snprintf(santoni_lba0_note, sizeof(santoni_lba0_note),
	    "no-width e=%d r1=0x%08x", santoni_data_error, santoni_data_r1);
	return false;
}

/*
 * Read the 512-byte EXT_CSD. The bus width is negotiated with CMD6 before
 * every attempt because a card/host width mismatch produces exactly the same
 * silent data timeout as a dead transfer engine. 1-bit goes first (the width
 * an eMMC guarantees after CMD0), then 8-bit (the width the ABL used).
 *
 * SDMA goes first: ABL loaded this kernel through the same controller, while
 * the 0156 boot proved the PIO FIFO handshake reports success with an empty
 * block. A read only counts when the payload names its own revision,
 * structure and capacity.
 */
static int
sdhci_read_ext_csd(uint32_t *width)
{
	static const struct {
		uint8_t value;
		uint8_t set;
		const char *what;
	} attempts[] = {
		{ EXT_CSD_BUS_WIDTH_1, 0, "BUS_WIDTH=1 (EXT_CSD read)" },
		{ EXT_CSD_BUS_WIDTH_8, SDHCI_CTRL_8BITBUS,
		    "BUS_WIDTH=8 (EXT_CSD read)" },
	};
	uint32_t lba0_lanes = 1;
	unsigned int i;
	int error, last = EIO;

	/*
	 * If the core is already inhibited here, the wedge predates the data
	 * phase: name it and try the one reset that also cycles the card
	 * clock, instead of letting eight EBUSY attempts look like eight
	 * different failures.
	 */
	if (hc_rd(SDHCI_PRESENT_STATE) &
	    (SDHCI_CMD_INHIBIT | SDHCI_DATA_INHIBIT)) {
		printf("WHYRED SDHCI: enumeration: core inhibited before the "
		    "data phase (PRESENT_STATE 0x%08x); resetting\n",
		    hc_rd(SDHCI_PRESENT_STATE));
		if (sdhci_reset_paths() != 0) {
			printf("WHYRED SDHCI: enumeration: cannot start the "
			    "data phase; the controller is wedged\n");
			return EBUSY;
		}
	}

	/*
	 * Prove the data path first, in the state ABL left it. If LBA 0 comes
	 * back with its MBR signature there is no reason to change anything:
	 * EXT_CSD is then read the same way, and the timing switch and width
	 * ladder are skipped entirely. The 0159 boot shows that the first
	 * CMD6 in this path can latch CMD/DAT inhibit for good, so the fewer
	 * switches the better.
	 */
	if (sdhci_probe_lba0("after CMD0", &lba0_lanes)) {
		bool use_sdma = !santoni_lba0_pio;

		/*
		 * The 0164 boot moved the first block this driver ever read:
		 * LBA0 came back 55 aa at 1-bit. The very next data command
		 * (CMD8, EXT_CSD) was issued through SDMA anyway and failed at
		 * once with ADMAERR (INT_STATUS 0x02080000, 15 us), aborting
		 * the data phase; the PIO retry that followed then timed out
		 * because the card was still mid-block, and the width ladder
		 * that came after was pure noise.
		 *
		 * So do not try both engines. EXT_CSD is the same single-block
		 * read as LBA0, one command later, at the same width: issue it
		 * once with the engine that just moved a block and let the
		 * verdict say whether a second data command is possible at all.
		 */
		printf("WHYRED SDHCI: enumeration: LBA0 moved a block with "
		    "%s at %u-bit; reading EXT_CSD the same way\n",
		    santoni_lba0_pio ? "PIO" : "SDMA", lba0_lanes);
		(void)sdhci_cmd(16, SDHCI_BLOCK, 0,
		    SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX,
		    NULL);
		sdhci_ext_csd_clear();
		if (use_sdma) {
			error = sdhci_read_sdma(8, 0, 1,
			    se.dma_pa + DMA_BOUNCE);
		} else {
			error = sdhci_read_pio_once(8, 0, 1,
			    (void *)(se.dma + DMA_BOUNCE));
		}
		dma_sync(SDHCI_BLOCK, TRUE);
		if (sdhci_ext_csd_sane()) {
			*width = lba0_lanes;
			printf("WHYRED SDHCI: enumeration: EXT_CSD read works "
			    "at %u-bit (%s%s)\n", lba0_lanes,
			    use_sdma ? "SDMA" : "PIO",
			    error ? ", status error ignored" : "");
			return 0;
		}
		printf("WHYRED SDHCI: EXT_CSD verdict: %s at %u-bit failed "
		    "(%d), R1 0x%08x, PRESENT_STATE 0x%08x, INT_STATUS 0x%08x, "
		    "BS 0x%04x BC 0x%04x TM 0x%04x\n",
		    use_sdma ? "SDMA" : "PIO", lba0_lanes, error,
		    santoni_data_r1, hc_rd(SDHCI_PRESENT_STATE),
		    hc_rd(SDHCI_INT_STATUS), hc_rd16(SDHCI_BLOCK_SIZE),
		    hc_rd16(SDHCI_BLOCK_COUNT), hc_rd16(SDHCI_TRANSFER_MODE));
		return EIO;
	}

	/*
	 * If the sweep left the core inhibited, nothing after it can work:
	 * every command would print the same EBUSY and bury the one line that
	 * matters. Print the verdict (with the card's R1 and the transfer
	 * registers) and stop.
	 */
	if (hc_rd(SDHCI_PRESENT_STATE) &
	    (SDHCI_CMD_INHIBIT | SDHCI_DATA_INHIBIT)) {
		printf("WHYRED SDHCI: data verdict: no width or engine moved "
		    "a byte; core inhibited (PRESENT_STATE 0x%08x, INT_STATUS "
		    "0x%08x, last error %d, R1 0x%08x)\n",
		    hc_rd(SDHCI_PRESENT_STATE), hc_rd(SDHCI_INT_STATUS),
		    santoni_data_error, santoni_data_r1);
		printf("WHYRED SDHCI: data verdict: BS 0x%04x BC 0x%04x TM "
		    "0x%04x SDMA 0x%08x HOST_CONTROL 0x%02x VENDOR_SPEC3 "
		    "0x%08x\n",
		    hc_rd16(SDHCI_BLOCK_SIZE), hc_rd16(SDHCI_BLOCK_COUNT),
		    hc_rd16(SDHCI_TRANSFER_MODE), hc_rd(SDHCI_SDMA_ADDR),
		    hc_rd8(SDHCI_HOST_CONTROL), hc_rd(CORE_VENDOR_SPEC3));
		return EBUSY;
	}

	/*
	 * The card may still be in the timing mode ABL handed it, and on this
	 * board that has been HS400: HOST_CONTROL2 reached the data probe
	 * with TUNED_CLK set in the 0156 boot. An HS400 card delivers read
	 * data strobed by DS, not as a plain block to a legacy 400 kHz host,
	 * which is what every zero-word EXT_CSD read so far looks like.
	 * HS_TIMING and BUS_WIDTH are independent EXT_CSD fields, so dropping
	 * the card back to legacy timing does not disturb the width ladder.
	 */
	if (sdhci_switch_ext_csd(EXT_CSD_HS_TIMING, 0, 0, SDHCI_CTRL_HISPD,
	    "HS_TIMING=0 (legacy, before EXT_CSD)") != 0) {
		printf("WHYRED SDHCI: enumeration: HS_TIMING=0 failed; the "
		    "card may still be in a high-speed timing mode\n");
	}

	for (i = 0; i < sizeof(attempts) / sizeof(attempts[0]); i++) {
		uint32_t lanes = (attempts[i].set == SDHCI_CTRL_8BITBUS) ?
		    8 : 1;

		error = sdhci_switch_ext_csd(EXT_CSD_BUS_WIDTH,
		    attempts[i].value, attempts[i].set, SDHCI_CTRL_BUS_MASK,
		    attempts[i].what);
		if (error) {
			printf("WHYRED SDHCI: enumeration: %s failed (%d)\n",
			    attempts[i].what, error);
			last = error;
			continue;
		}

		/* The block length is fixed at 512, but CMD16 is free. */
		(void)sdhci_cmd(16, SDHCI_BLOCK, 0,
		    SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX,
		    NULL);
		sdhci_ext_csd_clear();
		error = sdhci_read_sdma(8, 0, 1, se.dma_pa + DMA_BOUNCE);
		dma_sync(SDHCI_BLOCK, TRUE);
		/*
		 * The status path has already declared a timeout after the
		 * engine delivered the block on this core (CMD13 and the PIO
		 * FIFO both did it). A payload that names the card is worth
		 * more than the status bit, so keep it even on an error.
		 */
		if (sdhci_ext_csd_sane()) {
			*width = lanes;
			printf("WHYRED SDHCI: enumeration: EXT_CSD read works "
			    "at %u-bit (SDMA%s)\n", lanes,
			    error ? ", status error ignored" : "");
			return 0;
		}
		if (!error) {
			(void)sdhci_ext_csd_check(lanes, "SDMA");
			error = EIO;
		} else {
			printf("WHYRED SDHCI: enumeration: EXT_CSD SDMA read "
			    "at %u-bit failed (%d); trying PIO\n", lanes,
			    error);
		}

		(void)sdhci_cmd(16, SDHCI_BLOCK, 0,
		    SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX,
		    NULL);
		sdhci_ext_csd_clear();
		error = sdhci_read_pio_once(8, 0, 1,
		    (void *)(se.dma + DMA_BOUNCE));
		dma_sync(SDHCI_BLOCK, TRUE);
		if (sdhci_ext_csd_sane()) {
			*width = lanes;
			printf("WHYRED SDHCI: enumeration: EXT_CSD read works "
			    "at %u-bit (PIO%s)\n", lanes,
			    error ? ", status error ignored" : "");
			return 0;
		}
		if (!error) {
			(void)sdhci_ext_csd_check(lanes, "PIO");
			error = EIO;
		}
		printf("WHYRED SDHCI: enumeration: EXT_CSD unreadable at "
		    "%u-bit (%d), trying the next width\n", lanes, error);
		last = error;
	}
	/*
	 * Neither width moved a byte through PIO or SDMA. Run the matrix: it
	 * covers the remaining setup differences, and if one of them works the
	 * caller never notices the detour.
	 */
	if (sdhci_data_matrix(width) == 0) {
		return 0;
	}
	return last;
}

/*
 * READY -> IDENT -> STBY -> TRAN, EXT_CSD, then 25 MHz / wide bus / HS.
 *
 * CMD1 leaves the card in READY with no RCA: CMD3 assigns RCA 2, CMD9/CMD10
 * are addressed R2 reads and CMD7 selects it. CMD8 then reads EXT_CSD through
 * sdhci_read_ext_csd(), which negotiates the width because this first data
 * phase is the one the port has never completed. Only after the 512-byte
 * EXT_CSD read succeeds do we move to 25 MHz and use CMD6 to change the
 * card's timing and settle the final width. These are configuration writes
 * to EXT_CSD, not user-data writes.
 */
static int
sdhci_enumerate_card(void)
{
	uint32_t cid[4] = { 0 }, csd[4] = { 0 };
	uint32_t sec_count = 0, r1 = 0;
	uint32_t bus_width = 8;
	char pnm[7];
	int error;

	printf("WHYRED SDHCI: enumeration: CMD2 ALL_SEND_CID (R2)\n");
	error = sdhci_read_response128(2, 0, cid);
	if (error) {
		printf("WHYRED SDHCI: enumeration: CMD2 failed (%d)\n", error);
		return error;
	}
	printf("WHYRED SDHCI: enumeration: CID 0x%08x 0x%08x 0x%08x "
	    "0x%08x\n", cid[0], cid[1], cid[2], cid[3]);

	printf("WHYRED SDHCI: enumeration: CMD3 SET_RELATIVE_ADDR RCA %u\n",
	    SANTONI_CARD_RCA);
	error = sdhci_cmd(3, SANTONI_CARD_RCA << 16, 0,
	    SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX, &r1);
	if (error) {
		printf("WHYRED SDHCI: enumeration: CMD3 failed (%d)\n", error);
		return error;
	}
	se.rca = SANTONI_CARD_RCA;
	printf("WHYRED SDHCI: enumeration: CMD3 -> R1 0x%08x (state %u)\n",
	    r1, (r1 & SDHCI_R1_STATE_MASK) >> 9);

	printf("WHYRED SDHCI: enumeration: CMD9 SEND_CSD RCA %u (R2)\n",
	    se.rca);
	error = sdhci_read_response128(9, se.rca << 16, csd);
	if (error) {
		printf("WHYRED SDHCI: enumeration: CMD9 failed (%d)\n", error);
		return error;
	}
	printf("WHYRED SDHCI: enumeration: CSD 0x%08x 0x%08x 0x%08x "
	    "0x%08x\n", csd[0], csd[1], csd[2], csd[3]);

	pnm[0] = (char)(cid[0] & 0xff);
	pnm[1] = (char)(cid[1] >> 24);
	pnm[2] = (char)(cid[1] >> 16);
	pnm[3] = (char)(cid[1] >> 8);
	pnm[4] = (char)cid[1];
	pnm[5] = (char)(cid[2] >> 24);
	pnm[6] = 0;
	for (int i = 0; i < 6; i++) {
		if (pnm[i] < 0x20 || pnm[i] > 0x7e) {
			pnm[i] = '.';
		}
	}
	printf("WHYRED SDHCI: eMMC MID 0x%02x CBX 0x%02x OID 0x%02x "
	    "PNM \"%s\" rev %u.%u S/N 0x%08x MDT 0x%02x CSD 0x%08x\n",
	    (cid[0] >> 24) & 0xff, (cid[0] >> 16) & 0xff,
	    (cid[0] >> 8) & 0xff, pnm, (cid[2] >> 20) & 0xf,
	    (cid[2] >> 16) & 0xf,
	    ((cid[2] & 0xffffu) << 16) | (cid[3] >> 16),
	    (cid[3] >> 8) & 0xff, csd[0]);

	printf("WHYRED SDHCI: enumeration: CMD7 SELECT_CARD RCA %u\n",
	    se.rca);
	error = sdhci_select_card(se.rca);
	if (error) {
		printf("WHYRED SDHCI: enumeration: CMD7 failed (%d)\n", error);
		return error;
	}
	printf("WHYRED SDHCI: enumeration: card selected (TRAN)\n");

	/*
	 * CMD7 is the only command that may move the card to Transfer, so ask
	 * CMD13 what state it really reached. CMD8 has no data phase from
	 * Standby: the card answers ILLEGAL_COMMAND and the host waits out a
	 * data timeout with INT_STATUS still 0x0, which is exactly the dead
	 * end seen on hardware.
	 */
	error = sdhci_send_status(se.rca, &r1);
	if (error) {
		printf("WHYRED SDHCI: enumeration: CMD13 after CMD7 failed "
		    "(%d)\n", error);
		return error;
	}
	printf("WHYRED SDHCI: enumeration: CMD13 after CMD7 -> R1 0x%08x "
	    "(state %u), HOST_CONTROL 0x%02x\n", r1,
	    (r1 & SDHCI_R1_STATE_MASK) >> 9, hc_rd8(SDHCI_HOST_CONTROL));
	if ((r1 & SDHCI_R1_STATE_MASK) != (SDHCI_R1_STATE_TRAN << 9)) {
		printf("WHYRED SDHCI: enumeration: card is not in TRAN after "
		    "CMD7; an EXT_CSD read would have no data phase\n");
		return EIO;
	}

	/*
	 * The card is in TRAN and answered CMD8 before (R1 0x900), so the
	 * missing data phase is a transfer problem, not a command problem.
	 * Try each plausible width; sdhci_read_pio() also retries every
	 * attempt through SDMA.
	 */
	printf("WHYRED SDHCI: enumeration: CMD8 SEND_EXT_CSD "
	    "(512-byte read at 400 kHz)\n");
	error = sdhci_read_ext_csd(&bus_width);
	if (error) {
		printf("WHYRED SDHCI: enumeration: CMD8 failed (%d): EXT_CSD "
		    "unreadable at 1-bit and 8-bit, both PIO and SDMA\n",
		    error);
		return error;
	}
	dma_sync(SDHCI_BLOCK, TRUE);
	memcpy(&sec_count, (const void *)(se.dma + DMA_BOUNCE + 212), 4);
	if (sec_count == 0) {
		printf("WHYRED SDHCI: enumeration: EXT_CSD SEC_COUNT is 0; "
		    "CMD8 data is not trustworthy, not switching the bus\n");
		return EIO;
	}
	se.sec_count = sec_count;
	printf("WHYRED SDHCI: enumeration: EXT_CSD SEC_COUNT %u "
	    "(%llu MiB)\n", sec_count,
	    (unsigned long long)((uint64_t)sec_count * SDHCI_BLOCK >> 20));

	printf("WHYRED SDHCI: enumeration: switch GCC to 25 MHz\n");
	error = sdhci_gcc_set_apps(&santoni_gcc_rates[SANTONI_GCC_RATE_25M]);
	if (error) {
		printf("WHYRED SDHCI: enumeration: 25 MHz failed (%d)\n",
		    error);
		return error;
	}
	printf("WHYRED SDHCI: enumeration: CLOCK_CONTROL 0x%04x "
	    "HOST_CONTROL 0x%02x HOST_CONTROL2 0x%04x\n",
	    hc_rd16(SDHCI_CLOCK_CONTROL), hc_rd8(SDHCI_HOST_CONTROL),
	    hc_rd16(SDHCI_HOST_CONTROL2));
	sdhci_clock_output();

	error = sdhci_switch_ext_csd(EXT_CSD_BUS_WIDTH,
	    EXT_CSD_BUS_WIDTH_8, SDHCI_CTRL_8BITBUS, SDHCI_CTRL_BUS_MASK,
	    "BUS_WIDTH=8");
	if (!error) {
		/*
		 * The 0165 boot switched the card and the host to 8-bit and
		 * then reported "1-bit", because bus_width still held the
		 * width that had just read EXT_CSD. The card was fine
		 * (HOST_CONTROL 0x24, DAT[7:0] 8-bit HS verified, GPT read),
		 * but the driver's own state was wrong, and any later re-init
		 * would have used it. Record the width that is now active.
		 */
		bus_width = 8;
	}
	if (error) {
		printf("WHYRED SDHCI: enumeration: 8-bit switch failed "
		    "(%d); trying 4-bit\n", error);
		error = sdhci_switch_ext_csd(EXT_CSD_BUS_WIDTH,
		    EXT_CSD_BUS_WIDTH_4, SDHCI_CTRL_4BITBUS,
		    SDHCI_CTRL_BUS_MASK, "BUS_WIDTH=4");
		if (!error) {
			bus_width = 4;
		}
	}
	if (error) {
		printf("WHYRED SDHCI: enumeration: 4-bit switch failed "
		    "(%d); staying 1-bit\n", error);
		error = sdhci_switch_ext_csd(EXT_CSD_BUS_WIDTH,
		    EXT_CSD_BUS_WIDTH_1, 0, SDHCI_CTRL_BUS_MASK,
		    "BUS_WIDTH=1");
		if (!error) {
			bus_width = 1;
		}
	}
	if (error) {
		return error;
	}

	error = sdhci_switch_ext_csd(EXT_CSD_HS_TIMING,
	    EXT_CSD_TIMING_HS, SDHCI_CTRL_HISPD, 0, "HS_TIMING=1");
	if (error) {
		return error;
	}

	se.enumerated = true;
	printf("WHYRED SDHCI: enumeration complete: 25 MHz, %u-bit, "
	    "high-speed (HOST_CONTROL 0x%02x)\n", bus_width,
	    hc_rd8(SDHCI_HOST_CONTROL));
	return 0;
}

/*
 * The bootloader does not necessarily hand the card over in Transfer state:
 * LK deselects before sleeping it, so Standby (3) and Sleep (5) are both
 * plausible. Repair with the one command that fits and re-check; each branch
 * is logged, so the boot log says which state the hand-off was in.
 */
static int
sdhci_restore_state(uint32_t *r1)
{
	for (int step = 0; step < 3; step++) {
		uint32_t state = (*r1 >> 9) & 0xf;

		if (state == 4) {               /* TRAN: usable */
			return 0;
		}
		printf("WHYRED SDHCI: card state %u (not TRAN), R1 0x%08x\n",
		    state, *r1);
		if (state == 3) {
			printf("WHYRED SDHCI: selecting RCA %u (CMD7)\n",
			    se.rca);
			if (sdhci_select_card(se.rca) != 0) {
				return EIO;
			}
		} else if (state == 5) {
			printf("WHYRED SDHCI: waking RCA %u (CMD5 "
			    "SLEEP_AWAKE)\n", se.rca);
			if (sdhci_sleep_awake(se.rca, r1) != 0) {
				return EIO;
			}
		} else {
			printf("WHYRED SDHCI: no repair for card state %u\n",
			    state);
			return ENXIO;
		}
		if (sdhci_send_status(se.rca, r1) != 0) {
			return EIO;
		}
	}
	return ENXIO;
}

/*
 * Find the RCA the card answers on. The bootloader assigned it in CMD3, and
 * Qualcomm LK/ABL use MMC_RCA 2 (bootloader_lk include/mmc.h), but that is
 * checked rather than assumed: an addressed command with the wrong RCA gets no
 * response at all, which is exactly the "command 13 timed out" seen on
 * hardware. The ladder is ordered by cost; every rung is logged so that one
 * boot log identifies the culprit:
 *
 *   1. CMD13 with RCA 2              - the LK/ABL address
 *   2. CMD13 with RCA 1              - the other address on a one-card bus
 *   3. CORE_VENDOR_SPEC <- POR and   - what Linux writes at probe and on
 *      the HC MCLK re-selected        every rate change, re-asserted here
 *   4. CMD5 awake, RCA 2 then RCA 1  - the card may be asleep
 *   5. CMD0                          - clock test; resets the card, so last
 */
static int
sdhci_claim_card(void)
{
	static const uint32_t candidates[] = { SANTONI_CARD_RCA, 1u };
	uint32_t r1 = 0;
	int error = ETIMEDOUT;
	unsigned i;

	/*
	 * Enumeration already selected the card and moved it to 25 MHz/8-bit/
	 * high-speed. The ladder below is for LK's 400 kHz hand-off. If CMD13
	 * fails now, do not rebuild the 1-bit legacy path: the card would still
	 * be driving eight data bits, so any later read would be garbage.
	 */
	if (se.enumerated) {
		error = sdhci_send_status(se.rca, &r1);
		if (!error) {
			return sdhci_restore_state(&r1);
		}
		printf("WHYRED SDHCI: post-enumeration CMD13 failed (%d); "
		    "not downgrading the card bus\n", error);
		return ENXIO;
	}

	for (i = 0; i < sizeof(candidates) / sizeof(candidates[0]); i++) {
		error = sdhci_send_status(candidates[i], &r1);
		if (!error) {
			break;
		}
		printf("WHYRED SDHCI: CMD13 with RCA %u got no answer\n",
		    candidates[i]);
	}
	if (!error) {
		se.rca = candidates[i];
		printf("WHYRED SDHCI: card answered CMD13 on RCA %u "
		    "(R1 0x%08x, state %u)\n", se.rca, r1, (r1 >> 9) & 0xf);
		return sdhci_restore_state(&r1);
	}

	/*
	 * Rung 3: rebuild the controller. An SDHCI reset wipes
	 * POWER_CONTROL, CLOCK_CONTROL, HOST_CONTROL, HOST_CONTROL2,
	 * TIMEOUT_CONTROL and the interrupt enables; only GCC and the vendor
	 * window survive. Re-writing just CORE_VENDOR_SPEC (the old rung)
	 * left the core unpowered and unclocked, and a bare POR value also
	 * cleared LK's pad power bits. Nothing has answered at this point,
	 * so a full restore cannot cost a working card.
	 */
	printf("WHYRED SDHCI: rung 3: restoring the controller "
	    "(VENDOR_SPEC 0x%08x, CLOCK_CONTROL 0x%04x, POWER_CONTROL 0x%02x, "
	    "HOST_CONTROL 0x%02x, HOST_CONTROL2 0x%04x, TIMEOUT 0x%02x)\n",
	    hc_rd(CORE_VENDOR_SPEC), hc_rd16(SDHCI_CLOCK_CONTROL),
	    hc_rd8(SDHCI_POWER_CONTROL), hc_rd8(SDHCI_HOST_CONTROL),
	    hc_rd16(SDHCI_HOST_CONTROL2), hc_rd8(SDHCI_TIMEOUT_CONTROL));
	if (sdhci_restore_controller() == 0) {
		if (sdhci_send_status(SANTONI_CARD_RCA, &r1) == 0) {
			se.rca = SANTONI_CARD_RCA;
			printf("WHYRED SDHCI: CMD13 answers on RCA %u after "
			    "the controller restore (R1 0x%08x, state %u)\n",
			    se.rca, r1, (r1 >> 9) & 0xf);
			return sdhci_restore_state(&r1);
		}
		printf("WHYRED SDHCI: rung 3: still no CMD13 after a full "
		    "controller restore; the card may have lost VCC/RST_n and "
		    "need CMD0 re-enumeration\n");
	} else {
		printf("WHYRED SDHCI: rung 3: controller restore failed, "
		    "continuing with the card-state rungs\n");
	}

	/*
	 * Rung 4: SLEEP. A sleeping card answers CMD5 but not CMD13, and the
	 * CMD5 response repeats the RCA, so both candidate addresses are
	 * tried and the one that answers wins.
	 */
	for (i = 0; i < sizeof(candidates) / sizeof(candidates[0]); i++) {
		error = sdhci_sleep_awake(candidates[i], &r1);
		if (error) {
			(void)sdhci_reset_paths();
			continue;
		}
		printf("WHYRED SDHCI: CMD5 awake RCA %u -> R1 0x%08x\n",
		    candidates[i], r1);
		se.rca = candidates[i];
		if (sdhci_send_status(se.rca, &r1) != 0) {
			continue;
		}
		printf("WHYRED SDHCI: CMD13 after wake-up (R1 0x%08x, "
		    "state %u)\n", r1, (r1 >> 9) & 0xf);
		return sdhci_restore_state(&r1);
	}

	/*
	 * Rung 5: CMD0 needs no response. It separates "the command path never
	 * shifted" from "the card does not answer", and it sends the card back
	 * to Idle, so it is the very last thing this probe does. Re-identifying
	 * the card from Idle is a later patch, not this one.
	 */
	if (sdhci_cmd(0, 0, 0, 0, NULL) == 0) {
		printf("WHYRED SDHCI: CMD0 completed: the command path shifts, "
		    "but no addressed command answered\n");
	} else {
		printf("WHYRED SDHCI: CMD0 did not complete either: no SDCLK "
		    "reaches the card\n");
	}
	return ENXIO;
}

/*
 * Verify that the 25 MHz 8-bit High-Speed switch is functionally working
 * across the 8 data lines (DAT[7:0]) by re-reading EXT_CSD (CMD8) and
 * verifying that SEC_COUNT and HS_TIMING match expectations.
 *
 * (Note: Per JEDEC JESD84-B51, EXT_CSD[183] BUS_WIDTH is write-only and reads
 * back 0x00, so we check EXT_CSD[185] HS_TIMING == 1 and SEC_COUNT).
 *
 * If 8-bit High-Speed sampling fails, fall back to legacy edge timing
 * (HS_TIMING=0) or 4-bit bus width.
 */
static int
sdhci_verify_highspeed_bus(uint32_t expected_sec_count)
{
	uint32_t verify_sec_count = 0;
	uint8_t hs_timing = 0;
	int error;

	/* Re-read EXT_CSD (CMD8) at 25 MHz 8-bit High-Speed to exercise DAT[7:0] */
	dma_sync(SDHCI_BLOCK, TRUE);
	error = sdhci_read_pio(8, 0, 1, (void *)(se.dma + DMA_BOUNCE));
	if (error == 0) {
		dma_sync(SDHCI_BLOCK, TRUE);
		memcpy(&verify_sec_count, (const void *)(se.dma + DMA_BOUNCE + 212), 4);
		hs_timing = *(const uint8_t *)(se.dma + DMA_BOUNCE + 185);

		if (verify_sec_count == expected_sec_count && hs_timing == 1) {
			printf("WHYRED SDHCI: DAT[7:0] 8-bit HS verified "
			    "(SEC_COUNT %u, HS_TIMING %u)\n",
			    verify_sec_count, hs_timing);
			return 0;
		}
		printf("WHYRED SDHCI: EXT_CSD mismatch at 8-bit HS (got %u vs %u, HS=%u)\n",
		    verify_sec_count, expected_sec_count, hs_timing);
	} else {
		printf("WHYRED SDHCI: EXT_CSD read failed at 8-bit HS (%d)\n", error);
	}

	/* Fallback 1: Disable Host High-Speed edge mode & revert card HS_TIMING=0 */
	printf("WHYRED SDHCI: rolling back HS_TIMING to legacy edge at 25 MHz\n");
	(void)sdhci_switch_ext_csd(EXT_CSD_HS_TIMING, 0, 0, SDHCI_CTRL_HISPD, "HS_TIMING=0");

	dma_sync(SDHCI_BLOCK, TRUE);
	error = sdhci_read_pio(8, 0, 1, (void *)(se.dma + DMA_BOUNCE));
	if (error == 0) {
		dma_sync(SDHCI_BLOCK, TRUE);
		memcpy(&verify_sec_count, (const void *)(se.dma + DMA_BOUNCE + 212), 4);
		if (verify_sec_count == expected_sec_count) {
			printf("WHYRED SDHCI: DAT[7:0] 8-bit legacy edge verified (SEC_COUNT %u)\n",
			    verify_sec_count);
			return 0;
		}
	}

	/* Fallback 2: Drop from 8-bit to 4-bit bus width */
	printf("WHYRED SDHCI: rolling back bus width to 4-bit\n");
	(void)sdhci_switch_ext_csd(EXT_CSD_BUS_WIDTH, EXT_CSD_BUS_WIDTH_4,
	    SDHCI_CTRL_4BITBUS, SDHCI_CTRL_BUS_MASK, "BUS_WIDTH=4");

	dma_sync(SDHCI_BLOCK, TRUE);
	error = sdhci_read_pio(8, 0, 1, (void *)(se.dma + DMA_BOUNCE));
	if (error == 0) {
		dma_sync(SDHCI_BLOCK, TRUE);
		memcpy(&verify_sec_count, (const void *)(se.dma + DMA_BOUNCE + 212), 4);
		if (verify_sec_count == expected_sec_count) {
			printf("WHYRED SDHCI: 4-bit bus verified (SEC_COUNT %u)\n",
			    verify_sec_count);
			return 0;
		}
	}

	/* Fallback 3: one DAT line, legacy edge. */
	printf("WHYRED SDHCI: rolling back bus width to 1-bit\n");
	(void)sdhci_switch_ext_csd(EXT_CSD_BUS_WIDTH, EXT_CSD_BUS_WIDTH_1, 0,
	    SDHCI_CTRL_BUS_MASK, "BUS_WIDTH=1");

	dma_sync(SDHCI_BLOCK, TRUE);
	error = sdhci_read_pio(8, 0, 1, (void *)(se.dma + DMA_BOUNCE));
	if (error == 0) {
		dma_sync(SDHCI_BLOCK, TRUE);
		memcpy(&verify_sec_count, (const void *)(se.dma + DMA_BOUNCE + 212), 4);
		if (verify_sec_count == expected_sec_count) {
			printf("WHYRED SDHCI: 1-bit bus verified (SEC_COUNT %u)\n",
			    verify_sec_count);
			return 0;
		}
	}
	return EIO;
}

static int
santoni_sdhci_read_blocks(uint64_t lba, uint32_t count, void *buf)
{
	if (count == 0 || count > BOUNCE_SIZE / SDHCI_BLOCK || lba > UINT32_MAX) {
		return EINVAL;
	}
	if (se.sec_count && lba + (uint64_t)count > se.sec_count) {
		printf("WHYRED SDHCI: refusing read of %u blocks at 0x%llx "
		    "(device has %u sectors)\n", count, (unsigned long long)lba,
		    se.sec_count);
		return EINVAL;
	}
	return sdhci_read_pio(count == 1 ? 17 : 18, (uint32_t)lba, count, buf);
}

static int
santoni_sdhci_write_blocks(uint64_t lba, uint32_t count, const void *buf)
{
	uint64_t off;

	if (!se.writable) {
		return EROFS;
	}
	if (count == 0 || count > BOUNCE_SIZE / SDHCI_BLOCK ||
	    lba < se.first_block || lba > UINT32_MAX) {
		goto refuse;
	}
	off = lba - se.first_block;
	if (off >= se.nblocks || count > se.nblocks - off) {
		goto refuse;
	}
	if (se.sec_count && lba + (uint64_t)count > se.sec_count) {
		printf("WHYRED SDHCI: refusing write of %u blocks at 0x%llx "
		    "(device has %u sectors)\n", count, (unsigned long long)lba,
		    se.sec_count);
		return EINVAL;
	}
	return sdhci_write_pio((uint32_t)lba, count, buf);

refuse:
	printf("WHYRED SDHCI: refusing write of %u blocks at 0x%llx "
	    "(outside \"%s\")\n", count, (unsigned long long)lba, se.part);
	return EPERM;
}

/*
 * UEFI 2.8 GPT header (section 5.3.1), 92 bytes on the wire. Only used to
 * decode the fields we validate; the CRC32 is computed over the raw bytes.
 */
struct santoni_gpt_header {
	char signature[8];                      /* "EFI PART" */
	uint32_t revision;
	uint32_t header_size;                   /* 92 */
	uint32_t header_crc32;                  /* CRC32 with this field zeroed */
	uint32_t reserved;
	uint64_t my_lba;
	uint64_t alternate_lba;
	uint64_t first_usable_lba;
	uint64_t last_usable_lba;
	uint8_t disk_guid[16];
	uint64_t partition_entry_lba;
	uint32_t num_partition_entries;
	uint32_t sizeof_partition_entry;         /* 128 */
	uint32_t partition_entry_array_crc32;
} __attribute__((packed));

/*
 * IEEE 802.3 CRC32 (reflected, polynomial 0xEDB88320). Matches the UEFI
 * specification, tests/test_gpt_parser.c and every other GPT implementation.
 */
static uint32_t
santoni_crc32(const uint8_t *data, size_t len)
{
	uint32_t crc = 0xFFFFFFFFu;

	for (size_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (int bit = 0; bit < 8; bit++) {
			crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
		}
	}
	return ~crc;
}

/*
 * Validate the GPT and look up both partitions we care about:
 *  - "userdata" (or the emmcroot= override) becomes the BSD disk emmcroot0
 *    (se.first_block / se.nblocks),
 *  - "system" is kept only as the persistent console log area
 *    (se.sys_first_block / se.sys_nblocks); it is intentionally not exposed
 *    as a BSD device.
 * The primary header is verified with its own CRC32 (header_crc32 zeroed) and
 * the partition entry array with the CRC32 over num_entries * entry_size.
 */
static int
santoni_sdhci_find_partition(const char *want)
{
	const uint8_t *b = (const uint8_t *)(se.dma + DMA_BOUNCE);
	struct santoni_gpt_header hdr;
	uint8_t hdr_raw[SDHCI_BLOCK];
	uint64_t entry_lba;
	uint32_t nent, esz, entry_bytes, entry_blocks;
	bool root_found = false;
	uint64_t sys_first = 0, sys_count = 0;
	bool sys_found = false;
	int error;

	error = santoni_sdhci_read_blocks(1, 1, (void *)b);
	if (error) {
		return error;
	}
	if (memcmp(b, "EFI PART", 8) != 0) {
		printf("WHYRED SDHCI: eMMC has no GPT\n");
		return ENXIO;
	}
	memcpy(&hdr, b, sizeof(hdr));
	if (hdr.header_size < sizeof(hdr) || hdr.header_size > SDHCI_BLOCK) {
		printf("WHYRED SDHCI: bad GPT header size %u\n", hdr.header_size);
		return EINVAL;
	}
	/* header_crc32 lives at offset 16 of the header */
	memcpy(hdr_raw, b, SDHCI_BLOCK);
	memset(hdr_raw + 16, 0, 4);
	if (santoni_crc32(hdr_raw, hdr.header_size) != hdr.header_crc32) {
		printf("WHYRED SDHCI: GPT header CRC32 mismatch "
		    "(stored 0x%08x)\n", hdr.header_crc32);
		return EBADMSG;
	}

	esz = hdr.sizeof_partition_entry;
	nent = hdr.num_partition_entries;
	if (esz != 128 || nent == 0 || nent > BOUNCE_SIZE / esz) {
		printf("WHYRED SDHCI: unsupported GPT entries %u x %u\n", nent, esz);
		return ENXIO;
	}
	entry_lba = hdr.partition_entry_lba;
	entry_bytes = nent * esz;
	entry_blocks = (entry_bytes + SDHCI_BLOCK - 1) / SDHCI_BLOCK;
	error = santoni_sdhci_read_blocks(entry_lba, entry_blocks, (void *)b);
	if (error) {
		return error;
	}
	if (santoni_crc32(b, entry_bytes) != hdr.partition_entry_array_crc32) {
		printf("WHYRED SDHCI: GPT entry array CRC32 mismatch "
		    "(stored 0x%08x)\n", hdr.partition_entry_array_crc32);
		return EBADMSG;
	}

	for (uint32_t i = 0; i < nent; i++) {
		const uint8_t *e = b + (uint64_t)i * esz;
		char name[37];
		uint64_t first, last;
		unsigned c;

		for (c = 0; c < 36; c++) {
			uint16_t ch = (uint16_t)(e[56 + 2 * c] | e[57 + 2 * c] << 8);
			if (ch == 0 || ch > 0x7f) {
				break;
			}
			name[c] = (char)ch;
		}
		name[c] = 0;
		if (c == 0) {
			continue;
		}
		memcpy(&first, e + 32, 8);
		memcpy(&last, e + 40, 8);
		if (last < first) {
			printf("WHYRED SDHCI: GPT entry \"%s\" has inverted "
			    "bounds 0x%llx..0x%llx\n", name,
			    (unsigned long long)last, (unsigned long long)first);
			return ENXIO;
		}
		if (strcmp(name, "system") == 0) {
			sys_first = first;
			sys_count = last - first + 1;
			sys_found = true;
			continue;
		}
		if (strcmp(name, want) == 0) {
			se.first_block = first;
			se.nblocks = last - first + 1;
			root_found = true;
		}
	}
	if (!root_found) {
		printf("WHYRED SDHCI: no GPT partition named \"%s\"\n", want);
		return ENOENT;
	}

	se.sys_first_block = 0;
	se.sys_nblocks = 0;
	se.log_ok = false;
	if (sys_found && sys_count >= SANTONI_LOG_SYSTEM_BLOCKS) {
		uint64_t root_end = se.first_block + se.nblocks;
		uint64_t sys_end = sys_first + sys_count;

		/* Equality is not enough: a double-booked or nested "system"
		 * entry would put log writes on the mounted root. */
		if (sys_first < root_end && se.first_block < sys_end) {
			printf("WHYRED SDHCI: \"system\" (0x%llx..0x%llx) "
			    "overlaps \"%s\" (0x%llx..0x%llx): persistent log "
			    "disabled\n",
			    (unsigned long long)sys_first,
			    (unsigned long long)(sys_end - 1), se.part,
			    (unsigned long long)se.first_block,
			    (unsigned long long)(root_end - 1));
		} else {
			se.sys_first_block = sys_first;
			se.sys_nblocks = sys_count;
			se.log_ok = true;
		}
	} else if (sys_found) {
		printf("WHYRED SDHCI: \"system\" too small for the log "
		    "(%llu sectors, need %u)\n",
		    (unsigned long long)sys_count, SANTONI_LOG_SYSTEM_BLOCKS);
	}
	return 0;
}

/*
 * Raw sector writes for the persistent console log flusher
 * (xnu/patches/0143). This is the only write path that does not require
 * boot-arg emmcrw=1 and it is fenced hard to the SYSTEM partition log area:
 * sectors [sys_first_block, sys_first_block + 8193), which can only ever
 * touch the 36-byte header and the 4 MiB ring - never a filesystem.
 */
int santoni_sdhci_write_log_blocks(uint64_t lba, uint32_t count,
    const void *buf);

int
santoni_sdhci_write_log_blocks(uint64_t lba, uint32_t count, const void *buf)
{
	int error;

	/* Off when there is no SYSTEM partition, it is too small, or it
	 * overlaps the root (see santoni_sdhci_find_partition()). */
	if (!se.log_ok) {
		return EPERM;
	}
	if (count == 0 || count > BOUNCE_SIZE / SDHCI_BLOCK) {
		return EINVAL;
	}
	if (se.sys_nblocks < SANTONI_LOG_SYSTEM_BLOCKS ||
	    lba < se.sys_first_block ||
	    lba + (uint64_t)count >
	    se.sys_first_block + SANTONI_LOG_SYSTEM_BLOCKS ||
	    lba > UINT32_MAX) {
		printf("WHYRED SDHCI: refusing log write of %u blocks at "
		    "0x%llx (SYSTEM log is 0x%llx..0x%llx)\n", count,
		    (unsigned long long)lba,
		    (unsigned long long)se.sys_first_block,
		    (unsigned long long)(se.sys_first_block +
		    SANTONI_LOG_SYSTEM_BLOCKS - 1));
		return EPERM;
	}

	/*
	 * WR-01: the caller passes a pointer into the static log ring, never
	 * se.dma + DMA_BOUNCE. strategy() owns that bounce buffer under this
	 * lock; staging the ring into it from the flusher (outside the lock,
	 * as 0143 used to) let a console flush overwrite a mounted-root write
	 * in flight. The ring is read here, under the same lock as the PIO.
	 */
	lck_mtx_lock(&santoni_sdhci_lock);
	error = sdhci_write_pio((uint32_t)lba, count, buf);
	lck_mtx_unlock(&santoni_sdhci_lock);
	return error;
}

static int
santoni_sdhci_open(__unused dev_t dev, int flags, __unused int devtype,
    __unused struct proc *p)
{
	if (!se.ready) {
		return ENXIO;
	}
	return (flags & FWRITE) && !se.writable ? EROFS : 0;
}

static int
santoni_sdhci_close(__unused dev_t dev, __unused int flags,
    __unused int devtype, __unused struct proc *p)
{
	return 0;
}

/*
 * Is this a kernel VA? buf_map() returns bp->b_datap unchanged for a
 * non-clustered buffer, so a B_PHYS buffer carries the (user) address that
 * physio() put there. Kernel VAs live in the high canonical half on arm64.
 */
static inline bool
santoni_kernel_address(caddr_t addr)
{
	return (uintptr_t)addr >= VM_MIN_KERNEL_ADDRESS;
}

/*
 * 0174: take the bounce buffer, and say once if another request already had
 * it. Before this patch the lock was released before the copy, so overlap
 * between two requests looked harmless while it silently swapped the data.
 * This line in a boot log is the proof that the window was reachable.
 */
static void
santoni_sdhci_bounce_lock(void)
{
	static bool contention_logged;

	if (lck_mtx_try_lock(&santoni_sdhci_lock)) {
		return;
	}
	if (!contention_logged) {
		contention_logged = true;
		printf("WHYRED SDHCI: bounce buffer already owned by another "
		    "request; keeping the copy inside the transfer's lock\n");
	}
	lck_mtx_lock(&santoni_sdhci_lock);
}

static void
santoni_sdhci_strategy(buf_t bp)
{
	int error = 0;
	caddr_t vaddr = NULL;
	uint64_t off = (uint64_t)buf_blkno(bp) * se.secsize;
	uint32_t count = buf_count(bp);
	uint64_t part_bytes = se.nblocks * SDHCI_BLOCK;
	bool rd = (buf_flags(bp) & B_READ) != 0;
	bool user;

	buf_setresid(bp, count);
	if (!rd && !se.writable) {
		error = EROFS;
		goto done;
	}
	if (off > part_bytes || count > part_bytes - off) {
		error = EINVAL;
		goto done;
	}
	if (count == 0) {
		goto done;
	}
	if (buf_map(bp, &vaddr)) {
		error = EIO;
		goto done;
	}

	/*
	 * Normal VFS I/O passes kernel-mapped buffers (or UPL-backed B_CLUSTER
	 * buffers), for which memcpy() is correct. A B_PHYS buffer without
	 * B_CLUSTER is different: physio() only vslock()s the pages and leaves
	 * a raw user VA in b_datap. Such a page can still be mapped read-only
	 * with PP_ATTR_MODFAULT, so an EL1 store is a permission fault that
	 * handle_kernel_abort() cannot recover (it looks the address up in
	 * kernel_map). Use copyin()/copyout() so the copyio recovery entry and
	 * the user pmap fault path are used instead.
	 */
	user = (buf_flags(bp) & B_PHYS) != 0 &&
	    !santoni_kernel_address(vaddr);

	for (uint32_t done = 0; done < count;) {
		uint64_t pos = off + done;
		uint64_t lba = pos / SDHCI_BLOCK;
		uint32_t skip = (uint32_t)(pos % SDHCI_BLOCK);
		uint32_t want = MIN(count - done, (uint32_t)BOUNCE_SIZE - skip);
		uint32_t blocks = (skip + want + SDHCI_BLOCK - 1) / SDHCI_BLOCK;
		char *bounce = (char *)(se.dma + DMA_BOUNCE);

		/*
		 * 0174: the bounce buffer is owned by the request that holds
		 * the lock, and only by that request. It used to be released
		 * right after the transfer and the caller's copy happened
		 * afterwards; a second strategy() call could then refill the
		 * bounce buffer before the first one copied it out, so the
		 * caller got a different block with a success status. That is
		 * exactly the shape of the code-signing failure: the expected
		 * hash matched the file on disk while the kernel hashed bytes
		 * from somewhere else, and a signature read came back as the
		 * string table of a different file. The copy therefore stays
		 * inside the same critical section as the transfer. A physio
		 * buffer has its pages vslock()ed, so copyin()/copyout() under
		 * the mutex cannot fault on this same device.
		 */
		if (rd) {
			santoni_sdhci_bounce_lock();
			error = santoni_sdhci_read_blocks(se.first_block + lba,
			    blocks, bounce);
			if (!error) {
				if (user) {
					error = copyout(bounce + skip,
					    (user_addr_t)(vaddr + done), want);
				} else {
					memcpy(vaddr + done, bounce + skip, want);
				}
			}
			lck_mtx_unlock(&santoni_sdhci_lock);
		} else {
			santoni_sdhci_bounce_lock();
			if (skip || (want % SDHCI_BLOCK)) {
				error = santoni_sdhci_read_blocks(se.first_block + lba,
				    blocks, bounce);
			}
			if (!error) {
				if (user) {
					error = copyin((user_addr_t)(vaddr + done),
					    bounce + skip, want);
				} else {
					memcpy(bounce + skip, vaddr + done, want);
				}
			}
			if (!error) {
				error = santoni_sdhci_write_blocks(
				    se.first_block + lba, blocks, bounce);
			}
			lck_mtx_unlock(&santoni_sdhci_lock);
		}
		if (error) {
			break;
		}
		done += want;
		buf_setresid(bp, count - done);
	}
	buf_unmap(bp);

done:
	if (error) {
		buf_seterror(bp, error);
	}
	buf_biodone(bp);
}

static int
santoni_sdhci_ioctl(__unused dev_t dev, u_long cmd, caddr_t data,
    __unused int flag, __unused struct proc *p, int is_char)
{
	uint32_t *f = (uint32_t *)data;
	uint64_t *o = (uint64_t *)data;

	switch (cmd) {
	case DKIOCGETBLOCKSIZE:
		*f = se.secsize;
		return 0;
	case DKIOCSETBLOCKSIZE:
		if (is_char) {
			return ENODEV;
		}
		if (*f < DEV_BSIZE || *f > SDHCI_BLOCK || (*f & (*f - 1))) {
			return EINVAL;
		}
		se.secsize = *f;
		return 0;
	case DKIOCGETPHYSICALBLOCKSIZE:
		*f = SDHCI_BLOCK;
		return 0;
	case DKIOCGETBLOCKCOUNT:
		*o = se.nblocks * SDHCI_BLOCK / se.secsize;
		return 0;
	case DKIOCISWRITABLE:
		*f = se.writable ? 1 : 0;
		return 0;
	case DKIOCGETMAXBLOCKCOUNTREAD:
		*o = BOUNCE_SIZE / se.secsize;
		return 0;
	case DKIOCGETMAXBLOCKCOUNTWRITE:
		*o = se.writable ? BOUNCE_SIZE / se.secsize : 0;
		return 0;
	case DKIOCGETMAXSEGMENTCOUNTREAD:
	case DKIOCGETMAXSEGMENTCOUNTWRITE:
		*o = 1;
		return 0;
	default:
		return ENOTTY;
	}
}

static int
santoni_sdhci_bioctl(dev_t dev, u_long cmd, caddr_t data, int flag,
    struct proc *p)
{
	return santoni_sdhci_ioctl(dev, cmd, data, flag, p, 0);
}

static int
santoni_sdhci_cioctl(dev_t dev, u_long cmd, caddr_t data, int flag,
    struct proc *p)
{
	return santoni_sdhci_ioctl(dev, cmd, data, flag, p, 1);
}

static int
santoni_sdhci_psize(__unused dev_t dev)
{
	return se.ready ?
	       (int)MIN(se.nblocks * SDHCI_BLOCK / se.secsize, (uint64_t)INT_MAX) :
	       -1;
}

/*
 * Character-device transfers use a private kernel bounce buffer and
 * uiomove(), never physio()/buf_map(). physio() only vslock()s the user
 * range and passes the raw user VA to strategy(); a freshly wired anonymous
 * page may still be mapped read-only with PP_ATTR_MODFAULT, and the first
 * store into it from EL1 is a permission fault (ESR DFSC 0x0f). There is no
 * copyio recovery entry for a plain kernel memcpy(), so handle_kernel_abort()
 * resolves the address in kernel_map and panics. uiomove() uses copyin()/
 * copyout(), which have a recovery entry and take the user pmap fault path.
 *
 * The controller bounce buffer (se.dma) is shared with strategy() and the log
 * flusher, so allocate a bounce per call and never hold santoni_sdhci_lock
 * across uiomove(): a user fault can page in through the root filesystem and
 * re-enter this driver.
 */
static int
santoni_sdhci_uio(struct uio *uio, bool rd)
{
	uint64_t part_bytes = se.nblocks * SDHCI_BLOCK;
	vm_offset_t bounce = 0;
	vm_size_t bsize;
	int error = 0;

	if (!se.ready) {
		return ENXIO;
	}
	if (uio_resid(uio) <= 0) {
		return 0;
	}

	bsize = (vm_size_t)MIN((uint64_t)uio_resid(uio) + SDHCI_BLOCK,
	    (uint64_t)BOUNCE_SIZE);
	bsize = roundup(bsize, SDHCI_BLOCK);
	if (kmem_alloc(kernel_map, &bounce, bsize, KMA_DATA,
	    VM_KERN_MEMORY_BSD) != KERN_SUCCESS) {
		return ENOMEM;
	}

	while (uio_resid(uio) > 0) {
		off_t uoff = uio_offset(uio);
		uint64_t pos, left;
		uint32_t skip, want, blocks;
		char *buf = (char *)bounce;

		if (uoff < 0 || (uint64_t)uoff >= part_bytes) {
			break;                  /* EOF: short transfer */
		}
		pos = (uint64_t)uoff;
		left = MIN((uint64_t)uio_resid(uio), part_bytes - pos);
		skip = (uint32_t)(pos % SDHCI_BLOCK);
		want = (uint32_t)MIN(left, (uint64_t)(BOUNCE_SIZE - skip));
		blocks = (skip + want + SDHCI_BLOCK - 1) / SDHCI_BLOCK;

		if (rd) {
			lck_mtx_lock(&santoni_sdhci_lock);
			error = santoni_sdhci_read_blocks(se.first_block +
			    pos / SDHCI_BLOCK, blocks, buf);
			lck_mtx_unlock(&santoni_sdhci_lock);
			if (error) {
				break;
			}
			error = uiomove(buf + skip, (int)want, uio);
		} else {
			if (!se.writable) {
				error = EROFS;
				break;
			}
			if (skip || (want % SDHCI_BLOCK)) {
				lck_mtx_lock(&santoni_sdhci_lock);
				error = santoni_sdhci_read_blocks(se.first_block +
				    pos / SDHCI_BLOCK, blocks, buf);
				lck_mtx_unlock(&santoni_sdhci_lock);
				if (error) {
					break;
				}
			}
			error = uiomove(buf + skip, (int)want, uio);
			if (error) {
				break;
			}
			lck_mtx_lock(&santoni_sdhci_lock);
			error = santoni_sdhci_write_blocks(se.first_block +
			    pos / SDHCI_BLOCK, blocks, buf);
			lck_mtx_unlock(&santoni_sdhci_lock);
		}
		if (error) {
			break;
		}
	}

	kmem_free(kernel_map, bounce, bsize);
	return error;
}

static int
santoni_sdhci_read(__unused dev_t dev, struct uio *uio, __unused int ioflag)
{
	return santoni_sdhci_uio(uio, true);
}

static int
santoni_sdhci_write(__unused dev_t dev, struct uio *uio, __unused int ioflag)
{
	if (!se.writable) {
		return EROFS;
	}
	return santoni_sdhci_uio(uio, false);
}

static const struct bdevsw santoni_sdhci_bdevsw = {
	.d_open     = santoni_sdhci_open,
	.d_close    = santoni_sdhci_close,
	.d_strategy = santoni_sdhci_strategy,
	.d_ioctl    = santoni_sdhci_bioctl,
	.d_dump     = eno_dump,
	.d_psize    = santoni_sdhci_psize,
	.d_type     = D_DISK,
};

static const struct cdevsw santoni_sdhci_cdevsw = {
	.d_open       = santoni_sdhci_open,
	.d_close      = santoni_sdhci_close,
	.d_read       = santoni_sdhci_read,
	.d_write      = santoni_sdhci_write,
	.d_ioctl      = santoni_sdhci_cioctl,
	.d_stop       = eno_stop,
	.d_reset      = eno_reset,
	.d_ttys       = NULL,
	.d_select     = eno_select,
	.d_mmap       = eno_mmap,
	.d_strategy   = eno_strat,
	.d_reserved_1 = eno_getc,
	.d_reserved_2 = eno_putc,
	.d_type       = D_DISK,
};

static char santoni_fail_text[192];

static int
santoni_sdhci_init(void)
{
	DTEntry node;
	uintptr_t const *reg = NULL;
	unsigned int size = 0;
	int error, rw = 0;
	const char *stage = "init";

	if (SecureDTLookupEntry(NULL, "/santoni-sdhci", &node) != kSuccess ||
	    SecureDTGetProperty(node, "reg", (void const **)&reg, &size) != kSuccess ||
	    size < 2 * sizeof(uintptr_t)) {
		printf("WHYRED SDHCI: no /santoni-sdhci node in the device tree\n");
		return ENXIO;
	}
	if (reg[1] < 0x1000) {
		printf("WHYRED SDHCI: /santoni-sdhci reg window too small "
		    "(0x%llx)\n", (unsigned long long)reg[1]);
		return ENXIO;
	}
	strlcpy(se.part, "userdata", sizeof(se.part));
	PE_parse_boot_argn("emmcroot", se.part, sizeof(se.part));
	if (PE_parse_boot_argn("emmcrw", &rw, sizeof(rw))) {
		se.writable = (rw != 0);
	}
	se.secsize = DEV_BSIZE;

	se.regs = ml_io_map(reg[0], reg[1]);
	/*
	 * Only the GCC page holding sdcc1_apps_clk_src is needed (CMD_RCGR
	 * 0x42004 through CBCR 0x42018). Map it separately from the HC window;
	 * both live in device memory and are accessed uncached.
	 */
	se.gcc = ml_io_map(GCC_SDCC1_APPS_PHYS, GCC_SDCC1_APPS_SIZE);
	if (se.gcc == 0) {
		printf("WHYRED SDHCI: no mapping for GCC sdcc1_apps\n");
		return ENXIO;
	}
	if (kmem_alloc_contig(kernel_map, &se.dma, BOUNCE_SIZE, PAGE_MASK, 0, 0,
	    KMA_DATA | KMA_ZERO | KMA_PERMANENT, VM_KERN_MEMORY_IOKIT) !=
	    KERN_SUCCESS) {
		printf("WHYRED SDHCI: no contiguous bounce buffer\n");
		return ENOMEM;
	}
	ppnum_t pn = pmap_find_phys(kernel_pmap, (addr64_t)se.dma);
	if (!pn) {
		return ENOMEM;
	}
	se.dma_pa = ptoa_64(pn);

	lck_mtx_lock(&santoni_sdhci_lock);
	stage = "take-over";
	error = sdhci_take_over();
	if (!error) {
		stage = "EXT_CSD 8-bit verify";
		error = sdhci_verify_highspeed_bus(se.sec_count);
	}
	if (!error) {
		stage = "GPT partition";
		error = santoni_sdhci_find_partition(se.part);
	}
	lck_mtx_unlock(&santoni_sdhci_lock);
	if (error) {
		/*
		 * The per-step printf() burst above scrolls off the 90x27
		 * console within one probe, and the probe is cached, so the
		 * only line IOKitBSDInit can repeat is the error code. Keep a
		 * compact snapshot of exactly where it stopped and the state
		 * that matters for the next boot.
		 */
		snprintf(santoni_fail_text, sizeof(santoni_fail_text),
		    "step=%s err=%d cmd1=%d ocr=0x%08x sec=%u rca=%u enum=%d\n"
		    "clkctl=0x%04x hostctl=0x%02x hostctl2=0x%04x "
		    "int=0x%08x last=%llu us lba0=%s",
		    stage, error, se.cmd1_error, se.cmd1_ocr, se.sec_count,
		    se.rca, se.enumerated ? 1 : 0,
		    hc_rd16(SDHCI_CLOCK_CONTROL), hc_rd8(SDHCI_HOST_CONTROL),
		    hc_rd16(SDHCI_HOST_CONTROL2), hc_rd(SDHCI_INT_STATUS),
		    (unsigned long long)(se.last_cmd_ns / 1000),
		    santoni_lba0_note);
		printf("WHYRED SDHCI: probe failed: %s\n", santoni_fail_text);
		return error;
	}

	se.bmajor = bdevsw_add(-1, &santoni_sdhci_bdevsw);
	if (se.bmajor < 0) {
		return ENXIO;
	}
	se.cmajor = cdevsw_add_with_bdev(-1, &santoni_sdhci_cdevsw, se.bmajor);
	if (se.cmajor < 0) {
		return ENXIO;
	}
	se.bdev = makedev(se.bmajor, 0);
	devfs_make_node(se.bdev, DEVFS_BLOCK, UID_ROOT, GID_OPERATOR, 0640,
	    "emmcroot0");
	devfs_make_node(makedev(se.cmajor, 0), DEVFS_CHAR, UID_ROOT,
	    GID_OPERATOR, 0640, "remmcroot0");
	se.ready = true;
	if (se.sys_nblocks) {
		printf("WHYRED SDHCI: \"system\" first block 0x%llx, %llu MiB%s\n",
		    (unsigned long long)se.sys_first_block,
		    (unsigned long long)(se.sys_nblocks * SDHCI_BLOCK >> 20),
		    se.sys_nblocks >= SANTONI_LOG_SYSTEM_BLOCKS ?
		    " (console log area available)" :
		    " (too small for the console log)");
	}
	printf("WHYRED SDHCI: \"%s\" first block 0x%llx, %llu MiB, %s\n",
	    se.part, (unsigned long long)se.first_block,
	    (unsigned long long)(se.nblocks * SDHCI_BLOCK >> 20),
	    se.writable ? "writable (emmcrw=1)" : "read-only");

	/* Private console log area on SYSTEM, if the partition is there. */
	santoni_log_start();
	/* After the log: the WHYRED FB lines belong in the SYSTEM ring. */
	santoni_fb_test();
	/* Input: touchscreen driver for SDM660 (lavender) / SDM636 (whyred) */
	sweet_touch_init();
	return 0;
}

/*
 * Text IOKitBSDInit repeats after "eMMC probe failed": the probe itself is
 * cached, so without this the console only ever shows the errno.
 */
const char *
santoni_sdhci_failure_text(void)
{
	return santoni_fail_text[0] ? santoni_fail_text : "(no stage recorded)";
}

/*
 * Persistent console log ("santonilog").
 * Filming the phone screen is impractical for non-visual and early daemon
 * panics. Every character that reaches the console (_cnputs) is appended to a
 * lock-free 4 MiB circular ring buffer. A dedicated kernel thread wakes up
 * periodically and commits dirty ring blocks to the idle SYSTEM partition
 * (se.sys_first_block, 8193 sectors: 1 header + 8192 ring sectors).
 *
 * Flash layout on the SYSTEM partition:
 *   Sector 0: struct santoni_log_header (magic "SANTONILOG1", v1, 4 MiB, total, seq)
 *   Sectors 1..8192: 8192 blocks (4 MiB) circular ring buffer
 *
 * Writes go through santoni_sdhci_write_log_blocks(), which is fenced to this
 * 8193-sector area and does not need emmcrw=1, so the mounted HFS+ root on
 * USERDATA is never touched. Extracted from recovery via
 * tools/sweet-log-pull.sh. Enabled only by boot-arg sweetlog=1; default off.
 */
#define SANTONI_LOG_SIZE          (4u << 20)
#define SANTONI_LOG_BLOCKS        (1 + SANTONI_LOG_SIZE / SDHCI_BLOCK)

static char santoni_log_ring[SANTONI_LOG_SIZE];
static _Atomic uint64_t santoni_log_total;
static int santoni_log_chan;

/* Called for all console output from any context: never blocks */
void santoni_log_append(const char *c, int size);

void
santoni_log_append(const char *c, int size)
{
	if (size <= 0) {
		return;
	}
	uint64_t at = os_atomic_add_orig(&santoni_log_total, (uint64_t)size,
	    relaxed);

	for (int i = 0; i < size; i++) {
		santoni_log_ring[(at + (uint64_t)i) % SANTONI_LOG_SIZE] = c[i];
	}
}

/*
 * 0176: one-shot test pattern in the boot framebuffer, gated on
 * sweetfbtest=1. The console is on the UART (serial=3) and kPEAcquireScreen
 * clears the boot framebuffer, so a black panel says nothing about MDSS/DSI.
 * Four bars tell the two apart: bars on the panel mean ABL's continuous
 * splash is still scanned out and a black UI is a userspace problem; no bars
 * mean the panel is off and the display path must be brought up before any
 * UI can be visible. Runs once, early in BSD init, long before backboardd.
 *
 * 0177: map it the way the console does. ml_io_map_wcomb() allocates from
 * io_submap, which is only 8 MiB (IO_MAP_SIZE, osfmk/arm/io_map.c) and has
 * no room for a 1080x2160x32 framebuffer: the 0176 boot panicked before the
 * first pixel with "kmem_alloc(...): failed with 3" (KERN_NO_SPACE).
 * ml_io_map_unmappable() allocates from kernel_map and is the call
 * initialize_screen() itself uses for this framebuffer in
 * osfmk/console/video_console.c. VM_WIMG_IO matches that mapping: this board
 * config has no HAS_UCNORMAL_MEM, so the console maps the panel uncached
 * device memory too.
 */
#define WHYRED_FB_PHYS          0x9d400000ULL
#define WHYRED_FB_WIDTH         1080u
#define WHYRED_FB_HEIGHT        2160u
#define WHYRED_FB_STRIDE        4320u   /* bytes, 32bpp */

/* osfmk/arm/machine_routines.h, not exported to the BSD layer */
extern vm_offset_t ml_io_map_unmappable(vm_offset_t phys_addr, vm_size_t size,
    uint32_t flags);

static void
santoni_fb_test(void)
{
	static const uint32_t bars[4] = {
		0xffe02020, 0xff20e020, 0xff2020e0, 0xffe8e8e8
	};
	PE_Video console;
	uint32_t on = 0;
	vm_size_t fb_size = (vm_size_t)WHYRED_FB_STRIDE * WHYRED_FB_HEIGHT;
	volatile uint32_t *fb;

	if (!PE_parse_boot_argn("sweetfbtest", &on, sizeof(on)) || !on) {
		return;
	}

	/*
	 * PE_current_console() returns PE_state.video, whose v_baseAddr stays
	 * the physical address ABL put in boot_args (pexpert/arm/pe_init.c);
	 * the virtual mapping initialize_screen() made lives in vc_info and is
	 * not exported. Print it so every boot shows which address is physical
	 * and whether the geometry still matches the constants above.
	 */
	PE_current_console(&console);
	printf("WHYRED FB: console base 0x%llx %llux%llu stride %llu depth "
	    "%llu\n",
	    (unsigned long long)console.v_baseAddr,
	    (unsigned long long)console.v_width,
	    (unsigned long long)console.v_height,
	    (unsigned long long)console.v_rowBytes,
	    (unsigned long long)console.v_depth);

	fb = (volatile uint32_t *)ml_io_map_unmappable(WHYRED_FB_PHYS,
	    fb_size, VM_WIMG_IO);
	if (fb == NULL) {
		printf("WHYRED FB: test pattern map failed (%llu bytes)\n",
		    (unsigned long long)fb_size);
		return;
	}
	for (uint32_t y = 0; y < WHYRED_FB_HEIGHT; y++) {
		volatile uint32_t *line = fb + (uint64_t)y *
		    (WHYRED_FB_STRIDE / 4);
		for (uint32_t x = 0; x < WHYRED_FB_WIDTH; x++) {
			line[x] = bars[x / (WHYRED_FB_WIDTH / 4)];
		}
	}
	printf("WHYRED FB: test pattern 4 bars at 0x%llx (%ux%u, stride %u, "
	    "%llu B)\n",
	    (unsigned long long)WHYRED_FB_PHYS, WHYRED_FB_WIDTH,
	    WHYRED_FB_HEIGHT, WHYRED_FB_STRIDE, (unsigned long long)fb_size);
	/* Read back one pixel per bar: proves the stores reached the bus even
	 * when the panel stays dark. */
	printf("WHYRED FB: readback %08x %08x %08x %08x\n",
	    fb[0], fb[WHYRED_FB_WIDTH / 4], fb[WHYRED_FB_WIDTH / 2],
	    fb[3 * WHYRED_FB_WIDTH / 4]);
}

struct santoni_log_header {
	char magic[12];
	uint32_t version;
	uint32_t size;
	uint64_t total;
	uint64_t seq;
} __attribute__((packed));

/*
 * Block 0 of the log area: the "SANTONILOG1" header. Split out because 0177
 * showed the header is the fragile part. A boot with sweetlog=1 left 26
 * blocks of real console text on SYSTEM (offset 0x200 onwards) while sector 0
 * was still all zero, so every pull refused the area and the log looked
 * missing. The header is now written before the first ring block and again
 * after every flush; a header failure is reported but no longer stops the
 * flusher, because the ring blocks are the payload and a stale header is
 * still better than none.
 */
static int
santoni_log_write_header(uint64_t base, uint64_t total, uint64_t seq)
{
	struct santoni_log_header h = {
		.magic = "SANTONILOG1",
		.version = 1,
		.size = SANTONI_LOG_SIZE,
		.total = total,
		.seq = seq,
	};
	uint8_t bounce[SDHCI_BLOCK];

	bzero(bounce, SDHCI_BLOCK);
	memcpy(bounce, &h, sizeof(h));
	return santoni_sdhci_write_log_blocks(base, 1, bounce);
}

static void
santoni_log_thread(__unused void *arg, __unused wait_result_t wr)
{
	uint64_t base = se.sys_first_block;
	uint64_t flushed = 0, seq = 0;
	bool header_warned = false, first_flush = false;
	int herr;

	printf("WHYRED LOG: console log flusher active on SYSTEM partition "
	    "(block 0x%llx..0x%llx, %u KiB)\n",
	    (unsigned long long)base,
	    (unsigned long long)(base + SANTONI_LOG_BLOCKS - 1),
	    (SANTONI_LOG_BLOCKS * SDHCI_BLOCK) >> 10);

	/* Label the area while the card is idle: sector 0 must identify the
	 * ring even if this boot stops in the middle of the first flush. */
	herr = santoni_log_write_header(base, 0, 0);
	if (herr) {
		printf("WHYRED LOG: header write failed (%d), ring blocks "
		    "still written\n", herr);
	}
	for (;;) {
		(void)tsleep(&santoni_log_chan, PZERO, "santonilog", hz);

		uint64_t total = os_atomic_load(&santoni_log_total, relaxed);
		if (total == flushed) {
			continue;
		}
		uint64_t pos = total - flushed >= SANTONI_LOG_SIZE ?
		    total - SANTONI_LOG_SIZE : flushed;
		int error = 0;

		while (pos < total && !error) {
			uint32_t off = (uint32_t)(pos % SANTONI_LOG_SIZE);
			uint32_t blk = off / SDHCI_BLOCK;
			uint64_t need = ((uint64_t)(off % SDHCI_BLOCK) +
			    (total - pos) + SDHCI_BLOCK - 1) / SDHCI_BLOCK;
			uint32_t n = (uint32_t)MIN(MIN(need,
			    (uint64_t)(SANTONI_LOG_SIZE / SDHCI_BLOCK - blk)),
			    (uint64_t)(BOUNCE_SIZE / SDHCI_BLOCK));

			error = santoni_sdhci_write_log_blocks(base + 1 + blk,
			    n, santoni_log_ring + (uint64_t)blk * SDHCI_BLOCK);
			pos += (uint64_t)(blk + n) * SDHCI_BLOCK - off;
		}
		if (error) {
			printf("WHYRED LOG: ring write failed (%d), stopping "
			    "flusher\n", error);
			return;
		}
		herr = santoni_log_write_header(base, total, ++seq);
		if (herr && !header_warned) {
			header_warned = true;
			printf("WHYRED LOG: header update failed (%d), ring "
			    "still written\n", herr);
		}
		if (!first_flush) {
			first_flush = true;
			/* Decisive line: the first flush used to be the end of
			 * the log, because the header write that followed it
			 * took the flusher down. */
			printf("WHYRED LOG: first flush ok (%llu bytes, header "
			    "%s)\n", (unsigned long long)total,
			    herr ? "failed" : "ok");
		}
		flushed = total;
	}
}

static void
santoni_log_start(void)
{
	uint32_t on = 0;
	thread_t thread;

	PE_parse_boot_argn("sweetlog", &on, sizeof(on));
	if (!on || !se.log_ok) {
		return;
	}
	if (kernel_thread_start(santoni_log_thread, NULL, &thread) ==
	    KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}

/*
 * Idempotent, non-fatal probe. IOKitBSDInit calls this on every boot before
 * IOFindBSDRoot: a missing or unhappy eMMC must not panic a RAM-disk boot.
 * The first attempt is remembered, so rd=emmcroot0 cannot initialise twice.
 */
static bool attempted;
static int init_error;

int santoni_sdhci_probe(void);

int
santoni_sdhci_probe(void)
{
	if (!attempted) {
		attempted = true;
		init_error = santoni_sdhci_init();
	}
	return init_error;
}

/* Root device for rd=emmcroot0 (IOFindBSDRoot). */
int santoni_sdhci_rootdev(dev_t *dev);

int
santoni_sdhci_rootdev(dev_t *dev)
{
	int error = santoni_sdhci_probe();

	if (error) {
		return error;
	}
	if (!se.ready) {
		return ENXIO;
	}
	*dev = se.bdev;
	return 0;
}

#endif /* ARM64_BOARD_CONFIG_WHYRED || ARM64_BOARD_CONFIG_LAVENDER */
