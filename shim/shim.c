/*
 * lavender bare-metal shim: entry and diagnostic loader for SDM660 (Redmi Note 7)
 *   - CPU feature registers (16K granule, ID_AA64MMFR0_EL1, MIDR_EL1)
 *   - Core handoff: wakes Cortex-A73 Gold core 0x100 via PSCI CPU_ON if on Silver
 *   - GICv3 setup: Distributor (0x17a00000) & Redistributor (0x17b00000)
 *   - Stage-1 MMU bring-up (16K granule, 36-bit physical addressing)
 *   - Continuous splash 1080x2340 32-bit simplefb console at 0x9d400000
 *   - Mach-O payload segment loader and boot_args builder
 * Any unexpected exception paints a blue panic screen.
 */
#include <stdint.h>
#include "console.h"
#include "mmu.h"
#include "loader.h"

#define IMEM_RESTART_REASON	0x146bf65cUL	/* qcom,msm-imem@146bf000 restart_reason@65c */
#define RESTART_BOOTLOADER	0x77665500U
#define PSHOLD			0x010ac000UL	/* qcom,pshold@10ac000 */

#define GICD_BASE		0x17a00000UL	/* SDM660 GICD (64 KiB) */
#define GICR_BASE		0x17b00000UL	/* SDM660 GICR (1 MiB, 8 PEs) */
#define GICR_SIZE		0x100000UL
#define GICR_PE_SIZE		0x20000UL
#define GICR_TYPER		0x08
#define GICR_WAKER		0x14
#define GICR_SGI		0x10000
#define GICR_IGROUPR0		(GICR_SGI + 0x080)
#define GICR_ISENABLER0		(GICR_SGI + 0x100)
#define GICR_ICENABLER0		(GICR_SGI + 0x180)
#define GICR_IPRIORITYR		(GICR_SGI + 0x400)
#define GICR_ISPENDR0		(GICR_SGI + 0x200)
#define GICR_ISACTIVER0		(GICR_SGI + 0x300)

#define VTIMER_INTID		19	/* SDM660: <GIC_PPI 3 ...> = 16 + 3 */
#define PTIMER_INTID		18	/* SDM660: <GIC_PPI 2 ...> = 16 + 2 */
#define SGI_INTID		1

#define PSCI_VERSION		0x84000000UL
#define PSCI_CPU_SUSPEND	0xc4000001UL
#define PSCI_CPU_OFF		0x84000002UL
#define PSCI_CPU_ON		0xc4000003UL
#define PSCI_SYSTEM_RESET	0x84000009UL
#define SMCCC_VERSION		0x80000000UL

#define NR_CPUS			8

#define rsr(r) ({ uint64_t _v; __asm__ volatile("mrs %0, " r : "=r"(_v)); _v; })
#define wsr(r, v) __asm__ volatile("msr " r ", %0" : : "r"((uint64_t)(v)))
#define isb() __asm__ volatile("isb" ::: "memory")

static inline uint32_t rd32(uintptr_t a) { return *(volatile uint32_t *)a; }
static inline uint64_t rd64(uintptr_t a) { return *(volatile uint64_t *)a; }
static inline void wr32(uintptr_t a, uint32_t v) { *(volatile uint32_t *)a = v; }
static inline void wr8(uintptr_t a, uint8_t v) { *(volatile uint8_t *)a = v; }

extern char secondary_entry[];

static const char *stage;
static volatile uint64_t gold_dtb;
static volatile uint64_t silver_ticks;
static volatile uint64_t sec_mpidr[NR_CPUS];


static uint64_t smc(uint64_t fn, uint64_t a1, uint64_t a2, uint64_t a3)
{
	register uint64_t x0 __asm__("x0") = fn;
	register uint64_t x1 __asm__("x1") = a1;
	register uint64_t x2 __asm__("x2") = a2;
	register uint64_t x3 __asm__("x3") = a3;
	__asm__ volatile("smc #0"
			 : "+r"(x0), "+r"(x1), "+r"(x2), "+r"(x3)
			 :
			 : "x4", "x5", "x6", "x7", "x8", "x9", "x10", "x11",
			   "x12", "x13", "x14", "x15", "x16", "x17", "memory");
	return x0;
}

static uint64_t now(void)
{
	isb();
	return rsr("cntpct_el0");
}

static void delay_ms(unsigned ms)
{
	uint64_t end = now() + rsr("cntfrq_el0") / 1000 * ms;
	while (now() < end)
		;
}

static void __attribute__((noreturn)) reset(unsigned after_s)
{
	con_skip();
	con_str("RESET", after_s > 20 ? "in 40s" : "in 20s");
	delay_ms(after_s * 1000);
	wr32(IMEM_RESTART_REASON, RESTART_BOOTLOADER);
	__asm__ volatile("dsb sy" ::: "memory");
	smc(PSCI_SYSTEM_RESET, 0, 0, 0);
	wr32(PSHOLD, 0);
	for (;;)
		__asm__ volatile("wfe");
}

/* ---- exceptions ---- */

void exc_handler(uint64_t slot, uint64_t *regs)
{
	if (slot == 5) {	/* IRQ from current EL, SPx */
		uint64_t iar = rsr("icc_iar1_el1");
		uint32_t id = iar & 0xffffff;
		if (id == VTIMER_INTID)
			wsr("cntv_tval_el0", rsr("cntfrq_el0") / 100);
		wsr("icc_eoir1_el1", iar);
		isb();
		return;
	}
	con_clear(BLUE);
	switch (slot & 3) {
	case 0: con_str("PANIC", "SYNC"); break;
	case 1: con_str("PANIC", "IRQ"); break;
	case 2: con_str("PANIC", "FIQ"); break;
	default: con_str("PANIC", "SERROR"); break;
	}
	con_hex2("SLOT", "n=", slot, "cpu=", rsr("mpidr_el1"));
	con_str("STAGE", stage);
	con_hex("ESR", rsr("esr_el1"));
	con_hex("ELR", rsr("elr_el1"));
	con_hex("FAR", rsr("far_el1"));
	con_hex("SPSR", rsr("spsr_el1"));
	con_hex("X0", regs[0]);
	con_hex("X1", regs[1]);
	con_hex("X30", regs[29]);
	reset(40);
}

/* ---- GICv3 setup ---- */

static uintptr_t find_gicr(void)
{
	uint64_t aff = rsr("mpidr_el1");
	uint32_t want = (aff & 0xffffff) | ((aff >> 8) & 0xff000000);

	for (uintptr_t off = 0; off < GICR_SIZE; off += GICR_PE_SIZE) {
		uint64_t typer = rd64(GICR_BASE + off + GICR_TYPER);
		if ((uint32_t)(typer >> 32) == want)
			return GICR_BASE + off;
		if (typer & (1 << 4))	/* Last */
			break;
	}
	return 0;
}

static void gicd_wait_rwp(void)
{
	for (int i = 0; i < 1000000 && (rd32(GICD_BASE) & (1U << 31)); i++)
		;
}

static void gold_gicr_prepare(void)
{
	for (uint32_t aff = 0x101; aff <= 0x103; aff++) {
		uintptr_t r = 0;
		for (uintptr_t off = 0; off < GICR_SIZE; off += GICR_PE_SIZE) {
			uint64_t typer = rd64(GICR_BASE + off + GICR_TYPER);
			if ((uint32_t)(typer >> 32) == aff) {
				r = GICR_BASE + off;
				break;
			}
			if (typer & (1 << 4))
				break;
		}
		if (!r)
			continue;
		uint32_t waker = rd32(r + GICR_WAKER);
		if (waker & 2) {
			wr32(r + GICR_WAKER, waker & ~2U);
			for (int i = 0; i < 100000 && (rd32(r + GICR_WAKER) & 4); i++)
				;
		}
		wr32(r + GICR_IGROUPR0, 0xffffffff);
		for (int id = 0; id < 32; id++)
			wr8(r + GICR_IPRIORITYR + id, 0xa0);
		wr32(r + GICR_ISENABLER0, (1U << VTIMER_INTID) | (1U << PTIMER_INTID) | (1U << SGI_INTID));
	}
}

static void test_gic(void)
{
	stage = "gic sre";
	uint64_t sre = rsr("icc_sre_el1");
	if (!(sre & 1)) {
		wsr("icc_sre_el1", sre | 1);
		isb();
	}
	if (!(rsr("icc_sre_el1") & 1)) {
		con_str("GIC", "no sysreg iface, skip");
		return;
	}

	stage = "gic dist";
	uint32_t dctlr = rd32(GICD_BASE);
	wr32(GICD_BASE, dctlr | 0x13);
	gicd_wait_rwp();
	con_hex2("GICD", "was=", dctlr, "now=", rd32(GICD_BASE));

	stage = "gic redist";
	uintptr_t r = find_gicr();
	if (!r) {
		con_str("GICR", "not found");
		return;
	}
	uint32_t waker = rd32(r + GICR_WAKER);
	if (waker & 2) {
		wr32(r + GICR_WAKER, waker & ~2U);
		for (int i = 0; i < 100000 && (rd32(r + GICR_WAKER) & 4); i++)
			;
	}
	con_hex2("GICR", "base=", r, "waker=", rd32(r + GICR_WAKER));

	stage = "gic config";
	wr32(r + GICR_IGROUPR0, 0xffffffff);
	for (int id = 0; id < 32; id++)
		wr8(r + GICR_IPRIORITYR + id, 0xa0);
	wr32(r + GICR_ISENABLER0, (1U << VTIMER_INTID) | (1U << PTIMER_INTID) | (1U << SGI_INTID));

	uint64_t ctlr = rsr("icc_ctlr_el1");
	wsr("icc_ctlr_el1", ctlr & ~2ULL);	/* EOImode 0 */
	wsr("icc_pmr_el1", 0xf0);
	wsr("icc_bpr1_el1", 0);
	wsr("icc_igrpen1_el1", 1);
	isb();
	con_hex2("ICC", "ctlr=", (uint32_t)ctlr, "grpen1=", (uint32_t)rsr("icc_igrpen1_el1"));
}

/* Speed benchmark loop */
static uint64_t speed_ticks(void)
{
	uint64_t sctlr = rsr("sctlr_el1");
	wsr("sctlr_el1", sctlr | (1UL << 12));
	isb();
	uint64_t t = now(), x = 0;
	for (uint64_t i = 0; i < 20000000; i++)
		__asm__ volatile("add %0, %0, #1" : "+r"(x));
	t = now() - t;
	wsr("sctlr_el1", sctlr);
	isb();
	return t;
}

static void __attribute__((noreturn)) gold_main(uint64_t dtb)
{
	uint64_t mmfr0 = rsr("id_aa64mmfr0_el1");

	con_hex2("GOLD", "midr=", (uint32_t)rsr("midr_el1"), "mpidr=", (uint32_t)rsr("mpidr_el1"));
	con_hex2("MMFR0", "lo=", (uint32_t)mmfr0, "tg16=", (mmfr0 >> 20) & 0xf);
	con_skip();
	if (((mmfr0 >> 20) & 0xf) == 0) {
		con_str("GOLD", "no 16K granule here either");
	} else {
		test_gic();
		gold_gicr_prepare();
		uint64_t gt = speed_ticks();
		con_hex2("SPEED", "silver=", (uint32_t)silver_ticks, "gold=", (uint32_t)gt);
		delay_ms(500);
		con_skip();
		con_str("LOADER", loader_boot(dtb));
	}
	for (;;)
		__asm__ volatile("wfe");
}

void secondary_main(uint64_t idx)
{
	if (gold_dtb)
		gold_main(gold_dtb);
	if (idx < NR_CPUS)
		sec_mpidr[idx] = rsr("mpidr_el1") | (1ULL << 63);
	__asm__ volatile("dsb sy" ::: "memory");
	smc(PSCI_CPU_OFF, 0, 0, 0);
}

void shim_main(uint64_t dtb)
{
	stage = "loader";
	con_clear(RED);
	con_str("SHIM", "lavender bare-metal shim (SDM660)");

	uint64_t mmfr0 = rsr("id_aa64mmfr0_el1");
	con_hex2("MMFR0", "lo=", (uint32_t)mmfr0, "tg16=", (mmfr0 >> 20) & 0xf);
	con_hex2("MIDR", "el1=", (uint32_t)rsr("midr_el1"), "mpidr=", (uint32_t)rsr("mpidr_el1"));

	/*
	 * lavender: XNU requires 16K translation granule. ABL may start us on
	 * a Kryo 260 Silver core (Cortex-A53 class, TGran16 = 0). The Gold A73
	 * cluster (0x100..0x103) has 16K support. Hand off to Gold 0x100 via PSCI.
	 */
	if (((mmfr0 >> 20) & 0xf) == 0) {
		con_str("SILVER", "no 16K granule: waking gold CPU 0x100");
		con_skip();
		silver_ticks = speed_ticks();
		gold_dtb = dtb;
		__asm__ volatile("dsb sy; isb" ::: "memory");
		int64_t r = smc(PSCI_CPU_ON, 0x100, (uintptr_t)secondary_entry, 4);
		if (r == 0) {
			for (;;)
				__asm__ volatile("wfi");
		}
		gold_dtb = 0;
		con_hex("CPU_ON", (uint64_t)r);
	} else {
		test_gic();
		gold_gicr_prepare();
	}

	con_skip();
	const char *why = loader_boot(dtb);
	con_str("LOADER", why);
	con_skip();
	reset(20);
}
