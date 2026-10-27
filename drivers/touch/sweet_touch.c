/*
 * SDM660 (lavender) & SDM636 (whyred) BLSP1 QUP1 Touchscreen Driver.
 * Novatek NT36672A (F7A Tianma) and Synaptics TCM (F7A).
 *
 * Hardware connections (from vendor device tree & schematics):
 *   I2C      BLSP1 QUP1 @ 0x0c175000 (FIFO mode, 400 kHz, QUP v1 tags)
 *   Clocks   GCC_BLSP1_AHB_CLK (0x00100000 + 0x52004 bit 17)
 *            GCC_BLSP1_QUP1_I2C_APPS_CLK (0x00100000 + 0x19008 bit 0)
 *   Pins     TLMM SOUTH (0x03100000): GPIO 2 (SDA), GPIO 3 (SCL) func 3 (blsp_i2c1)
 *            TLMM NORTH (0x03900000): GPIO 66 (Reset, out high), GPIO 67 (IRQ, in pull-up)
 *   Power    1.8V VCC_I2C from PM660 L11 (already enabled by ABL display bring-up)
 *   Protocol Novatek: command {0x00,0x69} to 0x62 boots the MCU; firmware then
 *            answers at 7-bit 0x01 (the vendor's "FW address"); reports are
 *            65 bytes from register 0x00, records at 0+6*n. Reading the
 *            vendor driver confirmed 0x62 is only the hardware command port.
 *            Synaptics TCM answers at 0x20 with plain reads (no register
 *            pointer): 4-byte header {0xa5, code, len lo, len hi}, payload,
 *            padding 0x5a. TCM report parsing needs the config descriptor,
 *            which is not fetched yet; the header is logged for now.
 *   Events   Dispatched directly to Apple IOHIDEventService (SweetHIDTouch)
 */
#if defined(ARM64_BOARD_CONFIG_SWEET) || defined(ARM64_BOARD_CONFIG_LAVENDER) || defined(ARM64_BOARD_CONFIG_WHYRED)
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/errno.h>
#include <kern/clock.h>
#include <kern/thread.h>
#include <kern/sched_prim.h>
#include <machine/machine_routines.h>
#include <pexpert/pexpert.h>

extern vm_offset_t ml_io_map(vm_offset_t, vm_size_t);
extern kern_return_t thread_terminate(thread_t);
extern void sweet_hid_touch(int finger, int down, int x, int y);
void sweet_touch_init(void);

#define TOUCH_GCC_PA         0x00100000u
#define TOUCH_QUP_PA         0x0c175000u
#define TOUCH_TLMM_SOUTH_PA  0x03100000u
#define TOUCH_TLMM_NORTH_PA  0x03900000u

#define TOUCH_GCC_AHB_VOTE   0x52004u
#define TOUCH_GCC_QUP_CBCR   0x19008u
#define TOUCH_GCC_QUP_RCGR   0x19020u

#define TOUCH_TLMM_GPIO2     0x02000u   /* SDA */
#define TOUCH_TLMM_GPIO3     0x03000u   /* SCL */
#define TOUCH_TLMM_GPIO66    0x42000u   /* Reset */
#define TOUCH_TLMM_GPIO67    0x43000u   /* IRQ */

#define TOUCH_ADDR_NVT_HW    0x62u      /* Novatek command port */
#define TOUCH_ADDR_NVT       0x01u      /* Novatek firmware/report port */
#define TOUCH_ADDR_SYNA      0x20u
#define TOUCH_MAX            10u
#define TOUCH_NOVATEK_LEN    65u
#define TOUCH_NVT_EVENT_BUF  0x21C00u   /* NT36672A memory map */
#define TOUCH_TCM_BUF        160u

#define TOUCH_KIND_NONE      0
#define TOUCH_KIND_NOVATEK   1
#define TOUCH_KIND_SYNA      2

/* QUP registers (Qualcomm QUP v2 in version-1 tag FIFO mode) */
#define QUP_CONFIG           0x000u
#define QUP_STATE            0x004u
#define QUP_IO_MODE          0x008u
#define QUP_SW_RESET         0x00cu
#define QUP_OPERATIONAL      0x018u
#define QUP_ERROR_FLAGS      0x01cu
#define QUP_HW_VERSION       0x030u
#define QUP_MX_OUTPUT_CNT    0x100u
#define QUP_OUT_FIFO         0x110u
#define QUP_MX_WRITE_CNT     0x150u
#define QUP_MX_INPUT_CNT     0x200u
#define QUP_MX_READ_CNT      0x208u
#define QUP_IN_FIFO          0x218u
#define QUP_I2C_CLK_CTL      0x400u
#define QUP_I2C_STATUS       0x404u
#define QUP_I2C_MASTER_GEN   0x408u

#define QUP_STATE_RESET      0u
#define QUP_STATE_RUN        1u
#define QUP_STATE_PAUSE      3u
#define QUP_STATE_VALID      (1u << 2)
#define QUP_STATE_MASTER     (1u << 4)
#define QUP_IN_NOT_EMPTY     (1u << 5)
#define QUP_OUT_SVC          (1u << 8)
#define QUP_IN_SVC           (1u << 9)
#define QUP_MX_OUTPUT_DONE   (1u << 10)
#define QUP_NO_INPUT         (1u << 7)
#define QUP_MINI_CORE        (2u << 8)
#define QUP_N_VAL            15u
#define QUP_REPACK_EN        (3u << 14)
#define QUP_TAG_START        (1u << 8)
#define QUP_TAG_DATA         (2u << 8)
#define QUP_TAG_STOP         (3u << 8)
#define QUP_TAG_REC          (4u << 8)
#define QUP_BUS_ACTIVE       (1u << 8)
#define QUP_BUS_ERRORS       0x38000fcu
#define QUP_ERRORS           0x7cu

/* 400 kHz from 19.2 MHz source: fs_div=28, ht_div=14 */
#define QUP_CLK_400K         ((14u << 16) | (3u << 8) | 28u)

static struct {
	bool started;
	int kind;
	vm_offset_t gcc, qup, south, north;
	unsigned nvt_xmax, nvt_ymax;
	unsigned nvt_dumps, nvt_errs, tcm_logs;
	uint32_t last_bus, last_qerr;
	uint32_t active;
	int x[TOUCH_MAX], y[TOUCH_MAX];
} st;

static int touch_sleep_chan;

static inline uint32_t
touch_rd(vm_offset_t base, uint32_t off)
{
	uint32_t v = *(volatile uint32_t *)(base + off);
	__asm__ volatile("dmb oshld" ::: "memory");
	return v;
}

static inline void
touch_wr(vm_offset_t base, uint32_t off, uint32_t val)
{
	*(volatile uint32_t *)(base + off) = val;
	__asm__ volatile("dmb oshst" ::: "memory");
}

static void
touch_sleep_ms(unsigned ms)
{
	assert_wait_timeout(&touch_sleep_chan, THREAD_UNINT, ms, 1000000);
	thread_block(THREAD_CONTINUE_NULL);
}

static bool
qup_state(uint32_t state)
{
	for (unsigned i = 0; i < 100; i++) {
		uint32_t s = touch_rd(st.qup, QUP_STATE);
		if ((s & QUP_STATE_VALID) && (s & 3u) == state) {
			return true;
		}
		if (s & QUP_STATE_VALID) {
			touch_wr(st.qup, QUP_STATE, state);
		}
		delay(1);
	}
	return false;
}

static int
qup_msg(uint8_t addr, bool rd, uint8_t *buf, unsigned len)
{
	unsigned tx = rd ? 2u : len + 1u;
	uint32_t a8 = (addr << 1) | (rd ? 1u : 0u);
	if (!qup_state(QUP_STATE_RESET) ||
	    !(touch_rd(st.qup, QUP_STATE) & QUP_STATE_MASTER)) {
		return EIO;
	}
	touch_wr(st.qup, QUP_MX_OUTPUT_CNT, 0);
	touch_wr(st.qup, QUP_MX_WRITE_CNT, tx);
	if (rd) {
		touch_wr(st.qup, QUP_MX_INPUT_CNT, 0);
		touch_wr(st.qup, QUP_MX_READ_CNT, len);
	}
	touch_wr(st.qup, QUP_CONFIG, QUP_MINI_CORE | QUP_N_VAL | (rd ? 0u : QUP_NO_INPUT));
	touch_wr(st.qup, QUP_IO_MODE, QUP_REPACK_EN);
	if (!qup_state(QUP_STATE_RUN)) {
		return EIO;
	}
	touch_wr(st.qup, QUP_I2C_CLK_CTL, QUP_CLK_400K);
	if (!qup_state(QUP_STATE_PAUSE)) {
		return EIO;
	}
	if (rd) {
		touch_wr(st.qup, QUP_OUT_FIFO, ((QUP_TAG_REC | len) << 16) | QUP_TAG_START | a8);
	} else {
		uint32_t w = QUP_TAG_START | a8;
		for (unsigned i = 0; i < len; i++) {
			uint32_t tag = ((i == len - 1) ? QUP_TAG_STOP : QUP_TAG_DATA) | buf[i];
			if ((i & 1u) == 0) {
				touch_wr(st.qup, QUP_OUT_FIFO, w | (tag << 16));
				w = 0;
			} else {
				w = tag;
			}
		}
		if (!(len & 1u)) {
			touch_wr(st.qup, QUP_OUT_FIFO, w);
		}
	}
	if (!qup_state(QUP_STATE_RUN)) {
		return EIO;
	}
	unsigned pos = 0;
	int err = EIO;
	for (unsigned t = 0; t < 20000; t++) {
		uint32_t bus = touch_rd(st.qup, QUP_I2C_STATUS) & QUP_BUS_ERRORS;
		uint32_t qerr = touch_rd(st.qup, QUP_ERROR_FLAGS) & QUP_ERRORS;
		uint32_t op = touch_rd(st.qup, QUP_OPERATIONAL);
		if (bus || qerr) {
			st.last_bus = bus;
			st.last_qerr = qerr;
			touch_wr(st.qup, QUP_ERROR_FLAGS, qerr);
			touch_wr(st.qup, QUP_I2C_STATUS, bus);
			err = (bus & (1u << 3)) ? ENXIO : EIO;
			break;
		}
		if (op & (QUP_OUT_SVC | QUP_IN_SVC)) {
			touch_wr(st.qup, QUP_OPERATIONAL, op & (QUP_OUT_SVC | QUP_IN_SVC));
		}
		if (rd) {
			while (pos < len && (touch_rd(st.qup, QUP_OPERATIONAL) & QUP_IN_NOT_EMPTY)) {
				uint32_t v = touch_rd(st.qup, QUP_IN_FIFO);
				buf[pos++] = (uint8_t)v;
				if (pos < len) {
					buf[pos++] = (uint8_t)(v >> 16);
				}
			}
			if (pos == len) {
				err = 0;
				break;
			}
		} else if (op & QUP_MX_OUTPUT_DONE) {
			err = 0;
			break;
		}
		delay(1);
	}
	for (unsigned t = 0; t < 1000 && (touch_rd(st.qup, QUP_I2C_STATUS) & QUP_BUS_ACTIVE); t++) {
		delay(1);
	}
	touch_wr(st.qup, QUP_STATE, QUP_STATE_RESET);
	return err;
}

static int
qup_write(uint8_t addr, const uint8_t *data, unsigned len)
{
	return qup_msg(addr, false, (uint8_t *)data, len);
}

static int
qup_read_reg(uint8_t addr, uint8_t reg, uint8_t *buf, unsigned len)
{
	int err = qup_write(addr, &reg, 1);
	if (err) {
		return err;
	}
	return qup_msg(addr, true, buf, len);
}

static bool
touch_clocks(void)
{
	/* Enable BLSP1 AHB vote (bit 17) */
	touch_wr(st.gcc, TOUCH_GCC_AHB_VOTE, touch_rd(st.gcc, TOUCH_GCC_AHB_VOTE) | (1u << 17));
	/* QUP1 I2C RCG: XO parent, divider 1 (19.2 MHz), then commit */
	touch_wr(st.gcc, TOUCH_GCC_QUP_RCGR + 4, 0);
	touch_wr(st.gcc, TOUCH_GCC_QUP_RCGR, 1u);
	for (unsigned i = 0; i < 1000; i++) {
		if (!(touch_rd(st.gcc, TOUCH_GCC_QUP_RCGR) & 1u)) {
			break;
		}
		delay(1);
	}
	/* Enable QUP1 I2C branch clock (bit 0) */
	touch_wr(st.gcc, TOUCH_GCC_QUP_CBCR, touch_rd(st.gcc, TOUCH_GCC_QUP_CBCR) | 1u);
	for (unsigned i = 0; i < 1000; i++) {
		if (!(touch_rd(st.gcc, TOUCH_GCC_QUP_CBCR) & (1u << 31))) {
			printf("LAVENDER TOUCH: clocks on (ahb=%#x qup_cbcr=%#x rcgr=%#x)\n",
			    touch_rd(st.gcc, TOUCH_GCC_AHB_VOTE),
			    touch_rd(st.gcc, TOUCH_GCC_QUP_CBCR),
			    touch_rd(st.gcc, TOUCH_GCC_QUP_RCGR));
			return true;
		}
		delay(1);
	}
	printf("LAVENDER TOUCH: QUP1 clock timeout (cbcr=%#x)\n",
	    touch_rd(st.gcc, TOUCH_GCC_QUP_CBCR));
	return false;
}

static void
touch_gpio_init(void)
{
	/* SDA (GPIO 2) & SCL (GPIO 3): Function 3 (blsp_i2c1), 2mA, no pull */
	touch_wr(st.south, TOUCH_TLMM_GPIO2, (3u << 2) | (0u << 0) | (0u << 6));
	touch_wr(st.south, TOUCH_TLMM_GPIO3, (3u << 2) | (0u << 0) | (0u << 6));

	/* Reset (GPIO 66): Function 0 (GPIO), output HIGH (de-assert reset) */
	touch_wr(st.north, TOUCH_TLMM_GPIO66 + 4, 2u); /* out latch = 1 */
	touch_wr(st.north, TOUCH_TLMM_GPIO66, (1u << 9)); /* oe = 1 */

	/* IRQ (GPIO 67): Function 0 (GPIO), input with pull-up */
	touch_wr(st.north, TOUCH_TLMM_GPIO67, (3u << 0)); /* pull-up, oe = 0 */

	delay(10000); /* 10 ms for controller startup */
}

static bool
qup_init(void)
{
	touch_wr(st.qup, QUP_SW_RESET, 1);
	delay(10);
	if (!qup_state(QUP_STATE_RESET)) {
		printf("LAVENDER TOUCH: QUP does not reach RESET (state=%#x)\n",
		    touch_rd(st.qup, QUP_STATE));
		return false;
	}
	touch_wr(st.qup, QUP_CONFIG, QUP_MINI_CORE | QUP_N_VAL);
	touch_wr(st.qup, QUP_I2C_MASTER_GEN, 0); /* version-1 tags */
	uint32_t io = touch_rd(st.qup, QUP_IO_MODE);
	static const unsigned blk[4] = {4, 16, 32, 0};
	unsigned in_fifo = blk[(io >> 5) & 3u] * (2u << ((io >> 7) & 7u));
	printf("LAVENDER TOUCH: QUP hw=%#x io=%#x fifo=%u state=%#x gen=%#x\n",
	    touch_rd(st.qup, QUP_HW_VERSION), io, in_fifo,
	    touch_rd(st.qup, QUP_STATE), touch_rd(st.qup, QUP_I2C_MASTER_GEN));
	return in_fifo >= 32;
}

static void
touch_report(uint32_t mask, const int *x, const int *y)
{
	static unsigned logged;
	for (unsigned id = 0; id < TOUCH_MAX; ++id) {
		uint32_t bit = 1u << id;
		if (mask & bit) {
			if (!(st.active & bit) || x[id] != st.x[id] || y[id] != st.y[id]) {
				sweet_hid_touch((int)id, 1, x[id], y[id]);
				if (!(st.active & bit) && logged++ < 40) {
					printf("LAVENDER TOUCH: down %u at %d,%d\n", id, x[id], y[id]);
				}
			}
			st.x[id] = x[id];
			st.y[id] = y[id];
		} else if (st.active & bit) {
			sweet_hid_touch((int)id, 0, st.x[id], st.y[id]);
			if (logged++ < 40) {
				printf("LAVENDER TOUCH: up %u\n", id);
			}
		}
	}
	st.active = mask;
}

static void
nvt_dump(const char *what, const uint8_t *p)
{
	if (st.nvt_dumps >= 3) {
		return;
	}
	st.nvt_dumps++;
	printf("LAVENDER TOUCH: NVT %s %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
	    what, p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[10], p[11]);
}

/*
 * Novatek: the MCU is booted by writing command 0x69 to the hardware
 * command port 0x62; the firmware then listens at 0x01 and exposes the
 * reset-complete register at 0x60 (values 0xa0..0xaf).
 */
static bool
nvt_boot(void)
{
	uint8_t cmd[2] = {0x00, 0x69};
	int err = qup_write(TOUCH_ADDR_NVT_HW, cmd, sizeof(cmd));
	if (err) {
		printf("LAVENDER TOUCH: NVT boot cmd on 0x62 err=%d bus=%#x qerr=%#x\n",
		    err, st.last_bus, st.last_qerr);
		return false;
	}
	touch_sleep_ms(35);
	for (unsigned i = 0; i < 100; i++) {
		uint8_t s[6] = {0};
		err = qup_read_reg(TOUCH_ADDR_NVT, 0x60, s, sizeof(s));
		if (!err && s[0] >= 0xA0 && s[0] <= 0xAF) {
			printf("LAVENDER TOUCH: NVT reset state=%02x (try %u)\n", s[0], i);
			return true;
		}
		touch_sleep_ms(10);
	}
	printf("LAVENDER TOUCH: NVT reset state timeout (err=%d bus=%#x qerr=%#x)\n",
	    err, st.last_bus, st.last_qerr);
	return false;
}

static void
nvt_info(void)
{
	uint8_t page[3] = {0xFF, (TOUCH_NVT_EVENT_BUF >> 16) & 0xFF,
	    (TOUCH_NVT_EVENT_BUF >> 8) & 0xFF};
	uint8_t fi[16] = {0};
	int err = qup_write(TOUCH_ADDR_NVT, page, sizeof(page));
	if (!err) {
		err = qup_read_reg(TOUCH_ADDR_NVT, 0x78, fi, sizeof(fi));
	}
	if (err) {
		printf("LAVENDER TOUCH: NVT fw info err=%d\n", err);
		return;
	}
	unsigned x = ((unsigned)fi[4] << 8) | fi[5];
	unsigned y = ((unsigned)fi[6] << 8) | fi[7];
	printf("LAVENDER TOUCH: NVT fw=%02x~%02x x_max=%u y_max=%u buttons=%u\n",
	    fi[0], fi[1], x, y, fi[10]);
	if (fi[0] + fi[1] != 0xFF) {
		printf("LAVENDER TOUCH: NVT fw info checksum bad\n");
	}
	if (x > 100 && x <= 2000) {
		st.nvt_xmax = x;
	}
	if (y > 100 && y <= 3000) {
		st.nvt_ymax = y;
	}
}

static int
nvt_poll(void)
{
	uint8_t p[TOUCH_NOVATEK_LEN];
	int err = qup_read_reg(TOUCH_ADDR_NVT, 0x00, p, sizeof(p));
	if (err) {
		if (st.nvt_errs++ < 5) {
			printf("LAVENDER TOUCH: NVT report err=%d bus=%#x qerr=%#x\n",
			    err, st.last_bus, st.last_qerr);
		}
		return err;
	}
	nvt_dump("report", p);

	uint32_t mask = 0;
	int x[TOUCH_MAX] = {0}, y[TOUCH_MAX] = {0};

	for (unsigned i = 0; i < TOUCH_MAX; i++) {
		const uint8_t *r = &p[6 * i];
		unsigned id = r[0] >> 3;
		unsigned status = r[0] & 0x07;
		if (id == 0 || id > TOUCH_MAX) {
			continue;
		}
		if (status != 1 && status != 2) {
			continue;
		}
		unsigned px = ((unsigned)r[1] << 4) | (r[3] >> 4);
		unsigned py = ((unsigned)r[2] << 4) | (r[3] & 0x0F);
		if (px > st.nvt_xmax || py > st.nvt_ymax) {
			if (st.nvt_errs++ < 5) {
				printf("LAVENDER TOUCH: NVT drop id=%u %u,%u\n", id, px, py);
			}
			continue;
		}
		unsigned idx = id - 1;
		mask |= 1u << idx;
		x[idx] = (int)px;
		y[idx] = (int)py;
	}
	touch_report(mask, x, y);
	return 0;
}

/* Synaptics TCM: plain read at 0x20, no register pointer. */
static bool
tcm_detect(void)
{
	uint8_t hdr[4] = {0};
	int err = qup_msg(TOUCH_ADDR_SYNA, true, hdr, sizeof(hdr));
	if (err) {
		printf("LAVENDER TOUCH: TCM read err=%d bus=%#x qerr=%#x\n",
		    err, st.last_bus, st.last_qerr);
		return false;
	}
	printf("LAVENDER TOUCH: TCM hdr=%02x %02x %02x %02x\n",
	    hdr[0], hdr[1], hdr[2], hdr[3]);
	if (hdr[0] != 0xA5) {
		return false;
	}
	unsigned len = hdr[2] | ((unsigned)hdr[3] << 8);
	uint8_t pay[TOUCH_TCM_BUF] = {0};
	unsigned take = len + 1;
	if (take > sizeof(pay)) {
		take = sizeof(pay);
	}
	if (take != 0) {
		err = qup_msg(TOUCH_ADDR_SYNA, true, pay, take);
	}
	printf("LAVENDER TOUCH: TCM code=%02x len=%u first=%02x %02x %02x %02x err=%d\n",
	    hdr[1], len, pay[0], pay[1], pay[2], pay[3], err);
	return true;
}

static void
tcm_poll(void)
{
	uint8_t hdr[4] = {0};
	int err = qup_msg(TOUCH_ADDR_SYNA, true, hdr, sizeof(hdr));
	if (err) {
		return;
	}
	if (hdr[0] != 0xA5) {
		if (st.tcm_logs++ < 5) {
			printf("LAVENDER TOUCH: TCM bad marker %02x\n", hdr[0]);
		}
		return;
	}
	unsigned len = hdr[2] | ((unsigned)hdr[3] << 8);
	uint8_t pay[TOUCH_TCM_BUF] = {0};
	unsigned take = len + 1;
	if (take > sizeof(pay)) {
		take = sizeof(pay);
	}
	err = qup_msg(TOUCH_ADDR_SYNA, true, pay, take);
	if (st.tcm_logs < 20) {
		st.tcm_logs++;
		printf("LAVENDER TOUCH: TCM code=%02x len=%u %02x %02x %02x %02x %02x %02x err=%d\n",
		    hdr[1], len, pay[0], pay[1], pay[2], pay[3], pay[4], pay[5], err);
	}
}

static void
touch_thread(void *arg, wait_result_t result)
{
	(void)arg;
	(void)result;

	touch_sleep_ms(3000);

	printf("LAVENDER TOUCH: initializing SDM660 BLSP1 QUP1 touch driver...\n");

	st.gcc = ml_io_map(TOUCH_GCC_PA, 0x60000u);
	st.qup = ml_io_map(TOUCH_QUP_PA, 0x1000u);
	st.south = ml_io_map(TOUCH_TLMM_SOUTH_PA, 0x10000u);
	st.north = ml_io_map(TOUCH_TLMM_NORTH_PA, 0x50000u);

	if (!st.gcc || !st.qup || !st.south || !st.north) {
		printf("LAVENDER TOUCH: ml_io_map failed\n");
		thread_terminate(current_thread());
		return;
	}

	if (!touch_clocks()) {
		printf("LAVENDER TOUCH: clocks failed; stopping\n");
		thread_terminate(current_thread());
		return;
	}

	touch_gpio_init();
	printf("LAVENDER TOUCH: gpio cfg 2=%#x 3=%#x 66=%#x 67=%#x irq=%u\n",
	    touch_rd(st.south, TOUCH_TLMM_GPIO2),
	    touch_rd(st.south, TOUCH_TLMM_GPIO3),
	    touch_rd(st.north, TOUCH_TLMM_GPIO66),
	    touch_rd(st.north, TOUCH_TLMM_GPIO67),
	    touch_rd(st.north, TOUCH_TLMM_GPIO67 + 4) & 1u);

	if (!qup_init()) {
		printf("LAVENDER TOUCH: QUP init failed; stopping\n");
		thread_terminate(current_thread());
		return;
	}

	st.nvt_xmax = 1080;
	st.nvt_ymax = 2340;
	printf("LAVENDER TOUCH: IRQ GPIO67 level=%u\n",
	    touch_rd(st.north, TOUCH_TLMM_GPIO67 + 4) & 1u);

	if (nvt_boot()) {
		st.kind = TOUCH_KIND_NOVATEK;
		nvt_info();
	} else if (tcm_detect()) {
		st.kind = TOUCH_KIND_SYNA;
	} else {
		st.kind = TOUCH_KIND_NONE;
		printf("LAVENDER TOUCH: no controller answered\n");
		thread_terminate(current_thread());
		return;
	}

	printf("LAVENDER TOUCH: %s polling loop active (120 Hz)\n",
	    st.kind == TOUCH_KIND_NOVATEK ? "Novatek" : "Synaptics TCM");

	while (1) {
		if (st.kind == TOUCH_KIND_NOVATEK) {
			nvt_poll();
		} else {
			tcm_poll();
		}
		touch_sleep_ms(8); /* ~120 Hz */
	}
}

void
sweet_touch_init(void)
{
	thread_t thread;
	if (st.started) {
		return;
	}
	st.started = true;
	printf("LAVENDER TOUCH: starting touch thread...\n");
	if (kernel_thread_start(touch_thread, NULL, &thread) != KERN_SUCCESS) {
		printf("LAVENDER TOUCH: failed to start touch thread\n");
	}
}

#endif /* ARM64_BOARD_CONFIG_SWEET || ARM64_BOARD_CONFIG_LAVENDER || ARM64_BOARD_CONFIG_WHYRED */
