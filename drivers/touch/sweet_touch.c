/*
 * SDM660 (lavender) & SDM636 (whyred) BLSP1 QUP1 Touchscreen Driver.
 * Supports Novatek NT36672A (0x62) and Synaptics TCM (0x20).
 *
 * Hardware connections (from vendor device tree & schematics):
 *   I2C      BLSP1 QUP1 @ 0x0c175000 (FIFO mode, 400 kHz, Qualcomm QUP v2)
 *   Clocks   GCC_BLSP1_AHB_CLK (0x00100000 + 0x52004 bit 17)
 *            GCC_BLSP1_QUP1_I2C_APPS_CLK (0x00100000 + 0x19008 bit 0)
 *   Pins     TLMM SOUTH (0x03100000): GPIO 2 (SDA), GPIO 3 (SCL) func 3 (blsp_i2c1)
 *            TLMM NORTH (0x03900000): GPIO 66 (Reset, out high), GPIO 67 (IRQ, in pull-up)
 *   Power    1.8V VCC_I2C from PM660 L11 (already enabled by ABL display bring-up)
 *   Protocol Novatek NT36672A: read 65 bytes from register 0x01 on address 0x62
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

#define TOUCH_ADDR_NOVATEK   0x62u
#define TOUCH_ADDR_SYNA      0x20u
#define TOUCH_MAX            10u
#define TOUCH_NOVATEK_LEN    65u

#define TOUCH_KIND_NONE      0
#define TOUCH_KIND_NOVATEK   1
#define TOUCH_KIND_SYNA      2

/* QUP registers (Qualcomm QUP v2 in FIFO mode) */
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
	unsigned rx_max;
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
	/* Ensure QUP1 I2C RCG is configured to 19.2 MHz XO */
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
	touch_wr(st.north, TOUCH_TLMM_GPIO66 + 4, 2u); /* bit 1 = 1 */
	touch_wr(st.north, TOUCH_TLMM_GPIO66, (0u << 2) | (1u << 9)); /* oe = 1 */

	/* IRQ (GPIO 67): Function 0 (GPIO), input with pull-up */
	touch_wr(st.north, TOUCH_TLMM_GPIO67, (0u << 2) | (3u << 0)); /* pull-up, oe = 0 */

	delay(10000); /* 10 ms delay for controller startup */
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
	touch_wr(st.qup, QUP_I2C_MASTER_GEN, 0); /* v1 tags */
	uint32_t io = touch_rd(st.qup, QUP_IO_MODE);
	static const unsigned blk[4] = {4, 16, 32, 0};
	unsigned in_fifo = blk[(io >> 5) & 3u] * (2u << ((io >> 7) & 7u));
	st.rx_max = in_fifo / 2u;
	printf("LAVENDER TOUCH: QUP hw=%#x io=%#x fifo=%u rx_max=%u\n",
	    touch_rd(st.qup, QUP_HW_VERSION), io, in_fifo, st.rx_max);
	return st.rx_max >= 16;
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

static int
touch_poll_novatek(void)
{
	uint8_t p[TOUCH_NOVATEK_LEN + 1];
	int err = qup_read_reg(TOUCH_ADDR_NOVATEK, 0x01, p, TOUCH_NOVATEK_LEN);
	if (err) {
		return err;
	}
	uint32_t mask = 0;
	int x[TOUCH_MAX] = {0}, y[TOUCH_MAX] = {0};

	for (unsigned i = 0; i < TOUCH_MAX; i++) {
		unsigned pos = 1 + 6 * i;
		if (pos + 4 >= TOUCH_NOVATEK_LEN) {
			break;
		}
		uint8_t id = p[pos] >> 3;
		if (id == 0 || id > TOUCH_MAX) {
			continue;
		}
		uint8_t st_val = p[pos] & 0x07;
		if (st_val == 1 || st_val == 2) {
			uint32_t px = ((uint32_t)p[pos + 1] << 4) | (p[pos + 3] >> 4);
			uint32_t py = ((uint32_t)p[pos + 2] << 4) | (p[pos + 3] & 0x0F);
			if (px <= 1080 && py <= 2400) {
				unsigned idx = id - 1;
				mask |= 1u << idx;
				x[idx] = (int)px;
				y[idx] = (int)py;
			}
		}
	}
	touch_report(mask, x, y);
	return 0;
}

static int
touch_poll_synaptics(void)
{
	uint8_t p[32] = {0};
	int err = qup_read_reg(TOUCH_ADDR_SYNA, 0x00, p, 32);
	if (err) {
		return err;
	}
	uint32_t mask = 0;
	int x[TOUCH_MAX] = {0}, y[TOUCH_MAX] = {0};
	for (unsigned i = 0; i < 4; i++) {
		const uint8_t *obj = p + 8 * i;
		if (obj[0] == 1 || obj[0] == 2) {
			mask |= 1u << i;
			x[i] = obj[1] | (obj[2] << 8);
			y[i] = obj[3] | (obj[4] << 8);
		}
	}
	touch_report(mask, x, y);
	return 0;
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

	if (!qup_init()) {
		printf("LAVENDER TOUCH: QUP init failed; stopping\n");
		thread_terminate(current_thread());
		return;
	}

	/* Probe Novatek NT36672A @ 0x62 */
	uint8_t probe_buf[2] = {0};
	int err = qup_read_reg(TOUCH_ADDR_NOVATEK, 0x01, probe_buf, 2);
	if (err == 0) {
		printf("LAVENDER TOUCH: found Novatek NT36672A @ 0x62 (reg01=%02x %02x)\n",
		    probe_buf[0], probe_buf[1]);
		st.kind = TOUCH_KIND_NOVATEK;
	} else {
		printf("LAVENDER TOUCH: 0x62 probe err=%d, probing Synaptics @ 0x20...\n", err);
		err = qup_write(TOUCH_ADDR_SYNA, probe_buf, 1);
		if (err == 0) {
			printf("LAVENDER TOUCH: found Synaptics @ 0x20\n");
			st.kind = TOUCH_KIND_SYNA;
		} else {
			printf("LAVENDER TOUCH: defaulting to Novatek NT36672A\n");
			st.kind = TOUCH_KIND_NOVATEK;
		}
	}

	printf("LAVENDER TOUCH: polling loop active (120 Hz)\n");

	while (1) {
		/* GPIO 67 in NORTH tile (+0x43004): active low when touched */
		bool attn = (touch_rd(st.north, TOUCH_TLMM_GPIO67 + 4) & 1u) == 0;
		if (attn || st.active) {
			if (st.kind == TOUCH_KIND_NOVATEK) {
				touch_poll_novatek();
			} else {
				touch_poll_synaptics();
			}
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
