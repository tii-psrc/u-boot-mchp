// SPDX-License-Identifier: GPL-2.0
/*
 * cmd/imu.c - SCAI NAVC IMU debug commands.
 *
 * The NAVC carries two CEVA/Bosch BNO085 nine-axis sensor hubs, nominal and
 * redundant. They are not on an MSS bus: each sits behind a custom FPGA SPI
 * master in the FIC3 fabric APB window, which also drives the BNO085 sideband
 * pins (NRST, BOOTN, PS0/WAKE, H_CSN) and reads their raw state back.
 *
 * The BNO085 is a sensor hub rather than a register file. Talking to it means
 * SHTP framing (4-byte header: length, channel, sequence) carrying SH-2 report
 * IDs. See docs/bno085-imu/README.md in the scai-bsp workspace, the BNO08X
 * datasheet (CEVA 1000-3927) and the SH-2 Reference Manual (1000-3625).
 *
 * The bring-up sequence here was verified by hand at the u-boot prompt on
 * 2026-08-19 and produced a valid SHTP advertisement header from the nominal
 * part. Two things are load-bearing and easy to get wrong:
 *
 *   - CTRL1.USE_GPIO must be set or the sideband pin drive bits do nothing.
 *   - NRST must be pulsed low with PS0/BOOTN already high, because the BNO085
 *     samples the protocol-select pins on the rising edge of reset.
 */

#include <common.h>
#include <command.h>
#include <console.h>
#include <errno.h>
#include <time.h>
#include <vsprintf.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/string.h>

/* ------------------------------------------------------------------ */
/* Fabric register map                                                 */
/* ------------------------------------------------------------------ */

#define APB_BASE_ADDRESS	0x40000000UL

/* GPIO banks are 16 bytes apart; WDATA at +0x00, RDATA at +0x04. */
#define GPIO_BANK_BASE(n)	(APB_BASE_ADDRESS + 0x0100UL + ((n) * 0x10UL))
#define GPIO_WDATA		0x00
#define GPIO_RDATA		0x04

/* Power switches live in bank 1. Bit 4 of that bank is the boot flash chip
 * select (GPIO1_ENA_SS1 in drivers/spi/scai_fpgaqspi.c), so every write here
 * must be read-modify-write or the board loses the flash it booted from.
 */
#define IMU_PWR_GPIO_BANK	1
#define H18_E_PWR_IMU_N		11
#define F22_E_PWR_IMU_R		10

#define IMUS_BASE_ADDRESS	(APB_BASE_ADDRESS + 0x0700UL)
#define IMU_NOM_BASE_ADDRESS	(IMUS_BASE_ADDRESS + 0x0000UL)
#define IMU_RED_BASE_ADDRESS	(IMUS_BASE_ADDRESS + 0x0040UL)

/* Write and read registers are overlaid at the same offsets. */
enum imu_regs {
	I_WR_DATA	= 0,
	I_WR_CTRL1	= 4,
	I_WR_CTRL2	= 8,
	I_WR_CTRL3	= 12,
	I_RD_DATA	= 0,
	I_RD_ST1	= 4,
	I_RD_ST2	= 8,
};

enum imu_ctrl {
	I_CTRL_ENA_MASK		= 0x00000001,
	I_CTRL_START_OP_MASK	= 0x00000002,
	I_CTRL_DIV_MASK		= 0x000003FC,
	I_CTRL_nINT_MASK	= 0x00000400,
	I_CTRL_PS0_MASK		= 0x00000800,
	I_CTRL_nBOOT_MASK	= 0x00001000,
	I_CTRL_nRST_MASK	= 0x00002000,
	I_CTRL_USE_GPIO_MASK	= 0x00004000,
	I_CTRL_CE_MASK		= 0x00008000,
	I_CTRL_G_DOUT_MASK	= 0x00FF0000,
	I_CTRL_G_ENA_MASK	= 0xFF000000,
	I_CTRL2_CLK_LEVEL_MASK	= 0x00008000,
	I_CTRL3_ENA_IRQ_MASK	= 0x00000001,
};

#define I_CTRL_S_DIV		2

/* STATUS1 bits 6..13 are a raw readback of the physical pins, which makes
 * most wiring questions a register read rather than a scope session.
 */
enum imu_sts1 {
	I_STS1_I_nRST	= 0x00002000,
	I_STS1_I_PS0	= 0x00001000,
	I_STS1_I_nBOOT	= 0x00000800,
	I_STS1_I_nINT	= 0x00000400,
	I_STS1_I_CE	= 0x00000200,
	I_STS1_I_CLK	= 0x00000100,
	I_STS1_I_MISO	= 0x00000080,
	I_STS1_I_MOSI	= 0x00000040,
	I_STS1_OVERUN	= 0x00000020,
	I_STS1_FT_FULL	= 0x00000010,
	I_STS1_FT_EMPTY	= 0x00000008,
	I_STS1_FR_FULL	= 0x00000004,
	I_STS1_FR_EMPTY	= 0x00000002,
	I_STS1_IDLE	= 0x00000001,	/* unreliable, see imu_wait_idle() */
};

enum imu_sts2 {
	I_STS2_IDLE	= 0x80000000,
	I_STS2_FT_WRCNT	= 0x3F000000,
	I_STS2_FT_RDCNT	= 0x00FC0000,
	I_STS2_FT_EMPTY	= 0x00020000,
	I_STS2_FT_FULL	= 0x00010000,
	I_STS2_FR_WRCNT	= 0x00003F00,
	I_STS2_FR_RDCNT	= 0x000000FC,
	I_STS2_FR_EMPTY	= 0x00000002,
	I_STS2_FR_FULL	= 0x00000001,
};

/* ------------------------------------------------------------------ */
/* SHTP / SH-2 protocol                                                */
/* ------------------------------------------------------------------ */

#define SHTP_HDR_LEN		4
#define SHTP_CHAN_COMMAND	0
#define SHTP_CHAN_EXECUTABLE	1
#define SHTP_CHAN_CONTROL	2
#define SHTP_CHAN_INPUT		3
#define SHTP_CHAN_WAKE_INPUT	4
#define SHTP_CHAN_GYRO_RV	5
#define SHTP_CHAN_COUNT		6

/* A length field of 0xFFFF is reserved: the datasheet notes a failed
 * peripheral produces it far too easily to be a legal cargo size.
 */
#define SHTP_LEN_BOGUS		0xFFFF
#define SHTP_LEN_CONTINUATION	0x8000

#define SH2_PRODUCT_ID_REQ	0xF9
#define SH2_PRODUCT_ID_RESP	0xF8
#define SH2_GET_FEATURE_REQ	0xFE
#define SH2_GET_FEATURE_RESP	0xFC
#define SH2_SET_FEATURE_CMD	0xFD
#define SH2_BASE_TIMESTAMP	0xFB
#define SH2_TIMESTAMP_REBASE	0xFA

#define SH2_RPT_ACCEL		0x01
#define SH2_RPT_GYRO		0x02
#define SH2_RPT_MAG		0x03
#define SH2_RPT_LINEAR_ACCEL	0x04
#define SH2_RPT_ROTATION	0x05
#define SH2_RPT_GRAVITY		0x06

#define SH2_SET_FEATURE_LEN	17

/* Longest cargo we keep. The advertisement is 284 bytes; sensor packets at
 * the rates this command is useful for are far smaller. Anything longer is
 * clocked through and discarded rather than truncating the SPI transaction,
 * which would desynchronise the part.
 */
#define IMU_BUF_LEN		320

#define IMU_XFER_TIMEOUT_MS	20
#define IMU_INT_TIMEOUT_MS	500
#define IMU_RESET_SETTLE_MS	5
#define IMU_PWR_SETTLE_MS	20

/* ------------------------------------------------------------------ */

enum scai_imu_id {
	SCAI_IMU_NOM = 0,
	SCAI_IMU_RED,
	SCAI_IMU_COUNT,
};

struct scai_imu {
	const char	*name;
	phys_addr_t	phys;
	uint		pwr_bit;
	void __iomem	*regs;
	u32		ctrl1;		/* software copy: CTRL1 is write-only */
	u8		seq[SHTP_CHAN_COUNT];
	bool		ready;
};

static struct scai_imu imu[SCAI_IMU_COUNT] = {
	[SCAI_IMU_NOM] = {
		.name	 = "nominal",
		.phys	 = IMU_NOM_BASE_ADDRESS,
		.pwr_bit = H18_E_PWR_IMU_N,
	},
	[SCAI_IMU_RED] = {
		.name	 = "redundant",
		.phys	 = IMU_RED_BASE_ADDRESS,
		.pwr_bit = F22_E_PWR_IMU_R,
	},
};

/*
 * SPI clock divider. DIV=40 is confirmed working; the exact resulting rate is
 * unknown because the divider semantics and fabric reference clock are not
 * documented. DIV=20, as the original code used, may exceed the BNO085's
 * 3 MHz maximum if the divider is a plain /DIV off a 100 MHz reference.
 */
static uint imu_div = 40;

/*
 * TX FIFO byte packing. drivers/spi/scai_fpgaqspi.c puts the transmit byte in
 * the MSB and receives in the LSB; the receive half is confirmed for this IP,
 * the transmit half is not, because only 0x00 has been sent so far. "imu id"
 * probes both and latches whichever answers.
 */
static uint imu_tx_shift = 24;

static u8 imu_buf[IMU_BUF_LEN];

/* ------------------------------------------------------------------ */
/* Low level                                                           */
/* ------------------------------------------------------------------ */

static void __iomem *imu_gpio_bank(uint bank)
{
	return phys_to_virt(GPIO_BANK_BASE(bank));
}

static void imu_power(struct scai_imu *p, bool on)
{
	void __iomem *gpio = imu_gpio_bank(IMU_PWR_GPIO_BANK);
	u32 val = readl(gpio + GPIO_RDATA);

	if (on)
		val |= BIT(p->pwr_bit);
	else
		val &= ~BIT(p->pwr_bit);

	writel(val, gpio + GPIO_WDATA);
}

/* CTRL1 with the sideband pins driven to their idle-safe state and reset
 * asserted. USE_GPIO is what makes the pin bits take effect at all.
 */
static u32 imu_ctrl1_idle(void)
{
	return I_CTRL_ENA_MASK |
	       ((imu_div << I_CTRL_S_DIV) & I_CTRL_DIV_MASK) |
	       I_CTRL_nINT_MASK | I_CTRL_PS0_MASK | I_CTRL_nBOOT_MASK |
	       I_CTRL_USE_GPIO_MASK | I_CTRL_CE_MASK;
}

static void imu_write_ctrl1(struct scai_imu *p)
{
	writel(p->ctrl1, p->regs + I_WR_CTRL1);
}

/* CE clear drives H_CSN low, i.e. chip select asserted. */
static void imu_cs(struct scai_imu *p, bool assert)
{
	if (assert)
		p->ctrl1 &= ~I_CTRL_CE_MASK;
	else
		p->ctrl1 |= I_CTRL_CE_MASK;

	imu_write_ctrl1(p);
}

static bool imu_int_asserted(struct scai_imu *p)
{
	return !(readl(p->regs + I_RD_ST1) & I_STS1_I_nINT);
}

static int imu_wait_int(struct scai_imu *p, ulong timeout_ms)
{
	ulong start = get_timer(0);

	while (!imu_int_asserted(p)) {
		if (get_timer(start) > timeout_ms)
			return -ETIMEDOUT;
		if (ctrlc())
			return -EINTR;
	}

	return 0;
}

/*
 * One byte, full duplex. Writing DATA starts the transfer immediately -
 * START_OP is not involved and the TX FIFO never fills. Reading DATA only
 * pops the receive FIFO, so we must always pop even when discarding, or the
 * FIFO fills and the next transfer stalls.
 *
 * Going a byte at a time keeps the 64-entry FIFOs irrelevant, which is what
 * lets a multi-hundred-byte cargo be read inside a single chip select.
 */
static int imu_xfer(struct scai_imu *p, u8 out, u8 *in)
{
	ulong start;
	u8 val;

	writel((u32)out << imu_tx_shift, p->regs + I_WR_DATA);

	start = get_timer(0);
	while (readl(p->regs + I_RD_ST2) & I_STS2_FR_EMPTY) {
		if (get_timer(start) > IMU_XFER_TIMEOUT_MS) {
			printf("imu: %s: byte transfer timed out\n", p->name);
			return -ETIMEDOUT;
		}
	}

	val = readl(p->regs + I_RD_DATA) & 0xff;
	if (in)
		*in = val;

	return 0;
}

/* ------------------------------------------------------------------ */
/* SHTP                                                                */
/* ------------------------------------------------------------------ */

/*
 * Read one cargo. Header and body must come out inside a single chip select:
 * dropping CS restarts the packet, since the part treats the next assertion
 * as a fresh (possibly partial) read.
 *
 * Returns the on-the-wire length, which may exceed maxlen - bytes past the
 * buffer are clocked out and discarded so the part stays in sync.
 */
static int shtp_read(struct scai_imu *p, u8 *buf, int maxlen)
{
	u8 hdr[SHTP_HDR_LEN];
	int i, rc, len;
	u16 raw;

	imu_cs(p, true);

	for (i = 0; i < SHTP_HDR_LEN; i++) {
		rc = imu_xfer(p, 0, &hdr[i]);
		if (rc)
			goto out;
	}

	raw = hdr[0] | (hdr[1] << 8);
	if (raw == SHTP_LEN_BOGUS) {
		printf("imu: %s: length 0xffff - bus fault, not a long packet\n",
		       p->name);
		rc = -EIO;
		goto out;
	}

	len = raw & ~SHTP_LEN_CONTINUATION;
	if (len < SHTP_HDR_LEN) {
		printf("imu: %s: short cargo %d (hdr %02x %02x %02x %02x)\n",
		       p->name, len, hdr[0], hdr[1], hdr[2], hdr[3]);
		rc = -EIO;
		goto out;
	}

	for (i = 0; i < SHTP_HDR_LEN && i < maxlen; i++)
		buf[i] = hdr[i];

	for (i = SHTP_HDR_LEN; i < len; i++) {
		u8 b;

		rc = imu_xfer(p, 0, &b);
		if (rc)
			goto out;
		if (i < maxlen)
			buf[i] = b;
	}

	rc = len;
out:
	imu_cs(p, false);
	return rc;
}

static int shtp_write(struct scai_imu *p, u8 chan, const u8 *data, int len)
{
	u8 hdr[SHTP_HDR_LEN];
	int total = len + SHTP_HDR_LEN;
	int i, rc = 0;

	hdr[0] = total & 0xff;
	hdr[1] = (total >> 8) & 0xff;
	hdr[2] = chan;
	hdr[3] = p->seq[chan]++;

	imu_cs(p, true);

	for (i = 0; i < SHTP_HDR_LEN && !rc; i++)
		rc = imu_xfer(p, hdr[i], NULL);
	for (i = 0; i < len && !rc; i++)
		rc = imu_xfer(p, data[i], NULL);

	imu_cs(p, false);
	return rc;
}

/* Read and throw away whatever is queued, so a later request is not stuck
 * behind the 284-byte startup advertisement.
 */
static int imu_drain(struct scai_imu *p, int max_packets)
{
	int n = 0;

	while (n < max_packets && imu_int_asserted(p)) {
		if (shtp_read(p, imu_buf, sizeof(imu_buf)) < 0)
			break;
		n++;
	}

	return n;
}

/* ------------------------------------------------------------------ */
/* SH-2                                                                */
/* ------------------------------------------------------------------ */

static int sh2_report_len(u8 rid)
{
	switch (rid) {
	case SH2_BASE_TIMESTAMP:
	case SH2_TIMESTAMP_REBASE:
		return 5;
	case SH2_RPT_ACCEL:
	case SH2_RPT_GYRO:
	case SH2_RPT_MAG:
	case SH2_RPT_LINEAR_ACCEL:
	case SH2_RPT_GRAVITY:
		return 10;
	case SH2_RPT_ROTATION:
		return 14;
	case 0x07:		/* gyroscope uncalibrated */
	case 0x0F:		/* magnetic field uncalibrated */
		return 16;
	case 0x08:		/* game rotation vector */
		return 12;
	default:
		return -1;
	}
}

/*
 * Print a Q-point fixed value with four decimals. Reports are s16, so
 * raw * 10000 stays inside s32 and no 64-bit division is needed.
 */
static void imu_print_q(s16 raw, int q)
{
	s32 scaled = ((s32)raw * 10000) / (1 << q);
	s32 ip = scaled / 10000;
	s32 fp = scaled % 10000;

	if (fp < 0)
		fp = -fp;

	if (scaled < 0 && ip == 0)
		printf("-0.%04d", fp);
	else
		printf("%d.%04d", ip, fp);
}

static void sh2_print_vec(const char *name, const u8 *r, int q,
			  const char *unit)
{
	s16 x = (s16)(r[4] | (r[5] << 8));
	s16 y = (s16)(r[6] | (r[7] << 8));
	s16 z = (s16)(r[8] | (r[9] << 8));

	printf("  %-8s x=", name);
	imu_print_q(x, q);
	printf(" y=");
	imu_print_q(y, q);
	printf(" z=");
	imu_print_q(z, q);
	printf(" %-6s seq=%3u acc=%u\n", unit, r[1], r[2] & 3);
}

static void sh2_dump_reports(const u8 *buf, int len)
{
	int off = SHTP_HDR_LEN;

	while (off < len) {
		u8 rid = buf[off];
		int rlen = sh2_report_len(rid);

		if (rlen < 0) {
			printf("  report 0x%02x: unknown, stopping at offset %d\n",
			       rid, off);
			return;
		}
		if (off + rlen > len) {
			printf("  report 0x%02x: truncated\n", rid);
			return;
		}

		switch (rid) {
		case SH2_BASE_TIMESTAMP:
		case SH2_TIMESTAMP_REBASE: {
			s32 d = buf[off + 1] | (buf[off + 2] << 8) |
				(buf[off + 3] << 16) | (buf[off + 4] << 24);

			/* 100 us ticks, relative to the H_INTN assert */
			printf("  %-8s %d.%01d ms\n",
			       rid == SH2_BASE_TIMESTAMP ? "timebase" : "rebase",
			       d / 10, d < 0 ? -(d % 10) : d % 10);
			break;
		}
		case SH2_RPT_ACCEL:
			sh2_print_vec("accel", buf + off, 8, "m/s^2");
			break;
		case SH2_RPT_LINEAR_ACCEL:
			sh2_print_vec("lin.acc", buf + off, 8, "m/s^2");
			break;
		case SH2_RPT_GRAVITY:
			sh2_print_vec("gravity", buf + off, 8, "m/s^2");
			break;
		case SH2_RPT_GYRO:
			sh2_print_vec("gyro", buf + off, 9, "rad/s");
			break;
		case SH2_RPT_MAG:
			sh2_print_vec("mag", buf + off, 4, "uT");
			break;
		default:
			printf("  report 0x%02x (%d bytes)\n", rid, rlen);
			break;
		}

		off += rlen;
	}
}

static int sh2_set_feature(struct scai_imu *p, u8 rid, u32 interval_us)
{
	u8 cmd[SH2_SET_FEATURE_LEN];

	memset(cmd, 0, sizeof(cmd));
	cmd[0] = SH2_SET_FEATURE_CMD;
	cmd[1] = rid;
	/* 2 flags, 3..4 change sensitivity, 9..12 batch, 13..16 config: all 0 */
	cmd[5] = interval_us & 0xff;
	cmd[6] = (interval_us >> 8) & 0xff;
	cmd[7] = (interval_us >> 16) & 0xff;
	cmd[8] = (interval_us >> 24) & 0xff;

	return shtp_write(p, SHTP_CHAN_CONTROL, cmd, sizeof(cmd));
}

/* Send a Product ID request and wait for the 0xF8 answer, skipping whatever
 * else the part decides to send first.
 */
static int sh2_product_id(struct scai_imu *p, bool verbose)
{
	u8 req[2] = { SH2_PRODUCT_ID_REQ, 0 };
	int rc, tries;

	rc = shtp_write(p, SHTP_CHAN_CONTROL, req, sizeof(req));
	if (rc)
		return rc;

	for (tries = 0; tries < 16; tries++) {
		rc = imu_wait_int(p, 200);
		if (rc)
			return rc;

		rc = shtp_read(p, imu_buf, sizeof(imu_buf));
		if (rc < 0)
			return rc;
		if (rc < SHTP_HDR_LEN + 16)
			continue;
		if (imu_buf[2] != SHTP_CHAN_CONTROL)
			continue;
		if (imu_buf[SHTP_HDR_LEN] != SH2_PRODUCT_ID_RESP)
			continue;

		if (verbose) {
			const u8 *r = imu_buf + SHTP_HDR_LEN;
			u32 part = r[4] | (r[5] << 8) | (r[6] << 16) |
				   (r[7] << 24);
			u32 build = r[8] | (r[9] << 8) | (r[10] << 16) |
				    (r[11] << 24);
			u16 patch = r[12] | (r[13] << 8);

			printf("%s: SH-2 firmware %u.%u.%u, part %u, build %u\n",
			       p->name, r[2], r[3], patch, part, build);
			printf("%s: reset cause %u\n", p->name, r[1]);
		}

		return 0;
	}

	return -ETIMEDOUT;
}

/* ------------------------------------------------------------------ */

static struct scai_imu *imu_get(const char *arg)
{
	ulong id = dectoul(arg, NULL);

	if (id >= SCAI_IMU_COUNT) {
		printf("imu: bad id %lu, expected 0 (nominal) or 1 (redundant)\n",
		       id);
		return NULL;
	}

	return &imu[id];
}

static struct scai_imu *imu_get_ready(const char *arg)
{
	struct scai_imu *p = imu_get(arg);

	if (!p)
		return NULL;
	if (!p->ready) {
		printf("imu: %s not initialised, run 'imu init' first\n",
		       p->name);
		return NULL;
	}

	return p;
}

static int do_imu_list(struct cmd_tbl *cmdtp, int flag, int argc,
		       char *const argv[])
{
	int i;

	printf("id  name       base        state\n");
	for (i = 0; i < SCAI_IMU_COUNT; i++)
		printf("%2d  %-9s  %#010llx  %s\n", i, imu[i].name,
		       (unsigned long long)imu[i].phys,
		       imu[i].ready ? "ready" : "uninitialised");

	printf("\nSPI divider %u, TX byte shift %u (%s)\n",
	       imu_div, imu_tx_shift, imu_tx_shift ? "MSB" : "LSB");

	return CMD_RET_SUCCESS;
}

static int do_imu_init(struct cmd_tbl *cmdtp, int flag, int argc,
		       char *const argv[])
{
	struct scai_imu *p;
	int drained;

	if (argc != 2)
		return CMD_RET_USAGE;

	p = imu_get(argv[1]);
	if (!p)
		return CMD_RET_FAILURE;

	p->regs = phys_to_virt(p->phys);
	p->ready = false;
	memset(p->seq, 0, sizeof(p->seq));

	imu_power(p, true);
	mdelay(IMU_PWR_SETTLE_MS);

	/* Protocol select must be stable before reset is released: PS1 is
	 * strapped high on the board and PS0 is driven high here, so the part
	 * latches SPI mode. BOOTN high keeps it out of the bootloader.
	 */
	writel(I_CTRL2_CLK_LEVEL_MASK, p->regs + I_WR_CTRL2);
	writel(0, p->regs + I_WR_CTRL3);

	p->ctrl1 = imu_ctrl1_idle();
	imu_write_ctrl1(p);
	mdelay(IMU_RESET_SETTLE_MS);

	p->ctrl1 |= I_CTRL_nRST_MASK;
	imu_write_ctrl1(p);

	/* Datasheet: 90 ms internal init, then 4 ms configuration, before
	 * H_INTN is asserted for the first time.
	 */
	if (imu_wait_int(p, IMU_INT_TIMEOUT_MS)) {
		printf("imu: %s: H_INTN never asserted - check power, the "
		       "clock source, and STATUS1 pin readback\n", p->name);
		return CMD_RET_FAILURE;
	}

	p->ready = true;

	drained = imu_drain(p, 8);
	printf("%s: up, discarded %d startup packet%s\n",
	       p->name, drained, drained == 1 ? "" : "s");

	return CMD_RET_SUCCESS;
}

static int do_imu_off(struct cmd_tbl *cmdtp, int flag, int argc,
		      char *const argv[])
{
	struct scai_imu *p;

	if (argc != 2)
		return CMD_RET_USAGE;

	p = imu_get(argv[1]);
	if (!p)
		return CMD_RET_FAILURE;

	if (p->regs) {
		p->ctrl1 = 0;
		imu_write_ctrl1(p);
	}
	imu_power(p, false);
	p->ready = false;

	printf("%s: powered down\n", p->name);

	return CMD_RET_SUCCESS;
}

static int do_imu_id(struct cmd_tbl *cmdtp, int flag, int argc,
		     char *const argv[])
{
	struct scai_imu *p;

	if (argc != 2)
		return CMD_RET_USAGE;

	p = imu_get_ready(argv[1]);
	if (!p)
		return CMD_RET_FAILURE;

	if (!sh2_product_id(p, true))
		return CMD_RET_SUCCESS;

	/* Nothing came back. The one remaining unknown in this IP is which end
	 * of the word the TX FIFO takes its byte from, so try the other.
	 */
	printf("%s: no response with TX shift %u, trying %u\n",
	       p->name, imu_tx_shift, imu_tx_shift ? 0 : 24);

	imu_tx_shift = imu_tx_shift ? 0 : 24;
	imu_drain(p, 8);

	if (!sh2_product_id(p, true)) {
		printf("%s: TX byte packing is %s - keeping shift %u\n",
		       p->name, imu_tx_shift ? "MSB" : "LSB", imu_tx_shift);
		return CMD_RET_SUCCESS;
	}

	imu_tx_shift = imu_tx_shift ? 0 : 24;
	printf("%s: no product ID response either way\n", p->name);

	return CMD_RET_FAILURE;
}

static int do_imu_enable(struct cmd_tbl *cmdtp, int flag, int argc,
			 char *const argv[])
{
	struct scai_imu *p;
	ulong hz = 1;
	u32 interval;
	int rc;

	if (argc < 2 || argc > 3)
		return CMD_RET_USAGE;

	p = imu_get_ready(argv[1]);
	if (!p)
		return CMD_RET_FAILURE;

	if (argc == 3)
		hz = dectoul(argv[2], NULL);
	if (!hz || hz > 500) {
		printf("imu: rate must be 1..500 Hz\n");
		return CMD_RET_USAGE;
	}

	interval = 1000000UL / hz;

	rc = sh2_set_feature(p, SH2_RPT_ACCEL, interval);
	if (!rc)
		rc = sh2_set_feature(p, SH2_RPT_GYRO, interval);
	if (rc) {
		printf("%s: set feature failed (%d)\n", p->name, rc);
		return CMD_RET_FAILURE;
	}

	/* The hub negotiates: 0.9x <= configured <= 2.1x requested. */
	printf("%s: accel + gyro requested at %lu Hz (interval %u us)\n",
	       p->name, hz, interval);

	return CMD_RET_SUCCESS;
}

static int do_imu_disable(struct cmd_tbl *cmdtp, int flag, int argc,
			  char *const argv[])
{
	struct scai_imu *p;

	if (argc != 2)
		return CMD_RET_USAGE;

	p = imu_get_ready(argv[1]);
	if (!p)
		return CMD_RET_FAILURE;

	sh2_set_feature(p, SH2_RPT_ACCEL, 0);
	sh2_set_feature(p, SH2_RPT_GYRO, 0);
	printf("%s: accel + gyro disabled\n", p->name);

	return CMD_RET_SUCCESS;
}

/* Enable an arbitrary report, for poking at anything the canned commands do
 * not cover: "imu feature 0 5 10" gives the rotation vector at 10 Hz.
 */
static int do_imu_feature(struct cmd_tbl *cmdtp, int flag, int argc,
			  char *const argv[])
{
	struct scai_imu *p;
	ulong rid, hz;
	int rc;

	if (argc != 4)
		return CMD_RET_USAGE;

	p = imu_get_ready(argv[1]);
	if (!p)
		return CMD_RET_FAILURE;

	rid = hextoul(argv[2], NULL);
	hz = dectoul(argv[3], NULL);
	if (rid > 0xff) {
		printf("imu: report id out of range\n");
		return CMD_RET_USAGE;
	}

	rc = sh2_set_feature(p, rid, hz ? 1000000UL / hz : 0);
	if (rc)
		return CMD_RET_FAILURE;

	printf("%s: report 0x%02lx at %lu Hz\n", p->name, rid, hz);

	return CMD_RET_SUCCESS;
}

static int do_imu_read(struct cmd_tbl *cmdtp, int flag, int argc,
		       char *const argv[])
{
	struct scai_imu *p;
	ulong count = 0;	/* 0 = until Ctrl-C */
	ulong got = 0;

	if (argc < 2 || argc > 3)
		return CMD_RET_USAGE;

	p = imu_get_ready(argv[1]);
	if (!p)
		return CMD_RET_FAILURE;

	if (argc == 3)
		count = dectoul(argv[2], NULL);

	printf("%s: reading (Ctrl-C to stop)\n", p->name);

	while (!count || got < count) {
		int len, rc;

		/* ctrlc() consumes the keypress, so imu_wait_int() has to
		 * report it rather than us re-testing after the fact.
		 */
		if (ctrlc()) {
			printf("interrupted\n");
			break;
		}

		rc = imu_wait_int(p, IMU_INT_TIMEOUT_MS);
		if (rc == -EINTR) {
			printf("interrupted\n");
			break;
		}
		if (rc) {
			/* A sensor at 1 Hz legitimately leaves us waiting. */
			continue;
		}

		len = shtp_read(p, imu_buf, sizeof(imu_buf));
		if (len < 0)
			return CMD_RET_FAILURE;

		printf("packet chan %u len %d seq %u\n",
		       imu_buf[2], len, imu_buf[3]);

		if (len > (int)sizeof(imu_buf)) {
			printf("  (truncated to %d bytes)\n",
			       (int)sizeof(imu_buf));
			len = sizeof(imu_buf);
		}

		if (imu_buf[2] == SHTP_CHAN_INPUT ||
		    imu_buf[2] == SHTP_CHAN_WAKE_INPUT ||
		    imu_buf[2] == SHTP_CHAN_GYRO_RV)
			sh2_dump_reports(imu_buf, len);

		got++;
	}

	return CMD_RET_SUCCESS;
}

static int do_imu_regs(struct cmd_tbl *cmdtp, int flag, int argc,
		       char *const argv[])
{
	struct scai_imu *p;
	u32 s1, s2;

	if (argc != 2)
		return CMD_RET_USAGE;

	p = imu_get(argv[1]);
	if (!p)
		return CMD_RET_FAILURE;

	if (!p->regs)
		p->regs = phys_to_virt(p->phys);

	s1 = readl(p->regs + I_RD_ST1);
	s2 = readl(p->regs + I_RD_ST2);

	printf("%s @ %#010llx  ctrl1 %#010x (software copy)\n",
	       p->name, (unsigned long long)p->phys, p->ctrl1);
	printf("status1 %#010x  nRST=%d PS0=%d nBOOT=%d nINT=%d CE=%d "
	       "CLK=%d MISO=%d MOSI=%d\n", s1,
	       !!(s1 & I_STS1_I_nRST), !!(s1 & I_STS1_I_PS0),
	       !!(s1 & I_STS1_I_nBOOT), !!(s1 & I_STS1_I_nINT),
	       !!(s1 & I_STS1_I_CE), !!(s1 & I_STS1_I_CLK),
	       !!(s1 & I_STS1_I_MISO), !!(s1 & I_STS1_I_MOSI));
	printf("            overrun=%d tx[full=%d empty=%d] rx[full=%d empty=%d]\n",
	       !!(s1 & I_STS1_OVERUN),
	       !!(s1 & I_STS1_FT_FULL), !!(s1 & I_STS1_FT_EMPTY),
	       !!(s1 & I_STS1_FR_FULL), !!(s1 & I_STS1_FR_EMPTY));
	printf("status2 %#010x  idle=%d tx[wr=%u rd=%u] rx[wr=%u rd=%u]\n", s2,
	       !!(s2 & I_STS2_IDLE),
	       (s2 & I_STS2_FT_WRCNT) >> 24, (s2 & I_STS2_FT_RDCNT) >> 18,
	       (s2 & I_STS2_FR_WRCNT) >> 8, (s2 & I_STS2_FR_RDCNT) >> 2);
	printf("H_INTN %s\n",
	       imu_int_asserted(p) ? "asserted (data waiting)" : "idle");

	return CMD_RET_SUCCESS;
}

static int do_imu_txmode(struct cmd_tbl *cmdtp, int flag, int argc,
			 char *const argv[])
{
	if (argc == 2) {
		ulong shift = dectoul(argv[1], NULL);

		if (shift != 0 && shift != 24) {
			printf("imu: TX shift must be 0 (LSB) or 24 (MSB)\n");
			return CMD_RET_USAGE;
		}
		imu_tx_shift = shift;
	} else if (argc != 1) {
		return CMD_RET_USAGE;
	}

	printf("TX byte shift %u (%s)\n", imu_tx_shift,
	       imu_tx_shift ? "MSB" : "LSB");

	return CMD_RET_SUCCESS;
}

static int do_imu_div(struct cmd_tbl *cmdtp, int flag, int argc,
		      char *const argv[])
{
	if (argc == 2) {
		ulong div = dectoul(argv[1], NULL);

		if (!div || div > 255) {
			printf("imu: divider must be 1..255\n");
			return CMD_RET_USAGE;
		}
		imu_div = div;
		printf("divider %u - takes effect on the next 'imu init'\n",
		       imu_div);
	} else if (argc == 1) {
		printf("divider %u\n", imu_div);
	} else {
		return CMD_RET_USAGE;
	}

	return CMD_RET_SUCCESS;
}

static char imu_help_text[] =
	"list                    - show both IMUs and current settings\n"
	"imu init <id>               - power up, reset, wait for H_INTN\n"
	"imu off <id>                - power down\n"
	"imu id <id>                 - SH-2 product ID (probes TX packing)\n"
	"imu enable <id> [hz]        - accel + gyro at hz (default 1)\n"
	"imu disable <id>            - stop accel + gyro\n"
	"imu feature <id> <rpt> <hz> - enable an arbitrary report id (hex)\n"
	"imu read <id> [n]           - decode n packets, or until Ctrl-C\n"
	"imu regs <id>               - decode STATUS1/STATUS2 and pin state\n"
	"imu txmode [0|24]           - TX FIFO byte shift, LSB or MSB\n"
	"imu div [n]                 - SPI clock divider\n"
	"\n"
	"id is 0 for nominal, 1 for redundant. Typical session:\n"
	"  imu init 0 ; imu id 0 ; imu enable 0 1 ; imu read 0\n";

U_BOOT_CMD_WITH_SUBCMDS(imu, "BNO085 IMU utils", imu_help_text,
	U_BOOT_SUBCMD_MKENT(list, 1, 1, do_imu_list),
	U_BOOT_SUBCMD_MKENT(init, 2, 0, do_imu_init),
	U_BOOT_SUBCMD_MKENT(off, 2, 0, do_imu_off),
	U_BOOT_SUBCMD_MKENT(id, 2, 0, do_imu_id),
	U_BOOT_SUBCMD_MKENT(enable, 3, 0, do_imu_enable),
	U_BOOT_SUBCMD_MKENT(disable, 2, 0, do_imu_disable),
	U_BOOT_SUBCMD_MKENT(feature, 4, 0, do_imu_feature),
	U_BOOT_SUBCMD_MKENT(read, 3, 0, do_imu_read),
	U_BOOT_SUBCMD_MKENT(regs, 2, 0, do_imu_regs),
	U_BOOT_SUBCMD_MKENT(txmode, 2, 0, do_imu_txmode),
	U_BOOT_SUBCMD_MKENT(div, 2, 0, do_imu_div));
