// SPDX-License-Identifier: GPL-2.0
/*
 * scai_fpgaqspi.c
 *
 * SCAI FPGA QSPI controller.
 *
 */

#include <common.h>
//#include <fdtdec.h> /* For dev_read_... helpers */
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/errno.h>
//#include <linux/iopoll.h>
#include <linux/bitops.h> /* For BIT() */
#include <dm.h>
#include <dm/device_compat.h>

#include <spi.h>
#include <spi-mem.h>

#define SCAI_NAND_FIFO_TIMEOUT 100
#define SCAI_NAND_FIFO_LENGTH  64

// Constants for packing a byte into a 32-bit word for the hardware.
// This is required if the hardware expects the byte in the MSB position.
#define SCAI_QSPI_FIFO_BYTE_SHIFT   24
#define SCAI_QSPI_FIFO_TX_BYTE_MASK 0xFF000000
#define SCAI_QSPI_FIFO_RX_BYTE_MASK 0x000000FF

/* --- SCAI QSPI Controller Register Offsets --- */
#define SCAI_QSPI_REG_DATA          0x00
#define SCAI_QSPI_REG_CTRL1         0x04
#define SCAI_QSPI_REG_CTRL2         0x08
#define SCAI_QSPI_REG_CTRL3         0x0C

#define SCAI_QSPI_REG_STATUS1       0x04
#define SCAI_QSPI_REG_STATUS2       0x08

/* --- SCAI QSPI Controller CTRL1 Register Bits --- */
#define CTRL1_CHIP_ENABLE           BIT(0)
#define CTRL1_NWP                   BIT(1)
#define CTRL1_RESET                 BIT(2)
#define CTRL1_DATA_MODE_WORD        BIT(3) /* 0 = Byte, 1 = Word */
#define CTRL1_LANE_WIDTH_X4         BIT(4) /* 0 = x1, 1 = x4 */
#define CTRL1_START                 BIT(9)
/* Count in bytes or words depending on CTRL1_DATA_MODE_WORD */
#define CTRL1_TX_COUNT(n)           (((n) & 0x7FF) << 10)
/* Count in bytes or words depending on CTRL1_DATA_MODE_WORD */
#define CTRL1_RX_COUNT(n)           (((n) & 0x7FF) << 21)

/* --- SCAI QSPI Controller Status2 Register Bits --- */
#define STATUS2_RX_FIFO_FULL         BIT(0)
#define STATUS2_RX_FIFO_EMPTY		 BIT(1)
#define STATUS2_RX_FIFO_RDCNT_MASK   0x7F
#define STATUS2_RX_FIFO_RDCNT_SHIFT  2
#define STATUS2_RX_FIFO_WrCnt_MASK   0x7F
#define STATUS2_RX_FIFO_WrCnt_SHIFT  9
#define STATUS2_TX_FIFO_FULL         BIT(16)
#define STATUS2_TX_FIFO_EMPTY        BIT(17)
#define STATUS2_TX_FIFO_RDCNT_MASK   0x7F
#define STATUS2_TX_FIFO_RDCNT_SHIFT  18
#define STATUS2_TX_FIFO_WRCNT_MASK   0x7F
#define STATUS2_TX_FIFO_WRCNT_SHIFT  25

/* --- SCAI QSPI Controller STATUS1 Register Bits --- */
#define STATUS1_IDLE                BIT(0)


/*
 * GPIO definitions based on scai_fpga_platform.h from HSS.
 */
#define GPIO_REG_WDATA_OFFSET   0x00
#define GPIO_REG_RDATA_OFFSET   0x04
#define GPIO1_ENA_SS1_MASK      BIT(4)
#define GPIO2_ENA_SS2_MASK      BIT(0)

#define TIMEOUT_MS             (1000 * 500)

#define MAX_DATA_CMD_LEN       0x440

/**
 * struct scai_fpgaqspi_priv - Private driver data structure
 */
struct scai_fpgaqspi_priv {
	/* Register base addresses */
	void __iomem	*regs;          /* QSPI register base */
	void __iomem	*gpio1_regs;    /* GPIO1 register base */
	void __iomem	*gpio2_regs;    /* GPIO2 register base */

	/* Software-maintained copies */
	u32		 ctrl1_sw_copy;  /* Cached CTRL1 register value */

	/* Transfer buffers */
	u8		*tx_buf;        /* TX buffer pointer */
	u8		*rx_buf;        /* RX buffer pointer */
	int		 tx_len;        /* TX length */
	int		 rx_len;        /* RX length */
};

static void scai_fpgaqspi_set_power(struct scai_fpgaqspi_priv *p,
		bool enable)
{
	u32 val1 = 0, val2 = 0;

	if (!p->gpio1_regs || !p->gpio2_regs) {
		printf("WARN: SCAI NAND: GPIO registers not mapped\n");
		return;
	}

	/* Read current GPIO state if needed
	 * val1 = readl(p->gpio1_regs + GPIO_REG_RDATA_OFFSET);
	 * val2 = readl(p->gpio2_regs + GPIO_REG_RDATA_OFFSET);
	 */
	if (enable) {
		val1 |= GPIO1_ENA_SS1_MASK;
		val2 |= GPIO2_ENA_SS2_MASK;
	} else {
		val1 &= ~GPIO1_ENA_SS1_MASK;
		val2 &= ~GPIO2_ENA_SS2_MASK;
	}

	writel(val1, p->gpio1_regs + GPIO_REG_WDATA_OFFSET);
	writel(val2, p->gpio2_regs + GPIO_REG_WDATA_OFFSET);
}
static int scai_fpgaqspi_wait_for_ready(struct spi_slave *slave)
{
	struct scai_fpgaqspi_priv *p = dev_get_priv(slave->dev->parent);
	unsigned long count = 0;
	u32 status;

	do {
		status = readl(p->regs + SCAI_QSPI_REG_STATUS1);
		if (status & STATUS1_IDLE)
			return 0;

		udelay(1);
		count++;
	} while (count < TIMEOUT_MS);

	printf("%s: timeout 0x%08X\n", __func__, status);
	return -ETIMEDOUT;
}

static void scai_fpgaqspi_set_operate_mode(struct scai_fpgaqspi_priv *p,
		bool word)
{
	u32 ctrl = p->ctrl1_sw_copy;
	ctrl &= ~(CTRL1_LANE_WIDTH_X4 | CTRL1_DATA_MODE_WORD);

	if (word)
		ctrl |= (CTRL1_LANE_WIDTH_X4 | CTRL1_DATA_MODE_WORD);

	p->ctrl1_sw_copy = ctrl;
}

static int scai_fpgaqspi_write_op(struct scai_fpgaqspi_priv *p, bool word)
{
	u32 data, status;
	int err = 0;

	if (word) {
		while (p->tx_len) {
			do {
				status = readl(p->regs + SCAI_QSPI_REG_STATUS2);
			} while (status & STATUS2_TX_FIFO_FULL);

			data = *(u32 *)p->tx_buf;
			p->tx_buf += 4;
			p->tx_len -= 4;
#if 0
			debug("%s-word: data(0x%08X)\n", __func__, data);
#endif
			writel(data, p->regs + SCAI_QSPI_REG_DATA);
		}
	} else {
		while (p->tx_len--) {
			do {
				status = readl(p->regs + SCAI_QSPI_REG_STATUS2);
			} while (status & STATUS2_TX_FIFO_FULL);

			data =  (u32)((*p->tx_buf <<
						SCAI_QSPI_FIFO_BYTE_SHIFT) & SCAI_QSPI_FIFO_TX_BYTE_MASK);
			data |= ~SCAI_QSPI_FIFO_TX_BYTE_MASK;

#if 0
			debug("%s-byte: data(0x%08X)\n", __func__, data);
#endif
			writel(data, p->regs + SCAI_QSPI_REG_DATA);
			p->tx_buf++;
		}
	}

	return err;
}

static int scai_fpgaqspi_read_op(struct scai_fpgaqspi_priv *p, bool word)
{
	u32 data, status;

	if (!p->rx_len)
		return -1;

	if (word) {

		while (p->rx_len) {
			do {
				status = readl(p->regs + SCAI_QSPI_REG_STATUS2);
			} while (status & STATUS2_RX_FIFO_EMPTY);

			data = readl(p->regs + SCAI_QSPI_REG_DATA);
#if 0
			debug("%s-word: data(0x%08X)\n", __func__, data);
#endif
			*(u32 *)p->rx_buf = data;
			p->rx_buf += 4;
			p->rx_len -= 4;
		}
	} else {
		while (p->rx_len--) {
			do {
				status = readl(p->regs + SCAI_QSPI_REG_STATUS2);
			} while (status & STATUS2_RX_FIFO_EMPTY);

			data = readl(p->regs + SCAI_QSPI_REG_DATA);
#if 0
			debug("%s-byte: data(0x%08X)\n", __func__, data);
#endif
			*p->rx_buf++ = (data & SCAI_QSPI_FIFO_RX_BYTE_MASK);
		}
	}

	return 0;
}

static int scai_fpgaqspi_start_transaction(struct scai_fpgaqspi_priv *p,
		u32 tx_len, u32 rx_len)
{
	u32 ctrl = p->ctrl1_sw_copy;
	ctrl &= ~(CTRL1_TX_COUNT(0x7FF) | CTRL1_RX_COUNT(0x7FF));
	ctrl |= CTRL1_TX_COUNT(tx_len) | CTRL1_RX_COUNT(rx_len);
	ctrl |= CTRL1_START | CTRL1_CHIP_ENABLE;

	writel(ctrl, p->regs + SCAI_QSPI_REG_CTRL1);
	p->ctrl1_sw_copy = ctrl;
	return 0;
}

static void scai_fpgaqspi_finish_transaction(struct scai_fpgaqspi_priv *p,
		bool keep_ce)
{
	u32 ctrl1 = p->ctrl1_sw_copy;

	ctrl1 &= ~(CTRL1_START | CTRL1_TX_COUNT(0x7FF) | CTRL1_RX_COUNT(0x7FF));

	if (!keep_ce) {
		ctrl1 &= ~CTRL1_CHIP_ENABLE;
	}

	writel(ctrl1, p->regs + SCAI_QSPI_REG_CTRL1);
	p->ctrl1_sw_copy = ctrl1;
}

static int __do_exec_word_op(struct spi_slave *slave,
				 const struct spi_mem_op *op)
{
	struct scai_fpgaqspi_priv *p = dev_get_priv(slave->dev->parent);
	u32 total_tx_words, total_rx_words;
	int err = 0;

	if (op->data.buswidth == 4 || op->data.buswidth == 2) {
		total_tx_words = 0;
		total_rx_words = (op->data.nbytes + 3) / 4;
		if (op->data.dir == SPI_MEM_DATA_OUT) {
			total_tx_words = (op->data.nbytes + 3) / 4;
			total_rx_words = 0;
		};

		scai_fpgaqspi_set_operate_mode(p, true);
		scai_fpgaqspi_start_transaction(p, total_tx_words, total_rx_words);

		if (op->data.dir == SPI_MEM_DATA_OUT) {
			p->tx_buf = (u8 *)op->data.buf.out;
			p->rx_buf = NULL;
			p->rx_len = 0;
			p->tx_len = op->data.nbytes;
			scai_fpgaqspi_write_op(p, true);
		} else {
			p->tx_buf = NULL;
			p->rx_buf = (u8 *)op->data.buf.in;
			p->rx_len = op->data.nbytes;
			p->tx_len = 0;
			scai_fpgaqspi_read_op(p, true);
		}
	}

	scai_fpgaqspi_finish_transaction(p, false);

	return err;
}

static int __do_exec_byte_op(struct spi_slave *slave,
				 const struct spi_mem_op *op)
{
	struct scai_fpgaqspi_priv *p = dev_get_priv(slave->dev->parent);
	u32 address = op->addr.val;
	u8 opcode = op->cmd.opcode;
	u8 opaddr[5];
	u32 total_tx_bytes, total_rx_bytes;
	int err = 0, i;

	total_tx_bytes = op->cmd.nbytes + op->addr.nbytes + op->dummy.nbytes;
	total_rx_bytes = op->data.nbytes;
	if (op->data.dir == SPI_MEM_DATA_OUT) {
		total_tx_bytes += op->data.nbytes;
		total_rx_bytes -= op->data.nbytes;
	};

	scai_fpgaqspi_set_operate_mode(p, false);
	scai_fpgaqspi_start_transaction(p, total_tx_bytes, total_rx_bytes);

	if (op->cmd.opcode) {
		p->tx_buf = &opcode;
		p->rx_buf = NULL;
		p->tx_len = op->cmd.nbytes;
		p->rx_len = 0;
		scai_fpgaqspi_write_op(p, false);
	}

	p->tx_buf = &opaddr[0];
	if (op->addr.nbytes) {
		for (i = 0; i < op->addr.nbytes; i++)
			p->tx_buf[i] = address >> (8 * (op->addr.nbytes - i - 1));

		p->rx_buf = NULL;
		p->tx_len = op->addr.nbytes;
		p->rx_len = 0;
		scai_fpgaqspi_write_op(p, false);
	}

	if (op->dummy.nbytes) {
		/* Put dummy bytes */
		for (i = 0; i < op->addr.nbytes; i++) {
			if (i > sizeof(opaddr)) {
				break;
			}
			opaddr[i] = 0;
		}

		p->tx_buf = &opaddr[0];
		p->rx_buf = NULL;
		p->tx_len = op->dummy.nbytes;
		p->rx_len = 0;
		scai_fpgaqspi_write_op(p, false);
	}

	if (op->data.nbytes && op->data.buswidth == 1) {
		if (op->data.dir == SPI_MEM_DATA_OUT) {
			p->tx_buf = (u8 *)op->data.buf.out;
			p->rx_buf = NULL;
			p->rx_len = 0;
			p->tx_len = op->data.nbytes;
			scai_fpgaqspi_write_op(p, false);
		} else {
			p->tx_buf = NULL;
			p->rx_buf = (u8 *)op->data.buf.in;
			p->rx_len = op->data.nbytes;
			p->tx_len = 0;
			scai_fpgaqspi_read_op(p, false);
		}
	}

	scai_fpgaqspi_finish_transaction(p, true);

	return err;
}

#if 0
static void dump_mem_op_info(const struct spi_mem_op *op)
{
	debug("\n");
	debug("==========================\n");
	debug("%s(%d):  op->cmd.opcode(0x%04X)\n",
			__func__, __LINE__, op->cmd.opcode);
	debug("%s(%d):  op->cmd.nbytes(0x%02X)\n",
			__func__, __LINE__, op->cmd.nbytes);
	debug("%s(%d):  op->cmd.buswidth(0x%02X)\n",
			__func__, __LINE__, op->cmd.buswidth);
	debug("%s(%d):  op->cmd.dtr(0x%02X)\n",
			__func__, __LINE__, op->cmd.dtr);
	debug("\n");

	debug("%s(%d):  op->addr.val(0x%016llX)\n",
			__func__, __LINE__, op->addr.val);
	debug("%s(%d):  op->addr.nbytes(0x%02X)\n",
			__func__, __LINE__, op->addr.nbytes);
	debug("%s(%d):  op->addr.buswidth(0x%02X)\n",
			__func__, __LINE__, op->addr.buswidth);
	debug("%s(%d):  op->addr.dtr(0x%02X)\n",
			__func__, __LINE__, op->addr.dtr);
	debug("\n");

	debug("%s(%d):  op->dummy.buswidth(0x%02X)\n",
			__func__, __LINE__, op->dummy.buswidth);
	debug("%s(%d):  op->dummy.dtr(0x%02X)\n",
			__func__, __LINE__, op->dummy.dtr);
	debug("%s(%d):  op->dummy.nbytes(0x%02X)\n",
			__func__, __LINE__, op->dummy.nbytes);
	debug("\n");

	debug("%s(%d):  op->data.buswidth(0x%02X)\n",
			__func__, __LINE__, op->data.buswidth);
	debug("%s(%d):  op->data.dtr(0x%02X)\n",
			__func__, __LINE__, op->data.dtr);
	debug("%s(%d):  op->data.nbytes(0x%08X)\n",
			__func__, __LINE__, op->data.nbytes);
	debug("==========================\n");
	debug("\n");
}
#endif

static int scai_fpgaqspi_exec_op(struct spi_slave *slave,
				 const struct spi_mem_op *op)
{
	int err = 0;

#if 0
	dump_mem_op_info(op);
#endif

	err = scai_fpgaqspi_wait_for_ready(slave);
	if (err)
		return err;

	err = __do_exec_byte_op(slave, op);
	err = __do_exec_word_op(slave, op);

	return 0;
}

static int scai_fpgaqspi_adjust_op_size(struct spi_slave *slave,
		struct spi_mem_op *op)
{
	if (op->data.dir == SPI_MEM_DATA_OUT &&
			op->data.nbytes > MAX_DATA_CMD_LEN)
	{
		op->data.nbytes = MAX_DATA_CMD_LEN;
#if 0
		debug("%s(%d):  op->data.nbytes(0x%08X)\n",
				__func__, __LINE__, op->data.nbytes);
#endif
	}

	return 0;
}

static bool scai_fpgaqspi_supports_op(struct spi_slave *slave,
		const struct spi_mem_op *op)
{
	if (!spi_mem_default_supports_op(slave, op))
		return false;

	if ((op->data.buswidth == 2 || op->data.buswidth == 4) &&
	    (op->cmd.buswidth == 1 && (op->addr.buswidth <= 1)) &&
	    op->data.dir == SPI_MEM_DATA_OUT)
		return false;

	return true;
}

static int scai_fpgaqspi_set_speed(struct udevice *dev, uint speed)
{
	return 0;
}
static int scai_fpgaqspi_set_mode(struct udevice *dev, uint mode)
{
	return 0;
}
static int scai_fpgaqspi_claim_bus(struct udevice *dev)
{
	return 0;
}
static int scai_fpgaqspi_release_bus(struct udevice *dev)
{
	return 0;
}

static int scai_fpgaqspi_probe(struct udevice *dev)
{
	struct scai_fpgaqspi_priv *p = dev_get_priv(dev);
	u32 control;

	/* Map QSPI and GPIO registers */
	p->regs = dev_remap_addr_index(dev, 0);
	p->gpio1_regs = dev_remap_addr_index(dev, 1);
	p->gpio2_regs = dev_remap_addr_index(dev, 2);

	if (!p->regs || !p->gpio1_regs || !p->gpio2_regs) {
		dev_err(dev, "Failed to map QSPI registers\n");
		return -EINVAL;
	}

	dev_info(dev, "SCAI FPGA QSPI REG mapped to VA: %p\n", p->regs);
	dev_info(dev, "GPIO1 mapped to VA: %p, Value: 0x%08X\n",
			p->gpio1_regs, readl(p->gpio1_regs + GPIO_REG_RDATA_OFFSET));
	dev_info(dev, "GPIO2 mapped to VA: %p, Value: 0x%08X\n",
			p->gpio2_regs, readl(p->gpio2_regs + GPIO_REG_RDATA_OFFSET));

	scai_fpgaqspi_set_power(p, true);
	dev_info(dev, "Enabled MT29F power via custom GPIOs\n");

	control = CTRL1_RESET;
	p->ctrl1_sw_copy = control;
	writel(control, p->regs + SCAI_QSPI_REG_CTRL1);

	writel(0, p->regs + SCAI_QSPI_REG_CTRL2);
	writel(BIT(16), p->regs + SCAI_QSPI_REG_CTRL3);

	return 0;
}

static const struct spi_controller_mem_ops scai_fpgaqspi_mem_ops = {
	.adjust_op_size = scai_fpgaqspi_adjust_op_size,
	.supports_op   = scai_fpgaqspi_supports_op,
	.exec_op       = scai_fpgaqspi_exec_op,
};

static const struct dm_spi_ops scai_fpgaqspi_ops = {
	.claim_bus      = scai_fpgaqspi_claim_bus,
	.release_bus    = scai_fpgaqspi_release_bus,
	.set_speed      = scai_fpgaqspi_set_speed,
	.set_mode       = scai_fpgaqspi_set_mode,
	.mem_ops        = &scai_fpgaqspi_mem_ops,
};

static const struct udevice_id scai_fpgaqspi_of_match[] = {
	{ .compatible = "scai-fpgaqspi,navc-mt29f" },
	{ .compatible = "scai-fpgaqspi,navc-backup-w25" },
	{ .compatible = "scai-fpgaqspi,navc-nor" },
	{ /* sentinel */ }
};

U_BOOT_DRIVER(scai_fpgaqspi) = {
	.name      = "scai_fpgaqspi",
	.id        = UCLASS_SPI,
	.of_match  = scai_fpgaqspi_of_match,
	.probe     = scai_fpgaqspi_probe,
	.ops       = &scai_fpgaqspi_ops,
	.priv_auto = sizeof(struct scai_fpgaqspi_priv),
};
