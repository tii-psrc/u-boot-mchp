// SPDX-License-Identifier: GPL-2.0
/*
 * mchp_scai_nand.c
 *
 * MTD NAND driver for Micron MT29F flash attached to a custom SCAI QSPI
 * controller.
 *
 * Uses U-Boot MTD/NAND framework (add_mtd_device, del_mtd_device).
 */

#include <common.h>
#include <dm.h>
#include <fdtdec.h> /* For dev_read_... helpers */
#include <log.h>    /* For dev_err/warn/info */
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h> /* Modern nand_device API */
#include <linux/errno.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/mtd/scai_mt29f_regs.h>
#include <dm/device.h>
#include <dm/device_compat.h>
#include <dm/uclass.h>

/*
 * GPIO definitions based on scai_fpga_platform.h from HSS.
 */
#define GPIO_REG_WDATA_OFFSET   0x00
#define GPIO_REG_RDATA_OFFSET   0x04
#define GPIO1_ENA_SS1_MASK      BIT(4)
#define GPIO2_ENA_SS2_MASK      BIT(0)

/* JEDEC ID for Micron MT29F8G01ADBFD12-AAT (8Gbit, 1GByte, 4K page) */
#define MT29F_JEDEC_MANUFACTURER_ID 0x2C
#define MT29F_JEDEC_DEVICE_ID_0     0x46 /* 8Gbit, 1.8V, 2-Die */
// #define MT29F_JEDEC_DEVICE_ID_1     0x90 /* 4KB Page, 256KB Block, 256B OOB */
// #define MT29F_JEDEC_DEVICE_ID_2     0xA6 /* SPI, 2-Plane */

/* MT29F Geometry (Based on provided Datasheet MICT-S-A0015861608-1.pdf) */
#define MT29F_PAGE_SIZE			4096
#define MT29F_OOB_SIZE			256
#define MT29F_PAGES_PER_BLOCK		64
#define MT29F_BLOCKS_PER_DIE		2048
#define MT29F_LUNS_PER_DIE		1
#define MT29F_BITS_PER_CELL		1

/**
 * struct scai_nand_priv - Private driver data structure
 * @nand:           NAND device object (for modern MTD framework)
 * @mtd:            MTD info object (for modern MTD framework)
 * @regs:           Memory-mapped pointer to the QSPI controller registers
 * @gpio1_regs:     Memory-mapped pointer to the GPIO_1 control registers
 * @gpio2_regs:     Memory-mapped pointer to the GPIO_2 control registers
 * @ctrl1_sw_copy:  Software cache of the CTRL1 register state
 * @is_quad:        Flag indicating if Quad I/O mode is enabled
 * @pages_per_die:  Number of pages in a single die
 * @current_die:    The currently selected die (0 or 1)
 */
struct scai_nand_priv {
	struct nand_device nand;
	struct mtd_info mtd;
	void __iomem *regs;
	void __iomem *gpio1_regs;
	void __iomem *gpio2_regs;
	u32 ctrl1_sw_copy;
	bool is_quad;
	u32 pages_per_die;
	int current_die;
};

static int scai_nand_exec_transaction(struct scai_nand_priv *priv,
				      const void *tx_buf, u32 tx_len_elems,
				      void *rx_buf, u32 rx_len_elems,
				      bool keep_ce);
/* --- Low-level QSPI controller functions --- */

// ==================== VERIFIED ====================
static void scai_set_reg(void __iomem *base, u32 offset, u32 value)
{
	// printf("W 0x%08X : 0x%08X\n", base + offset, value);
	writel(value, base + offset);
}

static u32 scai_get_reg(void __iomem *base, u32 offset)
{
	u32 val = readl(base + offset);
	// printf("R 0x%08X : 0x%08X\n", base + offset, val);
	return val;
}

static int scai_nand_get_feature(struct scai_nand_priv *priv, u8 feature, u8 *value)
{
	const u8 cmd[] = { MT29F_CMD_GET_FEATURES, feature };
	u8 val = 0;
	int ret;

	priv->ctrl1_sw_copy &= ~(CTRL1_LANE_WIDTH_X4 | CTRL1_DATA_MODE_WORD);

	ret =scai_nand_exec_transaction(priv, cmd, sizeof(cmd), &val, sizeof(val), false);
	*value = val;
	return ret;
}

static int scai_nand_set_feature(struct scai_nand_priv *priv, u8 feature, u8 value)
{
	const u8 cmd[] = { MT29F_CMD_SET_FEATURES, feature, value};

	priv->ctrl1_sw_copy &= ~(CTRL1_LANE_WIDTH_X4 | CTRL1_DATA_MODE_WORD);

	return scai_nand_exec_transaction(priv, cmd, sizeof(cmd), NULL, 0, false);
}

static int scai_nand_write_enable(struct scai_nand_priv *priv)
{
	const u8 cmd = MT29F_CMD_WRITE_ENABLE;

	priv->ctrl1_sw_copy &= ~(CTRL1_LANE_WIDTH_X4 | CTRL1_DATA_MODE_WORD);

	return scai_nand_exec_transaction(priv, &cmd, sizeof(cmd), NULL, 0, false);
}

static int scai_nand_wait_flash_ready(struct scai_nand_priv *priv)
{
	u8 status = 0;
	int retries = 1000;
	int err;

	while (--retries) {
		err = scai_nand_get_feature(priv, MT29F_REG_STATUS, &status);
		if (err)
			return -EIO;
		if (!(status & STATUS_OIP_BIT))
			return 0;
		udelay(150);
	}

	dev_err(priv->mtd.dev, "Flash wait ready timeout\n");
	return -ETIMEDOUT;
}

static int scai_nand_select_die(struct scai_nand_priv *priv, int die)
{
	if (die < 0 || die > 1)
		return -EINVAL;

	if (priv->current_die == die)
		return 0;

	u8 die_val = (die == 1) ? MT29F_DIE_1 : MT29F_DIE_0;
	int ret = scai_nand_set_feature(priv, MT29F_REG_DIE_SELECT, die_val);
	if (ret == 0)
		priv->current_die = die;

	return ret;
}

static int scai_nand_unlock_all_blocks(struct scai_nand_priv *priv)
{
	return scai_nand_set_feature(priv, MT29F_REG_LOCK, MT29F_UNLOCK_ALL);
}

// static int scai_nand_reset_device(struct scai_nand_priv *priv)
// {
// 	const u8 cmd = MT29F_CMD_RESET_DEVICE;
// 	int ret;
// 
// 	ret = scai_nand_exec_transaction(priv, &cmd, sizeof(cmd), NULL, 0, false, false);
// 	if (ret)
// 		return ret;
// 
// 	priv->current_die = 0; /* Resetting chip resets die select to 0 */
// 	return scai_nand_wait_flash_ready(priv);
// }

static int scai_read_id(struct scai_nand_priv *priv, u8 *jedec_ids, u32 len)
{
	const u8 cmd[] = { MT29F_CMD_READ_ID, 0xFF };
	return scai_nand_exec_transaction(priv, cmd, sizeof(cmd), jedec_ids, len, false);
}

static int scai_nand_init_device(struct scai_nand_priv *priv)
{
	int ret;	

	u8 config_reg = 0;
	ret = scai_nand_get_feature(priv, MT29F_REG_CONFIG, &config_reg);
	if (ret)
		return ret;

	config_reg |= CONFIG_CONTINUOUS;
	ret = scai_nand_set_feature(priv, MT29F_REG_CONFIG, config_reg);
	if (ret)
		return ret;

	ret = scai_nand_unlock_all_blocks(priv);
	if (ret)
		return ret;

	return ret;
}

static u32 scai_nand_fifo_write(struct scai_nand_priv *priv,
				const void* tx_buffer,
				u32 tx_len)
{
	u32 status2_word     = 0;
	u32 elements_written = 0;
	bool  is_word        = (priv->ctrl1_sw_copy & CTRL1_DATA_MODE_WORD) != 0;

	const u8* buf8      = (const u8*) tx_buffer;
	const u32* buf32     = (const u32*)tx_buffer;

	while (elements_written < tx_len) {
		u32 timeout_counter = SCAI_NAND_FIFO_TIMEOUT;
		/* Check for free space in FIFO */
		do {
			status2_word = scai_get_reg(priv->regs, SCAI_QSPI_REG_STATUS2);
			if (!(status2_word & STATUS2_TX_FIFO_FULL)) {
				break; /* Free space available, exit wait loop */
			}
			timeout_counter--;
		} while (timeout_counter > 0);

		if (timeout_counter == 0) {
			dev_err(priv->mtd.dev, "Tx FIFO timeout\n");
			return elements_written; /* Return the number of elements written */
		}

		/* Determine how much data can be written */
		u32 fifo_wr_cnt = (status2_word & STATUS2_TX_FIFO_WRCNT_MASK) >>
				  STATUS2_TX_FIFO_WRCNT_SHIFT;
		u32 free_space_words = SCAI_NAND_FIFO_LENGTH - fifo_wr_cnt;
		u32 chunk_size = tx_len - elements_written;
		if (chunk_size > free_space_words) {
			chunk_size = free_space_words;
		}

		/* Write the data to the FIFO */
		for (u32 i = 0; i < chunk_size; ++i) {
			u32 data_to_write = 0;

			/* HSS-based logic: if tx_buffer is NULL, write 0 as dummy data */
			if (tx_buffer) {
				if (is_word) {
					data_to_write = buf32[elements_written];
				} else {
					data_to_write = (((u32)buf8[elements_written]) <<
							 SCAI_QSPI_FIFO_BYTE_SHIFT) &
							SCAI_QSPI_FIFO_TX_BYTE_MASK;
					data_to_write |= ~SCAI_QSPI_FIFO_TX_BYTE_MASK;
				}
			} else {
				/*
				 * Send 0 as dummy word.
				 * This is needed for "Fake Write" in program_load.
				 */
				data_to_write = 0;
			}
			
			scai_set_reg(priv->regs, SCAI_QSPI_REG_DATA, data_to_write);
			elements_written++;
		}
	}

	return elements_written;
}

static u32 scai_nand_fifo_read(struct scai_nand_priv *priv,
			       void* rx_buffer,
			       u32 rx_len)
{
	u32  status2_word  = 0;
	u32  elements_read = 0;
	bool is_word       = (priv->ctrl1_sw_copy & CTRL1_DATA_MODE_WORD) != 0;

	u8* buf8          = (u8*)  rx_buffer;
	u32* buf32         = (u32*) rx_buffer;

	while (elements_read < rx_len) {
		u32 timeout_counter = SCAI_NAND_FIFO_TIMEOUT;

		/* Wait for data in Rx FIFO */
		do {
			status2_word = scai_get_reg(priv->regs, SCAI_QSPI_REG_STATUS2);
			if (!(status2_word & STATUS2_RX_FIFO_EMPTY)) {
				break; /* Data is available, exit the wait loop. */
			}
			timeout_counter--;
		} while (timeout_counter > 0);

		if (timeout_counter == 0) {
			dev_err(priv->mtd.dev, "Rx FIFO timeout\n");
			return elements_read;
		}

		/* Determine how much data can be read */
		u32 words_available = (status2_word & STATUS2_RX_FIFO_RdCnt_MASK) >>
				      STATUS2_RX_FIFO_RdCnt_SHIFT;
		u32 chunk_size = rx_len - elements_read;
		if (chunk_size > words_available) {
			chunk_size = words_available;
		}

		/* Read data */
		for (u32 i = 0; i < chunk_size; ++i) {
			u32 value = scai_get_reg(priv->regs, SCAI_QSPI_REG_DATA);

			/*
			 * HSS-based logic: if rx_buffer is NULL, discard the read.
			 * This is needed for the "Fake Read" operation.
			 */
			if (rx_buffer) {
				if (is_word) {
					buf32[elements_read] = value;
				} else {
					// As per softcore example, extract the LSB for byte-wise reads.
					buf8[elements_read] = (u8)(value & SCAI_QSPI_FIFO_RX_BYTE_MASK);
				}
			}
			elements_read++;
		}
	}

	/* Clean up FIFO */
	status2_word = scai_get_reg(priv->regs, SCAI_QSPI_REG_STATUS2);

	if (!(status2_word & STATUS2_RX_FIFO_EMPTY)) {
		u32 words_available = (status2_word & STATUS2_RX_FIFO_RdCnt_MASK) >>
				      STATUS2_RX_FIFO_RdCnt_SHIFT;
		dev_warn(priv->mtd.dev, "Draining %u unexpected words from RX FIFO\n",
			 words_available);
		for (u32 i = 0; i < words_available; ++i) {
			scai_get_reg(priv->regs, SCAI_QSPI_REG_DATA);
		}
	}

	return elements_read;
}

static void scai_nand_start_transaction(struct scai_nand_priv *priv, u32 tx_len_elems, u32 rx_len_elems)
{
	u32 ctrl1 = priv->ctrl1_sw_copy;

	ctrl1 &= ~(CTRL1_TX_COUNT(0x7FF) | CTRL1_RX_COUNT(0x7FF));
	ctrl1 |= CTRL1_TX_COUNT(tx_len_elems) | CTRL1_RX_COUNT(rx_len_elems);
	ctrl1 |= CTRL1_START;
	scai_set_reg(priv->regs, SCAI_QSPI_REG_CTRL1, ctrl1);

	ctrl1 |= CTRL1_CHIP_ENABLE;
	scai_set_reg(priv->regs, SCAI_QSPI_REG_CTRL1, ctrl1);
	priv->ctrl1_sw_copy = ctrl1;
}

static void scai_nand_finish_transaction(struct scai_nand_priv *priv, bool keep_ce)
{
	u32 ctrl1 = priv->ctrl1_sw_copy;

	ctrl1 &= ~(CTRL1_START | CTRL1_TX_COUNT(0x7FF) | CTRL1_RX_COUNT(0x7FF));
	scai_set_reg(priv->regs, SCAI_QSPI_REG_CTRL1, ctrl1);

	if (!keep_ce) {
		ctrl1 &= ~CTRL1_CHIP_ENABLE;
		scai_set_reg(priv->regs, SCAI_QSPI_REG_CTRL1, ctrl1);
	}
	priv->ctrl1_sw_copy = ctrl1;
}
// =============================================================

static int scai_nand_wait_idle(struct scai_nand_priv *priv)
{
	u32 status;
	int ret;

	ret = readl_poll_timeout(priv->regs + SCAI_QSPI_REG_STATUS1, status,
				 status & STATUS1_IDLE, 100000);
	if (ret)
		dev_err(priv->mtd.dev, "QSPI controller idle wait timeout\n");

	return ret;
}

static int scai_nand_exec_transaction(struct scai_nand_priv *priv,
				      const void *tx_buf, u32 tx_len_elems,
				      void *rx_buf, u32 rx_len_elems,
				      bool keep_ce)
{
	int ret = 0;	

	/* Start transaction */
	scai_nand_start_transaction(priv, tx_len_elems, rx_len_elems);

	/* Handle Tx FIFO operations */
	if (tx_len_elems > 0) {
		u32 written = scai_nand_fifo_write(priv, tx_buf, tx_len_elems);

		if (written < tx_len_elems) {
			dev_err(priv->mtd.dev, "FIFO write failed (wrote %u of %u)\n",
				written, tx_len_elems);
				scai_nand_finish_transaction(priv, keep_ce);			
				return -ETIMEDOUT;
		}
	}

	if (rx_len_elems > 0) {
		u32 read_count = scai_nand_fifo_read(priv, rx_buf, rx_len_elems);

		if (read_count < rx_len_elems) {
			dev_err(priv->mtd.dev, "FIFO read failed (read %u of %u)\n",
				read_count, rx_len_elems);
			scai_nand_finish_transaction(priv, keep_ce);
			return -ETIMEDOUT;
		}
	}
dev_err(priv->mtd.dev, "wait IDLE\n");
	/* Wait for controller to finish */
	ret = scai_nand_wait_idle(priv);
	if (ret)
		return ret;

	/* Finalize transaction */
	scai_nand_finish_transaction(priv, keep_ce);
	return 0;
}

/* --- SPI-NAND-like Protocol Helpers --- */

static int scai_nand_page_read_to_cache(struct scai_nand_priv *priv, int page_addr)
{
	u8 cmd[4];

	cmd[0] = MT29F_CMD_PAGE_READ_TO_CACHE;
	cmd[1] = (page_addr >> 16) & 0xFF; /* Row Addr 2 (local) */
	cmd[2] = (page_addr >> 8) & 0xFF;  /* Row Addr 1 (local) */
	cmd[3] = page_addr & 0xFF;         /* Row Addr 0 (local) */

	/* This is a single transaction, CE must be released at the end */
	/* Always x1, Byte mode */
	priv->ctrl1_sw_copy &= ~(CTRL1_LANE_WIDTH_X4 | CTRL1_DATA_MODE_WORD);
	return scai_nand_exec_transaction(priv, cmd, sizeof(cmd), NULL, 0, false);
}

/**
 * scai_nand_read_from_cache() - Read data from cache
 * @priv: Private driver data
 * @col: Column address (0 for page data, MT29F_PAGE_SIZE for OOB)
 * @buf: Buffer to store read data
 * @len_bytes: Length in bytes
 * @use_word_mode: True if using word mode (for data phase)
 */
static int scai_nand_read_from_cache(struct scai_nand_priv *priv, u16 col, u8 *buf,
				     u32 len_bytes, bool use_word_mode)
{
	u8 cmd[4];
	int ret;
	u32 len_elems = use_word_mode ? ((len_bytes + 3) / 4) : len_bytes;

	dev_err(priv->mtd.dev, "Read from cache: word_mode = %d, quad mode = %d\n", use_word_mode, priv->is_quad);

	cmd[0] = priv->is_quad ? MT29F_CMD_READ_FROM_CACHE_X4 : MT29F_CMD_READ_FROM_CACHE_X1;
	cmd[1] = (col >> 8) & 0xFF; /* col addr MSB */
	cmd[2] = col & 0xFF; /* col addr LSB */
	cmd[3] = 0x00; /* dummy byte */

	/*
	 * --- Phase 1: Send Command (in x1, byte mode) ---
	 * We must send the command in x1, byte mode. Keep CE active.
	 */
	dev_err(priv->mtd.dev, "Phase 1: send command\n");
	priv->ctrl1_sw_copy &= ~(CTRL1_LANE_WIDTH_X4 | CTRL1_DATA_MODE_WORD);
	ret = scai_nand_exec_transaction(priv, cmd, sizeof(cmd), NULL, 0, true);
	if (ret)
		return ret;

	/*
	 * --- Phase 2: Fake Read (HSS SCAI Quirk) ---
	 * Only required in quad mode to "fast-forward" to the correct col addr,
	 * as the controller starts streaming from col 0 regardless.
	 */
	if (priv->is_quad) {
		/* HSS code does (col_addr >> 2) */
		u32 dummy_rx_len_words = (col >> 2);

		/* Switch to Quad, Word mode for dummy read */
		priv->ctrl1_sw_copy |= (CTRL1_LANE_WIDTH_X4 | CTRL1_DATA_MODE_WORD);

		/*
			* Execute dummy read. Pass NULL as rx_buf to discard data.
			* Keep CE active.
			*/
		dev_err(priv->mtd.dev, "Phase 2 - dummy read\n");
		ret = scai_nand_exec_transaction(priv, NULL, 0,
							NULL, dummy_rx_len_words,
							true);
		if (ret)
			return ret;
	}

	/*
	 * --- Phase 3: Real Read ---
	 * Set the correct mode for the real data transfer.
	 */
	if (priv->is_quad) {
		priv->ctrl1_sw_copy |= CTRL1_LANE_WIDTH_X4;
	} else {
		priv->ctrl1_sw_copy &= ~CTRL1_LANE_WIDTH_X4;
	}

	if (use_word_mode) {
		priv->ctrl1_sw_copy |= CTRL1_DATA_MODE_WORD;
	} else {
		priv->ctrl1_sw_copy &= ~CTRL1_DATA_MODE_WORD;
	}

	/* Execute real data read. Release CE at the end. */
	dev_err(priv->mtd.dev, "Phase 3 - real read\n");
	ret = scai_nand_exec_transaction(priv, NULL, 0, buf, len_elems, false);

	return ret;
}

/**
 * scai_nand_program_load() - Send PROGRAM LOAD command and data
 * @priv: Private driver data
 * @col: Column address (0 for page data, MT29F_PAGE_SIZE for OOB)
 * @buf: Buffer containing data to write
 * @len_bytes: Length in bytes
 * @use_word_mode: True if using word mode (for data phase)
 */
static int scai_nand_program_load(struct scai_nand_priv *priv, u16 col, const u8 *buf,
				  u32 len_bytes, bool use_word_mode)
{
	u8 cmd[3];
	int ret;
	u32 len_elems = use_word_mode ? ((len_bytes + 3) / 4) : len_bytes;

	cmd[0] = priv->is_quad ? MT29F_CMD_PROGRAM_LOAD_X4 : MT29F_CMD_PROGRAM_LOAD_X1;
	cmd[1] = (col >> 8) & 0xFF; /* Column address MSB */
	cmd[2] = col & 0xFF; /* Column address LSB */

	/*
	 * --- Phase 1: Send Command (in x1, byte mode) ---
	 * Keep CE active.
	 */
	priv->ctrl1_sw_copy &= ~(CTRL1_LANE_WIDTH_X4 | CTRL1_DATA_MODE_WORD);
	ret = scai_nand_exec_transaction(priv, cmd, sizeof(cmd), NULL, 0, true);
	if (ret)
		return ret;

	/*
	 * --- Phase 2: Fake Write (HSS SCAI Quirk) ---
	 * Only required in quad mode to "fast-forward" to the correct col addr.
	 */
	if (priv->is_quad) {
		/* HSS code does (col_addr >> 2) */
		u32 dummy_tx_len_words = (col >> 2);

		if (dummy_tx_len_words > 0) {
			/* Switch to Quad, Word mode for dummy write */
			priv->ctrl1_sw_copy |= (CTRL1_LANE_WIDTH_X4 | CTRL1_DATA_MODE_WORD);

			/*
			 * Execute dummy write. Pass NULL as tx_buf to send 0s.
			 * Keep CE active.
			 */
			ret = scai_nand_exec_transaction(priv, NULL, dummy_tx_len_words,
						       NULL, 0,
						       true);
			if (ret)
				return ret;
		}
	}

	/*
	 * --- Phase 3: Real Write ---
	 * Set the correct mode for the real data transfer.
	 */
	if (priv->is_quad)
		priv->ctrl1_sw_copy |= CTRL1_LANE_WIDTH_X4;
	else
		priv->ctrl1_sw_copy &= ~CTRL1_LANE_WIDTH_X4;

	if (use_word_mode)
		priv->ctrl1_sw_copy |= CTRL1_DATA_MODE_WORD;
	else
		priv->ctrl1_sw_copy &= ~CTRL1_DATA_MODE_WORD;

	/* Execute real data write. Release CE at the end. */
	return scai_nand_exec_transaction(priv, buf, len_elems, NULL, 0, false);
}

static int scai_nand_program_execute(struct scai_nand_priv *priv, int page_addr)
{
	u8 cmd[4];

	cmd[0] = MT29F_CMD_PROGRAM_EXECUTE;
	cmd[1] = (page_addr >> 16) & 0xFF; /* Row Addr 2 (local) */
	cmd[2] = (page_addr >> 8) & 0xFF;  /* Row Addr 1 (local) */
	cmd[3] = page_addr & 0xFF;         /* Row Addr 0 (local) */

	/* This is a single transaction, CE must be released at the end */
	/* Always x1, Byte mode */
	priv->ctrl1_sw_copy &= ~(CTRL1_LANE_WIDTH_X4 | CTRL1_DATA_MODE_WORD);
	return scai_nand_exec_transaction(priv, cmd, sizeof(cmd), NULL, 0, false);
}

static int scai_nand_block_erase(struct scai_nand_priv *priv, int page_addr)
{
	u8 cmd[4];

	cmd[0] = MT29F_CMD_BLOCK_ERASE;
	cmd[1] = (page_addr >> 16) & 0xFF; /* Row Addr 2 (local) */
	cmd[2] = (page_addr >> 8) & 0xFF;  /* Row Addr 1 (local) */
	cmd[3] = page_addr & 0xFF;         /* Row Addr 0 (local) */

	/* This is a single transaction, CE must be released at the end */
	/* Always x1, Byte mode */
	priv->ctrl1_sw_copy &= ~(CTRL1_LANE_WIDTH_X4 | CTRL1_DATA_MODE_WORD);
	return scai_nand_exec_transaction(priv, cmd, sizeof(cmd), NULL, 0, false);
}


/* --- Custom GPIO Control Helper --- */

static void scai_nand_set_power(struct scai_nand_priv *priv, bool enable)
{
	u32 val1, val2;

	if (!priv->gpio1_regs || !priv->gpio2_regs) {
		printf("WARN: SCAI NAND: GPIO registers not mapped\n");
		return;
	}

	// val1 = scai_get_reg(priv->gpio1_regs, GPIO_REG_RDATA_OFFSET);
	val1 = 0;
	// val2 = scai_get_reg(priv->gpio2_regs, GPIO_REG_RDATA_OFFSET);
	val2 = 0;

	if (enable) {
		val1 |= GPIO1_ENA_SS1_MASK;
		val2 |= GPIO2_ENA_SS2_MASK;
	} else {
		val1 &= ~GPIO1_ENA_SS1_MASK;
		val2 &= ~GPIO2_ENA_SS2_MASK;
	}

	scai_set_reg(priv->gpio1_regs, GPIO_REG_WDATA_OFFSET, val1);
	scai_set_reg(priv->gpio2_regs, GPIO_REG_WDATA_OFFSET, val2);
}

/* --- MTD NAND Callbacks --- */

static int scai_nand_op_erase(struct nand_device *nand,
			    const struct nand_pos *pos)
{
	struct scai_nand_priv *priv = container_of(nand, struct scai_nand_priv, nand);
	int ret;
	int row = nanddev_pos_to_row(nand, pos);

	dev_err(priv->mtd.dev, "Erasing row %d\n", row);

	ret = scai_nand_select_die(priv, pos->target);
	if (ret)
		return ret;

	/* Reset controller to x1 mode for command */
	priv->ctrl1_sw_copy &= ~(CTRL1_LANE_WIDTH_X4 | CTRL1_DATA_MODE_WORD);

	ret = scai_nand_write_enable(priv);
	if (ret)
		return ret;

	ret = scai_nand_block_erase(priv, row);
	if (ret)
		return ret;

	return scai_nand_wait_flash_ready(priv);
}

static bool scai_nand_op_isbad(struct nand_device *nand,
			     const struct nand_pos *pos)
{
	dev_err(nand->mtd->dev, "Debug: scai_nand_op_isbad() called.\n");
	/* This raw driver does not support bad block management */
	return false;
}

static int scai_nand_op_markbad(struct nand_device *nand,
			      const struct nand_pos *pos)
{
	dev_err(nand->mtd->dev, "Debug: scai_nand_op_markbad() called.\n");
	return -EOPNOTSUPP;
}

static const struct nand_ops scai_nand_ops = {
	.erase = scai_nand_op_erase,
	.isbad = scai_nand_op_isbad,
	.markbad = scai_nand_op_markbad,
};

static int scai_nand_mtd_read_oob(struct mtd_info *mtd, loff_t from,
				  struct mtd_oob_ops *ops)
{
	dev_err(mtd->dev, "Debug: scai_nand_mtd_read_oob() called.\n");

	struct nand_device *nand    = mtd_to_nanddev(mtd);
	struct scai_nand_priv *priv = container_of(nand, struct scai_nand_priv, nand);
	struct nand_io_iter iter;
	int ret = 0;
	/*
	 * Word mode is only used for quad.
	 * OOB is always forced to byte mode.
	 */
	bool use_word_mode_data = priv->is_quad;

	nanddev_io_for_each_page(nand, from, ops, &iter) {
		const struct nand_pos *pos = &iter.req.pos;
		int row = nanddev_pos_to_row(nand, pos);

		dev_err(mtd->dev, "Debug: Read loop start (Row: %d, Target: %d)\n", row, pos->target);

		dev_err(mtd->dev, "Debug: Read selecting die...\n");
		ret = scai_nand_select_die(priv, pos->target);
		if (ret) {
			dev_err(mtd->dev, "Debug: Read select die FAILED\n");
			break;
		}

		dev_err(mtd->dev, "Debug: Read (A) calling page_read_to_cache...\n");
		ret = scai_nand_page_read_to_cache(priv, row);
		if (ret) {
			dev_err(mtd->dev, "Debug: Read page_read_to_cache FAILED\n");
			break;
		}

		dev_err(mtd->dev, "Debug: Read (B) calling wait_flash_ready...\n");
		ret = scai_nand_wait_flash_ready(priv);
		if (ret) {
			dev_err(mtd->dev, "Debug: Read wait_flash_ready FAILED\n");
			break;
		}

		/* Read page data */
		if (iter.req.datalen) {
			dev_err(mtd->dev, "Debug: Read (C) calling read_from_cache (data)...\n");
			ret = scai_nand_read_from_cache(priv, iter.req.dataoffs,
							iter.req.databuf.in,
							iter.req.datalen,
							use_word_mode_data);
			if (ret) {
				dev_err(mtd->dev, "Debug: Read read_from_cache (data) FAILED\n");
				break;
			}
		}

		/* Read OOB data */
		if (iter.req.ooblen) {
			u16 col = nand->memorg.pagesize + iter.req.ooboffs;
			dev_err(mtd->dev, "Debug: Read (D) calling read_from_cache (oob)...\n");
			ret = scai_nand_read_from_cache(priv, col,
							iter.req.oobbuf.in,
							iter.req.ooblen,
							false); /* Force byte mode for OOB */
			if (ret) {
				dev_err(mtd->dev, "Debug: Read read_from_cache (oob) FAILED\n");
				break;
			}
		}
		dev_err(mtd->dev, "Debug: Read loop finished for row %d\n", row);
	}

	ops->retlen = ops->len - iter.dataleft;
	ops->oobretlen = ops->ooblen - iter.oobleft;
	dev_err(mtd->dev, "Debug: scai_nand_mtd_read_oob() exiting.\n");
	return ret;
}

static int scai_nand_mtd_write_oob(struct mtd_info *mtd, loff_t to,
				   struct mtd_oob_ops *ops)
{
	struct nand_device *nand = mtd_to_nanddev(mtd);
	struct scai_nand_priv *priv = container_of(nand, struct scai_nand_priv, nand);
	struct nand_io_iter iter;
	int ret = 0;
	/* Word mode is only used for quad. OOB is forced to byte mode. */
	bool use_word_mode_data = priv->is_quad;

	nanddev_io_for_each_page(nand, to, ops, &iter) {
		const struct nand_pos *pos = &iter.req.pos;
		int row = nanddev_pos_to_row(nand, pos);

		ret = scai_nand_select_die(priv, pos->target);
		if (ret)
			break;

		/* Load page data */
		if (iter.req.datalen) {
			ret = scai_nand_write_enable(priv);
			if (ret) break;
			ret = scai_nand_program_load(priv, iter.req.dataoffs,
						     iter.req.databuf.out,
						     iter.req.datalen,
						     use_word_mode_data);
			if (ret) break;
			ret = scai_nand_wait_flash_ready(priv);
			if (ret) break;
		}

		/* Load OOB data */
		if (iter.req.ooblen) {
			u16 col = nand->memorg.pagesize + iter.req.ooboffs;
			ret = scai_nand_write_enable(priv);
			if (ret) break;
			ret = scai_nand_program_load(priv, col,
						     iter.req.oobbuf.out,
						     iter.req.ooblen,
						     false); /* Force byte mode for OOB */
			if (ret) break;
			ret = scai_nand_wait_flash_ready(priv);
			if (ret) break;
		}

		/* Execute program */
		ret = scai_nand_write_enable(priv);
		if (ret) break;

		ret = scai_nand_program_execute(priv, row);
		if (ret) break;

		ret = scai_nand_wait_flash_ready(priv);
		if (ret) break;
	}

	ops->retlen = ops->len - iter.dataleft;
	ops->oobretlen = ops->ooblen - iter.oobleft;
	return ret;
}

/*
 * MTD ERASE WRAPPER FUNCTION
 */
static int scai_nand_mtd_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	dev_err(mtd->dev, "Debug: scai_nand_mtd_erase() called. Addr: 0x%llx, Len: 0x%llx\n",
		instr->addr, instr->len);

	/* Bridge to nanddev handler, which will call nand->ops->erase */
	/* This is the correct call as per nand.h and spi-nand-core.c */
	return nanddev_mtd_erase(mtd, instr);
}

/*
 * MTD BAD BLOCK MANAGEMENT WRAPPERS
 */
static int scai_nand_mtd_block_isbad(struct mtd_info *mtd, loff_t offs)
{
	struct nand_device *nand = mtd_to_nanddev(mtd);
	struct nand_pos pos;
	int ret;

	dev_err(mtd->dev, "Debug: scai_nand_mtd_block_isbad() called.\n");

	nanddev_offs_to_pos(nand, offs, &pos);

	if (!nand->ops->isbad) {
		dev_warn(mtd->dev, "isbad operation not supported!\n");
		return 0; /* Assume good */
	}

	ret = nand->ops->isbad(nand, &pos);
	dev_err(mtd->dev, "Debug: scai_nand_mtd_block_isbad() exiting.\n");
	return ret;
}

static int scai_nand_mtd_block_markbad(struct mtd_info *mtd, loff_t offs)
{
	struct nand_device *nand = mtd_to_nanddev(mtd);
	struct nand_pos pos;
	int ret;

	dev_err(mtd->dev, "Debug: scai_nand_mtd_block_markbad() called.\n");

	nanddev_offs_to_pos(nand, offs, &pos);

	if (!nand->ops->markbad) {
		dev_warn(mtd->dev, "markbad operation not supported!\n");
		return -EOPNOTSUPP;
	}
	
	ret = nand->ops->markbad(nand, &pos);
	dev_err(mtd->dev, "Debug: scai_nand_mtd_block_markbad() finished.\n");
	return ret;
}

static int scai_nand_mtd_block_isreserved(struct mtd_info *mtd, loff_t offs)
{
	dev_err(mtd->dev, "Debug: scai_nand_mtd_block_isreserved() called.\n");
	/*
	 * We don't have a specific op for this,
	 * just report "not reserved".
	 */
	return 0;
}


/* --- U-Boot Driver Model Probe and Remove --- */

static int scai_nand_probe(struct udevice *dev)
{
	struct scai_nand_priv *priv = dev_get_priv(dev);
	struct    nand_device *nand = &priv->nand;
	struct       mtd_info *mtd  = &priv->mtd;
	u8 jedec_ids[2];
	int ret;

	/* Map QSPI controller registers */
	priv->regs = dev_remap_addr_index(dev, 0);
	if (!priv->regs) {
		dev_err(dev, "Failed to map QSPI registers\n");
		return -EINVAL;
	}

	dev_err(dev, "6 QSPI_REG mapped to VA: %p\n", priv->regs);

	/* Map GPIO_1 control registers */
	priv->gpio1_regs = dev_remap_addr_index(dev, 1);
	if (!priv->gpio1_regs) {
		dev_err(dev, "Failed to map GPIO_1 registers\n");
		return -EINVAL;
	}

	dev_err(dev, "GPIO_1 mapped to VA: %p, Value: 0x%08X\n",
             priv->gpio1_regs, scai_get_reg(priv->gpio1_regs, GPIO_REG_RDATA_OFFSET));

	/* Map GPIO_2 control registers */
	priv->gpio2_regs = dev_remap_addr_index(dev, 2);
	if (!priv->gpio2_regs) {
		dev_err(dev, "Failed to map GPIO_2 registers\n");
		return -EINVAL;
	}

	dev_err(dev, "GPIO_2 mapped to VA: %p, Value: 0x%08X\n",
             priv->gpio2_regs, scai_get_reg(priv->gpio2_regs, GPIO_REG_RDATA_OFFSET));

	/* Enable flash power via custom GPIO logic */
	scai_nand_set_power(priv, true);
	dev_err(dev, "Enabled MT29F power via custom GPIOs\n");

	/* Initialize MTD and NAND structures */
	nand->mtd = mtd;
	mtd->priv = nand;
	mtd->dev = dev;
	mtd->name = (char *)dev->name;
	priv->current_die = -1; /* Force initial die select */

	priv->is_quad = dev_read_bool(dev, "spi-tx-bus-width-4");
	if (priv->is_quad)
		dev_err(dev, "Quad mode selected\n");
	else {
		dev_err(dev, "Single mode\n");
		dev_err(dev, "Temporary set to true\n");
		priv->is_quad = true;
	}

	/* Initial CTRL1 software copy - Set RESET high */
	priv->ctrl1_sw_copy = CTRL1_RESET; /* nReset = 1 */
	priv->ctrl1_sw_copy &= ~(CTRL1_LANE_WIDTH_X4 | CTRL1_DATA_MODE_WORD);
	scai_set_reg(priv->regs, SCAI_QSPI_REG_CTRL1, priv->ctrl1_sw_copy);

	/* Reset the flash chip */
	// ret = scai_nand_reset_device(priv);
	// if (ret) {
	// 	dev_err(dev, "Failed to reset device on probe\n");
	// 	goto err_power_off;
	// }

	ret = scai_nand_init_device(priv);
	if (ret) {
		dev_err(dev, "Failed to initialize device on probe\n");
		goto err_power_off;
	}

	/* Read JEDEC ID */
	ret = scai_read_id(priv, jedec_ids, sizeof(jedec_ids));
	if (ret)
		goto err_power_off;

	dev_err(dev, "JEDEC ID: %02X %02X\n",
		 jedec_ids[0], jedec_ids[1]);

	/* Verify JEDEC ID */
	if (jedec_ids[0] != MT29F_JEDEC_MANUFACTURER_ID ||
	    jedec_ids[1] != MT29F_JEDEC_DEVICE_ID_0) {
		dev_warn(dev, "JEDEC ID mismatch, expected %02X %02X\n",
			 MT29F_JEDEC_MANUFACTURER_ID, MT29F_JEDEC_DEVICE_ID_0);
		/* Continue anyway, assuming compatible geometry */
	}

	/* Fill memory organization for one die */
	nand->memorg = (struct nand_memory_organization) {
		.bits_per_cell = MT29F_BITS_PER_CELL,
		.pagesize = MT29F_PAGE_SIZE,
		.oobsize = MT29F_OOB_SIZE,
		.pages_per_eraseblock = MT29F_PAGES_PER_BLOCK,
		.eraseblocks_per_lun = MT29F_BLOCKS_PER_DIE,
		.luns_per_target = MT29F_LUNS_PER_DIE,
		.ntargets = 1, /* Start with 1 target */
		.planes_per_lun = 1, /* Datasheet (Fig 6) shows 1 plane per die */
	};

	/* This raw driver, no ECC handled here */
	nand->eccreq.strength = 0;
	nand->eccreq.step_size = 0;

	/* Initialize nand_device */
	ret = nanddev_init(nand, &scai_nand_ops, NULL);
	if (ret) {
		dev_err(dev, "nanddev_init failed: %d\n", ret);
		goto err_power_off;
	}

	/*
	 * Set up MTD hooks.
	 */
	mtd->_read = NULL;
	mtd->_write = NULL;
	mtd->_read_oob = scai_nand_mtd_read_oob;
	mtd->_write_oob = scai_nand_mtd_write_oob;
	mtd->_erase = scai_nand_mtd_erase;

	/*
	 * Register bad block management hooks to prevent segfault
	 * when MTD framework checks block status before erase.
	 */
	mtd->_block_isbad = scai_nand_mtd_block_isbad;
	mtd->_block_markbad = scai_nand_mtd_block_markbad;
	mtd->_block_isreserved = scai_nand_mtd_block_isreserved;

	mtd->flags = MTD_CAP_NANDFLASH | MTD_WRITEABLE;

	/* Manually adjust size for dual-die */
	priv->pages_per_die = nand->memorg.pages_per_eraseblock *
			      nand->memorg.eraseblocks_per_lun;
	nand->memorg.ntargets = 2; /* Set to 2 dies */
	mtd->size = nanddev_size(nand); /* Recalculate size */

	dev_info(dev, "Found 1 die, size %llu. Adjusting for 2 dies.\n",
		 (unsigned long long)mtd->size / 2);

	/* Register with MTD subsystem using U-Boot function */
	ret = add_mtd_device(mtd);
	if (ret) {
		dev_err(dev, "add_mtd_device failed: %d\n", ret);
		goto err_nand_cleanup;
	}

	dev_info(dev, "SCAI MT29F driver initialized (Page: %u, OOB: %u, Block: %uKB, Total: %lluMB)\n",
		 mtd->writesize, mtd->oobsize, mtd->erasesize / 1024,
		 (unsigned long long)mtd->size >> 20);
	return 0;

err_nand_cleanup:
	nanddev_cleanup(nand);
err_power_off:
	/* Power remains on */
	return ret;
}

static int scai_nand_remove(struct udevice *dev)
{
	struct scai_nand_priv *priv = dev_get_priv(dev);
	struct nand_device *nand = &priv->nand;
	struct mtd_info *mtd = &priv->mtd;
	int ret;

	/* Unregister from MTD using U-Boot function */
	ret = del_mtd_device(mtd);
	if (ret)
		dev_err(dev, "del_mtd_device failed: %d\n", ret);

	/* Cleanup nand_device */
	nanddev_cleanup(nand);

	/* Power remains on */
	dev_info(dev, "MT29F power left enabled\n");

	return ret;
}

static const struct udevice_id scai_nand_of_match[] = {
	{ .compatible = "navc,scai-qspi-mt29f" },
	{ /* sentinel */ }
};

U_BOOT_DRIVER(mchp_scai_nand) = {
	.name           = "mchp_scai_nand",
	.id             = UCLASS_MTD,
	.of_match       = scai_nand_of_match,
	.probe          = scai_nand_probe,
	.remove         = scai_nand_remove,
	.priv_auto      = sizeof(struct scai_nand_priv),
};

