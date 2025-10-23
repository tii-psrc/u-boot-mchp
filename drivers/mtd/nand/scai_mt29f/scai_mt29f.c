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

/* --- Low-level QSPI controller functions --- */

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
				      const u8 *tx_buf, u32 tx_len_elems,
				      u8 *rx_buf, u32 rx_len_elems,
				      bool use_word_mode, bool keep_ce)
{
	u32 ctrl1;
	u32 total_tx_bytes = use_word_mode ? tx_len_elems * 4 : tx_len_elems;
	u32 total_rx_bytes = use_word_mode ? rx_len_elems * 4 : rx_len_elems;
	int ret;

	/* Prepare CTRL1 register value */
	ctrl1 = priv->ctrl1_sw_copy;
	ctrl1 &= ~(CTRL1_TX_COUNT(0x7FF) | CTRL1_RX_COUNT(0x7FF) | CTRL1_DATA_MODE_WORD);
	ctrl1 |= CTRL1_TX_COUNT(tx_len_elems) | CTRL1_RX_COUNT(rx_len_elems);

	if (use_word_mode)
		ctrl1 |= CTRL1_DATA_MODE_WORD;

	/* Start transaction */
	ctrl1 |= CTRL1_START | CTRL1_CHIP_ENABLE;
	writel(ctrl1, priv->regs + SCAI_QSPI_REG_CTRL1);
	ctrl1 &= ~CTRL1_START;
	writel(ctrl1, priv->regs + SCAI_QSPI_REG_CTRL1);

	/* Handle FIFO operations */
	if (tx_len_elems > 0 && tx_buf) {
		for (u32 i = 0; i < total_tx_bytes; i += 4) {
			u32 word_to_write = 0;
			size_t bytes_to_copy = min((size_t)4, (size_t)(total_tx_bytes - i));
			memcpy(&word_to_write, tx_buf + i, bytes_to_copy);
			writel(word_to_write, priv->regs + SCAI_QSPI_REG_DATA);
		}
	}

	if (rx_len_elems > 0 && rx_buf) {
		for (u32 i = 0; i < total_rx_bytes; i += 4) {
			u32 read_word = readl(priv->regs + SCAI_QSPI_REG_DATA);
			size_t bytes_to_copy = min((size_t)4, (size_t)(total_rx_bytes - i));
			memcpy(rx_buf + i, &read_word, bytes_to_copy);
		}
	}

	/* Wait for controller to finish */
	ret = scai_nand_wait_idle(priv);
	if (ret)
		return ret;

	/* Finalize transaction (deassert CE) */
	if (!keep_ce) {
		ctrl1 &= ~CTRL1_CHIP_ENABLE;
		writel(ctrl1, priv->regs + SCAI_QSPI_REG_CTRL1);
	}

	priv->ctrl1_sw_copy = ctrl1;
	return 0;
}

/* --- SPI-NAND-like Protocol Helpers --- */

static int scai_nand_read_reg(struct scai_nand_priv *priv, u8 reg, u8 *val)
{
	const u8 cmd[] = { MT29F_CMD_GET_FEATURES, reg };
	return scai_nand_exec_transaction(priv, cmd, sizeof(cmd), val, 1, false, false);
}

static int scai_nand_write_reg(struct scai_nand_priv *priv, u8 reg, u8 val)
{
	const u8 cmd[] = { MT29F_CMD_SET_FEATURES, reg, val };
	return scai_nand_exec_transaction(priv, cmd, sizeof(cmd), NULL, 0, false, false);
}

static int scai_nand_wait(struct scai_nand_priv *priv)
{
	u8 status;
	int retries = 1000;

	do {
		if (scai_nand_read_reg(priv, MT29F_REG_STATUS, &status))
			return -EIO;
		if (!(status & STATUS_OIP_BIT))
			return 0;
		udelay(150);
	} while (--retries);

	dev_err(priv->mtd.dev, "Flash wait ready timeout\n");
	return -ETIMEDOUT;
}

static int scai_nand_write_enable(struct scai_nand_priv *priv)
{
	const u8 cmd = MT29F_CMD_WRITE_ENABLE;
	return scai_nand_exec_transaction(priv, &cmd, 1, NULL, 0, false, false);
}

static int scai_nand_reset_device(struct scai_nand_priv *priv)
{
	const u8 cmd = MT29F_CMD_RESET_DEVICE;
	int ret;

	ret = scai_nand_exec_transaction(priv, &cmd, 1, NULL, 0, false, false);
	if (ret)
		return ret;

	priv->current_die = 0; /* Resetting chip resets die select to 0 */
	return scai_nand_wait(priv);
}

static int scai_nand_select_target(struct scai_nand_priv *priv, int target_die)
{
	if (target_die < 0 || target_die > 1)
		return -EINVAL;

	if (priv->current_die == target_die)
		return 0;

	u8 die_val = (target_die == 1) ? 0x40 : 0x00;
	int ret = scai_nand_write_reg(priv, MT29F_REG_DIE_SELECT, die_val);
	if (ret == 0)
		priv->current_die = target_die;

	return ret;
}

static int scai_nand_page_read_to_cache(struct scai_nand_priv *priv, int page_addr)
{
	u8 cmd[4];

	cmd[0] = MT29F_CMD_PAGE_READ_TO_CACHE;
	cmd[1] = (page_addr >> 16) & 0xFF; /* Row Addr 2 (local) */
	cmd[2] = (page_addr >> 8) & 0xFF;  /* Row Addr 1 (local) */
	cmd[3] = page_addr & 0xFF;         /* Row Addr 0 (local) */

	return scai_nand_exec_transaction(priv, cmd, sizeof(cmd), NULL, 0, false, false);
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

	cmd[0] = priv->is_quad ? MT29F_CMD_READ_FROM_CACHE_X4 : MT29F_CMD_READ_FROM_CACHE_X1;
	cmd[1] = (col >> 8) & 0xFF; /* col addr MSB */
	cmd[2] = col & 0xFF; /* col addr LSB */
	cmd[3] = 0x00; /* dummy byte */

	/* Send READ FROM CACHE command, keep CE active */
	ret = scai_nand_exec_transaction(priv, cmd, sizeof(cmd), NULL, 0, false, true);
	if (ret)
		return ret;

	/* Receive data, release CE */
	return scai_nand_exec_transaction(priv, NULL, 0, buf, len_elems, use_word_mode, false);
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

	/* Send PROGRAM LOAD command, keep CE active */
	ret = scai_nand_exec_transaction(priv, cmd, sizeof(cmd), NULL, 0, false, true);
	if (ret)
		return ret;

	/* Send data for programming, release CE */
	return scai_nand_exec_transaction(priv, buf, len_elems, NULL, 0, use_word_mode, false);
}

static int scai_nand_program_execute(struct scai_nand_priv *priv, int page_addr)
{
	u8 cmd[4];

	cmd[0] = MT29F_CMD_PROGRAM_EXECUTE;
	cmd[1] = (page_addr >> 16) & 0xFF; /* Row Addr 2 (local) */
	cmd[2] = (page_addr >> 8) & 0xFF;  /* Row Addr 1 (local) */
	cmd[3] = page_addr & 0xFF;         /* Row Addr 0 (local) */

	return scai_nand_exec_transaction(priv, cmd, sizeof(cmd), NULL, 0, false, false);
}

static int scai_nand_block_erase(struct scai_nand_priv *priv, int page_addr)
{
	u8 cmd[4];

	cmd[0] = MT29F_CMD_BLOCK_ERASE;
	cmd[1] = (page_addr >> 16) & 0xFF; /* Row Addr 2 (local) */
	cmd[2] = (page_addr >> 8) & 0xFF;  /* Row Addr 1 (local) */
	cmd[3] = page_addr & 0xFF;         /* Row Addr 0 (local) */

	return scai_nand_exec_transaction(priv, cmd, sizeof(cmd), NULL, 0, false, false);
}


/* --- Custom GPIO Control Helper --- */

static void scai_nand_set_power(struct scai_nand_priv *priv, bool enable)
{
	u32 val1, val2;

	if (!priv->gpio1_regs || !priv->gpio2_regs) {
		printf("WARN: SCAI NAND: GPIO registers not mapped\n");
		return;
	}

	val1 = readl(priv->gpio1_regs + GPIO_REG_RDATA_OFFSET);
	val2 = readl(priv->gpio2_regs + GPIO_REG_RDATA_OFFSET);

	if (enable) {
		val1 |= GPIO1_ENA_SS1_MASK;
		val2 |= GPIO2_ENA_SS2_MASK;
	} else {
		val1 &= ~GPIO1_ENA_SS1_MASK;
		val2 &= ~GPIO2_ENA_SS2_MASK;
	}

	writel(val1, priv->gpio1_regs + GPIO_REG_WDATA_OFFSET);
	writel(val2, priv->gpio2_regs + GPIO_REG_WDATA_OFFSET);
}

/* --- MTD NAND Callbacks --- */

static int scai_nand_op_erase(struct nand_device *nand,
			    const struct nand_pos *pos)
{
	struct scai_nand_priv *priv = container_of(nand, struct scai_nand_priv, nand);
	int ret;
	int row = nanddev_pos_to_row(nand, pos);

	ret = scai_nand_select_target(priv, pos->target);
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

	return scai_nand_wait(priv);
}

static bool scai_nand_op_isbad(struct nand_device *nand,
			     const struct nand_pos *pos)
{
	/* This raw driver does not support bad block management */
	return false;
}

static int scai_nand_op_markbad(struct nand_device *nand,
			      const struct nand_pos *pos)
{
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
	struct nand_device *nand = mtd_to_nanddev(mtd);
	struct scai_nand_priv *priv = container_of(nand, struct scai_nand_priv, nand);
	struct nand_io_iter iter;
	int ret = 0;
	bool use_word_mode_data = priv->is_quad && ((nand->memorg.pagesize % 4) == 0);

	nanddev_io_for_each_page(nand, from, ops, &iter) {
		const struct nand_pos *pos = &iter.req.pos;
		int row = nanddev_pos_to_row(nand, pos);

		ret = scai_nand_select_target(priv, pos->target);
		if (ret)
			break;

		/* Reset controller to x1 mode for command */
		priv->ctrl1_sw_copy &= ~(CTRL1_LANE_WIDTH_X4 | CTRL1_DATA_MODE_WORD);
		ret = scai_nand_page_read_to_cache(priv, row);
		if (ret)
			break;

		ret = scai_nand_wait(priv);
		if (ret)
			break;

		/* Set controller lane width for data phase */
		if (priv->is_quad)
			priv->ctrl1_sw_copy |= CTRL1_LANE_WIDTH_X4;

		/* Read page data */
		if (iter.req.datalen) {
			ret = scai_nand_read_from_cache(priv, iter.req.dataoffs,
							iter.req.databuf.in,
							iter.req.datalen,
							use_word_mode_data);
			if (ret)
				break;
		}

		/* Read OOB data */
		if (iter.req.ooblen) {
			u16 col = nand->memorg.pagesize + iter.req.ooboffs;
			ret = scai_nand_read_from_cache(priv, col,
							iter.req.oobbuf.in,
							iter.req.ooblen,
							false); /* Force byte mode for OOB */
			if (ret)
				break;
		}
	}

	ops->retlen = ops->len - iter.dataleft;
	ops->oobretlen = ops->ooblen - iter.oobleft;
	return ret;
}

static int scai_nand_mtd_write_oob(struct mtd_info *mtd, loff_t to,
				   struct mtd_oob_ops *ops)
{
	struct nand_device *nand = mtd_to_nanddev(mtd);
	struct scai_nand_priv *priv = container_of(nand, struct scai_nand_priv, nand);
	struct nand_io_iter iter;
	int ret = 0;
	bool use_word_mode_data = priv->is_quad && ((nand->memorg.pagesize % 4) == 0);

	nanddev_io_for_each_page(nand, to, ops, &iter) {
		const struct nand_pos *pos = &iter.req.pos;
		int row = nanddev_pos_to_row(nand, pos);

		ret = scai_nand_select_target(priv, pos->target);
		if (ret)
			break;

		/* Set controller lane width for data phase */
		if (priv->is_quad)
			priv->ctrl1_sw_copy |= CTRL1_LANE_WIDTH_X4;
		else
			priv->ctrl1_sw_copy &= ~CTRL1_LANE_WIDTH_X4;

		/* Load page data */
		if (iter.req.datalen) {
			ret = scai_nand_write_enable(priv);
			if (ret) break;
			ret = scai_nand_program_load(priv, iter.req.dataoffs,
						     iter.req.databuf.out,
						     iter.req.datalen,
						     use_word_mode_data);
			if (ret) break;
			ret = scai_nand_wait(priv);
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
			ret = scai_nand_wait(priv);
			if (ret) break;
		}

		/* Execute program */
		/* Reset controller to x1 mode for command */
		priv->ctrl1_sw_copy &= ~(CTRL1_LANE_WIDTH_X4 | CTRL1_DATA_MODE_WORD);

		ret = scai_nand_write_enable(priv);
		if (ret) break;

		ret = scai_nand_program_execute(priv, row);
		if (ret) break;

		ret = scai_nand_wait(priv);
		if (ret) break;
	}

	ops->retlen = ops->len - iter.dataleft;
	ops->oobretlen = ops->ooblen - iter.oobleft;
	return ret;
}


/* --- U-Boot Driver Model Probe and Remove --- */

static int scai_nand_probe(struct udevice *dev)
{
	struct scai_nand_priv *priv = dev_get_priv(dev);
	struct nand_device *nand = &priv->nand;
	struct mtd_info *mtd = &priv->mtd;
	u8 jedec_ids[2];
	const u8 read_id_cmd[] = { MT29F_CMD_READ_ID, 0x00 };
	int ret;

	/* Map QSPI controller registers */
	priv->regs = dev_remap_addr_index(dev, 0);
	if (!priv->regs) {
		dev_err(dev, "Failed to map QSPI registers\n");
		return -EINVAL;
	}

	/* Map GPIO_1 control registers */
	priv->gpio1_regs = dev_remap_addr_index(dev, 1);
	if (!priv->gpio1_regs) {
		dev_err(dev, "Failed to map GPIO_1 registers\n");
		return -EINVAL;
	}

	/* Map GPIO_2 control registers */
	priv->gpio2_regs = dev_remap_addr_index(dev, 2);
	if (!priv->gpio2_regs) {
		dev_err(dev, "Failed to map GPIO_2 registers\n");
		return -EINVAL;
	}

	/* Enable flash power via custom GPIO logic */
	scai_nand_set_power(priv, true);
	dev_info(dev, "Enabled MT29F power via custom GPIOs\n");

	/* Initialize MTD and NAND structures */
	nand->mtd = mtd;
	mtd->priv = nand;
	mtd->dev = dev;
	mtd->name = (char *)dev->name;
	priv->current_die = -1; /* Force initial die select */

	priv->is_quad = dev_read_bool(dev, "spi-tx-bus-width-4");

	/* Initial CTRL1 software copy - Set RESET high */
	priv->ctrl1_sw_copy = CTRL1_RESET; /* nReset = 1 */
	writel(priv->ctrl1_sw_copy, priv->regs + SCAI_QSPI_REG_CTRL1);

	/* Reset the flash chip */
	ret = scai_nand_reset_device(priv);
	if (ret) {
		dev_err(dev, "Failed to reset device on probe\n");
		goto err_power_off;
	}

	/* Read JEDEC ID */
	priv->ctrl1_sw_copy &= ~(CTRL1_LANE_WIDTH_X4 | CTRL1_DATA_MODE_WORD);
	ret = scai_nand_exec_transaction(priv, read_id_cmd, sizeof(read_id_cmd),
					 NULL, 0, false, true); /* Keep CE */
	if (ret)
		goto err_power_off;
	ret = scai_nand_exec_transaction(priv, NULL, 0, jedec_ids,
					 sizeof(jedec_ids), false, false); /* Read, release CE */
	if (ret)
		goto err_power_off;

	dev_info(dev, "JEDEC ID: %02X %02X\n",
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

	/* This is a raw driver, no ECC handled here */
	nand->eccreq.strength = 0;
	nand->eccreq.step_size = 0;

	/* Initialize nand_device */
	ret = nanddev_init(nand, &scai_nand_ops, NULL);
	if (ret) {
		dev_err(dev, "nanddev_init failed: %d\n", ret);
		goto err_power_off;
	}

	/* Override MTD hooks for raw r/w */
	mtd->_read = NULL;  /* Use default mtd_read -> mtd_read_oob */
	mtd->_write = NULL; /* Use default mtd_write -> mtd_write_oob */
	mtd->_read_oob = scai_nand_mtd_read_oob;
	mtd->_write_oob = scai_nand_mtd_write_oob;
	/* Erase hook is set via nand_ops passed to nanddev_init */
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

