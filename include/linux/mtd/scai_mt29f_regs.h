/*
 * mchp_scai_nand_regs.h
 *
 * Register and command definitions for the SCAI QSPI controller and MT29F flash.
 */

#ifndef __MCHP_SCAI_NAND_REGS_H
#define __MCHP_SCAI_NAND_REGS_H

#include <linux/bitops.h> /* For BIT() */

/*
 * Эти определения больше не используются, но оставлены для справки.
 * #define CONFIG_SYS_NAND_MAX_CHIPS 1
 * #define CFG_SYS_NAND_BASE         0x20000000
 */

/* --- SCAI QSPI Controller Register Offsets --- */
#define SCAI_QSPI_REG_DATA          0x00
#define SCAI_QSPI_REG_CTRL1         0x04
#define SCAI_QSPI_REG_STATUS1       0x04 /* Status is read from same offset as CTRL1 write */

/* --- SCAI QSPI Controller CTRL1 Register Bits --- */
#define CTRL1_CHIP_ENABLE           BIT(0)
#define CTRL1_NWP                   BIT(1)
#define CTRL1_RESET                 BIT(2)
#define CTRL1_DATA_MODE_WORD        BIT(3) /* 0 = Byte, 1 = Word */
#define CTRL1_LANE_WIDTH_X4         BIT(4) /* 0 = x1, 1 = x4 */
#define CTRL1_START                 BIT(9)
#define CTRL1_TX_COUNT(n)           (((n) & 0x7FF) << 10) /* Count in bytes or words depending on CTRL1_DATA_MODE_WORD */
#define CTRL1_RX_COUNT(n)           (((n) & 0x7FF) << 21) /* Count in bytes or words depending on CTRL1_DATA_MODE_WORD */

/* --- SCAI QSPI Controller STATUS1 Register Bits --- */
#define STATUS1_IDLE                BIT(0)

/* --- MT29F Flash Command Opcodes --- */
#define MT29F_CMD_WRITE_ENABLE          0x06
#define MT29F_CMD_GET_FEATURES          0x0F
#define MT29F_CMD_SET_FEATURES          0x1F
#define MT29F_CMD_PAGE_READ_TO_CACHE    0x13
#define MT29F_CMD_READ_FROM_CACHE_X1    0x03
#define MT29F_CMD_READ_FROM_CACHE_X4    0x6B
#define MT29F_CMD_PROGRAM_LOAD_X1       0x02
#define MT29F_CMD_PROGRAM_LOAD_X4       0x32
#define MT29F_CMD_PROGRAM_EXECUTE       0x10
#define MT29F_CMD_BLOCK_ERASE           0xD8
#define MT29F_CMD_READ_ID               0x9F
#define MT29F_CMD_RESET_DEVICE          0xFF

/* --- MT29F Feature/Register Addresses --- */
#define MT29F_REG_STATUS                0xC0
#define MT29F_REG_LOCK                  0xA0
#define MT29F_REG_CONFIG                0xB0
#define MT29F_REG_DIE_SELECT            0xD0

/* --- MT29F Status Register Bits --- */
#define STATUS_OIP_BIT                  BIT(0) /* Operation In Progress */

#endif /* __MCHP_SCAI_NAND_REGS_H */
