// SPDX-License-Identifier: GPL-2.0
/*
 * TI PHY drivers
 *
 */
#include <common.h>
#include <log.h>
#include <phy.h>
#include <malloc.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/compat.h>

#define DP83TG720_CS_1_1_PHY_ID			0x2000a284
#define DP83TG721_CS_1_0_PHY_ID			0x2000a290
#define MMD1F							0x1f
#define MMD1							0x1

#define MII_DP83TG720_INT_STAT1			0x12
#define MII_DP83TG720_INT_STAT2			0x13
#define MII_DP83TG720_INT_STAT3			0x18
#define MII_DP83TG720_RESET_CTRL		0x1f

#define PMA_PMD_CONTROL					0x1834
#define DP83TG720_TDR_CFG5				0x0306
#define DP83TG720_CDCR					0x1E
#define TDR_DONE						BIT(1)
#define TDR_FAIL						BIT(0)
#define DP83TG720_TDR_TC1				0x310
#define DP83TG720_TDR_START_BIT			BIT(15)	
#define BRK_MS_CFG						BIT(14)
#define PEAK_DETECT						BIT(7)
#define PEAK_SIGN						BIT(6)

#define DP83TG720_HW_RESET				BIT(15)
#define DP83TG720_SW_RESET				BIT(14)

#define DP83TG720_STRAP					0x45d
#define DP83TG720_SGMII_CTRL			0x608
#define SGMII_CONFIG_VAL				0x027B

/* INT_STAT1 bits */
#define DP83TG720_ANEG_COMPLETE_INT_EN	BIT(2)
#define DP83TG720_ESD_EVENT_INT_EN		BIT(3)
#define DP83TG720_LINK_STAT_INT_EN		BIT(5)
#define DP83TG720_ENERGY_DET_INT_EN		BIT(6)
#define DP83TG720_LINK_QUAL_INT_EN		BIT(7)

/* INT_STAT2 bits */
#define DP83TG720_SLEEP_MODE_INT_EN		BIT(2)
#define DP83TG720_OVERTEMP_INT_EN		BIT(3)
#define DP83TG720_OVERVOLTAGE_INT_EN	BIT(6)
#define DP83TG720_UNDERVOLTAGE_INT_EN	BIT(7)

/* INT_STAT3 bits */
#define DP83TG720_LPS_INT_EN			BIT(0)
#define DP83TG720_WAKE_REQ_EN			BIT(1)
#define DP83TG720_NO_FRAME_INT_EN		BIT(2)
#define DP83TG720_POR_DONE_INT_EN		BIT(3)

/* SGMII CTRL bits */
#define DP83TG720_SGMII_AUTO_NEG_EN		BIT(0)
#define DP83TG720_SGMII_EN				BIT(9)

/* Strap bits */
#define DP83TG720_MASTER_MODE			BIT(5)
#define DP83TG720_RGMII_IS_EN			BIT(12)
#define DP83TG720_SGMII_IS_EN			BIT(13)
#define DP83TG720_RX_SHIFT_EN			BIT(14)
#define DP83TG720_TX_SHIFT_EN			BIT(15)

/* RGMII ID CTRL */
#define DP83TG720_RGMII_ID_CTRL			0x602
#define DP83TG720_RX_CLK_SHIFT			BIT(1)
#define DP83TG720_TX_CLK_SHIFT			BIT(0)

/* SQI Status Bits */
#define DP83TG720_sqi_reg_1				0x871
#define MAX_SQI_VALUE					0x7

enum DP83TG720_chip_type {
	DP83TG720_CS1_1,
	DP83TG721_CS1,
};

struct DP83TG720_init_reg {
	int MMD;
	int reg;
	int val;
};

static const struct DP83TG720_init_reg DP83TG720_cs1_1_master_init[] = {
	{0x1F, 0x001F, 0X8000},
	{0x1F, 0x0573, 0x0101},
	{0x1, 0x0834, 0xC001},
	{0x1F, 0x0405, 0x5800},
	{0x1F, 0x08AD, 0x3C51},
	{0x1F, 0x0894, 0x5DF7},
	{0x1F, 0x08A0, 0x09E7},
	{0x1F, 0x08C0, 0x4000},
	{0x1F, 0x0814, 0x4800},
	{0x1F, 0x080D, 0x2EBF},
	{0x1F, 0x08C1, 0x0B00},
	{0x1F, 0x087D, 0x0001},
	{0x1F, 0x082E, 0x0000},
	{0x1F, 0x0837, 0x00F4},
	{0x1F, 0x08BE, 0x0200},
	{0x1F, 0x08C5, 0x4000},
	{0x1F, 0x08C7, 0x2000},
	{0x1F, 0x08B3, 0x005A},
	{0x1F, 0x08B4, 0x005A},
	{0x1F, 0x08B0, 0x0202},
	{0x1F, 0x08B5, 0x00EA},
	{0x1F, 0x08BA, 0x2828},
	{0x1F, 0x08BB, 0x6828},
	{0x1F, 0x08BC, 0x0028},
	{0x1F, 0x08BF, 0x0000},
	{0x1F, 0x08B1, 0x0014},
	{0x1F, 0x08B2, 0x0008},
	{0x1F, 0x08EC, 0x0000},
	{0x1F, 0x08C8, 0x0003},
	{0x1F, 0x08BE, 0x0201},
	{0x1F, 0x018C, 0x0001},
	{0x1F, 0x001F, 0x4000},
	{0x1F, 0x0573, 0x0001},
	{0x1F, 0x056A, 0x5F41},
};

static const struct DP83TG720_init_reg DP83TG720_cs1_1_slave_init[] = {
	{0x1F, 0x001F, 0x8000},
	{0x1F, 0x0573, 0x0101},
	{0x1, 0x0834, 0x8001},
	{0x1F, 0x0894, 0x5DF7},
	{0x1F, 0x056a, 0x5F40},
	{0x1F, 0x0405, 0x5800},
	{0x1F, 0x08AD, 0x3C51},
	{0x1F, 0x0894, 0x5DF7},
	{0x1F, 0x08A0, 0x09E7},
	{0x1F, 0x08C0, 0x4000},
	{0x1F, 0x0814, 0x4800},
	{0x1F, 0x080D, 0x2EBF},
	{0x1F, 0x08C1, 0x0B00},
	{0x1F, 0x087d, 0x0001},
	{0x1F, 0x082E, 0x0000},
	{0x1F, 0x0837, 0x00f4},
	{0x1F, 0x08BE, 0x0200},
	{0x1F, 0x08C5, 0x4000},
	{0x1F, 0x08C7, 0x2000},
	{0x1F, 0x08B3, 0x005A},
	{0x1F, 0x08B4, 0x005A},
	{0x1F, 0x08B0, 0x0202},
	{0x1F, 0x08B5, 0x00EA},
	{0x1F, 0x08BA, 0x2828},
	{0x1F, 0x08BB, 0x6828},
	{0x1F, 0x08BC, 0x0028},
	{0x1F, 0x08BF, 0x0000},
	{0x1F, 0x08B1, 0x0014},
	{0x1F, 0x08B2, 0x0008},
	{0x1F, 0x08EC, 0x0000},
	{0x1F, 0x08C8, 0x0003},
	{0x1F, 0x08BE, 0x0201},
	{0x1F, 0x056A, 0x5F40},
	{0x1F, 0x018C, 0x0001},
	{0x1F, 0x001F, 0x4000},
	{0x1F, 0x0573, 0x0001},
	{0x1F, 0x056A, 0X5F41},
};

static const struct DP83TG720_init_reg DP83TG721_cs1_master_init[] = {
	{0x1F, 0x001F, 0x8000},
	{0x1F, 0x0573, 0x0801},
	{0x1, 0x0834, 0xC001},
	{0x1F, 0x0405, 0x6C00},
	{0x1F, 0x08AD, 0x3C51},
	{0x1F, 0x0894, 0x5DF7},
	{0x1F, 0x08A0, 0x09E7},
	{0x1F, 0x08C0, 0x4000},
	{0x1F, 0x0814, 0x4800},
	{0x1F, 0x080D, 0x2EBF},
	{0x1F, 0x08C1, 0x0B00},
	{0x1F, 0x087D, 0x0001},
	{0x1F, 0x082E, 0x0000},
	{0x1F, 0x0837, 0x00F8},
	{0x1F, 0x08BE, 0x0200},
	{0x1F, 0x08C5, 0x4000},
	{0x1F, 0x08C7, 0x2000},
	{0x1F, 0x08B3, 0x005A},
	{0x1F, 0x08B4, 0x005A},
	{0x1F, 0x08B0, 0x0202},
	{0x1F, 0x08B5, 0x00EA},
	{0x1F, 0x08BA, 0x2828},
	{0x1F, 0x08BB, 0x6828},
	{0x1F, 0x08BC, 0x0028},
	{0x1F, 0x08BF, 0x0000},
	{0x1F, 0x08B1, 0x0014},
	{0x1F, 0x08B2, 0x0008},
	{0x1F, 0x08EC, 0x0000},
	{0x1F, 0x08FC, 0x0091},
	{0x1F, 0x08BE, 0x0201},
	{0x1F, 0x0335, 0x0010},
	{0x1F, 0x0336, 0x0009},
	{0x1F, 0x0337, 0x0208},
	{0x1F, 0x0338, 0x0208},
	{0x1F, 0x0339, 0x02CB},
	{0x1F, 0x033A, 0x0208},
	{0x1F, 0x033B, 0x0109},
	{0x1F, 0x0418, 0x0380},
	{0x1F, 0x0420, 0xFF10},
	{0x1F, 0x0421, 0x4033},
	{0x1F, 0x0422, 0x0800},
	{0x1F, 0x0423, 0x0002},
	{0x1F, 0x0484, 0x0003},
	{0x1F, 0x055D, 0x0008},
	{0x1F, 0x042B, 0x0018},
	{0x1F, 0x087C, 0x0080},
	{0x1F, 0x08C1, 0x0900},
	{0x1F, 0x08fc, 0x4091},
	{0x1F, 0x0881, 0x5146},
	{0x1F, 0x08be, 0x02a1},
	{0x1F, 0x0867, 0x9999},
	{0x1F, 0x0869, 0x9666},
	{0x1F, 0x086a, 0x0009},
	{0x1F, 0x0822, 0x11e1},
	{0x1F, 0x08f9, 0x1f11},
	{0x1F, 0x08a3, 0x24e8},
	{0x1F, 0x018C, 0x0001},
	{0x1F, 0x001F, 0x4000},
	{0x1F, 0x0573, 0x0001},
	{0x1F, 0x056A, 0x5F41},
};

static const struct DP83TG720_init_reg DP83TG721_cs1_slave_init[] = {
	{0x1F, 0x001F, 0x8000},
	{0x1F, 0x0573, 0x0801},
	{0x1, 0x0834, 0x8001},
	{0x1F, 0x0405, 0X6C00},
	{0x1F, 0x08AD, 0x3C51},
	{0x1F, 0x0894, 0x5DF7},
	{0x1F, 0x08A0, 0x09E7},
	{0x1F, 0x08C0, 0x4000},
	{0x1F, 0x0814, 0x4800},
	{0x1F, 0x080D, 0x2EBF},
	{0x1F, 0x08C1, 0x0B00},
	{0x1F, 0x087D, 0x0001},
	{0x1F, 0x082E, 0x0000},
	{0x1F, 0x0837, 0x00F8},
	{0x1F, 0x08BE, 0x0200},
	{0x1F, 0x08C5, 0x4000},
	{0x1F, 0x08C7, 0x2000},
	{0x1F, 0x08B3, 0x005A},
	{0x1F, 0x08B4, 0x005A},
	{0x1F, 0x08B0, 0x0202},
	{0x1F, 0x08B5, 0x00EA},
	{0x1F, 0x08BA, 0x2828},
	{0x1F, 0x08BB, 0x6828},
	{0x1F, 0x08BC, 0x0028},
	{0x1F, 0x08BF, 0x0000},
	{0x1F, 0x08B1, 0x0014},
	{0x1F, 0x08B2, 0x0008},
	{0x1F, 0x08EC, 0x0000},
	{0x1F, 0x08FC, 0x0091},
	{0x1F, 0x08BE, 0x0201},
	{0x1F, 0x0456, 0x0160},
	{0x1F, 0x0335, 0x0010},
	{0x1F, 0x0336, 0x0009},
	{0x1F, 0x0337, 0x0208},
	{0x1F, 0x0338, 0x0208},
	{0x1F, 0x0339, 0x02CB},
	{0x1F, 0x033A, 0x0208},
	{0x1F, 0x033B, 0x0109},
	{0x1F, 0x0418, 0x0380},
	{0x1F, 0x0420, 0xFF10},
	{0x1F, 0x0421, 0x4033},
	{0x1F, 0x0422, 0x0800},
	{0x1F, 0x0423, 0x0002},
	{0x1F, 0x0484, 0x0003},
	{0x1F, 0x055D, 0x0008},
	{0x1F, 0x042B, 0x0018},
	{0x1F, 0x082D, 0x120F},
	{0x1F, 0x0888, 0x0438},
	{0x1F, 0x0824, 0x09E0},
	{0x1F, 0x0883, 0x5146},
	{0x1F, 0x08BE, 0x02A1},
	{0x1F, 0x0822, 0x11E1},
	{0x1F, 0x056A, 0x5F40},
	{0x1F, 0x08C1, 0x0900},
	{0x1F, 0x08FC, 0x4091},
	{0x1F, 0x08F9, 0x1F11},
	{0x1F, 0x084F, 0x290C},
	{0x1F, 0x0850, 0x3D33},
	{0x1F, 0x018C, 0x0001},
	{0x1F, 0x001F, 0x4000},
	{0x1F, 0x0573, 0x0001},
	{0x1F, 0x056A, 0x5F41},
};

#if 0
static const struct DP83TG720_init_reg DP83TG720_tdr_config_init[] = {
	{0x1F, 0x301, 0xA008},
	{0x1F, 0x303, 0x0928},
	{0x1F, 0x304, 0x0004},
	{0x1F, 0x405, 0x6400},
};
#endif

struct dp83tg720_private {
	int chip;
	bool is_master;
	bool is_rgmii;
	bool is_sgmii;
	bool rx_shift;
	bool tx_shift;
};

static int dp83tg720_readext(struct phy_device *phydev, int addr, int devad, int reg)
{
	return phy_read_mmd(phydev, devad, reg);
}

static int dp83tg720_writeext(struct phy_device *phydev, int addr, int devad, int reg, u16 val)
{
	return phy_write_mmd(phydev, devad, reg, val);
}

static int dp83tg720_read_straps(struct phy_device *phydev)
{
	struct dp83tg720_private *dp83tg720 = phydev->priv;
	int strap;

	strap = phy_read_mmd(phydev, MMD1F, DP83TG720_STRAP);
	if (strap < 0)
		return strap;

	if (strap & DP83TG720_MASTER_MODE)
		dp83tg720->is_master = true;

	if (strap & DP83TG720_RGMII_IS_EN)
		dp83tg720->is_rgmii = true;

	if (strap & DP83TG720_SGMII_IS_EN)
		dp83tg720->is_sgmii = true;

	if (strap & DP83TG720_RX_SHIFT_EN)
		dp83tg720->rx_shift = true;

	if (strap & DP83TG720_TX_SHIFT_EN)
		dp83tg720->tx_shift = true;

	return 0;
};

static int dp83tg720_reset(struct phy_device *phydev, bool hw_reset)
{
	int ret;

	if (hw_reset)
		ret = phy_write_mmd(phydev, MMD1F, MII_DP83TG720_RESET_CTRL,
				DP83TG720_HW_RESET);
	else
		ret = phy_write_mmd(phydev, MMD1F, MII_DP83TG720_RESET_CTRL,
				DP83TG720_SW_RESET);
	if (ret)
		return ret;

	mdelay(100);

	return 0;
}
static int dp83tg720_write_seq(struct phy_device *phydev,
			     const struct DP83TG720_init_reg *init_data, int size)
{
	int ret;
	int i;

	for (i = 0; i < size; i++) {
			ret = phy_write_mmd(phydev, init_data[i].MMD, init_data[i].reg,
				init_data[i].val);
			if (ret)
					return ret;
	}

	return 0;
}

static int dp83tg720_chip_init(struct phy_device *phydev)
{
	struct dp83tg720_private *dp83tg720 = phydev->priv;
	int ret;

	ret = dp83tg720_reset(phydev, true);
	if (ret)
		return ret;
	
	phydev->autoneg = AUTONEG_DISABLE;
    	phydev->speed = SPEED_1000;
	phydev->duplex = DUPLEX_FULL;
  phydev->supported |= SUPPORTED_1000baseT_Full;
#if 0
    	linkmode_set_bit(ETHTOOL_LINK_MODE_1000baseT_Full_BIT,
                              phydev->supported);
#endif

	if (dp83tg720->is_master)
	        ret = phy_write_mmd(phydev, MMD1, 0x0834,
				0xc001);
	else
	        ret = phy_write_mmd(phydev, MMD1, 0x0834,
				0x8001);
	if (ret)
		return ret;

	switch (dp83tg720->chip) {
	case DP83TG720_CS1_1:
		ret = phy_write_mmd(phydev, MMD1F, 0x573, 0x101);
		if (ret)
			return ret;

		if (dp83tg720->is_master)
			ret = dp83tg720_write_seq(phydev, DP83TG720_cs1_1_master_init,
						ARRAY_SIZE(DP83TG720_cs1_1_master_init));
		else
			ret = dp83tg720_write_seq(phydev, DP83TG720_cs1_1_slave_init,
						ARRAY_SIZE(DP83TG720_cs1_1_slave_init));

		ret = dp83tg720_reset(phydev, false);

		ret = phy_write_mmd(phydev, MMD1F, 0x573, 0x001);
	        if (ret)
	                return ret;

		return phy_write_mmd(phydev, MMD1F, 0x56a, 0x5f41);
	case DP83TG721_CS1:
		ret = phy_write_mmd(phydev, MMD1F, 0x573, 0x101);
		if (ret)
			return ret;

		if (dp83tg720->is_master)
			ret = dp83tg720_write_seq(phydev, DP83TG721_cs1_master_init,
						ARRAY_SIZE(DP83TG721_cs1_master_init));
		else
			ret = dp83tg720_write_seq(phydev, DP83TG721_cs1_slave_init,
						ARRAY_SIZE(DP83TG721_cs1_slave_init));

		ret = dp83tg720_reset(phydev, false);

		ret = phy_write_mmd(phydev, MMD1F, 0x573, 0x001);
	        if (ret)
	                return ret;

		return phy_write_mmd(phydev, MMD1F, 0x56a, 0x5f41);
	default:
		return -EINVAL;
	};

	if (ret)
		return ret;

	/* Enable the PHY */
	ret = phy_write_mmd(phydev, MMD1F, 0x18c, 0x1);
	if (ret)
		return ret;

	mdelay(10);

	/* Do a software reset to restart the PHY with the updated values */
	return dp83tg720_reset(phydev, false);
}

static int dp83tg720_config_rgmii_delay(struct phy_device *phydev, bool rx_shift, bool tx_shift)
{
	int value, ret;

  value = phy_read_mmd(phydev, MMD1F, DP83TG720_SGMII_CTRL);

  if (value)
    return value;

  if (!rx_shift)
    value &= ~DP83TG720_TX_CLK_SHIFT;
  else
    value |= DP83TG720_TX_CLK_SHIFT;
  
  if (!tx_shift)
    value &= ~DP83TG720_RX_CLK_SHIFT;
  else
    value |= DP83TG720_RX_CLK_SHIFT;
	
  ret = phy_write_mmd(phydev, MMD1F, DP83TG720_SGMII_CTRL, value);
  return ret;
}

static int dp83tg720_config_init(struct phy_device *phydev)
{
	struct dp83tg720_private *dp83tg720 = phydev->priv;
	int value, ret;

  ret = dp83tg720_chip_init(phydev);
	if (ret)
		return ret;

	if (phy_interface_is_rgmii(phydev)) {
    ret = dp83tg720_config_rgmii_delay(phydev, dp83tg720->tx_shift, 
        dp83tg720->rx_shift);
			if (ret)
				return ret;
	}

	value = phy_read_mmd(phydev, MMD1F, DP83TG720_SGMII_CTRL);
	if (value < 0)
		return value;

	if (phydev->interface == PHY_INTERFACE_MODE_SGMII)
		value |= DP83TG720_SGMII_EN;
	else
		value &= ~DP83TG720_SGMII_EN;

	ret = phy_write_mmd(phydev, MMD1F, DP83TG720_SGMII_CTRL, value);
	if (ret < 0)
		return ret;

	return 0;
}

static int dp83tg720_config(struct phy_device *phydev)
{
	struct dp83tg720_private *dp83tg720;
	int ret;

	ret = dp83tg720_read_straps(phydev);
	if (ret)
		return ret;

  dp83tg720 = (struct dp83tg720_private *)phydev->priv;

	switch (phydev->phy_id) {
	case DP83TG720_CS_1_1_PHY_ID:
		dp83tg720->chip = DP83TG720_CS1_1;
		break;
	case DP83TG721_CS_1_0_PHY_ID:
		dp83tg720->chip = DP83TG721_CS1;
		break;
	default:
		return -EINVAL;
	};

	return dp83tg720_config_init(phydev);
}

static int dp83tg720_probe(struct phy_device *phydev)
{
	struct dp83tg720_private *dp83tg720;

  log_debug("dp83tg720_probe() invoked...\n");

	dp83tg720 = kzalloc(sizeof(*dp83tg720), GFP_KERNEL);
	if (!dp83tg720)
		return -ENOMEM;

	phydev->priv = dp83tg720;
  return 0;
}

U_BOOT_PHY_DRIVER(dp83867) = {
	.name = "TI DP83TG720S",
	.uid = 0x2000a284,
	.mask = 0xfffffff0,
	.features = PHY_GBIT_FEATURES,
	.probe = &dp83tg720_probe,
	.config = &dp83tg720_config,
	.startup = &genphy_startup,
	.shutdown = &genphy_shutdown,
	.readext = dp83tg720_readext,
	.writeext = dp83tg720_writeext
};
