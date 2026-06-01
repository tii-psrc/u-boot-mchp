
#include <common.h>
#include <command.h>
#include <display_options.h>
#include <timestamp.h>
#include <linux/compiler.h>
#include <asm/io.h>


typedef enum {
	I_WR_DATA   = 0,
	I_WR_CTRL1  = 4,
	I_WR_CTRL2  = 8,
	I_WR_CTRL3  = 12,
	I_RD_DATA   = 0,
	I_RD_ST1    = 4,
	I_RD_ST2    = 8,
} IMUS_REGS;

typedef enum {
	I_CTRL_ENA_MASK         = 0x00000001,
	I_CTRL_START_OP_MASK    = 0x00000002,
	I_CTRL_DIV_MASK         = 0x000003FC,
	I_CTRL_nINT_MASK        = 0x00000400,
	I_CTRL_PS0_MASK         = 0x00000800,
	I_CTRL_nBOOT_MASK       = 0x00001000,
	I_CTRL_nRST_MASK        = 0x00002000,
	I_CTRL_USE_GPIO_MASK    = 0x00004000,
	I_CTRL_CE_MASK          = 0x00008000,
	I_CTRL_G_DOUT_MASK      = 0x00FF0000,
	I_CTRL_G_ENA_MASK       = 0xFF000000,
	I_CTRL2_CLK_LEVEL_MASK  = 0x00008000,
	I_CTRL3_ENA_IRQ_MASK    = 0x00000001
} IMU_CTRL;

typedef enum {
	I_CTRL_S_DIV            = 2,
} IMU_CTRL_SHIFT;

typedef enum {
	I_STS1_I_nRST           = 0x00002000,   //readout of the external signal
	I_STS1_I_PS0            = 0x00001000,   //readout of the external signal
	I_STS1_I_nBOOT          = 0x00000800,   //readout of the external signal
	I_STS1_I_nINT           = 0x00000400,   //readout of the external signal
	I_STS1_I_CE             = 0x00000200,   //readout of the external signal
	I_STS1_I_CLK            = 0x00000100,   //readout of the external signal
	I_STS1_I_MISO           = 0x00000080,   //readout of the external signal
	I_STS1_I_MOSI           = 0x00000040,   //readout of the external signal
	I_STS1_OVERUN           = 0x00000020,
	I_STS1_FT_FULL          = 0x00000010,
	I_STS1_FT_EMPTY         = 0x00000008,
	I_STS1_FR_FULL          = 0x00000004,
	I_STS1_FR_EMPTY         = 0x00000002,
	I_STS1_IDLE             = 0x00000001,
} IMU_STS1;

typedef enum {
	I_STS2_IDLE             = 0x80000000,
	I_STS2_SPARE1           = 0x40000000,
	I_STS2_FT_WRCNT         = 0x3F000000,
	I_STS2_FT_RDCNT         = 0x00FC0000,
	I_STS2_FT_EMPTY         = 0x00020000,
	I_STS2_FT_FULL          = 0x00010000,
	I_STS2_SPARE2           = 0x0000C000,
	I_STS2_FR_WRCNT         = 0x00003F00,
	I_STS2_FR_RDCNT         = 0x000000FC,
	I_STS2_FR_EMPTY         = 0x00000002,
	I_STS2_FR_FULL          = 0x00000001,
} IMU_STS2;


#define APB_BASE_ADDRESS    0x40000000UL
#define GPIOs_BASE_ADDRESS  (APB_BASE_ADDRESS + 0x0100L)

#define GPIOs0_BASE_ADDRESS (GPIOs_BASE_ADDRESS +  0)
#define GPIOs1_BASE_ADDRESS (GPIOs_BASE_ADDRESS + 16)
#define GPIOs2_BASE_ADDRESS (GPIOs_BASE_ADDRESS + 32)
#define GPIOs3_BASE_ADDRESS (GPIOs_BASE_ADDRESS + 48)

#define H18_E_PWR_IMU_N 11
#define F22_E_PWR_IMU_R 10

#define IMUs_BASE_ADDRESS   (APB_BASE_ADDRESS  + 0x0700L)
#define IMU_NOM_BASE_ADDRESS   (IMUs_BASE_ADDRESS + 0x0000L)
#define IMU_RED_BASE_ADDRESS   (IMUs_BASE_ADDRESS + 0x0040L)

enum scai_imu_id {
	SCAI_IMU_NOM = 0,
	SCAI_IMU_RED,
	SCAI_IMU_COUNT,
};

struct scai_navc_gpio {
	uint32_t gpio;
	uint32_t pin;
};

struct scai_imu_priv {
	const char *name;
	void __iomem *regs;
	phys_addr_t phys;
	uint32_t ctrl[3];
	struct scai_navc_gpio power_switch;
};

static int scai_navc_gpio_config(unsigned int gpio_num, unsigned int mask,
		unsigned int mode)
{
	volatile unsigned int *gpio_addr[4] = {
		(unsigned int *)GPIOs0_BASE_ADDRESS,
		(unsigned int *)GPIOs1_BASE_ADDRESS,
		(unsigned int *)GPIOs2_BASE_ADDRESS,
		(unsigned int *)GPIOs3_BASE_ADDRESS
	};
	unsigned int data = 0;

	data = *gpio_addr[gpio_num];
	printf("[pre]\tdata : 0x%08X @0x%p\n", data, (void *)gpio_addr[gpio_num]);

	if (mode == 0) //clear bit
		data &= ~mask;
	else if (mode == 1) //enabled bit
		data |= mask;
	else if (mode == 2) //read bit
		return (data &= mask);
	else
		printf("%s: unknown request (%d)...\n", __func__, mode);

	*gpio_addr[gpio_num] = data;

	data = *gpio_addr[gpio_num];
	printf("[post]\tdata : 0x%08X @0x%p\n", data, (void *)gpio_addr[gpio_num]);

	return 0;
}

static struct scai_imu_priv imu[SCAI_IMU_COUNT] = {
	[SCAI_IMU_NOM] = {
		.name = "nominal",
		.regs = NULL,
		.phys = IMU_NOM_BASE_ADDRESS,
		.ctrl = { 0, 0, 0 },
		.power_switch = { 1, H18_E_PWR_IMU_N },
	},
	[SCAI_IMU_RED] = {
		.name = "redundant",
		.regs = NULL,
		.phys = IMU_RED_BASE_ADDRESS,
		.ctrl = { 0, 0, 0 },
		.power_switch = { 1, F22_E_PWR_IMU_R },
	},
};

static int do_imu_init(struct cmd_tbl *cmdtp, int flag, int argc,
		char *const argv[])
{
	enum scai_imu_id id;

	if (argc < 2 || argc > 2)
		return CMD_RET_USAGE;

	id = (enum scai_imu_id)hextoul(argv[1], NULL);
	if (id != SCAI_IMU_NOM || id != SCAI_IMU_RED) {
		printf("Wrong id(%d) ...\n", id);
		return CMD_RET_USAGE;
	}

	do {
		scai_navc_gpio_config(imu[id].power_switch.gpio,
				BIT(imu[id].power_switch.pin), 0);

		imu[id].regs = phys_to_virt(imu[id].phys);
		imu[id].ctrl[0] = I_CTRL_ENA_MASK | (20<<I_CTRL_S_DIV) |
			I_CTRL_nINT_MASK|I_CTRL_PS0_MASK | I_CTRL_nBOOT_MASK  |
			I_CTRL_nRST_MASK | I_CTRL_CE_MASK;
		imu[id].ctrl[1] = I_CTRL2_CLK_LEVEL_MASK;
		imu[id].ctrl[2] = 0;

		scai_navc_gpio_config(imu[id].power_switch.gpio,
				BIT(imu[id].power_switch.pin), 1);

		writel(imu[id].ctrl[0], imu[id].regs + I_WR_CTRL1);
		writel(imu[id].ctrl[1], imu[id].regs + I_WR_CTRL2);
		writel(imu[id].ctrl[2], imu[id].regs + I_WR_CTRL3);
	} while (0);
	printf("%s init finished ...\n", imu[id].name);

	return CMD_RET_SUCCESS;
}

static int do_imu_list(struct cmd_tbl *cmdtp, int flag, int argc,
		       char *const argv[])
{
	int dev_nb = 0;

	printf("List of IMU devices:\n");
	for (dev_nb = 0; dev_nb < SCAI_IMU_COUNT; dev_nb++) {
		printf("[%d] %s(%#llx)\n", dev_nb, imu[dev_nb].name, imu[dev_nb].phys);
	}

	if (!dev_nb) {
		printf("No IMU device found\n");
		return CMD_RET_FAILURE;
	}

	return CMD_RET_SUCCESS;
}


static char imu_help_text[] =
	"- generic operations on imu devices\n\n"
	"imu list\n"
	"mtd init <dev>\n";

U_BOOT_CMD_WITH_SUBCMDS(imu, "IMU utils", imu_help_text,
		U_BOOT_SUBCMD_MKENT(list, 1, 1, do_imu_list),
		U_BOOT_SUBCMD_MKENT(init, 2, 0, do_imu_init));
