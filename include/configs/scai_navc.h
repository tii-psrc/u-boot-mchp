/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2019 Microchip Technology Inc.
 * Padmarao Begari <padmarao.begari@microchip.com>
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include <linux/sizes.h>

#define CFG_SYS_SDRAM_BASE       0x80000000
#define CFG_SYS_BAUDRATE_TABLE  { 115200, 921600, 1000000, 1500000 }

/* Environment options */

#if defined(CONFIG_CMD_DHCP)
#define BOOT_TARGET_DEVICES_DHCP(func)	func(DHCP, dhcp, na)
#else
#define BOOT_TARGET_DEVICES_DHCP(func)
#endif

#if defined(CONFIG_CMD_MMC)
#define BOOT_TARGET_DEVICES_MMC(func)	func(MMC, mmc, 0)
#else
#define BOOT_TARGET_DEVICES_MMC(func)
#endif

#if defined(CONFIG_CMD_UBIFS)
#define BOOT_TARGET_DEVICE_UBIFS(func)	func(UBIFS, ubifs, 0, ubi, rootfs)
#else
#define BOOT_TARGET_DEVICE_UBIFS(func)
#endif

#if defined(CONFIG_MPFS_PRIORITISE_QSPI_BOOT)
#define BOOT_TARGET_DEVICES(func) \
	BOOT_TARGET_DEVICE_UBIFS(func)	\
	BOOT_TARGET_DEVICES_MMC(func)\
	BOOT_TARGET_DEVICES_DHCP(func)
#else
#define BOOT_TARGET_DEVICES(func) \
	BOOT_TARGET_DEVICES_MMC(func)\
	BOOT_TARGET_DEVICES_DHCP(func)
#endif

#define BOOTENV_DESIGN_OVERLAYS \
	"design_overlays=" \
	"if test -n ${no_of_overlays}; then " \
		"setenv inc 1; " \
		"setenv idx 0; " \
		"fdt resize ${dtbo_size}; " \
		"while test $idx -ne ${no_of_overlays}; do " \
			"setenv dtbo_name dtbo_image${idx}; " \
			"setenv fdt_cmd \"fdt apply $\"$dtbo_name; " \
			"run fdt_cmd; " \
			"setexpr idx $inc + $idx; " \
		"done; " \
	"fi;\0 " \

#define BOOTENV_NAVC \
	"bootargs_base=earlycon console=ttyS1,1500000n8 " \
		"uio_pdrv_genirq.of_id=generic-uio " \
		"root=ubi0:rootfs rootfstype=ubifs rootwait rw\0" \
	"set_bootargs=setenv bootargs ${bootargs_base} ubi.mtd=ubi_${active_slot}\0" \
	"get_inactive=" \
		"if test \"${active_slot}\" = \"a\"; then inactive_slot=b; " \
		"else inactive_slot=a; fi; " \
		"setenv active_slot ${inactive_slot}; \0" \
	"boot_scripts=boot.scr\0" \
	"boot_prefixes=/ /boot/\0" \
	"boot_a_script=" \
		"load ${devtype} ${devnum}:${distro_bootpart} ${scriptaddr} ${prefix}${script}; " \
		"source ${scriptaddr}\0" \
	"scan_dev_for_scripts=" \
		"for script in ${boot_scripts}; do " \
			"if test -e ${devtype} ${devnum}:${distro_bootpart} ${prefix}${script}; then " \
				"echo Found U-Boot script ${prefix}${script}; " \
				"run boot_a_script; " \
				"echo SCRIPT FAILED: continuing...; " \
			"fi; " \
		"done\0" \
	"scan_dev_for_boot=" \
		"echo Scanning ${devtype} ${devnum}:${distro_bootpart}...; " \
		"for prefix in ${boot_prefixes}; do run scan_dev_for_scripts; done;\0" \
	"ubifs_boot=" \
		"if ubi part ${bootubipart} ${bootubioff} && " \
		   "ubifsmount ubi0:${bootubivol}; then " \
			"devtype=ubi; devnum=ubi0; bootfstype=ubifs; " \
			"distro_bootpart=${bootubivol}; " \
			"run scan_dev_for_boot; " \
			"ubifsumount; " \
		"fi\0" \
	"bootcmd_ubifs_slot=" \
		"echo Booting slot ${slot}; " \
		"bootubipart=ubi_${slot}; bootubivol=rootfs; bootubioff=; " \
		"run ubifs_boot;\0" \
	"default_active_slot=a\0" \
	"ensure_active_slot=" \
		"if env exists active_slot; then " \
			"echo \"Using active_slot from HSS: ${active_slot}\"; " \
		"else " \
			"echo \"active_slot not set, using default\"; " \
			"setenv active_slot ${default_active_slot}; " \
		"fi; " \
		"if test \"${active_slot}\" != \"a\" && test \"${active_slot}\" != \"b\"; then " \
			"echo \"active_slot invalid, using default\"; " \
			"setenv active_slot ${default_active_slot}; " \
		"fi\0" \
	"bootcmd_ubifs=" \
		"run ensure_active_slot; " \
		"run set_bootargs; " \
		"echo === [BOOT] ACTIVE SLOT:${active_slot} ===; " \
		"slot=${active_slot}; " \
		"run bootcmd_ubifs_slot; " \
		"echo === [BOOT] FAILOVER ===; " \
		"run get_inactive; " \
		"run set_bootargs; " \
		"slot=${inactive_slot}; " \
		"run bootcmd_ubifs_slot;\0" \
	"boot_targets=ubifs\0" \
	"distro_bootcmd=for target in ${boot_targets}; do run bootcmd_${target}; done\0" \
	"bootcmd=run distro_bootcmd\0" \

#if !defined(CONFIG_FIT_SIGNATURE)
#include <config_distro_bootcmd.h>

#define CFG_EXTRA_ENV_SETTINGS \
	"bootm_size=0x10000000\0" \
	"kernel_addr_r=0x80200000\0" \
	"fdt_addr_r=0x8a000000\0" \
	"fdtoverlay_addr_r=0x8a080000\0" \
	"ramdisk_addr_r=0x8aa00000\0" \
	"scriptaddr=0x8e000000\0" \
	BOOTENV_NAVC \

#endif
#endif /* __CONFIG_H */
