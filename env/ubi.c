// SPDX-License-Identifier: GPL-2.0+
/*
 * (c) Copyright 2012 by National Instruments,
 *        Joe Hershberger <joe.hershberger@ni.com>
 */

#include <common.h>
#include <asm/global_data.h>

#include <command.h>
#include <env.h>
#include <env_internal.h>
#include <errno.h>
#include <malloc.h>
#include <memalign.h>
#include <search.h>
#include <ubi_uboot.h>
#undef crc32

#define _QUOTE(x) #x
#define QUOTE(x) _QUOTE(x)

#if (CONFIG_ENV_UBI_VID_OFFSET == 0)
 #define UBI_VID_OFFSET NULL
#else
 #define UBI_VID_OFFSET QUOTE(CONFIG_ENV_UBI_VID_OFFSET)
#endif

DECLARE_GLOBAL_DATA_PTR;

static const char *get_env_active_slot(void)
{
	static char slot[2];

	if (!gd->env_active_slot)
		return NULL;

	char *ptr = (char *)gd->env_active_slot;

	switch (*ptr) {
		case 'a':
		case 'b':
			slot[0] = *ptr;
			slot[1] = '\0';
			break;
		default:
			printf("env_active_slot raw: 0x%lx\n",
					(unsigned long)gd->env_active_slot);
			printf("slot addr: 0x%lx, value: 0x%02x\n",
					(unsigned long)ptr, *ptr);
			printf("Invalid slot value: 0x%02x\n", *ptr);

			slot[0] = '?';
			slot[1] = '\0';
	}

	return slot;
}

static const char *get_env_ubipart_name(void)
{
	static char ubipart_name[16];
	const char *slot;

	if (!gd->env_active_slot)
		return CONFIG_ENV_UBI_PART;

	slot = get_env_active_slot();
	if (!slot || slot[0] == '?')
		return CONFIG_ENV_UBI_PART;

	snprintf(ubipart_name, sizeof(ubipart_name),
			"%s_%s", CONFIG_ENV_UBI_PART, slot);

	return ubipart_name;
}

#if CONFIG_SYS_REDUNDAND_ENVIRONMENT
#define ENV_UBI_VOLUME_REDUND CONFIG_ENV_UBI_VOLUME_REDUND
#else
#define ENV_UBI_VOLUME_REDUND "invalid"
#endif

#ifdef CONFIG_CMD_SAVEENV
#ifdef CONFIG_SYS_REDUNDAND_ENVIRONMENT

static int env_ubi_save(void)
{
	ALLOC_CACHE_ALIGN_BUFFER(env_t, env_new, 1);
	int ret;

	ret = env_export(env_new);
	if (ret)
		return ret;

	if (ubi_part(get_env_ubipart_name(), UBI_VID_OFFSET)) {
		printf("\n** Cannot find mtd partition \"%s\"\n",
		       get_env_ubipart_name());
		return 1;
	}

	if (gd->env_valid == ENV_VALID) {
		puts("Writing to redundant UBI... ");
		if (ubi_volume_write(CONFIG_ENV_UBI_VOLUME_REDUND,
				     (void *)env_new, CONFIG_ENV_SIZE)) {
			printf("\n** Unable to write env to %s:%s **\n",
			       get_env_ubipart_name(),
			       CONFIG_ENV_UBI_VOLUME_REDUND);
			return 1;
		}
	} else {
		puts("Writing to UBI... ");
		if (ubi_volume_write(CONFIG_ENV_UBI_VOLUME,
				     (void *)env_new, CONFIG_ENV_SIZE)) {
			printf("\n** Unable to write env to %s:%s **\n",
			       get_env_ubipart_name(),
			       CONFIG_ENV_UBI_VOLUME);
			return 1;
		}
	}

	puts("done\n");

	gd->env_valid = gd->env_valid == ENV_REDUND ? ENV_VALID : ENV_REDUND;

	return 0;
}
#else /* ! CONFIG_SYS_REDUNDAND_ENVIRONMENT */
static int env_ubi_save(void)
{
	ALLOC_CACHE_ALIGN_BUFFER(env_t, env_new, 1);
	int ret;

	ret = env_export(env_new);
	if (ret)
		return ret;

	if (ubi_part(get_env_ubipart_name(), UBI_VID_OFFSET)) {
		printf("\n** Cannot find mtd partition \"%s\"\n",
		       get_env_ubipart_name());
		return 1;
	}

	if (ubi_volume_write(CONFIG_ENV_UBI_VOLUME, (void *)env_new,
			     CONFIG_ENV_SIZE)) {
		printf("\n** Unable to write env to %s:%s **\n",
		       get_env_ubipart_name(), CONFIG_ENV_UBI_VOLUME);
		return 1;
	}

	puts("done\n");
	return 0;
}
#endif /* CONFIG_SYS_REDUNDAND_ENVIRONMENT */
#endif /* CONFIG_CMD_SAVEENV */

#ifdef CONFIG_SYS_REDUNDAND_ENVIRONMENT
static int env_ubi_load(void)
{
	ALLOC_CACHE_ALIGN_BUFFER(char, env1_buf, CONFIG_ENV_SIZE);
	ALLOC_CACHE_ALIGN_BUFFER(char, env2_buf, CONFIG_ENV_SIZE);
	int read1_fail, read2_fail;
	env_t *tmp_env1, *tmp_env2;
	int ret = 0;

	/*
	 * In case we have restarted u-boot there is a chance that buffer
	 * contains old environment (from the previous boot).
	 * If UBI volume is zero size, ubi_volume_read() doesn't modify the
	 * buffer.
	 * We need to clear buffer manually here, so the invalid CRC will
	 * cause setting default environment as expected.
	 */
	memset(env1_buf, 0x0, CONFIG_ENV_SIZE);
	memset(env2_buf, 0x0, CONFIG_ENV_SIZE);

	tmp_env1 = (env_t *)env1_buf;
	tmp_env2 = (env_t *)env2_buf;

	if (ubi_part(get_env_ubipart_name(), UBI_VID_OFFSET)) {
		printf("\n** Cannot find mtd partition \"%s\"\n",
		       get_env_ubipart_name());
		env_set_default(NULL, 0);
		return -EIO;
	}

	read1_fail = ubi_volume_read(CONFIG_ENV_UBI_VOLUME, (void *)tmp_env1,
				     CONFIG_ENV_SIZE);
	if (read1_fail)
		printf("\n** Unable to read env from %s:%s **\n",
		       get_env_ubipart_name(), CONFIG_ENV_UBI_VOLUME);

	read2_fail = ubi_volume_read(CONFIG_ENV_UBI_VOLUME_REDUND,
				     (void *)tmp_env2, CONFIG_ENV_SIZE);
	if (read2_fail)
		printf("\n** Unable to read redundant env from %s:%s **\n",
		       get_env_ubipart_name(), CONFIG_ENV_UBI_VOLUME_REDUND);

	ret = env_import_redund((char *)tmp_env1, read1_fail, (char *)tmp_env2,
				 read2_fail, H_EXTERNAL);
	if (ret)
		return ret;

	return gd->env_active_slot != 0 ?
		env_set("active_slot", get_env_active_slot()) : ret;
}
#else /* ! CONFIG_SYS_REDUNDAND_ENVIRONMENT */
static int env_ubi_load(void)
{
	ALLOC_CACHE_ALIGN_BUFFER(char, buf, CONFIG_ENV_SIZE);
	int ret = 0;

	/*
	 * In case we have restarted u-boot there is a chance that buffer
	 * contains old environment (from the previous boot).
	 * If UBI volume is zero size, ubi_volume_read() doesn't modify the
	 * buffer.
	 * We need to clear buffer manually here, so the invalid CRC will
	 * cause setting default environment as expected.
	 */
	memset(buf, 0x0, CONFIG_ENV_SIZE);

	if (ubi_part(get_env_ubipart_name(), UBI_VID_OFFSET)) {
		printf("\n** Cannot find mtd partition \"%s\"\n",
		       get_env_ubipart_name());
		env_set_default(NULL, 0);
		return -EIO;
	}

	if (ubi_volume_read(CONFIG_ENV_UBI_VOLUME, buf, CONFIG_ENV_SIZE)) {
		printf("\n** Unable to read env from %s:%s **\n",
		       get_env_ubipart_name(), CONFIG_ENV_UBI_VOLUME);
		env_set_default(NULL, 0);
		return -EIO;
	}

	ret = env_import(buf, 1, H_EXTERNAL);
	if (ret)
		return ret;

	return gd->env_active_slot != 0 ?
		env_set("active_slot", get_env_active_slot()) : ret;
}
#endif /* CONFIG_SYS_REDUNDAND_ENVIRONMENT */

static int env_ubi_erase(void)
{
	ALLOC_CACHE_ALIGN_BUFFER(char, env_buf, CONFIG_ENV_SIZE);
	int ret = 0;

	if (ubi_part(get_env_ubipart_name(), UBI_VID_OFFSET)) {
		printf("\n** Cannot find mtd partition \"%s\"\n",
		       get_env_ubipart_name());
		return 1;
	}

	memset(env_buf, 0x0, CONFIG_ENV_SIZE);

	if (ubi_volume_write(CONFIG_ENV_UBI_VOLUME,
			     (void *)env_buf, CONFIG_ENV_SIZE)) {
		printf("\n** Unable to erase env to %s:%s **\n",
		       get_env_ubipart_name(),
		       CONFIG_ENV_UBI_VOLUME);
		ret = 1;
	}
	if (IS_ENABLED(CONFIG_SYS_REDUNDAND_ENVIRONMENT)) {
		if (ubi_volume_write(ENV_UBI_VOLUME_REDUND,
				     (void *)env_buf, CONFIG_ENV_SIZE)) {
			printf("\n** Unable to erase env to %s:%s **\n",
			       get_env_ubipart_name(),
			       ENV_UBI_VOLUME_REDUND);
			ret = 1;
		}
	}

	return ret;
}

U_BOOT_ENV_LOCATION(ubi) = {
	.location	= ENVL_UBI,
	ENV_NAME("UBI")
	.load		= env_ubi_load,
	.save		= env_save_ptr(env_ubi_save),
	.erase		= ENV_ERASE_PTR(env_ubi_erase),
};
