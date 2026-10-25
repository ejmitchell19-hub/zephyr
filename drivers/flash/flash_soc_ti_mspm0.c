/*
 * Copyright (c) 2026 Texas Instruments
 * Copyright (c) 2026 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * MSPM0 main flash driver on the DriverLib flash controller calls (hal_ti).
 * Based on the pull request zephyrproject-rtos/zephyr#91500 (Linumiz, TI), changed to:
 *  - a mutex for the driver lock,
 *  - polling instead of the flash "done" interrupt (the flash controller node in
 *    main has no interrupts property),
 *  - clear the command status before every command and check the result (the
 *    original returned success whatever the controller reported),
 *  - take the memory range from the zephyr,flash chosen node, because boards with
 *    two banks (G3519) have several soc-nv-flash children under the controller.
 *
 * Scope: this instance serves the one soc-nv-flash node that zephyr,flash names (the
 * bank the image runs from, flash0). A second bank (flash1) or the data flash (flash2)
 * is not exposed as a second flash device.
 */

#define DT_DRV_COMPAT ti_mspm0_flash_controller

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <string.h>
#include <driverlib/dl_flashctl.h>

LOG_MODULE_REGISTER(flash_ti_mspm0, CONFIG_FLASH_LOG_LEVEL);

#define MSPM0_FLASH_NODE DT_CHOSEN(zephyr_flash)

BUILD_ASSERT(DT_NODE_HAS_COMPAT(MSPM0_FLASH_NODE, soc_nv_flash),
	     "zephyr,flash must be a soc-nv-flash node under the MSPM0 flash controller");
BUILD_ASSERT(DT_SAME_NODE(DT_PARENT(MSPM0_FLASH_NODE), DT_DRV_INST(0)),
	     "zephyr,flash must be a child of the MSPM0 flash controller");

#define MSPM0_FLASH_SIZE         DT_REG_SIZE(MSPM0_FLASH_NODE)
#define MSPM0_FLASH_BASE_ADDRESS DT_REG_ADDR(MSPM0_FLASH_NODE)
#define MSPM0_FLASH_ERASE_SIZE   DT_PROP(MSPM0_FLASH_NODE, erase_block_size)
#define MSPM0_FLASH_WRITE_SIZE   DT_PROP(MSPM0_FLASH_NODE, write_block_size)

BUILD_ASSERT(MSPM0_FLASH_WRITE_SIZE == 8, "The flash programs 64 bit words with generated ECC");

struct flash_ti_mspm0_config {
	FLASHCTL_Regs *regs;
	struct flash_parameters parameters;
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	struct flash_pages_layout layout;
#endif
};

struct flash_ti_mspm0_data {
	struct k_mutex lock;
};

static bool flash_ti_mspm0_valid_range(off_t offset, size_t len)
{
	return (offset >= 0) && (offset <= MSPM0_FLASH_SIZE) &&
	       (len <= (size_t)(MSPM0_FLASH_SIZE - offset));
}

/* Wait for the command the caller started and report what the controller says. */
static int flash_ti_mspm0_finish(const struct flash_ti_mspm0_config *cfg)
{
	if (!DL_FlashCTL_waitForCmdDone(cfg->regs)) {
		LOG_ERR("Flash command failed, status %d, fail type 0x%x",
			(int)DL_FlashCTL_getCommandStatus(cfg->regs),
			(unsigned int)DL_FlashCTL_getFailureStatus(cfg->regs));
		return -EIO;
	}

	return 0;
}

static int flash_ti_mspm0_erase(const struct device *dev, off_t offset, size_t len)
{
	const struct flash_ti_mspm0_config *cfg = dev->config;
	struct flash_ti_mspm0_data *data = dev->data;
	int ret = 0;

	if (len == 0) {
		return 0;
	}

	if (!flash_ti_mspm0_valid_range(offset, len)) {
		LOG_ERR("Erase range invalid. Offset %ld, len %zu", (long)offset, len);
		return -EINVAL;
	}

	if (!IS_ALIGNED(offset, MSPM0_FLASH_ERASE_SIZE) ||
	    !IS_ALIGNED(len, MSPM0_FLASH_ERASE_SIZE)) {
		LOG_ERR("Offset and length must be aligned to the %d byte sector",
			MSPM0_FLASH_ERASE_SIZE);
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	while (len != 0) {
		uint32_t address = MSPM0_FLASH_BASE_ADDRESS + offset;

		DL_FlashCTL_executeClearStatus(cfg->regs);
		DL_FlashCTL_unprotectSector(cfg->regs, address, DL_FLASHCTL_REGION_SELECT_MAIN);
		DL_FlashCTL_eraseMemory(cfg->regs, address, DL_FLASHCTL_COMMAND_SIZE_SECTOR);

		ret = flash_ti_mspm0_finish(cfg);
		if (ret != 0) {
			break;
		}

		offset += MSPM0_FLASH_ERASE_SIZE;
		len -= MSPM0_FLASH_ERASE_SIZE;
	}

	k_mutex_unlock(&data->lock);

	return ret;
}

static int flash_ti_mspm0_write(const struct device *dev, off_t offset, const void *buf, size_t len)
{
	const struct flash_ti_mspm0_config *cfg = dev->config;
	struct flash_ti_mspm0_data *data = dev->data;
	const uint8_t *src = buf;
	uint32_t word[MSPM0_FLASH_WRITE_SIZE / sizeof(uint32_t)] __aligned(4);
	int ret = 0;

	if (len == 0) {
		return 0;
	}

	if (!flash_ti_mspm0_valid_range(offset, len)) {
		LOG_ERR("Write range invalid. Offset %ld, len %zu", (long)offset, len);
		return -EINVAL;
	}

	if (!IS_ALIGNED(offset, MSPM0_FLASH_WRITE_SIZE) ||
	    !IS_ALIGNED(len, MSPM0_FLASH_WRITE_SIZE)) {
		LOG_ERR("Offset and length must be aligned to the %d byte write block",
			MSPM0_FLASH_WRITE_SIZE);
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	while (len != 0) {
		uint32_t address = MSPM0_FLASH_BASE_ADDRESS + offset;

		/* The source may sit in flash or be unaligned: program from a RAM copy. */
		memcpy(word, src, MSPM0_FLASH_WRITE_SIZE);

		DL_FlashCTL_executeClearStatus(cfg->regs);
		DL_FlashCTL_unprotectSector(cfg->regs, address, DL_FLASHCTL_REGION_SELECT_MAIN);
		DL_FlashCTL_programMemory64WithECCGenerated(cfg->regs, address, word);

		ret = flash_ti_mspm0_finish(cfg);
		if (ret != 0) {
			break;
		}

		offset += MSPM0_FLASH_WRITE_SIZE;
		src += MSPM0_FLASH_WRITE_SIZE;
		len -= MSPM0_FLASH_WRITE_SIZE;
	}

	k_mutex_unlock(&data->lock);

	return ret;
}

static int flash_ti_mspm0_read(const struct device *dev, off_t offset, void *buf, size_t len)
{
	ARG_UNUSED(dev);

	if (len == 0) {
		return 0;
	}

	if (!flash_ti_mspm0_valid_range(offset, len)) {
		LOG_ERR("Read range invalid. Offset %ld, len %zu", (long)offset, len);
		return -EINVAL;
	}

	memcpy(buf, (const void *)(MSPM0_FLASH_BASE_ADDRESS + offset), len);

	return 0;
}

static const struct flash_parameters *flash_ti_mspm0_get_parameters(const struct device *dev)
{
	const struct flash_ti_mspm0_config *cfg = dev->config;

	return &cfg->parameters;
}

static int flash_ti_mspm0_get_size(const struct device *dev, uint64_t *size)
{
	ARG_UNUSED(dev);

	*size = MSPM0_FLASH_SIZE;

	return 0;
}

#ifdef CONFIG_FLASH_PAGE_LAYOUT
static void flash_ti_mspm0_page_layout(const struct device *dev,
				       const struct flash_pages_layout **layout,
				       size_t *layout_size)
{
	const struct flash_ti_mspm0_config *cfg = dev->config;

	*layout = &cfg->layout;
	*layout_size = 1;
}
#endif

static DEVICE_API(flash, flash_ti_mspm0_api) = {
	.erase = flash_ti_mspm0_erase,
	.write = flash_ti_mspm0_write,
	.read = flash_ti_mspm0_read,
	.get_parameters = flash_ti_mspm0_get_parameters,
	.get_size = flash_ti_mspm0_get_size,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = flash_ti_mspm0_page_layout,
#endif
};

static int flash_ti_mspm0_init(const struct device *dev)
{
	struct flash_ti_mspm0_data *data = dev->data;

	k_mutex_init(&data->lock);

	return 0;
}

static const struct flash_ti_mspm0_config flash_ti_mspm0_config0 = {
	.regs = (FLASHCTL_Regs *)DT_INST_REG_ADDR(0),
	.parameters = {
		.write_block_size = MSPM0_FLASH_WRITE_SIZE,
		.erase_value = 0xff,
	},
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.layout = {
		.pages_count = MSPM0_FLASH_SIZE / MSPM0_FLASH_ERASE_SIZE,
		.pages_size = MSPM0_FLASH_ERASE_SIZE,
	},
#endif
};

static struct flash_ti_mspm0_data flash_ti_mspm0_data0;

DEVICE_DT_INST_DEFINE(0, flash_ti_mspm0_init, NULL, &flash_ti_mspm0_data0,
		      &flash_ti_mspm0_config0, POST_KERNEL, CONFIG_FLASH_INIT_PRIORITY,
		      &flash_ti_mspm0_api);

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1, "One MSPM0 flash controller per SoC");
