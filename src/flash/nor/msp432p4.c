/***************************************************************************
 *   Copyright (C) 2014 by Vlad Ungureanu                                  *
 *   vvu@vdev.ro                                                           *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "msp432p4.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/armv7m.h>
#include <target/image.h>

#define FLASH_FUNCTION			0x20000150
#define FLASH_MASS_ERASE		0x01
#define FLASH_SECTOR_ERASE		0x02
#define FLASH_PROGRAM			0x04
#define FLASH_INIT				0x08

#define RETURN_CODE				0x20000154
#define DEFAULT_CODE			0x00000DEF
#define ENTRY_POINT				0x01000180

#define SRC_ADDRESS				0x20000160
#define DST_ADDRESS				0x20000164
#define SRC_LENGTH				0x20000168

#define ERASE_PARAMETER			0x2000016C
#define ERASE_MAIN				0x01
#define ERASE_INFO				0x02
#define ERASE_MAIN_INFO			0x03

#define SRC_START				0x20002000
#define STC_MAX_LENGTH			4096

#define FLASH_BUSY				0x00000001
#define FLASH_SUCCESS			0x00000ACE
#define FLASH_ERROR				0x0000DEAD
#define FLASH_TIMEOUT_ERROR		0xDEAD0000
#define FLASH_VERIFY_ERROR		0xDEADDEAD
#define FLASH_WRONG_CMD			0x00000BAD
#define FLASH_PWR_ERR			0x00DEAD00

#define FLASH_SIZE_REG			0xE0043020
#define CPU_TYPE				0xE000ED00
#define DDDS_DEV_ID				0x00203008
#define TIMEOUT					2000

struct msp432p4_flash_bank {
	uint32_t register_base;
	uint32_t user_bank_size;
	int probed, initialized;
};

static int msp432p4_init(struct flash_bank *bank);

void hexDump (char *desc, const uint8_t *addr, int len) {
    int i;
    unsigned char buff[17];
    unsigned char *pc = (unsigned char*)addr;

    // Output description if given.
    if (desc != NULL)
        printf ("%s:\n", desc);

    // Process every byte in the data.
    for (i = 0; i < len; i++) {
        // Multiple of 16 means new line (with line offset).

        if ((i % 16) == 0) {
            // Just don't print ASCII for the zeroth line.
            if (i != 0)
                printf ("  %s\n", buff);

            // Output the offset.
            printf ("  %04x ", i);
        }

        // Now the hex code for the specific character.
        printf (" %02x", pc[i]);

        // And store a printable ASCII character for later.
        if ((pc[i] < 0x20) || (pc[i] > 0x7e))
            buff[i % 16] = '.';
        else
            buff[i % 16] = pc[i];
        buff[(i % 16) + 1] = '\0';
    }

    // Pad out last line if not exactly 16 characters.
    while ((i % 16) != 0) {
        printf ("   ");
        i++;
    }

    // And print the final ASCII bit.
    printf ("  %s\n", buff);
}


   FLASH_BANK_COMMAND_HANDLER(msp432p4_flash_bank_command)
{
	struct msp432p4_flash_bank *msp432p4_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	msp432p4_info = malloc(sizeof(struct msp432p4_flash_bank));

	bank->driver_priv = msp432p4_info;
	msp432p4_info->register_base = bank->base;
	msp432p4_info->user_bank_size = bank->size;
	msp432p4_info->probed = 0;
	msp432p4_info->initialized = 0;

	return ERROR_OK;
}

static char* msp432p4_decode_error(uint32_t error_code)
{
	switch (error_code) {
		case FLASH_BUSY:
			return "FLASH_BUSY";
		case FLASH_SUCCESS:
			return "FLASH_SUCCESS";
		case FLASH_ERROR:
			return "FLASH_ERROR";
		case FLASH_TIMEOUT_ERROR:
			return "FLASH_TIMEOUT_ERROR";
		case FLASH_VERIFY_ERROR:
			return "FLASH_VERIFY_WRONG";
		case FLASH_WRONG_CMD:
			return "FLASH_WRONG_CMD";
		case FLASH_PWR_ERR:
			return "FLASH_PWR_ERR";
	}
	return "UNDEFINED";
}

static int msp432p4_init_if_needed(struct flash_bank *bank)
{
	struct msp432p4_flash_bank *msp432p4_info = bank->driver_priv;
	if (!msp432p4_info->initialized)
	{
		int retval = msp432p4_init(bank);
		if (retval != ERROR_OK)
		{
			msp432p4_info->initialized = retval;
			return retval;
		}
		msp432p4_info->initialized = 1;
	}
	
	return ERROR_OK;
}

static int msp432p4_run_algo(struct flash_bank *bank, uint32_t function,
			uint32_t src_address, uint32_t dst_address,
			uint32_t src_len)
{
	struct target *target = bank->target;
	struct armv7m_algorithm armv7m_info;
	struct working_area *write_algorithm;
	int retval;

	if (target_alloc_working_area(target, sizeof(msp432p4_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	retval = target_write_buffer(target, write_algorithm->address,
			sizeof(msp432p4_flash_write_code), msp432p4_flash_write_code);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, FLASH_FUNCTION, function);
	if (retval != ERROR_OK)
		return retval;

	if (src_address > 0) {
		retval = target_write_u32(target, SRC_ADDRESS, src_address);
		if (retval != ERROR_OK)
			return retval;
	}

	if (function == FLASH_PROGRAM || function == FLASH_SECTOR_ERASE) {
		retval = target_write_u32(target, DST_ADDRESS, dst_address);
		if (retval != ERROR_OK)
			return retval;
	}

	if (src_len > 0) {
		retval = target_write_u32(target, SRC_LENGTH, src_len);
		if (retval != ERROR_OK)
			return retval;
	}

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	retval = target_run_algorithm(target, 0, NULL, 0, NULL,
				ENTRY_POINT, 0,
				TIMEOUT, &armv7m_info);

	target_free_working_area(target, write_algorithm);

	return retval;
}

static int msp432p4_init(struct flash_bank *bank)
{
	struct target *target = bank->target;
	uint32_t reg_value;
	char *error_name;
	int retval;

	retval = target_read_u32(target, DDDS_DEV_ID, &reg_value);
	if (retval != ERROR_OK)
		return retval;

	retval = msp432p4_run_algo(bank, FLASH_INIT, 0, 0, 0);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, RETURN_CODE, &reg_value);
	if (retval != ERROR_OK)
		return retval;

	if (reg_value != FLASH_SUCCESS) {
		error_name = msp432p4_decode_error(reg_value);
		LOG_ERROR("Cannot init flash controller: %s.", error_name);
	}

	retval = target_write_u32(target, RETURN_CODE, DEFAULT_CODE);
	if (retval != ERROR_OK)
		return retval;

	return retval;
}

static int msp432p4_mass_erase(struct flash_bank *bank)
{
	struct target *target = bank->target;
	uint32_t reg_value;
	char *error_name;
	int retval;

	retval = msp432p4_init_if_needed(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = msp432p4_run_algo(bank, FLASH_MASS_ERASE, 0, 0, 0);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, RETURN_CODE, &reg_value);
	if (retval != ERROR_OK)
		return retval;

	if (reg_value != FLASH_SUCCESS) {
		error_name = msp432p4_decode_error(reg_value);
		LOG_ERROR("Cannot mass erase: %s.", error_name);
	}

	retval = target_write_u32(target, RETURN_CODE, DEFAULT_CODE);
	if (retval != ERROR_OK)
		return retval;

	return retval;
}

static int msp432p4_sector_erase(struct flash_bank *bank, uint32_t dst_address)
{
	struct target *target = bank->target;
	uint32_t reg_value;
	char *error_name;
	int retval;

	retval = msp432p4_init_if_needed(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = msp432p4_run_algo(bank, FLASH_SECTOR_ERASE, 0, dst_address, 0);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, RETURN_CODE, &reg_value);
	if (retval != ERROR_OK)
		return retval;

	if (reg_value != FLASH_SUCCESS) {
		error_name = msp432p4_decode_error(reg_value);
		LOG_ERROR("Cannot erase sector at address: %s.", error_name);
	}

	retval = target_write_u32(target, RETURN_CODE, DEFAULT_CODE);
	if (retval != ERROR_OK)
		return retval;

	return retval;
}

COMMAND_HANDLER(msp432p4_handle_init_command)
{
	struct flash_bank *bank;
	int retval;

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	return msp432p4_init(bank);
}

COMMAND_HANDLER(msp432p4_handle_mass_erase_command)
{
	struct flash_bank *bank;
	int retval;

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	return msp432p4_mass_erase(bank);
}

COMMAND_HANDLER(msp432p4_handle_erase_sector)
{
	struct flash_bank *bank;
	int retval;
	uint32_t dst_address;

	if (CMD_ARGC < 2)
			return ERROR_COMMAND_SYNTAX_ERROR;

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], dst_address);

	return msp432p4_sector_erase(bank, dst_address);
}

static const struct command_registration msp432p4_exec_command_handlers[] = {
	{
		.name = "mass_erase",
		.handler = msp432p4_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.help = "Erase entire flash device.",
	},
	{
		.name = "init",
		.handler = msp432p4_handle_init_command,
		.mode = COMMAND_EXEC,
		.help = "Init flash for operations.",
	},
	{
		.name = "erase_sector",
		.handler = msp432p4_handle_erase_sector,
		.mode = COMMAND_EXEC,
		.help = "Erase a single sector from flash.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration msp432p4_command_handlers[] = {
	{
		.name = "msp432p4",
		.mode = COMMAND_ANY,
		.help = "msp432p4 flash command group",
		.usage = "Set of flashing commands for a msp432p4 target",
		.chain = msp432p4_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

static int msp432p4_erase(struct flash_bank *bank, int first, int last)
{
	struct target *target = bank->target;
	int retval = ERROR_OK, i;

	retval = msp432p4_init_if_needed(bank);
	if (retval != ERROR_OK)
		return retval;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if ((first == 0) && (last == (bank->num_sectors - 1))) {
		return msp432p4_mass_erase(bank);
	}

	for (i = first; i <= last; ++i) {
		retval = msp432p4_sector_erase(bank, bank->sectors[i].offset);
		if (retval != ERROR_OK)
			return retval;
	}

	return retval;
}

static int msp432p4_protect(struct flash_bank *bank, int set, int first, int last)
{
	/* Function not required. At startup all sectors are protected.
	 * Value will be reset at power on. */
	return ERROR_OK;
}

static int msp432p4_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct working_area *write_algorithm;
	uint32_t reg_value;
	char *error_name;
	struct armv7m_algorithm armv7m_info;
	int retval = ERROR_OK, size = 0, ofs = 0;

	retval = msp432p4_init_if_needed(bank);
	if (retval != ERROR_OK)
		return retval;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* flash write code */
	if (target_alloc_working_area(target, sizeof(msp432p4_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};

	retval = target_write_buffer(target, write_algorithm->address,
			sizeof(msp432p4_flash_write_code), msp432p4_flash_write_code);
	if (retval != ERROR_OK)
		return retval;

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	while (count) {
		if (count > 4096)
			size = 4096;
		else
			size = count;

		retval = target_write_buffer(target, SRC_START, size, buffer + ofs);
		if (retval != ERROR_OK) {
			LOG_ERROR("Unable to write block write code to target");
			return ERROR_FLASH_OPERATION_FAILED;
		}

		retval = target_write_u32(target, FLASH_FUNCTION, FLASH_PROGRAM);
		if (retval != ERROR_OK)
			return retval;

		retval = target_write_u32(target, SRC_ADDRESS, SRC_START);
		if (retval != ERROR_OK)
			return retval;

		retval = target_write_u32(target, DST_ADDRESS, ofs);
		if (retval != ERROR_OK)
			return retval;

		retval = target_write_u32(target, SRC_LENGTH, size);
		if (retval != ERROR_OK)
			return retval;
		count -= size;
		ofs += size;

		retval = target_run_algorithm(target, 0, NULL, 0, NULL,
					ENTRY_POINT, 0,
					TIMEOUT, &armv7m_info);
		if (retval == ERROR_FLASH_OPERATION_FAILED) {
			LOG_ERROR("Cannot flash!");
		}

		retval = target_read_u32(target, RETURN_CODE, &reg_value);
		if (retval != ERROR_OK)
			return retval;

		if (reg_value != FLASH_SUCCESS) {
			error_name = msp432p4_decode_error(reg_value);
			LOG_ERROR("Cannot write flash program: %s.", error_name);
		}
	}

	target_free_working_area(target, write_algorithm);

	return retval;
}

static int msp432p4_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct msp432p4_flash_bank *msp432p4_info = bank->driver_priv;
	uint32_t reg_value, cpu_id;
	uint32_t base_address = 0x00;
	uint32_t flash_size;
	int retval, i, number_sectors;

	retval = target_read_u32(target, CPU_TYPE, &cpu_id);
	if (retval != ERROR_OK)
		return retval;

	if (((cpu_id >> 4) & 0xFFF) != 0xC24)
		return ERROR_FAIL;

	retval = target_read_u32(target, FLASH_SIZE_REG, &reg_value);
	if (retval != ERROR_OK)
		return retval;

	number_sectors = reg_value / 4096;
	flash_size = reg_value;

	if (bank->sectors) {
		free(bank->sectors);
		bank->sectors = NULL;
	}

	bank->base = base_address;
	bank->size = flash_size;
	bank->num_sectors = number_sectors;
	bank->sectors = malloc(sizeof(struct flash_sector) * number_sectors);

	for (i = 0; i < number_sectors; i++) {
		bank->sectors[i].offset = i * 4096;
		bank->sectors[i].size = 4096;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 1;
	}

	msp432p4_info->probed = 1;

	return ERROR_OK;
}
static int msp432p4_auto_probe(struct flash_bank *bank)
{
	int probed = ((struct msp432p4_flash_bank *)bank->driver_priv)->probed;

	if (probed < 0)
		return probed;
	else if (probed)
		return ERROR_OK;
	else
		return msp432p4_probe(bank);
}

static int msp432p4_protect_check(struct flash_bank *bank)
{
	/* Function not required. At startup all sectors are protected. */
	return ERROR_OK;
}
static int get_msp432p4_info(struct flash_bank *bank, char *buf, int buf_size)
{
	return ERROR_OK;
}

struct flash_driver msp432p4_flash = {
	.name = "msp432p4",
	.commands = msp432p4_command_handlers,
	.flash_bank_command = msp432p4_flash_bank_command,
	.erase = msp432p4_erase,
	.protect = msp432p4_protect,
	.write = msp432p4_write,
	.read = default_flash_read,
	.probe = msp432p4_probe,
	.auto_probe = msp432p4_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = msp432p4_protect_check,
	.info = get_msp432p4_info,
};
