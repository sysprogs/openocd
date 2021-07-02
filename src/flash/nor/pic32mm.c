/***************************************************************************
 *   This is a FLASH driver for the PIC32MM family based on the original   *
 *   PIC32MX driver. Due to complexity constraints, it is forked rather    *
 *   merged.															   *
 *																		   *
 *	 Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2008 by John McCarthy                                   *
 *   jgmcc@magma.ca                                                        *
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
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/jtag.h>
#include "imp.h"
#include <target/algorithm.h>
#include <target/mips32.h>
#include <target/mips_m4k.h>

#include <target/target_type.h>
#include <target/register.h>

#define PIC32_MANUF_ID	0x029

/* pic32mm memory locations */

#define PIC32MM_PHYS_RAM			0x00000000
#define PIC32MM_PHYS_PGM_FLASH		0x1D000000
#define PIC32MM_PHYS_PERIPHERALS	0x1F800000
#define PIC32MM_PHYS_BOOT_FLASH		0x1FC00000

/*
 * Translate Virtual and Physical addresses.
 * Note: These macros only work for KSEG0/KSEG1 addresses.
 */

#define Virt2Phys(v)	((v) & 0x1FFFFFFF)

/* pic32mm configuration register locations */

#define PIC32MM_DEVID		0xBF803660

/* pic32mm flash controller register locations */

#define PIC32MM_NVMCON		0xBF802930
#define PIC32MM_NVMCONCLR	0xBF802934
#define PIC32MM_NVMCONSET	0xBF802938
#define PIC32MM_NVMCONINV	0xBF80293C
#define NVMCON_NVMWR		(1 << 15)
#define NVMCON_NVMWREN		(1 << 14)
#define NVMCON_NVMERR		(1 << 13)
#define NVMCON_LVDERR		(1 << 12)
#define NVMCON_LVDSTAT		(1 << 11)
#define NVMCON_OP_PFM_ERASE		0x5
#define NVMCON_OP_PAGE_ERASE	0x4
#define NVMCON_OP_ROW_PROG		0x3
#define NVMCON_OP_DWORD_PROG	0x2
#define NVMCON_OP_WORD_PROG		0x1
#define NVMCON_OP_NOP			0x0

#define PIC32MM_NVMKEY		0xBF802940
#define PIC32MM_NVMADDR		0xBF802950
#define PIC32MM_NVMADDRCLR	0xBF802954
#define PIC32MM_NVMADDRSET	0xBF802958
#define PIC32MM_NVMADDRINV	0xBF80295C
#define PIC32MM_NVMDATA0	0xBF802960
#define PIC32MM_NVMDATA1	0xBF802970
#define PIC32MM_NVMSRCADDR	0xBF802980

#define PIC32MM_NVMPWP		0xBF802990
#define PIC32MM_NVMBWP		0xBF8029A0

/* flash unlock keys */

#define NVMKEY1			0xAA996655
#define NVMKEY2			0x556699AA

/*
 * DEVID values as per PIC32MM Flash Programming Specification Rev N
 */

static const struct {
	const char *name;
	uint32_t devid;
} pic32mm_devs[] = {
	//See table 18-1 in DS60001364D
	{"PIC32MM0016GPL020", 0x6B04},
	{"PIC32MM0032GPL020", 0x6B0C},
	{"PIC32MM0064GPL020", 0x6B14},
	{"PIC32MM0016GPL028", 0x6B02},
	{"PIC32MM0032GPL028", 0x6B0A},
	{"PIC32MM0064GPL028", 0x6B12},
	{"PIC32MM0016GPL036", 0x6B06},
	{"PIC32MM0032GPL036", 0x6B0B},
	{"PIC32MM0064GPL036", 0x6B16},
	{"PIC32MM0064GPM028", 0x7708},
	{"PIC32MM0128GPM028", 0x7710},
	{"PIC32MM0256GPM028", 0x7718},
	{"PIC32MM0064GPM036", 0x770A},
	{"PIC32MM0128GPM036", 0x7712},
	{"PIC32MM0256GPM036", 0x771A},
	{"PIC32MM0064GPM048", 0x772C},
	{"PIC32MM0128GPM048", 0x7734},
	{"PIC32MM0256GPM048", 0x773C},
	{"PIC32MM0064GPM064", 0x770E},
	{"PIC32MM0128GPM064", 0x7716},
	{"PIC32MM0256GPM064", 0x771E},
};

struct pic32mm_device_layout
{
	uint32_t flash_size_in_bytes, ram_size_in_bytes;
	uint32_t page_size_in_words, row_size_in_words;
	uint32_t boot_pages_per_protection_region;
};

enum
{
	PIC32MM_FLASH_WORD_SIZE_IN_BYTES = 4,
};

static inline bool pic32mm_is_boot_bank(struct flash_bank *bank)
{
	return Virt2Phys(bank->base) == PIC32MM_PHYS_BOOT_FLASH;
}

static int pic32mm_compute_device_layout(const char *pDeviceName, struct pic32mm_device_layout *layout, bool isBootFLASH)
{
	memset(layout, 0, sizeof(*layout));
	int FLASHNumber = atoi(pDeviceName + 7);
	int bootFLASHProtectionRegionSize = 0;
	
	if (pDeviceName[13] == 'M')
	{
		//See figures 4-1, 4-2 and 4-3 of DS60001387D
		layout->ram_size_in_bytes = ((FLASHNumber >= 256) ? 32 : 16) * 1024;
		bootFLASHProtectionRegionSize = 0x800;	//Register 5-7 in DS60001387D. 0x4000 according to the datasheet, but 0x800 in practice.
	}
	else if (pDeviceName[13] == 'L')
	{
		//See figures 4-1, 4-2 and 4-3 of DS60001324C
		layout->ram_size_in_bytes = ((FLASHNumber >= 32) ? 8 : 4) * 1024;
		bootFLASHProtectionRegionSize = 0x800;
	}
	else
		return ERROR_FAIL;
	
	if (isBootFLASH)
		layout->flash_size_in_bytes = 0x1700;	//From figures 4-1, 4-2 and 4-3 of the datasheets
	else
		layout->flash_size_in_bytes = FLASHNumber * 1024;

	layout->row_size_in_words = 64;
	layout->page_size_in_words = 512;
	layout->boot_pages_per_protection_region = bootFLASHProtectionRegionSize / (layout->page_size_in_words * PIC32MM_FLASH_WORD_SIZE_IN_BYTES);
	return ERROR_OK;
}

struct pic32mm_flash_bank {
	bool probed;
	struct pic32mm_device_layout layout;
	uint32_t protection_status;
};

/* flash bank pic32mm <base> <size> 0 0 <target#>
 */
FLASH_BANK_COMMAND_HANDLER(pic32mm_flash_bank_command)
{
	struct pic32mm_flash_bank *pic32mm_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	pic32mm_info = calloc(1, sizeof(struct pic32mm_flash_bank));
	bank->driver_priv = pic32mm_info;

	pic32mm_info->probed = false;

	return ERROR_OK;
}

static uint32_t pic32mm_get_flash_status(struct flash_bank *bank)
{
	struct target *target = bank->target;
	uint32_t status;

	target_read_u32(target, PIC32MM_NVMCON, &status);

	return status;
}

static uint32_t pic32mm_wait_status_busy(struct flash_bank *bank, int timeout)
{
	uint32_t status;

	/* wait for busy to clear */
	while (((status = pic32mm_get_flash_status(bank)) & NVMCON_NVMWR) && (timeout-- > 0)) {
		LOG_DEBUG("status: 0x%" PRIx32, status);
		alive_sleep(1);
	}
	if (timeout <= 0)
		LOG_DEBUG("timeout: status: 0x%" PRIx32, status);

	return status;
}

static int pic32mm_nvm_exec(struct flash_bank *bank, uint32_t op, uint32_t timeout)
{
	struct target *target = bank->target;
	uint32_t status;

	target_write_u32(target, PIC32MM_NVMCON, NVMCON_NVMWREN | op);
	uint32_t tmp;
	target_read_u32(target, PIC32MM_NVMCON, &tmp);

	/* unlock flash registers */
	target_write_u32(target, PIC32MM_NVMKEY, NVMKEY1);
	target_write_u32(target, PIC32MM_NVMKEY, NVMKEY2);

	/* start operation */
	target_write_u32(target, PIC32MM_NVMCONSET, NVMCON_NVMWR);

	status = pic32mm_wait_status_busy(bank, timeout);

	/* lock flash registers */
	target_write_u32(target, PIC32MM_NVMCONCLR, NVMCON_NVMWREN);

	return status;
}

static int pic32mm_protect_check(struct flash_bank *bank)
{
#if 0
	struct target *target = bank->target;
	struct pic32mm_flash_bank *pic32mm_info = bank->driver_priv;

	uint32_t config0_address;
	uint32_t devcfg0;
	unsigned int s, num_pages;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	switch (pic32mm_info->dev_type) {
	case	MX_1xx_2xx:
	case	MX_17x_27x:
		config0_address = PIC32MM_DEVCFG0_1xx_2xx;
		break;
	default:
		config0_address = PIC32MM_DEVCFG0;
		break;
	}

	target_read_u32(target, config0_address, &devcfg0);

	if ((devcfg0 & (1 << 28)) == 0) /* code protect bit */
		num_pages = 0xffff;			/* All pages protected */
	else if (Virt2Phys(bank->base) == PIC32MM_PHYS_BOOT_FLASH) {
		if (devcfg0 & (1 << 24))
			num_pages = 0;			/* All pages unprotected */
		else
			num_pages = 0xffff;		/* All pages protected */
	} else {
		/* pgm flash */
		switch (pic32mm_info->dev_type) {
		case	MX_1xx_2xx:
			num_pages = (~devcfg0 >> 10) & 0x7f;
			break;
		case	MX_17x_27x:
			num_pages = (~devcfg0 >> 10) & 0x1ff;
			break;
		default:
			num_pages = (~devcfg0 >> 12) & 0xff;
			break;
		}
	}

	for (s = 0; s < bank->num_sectors && s < num_pages; s++)
		bank->sectors[s].is_protected = 1;
	for (; s < bank->num_sectors; s++)
		bank->sectors[s].is_protected = 0;
#endif
	
	return ERROR_OK;
}

static int pic32mm_unprotect_sectors(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	uint32_t reg, mask = 0;
	int retval;
	struct pic32mm_flash_bank *pic32mm_info = bank->driver_priv;
	
	if (pic32mm_is_boot_bank(bank))
	{
		retval = target_read_u32(bank->target, PIC32MM_NVMBWP, &reg);
		if (retval != ERROR_OK)
			return retval;
		
		for (unsigned page = first; page <= last; page++)
		{
			uint32_t group = page / pic32mm_info->layout.boot_pages_per_protection_region;
			if (group > 2)
				break;
			
			mask |= (1 << (8 + group));
		}
		
		if (reg & mask)
		{
			if (!(reg & 0x8000))
			{
				LOG_ERROR("The NVMBWP register is locked and cannot be modified");
				return ERROR_FLASH_OPERATION_FAILED;
			}
			
			reg &= ~mask;
			
			if (!(reg & 0x8000))
			{
				LOG_ERROR("Internal error: trying to permanently lock NVMBWP");
				return ERROR_FLASH_OPERATION_FAILED;
			}
			
			target_write_u32(bank->target, PIC32MM_NVMKEY, NVMKEY1);
			target_write_u32(bank->target, PIC32MM_NVMKEY, NVMKEY2);

			retval = target_write_u32(bank->target, PIC32MM_NVMBWP, reg);
		}
		
		return retval;
	}
	else
	{
		retval = target_read_u32(bank->target, PIC32MM_NVMPWP, &reg);
		if (retval != ERROR_OK)
			return retval;
		
		if (!(reg & 0x00FFFFFF))
			return ERROR_OK;	//Memory is not locked
		
		uint32_t sector_offset = (last + 1) * pic32mm_info->layout.page_size_in_words * PIC32MM_FLASH_WORD_SIZE_IN_BYTES;
		if (sector_offset < (reg & 0x00FFFFFF))
		{
			if (!(reg & 0x80000000))
			{
				LOG_ERROR("The NVMPWP register is locked and cannot be modified");
				return ERROR_FLASH_OPERATION_FAILED;
			}
			
			reg = (reg & 0xFF000000) | sector_offset;
			
			if (!(reg & 0x80000000))
			{
				LOG_ERROR("Internal error: trying to permanently lock NVMPWP");
				return ERROR_FLASH_OPERATION_FAILED;
			}			
			
			target_write_u32(bank->target, PIC32MM_NVMKEY, NVMKEY1);
			target_write_u32(bank->target, PIC32MM_NVMKEY, NVMKEY2);

			retval = target_write_u32(bank->target, PIC32MM_NVMPWP, reg);
		}

		return retval;
	}
}

static int pic32mm_invalidate_flash_line_buffer(struct flash_bank *bank)
{
	//Invalidate the data stored in the FLASH line buffer by reading 2 unrelated sectors one after another
	uint8_t tmp[4];
	int res = target_read_memory(bank->target, bank->base, 4, 1, tmp);
	if (res != ERROR_OK)
		return res;
	
	struct pic32mm_flash_bank *pic32mm_info = bank->driver_priv;
	res = target_read_memory(bank->target, bank->base + pic32mm_info->layout.page_size_in_words * PIC32MM_FLASH_WORD_SIZE_IN_BYTES, 4, 1, tmp);
	
	return res;
}

static int pic32mm_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct target *target = bank->target;
	uint32_t status;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	
	status = pic32mm_unprotect_sectors(bank, first, last);
	if (status != ERROR_OK)
		return status;

	if ((first == 0) && (last == (bank->num_sectors - 1))
		&& (Virt2Phys(bank->base) == PIC32MM_PHYS_PGM_FLASH)) {
		/* this will only erase the Program Flash (PFM), not the Boot Flash (BFM)
		 * we need to use the MTAP to perform a full erase */
		LOG_DEBUG("Erasing entire program flash");
		status = pic32mm_nvm_exec(bank, NVMCON_OP_PFM_ERASE, 50);
		if (status & NVMCON_NVMERR)
			return ERROR_FLASH_OPERATION_FAILED;
		if (status & NVMCON_LVDERR)
			return ERROR_FLASH_OPERATION_FAILED;
		return ERROR_OK;
	}

	for (unsigned int i = first; i <= last; i++) {
		target_write_u32(target, PIC32MM_NVMADDR, Virt2Phys(bank->base + bank->sectors[i].offset));

		status = pic32mm_nvm_exec(bank, NVMCON_OP_PAGE_ERASE, 10);

		if (status & NVMCON_NVMERR)
			return ERROR_FLASH_OPERATION_FAILED;
		if (status & NVMCON_LVDERR)
			return ERROR_FLASH_OPERATION_FAILED;
		bank->sectors[i].is_erased = 1;
	}

	return ERROR_OK;
}

static int pic32mm_protect(struct flash_bank *bank, int set, unsigned int first,
		unsigned int last)
{
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	return ERROR_OK;
}

/* see contrib/loaders/flash/pic32mm.s for src */

static uint32_t pic32mm_flash_write_code[] = {
					/* write: */
	0x3C08AA99,		/* lui $t0, 0xaa99 */
	0x35086655,		/* ori $t0, 0x6655 */
	0x3C095566,		/* lui $t1, 0x5566 */
	0x352999AA,		/* ori $t1, 0x99aa */
	0x3C0ABF80,		/* lui $t2, 0xbf80 */
	0x354A2930,		/* ori $t2, 0x2930 */
	0x340B4003,		/* ori $t3, $zero, 0x4003 */
	0x340C8000,		/* ori $t4, $zero, 0x8000 */
					/* write_row: */
	0x2CD30080,		/* sltiu $s3, $a2, 128 */
	0x16600008,		/* bne $s3, $zero, write_word */
	0x340D4000,		/* ori $t5, $zero, 0x4000 */
	0xAD450020,		/* sw $a1, 32($t2) */
	0xAD440040,		/* sw $a0, 64($t2) */
	0x04110016,		/* bal progflash */
	0x24840200,		/* addiu $a0, $a0, 512 */
	0x24A50200,		/* addiu $a1, $a1, 512 */
	0x1000FFF7,		/* beq $zero, $zero, write_row */
	0x24C6FF80,		/* addiu $a2, $a2, -128 */
					/* write_word: */
	0x3C15A000,		/* lui $s5, 0xa000 */
	0x36B50000,		/* ori $s5, $s5, 0x0 */
	0x00952025,		/* or $a0, $a0, $s5 */
	0x10000008,		/* beq $zero, $zero, next_word */
	0x340B4001,		/* ori $t3, $zero, 0x4001 */
					/* prog_word: */
	0x8C940000,		/* lw $s4, 0($a0) */
	0xAD540030,		/* sw $s4, 48($t2) */
	0xAD450020,		/* sw $a1, 32($t2) */
	0x04110009,		/* bal progflash */
	0x24840004,		/* addiu $a0, $a0, 4 */
	0x24A50004,		/* addiu $a1, $a1, 4 */
	0x24C6FFFF,		/* addiu $a2, $a2, -1 */
					/* next_word: */
	0x14C0FFF8,		/* bne $a2, $zero, prog_word */
	0x00000000,		/* nop */
					/* done: */
	0x10000002,		/* beq $zero, $zero, exit */
	0x24040000,		/* addiu $a0, $zero, 0 */
					/* error: */
	0x26240000,		/* addiu $a0, $s1, 0 */
					/* exit: */
	0x7000003F,		/* sdbbp */
					/* progflash: */
	0xAD4B0000,		/* sw $t3, 0($t2) */
	0xAD480010,		/* sw $t0, 16($t2) */
	0xAD490010,		/* sw $t1, 16($t2) */
	0xAD4C0008,		/* sw $t4, 8($t2) */
					/* waitflash: */
	0x8D500000,		/* lw $s0, 0($t2) */
	0x020C8024,		/* and $s0, $s0, $t4 */
	0x1600FFFD,		/* bne $s0, $zero, waitflash */
	0x00000000,		/* nop */
	0x00000000,		/* nop */
	0x00000000,		/* nop */
	0x00000000,		/* nop */
	0x00000000,		/* nop */
	0x8D510000,		/* lw $s1, 0($t2) */
	0x30113000,		/* andi $s1, $zero, 0x3000 */
	0x1620FFEF,		/* bne $s1, $zero, error */
	0xAD4D0004,		/* sw $t5, 4($t2) */
	0x03E00008,		/* jr $ra */
	0x00000000		/* nop */
};

static int pic32mm_write_block(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	int retval = ERROR_OK;
#if 0
	struct target *target = bank->target;
	struct working_area *write_algorithm;
	//struct working_area *source;
	uint32_t address = bank->base + offset;
	struct reg_param reg_params[3];
	uint32_t row_size;

	struct pic32mm_flash_bank *pic32mm_info = bank->driver_priv;
	struct mips32_algorithm mips32_info;

	/* flash write code */
	if (target_alloc_working_area(target, sizeof(pic32mm_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* Change values for counters and row size, depending on variant */
	switch (pic32mm_info->dev_type) {
	case	MX_1xx_2xx:
	case	MX_17x_27x:
		/* 128 byte row */
		pic32mm_flash_write_code[8] = 0x2CD30020;
		pic32mm_flash_write_code[14] = 0x24840080;
		pic32mm_flash_write_code[15] = 0x24A50080;
		pic32mm_flash_write_code[17] = 0x24C6FFE0;
		row_size = 128;
		break;
	default:
		/* 256 byte row */
		pic32mm_flash_write_code[8] = 0x2CD30040;
		pic32mm_flash_write_code[14] = 0x24840100;
		pic32mm_flash_write_code[15] = 0x24A50100;
		pic32mm_flash_write_code[17] = 0x24C6FF00;
		row_size = 256;
		break;
	}

	uint8_t code[sizeof(pic32mm_flash_write_code)];
	target_buffer_set_u32_array(target, code, ARRAY_SIZE(pic32mm_flash_write_code),
			pic32mm_flash_write_code);
	retval = target_write_buffer(target, write_algorithm->address, sizeof(code), code);
	if (retval != ERROR_OK)
		return retval;

	target_addr_t dataBuffer = 0xa0000000;
	uint32_t buffer_size = 0x800;

	mips32_info.common_magic = MIPS32_COMMON_MAGIC;
	mips32_info.isa_mode = MIPS32_ISA_MIPS32;

	init_reg_param(&reg_params[0], "r4", 32, PARAM_IN_OUT);
	init_reg_param(&reg_params[1], "r5", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "r6", 32, PARAM_OUT);

	int row_offset = offset % row_size;
	uint8_t *new_buffer = NULL;
	if (row_offset && (count >= (row_size / 4))) {
		new_buffer = malloc(buffer_size);
		if (new_buffer == NULL) {
			LOG_ERROR("Out of memory");
			return ERROR_FAIL;
		}
		memset(new_buffer,  0xff, row_offset);
		address -= row_offset;
	} else
		row_offset = 0;

	while (count > 0) {
		uint32_t status;
		uint32_t thisrun_count;

		if (row_offset) {
			thisrun_count = (count > ((buffer_size - row_offset) / 4)) ?
				((buffer_size - row_offset) / 4) : count;

			memcpy(new_buffer + row_offset, buffer, thisrun_count * 4);

			retval = target_write_buffer(target,
				dataBuffer,
				row_offset + thisrun_count * 4, new_buffer);
			if (retval != ERROR_OK)
				break;
		} else {
			thisrun_count = (count > (buffer_size / 4)) ?
					(buffer_size / 4) : count;

			retval = target_write_buffer(target,
				dataBuffer,
					thisrun_count * 4, buffer);
			if (retval != ERROR_OK)
				break;
		}

		buf_set_u32(reg_params[0].value, 0, 32, Virt2Phys(dataBuffer));
		buf_set_u32(reg_params[1].value, 0, 32, Virt2Phys(address));
		buf_set_u32(reg_params[2].value, 0, 32, thisrun_count + row_offset / 4);

		retval = target_run_algorithm(target, 0, NULL, 3, reg_params,
				write_algorithm->address,
				0, 10000, &mips32_info);
		if (retval != ERROR_OK) {
			LOG_ERROR("error executing pic32mm flash write algorithm");
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}

		status = buf_get_u32(reg_params[0].value, 0, 32);

		if (status & NVMCON_NVMERR) {
			LOG_ERROR("Flash write error NVMERR (status = 0x%08" PRIx32 ")", status);
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}

		if (status & NVMCON_LVDERR) {
			LOG_ERROR("Flash write error LVDERR (status = 0x%08" PRIx32 ")", status);
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}

		buffer += thisrun_count * 4;
		address += thisrun_count * 4;
		count -= thisrun_count;
		if (row_offset) {
			address += row_offset;
			row_offset = 0;
		}
	}

	//target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);

	free(new_buffer);
#endif
	return retval;
}

static int pic32mm_write_dword(struct flash_bank *bank, uint32_t address, uint32_t word0, uint32_t word1)
{
	struct target *target = bank->target;

	target_write_u32(target, PIC32MM_NVMADDR, Virt2Phys(address));
	target_write_u32(target, PIC32MM_NVMDATA0, word0);
	target_write_u32(target, PIC32MM_NVMDATA1, word1);

	return pic32mm_nvm_exec(bank, NVMCON_OP_DWORD_PROG, 5);
}

static int pic32mm_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	uint32_t words_remaining = (count / 4);
	uint32_t bytes_remaining = (count & 0x00000003);
	uint32_t address = bank->base + offset;
	uint32_t bytes_written = 0;
	uint32_t status;
	int retval;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_DEBUG("writing to flash at address " TARGET_ADDR_FMT " at offset 0x%8.8" PRIx32
			" count: 0x%8.8" PRIx32 "", bank->base, offset, count);

	if (offset & 0x3) {
		LOG_WARNING("offset 0x%" PRIx32 "breaks required 4-byte alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	/* multiple words (4-byte) to be programmed? */
	if (words_remaining > 0) {
		/* try using a block write */
		retval = pic32mm_write_block(bank, buffer, offset, words_remaining);
		if (retval != ERROR_OK) {
			if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
				/* if block write failed (no sufficient working area),
				 * we use normal (slow) single dword accesses */
				LOG_WARNING("couldn't use block writes, falling back to single memory accesses");
			} else if (retval == ERROR_FLASH_OPERATION_FAILED) {
				LOG_ERROR("flash writing failed");
				return retval;
			}
		} else {
			buffer += words_remaining * 4;
			address += words_remaining * 4;
			words_remaining = 0;
		}
	}

	while (words_remaining > 1) {
		uint32_t value[2];
		memcpy(value, buffer + bytes_written, 8);

		status = pic32mm_write_dword(bank, address, value[0], value[1]);

		if (status & NVMCON_NVMERR) {
			LOG_ERROR("Flash write error NVMERR (status = 0x%08" PRIx32 ")", status);
			return ERROR_FLASH_OPERATION_FAILED;
		}

		if (status & NVMCON_LVDERR) {
			LOG_ERROR("Flash write error LVDERR (status = 0x%08" PRIx32 ")", status);
			return ERROR_FLASH_OPERATION_FAILED;
		}

		bytes_written += 8;
		words_remaining-=2;
		address += 8;
	}

	if (bytes_remaining) {
		uint32_t value[2] = { -1, -1 };
		memcpy(value, buffer + bytes_written, bytes_remaining);

		status = pic32mm_write_dword(bank, address, value[0], value[1]);

		if (status & NVMCON_NVMERR) {
			LOG_ERROR("Flash write error NVMERR (status = 0x%08" PRIx32 ")", status);
			return ERROR_FLASH_OPERATION_FAILED;
		}

		if (status & NVMCON_LVDERR) {
			LOG_ERROR("Flash write error LVDERR (status = 0x%08" PRIx32 ")", status);
			return ERROR_FLASH_OPERATION_FAILED;
		}
	}

	return ERROR_OK;
}



static int pic32mm_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct pic32mm_flash_bank *pic32mm_info = bank->driver_priv;
	struct mips32_common *mips32 = target->arch_info;
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;
	
	//See Table 18-2 of DS60001364D
	uint32_t devID = (unsigned)((ejtag_info->idcode >> 12) & 0xffff);
	uint32_t manufacturingID = (unsigned)((ejtag_info->idcode >> 1) & 0x7ff);
	const char *pDeviceName = NULL;
	int retval;
	
	pic32mm_info->probed = false;

	LOG_INFO("device id = 0x%08" PRIx32 " (manuf 0x%03x dev 0x%04x, ver 0x%02x)",
		ejtag_info->idcode,
		manufacturingID,
		(unsigned)((ejtag_info->idcode >> 12) & 0xffff),
		devID);
	
	if (manufacturingID != PIC32_MANUF_ID) {
		LOG_WARNING("Cannot identify target as a PIC32MM family. Unexpected manufacturing ID: %02x", manufacturingID);
		return ERROR_FLASH_OPERATION_FAILED;
	}
	
	for (int i = 0; i < (sizeof(pic32mm_devs) / sizeof(pic32mm_devs[0])); i++) {
		if (pic32mm_devs[i].devid == devID ) {
			pDeviceName = pic32mm_devs[i].name;
			break;
		}
	}
	
	if (!pDeviceName)
	{
		LOG_WARNING("Cannot identify target as a PIC32MM family. Unexpected device ID: %02x", devID);
		return ERROR_FLASH_OPERATION_FAILED;
	}
	
	retval = pic32mm_compute_device_layout(pDeviceName, &pic32mm_info->layout, pic32mm_is_boot_bank(bank));
	if (retval != ERROR_OK)
	{
		LOG_WARNING("Cannot compute FLASH memory layout");
		return ERROR_FLASH_OPERATION_FAILED;
	}
	
	LOG_INFO("Detected %s with %d KB FLASH and %d KB RAM.", 
		pDeviceName,
		pic32mm_info->layout.flash_size_in_bytes / 1024,
		pic32mm_info->layout.ram_size_in_bytes / 1024);
	
	LOG_INFO("FLASH row size is %d words and page size is %d words",
		pic32mm_info->layout.row_size_in_words,
		pic32mm_info->layout.page_size_in_words);	
	
	if (pic32mm_is_boot_bank(bank))
	{
		if (target_read_u32(target, PIC32MM_NVMBWP, &pic32mm_info->protection_status) != ERROR_OK)
			pic32mm_info->protection_status = 0;
	}
	else
	{
		if (target_read_u32(target, PIC32MM_NVMPWP, &pic32mm_info->protection_status) != ERROR_OK)
			pic32mm_info->protection_status = 0;
	}

	free(bank->sectors);
	
	unsigned page_size_in_bytes = pic32mm_info->layout.page_size_in_words * PIC32MM_FLASH_WORD_SIZE_IN_BYTES;
	bank->size = pic32mm_info->layout.flash_size_in_bytes;
	bank->num_sectors = (pic32mm_info->layout.flash_size_in_bytes + page_size_in_bytes - 1) / page_size_in_bytes;
	bank->sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);

	for (unsigned i = 0; i < bank->num_sectors; i++) {
		bank->sectors[i].offset = i * page_size_in_bytes;
		bank->sectors[i].size = page_size_in_bytes;
		bank->sectors[i].is_erased = -1;
		
		if ((bank->sectors[i].offset + bank->sectors[i].size) > bank->size)
			bank->sectors[i].size = bank->size - bank->sectors[i].offset;	//Boot ROM size is not page-aligned according to the datasheet.
		
		if (pic32mm_is_boot_bank(bank))
		{
			int mask = 1 << (8 + (i / pic32mm_info->layout.boot_pages_per_protection_region));	//NVMBWP: Register 5-7 in DS60001387D
			bank->sectors[i].is_protected = (pic32mm_info->protection_status & mask) != 0;
		}
		else if (pic32mm_info->protection_status & 0x00FFFFFF)
			bank->sectors[i].is_protected = (bank->sectors[i].offset + page_size_in_bytes) <= (pic32mm_info->protection_status & 0x00FFFFFF);
	}

	pic32mm_info->probed = true;
	return ERROR_OK;
}

static int pic32mm_auto_probe(struct flash_bank *bank)
{
	struct pic32mm_flash_bank *pic32mm_info = bank->driver_priv;
	if (pic32mm_info->probed)
		return ERROR_OK;
	return pic32mm_probe(bank);
}

static int pic32mm_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct target *target = bank->target;
	struct mips32_common *mips32 = target->arch_info;
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;
	uint32_t device_id;

	device_id = ejtag_info->idcode;

	if (((device_id >> 1) & 0x7ff) != PIC32_MANUF_ID) {
		command_print_sameline(cmd,
			"Cannot identify target as a PIC32MM family (manufacturer 0x%03x != 0x%03x)\n",
			(unsigned)((device_id >> 1) & 0x7ff),
			PIC32_MANUF_ID);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	int i;
	for (i = 0; pic32mm_devs[i].name != NULL; i++) {
		if (pic32mm_devs[i].devid == (device_id & 0x0fffffff)) {
			command_print_sameline(cmd, "PIC32MM%s", pic32mm_devs[i].name);
			break;
		}
	}

	if (pic32mm_devs[i].name == NULL)
		command_print_sameline(cmd, "Unknown");

	command_print_sameline(cmd,
		" Ver: 0x%02x",
		(unsigned)((device_id >> 28) & 0xf));

	return ERROR_OK;
}

extern struct flash_driver virtual_flash;
struct flash_bank *virtual_get_master_bank(struct flash_bank *bank);

	
COMMAND_HANDLER(pic32mm_handle_pgm_word_command)
{
	uint32_t address;
	uint64_t value;
	int status, res;

	if (CMD_ARGC != 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], address);
	COMMAND_PARSE_NUMBER(u64, CMD_ARGV[1], value);

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 2, &bank);
	if (ERROR_OK != retval)
		return retval;

	if (address < bank->base || address >= (bank->base + bank->size)) {
		command_print(CMD, "flash address '%s' is out of bounds", CMD_ARGV[0]);
		return ERROR_OK;
	}
	
	struct flash_bank *pic32mm_bank = bank;
	
	if (bank->driver == &virtual_flash)
		pic32mm_bank = virtual_get_master_bank(bank);
	
	struct pic32mm_flash_bank *pic32mm_info = pic32mm_bank->driver_priv;
	
	unsigned sector = (address - bank->base) / (pic32mm_info->layout.page_size_in_words * PIC32MM_FLASH_WORD_SIZE_IN_BYTES);
	res = pic32mm_unprotect_sectors(pic32mm_bank, sector, sector);
	if (res != ERROR_OK)
	{
		command_print(CMD, "failed to unlock FLASH");
		return ERROR_OK;
	}
	
	res = ERROR_OK;
	status = pic32mm_write_dword(pic32mm_bank, address, (uint32_t)value, (uint32_t)(value >> 32));
	if (status & NVMCON_NVMERR)
		res = ERROR_FLASH_OPERATION_FAILED;
	if (status & NVMCON_LVDERR)
		res = ERROR_FLASH_OPERATION_FAILED;

	if (res != ERROR_OK)
		command_print(CMD, "pic32mm pgm word failed (status = 0x%x)", status);
	else
	{
		pic32mm_invalidate_flash_line_buffer(bank);
		
		uint64_t read_value;
		res = target_read_memory(bank->target, address, 4, 2, (uint8_t *)&read_value);
		if (read_value == value)
			command_print(CMD, "pic32mm: programmed and verified word at 0x%08x", address);
		else
			command_print(CMD, "pic32mm: failed to verify word at 0x%08x. Read 0x%16llx", address, read_value);
	}

	return ERROR_OK;
}

COMMAND_HANDLER(pic32mm_handle_unlock_command)
{
	struct target *target = NULL;
	struct mips_m4k_common *mips_m4k;
	struct mips_ejtag *ejtag_info;
	int timeout = 10;

	if (CMD_ARGC < 1) {
		command_print(CMD, "pic32mm unlock <bank>");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	target = bank->target;
	mips_m4k = target_to_m4k(target);
	ejtag_info = &mips_m4k->mips32.ejtag_info;

	/* we have to use the MTAP to perform a full erase */
	mips_ejtag_set_instr(ejtag_info, MTAP_SW_MTAP);
	mips_ejtag_set_instr(ejtag_info, MTAP_COMMAND);

	/* first check status of device */
	uint8_t mchip_cmd = MCHP_STATUS;
	mips_ejtag_drscan_8(ejtag_info, &mchip_cmd);
	if (mchip_cmd & (1 << 7)) {
		/* device is not locked */
		command_print(CMD, "pic32mm is already unlocked, erasing anyway");
	}

	/* unlock/erase device */
	mips_ejtag_drscan_8_out(ejtag_info, MCHP_ASERT_RST);
	jtag_add_sleep(200);

	mips_ejtag_drscan_8_out(ejtag_info, MCHP_ERASE);

	do {
		mchip_cmd = MCHP_STATUS;
		mips_ejtag_drscan_8(ejtag_info, &mchip_cmd);
		if (timeout-- == 0) {
			LOG_DEBUG("timeout waiting for unlock: 0x%" PRIx8 "", mchip_cmd);
			break;
		}
		alive_sleep(1);
	} while ((mchip_cmd & (1 << 2)) || (!(mchip_cmd & (1 << 3))));

	mips_ejtag_drscan_8_out(ejtag_info, MCHP_DE_ASSERT_RST);

	/* select ejtag tap */
	mips_ejtag_set_instr(ejtag_info, MTAP_SW_ETAP);

	command_print(CMD, "pic32mm unlocked.\n"
			"INFO: a reset or power cycle is required "
			"for the new settings to take effect.");

	return ERROR_OK;
}

static const struct command_registration pic32mm_exec_command_handlers[] = {
	{
		.name = "pgm_word",
		.usage = "<addr> <value> <bank>",
		.handler = pic32mm_handle_pgm_word_command,
		.mode = COMMAND_EXEC,
		.help = "program a word",
	},
	{
		.name = "unlock",
		.handler = pic32mm_handle_unlock_command,
		.mode = COMMAND_EXEC,
		.usage = "[bank_id]",
		.help = "Unlock/Erase entire device.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration pic32mm_command_handlers[] = {
	{
		.name = "pic32mm",
		.mode = COMMAND_ANY,
		.help = "pic32mm flash command group",
		.usage = "",
		.chain = pic32mm_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver pic32mm_flash = {
	.name = "pic32mm",
	.commands = pic32mm_command_handlers,
	.flash_bank_command = pic32mm_flash_bank_command,
	.erase = pic32mm_erase,
	.protect = pic32mm_protect,
	.write = pic32mm_write,
	.read = default_flash_read,
	.probe = pic32mm_probe,
	.auto_probe = pic32mm_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = pic32mm_protect_check,
	.info = pic32mm_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
