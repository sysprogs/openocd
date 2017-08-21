/***************************************************************************
 *   Copyright (C) 2010 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2011 Øyvind Harboe                                      *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2015 Uwe Bonnes                                         *
 *   bon@elektron.ikp.physik.tu-darmstadt.de                               *
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
 *   Free Software Foundation, Inc.                                        *
 ***************************************************************************/

	.text
	.syntax unified
	.cpu cortex-m4
	.thumb
	.thumb_func

/* To assemble:
 * arm-none-eabi-gcc -c stm32h7x.S
 *
 * To disassemble:
 * arm-none-eabi-objdump -o stm32h7x.o
 *
 * To generate binary file:
 * arm-none-eabi-objcopy -O binary stm32h7x.o stm32h7_flash_write_code.bin
 *
 * To generate include file:
 * xxd -i stm32h7_flash_write_code.bin
 */

/*
 * Params :
 * r0 = workarea start, status (out)
 * r1 = workarea end
 * r2 = target address
 * r3 = count (256 bit words)
 * r4 = flash reg base
 *
 * Clobbered:
 * r5 - index for loop
 * r6 - temp
 * r7 - rp
 * r8 - wp, tmp
 */

#define STM32_FLASH_CR_OFFSET	0x0C	/* offset of CR register in FLASH struct */
#define STM32_FLASH_SR_OFFSET	0x10	/* offset of SR register in FLASH struct */
#define STM32_PROG				0x32	/* Flash CR = PSIZE64 | PG */

wait_fifo:
	ldr 	r8, [r0, #0]	/* read wp */
	cmp 	r8, #0			/* abort if wp == 0 */
	beq 	exit
	ldr 	r7, [r0, #4]	/* read rp */
	subs	r6, r8, r7		/* number of bytes available for read in r6 */
	itt 	mi              /* if wrapped around*/
	addmi	r6, r1          /* add size of buffer */
	submi	r6, r0
	cmp 	r6, #32			/* wait until 32 bytes are available */
	bcc 	wait_fifo

	mov		r6, #STM32_PROG
	str		r6, [r4, #STM32_FLASH_CR_OFFSET]

	mov		r5, #8			/* program by 8 words = 32 bytes */
write_flash:
	ldr 	r6, [r7], #0x04	/* read one word from src, increment ptr */
	str 	r6, [r2], #0x04	/* write one word to dst, increment ptr */
	dsb
	subs	r5, r5, #1		/* decrement index */
	bne		write_flash		/* loop if not done */

busy:
	ldr 	r6, [r4, #STM32_FLASH_SR_OFFSET]
	tst 	r6, #0x1		/* BSY (bit1) == 1 => operation in progress */
	bne 	busy			/* wait more... */
	tst		r6, #0x01ee0000	/* SNECCERR | RDSERR | RDPERR | OPERR | INCERR | STRBERR | PGSERR | WRPERR */
	bne		error			/* fail... */
	tst		r6, #0x02000000	/* DBECCERR */
	bne		error			/* fail... */

	cmp 	r7, r1			/* wrap rp at end of buffer */
	it  	cs
	addcs	r7, r0, #8		/* skip loader args */
	str 	r7, [r0, #4]	/* store rp */
	subs	r3, r3, #1		/* decrement count */
	cbz 	r3, exit		/* loop if not done */
	b		wait_fifo
error:
	movs	r1, #0
	str		r1, [r0, #4]	/* set rp = 0 on error */
exit:
	mov		r0, r6			/* return status in r0 */
	bkpt	#0x00
