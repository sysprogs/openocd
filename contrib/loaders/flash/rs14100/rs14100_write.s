/********************************************************************/

	.text
	.syntax unified
	.arch armv7-m
	.cpu cortex-m4
	.thumb
	.thumb_func
/*
 * Params :
 * r0 = workarea start, status (out)
 * r1 = workarea end
 * r2 = target address
 * r3 = count (bytes)
 * r4 = QSPI base
 * r5 = checksum
 *
 * Clobbered:
 * r6 - temp
 * r7 - rp
 * r8 - wp, tmp
 * r9 - checksum location
 */
/*
 * This code is embedded within: src/flash/nor/rs14100.c as a "C" array.
 *
 * To rebuild:
 *   arm-none-eabi-gcc -c rs14100_write_new.s
 *   arm-none-eabi-objcopy -O binary rs14100_write_new.o rs14100_write_new.bin
 *   xxd -c 8 -i rs14100_write_new.bin > rs14100_write_new.txt
 *
 * Then read and edit this result into the "C" source.
 */

set_manual:
	mov.w	r6, #0x0C00	 		/* Disable automode and select manual mode (clear 6th bit) */
	movt	r6, #0x0001
	str.w	r6, [r4, #0x04]
	bl	wait_auto_mode_disable
	mov.w	r6, #0x0001			/* Set bus mode to single */
	movt	r6, #0x0208
	str.w	r6, [r4, #0x10]
	mov.w	r6, #0x0C00	 		
	movt	r6, #0x0001 			
	str.w	r6, [r4, #0x04]
	mov.w	r9, #0x20EC			/* checksum location - 236(0xEC) -0x12000+0xEC */
	movt	r9, #0x0001
wait_fifo:
	ldr 	r8, [r0, #0]  			/* read the write pointer */
	cmp 	r8, #0 				/* if it's zero, we're gonzo */
	beq 	exit
	ldr 	r7, [r0, #4] 			/* read the read pointer */
	cmp 	r7, r8 				/* wait until they are not equal */
	beq 	wait_fifo
write_enable:
	mov.w 	r6, #8 
	str.w	r6, [r4, #0x80]		
	mov.w 	r6, #0x6	
	str.w	r6, [r4, #0x40]	
	mov.w	r6, #0x0002			
	movt	r6, #0x0208	
	str.w	r6, [r4, #0x10]
	bl	wait_flash_busy
	mov.w	r6, #0x0001			/* DEASSERT_CSN    */
	movt	r6, #0x0208
	str.w	r6, [r4, #0x10]
write_command:
	mov.w 	r6, #8 
	str.w	r6, [r4, #0x80]		
	mov.w 	r6, #0x2	       		/* Send the page program command */
	str.w	r6, [r4, #0x40]
	mov.w	r6, #0x0002			
	movt	r6, #0x0208	
	str.w	r6, [r4, #0x10]
	bl	wait_flash_busy
write_address:	
	mov.w 	r6, #24 
	str.w	r6, [r4, #0x80]			
	mov 	r6, r2				/* Give target address */
	add	r2, r2, #4			/* Increment target address by 4 */
	str.w	r6, [r4, #0x40]
	mov.w	r6, #0x0002			
	movt	r6, #0x0208	
	str.w	r6, [r4, #0x10]
	bl	wait_flash_busy
write_swap_en:
	mov.w 	r6, #0xf1 
	str.w	r6, [r4, #0x14]
write_data:
	mov.w 	r6, #32 
	str.w	r6, [r4, #0x80]	
	ldr 	r6, [r7], #4			/* Load 4 bytes from the FIFO,  */ /* add 	r7, #4	/* increment the read pointer by 4 */
	sub	r8, r2, #4
	cmp 	r8, r9
	it	eq
	moveq	r6, r5				/* Write checksum @236 */
	str	r6, [r4, #0x40]
	mov.w	r6, #0x0002			
	movt	r6, #0x0208	
	str.w	r6, [r4, #0x10]
	bl	wait_flash_busy	
write_swap_disable:
	mov.w 	r6, #0xf0
	str.w	r6, [r4, #0x14]

	mov.w	r6, #0x0001			
	movt	r6, #0x0208
	str.w	r6, [r4, #0x10]
	bl	wait_flash_busy
	bl 	dowait		
	cmp	r7, r1				/* wrap the read pointer if it is at the end */
	it  	cs
	addcs	r7, r0, #8			/* skip loader args */
	str.w 	r7, [r0, #4]			/* store the new read pointer */
	subs	r3, r3, #4			/* decrement count */
	cbz	r3, write_disable		/* Exit if we have written everything */ /*add 	r2, #4				/* Increment flash address by 4 */
	b 	wait_fifo 			/* continue write */
wait_flash_busy:				/* Wait for the flash to finish the previous page write */
	ldr 	r6, [r4, #0x20]	 		/* Get status register */
	tst 	r6, #0x01 			/* If it isn't done, keep waiting */
	bne 	wait_flash_busy
	bx 	lr
dowait:
	mov.w 	r6, #0x0F0A
dowaitloop:
   	subs 	r6,#1
   	bne 	dowaitloop
   	bx 	lr  
wait_auto_mode_disable:
	ldr 	r6, [r4, #0x20]	 		/* Get status register */
	tst 	r6, #0x1000 			/* Check 12th bit for Auto_Mode disable, keep waiting */
	bne 	wait_auto_mode_disable
	bx 	lr
write_disable:
	mov.w 	r6, #0x8 
	str.w	r6, [r4, #0x80]		
	mov.w 	r6, #0x4			/* Give write disable */
	str.w	r6, [r4, #0x40]	
	mov.w	r6, #0x0002			
	movt	r6, #0x0208
	str.w	r6, [r4, #0x10]	
	bl	wait_flash_busy
	mov.w	r6, #0x0001			
	movt	r6, #0x0208
	str.w	r6, [r4, #0x10]
	mov.w	r6, #0x0c40			
	movt	r6, #0x0001 			
	str.w	r6, [r4, #0x04]
exit:
	bkpt 	#0x00
