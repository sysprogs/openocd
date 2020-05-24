/********************************************************************/

	.text
	.syntax unified
	.arch armv7-m
	.cpu cortex-m4
	.thumb
	.thumb_func
/*
 * Params :
 *
 * Clobbered:
 * r6 - temp
 */
/*
 * This code is embedded within: src/flash/nor/rs14100.c as a "C" array.
 *
 * To rebuild:
 *   arm-none-eabi-gcc -c rs14100_init.s
 *   arm-none-eabi-objcopy -O binary rs14100_init.o rs14100_init.bin
 *   xxd -c 8 -i rs14100_init.bin > rs14100_init.txt
 *
 * Then read and edit this result into the "C" source.
 */
code:
	mov.w	r4, #0x0000	 		/* store QSPI base addr in R4 */
	movt	r4, #0x1200
	mov.w	r6, #0x0C00	 		/* neg edge sampling, d3 d2 bit status */
	movt	r6, #0x0001
	str.w	r6, [r4, #0x04]
	mov.w	r6, #0xf			/* d4-d7 status */
	str.w	r6, [r4, #0xb0]
	mov.w	r6, #0x0000	 		/* dual flash disable */
	movt	r6, #0x0002
	str.w	r6, [r4, #0xc4]
	mov.w	r6, #0x0001			/* Resetting flash in Single mode */
	movt	r6, #0x0228
	str.w	r6, [r4, #0x10]
	mov.w	r6, #0x0C00	 		/* Setting bus mode Single/Quad */
	movt	r6, #0x0001
	str.w	r6, [r4, #0x04]
	bl	wait_auto_mode_disable
	mov.w	r6, #0x8			
	str.w	r6, [r4, #0x80]
	mov.w	r6, #0xff			
	str.w	r6, [r4, #0x40]
	mov.w	r6, #0x0002			
	movt	r6, #0x0208
	str.w	r6, [r4, #0x10]
	bl	wait_flash_busy
	mov.w	r6, #0x0001			
	movt	r6, #0x0208
	str.w	r6, [r4, #0x10]
	mov.w	r6, #0x0001			/* Resetting flash in Quad mode */
	movt	r6, #0x0208
	str.w	r6, [r4, #0x10]
	mov.w	r6, #0x0C04	 		/* Setting bus mode Single/Quad */
	movt	r6, #0x0001
	str.w	r6, [r4, #0x04]
	bl	wait_auto_mode_disable
	mov.w	r6, #0x8			
	str.w	r6, [r4, #0x80]
	mov.w	r6, #0xff			
	str.w	r6, [r4, #0x40]
	mov.w	r6, #0x0002			
	movt	r6, #0x0208
	str.w	r6, [r4, #0x10]
	bl	wait_flash_busy
	mov.w	r6, #0x0001			
	movt	r6, #0x0208
	str.w	r6, [r4, #0x10]
	mov.w	r6, #0x0001			
	movt	r6, #0x0208
	str.w	r6, [r4, #0x10]
	mov.w	r6, #0x0C00	 		
	movt	r6, #0x0001
	str.w	r6, [r4, #0x04]
	mov.w	r6, #0x100	 		
	str.w	r6, [r4, #0x00]
	mov.w	r6, #0x0C00	 		
	movt	r6, #0x0001
	str.w	r6, [r4, #0x04]
	mov.w	r6, #0x0001			/* CSN and HW control mode enable */
	movt	r6, #0x0208
	str.w	r6, [r4, #0x10]
	mov.w	r6, #0xf0	 		
	str.w	r6, [r4, #0x14]
	mov.w	r6, #0xf0	 		
	str.w	r6, [r4, #0x14]
	mov.w	r6, #0xf0	 		
	str.w	r6, [r4, #0x14]
	mov.w	r6, #0xa	 		
	str.w	r6, [r4, #0x130]
	mov.w	r6, #0x0C00	 		
	movt	r6, #0x0001
	str.w	r6, [r4, #0x04]
	mov.w	r6, #0x0	 		
	str.w	r6, [r4, #0x90]
	mov.w	r6, #0x0	 		
	str.w	r6, [r4, #0x90]
	mov.w	r6, #0x0311	 		
	movt	r6, #0x0003
	str.w	r6, [r4, #0x0c]
	mov.w	r6, #0x0	 		
	str.w	r6, [r4, #0x08]
	mov.w	r6, #0x0C00	 		
	movt	r6, #0x0001
	str.w	r6, [r4, #0x04]
	mov.w	r6, #0x0C40	 		
	movt	r6, #0x0001
	str.w	r6, [r4, #0x04]
	b 	done
wait_flash_busy:				/* Wait for the flash to finish the previous page write */
	ldr 	r6, [r4, #0x20]	 		/* Get status register */
	tst 	r6, #0x01 			/* If it isn't done, keep waiting */
	bne 	wait_flash_busy
	bx 	lr
wait_auto_mode_disable:
	ldr 	r6, [r4, #0x20]	 		/* Get status register */
	tst 	r6, #0x1000 			/* Check 12th bit for Auto_Mode disable, keep waiting */
	bne 	wait_auto_mode_disable
	bx 	lr
done:
	bkpt 	#0
