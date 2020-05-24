/********************************************************************/

	.text
	.syntax unified
	.arch armv7-m
	.cpu cortex-m4
	.thumb
	.thumb_func
/*
 * Params :
 * r0 - first
 * r1 - last
 *
 * Clobbered:
 * r6 - temp
 * r4 - QSPI base addr (0x12000000)
 * r3 - 0x02080001
 * r5 - 0x02080002
 * r8 - 0x8
 */
/*
 * This code is embedded within: src/flash/nor/rs14100.c as a "C" array.
 *
 * To rebuild:
 *   arm-none-eabi-gcc -c rs14100_erase.s
 *   arm-none-eabi-objcopy -O binary rs14100_erase.o rs14100_erase.bin
 *   xxd -c 8 -i rs14100_erase.bin > rs14100_erase.txt
 *
 * Then read and edit this result into the "C" source.
 */

set_manual:
	mov.w	r4, #0x0000	 		/* store QSPI base addr in R4 for accessing all QSPI registers*/
	movt	r4, #0x1200	
	mov.w	r6, #0x0C00	 		
	movt	r6, #0x0001
	str.w	r6, [r4, #0x04]			/* Disable automode and select manual mode (clear 6th bit) */
	bl	wait_auto_mode_disable
	mov.w	r6, #0x0001			
	movt	r6, #0x0208	
	str.w	r6, [r4, #0x10]			/* Set bus mode to single */
	mov.w	r6, #0x0C00	 		
	movt	r6, #0x0001 			
	str.w	r6, [r4, #0x04]
	mov.w	r2, #0x2000			/* store Flash base addr in R2 for providind erase addr, incremented by 4k every iteration*/
	movt	r2, #0x0001
	mov.w	r3, #0x0001			/* store 0x02080001*/
	movt	r3, #0x0208
	mov.w	r5, #0x0002			/* store 0x02080002 */
	movt	r5, #0x0208
	mov.w 	r8, #8
write_enable:
	str.w	r8, [r4, #0x80]		
	mov.w 	r6, #0x6	
	str.w	r6, [r4, #0x40]	
	str.w	r5, [r4, #0x10]
	bl	wait_qspi_idle
	str.w	r3, [r4, #0x10]			/* DEASSERT_CSN */
sector_erase_command:
	str.w	r8, [r4, #0x80]		
	mov.w 	r6, #0x20	       		/* Send the sector erase command */
	str.w	r6, [r4, #0x40]
	str.w	r5, [r4, #0x10]
	bl	wait_qspi_idle
write_address:	
	mov.w 	r6, #24 
	str.w	r6, [r4, #0x80]			
	str.w	r2, [r4, #0x40]			/* Give target address [TODO: use STRIT/LDIA to auto increment addr] */
	add	r2, r2, #0x1000  		/* Increment target address by 4k */
	str.w	r5, [r4, #0x10]
	bl	wait_qspi_idle
	str.w	r3, [r4, #0x10]	
	mov.w 	r6, #8	    
	str.w	r6, [r4, #0x80]		
	mov.w 	r6, #0x05	       		/* Send the status reg read command */
	str.w	r6, [r4, #0x40]
	str.w	r5, [r4, #0x10]	
	bl	wait_qspi_idle
	bl	dowait	
	str.w	r3, [r4, #0x10]	
	subs	r1, r1, #1			/* decrement 'last' sector */
	cbz	r1, write_disable
	b	write_enable
wait_rx_fifo_empty:
	ldr 	r6, [r4, #0x20]	 		/* Get status register */
	tst 	r6, #0x80 			/* Check for BIT(7) ,if it isn't done, keep waiting */
	bne 	wait_fifo_empty			/* Wait for the flash to finish the previous page write */
	ldr 	r6, [r4, #0x40]			/* Get QSPI_MANUAL_RD_WR_DATA_REG register */ 
	tst 	r6, #0x01 			/* If it isn't done, keep waiting */
	bne 	wait_rx_fifo_empty
	bx	lr
wait_qspi_idle:
	ldr 	r6, [r4, #0x20]			/* Get QSPI status register */ 
	tst 	r6, #0x01 			/* If it isn't done, keep waiting */
	bne 	wait_qspi_idle
	bx 	lr
wait_flash_busy:
	ldr 	r6, [r4, #0x40]			/* Get QSPI_MANUAL_RD_WR_DATA_REG register */ 
	tst 	r6, #0x01 			/* If it isn't done, keep waiting */
	bne 	wait_flash_busy
	bx 	lr
dowait:
	mov.w 	r6, #0x000F
	movt 	r6, #0x000F 	
dowaitloop:
   	subs 	r6,#1
   	bne 	dowaitloop
   	bx 	lr  
wait_auto_mode_disable:
	ldr 	r6, [r4, #0x20]	 		/* Get status register */
	tst 	r6, #0x1000 			/* Check 12th bit for Auto_Mode disable, keep waiting */
	bne 	wait_auto_mode_disable
	bx 	lr
wait_flash_status_idle:
	mov.w 	r6, #8	    
	str.w	r6, [r4, #0x80]		
	mov.w 	r6, #0x05	       		/* Send the status reg read command */
	str.w	r6, [r4, #0x40]
	str.w	r5, [r4, #0x10]
	bl	wait_qspi_idle	
	str.w	r3, [r4, #0x10]			/* DEASSERT_CSN */
	mov.w	r6, #0x0C00	 		
	movt	r6, #0x0001
	str.w	r6, [r4, #0x04]			/* Disable automode and select manual mode (clear 6th bit) */
	bx 	lr
write_disable:
	str.w	r8, [r4, #0x80]		
	mov.w 	r6, #0x4			/* Give write disable */
	str.w	r6, [r4, #0x40]	
	str.w	r5, [r4, #0x10]	
	bl	wait_qspi_idle
	str.w	r3, [r4, #0x10]
	mov.w	r6, #0x0c40			
	movt	r6, #0x0001 			
	str.w	r6, [r4, #0x04]
	str.w	r3, [r4, #0x10]
exit:
	bkpt 	#0x00
