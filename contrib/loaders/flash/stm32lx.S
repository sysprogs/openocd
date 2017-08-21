/***************************************************************************
 *   Copyright (C) 2010 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2011 Øyvind Harboe                                      *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2011 Clement Burin des Roziers                          *
 *   clement.burin-des-roziers@hikob.com                                   *
 *                                                                         *
 *   Copyright (C) 2016 Cezary Gapiński                                    *
 *   cezary.gapinski@gmail.com                                             *
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


// Build : arm-eabi-gcc -c stm32lx.S
	.text
	.syntax unified
	.cpu cortex-m0
	.thumb
	.thumb_func
	.global write

/*
	r0 - destination address
	r1 - source address
	r2 - count to write
	r3 - count written
	r4 - temp
*/

	// Set 0 to r3
	movs	r3, #0
	// Go to compare
	b		test_done

write_word:
	// Load one word from address in r1 to r4, increment r1
	ldmia	r1!, {r4}
	// Store the word to address in r0 from r4, increment r0
	stmia	r0!, {r4}
	// Increment r3
	adds	r3, #1

test_done:
	// Compare r3 and r2
	cmp		r3, r2
	// Loop if not zero
	bcc		write_word

	// Set breakpoint to exit
	bkpt	#0x00

