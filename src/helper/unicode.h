/***************************************************************************
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 ***************************************************************************/

#ifndef OPENOCD_HELPER_UNICODE_H
#define OPENOCD_HELPER_UNICODE_H

#include <stdint.h>
#include <stdlib.h>


/* Convert utf-8 encoded byte string to readable text.  Printable
 * characters are returned as is, all other digits are returned as
 * '\xHH' hex representation.
 *
 * Output in 'buf' will always be NULL terminated.  Returns number of
 * characters written into buf.
 */
int utf8_to_text(const uint8_t *utf_8, char* buf, size_t buflen);


/* Convert utf-16le encoded byte string to utf-8 encoded string.
 * Returns number of characters written into buf.  Output will always
 * be NULL terminated.  Negative value returned on error.
 */
int utf16le_to_utf8(const uint8_t *utf_16le, size_t len, uint8_t* buf, size_t buflen);


#endif	/* OPENOCD_HELPER_UNICODE_H */
