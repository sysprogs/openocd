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

/* this file contains various functionality useful to standalone systems */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>
#include <string.h>

#include "unicode.h"


int utf8_to_text(const uint8_t *utf_8, char* buf, size_t buflen)
{
	size_t i;

	if (buflen < 1)
		return -1;

	if (!utf_8) {
		buf[0] = '\0';
		return 0;
	}

	size_t n = 0;
	char *s = buf;

	for (i = 0; utf_8[i] && n < (buflen - 1); ++i) {
		if (isprint(utf_8[i])) {
			*s++ = utf_8[i];
			n++;
		} else {
			int n_chars = snprintf(s, buflen - 1 - (s - buf), "\\x%02x",
					(unsigned char)utf_8[i]);
			s += n_chars;
			n += n_chars;
		}
	}

	*s++ = '\0';

	return n;
}


/*
 * Unicode conversion functions.  Conversion function inspired by
 * http://stackoverflow.com/questions/23919515/how-to-convert-from-utf-16-to-utf-32-on-linux-with-std-library
 * utf16le first converted to utf-32 and then back to utf-8
 */
static int is_surrogate(uint16_t uc)
{
	return uc >= 0xd800 && uc <= 0xdfff;
}

static int is_high_surrogate(uint16_t uc)
{
	return (uc & 0xfffffc00) == 0xd800;
}

static int is_low_surrogate(uint16_t uc)
{
	return (uc & 0xfffffc00) == 0xdc00;
}

static uint32_t surrogate_to_utf32(uint16_t high, uint16_t low)
{
	return (high << 10) + low - 0x35fdc00;
}

static int convert_utf16le_to_utf32(const uint8_t *input, size_t input_size,
		uint32_t *output, size_t output_size)
{
	const uint8_t *const end = input + input_size;
	int n_output = 0;

	while (input < end) {
		/* Ensure at least 2 bytes are available to make up utf-16 input */
		if (input+1 >= end)
			return -1;

		uint16_t uc = *input++;
		uc |= *input++ << 8;

		if (!is_surrogate(uc)) {
			if ((size_t)n_output >= output_size)
				return -1;
			*output++ = uc;
			++n_output;
		} else {
			if (is_high_surrogate(uc) && input < end && is_low_surrogate(*input)) {
				if ((size_t)n_output >= output_size)
					return -1;
				*output++ = surrogate_to_utf32(uc, *input++);
				++n_output;
			} else {
				/* Error */
				return -1;
			}
		}
	}

	return n_output;
}

int utf16le_to_utf8(const uint8_t *utf_16le, size_t len, uint8_t* buf, size_t buflen)
{
	uint32_t utf32[256];
	uint8_t *p = buf;
	uint8_t *endbuf = p + buflen;
	int n_code_points;
	int i;


	if (len < 1)
		return -1;

	n_code_points = convert_utf16le_to_utf32(utf_16le, len,
			&utf32[0], sizeof(utf32)/sizeof(utf32[0]));

	/* Invalid input is returned as a null string.  */
	if (n_code_points < 0)
		return -1;

	for (i = 0; i < n_code_points; ++i) {
		uint32_t uc = utf32[i];
		uint8_t utf8_uc[6];
		size_t n_utf8_uc = 0;

		if (uc <= 0x7f) {
			utf8_uc[0] = uc;
			n_utf8_uc = 1;
		} else if (uc <= 0x7ff) {
			utf8_uc[0] = 0xc0 | ((uc >> 6) & 0x1f);
			utf8_uc[1] = 0x80 | (uc & 0x3f);
			n_utf8_uc = 2;
		} else if (uc <= 0xffff) {
			utf8_uc[0] = 0x70 | ((uc >> 12) & 0x0f);
			utf8_uc[1] = 0x80 | ((uc >> 6) & 0x3f);
			utf8_uc[2] = 0x80 | (uc & 0x3f);
			n_utf8_uc = 3;
		} else if (uc <= 0x1fffff) {
			utf8_uc[0] = 0xf0 | ((uc >> 18) & 0x07);
			utf8_uc[1] = 0x80 | ((uc >> 12) & 0x3f);
			utf8_uc[2] = 0x80 | ((uc >> 6) & 0x3f);
			utf8_uc[3] = 0x80 | (uc & 0x3f);
			n_utf8_uc = 4;
		} else if (uc <= 0x3ffffff) {
			utf8_uc[0] = 0xf8 | ((uc >> 24) & 0x03);
			utf8_uc[1] = 0x80 | ((uc >> 18) & 0x3f);
			utf8_uc[2] = 0x80 | ((uc >> 12) & 0x3f);
			utf8_uc[3] = 0x80 | ((uc >> 6) & 0x3f);
			utf8_uc[4] = 0x80 | (uc & 0x3f);
			n_utf8_uc = 5;
		} else if (uc <= 0x7fffffff) {
			utf8_uc[0] = 0xfc | ((uc >> 30) & 0x01);
			utf8_uc[1] = 0x80 | ((uc >> 24) & 0x3f);
			utf8_uc[2] = 0x80 | ((uc >> 18) & 0x3f);
			utf8_uc[3] = 0x80 | ((uc >> 12) & 0x3f);
			utf8_uc[4] = 0x80 | ((uc >> 6) & 0x3f);
			utf8_uc[5] = 0x80 | (uc & 0x3f);
			n_utf8_uc = 6;
		} else {
			/* Invalid utf-32 value. */
			return -1;
		}

		/* Out of space in output buffer? */
		if (p + n_utf8_uc + 1 >= endbuf)
			return -1;

		memcpy(p, utf8_uc, n_utf8_uc);
		p += n_utf8_uc;
	}

	*p = '\0';
	return p - buf;
}
