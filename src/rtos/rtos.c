// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2011 by Broadcom Corporation                            *
 *   Evan Hunter - ehunter@broadcom.com                                    *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "rtos.h"
#include "target/target.h"
#include "helper/log.h"
#include "helper/binarybuffer.h"
#include "server/gdb_server.h"

static const struct rtos_type *rtos_types[] = {
	&threadx_rtos,
	&freertos_rtos,
	&ecos_rtos,
	&linux_rtos,
	&chibios_rtos,
	&chromium_ec_rtos,
	&embkernel_rtos,
	&mqx_rtos,
	&ucos_iii_rtos,
	&nuttx_rtos,
	&riot_rtos,
	&zephyr_rtos,
	&rtkernel_rtos,
	/* keep this as last, as it always matches with rtos auto */
	&hwthread_rtos,
	NULL
};

static int rtos_try_next(struct target *target);

int rtos_smp_init(struct target *target)
{
	if (target->rtos->type->smp_init)
		return target->rtos->type->smp_init(target);
	return ERROR_TARGET_INIT_FAILED;
}

static int rtos_target_for_threadid(struct connection *connection, int64_t threadid, struct target **t)
{
	struct target *curr = get_target_from_connection(connection);
	if (t)
		*t = curr;

	return ERROR_OK;
}

static int os_alloc(struct target *target, const struct rtos_type *ostype)
{
	struct rtos *os = target->rtos = calloc(1, sizeof(struct rtos));

	if (!os)
		return JIM_ERR;

	os->type = ostype;
	os->current_threadid = -1;
	os->current_thread = 0;
	os->symbols = NULL;
	os->target = target;

	/* RTOS drivers can override the packet handler in _create(). */
	os->gdb_thread_packet = rtos_thread_packet;
	os->gdb_target_for_threadid = rtos_target_for_threadid;

	return JIM_OK;
}

static void os_free(struct target *target)
{
	if (!target->rtos)
		return;

	free(target->rtos->symbols);
	free(target->rtos);
	target->rtos = NULL;
}

static int os_alloc_create(struct target *target, const struct rtos_type *ostype)
{
	int ret = os_alloc(target, ostype);

	if (ret == JIM_OK) {
		ret = target->rtos->type->create(target);
		if (ret != JIM_OK)
			os_free(target);
	}

	return ret;
}

int rtos_create(struct jim_getopt_info *goi, struct target *target)
{
	int x;
	const char *cp;
	Jim_Obj *res;
	int e;

	if (!goi->isconfigure && goi->argc != 0) {
		Jim_WrongNumArgs(goi->interp, goi->argc, goi->argv, "NO PARAMS");
		return JIM_ERR;
	}

	os_free(target);

	e = jim_getopt_string(goi, &cp, NULL);
	if (e != JIM_OK)
		return e;

	if (strcmp(cp, "none") == 0)
		return JIM_OK;

	if (strcmp(cp, "auto") == 0) {
		/* Auto detect tries to look up all symbols for each RTOS,
		 * and runs the RTOS driver's _detect() function when GDB
		 * finds all symbols for any RTOS. See rtos_qsymbol(). */
		target->rtos_auto_detect = true;

		/* rtos_qsymbol() will iterate over all RTOSes. Allocate
		 * target->rtos here, and set it to the first RTOS type. */
		return os_alloc(target, rtos_types[0]);
	}

	for (x = 0; rtos_types[x]; x++)
		if (strcmp(cp, rtos_types[x]->name) == 0)
			return os_alloc_create(target, rtos_types[x]);

	Jim_SetResultFormatted(goi->interp, "Unknown RTOS type %s, try one of: ", cp);
	res = Jim_GetResult(goi->interp);
	for (x = 0; rtos_types[x]; x++)
		Jim_AppendStrings(goi->interp, res, rtos_types[x]->name, ", ", NULL);
	Jim_AppendStrings(goi->interp, res, ", auto or none", NULL);

	return JIM_ERR;
}

void rtos_destroy(struct target *target)
{
	os_free(target);
}

int gdb_thread_packet(struct connection *connection, char const *packet, int packet_size)
{
	struct target *target = get_target_from_connection(connection);
	if (!target->rtos)
		return rtos_thread_packet(connection, packet, packet_size);	/* thread not
										 *found*/
	return target->rtos->gdb_thread_packet(connection, packet, packet_size);
}

static struct symbol_table_elem *find_symbol(const struct rtos *os, const char *symbol)
{
	struct symbol_table_elem *s;

	for (s = os->symbols; s->symbol_name; s++)
		if (!strcmp(s->symbol_name, symbol))
			return s;

	return NULL;
}

static struct symbol_table_elem *next_symbol(struct rtos *os, char *cur_symbol, uint64_t cur_addr)
{
	if (!os->symbols)
		os->type->get_symbol_list_to_lookup(&os->symbols);

	if (!cur_symbol[0])
		return &os->symbols[0];

	struct symbol_table_elem *s = find_symbol(os, cur_symbol);
	if (!s)
		return NULL;

	s->address = cur_addr;
	s++;
	return s;
}

/* rtos_qsymbol() processes and replies to all qSymbol packets from GDB.
 *
 * GDB sends a qSymbol:: packet (empty address, empty name) to notify
 * that it can now answer qSymbol::hexcodedname queries, to look up symbols.
 *
 * If the qSymbol packet has no address that means GDB did not find the
 * symbol, in which case auto-detect will move on to try the next RTOS.
 *
 * rtos_qsymbol() then calls the next_symbol() helper function, which
 * iterates over symbol names for the current RTOS until it finds the
 * symbol in the received GDB packet, and then returns the next entry
 * in the list of symbols.
 *
 * If GDB replied about the last symbol for the RTOS and the RTOS was
 * specified explicitly, then no further symbol lookup is done. When
 * auto-detecting, the RTOS driver _detect() function must return success.
 *
 * The symbol is tried twice to handle the -flto case with gcc.  The first
 * attempt uses the symbol as-is, and the second attempt tries the symbol
 * with ".lto_priv.0" appended to it.  We only consider the first static
 * symbol here from the -flto case.  (Each subsequent static symbol with
 * the same name is exported as .lto_priv.1, .lto_priv.2, etc.)
 *
 * rtos_qsymbol() returns 1 if an RTOS has been detected, or 0 otherwise.
 */
int rtos_qsymbol(struct connection *connection, char const *packet, int packet_size)
{
	int rtos_detected = 0;
	uint64_t addr = 0;
	size_t reply_len;
	char reply[GDB_BUFFER_SIZE + 1], cur_sym[GDB_BUFFER_SIZE / 2 + 1] = ""; /* Extra byte for null-termination */
	struct symbol_table_elem *next_sym = NULL;
	struct target *target = get_target_from_connection(connection);
	struct rtos *os = target->rtos;

	reply_len = sprintf(reply, "OK");

	if (!os)
		goto done;

	/* Decode any symbol name in the packet*/
	size_t len = unhexify((uint8_t *)cur_sym, strchr(packet + 8, ':') + 1, strlen(strchr(packet + 8, ':') + 1));
	cur_sym[len] = 0;

	const char no_suffix[] = "";
	const char lto_suffix[] = ".lto_priv.0";
	const size_t lto_suffix_len = strlen(lto_suffix);

	const char *cur_suffix;
	const char *next_suffix;

	/* Detect what suffix was used during the previous symbol lookup attempt, and
	 * speculatively determine the next suffix (only used for the unknown address case) */
	if (len > lto_suffix_len && !strcmp(cur_sym + len - lto_suffix_len, lto_suffix)) {
		/* Trim the suffix from cur_sym for comparison purposes below */
		cur_sym[len - lto_suffix_len] = '\0';
		cur_suffix = lto_suffix;
		next_suffix = NULL;
	} else {
		cur_suffix = no_suffix;
		next_suffix = lto_suffix;
	}

	if ((strcmp(packet, "qSymbol::") != 0) &&               /* GDB is not offering symbol lookup for the first time */
	    (!sscanf(packet, "qSymbol:%" SCNx64 ":", &addr))) { /* GDB did not find an address for a symbol */

		/* GDB could not find an address for the previous symbol */
		struct symbol_table_elem *sym = find_symbol(os, cur_sym);

		if (next_suffix) {
			next_sym = sym;
		} else if (sym && !sym->optional) {	/* the symbol is mandatory for this RTOS */
			if (!target->rtos_auto_detect) {
				LOG_WARNING("RTOS %s not detected. (GDB could not find symbol \'%s\')", os->type->name, cur_sym);
				goto done;
			} else {
				/* Autodetecting RTOS - try next RTOS */
				if (!rtos_try_next(target)) {
					LOG_WARNING("No RTOS could be auto-detected!");
					goto done;
				}

				/* Next RTOS selected - invalidate current symbol */
				cur_sym[0] = '\x00';
			}
		}
	}

	LOG_DEBUG("RTOS: Address of symbol '%s%s' is 0x%" PRIx64, cur_sym, cur_suffix, addr);

	if (!next_sym) {
		next_sym = next_symbol(os, cur_sym, addr);
		next_suffix = no_suffix;
	}

	/* Should never happen unless the debugger misbehaves */
	if (!next_sym) {
		LOG_WARNING("RTOS: Debugger sent us qSymbol with '%s%s' that we did not ask for", cur_sym, cur_suffix);
		goto done;
	}

	if (!next_sym->symbol_name) {
		/* No more symbols need looking up */

		if (!target->rtos_auto_detect) {
			rtos_detected = 1;
			goto done;
		}

		if (os->type->detect_rtos(target)) {
			LOG_INFO("Auto-detected RTOS: %s", os->type->name);
			rtos_detected = 1;
			goto done;
		} else {
			LOG_WARNING("No RTOS could be auto-detected!");
			goto done;
		}
	}

	assert(next_suffix);

	reply_len = 8;                                   /* snprintf(..., "qSymbol:") */
	reply_len += 2 * strlen(next_sym->symbol_name);  /* hexify(..., next_sym->symbol_name, ...) */
	reply_len += 2 * strlen(next_suffix);            /* hexify(..., next_suffix, ...) */
	reply_len += 1;                                  /* Terminating NUL */
	if (reply_len > sizeof(reply)) {
		LOG_ERROR("ERROR: RTOS symbol '%s%s' name is too long for GDB!", next_sym->symbol_name, next_suffix);
		goto done;
	}

	LOG_DEBUG("RTOS: Requesting symbol lookup of '%s%s' from the debugger", next_sym->symbol_name, next_suffix);

	reply_len = snprintf(reply, sizeof(reply), "qSymbol:");
	reply_len += hexify(reply + reply_len,
		(const uint8_t *)next_sym->symbol_name, strlen(next_sym->symbol_name),
		sizeof(reply) - reply_len);
	reply_len += hexify(reply + reply_len,
		(const uint8_t *)next_suffix, strlen(next_suffix),
		sizeof(reply) - reply_len);

done:
	gdb_put_packet(connection, reply, reply_len);
	return rtos_detected;
}

int rtos_thread_packet(struct connection *connection, char const *packet, int packet_size)
{
	struct target *target = get_target_from_connection(connection);

	if (strncmp(packet, "qThreadExtraInfo,", 17) == 0) {
		if ((target->rtos) && (target->rtos->thread_details) &&
				(target->rtos->thread_count != 0)) {
			threadid_t threadid = 0;
			int found = -1;
			sscanf(packet, "qThreadExtraInfo,%" SCNx64, &threadid);

			if ((target->rtos) && (target->rtos->thread_details)) {
				int thread_num;
				for (thread_num = 0; thread_num < target->rtos->thread_count; thread_num++) {
					if (target->rtos->thread_details[thread_num].threadid == threadid) {
						if (target->rtos->thread_details[thread_num].exists)
							found = thread_num;
					}
				}
			}
			if (found == -1) {
				gdb_put_packet(connection, "E01", 3);	/* thread not found */
				return ERROR_OK;
			}

			struct thread_detail *detail = &target->rtos->thread_details[found];

			int str_size = 0;
			if (detail->thread_name_str)
				str_size += strlen(detail->thread_name_str);
			if (detail->extra_info_str)
				str_size += strlen(detail->extra_info_str);

			char *tmp_str = calloc(str_size + 9, sizeof(char));
			char *tmp_str_ptr = tmp_str;

			if (detail->thread_name_str)
				tmp_str_ptr += sprintf(tmp_str_ptr, "Name: %s", detail->thread_name_str);
			if (detail->extra_info_str) {
				if (tmp_str_ptr != tmp_str)
					tmp_str_ptr += sprintf(tmp_str_ptr, ", ");
				tmp_str_ptr += sprintf(tmp_str_ptr, "%s", detail->extra_info_str);
			}

			assert(strlen(tmp_str) ==
				(size_t) (tmp_str_ptr - tmp_str));

			char *hex_str = malloc(strlen(tmp_str) * 2 + 1);
			size_t pkt_len = hexify(hex_str, (const uint8_t *)tmp_str,
				strlen(tmp_str), strlen(tmp_str) * 2 + 1);

			gdb_put_packet(connection, hex_str, pkt_len);
			free(hex_str);
			free(tmp_str);
			return ERROR_OK;

		}
		gdb_put_packet(connection, "", 0);
		return ERROR_OK;
	} else if (strncmp(packet, "qSymbol", 7) == 0) {
		if (rtos_qsymbol(connection, packet, packet_size) == 1) {
			if (target->rtos_auto_detect == true) {
				target->rtos_auto_detect = false;
				target->rtos->type->create(target);
			}
			target->rtos->type->update_threads(target->rtos);
		}
		return ERROR_OK;
	} else if (strncmp(packet, "qfThreadInfo", 12) == 0) {
		int i;
		if (target->rtos) {
			if (target->rtos->thread_count == 0) {
				gdb_put_packet(connection, "l", 1);
			} else {
				/*thread id are 16 char +1 for ',' */
				char *out_str = malloc(17 * target->rtos->thread_count + 1);
				char *tmp_str = out_str;
				for (i = 0; i < target->rtos->thread_count; i++) {
					tmp_str += sprintf(tmp_str, "%c%016" PRIx64, i == 0 ? 'm' : ',',
										target->rtos->thread_details[i].threadid);
				}
				gdb_put_packet(connection, out_str, strlen(out_str));
				free(out_str);
			}
		} else
			gdb_put_packet(connection, "l", 1);

		return ERROR_OK;
	} else if (strncmp(packet, "qsThreadInfo", 12) == 0) {
		gdb_put_packet(connection, "l", 1);
		return ERROR_OK;
	} else if (strncmp(packet, "qAttached", 9) == 0) {
		gdb_put_packet(connection, "1", 1);
		return ERROR_OK;
	} else if (strncmp(packet, "qOffsets", 8) == 0) {
		char offsets[] = "Text=0;Data=0;Bss=0";
		gdb_put_packet(connection, offsets, sizeof(offsets)-1);
		return ERROR_OK;
	} else if (strncmp(packet, "qCRC:", 5) == 0) {
		/* make sure we check this before "qC" packet below
		 * otherwise it gets incorrectly handled */
		return GDB_THREAD_PACKET_NOT_CONSUMED;
	} else if (strncmp(packet, "qC", 2) == 0) {
		if (target->rtos) {
			char buffer[19];
			int size;
			size = snprintf(buffer, 19, "QC%016" PRIx64, target->rtos->current_thread);
			gdb_put_packet(connection, buffer, size);
		} else
			gdb_put_packet(connection, "QC0", 3);
		return ERROR_OK;
	} else if (packet[0] == 'T') {	/* Is thread alive? */
		threadid_t threadid;
		int found = -1;
		sscanf(packet, "T%" SCNx64, &threadid);
		if ((target->rtos) && (target->rtos->thread_details)) {
			int thread_num;
			for (thread_num = 0; thread_num < target->rtos->thread_count; thread_num++) {
				if (target->rtos->thread_details[thread_num].threadid == threadid) {
					if (target->rtos->thread_details[thread_num].exists)
						found = thread_num;
				}
			}
		}
		if (found != -1)
			gdb_put_packet(connection, "OK", 2);	/* thread alive */
		else
			gdb_put_packet(connection, "E01", 3);	/* thread not found */
		return ERROR_OK;
	} else if (packet[0] == 'H') {	/* Set current thread ( 'c' for step and continue, 'g' for
					 * all other operations ) */
		if ((packet[1] == 'g') && (target->rtos)) {
			threadid_t threadid;
			sscanf(packet, "Hg%16" SCNx64, &threadid);
			LOG_DEBUG("RTOS: GDB requested to set current thread to 0x%" PRIx64, threadid);
			/* threadid of 0 indicates target should choose */
			if (threadid == 0)
				target->rtos->current_threadid = target->rtos->current_thread;
			else
				target->rtos->current_threadid = threadid;
		}
		gdb_put_packet(connection, "OK", 2);
		return ERROR_OK;
	}

	return GDB_THREAD_PACKET_NOT_CONSUMED;
}

static int rtos_put_gdb_reg_list(struct connection *connection,
		struct rtos_reg *reg_list, int num_regs)
{
	size_t num_bytes = 1; /* NUL */
	for (int i = 0; i < num_regs; ++i)
		num_bytes += DIV_ROUND_UP(reg_list[i].size, 8) * 2;

	char *hex = malloc(num_bytes);
	char *hex_p = hex;

	for (int i = 0; i < num_regs; ++i) {
		size_t count = DIV_ROUND_UP(reg_list[i].size, 8);
		size_t n = hexify(hex_p, reg_list[i].value, count, num_bytes);
		hex_p += n;
		num_bytes -= n;
	}

	gdb_put_packet(connection, hex, strlen(hex));
	free(hex);

	return ERROR_OK;
}

/** Look through all registers to find this register. */
int rtos_get_gdb_reg(struct connection *connection, int reg_num)
{
	struct target *target = get_target_from_connection(connection);
	int64_t current_threadid = target->rtos->current_threadid;
	if ((target->rtos) && (current_threadid != -1) &&
			(current_threadid != 0) &&
			((current_threadid != target->rtos->current_thread) ||
			(target->smp))) {	/* in smp several current thread are possible */
		struct rtos_reg *reg_list;
		int num_regs;

		LOG_DEBUG("getting register %d for thread 0x%" PRIx64
				  ", target->rtos->current_thread=0x%" PRIx64,
										reg_num,
										current_threadid,
										target->rtos->current_thread);

		int retval;
		if (target->rtos->type->get_thread_reg) {
			reg_list = calloc(1, sizeof(*reg_list));
			num_regs = 1;
			retval = target->rtos->type->get_thread_reg(target->rtos,
					current_threadid, reg_num, &reg_list[0]);
			if (retval != ERROR_OK) {
				LOG_ERROR("RTOS: failed to get register %d", reg_num);
				return retval;
			}
		} else {
			retval = target->rtos->type->get_thread_reg_list(target->rtos,
					current_threadid,
					&reg_list,
					&num_regs);
			if (retval != ERROR_OK) {
				LOG_ERROR("RTOS: failed to get register list");
				return retval;
			}
		}

		for (int i = 0; i < num_regs; ++i) {
			if (reg_list[i].number == (uint32_t)reg_num) {
				rtos_put_gdb_reg_list(connection, reg_list + i, 1);
				free(reg_list);
				return ERROR_OK;
			}
		}

		free(reg_list);
	}
	return ERROR_FAIL;
}

/** Return a list of general registers. */
int rtos_get_gdb_reg_list(struct connection *connection)
{
	struct target *target = get_target_from_connection(connection);
	int64_t current_threadid = target->rtos->current_threadid;
	if ((target->rtos) && (current_threadid != -1) &&
			(current_threadid != 0) &&
			((current_threadid != target->rtos->current_thread) ||
			(target->smp))) {	/* in smp several current thread are possible */
		struct rtos_reg *reg_list;
		int num_regs;

		LOG_DEBUG("RTOS: getting register list for thread 0x%" PRIx64
				  ", target->rtos->current_thread=0x%" PRIx64 "\r\n",
										current_threadid,
										target->rtos->current_thread);

		int retval = target->rtos->type->get_thread_reg_list(target->rtos,
				current_threadid,
				&reg_list,
				&num_regs);
		if (retval != ERROR_OK) {
			LOG_ERROR("RTOS: failed to get register list");
			return retval;
		}

		rtos_put_gdb_reg_list(connection, reg_list, num_regs);
		free(reg_list);

		return ERROR_OK;
	}
	return ERROR_FAIL;
}

int rtos_set_reg(struct connection *connection, int reg_num,
		uint8_t *reg_value)
{
	struct target *target = get_target_from_connection(connection);
	int64_t current_threadid = target->rtos->current_threadid;
	if ((target->rtos) &&
			(target->rtos->type->set_reg) &&
			(current_threadid != -1) &&
			(current_threadid != 0)) {
		return target->rtos->type->set_reg(target->rtos, reg_num, reg_value);
	}
	return ERROR_FAIL;
}

int rtos_generic_stack_read(struct target *target,
	const struct rtos_register_stacking *stacking,
	int64_t stack_ptr,
	struct rtos_reg **reg_list,
	int *num_regs)
{
	int retval;

	if (stack_ptr == 0) {
		LOG_ERROR("Error: null stack pointer in thread");
		return -5;
	}
	/* Read the stack */
	uint8_t *stack_data = malloc(stacking->stack_registers_size);
	uint32_t address = stack_ptr;

	if (stacking->stack_growth_direction == 1)
		address -= stacking->stack_registers_size;
	if (stacking->read_stack)
		retval = stacking->read_stack(target, address, stacking, stack_data);
	else
		retval = target_read_buffer(target, address, stacking->stack_registers_size, stack_data);
	if (retval != ERROR_OK) {
		free(stack_data);
		LOG_ERROR("Error reading stack frame from thread");
		return retval;
	}
	LOG_DEBUG("RTOS: Read stack frame at 0x%" PRIx32, address);

#if 0
		LOG_OUTPUT("Stack Data :");
		for (i = 0; i < stacking->stack_registers_size; i++)
			LOG_OUTPUT("%02X", stack_data[i]);
		LOG_OUTPUT("\r\n");
#endif

	target_addr_t new_stack_ptr;
	if (stacking->calculate_process_stack) {
		new_stack_ptr = stacking->calculate_process_stack(target,
				stack_data, stacking, stack_ptr);
	} else {
		new_stack_ptr = stack_ptr - stacking->stack_growth_direction *
			stacking->stack_registers_size;
	}

	*reg_list = calloc(stacking->num_output_registers, sizeof(struct rtos_reg));
	*num_regs = stacking->num_output_registers;

	for (int i = 0; i < stacking->num_output_registers; ++i) {
		(*reg_list)[i].number = stacking->register_offsets[i].number;
		(*reg_list)[i].size = stacking->register_offsets[i].width_bits;

		int offset = stacking->register_offsets[i].offset;
		if (offset == -2)
			buf_cpy(&new_stack_ptr, (*reg_list)[i].value, (*reg_list)[i].size);
		else if (offset != -1)
			buf_cpy(stack_data + offset, (*reg_list)[i].value, (*reg_list)[i].size);
	}

	free(stack_data);
/*	LOG_OUTPUT("Output register string: %s\r\n", *hex_reg_list); */
	return ERROR_OK;
}

static int rtos_try_next(struct target *target)
{
	struct rtos *os = target->rtos;
	const struct rtos_type **type = rtos_types;

	if (!os)
		return 0;

	while (*type && os->type != *type)
		type++;

	if (!*type || !*(++type))
		return 0;

	os->type = *type;

	free(os->symbols);
	os->symbols = NULL;

	return 1;
}

int rtos_update_threads(struct target *target)
{
	if ((target->rtos) && (target->rtos->type))
		target->rtos->type->update_threads(target->rtos);
	return ERROR_OK;
}

void rtos_free_threadlist(struct rtos *rtos)
{
	if (rtos->thread_details) {
		int j;

		for (j = 0; j < rtos->thread_count; j++) {
			struct thread_detail *current_thread = &rtos->thread_details[j];
			free(current_thread->thread_name_str);
			free(current_thread->extra_info_str);
		}
		free(rtos->thread_details);
		rtos->thread_details = NULL;
		rtos->thread_count = 0;
		rtos->current_threadid = -1;
		rtos->current_thread = 0;
	}
}

int rtos_read_buffer(struct target *target, target_addr_t address,
		uint32_t size, uint8_t *buffer)
{
	if (target->rtos->type->read_buffer)
		return target->rtos->type->read_buffer(target->rtos, address, size, buffer);
	return ERROR_NOT_IMPLEMENTED;
}

int rtos_write_buffer(struct target *target, target_addr_t address,
		uint32_t size, const uint8_t *buffer)
{
	if (target->rtos->type->write_buffer)
		return target->rtos->type->write_buffer(target->rtos, address, size, buffer);
	return ERROR_NOT_IMPLEMENTED;
}
