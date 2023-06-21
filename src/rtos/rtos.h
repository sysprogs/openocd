/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2011 by Broadcom Corporation                            *
 *   Evan Hunter - ehunter@broadcom.com                                    *
 ***************************************************************************/

#ifndef OPENOCD_RTOS_RTOS_H
#define OPENOCD_RTOS_RTOS_H

#include "server/server.h"
#include "target/target.h"
#include <helper/jim-nvp.h>

typedef int64_t threadid_t;
typedef int64_t symbol_address_t;

struct reg;

/**
 * Table should be terminated by an element with NULL in symbol_name
 */
struct symbol_table_elem {
	const char *symbol_name;
	symbol_address_t address;
	bool optional;
};

struct thread_detail {
	threadid_t threadid;
	bool exists;
	char *thread_name_str;
	char *extra_info_str;
};

struct rtos {
	const struct rtos_type *type;

	struct symbol_table_elem *symbols;
	struct target *target;
	/*  add a context variable instead of global variable */
	/* The thread currently selected by gdb. */
	int64_t current_threadid;
	/* The currently selected thread according to the target. */
	threadid_t current_thread;
	struct thread_detail *thread_details;
	int thread_count;
	int (*gdb_thread_packet)(struct connection *connection, char const *packet, int packet_size);
	int (*gdb_target_for_threadid)(struct connection *connection, int64_t thread_id, struct target **p_target);
	void *rtos_specific_params;
};

struct rtos_reg {
	uint32_t number;
	uint32_t size;
	uint8_t value[16];
};

struct rtos_type {
	const char *name;
	bool (*detect_rtos)(struct target *target);
	int (*create)(struct target *target);
	int (*smp_init)(struct target *target);
	int (*update_threads)(struct rtos *rtos);
	/** Return a list of general registers, with their values filled out. */
	int (*get_thread_reg_list)(struct rtos *rtos, int64_t thread_id,
			struct rtos_reg **reg_list, int *num_regs);
	int (*get_thread_reg)(struct rtos *rtos, int64_t thread_id,
			uint32_t reg_num, struct rtos_reg *reg);
	int (*get_symbol_list_to_lookup)(struct symbol_table_elem *symbol_list[]);
	int (*clean)(struct target *target);
	char * (*ps_command)(struct target *target);
	int (*set_reg)(struct rtos *rtos, uint32_t reg_num, uint8_t *reg_value);
	/* Implement these if different threads in the RTOS can see memory
	 * differently (for instance because address translation might be different
	 * for each thread). */
	int (*read_buffer)(struct rtos *rtos, target_addr_t address, uint32_t size,
			uint8_t *buffer);
	int (*write_buffer)(struct rtos *rtos, target_addr_t address, uint32_t size,
			const uint8_t *buffer);
    int (*step_hook)(struct target *target, int current, uint32_t address, int handle_breakpoints);
};

struct stack_register_offset {
	unsigned short number;		/* register number */
	signed short offset;		/* offset in bytes from stack head, or -1 to indicate
					 * register is not stacked, or -2 to indicate this is the
					 * stack pointer register */
	unsigned short width_bits;
};

struct rtos_register_stacking {
	unsigned char stack_registers_size;
	signed char stack_growth_direction;
	unsigned char num_output_registers;
	/* Some targets require evaluating the stack to determine the
	 * actual stack pointer for a process.  If this field is NULL,
	 * just use stacking->stack_registers_size * stack_growth_direction
	 * to calculate adjustment.
	 */
	target_addr_t (*calculate_process_stack)(struct target *target,
		const uint8_t *stack_data,
		const struct rtos_register_stacking *stacking,
		target_addr_t stack_ptr);
	const struct stack_register_offset *register_offsets;
	/* Optional field for targets which may have to implement their own stack read function.
	 * Because stack format can be weird or stack data needed to be edited before passing to the gdb.
	 */
	int (*read_stack)(struct target *target,
		int64_t stack_ptr,
		const struct rtos_register_stacking *stacking,
		uint8_t *stack_data);
};

#define GDB_THREAD_PACKET_NOT_CONSUMED (-40)

int rtos_create(struct jim_getopt_info *goi, struct target *target);
void rtos_destroy(struct target *target);
int rtos_set_reg(struct connection *connection, int reg_num,
		uint8_t *reg_value);
int rtos_generic_stack_read(struct target *target,
		const struct rtos_register_stacking *stacking,
		int64_t stack_ptr,
		struct rtos_reg **reg_list,
		int *num_regs);
int gdb_thread_packet(struct connection *connection, char const *packet, int packet_size);
int rtos_thread_packet(struct connection *connection, const char *packet, int packet_size);
int rtos_get_gdb_reg(struct connection *connection, int reg_num);
int rtos_get_gdb_reg_list(struct connection *connection);
int rtos_update_threads(struct target *target);
void rtos_free_threadlist(struct rtos *rtos);
int rtos_smp_init(struct target *target);
/*  function for handling symbol access */
int rtos_qsymbol(struct connection *connection, char const *packet, int packet_size);
int rtos_read_buffer(struct target *target, target_addr_t address,
		uint32_t size, uint8_t *buffer);
int rtos_write_buffer(struct target *target, target_addr_t address,
		uint32_t size, const uint8_t *buffer);

extern const struct rtos_type chibios_rtos;
extern const struct rtos_type chromium_ec_rtos;
extern const struct rtos_type ecos_rtos;
extern const struct rtos_type embkernel_rtos;
extern const struct rtos_type freertos_rtos;
extern const struct rtos_type hwthread_rtos;
extern const struct rtos_type linux_rtos;
extern const struct rtos_type mqx_rtos;
extern const struct rtos_type nuttx_rtos;
extern const struct rtos_type riot_rtos;
extern const struct rtos_type rtkernel_rtos;
extern const struct rtos_type threadx_rtos;
extern const struct rtos_type ucos_iii_rtos;
extern const struct rtos_type zephyr_rtos;

#endif /* OPENOCD_RTOS_RTOS_H */
