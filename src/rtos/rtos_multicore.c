/***************************************************************************
 *   Copyright (C) 2016 by Sysprogs                                        *
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/time_support.h>
#include <jtag/jtag.h>
#include <target/register.h>
#include "target/target.h"
#include "target/target_type.h"
#include "server/server.h"
#include "server/gdb_server.h"
#include "rtos.h"
#include "helper/log.h"
#include "helper/types.h"
#include "rtos_standard_stackings.h"

static int multicore_create(struct target *target);
static int multicore_smp_init(struct target *target);
static int multicore_update_threads(struct rtos *rtos);
static int multicore_get_thread_reg_list(struct rtos *rtos, int64_t thread_id, char **hex_reg_list);
static int multicore_step_hook(struct target *target, int current, uint32_t address, int handle_breakpoints);
static int multicore_symbol_list_lookup(symbol_table_elem_t *symbol_list[]);


const struct rtos_type multicore_rtos = {
	.name = "multicore",

	.create = multicore_create,
	.update_threads = multicore_update_threads,
	.get_thread_reg_list = multicore_get_thread_reg_list,
    .smp_init = multicore_smp_init,
    .step_hook = multicore_step_hook,
    .get_symbol_list_to_lookup = multicore_symbol_list_lookup
};

enum
{
    ThreadIdBase = 1,
};

static int multicore_symbol_list_lookup(symbol_table_elem_t *symbol_list[])
{
    *symbol_list = (symbol_table_elem_t *)malloc(sizeof(symbol_table_elem_t));
    memset(*symbol_list, 0, sizeof(symbol_table_elem_t));
    return 0;
}

static int multicore_update_threads(struct rtos *rtos)
{
    int threadCnt = 0;
    for (struct target_list *pLst = rtos->target->head; pLst; pLst = pLst->next)
    {
        struct target *pThisTarget = pLst->target;
        if (!pThisTarget)
            continue;
        threadCnt++;
    }
    
    struct thread_detail *details = (struct thread_detail *)malloc(sizeof(struct thread_detail) * threadCnt);
    memset(details, 0, sizeof(struct thread_detail) * threadCnt);
    int i = 0;
    for (struct target_list *pLst = rtos->target->head; pLst; pLst = pLst->next)
    {
        struct target *pThisTarget = pLst->target;
        if (!pThisTarget)
            continue;
        
        if (i >= threadCnt)
            break;
        
        details[i].exists = true;
        details[i].threadid = pThisTarget->coreid + ThreadIdBase;
        details[i].thread_name_str = strdup(pThisTarget->cmd_name);
        i++;
    }

    rtos->thread_count = i;
    if (rtos->thread_details)
        free(rtos->thread_details);
    rtos->thread_details = details;
    if (rtos->target->gdb_service && rtos->target->gdb_service->target)
        rtos->current_thread = rtos->target->gdb_service->target->coreid + ThreadIdBase;
    else
        rtos->current_thread = 0;
    
	return 0;
}

void gdb_str_to_target(struct target *target,
    char *tstr,
    struct reg *reg);

static struct target *lookup_target(struct rtos *rtos, int64_t thread_id)
{
    for (struct target_list *pLst = rtos->target->head; pLst; pLst = pLst->next)
    {
        struct target *pThisTarget = pLst->target;
        if (!pThisTarget)
            continue;
        if ((pThisTarget->coreid + ThreadIdBase) == thread_id)
            return pThisTarget;
    }
    return NULL;
}

static struct target *freeze_targets(struct target *target, int delta)
{
    for (struct target_list *pLst = target->head; pLst; pLst = pLst->next)
    {
        struct target *pThisTarget = pLst->target;
        if (!pThisTarget || pThisTarget == target)
            continue;
        pThisTarget->frozen += delta;
    }
    return NULL;
}

static int multicore_get_thread_reg_list(struct rtos *rtos, int64_t thread_id, char **hex_reg_list)
{
    struct reg **reg_list;
    int reg_list_size, reg_packet_size = 0;
    int retval, i;
    struct target *target = lookup_target(rtos, thread_id);

    if (!target)
    {
        LOG_ERROR("ERROR: invalid thread ID %d", (int)thread_id);
        return -1;
    }

    retval = target_get_gdb_reg_list(target,
        &reg_list,
        &reg_list_size,
        REG_CLASS_GENERAL);
    if (retval != ERROR_OK)
        return retval;
    
    for (i = 0; i < reg_list_size; i++)
        reg_packet_size += DIV_ROUND_UP(reg_list[i]->size, 8) * 2;

    assert(reg_packet_size > 0);

    char *reg_packet = malloc(reg_packet_size + 1);
    *hex_reg_list = reg_packet;
    if (reg_packet == NULL)
        return ERROR_FAIL;
    
    for (i = 0; i < reg_list_size; i++) {
        if (!reg_list[i]->valid)
            reg_list[i]->type->get(reg_list[i]);
        
        gdb_str_to_target(target, reg_packet, reg_list[i]);
        reg_packet += DIV_ROUND_UP(reg_list[i]->size, 8) * 2;
    }
    
    return 0;
}

int rtos_thread_packet(struct connection *connection, const char *packet, int packet_size);

int multicore_thread_packet(struct connection *connection, char const *packet, int packet_size)
{
    struct target *target = get_target_from_connection(connection);
    if (target && target->rtos && !target->rtos->current_thread)
    {
        target->rtos->current_thread = target->coreid + ThreadIdBase;
    }
    
    return rtos_thread_packet(connection, packet, packet_size);
}

static int multicore_create(struct target *target)
{
	target->rtos->current_thread = 0;
	target->rtos->thread_details = NULL;
    target->rtos->gdb_thread_packet = multicore_thread_packet;
	return 0;
}

typedef int(*PSTEPHANDLER)(struct target *target, int current, uint32_t address, int handle_breakpoints);

int multicore_step_hook(struct target *target, int current, uint32_t address, int handle_breakpoints)
{
    if (target->rtos->current_threadid >= ThreadIdBase)
    {
        struct target *selected_target = lookup_target(target->rtos, target->rtos->current_threadid);
        if (selected_target)
            target = selected_target;
    }
    
    freeze_targets(target, 1);
    int retval = target->type->step(target, current, address, handle_breakpoints);
    freeze_targets(target, -1);
    return retval;
}

int multicore_smp_init(struct target *target)
{
    for (struct target_list *pLst = target->head; pLst; pLst = pLst->next)
    {
        if (pLst->target->rtos != target->rtos)
        {
            free(pLst->target->rtos);
            pLst->target->rtos = target->rtos;
        }
    }
    
    multicore_update_threads(target->rtos);
    return 0;
}
