/*
 * Copyright 2021 BLUESINK Co., Ltd.
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT.
 *
 */

#ifndef __CANOPEN_LIST_H__
#define __CANOPEN_LIST_H__

typedef struct CO_LIST{
	uint16_t cobID;
  	uint8_t data[8];
	struct CO_LIST *next, *prev;
}CO_LIST;

static inline void CANOpen_list_init(CO_LIST *list){
	list->next = list;
	list->prev = list;
}

static inline void CANOpen_list_add(CO_LIST *new,
	CO_LIST *prev,
	CO_LIST *next)
{
	next->prev = new;
	new->next = next;
	new->prev = prev;
	prev->next = new;
}

static inline void CANOpen_list_add_next(CO_LIST *new, CO_LIST *head){
	CANOpen_list_add(new, head, head->next);
}

static inline void CANOpen_list_add_prev(CO_LIST *new, CO_LIST *head){
	CANOpen_list_add(new, head->prev, head);
}

static inline void CANOpen_list_del(CO_LIST *entry){
	entry->next->prev = entry->prev;
	entry->prev->next = entry->next;
}

#endif