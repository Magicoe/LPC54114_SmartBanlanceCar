/**
 ****************************************************************************************
 *
 * @file app_msg.h
 *
 * @brief Application Message functions
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: $
 *
 ****************************************************************************************
 */

#ifndef _APP_MSG_H_
#define _APP_MSG_H_

/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

/// structure of a queue element
struct eaci_msg_que_elm
{
    struct eaci_msg_que_elm *next;
};

/// structure of a queue
struct eaci_msg_que
{
    /// pointer to first element
    struct eaci_msg_que_elm *first;
    /// pointer to the last element
    struct eaci_msg_que_elm *last;
};

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Check if the queue is empty.
 *
 ****************************************************************************************
 */
bool eaci_msg_que_is_empty(const struct eaci_msg_que *const que);

/*
 ****************************************************************************************
 * @brief Push one element as last in the queue.
 *
 ****************************************************************************************
 */
void eaci_msg_que_push(struct eaci_msg_que *que, void *msg);

/*
 ****************************************************************************************
 * @brief Pop the first one element from the queue.
 * 
 ****************************************************************************************
 */
void *eaci_msg_que_pop(struct eaci_msg_que *que);

#endif
