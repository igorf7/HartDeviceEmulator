/**
 * @brief task.h
 *        (c)2018 I.Filippov
 */
#ifndef __TASK_H
#define __TASK_H

#include "stm32l0xx.h"
#include "stddef.h"

#define TASK_QUEUE_SIZE		16

/* Structure of the task queue item */
typedef struct{
	
    void (*FuncPtr)(void*); // pointer to the handler function
    void* Param;            // parameter of the handler function
    void* Next;             // pointer to the next element of the queue
}QueueItem_t;

/**
 * Function puts event in task queue
 */
void PutEvent(void(*func)(void*), void* param);
void InitTaskQueue(void);
#endif
//eof
