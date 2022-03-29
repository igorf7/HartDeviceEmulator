/**
 * @file task.c
 * @brief 
 *        (c)2018 I.Filippov
 */
#include "task.h"

QueueItem_t TaskQueue[TASK_QUEUE_SIZE]; // task queue
QueueItem_t* WriteQPtr = NULL;          // queue write pointer
QueueItem_t* ReadQPtr  = NULL;          // queue read pointer

void InitTaskQueue(void)
{
    for (uint32_t i = 0; i < TASK_QUEUE_SIZE; i++) {
		TaskQueue[i].FuncPtr = NULL;
		TaskQueue[i].Param = NULL;
		TaskQueue[i].Next = &TaskQueue[i+1];
	}
	TaskQueue[TASK_QUEUE_SIZE-1].Next = &TaskQueue[0]; // loop the queue
	WriteQPtr = ReadQPtr  = &TaskQueue[0]; // initializing pointers
}

/**
 * @brief Adds a task to the processing queue
 * @param 
 * @param 
 */
void PutEvent(void(*func)(void*), void* param)
{
__disable_irq();                    // disable interrupts
	WriteQPtr->FuncPtr = func;      // pointer to the task handler
	WriteQPtr->Param = param;       // pointer to the handler parameter
	WriteQPtr = WriteQPtr->Next;    // pointer to the next element of the queue
	if (WriteQPtr == ReadQPtr)
		ReadQPtr = ReadQPtr->Next;	// crawl control on the head
__enable_irq();                     // enable interrupt
}
//eof
