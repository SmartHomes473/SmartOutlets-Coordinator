/*
 * outlet_task_handler.c
 *
 *  Created on: Nov 14, 2014
 *      Author: nick
 */

// C headers
#include <stdlib.h>

// kernel headers
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

// project headers
#include "tasks/outlet_task_handler.h"


// handle instances defined by cfg
extern Semaphore_Handle outletTaskQueue_sem;
extern Semaphore_Handle taskQueue_mutex;
extern Queue_Handle outletTask_queue;


void outlet_task_handler ( UArg arg0, UArg arg1 )
{
	while(1) {
		OutletTask *task;

		// wait for a task to come in
		Semaphore_pend(outletTaskQueue_sem, BIOS_WAIT_FOREVER);

		// lock the task queue
		Semaphore_pend(taskQueue_mutex, BIOS_WAIT_FOREVER);

		task = Queue_get(outletTask_queue);

		// release the mutex
		Semaphore_post(taskQueue_mutex);

		// run the task handler
		task->action(task);

		// free up memory
		free(task);
	}
}
