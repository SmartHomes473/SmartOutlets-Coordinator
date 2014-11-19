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
#include "outlets/outlet_task_handler.h"
#include "outlets/outlet_tasks.h"


// handle instances defined by cfg
extern Semaphore_Handle outletTaskQueue_sem;
extern Semaphore_Handle taskQueue_mutex;
extern Queue_Handle outletTask_queue;


void outlet_task_handler ( UArg arg0, UArg arg1 )
{
	while(1) {
		OutletTask *task;
		static int foo = 0;

		/// ***** DEBUG
		__delay_cycles(40000000);
		OutletTask debug_task;
		debug_task.action = foo++ % 2 ? OUTLET_on : OUTLET_off;
		debug_task.target = 0x12;
		Semaphore_pend(taskQueue_mutex, BIOS_WAIT_FOREVER);
		Queue_enqueue(outletTask_queue, (Queue_Elem*)&debug_task);
		Semaphore_post(outletTaskQueue_sem);
		Semaphore_post(taskQueue_mutex);
		/// ***** END DEBUG


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
