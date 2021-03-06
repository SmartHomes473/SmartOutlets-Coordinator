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
#include <ti/sysbios/knl/Task.h>
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
//	int foo = 0;
	while(1) {
		OutletTask *task;

		/// ***** DEBUG
//		Task_sleep(200);
//		OutletTask debug_task;
//		switch (foo) {
//		case 0:
//			debug_task.action = OUTLET_on;
//			foo = 1;
//			break;
//		case 1:
//			debug_task.action = OUTLET_on;
//			foo = 2;
//			break;
//		case 2:
//			debug_task.action = OUTLET_get_power;
//			foo = 0;
//			break;
//		}
//		debug_task.target = 0x12;
//		Semaphore_pend(taskQueue_mutex, BIOS_WAIT_FOREVER);
//		Queue_enqueue(outletTask_queue, (Queue_Elem*)&debug_task);
//		Semaphore_post(outletTaskQueue_sem);
//		Semaphore_post(taskQueue_mutex);
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


void update_power_task  ( UArg arg0, UArg arg1 )
{
	OutletTask *ballroom, *mancave;

	while (1) {
		Task_sleep(500);

		mancave = (OutletTask*)malloc(sizeof(OutletTask));
		mancave->action = OUTLET_get_power;
		mancave->target = 0x22;

		Semaphore_pend(taskQueue_mutex, BIOS_WAIT_FOREVER);
		Queue_enqueue(outletTask_queue, (Queue_Elem*)mancave);
		Semaphore_post(outletTaskQueue_sem);
		Semaphore_post(taskQueue_mutex);

		Task_sleep(500);

		ballroom = (OutletTask*)malloc(sizeof(OutletTask));
		ballroom->action = OUTLET_get_power;
		ballroom->target = 0x12;

		Semaphore_pend(taskQueue_mutex, BIOS_WAIT_FOREVER);
		Queue_enqueue(outletTask_queue, (Queue_Elem*)ballroom);
		Semaphore_post(outletTaskQueue_sem);
		Semaphore_post(taskQueue_mutex);
	}
}
