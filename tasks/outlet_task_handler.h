/*
 * outlet_task_handler.h
 *
 *  Created on: Nov 14, 2014
 *      Author: nick
 */

#ifndef OUTLET_TASK_HANDLER_H_
#define OUTLET_TASK_HANDLER_H_

#include <stdint.h>
#include <ti/sysbios/knl/Queue.h>


typedef uint8_t OutletID;

typedef struct OutletTask OutletTask;

typedef void(*OutletTaskAction)(OutletTask*);

typedef struct OutletTask {
	Queue_Elem elem;
	OutletTaskAction action;
	OutletID target;
} OutletTask;



#endif /* OUTLET_TASK_HANDLER_H_ */
