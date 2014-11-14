/*
 * outlet_tasks.h
 *
 *  Created on: Nov 14, 2014
 *      Author: nick
 */

#ifndef OUTLET_TASKS_H_
#define OUTLET_TASKS_H_

#include <stdint.h>
#include <ti/sysbios/knl/Queue.h>


typedef uint8_t OutletID;

typedef struct OutletTask OutletTask;

typedef void(*OutletTaskAction)(OutletTask*);

struct OutletTask {
	Queue_Elem elem;
	OutletTaskAction action;
	OutletID target;
};

#endif /* OUTLET_TASKS_H_ */
