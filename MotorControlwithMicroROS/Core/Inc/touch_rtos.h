/*
 * touch_rtos.h
 *
 *  Created on: Jan 13, 2026
 *      Author: khoi2
 */

#ifndef INC_TOUCH_RTOS_H_
#define INC_TOUCH_RTOS_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "touch_event.h"

extern QueueHandle_t g_touchQueue;
extern TaskHandle_t  g_touchTaskHandle;

void TouchTask_Start(void *argument);

#endif /* INC_TOUCH_RTOS_H_ */
