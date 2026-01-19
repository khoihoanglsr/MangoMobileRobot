/*
 * touch_event.h
 *
 *  Created on: Jan 13, 2026
 *      Author: khoi2
 */

#ifndef INC_TOUCH_EVENT_H_
#define INC_TOUCH_EVENT_H_

#include <stdint.h>

typedef struct {
    uint16_t x;
    uint16_t y;
    uint8_t  pressed;   // 1 = pressed, 0 = released (nếu bạn muốn báo nhả)
} TouchEvent_t;

extern volatile TouchEvent_t g_touchState;

void Touch_Init_Lib();

#endif /* INC_TOUCH_EVENT_H_ */
