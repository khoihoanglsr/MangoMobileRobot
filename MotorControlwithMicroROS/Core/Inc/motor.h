/*
 * motor.h
 *
 *  Created on: Dec 2, 2025
 *      Author: khoi2
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include <stdint.h>        // <--- thêm dòng này
#include "stm32f7xx_hal.h"

// Thang điều khiển "logic" của mình: -1000 .. 1000
#define MOTOR_SPEED_MAX    1200

typedef enum {
    MOTOR_1 = 0,    //bánh bên phải
    MOTOR_2         //bánh bên trái
} MotorId;

void Motor_Init(void);
void Motor_SetSpeed(MotorId id, int16_t speed);

#endif /* INC_MOTOR_H_ */
