/*
 * encoder.h
 *
 *  Created on: Dec 2, 2025
 *      Author: khoi2
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "stm32f7xx_hal.h"
#include <stdint.h>

typedef enum {
    ENCODER_1 = 0,   // TIM3
    ENCODER_2        // TIM4
} EncoderId;

void   Encoder_Init(void);
int32_t Encoder_GetCount(EncoderId id);
void   Encoder_Reset(EncoderId id);

#endif /* INC_ENCODER_H_ */
