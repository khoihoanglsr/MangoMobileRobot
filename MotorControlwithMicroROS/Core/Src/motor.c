/*
 * motor.c
 *
 *  Created on: Dec 2, 2025
 *      Author: khoi2
 */
#include "motor.h"
#include "tim.h"    // htim9
#include "gpio.h"   // HAL_GPIO_WritePin
#include <stdlib.h> // abs

// Map chân DIR
#define MOTOR1_DIR_GPIO_Port   GPIOF
#define MOTOR1_DIR_Pin         GPIO_PIN_14   // PF14

#define MOTOR2_DIR_GPIO_Port   GPIOF
#define MOTOR2_DIR_Pin         GPIO_PIN_13   // PF13

// Nếu motor nào quay ngược thì đặt SIGN = -1
#define MOTOR1_SIGN   (-1)   // <<< Motor 1 đang ngược chiều nên để -1
#define MOTOR2_SIGN   (+1)   // <<< Motor 2 đúng rồi thì để +1

// Dùng TIM1 Channel 1 & 2
extern TIM_HandleTypeDef htim9;

static void Motor_SetDir(MotorId id, uint8_t dir)
{
    GPIO_TypeDef *port;
    uint16_t pin;

    if (id == MOTOR_1) {
        port = MOTOR1_DIR_GPIO_Port;
        pin  = MOTOR1_DIR_Pin;
    } else {
        port = MOTOR2_DIR_GPIO_Port;
        pin  = MOTOR2_DIR_Pin;
    }

    HAL_GPIO_WritePin(port, pin, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Motor_Init(void)
{
    // Start PWM cho 2 kênh TIM1
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);

    // Mặc định: Motor dừng
    Motor_SetSpeed(MOTOR_1, 0);
    Motor_SetSpeed(MOTOR_2, 0);
}

void Motor_SetSpeed(MotorId id, int16_t speed)
{
    // Áp dụng SIGN cho từng motor trước khi xử lý
    if (id == MOTOR_1) {
        speed = (int16_t)(MOTOR1_SIGN * speed);
    } else {
        speed = (int16_t)(MOTOR2_SIGN * speed);
    }

    // Clamp speed vào [-MOTOR_SPEED_MAX .. MOTOR_SPEED_MAX]
    if (speed >  MOTOR_SPEED_MAX) speed =  MOTOR_SPEED_MAX;
    if (speed < -MOTOR_SPEED_MAX) speed = -MOTOR_SPEED_MAX;

    // Xác định chiều
    uint8_t dir = (speed >= 0) ? 1 : 0;
    int16_t mag = (speed >= 0) ? speed : -speed;   // độ lớn tuyệt đối

    Motor_SetDir(id, dir);

    // Lấy ARR thực tế của TIM1 (ví dụ 1199 → 1200 mức)
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim9); // = 1199
    uint32_t max_counts = arr + 1U;                  // = 1200

    // Scale từ [0..MOTOR_SPEED_MAX] → [0..ARR]
    uint32_t duty_counts = (uint32_t)mag * max_counts / MOTOR_SPEED_MAX;
    if (duty_counts > arr) {
        duty_counts = arr;   // đảm bảo không vượt ARR
    }

    uint32_t channel;
    if (id == MOTOR_1) {
        channel = TIM_CHANNEL_1;   // PE9
    } else {
        channel = TIM_CHANNEL_2;   // PE11
    }

    __HAL_TIM_SET_COMPARE(&htim9, channel, duty_counts);
}
