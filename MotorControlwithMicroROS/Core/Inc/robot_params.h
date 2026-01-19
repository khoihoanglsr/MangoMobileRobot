/*
 * robot_params.h
 *
 *  Created on: Dec 2, 2025
 *      Author: khoi2
 */

#ifndef INC_ROBOT_PARAMS_H_
#define INC_ROBOT_PARAMS_H_

#include "pid.h"
// Encoder & cơ khí
#define ENCODER_PPR        500       // xung / vòng / kênh
#define QUAD_FACTOR        4         // TI12: x4
#define GEAR_RATIO         14.0f
#define TICKS_PER_REV      (ENCODER_PPR * QUAD_FACTOR * GEAR_RATIO) // 28000

#define WHEEL_DIAMETER_M   0.10f     // 10 cm
#define WHEEL_CIRC_M       (3.1415926f * WHEEL_DIAMETER_M)
#define WHEEL_BASE_M   0.30f

//sample time for each tasks
#define ODOMETRY_PERIOD_MS   5
#define MOTORCONTROL_PERIOD_MS 5
#define ENCODER_PERIOD_MS 5
#define UART_PERIOD_MS 50
#define DT_TIME 0.005f

extern PID_TypeDef PosPID1, PosPID2;
extern PID_TypeDef SpeedPID1, SpeedPID2;

extern double CurPos1, CurSpeed1, PosPIDOut1, SpeedPIDOut1, DesiredPos1, DesiredSpeed1;
extern double CurPos2, CurSpeed2, PosPIDOut2, SpeedPIDOut2, DesiredPos2, DesiredSpeed2;

extern double Kp1, Ki1, Kd1;
extern double Kp2, Ki2, Kd2;

extern int32_t PID_sampletime;

extern int32_t g_encCount1;
extern int32_t g_encCount2;
extern float g_speed1_mps;
extern float g_speed2_mps;
extern float g_revCount1;
extern float g_revCount2;

#endif /* INC_ROBOT_PARAMS_H_ */
