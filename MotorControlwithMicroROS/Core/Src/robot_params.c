/*
 * robot_params.c
 *
 *  Created on: Dec 3, 2025
 *      Author: khoi2
 */
#include "robot_params.h"

int32_t g_encCount1 = 0;
int32_t g_encCount2 = 0;
float g_speed1_mps = 0.0f;
float g_speed2_mps = 0.0f;

PID_TypeDef PosPID1, PosPID2;
PID_TypeDef SpeedPID1, SpeedPID2;

double CurPos1 = 0, CurSpeed1 = 0, PosPIDOut1 = 0, SpeedPIDOut1 = 0, DesiredPos1 = 0, DesiredSpeed1 = 0;
double CurPos2 = 0, CurSpeed2 = 0, PosPIDOut2 = 0, SpeedPIDOut2 = 0, DesiredPos2 = 0, DesiredSpeed2 = 0;

double Kp1 = 2500, Ki1 = 200, Kd1 = 0;
double Kp2 = 2500, Ki2 = 200, Kd2 = 0;

float g_revCount1 = 0.0f;
float g_revCount2 = 0.0f;



