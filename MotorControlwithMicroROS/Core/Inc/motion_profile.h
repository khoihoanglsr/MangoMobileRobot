/*
 * motion_profile.h
 *
 *  Created on: Jan 24, 2026
 *      Author: khoi2
 */

#ifndef INC_MOTION_PROFILE_H_
#define INC_MOTION_PROFILE_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MP_MODE_VELOCITY = 0,
    MP_MODE_POSITION = 1
} MP_Mode_t;

extern volatile MP_Mode_t modCtrl;

typedef struct {
    float dt;        // [s] sample time, e.g., 0.005
    float vmax;      // [unit/s]
    float amax;      // [unit/s^2]
    float vel_eps;   // [unit/s] small threshold to avoid chattering
    float pos_eps;   // [unit]   threshold to "snap" to target
} MP_Config_t;

typedef struct {
    float pos_ref;   // internal reference position
    float vel_ref;   // internal reference velocity
} MP_State_t;

typedef struct {
    float pos_next;
    float vel_next;
    float acc_used;
    uint8_t reached; // 1 when target reached (position mode) or velocity reached (velocity mode)
} MP_Output_t;

void MP_Init(MP_State_t* st, float init_pos, float init_vel);

/**
 * Step in VELOCITY mode:
 *  - Ramps vel_ref toward v_des with acceleration limit.
 *  - Also integrates pos_ref using the new vel_ref.
 */
MP_Output_t MP_StepVelocity(const MP_Config_t* cfg, MP_State_t* st, float v_des);

/**
 * Step in POSITION mode:
 *  - Drives to pos_des using acceleration-limited profile (trapezoidal/triangular),
 *    respecting vmax/amax and braking distance.
 */
MP_Output_t MP_StepPosition(const MP_Config_t* cfg, MP_State_t* st, float pos_des);

#ifdef __cplusplus
}
#endif

#endif /* INC_MOTION_PROFILE_H_ */
