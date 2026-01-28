/*
 * pid_params_flash.h
 *
 *  Created on: Jan 24, 2026
 *      Author: khoi2
 */

#ifndef INC_PID_PARAMS_FLASH_H_
#define INC_PID_PARAMS_FLASH_H_

#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
  float kp1, ki1, kd1;
  float kp2, ki2, kd2;
} PIDGains_t;

// Load gains from flash.
// Returns true if valid data found; false -> use your defaults.
bool PIDFlash_Load(PIDGains_t *out);

// Save gains to flash (call only when user presses "Save"/"Commit").
// Returns true on success.
bool PIDFlash_Save(const PIDGains_t *in);

#endif /* INC_PID_PARAMS_FLASH_H_ */
