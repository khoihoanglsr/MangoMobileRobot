/*
 * ps2xlib.h
 *
 *  Created on: Jan 20, 2026
 *      Author: khoi2
 */

#ifndef INC_PS2XLIB_H_
#define INC_PS2XLIB_H_

#include "stm32f7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- Button bit masks (active HIGH in ps2->buttons; raw from pad is active LOW) ----
#define PSB_SELECT      (1u<<0)
#define PSB_L3          (1u<<1)
#define PSB_R3          (1u<<2)
#define PSB_START       (1u<<3)
#define PSB_UP          (1u<<4)
#define PSB_RIGHT       (1u<<5)
#define PSB_DOWN        (1u<<6)
#define PSB_LEFT        (1u<<7)

#define PSB_L2          (1u<<8)
#define PSB_R2          (1u<<9)
#define PSB_L1          (1u<<10)
#define PSB_R1          (1u<<11)
#define PSB_TRIANGLE    (1u<<12)
#define PSB_CIRCLE      (1u<<13)
#define PSB_CROSS       (1u<<14)
#define PSB_SQUARE      (1u<<15)

// ---- Analog indices in returned data (when analog mode is on) ----
typedef enum {
  PSS_RX = 0,  // Right stick X
  PSS_RY = 1,  // Right stick Y
  PSS_LX = 2,  // Left stick X
  PSS_LY = 3   // Left stick Y
} ps2_stick_t;

// ---- Debug / error codes for bring-up ----
typedef enum {
  PS2_OK = 0,

  // Low-level transport problems
  PS2_FAIL_SPI,         // HAL_SPI_TransmitReceive != HAL_OK
  PS2_FAIL_SIGNATURE,   // rx[2] != 0x5A

  // Higher-level stage failures in PS2X_ConfigGamepad
  PS2_FAIL_POLL1,
  PS2_FAIL_ENTER_CFG,
  PS2_FAIL_SET_ANALOG,
  PS2_FAIL_RUMBLE,
  PS2_FAIL_EXIT_CFG,
  PS2_FAIL_POLL2
} PS2_Error;

// Main handle
typedef struct {
  SPI_HandleTypeDef *hspi;

  // Raw response / command buffers
  uint8_t rx[21];
  uint8_t tx[21];
  uint8_t rx_len;

  // Parsed state
  uint16_t buttons_raw;   // active LOW bits from controller
  uint16_t buttons;       // converted to active HIGH pressed bits
  uint16_t last_buttons;

  // Analog bytes (0..255)
  uint8_t analog[4];      // RX, RY, LX, LY

  // Mode bytes
  uint8_t id;             // rx[1]
  bool    analog_enabled; // inferred from id (0x73 common)

  // ---- Debug fields (for bring-up) ----
  PS2_Error         last_err;        // last failure reason / stage
  HAL_StatusTypeDef last_spi_status; // last HAL SPI return code
  uint8_t           last_rx0;        // rx[0] from last poll
  uint8_t           last_rx1;        // rx[1] from last poll (ID)
  uint8_t           last_rx2;        // rx[2] from last poll (signature)
} PS2X_Handle;

// API
void PS2X_Init(PS2X_Handle *ps2, SPI_HandleTypeDef *hspi);

// Returns true if controller replies with valid signature (0x5A) and config succeeded.
bool PS2X_ConfigGamepad(PS2X_Handle *ps2, bool enable_analog, bool enable_rumble);

// Polls controller. If enable_rumble was configured, you can set motors each poll.
bool PS2X_ReadGamepad(PS2X_Handle *ps2, uint8_t motor_small, uint8_t motor_large);

// Convenience helpers
static inline bool PS2X_Button(const PS2X_Handle *ps2, uint16_t mask) {
  return (ps2->buttons & mask) != 0;
}

static inline bool PS2X_ButtonPressed(const PS2X_Handle *ps2, uint16_t mask) {
  return ((ps2->buttons & mask) != 0) && ((ps2->last_buttons & mask) == 0);
}

static inline bool PS2X_ButtonReleased(const PS2X_Handle *ps2, uint16_t mask) {
  return ((ps2->buttons & mask) == 0) && ((ps2->last_buttons & mask) != 0);
}

static inline uint8_t PS2X_Analog(const PS2X_Handle *ps2, ps2_stick_t s) {
  return ps2->analog[(int)s];
}

// Optional: decode error codes quickly in debugger / logs
static inline const char* PS2X_ErrorStr(PS2_Error e) {
  switch (e) {
    case PS2_OK:              return "PS2_OK";
    case PS2_FAIL_SPI:        return "PS2_FAIL_SPI";
    case PS2_FAIL_SIGNATURE:  return "PS2_FAIL_SIGNATURE";
    case PS2_FAIL_POLL1:      return "PS2_FAIL_POLL1";
    case PS2_FAIL_ENTER_CFG:  return "PS2_FAIL_ENTER_CFG";
    case PS2_FAIL_SET_ANALOG: return "PS2_FAIL_SET_ANALOG";
    case PS2_FAIL_RUMBLE:     return "PS2_FAIL_RUMBLE";
    case PS2_FAIL_EXIT_CFG:   return "PS2_FAIL_EXIT_CFG";
    case PS2_FAIL_POLL2:      return "PS2_FAIL_POLL2";
    default:                  return "PS2_UNKNOWN";
  }
}

#ifdef __cplusplus
}
#endif

#endif /* INC_PS2XLIB_H_ */
