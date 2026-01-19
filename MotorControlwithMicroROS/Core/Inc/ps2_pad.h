/*
 * ps2_pad.h
 *
 *  Created on: Jan 14, 2026
 *      Author: khoi2
 */

#ifndef INC_PS2_PAD_H_
#define INC_PS2_PAD_H_

#include <stdint.h>
#include "usbh_core.h"


typedef struct
{
  int16_t lx, ly, rx, ry;   // -128..127 (tùy report)
  uint8_t square, cross, circle, triangle;
  uint8_t l1, r1, l2, r2;
  uint8_t select, start, l3, r3;
  uint8_t dpad_up, dpad_down, dpad_left, dpad_right;
  uint8_t connected;
} PS2Signals_t;

extern volatile PS2Signals_t g_ps2;

void PS2_HID_Init(void);
void PS2_HID_ProcessReports(void); // gọi trong task riêng (khuyến nghị)
void PS2_HID_PushReport(const uint8_t *data, uint16_t len);

#endif /* INC_PS2_PAD_H_ */
