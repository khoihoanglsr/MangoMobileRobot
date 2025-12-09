#ifndef ENCODER_PROCESS_H
#define ENCODER_PROCESS_H

#include "Global.h"

// ISR cho Encoder Motor 0 (trái)
void IRAM_ATTR encoderISR0A();  // CHANGE trên M0_ENC_A_PIN
void IRAM_ATTR encoderISR0B();  // CHANGE trên M0_ENC_B_PIN

// ISR cho Encoder Motor 1 (phải)
void IRAM_ATTR encoderISR1A();  // CHANGE trên M1_ENC_A_PIN
void IRAM_ATTR encoderISR1B();  // CHANGE trên M1_ENC_B_PIN

// --- Gán PCNT unit cho từng motor ---
#define M0_PCNT_UNIT   PCNT_UNIT_0
#define M1_PCNT_UNIT   PCNT_UNIT_1

// Giới hạn đếm (16-bit signed)
#define PCNT_H_LIM     32767
#define PCNT_L_LIM    -32768

#endif
