#ifndef ENCODER_XULY_H
#define ENCODER_XULY_H

#include "Global.h"

// ISR cho Encoder Motor 0 (trái)
void IRAM_ATTR encoderISR0A();  // CHANGE trên M0_ENC_A_PIN
void IRAM_ATTR encoderISR0B();  // CHANGE trên M0_ENC_B_PIN

// ISR cho Encoder Motor 1 (phải)
void IRAM_ATTR encoderISR1A();  // CHANGE trên M1_ENC_A_PIN
void IRAM_ATTR encoderISR1B();  // CHANGE trên M1_ENC_B_PIN

#endif
