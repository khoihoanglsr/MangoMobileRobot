#include "Encoder_Process.h"
#include "global.h"
#include "soc/gpio_reg.h"

// Bảng giải mã x4 (giữ nguyên)
static const int8_t DRAM_ATTR qtab[16] = {
  0, -1, +1, 0,
 +1,  0,  0, -1,
 -1,  0,  0, +1,
  0, +1, -1, 0
};

// Đọc nhanh mức logic của 1 pin (ESP32: bank0 0..31, bank1 32..39)
static inline uint8_t IRAM_ATTR fastReadPin(uint8_t pin){
  if (pin < 32) {
    uint32_t in0 = REG_READ(GPIO_IN_REG);
    return (in0 >> pin) & 1U;
  } else {
    uint32_t in1 = REG_READ(GPIO_IN1_REG);
    return (in1 >> (pin - 32)) & 1U;
  }
}

// Đọc 2 bit A/B theo chỉ số motor (0: M0, 1: M1)
static inline uint8_t IRAM_ATTR readAB_bits_idx(int i){
  uint8_t a, b;
  if (i == 0) {
    a = fastReadPin(M0_ENC_A_PIN);
    b = fastReadPin(M0_ENC_B_PIN);
  } else {
    a = fastReadPin(M1_ENC_A_PIN);
    b = fastReadPin(M1_ENC_B_PIN);
  }
  return (uint8_t)((a << 1) | b); // 0..3
}

// Cập nhật encoder cho motor i
static inline void IRAM_ATTR encUpdate(int i){
  uint8_t curr = readAB_bits_idx(i);
  uint8_t idx  = (prevAB[i] << 2) | curr; // 0..15
  int8_t step  = qtab[idx];
  if (step) encCount[i] += step;
  prevAB[i] = curr;
}

// ISR cho từng cạnh A/B của M0
void IRAM_ATTR encoderISR0A(){ encUpdate(0); }
void IRAM_ATTR encoderISR0B(){ encUpdate(0); }

// ISR cho từng cạnh A/B của M1
void IRAM_ATTR encoderISR1A(){ encUpdate(1); }
void IRAM_ATTR encoderISR1B(){ encUpdate(1); }
