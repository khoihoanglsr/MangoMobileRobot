#include <Arduino.h>
#include <math.h>
#include "driver/pcnt.h"

// ---- Your internal libs ----
#include "Global.h" 
#include "PWM_DIR.h"
#include "Encoder_Process.h"
#include "Calc_Speed.h" 

// ---- Khai báo hàm helper (từ code gốc) ----
#ifndef ANALOG_WRITE_INIT_BOTH_DEFINED
#define ANALOG_WRITE_INIT_BOTH_DEFINED

static inline void analogWriteInitBoth() {
  // Giả định PWM_FREQ_HZ = 2000 (từ Global.cpp)
  // Biến PWM_BITS và các chân được lấy từ Global.h
  // Giá trị 2000Hz được giả định là giá trị của PWM_FREQ_HZ
  analogWriteResolution(M0_PWM_PIN, PWM_BITS);
  analogWriteResolution(M1_PWM_PIN, PWM_BITS);
  analogWriteFrequency(M0_PWM_PIN, 2000); 
  analogWriteFrequency(M1_PWM_PIN, 2000);
  analogWrite(M0_PWM_PIN, 0);
  analogWrite(M1_PWM_PIN, 0);
}
#endif

// Lệnh PWM cố định (Điều khiển Vòng hở)
static const int OPEN_LOOP_PWM = 250 ; // Mức công suất cố định (0-255)
unsigned long lastPrintTime = 0;   // thời điểm lần in cuối
const unsigned long printInterval = 500;  // 500 ms = 0.5s

// Biến đếm dài hạn (kiểu long)
int oldEncCountLeft = 0;
int oldEncCountRight = 0;

volatile long encCountLeft  = 0;
volatile long encCountRight = 0;

// Mục tiêu: Quay N vòng (N * 7000 xung X1)
  const long Round = 10;
  const float ERR_COEFF = 0.0584L;
  static const long TARGET_PULSES = 7000L * Round;

// ===================== VÒNG LẶP ĐIỀU KHIỂN CHÍNH (Vòng hở) =====================31909608 +21682   interval: 733501
left: -10588961 +-22629   right: 31931324 +21716   interval: 734001
left: -10611613 +-22652   right: 31953087 

// Hàm đọc PCNT và cộng dồn vào biến long
void updateEncoderLong(pcnt_unit_t unit, volatile long &accum) {
  int16_t raw_cnt = 0;

  // Đọc giá trị 16-bit hiện tại từ PCNT
  pcnt_get_counter_value(unit, &raw_cnt);

  // Xóa counter để tránh tràn
  pcnt_counter_clear(unit);
  // Cộng dồn vào biến long31909608 +21682   interval: 733501
left: -10588961 +-22629   right: 31931324 +21716   interval: 734001
left: -10611613 +-22652   right: 31953087 
  accum += raw_cnt;
}

// Hàm cấu hình PCNT cho 1 encoder: A = pulse, B = ctrl
void setup_pcnt_encoder(pcnt_unit_t unit, int pulse_gpio, int ctrl_gpio) {
  pcnt_config_t pcnt_config = {};

  pcnt_config.pulse_gpio_num = pulse_gpio;   31909608 +21682   interval: 733501
left: -10588961 +-22629   right: 31931324 +21716   interval: 734001
left: -10611613 +-22652   right: 31953087 // Chân A
  pcnt_config.ctrl_gpio_num  = ctrl_gpio;    // Chân B
  pcnt_config.channel        = PCNT_CHANNEL_0;
  pcnt_config.unit           = unit;
  // Đếm khi có cạnh lên ở pulse
  pcnt_config.pos_mode       = PCNT_COUNT_INC;   // Cạnh lên: +1
  pcnt_config.neg_mode       = PCNT_COUNT_DIS;   // Cạnh xuống: không đếm

  // Điều khiển hướng bằng mức logic của CTRL (B)
  // B = 1 (HIGH): KEEP -> vẫn +1
  // B = 0 (LOW): REVERSE -> đổi dấu -> -1
  pcnt_config.hctrl_mode     = PCNT_MODE_KEEP;      // CTRL = HIGH
  pcnt_config.lctrl_mode     = PCNT_MODE_REVERSE;   // CTRL = LOW

  // Giới hạn
  pcnt_config.counter_h_lim  = PCNT_H_LIM;
  pcnt_config.counter_l_lim  = PCNT_L_LIM;

  // Áp dụng cấu hình
  pcnt_unit_config(&pcnt_config);

  // Bật glitch filter để lọc nhiễu xung quá ngắn
  // Giá trị là số chu kỳ clock APB (80 MHz); ví dụ 400ms ~ 5 us
  pcnt_set_filter_value(unit, 400);
  pcnt_filter_enable(unit);

  // Reset counter
  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);
  pcnt_counter_resume(unit);
}


// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  5131 +0   interval: 10001
left: 75041 +0   right: 75
  // --- Khởi tạo Phần cứng ---
  pinMode(M0_DIR_PIN, OUTPUT);
  pinMode(M1_DIR_PIN, OUTPUT);
  digitalWrite(M0_DIR_PIN, LOW);
  digitalWrite(M1_DIR_PIN, LOW);

  analogWriteInitBoth();  
  stopHard();          

  pinMode(M0_ENC_A_PIN, INPUT);
  pinMode(M0_ENC_B_PIN, INPUT);
  pinMode(M1_ENC_A_PIN, INPUT);
  pinMode(M1_ENC_B_PIN, INPUT);

  setup_pcnt_encoder(M0_PCNT_UNIT, M0_ENC_A_PIN, M0_ENC_B_PIN);
  setup_pcnt_encoder(M1_PCNT_UNIT, M1_ENC_A_PIN, M1_ENC_B_PIN);

  setPWM_8bit_mag(M_LEFT,OPEN_LOOP_PWM);
  setPWM_8bit_mag(M_RIGHT,OPEN_LOOP_PWM);
}

// ===================== LOOP =====================
void loop() {
  updateEncoderLong(PCNT_UNIT_0, encCountLeft);
  updateEncoderLong(PCNT_UNIT_1, encCountRight);
  

  // 1. VÒNG LẶP ĐIỀU KHIỂN
  if (millis() - lastCalcMs >= calcPeriodMs) {
    lastCalcMs = millis();
  }
  
  // 2. LOGIC ĐẶT MỤC TIÊU MỚI (Lặp lại lệnh N vòng quay)
  static unsigned long last_change_ms = 0;

  unsigned long now = millis();

  if (now - lastPrintTime >= printInterval) {
      lastPrintTime = now;

      Serial.print("left: ");
      Serial.print(encCountLeft);
      Serial.print(" +");
      Serial.print(encCountLeft - oldEncCountLeft);
      Serial.print("   right: ");
      Serial.print(encCountRight);
      Serial.print(" +");
      Serial.print(encCountRight - oldEncCountRight);
      Serial.print("   interval: ");
      Serial.println(now);

      oldEncCountLeft = encCountLeft;
      oldEncCountRight = encCountRight ;

  }

  if (encCountLeft >= TARGET_PULSES)
  {
    setPWM_8bit_mag(M_LEFT,0);
  }


  if (encCountRight >= TARGET_PULSES) {
    setPWM_8bit_mag(M_RIGHT,0);
  }
      
  delay(2); 
}