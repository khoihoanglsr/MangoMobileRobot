// AdaptiveRulePID.cpp — Rule-based adaptive tuning cho PID tốc độ
#include <Arduino.h>
#include "Tien_ich.h"         // chứa extern kp, ki, kd, sp_rpm, ...
#include "AdaptiveRulePID.h"

// Giới hạn an toàn (tùy chỉnh theo thực nghiệm)
static const float KP_MIN = 0.30f, KP_MAX = 2.50f;
static const float KI_MIN = 0.00f, KI_MAX = 1.20f;
static const float KD_MIN = 0.00f, KD_MAX = 0.40f;

// Hệ số học "nhẹ" (đơn vị theo giây)
static const float AKP_UP   = 0.60f;   // tăng Kp khi sai số lớn
static const float AKI_UP   = 0.20f;   // tăng Ki khi sai số lớn
static const float BKP_DN   = 1.00f;   // giảm Kp khi dao động
static const float CKD_UP   = 0.40f;   // tăng Kd khi dao động

// Ngưỡng/suy luận nhanh
static const float E_OFFS   = 10.0f;   // +10 RPM để tránh 0 setpoint
static const int   OSC_HITS = 4;       // số lần đổi dấu e liên tiếp => coi là dao động
static const float SAT_DECAY = 0.98f;  // xả tích phân khi bão hòa

// Biến trạng thái cục bộ
static float   e_prev = 0.0f;
static int     osc_cnt = 0;
static uint32_t last_ms = 0;

void ruleTuning_step(){
  // Chỉ áp dụng trong chế độ PID tốc độ
  if (controlMode != MODE_SPEED) return;

  uint32_t now = millis();
  float dt = (last_ms == 0) ? 0.02f : (now - last_ms) / 1000.0f;
  if (dt <= 0.0f) dt = 0.02f;
  last_ms = now;

  // Sai số trung bình hai bánh (giả định setpoint chung sp_rpm)
  float meas_avg = 0.5f * (meas_rpm_out[0] + meas_rpm_out[1]);
  float sp_eff   = (fabsf(sp_rpm) < 1e-3f) ? 0.0f : sp_rpm;   // setpoint hiệu lực
  float e        = sp_eff - meas_avg;

  // Phát hiện dao động: e đổi dấu => đếm
  if (e * e_prev < 0.0f) osc_cnt++;
  else if (osc_cnt > 0)  osc_cnt--;

  // 1) Sai số lớn -> tăng Kp, Ki
  //    Ngưỡng “lớn” tỉ lệ theo |setpoint| + bù E_OFFS
  float e_big_th = 0.40f * fabsf(sp_eff) + E_OFFS;  // ví dụ 40% setpoint + 10 RPM
  if (fabsf(e) > e_big_th) {
    kp += AKP_UP * dt;
    ki += AKI_UP * dt;
  }

  // 2) Dao động liên tiếp -> giảm Kp, tăng Kd để dập
  if (osc_cnt >= OSC_HITS) {
    kp -= BKP_DN * dt;
    kd += CKD_UP * dt;
    osc_cnt = 0; // reset để tránh tăng/giảm quá tay
  }

  // 3) Chống windup tích phân khi bão hòa đầu ra
  const int SAT = 255;
  if (abs(outCmd[0]) >= SAT || abs(outCmd[1]) >= SAT) {
    integ *= SAT_DECAY;  // xả tích phân từ từ
  }

  // Kẹp biên an toàn
  if (kp < KP_MIN) kp = KP_MIN; if (kp > KP_MAX) kp = KP_MAX;
  if (ki < KI_MIN) ki = KI_MIN; if (ki > KI_MAX) ki = KI_MAX;
  if (kd < KD_MIN) kd = KD_MIN; if (kd > KD_MAX) kd = KD_MAX;

  e_prev = e;
}

