#include "PID_TocDo.h"
#include <math.h>

void updatePIDSpeed(){
  if (armed && controlMode==MODE_SPEED && holdEnable) {
    static uint32_t lastPidMs = 0;
    static float measFilt = 0.0f;   // lọc rpm_avg
    static bool  filtInit = false;
    static float prevMeas = 0.0f;
    static float dmeas_filt = 0.0f;
    const  float ALPHA = 0.25f;

    uint32_t nowMs = millis();
    float dt_s = (lastPidMs==0) ? (calcPeriodMs/1000.0f) : (nowMs - lastPidMs)/1000.0f;
    if (dt_s < 1e-4f) dt_s = calcPeriodMs/1000.0f;
    lastPidMs = nowMs;

    // === ĐO: dùng rpm trung bình của 2 bánh ===
    float rpm_avg = 0.5f * (meas_rpm_out[0] + meas_rpm_out[1]);

    // === Khởi tạo & lọc số đo ===
    if (!filtInit) { 
      measFilt   = rpm_avg; 
      prevMeas   = rpm_avg; 
      dmeas_filt = 0.0f; 
      sp_rpm_cmd = sp_rpm; 
      filtInit   = true; 
    } else { 
      measFilt  += ALPHA * (rpm_avg - measFilt); 
    }

    // === Giới hạn tốc độ thay đổi setpoint (ramp) ===
    float maxStep = SP_RATE_RPM_PER_S * dt_s;
    float deltaSP = sp_rpm - sp_rpm_cmd;
    if      (deltaSP >  maxStep) sp_rpm_cmd += maxStep;
    else if (deltaSP < -maxStep) sp_rpm_cmd -= maxStep;
    else                         sp_rpm_cmd  = sp_rpm;

    // === Sai số & đạo hàm trên tín hiệu đo đã lọc ===
    float err   = sp_rpm_cmd - measFilt;
    float dmeas = (measFilt - prevMeas) / dt_s;
    prevMeas    = measFilt;
    dmeas_filt += KD_ALPHA * (dmeas - dmeas_filt);

    // === Thành phần PD (đúng tên biến: u_pd) ===
    float u_pd = kp * err - kd * dmeas_filt;

    // === Feedforward ===
    float u_ff = KFF * sp_rpm_cmd;
    if (fabsf(sp_rpm_cmd) > 1.0f) {
      u_ff += (sp_rpm_cmd > 0 ? +MIN_PWM_FF : -MIN_PWM_FF);
    }

    // === Anti-windup cho tích phân ===
    const float UMAX = 255.0f, UMIN = -255.0f;
    float integ_new = integ + err * dt_s;
    float u_try     = u_pd + u_ff + ki * integ_new;
    bool satPos     = (u_try > UMAX);
    bool satNeg     = (u_try < UMIN);

    // Ngăn tích phân tích lũy theo hướng bão hòa
    if ( (satPos && err > 0) || (satNeg && err < 0) ) {
      integ_new = integ;
    }

    // Kẹp tích phân theo biên còn lại
    if (ki > 1e-6f) {
      float iMax = (UMAX - u_pd - u_ff) / ki;
      float iMin = (UMIN - u_pd - u_ff) / ki;
      if (integ_new > iMax) integ_new = iMax;
      if (integ_new < iMin) integ_new = iMin;
    }
    integ = integ_new;

    // === Tổng điều khiển (sửa lỗi: u_vpd -> u_pd) ===
    float u = u_pd + u_ff + ki * integ;
    if (u > UMAX) u = UMAX;
    if (u < UMIN) u = UMIN;

    // Lưu output PID chung
    last_u_pid = u;

    // Áp cùng một lệnh cho cả 2 bánh (giữ nguyên triết lý PID chung)
    int u_cmd = (int)lroundf(u);
    outCmd[0] = u_cmd;
    outCmd[1] = u_cmd;

  } else {
    // Mở vòng / không giữ: pass-through per-motor
    outCmd[0] = cmdOpenLoop[0];
    outCmd[1] = cmdOpenLoop[1];
    // last_u_pid có thể phản ánh trung bình lệnh để tiện log
    last_u_pid = 0.5f * ( (float)outCmd[0] + (float)outCmd[1] );
  }
}
