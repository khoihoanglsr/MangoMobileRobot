#include "Tinh_toan_TocDoGoc.h"
#include <math.h>

void calcSpeed(){
  unsigned long now = millis();
  unsigned long dt = now - lastCalcMs;
  if (dt < calcPeriodMs) return;

  // Tính cho cả 2 bánh: i = 0 (M0/trái), i = 1 (M1/phải)
  for (int i = 0; i < 2; ++i) {
    long c  = encCount[i];
    long dc = c - lastCount[i];
    lastCount[i]  = c;

    // scale xung (nếu bạn đang dùng tỉ lệ nội suy)
    double dc_scaled = (double)dc * (double)encScale;

    // Tích lũy phần lẻ vòng mô-tơ (motor side)
    motFrac[i] += dc_scaled;
    long addMot = 0;
    if (motFrac[i] >= 1.0)       { addMot = (long)floor(motFrac[i]); motFrac[i] -= (double)addMot; }
    else if (motFrac[i] <= -1.0) { addMot = (long)ceil (motFrac[i]); motFrac[i] -= (double)addMot; }
    if (addMot != 0) encCountMotorScaled[i] += addMot;

    // cps và rpm OUT
    meas_cps[i] = (float)(dc_scaled * 1000.0 / (double)dt);
    float rps_motor = (CPRx4 > 0)      ? (meas_cps[i] / (float)CPRx4) : 0.0f;
    float rps_out   = (gearRatio > 0.f)? (rps_motor / gearRatio)      : 0.0f;
    meas_rpm_out[i] = rps_out * 60.0f;

    // Góc encoder (deg) – quy về [0..360)
    if (CPRx4 > 0) {
      goc_encoder_deg[i] += (float)(dc_scaled * 360.0 / (double)CPRx4);
      while (goc_encoder_deg[i] >= 360.0f) goc_encoder_deg[i] -= 360.0f;
      while (goc_encoder_deg[i] <    0.0f) goc_encoder_deg[i] += 360.0f;
    }

    // Góc OUT (deg) theo CPR hiển thị (CPR_USER * gearRatio)
    double CPR_OUT_user = (double)CPR_USER * (double)gearRatio;
    if (CPR_OUT_user > 0.0) {
      goc_out_deg[i] += (float)(dc_scaled * 360.0 / CPR_OUT_user);
      while (goc_out_deg[i] >= 360.0f) goc_out_deg[i] -= 360.0f;
      while (goc_out_deg[i] <    0.0f) goc_out_deg[i] += 360.0f;
    }

    // Đếm xung trục OUT (raw) qua tỉ số truyền
    if (gearRatio > 0.0f) {
      double dcout_raw = dc_scaled / (double)gearRatio;
      encOutFrac[i] += dcout_raw;
      long addOut = 0;
      if (encOutFrac[i] >= 1.0)        { addOut = (long)floor(encOutFrac[i]); encOutFrac[i] -= (double)addOut; }
      else if (encOutFrac[i] <= -1.0)  { addOut = (long)ceil (encOutFrac[i]); encOutFrac[i] -= (double)addOut; }
      if (addOut != 0) encCountOut_raw[i] += addOut;
    }
  }

  // cập nhật mốc thời gian sau khi xử lý cả 2 bánh
  lastCalcMs = now;
}
