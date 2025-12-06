#include "Calc_Speed.h"
#include <math.h>

void calcSpeed() {
  
  // --- Bước 1: Lấy thời gian (Delta-Time) ---
  unsigned long now = millis();
  unsigned long dt = now - lastCalcMs;

  if (dt < calcPeriodMs) return;

  // --- Bước 2: Lặp qua cả 2 bánh xe ---
  for (int i = 0; i < 2; ++i) {
    long c = encCount[i];
    long dc = c - lastCount[i];
    lastCount[i] = c;
    
    double dc_scaled = (double)dc * (double)encScale;
    motFrac[i] += dc_scaled;
    
    long addMot = 0;
    if (motFrac[i] >= 1.0) { 
      addMot = (long)floor(motFrac[i]); 
      motFrac[i] -= (double)addMot; }
    else if (motFrac[i] <= -1.0) { 
      addMot = (long)ceil (motFrac[i]); 
      motFrac[i] -= (double)addMot; }
    if (addMot != 0) 
      encCountMotorScaled[i] += addMot;

    // --- Bước 4: Tính toán RPM (Quan trọng nhất) ---

    // 1. Tính CPS (Counts Per Second - Xung trên giây).
    meas_cps[i] = (float)(dc_scaled * 1000.0 / (double)dt);
    
    // 2. Tính RPS_MOTOR
    float rps_motor = (CPRx4 > 0) ? (meas_cps[i] / (float)CPRx4) : 0.0f;
    
    // 3. Tính RPS_OUT
    float rps_out = (gearRatio > 0.f)? (rps_motor / gearRatio) : 0.0f;
    
    // 4. Tính RPM_OUT
    meas_rpm_out[i] = rps_out * 60.0f;
  
    // --- Bước 5: Tính toán Góc (Phụ trợ) ---
    
    // Tính toán góc quay của TRỤC ĐỘNG CƠ (quy về 0-360 độ).
    if (CPRx4 > 0) {
      goc_encoder_deg[i] += (float)(dc_scaled * 360.0 / (double)CPRx4);
      while (goc_encoder_deg[i] >= 360.0f) goc_encoder_deg[i] -= 360.0f;
      while (goc_encoder_deg[i] <    0.0f) goc_encoder_deg[i] += 360.0f;
    }

    // Tính toán góc quay của BÁNH XE (quy về 0-360 độ).
    double CPR_OUT_user = (double)CPR_USER * (double)gearRatio;
    if (CPR_OUT_user > 0.0) {
      goc_out_deg[i] += (float)(dc_scaled * 360.0 / CPR_OUT_user);
      while (goc_out_deg[i] >= 360.0f) goc_out_deg[i] -= 360.0f;
      while (goc_out_deg[i] < 0.0f) goc_out_deg[i] += 360.0f;
    }

    // (Phần code này dùng để tích lũy số vòng quay lẻ của TRỤC BÁNH XE)
    if (gearRatio > 0.0f) {
      double dcout_raw = dc_scaled / (double)gearRatio;
      encOutFrac[i] += dcout_raw;
      long addOut = 0;
      if (encOutFrac[i] >= 1.0) { 
        addOut = (long)floor(encOutFrac[i]); 
        encOutFrac[i] -= (double)addOut; 
      } else if (encOutFrac[i] <= -1.0) { 
        addOut = (long)ceil (encOutFrac[i]); 
        encOutFrac[i] -= (double)addOut; 
      } if (addOut != 0) 
        encCountOut_raw[i] += addOut;
    }
  } // Kết thúc vòng lặp for (hết 2 bánh)

  // --- Bước 6: Cập nhật mốc thời gian ---
  
  // Lưu mốc thời gian này để dùng cho lần chạy 'calcSpeed()' tiếp theo (cách đây 20ms)
  lastCalcMs = now;
}