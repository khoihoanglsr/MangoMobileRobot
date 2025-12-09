#include "Calc_Speed.h"
#include <math.h>

void calcSpeed() {
  
  // --- Bước 1: Lấy thời gian (Delta-Time) ---

  // Ghi lại mốc thời gian hiện tại (tính bằng mili-giây)
  unsigned long now = millis();
  
  // Tính khoảng thời gian (dt) đã trôi qua kể từ lần cuối chạy hàm này
  unsigned long dt = now - lastCalcMs;

  // Guard Clause: Bộ lọc (sampler).
  // Đảm bảo hàm này chỉ chạy ở một tần suất cố định (được định nghĩa bởi calcPeriodMs, vd: 20ms).
  // Nếu chưa đủ thời gian, thoát ngay để tiết kiệm CPU.
  if (dt < calcPeriodMs) return;

  // --- Bước 2: Lặp qua cả 2 bánh xe ---
  
  // Tính toán độc lập cho cả 2 bánh: i = 0 (M0/trái) và i = 1 (M1/phải)
  for (int i = 0; i < 2; ++i) {

    // --- Bước 3: Tính Delta-Count (Số xung thay đổi) ---

    // Đọc 'encCount[i]', đây là biến 'volatile' được cập nhật liên tục bởi hàm ngắt ISR (trong Encoder_Process.cpp)
    long c = encCount[i];
    
    // 'dc' (delta-count) là số xung đã thay đổi KỂ TỪ LẦN CUỐI HÀM NÀY CHẠY.
    // Đây là giá trị cốt lõi để tính vận tốc.
    long dc = c - lastCount[i];
    
    // Lưu lại tổng số xung hiện tại để dùng cho lần tính toán sau (cách đây 20ms)
    lastCount[i] = c;

    // Áp dụng hệ số bù (nếu có). Thường 'encScale' được đặt là 1.0 (trong Global.cpp)
    double dc_scaled = (double)dc * (double)encScale;

    // (Phần code này dùng để tích lũy số vòng quay lẻ của TRỤC ĐỘNG CƠ)
    // (Dùng cho các phép đo phụ, không bắt buộc cho RPM)
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
    //    Công thức: (Số xung thay đổi * 1000) / số mili-giây trôi qua.
    meas_cps[i] = (float)(dc_scaled * 1000.0 / (double)dt);
    
    // 2. Tính RPS_MOTOR (Vòng quay trên giây CỦA TRỤC ĐỘNG CƠ).
    //    Lấy CPS chia cho số xung trên 1 vòng động cơ (CPRx4, vd: 2000).
    float rps_motor = (CPRx4 > 0) ? (meas_cps[i] / (float)CPRx4) : 0.0f;
    
    // 3. Tính RPS_OUT (Vòng quay trên giây CỦA BÁNH XE).
    //    Lấy tốc độ động cơ chia cho tỷ số truyền (gearRatio, vd: 14.0).
    float rps_out = (gearRatio > 0.f)? (rps_motor / gearRatio) : 0.0f;
    
    // 4. Tính RPM_OUT (Vòng quay trên phút CỦA BÁNH XE).
    //    Đây là giá trị cuối cùng mà PID và Odometry sử dụng.
    //    Công thức: Vòng/giây * 60 = Vòng/phút.
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