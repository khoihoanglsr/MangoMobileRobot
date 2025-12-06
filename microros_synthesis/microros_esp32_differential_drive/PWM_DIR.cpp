#include "PWM_DIR.h"
#include "Global.h"

// Helper: trả về chân DIR/PWM theo chỉ số motor
static inline int dirPinOf(int i) {
  return (i == 0) ? M0_DIR_PIN : M1_DIR_PIN;
}
static inline int pwmPinOf(int i) {
  return (i == 0) ? M0_PWM_PIN : M1_PWM_PIN;
}

// ===================== ĐIỀU KHIỂN CHIỀU ===================== //
// Đặt chiều cho motor i (true = forward, false = reverse)
void setDir(int i, bool forward) {
  if (i < 0 || i > 1) return;
  bool dir_logic = forward ^ invert_dir[i];   // Hỗ trợ đảo chiều logic phần cứng
  digitalWrite(dirPinOf(i), dir_logic ? HIGH : LOW);
}

// ===================== ĐIỀU KHIỂN PWM ===================== //
// Xuất PWM 8-bit (0..255) cho motor i (độ lớn, không dấu)
void setPWM_8bit_mag(int i, uint8_t mag8) {
  if (i < 0 || i > 1) return;
  // Map 8-bit -> độ phân giải PWM_MAX (ví dụ 10-bit = 1023)
  uint32_t duty = (uint32_t)mag8 * PWM_MAX / 255U;
  analogWrite(pwmPinOf(i), duty);
}

// ===================== DỪNG KHẨN ===================== //
void stopHard() {
  setPWM_8bit_mag(0, 0);
  setPWM_8bit_mag(1, 0);
  
  sp_rpm[M_LEFT] = 0.0f;
  sp_rpm[M_RIGHT] = 0.0f;
}

// ===================== HÀM CHÍNH: ÁP LỆNH RA MOTOR ===================== //
void applyDutyFromCmd_modeaware(int i, int cmd) {
  if (i < 0 || i > 1) return;

  // ---- Vùng chết tín hiệu ----
  static bool lastFwd[2] = {true, true};

  int dz  = (controlMode == MODE_PULSE) ? CMD_DEAD : CMD_DEAD_PID;
  int mag = abs(cmd);
  if (mag <= dz) mag = 0;
 
  bool fwd;
  if (mag == 0) {
    // Giữ nguyên chiều cũ khi lệnh nhỏ
    fwd = lastFwd[i];
  } else {
    fwd = (cmd >= 0);
    lastFwd[i] = fwd;
  }

  // ---- Giới hạn giá trị PWM 0..255 ----
  if (mag > 255) mag = 255;

  // ---- Áp dụng chiều & PWM ----
  setDir(i, fwd);                        // có invert_dir[] trong setDir()
  setPWM_8bit_mag(i, (uint8_t)mag);

  // ---- Cập nhật telemetry ở thang 8-bit có dấu (−255..+255) ----
  outCmd[i] = fwd ? mag : -mag;
}
