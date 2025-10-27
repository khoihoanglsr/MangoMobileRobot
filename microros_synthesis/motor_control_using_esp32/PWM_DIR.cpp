#include "PWM_DIR.h"

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
  bool dir_logic = forward ^ invert_dir[i];   // Hỗ trợ đảo chiều logic
  digitalWrite(dirPinOf(i), dir_logic ? HIGH : LOW);
}

// ===================== ĐIỀU KHIỂN PWM ===================== //
// Xuất PWM 8-bit (0..255) cho motor i (độ lớn, không dấu)
void setPWM_8bit_mag(int i, uint8_t mag8) {
  if (i < 0 || i > 1) return;
  // Map 8-bit -> độ phân giải PWM_MAX (ví dụ 10-bit = 1023)
  uint32_t duty = (uint32_t)mag8 * PWM_MAX / 255U;
  analogWrite(pwmPinOf(i), duty);
  outCmd[i] = (int)duty; // lưu lại để telemetry
}

// ===================== DỪNG KHẨN ===================== //
void stopHard() {
  setPWM_8bit_mag(0, 0);
  setPWM_8bit_mag(1, 0);
}

// ===================== HÀM CHÍNH: ÁP LỆNH RA MOTOR ===================== //
void applyDutyFromCmd_modeaware(int i, int cmd) {
  if (i < 0 || i > 1) return;

  // ---- Vùng chết tín hiệu ----
  static bool lastFwd[2] = {true, true};

  int mag = abs(cmd);
  int dz = (controlMode == MODE_PULSE) ? CMD_DEAD : CMD_DEAD_PID;
  if (mag <= dz) mag = 0;

  // ---- Deadzone chiều quay (tránh đảo chiều liên tục khi dao động quanh 0) ----
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

  // ---- Ghi log để debug ----
  if (echoCmd) {
    Serial.print(F("[M"));
    Serial.print(i);
    Serial.print(F("] cmd="));
    Serial.print(cmd);
    Serial.print(F(" -> PWM8="));
    Serial.print(mag);
    Serial.print(F(" | Chieu="));
    Serial.println((fwd ^ invert_dir[i]) ? "THUAN" : "NGHICH");
  }
}
