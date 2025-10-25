#ifndef PWM_DIR_H
#define PWM_DIR_H

#include "Tien_ich.h"

/*
  ================== MÔ TẢ CHUNG ==================
  Thư viện này điều khiển 2 động cơ DC qua mạch cầu H
  sử dụng tín hiệu PWM + DIR độc lập cho từng bánh:

    - i = 0 → Motor Trái (M0)
    - i = 1 → Motor Phải (M1)

  Mỗi động cơ có 2 chân:
    + DIR: xác định chiều quay
    + PWM: điều khiển tốc độ (độ rộng xung)

  Các hàm hỗ trợ:
    - setDir(i, forward): đặt chiều quay motor i
    - setPWM_8bit_mag(i, mag8): đặt độ lớn PWM (0..255)
    - stopHard(): dừng cả hai motor ngay lập tức
    - applyDutyFromCmd_modeaware(i, cmd): áp lệnh điều khiển hoàn chỉnh
*/

/// ========== ĐẶT CHIỀU CHO MOTOR ==========
/// @param i: chỉ số motor (0 = trái, 1 = phải)
/// @param forward: true = quay thuận, false = quay nghịch
/// Ghi chú: tự động đảo chiều nếu invert_dir[i] = true
void setDir(int i, bool forward);


/// ========== XUẤT PWM 8-BIT ==========
/// @param i: chỉ số motor (0 = trái, 1 = phải)
/// @param mag8: độ lớn PWM 0..255 (không dấu)
/// Hàm sẽ map sang độ phân giải thật (PWM_BITS) trước khi analogWrite.
void setPWM_8bit_mag(int i, uint8_t mag8);


/// ========== DỪNG KHẨN CẤP ==========
/// Dừng ngay cả hai motor (PWM = 0)
/// Không thay đổi trạng thái DIR (chỉ tắt xung)
void stopHard();


/// ========== ÁP LỆNH ĐIỀU KHIỂN CHO MOTOR ==========
/// @param i: chỉ số motor (0 = trái, 1 = phải)
/// @param cmd: giá trị điều khiển -PWM_MAX..+PWM_MAX
///   âm → quay nghịch
///   dương → quay thuận
/// Tự động xử lý:
///   - Deadband (CMD_DEAD hoặc CMD_DEAD_PID)
///   - Giới hạn biên (clamp 0..255)
///   - Đảo chiều (invert_dir)
///   - Cập nhật telemety (outCmd[])
void applyDutyFromCmd_modeaware(int i, int cmd);

#endif
