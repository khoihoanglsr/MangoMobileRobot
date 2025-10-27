
#include <Arduino.h>            // Thư viện cơ bản của Arduino
#include "Tien_ich.h"           // Chứa các hàm tiện ích phụ (stopHard, printLine,...)
#include "PWM_DIR.h"            // Xử lý PWM và chiều quay của động cơ
#include "Encoder_Xuly.h"       // Hàm xử lý ngắt encoder và đếm xung
#include "Tinh_toan_TocDoGoc.h" // Hàm tính tốc độ quay (CPS, RPM, góc,...)
#include "PID_TocDo.h"          // Bộ điều khiển PID tốc độ
#include "Lenh_UART.h"          // Giao tiếp UART, giữ lại nếu cần lệnh phụ

// Bluetooth Classic (SPP) cho ESP32
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

// ===================== BIẾN TOÀN CỤC ===================== //
int PWM_FREQ_HZ = 2000; // Tần số PWM xuất cho động cơ (Hz)

// --- Thông số encoder & truyền động ---
volatile long CPRx4 = 2000;   // Encoder có 500 xung/vòng, nhân 4 do đọc cả 4 cạnh → 2000 xung/vòng
float gearRatio = 14.0f;      // Tỉ số truyền cơ khí (motor: bánh xe)
float encScale = 1.0f;        // Hệ số chia (nếu có encoder phụ)

// --- Đếm encoder cho từng bánh (0 = trái, 1 = phải) ---
volatile long encCount[2] = {0, 0};   // Bộ đếm xung encoder
volatile uint8_t prevAB[2] = {0, 0};  // Trạng thái A/B trước đó để xác định chiều quay

// --- Thời gian tính toán định kỳ ---
unsigned long lastCalcMs = 0; // Lưu thời điểm lần cuối tính tốc độ

// --- Các giá trị đo lường từ encoder ---
long lastCount[2] = {0, 0};           // Số xung ở chu kỳ trước
float cmdOpenLoop[2] = {0, 0};
float meas_cps[2] = {0.0f, 0.0f};     // Tốc độ (xung/giây)
float meas_rpm_out[2] = {0.0f, 0.0f}; // Tốc độ vòng/phút (RPM)
float goc_out_deg[2] = {0.0f, 0.0f};  // Góc quay đầu ra (bánh xe)
float goc_encoder_deg[2] = {0.0f, 0.0f}; // Góc quay encoder

// --- Đếm encoder quy đổi cho motor/bánh ---
volatile long encCountOut_raw[2] = {0, 0}; // Đếm xung thực tế (thô)
double encOutFrac[2] = {0.0, 0.0};         // Phần lẻ của xung (nếu chia tỷ lệ)
volatile long encCountMotorScaled[2] = {0, 0}; // Đếm xung đã chia tỷ lệ
double motFrac[2] = {0.0, 0.0};            // Phần lẻ tương ứng

// --- Biến giao tiếp ---
String rx = "";          // Chuỗi lệnh nhận qua UART hoặc Bluetooth
bool armed = false;      // Cờ bật điều khiển (an toàn)

// --- Lệnh điều khiển động cơ ---

// --- Đảo chiều & Stream dữ liệu ---
bool invert_dir[2] = {false, false}; // Đảo chiều riêng từng bánh nếu cần
bool streamOn = false;               // Bật gửi dữ liệu định kỳ
bool streamCSV = false;              // Định dạng CSV cho dữ liệu stream
unsigned long streamRateMs = 50;     // Chu kỳ gửi dữ liệu (ms)
unsigned long lastStreamMs = 0;      // Thời điểm lần cuối stream

// --- Làm mượt lệnh PWM (slew rate) ---
float SLEW_RATE_PER_MS = 0.002f; // Giới hạn thay đổi PWM theo thời gian (%/ms)

// --- PID & các biến điều khiển ---
int controlMode = MODE_PULSE;   // Mặc định chế độ hở vòng (tay)
float kp = 0.95f, ki = 0.3f, kd = 0.045f; // Hệ số PID
float integ = 0.0f;             // Thành phần tích phân

bool BYPASS_SLEW_IN_SPEED = true; // Bỏ ramp khi PID chạy (tránh trễ)

// --- Feedforward & các hệ số PID nâng cao ---
float KFF = 0.0f;               // Feedforward coefficient
float MIN_PWM_FF = 8.0f;        // Ngưỡng tối thiểu PWM khi dùng FF
float KD_ALPHA = 0.3f;          // Hệ số lọc D-term
float SP_RATE_RPM_PER_S = 800.0f; // Giới hạn tăng setpoint (RPM/s)
float sp_rpm_cmd = 0.0f;        // Tốc độ đặt mục tiêu

bool echoCmd = false;           // Bật in lệnh ra Serial
bool plotOn = false;            // Bật đồ thị
float last_u_pid = 0.0f;        // Lưu giá trị đầu ra PID gần nhất

// --- Thông số điều khiển tay ---
const int PWM_HAND_CONTROL = 255; // PWM khi điều khiển thủ công (Tiến/Lùi)

// ===================== HÀM XỬ LÝ LỆNH ===================== //
/**
 * Xử lý lệnh nhận (ARM/DISARM/F/B/L/R/S)
 * Giữ nguyên logic từ code gốc để tái sử dụng.
 */
void processWebCommand(String command) {
    // In ra Serial để debug
    if (echoCmd) {
        Serial.print(F("CMD: "));
        Serial.println(command);
    }
    
    // --- Bật động cơ ---
    if (command == "MOTOR_ON" || command == "ARM") {
        armed = true;
        Serial.println(F("ARMED: Bật điều khiển."));
        return;
    }
    // --- Tắt động cơ ---
    if (command == "MOTOR_OFF" || command == "DISARM") {
        armed = false;
        stopHard(); // Dừng ngay lập tức
        Serial.println(F("DISARMED: Dừng an toàn."));
        return;
    }

    // --- Nếu chưa bật động cơ thì bỏ qua ---
    if (!armed) return;

    // --- Đặt chế độ điều khiển hở vòng ---
    controlMode = MODE_PULSE;

    // --- Giải mã lệnh di chuyển ---
    if (command == "F") { // Tiến

        cmdOpenLoop[0] = PWM_HAND_CONTROL;
        cmdOpenLoop[1] = -PWM_HAND_CONTROL;
        
    /*
        cmdOpenLoop[0] = PWM_HAND_CONTROL;
        cmdOpenLoop[1] = PWM_HAND_CONTROL;
    */
    } else if (command == "B") { // Lùi
        cmdOpenLoop[0] = -PWM_HAND_CONTROL;
        cmdOpenLoop[1] = PWM_HAND_CONTROL;
        
    /*
        cmdOpenLoop[0] = -PWM_HAND_CONTROL;
        cmdOpenLoop[1] = -PWM_HAND_CONTROL;

    */
    } else if (command == "L") { // Rẽ trái
        cmdOpenLoop[0] = -PWM_HAND_CONTROL;
        cmdOpenLoop[1] = -PWM_HAND_CONTROL;
        
    /*
        cmdOpenLoop[0] = -PWM_HAND_CONTROL;
        cmdOpenLoop[1] = PWM_HAND_CONTROL;

    */

    } else if (command == "R") { // Rẽ phải
        cmdOpenLoop[0] = PWM_HAND_CONTROL;
        cmdOpenLoop[1] = PWM_HAND_CONTROL;
        
    /*
        cmdOpenLoop[0] = PWM_HAND_CONTROL;
        cmdOpenLoop[1] = -PWM_HAND_CONTROL;
    */

    } else if (command == "S") { // Dừng
        cmdOpenLoop[0] = 0;
        cmdOpenLoop[1] = 0;
    }

    // Cập nhật đầu ra (để áp dụng slew rate)
    outCmd[0] = cmdOpenLoop[0];
    outCmd[1] = cmdOpenLoop[1];
}

// ===================== HÀM PHỤ TRỢ ===================== //
// Khởi tạo PWM cho cả 2 động cơ
static inline void analogWriteInitBoth(){
  analogWriteResolution(M0_PWM_PIN, PWM_BITS);
  analogWriteResolution(M1_PWM_PIN, PWM_BITS);
  analogWriteFrequency(M0_PWM_PIN, PWM_FREQ_HZ);
  analogWriteFrequency(M1_PWM_PIN, PWM_FREQ_HZ);
}

// Đọc cặp tín hiệu A/B của encoder
static inline uint8_t readAB_pair(uint8_t pinA, uint8_t pinB){
  uint8_t a = (uint8_t)digitalRead(pinA);
  uint8_t b = (uint8_t)digitalRead(pinB);
  return (uint8_t)((a << 1) | b); // Gộp thành 2 bit
}

// ===================== HÀM SETUP ===================== //
void setup() {
    Serial.begin(115200); // Khởi động Serial debug
    delay(200);

    // --- Cấu hình chân PWM & DIR ---
    pinMode(M0_DIR_PIN, OUTPUT);
    pinMode(M1_DIR_PIN, OUTPUT);
    digitalWrite(M0_DIR_PIN, LOW);
    digitalWrite(M1_DIR_PIN, LOW);

    analogWriteInitBoth(); // Khởi tạo PWM
    stopHard(); // Dừng động cơ ban đầu

    // --- Cấu hình encoder ---
    pinMode(M0_ENC_A_PIN, INPUT);
    pinMode(M0_ENC_B_PIN, INPUT);
    pinMode(M1_ENC_A_PIN, INPUT);
    pinMode(M1_ENC_B_PIN, INPUT);

    // Lưu trạng thái ban đầu của A/B
    prevAB[0] = readAB_pair(M0_ENC_A_PIN, M0_ENC_B_PIN);
    prevAB[1] = readAB_pair(M1_ENC_A_PIN, M1_ENC_B_PIN);

    // Gắn ngắt encoder (đếm xung theo cạnh)
    attachInterrupt(digitalPinToInterrupt(M0_ENC_A_PIN), encoderISR0A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(M0_ENC_B_PIN), encoderISR0B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(M1_ENC_A_PIN), encoderISR1A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(M1_ENC_B_PIN), encoderISR1B, CHANGE);

    // --- Khởi tạo Bluetooth Classic (SPP) ---
    SerialBT.begin("ESP32_CAR_BT"); // Tên hiển thị khi dò Bluetooth
    Serial.println(F("Bluetooth Classic (SPP) KHỞI ĐỘNG!"));
    Serial.println(F("Tên thiết bị: ESP32_CAR_BT"));
    Serial.println(F("Kết nối từ app BT Car Controller (SPP) và gửi lệnh ARM/F/B/L/R/S"));

    // --- Cấu hình mặc định ---
    armed = true;
    streamOn = false;
    echoCmd = true; // Bật in lệnh nhận (có thể tắt để tiết kiệm flash)
}

// ===================== HÀM LOOP ===================== //
void loop() {
    // --- Nhận lệnh từ Bluetooth Classic (SPP) ---
    // Hỗ trợ cả chuỗi có newline và ký tự đơn lẻ
    if (SerialBT.available()) {
        // Đọc một dòng nếu có newline, nếu không thì đọc từng ký tự
        String cd;m
        // Nếu app gửi theo dòng (kèm '\n'), dùng readStringUntil
        cmd = SerialBT.readStringUntil('\n');
        cmd.trim();
        if (cmd.length() == 0) {
            // Nếu không có newline, có thể app gửi ký tự đơn -> đọc 1 byte
            while (SerialBT.available()) {
                char c = SerialBT.read();
                if (c == '\r' || c == '\n') continue;
                String t(1, c);
                processWebCommand(t);
            }
        } else {
            processWebCommand(cmd);
        }
    }

    // --- Tính tốc độ & PID định kỳ ---
    if (millis() - lastCalcMs >= calcPeriodMs) {
        calcSpeed();       // Cập nhật tốc độ encoder
        updatePIDSpeed();  // Tính PID và cập nhật outCmd[]
        lastCalcMs = millis(); // cập nhật timestamp tính toán
    }

    // --- Làm mượt lệnh PWM (slew rate) ---
    static uint32_t tSlew = 0;
    uint32_t now = millis();

    // Nếu đang ở chế độ PID tốc độ (MODE_SPEED)
    if (controlMode == MODE_SPEED && BYPASS_SLEW_IN_SPEED) {
        if (armed) {
            // Áp trực tiếp giá trị PID ra động cơ
            applyDutyFromCmd_modeaware(0, (int)lroundf(outCmd[0]));
            applyDutyFromCmd_modeaware(1, (int)lroundf(outCmd[1]));
        } else {
            stopHard();
        }
    } else {
        // Chế độ tay: có làm mượt PWM
        if (now - tSlew >= 1) {
            float dt_ms = (float)(now - tSlew);
            // maxStep = giới hạn thay đổi PWM trong khoảng thời gian dt
            float maxStep_8bit = SLEW_RATE_PER_MS * dt_ms * 2.55f * 100.0f;

            for (int i = 0; i < 2; ++i) {
                float diff = (float)outCmd[i] - slew_cmd[i];

                // Nếu thay đổi nhỏ hơn maxStep → cập nhật ngay
                if (SLEW_RATE_PER_MS <= 0.0f || fabsf(diff) <= maxStep_8bit) {
                    slew_cmd[i] = (float)outCmd[i];
                } else {
                    slew_cmd[i] += (diff > 0 ? +maxStep_8bit : -maxStep_8bit);
                }

                // Áp PWM ra motor
                if (armed)
                    applyDutyFromCmd_modeaware(i, (int)lroundf(slew_cmd[i]));
                else
                    stopHard();
            }
            tSlew = now;
        }
    }

    // --- Xuất dữ liệu định kỳ ra Serial ---
    if (streamOn && (millis() - lastStreamMs >= streamRateMs)) {
        lastStreamMs = millis();

        if (plotOn) {
            // Gửi dữ liệu phục vụ vẽ đồ thị
            float rpm_avg = 0.5f * (meas_rpm_out[0] + meas_rpm_out[1]);
            Serial.print("SP: ");  Serial.print((controlMode == MODE_SPEED) ? sp_rpm_cmd : 0.0f, 2);
            Serial.print("\tMEAS: "); Serial.print(rpm_avg, 2);
            Serial.print("\tPWM: "); Serial.println(last_u_pid, 2);
        } else {
            // Xuất dữ liệu dạng bảng
            printLine(streamCSV);
        }
    }

    // --- Nhúng chút delay nhỏ để giảm CPU spin (không ảnh hưởng timing chính) ---
    delay(1);
}
