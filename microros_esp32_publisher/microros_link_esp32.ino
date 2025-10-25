// Essential Libraries
#include <WiFi.h>
#include <Arduino.h>

// Integrated libraries of PID control in ESP32
#include "Tien_ich.h"
#include "PWM_DIR.h"
#include "Encoder_Xuly.h"
#include "Tinh_toan_TocDoGoc.h"
#include "PID_TocDo.h"
#include "Lenh_UART.h"

// Bluetooth Classic (SPP) for ESP32
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

// ===================== BIẾN TOÀN CỤC ===================== //
volatile bool bt_cmd_dirty = false;   // có lệnh mới từ BT
static char bt_buf[8] = {0};          // buffer an toàn để publish
static size_t bt_len = 0;             // độ dài hiện tại

volatile bool bt_connected = false;

static void bt_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
  switch(event){
    case ESP_SPP_SRV_OPEN_EVT:  bt_connected = true;  Serial.println("BT: client connected"); break;
    case ESP_SPP_CLOSE_EVT:     bt_connected = false; Serial.println("BT: client disconnected"); break;
    default: break;
  }
}

int PWM_FREQ_HZ = 2000; // Tần số PWM xuất cho động cơ (Hz)

// --- Thông số encoder & truyền động ---
volatile long CPRx4 = 2000;   // Encoder có 500 xung/vòng, nhân 4 do đọc cả 4 cạnh → 2000 xung/vòng
float gearRatio = 14.0f;      // Tỉ số truyền cơ khí (motor: bánh xe)
float encScale = 1.0f;        // Hệ số chia (nếu có encoder phụ)

// --- Đếm encoder cho từng bánh (0 = trái, 1 = phải) ---
volatile long encCount[2] = {0, 0};   // Bộ đếm xung encoder
volatile uint8_t prevAB[2] = {0, 0};  // Trạng thái A/B trước đó để xác định chiều quay

// --- Hiệu chỉnh encoder ---
bool calActive[2] = {false, false};   // Cờ bật hiệu chuẩn
long calStartCount[2] = {0, 0};       // Giá trị xung lúc bắt đầu hiệu chuẩn

// --- Thời gian tính toán định kỳ ---
unsigned long lastCalcMs = 0; // Lưu thời điểm lần cuối tính tốc độ

// --- Các giá trị đo lường từ encoder ---
long lastCount[2] = {0, 0};           // Số xung ở chu kỳ trước
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
int cmdOpenLoop[2] = {0, 0}; // Lệnh PWM đầu vào (tay) dạng -255..+255
int outCmd[2] = {0, 0};      // Lệnh PWM xuất thực tế (sau PID/hở vòng)
int CMD_DEAD = 6;            // Vùng chết đầu ra (loại bỏ nhiễu nhỏ)

// --- Đảo chiều & Stream dữ liệu ---
bool invert_dir[2] = {false, false}; // Đảo chiều riêng từng bánh nếu cần
bool streamOn = false;               // Bật gửi dữ liệu định kỳ
bool streamCSV = false;              // Định dạng CSV cho dữ liệu stream
unsigned long streamRateMs = 50;     // Chu kỳ gửi dữ liệu (ms)
unsigned long lastStreamMs = 0;      // Thời điểm lần cuối stream

// --- Làm mượt lệnh PWM (slew rate) ---
float SLEW_RATE_PER_MS = 0.002f; // Giới hạn thay đổi PWM theo thời gian (%/ms)
float slew_cmd[2] = {0.0f, 0.0f}; // PWM hiện tại sau làm mượt

// --- PID & các biến điều khiển ---
int controlMode = MODE_PULSE;   // Mặc định chế độ hở vòng (tay)
bool holdEnable = false;        // Giữ vị trí (nếu có)
float sp_rpm = 0.0f;            // Setpoint tốc độ mong muốn
float kp = 0.95f, ki = 0.3f, kd = 0.045f; // Hệ số PID
float integ = 0.0f;             // Thành phần tích phân

bool BYPASS_SLEW_IN_SPEED = true; // Bỏ ramp khi PID chạy (tránh trễ)
int CMD_DEAD_PID = 0;             // Vùng chết riêng cho PID

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

// ---- Bluetooth / Wi-Fi low-level (cho callback & tx power) ----
#include "esp_spp_api.h"    // có thể cần cho esp_spp_cb_event_t
#include "esp_wifi.h"       // cần cho esp_wifi_set_max_tx_power
#include "esp_bt.h"

// ---- micro-ROS Libraries----
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <stdint.h>         // int64_t
#include <rcl/rcl.h>
#include <rcl/timer.h>      // rcl_timer_t
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/u_int8.h>
#include <rmw/qos_profiles.h>

rcl_publisher_t publisher;  // Int32 nhịp hệ thống
std_msgs__msg__Int32 msg;

rcl_publisher_t bt_pub;             // Publisher cho lệnh BT
std_msgs__msg__UInt8 bt_msg;        // 1 byte -> giảm payload MẠNH

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

extern "C" void timer_callback(rcl_timer_t * timer, int64_t last_call_time);


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
        cmdOpenLoop[1] = PWM_HAND_CONTROL;
    } else if (command == "B") { // Lùi
        cmdOpenLoop[0] = -PWM_HAND_CONTROL;
        cmdOpenLoop[1] = -PWM_HAND_CONTROL;
    } else if (command == "L") { // Rẽ trái
        cmdOpenLoop[0] = -PWM_HAND_CONTROL;
        cmdOpenLoop[1] = PWM_HAND_CONTROL;
    } else if (command == "R") { // Rẽ phải
        cmdOpenLoop[0] = PWM_HAND_CONTROL;
        cmdOpenLoop[1] = -PWM_HAND_CONTROL;
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

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  const int SAFE_LED = 2; // or your board's LED
  pinMode(SAFE_LED, OUTPUT);
  while (1) { digitalWrite(SAFE_LED, !digitalRead(SAFE_LED)); delay(200); }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (!timer) return;

  // Nếu BT đang kết nối, giảm tần suất publish hệ thống (ví dụ: chỉ 1/3 nhịp)
  static uint8_t gate = 0;
  bool allow_pub = true;
  if (bt_connected) {
    gate = (gate + 1) % 3;     // 0,1,2
    allow_pub = (gate == 0);   // chỉ publish khi gate == 0
  }

  if (allow_pub) {
    msg.data++;
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  }

  // publish lệnh BT nếu có cập nhật (ưu tiên cao — vẫn gửi)
  if (bt_cmd_dirty) {
    bt_msg.data = (uint8_t)bt_buf[0];
    RCSOFTCHECK(rcl_publish(&bt_pub, &bt_msg, NULL));
    bt_cmd_dirty = false;
  }
}


void setup() {
  esp_bt_controller_mem_release(ESP_BT_MODE_BLE);  // chỉ dùng Classic SPP, bỏ BLE

  // --- Khởi tạo Bluetooth Classic (SPP) ---
  SerialBT.begin("ESP32_CAR_BT"); // Tên hiển thị khi dò Bluetooth

  set_microros_wifi_transports("RoboManGo","vv4022n1","10.206.139.28",8888);
  // Serial.begin(115200); // Khởi động Serial debug
  delay(50);

  // (Optional) print chip model to confirm Classic support
  Serial.printf("Chip: %s\n", ESP.getChipModel());    // S3/C3/C6 = no Classic SPP

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

  
  // Serial.println(F("Bluetooth Classic (SPP) KHỞI ĐỘNG!"));
  // Serial.println(F("Tên thiết bị: ESP32_CAR_BT"));
  // Serial.println(F("Kết nối từ app BT Car Controller (SPP) và gửi lệnh ARM/F/B/L/R/S"));

  SerialBT.register_callback(bt_cb); 
  
    // Start Classic SPP in SLAVE mode (do NOT pass 'true')
  // bool ok = SerialBT.begin("ESP32_CAR_BT");
  // Serial.printf("SerialBT.begin: %s\n", ok ? "OK" : "FAIL");
  // if (!ok) {
  //   Serial.println("BT start failed: check power, core version, or chip type.");
  // }

  // --- Cấu hình mặc định ---
  armed = true;
  streamOn = false;
  echoCmd = false; // In lệnh nhận

  // ----- rclc init -----
  allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // Publisher nhịp hệ thống (Int32)
  RCCHECK(rclc_publisher_init_default(
    &publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "sys_tick"
  ));

  // Publisher lệnh BT: UInt8 + QoS Best-Effort
  rmw_qos_profile_t qos = rmw_qos_profile_sensor_data; 

  // create publisher
  RCCHECK(rclc_publisher_init(
    &bt_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
    "bt_cmd",
    &qos
  ));

  // Create timer
  unsigned int timer_timeout = 1000;
  if (WiFi.isConnected() && WiFi.RSSI() < -70) timer_timeout = 3000;

  RCCHECK(rclc_timer_init_default(
    &timer, &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback
  ));

  // Create Executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  msg.data = 0;
}

void loop() {
  // --- Nhận lệnh từ Bluetooth Classic (SPP) ---
    // Hỗ trợ cả chuỗi có newline và ký tự đơn lẻ
  while (SerialBT.available()) {
    char c = (char)SerialBT.read();
    if (c=='\r' || c=='\n') continue;
    if (c>='a' && c<='z') c -= 32;

    switch (c) {
      case 'F': processWebCommand("F"); break;
      case 'B': processWebCommand("B"); break;
      case 'L': processWebCommand("L"); break;
      case 'R': processWebCommand("R"); break;
      case 'S': processWebCommand("S"); break;
      case 'A': processWebCommand("ARM"); break;
      case 'D': processWebCommand("DISARM"); break;
      default: break;
    }

    // 1 byte cho ROS
    bt_buf[0] = c;
    bt_buf[1] = '\0';
    bt_len = 1;
    bt_cmd_dirty = true;
  }

  // Keep-alive nhẹ cho SPP, tránh app time-out
  static uint32_t last_ka = 0;
  if (SerialBT.hasClient()) {
    uint32_t now_ms = millis();
    if (now_ms - last_ka >= 500) {
      SerialBT.write('K');   // 1 byte keep-alive
      last_ka = now_ms;
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
            // Serial.print("SP: ");  Serial.print((controlMode == MODE_SPEED) ? sp_rpm_cmd : 0.0f, 2);
            // Serial.print("\tMEAS: "); Serial.print(rpm_avg, 2);
            // Serial.print("\tPWM: "); Serial.println(last_u_pid, 2);
        } else {
            // Xuất dữ liệu dạng bảng
            printLine(streamCSV);
        }
    }

    // --- Delay nhỏ để giảm CPU spin (không ảnh hưởng timing chính) ---
  delay(2);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2)));
}
