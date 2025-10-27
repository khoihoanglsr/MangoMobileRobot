// ===== NO Wi-Fi: serial-only micro-ROS over USB =====

// Essential Libraries
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
volatile long CPRx4 = 2000;   // 500CPR x4 cạnh = 2000 xung/vòng
float gearRatio = 14.0f;
float encScale  = 1.0f;

// --- Đếm encoder cho từng bánh (0 = trái, 1 = phải) ---
volatile long encCount[2] = {0, 0};
volatile uint8_t prevAB[2] = {0, 0};

// --- Hiệu chỉnh encoder ---
bool calActive[2] = {false, false};
long calStartCount[2] = {0, 0};

// --- Thời gian tính toán định kỳ ---
unsigned long lastCalcMs = 0;

// --- Các giá trị đo lường ---
long  lastCount[2]      = {0, 0};
float meas_cps[2]       = {0.0f, 0.0f};
float meas_rpm_out[2]   = {0.0f, 0.0f};
float goc_out_deg[2]    = {0.0f, 0.0f};
float goc_encoder_deg[2]= {0.0f, 0.0f};

// --- Đếm encoder quy đổi ---
volatile long encCountOut_raw[2]    = {0, 0};
double       encOutFrac[2]          = {0.0, 0.0};
volatile long encCountMotorScaled[2]= {0, 0};
double       motFrac[2]             = {0.0, 0.0};

// --- Giao tiếp & an toàn ---
String rx = "";
bool armed = false;

// --- Lệnh điều khiển động cơ ---
int cmdOpenLoop[2] = {0, 0};
int outCmd[2]      = {0, 0};
int CMD_DEAD       = 6;

// --- Đảo chiều & Stream dữ liệu ---
bool invert_dir[2]   = {false, false};
bool streamOn        = false;
bool streamCSV       = false;
unsigned long streamRateMs = 50;
unsigned long lastStreamMs = 0;

// --- Làm mượt PWM ---
float SLEW_RATE_PER_MS = 0.002f;
float slew_cmd[2] = {0.0f, 0.0f};

// --- PID ---
int   controlMode = MODE_PULSE;   // mặc định hở vòng
bool  holdEnable  = false;
float sp_rpm      = 0.0f;
float kp = 0.95f, ki = 0.3f, kd = 0.045f;
float integ = 0.0f;

bool  BYPASS_SLEW_IN_SPEED = true;
int   CMD_DEAD_PID         = 0;

// --- Feedforward & nâng cao ---
float KFF = 0.0f;
float MIN_PWM_FF = 8.0f;
float KD_ALPHA   = 0.3f;
float SP_RATE_RPM_PER_S = 800.0f;
float sp_rpm_cmd = 0.0f;

bool  echoCmd = false;
bool  plotOn  = false;
float last_u_pid = 0.0f;

// --- Thông số điều khiển tay ---
const int PWM_HAND_CONTROL = 255;

// ---- Bluetooth / low-level ----
#include "esp_spp_api.h"
#include "esp_bt.h"              // vẫn giữ BT Classic; KHÔNG dùng Wi-Fi

// ---- micro-ROS Libraries ----
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <stdint.h>
#include <rcl/rcl.h>
#include <rcl/timer.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/u_int8.h>
#include <rmw/qos_profiles.h>

rcl_publisher_t publisher;   // Int32 nhịp hệ thống
std_msgs__msg__Int32 msg;

rcl_publisher_t bt_pub;      // Publisher cho lệnh BT
std_msgs__msg__UInt8 bt_msg; // 1 byte

rclc_executor_t executor;
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;
rcl_timer_t     timer;

extern "C" void timer_callback(rcl_timer_t * timer, int64_t last_call_time);

// ===================== HÀM XỬ LÝ LỆNH ===================== //
void processWebCommand(String command) {
  if (echoCmd) { Serial.print(F("CMD: ")); Serial.println(command); }

  if (command == "MOTOR_ON" || command == "ARM") {
    armed = true;
    Serial.println(F("ARMED: Bật điều khiển."));
    return;
  }
  if (command == "MOTOR_OFF" || command == "DISARM") {
    armed = false;
    stopHard();
    Serial.println(F("DISARMED: Dừng an toàn."));
    return;
  }

  if (!armed) return;

  controlMode = MODE_PULSE;

  if      (command == "F") { cmdOpenLoop[0] =  PWM_HAND_CONTROL; cmdOpenLoop[1] =  PWM_HAND_CONTROL; }
  else if (command == "B") { cmdOpenLoop[0] = -PWM_HAND_CONTROL; cmdOpenLoop[1] = -PWM_HAND_CONTROL; }
  else if (command == "L") { cmdOpenLoop[0] = -PWM_HAND_CONTROL; cmdOpenLoop[1] =  PWM_HAND_CONTROL; }
  else if (command == "R") { cmdOpenLoop[0] =  PWM_HAND_CONTROL; cmdOpenLoop[1] = -PWM_HAND_CONTROL; }
  else if (command == "S") { cmdOpenLoop[0] = 0; cmdOpenLoop[1] = 0; }

  outCmd[0] = cmdOpenLoop[0];
  outCmd[1] = cmdOpenLoop[1];
}

// ===================== HÀM PHỤ TRỢ ===================== //
static inline void analogWriteInitBoth(){
  analogWriteResolution(M0_PWM_PIN, PWM_BITS);
  analogWriteResolution(M1_PWM_PIN, PWM_BITS);
  analogWriteFrequency(M0_PWM_PIN, PWM_FREQ_HZ);
  analogWriteFrequency(M1_PWM_PIN, PWM_FREQ_HZ);
}

static inline uint8_t readAB_pair(uint8_t pinA, uint8_t pinB){
  uint8_t a = (uint8_t)digitalRead(pinA);
  uint8_t b = (uint8_t)digitalRead(pinB);
  return (uint8_t)((a << 1) | b);
}

#define RCCHECK(fn)     { rcl_ret_t rc = fn; if(rc != RCL_RET_OK){ error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; if(rc != RCL_RET_OK){ /* no-block */ } }

void error_loop(){
  const int SAFE_LED = 2; // tùy board
  pinMode(SAFE_LED, OUTPUT);
  while (1) { digitalWrite(SAFE_LED, !digitalRead(SAFE_LED)); delay(200); }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (!timer) return;

  // Giảm nhịp publish khi đang có BT (giữ nguyên logic của bạn)
  static uint8_t gate = 0;
  bool allow_pub = true;
  if (bt_connected) {
    gate = (gate + 1) % 3;
    allow_pub = (gate == 0);
  }

  if (allow_pub) {
    msg.data++;
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  }

  if (bt_cmd_dirty) {
    bt_msg.data = (uint8_t)bt_buf[0];
    RCSOFTCHECK(rcl_publish(&bt_pub, &bt_msg, NULL));
    bt_cmd_dirty = false;
  }
}

void setup() {
  // === Dùng Serial làm transport cho micro-ROS ===
  Serial.begin(115200);        // USB CDC hoặc UART-USB
  delay(2000);                 // chờ cổng sẵn sàng (đặc biệt ESP32-S3)
  set_microros_transports();  

  // --- Bluetooth Classic (SPP) ---
  esp_bt_controller_mem_release(ESP_BT_MODE_BLE);  // chỉ dùng Classic SPP
  SerialBT.begin("ESP32_CAR_BT");
  SerialBT.register_callback(bt_cb);

  Serial.printf("Chip: %s\n", ESP.getChipModel());

  // --- Cấu hình chân PWM & DIR ---
  pinMode(M0_DIR_PIN, OUTPUT);
  pinMode(M1_DIR_PIN, OUTPUT);
  digitalWrite(M0_DIR_PIN, LOW);
  digitalWrite(M1_DIR_PIN, LOW);

  analogWriteInitBoth();
  stopHard();

  // --- Encoder ---
  pinMode(M0_ENC_A_PIN, INPUT);
  pinMode(M0_ENC_B_PIN, INPUT);
  pinMode(M1_ENC_A_PIN, INPUT);
  pinMode(M1_ENC_B_PIN, INPUT);

  prevAB[0] = readAB_pair(M0_ENC_A_PIN, M0_ENC_B_PIN);
  prevAB[1] = readAB_pair(M1_ENC_A_PIN, M1_ENC_B_PIN);

  attachInterrupt(digitalPinToInterrupt(M0_ENC_A_PIN), encoderISR0A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M0_ENC_B_PIN), encoderISR0B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M1_ENC_A_PIN), encoderISR1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M1_ENC_B_PIN), encoderISR1B, CHANGE);

  // --- Cấu hình mặc định ---
  armed    = true;
  streamOn = false;
  echoCmd  = false;

  // ----- rclc init -----
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // Publisher nhịp hệ thống (Int32)
  RCCHECK(rclc_publisher_init_default(
    &publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "sys_tick"
  ));

  // Publisher lệnh BT: UInt8 + QoS Best Effort
  rmw_qos_profile_t qos = rmw_qos_profile_sensor_data;
  RCCHECK(rclc_publisher_init(
    &bt_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
    "bt_cmd",
    &qos
  ));

  // Timer 1s (không phụ thuộc Wi-Fi)
  unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer, &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback
  ));

  // Executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  msg.data = 0;
}

void loop() {
  // --- Nhận lệnh từ Bluetooth Classic (SPP) ---
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

    // 1 byte mirror lên ROS
    bt_buf[0] = c; bt_buf[1] = '\0';
    bt_len = 1;
    bt_cmd_dirty = true;
  }

  // Keep-alive nhẹ cho SPP
  static uint32_t last_ka = 0;
  if (SerialBT.hasClient()) {
    uint32_t now_ms = millis();
    if (now_ms - last_ka >= 500) {
      SerialBT.write('K');
      last_ka = now_ms;
    }
  }

  // --- PID/speed update định kỳ ---
  if (millis() - lastCalcMs >= calcPeriodMs) {
    calcSpeed();
    updatePIDSpeed();
    lastCalcMs = millis();
  }

  // --- Làm mượt PWM (slew rate) hoặc xuất trực tiếp khi PID ---
  static uint32_t tSlew = 0;
  uint32_t now = millis();

  if (controlMode == MODE_SPEED && BYPASS_SLEW_IN_SPEED) {
    if (armed) {
      applyDutyFromCmd_modeaware(0, (int)lroundf(outCmd[0]));
      applyDutyFromCmd_modeaware(1, (int)lroundf(outCmd[1]));
    } else {
      stopHard();
    }
  } else {
    if (now - tSlew >= 1) {
      float dt_ms = (float)(now - tSlew);
      float maxStep_8bit = SLEW_RATE_PER_MS * dt_ms * 2.55f * 100.0f;

      for (int i = 0; i < 2; ++i) {
        float diff = (float)outCmd[i] - slew_cmd[i];
        if (SLEW_RATE_PER_MS <= 0.0f || fabsf(diff) <= maxStep_8bit) {
          slew_cmd[i] = (float)outCmd[i];
        } else {
          slew_cmd[i] += (diff > 0 ? +maxStep_8bit : -maxStep_8bit);
        }

        if (armed) applyDutyFromCmd_modeaware(i, (int)lroundf(slew_cmd[i]));
        else       stopHard();
      }
      tSlew = now;
    }
  }

  // --- Stream dữ liệu định kỳ ---
  if (streamOn && (millis() - lastStreamMs >= streamRateMs)) {
    lastStreamMs = millis();
    if (plotOn) {
      float rpm_avg = 0.5f * (meas_rpm_out[0] + meas_rpm_out[1]);
      // Serial.printf("SP: %.2f\tMEAS: %.2f\tPWM: %.2f\n",
      //               (controlMode == MODE_SPEED) ? sp_rpm_cmd : 0.0f,
      //               rpm_avg, last_u_pid);
    } else {
      printLine(streamCSV);
    }
  }

  // Nhịp executor micro-ROS
  delay(2);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2)));
}
