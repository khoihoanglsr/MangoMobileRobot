#include <Arduino.h>

// ---- Your internal libs (same as publisher) ----
#include "Tien_ich.h"
#include "PWM_DIR.h"
#include "Encoder_Xuly.h"
#include "Tinh_toan_TocDoGoc.h"
#include "PID_TocDo.h"
#include "Lenh_UART.h"
#include "AdaptiveRulePID.h"

// ---- micro-ROS ----
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <stdint.h>
#include <rcl/rcl.h>
#include <rcl/timer.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <rmw/qos_profiles.h>

// ---- Math / Odom ----
#include <math.h>
#include "odometry.h"  // class Odometry của bạn

#ifndef ANALOG_WRITE_INIT_BOTH_DEFINED
#define ANALOG_WRITE_INIT_BOTH_DEFINED
static inline void analogWriteInitBoth() {
  analogWriteResolution(M0_PWM_PIN, PWM_BITS);
  analogWriteResolution(M1_PWM_PIN, PWM_BITS);
  analogWriteFrequency(M0_PWM_PIN, PWM_FREQ_HZ);
  analogWriteFrequency(M1_PWM_PIN, PWM_FREQ_HZ);
  analogWrite(M0_PWM_PIN, 0);
  analogWrite(M1_PWM_PIN, 0);
}
#endif

// ===================== THÔNG SỐ CƠ KHÍ =====================
static const float WHEEL_RADIUS_M = 0.050f;           // bán kính bánh (m)
static const float BASE_WIDTH_M   = 0.328f;            // khoảng cách 2 bánh (m)
static const float TWO_PI_R = 2.0f * (float)M_PI * WHEEL_RADIUS_M;

// ===================== PID GAINS =====================
static float KP_L = 0.95f, KI_L = 0.30f, KD_L = 0.045f;
static float KP_R = 0.95f, KI_R = 0.30f, KD_R = 0.045f;

// ===================== ROS2 =====================
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

rcl_subscription_t sub_cmdvel;
geometry_msgs__msg__Twist msg_cmdvel;

rcl_publisher_t odom_pub;
nav_msgs__msg__Odometry odom_msg;

rcl_timer_t ctrl_timer;  // 20 ms control loop
rcl_timer_t odom_timer;  // 20 ms odom loop

// ===================== ODOM helper =====================
Odometry odom;
unsigned long prev_odom_ms = 0;
unsigned long long time_offset_ms = 0;

// ===================== PID runtime vars =====================
static float sp_rpm_L = 0.0f, sp_rpm_R = 0.0f;  // setpoint RPM per wheel
static float ei_L = 0.0f, ei_R = 0.0f;
static float e_prev_L = 0.0f, e_prev_R = 0.0f;
static uint32_t last_pid_ms = 0;  // mốc thời gian cho PID

// ===================== Slew state / output =====================
static float out_pwm_L = 0.0f, out_pwm_R = 0.0f;  // mục tiêu sau PID (-255..255)
static float slew_L = 0.0f, slew_R = 0.0f;

// ===================== Macros =====================
#define RCCHECK(fn) \
  { \
    rcl_ret_t rc = fn; \
    if (rc != RCL_RET_OK) { error_loop(); } \
  }
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t rc = fn; \
    if (rc != RCL_RET_OK) { /* no-block */ \
    } \
  }

static inline void error_loop() {
  const int SAFE_LED = 2;
  pinMode(SAFE_LED, OUTPUT);
  while (1) {
    digitalWrite(SAFE_LED, !digitalRead(SAFE_LED));
    delay(200);
  }
}

// ===================== Time sync =====================
static void syncTime() {
  RCSOFTCHECK(rmw_uros_sync_session(10));
  unsigned long now_ms = millis();
  unsigned long long ros_ms = rmw_uros_epoch_millis();
  time_offset_ms = ros_ms > 0 ? (ros_ms - now_ms) : 0;
}

static inline struct timespec toRosNow() {
  unsigned long long now = (unsigned long long)millis() + time_offset_ms;
  struct timespec ts;
  ts.tv_sec = now / 1000ULL;
  ts.tv_nsec = (now % 1000ULL) * 1000000ULL;
  return ts;
}

// ===================== Kinematics v,ω → RPM =====================
static inline void computeWheelRPM(float v_lin, float v_ang, float &rpmL, float &rpmR) {
  // v_lin [m/s], v_ang [rad/s]
  float vL = v_lin - v_ang * (BASE_WIDTH_M * 0.5f);
  float vR = v_lin + v_ang * (BASE_WIDTH_M * 0.5f);
  rpmL = (vL / TWO_PI_R) * 60.0f;
  rpmR = (vR / TWO_PI_R) * 60.0f;
}

static uint32_t last_cmdvel_ms = 0;

// ===================== CMD_VEL callback =====================
static void cmdvel_cb(const void *msgin) {
  const geometry_msgs__msg__Twist *m = (const geometry_msgs__msg__Twist *)msgin;
  // setpoint RPM theo lệnh teleop_twist_keyboard
  computeWheelRPM(m->linear.x, m->angular.z, sp_rpm_L, sp_rpm_R);
  last_cmdvel_ms = millis();
}

// ===================== PID điều khiển tốc độ 2 bánh =====================
static void pid_speed_step() {
  uint32_t now = millis();
  float dt = (last_pid_ms == 0) ? 0.02f : (now - last_pid_ms) / 1000.0f;
  if (dt <= 0.0f) dt = 0.02f;
  last_pid_ms = now;

  // đảm bảo có số đo mới
  if (millis() - lastCalcMs >= calcPeriodMs) {
    calcSpeed();  // cập nhật meas_rpm_out[0], [1] từ encoder (lib của bạn)

    // Chuẩn hóa dấu encoder theo từng bánh
    for (int i = 0; i < 2; ++i) {
      if (invert_enc[i]) meas_rpm_out[i] = -meas_rpm_out[i];
    }
  }

  // L bánh
  float eL = sp_rpm_L - meas_rpm_out[0];
  ei_L += eL * dt;
  float dL = (eL - e_prev_L) / dt;
  float uL = KP_L * eL + KI_L * ei_L + KD_L * dL;
  e_prev_L = eL;

  // R bánh
  float eR = sp_rpm_R - meas_rpm_out[1];
  ei_R += eR * dt;
  float dR = (eR - e_prev_R) / dt;
  float uR = KP_R * eR + KI_R * ei_R + KD_R * dR;
  e_prev_R = eR;

  // giới hạn và offset ma sát nếu cần
  uL = constrain(uL, -255.0f, 255.0f);
  uR = constrain(uR, -255.0f, 255.0f);
  if (CMD_DEAD_PID > 0) {
    if (uL > 0) uL = max(uL, (float)CMD_DEAD_PID);
    if (uL < 0) uL = min(uL, (float)-CMD_DEAD_PID);
    if (uR > 0) uR = max(uR, (float)CMD_DEAD_PID);
    if (uR < 0) uR = min(uR, (float)-CMD_DEAD_PID);
  }

  out_pwm_L = uL;
  out_pwm_R = uR;
}

// ===================== Áp dụng PWM với slew rate =====================
static void apply_pwm_modeaware() {
  static uint32_t tSlew = 0;
  uint32_t now = millis();

  if (BYPASS_SLEW_IN_SPEED) {
    applyDutyFromCmd_modeaware(0, (int)lroundf(out_pwm_L));
    applyDutyFromCmd_modeaware(1, (int)lroundf(out_pwm_R));
    return;
  }

  if (now - tSlew >= 1) {
    float dt_ms = (float)(now - tSlew);
    float maxStep_8bit = SLEW_RATE_PER_MS * dt_ms * 2.55f * 100.0f;

    float diffL = out_pwm_L - slew_L;
    float diffR = out_pwm_R - slew_R;

    if (SLEW_RATE_PER_MS <= 0.0f || fabsf(diffL) <= maxStep_8bit) slew_L = out_pwm_L;
    else slew_L += (diffL > 0 ? +maxStep_8bit : -maxStep_8bit);

    if (SLEW_RATE_PER_MS <= 0.0f || fabsf(diffR) <= maxStep_8bit) slew_R = out_pwm_R;
    else slew_R += (diffR > 0 ? +maxStep_8bit : -maxStep_8bit);

    applyDutyFromCmd_modeaware(0, (int)lroundf(slew_L));
    applyDutyFromCmd_modeaware(1, (int)lroundf(slew_R));

    tSlew = now;
  }
}

// ===================== Timer callbacks =====================
static void ctrl_timer_cb(rcl_timer_t *, int64_t) {
  ruleTuning_step();  
  // PID + xuất PWM
  pid_speed_step();
  apply_pwm_modeaware();
}

static void odom_timer_cb(rcl_timer_t *, int64_t) {
  // Tính odom từ RPM hiện tại
  unsigned long now_ms = millis();
  float dt = (prev_odom_ms == 0) ? 0.02f : (now_ms - prev_odom_ms) / 1000.0f;
  prev_odom_ms = now_ms;

  // RPM → m/s, rad/s
  float rpsL = meas_rpm_out[0] / 60.0f;
  float rpsR = meas_rpm_out[1] / 60.0f;
  float lin_x = ((rpsL + rpsR) * 0.5f) * TWO_PI_R;                           // m/s
  float ang_z = ((-rpsL + rpsR) * 0.5f) * TWO_PI_R / (BASE_WIDTH_M * 0.5f);  // rad/s

  odom.update(dt, lin_x, 0.0f, ang_z);

  // Publish nav_msgs/Odometry
  odom_msg = odom.getData();
  struct timespec ts = toRosNow();
  odom_msg.header.stamp.sec = ts.tv_sec;
  odom_msg.header.stamp.nanosec = ts.tv_nsec;
  RCSOFTCHECK(rcl_publish(&odom_pub, &odom_msg, NULL));
}

// ===================== SETUP =====================
void setup() {
  // --- Serial transport for micro-ROS ---
  Serial.begin(115200);
  delay(1500);
  set_microros_transports();
  syncTime();

  // --- Motor DIR & PWM (helpers của bạn) ---
  pinMode(M0_DIR_PIN, OUTPUT);
  pinMode(M1_DIR_PIN, OUTPUT);
  digitalWrite(M0_DIR_PIN, LOW);
  digitalWrite(M1_DIR_PIN, LOW);

  analogWriteInitBoth();  // helper ở trên
  stopHard();             // helper từ lib của bạn

  // --- Encoder pins & ISR (trong Encoder_Xuly.h) ---
  pinMode(M0_ENC_A_PIN, INPUT);
  pinMode(M0_ENC_B_PIN, INPUT);
  pinMode(M1_ENC_A_PIN, INPUT);
  pinMode(M1_ENC_B_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(M0_ENC_A_PIN), encoderISR0A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M0_ENC_B_PIN), encoderISR0B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M1_ENC_A_PIN), encoderISR1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M1_ENC_B_PIN), encoderISR1B, CHANGE);

  // --- ROS2 init ---
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_diff_subscriber", "", &support));

  // Subscriber: /cmd_vel (teleop_twist_keyboard xuất topic này)
  RCCHECK(rclc_subscription_init_default(
    &sub_cmdvel, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // Publisher: /odom
  RCCHECK(rclc_publisher_init_default(
    &odom_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "odom/unfiltered"));

  // Timers: 20 ms control + 20 ms odom
  RCCHECK(rclc_timer_init_default(
    &ctrl_timer, &support, RCL_MS_TO_NS(20), ctrl_timer_cb));
  RCCHECK(rclc_timer_init_default(
    &odom_timer, &support, RCL_MS_TO_NS(20), odom_timer_cb));

  // Executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmdvel, &msg_cmdvel, &cmdvel_cb, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &ctrl_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &odom_timer));

  // Reset PID state
  ei_L = ei_R = 0.0f;
  e_prev_L = e_prev_R = 0.0f;
  out_pwm_L = out_pwm_R = 0.0f;
  slew_L = slew_R = 0.0f;

  last_pid_ms = millis();
  prev_odom_ms = millis();
}

// ===================== LOOP =====================
void loop() {
  if (millis() - last_cmdvel_ms > 400) {  // ví dụ 0.4giây không có lệnh
    sp_rpm_L = sp_rpm_R = 0;
    ei_L = ei_R = 0; 
    e_prev_L = e_prev_R = 0;
    out_pwm_L = out_pwm_R = 0;
    stopHard();
  }

  // Tick tốc độ/encoder (nếu lệch nhịp timer)
  if (millis() - lastCalcMs >= calcPeriodMs) {
    calcSpeed();
    lastCalcMs = millis();  // cập nhật timestamp tính toán
  }

  // ROS spin
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2)));
  delay(2);
}
