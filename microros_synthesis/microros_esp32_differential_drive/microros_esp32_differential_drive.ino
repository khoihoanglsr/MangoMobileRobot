#include <Arduino.h>

// ---- Your internal libs (same as publisher) ----
#include "Global.h"
#include "PWM_DIR.h"
#include "Encoder_Process.h"
#include "Calc_Speed.h"

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
#include "odometry.h"

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

// Biến trạng thái PID (tách biệt cho 2 bánh)
static float integ_L = 0.0f, integ_R = 0.0f;
static float measFilt_L = 0.0f, measFilt_R = 0.0f;
static float prevMeas_L = 0.0f, prevMeas_R = 0.0f;
static float dmeas_filt_L = 0.0f, dmeas_filt_R = 0.0f;
static float sp_rpm_cmd_L = 0.0f, sp_rpm_cmd_R = 0.0f; // Mục tiêu đã qua ramping
static bool  filtInit = false; // Dùng chung 1 cờ
static uint32_t last_pid_ms = 0;  // mốc thời gian cho PID

// ===================== ROS2 =====================
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

rcl_subscription_t sub_cmdvel;
geometry_msgs__msg__Twist msg_cmdvel;

rcl_publisher_t odom_pub;
nav_msgs__msg__Odometry odom_msg;

rcl_timer_t odom_timer;  // 20 ms odom loop

// ===================== ODOM helper =====================
Odometry odom;
unsigned long prev_odom_ms = 0;
unsigned long long time_offset_ms = 0;

// ===================== Slew state / output =====================
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
  
  // Ghi thẳng vào mảng global mà PID_Speed.cpp có thể đọc
  computeWheelRPM(m->linear.x, m->angular.z, sp_rpm[M_LEFT], sp_rpm[M_RIGHT]); 
  
  last_cmdvel_ms = millis();
}

// ===================== VÒNG LẶP ĐIỀU KHIỂN CHÍNH =====================
static void run_control_step() {
  // --- Tính dt (delta-time) ---
  uint32_t now = millis();
  float dt_s = (last_pid_ms == 0) ? (calcPeriodMs/1000.0f) : (now - last_pid_ms)/1000.0f;
  if (dt_s < 1e-4f) dt_s = calcPeriodMs/1000.0f;
  last_pid_ms = now;

  // 1. Tính toán tốc độ hiện tại từ encoder
  calcSpeed(); // Đọc meas_rpm_out[0] và meas_rpm_out[1]

  // Áp dụng cờ invert_enc cho tốc độ đo được
  if (invert_enc[M_LEFT])  meas_rpm_out[M_LEFT]  = -meas_rpm_out[M_LEFT];
  if (invert_enc[M_RIGHT]) meas_rpm_out[M_RIGHT] = -meas_rpm_out[M_RIGHT];

  // 2. === Khởi tạo & lọc số đo (Làm 2 lần) ===
  if (!filtInit) { 
    measFilt_L   = meas_rpm_out[M_LEFT]; 
    prevMeas_L   = meas_rpm_out[M_LEFT];
    sp_rpm_cmd_L = sp_rpm[M_LEFT]; 
    measFilt_R   = meas_rpm_out[M_RIGHT]; 
    prevMeas_R   = meas_rpm_out[M_RIGHT]; 
    sp_rpm_cmd_R = sp_rpm[M_RIGHT];
    filtInit     = true; 
  } else { 
    measFilt_L  += KD_ALPHA * (meas_rpm_out[M_LEFT] - measFilt_L); // Dùng KD_ALPHA để lọc
    measFilt_R  += KD_ALPHA * (meas_rpm_out[M_RIGHT] - measFilt_R);
  }

  // 3. === Giới hạn tốc độ thay đổi setpoint (Ramping) (Làm 2 lần) ===
  float maxStep = SP_RATE_RPM_PER_S * dt_s;
  
  float deltaSP_L = sp_rpm[M_LEFT] - sp_rpm_cmd_L;
  if      (deltaSP_L >  maxStep) sp_rpm_cmd_L += maxStep;
  else if (deltaSP_L < -maxStep) sp_rpm_cmd_L -= maxStep;
  else                           sp_rpm_cmd_L  = sp_rpm[M_LEFT];

  float deltaSP_R = sp_rpm[M_RIGHT] - sp_rpm_cmd_R;
  if      (deltaSP_R >  maxStep) sp_rpm_cmd_R += maxStep;
  else if (deltaSP_R < -maxStep) sp_rpm_cmd_R -= maxStep;
  else                           sp_rpm_cmd_R  = sp_rpm[M_RIGHT];

  // 4. === CHẠY PID KÉP (Dùng chung kp, ki, kd từ Global.h) ===
  
  // ----- L bánh -----
  float err_L   = sp_rpm_cmd_L - measFilt_L;
  float dmeas_L = (measFilt_L - prevMeas_L) / dt_s;
  prevMeas_L    = measFilt_L;
  dmeas_filt_L += KD_ALPHA * (dmeas_L - dmeas_filt_L);
  
  float u_pd_L = kp * err_L - kd * dmeas_filt_L;
  float u_ff_L = KFF * sp_rpm_cmd_L;
  if (fabsf(sp_rpm_cmd_L) > 1.0f) u_ff_L += (sp_rpm_cmd_L > 0 ? +MIN_PWM_FF : -MIN_PWM_FF);

  float u_L_final = u_pd_L + u_ff_L + ki * integ_L; // Tạm thời chưa anti-windup

  // ----- R bánh -----
  float err_R   = sp_rpm_cmd_R - measFilt_R;
  float dmeas_R = (measFilt_R - prevMeas_R) / dt_s;
  prevMeas_R    = measFilt_R;
  dmeas_filt_R += KD_ALPHA * (dmeas_R - dmeas_filt_R);

  float u_pd_R = kp * err_R - kd * dmeas_filt_R;
  float u_ff_R = KFF * sp_rpm_cmd_R;
  if (fabsf(sp_rpm_cmd_R) > 1.0f) u_ff_R += (sp_rpm_cmd_R > 0 ? +MIN_PWM_FF : -MIN_PWM_FF);
  
  float u_R_final = u_pd_R + u_ff_R + ki * integ_R;

  // 5. === Anti-windup (Làm 2 lần) ===
  const float UMAX = 255.0f, UMIN = -255.0f;
  
  // Anti-windup L
  if (u_L_final > UMAX && err_L > 0) { /* không cộng integ */ }
  else if (u_L_final < UMIN && err_L < 0) { /* không cộng integ */ }
  else {
    // Chỉ tích phân khi không bão hòa 
    integ_L += err_L * dt_s; 
  } 
  
  // Anti-windup R
  if (u_R_final > UMAX && err_R > 0) { /* không cộng integ */ }
  else if (u_R_final < UMIN && err_R < 0) { /* không cộng integ */ }
  else { 
    integ_R += err_R * dt_s; 
  }
  
  // Tính lại đầu ra cuối cùng với giá trị Tích phân đã cập nhật
  u_L_final = u_pd_L + u_ff_L + ki * integ_L;
  u_R_final = u_pd_R + u_ff_R + ki * integ_R;

  // 6. KẾT NỐI DỮ LIỆU RA (Output)
  out_pwm_L = constrain(u_L_final, UMIN, UMAX);
  out_pwm_R = constrain(u_R_final, UMIN, UMAX);

  // 7. ÁP DỤNG PWM VÀ SLEW-RATE
  apply_pwm_modeaware();
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

static void odom_timer_cb(rcl_timer_t *, int64_t) {
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

  // Timers: 20 ms odom
  RCCHECK(rclc_timer_init_default(
    &odom_timer, &support, RCL_MS_TO_NS(20), odom_timer_cb));

  // Executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmdvel, &msg_cmdvel, &cmdvel_cb, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &odom_timer));

  // Reset PID state
  integ_L = 0.0f;
  integ_R = 0.0f;
  measFilt_L = 0.0f;
  measFilt_R = 0.0f;
  prevMeas_L = 0.0f;
  prevMeas_R = 0.0f;
  dmeas_filt_L = 0.0f;
  dmeas_filt_R = 0.0f;
  sp_rpm_cmd_L = 0.0f;
  sp_rpm_cmd_R = 0.0f;
  filtInit = false;

  out_pwm_L = out_pwm_R = 0.0f; // Các biến này có trong Global.h
  slew_L = slew_R = 0.0f;

  last_pid_ms = millis();
  prev_odom_ms = millis();

  controlMode = MODE_SPEED;
}

// ===================== LOOP =====================
void loop() {
  // 1. Xử lý các tác vụ ROS (nhận /cmd_vel, gửi /odom)
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));

  // 2. VÒNG LẶP ĐIỀU KHIỂN (GIỐNG HỆT CODE BT)
  //    (calcPeriodMs được định nghĩa trong Global.h, vd: 20ms)
  if (millis() - lastCalcMs >= calcPeriodMs) {
    
    // Chạy toàn bộ logic: CalcSpeed -> PID -> ApplyPWM
    run_control_step();
    
    // Cập nhật mốc thời gian
    lastCalcMs = millis();
  }

  // 3. Cơ chế an toàn (Tùy chọn, nhưng nên giữ)
  //    Nếu không nhận được lệnh ROS trong 200ms, dừng robot
  if (millis() - last_cmdvel_ms > 200) {
    sp_rpm[M_LEFT] = 0.0f;    // Ghi vào mảng global
    sp_rpm[M_RIGHT] = 0.0f;
  }

  delay(2); // Thêm một độ trễ nhỏ để ổn định
}
