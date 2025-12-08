#include <Arduino.h>
#include "Calc_Speed.h"
#include "Encoder_Process.h"
#include "Global.h"
#include "PWM_DIR.h"
#include "driver/pcnt.h"
#include "math.h"

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
#include <std_msgs/msg/int32_multi_array.h>


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

// ===================== MECHANIC =====================
static const float WHEEL_RADIUS_M = 0.050f;           
static const float BASE_WIDTH_M   = 0.300f;
static const float TWO_PI_R = 2.0f * (float)M_PI * WHEEL_RADIUS_M;

// Biến trạng thái PID (tách biệt cho 2 bánh)
static float integ_L = 0.0f; 
static float integ_R = 0.0f;

static float measFilt_L = 0.0f;
static float measFilt_R = 0.0f;

static float prevMeas_L = 0.0f; 
static float prevMeas_R = 0.0f;

static float dmeas_filt_L = 0.0f; 
static float dmeas_filt_R = 0.0f;

static float sp_rpm_cmd_L = 0.0f;
static float sp_rpm_cmd_R = 0.0f;

static bool  filtInit = false; // Dùng chung 1 cờ
static uint32_t last_pid_ms = 0;  // mốc thời gian cho PID


// ============== ENCODER/PCNT ====================== 
static const float COUNTS_PER_REV = 7000.0f;

volatile long encCountLeft  = 0;
volatile long encCountRight = 0;

static long lastEncLeft  = 0;
static long lastEncRight = 0;

static float ratio_prev = 1.0f;


// ===================== ROS2 =====================
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

rcl_subscription_t sub_cmdvel;
geometry_msgs__msg__Twist msg_cmdvel;

rcl_publisher_t odom_pub;
nav_msgs__msg__Odometry odom_msg;

rcl_timer_t odom_timer;

rcl_publisher_t enc_pub;
std_msgs__msg__Int32MultiArray enc_msg;

// ===================== ODOM helper ===================
Odometry odom;
unsigned long prev_odom_ms = 0;
unsigned long long time_offset_ms = 0;

// -------- Manual odometry state (meters / radians) --------
static float odom_x = 0.0f;
static float odom_y = 0.0f;
static float odom_yaw = 0.0f;

static long last_enc_L = 0;
static long last_enc_R = 0;

// ===================== Slew state / output =====================
static float slew_L = 0.0f;
static float slew_R = 0.0f;

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

// ===================== Timer Callback =====================
static void ctrl_timer_cb(rcl_timer_t *timer, int64_t last_call_time) {
  (void) timer;
  (void) last_call_time;

  run_control_step();
}

// ===================== CMD_VEL callback =====================
static void cmdvel_cb(const void *msgin) {
  const geometry_msgs__msg__Twist *m = (const geometry_msgs__msg__Twist *)msgin;
  
  // Ghi thẳng vào mảng global
  computeWheelRPM(m->linear.x, m->angular.z, sp_rpm[M_LEFT], sp_rpm[M_RIGHT]);
  
  last_cmdvel_ms = millis();
}

// ===================== VÒNG LẶP ĐIỀU KHIỂN CHÍNH =====================
static void run_control_step() {
  // --- Tính dt (delta-time) ---
  uint32_t now = millis();
  float dt_s = (last_pid_ms == 0) ? (calcPeriodMs/1000.0f) : (now - last_pid_ms) / 1000.0f;

  if (dt_s < 1e-4f) dt_s = calcPeriodMs/1000.0f;
  last_pid_ms = now;

  // 1. Tính toán tốc độ hiện tại từ encoder
  // calcSpeed(); // Đọc meas_rpm_out[0] và meas_rpm_out[1]

  // 1. Cập nhật encoder từ PCNT (đếm tuyệt đối)
  updateEncoderLong(M0_PCNT_UNIT, encCountLeft);
  updateEncoderLong(M1_PCNT_UNIT, encCountRight);

  // 2. Tính delta counts trong khoảng dt_s
  long currEncL = encCountLeft;
  long currEncR = encCountRight;

  long dCountL = currEncL - lastEncLeft;
  long dCountR = currEncR - lastEncRight;

  lastEncLeft  = currEncL;
  lastEncRight = currEncR;

  // 3. Từ delta counts → RPM (độ lớn) 
  float cpsL = (float)dCountL / dt_s;  // counts per second
  float cpsR = (float)dCountR / dt_s;

  float rpsL = cpsL / COUNTS_PER_REV;  // rev/s
  float rpsR = cpsR / COUNTS_PER_REV;

  float rpmL_meas = rpsL * 60.0f;
  float rpmR_meas = rpsR * 60.0f;

   // 3. Lấy dấu từ setpoint (hoặc out_pwm) để xác định hướng
  float signL = (out_pwm_L >= 0.0f) ? +1.0f : -1.0f;
  float signR = (out_pwm_R >= 0.0f) ? +1.0f : -1.0f;

  rpmL_meas *= signL;
  rpmR_meas *= signR;

  meas_rpm_out[M_LEFT]  = rpmL_meas;
  meas_rpm_out[M_RIGHT] = rpmR_meas;

  // 5. Áp dụng invert_enc nếu cần (đảo chiều phần cứng)
  if (invert_enc[M_LEFT])  meas_rpm_out[M_LEFT]  = -meas_rpm_out[M_LEFT];
  if (invert_enc[M_RIGHT]) meas_rpm_out[M_RIGHT] = -meas_rpm_out[M_RIGHT];

  const float UMAX = 255.0f, UMIN = -255.0f;

  // ----------- 1) Khởi tạo & lọc số đo -----------
  if (!filtInit) {
    // L
    measFilt_L   = meas_rpm_out[M_LEFT];
    prevMeas_L   = meas_rpm_out[M_LEFT];
    dmeas_filt_L = 0.0f;
    sp_rpm_cmd_L = sp_rpm[M_LEFT];

    // R
    measFilt_R   = meas_rpm_out[M_RIGHT];
    prevMeas_R   = meas_rpm_out[M_RIGHT];
    dmeas_filt_R = 0.0f;
    sp_rpm_cmd_R = sp_rpm[M_RIGHT];

    filtInit = true;
  } else {
    // Lọc low-pass giống ALPHA trong PID_Speed (dùng KD_ALPHA cho tiện)
    measFilt_L += KD_ALPHA * (meas_rpm_out[M_LEFT]  - measFilt_L);
    measFilt_R += KD_ALPHA * (meas_rpm_out[M_RIGHT] - measFilt_R);
  }

  // ----------- 2) Ramping setpoint -----------
  float maxStep = SP_RATE_RPM_PER_S * dt_s;

  float deltaSP_L = sp_rpm[M_LEFT]  - sp_rpm_cmd_L;

  if (deltaSP_L >  maxStep) 
    sp_rpm_cmd_L += maxStep;
  else if (deltaSP_L < -maxStep) 
    sp_rpm_cmd_L -= maxStep;
  else                           
    sp_rpm_cmd_L  = sp_rpm[M_LEFT];

  float deltaSP_R = sp_rpm[M_RIGHT] - sp_rpm_cmd_R;

  if (deltaSP_R >  maxStep) 
    sp_rpm_cmd_R += maxStep;
  else if (deltaSP_R < -maxStep) 
    sp_rpm_cmd_R -= maxStep;
  else                           
    sp_rpm_cmd_R  = sp_rpm[M_RIGHT];

  // ----------- 3) PID cho từng bánh -----------

  // ===== LEFT WHEEL: MASTER =====
  float err_L   = sp_rpm_cmd_L - measFilt_L;
  float dmeas_L = (measFilt_L - prevMeas_L) / dt_s;

  prevMeas_L    = measFilt_L;
  dmeas_filt_L += KD_ALPHA * (dmeas_L - dmeas_filt_L);

  float u_pd_L = kp * err_L - kd * dmeas_filt_L;
  float u_ff_L = KFF * sp_rpm_cmd_L;

  if (fabsf(sp_rpm_cmd_L) > 1.0f) {
    u_ff_L += (sp_rpm_cmd_L > 0 ? +MIN_PWM_FF : -MIN_PWM_FF);
  }

  float integ_new_L = integ_L + err_L * dt_s;
  float u_try_L     = u_pd_L + u_ff_L + ki * integ_new_L;
  bool satPos_L     = (u_try_L > UMAX);
  bool satNeg_L     = (u_try_L < UMIN);

  if ( (satPos_L && err_L > 0) || (satNeg_L && err_L < 0) ) {
    integ_new_L = integ_L;
  }

  if (ki > 1e-6f) {
    float iMax_L = (UMAX - u_pd_L - u_ff_L) / ki;
    float iMin_L = (UMIN - u_pd_L - u_ff_L) / ki;

    if (integ_new_L > iMax_L) integ_new_L = iMax_L;
    if (integ_new_L < iMin_L) integ_new_L = iMin_L;
  }
  integ_L = integ_new_L;

  float u_L = u_pd_L + u_ff_L + ki * integ_L;
  if (u_L > UMAX) u_L = UMAX;
  if (u_L < UMIN) u_L = UMIN;

  // ===== RIGHT WHEEL: FOLLOWER =====

  const float RATIO_MAX = 2.0f;     
  const float RATIO_MIN = -2.0f;\
  const float RATIO_DEN_MIN = 5.0f;       

  float ratio_new = sp_rpm_cmd_L / sp_rpm_cmd_R;
  if (fabsf(sp_rpm_cmd_L) > RATIO_DEN_MIN) {
    // Giới hạn ratio để tránh nhảy điên
    if (ratio_new > RATIO_MAX) ratio_new = RATIO_MAX;
    if (ratio_new < RATIO_MIN) ratio_new = RATIO_MIN;
  }

  ratio_prev = ratio_new;

  // Bánh phải lấy PWM của bánh trái nhân với tỉ lệ
  float u_R = u_L * ratio_prev;

  if (u_R > UMAX) u_R = UMAX;
  if (u_R < UMIN) u_R = UMIN;

  // ----- VÙNG DỪNG MỀM -----
const float STOP_SP_THRESH = 5.0f;   // rpm

  if (fabsf(sp_rpm_cmd_L) < STOP_SP_THRESH &&
      fabsf(sp_rpm_cmd_R) < STOP_SP_THRESH) {

    // Nếu đã gần dừng, coi như dừng hẳn: bỏ FF + reset I
    integ_L = 0.0f;
    integ_R = 0.0f;

    u_L = 0.0f;
    u_R = 0.0f;
  }

  // 4) Kết nối dữ liệu ra
  out_pwm_L = u_L;
  out_pwm_R = u_R;

  // ---- Publish encoder counts ----
  enc_msg.data.data[0] = encCountLeft;
  enc_msg.data.data[1] = encCountRight;
  RCSOFTCHECK(rcl_publish(&enc_pub, &enc_msg, NULL));

  apply_pwm_modeaware();

}

// ===================== APPLY PWM WITH SLEW RATE =====================
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

    if (SLEW_RATE_PER_MS <= 0.0f || fabsf(diffL) <= maxStep_8bit) 
      slew_L = out_pwm_L;
    else 
      slew_L += (diffL > 0 ? +maxStep_8bit : -maxStep_8bit);

    if (SLEW_RATE_PER_MS <= 0.0f || fabsf(diffR) <= maxStep_8bit) 
      slew_R = out_pwm_R;
    else 
      slew_R += (diffR > 0 ? +maxStep_8bit : -maxStep_8bit);

    applyDutyFromCmd_modeaware(0, (int)lroundf(slew_L));
    applyDutyFromCmd_modeaware(1, (int)lroundf(slew_R));

    tSlew = now;
  }
}

// ===================== ODOM TIMER CALLBACK =====================
static void odom_timer_cb(rcl_timer_t *, int64_t)
{
    unsigned long now_ms = millis();
    static unsigned long prev_ms = now_ms;

    float dt = (now_ms - prev_ms) / 1000.0f;
    if (dt <= 0.0f) dt = 1e-3f;
    prev_ms = now_ms;

    // --- Read current encoder values (absolute counts) ---
    long currL = encCountLeft;
    long currR = encCountRight;

    // --- Raw (always positive) delta counts ---
    long dL_raw = currL - last_enc_L;
    long dR_raw = currR - last_enc_R;

    last_enc_L = currL;
    last_enc_R = currR;

    // --- Lấy dấu từ setpoint rpm (hoặc có thể dùng meas_rpm_out) ---
    float signL = (sp_rpm_cmd_L >= 0.0f) ? +1.0f : -1.0f;
    float signR = (sp_rpm_cmd_R >= 0.0f) ? +1.0f : -1.0f;

    // Nếu gần như dừng thì không cộng thêm nữa
    const float STOP_SP_THRESH = 5.0f; // rpm
    if (fabsf(sp_rpm_cmd_L) < STOP_SP_THRESH) signL = 0.0f;
    if (fabsf(sp_rpm_cmd_R) < STOP_SP_THRESH) signR = 0.0f;

    // --- Signed delta counts ---
    float dL = signL * (float)dL_raw;
    float dR = signR * (float)dR_raw;

    // --- Convert delta counts → distance for each wheel (m) ---
    float distL = dL / COUNTS_PER_REV * TWO_PI_R;
    float distR = dR / COUNTS_PER_REV * TWO_PI_R;

    // --- Differential-drive kinematics ---
    float dist = 0.5f * (distL + distR);                    // forward distance
    float dYaw = (distR - distL) / BASE_WIDTH_M;            // rad change

    // --- Integrate pose ---
    float yaw_mid = odom_yaw + 0.5f * dYaw;
    odom_x += dist * cosf(yaw_mid);
    odom_y += dist * sinf(yaw_mid);
    odom_yaw += dYaw;

    // Normalize yaw into [-π, π]
    if (odom_yaw > M_PI) odom_yaw -= 2 * M_PI;
    if (odom_yaw < -M_PI) odom_yaw += 2 * M_PI;

    // -------- Fill odom_msg --------
    struct timespec ts = toRosNow();
    odom_msg.header.stamp.sec = ts.tv_sec;
    odom_msg.header.stamp.nanosec = ts.tv_nsec;

    odom_msg.pose.pose.position.x = odom_x;
    odom_msg.pose.pose.position.y = odom_y;
    odom_msg.pose.pose.position.z = 0.0f;

    // Yaw → quaternion
    float cy = cosf(odom_yaw * 0.5f);
    float sy = sinf(odom_yaw * 0.5f);
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = sy;
    odom_msg.pose.pose.orientation.w = cy;

    // Velocities (optional)
    odom_msg.twist.twist.linear.x  = dist / dt;
    odom_msg.twist.twist.angular.z = dYaw / dt;

    // Publish
    RCSOFTCHECK(rcl_publish(&odom_pub, &odom_msg, NULL));
}


// ===== ENCODER / PCNT =====

// Hàm đọc PCNT và cộng dồn vào biến long
void updateEncoderLong(pcnt_unit_t unit, volatile long &accum) {
  int16_t raw_cnt = 0;
  
  pcnt_get_counter_value(unit, &raw_cnt); 

  pcnt_counter_clear(unit);

  accum += raw_cnt;
}


// Hàm cấu hình PCNT cho 1 encoder: chỉ dùng chân A làm nguồn xung (X1)
void setup_pcnt_encoder(pcnt_unit_t unit, int pulse_gpio) {
  pcnt_config_t pcnt_config = {};
  pcnt_config.unit = unit;
  pcnt_config.channel = PCNT_CHANNEL_0;

  pcnt_config.pulse_gpio_num = pulse_gpio;
  pcnt_config.ctrl_gpio_num  = PCNT_PIN_NOT_USED;

  pcnt_config.pos_mode       = PCNT_COUNT_INC;
  pcnt_config.neg_mode       = PCNT_COUNT_DIS;

  pcnt_config.hctrl_mode     = PCNT_MODE_KEEP;
  pcnt_config.lctrl_mode     = PCNT_MODE_KEEP;

  pcnt_config.counter_h_lim  = PCNT_H_LIM;
  pcnt_config.counter_l_lim  = PCNT_L_LIM;

  pcnt_unit_config(&pcnt_config);

  pcnt_set_filter_value(unit, 400);
  pcnt_filter_enable(unit);

  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);
  pcnt_counter_resume(unit);
}


// ===================== SETUP =====================
void setup() {
  // --- Serial transport for micro-ROS ---
  Serial.begin(115200);
  delay(200);

  set_microros_transports();
  syncTime();

  // --- Motor DIR ---
  pinMode(M0_DIR_PIN, OUTPUT);
  pinMode(M1_DIR_PIN, OUTPUT);
  digitalWrite(M0_DIR_PIN, LOW);
  digitalWrite(M1_DIR_PIN, LOW);

  analogWriteInitBoth();  
  stopHard();

  // --- Encoder pins & ISR ---
  pinMode(M0_ENC_A_PIN, INPUT);
  pinMode(M0_ENC_B_PIN, INPUT);
  pinMode(M1_ENC_A_PIN, INPUT);
  pinMode(M1_ENC_B_PIN, INPUT);

  setup_pcnt_encoder(M0_PCNT_UNIT, M0_ENC_A_PIN);  
  setup_pcnt_encoder(M1_PCNT_UNIT, M1_ENC_A_PIN);  

  setPWM_8bit_mag(M_LEFT, 0);
  setPWM_8bit_mag(M_RIGHT, 0);


  // --- ROS2 init ---
  allocator = rcl_get_default_allocator();

  RCCHECK(
    rclc_support_init(&support, 0, NULL, &allocator)
  );

  RCCHECK(
    rclc_node_init_default(&node, "esp32_diff_subscriber", "", &support)
  );

  // Subscriber: /cmd_vel 
  RCCHECK(
    rclc_subscription_init_default(
      &sub_cmdvel, 
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist ),
      "cmd_vel"
    )
  );

  // Publisher: /odom
  RCCHECK(rclc_publisher_init_default(
    &odom_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "odom/unfiltered")
  );

  // Publisher: /encRead  (encoder raw counts)
  RCCHECK(rclc_publisher_init_default(
    &enc_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "encRead"
  ));

  enc_msg.data.capacity = 2;
  enc_msg.data.size = 2;
  enc_msg.data.data = (int32_t*) malloc(sizeof(int32_t) * 2);


  // 1. Initial 2 timers
  const unsigned long odom_period_ms = 40; // 25Hz 
  // const unsigned long ctrl_period_ms = 20; // 50Hz

  RCCHECK(
    rclc_timer_init_default( 
      &odom_timer, 
      &support, 
      RCL_MS_TO_NS(odom_period_ms), 
      odom_timer_cb
    )
  );

  // rclc_timer_init_default(
  //   &ctrl_timer,         // timer object
  //   &support,            // ROS2 support (allocator + context)
  //   RCL_MS_TO_NS(20),    // period 20 ms (50 Hz)
  //   ctrl_timer_cb        // callback function
  // );

  // Executor (1 sub, 1 timer)
  RCCHECK(
    rclc_executor_init(&executor, &support.context, 2, &allocator)
  );

  RCCHECK(
    rclc_executor_add_subscription(&executor, &sub_cmdvel, &msg_cmdvel, &cmdvel_cb, ON_NEW_DATA)
  );

  RCCHECK(
    rclc_executor_add_timer(&executor, &odom_timer)
  );

  // RCCHECK(
  //   rclc_executor_add_timer(&executor, &ctrl_timer)
  // );

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

  out_pwm_L = 0.0f;
  out_pwm_R = 0.0f;

  slew_L = 0.0f;
  slew_R = 0.0f;

  last_pid_ms = millis();
  prev_odom_ms = millis();

  controlMode = MODE_PULSE;
}

// ===================== LOOP =====================
void loop() {
  // --- 1. PROCESS ROS TASK (receive /cmd_vel, send /odom) --- 
  RCSOFTCHECK(
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1))
  );

  // --- 2.CONTROL LOOP ---
  if (millis() - lastCalcMs >= calcPeriodMs) {
    run_control_step();
    lastCalcMs = millis();
  }

  // --- 3. SAFETY MECHANISM ---
  if (millis() - last_cmdvel_ms > 200) {
    sp_rpm[M_LEFT]  = 0.0f;
    sp_rpm[M_RIGHT] = 0.0f;

    sp_rpm_cmd_L = 0.0f;
    sp_rpm_cmd_R = 0.0f;

    stopHard();
  }

  delay(2); 
}