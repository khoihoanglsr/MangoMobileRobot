#include "Global.h"

// ===== PWM & encoder chung =====
int   PWM_FREQ_HZ = 2000;          // Hz
volatile long CPRx4 = 2000;        // 500 CPR x4 = 2000
float gearRatio    = 14.0f;        // tỷ số truyền
float encScale     = 1.0f;         // hệ số bù

// ===== Encoder theo bánh =====
volatile long    encCount[2] = {0, 0};
volatile uint8_t prevAB[2]   = {0, 0};

// ===== Hiệu chuẩn =====
bool calActive[2]     = {false, false};
long calStartCount[2] = {0, 0};

// ===== Tick tính toán =====
unsigned long lastCalcMs = 0;
unsigned long lastCmdMs = 0;
const uint32_t COMMAND_TIMEOUT_MS = 100;


// ===== Đếm & đo lường =====
long  lastCount[2]       = {0, 0};
float meas_cps[2]        = {0.0f, 0.0f};
float meas_rpm_out[2]    = {0.0f, 0.0f};
float goc_out_deg[2]     = {0.0f, 0.0f};
float goc_encoder_deg[2] = {0.0f, 0.0f};

// ===== Bộ đếm OUT & phần lẻ =====
volatile long encCountOut_raw[2]     = {0, 0};
double        encOutFrac[2]          = {0.0, 0.0};
volatile long encCountMotorScaled[2] = {0, 0};
double        motFrac[2]             = {0.0, 0.0};

// ===== Giao tiếp/arm =====
String rx = "";
bool   armed = false;

// ===== Điều khiển hở vòng & đầu ra =====
int cmdOpenLoop[2] = {0, 0};     // -255..+255
int outCmd[2]      = {0, 0};     // -255..+255
int CMD_DEAD       = 6;          // deadband cho MODE_PULSE

// ===== Đảo chiều =====
bool invert_dir[2] = {false, true};
bool invert_enc[2] = {false, true};

// ===== Slew-rate =====
float SLEW_RATE_PER_MS = 0.0015f;   // %/ms (0.0015 ~ 0.15%/ms)
float slew_cmd[2] = {0.0f, 0.0f};

// ===== Chế độ & HOLD =====
int  controlMode;     // MODE_PULSE / MODE_SPEED
bool holdEnable  = false;

// ===== PID CHUNG =====
float sp_rpm[2] = {0.0f, 0.0f};               
float kp = 0.95f, ki = 0.3f, kd = 0.045f;
float integ = 0.0f;
float last_u_pid = 0.0f;

// ===== Tuỳ chọn PID / Feedforward =====
bool  BYPASS_SLEW_IN_SPEED = true;
int   CMD_DEAD_PID         = 6;

float KFF              = 0.0f;     // feedforward gain
float MIN_PWM_FF       = 5.0f;     // ngưỡng PWM tối thiểu khi có FF
float KD_ALPHA         = 0.3f;     // hệ số lọc D-term
float SP_RATE_RPM_PER_S= 300.0f;   // giới hạn dốc SP (RPM/s)
float sp_rpm_cmd       = 0.0f;     // setpoint RPM

float out_pwm_L;
float out_pwm_R;

// ===================== THÔNG SỐ CƠ KHÍ =====================
const float WHEEL_RADIUS_M = 0.050f;           // bán kính bánh (m)
const float BASE_WIDTH_M   = 0.328f;            // khoảng cách 2 bánh (m)
const float TWO_PI_R = 2.0f * (float)M_PI * WHEEL_RADIUS_M;