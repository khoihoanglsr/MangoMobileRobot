// globals.cpp — ONE TRUE DEFINITION cho các biến extern
#include <Arduino.h>
#include "Tien_ich.h"
#include "PWM_DIR.h"
#include "Encoder_Xuly.h"
#include "Tinh_toan_TocDoGoc.h"

bool armed = true;
bool holdEnable = false;
float sp_rpm;
float sp_rpm_cmd;
float integ;

// ==== Encoder core ====
volatile long    encCount[2] = {0, 0};
volatile uint8_t prevAB[2]   = {0, 0};

// ==== Điều khiển/PWM ====
bool  invert_dir[2] = {false, true};
bool  invert_enc[2]  = {false, true};
int   cmdOpenLoop[2] = {0, 0};
int   outCmd[2]     = {0, 0};
int   controlMode   = MODE_PULSE;   
int   CMD_DEAD      = 6;
int   CMD_DEAD_PID  = 0;
bool  echoCmd       = false;

// ==== Đo lường tốc độ ====
unsigned long lastCalcMs     = 0;
float  meas_rpm_out[2]       = {0.0f, 0.0f};
long   lastCount[2]          = {0, 0};
float  meas_cps[2]           = {0.0f, 0.0f};
float  goc_out_deg[2]        = {0.0f, 0.0f};
float  goc_encoder_deg[2]    = {0.0f, 0.0f};

// ==== Tham số cấu hình ====
int   PWM_FREQ_HZ          = 2000;
float SLEW_RATE_PER_MS     = 0.002f;
bool  BYPASS_SLEW_IN_SPEED = true;

// ==== Thông số encoder/truyền động ====
volatile long CPRx4     = 2000;    // 500 CPR x4
float        gearRatio  = 14.0f;
float        encScale   = 1.0f;

// ==== Bộ đếm OUT & phần lẻ ====
volatile long encCountOut_raw[2]     = {0, 0};
double       encOutFrac[2]           = {0.0, 0.0};
volatile long encCountMotorScaled[2] = {0, 0};
double       motFrac[2]              = {0.0, 0.0};

// ==== Các biến khác trong Tien_ich.h (nếu dùng) ====
// String rx; bool armed; int cmdOpenLoop[2]; bool streamOn, streamCSV;
// unsigned long streamRateMs, lastStreamMs; bool plotOn; float slew_cmd[2];
// float sp_rpm, kp, ki, kd, integ; float last_u_pid;
// float KFF, MIN_PWM_FF, KD_ALPHA, SP_RATE_RPM_PER_S, sp_rpm_cmd;
// -> Nếu .cpp khác dùng mà linker báo thiếu, định nghĩa thêm ở đây giống kiểu.

float KD_ALPHA;
float kp, ki, kd;
float KFF;
float MIN_PWM_FF;
float last_u_pid;
float SP_RATE_RPM_PER_S;