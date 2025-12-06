#ifndef GLOBAL_H
#define GLOBAL_H

#include <Arduino.h>
#include <math.h>

#ifndef PWM_CONTROL
#define PWM_CONTROL 255   // giá trị PWM tay: 0..255
#endif


// ===== CHÂN (2 động cơ, DIR+PWM) =====
// Motor 0 (trái)
#define M0_PWM_PIN    25
#define M0_DIR_PIN    26
#define M0_ENC_A_PIN  34
#define M0_ENC_B_PIN  35
// Motor 1 (phải)
#define M1_PWM_PIN    27
#define M1_DIR_PIN    14
#define M1_ENC_A_PIN  32
#define M1_ENC_B_PIN  33

#define M_LEFT  0
#define M_RIGHT 1

// ===== PWM =====
static const int PWM_BITS = 10;                 // 0..1023
static const int PWM_MAX  = (1<<PWM_BITS)-1;
extern int   PWM_FREQ_HZ;

// ===== THỜI GIAN TÍNH TOÁN =====
static const unsigned long calcPeriodMs = 1;    

// CPR "hiển thị" mong muốn (X1)
static const int CPR_USER = 500;

// ----- Chế độ điều khiển -----
enum { MODE_PULSE=0, MODE_SPEED=1 };

// ===== BIẾN TOÀN CỤC (extern) =====

// Tham số encoder/chuyển đổi dùng chung
extern volatile long CPRx4;        // vd 2000 = 500CPR x4
extern float gearRatio;            // vd 14.0
extern float encScale;             // hệ số bù encoder

// Encoder theo từng bánh
extern volatile long    encCount[2];
extern volatile uint8_t prevAB[2];

// Hiệu chuẩn theo từng bánh
extern bool  calActive[2];
extern long  calStartCount[2];

// Tick tính toán
extern unsigned long lastCalcMs;

// Đếm & đo lường theo từng bánh
extern long   lastCount[2];
extern float  meas_cps[2];
extern float  meas_rpm_out[2];
extern float  sp_rpm[2];
extern float  goc_out_deg[2];
extern float  goc_encoder_deg[2];

// Bộ đếm OUT & phần lẻ theo từng bánh
extern volatile long encCountOut_raw[2];
extern double        encOutFrac[2];
extern volatile long encCountMotorScaled[2];
extern double        motFrac[2];

// Giao tiếp/arm
extern String rx;
extern bool   armed;

// Điều khiển hở vòng & đầu ra theo từng bánh
extern int   cmdOpenLoop[2];   // -255..+255
extern int   outCmd[2];        // -255..+255
extern int   CMD_DEAD;         // deadband cho PULSE

// Đảo chiều theo từng bánh
extern bool  invert_dir[2];
extern bool  invert_enc[2];

// Streaming/plot
extern bool  streamOn, streamCSV;
extern unsigned long streamRateMs, lastStreamMs;

// Slew-rate (giới hạn tốc độ thay đổi) theo từng bánh
extern float SLEW_RATE_PER_MS;
extern float slew_cmd[2];

// Chế độ & HOLD
extern int   controlMode;      // MODE_PULSE / MODE_SPEED
extern bool  holdEnable;

// PID CHUNG

extern float kp, ki, kd, integ;
extern float last_u_pid;

extern float out_pwm_L;
extern float out_pwm_R;

// Tuỳ chọn PID
extern bool  BYPASS_SLEW_IN_SPEED;
extern int   CMD_DEAD_PID;

extern float KFF;
extern float MIN_PWM_FF;

extern float KD_ALPHA;
extern float SP_RATE_RPM_PER_S;
extern float sp_rpm_cmd;

extern unsigned long lastCmdMs;
extern const uint32_t COMMAND_TIMEOUT_MS;

extern volatile long encCountOut_raw[2];
extern double        encOutFrac[2];

// Bộ đếm Motor Scaled & phần lẻ theo từng bánh
extern volatile long encCountMotorScaled[2];
extern double        motFrac[2];
#endif
