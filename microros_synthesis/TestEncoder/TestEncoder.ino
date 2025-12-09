#include "Global.h"
#include "Encoder_Process.h"

const int RUN_PWM = 150;          // tốc độ trung bình
long targetPulse = 0;
bool running = false;

// ===== RESET 2 ENC =====
void resetEncoder() {
  noInterrupts();
  encCount[0] = 0;
  encCount[1] = 0;
  prevAB[0] = 0;
  prevAB[1] = 0;
  interrupts();
}

// ===== STOP MOTOR =====
void stopMotors() {
  ledcWrite(0, 0);
  ledcWrite(1, 0);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  // ===== DIR =====
  pinMode(M0_DIR_PIN, OUTPUT);
  pinMode(M1_DIR_PIN, OUTPUT);

  // ===== ENCODER INPUT =====
  pinMode(M0_ENC_A_PIN, INPUT_PULLUP);
  pinMode(M0_ENC_B_PIN, INPUT_PULLUP);
  pinMode(M1_ENC_A_PIN, INPUT_PULLUP);
  pinMode(M1_ENC_B_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(M0_ENC_A_PIN), encoderISR0A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M0_ENC_B_PIN), encoderISR0B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M1_ENC_A_PIN), encoderISR1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M1_ENC_B_PIN), encoderISR1B, CHANGE);

  // ===== PWM LEDC =====
  ledcSetup(0, 2000, 8);          // channel 0, freq 2kHz, 8-bit
  ledcSetup(1, 2000, 8);          // channel 1
  ledcAttachPin(M0_PWM_PIN, 0);   // Motor 0 PWM
  ledcAttachPin(M1_PWM_PIN, 1);   // Motor 1 PWM

  stopMotors();

  Serial.println("Nhap so vong can chay:");
}

void loop() {

  // ===== Nhập số vòng =====
  if (Serial.available() && !running) {
    float vong = Serial.parseFloat();
    if (vong > 0.0f) {
      resetEncoder();

      targetPulse = (long)(vong * CPRx4 * gearRatio); 
      // 500 CPR x4 = 2000 → 2000 * 14 = 28000 pulses / vòng

      Serial.printf("Bat dau chay %.2f vong -> target = %ld xung\n", vong, targetPulse);

      // chạy cùng chiều
      digitalWrite(M0_DIR_PIN, HIGH);
      digitalWrite(M1_DIR_PIN, HIGH);

      running = true;
    }
    while (Serial.available()) Serial.read();
  }

  // ===== Nếu đang chạy =====
  if (running) {
    long c0 = encCount[0];
    long c1 = encCount[1];
    long avg = (c0 + c1) / 2;

    // ===== Điều khiển đồng tốc =====
    long diff = c0 - c1;

    int pwm0 = RUN_PWM;
    int pwm1 = RUN_PWM;

    if (diff > 20) pwm0 -= 15;    // M0 nhanh hơn → giảm
    if (diff < -20) pwm1 -= 15;   // M1 nhanh hơn → giảm

    pwm0 = constrain(pwm0, 0, 255);
    pwm1 = constrain(pwm1, 0, 255);

    ledcWrite(0, pwm0);
    ledcWrite(1, pwm1);

    // ===== Debug =====
    Serial.printf("c0=%ld  c1=%ld  diff=%ld  pwm0=%d  pwm1=%d\n", c0, c1, diff, pwm0, pwm1);

    // ===== Kiểm tra hoàn thành =====
    if (abs(avg) >= targetPulse) {
      stopMotors();
      running = false;

      Serial.println("\n=== DA HOAN THANH ===");
      Serial.printf("c0 final=%ld, c1 final=%ld\n", c0, c1); // <-- Corrected incomplete Serial.p

    } // <-- Added missing closing brace for 'if (abs(avg) >= targetPulse)'
  } // <-- Added missing closing brace for 'if (running)'
} // <-- Correctly closes 'void loop()'