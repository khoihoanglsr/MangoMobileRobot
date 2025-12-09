#include <Arduino.h>
#include "global.h"              // pin, hằng số, extern biến dùng chung
#include "PWM_DIR.h"             // điều khiển DIR/PWM
#include "Encoder_Process.h"     // ISR encoder
#include "Calc_Speed.h"          // calcSpeed()
#include "PID_Speed.h"           // updatePIDSpeed()

// Bluetooth Classic (SPP) cho ESP32
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

// ===================== HÀM XỬ LÝ LỆNH ===================== //
void processCmd(String command) {
  if (!armed) return;

  controlMode = MODE_PULSE;

  if (command == "S") { // STOP
    cmdOpenLoop[0] = 0;
    cmdOpenLoop[1] = 0;
    stopHard();
  } else if (command == "G") { // FORWARD LEFT
    cmdOpenLoop[0] = PWM_CONTROL/8;
    cmdOpenLoop[1] = PWM_CONTROL/4;
  } else if (command == "H") { // FORWARD RIGHT
    cmdOpenLoop[0] = PWM_CONTROL/4;
    cmdOpenLoop[1] = PWM_CONTROL/8;
  } else if (command == "I") { // BACKWARD LEFT
    cmdOpenLoop[0] = -PWM_CONTROL/8;
    cmdOpenLoop[1] = -PWM_CONTROL/4;
  } else if (command == "J") { // BACKWARD RIGHT
    cmdOpenLoop[0] = -PWM_CONTROL/4;
    cmdOpenLoop[1] = -PWM_CONTROL/8;
  } else if (command == "F") { // FORWARD
    cmdOpenLoop[0] = PWM_CONTROL;
    cmdOpenLoop[1] = PWM_CONTROL;
  } else if (command == "B") { // BACKWARD
    cmdOpenLoop[0] = -PWM_CONTROL;
    cmdOpenLoop[1] = -PWM_CONTROL;
  } else if (command == "L") { // LEFT
    cmdOpenLoop[0] = -PWM_CONTROL;
    cmdOpenLoop[1] =  PWM_CONTROL;
  } else if (command == "R") { // RIGHT
    cmdOpenLoop[0] =  PWM_CONTROL;
    cmdOpenLoop[1] = -PWM_CONTROL;
  } 

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

// Đọc cặp tín hiệu A/B của encoder
static inline uint8_t readAB_pair(uint8_t pinA, uint8_t pinB){
  uint8_t a = (uint8_t)digitalRead(pinA);
  uint8_t b = (uint8_t)digitalRead(pinB);
  return (uint8_t)((a << 1) | b); // Gộp thành 2 bit
}

// ===================== SETUP ===================== //
void setup() {
  pinMode(M0_DIR_PIN, OUTPUT);
  pinMode(M1_DIR_PIN, OUTPUT);
  digitalWrite(M0_DIR_PIN, LOW);
  digitalWrite(M1_DIR_PIN, LOW);

  analogWriteInitBoth();
  stopHard();

  pinMode(M0_ENC_A_PIN, INPUT);
  pinMode(M0_ENC_B_PIN, INPUT);
  pinMode(M1_ENC_A_PIN, INPUT);
  pinMode(M1_ENC_B_PIN, INPUT);

  // Lưu trạng thái ban đầu của A/B
    prevAB[0] = readAB_pair(M0_ENC_A_PIN, M0_ENC_B_PIN);
    prevAB[1] = readAB_pair(M1_ENC_A_PIN, M1_ENC_B_PIN);

  attachInterrupt(digitalPinToInterrupt(M0_ENC_A_PIN), encoderISR0A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M0_ENC_B_PIN), encoderISR0B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M1_ENC_A_PIN), encoderISR1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M1_ENC_B_PIN), encoderISR1B, CHANGE);

  SerialBT.begin("ESP32_CAR_BT");

  armed = true;

  lastCalcMs = millis();
  lastCmdMs  = millis(); 
}

// ===================== LOOP ===================== //
void loop() {
  // Nhận lệnh Bluetooth (ký tự đơn hoặc theo dòng)
  if (SerialBT.available()) {
    String cmd = SerialBT.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() == 0) {
      while (SerialBT.available()) {
        char c = SerialBT.read();
        if (c == '\r' || c == '\n') continue;
        String t(1, c);
        processCmd(t);
      }
    } else {
      processCmd(cmd);
    }
  }

  // Tick điều khiển định kỳ
  if (millis() - lastCalcMs >= calcPeriodMs) {
    calcSpeed();
    updatePIDSpeed();
    lastCalcMs = millis();
  }

  // Slew-rate limiter cho MODE_PULSE (và MODE_SPEED nếu bạn tắt bypass)
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

      for (int i=0;i<2;++i) {
        float diff = (float)outCmd[i] - slew_cmd[i];

        if (SLEW_RATE_PER_MS <= 0.0f || fabsf(diff) <= maxStep_8bit) {
          slew_cmd[i] = (float)outCmd[i];
        } else {
          slew_cmd[i] += (diff > 0 ? +maxStep_8bit : -maxStep_8bit);
        }

        if (armed) 
          applyDutyFromCmd_modeaware(i, (int)lroundf(slew_cmd[i]));
        else       
          stopHard();
      }
      tSlew = now;
    }
  }

  delay(1);
}
