#include "Tien_ich.h"
#include <math.h>

static const char* dirFromRpm(float rpm){
  if (rpm >  0.2f) return "THUAN";
  if (rpm < -0.2f) return "NGHICH";
  return "DUNG";
}

bool toLongSafe(const String &s, long &out){
  String t=s; t.trim(); if(!t.length()) return false;
  char buf[24]; t.substring(0,23).toCharArray(buf,sizeof(buf));
  char *e; long v=strtol(buf,&e,10); if(e==buf) return false; out=v; return true;
}

bool toFloatSafe(const String &s, float &out){
  String t=s; t.trim(); if(!t.length()) return false;
  char buf[32]; t.substring(0,31).toCharArray(buf,sizeof(buf));
  char *e; float v=strtof(buf,&e); if(e==buf) return false; out=v; return true;
}

void printState(){
  // CPR trục OUT theo đơn vị "hiển thị"
  const double cpr_out_user = (double)CPR_USER * (double)gearRatio;

  // In thông tin chung
  Serial.printf("an_toan=%s  PWM=%dHz  dao_chieu=[L:%s R:%s]  cmddead=%d  slew=%.4f  echo=%s\n",
    armed?"BAT":"TAT",
    PWM_FREQ_HZ,
    invert_dir[0]?"on":"off",
    invert_dir[1]?"on":"off",
    CMD_DEAD,
    SLEW_RATE_PER_MS,
    echoCmd?"on":"off"
  );

  // Tính trung bình rpm
  const float rpm_avg = 0.5f * (meas_rpm_out[0] + meas_rpm_out[1]);

  // In cấu hình & trạng thái tổng quát
  Serial.printf("mode=%s  CPRx4=%ld  gr=%.3f  encScale=%.6f  sp_rpm=%.2f  rpm_avg=%.2f  CPR_OUT=%.1f\n",
    (controlMode==MODE_PULSE)?"PULSE":"SPEED",
    CPRx4,
    gearRatio,
    encScale,
    sp_rpm,
    rpm_avg,
    cpr_out_user
  );

  // % duty từ lệnh (thang lệnh đang dùng là -255..+255)
  const float pctL = (float)(abs(outCmd[0])) * 100.0f / 255.0f;
  const float pctR = (float)(abs(outCmd[1])) * 100.0f / 255.0f;

  // In chi tiết từng động cơ
  Serial.printf("M0: rpm=%.2f  goc_OUT=%.2f  duty=%.1f%%  dir=%s\n",
    meas_rpm_out[0], goc_out_deg[0], pctL, dirFromRpm(meas_rpm_out[0]));
  Serial.printf("M1: rpm=%.2f  goc_OUT=%.2f  duty=%.1f%%  dir=%s\n",
    meas_rpm_out[1], goc_out_deg[1], pctR, dirFromRpm(meas_rpm_out[1]));
}

void printLine(bool csv){
  // % duty từ lệnh (thang lệnh -255..+255)
  const float pctL = (float)(abs(outCmd[0])) * 100.0f / 255.0f;
  const float pctR = (float)(abs(outCmd[1])) * 100.0f / 255.0f;

  // Dir theo rpm từng bánh
  const char* dirL = dirFromRpm(meas_rpm_out[0]);
  const char* dirR = dirFromRpm(meas_rpm_out[1]);

  // Trung bình
  const float rpm_avg = 0.5f * (meas_rpm_out[0] + meas_rpm_out[1]);

  if (!csv) {
    // Dạng cột dễ đọc
    // rpm0   goc0   dir0   pct0    |   rpm1   goc1   dir1   pct1    |   rpm_avg  sp  u_pid
    Serial.print(meas_rpm_out[0], 2);  Serial.print('\t');
    Serial.print(goc_out_deg[0], 2);   Serial.print('\t');
    Serial.print(dirL);                Serial.print('\t');
    Serial.print(pctL, 1);             Serial.print('\t'); Serial.print('|'); Serial.print('\t');

    Serial.print(meas_rpm_out[1], 2);  Serial.print('\t');
    Serial.print(goc_out_deg[1], 2);   Serial.print('\t');
    Serial.print(dirR);                Serial.print('\t');
    Serial.print(pctR, 1);             Serial.print('\t'); Serial.print('|'); Serial.print('\t');

    Serial.print(rpm_avg, 2);          Serial.print('\t');
    Serial.print(sp_rpm, 2);           Serial.print('\t');
    Serial.println(last_u_pid, 2);
  } else {
    // CSV: rpm0,goc0,dir0,pct0,rpm1,goc1,dir1,pct1,rpm_avg,sp_rpm,u_pid
    Serial.print(meas_rpm_out[0], 3);  Serial.print(',');
    Serial.print(goc_out_deg[0], 3);   Serial.print(',');
    Serial.print(dirL);                Serial.print(',');
    Serial.print(pctL, 2);             Serial.print(',');

    Serial.print(meas_rpm_out[1], 3);  Serial.print(',');
    Serial.print(goc_out_deg[1], 3);   Serial.print(',');
    Serial.print(dirR);                Serial.print(',');
    Serial.print(pctR, 2);             Serial.print(',');

    Serial.print(rpm_avg, 3);          Serial.print(',');
    Serial.print(sp_rpm, 3);           Serial.print(',');
    Serial.println(last_u_pid, 3);
  }
}

void printHeader(bool csv){
  if (!csv) {
    Serial.println(F("# rpm0\tgoc0\tdir0\tpct0\t|\trpm1\tgoc1\tdir1\tpct1\t|\trpm_avg\tsp_rpm\tu_pid"));
  } else {
    Serial.println(F("rpm0,goc0,dir0,pct0,rpm1,goc1,dir1,pct1,rpm_avg,sp_rpm,u_pid"));
  }
}
