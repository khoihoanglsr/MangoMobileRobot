#include "Lenh_UART.h"
#include "PWM_DIR.h"
#include "Tinh_toan_TocDoGoc.h"

static inline void analogWriteFrequencyBoth(int hz){
  // Nếu dùng Arduino AVR: analogWriteFrequency(pin, hz) không phải hàm chuẩn.
  // ESP32: dùng ledcSetup/Attach trong setup. Ở đây giữ nguyên gọi cho 2 chân nếu SDK hỗ trợ.
  analogWriteFrequency(M0_PWM_PIN, hz);
  analogWriteFrequency(M1_PWM_PIN, hz);
}

void processSerialCommand(){
  while(Serial.available()){
    char c=Serial.read();
    if(c=='\r') continue;
    if(c=='\n'){
      rx.trim();
      if(rx.length()){
        String line=rx; rx="";
        String u=line; u.toLowerCase();

        if(u=="help"){
          Serial.println(F("arm | disarm | state | dem(count) [/0,/1]"));
          Serial.println(F("-255..255 (PULSE, cả 2) | m0/m1 <-255..255> | raw <0..255> | raw0/raw1 <0..255>"));
          Serial.println(F("dir fw|rv (cả 2) | dir0 fw|rv | dir1 fw|rv | invert on|off | invert0 on|off | invert1 on|off"));
          Serial.println(F("pfreq <Hz> | cpr <N> | gr <G> | encscale <k> | zero [/0,/1] | speed [/0,/1]"));
          Serial.println(F("mode pulse|speed | hold <rpm> | holdoff | kp/ki/kd <x> | slew <pct/ms> | cmddead <n>"));
          Serial.println(F("V <rpm> | plot on/off | stream on/off | rate <ms> | csv on/off"));
          Serial.println(F("kff <x> | minff <x> | kdalp <0..1> | sprate <rpm/s> | slew_bypass on|off"));
          continue;
        }
        if(u=="state"){ printState(); continue; }

        if(u=="arm"){ armed=true;  Serial.println(F("[AN TOÀN] ĐÃ BẬT (ARMED)")); continue; }
        if(u=="disarm"){ armed=false; holdEnable=false; integ=0; stopHard(); Serial.println(F("[AN TOÀN] ĐÃ TẮT (DISARMED) -> DỪNG")); continue; }

        if(u=="mode speed"){
          controlMode = MODE_SPEED;
          holdEnable = true;
          integ = 0;
          Serial.println(F("[OK] MODE = SPEED (PID rpm). Dùng 'V <rpm>' để đặt tốc độ."));
          continue;
        }
        if(u=="mode pulse"){
          controlMode = MODE_PULSE;
          holdEnable = false;
          integ = 0;
          Serial.println(F("[OK] MODE = PULSE (PWM tay). Nhập -255..255 hoặc 'raw <0..255>'.")); 
          continue;
        }

        // ====== DIR (cả 2 / riêng từng bánh) ======
        if(u.startsWith("dir0")){
          if(u.endsWith("fw")){ setDir(0, true);  Serial.println(F("[OK] M0: Chiều = THUẬN")); }
          else if(u.endsWith("rv")){ setDir(0, false); Serial.println(F("[OK] M0: Chiều = NGHỊCH")); }
          else Serial.println(F("[LỖI] dir0 fw|rv"));
          continue;
        }
        if(u.startsWith("dir1")){
          if(u.endsWith("fw")){ setDir(1, true);  Serial.println(F("[OK] M1: Chiều = THUẬN")); }
          else if(u.endsWith("rv")){ setDir(1, false); Serial.println(F("[OK] M1: Chiều = NGHỊCH")); }
          else Serial.println(F("[LỖI] dir1 fw|rv"));
          continue;
        }
        if(u.startsWith("dir")){
          if(u.endsWith("fw")){ setDir(0,true); setDir(1,true);  Serial.println(F("[OK] Cả 2: THUẬN")); }
          else if(u.endsWith("rv")){ setDir(0,false); setDir(1,false); Serial.println(F("[OK] Cả 2: NGHỊCH")); }
          else Serial.println(F("[LỖI] dir fw|rv"));
          continue;
        }

        // ====== INVERT (cả 2 / riêng từng bánh) ======
        if(u.startsWith("invert0")){
          if(u.endsWith("on")) { invert_dir[0]=true;  Serial.println(F("[OK] M0: invert=ON")); }
          else if(u.endsWith("off")){ invert_dir[0]=false; Serial.println(F("[OK] M0: invert=OFF")); }
          else Serial.println(F("[LỖI] invert0 on|off"));
          continue;
        }
        if(u.startsWith("invert1")){
          if(u.endsWith("on")) { invert_dir[1]=true;  Serial.println(F("[OK] M1: invert=ON")); }
          else if(u.endsWith("off")){ invert_dir[1]=false; Serial.println(F("[OK] M1: invert=OFF")); }
          else Serial.println(F("[LỖI] invert1 on|off"));
          continue;
        }
        if(u.startsWith("invert")){
          if(u.endsWith("on")) { invert_dir[0]=invert_dir[1]=true;  Serial.println(F("[OK] invert=ON (cả 2)")); }
          else if(u.endsWith("off")){ invert_dir[0]=invert_dir[1]=false; Serial.println(F("[OK] invert=OFF (cả 2)")); }
          else Serial.println(F("[LỖI] invert on|off"));
          continue;
        }

        if(u.startsWith("echo")){
          if(u.endsWith("on")) { echoCmd=true;  Serial.println(F("[OK] echo=ON")); }
          else if(u.endsWith("off")){ echoCmd=false; Serial.println(F("[OK] echo=OFF")); }
          else Serial.println(F("[LỖI] echo on|off"));
          continue;
        }

        // ====== PWM frequency cho 2 pin PWM ======
        if(u.startsWith("pfreq")){
          long hz; if(toLongSafe(u.substring(5),hz)&&hz>=50&&hz<=30000){
            PWM_FREQ_HZ=(int)hz; 
            analogWriteFrequencyBoth(PWM_FREQ_HZ);
            Serial.printf("[OK] PWM=%d Hz (M0 & M1)\n", PWM_FREQ_HZ);
          } else Serial.println(F("[LỖI] pfreq 50..30000"));
          continue;
        }

        // ====== Thông số encoder/chuyển đổi ======
        if(u.startsWith("cpr")){
          long n; if(toLongSafe(u.substring(3),n)&&n>0){ CPRx4=n; Serial.printf("[OK] CPRx4=%ld\n",CPRx4); }
          else Serial.println(F("[LỖI] cpr <N>"));
          continue;
        }
        if(u.startsWith("gr")){
          float g; if(toFloatSafe(u.substring(2),g)&&g>0.01f){ gearRatio=g; Serial.printf("[OK] gr=%.3f (CPR_OUT=%.1f)\n",gearRatio, (double)CPR_USER*gearRatio); }
          else Serial.println(F("[LỖI] gr <G>"));
          continue;
        }
        if(u.startsWith("encscale")){
          float k; if(toFloatSafe(u.substring(8),k)&&k>0.2f&&k<5.0f){ encScale=k; Serial.printf("[OK] encScale=%.6f\n",encScale); }
          else Serial.println(F("[LỖI] encscale 0.2..5.0"));
          continue;
        }

        // ====== Calibrate vòng quay (áp dụng chung — nếu cần tách, có thể thêm calrot0/calrot1) ======
        if(u=="calrot start"){
          calActive[0]=calActive[1]=true; 
          noInterrupts(); calStartCount[0]=encCount[0]; calStartCount[1]=encCount[1]; interrupts();
          Serial.println(F("[CAL] Bắt đầu: quay đúng N vòng TRỤC OUT (cả 2), rồi gõ 'calrot done N'"));
          continue;
        }
        if(u.startsWith("calrot done")){
          if(!calActive[0] && !calActive[1]){ Serial.println(F("[LỖI] chưa 'calrot start'")); continue; }
          float turns; if(toFloatSafe(u.substring(11),turns) && turns>0.2f){
            for(int i=0;i<2;i++){
              if(!calActive[i]) continue;
              long c1; noInterrupts(); c1=encCount[i]; interrupts();
              long dc=c1 - calStartCount[i];
              double exp_counts = (double)turns * (double)CPRx4 * (double)gearRatio;
              if (fabs((double)dc) < 10.0 || exp_counts < 1.0){
                Serial.printf("[LỖI] M%d: quay chưa đủ xa hoặc tham số sai\n", i);
              } else {
                double k = exp_counts / fabs((double)dc);
                encScale = (float)constrain(k, 0.2, 5.0);
                Serial.printf("[CAL M%d] N=%.3f  đo=%ld  kỳ vọng=%.1f  => encScale=%.6f\n",
                              i, turns, dc, exp_counts, encScale);
              }
              calActive[i]=false;
            }
          } else Serial.println(F("[LỖI] calrot done <số_vòng_OUT>"));
          continue;
        }

        // ====== Giữ tốc / PID ======
        if(u=="holdoff"){ holdEnable=false; integ=0; Serial.println(F("[OK] Giữ tốc = OFF")); continue; }
        if(u.startsWith("hold")){
          float rpm; if(toFloatSafe(u.substring(4),rpm)){
            sp_rpm=rpm; holdEnable=true; integ=0; controlMode=MODE_SPEED;
            Serial.printf("[OK] Giữ tốc = ON, đặt %.2f rpm (MODE=SPEED)\n", sp_rpm);
          } else Serial.println(F("[LỖI] hold <rpm>"));
          continue;
        }
        if(u.startsWith("kp")){
          float v; if(toFloatSafe(u.substring(2),v)){ kp=v; Serial.printf("[OK] kp=%.4f\n",kp); }
          else Serial.println(F("[LỖI] kp"));
          continue;
        }
        if(u.startsWith("ki")){
          float v; if(toFloatSafe(u.substring(2),v)){ ki=v; Serial.printf("[OK] ki=%.4f\n",ki); }
          else Serial.println(F("[LỖI] ki"));
          continue;
        }
        if(u.startsWith("kd")){
          float v; if(toFloatSafe(u.substring(2),v)){ kd=v; Serial.printf("[OK] kd=%.4f\n",kd); }
          else Serial.println(F("[LỖI] kd"));
          continue;
        }

        if(u.startsWith("v")){
          float rpm;
          if(toFloatSafe(u.substring(1), rpm)){
            sp_rpm = rpm;
            controlMode = MODE_SPEED;
            holdEnable = true;
            integ = 0;
            Serial.printf("[OK] V=%.2f rpm (MODE=SPEED, hold=ON)\n", sp_rpm);
          } else {
            Serial.println(F("[LỖI] v <rpm>  (ví dụ: V 100)"));
          }
          continue;
        }

        // ====== Plot/Stream ======
        if(u=="plot on"){
          plotOn=true;
          streamOn=true;
          Serial.println(F("[OK] plotter ON (SP, MEAS, PWM)."));
          continue;
        }
        if(u=="plot off"){ plotOn=false; Serial.println(F("[OK] plotter OFF")); continue; }

        if(u.startsWith("kff")){
          float v; if(toFloatSafe(u.substring(3),v)){ KFF=v; Serial.printf("[OK] kff=%.3f\n",KFF); }
          else Serial.println(F("[LỖI] kff <x>"));
          continue;
        }
        if(u.startsWith("minff")){
          float v; if(toFloatSafe(u.substring(5),v)&&v>=0){ MIN_PWM_FF=v; Serial.printf("[OK] minff=%.2f\n",MIN_PWM_FF); }
          else Serial.println(F("[LỖI] minff <>=0>"));
          continue;
        }
        if(u.startsWith("kdalp")){
          float v; if(toFloatSafe(u.substring(5),v)&&v>=0&&v<=1){ KD_ALPHA=v; Serial.printf("[OK] kdalp=%.3f\n",KD_ALPHA); }
          else Serial.println(F("[LỖI] kdalp 0..1"));
          continue;
        }
        if(u.startsWith("sprate")){
          float v; if(toFloatSafe(u.substring(6),v)&&v>0){ SP_RATE_RPM_PER_S=v; Serial.printf("[OK] sprate=%.1f rpm/s\n",SP_RATE_RPM_PER_S); }
          else Serial.println(F("[LỖI] sprate >0"));
          continue;
        }
        if(u.startsWith("slew_bypass")){
          if(u.endsWith("on"))  { BYPASS_SLEW_IN_SPEED=true;  Serial.println(F("[OK] slew_bypass=ON (bỏ ramp ở SPEED)")); }
          else if(u.endsWith("off")) { BYPASS_SLEW_IN_SPEED=false; Serial.println(F("[OK] slew_bypass=OFF")); }
          else Serial.println(F("[LỖI] slew_bypass on|off"));
          continue;
        }

        if(u.startsWith("slew")){
          float v; if(toFloatSafe(u.substring(4),v)&&v>=0.0f){ SLEW_RATE_PER_MS=v; Serial.printf("[OK] slew=%.5f %%/ms\n",SLEW_RATE_PER_MS); }
          else Serial.println(F("[LỖI] slew <pct/ms>"));
          continue;
        }
        if(u.startsWith("cmddead")){
          long n; if(toLongSafe(u.substring(7),n)&&n>=0&&n<=50){ CMD_DEAD=(int)n; Serial.printf("[OK] cmddead=%d (PULSE)\n",CMD_DEAD); }
          else Serial.println(F("[LỖI] cmddead 0..50"));
          continue;
        }

        // ====== ZERO (cả 2 / riêng từng bánh) ======
        if(u=="zero0" || u=="zero 0"){
          noInterrupts(); encCount[0]=0; interrupts();
          lastCount[0]=0; goc_out_deg[0]=0.0f; goc_encoder_deg[0]=0.0f;
          encOutFrac[0]=0.0; encCountOut_raw[0]=0;
          motFrac[0]=0.0; encCountMotorScaled[0]=0;
          Serial.println(F("[OK] M0: Đã xóa bộ đếm & góc"));
          continue;
        }
        if(u=="zero1" || u=="zero 1"){
          noInterrupts(); encCount[1]=0; interrupts();
          lastCount[1]=0; goc_out_deg[1]=0.0f; goc_encoder_deg[1]=0.0f;
          encOutFrac[1]=0.0; encCountOut_raw[1]=0;
          motFrac[1]=0.0; encCountMotorScaled[1]=0;
          Serial.println(F("[OK] M1: Đã xóa bộ đếm & góc"));
          continue;
        }
        if(u=="zero"){
          for(int i=0;i<2;i++){
            noInterrupts(); encCount[i]=0; interrupts();
            lastCount[i]=0; goc_out_deg[i]=0.0f; goc_encoder_deg[i]=0.0f;
            encOutFrac[i]=0.0; encCountOut_raw[i]=0;
            motFrac[i]=0.0; encCountMotorScaled[i]=0;
          }
          Serial.println(F("[OK] Đã xóa bộ đếm & góc (cả 2)"));
          continue;
        }

        // ====== SPEED / đo nhanh (cả 2 / từng bánh) ======
        if(u=="speed0" || u=="speed 0"){
          calcSpeed();
          Serial.printf("[M0] rpm=%.2f  cps=%.2f  scale=%.6f  goc_OUT=%.2f° (CPR_OUT=%.1f)\n",
                        meas_rpm_out[0], meas_cps[0], encScale, goc_out_deg[0], (double)CPR_USER*gearRatio);
          continue;
        }
        if(u=="speed1" || u=="speed 1"){
          calcSpeed();
          Serial.printf("[M1] rpm=%.2f  cps=%.2f  scale=%.6f  goc_OUT=%.2f° (CPR_OUT=%.1f)\n",
                        meas_rpm_out[1], meas_cps[1], encScale, goc_out_deg[1], (double)CPR_USER*gearRatio);
          continue;
        }
        if(u=="speed"){
          calcSpeed();
          Serial.printf("[M0] rpm=%.2f  cps=%.2f  goc_OUT=%.2f°\n", meas_rpm_out[0], meas_cps[0], goc_out_deg[0]);
          Serial.printf("[M1] rpm=%.2f  cps=%.2f  goc_OUT=%.2f°\n", meas_rpm_out[1], meas_cps[1], goc_out_deg[1]);
          continue;
        }

        // ====== Đếm xung / vòng (cả 2 / từng bánh) ======
        if(u=="dem0" || u=="count0"){
          calcSpeed();
          double cpr_out_user = (double)CPR_USER * (double)gearRatio;
          double vong_out = (CPRx4>0 && gearRatio>0.0f)
                            ? ((double)encCountMotorScaled[0]/(double)CPRx4) / (double)gearRatio
                            : 0.0;
          long xung_truc_out = (CPRx4>0)
                            ? (long) llround( (double)encCountMotorScaled[0] * (double)CPR_USER / (double)CPRx4 )
                            : 0;
          long motor_raw; noInterrupts(); motor_raw = encCount[0]; interrupts();
          Serial.printf("[M0] xung_motor_raw=%ld | xung_truc_out=%ld | vong_truc_out=%.4f | CPR_OUT=%.1f | goc_out=%.2f°\n",
                        motor_raw, xung_truc_out, vong_out, cpr_out_user, goc_out_deg[0]);
          continue;
        }
        if(u=="dem1" || u=="count1"){
          calcSpeed();
          double cpr_out_user = (double)CPR_USER * (double)gearRatio;
          double vong_out = (CPRx4>0 && gearRatio>0.0f)
                            ? ((double)encCountMotorScaled[1]/(double)CPRx4) / (double)gearRatio
                            : 0.0;
          long xung_truc_out = (CPRx4>0)
                            ? (long) llround( (double)encCountMotorScaled[1] * (double)CPR_USER / (double)CPRx4 )
                            : 0;
          long motor_raw; noInterrupts(); motor_raw = encCount[1]; interrupts();
          Serial.printf("[M1] xung_motor_raw=%ld | xung_truc_out=%ld | vong_truc_out=%.4f | CPR_OUT=%.1f | goc_out=%.2f°\n",
                        motor_raw, xung_truc_out, vong_out, cpr_out_user, goc_out_deg[1]);
          continue;
        }
        if(u=="dem" || u=="count"){
          calcSpeed();
          double cpr_out_user = (double)CPR_USER * (double)gearRatio;
          for(int i=0;i<2;i++){
            double vong_out = (CPRx4>0 && gearRatio>0.0f)
                              ? ((double)encCountMotorScaled[i]/(double)CPRx4) / (double)gearRatio
                              : 0.0;
            long xung_truc_out = (CPRx4>0)
                              ? (long) llround( (double)encCountMotorScaled[i] * (double)CPR_USER / (double)CPRx4 )
                              : 0;
            long motor_raw; noInterrupts(); motor_raw = encCount[i]; interrupts();
            Serial.printf("[M%d] xung_motor_raw=%ld | xung_truc_out=%ld | vong_truc_out=%.4f | CPR_OUT=%.1f | goc_out=%.2f°\n",
                          i, motor_raw, xung_truc_out, vong_out, cpr_out_user, goc_out_deg[i]);
          }
          continue;
        }

        // ====== Stream/CSV ======
        if(u=="stream on"){ streamOn=true; if(!plotOn) printHeader(streamCSV); Serial.println(F("[OK] stream ON")); continue; }
        if(u=="stream off"){ streamOn=false; Serial.println(F("[OK] stream OFF")); continue; }
        if(u=="csv on"){ streamCSV=true; Serial.println(F("[OK] CSV ON")); continue; }
        if(u=="csv off"){ streamCSV=false; Serial.println(F("[OK] CSV OFF")); continue; }
        if(u.startsWith("rate")){
          long ms; 
          if(toLongSafe(u.substring(4),ms)&&ms>=10){
            streamRateMs=ms; lastStreamMs = millis();
            Serial.printf("[OK] rate=%lu ms\n",streamRateMs);
          } else Serial.println(F("[LỖI] rate >=10ms"));
          continue;
        }

        // ====== RAW PWM (cả 2 / từng bánh) ======
        if(u.startsWith("raw0")){
          long d; if(toLongSafe(u.substring(4),d)&&d>=0&&d<=255){
            if(!armed){ Serial.println(F("[AN TOÀN] Chưa ARM: bỏ qua RAW.")); continue; }
            if(controlMode!=MODE_PULSE){ Serial.println(F("[CẢNH BÁO] RAW chỉ dùng ở MODE=PULSE. Dùng 'mode pulse'.")); continue; }
            setPWM_8bit_mag(0, (uint8_t)d);
            Serial.printf("[RAW] M0 PWM8=%ld\n", d);
          } else Serial.println(F("[LỖI] raw0 <0..255>"));
          continue;
        }
        if(u.startsWith("raw1")){
          long d; if(toLongSafe(u.substring(4),d)&&d>=0&&d<=255){
            if(!armed){ Serial.println(F("[AN TOÀN] Chưa ARM: bỏ qua RAW.")); continue; }
            if(controlMode!=MODE_PULSE){ Serial.println(F("[CẢNH BÁO] RAW chỉ dùng ở MODE=PULSE. Dùng 'mode pulse'.")); continue; }
            setPWM_8bit_mag(1, (uint8_t)d);
            Serial.printf("[RAW] M1 PWM8=%ld\n", d);
          } else Serial.println(F("[LỖI] raw1 <0..255>"));
          continue;
        }
        if(u.startsWith("raw")){
          long d; if(toLongSafe(u.substring(3),d)&&d>=0&&d<=255){
            if(!armed){ Serial.println(F("[AN TOÀN] Chưa ARM: bỏ qua RAW.")); continue; }
            if(controlMode!=MODE_PULSE){ Serial.println(F("[CẢNH BÁO] RAW chỉ dùng ở MODE=PULSE. Dùng 'mode pulse'.")); continue; }
            setPWM_8bit_mag(0, (uint8_t)d);
            setPWM_8bit_mag(1, (uint8_t)d);
            Serial.printf("[RAW] PWM8=%ld (M0 & M1)\n", d);
          } else Serial.println(F("[LỖI] raw <0..255>"));
          continue;
        }

        // ====== Lệnh số -255..255 (PULSE) cho từng bánh: m0/m1, hoặc cả 2: số đơn ======
        if(u.startsWith("m0")){
          long val; if(toLongSafe(u.substring(2),val)&&val>=-255&&val<=255){
            if(controlMode!=MODE_PULSE){
              Serial.println(F("[CẢNH BÁO] m0 dùng ở MODE=PULSE. Dùng 'mode pulse' hoặc 'V <rpm>' ở MODE=SPEED."));
            } else {
              if(!armed) Serial.println(F("[AN TOÀN] Chưa ARM: lệnh được lưu, ARM để chạy."));
              cmdOpenLoop[0] = (int)val;
              applyDutyFromCmd_modeaware(0, cmdOpenLoop[0]);
              Serial.printf("[OK] m0=%ld\n", val);
            }
          } else Serial.println(F("[LỖI] m0 <-255..255>"));
          continue;
        }
        if(u.startsWith("m1")){
          long val; if(toLongSafe(u.substring(2),val)&&val>=-255&&val<=255){
            if(controlMode!=MODE_PULSE){
              Serial.println(F("[CẢNH BÁO] m1 dùng ở MODE=PULSE. Dùng 'mode pulse' hoặc 'V <rpm>' ở MODE=SPEED."));
            } else {
              if(!armed) Serial.println(F("[AN TOÀN] Chưa ARM: lệnh được lưu, ARM để chạy."));
              cmdOpenLoop[1] = (int)val;
              applyDutyFromCmd_modeaware(1, cmdOpenLoop[1]);
              Serial.printf("[OK] m1=%ld\n", val);
            }
          } else Serial.println(F("[LỖI] m1 <-255..255>"));
          continue;
        }

        // Số đơn: áp cho cả 2 bánh (giữ tương thích cũ)
        long val;
        if(toLongSafe(line,val)&&val>=-255&&val<=255){
          if(controlMode!=MODE_PULSE){
            Serial.println(F("[CẢNH BÁO] Lệnh số chỉ dùng ở MODE=PULSE. Dùng 'mode pulse' hoặc 'V <rpm>' ở MODE=SPEED."));
          } else {
            if(!armed) Serial.println(F("[AN TOÀN] Chưa ARM: lệnh được lưu, ARM để chạy."));
            cmdOpenLoop[0] = (int)val;
            cmdOpenLoop[1] = (int)val;
            applyDutyFromCmd_modeaware(0, cmdOpenLoop[0]);
            applyDutyFromCmd_modeaware(1, cmdOpenLoop[1]);
            Serial.printf("[OK] cmd=%ld (M0 & M1)\n", val);
          }
        } else {
          Serial.println(F("[LỖI] Lệnh không hợp lệ"));
        }
      }
    } else rx += c;
  }
}
