/* =======================================================================
   GeoNetra — flight-controller sensor firmware, ESP32-S3-PRO (Edgehax)
   ONE sketch, BOTH aircraft. Set AIRCRAFT below and flash.
   -----------------------------------------------------------------------
       #define AIRCRAFT  AC_GAS     -> MQ-4 + MQ-7 fitted, no LiDAR
       #define AIRCRAFT  AC_LIDAR   -> YDLIDAR X2 fitted, no gas

   The vero board is IDENTICAL for both. Only the modules you plug in
   differ, which is why the pin map below has no conflicts between them:
   the gas aircraft leaves GPIO18 empty, the LiDAR aircraft leaves GPIO1
   and GPIO2 empty.

   THIS IS NOT THE FLIGHT BUILD. No PID, no mixer, no arming, no takeoff.
   MOTORTEST spins each motor for 400 ms. PROPELLERS OFF. ALWAYS.

   Board:  ESP32S3 Dev Module      Arduino-ESP32 core 3.x
   Libs:   VL53L0X by Pololu. Everything else is inline.
   ======================================================================= */

#define AC_GAS    1
#define AC_LIDAR  2

#define AIRCRAFT  AC_GAS          // <<<<<< CHANGE THIS, THEN FLASH

/* ================= 1. PIN MAP — ESP32-S3-PRO ===========================

   This board breaks out:
     left  : G TX RX IO1 IO2 IO42 IO41 IO40 IO39 IO38 IO37 IO36 IO35
             IO0 IO45 IO48 IO47 IO21 IO20 IO19 G G
     right : RST 3V3 3V3 IO4 IO5 IO6 IO7 IO15 IO16 IO17 IO18 IO8 IO3
             IO46 IO13 IO14

   Which means, importantly:
     * GPIO9 and GPIO10 DO NOT EXIST on the header. SCL and the battery
       divider have moved off them.
     * ADC1 is GPIO1-GPIO10, so of the pins that exist, only 1-8 can read
       an analog voltage. All three analog inputs live in that range.
     * GPIO35-42 may be tied to PSRAM or the microSD slot depending on the
       module fitted. Nothing here uses them.
     * GPIO0, GPIO3, GPIO45, GPIO46 are strapping pins. Nothing here uses
       them either.
     * GPIO19/GPIO20 are native USB. Left alone so the USB port keeps
       working.
   ======================================================================= */

#define PIN_SDA      15         // GY-87 SDA + VL53L0X SDA
#define PIN_SCL      16         // GY-87 SCL + VL53L0X SCL
#define PIN_MQ4       1         // ADC1_CH0  via 10k/15k divider   [gas]
#define PIN_MQ7       2         // ADC1_CH1  via 10k/15k divider   [gas]
#define PIN_VBAT      8         // ADC1_CH7  via 100k/27k divider
#define PIN_LIDAR    18         // UART2 RX  <- X2 TX            [lidar]
#define PIN_LINK_TX  17         // UART1 TX  -> C5 GPIO4 (RX)
#define PIN_LINK_RX  21         // UART1 RX  <- C5 GPIO5 (TX)
#define PIN_BTN      13         // momentary to GND
#define PIN_BUZZ     14         // active buzzer +
#define PIN_LED      47         // LED + 330R to GND
#define PIN_ESC1      4
#define PIN_ESC2      5
#define PIN_ESC3      6
#define PIN_ESC4      7

/* ================= 2. CONFIGURATION ==================================== */

#define CALIBRATE_ESCS   0      // set 1, flash, run once, set back to 0
#define HAVE_ESCS        1

#if AIRCRAFT == AC_GAS
  #define HAVE_GAS    1
  #define HAVE_LIDAR  0
  #define AC_NAME     "gas"
#else
  #define HAVE_GAS    0
  #define HAVE_LIDAR  1
  #define AC_NAME     "lidar"
#endif

#define VBAT_CAL     4.7037f    // (100k+27k)/27k  -> MEASURE yours, edit this
#define MQ_DIV_RATIO 1.6667f    // 25/15, recovers true 0-5 V from the divider
#define MQ_VC        5.0f
#define MQ_RL_K      10.0f
#define PREHEAT_S    90         // DEMO value. Real gas work needs 20 MINUTES.

#define MQ4_A  1012.7f
#define MQ4_B  -2.786f
#define MQ7_A    99.042f
#define MQ7_B    -1.518f

#define RATE_IMU_HZ   200
#define RATE_TEL_HZ    10
#define RATE_GAS_HZ    10
#define RATE_LID_HZ     5
#define RATE_SCAN_HZ    2

#define ESC_FREQ      400
#define ESC_RES_BITS   14
#define ESC_MIN_US   1000
#define ESC_MAX_US   2000
#define ESC_TEST_US  1150
#define ESC_TEST_MS   400

#define ADDR_MPU   0x68
#define ADDR_BMP   0x77
#define ADDR_HMC   0x1E
#define ADDR_QMC   0x0D

#define LINK_BAUD  921600

#include <Wire.h>
#include <VL53L0X.h>

/* ================= 3. STATE ============================================ */

void gnI2cScan();
void handleCommand(String c);
void sendEvt(const char *kind, const char *msg, int sev);

HardwareSerial LinkPort(1);
#if HAVE_LIDAR
HardwareSerial LidarPort(2);
#endif
VL53L0X tof;

struct { float roll=0,pitch=0,yaw=0, gx=0,gy=0,gz=0, ax=0,ay=0,az=0,
         gxOff=0,gyOff=0,gzOff=0; bool ok=false; int failStreak=0; } imu;
struct { bool present=false, isQmc=false; int16_t mx=0,my=0,mz=0; float heading=0; } mag;
struct { bool present=false; float tempC=0,pressPa=0,altM=0,alt0=0;
         int16_t ac1,ac2,ac3,b1,b2,mb,mc,md; uint16_t ac4,ac5,ac6; } baro;
struct { bool present=false; float m=0; bool valid=false; } rng;
struct { float v4=0,v7=0,rs4=0,rs7=0,r04=0,r07=0,ch4=0,co=0;
         bool cal=false; uint8_t warn=0; } gas;
struct { bool present=false; uint16_t bin[360]; uint32_t lastPacket=0,packets=0;
         float sector[12], nearest=99.0f; int nearestBearing=0; } lidar;

float    vbat = 0;
uint32_t bootMs = 0, seq = 0;
bool     estop = false, preheating = true;
uint32_t preheatEndMs = 0;

/* ================= 4. I2C HELPERS ====================================== */

bool i2cWrite8(uint8_t a, uint8_t r, uint8_t v){
  Wire.beginTransmission(a); Wire.write(r); Wire.write(v);
  return Wire.endTransmission()==0;
}
bool i2cRead(uint8_t a, uint8_t r, uint8_t *b, uint8_t n){
  Wire.beginTransmission(a); Wire.write(r);
  if (Wire.endTransmission(false)) return false;
  if (Wire.requestFrom((int)a,(int)n)!=n) return false;
  for (uint8_t i=0;i<n;i++) b[i]=Wire.read();
  return true;
}
bool i2cPing(uint8_t a){ Wire.beginTransmission(a); return Wire.endTransmission()==0; }

/* ================= 5. MPU6050 ========================================== */

bool mpuBegin(){
  if (!i2cPing(ADDR_MPU)) return false;
  i2cWrite8(ADDR_MPU,0x6B,0x80); delay(100);
  i2cWrite8(ADDR_MPU,0x6B,0x01); delay(20);
  i2cWrite8(ADDR_MPU,0x1A,0x03);
  i2cWrite8(ADDR_MPU,0x1B,0x08);
  i2cWrite8(ADDR_MPU,0x1C,0x10);
  i2cWrite8(ADDR_MPU,0x19,0x04);
  /* GY-87 trap: the compass hangs off the MPU's AUXILIARY bus. Until these
     two writes run, an I2C scan shows no magnetometer at all. */
  i2cWrite8(ADDR_MPU,0x6A,0x00);   // USER_CTRL    I2C_MST_EN = 0
  i2cWrite8(ADDR_MPU,0x37,0x02);   // INT_PIN_CFG  I2C_BYPASS_EN = 1
  delay(20);
  return true;
}

bool mpuRead(){
  uint8_t b[14];
  if (!i2cRead(ADDR_MPU,0x3B,b,14)) { imu.failStreak++; return false; }
  imu.failStreak = 0;
  imu.ax = (int16_t)(b[0]<<8|b[1])  / 4096.0f;
  imu.ay = (int16_t)(b[2]<<8|b[3])  / 4096.0f;
  imu.az = (int16_t)(b[4]<<8|b[5])  / 4096.0f;
  imu.gx = (int16_t)(b[8]<<8|b[9])  / 65.5f - imu.gxOff;
  imu.gy = (int16_t)(b[10]<<8|b[11])/ 65.5f - imu.gyOff;
  imu.gz = (int16_t)(b[12]<<8|b[13])/ 65.5f - imu.gzOff;
  return true;
}

void gyroCalibrate(){
  Serial.println(F("[gyro] hold still, 3 s..."));
  digitalWrite(PIN_LED,HIGH);
  imu.gxOff=imu.gyOff=imu.gzOff=0;
  double sx=0,sy=0,sz=0; int n=0; uint32_t t0=millis();
  while (millis()-t0 < 3000){ if (mpuRead()){ sx+=imu.gx; sy+=imu.gy; sz+=imu.gz; n++; } delay(5); }
  if (n>100){ imu.gxOff=sx/n; imu.gyOff=sy/n; imu.gzOff=sz/n; }
  digitalWrite(PIN_LED,LOW);
  Serial.printf("[gyro] offsets %.2f %.2f %.2f (n=%d)\n", imu.gxOff,imu.gyOff,imu.gzOff,n);
}

void attitudeUpdate(float dt){
  float ar = atan2f(imu.ay, imu.az) * 57.29578f;
  float ap = atan2f(-imu.ax, sqrtf(imu.ay*imu.ay + imu.az*imu.az)) * 57.29578f;
  const float a = 0.98f;
  imu.roll  = a*(imu.roll  + imu.gx*dt) + (1-a)*ar;
  imu.pitch = a*(imu.pitch + imu.gy*dt) + (1-a)*ap;
  imu.yaw  += imu.gz*dt;
  if (imu.yaw>=360) imu.yaw-=360;
  if (imu.yaw<0)    imu.yaw+=360;
}

/* ================= 6. MAGNETOMETER ===================================== */

void magBegin(){
  uint8_t id[3];
  if (i2cPing(ADDR_HMC) && i2cRead(ADDR_HMC,0x0A,id,3) &&
      id[0]=='H'&&id[1]=='4'&&id[2]=='3'){
    mag.present=true; mag.isQmc=false;
    i2cWrite8(ADDR_HMC,0x00,0x70); i2cWrite8(ADDR_HMC,0x01,0x20); i2cWrite8(ADDR_HMC,0x02,0x00);
    Serial.println(F("[mag] HMC5883L @0x1E")); return;
  }
  if (i2cPing(ADDR_QMC)){
    mag.present=true; mag.isQmc=true;
    i2cWrite8(ADDR_QMC,0x0B,0x01); i2cWrite8(ADDR_QMC,0x09,0x1D);
    Serial.println(F("[mag] QMC5883L @0x0D (normal - most GY-87s are QMC)")); return;
  }
  Serial.println(F("[mag] NOT FOUND - did mpuBegin() run first?"));
}

void magRead(){
  if (!mag.present) return;
  uint8_t b[6];
  if (mag.isQmc){
    if (!i2cRead(ADDR_QMC,0x00,b,6)) return;
    mag.mx=(int16_t)(b[1]<<8|b[0]); mag.my=(int16_t)(b[3]<<8|b[2]); mag.mz=(int16_t)(b[5]<<8|b[4]);
  } else {
    if (!i2cRead(ADDR_HMC,0x03,b,6)) return;
    mag.mx=(int16_t)(b[0]<<8|b[1]); mag.mz=(int16_t)(b[2]<<8|b[3]); mag.my=(int16_t)(b[4]<<8|b[5]);
  }
  float h = atan2f((float)mag.my,(float)mag.mx)*57.29578f;
  if (h<0) h+=360;
  mag.heading=h;
}

/* ================= 7. BMP180 =========================================== */

int16_t  bmpS16(uint8_t r){ uint8_t b[2]; if(!i2cRead(ADDR_BMP,r,b,2))return 0; return (int16_t)(b[0]<<8|b[1]); }
uint16_t bmpU16(uint8_t r){ uint8_t b[2]; if(!i2cRead(ADDR_BMP,r,b,2))return 0; return (uint16_t)(b[0]<<8|b[1]); }

bool bmpBegin(){
  uint8_t id;
  if (!i2cRead(ADDR_BMP,0xD0,&id,1) || id!=0x55) return false;
  baro.ac1=bmpS16(0xAA); baro.ac2=bmpS16(0xAC); baro.ac3=bmpS16(0xAE);
  baro.ac4=bmpU16(0xB0); baro.ac5=bmpU16(0xB2); baro.ac6=bmpU16(0xB4);
  baro.b1 =bmpS16(0xB6); baro.b2 =bmpS16(0xB8);
  baro.mb =bmpS16(0xBA); baro.mc =bmpS16(0xBC); baro.md=bmpS16(0xBE);
  baro.present=true; return true;
}

void bmpRead(){
  if (!baro.present) return;
  i2cWrite8(ADDR_BMP,0xF4,0x2E); delay(5);
  int32_t ut = bmpU16(0xF6);
  i2cWrite8(ADDR_BMP,0xF4,0x74); delay(14);
  uint8_t b[3]; i2cRead(ADDR_BMP,0xF6,b,3);
  int32_t up = ((int32_t)b[0]<<16 | (int32_t)b[1]<<8 | b[2]) >> 7;

  int32_t x1 = ((ut-(int32_t)baro.ac6)*(int32_t)baro.ac5)>>15;
  int32_t x2 = ((int32_t)baro.mc<<11)/(x1+baro.md);
  int32_t b5 = x1+x2;
  baro.tempC = ((b5+8)>>4)/10.0f;

  int32_t b6=b5-4000;
  x1=(baro.b2*((b6*b6)>>12))>>11;
  x2=(baro.ac2*b6)>>11;
  int32_t x3=x1+x2;
  int32_t b3=((((int32_t)baro.ac1*4+x3)<<1)+2)>>2;
  x1=(baro.ac3*b6)>>13;
  x2=(baro.b1*((b6*b6)>>12))>>16;
  x3=((x1+x2)+2)>>2;
  uint32_t b4=(baro.ac4*(uint32_t)(x3+32768))>>15;
  uint32_t b7=((uint32_t)up-b3)*25000;
  int32_t  p =(b7<0x80000000)?(b7*2)/b4:(b7/b4)*2;
  x1=(p>>8)*(p>>8); x1=(x1*3038)>>16;
  x2=(-7357*p)>>16;
  p += (x1+x2+3791)>>4;

  baro.pressPa=p;
  baro.altM = 44330.0f*(1.0f-powf(p/101325.0f,0.1903f));
  if (baro.alt0==0) baro.alt0=baro.altM;
}

/* ================= 8. GAS ============================================== */

float adcVolts(int pin){
  uint32_t acc=0;
  for (int i=0;i<16;i++) acc += analogReadMilliVolts(pin);
  return (acc/16.0f)/1000.0f;
}

void gasRead(){
#if HAVE_GAS
  gas.v4 = adcVolts(PIN_MQ4)*MQ_DIV_RATIO;
  gas.v7 = adcVolts(PIN_MQ7)*MQ_DIV_RATIO;
  if (gas.v4>0.02f) gas.rs4 = MQ_RL_K*(MQ_VC-gas.v4)/gas.v4;
  if (gas.v7>0.02f) gas.rs7 = MQ_RL_K*(MQ_VC-gas.v7)/gas.v7;

  if (preheating && millis()>preheatEndMs){
    preheating=false;
    gas.r04 = gas.rs4/4.4f;     // MQ-4 clean-air ratio
    gas.r07 = gas.rs7/27.0f;    // MQ-7 clean-air ratio
    gas.cal = true;
    Serial.printf("[gas] R0 captured MQ4=%.2fk MQ7=%.2fk\n", gas.r04, gas.r07);
    tone(PIN_BUZZ,2000,120);
    sendEvt("GAS_CAL","R0 captured from clean air",0);
  }
  if (gas.cal && gas.r04>0.01f && gas.r07>0.01f){
    gas.ch4 = MQ4_A*powf(gas.rs4/gas.r04, MQ4_B);
    gas.co  = MQ7_A*powf(gas.rs7/gas.r07, MQ7_B);
  }
  gas.warn = 0;
  if (gas.ch4>5000  || gas.co>35)  gas.warn=1;
  if (gas.ch4>10000 || gas.co>200) gas.warn=2;
#endif
}

/* ================= 9. YDLIDAR X2 ======================================= */
/* AA 55 CT LSN FSA(2) LSA(2) CS(2) then LSN*2 little-endian samples.
   angle_deg = (FSA>>1)/64      distance_mm = sample/4                    */

void lidarPoll(){
#if HAVE_LIDAR
  static uint8_t buf[200];
  static int idx=0, need=0;
  while (LidarPort.available()){
    uint8_t c = LidarPort.read();
    if (idx==0){ if (c==0xAA) buf[idx++]=c; continue; }
    if (idx==1){ if (c==0x55) buf[idx++]=c; else idx=0; continue; }
    buf[idx++]=c;

    if (idx==10){
      uint8_t lsn=buf[3];
      if (!lsn || lsn>90){ idx=0; continue; }
      need = 10 + lsn*2;
    }
    if (need && idx>=need){
      uint8_t  lsn=buf[3];
      uint16_t fsa=buf[4]|(buf[5]<<8), lsa=buf[6]|(buf[7]<<8);
      float a0=(fsa>>1)/64.0f, a1=(lsa>>1)/64.0f;
      float span=a1-a0; if (span<0) span+=360.0f;
      float step=(lsn>1)?span/(lsn-1):0;
      for (uint8_t i=0;i<lsn;i++){
        uint16_t mm = (buf[10+i*2] | (buf[11+i*2]<<8)) / 4;
        if (mm<60 || mm>8000) continue;
        float ang=a0+step*i;
        while (ang>=360) ang-=360;
        while (ang<0)    ang+=360;
        lidar.bin[(int)ang]=mm;
      }
      lidar.packets++; lidar.lastPacket=millis(); lidar.present=true;
      idx=0; need=0;
    }
    if (idx>=(int)sizeof(buf)){ idx=0; need=0; }
  }
  if (lidar.present && millis()-lidar.lastPacket>1500) lidar.present=false;
#endif
}

void lidarSectors(){
  for (int s=0;s<12;s++) lidar.sector[s]=99.0f;
  lidar.nearest=99.0f; lidar.nearestBearing=0;
  for (int a=0;a<360;a++){
    if (!lidar.bin[a]) continue;
    float m = lidar.bin[a]/1000.0f;
    int s = a/30;
    if (m<lidar.sector[s]) lidar.sector[s]=m;
    if (m<lidar.nearest){ lidar.nearest=m; lidar.nearestBearing=a; }
  }
}

/* ================= 10. ESCs (test only) ================================ */

#if HAVE_ESCS
const int escPins[4] = { PIN_ESC1, PIN_ESC2, PIN_ESC3, PIN_ESC4 };
uint32_t usToDuty(int us){
  const float period = 1000000.0f/ESC_FREQ;
  return (uint32_t)((us/period)*((1<<ESC_RES_BITS)-1));
}
void escWrite(int i,int us){ ledcWrite(escPins[i], usToDuty(us)); }
void escAll(int us){ for(int i=0;i<4;i++) escWrite(i,us); }
void escBegin(){
  for (int i=0;i<4;i++){ ledcAttach(escPins[i],ESC_FREQ,ESC_RES_BITS); escWrite(i,ESC_MIN_US); }
}
void escCalibrate(){
  Serial.println(F("\n*** ESC CALIBRATION - PROPELLERS OFF ***"));
  Serial.println(F("Battery DISCONNECTED. Full throttle is on the signal now."));
  escAll(ESC_MAX_US);
  Serial.println(F("Connect the battery NOW. Wait for two beeps."));
  delay(8000);
  Serial.println(F("Dropping to minimum..."));
  escAll(ESC_MIN_US); delay(5000);
  Serial.println(F("Done. Set CALIBRATE_ESCS back to 0 and reflash.\n"));
}
void motorTest(){
  if (estop){ Serial.println(F("[motor] blocked: ESTOP latched")); return; }
  Serial.println(F("[motor] TEST - PROPS MUST BE OFF. 3..."));
  for (int i=3;i>0;i--){ tone(PIN_BUZZ,1500,120); Serial.printf("%d\n",i); delay(1000); }
  const char *nm[4]={"M1 rear-right","M2 front-right","M3 rear-left","M4 front-left"};
  for (int i=0;i<4;i++){
    Serial.printf("[motor] %s\n",nm[i]);
    escWrite(i,ESC_TEST_US); delay(ESC_TEST_MS);
    escWrite(i,ESC_MIN_US);  delay(500);
  }
  Serial.println(F("[motor] test complete"));
}
#endif

/* ================= 11. JSON OUT ======================================== */

void emit(const char *j){ LinkPort.println(j); }

static const char B64[]="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

void scanBase64(char *out){
  uint8_t raw[360];
  for (int a=0;a<360;a++){
    uint16_t mm=lidar.bin[a];
    raw[a] = (!mm) ? 255 : (uint8_t)min(254, mm/32);   // 32 mm per count
  }
  int o=0;
  for (int i=0;i<360;i+=3){
    uint32_t v=(raw[i]<<16)|(raw[i+1]<<8)|raw[i+2];
    out[o++]=B64[(v>>18)&63]; out[o++]=B64[(v>>12)&63];
    out[o++]=B64[(v>>6)&63];  out[o++]=B64[v&63];
  }
  out[o]=0;
}

void sendTel(){
  char j[440];
  const char *st = estop?"ESTOP":(preheating?"PREHEAT":"READY");
  int preLeft = preheating ? (int)((preheatEndMs-millis())/1000) : 0;
  snprintf(j,sizeof(j),
    "{\"t\":\"tel\",\"id\":\"%s\",\"seq\":%lu,\"ms\":%lu,\"st\":\"%s\","
    "\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.1f,"
    "\"gx\":%.1f,\"gy\":%.1f,\"gz\":%.1f,\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,"
    "\"alt\":%.3f,\"altOk\":%d,\"baroAlt\":%.2f,\"tempC\":%.1f,\"presPa\":%.0f,"
    "\"hdg\":%.1f,\"magOk\":%d,\"vbat\":%.2f,\"pre\":%d,\"imuOk\":%d}",
    AC_NAME,(unsigned long)seq,(unsigned long)(millis()-bootMs),st,
    imu.roll,imu.pitch,imu.yaw, imu.gx,imu.gy,imu.gz, imu.ax,imu.ay,imu.az,
    rng.m, rng.valid?1:0, baro.altM-baro.alt0, baro.tempC, baro.pressPa,
    mag.heading, mag.present?1:0, vbat, preLeft, imu.ok?1:0);
  emit(j);
}

void sendGas(){
#if HAVE_GAS
  char j[300];
  snprintf(j,sizeof(j),
    "{\"t\":\"gas\",\"id\":\"%s\",\"v4\":%.3f,\"v7\":%.3f,"
    "\"rs4\":%.2f,\"rs7\":%.2f,\"r04\":%.2f,\"r07\":%.2f,"
    "\"ch4\":%.0f,\"co\":%.1f,\"warn\":%d,\"cal\":%s}",
    AC_NAME,gas.v4,gas.v7,gas.rs4,gas.rs7,gas.r04,gas.r07,
    gas.ch4,gas.co,gas.warn,gas.cal?"true":"false");
  emit(j);
#endif
}

void sendLid(){
#if HAVE_LIDAR
  lidarSectors();
  char j[330]; int o=0;
  o += snprintf(j+o,sizeof(j)-o,
       "{\"t\":\"lid\",\"id\":\"%s\",\"ok\":%d,\"near\":%.2f,\"nb\":%d,\"s\":[",
       AC_NAME, lidar.present?1:0, lidar.nearest, lidar.nearestBearing);
  for (int s=0;s<12;s++)
    o += snprintf(j+o,sizeof(j)-o,"%s%.2f", s?",":"",
                  lidar.sector[s]>90?0.0f:lidar.sector[s]);
  snprintf(j+o,sizeof(j)-o,"],\"pkts\":%lu}",(unsigned long)lidar.packets);
  emit(j);
#endif
}

void sendScan(){
#if HAVE_LIDAR
  static char b64[500], j[660];
  scanBase64(b64);
  snprintf(j,sizeof(j),
    "{\"t\":\"scan\",\"id\":\"%s\",\"res\":1,\"unit\":32,"
    "\"z\":%.3f,\"yaw\":%.1f,\"roll\":%.1f,\"pitch\":%.1f,\"st\":\"BENCH\",\"d\":\"%s\"}",
    AC_NAME, rng.m, imu.yaw, imu.roll, imu.pitch, b64);
  emit(j);
  for (int a=0;a<360;a++) lidar.bin[a]=0;   // fresh sweep each time
#endif
}

void sendEvt(const char *kind, const char *msg, int sev){
  char j[220];
  snprintf(j,sizeof(j),
    "{\"t\":\"evt\",\"id\":\"%s\",\"kind\":\"%s\",\"sev\":%d,\"msg\":\"%s\"}",
    AC_NAME,kind,sev,msg);
  emit(j);
}

/* ================= 12. COMMANDS ======================================== */

void handleCommand(String c){
  c.trim(); c.toUpperCase();
  if (!c.length()) return;

  /* Commands may be addressed:  GAS:MOTORTEST  /  LIDAR:CALGYRO  /  ALL:ESTOP
     An unaddressed command is accepted by both aircraft. */
  int colon = c.indexOf(':');
  if (colon > 0){
    String who = c.substring(0,colon);
    if (who != "ALL" && !who.equalsIgnoreCase(AC_NAME)) return;
    c = c.substring(colon+1);
  }
  Serial.printf("[cmd] %s\n", c.c_str());

  if (c=="MOTORTEST"){
#if HAVE_ESCS
    motorTest(); sendEvt("MOTORTEST","motor test complete",0);
#else
    sendEvt("MOTORTEST","ESCs not compiled in",1);
#endif
  }
  else if (c=="CALGYRO")     { gyroCalibrate(); sendEvt("CALGYRO","gyro zeroed",0); }
  else if (c=="ZEROALT")     { baro.alt0=baro.altM; sendEvt("ZEROALT","baro datum set",0); }
  else if (c=="ESTOP")       { estop=true;
#if HAVE_ESCS
                               escAll(ESC_MIN_US);
#endif
                               tone(PIN_BUZZ,3000,600);
                               sendEvt("ESTOP","emergency stop latched",2); }
  else if (c=="CLEAR")       { estop=false; sendEvt("CLEAR","estop cleared",0); }
  else if (c=="SKIPPREHEAT") { preheatEndMs=millis();
                               sendEvt("GAS_UNCAL","preheat skipped - ppm not trustworthy",1); }
  else if (c=="BUZZ")        { tone(PIN_BUZZ,2200,300); }
  else if (c=="SCAN")        { gnI2cScan(); }
  else                       { sendEvt("CMD","unknown command",1); }
}

void gnI2cScan(){
  Serial.println(F("\n--- I2C scan ---"));
  int n=0;
  for (uint8_t a=1;a<127;a++){
    if (i2cPing(a)){
      Serial.printf("  0x%02X",a);
      switch(a){
        case 0x68: Serial.print(F("  MPU6050"));  break;
        case 0x77: Serial.print(F("  BMP180"));   break;
        case 0x1E: Serial.print(F("  HMC5883L")); break;
        case 0x0D: Serial.print(F("  QMC5883L")); break;
        case 0x29: Serial.print(F("  VL53L0X"));  break;
      }
      Serial.println(); n++;
    }
  }
  if (!n) Serial.println(F("  nothing. check SDA=GPIO15 / SCL=GPIO16 / 3V3 / GND."));
  Serial.println(F("----------------\n"));
}

/* ================= 13. SETUP / LOOP ==================================== */

void setup(){
  Serial.begin(115200);
  delay(400);
  Serial.printf("\n=== GeoNetra %s aircraft - ESP32-S3-PRO ===\n", AC_NAME);

  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_BUZZ,OUTPUT);
  pinMode(PIN_BTN,INPUT_PULLUP);

  LinkPort.begin(LINK_BAUD, SERIAL_8N1, PIN_LINK_RX, PIN_LINK_TX);
#if HAVE_LIDAR
  LidarPort.begin(115200, SERIAL_8N1, PIN_LIDAR, -1);
#endif

  analogSetPinAttenuation(PIN_VBAT, ADC_11db);
#if HAVE_GAS
  analogSetPinAttenuation(PIN_MQ4, ADC_11db);
  analogSetPinAttenuation(PIN_MQ7, ADC_11db);
#endif

#if HAVE_ESCS
  escBegin();
  #if CALIBRATE_ESCS
    escCalibrate();
    while (true) delay(1000);
  #endif
#endif

  Wire.begin(PIN_SDA, PIN_SCL, 400000);

  /* ORDER MATTERS. mpuBegin() opens the aux-bus bypass; without it the
     magnetometer is invisible. Do not reorder these three lines. */
  imu.ok = mpuBegin();
  Serial.printf("[imu]  MPU6050 %s\n", imu.ok?"ok":"MISSING");
  magBegin();
  Serial.printf("[baro] BMP180  %s\n", bmpBegin()?"ok":"MISSING");

  tof.setBus(&Wire);
  tof.setTimeout(200);
  if (tof.init()){
    tof.setMeasurementTimingBudget(33000);
    tof.startContinuous(30);
    rng.present=true;
    Serial.println(F("[tof]  VL53L0X ok"));
  } else Serial.println(F("[tof]  VL53L0X MISSING"));

  gnI2cScan();
  if (imu.ok) gyroCalibrate();
  bmpRead(); baro.alt0 = baro.altM;

#if HAVE_GAS
  preheatEndMs = millis() + (uint32_t)PREHEAT_S*1000UL;
  Serial.printf("[gas] preheating %d s, R0 captured at the end.\n", PREHEAT_S);
  Serial.println(F("[gas] DEMO VALUE. Real gas work needs the full 20 minutes."));
#else
  preheating = false;
#endif

  tone(PIN_BUZZ,1800,100); delay(150); tone(PIN_BUZZ,2400,100);
  Serial.println(F("=== running. MOTORTEST CALGYRO ZEROALT ESTOP CLEAR SKIPPREHEAT BUZZ SCAN ==="));
  bootMs = millis();
}

void loop(){
  static uint32_t tImu=0,tTel=0,tGas=0,tLid=0,tScan=0,tBaro=0,tMag=0,tLed=0;
  static uint32_t lastImuUs=0;
  uint32_t now = millis();

  if (now-tImu >= 1000/RATE_IMU_HZ){
    tImu=now;
    uint32_t us=micros();
    float dt = lastImuUs ? (us-lastImuUs)/1e6f : 0.005f;
    lastImuUs=us;
    if (mpuRead()) attitudeUpdate(dt);
    if (imu.failStreak>20 && imu.ok){
      imu.ok=false; sendEvt("IMU_FAIL","consecutive I2C failures on the MPU",2);
    }
  }

  lidarPoll();

  if (rng.present){
    uint16_t mm = tof.readRangeContinuousMillimeters();
    if (!tof.timeoutOccurred() && mm>0 && mm<2000){ rng.m=mm/1000.0f; rng.valid=true; }
    else rng.valid=false;
  }

  if (now-tBaro >= 500){ tBaro=now; bmpRead(); }
  if (now-tMag  >= 100){ tMag=now;  magRead(); }

  vbat = adcVolts(PIN_VBAT)*VBAT_CAL;

  if (digitalRead(PIN_BTN)==LOW && !estop) handleCommand("ESTOP");

  if (now-tTel  >= 1000/RATE_TEL_HZ ){ tTel=now; seq++; sendTel(); }
  if (now-tGas  >= 1000/RATE_GAS_HZ ){ tGas=now; gasRead(); sendGas(); }
  if (now-tLid  >= 1000/RATE_LID_HZ ){ tLid=now; sendLid(); }
  if (now-tScan >= 1000/RATE_SCAN_HZ){ tScan=now; sendScan(); }

  if (estop) digitalWrite(PIN_LED,HIGH);
  else if (now-tLed >= (preheating?150:800)){ tLed=now; digitalWrite(PIN_LED,!digitalRead(PIN_LED)); }

  static String lineLink, lineUsb;
  while (LinkPort.available()){
    char c=LinkPort.read();
    if (c=='\n'){ handleCommand(lineLink); lineLink=""; }
    else if (c!='\r' && lineLink.length()<64) lineLink+=c;
  }
  while (Serial.available()){
    char c=Serial.read();
    if (c=='\n'){ handleCommand(lineUsb); lineUsb=""; }
    else if (c!='\r' && lineUsb.length()<64) lineUsb+=c;
  }
}
