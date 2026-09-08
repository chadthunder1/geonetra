/* =======================================================================
   GeoNetra — 5 GHz link board, ESP32-C5-MINI_V1.0
   ONE sketch, BOTH aircraft AND both network topologies.
   -----------------------------------------------------------------------
   Set DRONE_ID and LINK_ROLE below, then flash.

   TOPOLOGY A — no router (default, works out of the box)
       gas C5    : LINK_ROLE = ROLE_AP     creates GEONETRA_5G on 5 GHz
       lidar C5  : LINK_ROLE = ROLE_STA    joins it
       Pi 5      : joins it as a station

   TOPOLOGY B — Pi 5 runs hostapd (more robust with two aircraft)
       both C5s  : LINK_ROLE = ROLE_STA
       Pi 5      : is the access point

   Topology A puts the whole network on a board that is also flying. It
   works, and it needs nothing you do not already own. If you have time
   tonight, B is the one that will not surprise you: the Pi has a real
   antenna, mains power, and does not reboot when you unplug a drone.

   Board:  ESP32C5 Dev Module
   Core:   Arduino-ESP32 3.3.0 or newer. Older cores have no C5 support and
           no setBandMode() -- you get 2.4 GHz silently, or a compile error.
   ======================================================================= */

#define ROLE_AP   1
#define ROLE_STA  2

#define DRONE_ID   "gas"        // <<<<<< "gas"  or  "lidar"
#define LINK_ROLE  ROLE_AP      // <<<<<< see the table above

#define AP_SSID     "GEONETRA_5G"
#define AP_PASS     "geonetra2026"     // >= 8 characters
#define AP_CHANNEL  36                 // 5 GHz: 36 / 40 / 44 / 48
#define AP_MAXCLIENT 4

#define PORT_TELEM   9000              // C5 -> Pi
#define PORT_CMD     9001              // Pi -> C5

/* ---- PIN MAP — ESP32-C5-MINI_V1.0 --------------------------------------
   This board breaks out:
     left  : IO15 TXD RXD IO10 IO9 IO8 IO7 IO6 IO5
     right : IO0 IO1 IO2 IO3 IO4 RST 3V3 GND 5V

   There is no GPIO23 or GPIO24 on this module, so the S3 link has moved
   to IO4 / IO5. IO2 and IO7 are left alone -- they are strapping pins.
   TXD / RXD belong to the USB-serial console; do not reuse them.        */

#define PIN_LINK_RX   4         // <- S3 GPIO17 (TX)
#define PIN_LINK_TX   5         // -> S3 GPIO21 (RX)
#define PIN_LED      -1         // set to a real pin if your board has an LED

#define LINK_BAUD 921600

#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_wifi.h>

HardwareSerial S3(1);
WiFiUDP udpOut, udpCmd;

IPAddress piIp;
bool      piKnown = false;

uint32_t txCount=0, rxCount=0, dropCount=0;
uint32_t pingSentUs=0, lastRttUs=0;
bool     pingPending=false;

/* ================= HELPERS ============================================= */

IPAddress fallbackTarget(){
#if LINK_ROLE == ROLE_AP
  return IPAddress(192,168,4,255);          // our own subnet broadcast
#else
  IPAddress ip = WiFi.localIP(), mask = WiFi.subnetMask();
  if (!ip) return IPAddress(255,255,255,255);
  IPAddress bc;
  for (int i=0;i<4;i++) bc[i] = ip[i] | (~mask[i] & 0xFF);
  return bc;
#endif
}

void sendUdp(const char *s, size_t n){
  IPAddress dst = piKnown ? piIp : fallbackTarget();
  if (!udpOut.beginPacket(dst, PORT_TELEM)) { dropCount++; return; }
  udpOut.write((const uint8_t*)s, n);
  if (udpOut.endPacket()) txCount++; else dropCount++;
}

uint8_t currentChannel(){
  uint8_t pri; wifi_second_chan_t sec;
  esp_wifi_get_channel(&pri,&sec);
  return pri;
}
const char *bandName(){ return currentChannel() >= 15 ? "5GHz" : "2.4GHz"; }

/* Software RTT expressed as a distance, exactly as if it were a time of
   flight. It is wrong by four orders of magnitude, and that is the point:
   next to a hardware measurement it shows why FTM exists. */
float rttToKm(uint32_t us){ return (299.792458f*us)/2000.0f; }

void sendLink(){
  char j[380];
  snprintf(j,sizeof(j),
    "{\"t\":\"link\",\"id\":\"%s\",\"role\":\"%s\",\"ap\":\"%s\",\"ch\":%d,"
    "\"band\":\"%s\",\"rssi\":%d,\"ip\":\"%s\",\"pi\":\"%s\","
    "\"tx\":%lu,\"rx\":%lu,\"drop\":%lu,\"rttUs\":%lu,\"cDistKm\":%.1f,"
    "\"note\":\"software RTT is link health, not range\"}",
    DRONE_ID,
#if LINK_ROLE == ROLE_AP
    "ap",
#else
    "sta",
#endif
    AP_SSID, currentChannel(), bandName(),
#if LINK_ROLE == ROLE_AP
    (int)WiFi.softAPgetStationNum(),
    WiFi.softAPIP().toString().c_str(),
#else
    (int)WiFi.RSSI(),
    WiFi.localIP().toString().c_str(),
#endif
    piKnown ? piIp.toString().c_str() : "unknown",
    (unsigned long)txCount,(unsigned long)rxCount,(unsigned long)dropCount,
    (unsigned long)lastRttUs, rttToKm(lastRttUs));
  sendUdp(j, strlen(j));
}

void sendPing(){
  pingSentUs = micros();
  pingPending = true;
  char j[80];
  snprintf(j,sizeof(j),"{\"t\":\"ping\",\"id\":\"%s\",\"us\":%lu}",
           DRONE_ID,(unsigned long)pingSentUs);
  sendUdp(j, strlen(j));
}

/* ================= SETUP =============================================== */

void setup(){
  Serial.begin(115200);
  delay(400);
  Serial.printf("\n=== GeoNetra link board - %s aircraft - ESP32-C5 ===\n", DRONE_ID);

  if (PIN_LED >= 0) pinMode(PIN_LED, OUTPUT);
  S3.begin(LINK_BAUD, SERIAL_8N1, PIN_LINK_RX, PIN_LINK_TX);

  /* Force 5 GHz. Without this the C5 happily lands on 2.4 GHz and
     everything still "works" -- on the band the flight controllers and
     every phone in the room are already using. */
#if defined(SOC_WIFI_SUPPORT_5G)
  WiFi.setBandMode(WIFI_BAND_MODE_5G_ONLY);
#else
  #warning "This core does not expose 5 GHz band mode. Update to Arduino-ESP32 3.3.0+."
#endif

#if LINK_ROLE == ROLE_AP
  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL, 0, AP_MAXCLIENT);
  delay(500);
  Serial.printf("[ap] %s %s  channel %d  band %s  ip %s\n",
                AP_SSID, ok?"up":"FAILED", currentChannel(), bandName(),
                WiFi.softAPIP().toString().c_str());
#else
  WiFi.mode(WIFI_STA);
  WiFi.begin(AP_SSID, AP_PASS);
  Serial.printf("[sta] joining %s", AP_SSID);
  uint32_t t0 = millis();
  while (WiFi.status()!=WL_CONNECTED && millis()-t0 < 20000){ delay(300); Serial.print("."); }
  Serial.println();
  if (WiFi.status()==WL_CONNECTED)
    Serial.printf("[sta] joined  channel %d  band %s  rssi %d  ip %s\n",
                  currentChannel(), bandName(), WiFi.RSSI(),
                  WiFi.localIP().toString().c_str());
  else
    Serial.println(F("[sta] NOT CONNECTED - is the AP up? right SSID and band?"));
#endif

  if (currentChannel() < 15){
    Serial.println(F("*** WARNING: channel is below 15. That is 2.4 GHz."));
    Serial.println(F("*** The link is not on 5 GHz. Check your core version."));
  }

  udpOut.begin(PORT_TELEM+100);
  udpCmd.begin(PORT_CMD);
  Serial.printf("[udp] telemetry -> %d   commands <- %d\n", PORT_TELEM, PORT_CMD);
  Serial.println(F("[udp] broadcasting until the Pi says hello on 9001"));
}

/* ================= LOOP ================================================ */

void loop(){
  static uint32_t tLink=0, tPing=0, tLed=0;
  uint32_t now = millis();

  /* ---- UART1 in: one JSON line becomes one datagram ---- */
  static char line[900];
  static int  li=0;
  while (S3.available()){
    char c = S3.read();
    if (c=='\n'){ if (li>2) sendUdp(line,li); li=0; }
    else if (c!='\r'){
      if (li < (int)sizeof(line)-1) line[li++]=c;
      else li=0;                                  // overlong line, drop it
    }
  }

  /* ---- UDP in: commands and pongs from the Pi ---- */
  int sz = udpCmd.parsePacket();
  if (sz > 0){
    char buf[200];
    int n = udpCmd.read(buf, sizeof(buf)-1);
    if (n > 0){
      buf[n]=0; rxCount++;
      if (!piKnown || piIp != udpCmd.remoteIP()){
        piIp = udpCmd.remoteIP(); piKnown = true;
        Serial.printf("[udp] Pi is at %s - unicast from now on\n", piIp.toString().c_str());
      }
      if      (!strncmp(buf,"PONG",4)) { if (pingPending){ lastRttUs = micros()-pingSentUs; pingPending=false; } }
      else if (!strncmp(buf,"HELLO",5)){ Serial.println(F("[udp] hello from the Pi")); }
      else { S3.println(buf); Serial.printf("[cmd] -> S3: %s\n", buf); }
    }
  }

  if (now-tLink >= 1000){ tLink=now; sendLink(); }
  if (now-tPing >=  500){ tPing=now; if (piKnown) sendPing(); }

  if (PIN_LED >= 0){
#if LINK_ROLE == ROLE_AP
    uint32_t period = WiFi.softAPgetStationNum() ? 900 : 150;
#else
    uint32_t period = (WiFi.status()==WL_CONNECTED) ? 900 : 150;
#endif
    if (now-tLed >= period){ tLed=now; digitalWrite(PIN_LED, !digitalRead(PIN_LED)); }
  }
}
