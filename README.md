# GeoNetra — two aircraft, real boards

Built for the **Edgehax ESP32-S3-PRO** and the **ESP32-C5-MINI_V1.0**, not a
generic reference pinout. The differences that matter are in §1.

```
drone_fc_s3/drone_fc_s3.ino       ONE sketch, both aircraft (#define AIRCRAFT)
drone_link_c5/drone_link_c5.ino   ONE sketch, both link boards (#define DRONE_ID, LINK_ROLE)
groundstation/gs_server.py        Pi 5 - UDP ingest for both, web server
groundstation/dashboard.html      two-aircraft dashboard, single file, no CDN
groundstation/sim_two.py          rehearse both aircraft with no hardware
wiring_v5.html                    the schematic
```

---

## 1. What changed because of these boards

**The S3-PRO does not break out GPIO9 or GPIO10.** The header carries
IO0–IO8, IO13–IO21, IO35–IO42, IO45–IO48, plus TX/RX. So:

| Was | Now | Why |
|---|---|---|
| SCL on GPIO9 | **GPIO16** | GPIO9 does not exist on this header |
| VBAT on GPIO10 | **GPIO8** | GPIO10 does not exist; ADC1 is GPIO1–10, so battery sense had to land inside GPIO1–8 |
| SDA on GPIO8 | **GPIO15** | GPIO8 was needed for the battery divider |
| LiDAR RX on GPIO15 | **GPIO18** | GPIO15 became SDA |
| Link RX on GPIO18 | **GPIO21** | GPIO18 became the LiDAR |
| LED on GPIO21 | **GPIO47** | GPIO21 became the link RX |

**The C5-MINI stops at IO15.** There is no GPIO23 or GPIO24, so the UART to
the flight controller is now **IO4 (RX)** and **IO5 (TX)**. IO2 and IO7 are
left free — they are strapping pins. TXD/RXD belong to the USB console.

**Check the S3-PRO for a 5 V / VIN pin before wiring power.** The right-hand
header shows `RST 3V3 3V3`. If there is no 5 V input broken out, power it
through its USB-C port from the BEC (a USB-C breakout, or a sacrificial
cable). **Do not feed 5 V into a 3V3 pin** — that bypasses the on-board
regulator and puts 5 V on a 3.3 V rail.

---

## 2. Final pin map — ESP32-S3-PRO

Identical on both aircraft. Only the populated modules differ.

| Pin | Function | Fitted on |
|---|---|---|
| GPIO15 | I²C SDA — GY-87 + VL53L0X | both |
| GPIO16 | I²C SCL — GY-87 + VL53L0X | both |
| GPIO8 | VBAT via 100k/27k divider | both |
| GPIO17 | UART1 TX → C5 **IO4** | both |
| GPIO21 | UART1 RX ← C5 **IO5** | both |
| GPIO14 | active buzzer + | both |
| GPIO47 | LED anode via 330 Ω | both |
| GPIO13 | safety button → GND | both |
| GPIO4/5/6/7 | ESC 1/2/3/4 signal | both |
| GPIO1 | MQ-4 AOUT via 10k/15k | **gas only** |
| GPIO2 | MQ-7 AOUT via 10k/15k | **gas only** |
| GPIO18 | YDLIDAR X2 TX | **lidar only** |
| 3V3 | GY-87 VCC, VL53L0X VIN | both — **not 5 V** |

Left alone: GPIO0, GPIO3, GPIO45, GPIO46 (strapping), GPIO19/20 (native USB),
GPIO35–42 (may be PSRAM or the microSD slot depending on the module fitted).

## 3. Final pin map — ESP32-C5-MINI_V1.0

| Pin | Function |
|---|---|
| IO4 | UART1 RX ← S3 GPIO17 |
| IO5 | UART1 TX → S3 GPIO21 |
| 5V | from the PDB BEC — its own feed, do not bridge rails with the S3 |
| GND | common with the S3 — **mandatory**, or the link produces garbage |

Nothing else is connected. It is a radio with a serial port.

---

## 4. Building the firmware

**Board manager:** `esp32` by Espressif, **3.3.0 or newer**. Older versions
have no C5 support and no `setBandMode()`.
**Library:** `VL53L0X` by Pololu. That is the only external dependency.

| Sketch | Board | Set before flashing |
|---|---|---|
| `drone_fc_s3` | ESP32S3 Dev Module | `#define AIRCRAFT AC_GAS` or `AC_LIDAR` |
| `drone_link_c5` | ESP32C5 Dev Module | `#define DRONE_ID "gas"` / `"lidar"` and `LINK_ROLE` |

Four builds total. Label the boards as you flash them — a gas link board
flashed with `DRONE_ID "lidar"` looks completely healthy and reports under
the wrong name.

---

## 5. Network — pick one

### Topology A — no router (default in the sketches)

```
gas C5     LINK_ROLE = ROLE_AP     creates GEONETRA_5G on 5 GHz ch 36
lidar C5   LINK_ROLE = ROLE_STA    joins it
Pi 5                               joins it
```

Nothing extra to own. The caveat worth knowing before you rely on it: the
LiDAR aircraft's packets reach the Pi by going station → AP → station, which
means the whole network lives on a board that is also flying, and the AP
reboots when you unplug that aircraft.

### Topology B — Pi 5 is the access point (more robust with two aircraft)

Set **both** C5s to `ROLE_STA`, then on the Pi:

```bash
sudo apt install -y hostapd dnsmasq
sudo nmcli con add type wifi ifname wlan0 con-name geoap autoconnect yes ssid GEONETRA_5G
sudo nmcli con modify geoap 802-11-wireless.mode ap 802-11-wireless.band a \
     802-11-wireless.channel 36 ipv4.method shared \
     wifi-sec.key-mgmt wpa-psk wifi-sec.psk "geonetra2026"
sudo nmcli con up geoap
```

The Pi keeps ethernet at the same time, so you can still SSH in over the wire.
This is the one that will not surprise you: mains power, a real antenna, and
it does not go down when a drone does.

### Joining the C5's AP from the Pi (topology A)

```bash
nmcli dev wifi connect GEONETRA_5G password geonetra2026 ifname wlan0
ip addr show wlan0        # expect 192.168.4.x
```

---

## 6. Running it

```bash
pip install fastapi uvicorn websockets --break-system-packages
cd groundstation
python3 gs_server.py
```

Open `http://<pi-address>:8000/`. Both aircraft appear as chips in the header;
each column fills in as its link board starts talking.

**Rehearse tonight, before the boards exist:**

```bash
python3 gs_server.py                       # terminal 1
python3 sim_two.py                         # terminal 2
python3 sim_two.py --gas-event             # methane climbs through the alarm
python3 sim_two.py --only lidar            # one aircraft
```

### Commands

Every button is addressed, so one dashboard drives both aircraft:

```
ALL:ESTOP   GAS:MOTORTEST   LIDAR:CALGYRO   GAS:SKIPPREHEAT
```

The firmware ignores anything not addressed to it. Unaddressed commands typed
into the USB serial monitor are accepted by whichever board you typed them
into, which is what you want when you are debugging one aircraft on the bench.

---

## 7. Bring-up order

Do this per aircraft, on USB, **before** any battery:

```
[ ] Every stripboard cut verified with a multimeter
[ ] No short between 5 V and GND, or VBAT and GND
[ ] MQ dividers measured with 5 V applied - BOTH below 3.3 V   [gas]
[ ] GY-87 and VL53L0X on 3V3, not 5 V
[ ] S3 GPIO17 -> C5 IO4, S3 GPIO21 -> C5 IO5, grounds common
[ ] PROPELLERS OFF
```

1. **C5 first.** Serial monitor at 115200. The AP board must print
   `channel 36 band 5GHz`. Below channel 15 means it came up on 2.4 GHz —
   check the core version and stop there.
2. **S3 second.** The I²C scan must find `0x68`, `0x77`, a magnetometer
   (`0x0D` or `0x1E`) and `0x29`. Then a 3-second gyro calibration — keep it
   still. Then, on the gas aircraft, the preheat countdown.
3. **Pi.** Both chips go green within a second of the first packet.

`PREHEAT_S` is 90 seconds so a demo is not held hostage. Real gas work needs
the full 20 minutes, and the dashboard marks every reading uncalibrated until
the countdown ends.

---

## 8. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| C5 prints channel < 15 | Old Arduino-ESP32 core | Update to 3.3.0+ |
| C5 STA never joins | AP board not up, or 2.4 GHz | Bring up the AP board first and check its printed band |
| Pi can't see the SSID | Regulatory domain blocks ch 36 | `sudo raspi-config` → Localisation → WLAN Country → IN, or set `AP_CHANNEL 44` |
| Telemetry stops after seconds | No common ground S3↔C5 | Add the ground wire between the boards |
| Garbage on the UART | TX↔TX instead of TX↔RX | S3 GPIO17→C5 IO4, S3 GPIO21→C5 IO5 |
| No magnetometer in the scan | Aux-bus bypass did not run | Confirm the MPU is found at 0x68 first — the compass sits behind it |
| One drone shows under the other's name | Wrong `DRONE_ID` | Reflash that C5 |
| ppm figures absurd | Preheat skipped, or R0 in dirty air | Power-cycle in clean air, let the countdown finish |
| ToF reads 0 or nothing | Out of range or direct sunlight | Aim at a surface 5 cm–1.2 m away, indoors |
| Motors twitch, don't spin | ESCs never calibrated | `CALIBRATE_ESCS 1`, props off, follow the serial prompts |

---

## 9. Say these before you are asked

- **This is 5 GHz Wi-Fi, not 5G.** The 5G hop in the full system is
  Pi → RUTX50 → KIIT 5G Lab MEC, and it is not in this build.
- **No flight control here.** No PID, no arming, no takeoff — this is the
  sensing and telemetry half, deliberately.
- **No horizontal position sensor** anywhere in the project. IMU + baro +
  downward ToF give attitude and altitude and nothing else.
- **FTM ranging is not in this build.** The link panel's RTT-versus-reality
  comparison makes the same argument live, without a second radio mode to go
  wrong in front of an audience.
