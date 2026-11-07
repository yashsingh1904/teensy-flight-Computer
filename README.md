# Teensy 4.1 Flight Computer & Telemetry

Flight computer for a high-power rocket built by **DTU Rocketry (Project Akaash)**.
It reads altitude, motion, and position sensors, detects apogee, controls
parachute deployment, logs everything to an SD card, and streams telemetry to a
ground station over XBee (tested end-to-end up to **1 km**).

Flown at the **IN-SPACe & ISRO Rocketry Competition 2025** (Excellent grade for the
avionics and software subsystem).

Assembled flight computer
<img width="438" height="488" alt="image" src="https://github.com/user-attachments/assets/43d1c329-c5e7-4994-afa6-c93f356e6c6c" />


PCB 3D render:
<img width="795" height="740" alt="image" src="https://github.com/user-attachments/assets/67268ba8-81f7-47fc-9911-b14f37bd407f" />


## Hardware

| Component | Role | Interface |
|---|---|---|
| Teensy 4.1 | Main microcontroller | - |
| BMP390 | Pressure, temperature, altitude | I2C (Wire2, 0x77) |
| BNO085 | Acceleration, gyro, orientation | I2C (Wire2, 0x4A / 0x4B) |
| Quectel L89 | GNSS position, speed, time | UART (Serial2, 9600) |
| XBee | Wireless telemetry link | UART (Serial3, 9600) |
| SD card (built-in) | Onboard CSV logging | SDIO |
| 3 pyro channels | Parachute / payload deployment | GPIO |
| Buzzer | Startup status beeps | GPIO |

## How it works

1. **Startup:** initializes the sensors, GNSS, SD card, and pyro pins. Three beeps
   mean everything initialized correctly.
2. **Reference altitude:** after discarding the first few barometer readings, the
   pad altitude is recorded; all heights are relative to it.
3. **Apogee / descent detection:** descent is detected from consecutive falling
   barometric height samples. Apogee deployment fires once the rocket has climbed
   at least 10 m and drops 0.7 m below its maximum height.
4. **Deployment sequence:**
   - **PYRO2:** drogue at apogee
   - **PYRO3:** payload deployment 0.75 s after PYRO2
   - **PYRO4:** main parachute at 450 m during descent
   Each channel is energized for 1 s using millis()-based timing (no blocking delays).
5. **Telemetry:** every cycle (about 20 packets per second) a CSV packet is sent over
   XBee, printed to USB serial, and appended to the SD card.

## Telemetry packet

One CSV line per packet:

```
TEAM_ID,TIME_MS,PACKET,HEIGHT,PRESSURE,TEMP,GNSS_TIME,LAT,LON,ALT_GNSS,SATS,
ACCEL,GYRO,ROLL,PITCH,YAW,FLIGHT_STATE,VELOCITY,MAX_HEIGHT,PYRO2,PYRO3,PYRO4
```
- Pressure is in Pa and temperature in Kelvin.
- `ACCEL` and `GYRO` are `x|y|z` triples.
- `FLIGHT_STATE` is `ARMED`, `DROGUE`, or `MAIN`.

## Ground station

Packets are received by a second XBee connected to a laptop over USB serial and
shown live on a dashboard:
[Real-Time-Telemetry-Dashboard](https://github.com/yashsingh1904/Real-Time-Telemetry-Dashboard)


## Build and flash

1. Install the Arduino IDE with **Teensyduino**.
2. Install libraries: `Adafruit BMP3XX`, `Adafruit BNO08x`, `TinyGPSPlus`.
3. Select **Teensy 4.1**, open `flight_computer/flight_computer.ino`, upload.

## Notes

- Pyro outputs drive recovery charges on a model rocket. Test with no charges
  connected and follow your range's safety rules.
- Barometric height is used for deployment decisions; GNSS is telemetry-only.

## Author

Yash Kumar Singh: avionics and software lead, DTU Rocketry.
[LinkedIn](https://www.linkedin.com/in/yash-kumar-singh-5971472b9/)
