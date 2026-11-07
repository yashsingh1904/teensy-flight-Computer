#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_BNO08x.h>
#include <TinyGPSPlus.h>
#include <SD.h>
#include <SPI.h>

// ---------- Sensor Objects ----------
Adafruit_BMP3XX bmp;
Adafruit_BNO08x bno = Adafruit_BNO08x();
sh2_SensorValue_t sensorValue;

// ---------- GPS ----------
#define L89_RX 7
#define L89_TX 8
#define L89_BAUD 9600
#define L89_UART Serial2
TinyGPSPlus gps;

// ---------- Telemetry UART ----------
#define TELEMETRY_UART Serial3

// ---------- Debug USB Serial ----------
#define DEBUG_UART Serial

// ---------- Pyro Pins ----------
#define PYRO2 6
#define PYRO3 5
#define PYRO4 4

// ---------- Buzzer ----------
#define BUZZER_PIN 41

// ---------- Deployment Config ----------
const unsigned long PYRO_FIRE_MS = 1000;
const float MAIN_HEIGHT_THRESHOLD = 450;
const float MIN_ASCENT_FOR_DROGUE = 10;
const float DESCENT_DELTA = 0.02;

// ---------- State Flags ----------
bool pyro2Deployed = false;
bool pyro3Fired = false;
bool pyro4Deployed = false;

bool pyro2Firing = false;
bool pyro3Firing = false;
bool pyro4Firing = false;

unsigned long pyro2Start = 0;
unsigned long pyro3Start = 0;
unsigned long pyro4Start = 0;

float refAltitude = NAN;
float altitude = 0.0, height = 0.0, maxHeight = 0.0;

// ---------- Descent Detection ----------
float lastHeights[3] = {0, 0, 0};
bool isDescending = false;
const int IGNORE_COUNT = 3;
int bmpCount = 0;

// ---------- Telemetry ----------
unsigned long packetCount = 0;
const char TEAM_ID[] = "2024-ASI-051";

// ---------- SD ----------
File telemetryFile;

// ---------- GPS Data ----------
double lastLat = 0.0, lastLon = 0.0, lastAltGNSS = 0.0;
int lastSats = 0;
String lastGnssTime = "";
float lastVelocity = 0;

// ---------- Orientation ----------
float roll = 0, pitch = 0, yaw = 0;

// ---------- Flight State Machine ----------
enum FlightState {
  STATE_PAD_IDLE,
  STATE_ASCENT,
  STATE_DROGUE,
  STATE_MAIN,
  STATE_LANDED
};
FlightState currentFlightState = STATE_PAD_IDLE;
String flightState = "ARMED";

// ===============================================================
//                       HELPER FUNCTIONS
// ===============================================================
void beep(int times, int duration, int gap) {
  for (int i = 0; i < times; i++) {
    tone(BUZZER_PIN, 2000);
    delay(duration);
    noTone(BUZZER_PIN);
    delay(gap);
  }
}

// ===============================================================
//                             SETUP
// ===============================================================
void setup() {
  DEBUG_UART.begin(115200);
  TELEMETRY_UART.begin(9600);
  L89_UART.begin(L89_BAUD);
  delay(1000);

  DEBUG_UART.println("Flight Computer Startup...");

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Wire2.begin();
  Wire2.setClock(400000);
  delay(50);

  bool initOK = true;

  // BMP390
  if (!bmp.begin_I2C(0x77, &Wire2)) {
    DEBUG_UART.println("BMP390 not found!");
    initOK = false;
  } else {
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    DEBUG_UART.println("BMP390 OK");
  }

  // BNO085
  bool bnoOk = bno.begin_I2C(0x4A, &Wire2);
  if (!bnoOk) bnoOk = bno.begin_I2C(0x4B, &Wire2);
  if (!bnoOk) {
    DEBUG_UART.println("BNO085 not found!");
    initOK = false;
  } else {
    bno.enableReport(SH2_ACCELEROMETER);
    bno.enableReport(SH2_GYROSCOPE_CALIBRATED);
    bno.enableReport(SH2_ROTATION_VECTOR);
    DEBUG_UART.println("BNO085 OK");
  }

  // Pyros
  pinMode(PYRO2, OUTPUT);
  pinMode(PYRO3, OUTPUT);
  pinMode(PYRO4, OUTPUT);
  digitalWrite(PYRO2, LOW);
  digitalWrite(PYRO3, LOW);
  digitalWrite(PYRO4, LOW);

  // SD card
  if (!SD.begin(BUILTIN_SDCARD)) {
    DEBUG_UART.println("SD card init failed!");
    initOK = false;
  } else {
    DEBUG_UART.println("SD card ready.");
    telemetryFile = SD.open("2024-ASI-Rocketery-051.csv", FILE_WRITE);
    if (telemetryFile) {
      telemetryFile.println("TEAM_ID,TIME_MS,PACKET,HEIGHT,PRESSURE,TEMP,GNSS_TIME,LAT,LON,ALT_GNSS,SATS,"
                            "ACCEL,GYRO,ROLL,PITCH,YAW,FLIGHT_STATE,VELOCITY,MAX_HEIGHT,PYRO2,PYRO3,PYRO4");
      telemetryFile.flush();
    }
  }

  if (initOK) {
    DEBUG_UART.println("✅ All systems initialized successfully!");
    beep(3, 150, 150);
  } else {
    DEBUG_UART.println("⚠ Initialization error!");
  }

  DEBUG_UART.println("System Ready...");
}

// ===============================================================
//                             LOOP
// ===============================================================
void loop() {
  unsigned long now = millis();
  float currentAlt = 0, temp = 0, pressure = 0;

  // ---------- BMP390 Reading ----------
  if (bmp.performReading()) {
    currentAlt = bmp.readAltitude(1013.25);
    temp = bmp.temperature + 273.15; // Kelvin
    pressure = bmp.pressure;         // Pa
  }

  bmpCount++;
  if (bmpCount <= IGNORE_COUNT) return;

  // ---------- Reference Altitude ----------
  if (isnan(refAltitude)) {
    refAltitude = currentAlt;
    DEBUG_UART.print("Ref Altitude set: ");
    DEBUG_UART.println(refAltitude, 2);
    return;
  }

  altitude = currentAlt;
  height = altitude - refAltitude;
  if (height > maxHeight) maxHeight = height;

  // ---------- Descent Detection ----------
  lastHeights[0] = lastHeights[1];
  lastHeights[1] = lastHeights[2];
  lastHeights[2] = height;
  isDescending = (lastHeights[2] < lastHeights[1] - DESCENT_DELTA &&
                  lastHeights[1] < lastHeights[0] - DESCENT_DELTA);

  // =====================================================
  //               PYRO LOGIC (REDESIGNED)
  // =====================================================
  if (!pyro2Deployed && isDescending &&
      height <= maxHeight - 0.7 &&
      maxHeight >= MIN_ASCENT_FOR_DROGUE) {
    pyro2Deployed = true;
    pyro2Firing = true;
    pyro2Start = now;
    digitalWrite(PYRO2, HIGH);
    DEBUG_UART.println("** PYRO2 FIRED! (Apogee - 0.7m)**");
  }

  if (pyro2Deployed && !pyro3Fired && (now - pyro2Start >= 750)) {
    pyro3Fired = true;
    pyro3Firing = true;
    pyro3Start = now;
    digitalWrite(PYRO3, HIGH);
    DEBUG_UART.println("** PYRO3 FIRED! (Payload Deploy 0.75s after Pyro2)**");
  }

  if (!pyro4Deployed && isDescending && pyro2Deployed && pyro3Fired &&
      height <= 450.0) {
    pyro4Deployed = true;
    pyro4Firing = true;
    pyro4Start = now;
    digitalWrite(PYRO4, HIGH);
    DEBUG_UART.println("** PYRO4 FIRED! (Main Chute at 450m)**");
  }

  // ---- Turn off pyros after duration ----
  if (pyro2Firing && (now - pyro2Start >= PYRO_FIRE_MS)) {
    digitalWrite(PYRO2, LOW);
    pyro2Firing = false;
    DEBUG_UART.println("PYRO2 OFF");
  }
  if (pyro3Firing && (now - pyro3Start >= PYRO_FIRE_MS)) {
    digitalWrite(PYRO3, LOW);
    pyro3Firing = false;
    DEBUG_UART.println("PYRO3 OFF");
  }
  if (pyro4Firing && (now - pyro4Start >= PYRO_FIRE_MS)) {
    digitalWrite(PYRO4, LOW);
    pyro4Firing = false;
    DEBUG_UART.println("PYRO4 OFF");
  }

  // =====================================================
  //               SENSOR + TELEMETRY
  // =====================================================
  float ax = 0, ay = 0, az = 0;
  float gx = 0, gy = 0, gz = 0;

  while (bno.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ACCELEROMETER) {
      ax = sensorValue.un.accelerometer.x;
      ay = sensorValue.un.accelerometer.y;
      az = sensorValue.un.accelerometer.z;
    } else if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
      gx = sensorValue.un.gyroscope.x;
      gy = sensorValue.un.gyroscope.y;
      gz = sensorValue.un.gyroscope.z;
    } else if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      float qw = sensorValue.un.rotationVector.real;
      float qx = sensorValue.un.rotationVector.i;
      float qy = sensorValue.un.rotationVector.j;
      float qz = sensorValue.un.rotationVector.k;
      float sqw = qw * qw;
      float sqx = qx * qx;
      float sqy = qy * qy;
      float sqz = qz * qz;
      roll  = atan2(2.0 * (qx * qy + qw * qz), sqw + sqx - sqy - sqz);
      pitch = -asin(2.0 * (qx * qz - qw * qy));
      yaw   = atan2(2.0 * (qy * qz + qw * qx), sqw - sqx - sqy + sqz);
    }
  }

  // ---------- GPS ----------
  while (L89_UART.available()) gps.encode(L89_UART.read());
  if (gps.location.isValid()) { lastLat = gps.location.lat(); lastLon = gps.location.lng(); }
  if (gps.altitude.isValid()) lastAltGNSS = gps.altitude.meters();
  if (gps.satellites.isValid()) lastSats = gps.satellites.value();
  if (gps.speed.isValid()) lastVelocity = gps.speed.mps();
  if (gps.time.isValid()) {
    lastGnssTime = String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second());
  }

  // ---------- Telemetry ----------
  String telemLine =
    String(TEAM_ID) + "," + String(now) + "," + String(packetCount++) + "," +
    String(height, 2) + "," + String(pressure, 2) + "," + String(temp, 2) + "," +
    lastGnssTime + "," + String(lastLat, 6) + "," + String(lastLon, 6) + "," +
    String(lastAltGNSS, 2) + "," + String(lastSats) + "," +
    String(ax, 2) + "|" + String(ay, 2) + "|" + String(az, 2) + "," +
    String(gx, 2) + "|" + String(gy, 2) + "|" + String(gz, 2) + "," +
    String(roll, 2) + "," + String(pitch, 2) + "," + String(yaw, 2) + "," +
    flightState + "," + String(lastVelocity, 2) + "," + String(maxHeight, 2) + "," +
    String(pyro2Deployed ? 1 : 0) + "," + String(pyro3Fired ? 1 : 0) + "," + String(pyro4Deployed ? 1 : 0);

  TELEMETRY_UART.println(telemLine);
  DEBUG_UART.println(telemLine);

  if (telemetryFile) {
    telemetryFile.println(telemLine);
    telemetryFile.flush();
  }

  delay(50);
}
