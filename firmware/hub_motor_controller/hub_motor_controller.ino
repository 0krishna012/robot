// ESP32 hub-motor controller — hall tachometer + DAC throttle + MPU6050 IMU, serial link to Jetson.
//
// Host -> ESP32:  "C <left_frac> <right_frac>\n"   frac in [-1.0, 1.0], sign = direction
// ESP32 -> Host:  "E <left_ticks> <right_ticks> <dt_ms>\n"        sent every LOOP_MS
// ESP32 -> Host:  "I <ax> <ay> <az> <gx> <gy> <gz>\n"             sent every LOOP_MS
//                 accel in m/s^2, gyro in rad/s
//
// left_frac/right_frac are fractions of max motor effort, NOT m/s — the host
// (serial_bridge_node) does the m/s <-> fraction conversion and all odometry math.

#include <Arduino.h>
#include <Wire.h>

// ---- Pin map ----
constexpr int HALL_L_A = 32, HALL_L_B = 34, HALL_L_C = 35;
constexpr int HALL_R_A = 13, HALL_R_B = 14, HALL_R_C = 27;
constexpr int DAC_R_PIN = 25;   // DAC_chan_0
constexpr int DAC_L_PIN = 26;   // DAC_chan_1
constexpr int DIR_L_PIN = 2;
constexpr int DIR_R_PIN = 4;
constexpr int IMU_SDA_PIN = 21;
constexpr int IMU_SCL_PIN = 22;

// ---- Motor scaling ----
constexpr int DAC_MIN = 108;              // slowest throttle that still moves the wheel
constexpr int DAC_MAX = 135;              // full throttle
constexpr uint32_t LOOP_MS = 20;          // 50 Hz control/telemetry loop
constexpr uint32_t CMD_TIMEOUT_MS = 500;  // stop motors if host goes silent

// ---- MPU6050 ----
constexpr uint8_t MPU_ADDR = 0x68;
constexpr uint8_t PWR_MGMT_1 = 0x6B;
constexpr uint8_t ACCEL_XOUT_H = 0x3B;
constexpr float ACCEL_SCALE = 16384.0f;   // LSB/g at +/-2g full scale
constexpr float GYRO_SCALE = 131.0f;      // LSB/(deg/s) at +/-250 dps full scale
constexpr float G_ACCEL = 9.80665f;

bool imuInit() {
  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0);  // wake the sensor up
  return Wire.endTransmission() == 0;
}

bool imuRead(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, 14, (int)true) != 14) return false;

  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();  // discard temperature
  int16_t rawGx = (Wire.read() << 8) | Wire.read();
  int16_t rawGy = (Wire.read() << 8) | Wire.read();
  int16_t rawGz = (Wire.read() << 8) | Wire.read();

  ax = rawAx / ACCEL_SCALE * G_ACCEL;
  ay = rawAy / ACCEL_SCALE * G_ACCEL;
  az = rawAz / ACCEL_SCALE * G_ACCEL;
  gx = radians(rawGx / GYRO_SCALE);
  gy = radians(rawGy / GYRO_SCALE);
  gz = radians(rawGz / GYRO_SCALE);
  return true;
}

// ---- Tick counters, updated from ISR ----
volatile long ticksL = 0, ticksR = 0;

void IRAM_ATTR onHallL() { ticksL++; }
void IRAM_ATTR onHallR() { ticksR++; }

float cmdL = 0.0f, cmdR = 0.0f;   // last commanded fraction [-1,1]
uint32_t lastCmdMs = 0;
uint32_t lastLoopMs = 0;

void setMotor(float frac, int dacPin, int dirPin) {
  frac = constrain(frac, -1.0f, 1.0f);
  digitalWrite(dirPin, frac < 0 ? HIGH : LOW);
  int dac = (fabs(frac) < 0.01f) ? 0 : DAC_MIN + (int)(fabs(frac) * (DAC_MAX - DAC_MIN));
  dacWrite(dacPin, dac);
}

void applyStop() {
  cmdL = 0; cmdR = 0;
  setMotor(0, DAC_L_PIN, DIR_L_PIN);
  setMotor(0, DAC_R_PIN, DIR_R_PIN);
}

void parseLine(char *line) {
  if (line[0] != 'C') return;
  float l, r;
  if (sscanf(line + 1, "%f %f", &l, &r) == 2) {
    cmdL = constrain(l, -1.0f, 1.0f);
    cmdR = constrain(r, -1.0f, 1.0f);
    lastCmdMs = millis();
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(HALL_L_A, INPUT); pinMode(HALL_L_B, INPUT); pinMode(HALL_L_C, INPUT);
  pinMode(HALL_R_A, INPUT); pinMode(HALL_R_B, INPUT); pinMode(HALL_R_C, INPUT);
  pinMode(DIR_L_PIN, OUTPUT); pinMode(DIR_R_PIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(HALL_L_A), onHallL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_L_B), onHallL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_L_C), onHallL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_R_A), onHallR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_R_B), onHallR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_R_C), onHallR, CHANGE);

  applyStop();
  imuInit();
  lastLoopMs = millis();
  lastCmdMs = millis();
}

char lineBuf[64];
uint8_t lineLen = 0;

void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      parseLine(lineBuf);
      lineLen = 0;
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    }
  }
}

void loop() {
  pollSerial();

  uint32_t now = millis();
  if (now - lastCmdMs > CMD_TIMEOUT_MS) {
    cmdL = 0; cmdR = 0;   // failsafe stop on lost link
  }

  if (now - lastLoopMs >= LOOP_MS) {
    uint32_t dt = now - lastLoopMs;
    lastLoopMs = now;

    noInterrupts();
    long dL = ticksL; ticksL = 0;
    long dR = ticksR; ticksR = 0;
    interrupts();

    // Hall edges carry no sensed direction by themselves — sign the delta using
    // the direction we just commanded (dir pin controls actual spin direction).
    long sdL = (cmdL < 0) ? -dL : dL;
    long sdR = (cmdR < 0) ? -dR : dR;

    setMotor(cmdL, DAC_L_PIN, DIR_L_PIN);
    setMotor(cmdR, DAC_R_PIN, DIR_R_PIN);

    Serial.printf("E %ld %ld %lu\n", sdL, sdR, dt);

    float ax, ay, az, gx, gy, gz;
    if (imuRead(ax, ay, az, gx, gy, gz)) {
      Serial.printf("I %.4f %.4f %.4f %.4f %.4f %.4f\n", ax, ay, az, gx, gy, gz);
    }
  }
}
