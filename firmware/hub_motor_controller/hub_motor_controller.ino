#include "driver/dac.h"
#include <Wire.h>

// Hall sensors
#define HALL_LA 32
#define HALL_LB 34
#define HALL_LC 35
#define HALL_RA 13
#define HALL_RB 14
#define HALL_RC 27

// DAC channels
#define THROTTLE_R DAC_CHAN_0   // GPIO25
#define THROTTLE_L DAC_CHAN_1   // GPIO26

// Direction pins (active-LOW: LOW = reverse)
#define DIR_L 2
#define DIR_R 4

// DAC range
#define DAC_MIN       108
#define DAC_MAX       135
#define POLE_PAIRS    10
#define TICKS_PER_REV (POLE_PAIRS * 6)

// Ultrasonic sensors (HC-SR04)
#define US_TRIG          16
#define US_NUM           4
#define US_INTERVAL_MS   100UL
#define US_TIMEOUT_US    25000UL

const uint8_t US_ECHO[US_NUM] = {18, 19, 5, 17};

volatile uint32_t _usRise[US_NUM];
volatile uint32_t _usDur[US_NUM];
volatile bool     _usHit[US_NUM];

#define DEF_US_ISR(IDX, PIN)                                        \
  void IRAM_ATTR _usISR_##IDX() {                                   \
    if (digitalRead(PIN)) {                                          \
      _usRise[IDX] = micros();                                       \
    } else if (_usRise[IDX]) {                                       \
      _usDur[IDX] = micros() - _usRise[IDX];                        \
      _usHit[IDX] = true;                                            \
    }                                                                \
  }

DEF_US_ISR(0, 18)
DEF_US_ISR(1, 19)
DEF_US_ISR(2,  5)
DEF_US_ISR(3, 17)

void fireUS() {
  for (int i = 0; i < US_NUM; i++) { _usRise[i] = 0; _usHit[i] = false; }
  digitalWrite(US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG, LOW);
}

static uint32_t _lastUsTrigMs = 0;

void updateUS(uint32_t now) {
  if (now - _lastUsTrigMs < US_INTERVAL_MS) return;
  _lastUsTrigMs = now;

  int d[US_NUM];
  for (int i = 0; i < US_NUM; i++) {
    if (_usHit[i] && _usDur[i] < US_TIMEOUT_US) {
      int cm = (int)(_usDur[i] / 58U);
      d[i] = (cm >= 2 && cm <= 400) ? cm : -1;
    } else {
      d[i] = -1;
    }
  }

  Serial.printf("U %d %d %d %d\n", d[0], d[1], d[2], d[3]);

  fireUS();
}

#define MPU_ADDR        0x68
#define MPU_PWR_MGMT_1  0x6B
#define MPU_ACCEL_XOUT_H 0x3B
#define MPU_ACCEL_LSB_PER_G   16384.0f
#define MPU_GYRO_LSB_PER_DPS  131.0f
#define G_TO_MPS2       9.80665f
#define DEG_TO_RAD      0.017453293f

bool  imuOk = false;
float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;

bool mpuWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool mpuReadRaw(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, 14) != 14) return false;

  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();
  int16_t rawGx = (Wire.read() << 8) | Wire.read();
  int16_t rawGy = (Wire.read() << 8) | Wire.read();
  int16_t rawGz = (Wire.read() << 8) | Wire.read();

  ax = (rawAx / MPU_ACCEL_LSB_PER_G) * G_TO_MPS2;
  ay = (rawAy / MPU_ACCEL_LSB_PER_G) * G_TO_MPS2;
  az = (rawAz / MPU_ACCEL_LSB_PER_G) * G_TO_MPS2;
  gx = (rawGx / MPU_GYRO_LSB_PER_DPS) * DEG_TO_RAD;
  gy = (rawGy / MPU_GYRO_LSB_PER_DPS) * DEG_TO_RAD;
  gz = (rawGz / MPU_GYRO_LSB_PER_DPS) * DEG_TO_RAD;
  return true;
}

void mpuCalibrateGyroBias() {
  const int N = 200;
  float sx = 0, sy = 0, sz = 0, ax, ay, az, gx, gy, gz;
  int got = 0;
  for (int i = 0; i < N; i++) {
    if (mpuReadRaw(ax, ay, az, gx, gy, gz)) { sx += gx; sy += gy; sz += gz; got++; }
    delay(3);
  }
  if (got > 0) {
    gyroBiasX = sx / got;
    gyroBiasY = sy / got;
    gyroBiasZ = sz / got;
  }
}

#define LEFT_TICK_SCALE 0.66f

#define KP     3.0f
#define KI     2.0f
#define I_MAX  12.0f

volatile bool leftReverse  = false;
volatile bool rightReverse = false;

volatile long     leftTicks   = 0;
volatile long     rightTicks  = 0;
volatile uint32_t leftPulses  = 0;
volatile uint32_t rightPulses = 0;

void IRAM_ATTR leftISR() {
  if (leftReverse) leftTicks -= 1; else leftTicks += 1;
  leftPulses += 1;
}
void IRAM_ATTR rightISR() {
  if (rightReverse) rightTicks -= 1; else rightTicks += 1;
  rightPulses += 1;
}

void setDAC(int l, int r) {
  digitalWrite(DIR_L, leftReverse  ? LOW : HIGH);
  digitalWrite(DIR_R, rightReverse ? LOW : HIGH);
  dac_output_voltage(THROTTLE_L, (l == 0) ? 0 : constrain(abs(l), DAC_MIN, DAC_MAX));
  dac_output_voltage(THROTTLE_R, (r == 0) ? 0 : constrain(abs(r), DAC_MIN, DAC_MAX));
}

float targetRpmL = 0.0f, targetRpmR = 0.0f;
float integralL  = 0.0f, integralR  = 0.0f;
int   dacL = 0, dacR = 0;
uint8_t holdCyclesL = 0, holdCyclesR = 0;
#define DIR_HOLD_CYCLES 3

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);

  pinMode(HALL_LA, INPUT); pinMode(HALL_LB, INPUT); pinMode(HALL_LC, INPUT);
  pinMode(HALL_RA, INPUT); pinMode(HALL_RB, INPUT); pinMode(HALL_RC, INPUT);

  attachInterrupt(digitalPinToInterrupt(HALL_LA), leftISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_LB), leftISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_LC), leftISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_RA), rightISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_RB), rightISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_RC), rightISR, CHANGE);

  pinMode(DIR_L, OUTPUT); digitalWrite(DIR_L, HIGH);
  pinMode(DIR_R, OUTPUT); digitalWrite(DIR_R, HIGH);

  dac_output_enable(THROTTLE_L);
  dac_output_enable(THROTTLE_R);
  setDAC(0, 0);

  pinMode(US_TRIG, OUTPUT);
  digitalWrite(US_TRIG, LOW);
  for (int i = 0; i < US_NUM; i++) {
    pinMode(US_ECHO[i], INPUT);
  }
  attachInterrupt(digitalPinToInterrupt(18), _usISR_0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(19), _usISR_1, CHANGE);
  attachInterrupt(digitalPinToInterrupt( 5), _usISR_2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(17), _usISR_3, CHANGE);
  delay(10);
  fireUS();

  Wire.begin();
  imuOk = mpuWriteReg(MPU_PWR_MGMT_1, 0x00);
  if (imuOk) {
    delay(50);
    mpuCalibrateGyroBias();
    Serial.println("MPU6050 READY");
  } else {
    Serial.println("MPU6050 NOT FOUND -- IMU data disabled");
  }

  Serial.println("ARGO MINI READY");
}

void loop() {
  uint32_t now = millis();
  updateUS(now);

  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line.startsWith("V ")) {
      int spaceIdx = line.indexOf(' ', 2);
      if (spaceIdx > 0) {
        float rL = line.substring(2, spaceIdx).toFloat();
        float rR = line.substring(spaceIdx + 1).toFloat();

        if (rL > 0 && rR < 0) rR = -rL;
        else if (rL < 0 && rR > 0) rL = -rR;

        bool newLeftRev  = (rL < 0);
        bool newRightRev = (rR < 0);

        if (newLeftRev  != leftReverse)  { integralL = 0.0f; holdCyclesL = DIR_HOLD_CYCLES; }
        if (newRightRev != rightReverse) { integralR = 0.0f; holdCyclesR = DIR_HOLD_CYCLES; }

        targetRpmL = rL;
        targetRpmR = rR;

        leftReverse  = newLeftRev;
        rightReverse = newRightRev;
        digitalWrite(DIR_L, leftReverse  ? LOW : HIGH);
        digitalWrite(DIR_R, rightReverse ? LOW : HIGH);
      }
    } else if (line == "S") {
      targetRpmL = 0; targetRpmR = 0;
      integralL  = 0; integralR  = 0;
      dacL = 0;       dacR = 0;
      holdCyclesL = 0; holdCyclesR = 0;
      leftReverse  = false;
      rightReverse = false;
      setDAC(0, 0);
      Serial.println("STOP");
    } else if (line == "R") {
      noInterrupts();
      leftTicks = 0; rightTicks = 0;
      interrupts();
      Serial.println("ODOM_RESET");
    }
  }

  static uint32_t lastPrint = 0;
  if (now - lastPrint >= 50) {
    float elapsed = (now - lastPrint) / 1000.0f;
    lastPrint = now;

    noInterrupts();
    uint32_t lp = leftPulses;  leftPulses  = 0;
    uint32_t rp = rightPulses; rightPulses = 0;
    long lt = leftTicks;
    long rt = rightTicks;
    interrupts();

    float measRpmL = (float)lp / elapsed * 60.0f / TICKS_PER_REV * LEFT_TICK_SCALE;
    float measRpmR = (float)rp / elapsed * 60.0f / TICKS_PER_REV;

    if (holdCyclesL > 0) {
      dacL = 0;
      holdCyclesL--;
    } else if (abs(targetRpmL) < 1.0f) {
      dacL = 0;
      integralL = 0.0f;
    } else {
      float errL = abs(targetRpmL) - measRpmL;
      integralL = constrain(integralL + errL * elapsed, -I_MAX, I_MAX);
      dacL = constrain(DAC_MIN + (int)(KP * errL + KI * integralL), DAC_MIN, DAC_MAX);
    }

    if (holdCyclesR > 0) {
      dacR = 0;
      holdCyclesR--;
    } else if (abs(targetRpmR) < 1.0f) {
      dacR = 0;
      integralR = 0.0f;
    } else {
      float errR = abs(targetRpmR) - measRpmR;
      integralR = constrain(integralR + errR * elapsed, -I_MAX, I_MAX);
      dacR = constrain(DAC_MIN + (int)(KP * errR + KI * integralR), DAC_MIN, DAC_MAX);
    }

    setDAC(dacL, dacR);

    Serial.printf("O %ld %ld\n", lt, rt);

    if (imuOk) {
      float ax, ay, az, gx, gy, gz;
      if (mpuReadRaw(ax, ay, az, gx, gy, gz)) {
        gx -= gyroBiasX;
        gy -= gyroBiasY;
        gz -= gyroBiasZ;
        Serial.printf("I %.4f %.4f %.4f %.5f %.5f %.5f\n", ax, ay, az, gx, gy, gz);
      }
    }

    if (now % 500 < 50) {
      Serial.printf("R %.1f %.1f\n", measRpmL, measRpmR);
    }
  }
}
