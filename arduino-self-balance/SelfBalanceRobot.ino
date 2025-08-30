// Self-balancing robot for Arduino Nano + MPU6050 + dual stepper (A4988/DRV8825)
// No external libraries required beyond Wire.

#include <Wire.h>

// ============================
// Configuration
// ============================

// I2C address of MPU6050
const uint8_t MPU6050_ADDR = 0x68; // AD0 low = 0x68, AD0 high = 0x69

// Stepper driver pins (A4988/DRV8825). Adjust to your wiring.
const uint8_t PIN_LEFT_STEP  = 3;
const uint8_t PIN_LEFT_DIR   = 4;
const uint8_t PIN_RIGHT_STEP = 5;
const uint8_t PIN_RIGHT_DIR  = 6;
const uint8_t PIN_ENABLE     = 7; // Active LOW for A4988/DRV8825

// Optional: microstep pins if you want to control microstepping via code
// Set to 255 to ignore
const uint8_t PIN_MS1 = 255;
const uint8_t PIN_MS2 = 255;
const uint8_t PIN_MS3 = 255;

// Invert direction if wheels spin the wrong way
const bool LEFT_DIR_INVERTED  = false;
const bool RIGHT_DIR_INVERTED = true;

// PID gains (output is steps/second). Start with Ki=Kd=0, tune Kp first.
float Kp = 240.0f;  // proportional gain
float Ki = 0.0f;    // integral gain
float Kd = 4.0f;    // derivative gain

// Complementary filter blending (0..1). Higher favors gyro integration.
const float COMPLEMENTARY_ALPHA = 0.98f;

// Control loop target rate (Hz). 200-500 Hz recommended.
const float LOOP_HZ = 250.0f;

// Angle setpoint (degrees). 0 means perfectly upright.
float angleSetpointDeg = 0.0f;

// Safety limits
const float FALL_ANGLE_DEG              = 45.0f;  // disable if tilted beyond this
const float REENABLE_ANGLE_DEG          = 8.0f;   // require within this to re-enable
const uint16_t REENABLE_STABLE_CYCLES   = 40;     // consecutive cycles within re-enable band

// Output limits (steps per second)
const float MAX_STEP_RATE_SPS = 6000.0f; // keep under ~8-10k for reliability
const float MIN_ACTIVE_RATE_SPS = 30.0f; // deadband to avoid jitter

// Gyro calibration
const uint16_t GYRO_CALIBRATION_SAMPLES = 1000;   // keep robot still on power-up

// Debug prints
const bool DEBUG_SERIAL = true;

// ============================
// MPU6050 low-level registers
// ============================
const uint8_t REG_PWR_MGMT_1 = 0x6B;
const uint8_t REG_SMPLRT_DIV = 0x19;
const uint8_t REG_CONFIG     = 0x1A;
const uint8_t REG_GYRO_CONFIG= 0x1B;
const uint8_t REG_ACCEL_CONFIG=0x1C;
const uint8_t REG_INT_ENABLE = 0x38;
const uint8_t REG_ACCEL_XOUT = 0x3B;

// Sensitivities for +/-2g accel and +/-250 dps gyro
const float ACCEL_SENS = 16384.0f; // LSB/g
const float GYRO_SENS  = 131.0f;   // LSB/(deg/s)

// ============================
// State
// ============================
volatile float angleDeg = 0.0f;        // estimated pitch angle (deg), +forward tilt
volatile float gyroBiasX = 0.0f;       // bias for gyro X (deg/s)

float pidIntegral = 0.0f;
float previousError = 0.0f;
unsigned long lastLoopMicros = 0;

// stepper driver abstraction
struct StepperDriver {
  uint8_t stepPin;
  uint8_t dirPin;
  bool dirInverted;
  float targetRateSps; // desired steps per second, signed
  unsigned long nextStepDueMicros;
  unsigned long stepIntervalMicros;    // computed from |targetRateSps|
  bool enabled;
};

StepperDriver motorLeft  = { PIN_LEFT_STEP,  PIN_LEFT_DIR,  LEFT_DIR_INVERTED,  0.0f, 0UL, 0UL, false };
StepperDriver motorRight = { PIN_RIGHT_STEP, PIN_RIGHT_DIR, RIGHT_DIR_INVERTED, 0.0f, 0UL, 0UL, false };

// re-enable gating
bool controlEnabled = false;
uint16_t stableCounter = 0;

// ============================
// Utility
// ============================
static inline void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
}

static inline void readRegisters(uint8_t startReg, uint8_t count, uint8_t* dest) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(startReg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU6050_ADDR, (int)count, (int)true);
  for (uint8_t i = 0; i < count && Wire.available(); ++i) {
    dest[i] = Wire.read();
  }
}

void mpuInit() {
  delay(100);
  // Wake up device and select PLL with X axis gyroscope reference (better stability)
  writeRegister(REG_PWR_MGMT_1, 0x01);
  delay(10);

  // Set sample rate to 1kHz / (1 + SMPLRT_DIV). Here, 1kHz/5 = 200Hz.
  writeRegister(REG_SMPLRT_DIV, 4);

  // DLPF: set to 44Hz accel, 42Hz gyro (CONFIG=3) for reduced noise
  writeRegister(REG_CONFIG, 0x03);

  // Gyro full scale +/-250 dps
  writeRegister(REG_GYRO_CONFIG, 0x00);

  // Accel full scale +/-2g
  writeRegister(REG_ACCEL_CONFIG, 0x00);

  // Disable interrupts
  writeRegister(REG_INT_ENABLE, 0x00);
}

void mpuReadRaw(int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz) {
  uint8_t buf[14];
  readRegisters(REG_ACCEL_XOUT, 14, buf);
  ax = (int16_t)((buf[0] << 8) | buf[1]);
  ay = (int16_t)((buf[2] << 8) | buf[3]);
  az = (int16_t)((buf[4] << 8) | buf[5]);
  gx = (int16_t)((buf[8] << 8) | buf[9]);
  gy = (int16_t)((buf[10] << 8) | buf[11]);
  gz = (int16_t)((buf[12] << 8) | buf[13]);
}

void calibrateGyroBias() {
  long sum = 0;
  const uint16_t samples = GYRO_CALIBRATION_SAMPLES;
  for (uint16_t i = 0; i < samples; ++i) {
    int16_t ax, ay, az, gx, gy, gz;
    mpuReadRaw(ax, ay, az, gx, gy, gz);
    sum += gx;
    delay(2); // ~500 Hz
  }
  float avg = (float)sum / (float)samples;
  gyroBiasX = avg / GYRO_SENS; // convert to deg/s
}

// Compute pitch angle around X-axis (front-back tilt), in degrees
float estimateAngleDeg(float dtSeconds) {
  int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw;
  mpuReadRaw(axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw);

  // accel angle from Y and Z. atan2 returns radians.
  float accAngleRad = atan2f((float)ayRaw, (float)azRaw);
  float accAngleDeg = accAngleRad * 57.29578f; // RAD_TO_DEG

  // gyro rate around X (deg/s), subtract bias
  float gyroRateDegPerSec = ((float)gxRaw / GYRO_SENS) - gyroBiasX;

  // integrate gyro
  float gyroIntegrated = angleDeg + gyroRateDegPerSec * dtSeconds;

  // complementary blend
  float blended = COMPLEMENTARY_ALPHA * gyroIntegrated + (1.0f - COMPLEMENTARY_ALPHA) * accAngleDeg;
  return blended;
}

// ============================
// Stepper control
// ============================
void setDriverEnabled(bool enable) {
  if (enable) {
    digitalWrite(PIN_ENABLE, LOW); // active low
  } else {
    digitalWrite(PIN_ENABLE, HIGH);
  }
}

void applyMotorTarget(StepperDriver& m) {
  float rate = m.targetRateSps;
  if (fabs(rate) < MIN_ACTIVE_RATE_SPS) {
    m.stepIntervalMicros = 0UL;
    return;
  }

  bool forward = rate >= 0.0f;
  bool dirLevel = m.dirInverted ? !forward : forward;
  digitalWrite(m.dirPin, dirLevel ? HIGH : LOW);

  float absRate = fabs(rate);
  if (absRate > MAX_STEP_RATE_SPS) absRate = MAX_STEP_RATE_SPS;
  m.stepIntervalMicros = (unsigned long)(1000000.0f / absRate);
  if (m.stepIntervalMicros == 0) m.stepIntervalMicros = 1; // guard
}

void serviceMotor(StepperDriver& m) {
  if (!m.enabled || m.stepIntervalMicros == 0UL) return;
  unsigned long now = micros();
  if ((long)(now - m.nextStepDueMicros) >= 0) {
    digitalWrite(m.stepPin, HIGH);
    delayMicroseconds(2); // A4988/DRV8825 step pulse width
    digitalWrite(m.stepPin, LOW);
    m.nextStepDueMicros = now + m.stepIntervalMicros;
  }
}

// ============================
// Arduino setup/loop
// ============================
void setup() {
  if (DEBUG_SERIAL) {
    Serial.begin(115200);
    while (!Serial) {}
    Serial.println("SelfBalanceRobot: init");
  }

  pinMode(PIN_LEFT_STEP, OUTPUT);
  pinMode(PIN_LEFT_DIR, OUTPUT);
  pinMode(PIN_RIGHT_STEP, OUTPUT);
  pinMode(PIN_RIGHT_DIR, OUTPUT);
  pinMode(PIN_ENABLE, OUTPUT);

  if (PIN_MS1 != 255) pinMode(PIN_MS1, OUTPUT);
  if (PIN_MS2 != 255) pinMode(PIN_MS2, OUTPUT);
  if (PIN_MS3 != 255) pinMode(PIN_MS3, OUTPUT);

  // Default microstepping: leave floating or set as desired
  if (PIN_MS1 != 255) digitalWrite(PIN_MS1, LOW);
  if (PIN_MS2 != 255) digitalWrite(PIN_MS2, LOW);
  if (PIN_MS3 != 255) digitalWrite(PIN_MS3, LOW);

  setDriverEnabled(false);
  motorLeft.enabled = false;
  motorRight.enabled = false;

  Wire.begin();
  mpuInit();
  delay(100);
  calibrateGyroBias();

  // Initialize loop timing
  lastLoopMicros = micros();

  if (DEBUG_SERIAL) {
    Serial.println("Calibration done. Waiting for upright position to enable control.");
  }
}

void loop() {
  const float loopPeriod = 1.0f / LOOP_HZ;

  // simple loop timing
  unsigned long nowMicros = micros();
  float dt = (nowMicros - lastLoopMicros) * 1e-6f;
  if (dt < loopPeriod * 0.5f) {
    // service steppers while waiting
    serviceMotor(motorLeft);
    serviceMotor(motorRight);
    return;
  }
  lastLoopMicros = nowMicros;

  // estimate angle
  angleDeg = estimateAngleDeg(dt);

  // Safety: enable only when close to upright
  if (fabs(angleDeg) < REENABLE_ANGLE_DEG) {
    if (stableCounter < REENABLE_STABLE_CYCLES) stableCounter++;
  } else {
    stableCounter = 0;
  }

  if (!controlEnabled) {
    if (stableCounter >= REENABLE_STABLE_CYCLES) {
      controlEnabled = true;
      pidIntegral = 0.0f;
      previousError = 0.0f;
      setDriverEnabled(true);
      motorLeft.enabled = true;
      motorRight.enabled = true;
      motorLeft.nextStepDueMicros = micros();
      motorRight.nextStepDueMicros = motorLeft.nextStepDueMicros;
      if (DEBUG_SERIAL) Serial.println("Control ENABLED");
    }
  }

  if (fabs(angleDeg) > FALL_ANGLE_DEG) {
    // fell down, disable
    controlEnabled = false;
    setDriverEnabled(false);
    motorLeft.enabled = false;
    motorRight.enabled = false;
    motorLeft.stepIntervalMicros = 0;
    motorRight.stepIntervalMicros = 0;
    pidIntegral = 0.0f;
    if (DEBUG_SERIAL) Serial.println("FALL detected -> DISABLED");
  }

  float outputSps = 0.0f;
  if (controlEnabled) {
    // PID control on angle
    float error = angleSetpointDeg - angleDeg; // positive when tilting forward
    pidIntegral += error * dt;
    // anti-windup clamp
    const float integralMax = MAX_STEP_RATE_SPS / max(0.001f, Ki == 0.0f ? 1e9f : Ki);
    if (pidIntegral > integralMax) pidIntegral = integralMax;
    if (pidIntegral < -integralMax) pidIntegral = -integralMax;

    float derivative = (error - previousError) / (dt > 0.0005f ? dt : 0.0005f);
    previousError = error;

    outputSps = Kp * error + Ki * pidIntegral + Kd * derivative;

    // Saturate
    if (outputSps > MAX_STEP_RATE_SPS) outputSps = MAX_STEP_RATE_SPS;
    if (outputSps < -MAX_STEP_RATE_SPS) outputSps = -MAX_STEP_RATE_SPS;
  } else {
    outputSps = 0.0f;
  }

  // Mix to motors: same command to both wheels (forward/back)
  motorLeft.targetRateSps  = outputSps;
  motorRight.targetRateSps = outputSps;

  // Apply to hardware
  applyMotorTarget(motorLeft);
  applyMotorTarget(motorRight);

  // Service steppers at least once per loop
  serviceMotor(motorLeft);
  serviceMotor(motorRight);

  if (DEBUG_SERIAL) {
    static uint16_t dbgDiv = 0;
    if (++dbgDiv >= 25) { // ~10 Hz if LOOP_HZ=250
      dbgDiv = 0;
      Serial.print("angle="); Serial.print(angleDeg, 2);
      Serial.print(" err="); Serial.print(angleSetpointDeg - angleDeg, 2);
      Serial.print(" out="); Serial.print(outputSps, 0);
      Serial.print(" en="); Serial.print(controlEnabled);
      Serial.println();
    }
  }
}

