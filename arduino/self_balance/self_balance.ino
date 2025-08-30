#include <Wire.h>

// ===================== Pin Configuration =====================
const uint8_t PIN_STEP_LEFT  = 2;   // STEP for left driver
const uint8_t PIN_DIR_LEFT   = 3;   // DIR  for left driver
const uint8_t PIN_STEP_RIGHT = 5;   // STEP for right driver
const uint8_t PIN_DIR_RIGHT  = 6;   // DIR  for right driver
const uint8_t PIN_ENABLE     = 8;   // EN (Enable) shared for both drivers

// Active level for EN pin on common stepper drivers (A4988/DRV8825): LOW = enable
const bool ENABLE_ACTIVE_LOW = true;

// If a motor is mechanically mirrored, invert its direction here
const bool INVERT_DIR_LEFT  = false;
const bool INVERT_DIR_RIGHT = true;

// ===================== IMU/Filter Config =====================
const uint8_t MPU_ADDR = 0x68; // I2C address of MPU6050

// Complementary filter constant: higher => more gyro trust
const float COMPLEMENTARY_ALPHA = 0.98f;

// If your board orientation or angle sign is opposite, flip one or both of these
const float ANGLE_SIGN  =  1.0f; // Flip to -1 if measured angle sign is opposite
const float OUTPUT_SIGN =  1.0f; // Flip to -1 if robot runs away when it leans

// ===================== PID Config =====================
// PID gains map angle (deg) and gyro (deg/s) to motor speed (steps/s)
float pidKp = 20.0f;  // Start with P-only, then add D and a touch of I
float pidKi = 0.8f;   // Small integral to remove steady-state bias
float pidKd = 0.40f;  // D helps dampen oscillations

// Integral windup limit (in terms of the integral accumulator value)
const float INTEGRAL_LIMIT = 500.0f;

// Angle trim to bias the setpoint a little forward/backward (deg)
float angleTrimDeg = 0.0f;

// ===================== Stepper Control =====================
// Choose a conservative max speed; increase as your mechanics allow
const int32_t MAX_STEPS_PER_SECOND      = 2800;    // depends on microstepping/voltage
const int32_t MAX_ACCEL_STEPS_PER_SEC2  = 15000;   // acceleration limit for ramping
const uint16_t STEP_PULSE_MICROS        = 3;       // step pulse width (us)

// Safety thresholds
const float FALL_ANGLE_DEG   = 45.0f;  // disable if tilt exceeds this
const float RECOVER_ANGLE_DEG = 10.0f; // re-enable when tilt within this

// ===================== State =====================
volatile bool motorsEnabled = false;

// IMU state
float angleDeg = 0.0f;           // fused pitch angle
float gyroBiasDegPerSec = 0.0f;  // gyro Y bias determined at startup

// PID state
float integralTerm = 0.0f;
float previousError = 0.0f;

// Motion state
float targetStepsPerSecond  = 0.0f;
float currentStepsPerSecond = 0.0f;
int32_t stepIntervalMicros  = 0; // derived from currentStepsPerSecond
uint32_t lastStepMicros     = 0;

// Timing
uint32_t lastLoopMicros  = 0;
uint32_t lastPrintMillis = 0;

// ===================== I2C Helpers =====================
void mpuWriteByte(uint8_t reg, uint8_t data) {
	Wire.beginTransmission(MPU_ADDR);
	Wire.write(reg);
	Wire.write(data);
	Wire.endTransmission(true);
}

bool mpuReadBytes(uint8_t startReg, uint8_t count, uint8_t* dest) {
	Wire.beginTransmission(MPU_ADDR);
	Wire.write(startReg);
	if (Wire.endTransmission(false) != 0) return false;

	uint32_t startWait = micros();
	Wire.requestFrom((int)MPU_ADDR, (int)count, (int)true);
	while (Wire.available() < count) {
		if ((micros() - startWait) > 2000) {
			return false;
		}
	}
	for (uint8_t i = 0; i < count; i++) {
		dest[i] = Wire.read();
	}
	return true;
}

bool readMPU6050Raw(int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz) {
	uint8_t buffer[14];
	if (!mpuReadBytes(0x3B, 14, buffer)) return false;
	ax = (int16_t)((buffer[0] << 8) | buffer[1]);
	ay = (int16_t)((buffer[2] << 8) | buffer[3]);
	az = (int16_t)((buffer[4] << 8) | buffer[5]);
	// int16_t temp = (int16_t)((buffer[6] << 8) | buffer[7]); // not used
	gx = (int16_t)((buffer[8] << 8) | buffer[9]);
	gy = (int16_t)((buffer[10] << 8) | buffer[11]);
	gz = (int16_t)((buffer[12] << 8) | buffer[13]);
	return true;
}

void initMPU6050() {
	// Wake and configure filters/ranges
	mpuWriteByte(0x6B, 0x01); // PWR_MGMT_1: clock source = PLL with X axis gyro
	mpuWriteByte(0x1A, 0x03); // CONFIG: DLPF_CFG = 3 (~44 Hz bandwidth)
	mpuWriteByte(0x19, 0x04); // SMPLRT_DIV: sample rate = 1kHz/(1+4) = 200 Hz
	mpuWriteByte(0x1B, 0x00); // GYRO_CONFIG: +/- 250 deg/s
	mpuWriteByte(0x1C, 0x00); // ACCEL_CONFIG: +/- 2g
}

float computeAccPitchDeg(int16_t axRaw, int16_t azRaw) {
	// atan2(X, Z) assuming X is forward axis, Z is up; adjust ANGLE_SIGN for orientation
	const float ax = (float)axRaw;
	const float az = (float)azRaw;
	float angle = atan2f(ax, az) * 57.2957795f; // RAD_TO_DEG
	return angle * ANGLE_SIGN;
}

void calibrateSensors(uint16_t samples = 600) {
	int32_t gySum = 0;
	int32_t axSum = 0;
	int32_t azSum = 0;
	int16_t ax, ay, az, gx, gy, gz;
	for (uint16_t i = 0; i < samples; i++) {
		if (readMPU6050Raw(ax, ay, az, gx, gy, gz)) {
			gySum += (int32_t)gy;
			axSum += (int32_t)ax;
			azSum += (int32_t)az;
		}
		delay(2); // ~300 Hz sampling during calibration
	}
	gyroBiasDegPerSec = ((float)gySum / (float)samples) / 131.0f; // 131 LSB/(deg/s) at +/-250 dps
	// Initialize angle from accelerometer
	float accAngle = computeAccPitchDeg((int16_t)(axSum / (int32_t)samples), (int16_t)(azSum / (int32_t)samples));
	angleDeg = accAngle;
}

// ===================== Motor Helpers =====================
void enableMotors(bool enable) {
	motorsEnabled = enable;
	if (ENABLE_ACTIVE_LOW) {
		digitalWrite(PIN_ENABLE, enable ? LOW : HIGH);
	} else {
		digitalWrite(PIN_ENABLE, enable ? HIGH : LOW);
	}
}

void applySpeedToSteppers(float stepsPerSecond) {
	if (!motorsEnabled) {
		stepIntervalMicros = 0;
		return;
	}
	if (stepsPerSecond > (float)MAX_STEPS_PER_SECOND) stepsPerSecond = (float)MAX_STEPS_PER_SECOND;
	if (stepsPerSecond < (float)-MAX_STEPS_PER_SECOND) stepsPerSecond = (float)-MAX_STEPS_PER_SECOND;

	if (fabs(stepsPerSecond) < 1.0f) {
		stepIntervalMicros = 0;
		return;
	}

	bool forward = (stepsPerSecond > 0.0f);
	bool leftDir  = INVERT_DIR_LEFT  ? !forward : forward;
	bool rightDir = INVERT_DIR_RIGHT ? !forward : forward;

	digitalWrite(PIN_DIR_LEFT,  leftDir  ? HIGH : LOW);
	digitalWrite(PIN_DIR_RIGHT, rightDir ? HIGH : LOW);

	float absSps = fabs(stepsPerSecond);
	stepIntervalMicros = (int32_t)(1000000.0f / absSps);
}

void handleSteppers() {
	if (!motorsEnabled) return;
	if (stepIntervalMicros <= 0) return;

	uint32_t now = micros();
	if ((now - lastStepMicros) >= (uint32_t)stepIntervalMicros) {
		lastStepMicros = now;
		digitalWrite(PIN_STEP_LEFT, HIGH);
		digitalWrite(PIN_STEP_RIGHT, HIGH);
		delayMicroseconds(STEP_PULSE_MICROS);
		digitalWrite(PIN_STEP_LEFT, LOW);
		digitalWrite(PIN_STEP_RIGHT, LOW);
	}
}

void updateSpeedRamp(float dtSec) {
	float maxDelta = (float)MAX_ACCEL_STEPS_PER_SEC2 * dtSec;
	float delta = targetStepsPerSecond - currentStepsPerSecond;
	if (delta >  maxDelta) delta =  maxDelta;
	if (delta < -maxDelta) delta = -maxDelta;
	currentStepsPerSecond += delta;
	if (fabs(currentStepsPerSecond) < 1.0f) currentStepsPerSecond = 0.0f;
	applySpeedToSteppers(currentStepsPerSecond);
}

// ===================== Serial Tuning =====================
void printHelp() {
	Serial.println(F("Tuning keys:"));
	Serial.println(F("  w/s: Kp +/- 1.0"));
	Serial.println(F("  e/d: Ki +/- 0.05"));
	Serial.println(F("  r/f: Kd +/- 0.05"));
	Serial.println(F("  y/t: angleTrim +/- 0.2 deg"));
	Serial.println(F("  m:   toggle motors enable"));
	Serial.println(F("  h:   help"));
}

void handleSerial() {
	while (Serial.available() > 0) {
		char c = (char)Serial.read();
		switch (c) {
			case 'w': pidKp += 1.0f; break;
			case 's': pidKp -= 1.0f; break;
			case 'e': pidKi += 0.05f; break;
			case 'd': pidKi -= 0.05f; break;
			case 'r': pidKd += 0.05f; break;
			case 'f': pidKd -= 0.05f; break;
			case 'y': angleTrimDeg += 0.2f; break;
			case 't': angleTrimDeg -= 0.2f; break;
			case 'm': enableMotors(!motorsEnabled); break;
			case 'h': default: printHelp(); break;
		}
	}
}

// ===================== Setup/Loop =====================
void setup() {
	pinMode(PIN_STEP_LEFT, OUTPUT);
	pinMode(PIN_DIR_LEFT, OUTPUT);
	pinMode(PIN_STEP_RIGHT, OUTPUT);
	pinMode(PIN_DIR_RIGHT, OUTPUT);
	pinMode(PIN_ENABLE, OUTPUT);

	enableMotors(false);

	Serial.begin(115200);
	delay(100);
	Serial.println(F("Self-Balancing Robot (Nano + MPU6050 + NEMA17)"));
	printHelp();

	Wire.begin();
	Wire.setClock(400000); // 400 kHz I2C
	initMPU6050();
	delay(100);
	Serial.println(F("Calibrating... Keep robot still"));
	calibrateSensors(800);
	Serial.print(F("Gyro bias (deg/s): ")); Serial.println(gyroBiasDegPerSec, 4);
	lastLoopMicros = micros();

	// Enable motors after calibration when near upright
	if (fabs(angleDeg) < RECOVER_ANGLE_DEG) {
		enableMotors(true);
	}
}

void loop() {
	handleSerial();
	uint32_t now = micros();
	float dt = (now - lastLoopMicros) * 1e-6f;
	if (dt <= 0.0f || dt > 0.05f) dt = 0.005f; // guard against abnormal dt
	lastLoopMicros = now;

	// Read IMU
	int16_t ax, ay, az, gx, gy, gz;
	if (readMPU6050Raw(ax, ay, az, gx, gy, gz)) {
		float gyroY = ((float)gy / 131.0f) - gyroBiasDegPerSec; // deg/s
		gyroY *= ANGLE_SIGN;
		float accAngle = computeAccPitchDeg(ax, az);
		// Complementary filter
		angleDeg = COMPLEMENTARY_ALPHA * (angleDeg + gyroY * dt) + (1.0f - COMPLEMENTARY_ALPHA) * accAngle;
	} else {
		// If read fails, keep integrating with last gyro
	}

	// Safety: disable if fallen
	if (fabs(angleDeg) > FALL_ANGLE_DEG) {
		if (motorsEnabled) {
			enableMotors(false);
			currentStepsPerSecond = 0.0f;
			targetStepsPerSecond = 0.0f;
			stepIntervalMicros = 0;
			Serial.println(F("FALL DETECTED: motors disabled"));
		}
	} else {
		if (!motorsEnabled && fabs(angleDeg) < RECOVER_ANGLE_DEG) {
			enableMotors(true);
		}
	}

	// PID control (only when enabled)
	if (motorsEnabled) {
		float setpoint = angleTrimDeg;
		float error = (angleDeg - setpoint); // positive when leaning forward (by our convention)
		// Integral with clamp
		integralTerm += error * dt;
		if (integralTerm >  INTEGRAL_LIMIT) integralTerm =  INTEGRAL_LIMIT;
		if (integralTerm < -INTEGRAL_LIMIT) integralTerm = -INTEGRAL_LIMIT;
		// Derivative on error (simple); alternatively use -gyroY for better noise behavior
		float derivative = (error - previousError) / dt;
		previousError = error;

		float pidOutput = pidKp * error + pidKi * integralTerm + pidKd * derivative;
		targetStepsPerSecond = OUTPUT_SIGN * pidOutput;
	} else {
		targetStepsPerSecond = 0.0f;
		integralTerm = 0.0f; // reset integral while disabled
	}

	updateSpeedRamp(dt);
	handleSteppers();

	// Periodic debug print
	if (millis() - lastPrintMillis >= 100) {
		lastPrintMillis = millis();
		Serial.print(F("ang=")); Serial.print(angleDeg, 2);
		Serial.print(F("  spdT=")); Serial.print(targetStepsPerSecond, 0);
		Serial.print(F("  spdC=")); Serial.print(currentStepsPerSecond, 0);
		Serial.print(F("  Kp=")); Serial.print(pidKp, 2);
		Serial.print(F(" Ki=")); Serial.print(pidKi, 3);
		Serial.print(F(" Kd=")); Serial.print(pidKd, 3);
		Serial.print(F("  trim=")); Serial.print(angleTrimDeg, 2);
		Serial.print(F("  en=")); Serial.println(motorsEnabled ? 1 : 0);
	}
}

