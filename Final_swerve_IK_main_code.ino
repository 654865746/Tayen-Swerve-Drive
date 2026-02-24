// Full integrated swerve: encoders (MT6835 SPI), PWM RC inputs, IK, unwrap, flip, drive-servos
// Created by TAYEN BROEMSER 
#include <SPI.h>
#include <CytronMotorDriver.h>
#include <Servo.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//---------------USER SET PARAMETERS---------------
// Chip Select Pins
#define CS_FR 30 // Front Right
#define CS_BL 35 // Back Left
#define CS_BR 34 // Back Right
#define CS_FL 31 // Front Left

// PWM Input Pins
#define PWM_CH1 23
#define PWM_CH2 22
// #define PWM_CH3 15
#define PWM_CH4 14

// Steering motor (Cytron) pins
#define PWM_FR 4
#define DIR_FR 5
#define PWM_BL 9
#define DIR_BL 8
#define PWM_BR 3
#define DIR_BR 6
#define PWM_FL 2
#define DIR_FL 7

#define BUZZER_PIN 41  // Teensy digital pin connected to active buzzer

// Drive motor (servo-style) pins (microsecond PWM)
const int FL_DM_PIN = 33;
const int FR_DM_PIN = 36;
const int BL_DM_PIN = 25;
const int BR_DM_PIN = 10;


// ENCODER OFFSETS
const float offsetFR = 21.0f;
const float offsetBL = 8.0f;
const float offsetBR = 157.0f;
const float offsetFL = 197.0f;

// PID GAINS
float Kp_angle = 7.0f;
float Ki_angle = 0.0f;
float Kd_angle = 0.8;

// SWERVE GEOMETRY -->> ROBOT DIMENSIONS IN METERS
const float L = 0.381f; // wheelbase
const float W = 0.381f; // trackwidth
const float R = sqrt(L*L + W*W);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const int SERVO_NEUTRAL = 1500;
const int SERVO_RANGE = 500; // 1500 +/- 500 => 1000..2000

// PWM input capture state (RC inputs)
volatile uint32_t pwmCh1 = 0;
volatile uint32_t pwmCh2 = 0;
volatile uint32_t pwmCh4 = 0;

volatile uint32_t lastRiseCh1 = 0;
volatile uint32_t lastRiseCh2 = 0;
volatile uint32_t lastRiseCh4 = 0;

// Encoder constants
#define MT6835_MAX 2097152.0f // 2^21
const float alpha = 0.1f;   // EMA smoothing factor

// Smoothed filtered angles (wrapped 0..360) for each encoder
float filteredFR = 0, filteredBL = 0, filteredBR = 0, filteredFL = 0;

// Unwrapped encoder angles and last wrapped readings
float unwrappedFR = 0, unwrappedBL = 0, unwrappedBR = 0, unwrappedFL = 0;
float lastWrappedFR = 0, lastWrappedBL = 0, lastWrappedBR = 0, lastWrappedFL = 0;

// Unwrapped joystick target
float unwrappedTarget = 0;
float lastWrappedTarget = 0;

// Target angles for each wheel (base from IK in -180..180)
float baseTargetFR = 0, baseTargetBL = 0, baseTargetBR = 0, baseTargetFL = 0;

// Continuous targets (after aligning to encoder and optional flip) — used by PID
float targetFR_cont = 0, targetBL_cont = 0, targetBR_cont = 0, targetFL_cont = 0;

// previous target saved to hold last when joystick neutral
float prevTargetFR = 0, prevTargetBL = 0, prevTargetBR = 0, prevTargetFL = 0;

// PID state for steering control (per wheel)
float prevErrorFR = 0, integralFR = 0;
float prevErrorBL = 0, integralBL = 0;
float prevErrorBR = 0, integralBR = 0;
float prevErrorFL = 0, integralFL = 0;

// SteeringMotor driver objects
CytronMD motorFR(PWM_DIR, PWM_FR, DIR_FR);
CytronMD motorBL(PWM_DIR, PWM_BL, DIR_BL);
CytronMD motorBR(PWM_DIR, PWM_BR, DIR_BR);
CytronMD motorFL(PWM_DIR, PWM_FL, DIR_FL);

// Drive servos
Servo FR_DM, BR_DM, BL_DM, FL_DM;

// Helper: SPI register command building for MT6835
inline uint8_t readCmdHi(uint16_t reg) { return (0x3 << 4) | ((reg >> 8) & 0x0F); }
inline uint8_t readCmdLo(uint16_t reg) { return reg & 0xFF; }

// ------------------------------ Encoder read: wrap-aware EMA (SPI-safe) ------------------------------
// ------------------------------ Encoder read: wrap-aware EMA with safety ------------------------------
float readEncoderAngle(int csPin, float &filteredValue, float offset, float &lastWrapped, float &unwrapped) {
    uint8_t b1 = 0, b2 = 0, b3 = 0, crc_read = 0;

    digitalWrite(csPin, LOW);

    // Simple SPI read sequence
    b1 = SPI.transfer(readCmdHi(0x003));
    b1 = SPI.transfer(readCmdLo(0x003));
    b1 = SPI.transfer(0x00);

    b2 = SPI.transfer(readCmdHi(0x004));
    b2 = SPI.transfer(readCmdLo(0x004));
    b2 = SPI.transfer(0x00);

    b3 = SPI.transfer(readCmdHi(0x005));
    b3 = SPI.transfer(readCmdLo(0x005));
    b3 = SPI.transfer(0x00);

    crc_read = SPI.transfer(readCmdHi(0x006));
    crc_read = SPI.transfer(readCmdLo(0x006));
    crc_read = SPI.transfer(0x00);

    digitalWrite(csPin, HIGH);

    // Reconstruct angle
    uint32_t raw24 = ((uint32_t)b1 << 16) | ((uint32_t)b2 << 8) | b3;
    uint32_t angle21 = (raw24 >> 3) & 0x1FFFFF;
    float degrees = (360.0f * (float)angle21) / MT6835_MAX;

    // Apply offset and wrap
    degrees -= offset;
    degrees = normalize360(degrees);

    // --- Sanity clamp to prevent huge jumps ---
    float deltaRaw = degrees - filteredValue;
    if (deltaRaw > 180.0f) deltaRaw -= 360.0f;
    else if (deltaRaw < -180.0f) deltaRaw += 360.0f;

    // Maximum allowed delta per loop (protects PID from bad read)
    const float MAX_DELTA = 20.0f; // degrees per loop
    if (deltaRaw > MAX_DELTA) deltaRaw = MAX_DELTA;
    else if (deltaRaw < -MAX_DELTA) deltaRaw = -MAX_DELTA;

    // optional tiny deadband to stop micro-jitter
    if(fabs(deltaRaw) < 0.05f) deltaRaw = 0.0f;

    filteredValue += (1.0f - alpha) * deltaRaw;
    filteredValue = normalize360(filteredValue);

    // --- Unwrap to continuous ---
    float delta = filteredValue - lastWrapped;
    if(delta > 180.0f) delta -= 360.0f;
    else if(delta < -180.0f) delta += 360.0f;

    unwrapped += delta;
    lastWrapped = filteredValue;

    // --- Final sanity: ensure unwrapped is finite ---
    if (!isfinite(unwrapped)) unwrapped = filteredValue;

    return unwrapped;
}

// ------------------------------ PWM input ISRs ------------------------------
void isrCh1() {
    if (digitalRead(PWM_CH1) == HIGH) lastRiseCh1 = micros();
    else pwmCh1 = micros() - lastRiseCh1;
}
void isrCh2() {
    if (digitalRead(PWM_CH2) == HIGH) lastRiseCh2 = micros();
    else pwmCh2 = micros() - lastRiseCh2;
}
void isrCh4() {
    if (digitalRead(PWM_CH4) == HIGH) lastRiseCh4 = micros();
    else pwmCh4 = micros() - lastRiseCh4;
}

// ------------------------------ Utilities ------------------------------
float normalize360(float ang) {
    // normalize to [0,360)
    while (ang < 0) ang += 360.0f;
    while (ang >= 360.0f) ang -= 360.0f;
    return ang;
}
float wrap180(float ang) {
    // normalize to (-180,180]
    ang = fmodf(ang + 180.0f, 360.0f);
    if (ang < 0) ang += 360.0f;
    return ang - 180.0f;
}

// Unwrap a wrapped (0..360) reading into a continuous variable
float unwrapAngle(float wrapped, float &lastWrapped, float &unwrapped) {
    float delta = wrapped - lastWrapped;
    if (delta > 180.0f) delta -= 360.0f;
    else if (delta < -180.0f) delta += 360.0f;
    unwrapped += delta;
    lastWrapped = wrapped;
    return unwrapped;
}

// Compute shortest diff between two continuous angles (target - current)
float continuousDiff(float target, float current) {
    return target - current; // both should be continuous/unwrapped
}

// PID compute (operates on continuous angles; output in -255..255)
float computeSteerPID(float target_cont, float current_cont, float &prevError, float &integral) {
    float err = continuousDiff(target_cont, current_cont);
    integral += err;
    float derivative = err - prevError;
    prevError = err;
    float out = Kp_angle * err + Ki_angle * integral + Kd_angle * derivative;
    out = constrain(out, -255.0f, 255.0f);
    return out;
}

// Apply deadzone/clamp for PWM inputs
uint32_t applyDeadzone(uint32_t pwmValue, uint32_t deadCenter = 1500, uint32_t deadZone = 20, uint32_t minVal = 950, uint32_t maxVal = 2050) {
    if (pwmValue > maxVal || pwmValue < minVal) return deadCenter;
    if (abs((int)pwmValue - (int)deadCenter) <= (int)deadZone) return deadCenter;
    return pwmValue;
}

// Map rc pwm 1000..2000 to -1..1 float
float pwmToUnit(int pwm) {
    // clamp
    if (pwm < 1000) pwm = 1000;
    if (pwm > 2000) pwm = 2000;
    return ((float)pwm - 1500.0f) / 500.0f; // -1..1
}

// clamp microsec for servo
int clampServo(int us) {
    if (us < 1000) return 1000;
    if (us > 2000) return 2000;
    return us;
}

// cytron motor deadzone
int16_t deadzoneClamp(int16_t val, int16_t threshold = 15) {
    if (abs(val) < threshold) return 0; // ignore small outputs
    return val;
}

// ------------------------------ Inverse Kinematics + flip logic ------------------------------
void computeIK_and_apply(float vx, float vy, float omega) {
    const float DEAD_JOYSTICK = 0.05f;
    bool joystickNeutral = (fabs(vx) < DEAD_JOYSTICK && fabs(vy) < DEAD_JOYSTICK && fabs(omega) < DEAD_JOYSTICK);

    if (joystickNeutral) {
        FR_DM.writeMicroseconds(SERVO_NEUTRAL);
        BR_DM.writeMicroseconds(SERVO_NEUTRAL);
        BL_DM.writeMicroseconds(SERVO_NEUTRAL);
        FL_DM.writeMicroseconds(SERVO_NEUTRAL);
        return;
    }

    // Compute normalization factor for rotation so pure rotation is full speed
    float k = sqrt((L/2)*(L/2) + (W/2)*(W/2));

    // Compute wheel velocities with correct rotation contributions
    float vx_FR_total = vx + omega * L/2 / k;
    float vy_FR_total = vy + omega * W/2 / k;

    float vx_BR_total = vx + omega * L/2 / k;
    float vy_BR_total = vy - omega * W/2 / k;

    float vx_BL_total = vx - omega * L/2 / k;
    float vy_BL_total = vy - omega * W/2 / k;

    float vx_FL_total = vx - omega * L/2 / k;
    float vy_FL_total = vy + omega * W/2 / k;

    // Steering angles (-180..180)
    float baseFR = atan2(vy_FR_total, vx_FR_total) * 180.0f / PI;
    float baseBR = atan2(vy_BR_total, vx_BR_total) * 180.0f / PI;
    float baseBL = atan2(vy_BL_total, vx_BL_total) * 180.0f / PI;
    float baseFL = atan2(vy_FL_total, vx_FL_total) * 180.0f / PI;

    baseFR = wrap180(baseFR);
    baseBR = wrap180(baseBR);
    baseBL = wrap180(baseBL);
    baseFL = wrap180(baseFL);

    // Speeds
    float spFR = sqrt(vx_FR_total*vx_FR_total + vy_FR_total*vy_FR_total);
    float spBR = sqrt(vx_BR_total*vx_BR_total + vy_BR_total*vy_BR_total);
    float spBL = sqrt(vx_BL_total*vx_BL_total + vy_BL_total*vy_BL_total);
    float spFL = sqrt(vx_FL_total*vx_FL_total + vy_FL_total*vy_FL_total);

    // Normalize speeds
    float maxS = max(max(spFR, spBR), spBL);
    if (maxS > 1.0f) {
        spFR /= maxS;
        spBR /= maxS;
        spBL /= maxS;
    }

    // Flip logic (unchanged)
    auto chooseTargetAndDrive = [&](float baseAngleWrapped, float unwrappedEncoder, float &targetCont, float &wheelSpeed) -> float {
        float candidate = baseAngleWrapped;
        while (candidate - unwrappedEncoder > 180.0f) candidate -= 360.0f;
        while (candidate - unwrappedEncoder < -180.0f) candidate += 360.0f;

        float diff_no_flip = fabs(candidate - unwrappedEncoder);
        float candidate_flip = candidate + (candidate >= unwrappedEncoder ? -180.0f : 180.0f);
        while (candidate_flip - unwrappedEncoder > 180.0f) candidate_flip -= 360.0f;
        while (candidate_flip - unwrappedEncoder < -180.0f) candidate_flip += 360.0f;
        float diff_flip = fabs(candidate_flip - unwrappedEncoder);

        bool flipChosen = (diff_flip + 0.0001f) < diff_no_flip;
        if (flipChosen) {
            targetCont = candidate_flip;
            wheelSpeed = -wheelSpeed;
        } else {
            // continuous target, unwrapped relative to encoder
            targetCont = unwrappedEncoder + wrap180(candidate - unwrappedEncoder);
        }
        return flipChosen ? -1.0f : 1.0f;
    };

    chooseTargetAndDrive(baseFR, unwrappedFR, targetFR_cont, spFR);
    chooseTargetAndDrive(baseBR, unwrappedBR, targetBR_cont, spBR);
    chooseTargetAndDrive(baseBL, unwrappedBL, targetBL_cont, spBL);
    chooseTargetAndDrive(baseFL, unwrappedFL, targetFL_cont, spFL);

    // Apply to drive servos
    FR_DM.writeMicroseconds(clampServo(SERVO_NEUTRAL + (int)(spFR * SERVO_RANGE)));
    BR_DM.writeMicroseconds(clampServo(SERVO_NEUTRAL + (int)(spBR * SERVO_RANGE)));
    BL_DM.writeMicroseconds(clampServo(SERVO_NEUTRAL + (int)(spBL * SERVO_RANGE)));
    FL_DM.writeMicroseconds(clampServo(SERVO_NEUTRAL + (int)(spFL * SERVO_RANGE)));
}

// --- Three short beeps ---
void playThreeBeeps() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(120);       // beep duration
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);       // space between beeps
  }
}

// ------------------------------ Setup & loop ------------------------------
void setup() {
    Serial.begin(115200);
    SPI.begin();
    // SPI settings: MT6835 needs MODE3 per datasheet
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));

    // CS pins
    pinMode(CS_FR, OUTPUT); digitalWrite(CS_FR, HIGH);
    pinMode(CS_BL, OUTPUT); digitalWrite(CS_BL, HIGH);
    pinMode(CS_BR, OUTPUT); digitalWrite(CS_BR, HIGH);
    pinMode(CS_FL, OUTPUT); digitalWrite(CS_FL, HIGH);

    // RC PWM inputs
    pinMode(PWM_CH1, INPUT);
    pinMode(PWM_CH2, INPUT);
    pinMode(PWM_CH4, INPUT);
    pinMode(BUZZER_PIN, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(PWM_CH1), isrCh1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PWM_CH2), isrCh2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PWM_CH4), isrCh4, CHANGE);

    // Drive servos attach
    FR_DM.attach(FR_DM_PIN);
    BR_DM.attach(BR_DM_PIN);
    BL_DM.attach(BL_DM_PIN);
    FL_DM.attach(FL_DM_PIN);

    // init unwrapped using an initial read so continuity starts reasonable
    delay(100);
    unwrappedFR = readEncoderAngle(CS_FR, filteredFR, offsetFR, lastWrappedFR, unwrappedFR);
    unwrappedBR = readEncoderAngle(CS_BR, filteredBR, offsetBR, lastWrappedBR, unwrappedBR);
    unwrappedBL = readEncoderAngle(CS_BL, filteredBL, offsetBL, lastWrappedBL, unwrappedBL);
    unwrappedFL = readEncoderAngle(CS_FL, filteredFL, offsetFL, lastWrappedFL, unwrappedFL);

    lastWrappedFR = filteredFR; unwrappedFR = filteredFR;
    lastWrappedBR = filteredBR; unwrappedBR = filteredBR;
    lastWrappedBL = filteredBL; unwrappedBL = filteredBL;
    lastWrappedFL = filteredFL; unwrappedFL = filteredFL;

    lastWrappedTarget = 0; unwrappedTarget = 0;

    // neutral drive
    FR_DM.writeMicroseconds(SERVO_NEUTRAL);
    BR_DM.writeMicroseconds(SERVO_NEUTRAL);
    BL_DM.writeMicroseconds(SERVO_NEUTRAL);
    FL_DM.writeMicroseconds(SERVO_NEUTRAL);

    playThreeBeeps();
}

void loop() {
    // Read Encoder Angles
    unwrappedFR = readEncoderAngle(CS_FR, filteredFR, offsetFR, lastWrappedFR, unwrappedFR);
    unwrappedBR = readEncoderAngle(CS_BR, filteredBR, offsetBR, lastWrappedBR, unwrappedBR);
    unwrappedBL = readEncoderAngle(CS_BL, filteredBL, offsetBL, lastWrappedBL, unwrappedBL);
    unwrappedFL = readEncoderAngle(CS_FL, filteredFL, offsetFL, lastWrappedFL, unwrappedFL);

    // Read PWM inputs
    uint32_t ch1_us, ch2_us, ch4_us;
    noInterrupts();
    ch1_us = pwmCh1;
    ch2_us = pwmCh2;
    ch4_us = pwmCh4;
    interrupts();

    // Apply deadzone/clamp
    ch1_us = applyDeadzone(ch1_us);
    ch2_us = applyDeadzone(ch2_us);
    ch4_us = applyDeadzone(ch4_us);

    // Compute joystick target angle (wrapped) and unwrap it
    bool joystickNeutral = (ch1_us == 1500 && ch2_us == 1500 && ch4_us == 1500);

    float jx = pwmToUnit((int)ch1_us); // -1..1
    float jy = pwmToUnit((int)ch2_us);
    float jw = pwmToUnit((int)ch4_us);

    // If joystick neutral, keep previous targets (hold last)
    if (joystickNeutral) {
        // keep previous unwrappedTarget (do not alter)
    } else {
        // Compute target direction from Ch1/Ch2 (atan2 gives -180..180)
        float targetWrapped = atan2(jy, jx) * 180.0f / PI; // CW/CCW per atan2 sign
        // normalize to [-180,180]
        targetWrapped = wrap180(targetWrapped);
        // unwrap joystick target continuously
        // bring targetWrapped close to lastWrappedTarget then accumulate to unwrappedTarget
        float tCandidate = targetWrapped;
        // align candidate near lastWrappedTarget
        while (tCandidate - lastWrappedTarget > 180.0f) tCandidate -= 360.0f;
        while (tCandidate - lastWrappedTarget < -180.0f) tCandidate += 360.0f;
        float deltaT = tCandidate - lastWrappedTarget;
        unwrappedTarget += deltaT;
        lastWrappedTarget = tCandidate;
    }

    // Compute linear vx, vy and rotation omega from joystick magnitude / channels
    // Scale vx/vy/omega to desired units (use -1..1)
    float vx = -jy;
    float vy = jx;
    float omega = jw;

    // Compute IK and apply drive servo outputs (also chooses flip if beneficial)
    computeIK_and_apply(vx, vy, omega);

    // Steering PID: compute using continuous target & unwrapped encoder
    float steerOutFR = computeSteerPID(targetFR_cont, unwrappedFR, prevErrorFR, integralFR);
    float steerOutBR = computeSteerPID(targetBR_cont, unwrappedBR, prevErrorBR, integralBR);
    float steerOutBL = computeSteerPID(targetBL_cont, unwrappedBL, prevErrorBL, integralBL);
    float steerOutFL = computeSteerPID(targetFL_cont, unwrappedFL, prevErrorFL, integralFL);

    // Convert PID outputs to cytron speeds (-255..255) and apply
    motorFR.setSpeed(deadzoneClamp((int16_t)steerOutFR));
    motorBL.setSpeed(-deadzoneClamp((int16_t)steerOutBL));
    motorBR.setSpeed(-deadzoneClamp((int16_t)steerOutBR));
    motorFL.setSpeed(deadzoneClamp((int16_t)steerOutFL));

    //Serial.print("FR: "); Serial.print(unwrappedFR,2);
    //Serial.print(" | BR: "); Serial.print(unwrappedBR,2);
    //Serial.print(" | BL: "); Serial.print(unwrappedBL,2);
    //Serial.print(" | FL: "); Serial.println(unwrappedFL,2);

    delay(15); // ~66Hz
}
