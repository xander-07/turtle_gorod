#include <Arduino.h>
#include <math.h>

// ================================================================
// turtle_gorod — нижний уровень
// Arduino Uno + ZK-5AD + 2x JGA25-370B с квадратурными энкодерами
//
// UART 115200:
//   SET_WHEELS_SPEED <left_mm_s> <right_mm_s>
//   SET_POSE <x_mm> <y_mm> <theta_rad>
//   SET_COEFF <kp> <ki> <kd> <kff>
//   SET_PWM <leftA> <leftB> <rightA> <rightB>
//   STOP
//   PING
//
// Телеметрия:
//   TEL,<ms>,<x_mm>,<y_mm>,<theta_rad>,<encL>,<encR>,
//       <vL_mm_s>,<vR_mm_s>,<targetL>,<targetR>,<watchdog>
// ================================================================

#define LEFT_MOTOR_A   9
#define LEFT_MOTOR_B   6
#define RIGHT_MOTOR_A  5
#define RIGHT_MOTOR_B  10

#define LEFT_ENCODER_A   2
#define LEFT_ENCODER_B   3
#define RIGHT_ENCODER_A  12
#define RIGHT_ENCODER_B  11

// Физическое направление приводов и энкодеров подтверждено стендовым тестом.
const int8_t MOTOR_DIR_L = +1;
const int8_t MOTOR_DIR_R = -1;
const int8_t ENC_DIR_L   = +1;
const int8_t ENC_DIR_R   = -1;

// Геометрия.
// Эффективный диаметр откалиброван дорожным тестом 2026-09-09:
// реальный путь 1000 мм, по энкодерам при 69 мм получалось ~1147.45 мм.
// 69.0 * 1000 / 1147.45 = 60.13 мм.
const float WHEEL_DIAMETER_MM = 60.13f;

// Геометрическое расстояние между центрами левого и правого ведущих колес
// измерено на роботе 2026-09-09: 245 мм.
// После этого значения эффективная база будет уточнена многократным разворотом
// на полу, так как проскальзывание может немного менять ее кинематическое значение.
const float WHEEL_BASE_MM     = 245.0f;

// Прямое измерение 2026-09-09: ровно 10 оборотов каждого колеса.
// Левое: 8974 тика / 10 = 897.4 тика/оборот.
// Правое: 8988 тиков / 10 = 898.8 тика/оборот.
const float LEFT_ENCODER_TICKS_PER_WHEEL_REV  = 897.4f;
const float RIGHT_ENCODER_TICKS_PER_WHEEL_REV = 898.8f;

const float LEFT_TICK_TO_MM =
    (PI * WHEEL_DIAMETER_MM) / LEFT_ENCODER_TICKS_PER_WHEEL_REV;
const float RIGHT_TICK_TO_MM =
    (PI * WHEEL_DIAMETER_MM) / RIGHT_ENCODER_TICKS_PER_WHEEL_REV;

const uint16_t CONTROL_PERIOD_MS   = 20;   // 50 Hz
const uint16_t TELEMETRY_PERIOD_MS = 50;   // 20 Hz
const uint16_t COMMAND_TIMEOUT_MS  = 350;
const float SPEED_FILTER_ALPHA     = 0.45f;

float pidKp  = 1.1f;
float pidKi  = 1.3f;
float pidKd  = 0.01f;
float pidKff = 0.25f;

volatile long leftEncoder = 0;
volatile long rightEncoder = 0;
volatile uint8_t prevL = 0;
volatile uint8_t prevR = 0;

static const int8_t QDEC[16] = {
   0, -1, +1,  0,
  +1,  0,  0, -1,
  -1,  0,  0, +1,
   0, +1, -1,  0
};

static inline uint8_t readLeftAB() {
  const uint8_t a = (PIND >> 2) & 1;
  const uint8_t b = (PIND >> 3) & 1;
  return a | (b << 1);
}

static inline uint8_t readRightAB() {
  const uint8_t a = (PINB >> 4) & 1;  // D12
  const uint8_t b = (PINB >> 3) & 1;  // D11
  return a | (b << 1);
}

static inline void updateLeft() {
  const uint8_t cur = readLeftAB();
  const uint8_t idx = (prevL << 2) | cur;
  leftEncoder += ENC_DIR_L * QDEC[idx];
  prevL = cur;
}

static inline void updateRight() {
  const uint8_t cur = readRightAB();
  const uint8_t idx = (prevR << 2) | cur;
  rightEncoder += ENC_DIR_R * QDEC[idx];
  prevR = cur;
}

void isrLeftA() { updateLeft(); }
void isrLeftB() { updateLeft(); }
ISR(PCINT0_vect) { updateRight(); }

static inline void snapshotEncoders(long &left, long &right) {
  noInterrupts();
  left = leftEncoder;
  right = rightEncoder;
  interrupts();
}

void setMotorsPWM(int leftA, int leftB, int rightA, int rightB) {
  analogWrite(LEFT_MOTOR_A,  constrain(leftA,  0, 255));
  analogWrite(LEFT_MOTOR_B,  constrain(leftB,  0, 255));
  analogWrite(RIGHT_MOTOR_A, constrain(rightA, 0, 255));
  analogWrite(RIGHT_MOTOR_B, constrain(rightB, 0, 255));
}

void setWheelSignedPWM(int pwmLeft, int pwmRight) {
  pwmLeft  = constrain(pwmLeft,  -255, 255) * MOTOR_DIR_L;
  pwmRight = constrain(pwmRight, -255, 255) * MOTOR_DIR_R;

  const int leftA  = pwmLeft  < 0 ? -pwmLeft : 0;
  const int leftB  = pwmLeft  >= 0 ? pwmLeft : 0;
  const int rightA = pwmRight < 0 ? -pwmRight : 0;
  const int rightB = pwmRight >= 0 ? pwmRight : 0;

  setMotorsPWM(leftA, leftB, rightA, rightB);
}

float targetLeftMmS = 0.0f;
float targetRightMmS = 0.0f;
float measuredLeftMmS = 0.0f;
float measuredRightMmS = 0.0f;

float xMm = 0.0f;
float yMm = 0.0f;
float thetaRad = 0.0f;

long lastControlLeftEncoder = 0;
long lastControlRightEncoder = 0;

uint32_t lastControlMs = 0;
uint32_t lastTelemetryMs = 0;
uint32_t lastCommandMs = 0;

bool rawPwmMode = false;
bool watchdogActive = true;

int rawLeftA = 0;
int rawLeftB = 0;
int rawRightA = 0;
int rawRightB = 0;

float errIntL = 0.0f;
float errIntR = 0.0f;
float prevErrL = 0.0f;
float prevErrR = 0.0f;

void resetPid() {
  errIntL = errIntR = 0.0f;
  prevErrL = prevErrR = 0.0f;
}

void stopMotion() {
  targetLeftMmS = 0.0f;
  targetRightMmS = 0.0f;
  rawPwmMode = false;
  resetPid();
  setMotorsPWM(0, 0, 0, 0);
}

void updateControl(float dt) {
  long encL, encR;
  snapshotEncoders(encL, encR);

  const long dTicksL = encL - lastControlLeftEncoder;
  const long dTicksR = encR - lastControlRightEncoder;
  lastControlLeftEncoder = encL;
  lastControlRightEncoder = encR;

  const float dLeftMm  = dTicksL * LEFT_TICK_TO_MM;
  const float dRightMm = dTicksR * RIGHT_TICK_TO_MM;

  const float instLeft  = dLeftMm / dt;
  const float instRight = dRightMm / dt;

  measuredLeftMmS =
      SPEED_FILTER_ALPHA * instLeft +
      (1.0f - SPEED_FILTER_ALPHA) * measuredLeftMmS;
  measuredRightMmS =
      SPEED_FILTER_ALPHA * instRight +
      (1.0f - SPEED_FILTER_ALPHA) * measuredRightMmS;

  const float dS = 0.5f * (dLeftMm + dRightMm);
  const float dTheta = (dRightMm - dLeftMm) / WHEEL_BASE_MM;
  const float thetaMid = thetaRad + 0.5f * dTheta;

  xMm += dS * cos(thetaMid);
  yMm += dS * sin(thetaMid);
  thetaRad += dTheta;
  thetaRad = atan2(sin(thetaRad), cos(thetaRad));

  const uint32_t now = millis();
  if ((uint32_t)(now - lastCommandMs) > COMMAND_TIMEOUT_MS) {
    if (!watchdogActive) {
      stopMotion();
    }
    watchdogActive = true;
    setMotorsPWM(0, 0, 0, 0);
    return;
  }
  watchdogActive = false;

  if (rawPwmMode) {
    setMotorsPWM(rawLeftA, rawLeftB, rawRightA, rawRightB);
    return;
  }

  const float integralLimit = 300.0f;
  const float stopEps = 1.0f;

  float errL = targetLeftMmS - measuredLeftMmS;
  float errR = targetRightMmS - measuredRightMmS;

  if (fabs(targetLeftMmS) < stopEps) {
    errL = 0.0f;
    errIntL = 0.0f;
    prevErrL = 0.0f;
  } else {
    errIntL += errL * dt;
    errIntL = constrain(errIntL, -integralLimit, integralLimit);
  }

  if (fabs(targetRightMmS) < stopEps) {
    errR = 0.0f;
    errIntR = 0.0f;
    prevErrR = 0.0f;
  } else {
    errIntR += errR * dt;
    errIntR = constrain(errIntR, -integralLimit, integralLimit);
  }

  const float dErrL = (errL - prevErrL) / dt;
  const float dErrR = (errR - prevErrR) / dt;
  prevErrL = errL;
  prevErrR = errR;

  float outL =
      pidKp * errL +
      pidKi * errIntL +
      pidKd * dErrL +
      pidKff * targetLeftMmS;

  float outR =
      pidKp * errR +
      pidKi * errIntR +
      pidKd * dErrR +
      pidKff * targetRightMmS;

  if (fabs(targetLeftMmS) < stopEps) outL = 0.0f;
  if (fabs(targetRightMmS) < stopEps) outR = 0.0f;

  setWheelSignedPWM((int)round(outL), (int)round(outR));
}

char rxLine[128];
uint8_t rxLen = 0;

char *nextToken(char **savePtr) {
  return strtok_r(NULL, " ", savePtr);
}

bool parseFloatToken(char **savePtr, float &value) {
  char *t = nextToken(savePtr);
  if (!t) return false;
  value = atof(t);
  return true;
}

bool parseIntToken(char **savePtr, int &value) {
  char *t = nextToken(savePtr);
  if (!t) return false;
  value = atoi(t);
  return true;
}

void processLine(char *line) {
  char *savePtr = nullptr;
  char *cmd = strtok_r(line, " ", &savePtr);
  if (!cmd) return;

  if (strcmp(cmd, "PING") == 0) {
    Serial.println(F("PONG"));
    return;
  }

  if (strcmp(cmd, "STOP") == 0) {
    lastCommandMs = millis();
    stopMotion();
    Serial.println(F("OK STOP"));
    return;
  }

  if (strcmp(cmd, "SET_WHEELS_SPEED") == 0) {
    float l, r;
    if (!parseFloatToken(&savePtr, l) || !parseFloatToken(&savePtr, r)) {
      Serial.println(F("ERR SET_WHEELS_SPEED"));
      return;
    }
    targetLeftMmS = l;
    targetRightMmS = r;
    rawPwmMode = false;
    lastCommandMs = millis();
    Serial.println(F("OK SET_WHEELS_SPEED"));
    return;
  }

  if (strcmp(cmd, "SET_POSE") == 0) {
    float x, y, th;
    if (!parseFloatToken(&savePtr, x) ||
        !parseFloatToken(&savePtr, y) ||
        !parseFloatToken(&savePtr, th)) {
      Serial.println(F("ERR SET_POSE"));
      return;
    }
    xMm = x;
    yMm = y;
    thetaRad = th;
    Serial.println(F("OK SET_POSE"));
    return;
  }

  if (strcmp(cmd, "SET_COEFF") == 0) {
    float kp, ki, kd, kff;
    if (!parseFloatToken(&savePtr, kp) ||
        !parseFloatToken(&savePtr, ki) ||
        !parseFloatToken(&savePtr, kd) ||
        !parseFloatToken(&savePtr, kff)) {
      Serial.println(F("ERR SET_COEFF"));
      return;
    }
    pidKp = kp;
    pidKi = ki;
    pidKd = kd;
    pidKff = kff;
    resetPid();
    Serial.println(F("OK SET_COEFF"));
    return;
  }

  if (strcmp(cmd, "SET_PWM") == 0) {
    int la, lb, ra, rb;
    if (!parseIntToken(&savePtr, la) ||
        !parseIntToken(&savePtr, lb) ||
        !parseIntToken(&savePtr, ra) ||
        !parseIntToken(&savePtr, rb)) {
      Serial.println(F("ERR SET_PWM"));
      return;
    }
    rawLeftA  = constrain(la, 0, 255);
    rawLeftB  = constrain(lb, 0, 255);
    rawRightA = constrain(ra, 0, 255);
    rawRightB = constrain(rb, 0, 255);
    rawPwmMode = true;
    lastCommandMs = millis();
    Serial.println(F("OK SET_PWM"));
    return;
  }

  Serial.println(F("ERR UNKNOWN"));
}

void pollSerial() {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      rxLine[rxLen] = '\0';
      if (rxLen > 0) processLine(rxLine);
      rxLen = 0;
      continue;
    }

    if (rxLen < sizeof(rxLine) - 1) {
      rxLine[rxLen++] = c;
    } else {
      rxLen = 0;
      Serial.println(F("ERR LINE_TOO_LONG"));
    }
  }
}

void publishTelemetry() {
  long encL, encR;
  snapshotEncoders(encL, encR);

  Serial.print(F("TEL,"));
  Serial.print(millis());
  Serial.print(',');
  Serial.print(xMm, 2);
  Serial.print(',');
  Serial.print(yMm, 2);
  Serial.print(',');
  Serial.print(thetaRad, 5);
  Serial.print(',');
  Serial.print(encL);
  Serial.print(',');
  Serial.print(encR);
  Serial.print(',');
  Serial.print(measuredLeftMmS, 2);
  Serial.print(',');
  Serial.print(measuredRightMmS, 2);
  Serial.print(',');
  Serial.print(targetLeftMmS, 2);
  Serial.print(',');
  Serial.print(targetRightMmS, 2);
  Serial.print(',');
  Serial.println(watchdogActive ? 1 : 0);
}

void setup() {
  Serial.begin(115200);

  pinMode(LEFT_MOTOR_A, OUTPUT);
  pinMode(LEFT_MOTOR_B, OUTPUT);
  pinMode(RIGHT_MOTOR_A, OUTPUT);
  pinMode(RIGHT_MOTOR_B, OUTPUT);

  pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);

  prevL = readLeftAB();
  prevR = readRightAB();

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), isrLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_B), isrLeftB, CHANGE);

  PCICR  |= _BV(PCIE0);
  PCMSK0 |= _BV(PCINT3) | _BV(PCINT4);

  setMotorsPWM(0, 0, 0, 0);

  long encL, encR;
  snapshotEncoders(encL, encR);
  lastControlLeftEncoder = encL;
  lastControlRightEncoder = encR;

  lastControlMs = millis();
  lastTelemetryMs = millis();
  lastCommandMs = millis();

  Serial.println(F("READY turtle_gorod_low_level_v2.3_wheelbase_measured"));
}

void loop() {
  pollSerial();

  const uint32_t now = millis();

  if ((uint32_t)(now - lastControlMs) >= CONTROL_PERIOD_MS) {
    const uint32_t elapsed = now - lastControlMs;
    lastControlMs = now;
    updateControl(elapsed / 1000.0f);
  }

  if ((uint32_t)(now - lastTelemetryMs) >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = now;
    publishTelemetry();
  }
}
