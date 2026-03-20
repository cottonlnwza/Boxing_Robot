#include <Arduino.h>
#include <ps5Controller.h>

// ==========================================================
//                 3 Omniwheel Drive + Servo
//                 ESP32 Core 3.x + PS5
// ==========================================================

// ---------------- Motor Pins ----------------
const int M1_IN1 = 27;   // ล้อหน้าซ้าย
const int M1_IN2 = 26;

const int M2_IN1 = 12;   // ล้อหน้าขวา
const int M2_IN2 = 14;

const int M3_IN1 = 25;   // ล้อหลัง
const int M3_IN2 = 33;

// ---------------- Servo Pin ----------------
const int SERVO_PIN = 13;

// ---------------- PWM Config ----------------
const int PWM_FREQ = 20000;
const int PWM_RES  = 8;

int MAX_SPEED = 255;

// ---------------- Offset ----------------
float offset1 = 1.0;
float offset2 = 1.0;
float offset3 = 1.0;

// ---------------- Motor direction ----------------
const bool M1_FORWARD_ON_IN2 = false;
const bool M2_FORWARD_ON_IN2 = false;
const bool M3_FORWARD_ON_IN2 = false;

// กลับทิศหมุน: ซ้ายต้องหมุนซ้าย ขวาต้องหมุนขวา
const float ROTATE_SIGN = -1.0;

// ---------------- Control tuning ----------------
const float DEADZONE_FB   = 0.18;
const float DEADZONE_DIAG = 0.22;
const float DEADZONE_ROT  = 0.20;

const float EXPO_FB   = 2.2;
const float EXPO_DIAG = 2.5;
const float EXPO_ROT  = 2.0;

const float AXIS_ASSIST_RATIO = 1.35;

// ==========================================================
// Servo Config
// ==========================================================
const int SERVO_FREQ = 50;
const int SERVO_RES = 12;

const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2400;

int servoAngle = 180;                  // เริ่มกลาง
const int SERVO_STEP = 2;             // ขยับทีละ 2 องศา
const int SERVO_MIN_ANGLE = 0;
const int SERVO_MAX_ANGLE = 180;

unsigned long lastServoMove = 0;
const int SERVO_MOVE_INTERVAL = 20;   // ms

// ==========================================================
float applyDeadzoneAndExpo(float v, float dz, float expo) {
  if (fabs(v) < dz) return 0.0;

  float sign = (v >= 0) ? 1.0 : -1.0;
  float mag = fabs(v);

  mag = (mag - dz) / (1.0 - dz);
  mag = pow(mag, expo);

  return sign * mag;
}

// ==========================================================
void motorDriveCustom(int pinIN1, int pinIN2, int speed, bool forwardOnIN2) {
  speed = constrain(speed, -255, 255);

  if (speed == 0) {
    ledcWrite(pinIN1, 0);
    ledcWrite(pinIN2, 0);
    return;
  }

  if (speed > 0) {
    if (forwardOnIN2) {
      ledcWrite(pinIN1, 0);
      ledcWrite(pinIN2, speed);
    } else {
      ledcWrite(pinIN1, speed);
      ledcWrite(pinIN2, 0);
    }
  } else {
    int pwm = -speed;
    if (forwardOnIN2) {
      ledcWrite(pinIN1, pwm);
      ledcWrite(pinIN2, 0);
    } else {
      ledcWrite(pinIN1, 0);
      ledcWrite(pinIN2, pwm);
    }
  }
}

void stopAllMotors() {
  motorDriveCustom(M1_IN1, M1_IN2, 0, M1_FORWARD_ON_IN2);
  motorDriveCustom(M2_IN1, M2_IN2, 0, M2_FORWARD_ON_IN2);
  motorDriveCustom(M3_IN1, M3_IN2, 0, M3_FORWARD_ON_IN2);
}

// ==========================================================
// Servo Function
// ==========================================================
int angleToDuty(int angle) {
  angle = constrain(angle, 0, 180);
  int pulseUs = map(angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);

  // 50Hz = 20,000 us
  int maxDuty = (1 << SERVO_RES) - 1;
  int duty = (pulseUs * maxDuty) / 20000;

  return duty;
}

void servoWriteAngle(int angle) {
  angle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  servoAngle = angle;
  ledcWrite(SERVO_PIN, angleToDuty(servoAngle));
}

// ==========================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  bool ok = true;

  ok &= ledcAttach(M1_IN1, PWM_FREQ, PWM_RES);
  ok &= ledcAttach(M1_IN2, PWM_FREQ, PWM_RES);
  ok &= ledcAttach(M2_IN1, PWM_FREQ, PWM_RES);
  ok &= ledcAttach(M2_IN2, PWM_FREQ, PWM_RES);
  ok &= ledcAttach(M3_IN1, PWM_FREQ, PWM_RES);
  ok &= ledcAttach(M3_IN2, PWM_FREQ, PWM_RES);

  // Servo attach
  ok &= ledcAttach(SERVO_PIN, SERVO_FREQ, SERVO_RES);

  if (!ok) {
    Serial.println("LEDC attach failed");
    while (1) delay(1000);
  }

  stopAllMotors();
  servoWriteAngle(servoAngle);

  ps5.begin("A0:FA:9C:B3:1E:C5");
  Serial.println("Drive + Servo Ready. Waiting for PS5...");
}

// ==========================================================
void loop() {
  if (!ps5.isConnected()) {
    stopAllMotors();
    return;
  }

  // =========================
  // Drive
  // =========================
  float rawLX = ps5.LStickX() / 127.0f;
  float rawLY = ps5.LStickY() / 127.0f;
  float rawRX = ps5.RStickX() / 127.0f;

  float diagCmd = applyDeadzoneAndExpo(rawLX, DEADZONE_DIAG, EXPO_DIAG);
  float fbCmd   = applyDeadzoneAndExpo(rawLY, DEADZONE_FB,   EXPO_FB);
  float rotCmd  = applyDeadzoneAndExpo(rawRX, DEADZONE_ROT,  EXPO_ROT);

  if (fabs(fbCmd) > fabs(diagCmd) * AXIS_ASSIST_RATIO) {
    diagCmd = 0;
  }

  float x = 0.0;
  float y = 0.0;
  float r = rotCmd * ROTATE_SIGN;

  y += fbCmd;

  if (diagCmd != 0) {
    x += diagCmd * 0.707f;
    y += diagCmd * 0.707f;
  }

  float M1 = (-0.5f * x) + (-0.866f * y) + r;
  float M2 = (-0.5f * x) + ( 0.866f * y) + r;
  float M3 = ( 1.0f * x) + r;

  float maxVal = max(max(fabs(M1), fabs(M2)), fabs(M3));
  if (maxVal > 1.0f) {
    M1 /= maxVal;
    M2 /= maxVal;
    M3 /= maxVal;
  }

  M1 *= MAX_SPEED * offset1;
  M2 *= MAX_SPEED * offset2;
  M3 *= MAX_SPEED * offset3;

  motorDriveCustom(M1_IN1, M1_IN2, (int)M1, M1_FORWARD_ON_IN2);
  motorDriveCustom(M2_IN1, M2_IN2, (int)M2, M2_FORWARD_ON_IN2);
  motorDriveCustom(M3_IN1, M3_IN2, (int)M3, M3_FORWARD_ON_IN2);

  // =========================
  // Servo: R1 / R2
  // =========================
  bool servoUp = ps5.R1();
  bool servoDown = ps5.R2();

  unsigned long now = millis();
  if (now - lastServoMove >= SERVO_MOVE_INTERVAL) {
    if (servoUp && !servoDown) {
      servoWriteAngle(servoAngle + SERVO_STEP);
    } else if (servoDown && !servoUp) {
      servoWriteAngle(servoAngle - SERVO_STEP);
    }
    lastServoMove = now;
  }

  // =========================
  // Debug
  // =========================
  Serial.print("diag=");
  Serial.print(diagCmd, 3);
  Serial.print(" fb=");
  Serial.print(fbCmd, 3);
  Serial.print(" rot=");
  Serial.print(rotCmd, 3);
  Serial.print(" | M1=");
  Serial.print(M1, 1);
  Serial.print(" M2=");
  Serial.print(M2, 1);
  Serial.print(" M3=");
  Serial.print(M3, 1);
  Serial.print(" | Servo=");
  Serial.println(servoAngle);

  delay(10);
}
