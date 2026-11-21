#include "config.h"

// Definition Interrupt Counters
volatile long tick_count_left = 0;
volatile long tick_count_right = 0;

// ================= INTERRUPT SERVICE ROUTINES (1X DECODING) =================
void IRAM_ATTR isr_encoder_left() {
  if (digitalRead(ENCODER_L_B) == HIGH) {
    tick_count_left++;
  } else {
    tick_count_left--;
  }
}

void IRAM_ATTR isr_encoder_right() {
  if (digitalRead(ENCODER_R_B) == HIGH) {
    tick_count_right--;
  } else {
    tick_count_right++;
  }
}

void initOdometry() {
  pinMode(ENCODER_L_A, INPUT_PULLUP);
  pinMode(ENCODER_L_B, INPUT); // GPIO 35 Input-Only
  pinMode(ENCODER_R_A, INPUT_PULLUP);
  pinMode(ENCODER_R_B, INPUT); // GPIO 36 Input-Only

  attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), isr_encoder_left, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), isr_encoder_right, RISING);
}

float normalizeAngleRad(float a) {
  while (a >  PI) a -= 2.0f * PI;
  while (a < -PI) a += 2.0f * PI;
  return a;
}

float normalizeAngleDeg(float a) {
  while (a >  180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

// ================= KALIBRASI GYRO Z (400 SAMPEL) =================
void calibrateGyroZ() {
  M5.Display.clear();
  M5.Display.setCursor(0, 40);
  M5.Display.setTextColor(YELLOW, BLACK);
  M5.Display.println(" KALIBRASI GYRO...");
  M5.Display.println(" Jangan gerakkan!");

  float sumZ = 0.0f;
  const int samples = 400;
  for (int i = 0; i < samples; i++) {
    M5.Imu.update();
    auto data = M5.Imu.getImuData();
    sumZ += data.gyro.z;
    delay(3);
  }

  portENTER_CRITICAL(&stateMux);
  gyroZOffset   = sumZ / (float)samples;
  headingDeg    = 0.0f;
  targetHeading = 0.0f;
  yawErrSum     = 0.0f;
  lastYawErr    = 0.0f;
  portEXIT_CRITICAL(&stateMux);

  savePreferences();
  M5.Display.clear();
}

// ================= RESET POSE & ODOMETRI =================
void resetPoseAndOdometry() {
  portENTER_CRITICAL(&stateMux);
  tick_count_left  = 0;
  tick_count_right = 0;
  odoX          = 0.0f;
  odoY          = 0.0f;
  odoTheta      = 0.0f;
  estVx         = 0.0f;
  estVth        = 0.0f;
  headingDeg    = 0.0f;
  targetHeading = 0.0f;
  yawErrSum     = 0.0f;
  lastYawErr    = 0.0f;
  portEXIT_CRITICAL(&stateMux);
}