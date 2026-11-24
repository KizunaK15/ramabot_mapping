#include "config.h"

// ================= INTERRUPT ENKODER (2X QUADRATURE DECODING ROBOT MAPPING) =================
void IRAM_ATTR isr_encoder_left() {
  (digitalRead(ENCODER_L_A) == digitalRead(ENCODER_L_B)) ? pulseL++ : pulseL--;
}

void IRAM_ATTR isr_encoder_right() {
  // Tanda disesuaikan hasil kalibrasi agar gerak maju bernilai positif (+)
  (digitalRead(ENCODER_R_A) == digitalRead(ENCODER_R_B)) ? pulseR-- : pulseR++;
}

void initOdometry() {
  // Jalur optocoupler ENCA & ENCB sudah dilengkapi pull-up eksternal 4K7 di PCB
  pinMode(ENCODER_L_A, INPUT);
  pinMode(ENCODER_L_B, INPUT);
  pinMode(ENCODER_R_A, INPUT);
  pinMode(ENCODER_R_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), isr_encoder_left, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), isr_encoder_right, CHANGE);
}

// ================= NORMALISASI SUDUT =================
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

// ================= RESET POSE & ODOMETRI (ALGORITMA ROBOT UTAMA) =================
void resetPoseAndOdometry() {
  portENTER_CRITICAL(&stateMux);
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

  noInterrupts();
  pulseL = 0;
  pulseR = 0;
  interrupts();
  prevPulseL = 0;
  prevPulseR = 0;
}