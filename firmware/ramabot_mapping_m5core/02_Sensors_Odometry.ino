// ================= INTERRUPT ENKODER =================
void IRAM_ATTR isrA() { (digitalRead(ENCCA) == digitalRead(ENCCB)) ? pulseA++ : pulseA--; }
void IRAM_ATTR isrB() { (digitalRead(ENCDA) == digitalRead(ENCDB)) ? pulseB++ : pulseB--; }

void setupEncoders() {
  pinMode(ENCCA, INPUT_PULLUP); pinMode(ENCCB, INPUT_PULLUP);
  pinMode(ENCDA, INPUT_PULLUP); pinMode(ENCDB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCCA), isrA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCDA), isrB, RISING);
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

// ================= KALIBRASI GYRO Z =================
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
  gyroZOffset = sumZ / (float)samples;
  headingDeg = 0.0f; targetHeading = 0.0f;
  yawErrSum = 0.0f; lastYawErr = 0.0f;
  portEXIT_CRITICAL(&stateMux);

  savePreferences();
  M5.Display.clear();
}

// ================= RESET POSE & ODOMETRI =================
void resetPoseAndOdometry() {
  portENTER_CRITICAL(&stateMux);
  odoX = 0.0f; odoY = 0.0f; odoTheta = 0.0f;
  headingDeg = 0.0f; targetHeading = 0.0f;
  yawErrSum = 0.0f; lastYawErr = 0.0f;
  portEXIT_CRITICAL(&stateMux);

  noInterrupts();
  pulseA = 0; pulseB = 0;
  interrupts();
  prevPulseA = 0; prevPulseB = 0;
}
