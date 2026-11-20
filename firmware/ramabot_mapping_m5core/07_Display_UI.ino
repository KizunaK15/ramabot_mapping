bool imuSerialDebug = false;

// ================= GAMBAR KOMPAS DIGITAL =================
void drawHeadingCompass(int centerX, int centerY, int radius, float angleDeg) {
  M5.Display.fillCircle(centerX, centerY, radius + 12, BLACK);
  M5.Display.drawCircle(centerX, centerY, radius, WHITE);
  M5.Display.drawCircle(centerX, centerY, 2, WHITE);

  float rad = (-angleDeg) * DEG_TO_RAD;
  int xTarget = centerX + (int)((radius - 6) * sinf(rad));
  int yTarget = centerY - (int)((radius - 6) * cosf(rad));
  int xFront  = centerX + (int)((radius + 7) * sinf(rad));
  int yFront  = centerY - (int)((radius + 7) * cosf(rad));

  M5.Display.drawLine(centerX, centerY, xTarget, yTarget, GREEN);
  M5.Display.fillCircle(xTarget, yTarget, 3, RED);

  M5.Display.setTextSize(1);
  M5.Display.setTextColor(CYAN, BLACK);
  M5.Display.setCursor(xFront - 3, yFront - 3);
  M5.Display.print("F");
  M5.Display.setTextSize(2);
}

// ================= KONTROL TOMBOL FISIK M5STACK =================
void handlePhysicalButtons() {
  if (M5.BtnA.wasPressed()) {
    screenMode = (UIMode)((screenMode + 1) % UI_MODE_COUNT);
    M5.Display.clear();
  }

  if (M5.BtnB.wasPressed()) {
    if (screenMode == DASHBOARD) { headingLockEnabled = !headingLockEnabled; savePreferences(); }
    else if (screenMode == TUNE_VEL_KP) { Kp += 0.05f; savePreferences(); }
    else if (screenMode == TUNE_VEL_KI) { Ki += 0.01f; savePreferences(); }
    else if (screenMode == TUNE_VEL_KD) { Kd += 0.01f; savePreferences(); }
    else if (screenMode == TUNE_YAW_KP) { Kp_yaw += 0.10f; savePreferences(); }
    else if (screenMode == TUNE_YAW_KD) { Kd_yaw += 0.02f; savePreferences(); }
    else if (screenMode == TUNE_MAX_RPM) { maxRPM += 10.0f; savePreferences(); }
    else if (screenMode == IMU_VIEW) { imuSerialDebug = !imuSerialDebug; }
  }

  if (M5.BtnC.wasPressed()) {
    if (screenMode == DASHBOARD) { testMode = (testMode + 1) % 2; errSum_A = 0; errSum_B = 0; }
    else if (screenMode == TUNE_VEL_KP) { Kp = max(0.0f, Kp - 0.05f); savePreferences(); }
    else if (screenMode == TUNE_VEL_KI) { Ki = max(0.0f, Ki - 0.01f); savePreferences(); }
    else if (screenMode == TUNE_VEL_KD) { Kd = max(0.0f, Kd - 0.01f); savePreferences(); }
    else if (screenMode == TUNE_YAW_KP) { Kp_yaw = max(0.0f, Kp_yaw - 0.10f); savePreferences(); }
    else if (screenMode == TUNE_YAW_KD) { Kd_yaw = max(0.0f, Kd_yaw - 0.02f); savePreferences(); }
    else if (screenMode == TUNE_MAX_RPM) { maxRPM = max(10.0f, maxRPM - 10.0f); savePreferences(); }
    else if (screenMode == IMU_VIEW) { calibrateGyroZ(); }
    else if (screenMode == ODOMETRY_VIEW) { resetPoseAndOdometry(); }
  }
}

// ================= RENDER HALAMAN LCD M5STACK =================
void renderLcdScreen() {
  M5.Display.setCursor(0, 0);
  if (screenMode == DASHBOARD) {
    M5.Display.setTextColor(WHITE, BLACK);
    M5.Display.printf("MODE:%s\n", testMode == 0 ? "OPEN" : "PID ");
    M5.Display.setTextColor(telemetryData.headingLock ? GREEN : YELLOW, BLACK);
    M5.Display.printf("LOCK:%s\n", telemetryData.headingLock ? "ON  " : "OFF ");
    M5.Display.setTextColor(CYAN, BLACK);
    M5.Display.printf("HDG:%+4.0f\n", telemetryData.headingDeg);

    drawHeadingCompass(255, 42, 28, telemetryData.headingDeg);

    M5.Display.setCursor(0, 86);
    M5.Display.setTextColor(WHITE, BLACK);
    M5.Display.println("--------------------");
    M5.Display.printf("TGT L:%+4.0f R:%+4.0f\n", telemetryData.targetL, telemetryData.targetR);
    M5.Display.printf("RPM L:%+4.0f R:%+4.0f\n", telemetryData.rpmL, telemetryData.rpmR);
    M5.Display.printf("PWM L:%+4.0f R:%+4.0f\n", telemetryData.pwmL, telemetryData.pwmR);
  }
  else if (screenMode == WIFI_INFO) {
    M5.Display.setTextColor(WHITE, BLACK);
    M5.Display.println(" == MAP BOT WIFI ==");
    M5.Display.printf("SSID: %s\n", WIFI_SSID);
    if (wifiConnected) {
      M5.Display.setTextColor(GREEN, BLACK);
      M5.Display.print("IP  : "); M5.Display.println(WiFi.localIP());
      M5.Display.setTextColor(CYAN, BLACK);
      M5.Display.println("Web : ramabot-map.local");
      M5.Display.printf("OTA : %s.local\n", OTA_HOSTNAME);
    } else {
      M5.Display.setTextColor(RED, BLACK);
      M5.Display.println("STAT: OFFLINE");
    }
  }
  else if (screenMode >= TUNE_VEL_KP && screenMode <= TUNE_VEL_KD) {
    M5.Display.setTextColor(WHITE, BLACK);
    M5.Display.println(" == PID KECEPATAN ==");
    M5.Display.printf(" %c Kp: %.2f\n", screenMode == TUNE_VEL_KP ? '>' : ' ', Kp);
    M5.Display.printf(" %c Ki: %.3f\n", screenMode == TUNE_VEL_KI ? '>' : ' ', Ki);
    M5.Display.printf(" %c Kd: %.2f\n", screenMode == TUNE_VEL_KD ? '>' : ' ', Kd);
    M5.Display.println("\nBtnB:+  BtnC:- (AutoSave)");
  }
  else if (screenMode == TUNE_YAW_KP || screenMode == TUNE_YAW_KD) {
    M5.Display.setTextColor(WHITE, BLACK);
    M5.Display.println(" == PID HEADING ==");
    M5.Display.printf(" %c Kp_yaw: %.2f\n", screenMode == TUNE_YAW_KP ? '>' : ' ', Kp_yaw);
    M5.Display.printf(" %c Kd_yaw: %.2f\n", screenMode == TUNE_YAW_KD ? '>' : ' ', Kd_yaw);
    M5.Display.printf(" YawOut  : %+5.1f\n", telemetryData.yawPidOut);
    M5.Display.println("\nBtnB:+  BtnC:- (AutoSave)");
  }
  else if (screenMode == TUNE_MAX_RPM) {
    M5.Display.setTextColor(WHITE, BLACK);
    M5.Display.println(" == TUNE MAX RPM ==");
    M5.Display.printf(" > Max RPM: %.1f\n", maxRPM);
    M5.Display.println("\nBtnB:+10  BtnC:-10");
  }
  else if (screenMode == IMU_VIEW) {
    float rateZ = GYRO_Z_SIGN * (gyroZ - gyroZOffset);
    const char* dirX = (accX < -0.15f) ? "X- MAJU  " : ((accX > 0.15f) ? "X+ MUNDUR" : "NETRAL   ");
    const char* dirY = (accY >  0.15f) ? "Y+ KANAN " : ((accY < -0.15f) ? "Y- KIRI  " : "NETRAL   ");
    const char* dirZ = (accZ >  0.50f) ? "Z+ ATAS  " : ((accZ < -0.50f) ? "Z- BAWAH " : "MIRING   ");

    M5.Display.setTextColor(CYAN, BLACK);
    M5.Display.println(" == DEBUG SUMBU IMU ==");
    M5.Display.setTextColor(WHITE, BLACK);
    M5.Display.printf("AccX:%+5.2f %s\n", accX, dirX);
    M5.Display.printf("AccY:%+5.2f %s\n", accY, dirY);
    M5.Display.printf("AccZ:%+5.2f %s\n", accZ, dirZ);
    M5.Display.println("----------------------");
    M5.Display.setTextColor(YELLOW, BLACK);
    M5.Display.printf("GyrZ:%+6.1f d/s\n", rateZ);
    M5.Display.setTextColor(GREEN, BLACK);
    M5.Display.printf("HDG :%+6.1f deg (%s)\n", telemetryData.headingDeg,
                      rateZ > 2.0f ? "CCW+" : (rateZ < -2.0f ? "CW- " : "DIAM"));

    M5.Display.setCursor(0, 222);
    M5.Display.setTextSize(1);
    M5.Display.setTextColor(LIGHTGREY, BLACK);
    M5.Display.printf("BtnA:Next | BtnB:Serial(%s) | BtnC:Calib", imuSerialDebug ? "ON" : "OFF");
    M5.Display.setTextSize(2);
  }
  else if (screenMode == ODOMETRY_VIEW) {
    M5.Display.setTextColor(WHITE, BLACK);
    M5.Display.println(" == FUSED ODOMETRY ==");
    M5.Display.printf("X : %+6.3f m\n", telemetryData.odoX);
    M5.Display.printf("Y : %+6.3f m\n", telemetryData.odoY);
    M5.Display.printf("Th: %+6.1f deg\n", telemetryData.odoTheta * RAD_TO_DEG);
    M5.Display.println("\nBtnC: Reset Pose");
  }
}

// ================= CORE 0: UI, WEB SERVER, OTA, UDP, SERIAL & ESP-NOW =================
void taskUiAndComm(void *parameter) {
  unsigned long lastUiUpdate = 0;
  unsigned long lastUdpTx = 0;
  unsigned long lastImuSerialTx = 0;
  unsigned long lastOdoSerialTx = 0;

  for (;;) {
    if (wifiConnected) {
      ArduinoOTA.handle();
      server.handleClient();
    }

    if (otaInProgress) {
      vTaskDelay(pdMS_TO_TICKS(2));
      continue;
    }

    M5.update();
    handlePhysicalButtons();

    if (imuSerialDebug && (millis() - lastImuSerialTx >= 50)) {
      lastImuSerialTx = millis();
      float rateZ = GYRO_Z_SIGN * (gyroZ - gyroZOffset);
      Serial.printf("AccX:%.2f,AccY:%.2f,AccZ:%.2f,GyroZ:%.2f,Hdg:%.1f\n",
                    accX, accY, accZ, rateZ, headingDeg);
    }
    else if (!imuSerialDebug && (millis() - lastOdoSerialTx >= 50)) {
      lastOdoSerialTx = millis();
      portENTER_CRITICAL(&stateMux);
      float x = odoX;
      float y = odoY;
      float th = odoTheta;
      float rL = rpmA;
      float rR = rpmB;
      portEXIT_CRITICAL(&stateMux);

      noInterrupts();
      long pL = pulseA;
      long pR = pulseB;
      interrupts();

      float vL = (rL / 60.0f) * (2.0f * PI * WHEEL_RADIUS);
      float vR = (rR / 60.0f) * (2.0f * PI * WHEEL_RADIUS);
      float vx = 0.5f * (vR + vL);
      float vth = (vR - vL) / TRACK_WIDTH;

      Serial.printf("ODO:%.4f,%.4f,%.4f,%.3f,%.3f,%ld,%ld\n",
                    x, y, th, vx, vth, pL, pR);
    }

    if (wifiConnected && (millis() - lastUdpTx >= 50)) {
      lastUdpTx = millis();
      sendUdpTelemetry();
    }

    if (millis() - lastUiUpdate >= 100) {
      lastUiUpdate = millis();
      sendTelemetryESPNow();
      renderLcdScreen();
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}
