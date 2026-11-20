// ================= SETUP ARDUINO OTA (IDE NETWORK PORT) =================
void setupArduinoOTA() {
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setMdnsEnabled(true);

  ArduinoOTA.onStart([]() {
    otaInProgress = true;
    motorRunning = false;
    webStepActive = false;
    driveMotors(0, 0);

    M5.Display.clear();
    M5.Display.setCursor(0, 20);
    M5.Display.setTextColor(YELLOW, BLACK);
    M5.Display.println("ARDUINO OTA UPDATE");
    M5.Display.println("Jangan matikan daya!");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    unsigned int pct = (progress / (total / 100));
    M5.Display.setCursor(0, 80);
    M5.Display.setTextColor(CYAN, BLACK);
    M5.Display.printf("Progress: %u%%   ", pct);
  });

  ArduinoOTA.onEnd([]() {
    M5.Display.setCursor(0, 110);
    M5.Display.setTextColor(GREEN, BLACK);
    M5.Display.println("SELESAI! REBOOT...");
  });

  ArduinoOTA.onError([](ota_error_t error) {
    otaInProgress = false;
    M5.Display.setCursor(0, 110);
    M5.Display.setTextColor(RED, BLACK);
    M5.Display.printf("OTA ERROR [%u]", error);
  });

  ArduinoOTA.begin();
}

// ================= SETUP WEB SERVER & WEB OTA ENDPOINTS =================
void setupWebServer() {
  server.on("/", HTTP_GET, []() {
    server.send_P(200, "text/html", INDEX_HTML);
  });

  server.on("/data", HTTP_GET, []() {
    portENTER_CRITICAL(&stateMux);
    float tL = targetRPM_A, tR = targetRPM_B;
    float rL = rpmA, rR = rpmB;
    float pL = outputPWM_A, pR = outputPWM_B;
    float hdg = headingDeg, thdg = targetHeading, yOut = yawPidOut;
    float x = odoX, y = odoY, th = odoTheta * RAD_TO_DEG;
    portEXIT_CRITICAL(&stateMux);

    char json[260];
    snprintf(json, sizeof(json),
             "{\"tgtL\":%.2f,\"tgtR\":%.2f,\"rpmL\":%.2f,\"rpmR\":%.2f,"
             "\"pwmL\":%.1f,\"pwmR\":%.1f,\"hdg\":%.2f,\"tgtHdg\":%.2f,"
             "\"yawOut\":%.2f,\"x\":%.4f,\"y\":%.4f,\"th\":%.2f}",
             tL, tR, rL, rR, pL, pR, hdg, thdg, yOut, x, y, th);
    server.send(200, "application/json", json);
  });

  server.on("/params", HTTP_GET, []() {
    char json[160];
    snprintf(json, sizeof(json),
             "{\"kp\":%.3f,\"ki\":%.4f,\"kd\":%.3f,\"kpy\":%.3f,\"kdy\":%.3f,\"maxrpm\":%.1f}",
             Kp, Ki, Kd, Kp_yaw, Kd_yaw, maxRPM);
    server.send(200, "application/json", json);
  });

  server.on("/set", HTTP_GET, []() {
    if (server.hasArg("kp"))     Kp = max(0.0f, server.arg("kp").toFloat());
    if (server.hasArg("ki"))     Ki = max(0.0f, server.arg("ki").toFloat());
    if (server.hasArg("kd"))     Kd = max(0.0f, server.arg("kd").toFloat());
    if (server.hasArg("kpy"))    Kp_yaw = max(0.0f, server.arg("kpy").toFloat());
    if (server.hasArg("kdy"))    Kd_yaw = max(0.0f, server.arg("kdy").toFloat());
    if (server.hasArg("maxrpm")) maxRPM = constrain(server.arg("maxrpm").toFloat(), 10.0f, 200.0f);
    savePreferences();
    server.send(200, "text/plain", "OK");
  });

  server.on("/step", HTTP_GET, []() {
    float r = server.hasArg("rpm") ? server.arg("rpm").toFloat() : 40.0f;
    unsigned long dur = server.hasArg("dur") ? server.arg("dur").toInt() : 3000;
    portENTER_CRITICAL(&stateMux);
    webStepTargetRPM = constrain(r, -maxRPM, maxRPM);
    webStepEndTime = millis() + dur;
    webStepActive = true;
    motorRunning = true;
    eStopActive = false;
    portEXIT_CRITICAL(&stateMux);
    server.send(200, "text/plain", "STEP STARTED");
  });

  server.on("/reset", HTTP_GET, []() {
    resetPoseAndOdometry();
    server.send(200, "text/plain", "RESET OK");
  });

  server.on("/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", Update.hasError() ? "FAIL" : "OK");
    delay(500);
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      otaInProgress = true;
      motorRunning = false;
      webStepActive = false;
      driveMotors(0, 0);

      M5.Display.clear();
      M5.Display.setCursor(0, 20);
      M5.Display.setTextColor(YELLOW, BLACK);
      M5.Display.println("WEB OTA UPDATE...");
      M5.Display.println("Jangan matikan daya!");
      Update.begin(UPDATE_SIZE_UNKNOWN);
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (Update.write(upload.buf, upload.currentSize) == upload.currentSize) {
        M5.Display.setCursor(0, 80);
        M5.Display.setTextColor(CYAN, BLACK);
        M5.Display.printf("Diterima: %u KB  ", Update.progress() / 1024);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) {
        M5.Display.setCursor(0, 110);
        M5.Display.setTextColor(GREEN, BLACK);
        M5.Display.println("SUKSES! REBOOT...");
      } else {
        otaInProgress = false;
        M5.Display.setCursor(0, 110);
        M5.Display.setTextColor(RED, BLACK);
        M5.Display.println("GAGAL WRITE FLASH!");
      }
    }
  });

  server.begin();
  MDNS.addService("http", "tcp", 80);
}

// ================= SIARAN UDP TELEMETRY (20Hz) =================
void sendUdpTelemetry() {
  portENTER_CRITICAL(&stateMux);
  float tL = targetRPM_A, tR = targetRPM_B, rL = rpmA, rR = rpmB;
  float pL = outputPWM_A, pR = outputPWM_B, hdg = headingDeg, yOut = yawPidOut;
  float x = odoX, y = odoY;
  portEXIT_CRITICAL(&stateMux);

  char udpBuf[160];
  int n = snprintf(udpBuf, sizeof(udpBuf),
                   "%lu,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.2f,%.2f,%.4f,%.4f\n",
                   millis(), tL, tR, rL, rR, pL, pR, hdg, yOut, x, y);
  udp.beginPacket(IPAddress(255, 255, 255, 255), UDP_PORT);
  udp.write((uint8_t*)udpBuf, n);
  udp.endPacket();
}
