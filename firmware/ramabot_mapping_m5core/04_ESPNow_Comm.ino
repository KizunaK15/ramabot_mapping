// ================= CALLBACK TERIMA ESP-NOW =================
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  if (otaInProgress) return;
  if (len == sizeof(struct_joy_msg) && incomingData[0] == 0xAB) {
    portENTER_CRITICAL(&stateMux);
    memcpy(&incomingJoy, incomingData, sizeof(incomingJoy));
    lastJoyRecvTime = millis();
    eStopActive = incomingJoy.btn_estop;
    motorRunning = !eStopActive;
    if (eStopActive) webStepActive = false;
    portEXIT_CRITICAL(&stateMux);

    if (incomingJoy.btn_reset_pose) {
      resetPoseAndOdometry();
    }
  }
}

// ================= INISIALISASI ESP-NOW =================
void setupESPNow() {
  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(OnDataRecv);
    memcpy(peerInfo.peer_addr, stickAddress, 6);
    peerInfo.channel = 0;
    peerInfo.ifidx = WIFI_IF_STA;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
  }
}

// ================= KIRIM TELEMETRI KE M5STICKC PLUS =================
void sendTelemetryESPNow() {
  portENTER_CRITICAL(&stateMux);
  telemetryData.targetL       = targetRPM_A;
  telemetryData.targetR       = targetRPM_B;
  telemetryData.rpmL          = rpmA;
  telemetryData.rpmR          = rpmB;
  telemetryData.pwmL          = outputPWM_A;
  telemetryData.pwmR          = outputPWM_B;
  telemetryData.headingDeg    = headingDeg;
  telemetryData.targetHeading = targetHeading;
  telemetryData.yawPidOut     = yawPidOut;
  telemetryData.odoX          = odoX;
  telemetryData.odoY          = odoY;
  telemetryData.odoTheta      = odoTheta;
  telemetryData.headingLock   = headingLockEnabled;
  telemetryData.motorRunning  = motorRunning;
  telemetryData.eStop         = eStopActive;
  telemetryData.ip3           = ipOctet3;
  telemetryData.ip4           = ipOctet4;
  portEXIT_CRITICAL(&stateMux);

  noInterrupts();
  telemetryData.pulseL = pulseA;
  telemetryData.pulseR = pulseB;
  interrupts();

  esp_now_send(stickAddress, (uint8_t *) &telemetryData, sizeof(telemetryData));
}
