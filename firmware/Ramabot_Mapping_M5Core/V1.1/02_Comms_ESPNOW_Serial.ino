#include "config.h"

// ================= CALLBACK TERIMA ESP-NOW =================
#if ESP_ARDUINO_VERSION_MAJOR >= 3
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
#else
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
#endif
  if (otaInProgress) return;
  if (len == sizeof(struct_joy_msg) && incomingData[0] == 0xAB) {
    portENTER_CRITICAL(&stateMux);
    memcpy(&incomingJoy, incomingData, sizeof(incomingJoy));
    lastJoyRecvTime = millis();
    eStopActive  = incomingJoy.btn_estop;
    boostActive  = incomingJoy.btn_boost;
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
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, stickAddress, 6);
    peerInfo.channel = 0;
    peerInfo.ifidx   = WIFI_IF_STA;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
  }
}

// ================= KIRIM TELEMETRI KE M5STICKC PLUS PORTRAIT =================
void sendTelemetryESPNow() {
  portENTER_CRITICAL(&stateMux);
  telemetryData.pwmL          = outputPWM_A;
  telemetryData.pwmR          = outputPWM_B;
  telemetryData.headingDeg    = headingDeg;
  telemetryData.targetHeading = targetHeading;
  telemetryData.pidOut        = yawPidOut;
  telemetryData.odoX          = odoX;
  telemetryData.odoY          = odoY;
  telemetryData.odoTheta      = odoTheta;
  telemetryData.kp            = Kp_yaw;
  telemetryData.ki            = Ki_yaw;
  telemetryData.kd            = Kd_yaw;
  telemetryData.activeSpeed   = (uint8_t)((boostActive ? 1.0f : normalSpeedPct) * 255.0f);
  telemetryData.headingLock   = headingLockEnabled;
  telemetryData.boostActive   = boostActive;
  telemetryData.eStop         = eStopActive;
  portEXIT_CRITICAL(&stateMux);

  esp_now_send(stickAddress, (uint8_t *)&telemetryData, sizeof(telemetryData));
}

// ================= BRIDGE SERIAL ROS 2 (JETSON XAVIER NX) =================
String serialCmdBuf = "";
void processRosSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      serialCmdBuf.trim();
      if (serialCmdBuf.startsWith("CMD:")) {
        int comma = serialCmdBuf.indexOf(',');
        if (comma > 4) {
          float vx  = serialCmdBuf.substring(4, comma).toFloat();
          float vth = serialCmdBuf.substring(comma + 1).toFloat();
          if (isfinite(vx) && isfinite(vth)) {
            portENTER_CRITICAL(&stateMux);
            rosCmdVx  = constrain(vx, -maxLinearVel, maxLinearVel);
            rosCmdVth = constrain(vth, -maxAngularVel, maxAngularVel);
            rosCmdExpireTime = millis() + 450;
            portEXIT_CRITICAL(&stateMux);
          }
        }
      } else if (serialCmdBuf == "RESET") {
        resetPoseAndOdometry();
      }
      serialCmdBuf = "";
    } else {
      serialCmdBuf += c;
    }
  }

  portENTER_CRITICAL(&stateMux);
  float x = odoX, y = odoY, th = odoTheta, vx = estVx, vth = estVth;
  portEXIT_CRITICAL(&stateMux);
  Serial.printf("ODO:%.4f,%.4f,%.4f,%.4f,%.4f,0,0\n", x, y, th, vx, vth);
}