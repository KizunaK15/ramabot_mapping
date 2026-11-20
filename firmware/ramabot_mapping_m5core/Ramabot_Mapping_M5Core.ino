#include <M5Unified.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Update.h>
#include <Preferences.h>
#include <math.h>
#include "config.h"

void setup() {
  Serial.begin(115200);
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Imu.begin();
  M5.Display.setTextSize(2);
  M5.Display.setTextColor(WHITE, BLACK);

  loadPreferences();
  calibrateGyroZ();

  // 1. Hubungkan ke Router Wi-Fi (SSID: Ramabot)
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  M5.Display.setCursor(0, 20);
  M5.Display.printf("MAP BOT WiFi:\n%s", WIFI_SSID);
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 8000) {
    delay(250);
    M5.Display.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    IPAddress ip = WiFi.localIP();
    ipOctet3 = ip[2];
    ipOctet4 = ip[3];
    setupArduinoOTA();
    setupWebServer();
    udp.begin(UDP_PORT);
    M5.Display.printf("\nIP: %s\n", ip.toString().c_str());
    delay(1000);
  }
  M5.Display.clear();

  // 2. Setup ESP-NOW (M5StickC Plus HatMiniJoyC)
  setupESPNow();

  // 3. Setup Enkoder & Motor PWM
  setupEncoders();
  setupMotorPWM();
  driveMotors(0, 0);

  // 4. Jalankan Dual-Core FreeRTOS
  xTaskCreatePinnedToCore(taskUiAndComm, "UI_Comm", 12288, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(taskControlLoop, "Control", 8192, NULL, 2, NULL, 1);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
