#include "config.h"

void setup() {
  Serial.begin(115200);
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Imu.begin();
  M5.Speaker.mute(); // Matikan speaker agar DAC GPIO 25 aman dipakai Encoder R_A
  
  M5.Display.setTextSize(2);
  M5.Display.setTextColor(WHITE, BLACK);
  M5.Display.clear();

  loadPreferences();
  calibrateGyroZ();

  // Inisialisasi Interrupt Pin Encoder Roda
  initOdometry();

  // 1. Hubungkan ke Router Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  M5.Display.setCursor(0, 20);
  M5.Display.printf("Konek WiFi:\n%s", WIFI_SSID);
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
    delay(800);
  }
  M5.Display.clear();

  // 2. Setup ESP-NOW & Motor PWM
  setupESPNow();
  setupMotorPWM();
  driveMotors(0, 0);

  // 3. Jalankan Dual-Core FreeRTOS
  xTaskCreatePinnedToCore(taskUiAndComm, "UI_Comm", 12288, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(taskControlLoop, "Control", 8192, NULL, 2, NULL, 1);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}