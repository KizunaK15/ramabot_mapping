#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Bluepad32.h>
#include <Preferences.h>

// --- PIN MOTOR, IMU & OLED (EKSISTING) ---
#define ML1 23
#define ML2 22
#define MR1 18
#define MR2 17
#define SPIN 32
#define SDA1 16
#define SCL1 5
#define SDA0 19
#define SCL0 21

// --- PIN ENCODER (KONFIRMASI BESOK) ---
// Pin bebas ESP32 yang aman untuk interrupt: 34, 35, 36, 39 atau 25, 26, 27, 33
#define ENC_L_A 34
#define ENC_L_B 35
#define ENC_R_A 36
#define ENC_R_B 39

volatile long encLeftCount = 0;
volatile long encRightCount = 0;

void IRAM_ATTR isrEncoderLeft() {
  if (digitalRead(ENC_L_B) == HIGH) encLeftCount++;
  else encLeftCount--;
}

void IRAM_ATTR isrEncoderRight() {
  if (digitalRead(ENC_R_B) == HIGH) encRightCount++;
  else encRightCount--;
}

ControllerPtr myControllers[BP32_MAX_GAMEPADS];
Preferences pref;
Adafruit_MPU6050 mpu;
Adafruit_SSD1306 display(128, 64, &Wire1, -1);
TaskHandle_t Task1, Task2;

bool calibGyro = false, modeSet = false;
bool silang, kotak, segitiga, bulet, kiri, kanan, atas, bawah, l1, l2, r1, r2, SELECT, START;
int IN, lastIN, analogLY = 0, analogLX = 0, analogRY = 0, analogRX = 0, speedrun;

float sudutZ = 0;
unsigned long waktuLalu = 0, waktuSekarang;
float gyroZOffset, dt;

float pwmL = 0, pwmR = 0, II, PID, dtPID, Err, lastErr;
unsigned long dtLast, dtNow;
unsigned long lastTelemetryMs = 0;

byte Speed, Boost, Tsamp;
bool statGyro;
int frekuensiPWM, autoOff, enterPID;
float Kp, Ki, Kd;
uint8_t macTersimpan[6];

void baca_preferences() {
  pref.begin("data1", false);
  Speed = pref.getUChar("Speed", 130);
  Boost = pref.getUChar("Boost", 255);
  Kp = pref.getFloat("Kp", 3.0);
  Ki = pref.getFloat("Ki", 0.0);
  Kd = pref.getFloat("Kd", 12.0);
  Tsamp = pref.getUChar("Tsamp", 4);
  autoOff = pref.getInt("autoOff", 1000);
  enterPID = pref.getInt("enterPID", 300);
  statGyro = pref.getBool("statGyro", true);
  gyroZOffset = pref.getFloat("gyroZOffset", 0);
  frekuensiPWM = pref.getInt("frekuensiPWM", 675);
  pref.getBytes("macStik", macTersimpan, 6);
}

int pushSW = 0, toOff = 0, toInPID = 0;
byte toTS;
hw_timer_t* timer = NULL;

void IRAM_ATTR onTimer() {
  pushSW++; toTS++; toOff++; toInPID++;
}

void codeForTask1(void* parameter) {
  const int centerX = 95, centerY = 35, radius = 22;
  while (1) {
    BP32.update();
    processControllers();
    if (SELECT) { menu_utama(); }

    tampiltext(0, 5, 1, "HEADING:");
    tampiltext(0, 20, 2, String((int)sudutZ)); display.write(248);

    display.drawCircle(centerX, centerY, radius, SSD1306_WHITE);
    display.drawCircle(centerX, centerY, 1, SSD1306_WHITE);
    float radian = (-sudutZ) * DEG_TO_RAD;
    int xTarget = centerX + (radius - 6) * sin(radian);
    int yTarget = centerY - (radius - 6) * cos(radian);
    int xFront = centerX + (radius + 8) * sin(radian);
    int yFront = centerY - (radius + 8) * cos(radian);

    display.drawLine(centerX, centerY, xTarget, yTarget, SSD1306_WHITE);
    display.fillCircle(xTarget, yTarget, 2, SSD1306_WHITE);
    tampiltext(xFront - 3, yFront - 4, 1, "F");
    tampiltext(0, 54, 1, "E:" + String(encLeftCount) + "/" + String(encRightCount));
    display.display();
    display.clearDisplay();
  }
}

void codeForTask2(void* parameter) {
  int analogX, analogY;
  waktuLalu = micros();
  while (1) {
    dtNow = millis();
    if (modeSet == true) {
      vTaskDelay(1);
      waktuLalu = dtNow;
      sudutZ = toInPID = II = 0;
      motor(0, 0);
    } else {
      if (calibGyro == false) {
        if (statGyro) { update_gyro(); }
        else { waktuLalu = dtNow; sudutZ = 0; }
      }

      if (l1) { digitalWrite(SPIN, HIGH); } else { digitalWrite(SPIN, LOW); }
      if (r1) { speedrun = Boost; } else { speedrun = Speed; }
      pwmL = pwmR = 0;

      analogX = (abs(analogLX) > 25) ? -constrain(analogLX, -500, 500) : 0;
      analogY = (abs(analogLY) > 25) ? -constrain(analogLY, -500, 500) : 0;

      if ((analogX != 0) || (analogY != 0)) {
        pwmL = pwmR = map(analogY, -500, 500, -speedrun, speedrun);
        pwmL -= map(analogX, -500, 500, -speedrun, speedrun);
        pwmR += map(analogX, -500, 500, -speedrun, speedrun);
        pwmL = constrain(pwmL, -speedrun, speedrun);
        pwmR = constrain(pwmR, -speedrun, speedrun);
      }
      if (atas || silang) { pwmR += speedrun; pwmL += speedrun; }
      if (bawah || bulet) { pwmR -= speedrun; pwmL -= speedrun; }
      if (kiri)           { pwmR += speedrun; pwmL -= speedrun; }
      if (kanan)          { pwmR -= speedrun; pwmL += speedrun; }
      pwmL = constrain(pwmL, -speedrun, speedrun);
      pwmR = constrain(pwmR, -speedrun, speedrun);

      if (lastIN != IN) {
        if (r2) { sudutZ -= 90.0; }
        if (l2) { sudutZ += 90.0; }
        lastIN = IN;
      }

      Err = sudutZ;
      if (pwmL != pwmR) { toInPID = 0; }
      if (toInPID < enterPID) { sudutZ = II = Err = 0; }

      if (Err != 0) { toOff = 0; }
      if (toOff >= autoOff) { toOff = autoOff; II = 0; }

      if (toTS >= Tsamp) {
        toTS = 0;
        dtPID = (float)(dtNow - dtLast) / 1000.0f;
        dtLast = dtNow;
        if (dtPID > 0) {
          II += Err * dtPID;
          II = constrain(II, -250, 250);
          PID = (Err * Kp) + (Ki * II) + ((Kd / 100.0f) * ((Err - lastErr) / dtPID));
          if (Ki != 0) {
            if (PID > 255) II -= (PID - 255) / Ki;
            else if (PID < -255) II -= (PID + 255) / Ki;
          }
          lastErr = Err;
        }
      }

      if (toInPID >= enterPID) {
        pwmL -= PID;
        pwmR += PID;
        pwmL = constrain(pwmL, -255, 255);
        pwmR = constrain(pwmR, -255, 255);
        toInPID = enterPID;
      }
      motor(pwmL, pwmR);

      if (dtNow - lastTelemetryMs >= 50) {
        lastTelemetryMs = dtNow;
        noInterrupts();
        long cL = encLeftCount;
        long cR = encRightCount;
        interrupts();
        Serial.printf("ENCL:%ld,ENCR:%ld,YAW:%.2f,PWML:%d,PWMR:%d\n",
                      cL, cR, sudutZ, (int)pwmL, (int)pwmR);
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA0, SCL0, 400000);
  mpu.begin(0x68, &Wire);
  Wire1.begin(SDA1, SCL1, 400000);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3c);
  baca_preferences();

  pinMode(ENC_L_A, INPUT);
  pinMode(ENC_L_B, INPUT);
  pinMode(ENC_R_A, INPUT);
  pinMode(ENC_R_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrEncoderRight, RISING);

  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000, true);
  timerAlarmEnable(timer);

  display.cp437(true);
  display.clearDisplay();

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.enableNewBluetoothConnections(true);
  BP32.enableVirtualDevice(false);

  pinMode(0, INPUT);
  pinMode(ML1, OUTPUT); pinMode(ML2, OUTPUT);
  pinMode(MR1, OUTPUT); pinMode(MR2, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(SPIN, OUTPUT); digitalWrite(SPIN, LOW);

  ledcSetup(3, 5, 8); ledcAttachPin(2, 3);
  ledcSetup(4, frekuensiPWM, 8); ledcAttachPin(ML1, 4);
  ledcSetup(5, frekuensiPWM, 8); ledcAttachPin(ML2, 5);
  ledcSetup(6, frekuensiPWM, 8); ledcAttachPin(MR1, 6);
  ledcSetup(7, frekuensiPWM, 8); ledcAttachPin(MR2, 7);
  ledcWrite(3, 60);

  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  connectGamepad();

  disableCore0WDT(); disableCore1WDT();
  xTaskCreatePinnedToCore(codeForTask1, "Aplikasi", 16384, NULL, 1, &Task1, 0);
  delay(500);
  xTaskCreatePinnedToCore(codeForTask2, "Hitung", 8192, NULL, 2, &Task2, 1);
}

void loop() {}

void update_gyro() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  waktuSekarang = micros();
  dt = (waktuSekarang - waktuLalu) / 1000000.0;
  waktuLalu = waktuSekarang;
  float kecepatanSudutZ = (g.gyro.z - gyroZOffset) * 57.2958;
  if (abs(kecepatanSudutZ) > 0.5) { sudutZ += kecepatanSudutZ * dt; }
}

void tampiltext(unsigned char x, unsigned char y, unsigned char ukuran, String text) {
  display.setTextSize(ukuran);
  display.setCursor(x, y);
  display.setTextColor(SSD1306_WHITE);
  display.print(text);
}

void motor(short motorki, short motorka) {
  motorka = constrain(motorka, -255, 255);
  motorki = constrain(motorki, -255, 255);
  if (motorki > 0) { ledcWrite(4, 0); ledcWrite(5, motorki); }
  else { ledcWrite(4, -motorki); ledcWrite(5, 0); }
  if (motorka > 0) { ledcWrite(7, 0); ledcWrite(6, motorka); }
  else { ledcWrite(7, -motorka); ledcWrite(6, 0); }
}
