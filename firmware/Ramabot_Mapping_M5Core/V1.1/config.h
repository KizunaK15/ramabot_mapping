#pragma once
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

// ================= KONFIGURASI WI-FI ROUTER & OTA =================
const char* WIFI_SSID    = "Ramabot";
const char* WIFI_PASS    = "RamabotXavier";
const char* OTA_HOSTNAME = "ramabot-map"; // Akses via http://ramabot-map.local
const uint16_t UDP_PORT  = 8888;

WebServer server(80);
WiFiUDP udp;
bool wifiConnected = false;
volatile bool otaInProgress = false;
uint8_t ipOctet3 = 0;
uint8_t ipOctet4 = 0;

// ================= DEFINISI PIN DRIVER MOTOR BTS7960 / IBT-2 =================
const int M1A = 2;    // Motor Kiri Maju (PWM_L_FWD)
const int M1B = 12;   // Motor Kiri Mundur (PWM_L_REV)
const int M2A = 5;    // Motor Kanan Maju (PWM_R_FWD)
const int M2B = 15;   // Motor Kanan Mundur (PWM_R_REV)

const int PWM_FREQ = 5000;
const int PWM_RES  = 8;

const int CH_M1A = 4;
const int CH_M1B = 5;
const int CH_M2A = 6;
const int CH_M2B = 7;

// ================= DEFINISI PIN & RESOLUSI ENCODER RODA =================
// Catatan Hardware: GPIO 35 & 36 adalah Input-Only (Wajib Resistor Pull-Up Eksternal ke 3.3V)
// GPIO 25 terhubung ke internal Speaker DAC (Audio di-mute pada setup)
const int ENCODER_L_A = 16;  // Channel A Kiri (Interrupt)
const int ENCODER_L_B = 35;  // Channel B Kiri (Input-Only)
const int ENCODER_R_A = 25;  // Channel A Kanan (Interrupt)
const int ENCODER_R_B = 36;  // Channel B Kanan (Input-Only)

const float ENCODER_CPR     = 816.0f; // 1X Decoding (16 PPR * 51 Gear Ratio)
const bool  USE_IMU_HEADING = true;   // Sensor Fusion: Gyro Z IMU + Quadrature Encoder

// ================= SPESIFIKASI MEKANIK & ORIENTASI IMU =================
const float WHEEL_RADIUS    = 0.0675f; // Meter (Diameter 13.5 cm)
const float TRACK_WIDTH     = 0.2370f; // Meter (Lebar antar pusat ban 23.7 cm)
const float WHEEL_BASE      = TRACK_WIDTH; // Alias kompatibilitas
const int   MOTOR_MIN_PWM   = 38;      // Kompensasi deadzone gearbox 1/51
const float MOTOR_LAG_TAU   = 0.08f;   // Waktu respons inersia motor

// Orientasi Fisik M5Stack Core Grey:
// X- = Maju (F), X+ = Mundur, Y+ = Kanan, Y- = Kiri, Z+ = Atas, Z- = Bawah
const float ACCEL_X_SIGN    = -1.0f;
const float GYRO_Z_SIGN     = -1.0f;   // Putar kiri (CCW) = +headingDeg

// ================= MAC ADDRESS M5STICKC PLUS =================
uint8_t stickAddress[] = {0x50, 0x02, 0x91, 0x92, 0x3A, 0x78};
esp_now_peer_info_t peerInfo;
Preferences pref;

// ================= STRUKTUR DATA ESP-NOW =================
typedef struct __attribute__((packed)) {
  uint8_t header;            // 0xAB
  int8_t  linear_speed;      // -100 .. +100
  int8_t  angular_speed;     // -100 .. +100
  bool    btn_estop;         // Toggle E-STOP
  bool    btn_boost;         // Mode Boost Speed
  bool    btn_reset_pose;    // Reset Odometri & Heading = 0
  int8_t  snap_turn;         // -1 = -90 deg, +1 = +90 deg
  uint8_t reserved;
} struct_joy_msg;
struct_joy_msg incomingJoy;

typedef struct __attribute__((packed)) {
  float   pwmL;
  float   pwmR;
  float   headingDeg;
  float   targetHeading;
  float   pidOut;
  float   odoX;
  float   odoY;
  float   odoTheta;
  float   kp;
  float   ki;
  float   kd;
  uint8_t activeSpeed;
  bool    headingLock;
  bool    boostActive;
  bool    eStop;
} struct_telemetry;
struct_telemetry telemetryData;

// ================= VARIABEL ODOMETRI (ENCODER + IMU) =================
float odoX = 0.0f, odoY = 0.0f, odoTheta = 0.0f;
float estVx = 0.0f, estVth = 0.0f;
float linearScale = 1.00f;   // Faktor kalibrasi fine-tuning jarak 1 meter
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

// Variabel Tick Interrupt Encoder (volatile)
extern volatile long tick_count_left;
extern volatile long tick_count_right;

// ================= VARIABEL IMU & HEADING =================
float accX = 0, accY = 0, accZ = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;

float gyroZOffset   = 0.0f;
float headingDeg    = 0.0f;
float targetHeading = 0.0f;
bool  headingLockEnabled = true;

// ================= VARIABEL KONTROL & HEADING PID =================
bool motorRunning = true;
bool eStopActive  = false;
bool boostActive  = false;
unsigned long lastJoyRecvTime = 0;

float rosCmdVx  = 0.0f;
float rosCmdVth = 0.0f;
unsigned long rosCmdExpireTime = 0;

bool webStepActive = false;
float webStepTargetVel = 0.0f;
unsigned long webStepEndTime = 0;

float maxLinearVel   = 0.695f; // m/s (100% PWM)
float normalSpeedPct = 0.65f;  // 65% kecepatan saat mode normal (~0.45 m/s)
float maxAngularVel  = 2.20f;  // rad/s
float outputPWM_A = 0.0f, outputPWM_B = 0.0f;

// Heading Lock PID
float Kp_yaw = 1.85f, Ki_yaw = 0.06f, Kd_yaw = 0.16f;
float yawErrSum = 0.0f, lastYawErr = 0.0f, yawPidOut = 0.0f;
float prevRateZ = 0.0f;
bool  yawLockedSettled = true;
unsigned long turnSettlingTimer = 0;
unsigned long yawZeroTimer = 0;

const unsigned long ENTER_PID_DELAY_MS = 250;
const unsigned long YAW_AUTO_OFF_MS    = 600;
const float YAW_LOCK_ENTER_DEG         = 0.35f;
const float YAW_LOCK_EXIT_DEG          = 1.10f;
const float YAW_DEADBAND_MOVE_DEG      = 0.45f;
const float YAW_MIN_ACTIVE_PWM         = 8.0f;
const float GYRO_NOISE_FLOOR_DPS       = 0.30f;

// ================= UI STATE MACHINE =================
enum UIMode {
  DASHBOARD,
  TUNE_YAW_KP,
  TUNE_YAW_KI,
  TUNE_YAW_KD,
  TUNE_MAX_VEL,
  TUNE_LIN_SCALE,
  IMU_VIEW,
  ODOMETRY_VIEW,
  WIFI_INFO,
  UI_MODE_COUNT
};
UIMode screenMode = DASHBOARD;

// ================= PENYIMPANAN FLASH =================
void loadPreferences() {
  pref.begin("ramabot_map2", false);
  maxLinearVel       = pref.getFloat("maxVel", 0.695f);
  normalSpeedPct     = pref.getFloat("normPct", 0.65f);
  linearScale        = pref.getFloat("linScale", 1.00f);
  Kp_yaw             = pref.getFloat("Kp_yaw", 1.60f);
  Ki_yaw             = pref.getFloat("Ki_yaw", 0.02f);
  Kd_yaw             = pref.getFloat("Kd_yaw", 0.14f);
  gyroZOffset        = pref.getFloat("gzOff", 0.0f);
  headingLockEnabled = pref.getBool("hLock", true);
}

void savePreferences() {
  pref.putFloat("maxVel", maxLinearVel);
  pref.putFloat("normPct", normalSpeedPct);
  pref.putFloat("linScale", linearScale);
  pref.putFloat("Kp_yaw", Kp_yaw);
  pref.putFloat("Ki_yaw", Ki_yaw);
  pref.putFloat("Kd_yaw", Kd_yaw);
  pref.putFloat("gzOff", gyroZOffset);
  pref.putBool("hLock", headingLockEnabled);
}

void resetDefaultPreferences() {
  maxLinearVel       = 0.695f;
  normalSpeedPct     = 0.65f;
  linearScale        = 1.00f;
  Kp_yaw             = 1.60f;
  Ki_yaw             = 0.02f;
  Kd_yaw             = 0.14f;
  headingLockEnabled = true;
  savePreferences();
}

// ================= FORWARD DECLARATIONS =================
float normalizeAngleRad(float a);
float normalizeAngleDeg(float a);
void calibrateGyroZ();
void resetPoseAndOdometry();

void initOdometry();

void setupESPNow();
void sendTelemetryESPNow();
void processRosSerial();

void setupMotorPWM();
void driveMotors(float pwmA, float pwmB);
void taskControlLoop(void *parameter);

void setupArduinoOTA();
void setupWebServer();
void sendUdpTelemetry();
void drawHeadingCompass(int centerX, int centerY, int radius, float angleDeg);
void handlePhysicalButtons();
void renderLcdScreen();
void taskUiAndComm(void *parameter);