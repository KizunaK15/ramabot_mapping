#pragma once

// ================= KONFIGURASI WI-FI ROUTER & OTA (KHUSUS MAPPING) =================
const char* WIFI_SSID    = "Ramabot";
const char* WIFI_PASS    = "RamabotXavier";
const char* OTA_HOSTNAME = "ramabot-map"; // Akses via http://ramabot-map.local
const uint16_t UDP_PORT  = 8889;

WebServer server(80);
WiFiUDP udp;
bool wifiConnected = false;
volatile bool otaInProgress = false;
uint8_t ipOctet3 = 0;
uint8_t ipOctet4 = 0;

// ================= [SESUAIKAN] DEFINISI PIN M5STACK CORE GREY =================
const int ENCCA = 13;
const int ENCCB = 34;
const int M1A   = 12;
const int M1B   = 2;

const int ENCDA = 17;
const int ENCDB = 26;
const int M2A   = 5;
const int M2B   = 15;

const int PWM_FREQ = 5000;
const int PWM_RES  = 8;

const int CH_M1A = 4;
const int CH_M1B = 5;
const int CH_M2A = 6;
const int CH_M2B = 7;

// ================= [SESUAIKAN] SPESIFIKASI MEKANIK ROBOT MAPPING =================
const float TOTAL_PPR_LEFT  = 1440.0f; // Ubah sesuai total pulsa/putaran roda kiri mapping
const float TOTAL_PPR_RIGHT = 1440.0f; // Ubah sesuai total pulsa/putaran roda kanan mapping

const float WHEEL_RADIUS = 0.042f;     // [METER] Ubah sesuai jari-jari roda robot mapping
const float TRACK_WIDTH  = 0.300f;     // [METER] Ubah sesuai jarak antar pusat roda kiri-kanan
const float DIST_PER_PULSE_LEFT  = (2.0f * PI * WHEEL_RADIUS) / TOTAL_PPR_LEFT;
const float DIST_PER_PULSE_RIGHT = (2.0f * PI * WHEEL_RADIUS) / TOTAL_PPR_RIGHT;
const float ENC_SIGN_LEFT  = 1.0f;     // Ubah ke -1.0f jika roda maju tapi pulseA berkurang
const float ENC_SIGN_RIGHT = 1.0f;     // Ubah ke -1.0f jika roda maju tapi pulseB berkurang
const float GYRO_Z_SIGN    = 1.0f;     // Ubah ke -1.0f jika koreksi putar gyro terbalik

// ================= MAC ADDRESS M5STICKC PLUS (HATMINIJOYC) =================
uint8_t stickAddress[] = {0x50, 0x02, 0x91, 0x92, 0x3A, 0x78};
esp_now_peer_info_t peerInfo;
Preferences pref;

// ================= STRUKTUR DATA ESP-NOW (IDENTIK DENGAN M5STICKC PLUS) =================
typedef struct struct_joy_msg {
    uint8_t header;           // 0xAB
    int8_t  linear_speed;     // -100 s/d 100
    int8_t  angular_speed;    // -100 s/d 100
    bool    btn_estop;
    bool    btn_test_straight;
    bool    btn_reset_pose;
    int8_t  snap_turn;        // -1: Putar kanan 90 deg, +1: Putar kiri 90 deg
} struct_joy_msg;
struct_joy_msg incomingJoy;

typedef struct struct_telemetry {
    float   targetL;
    float   targetR;
    float   rpmL;
    float   rpmR;
    float   pwmL;
    float   pwmR;
    float   headingDeg;
    float   targetHeading;
    float   yawPidOut;
    float   odoX;
    float   odoY;
    float   odoTheta;
    long    pulseL;
    long    pulseR;
    bool    headingLock;
    bool    motorRunning;
    bool    eStop;
    uint8_t ip3;
    uint8_t ip4;
} struct_telemetry;
struct_telemetry telemetryData;

// ================= VARIABEL ENKODER & ODOMETRI =================
volatile long pulseA = 0;
volatile long pulseB = 0;
long prevPulseA = 0;
long prevPulseB = 0;

float rpmA = 0.0f, rpmB = 0.0f;
float odoX = 0.0f, odoY = 0.0f, odoTheta = 0.0f;
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

// ================= VARIABEL IMU & HEADING =================
float accX = 0, accY = 0, accZ = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;
float magX = 0, magY = 0, magZ = 0;

float gyroZOffset = 0.0f;
float headingDeg = 0.0f;
float targetHeading = 0.0f;
bool  headingLockEnabled = true;

// ================= VARIABEL KONTROL & DUAL-LOOP PID =================
bool motorRunning = true;
bool eStopActive = false;
int  testMode = 1; // 1: Closed-Loop PID, 0: Open-Loop
unsigned long lastJoyRecvTime = 0;

bool webStepActive = false;
float webStepTargetRPM = 0.0f;
unsigned long webStepEndTime = 0;

float maxRPM = 100.0f;
float targetRPM_A = 0.0f, targetRPM_B = 0.0f;
float outputPWM_A = 0.0f, outputPWM_B = 0.0f;

// Inner-Loop PID (Kecepatan Roda)
float Kp = 0.37f, Ki = 0.01f, Kd = 0.04f;
float errSum_A = 0.0f, lastErr_A = 0.0f;
float errSum_B = 0.0f, lastErr_B = 0.0f;

// Outer-Loop PID (Auto-Correction / Heading Lock)
float Kp_yaw = 0.8f, Ki_yaw = 0.00f, Kd_yaw = 0.05f;
float yawErrSum = 0.0f, lastYawErr = 0.0f, yawPidOut = 0.0f;
unsigned long turnSettlingTimer = 0;
unsigned long yawZeroTimer = 0;
const unsigned long ENTER_PID_DELAY_MS = 300;
const unsigned long YAW_AUTO_OFF_MS    = 1000;
const float YAW_DEADBAND_DEG           = 0.5f;

// ================= UI STATE MACHINE =================
enum UIMode {
  DASHBOARD,
  TUNE_VEL_KP, TUNE_VEL_KI, TUNE_VEL_KD,
  TUNE_YAW_KP, TUNE_YAW_KD,
  TUNE_MAX_RPM,
  IMU_VIEW,
  ODOMETRY_VIEW,
  WIFI_INFO,
  UI_MODE_COUNT
};
UIMode screenMode = DASHBOARD;

// ================= PENYIMPANAN FLASH (PREFERENCES) =================
void loadPreferences() {
  pref.begin("ramabot_map", false);
  maxRPM             = pref.getFloat("maxRPM", 100.0f);
  Kp                 = pref.getFloat("Kp", 0.37f);
  Ki                 = pref.getFloat("Ki", 0.01f);
  Kd                 = pref.getFloat("Kd", 0.04f);
  Kp_yaw             = pref.getFloat("Kp_yaw", 0.80f);
  Ki_yaw             = pref.getFloat("Ki_yaw", 0.00f);
  Kd_yaw             = pref.getFloat("Kd_yaw", 0.05f);
  gyroZOffset        = pref.getFloat("gzOff", 0.0f);
  headingLockEnabled = pref.getBool("hLock", true);
}

void savePreferences() {
  pref.putFloat("maxRPM", maxRPM);
  pref.putFloat("Kp", Kp);
  pref.putFloat("Ki", Ki);
  pref.putFloat("Kd", Kd);
  pref.putFloat("Kp_yaw", Kp_yaw);
  pref.putFloat("Ki_yaw", Ki_yaw);
  pref.putFloat("Kd_yaw", Kd_yaw);
  pref.putFloat("gzOff", gyroZOffset);
  pref.putBool("hLock", headingLockEnabled);
}
