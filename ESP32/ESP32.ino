#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h> 

// ════════════════════════════════════════════════════════════════
//  PIN MOTOR (L298N / driver H-bridge)
// ════════════════════════════════════════════════════════════════
#define L_IN1  27
#define L_IN2  14
#define R_IN1  26
#define R_IN2  25

// ════════════════════════════════════════════════════════════════
//  SERIAL MAPPING
// ════════════════════════════════════════════════════════════════
#define NAV_SERIAL  Serial2
#define RXD2        16
#define TXD2        17
#define MEGA_BAUD   250000   


#define MP3_SERIAL  Serial // pakai UART1 hardware, bukan Serial (UART0/USB)
// #define MP3_RX      4           // pin RX ESP32 <- TX DFPlayer
// #define MP3_TX      5           // pin TX ESP32 -> RX DFPlayer
#define MP3_BAUD    9600

// ════════════════════════════════════════════════════════════════
//  WIFI & MQTT
// ════════════════════════════════════════════════════════════════
const char* WIFI_SSID     = "TECNO POVA Neo 2";
const char* WIFI_PASS     = "00000000";
const char* MQTT_BROKER   = "172.20.10.4";
const int   MQTT_PORT     = 1883;
const int   MQTT_BUF_SIZE = 4096;

const char* T_TELEMETRY    = "robot/telemetry";
const char* T_GPS_RAW      = "robot/gps/raw";
const char* T_GPS_FILTERED = "robot/gps/filtered";
const char* T_COMMAND      = "robot/command";
const char* T_WP_IN        = "robot/waypoint/set";
const char* T_ACK = "robot/ack";

const char* NTP_SERVER_1   = "pool.ntp.org";
const char* NTP_SERVER_2   = "time.google.com";
const long  GMT_OFFSET_SEC = 7 * 3600;   // WIB = UTC+7
const int   DST_OFFSET_SEC = 0;
bool ntpSynced = false;
unsigned long lastNtpCheck = 0;

uint32_t pktSeq = 0;
uint32_t sendTime[512];
float lastDelay = 0;
float jitter = 0;
float delayMs = 0;
// ════════════════════════════════════════════════════════════════
//  PARAMETER ROBOT
// ════════════════════════════════════════════════════════════════
const float WHEEL_BASE     = 0.52f;
const float WHEEL_RADIUS   = 0.05f;
const float V_MAX          = 1.4f;
const int   PWM_FREQ       = 1000;
const int   PWM_BITS       = 8;
int baseSpeed = 125;
const int BUMP_SPEED_PWM = 55;   // [SPEEDBUMP] ~0.3 m/s @ V_MAX 1.4 (0.3/1.4*255)


const int REVERSE_PWM_LIMIT = 60;   // 0..255, atur sesuai kebutuhan

// ════════════════════════════════════════════════════════════════
//  PID1 — OMEGA correction (Mode 1 & 2, koreksi omega DWA)
// ════════════════════════════════════════════════════════════════
float Kp = 1.53f;
float Ki = 0.0132;
float Kd = 0.252;
float pid_integral  = 0.0f;
float pid_lastError = 0.0f;

// ════════════════════════════════════════════════════════════════
//  PID2 — HEADING ERROR (Mode 3, navigasi tanpa DWA)
//  Lebih agresif karena ini satu-satunya kontrol arah
// ════════════════════════════════════════════════════════════════
float Kp2 = 1.5036f;
float Ki2 = 0.0488f;
float Kd2 = 0.5788f;
float pid2_integral  = 0.0f;
float pid2_lastError = 0.0f;
const float MODE3_BASE_SPEED = 0.4f;  // kecepatan dasar mode 3

// ════════════════════════════════════════════════════════════════
//  STATE TELEMETRI
// ════════════════════════════════════════════════════════════════
struct TelemetryState {
  float  lat, lon;
  float  lat0, lon0;
  float  gps_x, gps_y;
  float  x_ekf, y_ekf;

  float  headingErr;
  float  dist;
  float  trv;
  float  speed;
  float  heading;
  float  targetHeading;
  float  dwa_v;
  float  dwa_omega;

  float  vL, vR;
  float  odom_x, odom_y;

  int    wp;
  int    newWP;
  float  wlat, wlon;

  float  q_val, r_val;
  float  hekf;

  int    nav_mode;           // [FIX-1] navigasi: 0-3
  int    bump;               // [SPEEDBUMP] 1=speedbump terdeteksi
  char   delivery_status[12]; // [FIX-2] operasional: STANDBY/PICKUP/DELIVERY

  float  bat;
  float  load;

  float  eta_raw, eta_kf;
  int    mag_cal, sys_cal;

  unsigned long lastUpdate;
};

TelemetryState S = {};
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// ════════════════════════════════════════════════════════════════
//  MOTOR STATE
// ════════════════════════════════════════════════════════════════
struct MotorState {
  int leftPWM;
  int rightPWM;
};

MotorState motorOut       = {0, 0};
unsigned long lastMegaPacket = 0;

// ════════════════════════════════════════════════════════════════
//  NETWORK
// ════════════════════════════════════════════════════════════════
WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);

unsigned long lastWifiRetry = 0;
unsigned long lastMqttRetry = 0;
unsigned long lastPublish   = 0;
const unsigned long PUB_INTERVAL   = 200;
const unsigned long WIFI_RETRY_MS  = 5000;
const unsigned long MQTT_RETRY_MS  = 3000;
const unsigned long MOTOR_WATCHDOG = 500;

bool mqttWasConnected = false;

// ════════════════════════════════════════════════════════════════
//  SERIAL BUFFER
// ════════════════════════════════════════════════════════════════
char navBuf[768];
int  navIdx = 0;

// ════════════════════════════════════════════════════════════════
//  DFPLAYER MINI — DRIVER
// ════════════════════════════════════════════════════════════════
struct MP3Command {
  uint8_t cmd;
  uint16_t param;
};

QueueHandle_t mp3Queue;

void dfSendCmd(uint8_t cmd, uint16_t param) {
  uint8_t buf[10];
  buf[0] = 0x7E;
  buf[1] = 0xFF;
  buf[2] = 0x06;
  buf[3] = cmd;
  buf[4] = 0x00;
  buf[5] = (uint8_t)(param >> 8);
  buf[6] = (uint8_t)(param & 0xFF);
  int16_t chk = -(buf[1] + buf[2] + buf[3] + buf[4] + buf[5] + buf[6]);
  buf[7] = (uint8_t)(chk >> 8);
  buf[8] = (uint8_t)(chk & 0xFF);
  buf[9] = 0xEF;
  MP3_SERIAL.write(buf, 10);
}

void mp3PlayTrack(uint16_t track) {
  MP3Command c = {0x03, track};
  xQueueSend(mp3Queue, &c, 0);
}
void mp3SetVolume(uint8_t vol) {
  MP3Command c = {0x06, vol};
  xQueueSend(mp3Queue, &c, 0);
}
void mp3Stop() {
  MP3Command c = {0x0E, 0};
  xQueueSend(mp3Queue, &c, 0);
}
void mp3Reset() {
  MP3Command c = {0x0C, 0};
  xQueueSend(mp3Queue, &c, 0);
}

void TaskMP3(void *pvParameters) {
  vTaskDelay(pdMS_TO_TICKS(1500));
  dfSendCmd(0x0C, 0);
  vTaskDelay(pdMS_TO_TICKS(1000));
  dfSendCmd(0x06, 30);
  vTaskDelay(pdMS_TO_TICKS(200));

  MP3Command c;
  for (;;) {
    if (xQueueReceive(mp3Queue, &c, pdMS_TO_TICKS(100)) == pdTRUE) {
      dfSendCmd(c.cmd, c.param);
      vTaskDelay(pdMS_TO_TICKS(200));
    }
  }
}

// ════════════════════════════════════════════════════════════════
//  MOTOR HELPERS
// ════════════════════════════════════════════════════════════════
int applyDeadzone(int pwm) {
  if (pwm == 0) return 0;

  int minPWM = 60;

  if (pwm > 0)
    return minPWM + (pwm * (255 - minPWM) / 255);
  else
    return -minPWM + (pwm * (255 - minPWM) / 255);
}

void driveMotor(int left, int right) {

  left = applyDeadzone(left);
  right = applyDeadzone(right);

  if (left > 0) {
    ledcWrite(0, left);
    ledcWrite(1, 0);
  }
  else {
    ledcWrite(0, 0);
    ledcWrite(1, -left);
  }

  if (right > 0) {
    ledcWrite(2, right);
    ledcWrite(3, 0);
  }
  else {
    ledcWrite(2, 0);
    ledcWrite(3, -right);
  }
}

void stopMotor() {
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 0);
}

float computePID(float error) {
  pid_integral  = constrain(pid_integral + error, -100.0f, 100.0f);
  float deriv   = error - pid_lastError;
  pid_lastError = error;
  return (Kp * error) + (Ki * pid_integral) + (Kd * deriv);
}

// [v5] PID2 untuk mode 3 — heading error based navigation
float computePID2(float error) {
  pid2_integral  = constrain(pid2_integral + error, -100.0f, 100.0f);
  float deriv    = error - pid2_lastError;
  pid2_lastError = error;
  return (Kp2 * error) + (Ki2 * pid2_integral) + (Kd2 * deriv);
}

void computeMotorPWM(float v, float omega, int &leftPWM, int &rightPWM) {
  float vL = v - omega * (WHEEL_BASE / 2.0f);
  float vR = v + omega * (WHEEL_BASE / 2.0f);

  float maxV = max(fabs(vL), fabs(vR));
  if (maxV > V_MAX) {
    vL = vL / maxV * V_MAX;
    vR = vR / maxV * V_MAX;
  }

  leftPWM  = (int)((vL / V_MAX) * 255);
  rightPWM = (int)((vR / V_MAX) * 255);

  // [OPSI B] Roda hanya boleh MUNDUR saat v < 0. Saat v >= 0 (maju / belok di
  // tempat), PWM tiap roda dibatasi [0, 255] -> roda dalam paling lambat
  // BERHENTI, tidak berputar mundur. Saat v < 0, dibatasi [-255, 0].
  if (v >= 0.0f) {
    leftPWM  = constrain(leftPWM,  0, 255);
    rightPWM = constrain(rightPWM, 0, 255);
  } else {
    leftPWM  = constrain(leftPWM,  -255, 0);
    rightPWM = constrain(rightPWM, -255, 0);
  }
}

// ════════════════════════════════════════════════════════════════
//  TASK CONTROL (Core 1)
//  [v5] Dual PID:
//    Mode 1&2: PID1 koreksi omega (tambahan ke DWA omega)
//    Mode 3:   PID2 heading error → generate v dan omega sendiri
// ════════════════════════════════════════════════════════════════
void TaskControl(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(10);

  for (;;) {
    float localV, localOmega, localErr, localDist;
    int   localNavMode;
    unsigned long localLastPkt;

    portENTER_CRITICAL(&mux);
    localV        = S.dwa_v;
    localOmega    = S.dwa_omega;
    localErr      = S.headingErr;
    localDist     = S.dist;
    localNavMode  = S.nav_mode;
    int localBump = S.bump;
    localLastPkt  = lastMegaPacket;
    portEXIT_CRITICAL(&mux);

    bool timeout = (millis() - localLastPkt) > MOTOR_WATCHDOG;

    // Stop motor jika nav_mode==0 (standby) atau timeout
    if (localNavMode == 0 || timeout) {
      pid_integral   = 0.0f;
      pid_lastError  = 0.0f;
      pid2_integral  = 0.0f;
      pid2_lastError = 0.0f;
      stopMotor();
      vTaskDelayUntil(&xLastWakeTime, xPeriod);
      continue;
    }

    int leftPWM, rightPWM;

    if (localNavMode == 3) {
      // ════════   ══════════════════  ═══════════════════   ═══════
      //  MODE 3: Heading Error PID (tanpa DWA)
      //  Robot bergerak maju dengan kecepatan dasar,
      //  PID2 mengkoreksi omega berdasarkan heading error
      // ═══════════════════════════════════════════════════════
      // [FIX-SPEED] Pisahkan kecepatan maju dari koreksi belok:
      //  - Dead-band 1° dipertahankan agar tidak bergetar di sekitar 0.
      //  - Pakai fabs() => simetris kiri/kanan (dulu nilai bertanda: belok
      //    satu arah selalu pelan).
      //  - speedFactor mulus 1.0→0.6 TANPA lompatan ambang "60" yang bikin
      //    kecepatan berdenyut laju-pelan saat jalan lurus.
      float err = (fabs(localErr) < 1.0f) ? 0.0f : localErr;
      float errorRatio   = constrain(fabs(localErr) / 90.0f, 0.0f, 1.0f);
      float speedFactor  = 1.0f - (0.4f * errorRatio);   // range: 1.0 → 0.6 (mulus)
      int   adaptedSpeed = (int)(baseSpeed * speedFactor);
      // [SPEEDBUMP] Batasi kecepatan dasar saat speedbump terdeteksi (~0.3 m/s).
      if (localBump) adaptedSpeed = min(adaptedSpeed, BUMP_SPEED_PWM);

      // ── PID menghasilkan nilai koreksi belok ─────────────────────────
      float turn = computePID2(err);
      turn = constrain(turn, -(float)adaptedSpeed, (float)adaptedSpeed);
      // [FIX] gunakan variabel luar (jangan deklarasi ulang) agar motorOut benar
      leftPWM  = constrain((int)(adaptedSpeed + turn), 0, 255);
      rightPWM = constrain((int)(adaptedSpeed - turn), 0, 255);

      driveMotor(leftPWM, rightPWM);

    } else {
      // ══════════════════════════════════════════════════   ════
      //  MODE 1 & 2: DWA + PID1 correction
      //  v/omega dari Python DWA, PID1 koreksi tambahan kecil
      // ═══════════════════════════════════════════════════════
      // Reset PID2 saat tidak dipakai
      pid2_integral  = 0.0f;
      pid2_lastError = 0.0f;

      // ═══════════════════════════════════════════════════════════════
      //  [INTEGRASI v5] AKTUATOR MURNI — FEEDFORWARD KINEMATIK
      //  (v, omega) datang dari Python (PID global / DWA via arbiter).
      //  ESP32 TIDAK lagi ber-PID / mixing sendiri di mode 1&2.
      //  Konversi: vL = v - omega*(L/2) ; vR = v + omega*(L/2)
      //  Sama persis konvensi tanda dengan Mode 3 (kiri = base - turn).
      // ═══════════════════════════════════════════════════════════════
      computeMotorPWM(localV, localOmega, leftPWM, rightPWM);
      driveMotor(leftPWM, rightPWM);
    }

    portENTER_CRITICAL(&mux);
    motorOut.leftPWM  = leftPWM;
    motorOut.rightPWM = rightPWM;
    portEXIT_CRITICAL(&mux);

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

// ════════════════════════════════════════════════════════════════
//  PARSER SERIAL
// ════════════════════════════════════════════════════════════════
float extractFloat(const char *data, const char *key) {
  const char *p = strstr(data, key);
  if (!p) return 0.0f;
  p += strlen(key);
  return strtof(p, nullptr);
}
double extractDouble(const char *data, const char *key) {
  const char *p = strstr(data, key);
  if (!p) return 0.0;
  p += strlen(key);
  return strtod(p, nullptr);
}
int extractInt(const char *data, const char *key) {
  const char *p = strstr(data, key);
  if (!p) return 0;
  p += strlen(key);
  return (int)strtol(p, nullptr, 10);
}

// [FIX-2] Extract string field (e.g. DSTAT:DELIVERY)
void extractString(const char *data, const char *key, char *out, int maxLen) {
  const char *p = strstr(data, key);
  if (!p) { out[0] = '\0'; return; }
  p += strlen(key);
  int i = 0;
  while (*p && *p != ',' && *p != '>' && i < maxLen - 1) {
    out[i++] = *p++;
  }
  out[i] = '\0';
}

int prevNavMode = -1;
char prevDeliveryStat[12] = "";

void parseControlPacket(const char *data) {
  int   mode = 0, bump = 0;
  float err = 0, dist = 0, dv = 0, dw = 0;
  if (sscanf(data, "CTL,%d,%f,%f,%f,%f,%d", &mode, &err, &dist, &dv, &dw, &bump) != 6) return;
  if (fabs(err) > 360.0f) return;
  if (dist < 0 || dist > 2000.0f) return;
  portENTER_CRITICAL(&mux);
  S.nav_mode   = mode;
  S.headingErr = err;
  S.dist       = dist;
  S.dwa_v      = dv;
  S.dwa_omega  = dw;
  S.bump       = bump;
  lastMegaPacket = millis();
  portEXIT_CRITICAL(&mux);
}

void parseMegaPacket(const char *data) {
  // [v5] Handle NOTIFY events dari Mega (suara)
  if (strstr(data, "NOTIFY:ROUTE"))   { mp3PlayTrack(2); return; }
  if (strstr(data, "NOTIFY:DOOR"))    { mp3PlayTrack(3); return; }
  // if (strstr(data, "NOTIFY:PINFAIL")) { mp3PlayTrack(7); return; }
  if (strstr(data, "NOTIFY:DONE"))    { mp3PlayTrack(5); return; }
  // Handle PIN OK dari Mega keypad
  if (strstr(data, "PINOK,1"))        { mp3PlayTrack(3); return; }
  // if (strstr(data, "PINOK,0"))        { mp3PlayTrack(7); return; }
  if (strstr(data, "WP,1"))           { mp3PlayTrack(6); return; }
  // Play Musik
  if (strstr(data, "123,1"))        { mp3PlayTrack(7); return; }
  if (strstr(data, "456,1"))        { mp3PlayTrack(8); return; }

  if (!strstr(data, "LAT:") || !strstr(data, "MODE:")) return;

  TelemetryState tmp = {};

  tmp.lat           = (float)extractDouble(data, "LAT:");
  tmp.lon           = (float)extractDouble(data, "LON:");
  tmp.lat0          = (float)extractDouble(data, "LAT0:");
  tmp.lon0          = (float)extractDouble(data, "LON0:");

  tmp.nav_mode      = extractInt  (data, "MODE:");
  tmp.bump          = extractInt  (data, "BUMP:");
  extractString(data, "DSTAT:", tmp.delivery_status, sizeof(tmp.delivery_status));
  if (tmp.delivery_status[0] == '\0') {
    strncpy(tmp.delivery_status, "STANDBY", sizeof(tmp.delivery_status));
  }

  tmp.headingErr    = extractFloat(data, "ERR:");
  tmp.dist          = extractFloat(data, "DIS:");
  tmp.trv           = extractFloat(data, "TRV:");
  tmp.speed         = extractFloat(data, "SPEED:");
  tmp.heading       = extractFloat(data, "HDG:");
  tmp.targetHeading = extractFloat(data, "THEAD:");
  tmp.dwa_v         = extractFloat(data, "DWA_V:");
  tmp.dwa_omega     = extractFloat(data, "DWA_W:");

  tmp.vL            = extractFloat(data, "VL:");
  tmp.vR            = extractFloat(data, "VR:");
  tmp.gps_x         = extractFloat(data, "GPSX:");
  tmp.gps_y         = extractFloat(data, "GPSY:");
  tmp.x_ekf         = extractFloat(data, "XEKF:");
  tmp.y_ekf         = extractFloat(data, "YEKF:");
  tmp.odom_x        = extractFloat(data, "XODO:");
  tmp.odom_y        = extractFloat(data, "YODO:");
  tmp.hekf          = extractFloat(data, "HEKF:");

  tmp.wp            = extractInt  (data, "WP:");
  tmp.newWP         = extractInt  (data, "NEWWP:");
  tmp.wlat          = (float)extractDouble(data, "TLAT:");
  tmp.wlon          = (float)extractDouble(data, "TLON:");

  tmp.q_val         = extractFloat(data, "Q:");
  tmp.r_val         = extractFloat(data, "R:");

  tmp.bat           = extractFloat(data, "BAT:");
  tmp.load          = extractFloat(data, "LOAD:");

  tmp.eta_raw       = extractFloat(data, "ETARAW:");
  tmp.eta_kf        = extractFloat(data, "ETAEKF:");

  tmp.mag_cal       = extractInt  (data, "MAG:");
  tmp.sys_cal       = extractInt  (data, "SYSCAL:");

  tmp.lastUpdate    = millis();

  if (fabs(tmp.headingErr) > 360.0f) return;
  if (tmp.dist < 0 || tmp.dist > 2000.0f) return;

  portENTER_CRITICAL(&mux);
  S = tmp;
  lastMegaPacket = millis();

  if (tmp.newWP == 1) {
    pid_integral  = 0.0f;
    pid_lastError = 0.0f;
  }
  portEXIT_CRITICAL(&mux);

  // [FIX-4] Trigger MP3 berdasarkan delivery_status (bukan nav_mode)
  if (strcmp(tmp.delivery_status, prevDeliveryStat) != 0 && prevDeliveryStat[0] != '\0') {
    if      (strcmp(tmp.delivery_status, "DELIVERY") == 0) mp3PlayTrack(4);
    // else if (strcmp(tmp.delivery_status, "PICKUP")   == 0) mp3PlayTrack(3);
    // else if (strcmp(tmp.delivery_status, "STANDBY")  == 0) mp3Play Track(3);
  }
  strncpy(prevDeliveryStat, tmp.delivery_status, sizeof(prevDeliveryStat));

  prevNavMode = tmp.nav_mode;
}

// ════════════════════════════════════════════════════════════════
//  TASK SERIAL (Core 0)
// ════════════════════════════════════════════════════════════════
void TaskSerial(void *pvParameters) {
  for (;;) {
    while (NAV_SERIAL.available()) {
      char c = NAV_SERIAL.read();

      if (c == '<') {
        navIdx = 0;
      } else if (c == '>') {
        navBuf[navIdx] = '\0';
        if (strncmp(navBuf, "CTL,", 4) == 0) parseControlPacket(navBuf);
        else                                 parseMegaPacket(navBuf);
        navIdx = 0;
      } else {
        if (navIdx < (int)(sizeof(navBuf) - 1)) {
          navBuf[navIdx++] = c;
        } else {
          navIdx = 0;
        }
      }
    }
    // [FIX] Teruskan PWM AKTUAL motor ke Mega via serial (independen dari MQTT).
    // Mega akan forward sebagai <PWM,l,r> -> Python -> HTML. Backend tidak pakai
    // MQTT, jadi ini satu-satunya jalur PWM ASLI ESP32 menuju dashboard.
    static unsigned long lastPwmTx = 0;
    if (millis() - lastPwmTx >= 100) {            // ~10 Hz
      lastPwmTx = millis();
      int lpwm, rpwm;
      portENTER_CRITICAL(&mux);
      lpwm = motorOut.leftPWM;
      rpwm = motorOut.rightPWM;
      portEXIT_CRITICAL(&mux);
      char pwmBuf[32];
      snprintf(pwmBuf, sizeof(pwmBuf), "<PWM,%d,%d>", lpwm, rpwm);
      NAV_SERIAL.println(pwmBuf);
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
uint64_t getEpochMillis() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (uint64_t)tv.tv_sec * 1000ULL + (tv.tv_usec / 1000ULL);
}

void checkNtpSync()
{
    if (millis() - lastNtpCheck < 2000)
        return;

    lastNtpCheck = millis();

    struct tm timeinfo;

    if (getLocalTime(&timeinfo, 5))
    {
        ntpSynced = true;
    }
    else
    {
        ntpSynced = false;
        // opsional: retry configTime kalau lama gak sync
        static unsigned long lastRetry = 0;
        if (!ntpSynced && millis() - lastRetry > 30000) {
            lastRetry = millis();
            configTime(GMT_OFFSET_SEC, DST_OFFSET_SEC, NTP_SERVER_1, NTP_SERVER_2);
        }
    }
}
// ════════════════════════════════════════════════════════════════
//  MQTT CALLBACK
// ════════════════════════════════════════════════════════════════
void mqttCallback(char *topic, byte *payload, unsigned int len) {
  if (len == 0 || len >= 512) return;

  char msg[512];
  memcpy(msg, payload, len);
  msg[len] = '\0';

  if (strcmp(topic, T_WP_IN) == 0) {
    StaticJsonDocument<1024> doc;
    if (deserializeJson(doc, msg)) return;

    JsonArray arr = doc["waypoints"];
    if (arr.isNull()) return;

    int idx = 0;
    for (JsonObject wp : arr) {
      double wlat = wp["lat"].as<double>();
      double wlon = wp["lon"].as<double>();
      char buf[80];
      snprintf(buf, sizeof(buf), "<WP,%.6f,%.6f,%d>", wlat, wlon, idx);
      NAV_SERIAL.println(buf);
      delay(20);
      idx++;
    }
    char totBuf[32];
    snprintf(totBuf, sizeof(totBuf), "<TOTALWP,%d>", idx);
    NAV_SERIAL.println(totBuf);

    // [v4.1] Notifikasi suara: rute diterima dari MQTT
    mp3PlayTrack(2);
    return;
  }

  if (strcmp(topic, T_ACK) == 0){
      StaticJsonDocument<64> doc;

      if (deserializeJson(doc, msg))
          return;

      uint32_t seq = doc["seq"];

      uint32_t rtt = millis() - sendTime[seq % 512];

      delayMs = rtt / 2.0f;

      jitter = fabs(delayMs - lastDelay);

      lastDelay = delayMs;

      // Serial.printf(
      //     "ACK seq=%lu RTT=%lu ms Delay=%.2f ms Jitter=%.2f ms\n",
      //     seq,
      //     rtt,
      //     delayMs,
      //     jitter
      // );

      return;
  }

  if (strcmp(topic, T_COMMAND) == 0) {
    StaticJsonDocument<256> doc;
    if (deserializeJson(doc, msg)) return;

    if (doc.containsKey("mode")) {
      const char *m = doc["mode"] | "";
      if      (strcmp(m, "DELIVERY") == 0) NAV_SERIAL.println("<CMD:DELIVERY>");
      else if (strcmp(m, "PICKUP")   == 0) NAV_SERIAL.println("<CMD:PICKUP>");
      else if (strcmp(m, "STANDBY")  == 0) NAV_SERIAL.println("<CMD:STANDBY>");
      return;
    }

    const char *cmd = doc["cmd"] | "";

    if (strcmp(cmd, "open_door") == 0) {
      NAV_SERIAL.println("<CMD:DOOR>");
      mp3PlayTrack(3);
    }
    else if (strcmp(cmd, "reset_odo") == 0) NAV_SERIAL.println("<CMD:RESET>");
    else if (strcmp(cmd, "clear_wp")  == 0) NAV_SERIAL.println("<CMD:CLRWP>");
    else if (strcmp(cmd, "estop")     == 0) NAV_SERIAL.println("<CMD:STANDBY>");

    else if (strcmp(cmd, "set_speed")     == 0 ||
             strcmp(cmd, "set_dwa_speed") == 0 ||
             strcmp(cmd, "set_pid_speed") == 0) {
      float common    = doc["value"] | (doc["speed"] | -1.0f);
      bool  isDwaOnly = (strcmp(cmd, "set_dwa_speed") == 0);
      bool  isPidOnly = (strcmp(cmd, "set_pid_speed") == 0);
      float dwaVal    = doc["dwa"] | (isPidOnly ? -1.0f : common);
      float pidVal    = doc["pid"] | (isDwaOnly ? -1.0f : common);
      char  buf[48];
      if (dwaVal >= 0.0f) {
        snprintf(buf, sizeof(buf), "<SPD,dwa,%.3f>", dwaVal);
        NAV_SERIAL.println(buf);
      }
      if (pidVal >= 0.0f) {
        snprintf(buf, sizeof(buf), "<SPD,pid,%.3f>", pidVal);
        NAV_SERIAL.println(buf);
      }
    }

    else if (strcmp(cmd, "play_mp3") == 0) {
      uint16_t track = doc["track"] | 1;
      if (track >= 1 && track <= 255) mp3PlayTrack(track);
    }
    else if (strcmp(cmd, "mp3_vol") == 0) {
      uint8_t vol = (uint8_t)(doc["vol"] | 30);
      vol = constrain(vol, 0, 30);
      mp3SetVolume(vol);
    }
    else if (strcmp(cmd, "mp3_stop") == 0) {
      mp3Stop();
    }
  }
}

// ════════════════════════════════════════════════════════════════
//  WIFI & MQTT
// ════════════════════════════════════════════════════════════════
void maintainWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  if (millis() - lastWifiRetry < WIFI_RETRY_MS) return;
  lastWifiRetry = millis();
  WiFi.disconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

void maintainMQTT() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (mqtt.connected()) return;
  if (millis() - lastMqttRetry < MQTT_RETRY_MS) return;
  lastMqttRetry = millis();

  String clientId = "ARGO-ESP32-" + String((uint32_t)ESP.getEfuseMac(), HEX);

  if (mqtt.connect(clientId.c_str())) {
    mqtt.subscribe(T_COMMAND);
    mqtt.subscribe(T_ACK);
    mqtt.subscribe(T_WP_IN);
    NAV_SERIAL.println("MQTT:OK");
    // Serial.println("MQTT:OK");     
    mqttWasConnected = true;
    // mp3PlayTrack(6);
  } else {
    if (mqttWasConnected) {
      NAV_SERIAL.println("MQTT:FAIL");
      // Serial.println("MQTT:FAIL");
      mqttWasConnected = false;
    }
  }
}

// ════════════════════════════════════════════════════════════════
//  PUBLISH TELEMETRY
//  [FIX-4] modeStr dari delivery_status, [FIX-5] hasRef fix
// ═══════════════   ════════════════════════════════════════════════
void publishTelemetry() {
  if (!mqtt.connected()) return;
  if (millis() - lastPublish < PUB_INTERVAL) return;
  lastPublish = millis();

  TelemetryState snap;
  int leftPWM_snap, rightPWM_snap;

  portENTER_CRITICAL(&mux);
  snap          = S;
  leftPWM_snap  = motorOut.leftPWM;
  rightPWM_snap = motorOut.rightPWM;
  portEXIT_CRITICAL(&mux);

  double lat_kf = snap.lat;
  double lon_kf = snap.lon;

  // [FIX-5] Cek valid reference (lat0/lon0 non-zero)
  bool hasRef = (snap.lat0 != 0.0 && snap.lon0 != 0.0 &&
                 snap.lat0 > -15.0 && snap.lat0 < 10.0 &&
                 snap.lon0 >  90.0 && snap.lon0 < 145.0);

  if (hasRef && (snap.x_ekf != 0.0f || snap.y_ekf != 0.0f)) {
    // Pakai 111139 konsisten dengan Mega
    lat_kf = snap.lat0 + (snap.y_ekf / 111139.0);
    lon_kf = snap.lon0 + (snap.x_ekf / (111139.0 * cos(snap.lat0 * M_PI / 180.0)));
  }

  bool gpsValid = (fabs(snap.lat) > 1.0f && fabs(snap.lon) > 1.0f);

  // [FIX-4] delivery_status terpisah dari nav_mode
  const char* modeStr = snap.delivery_status;

  uint32_t currentSeq = pktSeq++;
  sendTime[currentSeq % 512] = millis();
  struct timeval tv;
  gettimeofday(&tv, NULL);

  uint64_t tsEpoch = getEpochMillis();

  StaticJsonDocument<3072> doc;

  doc["ts"] = millis();
  doc["seq"] = currentSeq;
  doc["ts_epoch"] = (double)tsEpoch;
  doc["time_synced"]    = ntpSynced;
  doc["gps_valid"]      = gpsValid;
  doc["lat_kf"]         = (float)lat_kf;
  doc["lon_kf"]         = (float)lon_kf;
  doc["lat_raw"]        = snap.lat;
  doc["lon_raw"]        = snap.lon;
  doc["lat0"]           = snap.lat0;
  doc["lon0"]           = snap.lon0;
  doc["gps_x"]          = snap.gps_x;
  doc["gps_y"]          = snap.gps_y;
  doc["x_ekf"]          = snap.x_ekf;
  doc["y_ekf"]          = snap.y_ekf;
  doc["x_odo"]          = snap.odom_x;
  doc["y_odo"]          = snap.odom_y;
  doc["heading"]        = snap.heading;
  doc["target_heading"] = snap.targetHeading;
  doc["heading_err"]    = snap.headingErr;
  doc["err"]            = snap.headingErr;
  doc["mag_cal"]        = snap.mag_cal;
  doc["sys_cal"]        = snap.sys_cal;
  doc["speed"]          = snap.speed;
  doc["omega"]          = snap.dwa_omega;
  doc["vL"]             = snap.vL;
  doc["vR"]             = snap.vR;
  doc["dwa_v"]          = snap.dwa_v;
  doc["dwa_w"]          = snap.dwa_omega;
  doc["pwm_l"]          = leftPWM_snap;
  doc["pwm_r"]          = rightPWM_snap;
  doc["distance"]       = snap.dist;
  doc["travelled"]      = snap.trv;
  doc["wp"]             = snap.wp;
  doc["wp_idx"]         = snap.wp;
  doc["tlat"]           = snap.wlat;
  doc["tlon"]           = snap.wlon;
  doc["target_lat"]     = snap.wlat;
  doc["target_lon"]     = snap.wlon;
  doc["eta"]            = snap.eta_kf;
  doc["eta_kf"]         = snap.eta_kf;
  doc["eta_raw"]        = snap.eta_raw;
  doc["nav_mode"]       = snap.nav_mode;
  doc["mode"]           = snap.nav_mode;
  doc["Cmd"]            = modeStr;
  doc["delivery_status"] = modeStr;
  doc["Q"]              = snap.q_val;
  doc["R"]              = snap.r_val;
  doc["battery"]        = snap.bat;
  doc["load"]           = snap.load;
  doc["weight"]         = snap.load;
  doc["gps_ready"]      = gpsValid;
  doc["gps_lost"]       = !gpsValid;
  doc["timestamp"]      = millis();
  doc["delay"] = delayMs;
  doc["jitter"] = jitter;

  char buf[3072];
  serializeJson(doc, buf);
  mqtt.publish(T_TELEMETRY, buf);

  // GPS Raw
  StaticJsonDocument<128> rawDoc;
  rawDoc["lat"] = snap.lat;
  rawDoc["lon"] = snap.lon;
  char rawBuf[128];
  serializeJson(rawDoc, rawBuf);
  // mqtt.publish(T_GPS_RAW, rawBuf);

  // GPS Filtered
  if (hasRef) {
    StaticJsonDocument<128> filtDoc;
    filtDoc["lat"] = (float)lat_kf;
    filtDoc["lon"] = (float)lon_kf;
    char filtBuf[128];
    serializeJson(filtDoc, filtBuf);
    // mqtt.publish(T_GPS_FILTERED, filtBuf);
  }
}

// ════════════════════════════════════════════════════════════════
//  TASK MQTT (Core 0)
// ════════════════════════════════════════════════════════════════
void TaskMQTT(void *pvParameters) {
  for (;;) {
    maintainWiFi();
    maintainMQTT();
    checkNtpSync();              // ← TAMBAHKAN INI
    if (mqtt.connected()) mqtt.loop();
    publishTelemetry();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ═══   ════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════════
void setup() {
  MP3_SERIAL.begin(MP3_BAUD);
  NAV_SERIAL.begin(MEGA_BAUD, SERIAL_8N1, RXD2, TXD2);
  mp3Queue = xQueueCreate(10, sizeof(MP3Command));

  pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT);
  pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT);

  ledcSetup(0, PWM_FREQ, PWM_BITS); ledcAttachPin(L_IN1, 0);
  ledcSetup(1, PWM_FREQ, PWM_BITS); ledcAttachPin(L_IN2, 1);
  ledcSetup(2, PWM_FREQ, PWM_BITS); ledcAttachPin(R_IN1, 2);
  ledcSetup(3, PWM_FREQ, PWM_BITS); ledcAttachPin(R_IN2, 3);
  stopMotor();

  memset(&S, 0, sizeof(S));
  strncpy(S.delivery_status, "STANDBY", sizeof(S.delivery_status));
  lastMegaPacket = millis();
  prevNavMode       = -1;
  prevDeliveryStat[0] = '\0';

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
    delay(200);
  }

  if (WiFi.status() == WL_CONNECTED) {

    configTime(
        GMT_OFFSET_SEC,
        DST_OFFSET_SEC,
        NTP_SERVER_1,
        NTP_SERVER_2
    );

    // Serial.println("Menunggu sinkronisasi NTP...");

    unsigned long ntpStart = millis();
    const unsigned long NTP_TIMEOUT_MS = 15000; // max tunggu 15 detik
    time_t now;
    while ((now = time(nullptr)) < 1704067200) {
        if (millis() - ntpStart > NTP_TIMEOUT_MS) {
            // Serial.println("NTP timeout, lanjut tanpa sync (akan dicoba lagi di background).");
            break;
        }
        delay(500);
        // Serial.print(".");
    }

    ntpSynced = (now >= 1704067200);
  }

  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setBufferSize(MQTT_BUF_SIZE);
  maintainMQTT();

  xTaskCreatePinnedToCore(TaskSerial,  "Serial",  4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(TaskControl, "Control", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(TaskMQTT,    "MQTT",    8192, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskMP3,     "MP3",     2048, NULL, 2, NULL, 0);

  mp3PlayTrack(1);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}