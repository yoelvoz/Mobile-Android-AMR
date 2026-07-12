#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include "HX711.h"

TinyGPSPlus gps;
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// ================= SERIAL KE ESP32 =================
#define NAV_SERIAL Serial3   // TX3=14 RX3=15

#define ENCODER1_A 3
#define ENCODER1_B 29
#define ENCODER2_A 2
#define ENCODER2_B 27

// ================= LOAD CELL =================
#define HX711_DT 5
#define HX711_SCK 6

HX711 scale;
float filteredLoad = 0;

// ================= SENSOR TEGANGAN ===========
#define VOLTAGE_PIN A0

// ================= KONFIGURASI ROBOT =================
const int PPR = 250;              // pulse per revolution
const float DIAMETER_RODA = 0.10; // meter (contoh 6.5 cm)
const float wheelBase = 0.52;
const int SAMPLE_TIME = 50;     // ms
const float KELILING = 3.14159 * DIAMETER_RODA;

// ================= SPEED ROBOT=================
long lastCount1 = 0;
long lastCount2 = 0;
float vL = 0;
float vR = 0;
float v = 0;
volatile long encoder1Count = 0;
volatile long encoder2Count = 0;
float speed1 = 0;
float speed2 = 0;
unsigned long lastTime = 0;
float distanceTravelled = 0;   // meter

// ================= STATE VARIABEL =================
float x = 0;
float y = 0;
float theta = 0;
float omega =  0;
float omega_enc = 0;
float odom_x = 0;
float odom_y = 0;
unsigned long lastEKF = 0;
bool gpsNewData = false;
bool wpJustAdvanced = false;

// ================= EKF =================
// covariance
float P[3][3] = {
  {1,0,0},
  {0,1,0},
  {0,0,1}
};

float Q_xy = 0.03;   
float Q_th = 0.008;  
float R_gps = 15;    
float R_imu = 0.03;  


float dt = SAMPLE_TIME / 1000.0;

float imuTheta = 0;
int gpsUpdateCount = 0;
unsigned long lastGpsUpdate = 0;
// ================= ETA =================
// float ETA     = 0;
float ETA_raw = 0;   
float ETA_kf  = 0;   
float target_x_local = 0;  
float target_y_local = 0;  


// ================= GPS =================
float gps_x = 0;
float gps_y = 0;
double lat0 = 0;
double lon0 = 0;
double currentlat = 0;
double currentlon = 0;
bool gpsReference = false;

// ================= HEADING & MOVING =================
float filteredHeading = 0;
float alpha = 0.15;  
float distanceToTarget = 0;
float headingError = 0;
int mode= 0;
float targetHeading = 0;
bool newOdo = false;
bool manualStop = false;

unsigned long lastWaypointTime = 0;
const unsigned long MIN_WP_INTERVAL = 3000; 
float distanceSmoothed = 999;
const float ALPHA_DIST = 0.3; 

bool waypointPause = false;
unsigned long waypointPauseStart = 0;
const unsigned long PAUSE_DURATION = 3000;

bool missionComplete = false;
uint8_t sys, gyro, accel, mag;

float wrap360(float angle) {
  while (angle < 0) angle += 360;
  while (angle >= 360) angle -= 360;
  return angle;
}

float wrap180(float angle)
{
  while(angle > 180) angle -= 360;
  while(angle < -180) angle += 360;
  return angle;
}

//calculate

#define EARTH_RADIUS 6371000.0

double toRadians(double degree){
  return degree * PI / 180.0;
}

double toDegrees(double radian){
  return radian * 180.0 / PI;
}

double calculateDistance(double lat1,double lon1,double lat2,double lon2){

  double phi1 = toRadians(lat1);
  double phi2 = toRadians(lat2);

  double deltaPhi = toRadians(lat2 - lat1);
  double deltaLambda = toRadians(lon2 - lon1);

  double a = sin(deltaPhi/2) * sin(deltaPhi/2) +
             cos(phi1) * cos(phi2) *
             sin(deltaLambda/2) * sin(deltaLambda/2);

  double c = 2 * atan2(sqrt(a), sqrt(1-a));

  return EARTH_RADIUS * c;
}

double calculateHeading(double lat1,double lon1,double lat2,double lon2){

  double phi1 = toRadians(lat1);
  double phi2 = toRadians(lat2);

  double lambda1 = toRadians(lon1);
  double lambda2 = toRadians(lon2);

  double deltaLambda = lambda2 - lambda1;

  double y = sin(deltaLambda) * cos(phi2);
  double x = cos(phi1) * sin(phi2) -
             sin(phi1) * cos(phi2) * cos(deltaLambda);

  double theta = atan2(y,x);

  double bearing = toDegrees(theta);

  bearing = fmod((bearing + 360.0),360.0);

  return bearing;
}

// ================= WAYPOINT =================
#define MAX_WP 10

double waypointList[MAX_WP][2];
int totalWaypoint = 0;
int currentWaypoint = 0;


float batteryVoltage = 0;
float loadCellValue = 0;

float calibration_factor = -1.0;

// ================= LCD =================

LiquidCrystal_I2C lcd(0x27, 20, 4);

unsigned long lastLCDUpdate = 0;
const int lcdInterval = 500; // 500 ms
bool lcdIntroDone = false;
unsigned long lcdIntroStart = 0;
unsigned long lastCharTime = 0;
int charIndex1 = 0;
int charIndex2 = 0;
int charIndex3 = 0;
String First1 = "    Hi, i'm Argo";
String First2 = "    Your Asisten";
String First3 = "   Delivery Robot";

// ================= RELAY =================
#define DOOR_PIN 24
#define SENL 26
#define SENR 28
unsigned long previousMillis = 0;
const unsigned long interval = 500;
int modeStep = 0;

// ================= KEYPAD =================
const byte ROWS = 4;
const byte COLS = 4;

char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

byte rowPins[ROWS] = {13,12,11,10};
byte colPins[COLS] = {9,8,7,4};
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);
char key;
bool doorActive = true;
unsigned long doorStartTime = 0;
const unsigned long doorDuration = 5000;
unsigned long lastStream = 0;
String espBuffer = "";                         
unsigned long lastSendToESP = 0;            
const unsigned long SEND_INTERVAL = 100; 
unsigned long lastLoadRead = 0;
const unsigned long LOAD_INTERVAL = 200;
void setup()
{
  Serial.begin(115200);
  Serial2.begin(115200);
  NAV_SERIAL.begin(115200);

  bno.begin();

  bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P1);
  bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P1);

  unsigned long waitStart = millis();
  while(millis() - waitStart < 500) {
    // tunggu 500ms tanpa blocking delay
  }

  imu::Quaternion quat = bno.getQuat();
  float yaw = atan2(2.0*(quat.w()*quat.z() + quat.x()*quat.y()),
                    1.0 - 2.0*(quat.y()*quat.y() + quat.z()*quat.z()));
  float h = wrap360(yaw * 180.0 / PI);
  h = wrap360(360 - h - 180);
  filteredHeading = h;
  imuTheta = h * PI / 180.0;
  theta = imuTheta;
  lastEKF = millis();

  lcd.init();
  lcd.backlight();

  pinMode(ENCODER1_A, INPUT_PULLUP);
  pinMode(ENCODER1_B, INPUT_PULLUP);
  pinMode(ENCODER2_A, INPUT_PULLUP);
  pinMode(ENCODER2_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER1_A), isr_encoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_A), isr_encoder2, RISING);

  pinMode(SENR, OUTPUT);
  pinMode(SENL, OUTPUT);
  pinMode(DOOR_PIN, OUTPUT);
  digitalWrite(DOOR_PIN, HIGH);

  // ===== LOAD CELL =====
  scale.begin(HX711_DT, HX711_SCK);
  scale.set_scale(calibration_factor);
  scale.tare(20);

  Serial.println("Load Cell Ready");

  // ===== SENSOR TEGANGAN =====
  pinMode(VOLTAGE_PIN, INPUT);

  // Serial.println("targetlat,targetlon,lat,lon,heading,target_heading,err_head,vL,vR,V,distance,distancetravelled,currentwp,mode");
  Serial.println("t,lat,lon,gps_x,gps_y,x_ekf,y_ekf,target_lat,target_lon,distance,speed,ETA_raw,ETA_kf,Q,R,gps_count,ms_since_gps,filteredHeading");
}

void loop() {
  readFromESP32();    
  readGPS();
  readIMU();
  readEncoder();

  EKF_UpdateHeading();
  if(newOdo) {
      EKF_Predict();
      newOdo = false;
  }
  if(gpsNewData) {
  EKF_Update();
  gpsNewData = false;
  }
  computeNavigation();
  computeETA();  

if(millis() - lastLoadRead >= LOAD_INTERVAL)
{
  lastLoadRead = millis();
    // ===== LOAD CELL =====
  if(scale.is_ready())
  {
    float rawLoad = scale.get_units(3);

    filteredLoad = 0.9 * filteredLoad + 0.1 * rawLoad;

    loadCellValue = filteredLoad;

    if(loadCellValue < 0)
      loadCellValue = 0;

    Serial.print("LOAD: ");
    Serial.print(loadCellValue, 4);

    Serial.print(" | BAT: ");
    Serial.println(batteryVoltage);
  }
}
  // ===== TEGANGAN =====
  batteryVoltage = readBatteryVoltage();

  if (millis() - lastSendToESP >= SEND_INTERVAL) {
    lastSendToESP = millis();
    sendToESP32();
  }
  monitor();
  updateLCD();
  sen();
  key = keypad.getKey();
  doorlock();

  //   if (millis() - lastStream > 180)
  // {
  //   lastStream = millis();
  //   streamData();
  // }
}

void computeETA() {
  if(totalWaypoint == 0) return;
  if(!gpsReference) return;
  int wpIdx = (currentWaypoint < totalWaypoint)
              ? currentWaypoint : totalWaypoint - 1;

  target_x_local = (waypointList[wpIdx][1] - lon0) * 111139 * cos(lat0 * PI/180);
  target_y_local = (waypointList[wpIdx][0] - lat0) * 111139;

  float dist_kf = sqrt(pow(x - target_x_local, 2) +
                       pow(y - target_y_local, 2));

  if (v > 0.03) {
    ETA_raw = distanceToTarget / v;
    ETA_kf  = dist_kf / v;

    if(ETA_kf  > 999) ETA_kf  = 999;
    if(ETA_raw > 999) ETA_raw = 999;

  } else {
    ETA_raw = 0;
    ETA_kf  = 0;
  }
}

void readGPS() {
  while(Serial2.available()) gps.encode(Serial2.read());

  if(!gps.location.isUpdated()) return;
  if(gps.hdop.value() > 180) return;

  // if (!gpsReference && lat0 == 0 && lon0 == 0) {
  //   lat0 = gps.location.lat() - 0.0001;
  //   lon0 = gps.location.lng() - 0.0001;
  // }

    // Set referensi sekali saja
  if (!gpsReference && gps.location.isValid()) {

      lat0 = gps.location.lat();
      lon0 = gps.location.lng();

      gpsReference = true;

      Serial.println("[GPS REF SET]");
  }

  // Selalu update posisi
  currentlat = gps.location.lat();
  currentlon = gps.location.lng();

  if (gpsReference) {
      gps_y = (currentlat - lat0) * 111139;
      gps_x = (currentlon - lon0) * 111139 * cos(lat0 * PI/180);
  }

  if (x == 0 && y == 0 && gpsReference) {
    x = gps_x;   
    y = gps_y;   
  }

  gpsNewData = true;

  gpsUpdateCount++;
  lastGpsUpdate = millis();
}

void readIMU() {

  sensors_event_t event;
  bno.getEvent(&event);

  imu::Quaternion quat = bno.getQuat();
  
  float qw = quat.w();
  float qx = quat.x();
  float qy = quat.y();
  float qz = quat.z();

  // hitung yaw
  float yaw = atan2(2.0 * (qw * qz + qx * qy),
                    1.0 - 2.0 * (qy * qy + qz * qz));

  float heading = yaw * 180.0 / PI;
  // heading = wrap360(heading)- 180;
  heading = wrap360(heading);
  // wrap ke 0-360

  // koreksi sensor terbalik
  heading = 360 - heading;
  heading = wrap360(270 - heading);
  //if (heading >= 360) heading -= 360;

  // low pass filter
  float diff = heading - filteredHeading;
  if (diff > 180)  diff -= 360;
  if (diff < -180) diff += 360;
  filteredHeading = wrap360(filteredHeading + alpha * diff);

  imuTheta = filteredHeading * PI / 180.0;
}

void readEncoder() {
  if (millis() - lastTime >= SAMPLE_TIME) {
    unsigned long now = millis();
    float waktu = (now - lastTime) / 1000.0;
    lastTime = now;

    noInterrupts();
    long currentCount1 = encoder1Count;
    long currentCount2 = encoder2Count;
    interrupts();

    long delta1 = currentCount1 - lastCount1;
    long delta2 = currentCount2 - lastCount2;
    if (abs(delta1) < 2) delta1 = 0;
    if (abs(delta2) < 2) delta2 = 0;

    float jarak1 = (delta1 / (float)PPR) * KELILING;
    float jarak2 = (delta2 / (float)PPR) * KELILING;

    float alpha_v = 0.3;
    vL = alpha_v * (jarak1 / waktu) + (1 - alpha_v) * vL;
    vR = alpha_v * (jarak2 / waktu) + (1 - alpha_v) * vR;

    lastCount1 = currentCount1;
    lastCount2 = currentCount2;
    v = (vR + vL) / 2.0;
    omega_enc = (vR - vL) / wheelBase;

    newOdo = true;  // 🔥 tandai update baru

    float deltaDistance = (jarak1 + jarak2) / 2.0;
    odom_x += deltaDistance * cos(theta);  // ← tambah ini
    odom_y += deltaDistance * sin(theta);
    distanceTravelled += deltaDistance;
  }
}

// ===== ISR =====
void isr_encoder1() {
  if (digitalRead(ENCODER1_B)) encoder1Count--;
  else encoder1Count++;
}

void isr_encoder2() {
  if (digitalRead(ENCODER2_B)) encoder2Count++;
  else encoder2Count--;
}

void EKF_Predict()
{
  unsigned long now = millis();
  float dt_ekf = (now - lastEKF) / 1000.0;
  lastEKF = now;

  if(dt_ekf <= 0) return;
  if(dt_ekf > 0.2f) dt_ekf = 0.2f;

  // Gunakan theta dari IMU langsung (sudah di-set EKF_UpdateHeading)
  x += v * cos(theta) * dt_ekf;
  y += v * sin(theta) * dt_ekf;

  while(theta > PI) theta -= 2*PI;
  while(theta < -PI) theta += 2*PI;


  float cos_t = cos(theta);
  float sin_t = sin(theta);
  P[0][0] += Q_xy * cos_t * cos_t * dt_ekf;
  P[1][1] += Q_xy * sin_t * sin_t * dt_ekf;
  P[2][2] += Q_th * dt_ekf;
}

void EKF_Update()
{
  float yx = gps_x - x;
  float yy = gps_y - y;

  float threshold = (x == gps_x && y == gps_y) ? 100.0 : 15.0;
  if (abs(yx) > threshold || abs(yy) > threshold) return;

  float Kx = P[0][0] / (P[0][0] + R_gps);
  float Ky = P[1][1] / (P[1][1] + R_gps);

  x += Kx * yx;
  y += Ky * yy;

  P[0][0] *= (1 - Kx);
  P[1][1] *= (1 - Ky);
  if (P[0][0] < 0.001) P[0][0] = 0.001; // ← TAMBAH
  if (P[1][1] < 0.001) P[1][1] = 0.001;
}

void EKF_UpdateHeading()
{
  // float R_imu = 0.03;

  float yawError = imuTheta - theta;

  while(yawError >  PI) yawError -= 2*PI;
  while(yawError < -PI) yawError += 2*PI;

  float S = P[2][2] + R_imu;
  float K = P[2][2] / S;

  theta += K * yawError;

  if(theta > PI) theta -= 2*PI;
  if(theta < -PI) theta += 2*PI;

  P[2][2] *= (1 - K);
  if (P[2][2] < 0.001) P[2][2] = 0.001; 
}

void readFromESP32()
{
  while (NAV_SERIAL.available())
  {
    char c = NAV_SERIAL.read();
    espBuffer += c;

    if (c == '>')
    {
      parseESP(espBuffer);
      espBuffer = "";
    }
  }
}

void parseESP(String data)
{
  // ================= COMMAND =================
  if (data.indexOf("DELIVERY") >= 0) {
      manualStop = false;
      lastWaypointTime = millis();
      mode = 1;
  }
  else if (data.indexOf("PICKUP") >= 0) {
      manualStop = false;
      lastWaypointTime = millis();
      mode = 1;
  }
  else if (data.indexOf("STANDBY") >= 0) {
      manualStop = true;
      mode = 0;
  }
  // ================= RESET =================
  if (data.indexOf("RESET") >= 0)
  {
    x = 0;
    y = 0;
    distanceTravelled = 0;
    distanceSmoothed = 999;
  }

  // ================= DOOR =================
  if (data.indexOf("DOOR") >= 0)
  {
    digitalWrite(DOOR_PIN, LOW);
  }

  // ================= CLEAR WAYPOINT =================
  if (data.indexOf("CLRWP") >= 0)
  {
    currentWaypoint = 0;
    totalWaypoint = 0;
    missionComplete = false;
    gpsReference = false; 
    x = 0; y = 0;
    mode = 0;              
    distanceSmoothed = 999;
    waypointPause = false;
    lat0 = 0;   
    lon0 = 0;
    distanceTravelled = 0;
    manualStop = false;
  }

  // ================= TOTAL WAYPOINT =================
  if (data.indexOf("<TOTALWP") >= 0)
  {
    int base  = data.indexOf("<TOTALWP");
    int start = data.indexOf(",", base) + 1;
    int end   = data.indexOf(">", base);
    int wp    = data.substring(start, end).toInt();
    if(wp > MAX_WP) wp = MAX_WP;
    totalWaypoint    = wp;
    gpsReference     = false;
    distanceSmoothed = 999;
    currentWaypoint  = 0;
    missionComplete  = false;
    lat0 = 0; lon0 = 0; x = 0; y = 0;
    Serial.print("[TOTALWP] total="); Serial.println(wp); 
  }

  // ================= WAYPOINT =================
  if (data.indexOf("<WP,") >= 0) {
    int base = data.indexOf("<WP,");
    int p1   = data.indexOf(",", base) + 1;
    int p2   = data.indexOf(",", p1);
    int p3   = data.indexOf(",", p2 + 1);
    int p4   = data.indexOf(">", base);

    double wlat = data.substring(p1, p2).toDouble();
    double wlon = data.substring(p2 + 1, p3).toDouble();
    int idx     = data.substring(p3 + 1, p4).toInt();

    if(idx >= 0 && idx < MAX_WP) {
      waypointList[idx][0] = wlat;
      waypointList[idx][1] = wlon;
      Serial.print("[WP SAVED] idx="); Serial.print(idx);
      Serial.print(" lat="); Serial.print(wlat, 6);
      Serial.print(" lon="); Serial.println(wlon, 6);
    }
  }
}


void computeNavigation()
{
  if(manualStop) { mode = 0; return; } 
  if(totalWaypoint == 0)
  {
    return;
  }
  
  
  if(missionComplete)
  {
    mode = 0;
    return;
  }

  if(currentWaypoint >= totalWaypoint) return;

    if(currentlat == 0 || currentlon == 0)
  {
    distanceToTarget = 0;
    return;
  }

  if(gps.satellites.value() < 4) return;          // ← TAMBAH INI
  if(gps.hdop.value() > 150) return;

  double targetLat = waypointList[currentWaypoint][0];
  double targetLon = waypointList[currentWaypoint][1];

  // ===== TARGET LOKAL =====
  float target_x = (targetLon - lon0) * 111139 * cos(lat0 * PI/180);
  float target_y = (targetLat - lat0) * 111139;

  // ===== ERROR POSISI EKF =====
  float dx = target_x - x;
  float dy = target_y - y;

  // ===== DISTANCE EKF =====
  distanceToTarget = sqrt(dx*dx + dy*dy);

  if(distanceToTarget > 500) return;

  // ===== SMOOTHING =====
  distanceSmoothed = ALPHA_DIST * distanceToTarget +
                    (1 - ALPHA_DIST) * distanceSmoothed;

  // ===== TARGET HEADING DARI EKF =====
  targetHeading = atan2(dy, dx) * 180.0 / PI;
  targetHeading = wrap360(targetHeading);

// ===== HEADING ERROR =====
  headingError = wrap180(targetHeading - filteredHeading);

  bool timeGuard = (millis() - lastWaypointTime) > 5000;
  if(distanceSmoothed < 1.0 && timeGuard)   // ← tambah timeGuard
  {
    lastWaypointTime = millis(); 
    currentWaypoint++;
    distanceTravelled = 0;
    distanceSmoothed = distanceToTarget;
    wpJustAdvanced = true; 

    if(currentWaypoint >= totalWaypoint)
    {
      missionComplete = true;
      mode = 0;
      currentWaypoint = totalWaypoint - 1;
      return;
    }

    mode = 1;
    return;
  }

  mode = 1;
}           


void monitor() {

  bno.getCalibration(&sys, &gyro, &accel, &mag);

}

void sendToESP32()
{
  NAV_SERIAL.print("<LAT:");
  NAV_SERIAL.print(currentlat,6);

  NAV_SERIAL.print(",LON:");
  NAV_SERIAL.print(currentlon,6);

  NAV_SERIAL.print(",MODE:");
  NAV_SERIAL.print(mode);

  if (totalWaypoint > 0) {
    int wpIdx = (currentWaypoint < totalWaypoint) ? currentWaypoint : totalWaypoint - 1;

    NAV_SERIAL.print(",ERR:");
    NAV_SERIAL.print(-headingError);

    NAV_SERIAL.print(",DIS:");
    NAV_SERIAL.print(distanceToTarget);

    NAV_SERIAL.print(",TRV:");
    NAV_SERIAL.print(distanceTravelled);

    NAV_SERIAL.print(",SPEED:");
    NAV_SERIAL.print(v);

    NAV_SERIAL.print(",WP:");
    NAV_SERIAL.print(currentWaypoint);

    NAV_SERIAL.print(",NEWWP:");
    NAV_SERIAL.print(wpJustAdvanced ? 1 : 0); 
    wpJustAdvanced = false; 

    NAV_SERIAL.print(",WLAT:");
    NAV_SERIAL.print(waypointList[wpIdx][0],6);

    NAV_SERIAL.print(",WLON:");
    NAV_SERIAL.print(waypointList[wpIdx][1],6);

    NAV_SERIAL.print(",BAT:");
    NAV_SERIAL.print(batteryVoltage);

    NAV_SERIAL.print(",LOAD:");
    NAV_SERIAL.print(loadCellValue);

    NAV_SERIAL.print(",ETARAW:");
    NAV_SERIAL.print(ETA_raw);
    NAV_SERIAL.print(",ETAKF:");
    NAV_SERIAL.print(ETA_kf);

    NAV_SERIAL.print(",GPSX:");
    NAV_SERIAL.print(gps_x);
    NAV_SERIAL.print(",GPSY:");
    NAV_SERIAL.print(gps_y);

    NAV_SERIAL.print(",XEKF:");
    NAV_SERIAL.print(x);
    NAV_SERIAL.print(",YEKF:");
    NAV_SERIAL.print(y);

    NAV_SERIAL.print(",Q:");
    NAV_SERIAL.print(Q_xy);
    NAV_SERIAL.print(",R:");
    NAV_SERIAL.print(R_gps);

    NAV_SERIAL.print(",HEAD:");
    NAV_SERIAL.print(filteredHeading);

    NAV_SERIAL.print(",THEAD:");
    NAV_SERIAL.print(targetHeading);

    NAV_SERIAL.print(",VL:");
    NAV_SERIAL.print(vL);

    NAV_SERIAL.print(",VR:");
    NAV_SERIAL.print(vR);

    NAV_SERIAL.print(",LAT0:");
    NAV_SERIAL.print(lat0, 6);
    NAV_SERIAL.print(",LON0:");
    NAV_SERIAL.print(lon0, 6);

    NAV_SERIAL.print(",ODMX:");
    NAV_SERIAL.print(odom_x);
    NAV_SERIAL.print(",ODMY:");
    NAV_SERIAL.print(odom_y);

    NAV_SERIAL.print(",ELAT:");
    NAV_SERIAL.print(lat0 + (y / 111139.0), 6);
    NAV_SERIAL.print(",ELON:");
    NAV_SERIAL.print(lon0 + (x / (111139.0 * cos(lat0 * PI/180))), 6);

    NAV_SERIAL.print(",HEKF:");
    NAV_SERIAL.print(theta * 180.0 / PI);

  }                          

  NAV_SERIAL.println(">");   
}                                         

void updateLCD()
{
  // ===== INTRO SCREEN =====
  if(!lcdIntroDone)
  {
    if(lcdIntroStart == 0)
    {
      lcdIntroStart = millis();
      lcd.clear();

      lcd.setCursor(0,0);
      lcd.print("       ()_()");
    }

    // animasi tulisan
    if(millis() - lastCharTime > 100)
    {
      lastCharTime = millis();

      if(charIndex1 < First1.length())
      {
        lcd.setCursor(charIndex1,1);
        lcd.print(First1[charIndex1]);
        charIndex1++;
      }
      else if(charIndex2 < First2.length())
      {
        lcd.setCursor(charIndex2,2);
        lcd.print(First2[charIndex2]);
        charIndex2++;
      }
      else if(charIndex3 < First3.length())
      {
        lcd.setCursor(charIndex3,3);
        lcd.print(First3[charIndex3]);
        charIndex3++;
      }
    }

    // setelah 3 detik pindah ke main display
    if(millis() - lcdIntroStart > 7000)
    {
      lcdIntroDone = true;
      lcd.clear();
    }

    return;
  }

  // ===== MAIN DISPLAY =====
  if (millis() - lastLCDUpdate >= lcdInterval)
  {
    lastLCDUpdate = millis();

    lcd.clear();

    lcd.setCursor(0,2);
    lcd.print("H:");
    lcd.print(filteredHeading,1);
    lcd.print(" EH:");
    lcd.print(headingError,1);

    lcd.setCursor(0,3);
    lcd.print("D:");
    lcd.print(distanceToTarget,1);

    lcd.setCursor(10,3);
    lcd.print("M:");
    lcd.print(mag);

    lcd.setCursor(0,0);
    lcd.print("Lat:");
    lcd.print(currentlat,6);

    lcd.setCursor(0,1);
    lcd.print("Lon:");
    lcd.print(currentlon,6);
  }
}

void sen() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    switch (modeStep) {
      case 0:
        digitalWrite(SENR, LOW);
        digitalWrite(SENL, LOW);
        break;

      case 1:
        digitalWrite(SENR, HIGH);
        digitalWrite(SENL, HIGH);
        break;
    }

    modeStep++;
    if (modeStep > 1) modeStep = 0;
  }
}

void doorlock() {
  if (key == '#') {
    digitalWrite(DOOR_PIN, LOW);
    doorStartTime = millis();
    doorActive = true;
  }

  if (doorActive && millis() - doorStartTime >= doorDuration) {
    digitalWrite(DOOR_PIN, HIGH);
    doorActive = false;
  }
}

float readBatteryVoltage()
{
  int adc = analogRead(VOLTAGE_PIN);

  float voltage = (adc / 1023.0) * 5.0;

  float battery = voltage * 6.7;

  return battery;
}