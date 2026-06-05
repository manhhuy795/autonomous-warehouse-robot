#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "esp_camera.h"



//  ================= WIFI + CAMERA UPLOAD CONFIG =================
//  Sua 3 dong nay theo Wi-Fi va IP may dang chay FastAPI.
#define WIFI_SSID       "MATCHALATTE"
#define WIFI_PASSWORD   "12345678"
#define AI_SERVER_BASE_URL "http://192.168.1.130:8000"
#define AI_SERVER_URL      AI_SERVER_BASE_URL "/api/frame"
#define ROBOT_ID           "robot-01"

// Robot command API da co san tren FastAPI:
// GET  /api/robot/command?robotId=robot-01
// POST /api/robot/ack
// POST /api/robot/telemetry
#define ROBOT_COMMAND_URL   AI_SERVER_BASE_URL "/api/robot/command?robotId=" ROBOT_ID
#define ROBOT_ACK_URL       AI_SERVER_BASE_URL "/api/robot/ack"
#define ROBOT_TELEMETRY_URL AI_SERVER_BASE_URL "/api/robot/telemetry"

// Gui anh ve backend /api/frame dang dung multipart field: frame, robotId, capturedAtMs.
// De khong lam giat PID, viec gui anh chay o FreeRTOS task rieng.
const unsigned long FRAME_UPLOAD_INTERVAL_MS = 333;  // On dinh hon de camera upload khong nghen command poll
const uint16_t CAMERA_HTTP_TIMEOUT_MS = 4000;

// Poll lenh server va gui telemetry. Dung interval vua phai de khong lam giat PID/camera.
const unsigned long COMMAND_POLL_INTERVAL_MS = 1000;
const unsigned long TELEMETRY_SEND_INTERVAL_MS = 1000;
const uint16_t ROBOT_HTTP_TIMEOUT_MS = 5000;

// Bat/tat camera upload. Neu chua gan camera thi de false de robot van chay line follower.
#define CAMERA_UPLOAD_ENABLED true

//  ================= CAMERA PIN CONFIG =================
//  CAN SUA LAI THEO MODULE CAMERA ESP32-S3 CUA BAN.
//  Neu pin sai, Serial se bao CAMERA INIT FAILED va robot van tiep tuc chay line follower.
//
//  Chu y: code robot hien tai da dung GPIO 1,2,14,17,18,35,36,37,38,39,40,41,42.
//  Khong duoc de camera trung pin voi motor, IR, encoder, UART sieu am.

#define CAM_PIN_PWDN    -1
#define CAM_PIN_RESET   -1

#define CAM_PIN_XCLK    15
#define CAM_PIN_SIOD    4
#define CAM_PIN_SIOC    5

#define CAM_PIN_D7      16
#define CAM_PIN_D6      17
#define CAM_PIN_D5      18
#define CAM_PIN_D4      12
#define CAM_PIN_D3      10
#define CAM_PIN_D2      8
#define CAM_PIN_D1      9
#define CAM_PIN_D0      11

#define CAM_PIN_VSYNC   6
#define CAM_PIN_HREF    7
#define CAM_PIN_PCLK    13

bool wifiReady = false;
bool cameraReady = false;
unsigned long lastWifiReconnectMillis = 0;

// ================= SERVER COMMAND STATE =================

bool serverStopActive = false;
bool serverPaused = false;
String activeTaskId = "";
String activeCommandId = "";
String lastProcessedCommandId = "";
String robotRuntimeState = "FOLLOW_LINE";

// Luu trang thai motion khi server/AI tam dung xe.
// Quan trong cho GAP_BRIDGE v4: neu pause giua luc qua vach dut,
// khi resume phai cong bu thoi gian pause de khong bi GAP_BRIDGE SAFE STOP ngay.
bool serverHoldActive = false;
bool serverHoldWasGapBridge = false;
bool serverHoldGapSafeStopped = false;
unsigned long serverHoldStartMillis = 0;

unsigned long lastCommandPollMillis = 0;
unsigned long lastTelemetrySendMillis = 0;
unsigned long lastFrameUploadedAtMs = 0;

SemaphoreHandle_t httpMutex = NULL;

//  ================= MOTOR PIN =================

//  Motor phai OUT1 OUT2
#define MOTOR_R_EN   39
#define MOTOR_R_IN1  36
#define MOTOR_R_IN2  37

//  Motor trai OUT3 OUT4
#define MOTOR_L_EN   38
#define MOTOR_L_IN3  35
#define MOTOR_L_IN4  40

//  PWM motor dung LEDC channel rieng de KHONG dung channel 0 cua camera XCLK.
//  Camera esp_camera dung LEDC_CHANNEL_0/LEDC_TIMER_0.
//  Neu dung analogWrite(), Arduino co the lay trung LEDC channel lam camera capture fail.
#define MOTOR_L_PWM_CHANNEL  4
#define MOTOR_R_PWM_CHANNEL  5
#define MOTOR_PWM_FREQ       1000
#define MOTOR_PWM_RES_BITS   8


//  ================= IR SENSOR PIN =================

#define IR_LEFT_PIN    1
#define IR_CENTER_PIN  2
#define IR_RIGHT_PIN   14

// Neu cam bien IR tra LOW khi thay line, de true.
// Neu cam bien IR tra HIGH khi thay line, doi thanh false.
const bool LINE_SENSOR_ACTIVE_LOW = false;


//  ================= ENCODER PIN =================

#define ENC_LEFT_A   41
#define ENC_RIGHT_A  42


//  ================= ULTRASONIC UART FROM UNO =================
//
//  UNO doc HC-SR04 va gui sang ESP32-S3 theo format:
//  DIST:23.4
//  DIST:ERROR
//
//  Noi day:
//  UNO D3 TX -> chia ap 5V ve 3.3V -> ESP32-S3 GPIO47
//  UNO GND   -> ESP32-S3 GND

#define ULTRASONIC_RX_PIN 47
#define ULTRASONIC_TX_PIN 48

HardwareSerial UnoSerial(1);

const float OBSTACLE_STOP_DISTANCE_CM = 15.0;
const float OBSTACLE_CLEAR_DISTANCE_CM = 20.0;
const unsigned long ULTRASONIC_DATA_TIMEOUT_MS = 1000;
const float ULTRASONIC_MIN_VALID_CM = 0.5;
const float ULTRASONIC_MAX_VALID_CM = 400.0;

const bool ULTRASONIC_FAILSAFE_STOP = false;

String ultrasonicLine = "";
float latestDistanceCm = -1.0;
unsigned long lastUltrasonicMillis = 0;
bool obstacleStopActive = false;
unsigned long lastObstacleDebugMillis = 0;
const unsigned long ULTRASONIC_DECISION_PRINT_INTERVAL_MS = 300;
unsigned long lastUltrasonicDecisionPrintMillis = 0;


//  ================= SPEED CONFIG =================

int BASE_SPEED = 50;

int MAX_SPEED = 80;
int MIN_SPEED = 0;

int SEARCH_FAST = 50;
int SEARCH_SLOW = 15;


//  ================= GAP BRIDGE CONFIG =================

const float WHEEL_DIAMETER_CM = 6.5;
const float TICKS_PER_WHEEL_REV = 20.0;

// GAP_BRIDGE v4:
// - Khong dung encoder lam dieu kien dung ngay luc dau, vi encoder/noise co the lam robot SAFE STOP qua som.
// - Robot se chay toi thieu GAP_MIN_DRIVE_TIME_MS khi mat line, sau do moi cho phep dung neu qua timeout.
// - Neu can dung encoder de gioi han khoang cach, doi GAP_USE_ENCODER_LIMIT thanh true sau khi encoder da test on.
const float GAP_MAX_CM = 8.0;
const float GAP_SPEED_SCALE = 1.0;
const int GAP_MIN_PWM = 55;
const int GAP_CURVE_MIN_PWM = 55;
const int GAP_CURVE_MIN_DIFF = 14;
const int GAP_TURN_MEMORY_LOOPS = 25;
const long GAP_MIN_TICKS_BEFORE_STOP = 8;
const unsigned long GAP_MIN_DRIVE_TIME_MS = 260;
const unsigned long GAP_MAX_TIME_MS = 950;
const bool GAP_USE_ENCODER_LIMIT = false;

// Chong nhieu IR: khong vao/thoat gap chi vi 1 lan doc loi.
const int GAP_LOST_CONFIRM_LOOPS = 2;
const int GAP_FOUND_CONFIRM_LOOPS = 2;

volatile long encoderLeftTicks = 0;
volatile long encoderRightTicks = 0;

enum RobotState {
  PID_FOLLOW,
  GAP_BRIDGE
};

RobotState robotState = PID_FOLLOW;

long gapStartLeftTicks = 0;
long gapStartRightTicks = 0;
unsigned long gapStartMillis = 0;
bool gapSafeStopped = false;
unsigned long lastGapDebugMillis = 0;
int lostLineConfirmCount = 0;
int foundLineConfirmCount = 0;

int lastLeftCommand = 50;
int lastRightCommand = 50;
int lastErrorBeforeGap = 0;
int recentTurnLeftCommand = 50;
int recentTurnRightCommand = 50;
int recentTurnError = 0;
int straightAfterTurnCount = 0;


//  ================= PID CONFIG =================

float Kp = 8.0;
float Ki = 0;
float Kd = 5.0;

float P = 0;
float I = 0;
float D = 0;

int errorNow = 0;
int errorPrev = 0;

float PID_value = 0;

bool REVERSE_LEFT_MOTOR  = true;
bool REVERSE_RIGHT_MOTOR = true;

int lastDirection = 0;


//  ================= FUNCTION PROTOTYPES =================

void connectWiFi();
bool initCamera();
void cameraUploadTask(void *parameter);
void handleFrameUpload();
bool uploadFrameToServer(camera_fb_t *fb, unsigned long capturedAtMs);
bool cameraPinsConfigured();

void serverCommunicationTask(void *parameter);
void handleServerCommunication();
void pollRobotCommand();
void executeRobotCommand(JsonObject command);
bool sendRobotAck(const char *commandId, const char *taskId, const char *status, bool ack, const String &message, const char *actualLocationId = nullptr);
void sendRobotTelemetry();
bool postJsonToServer(const char *url, JsonDocument &doc, String *responseOut = nullptr);
String getCommandString(JsonObject command);
void updateRuntimeStateFromMotion();
void holdMotionForServerPause();
void resumeMotionAfterServerPause();

int readLineSensorNormalized(uint8_t pin);
void lineFollowPID();
void setupMotorPwm();
void motorPwmWrite(uint8_t channel, int duty);
void moveForward(int leftSpeed, int rightSpeed);
void stopMotor();
void forceStopMotor();
void searchLine();

void IRAM_ATTR onLeftEncoderA();
void IRAM_ATTR onRightEncoderA();

long readLeftEncoderTicks();
long readRightEncoderTicks();
long getGapMaxTicks();

void startGapBridge();
void runGapBridge();
int scaleGapPwm(int pwm);

void updateUltrasonicFromUno();
void handleUltrasonicLine(String data);
bool isObstacleDetected();
void stopForObstacle();
void printUltrasonicDecision(const char *source);


//  ================= SETUP =================

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println();
  Serial.println("ESP32-S3 ROBOT STARTED");
  Serial.println("UNO UART RX: GPIO47, baud 9600");
  Serial.println("SERVER COMMAND MODE: ENABLED");
  Serial.println("BUILD: ROBOT_AI_COMMAND_GAP_TIMED_BRIDGE_V4_FIXED");

  httpMutex = xSemaphoreCreateMutex();

  // WiFi dung chung cho camera upload, command polling va telemetry.
  connectWiFi();

#if CAMERA_UPLOAD_ENABLED
  cameraReady = initCamera();

  if (cameraReady) {
    xTaskCreatePinnedToCore(
      cameraUploadTask,
      "cameraUploadTask",
      8192,
      NULL,
      1,
      NULL,
      0
    );
    Serial.println("CAMERA UPLOAD TASK STARTED");
  }
  else {
    Serial.println("CAMERA UPLOAD DISABLED - camera init failed or pins not configured");
  }
#endif

  xTaskCreatePinnedToCore(
    serverCommunicationTask,
    "serverCommunicationTask",
    8192,
    NULL,
    1,
    NULL,
    0
  );
  Serial.println("SERVER COMMUNICATION TASK STARTED");

  UnoSerial.begin(9600, SERIAL_8N1, ULTRASONIC_RX_PIN, ULTRASONIC_TX_PIN);
  UnoSerial.setTimeout(80);

  Serial.println("ESP32-S3 line follower started");
  Serial.println("Ultrasonic UART: UNO TX -> ESP32 GPIO47 RX");

  pinMode(IR_LEFT_PIN, INPUT);
  pinMode(IR_CENTER_PIN, INPUT);
  pinMode(IR_RIGHT_PIN, INPUT);

  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), onLeftEncoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), onRightEncoderA, RISING);

  pinMode(MOTOR_L_IN3, OUTPUT);
  pinMode(MOTOR_L_IN4, OUTPUT);

  pinMode(MOTOR_R_IN1, OUTPUT);
  pinMode(MOTOR_R_IN2, OUTPUT);

  setupMotorPwm();

  stopMotor();
  delay(1000);
}


//  ================= LOOP =================

void loop() {
  // Vong loop chi giu cac tac vu dieu khien thoi gian thuc:
  // UART ultrasonic + safety stop + PID line follow.
  // HTTP command/telemetry da chuyen sang task rieng de khong lam giat robot.
  updateUltrasonicFromUno();

  if (serverStopActive || serverPaused) {
    forceStopMotor();
    delay(20);
    return;
  }

  if (isObstacleDetected()) {
    stopForObstacle();
    delay(20);
    return;
  }

  lineFollowPID();
  delay(8);
}



//  ================= WIFI + CAMERA UPLOAD =================

bool cameraPinsConfigured() {
  return CAM_PIN_XCLK >= 0 && CAM_PIN_SIOD >= 0 && CAM_PIN_SIOC >= 0 &&
         CAM_PIN_D7 >= 0 && CAM_PIN_D6 >= 0 && CAM_PIN_D5 >= 0 && CAM_PIN_D4 >= 0 &&
         CAM_PIN_D3 >= 0 && CAM_PIN_D2 >= 0 && CAM_PIN_D1 >= 0 && CAM_PIN_D0 >= 0 &&
         CAM_PIN_VSYNC >= 0 && CAM_PIN_HREF >= 0 && CAM_PIN_PCLK >= 0;
}


void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    wifiReady = true;
    return;
  }

  Serial.print("WIFI CONNECTING: ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long startMillis = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - startMillis < 8000) {
    delay(250);
    Serial.print(".");
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    wifiReady = true;
    Serial.print("WIFI CONNECTED | IP: ");
    Serial.println(WiFi.localIP());
  }
  else {
    wifiReady = false;
    Serial.println("WIFI CONNECT FAILED - robot still runs, image upload paused");
  }
}


bool initCamera() {
  if (!cameraPinsConfigured()) {
    Serial.println("CAMERA PINS NOT CONFIGURED - edit CAM_PIN_* before using camera upload");
    return false;
  }

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = CAM_PIN_D0;
  config.pin_d1 = CAM_PIN_D1;
  config.pin_d2 = CAM_PIN_D2;
  config.pin_d3 = CAM_PIN_D3;
  config.pin_d4 = CAM_PIN_D4;
  config.pin_d5 = CAM_PIN_D5;
  config.pin_d6 = CAM_PIN_D6;
  config.pin_d7 = CAM_PIN_D7;
  config.pin_xclk = CAM_PIN_XCLK;
  config.pin_pclk = CAM_PIN_PCLK;
  config.pin_vsync = CAM_PIN_VSYNC;
  config.pin_href = CAM_PIN_HREF;
  config.pin_sscb_sda = CAM_PIN_SIOD;
  config.pin_sscb_scl = CAM_PIN_SIOC;
  config.pin_pwdn = CAM_PIN_PWDN;
  config.pin_reset = CAM_PIN_RESET;
  // Giong file test da chay on: OV3660 on dinh hon voi 10MHz
  config.xclk_freq_hz = 10000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    Serial.println("PSRAM FOUND - camera use QVGA");
    config.frame_size = FRAMESIZE_QVGA;     // 320x240
    config.jpeg_quality = 15;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  }
  else {
    Serial.println("PSRAM NOT FOUND - camera fallback QQVGA");
    config.frame_size = FRAMESIZE_QQVGA;    // 160x120 neu khong co PSRAM
    config.jpeg_quality = 18;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  }

  esp_err_t err = esp_camera_init(&config);

  if (err != ESP_OK) {
    Serial.print("CAMERA INIT FAILED, error=0x");
    Serial.println(err, HEX);
    return false;
  }

  Serial.println("CAMERA INIT OK");

  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor != NULL) {
    Serial.print("CAMERA SENSOR PID: 0x");
    Serial.println(sensor->id.PID, HEX);
    sensor->set_vflip(sensor, 0);
    sensor->set_hmirror(sensor, 0);
    sensor->set_brightness(sensor, 0);
    sensor->set_contrast(sensor, 0);
    sensor->set_saturation(sensor, 0);
  }

  // Bo frame dau de sensor on dinh, giong file test.
  for (int i = 0; i < 2; i++) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      Serial.print("CAMERA WARMUP FRAME ");
      Serial.print(i + 1);
      Serial.print(" bytes=");
      Serial.println(fb->len);
      esp_camera_fb_return(fb);
    }
    else {
      Serial.println("CAMERA WARMUP CAPTURE FAILED");
    }
    delay(200);
  }

  return true;
}


void cameraUploadTask(void *parameter) {
  delay(1500);
  Serial.println("CAMERA TASK LOOP ENTERED");

  while (true) {
    handleFrameUpload();
    vTaskDelay(pdMS_TO_TICKS(FRAME_UPLOAD_INTERVAL_MS));
  }
}


void handleFrameUpload() {
  if (!cameraReady) {
    return;
  }

  if (WiFi.status() != WL_CONNECTED) {
    wifiReady = false;

    unsigned long now = millis();
    if (now - lastWifiReconnectMillis >= 5000) {
      lastWifiReconnectMillis = now;
      Serial.println("WIFI LOST - reconnecting...");
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }

    return;
  }

  wifiReady = true;

  unsigned long capturedAtMs = millis();

  Serial.println("FRAME CAPTURE START");
  camera_fb_t *fb = esp_camera_fb_get();

  if (!fb) {
    Serial.println("CAMERA CAPTURE FAILED");
    return;
  }

  Serial.print("CAMERA CAPTURE OK | bytes=");
  Serial.println(fb->len);

  if (fb->format != PIXFORMAT_JPEG) {
    Serial.println("CAMERA FRAME IS NOT JPEG - upload skipped");
    esp_camera_fb_return(fb);
    return;
  }

  bool ok = uploadFrameToServer(fb, capturedAtMs);
  esp_camera_fb_return(fb);

  if (!ok) {
    Serial.println("FRAME UPLOAD FAILED");
  }
}


bool uploadFrameToServer(camera_fb_t *fb, unsigned long capturedAtMs) {
  if (WiFi.status() != WL_CONNECTED || fb == NULL || fb->buf == NULL || fb->len == 0) {
    return false;
  }

  String boundary = "----ESP32S3RobotBoundary";

  String head = "--" + boundary + "\r\n";
  head += "Content-Disposition: form-data; name=\"robotId\"\r\n\r\n";
  head += ROBOT_ID;
  head += "\r\n";

  head += "--" + boundary + "\r\n";
  head += "Content-Disposition: form-data; name=\"capturedAtMs\"\r\n\r\n";
  head += String(capturedAtMs);
  head += "\r\n";

  head += "--" + boundary + "\r\n";
  head += "Content-Disposition: form-data; name=\"frame\"; filename=\"frame.jpg\"\r\n";
  head += "Content-Type: image/jpeg\r\n\r\n";

  String tail = "\r\n--" + boundary + "--\r\n";

  size_t totalLen = head.length() + fb->len + tail.length();
  uint8_t *body = (uint8_t *)malloc(totalLen);

  if (body == NULL) {
    Serial.println("FRAME UPLOAD malloc FAILED");
    return false;
  }

  size_t offset = 0;
  memcpy(body + offset, head.c_str(), head.length());
  offset += head.length();
  memcpy(body + offset, fb->buf, fb->len);
  offset += fb->len;
  memcpy(body + offset, tail.c_str(), tail.length());

  HTTPClient http;

  Serial.print("FRAME UPLOAD START -> ");
  Serial.println(AI_SERVER_URL);

  bool locked = false;
  if (httpMutex != NULL) {
    locked = (xSemaphoreTake(httpMutex, pdMS_TO_TICKS(5000)) == pdTRUE);
    if (!locked) {
      Serial.println("FRAME UPLOAD HTTP MUTEX TIMEOUT");
      free(body);
      return false;
    }
  }

  http.begin(AI_SERVER_URL);
  http.setTimeout(CAMERA_HTTP_TIMEOUT_MS);
  http.addHeader("Content-Type", "multipart/form-data; boundary=" + boundary);
  http.addHeader("Connection", "close");

  int httpCode = http.POST(body, totalLen);
  String response = http.getString();

  free(body);
  http.end();

  if (locked) {
    xSemaphoreGive(httpMutex);
  }

  if (httpCode >= 200 && httpCode < 300) {
    lastFrameUploadedAtMs = millis();
  }

  Serial.print("FRAME UPLOAD HTTP: ");
  Serial.print(httpCode);
  Serial.print(" | bytes=");
  Serial.print(fb->len);

  if (response.length() > 0) {
    Serial.print(" | response=");
    Serial.print(response.substring(0, 120));
  }

  Serial.println();

  return httpCode >= 200 && httpCode < 300;
}




//  ================= SERVER COMMAND + TELEMETRY =================

void serverCommunicationTask(void *parameter) {
  delay(2500);
  Serial.println("SERVER TASK LOOP ENTERED");

  while (true) {
    handleServerCommunication();
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}


void handleServerCommunication() {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  unsigned long now = millis();

  if (now - lastCommandPollMillis >= COMMAND_POLL_INTERVAL_MS) {
    lastCommandPollMillis = now;
    pollRobotCommand();
  }

  if (now - lastTelemetrySendMillis >= TELEMETRY_SEND_INTERVAL_MS) {
    lastTelemetrySendMillis = now;
    sendRobotTelemetry();
  }
}


String getCommandString(JsonObject command) {
  const char *commandText = command["command"] | "";
  if (strlen(commandText) == 0) {
    commandText = command["type"] | "";
  }
  String result = String(commandText);
  result.toUpperCase();
  return result;
}


void pollRobotCommand() {
  bool locked = false;
  if (httpMutex != NULL) {
    locked = (xSemaphoreTake(httpMutex, pdMS_TO_TICKS(5000)) == pdTRUE);
    if (!locked) {
      Serial.println("[CMD] HTTP mutex busy, skip poll");
      return;
    }
  }

  HTTPClient http;
  http.begin(ROBOT_COMMAND_URL);
  http.setTimeout(ROBOT_HTTP_TIMEOUT_MS);

  int httpCode = http.GET();
  String response = http.getString();

  http.end();

  if (locked) {
    xSemaphoreGive(httpMutex);
  }

  if (httpCode != 200) {
    Serial.print("[CMD] Poll failed HTTP=");
    Serial.println(httpCode);
    return;
  }

  DynamicJsonDocument doc(8192);
  DeserializationError err = deserializeJson(doc, response);

  if (err) {
    Serial.print("[CMD] JSON parse failed: ");
    Serial.println(err.c_str());
    return;
  }

  bool hasCommand = doc["hasCommand"] | false;

  if (!hasCommand || doc["command"].isNull()) {
    return;
  }

  JsonObject command = doc["command"].as<JsonObject>();
  executeRobotCommand(command);
}


void executeRobotCommand(JsonObject command) {
  const char *commandId = command["commandId"] | "";
  const char *taskId = command["taskId"] | "";
  String commandText = getCommandString(command);

  activeCommandId = commandId;

  Serial.println();
  Serial.print("[CMD] RECEIVED: ");
  Serial.print(commandText);
  Serial.print(" | commandId=");
  Serial.print(commandId);
  Serial.print(" | taskId=");
  Serial.println(taskId);

  // Server reliable-delivery co the gui lai cung commandId neu ACK truoc do bi mat.
  // Neu gap duplicate, khong execute lai hanh dong; chi ACK lai de server clear in-flight.
  if (strlen(commandId) > 0 && lastProcessedCommandId == String(commandId)) {
    Serial.println("[CMD] DUPLICATE commandId - skip action, retry ACK");
    sendRobotAck(commandId, taskId, "RECEIVED", true, "Robot da xu ly command nay truoc do, gui ACK lai");
    return;
  }

  if (strlen(commandId) > 0) {
    lastProcessedCommandId = String(commandId);
  }

  if (commandText == "WMS_TASK") {
    activeTaskId = taskId;
    serverStopActive = false;
    serverPaused = false;
    robotRuntimeState = "FOLLOW_LINE_WMS_TASK";
    resumeMotionAfterServerPause();

    const char *itemId = command["itemId"] | "";
    const char *fromLocationId = command["fromLocationId"] | "";
    const char *toLocationId = command["toLocationId"] | "";
    const char *action = command["action"] | "";

    Serial.print("[CMD] WMS_TASK item=");
    Serial.print(itemId);
    Serial.print(" from=");
    Serial.print(fromLocationId);
    Serial.print(" to=");
    Serial.print(toLocationId);
    Serial.print(" action=");
    Serial.println(action);

    sendRobotAck(commandId, taskId, "RECEIVED", true, "Robot da nhan WMS_TASK va bat dau chay line theo task WMS");
    return;
  }

  if (commandText == "START_TASK") {
    activeTaskId = taskId;
    serverStopActive = false;
    serverPaused = false;
    robotRuntimeState = "FOLLOW_LINE_TO_TASK";
    resumeMotionAfterServerPause();

    // Server hien gui path/pickup/drop trong command.
    // Ban robot hien tai chua co dinh vi node, nen START_TASK duoc hieu la:
    // "bat dau chay line theo PID va bao da nhan task".
    const char *pickupNode = command["pickupNode"] | "";
    const char *dropNode = command["dropNode"] | "";

    Serial.print("[CMD] START_TASK pickup=");
    Serial.print(pickupNode);
    Serial.print(" drop=");
    Serial.println(dropNode);

    sendRobotAck(commandId, taskId, "RECEIVED", true, "Robot da nhan START_TASK va bat dau chay line");
    return;
  }

  // Lenh raw tu AI Vision safetyState. Backend moi thuong map sang PAUSE_TASK/RESUME_TASK,
  // nhung firmware van xu ly raw state de robot khong bi dung neu server gui truc tiep.
  if (commandText == "EMERGENCY_STOP_PERSON") {
    serverStopActive = true;
    serverPaused = false;
    robotRuntimeState = "EMERGENCY_STOP_BY_AI_VISION";
    serverHoldActive = false;
    serverHoldWasGapBridge = false;
    forceStopMotor();

    sendRobotAck(commandId, taskId, "RECEIVED", true, "Robot dung khan cap do AI Vision phat hien nguoi/vat can nguy hiem");
    return;
  }

  if (commandText == "STOP_AND_SCAN" || commandText == "STOP_AND_RECHECK" || commandText == "WAIT_CLEAR" ||
      commandText == "AI_STOP" || commandText == "VISION_STOP") {
    serverPaused = true;
    serverStopActive = false;
    robotRuntimeState = "STOPPED_BY_AI_VISION";
    holdMotionForServerPause();

    sendRobotAck(commandId, taskId, "RECEIVED", true, "Robot tam dung do AI Vision yeu cau dung/quet lai");
    return;
  }

  if (commandText == "RESUME_LINE" || commandText == "FOLLOW_LINE") {
    serverPaused = false;
    serverStopActive = false;
    robotRuntimeState = activeTaskId.length() > 0 ? "FOLLOW_LINE_TO_TASK" : "FOLLOW_LINE";
    resumeMotionAfterServerPause();

    if (command.containsKey("baseSpeed")) {
      BASE_SPEED = constrain((int)(command["baseSpeed"] | BASE_SPEED), 0, 255);
    }
    if (command.containsKey("maxSpeed")) {
      MAX_SPEED = constrain((int)(command["maxSpeed"] | MAX_SPEED), 0, 255);
    }

    sendRobotAck(commandId, taskId, "RECEIVED", true, "Robot tiep tuc do line theo AI Vision");
    return;
  }

  if (commandText == "SLOW_DOWN_AND_MONITOR" || commandText == "SLOW_DOWN") {
    BASE_SPEED = constrain((int)(command["baseSpeed"] | 35), 0, 255);
    MAX_SPEED = constrain((int)(command["maxSpeed"] | 60), 0, 255);
    serverPaused = false;
    serverStopActive = false;
    robotRuntimeState = "SLOW_DOWN_AI_VISION";
    resumeMotionAfterServerPause();

    Serial.print("[CMD] AI SLOW base=");
    Serial.print(BASE_SPEED);
    Serial.print(" max=");
    Serial.println(MAX_SPEED);

    sendRobotAck(commandId, taskId, "RECEIVED", true, "Robot giam toc theo AI Vision");
    return;
  }

  if (commandText == "WAIT" || commandText == "YIELD_OR_WAIT") {
    serverPaused = true;
    serverStopActive = false;
    robotRuntimeState = "WAITING_TRAFFIC";
    holdMotionForServerPause();

    sendRobotAck(commandId, taskId, "RECEIVED", true, "Robot cho theo lenh dieu phoi traffic/server");
    return;
  }

  if (commandText == "STOP" || commandText == "EMERGENCY_STOP" || commandText == "FORCE_STOP") {
    serverStopActive = true;
    serverPaused = false;
    robotRuntimeState = "STOPPED_BY_SERVER";

    // STOP/EMERGENCY_STOP la dung that su, khong phai pause tam thoi.
    // Xoa hold de lan RESUME sau khoi bi keo lai vao GAP_BRIDGE cu.
    serverHoldActive = false;
    serverHoldWasGapBridge = false;
    forceStopMotor();

    sendRobotAck(commandId, taskId, "RECEIVED", true, "Robot da dung theo lenh server");
    return;
  }

  if (commandText == "PAUSE" || commandText == "PAUSE_TASK") {
    serverPaused = true;
    serverStopActive = false;
    robotRuntimeState = "PAUSED_BY_SERVER";

    // Khong reset robotState/GAP_BRIDGE o day.
    // Neu AI tam dung giua luc dang qua line dut, can giu lai state de RESUME chay tiep.
    holdMotionForServerPause();

    sendRobotAck(commandId, taskId, "RECEIVED", true, "Robot da tam dung theo lenh server");
    return;
  }

  if (commandText == "RESUME" || commandText == "RESUME_TASK") {
    serverPaused = false;
    serverStopActive = false;
    robotRuntimeState = activeTaskId.length() > 0 ? "FOLLOW_LINE_TO_TASK" : "FOLLOW_LINE";

    // Neu truoc do AI pause khi dang GAP_BRIDGE, tiep tuc bridge thay vi reset ve PID_FOLLOW.
    resumeMotionAfterServerPause();

    // Neu AI server gui toc do mac dinh khi clear duong, khoi phuc lai sau SET_SPEED cham.
    if (command.containsKey("baseSpeed")) {
      BASE_SPEED = constrain((int)(command["baseSpeed"] | BASE_SPEED), 0, 255);
    }
    if (command.containsKey("maxSpeed")) {
      MAX_SPEED = constrain((int)(command["maxSpeed"] | MAX_SPEED), 0, 255);
    }

    sendRobotAck(commandId, taskId, "RECEIVED", true, "Robot da tiep tuc chay");
    return;
  }

  if (commandText == "SET_SPEED") {
    int newBaseSpeed = command["baseSpeed"] | BASE_SPEED;
    int newMaxSpeed = command["maxSpeed"] | MAX_SPEED;

    BASE_SPEED = constrain(newBaseSpeed, 0, 255);
    MAX_SPEED = constrain(newMaxSpeed, 0, 255);

    Serial.print("[CMD] SET_SPEED base=");
    Serial.print(BASE_SPEED);
    Serial.print(" max=");
    Serial.println(MAX_SPEED);

    sendRobotAck(commandId, taskId, "RECEIVED", true, "Robot da cap nhat toc do");
    return;
  }

  if (commandText == "COMPLETE_TASK") {
    robotRuntimeState = "WAITING_TASK";
    serverPaused = false;
    serverStopActive = false;
    activeTaskId = "";
    serverHoldActive = false;
    serverHoldWasGapBridge = false;

    sendRobotAck(commandId, taskId, "COMPLETED", true, "Robot da hoan thanh task theo lenh server");
    return;
  }

  Serial.print("[CMD] Unknown command: ");
  Serial.println(commandText);
  sendRobotAck(commandId, taskId, "FAILED", false, "Robot khong ho tro command: " + commandText);
}

bool sendRobotAck(const char *commandId, const char *taskId, const char *status, bool ack, const String &message, const char *actualLocationId) {
  DynamicJsonDocument doc(1024);

  doc["robotId"] = ROBOT_ID;
  doc["commandId"] = commandId;
  doc["taskId"] = taskId;
  doc["status"] = status;
  doc["ack"] = ack;
  doc["message"] = message;

  if (actualLocationId != nullptr && strlen(actualLocationId) > 0) {
    doc["actualLocationId"] = actualLocationId;
  }

  String response;
  bool ok = postJsonToServer(ROBOT_ACK_URL, doc, &response);

  Serial.print("[ACK] ");
  Serial.print(status);
  Serial.print(" -> ");
  Serial.println(ok ? "OK" : "FAILED");

  if (response.length() > 0) {
    Serial.print("[ACK] response=");
    Serial.println(response.substring(0, 160));
  }

  return ok;
}


void sendRobotTelemetry() {
  DynamicJsonDocument doc(2048);

  int irLeft = readLineSensorNormalized(IR_LEFT_PIN);
  int irCenter = readLineSensorNormalized(IR_CENTER_PIN);
  int irRight = readLineSensorNormalized(IR_RIGHT_PIN);

  bool lineDetected = !(irLeft == 0 && irCenter == 0 && irRight == 0);

  doc["robotId"] = ROBOT_ID;
  doc["timestamp"] = millis();
  doc["source"] = "esp32-s3";
  doc["state"] = robotRuntimeState;
  doc["mode"] = "AUTONOMOUS";
  doc["wifiRssi"] = WiFi.RSSI();
  doc["battery"] = nullptr;

  doc["irLeft"] = irLeft;
  doc["irCenter"] = irCenter;
  doc["irRight"] = irRight;
  doc["lineDetected"] = lineDetected;

  doc["ultrasonicFrontCm"] = latestDistanceCm > 0.0 ? latestDistanceCm : -1;
  doc["obstacleStopActive"] = obstacleStopActive;
  doc["serverStopActive"] = serverStopActive;
  doc["serverPaused"] = serverPaused;

  doc["encoderLeft"] = readLeftEncoderTicks();
  doc["encoderRight"] = readRightEncoderTicks();

  doc["leftMotorCommand"] = lastLeftCommand;
  doc["rightMotorCommand"] = lastRightCommand;

  doc["cameraConnected"] = cameraReady;
  doc["lastFrameAt"] = lastFrameUploadedAtMs;
  doc["lastFrameReason"] = "periodic";

  doc["activeTaskId"] = activeTaskId;
  doc["activeCommandId"] = activeCommandId;

  JsonObject connection = doc.createNestedObject("connection");
  connection["esp32"] = true;
  connection["wifi"] = WiFi.status() == WL_CONNECTED;
  connection["server"] = true;
  connection["uno"] = lastUltrasonicMillis > 0 && millis() - lastUltrasonicMillis < ULTRASONIC_DATA_TIMEOUT_MS;
  connection["uart"] = true;

  String response;
  bool ok = postJsonToServer(ROBOT_TELEMETRY_URL, doc, &response);

  if (!ok) {
    Serial.println("[TEL] send failed");
  }
}


bool postJsonToServer(const char *url, JsonDocument &doc, String *responseOut) {
  if (WiFi.status() != WL_CONNECTED) {
    return false;
  }

  String body;
  serializeJson(doc, body);

  bool locked = false;
  if (httpMutex != NULL) {
    locked = (xSemaphoreTake(httpMutex, pdMS_TO_TICKS(3000)) == pdTRUE);
    if (!locked) {
      Serial.println("[HTTP JSON] mutex busy");
      return false;
    }
  }

  HTTPClient http;
  http.begin(url);
  http.setTimeout(ROBOT_HTTP_TIMEOUT_MS);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Connection", "close");

  int httpCode = http.POST(body);

  String response = http.getString();

  http.end();

  if (locked) {
    xSemaphoreGive(httpMutex);
  }

  if (responseOut != nullptr) {
    *responseOut = response;
  }

  if (httpCode < 200 || httpCode >= 300) {
    Serial.print("[HTTP JSON] POST failed url=");
    Serial.print(url);
    Serial.print(" code=");
    Serial.println(httpCode);
    return false;
  }

  return true;
}



//  ================= SERVER PAUSE MOTION HOLD =================

void holdMotionForServerPause() {
  if (!serverHoldActive) {
    serverHoldActive = true;
    serverHoldStartMillis = millis();
    serverHoldWasGapBridge = (robotState == GAP_BRIDGE);
    serverHoldGapSafeStopped = gapSafeStopped;

    if (serverHoldWasGapBridge) {
      Serial.println("[GAP_BRIDGE] HOLD BY SERVER PAUSE");
    }
  }

  forceStopMotor();
}


void resumeMotionAfterServerPause() {
  if (!serverHoldActive) {
    return;
  }

  unsigned long now = millis();
  unsigned long pausedDuration = now - serverHoldStartMillis;

  if (serverHoldWasGapBridge && robotState == GAP_BRIDGE) {
    // Trong thoi gian PAUSE, motor dung nen khong duoc tinh vao GAP_MAX_TIME_MS.
    // Neu khong cong bu, xe vua RESUME se bi SAFE STOP ngay va mat chuc nang qua line dut.
    gapStartMillis += pausedDuration;
    gapSafeStopped = serverHoldGapSafeStopped;
    lastGapDebugMillis = 0;

    Serial.print("[GAP_BRIDGE] RESUME AFTER SERVER PAUSE | pausedMs=");
    Serial.println(pausedDuration);
  }

  serverHoldActive = false;
  serverHoldWasGapBridge = false;
  serverHoldGapSafeStopped = false;
}


//  ================= OBSTACLE STOP =================

void stopForObstacle() {
  I = 0;
  D = 0;
  PID_value = 0;
  errorPrev = 0;

  robotState = PID_FOLLOW;
  gapSafeStopped = false;

  forceStopMotor();
}


//  ================= ULTRASONIC OBSTACLE DETECTION =================

void updateUltrasonicFromUno() {
  while (UnoSerial.available()) {
    String line = UnoSerial.readStringUntil('\n');
    line.trim();

    if (line.length() > 0) {
      Serial.print("[UNO UART] ");
      Serial.println(line);

      handleUltrasonicLine(line);
    }
  }
}


void handleUltrasonicLine(String data) {
  if (!data.startsWith("DIST:")) {
    Serial.println("[ULTRASONIC] Invalid format");
    return;
  }

  String value = data.substring(5);
  value.trim();

  if (value == "ERROR") {
    Serial.println("[ULTRASONIC] UNO reported DIST:ERROR, keep previous valid distance");
    printUltrasonicDecision("RX_ERROR");
    return;
  }

  float distance = value.toFloat();

  if (distance >= ULTRASONIC_MIN_VALID_CM && distance <= ULTRASONIC_MAX_VALID_CM) {
    latestDistanceCm = distance;
    lastUltrasonicMillis = millis();

    if (latestDistanceCm <= OBSTACLE_STOP_DISTANCE_CM) {
      obstacleStopActive = true;
      forceStopMotor();
    }

    printUltrasonicDecision("RX");
  }
  else {
    Serial.print("[ULTRASONIC] Ignored invalid distance: ");
    Serial.print(distance, 1);
    Serial.println(" cm");
  }
}


void printUltrasonicDecision(const char *source) {
  unsigned long now = millis();

  if (now - lastUltrasonicDecisionPrintMillis < ULTRASONIC_DECISION_PRINT_INTERVAL_MS) {
    return;
  }

  lastUltrasonicDecisionPrintMillis = now;

  Serial.print("[ULTRASONIC ");
  Serial.print(source);
  Serial.print("] distance=");

  if (latestDistanceCm > 0.0) {
    Serial.print(latestDistanceCm, 1);
    Serial.print(" cm");
  } else {
    Serial.print("NO_DATA");
  }

  Serial.print(" | stop_threshold=");
  Serial.print(OBSTACLE_STOP_DISTANCE_CM, 1);
  Serial.print(" cm");

  Serial.print(" | clear_threshold=");
  Serial.print(OBSTACLE_CLEAR_DISTANCE_CM, 1);
  Serial.print(" cm");

  Serial.print(" | decision=");

  if (latestDistanceCm > 0.0 && latestDistanceCm <= OBSTACLE_STOP_DISTANCE_CM) {
    Serial.println("STOP");
  }
  else if (obstacleStopActive && latestDistanceCm > 0.0 && latestDistanceCm < OBSTACLE_CLEAR_DISTANCE_CM) {
    Serial.println("KEEP_STOP");
  }
  else if (latestDistanceCm >= OBSTACLE_CLEAR_DISTANCE_CM) {
    Serial.println("RUN");
  }
  else {
    Serial.println("WAIT_DATA");
  }
}


bool isObstacleDetected() {
  unsigned long now = millis();

  bool dataTimeout = false;

  if (lastUltrasonicMillis == 0 || now - lastUltrasonicMillis > ULTRASONIC_DATA_TIMEOUT_MS) {
    dataTimeout = true;
  }

  if (ULTRASONIC_FAILSAFE_STOP && dataTimeout) {
    if (!obstacleStopActive) {
      Serial.println("OBSTACLE STOP - ULTRASONIC TIMEOUT");
    }

    obstacleStopActive = true;
    return true;
  }

  if (latestDistanceCm > 0.0 && latestDistanceCm <= OBSTACLE_STOP_DISTANCE_CM) {
    if (!obstacleStopActive || now - lastObstacleDebugMillis >= 300) {
      Serial.print("OBSTACLE STOP - distance: ");
      Serial.print(latestDistanceCm, 1);
      Serial.println(" cm | action=STOP_MOTOR");

      printUltrasonicDecision("DECISION");

      lastObstacleDebugMillis = now;
    }

    obstacleStopActive = true;
    return true;
  }

  if (obstacleStopActive) {
    if (latestDistanceCm >= OBSTACLE_CLEAR_DISTANCE_CM) {
      obstacleStopActive = false;

      Serial.print("OBSTACLE CLEAR - distance: ");
      Serial.print(latestDistanceCm, 1);
      Serial.println(" cm | action=RESUME_LINE_FOLLOW");

      printUltrasonicDecision("DECISION");

      return false;
    }

    forceStopMotor();
    return true;
  }

  if (now - lastObstacleDebugMillis >= 500) {
    lastObstacleDebugMillis = now;
    printUltrasonicDecision("LOOP");
  }

  return false;
}


//  ================= LINE FOLLOW PID - LINE RONG =================

int readLineSensorNormalized(uint8_t pin) {
  int raw = digitalRead(pin);
  return LINE_SENSOR_ACTIVE_LOW ? !raw : raw;
}


void lineFollowPID() {
  int L = readLineSensorNormalized(IR_LEFT_PIN);
  int C = readLineSensorNormalized(IR_CENTER_PIN);
  int R = readLineSensorNormalized(IR_RIGHT_PIN);

  bool lineLost = (L == 0 && C == 0 && R == 0);

  // Dang chay qua doan line dut.
  // Chi thoat GAP_BRIDGE khi thay lai line lien tiep, tranh bi nhieu IR lam thoat som.
  if (robotState == GAP_BRIDGE) {
    if (lineLost) {
      foundLineConfirmCount = 0;
      runGapBridge();
      return;
    }

    foundLineConfirmCount++;

    if (foundLineConfirmCount < GAP_FOUND_CONFIRM_LOOPS) {
      runGapBridge();
      return;
    }

    robotState = PID_FOLLOW;
    gapSafeStopped = false;
    lostLineConfirmCount = 0;
    foundLineConfirmCount = 0;
    I = 0;
    D = 0;

    Serial.print("GAP_BRIDGE EXIT - FOUND LINE CONFIRMED | LCR=");
    Serial.print(L);
    Serial.print(C);
    Serial.println(R);
  }

  // Mat line: xac nhan 2 vong loop roi moi vao GAP_BRIDGE.
  // Trong luc xac nhan, van day motor tien de xe khong khung/dung lai tai dau gap.
  if (lineLost) {
    lostLineConfirmCount++;

    int holdLeft = max(lastLeftCommand, GAP_MIN_PWM);
    int holdRight = max(lastRightCommand, GAP_MIN_PWM);
    moveForward(holdLeft, holdRight);

    if (lostLineConfirmCount >= GAP_LOST_CONFIRM_LOOPS) {
      startGapBridge();
    }

    return;
  }
  else {
    lostLineConfirmCount = 0;
  }

  if (L == 1 && C == 1 && R == 1) {
    errorNow = 0;
    lastDirection = 0;
  }
  else if (L == 0 && C == 1 && R == 0) {
    errorNow = 0;
    lastDirection = 0;
  }
  else if (L == 1 && C == 1 && R == 0) {
    errorNow = -1;
    lastDirection = -1;
  }
  else if (L == 1 && C == 0 && R == 0) {
    errorNow = -2;
    lastDirection = -1;
  }
  else if (L == 0 && C == 1 && R == 1) {
    errorNow = 1;
    lastDirection = 1;
  }
  else if (L == 0 && C == 0 && R == 1) {
    errorNow = 2;
    lastDirection = 1;
  }
  else if (L == 1 && C == 0 && R == 1) {
    // Vach rong/giao diem/line bi ho giua: uu tien di thang.
    errorNow = 0;
    lastDirection = 0;
  }

  P = errorNow;

  I = I + errorNow;
  I = constrain(I, -20, 20);

  D = errorNow - errorPrev;
  errorPrev = errorNow;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);

  int leftSpeed  = BASE_SPEED + PID_value;
  int rightSpeed = BASE_SPEED - PID_value;

  leftSpeed  = constrain(leftSpeed, MIN_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);

  lastLeftCommand = leftSpeed;
  lastRightCommand = rightSpeed;
  lastErrorBeforeGap = errorNow;

  if (errorNow != 0) {
    recentTurnLeftCommand = leftSpeed;
    recentTurnRightCommand = rightSpeed;
    recentTurnError = errorNow;
    straightAfterTurnCount = 0;
  }
  else if (straightAfterTurnCount < GAP_TURN_MEMORY_LOOPS) {
    straightAfterTurnCount++;
  }
  else {
    recentTurnError = 0;
  }

  moveForward(leftSpeed, rightSpeed);
}


//  ================= ENCODER + GAP BRIDGE =================

void IRAM_ATTR onLeftEncoderA() {
  encoderLeftTicks++;
}


void IRAM_ATTR onRightEncoderA() {
  encoderRightTicks++;
}


long readLeftEncoderTicks() {
  noInterrupts();
  long ticks = encoderLeftTicks;
  interrupts();

  return ticks;
}


long readRightEncoderTicks() {
  noInterrupts();
  long ticks = encoderRightTicks;
  interrupts();

  return ticks;
}


long getGapMaxTicks() {
  float ticksPerCm = TICKS_PER_WHEEL_REV / (PI * WHEEL_DIAMETER_CM);
  long gapMaxTicks = (long)((GAP_MAX_CM * ticksPerCm) + 0.5);

  if (gapMaxTicks < GAP_MIN_TICKS_BEFORE_STOP) {
    gapMaxTicks = GAP_MIN_TICKS_BEFORE_STOP;
  }

  return gapMaxTicks;
}


void startGapBridge() {
  if (robotState == GAP_BRIDGE) {
    return;
  }

  if (lastErrorBeforeGap == 0 && recentTurnError != 0) {
    lastLeftCommand = recentTurnLeftCommand;
    lastRightCommand = recentTurnRightCommand;
    lastErrorBeforeGap = recentTurnError;
  }

  robotState = GAP_BRIDGE;

  gapStartLeftTicks = readLeftEncoderTicks();
  gapStartRightTicks = readRightEncoderTicks();
  gapStartMillis = millis();

  gapSafeStopped = false;
  lastGapDebugMillis = 0;
  foundLineConfirmCount = 0;

  I = 0;
  D = 0;

  Serial.print("GAP_BRIDGE START | lastError=");
  Serial.print(lastErrorBeforeGap);
  Serial.print(" | lastL=");
  Serial.print(lastLeftCommand);
  Serial.print(" | lastR=");
  Serial.println(lastRightCommand);
}


int scaleGapPwm(int pwm) {
  int scaledPwm = (int)((abs(pwm) * GAP_SPEED_SCALE) + 0.5);

  if (scaledPwm < GAP_MIN_PWM) {
    scaledPwm = GAP_MIN_PWM;
  }

  return constrain(scaledPwm, GAP_MIN_PWM, 255);
}


void runGapBridge() {
  if (gapSafeStopped) {
    stopMotor();
    return;
  }

  long leftDelta = labs(readLeftEncoderTicks() - gapStartLeftTicks);
  long rightDelta = labs(readRightEncoderTicks() - gapStartRightTicks);

  long averageDelta = (leftDelta + rightDelta) / 2;
  long gapMaxTicks = getGapMaxTicks();

  unsigned long gapElapsed = millis() - gapStartMillis;

  // Quan trong: khong cho SAFE STOP ngay dau gap.
  // Neu encoder doc sai/noise hoac PWM chua kip day xe, xe se bi dung tai dau doan dut.
  bool passedMinDriveTime = (gapElapsed >= GAP_MIN_DRIVE_TIME_MS);
  bool encoderLimitReached = (GAP_USE_ENCODER_LIMIT && averageDelta >= gapMaxTicks);
  bool timeLimitReached = (gapElapsed >= GAP_MAX_TIME_MS);

  if (passedMinDriveTime && (encoderLimitReached || timeLimitReached)) {
    stopMotor();
    gapSafeStopped = true;

    Serial.print("GAP_BRIDGE SAFE STOP | elapsed=");
    Serial.print(gapElapsed);
    Serial.print(" | avgDelta=");
    Serial.print(averageDelta);
    Serial.print(" | gapMaxTicks=");
    Serial.println(gapMaxTicks);
    return;
  }

  int leftGapSpeed = GAP_MIN_PWM;
  int rightGapSpeed = GAP_MIN_PWM;

  if (lastErrorBeforeGap == 0) {
    // Mat line sau khi dang di thang: giu thang bang encoder neu co, nhung khong phu thuoc encoder de dung.
    int baseGapSpeed = (abs(lastLeftCommand) + abs(lastRightCommand)) / 2;

    if (baseGapSpeed <= 0) {
      baseGapSpeed = BASE_SPEED;
    }

    baseGapSpeed = scaleGapPwm(baseGapSpeed);

    long deltaError = leftDelta - rightDelta;

    int correctionLimit = max(1, baseGapSpeed / 4);
    int correction = constrain((int)(deltaError * 2), -correctionLimit, correctionLimit);

    leftGapSpeed = constrain(baseGapSpeed - correction, GAP_MIN_PWM, 255);
    rightGapSpeed = constrain(baseGapSpeed + correction, GAP_MIN_PWM, 255);
  }
  else {
    // Mat line ngay sau cua: giu ti le lenh motor truoc do de tiep tuc cua nhe.
    leftGapSpeed = (int)((abs(lastLeftCommand) * GAP_SPEED_SCALE) + 0.5);
    rightGapSpeed = (int)((abs(lastRightCommand) * GAP_SPEED_SCALE) + 0.5);

    int outerSpeed = max(leftGapSpeed, rightGapSpeed);

    if (outerSpeed < GAP_MIN_PWM) {
      float boost = (float)GAP_MIN_PWM / max(1, outerSpeed);

      leftGapSpeed = (int)((leftGapSpeed * boost) + 0.5);
      rightGapSpeed = (int)((rightGapSpeed * boost) + 0.5);
    }

    leftGapSpeed = constrain(leftGapSpeed, GAP_CURVE_MIN_PWM, 255);
    rightGapSpeed = constrain(rightGapSpeed, GAP_CURVE_MIN_PWM, 255);

    int speedDiff = abs(leftGapSpeed - rightGapSpeed);

    if (speedDiff < GAP_CURVE_MIN_DIFF) {
      int extraDiff = (GAP_CURVE_MIN_DIFF - speedDiff + 1) / 2;

      if (lastErrorBeforeGap < 0) {
        leftGapSpeed = constrain(leftGapSpeed - extraDiff, GAP_CURVE_MIN_PWM, 255);
        rightGapSpeed = constrain(rightGapSpeed + extraDiff, GAP_CURVE_MIN_PWM, 255);
      }
      else {
        leftGapSpeed = constrain(leftGapSpeed + extraDiff, GAP_CURVE_MIN_PWM, 255);
        rightGapSpeed = constrain(rightGapSpeed - extraDiff, GAP_CURVE_MIN_PWM, 255);
      }
    }
  }

  moveForward(leftGapSpeed, rightGapSpeed);

  unsigned long now = millis();

  if (lastGapDebugMillis == 0 || now - lastGapDebugMillis >= 200) {
    Serial.print("GAP RUN | Lpwm=");
    Serial.print(leftGapSpeed);
    Serial.print(" | Rpwm=");
    Serial.print(rightGapSpeed);
    Serial.print(" | elapsed=");
    Serial.print(gapElapsed);
    Serial.print(" | avgDelta=");
    Serial.print(averageDelta);
    Serial.print(" | gapMaxTicks=");
    Serial.print(gapMaxTicks);
    Serial.print(" | encoderLimit=");
    Serial.println(GAP_USE_ENCODER_LIMIT ? "ON" : "OFF");

    lastGapDebugMillis = now;
  }
}


//  ================= MOTOR CONTROL =================

void setupMotorPwm() {
  ledcSetup(MOTOR_L_PWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RES_BITS);
  ledcSetup(MOTOR_R_PWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RES_BITS);

  ledcAttachPin(MOTOR_L_EN, MOTOR_L_PWM_CHANNEL);
  ledcAttachPin(MOTOR_R_EN, MOTOR_R_PWM_CHANNEL);

  motorPwmWrite(MOTOR_L_PWM_CHANNEL, 0);
  motorPwmWrite(MOTOR_R_PWM_CHANNEL, 0);

  Serial.println("MOTOR PWM INIT OK - channels 4/5");
}


void motorPwmWrite(uint8_t channel, int duty) {
  duty = constrain(duty, 0, 255);
  ledcWrite(channel, duty);
}


void moveForward(int leftSpeed, int rightSpeed) {
  if (obstacleStopActive) {
    forceStopMotor();
    return;
  }

  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  if (REVERSE_LEFT_MOTOR == false) {
    digitalWrite(MOTOR_L_IN3, HIGH);
    digitalWrite(MOTOR_L_IN4, LOW);
  }
  else {
    digitalWrite(MOTOR_L_IN3, LOW);
    digitalWrite(MOTOR_L_IN4, HIGH);
  }

  if (REVERSE_RIGHT_MOTOR == false) {
    digitalWrite(MOTOR_R_IN1, HIGH);
    digitalWrite(MOTOR_R_IN2, LOW);
  }
  else {
    digitalWrite(MOTOR_R_IN1, LOW);
    digitalWrite(MOTOR_R_IN2, HIGH);
  }

  motorPwmWrite(MOTOR_L_PWM_CHANNEL, leftSpeed);
  motorPwmWrite(MOTOR_R_PWM_CHANNEL, rightSpeed);
}


void forceStopMotor() {
  motorPwmWrite(MOTOR_L_PWM_CHANNEL, 0);
  motorPwmWrite(MOTOR_R_PWM_CHANNEL, 0);

  digitalWrite(MOTOR_L_IN3, LOW);
  digitalWrite(MOTOR_L_IN4, LOW);

  digitalWrite(MOTOR_R_IN1, LOW);
  digitalWrite(MOTOR_R_IN2, LOW);
}


void stopMotor() {
  forceStopMotor();
}


//  ================= SEARCH LINE =================

void searchLine() {
  I = 0;

  if (lastDirection == -1) {
    moveForward(SEARCH_SLOW, SEARCH_FAST);
  }
  else if (lastDirection == 1) {
    moveForward(SEARCH_FAST, SEARCH_SLOW);
  }
  else {
    moveForward(35, 35);
  }
}