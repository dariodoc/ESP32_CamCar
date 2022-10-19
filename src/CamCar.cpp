#include "CamCar.h"
#include <Arduino.h>
#include <esp_camera.h>
#include <WiFi.h>
#include "AsyncTCP.h"
#include "ESPAsyncWebServer.h"
#include <sstream>
#include <ArduinoOTA.h>
#include "ESP32Servo.h"
#include "SparkFun_TB6612.h"
#include "Melodies.h"
#include <soc/soc.h>
#include <soc/rtc_cntl_reg.h>
#include <SPIFFS.h>

//#define DEBUG // Uncomment to enable Serial Monitor

// Camera related constants
// ESP32-WROVER-E
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 21
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 19
#define Y4_GPIO_NUM 18
#define Y3_GPIO_NUM 5
#define Y2_GPIO_NUM 4
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

const int psramLimit = 10000;

const static int builtinPin = 2;

const static int lightPin = 13;
static bool enableLight = false;

const static int buzzerPin = 12;
const static int buzzerChannel = 3;
static bool melodyOn = false;

Servo panServo;
Servo tiltServo;

const static int panPin = 14;
const static int tiltPin = 15;
const static int panCenter = 80;

#ifdef DEBUG
const static int tiltCenter = 60;
#else
const static int tiltCenter = 90;
#endif

static bool enableObstacleAvoidance = false;

PCF8574 pcf8574(0x20, SIOD_GPIO_NUM, SIOC_GPIO_NUM); // PCF8574 library included in CamCar.h
Motor leftMotor(P3, P4, 32, 1, P2);                  // BIN1,BIN2,PWMB,OFFSET,STBY.
Motor rightMotor(P1, P0, 33, 1, P2);                 // AIN1,AIN2,PWMA,OFFSET,STBY.

static int motorSpeed = 255;
#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4
#define STOP 0
#define FORWARDLEFT 5
#define FORWARDRIGHT 6
#define BACKLEFT 7
#define BACKRIGHT 8

#define STACK_SIZE 1024 * 4
TaskHandle_t keepWiFiAliveTask;
TaskHandle_t arduinoOTATask;
TaskHandle_t sendCameraPictureTask;
TaskHandle_t obstacleAvoidanceModeTask;
TaskHandle_t playMelodyTask;
TaskHandle_t cleanupWSClientsTask;

AsyncWebServer server(80);
AsyncWebSocket wsCamera("/Camera");
AsyncWebSocket wsCarInput("/CarInput");
static int cameraClientId = 0;

// static unsigned long previousMillis = 0;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void handleRSSI(AsyncWebServerRequest *request)
{
  String rssi = "0";
  rssi = WiFi.RSSI();
  request->send(200, "text/plane", rssi);
}

void handleRoot(AsyncWebServerRequest *request)
{
  request->send(SPIFFS, "/index.html", String());
}

void handleStyle(AsyncWebServerRequest *request)
{
  request->send(SPIFFS, "/style.css", "text/css");
}

void handleNotFound(AsyncWebServerRequest *request)
{
  request->send(404, "text/plain", "File Not Found");
}

void ledIndicator(int blinkTimes, int delayTimeMS)
{
  unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  for (int i = 0; i < blinkTimes; i++)
  {
    digitalWrite(builtinPin, HIGH); // LED ON
    previousMillis = currentMillis;
    while (currentMillis - previousMillis <= delayTimeMS)
    {
      toneToPlay(buzzerPin, buzzerChannel, NOTE_G5, 1);
      currentMillis = millis();
    }
    digitalWrite(builtinPin, LOW); // LED OFF
    previousMillis = currentMillis;
    while (currentMillis - previousMillis <= delayTimeMS)
    {
      currentMillis = millis();
    }
  }
}

void ledIndicator(int state)
{
  if (state == HIGH)
  {
    digitalWrite(builtinPin, HIGH); // LED ON
  }
  if (state == LOW)
  {
    digitalWrite(builtinPin, LOW); // LED OFF
  }
}

void leftBackLed(int state)
{
  if (state == HIGH)
  {
    pcf8574.digitalWrite(P6, HIGH); // LED OFF
  }
  if (state == LOW)
  {
    pcf8574.digitalWrite(P6, LOW); // LED ON
  }
}

void rightBackLed(int state)
{
  if (state == HIGH)
  {
    pcf8574.digitalWrite(P7, HIGH); // LED OFF
  }
  if (state == LOW)
  {
    pcf8574.digitalWrite(P7, LOW); // LED ON
  }
}

void arduinoOTA(void *parameters)
{
#ifdef DEBUG
  Serial.printf("arduinoOTA() running on core: %d\n", xPortGetCoreID());
#endif
  ArduinoOTA.begin(); // enable to receive update/upload firmware via WiFi OTA
  for (;;)
  {
#ifdef DEBUG
// Serial.println( uxTaskGetStackHighWaterMark(nullptr));
#endif
    ArduinoOTA.handle(); // enable to receive update/upload firmware via Wifi OTA
  }
}

void keepWiFiAlive(void *parameters)
{
  // ===========================
  // Enter your WiFi credentials
  // ===========================
  // Wifi Credentials //
  const static char *sta_ssid = "My Pills";       // set Wifi network you want to connect to
  const static char *sta_password = "Ivonne2011"; // set password for Wifi network
  const static int WIFI_TIMEOUT_MS = 10000;
  unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();

  ////////////// Set ESP32 Wifi hostname based on chip mac address//////////////////
  char chip_id[15];
  snprintf(chip_id, 15, "%04X", (uint16_t)(ESP.getEfuseMac() >> 32));
  String hostname = "esp32cam-" + String(chip_id);
  /////////////////////////////////////////////////////////////////////////////////
#ifdef DEBUG
  Serial.printf("keepWiFiAlive() running on core: %d\n", xPortGetCoreID());
#endif
  for (;;)
  {
#ifdef DEBUG
// Serial.println( uxTaskGetStackHighWaterMark(nullptr));
#endif
    if (WiFi.status() == WL_CONNECTED)
    {
#ifdef DEBUG
      Serial.println("WiFi still connected");
#endif
      previousMillis = currentMillis;
      while (currentMillis - previousMillis <= WIFI_TIMEOUT_MS)
      {
        currentMillis = millis();
      }
      continue;
    }
    else
    {

// first, set ESP32 as STA mode to connect with a Wifi network
#ifdef DEBUG
      Serial.println("");
      Serial.print("Connecting to: ");
      Serial.println(sta_ssid);
      Serial.print("Password: ");
      Serial.println(sta_password);
#endif
      WiFi.mode(WIFI_STA);
      WiFi.begin(sta_ssid, sta_password);

      // try to connect with Wifi network about 10 seconds
      previousMillis = currentMillis;
      while (WiFi.status() != WL_CONNECTED && currentMillis - previousMillis <= WIFI_TIMEOUT_MS)
      {
#ifdef DEBUG
        unsigned long previousMillis1 = millis();
        while (previousMillis1 - currentMillis <= 1000)
        {
          previousMillis1 = millis();
        }
        Serial.print(".");
#endif
        ledIndicator(1, 500);
        currentMillis = millis();
      }
      // Check if connected with Wifi network
      if (WiFi.status() != WL_CONNECTED)
      {
#ifdef DEBUG
        Serial.println("[WIFI] FAILED");
#endif
        continue;
      }
      else if (WiFi.status() == WL_CONNECTED)
      {
#ifdef DEBUG
        IPAddress myIP = WiFi.localIP();
        Serial.println("");
        Serial.println("*WiFi-STA-Mode*");
        Serial.print("IP: ");
        Serial.println(myIP);
#endif
        ledIndicator(10, 50);
        ledIndicator(HIGH);
      }
    }
  }
}

void playMelody(void *parameters)
{
#ifdef DEBUG
// Serial.println( uxTaskGetStackHighWaterMark(nullptr));
#endif
  gameOfThrones(buzzerPin, buzzerChannel);
  melodyOn = false;
  ledcDetachPin(buzzerPin);
  vTaskDelete(playMelodyTask);
}

void cleanupWSClients(void *parameters)
{
  unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  for (;;)
  {
    previousMillis = currentMillis;
    while (currentMillis - previousMillis <= 1000)
    {
      currentMillis = millis();
    }
    wsCamera.cleanupClients();
    wsCarInput.cleanupClients();
#ifdef DEBUG
    Serial.println("ws cleaned");
#endif
  }
}

void sendCameraPicture(void *parameters)
{
  unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
#ifdef DEBUG
  Serial.printf("sendCameraPicture() running on core: %d\n", xPortGetCoreID());
#endif
  for (;;)
  {
#ifdef DEBUG
// Serial.println( uxTaskGetStackHighWaterMark(nullptr));
#endif
    if (cameraClientId == 0)
    {
      continue;
    }
    ///////////////////////////////////////////////////////////////////////////////////////////
    unsigned long startTime1 = millis();
    // capture a frame
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb)
    {
#ifdef DEBUG
      Serial.println("Frame buffer could not be acquired");
#endif
      continue;
    }
    ///////////////////////////////////////////////////////////////////////////////////////////
    unsigned long startTime2 = millis();
    wsCamera.binary(cameraClientId, fb->buf, fb->len);
    AsyncWebSocketClient *clientPointer = wsCamera.client(cameraClientId);
    esp_camera_fb_return(fb);

    // Wait for message to be delivered
    while (true)
    {
      if (!clientPointer || !(clientPointer->queueIsFull()))
      {
        break;
      }

      previousMillis = currentMillis;
      while (currentMillis - previousMillis <= 1)
      {
        currentMillis = millis();
      }
    }

    unsigned long startTime3 = millis();
#ifdef DEBUG
    Serial.printf("Time taken Total: %d|%d|%d\n", startTime3 - startTime1, startTime2 - startTime1, startTime3 - startTime2);
#endif
  }
}

void onCameraWebSocketEvent(AsyncWebSocket *server,
                            AsyncWebSocketClient *client,
                            AwsEventType type,
                            void *arg,
                            uint8_t *data,
                            size_t len)
{
  switch (type)
  {
  case WS_EVT_CONNECT:
#ifdef DEBUG
    Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
#endif
    cameraClientId = client->id();
    // Create task to sendCameraPicture
    xTaskCreatePinnedToCore(
        sendCameraPicture,            /* Function to implement the task */
        "sendCameraPicture",          /* Name of the task */
        STACK_SIZE,                   /* Stack size in words */
        NULL,                         /* Task input parameter */
        1,                            /* Priority of the task */
        &sendCameraPictureTask,       /* Task handle. */
        CONFIG_ARDUINO_RUNNING_CORE); /* Core where the task should run */

    // Create task to cleanupWSClients
    xTaskCreatePinnedToCore(
        cleanupWSClients,             /* Function to implement the task */
        "cleanupWSClients",           /* Name of the task */
        STACK_SIZE,                   /* Stack size in words */
        NULL,                         /* Task input parameter */
        1,                            /* Priority of the task */
        &cleanupWSClientsTask,        /* Task handle. */
        CONFIG_ARDUINO_RUNNING_CORE); /* Core where the task should run */

    break;
  case WS_EVT_DISCONNECT:
#ifdef DEBUG
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
#endif
    cameraClientId = 0;
    vTaskDelete(sendCameraPictureTask);
    vTaskDelete(cleanupWSClientsTask);
    break;
  case WS_EVT_DATA:
    break;
  case WS_EVT_PONG:
  case WS_EVT_ERROR:
    break;
  default:
    break;
  }
}

void moveCar(int inputValue)
{
#ifdef DEBUG
  Serial.printf("Got value as %d\n", inputValue);
#endif
  switch (inputValue)
  {
  case FORWARD:
    forward(leftMotor, rightMotor, motorSpeed);
    leftBackLed(HIGH);
    rightBackLed(HIGH);
    break;
  case BACKWARD:
    back(leftMotor, rightMotor, motorSpeed);
    leftBackLed(HIGH);
    rightBackLed(HIGH);
    break;
  case LEFT:
    left(leftMotor, rightMotor, motorSpeed);
    leftBackLed(HIGH);
    rightBackLed(HIGH);
    break;
  case RIGHT:
    right(leftMotor, rightMotor, motorSpeed);
    leftBackLed(HIGH);
    rightBackLed(HIGH);
    break;
  case FORWARDLEFT:
    forwardleft(leftMotor, rightMotor, motorSpeed);
    leftBackLed(HIGH);
    rightBackLed(HIGH);
    break;
  case FORWARDRIGHT:
    forwardright(leftMotor, rightMotor, motorSpeed);
    leftBackLed(HIGH);
    rightBackLed(HIGH);
    break;
  case BACKLEFT:
    backleft(leftMotor, rightMotor, motorSpeed);
    leftBackLed(HIGH);
    rightBackLed(HIGH);
    break;
  case BACKRIGHT:
    backright(leftMotor, rightMotor, motorSpeed);
    leftBackLed(HIGH);
    rightBackLed(HIGH);
    break;
  case STOP:
    brake(leftMotor, rightMotor);
    leftBackLed(LOW);
    rightBackLed(LOW);
    break;
  default:
    brake(leftMotor, rightMotor);
    leftBackLed(LOW);
    rightBackLed(LOW);
    break;
  }
}

void obstacleAvoidanceMode(void *parameters)
{
  unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  int detect;
#ifdef DEBUG
  Serial.printf("obstacleAvoidanceMode() running on core: %d\n", xPortGetCoreID());
#endif
  for (;;)
  {
#ifdef DEBUG
// Serial.println( uxTaskGetStackHighWaterMark(nullptr));
#endif
    detect = pcf8574.digitalRead(P5); // read obstacle status and store it into "detect"

    if (detect == LOW)
    {
#ifdef DEBUG
      Serial.println("Obstacle detected");
#endif
      moveCar(BACKWARD);
      previousMillis = currentMillis;
      while (currentMillis - previousMillis <= 500)
      {
        currentMillis = millis();
      }

      moveCar(RIGHT);
      previousMillis = currentMillis;
      while (currentMillis - previousMillis <= 800)
      {
        currentMillis = millis();
      }
    }
    else
    {
#ifdef DEBUG
      Serial.println("Clear - no obstacle detected");
#endif
      moveCar(FORWARD);
    }
    previousMillis = currentMillis;
    while (currentMillis - previousMillis <= 100)
    {
      currentMillis = millis();
    }
  }
}

void onCarInputWebSocketEvent(AsyncWebSocket *server,
                              AsyncWebSocketClient *client,
                              AwsEventType type,
                              void *arg,
                              uint8_t *data,
                              size_t len)
{
  switch (type)
  {
  case WS_EVT_CONNECT:
#ifdef DEBUG
    Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
#endif
    leftBackLed(LOW);
    rightBackLed(LOW);
    break;
  case WS_EVT_DISCONNECT:
#ifdef DEBUG
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
#endif
    // Actions when disconnected
    brake(leftMotor, rightMotor);
    leftBackLed(HIGH);
    rightBackLed(HIGH);

    enableLight = false;
    digitalWrite(lightPin, LOW);

    panServo.write(panCenter);
    tiltServo.write(tiltCenter);

    if (melodyOn)
    {
      melodyOn = false;
      ledcDetachPin(buzzerPin);
      vTaskDelete(playMelodyTask);
    }

    if (enableObstacleAvoidance)
    {
      enableObstacleAvoidance = false;
      vTaskDelete(obstacleAvoidanceModeTask);
    }

    break;
  case WS_EVT_DATA:
    AwsFrameInfo *info;
    info = (AwsFrameInfo *)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
    {
      std::string myData = "";
      myData.assign((char *)data, len);
      std::istringstream ss(myData);
      std::string key, value;
      std::getline(ss, key, ',');
      std::getline(ss, value, ',');
#ifdef DEBUG
      Serial.printf("Key [%s] Value[%s]\n", key.c_str(), value.c_str());
#endif
      int valueInt = atoi(value.c_str());
      if (key == "MoveCar")
      {
        moveCar(valueInt);
      }
      else if (key == "Speed")
      {
        motorSpeed = valueInt;
      }
      else if (key == "Light")
      {
        if (enableLight == false)
        {
          enableLight = true;
          digitalWrite(lightPin, HIGH);
        }
        else
        {
          enableLight = false;
          digitalWrite(lightPin, LOW);
        }
      }
      else if (key == "Pan")
      {
        int tempPan = map(valueInt, 0, 180, 180, 0);
        panServo.write(tempPan);
      }
      else if (key == "Tilt")
      {

#ifdef DEBUG
        int tempTilt = map(valueInt, 0, 180, 60, 0);
        tiltServo.write(tempTilt);
#else
        int tempTilt = map(valueInt, 0, 180, 180, 0);
        tiltServo.write(tempTilt);
#endif
      }
      else if (key == "CenterServos")
      {
        panServo.write(panCenter);
        tiltServo.write(tiltCenter);
      }
      else if (key == "InitMelody")
      {
        if (melodyOn == false)
        {
          melodyOn = true;
          xTaskCreatePinnedToCore(
              playMelody,                   /* Function to implement the task */
              "playMelody",                 /* Name of the task */
              STACK_SIZE,                   /* Stack size in words */
              NULL,                         /* Task input parameter */
              1,                            /* Priority of the task */
              &playMelodyTask,              /* Task handle. */
              CONFIG_ARDUINO_RUNNING_CORE); /* Core where the task should run */
#ifdef DEBUG
          Serial.println("Melody ON");
#endif
        }
        else
        {
          melodyOn = false;
          ledcDetachPin(buzzerPin);
          vTaskDelete(playMelodyTask);
#ifdef DEBUG
          Serial.println("Melody OFF");
#endif
        }
      }
      else if (key == "ObstacleAvoidance")
      {
        if (enableObstacleAvoidance == false)
        {
          enableObstacleAvoidance = true;
          xTaskCreatePinnedToCore(
              obstacleAvoidanceMode,        /* Function to implement the task */
              "obstacleAvoidanceMode",      /* Name of the task */
              STACK_SIZE,                   /* Stack size in words */
              NULL,                         /* Task input parameter */
              1,                            /* Priority of the task */
              &obstacleAvoidanceModeTask,   /* Task handle. */
              CONFIG_ARDUINO_RUNNING_CORE); /* Core where the task should run */
#ifdef DEBUG
          Serial.println("Obstacle Avoidance Mode ON");
#endif
        }
        else
        {
          enableObstacleAvoidance = false;
          vTaskDelete(obstacleAvoidanceModeTask);
          moveCar(STOP);
#ifdef DEBUG
          Serial.println("Obstacle Avoidance Mode OFF");
#endif
        }
      }
    }
    break;
  case WS_EVT_PONG:
  case WS_EVT_ERROR:
    break;
  default:
    break;
  }
}

void initServer()
{
#ifdef DEBUG
  Serial.printf("initServer() running on core: %d\n", xPortGetCoreID());
#endif

  for (;;)
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      server.on("/", HTTP_GET, handleRoot);
      server.on("/style.css", HTTP_GET, handleStyle);
      server.on("/readRSSI", HTTP_GET, handleRSSI);
      server.onNotFound(handleNotFound);
      wsCamera.onEvent(onCameraWebSocketEvent);
      server.addHandler(&wsCamera);
      wsCarInput.onEvent(onCarInputWebSocketEvent);
      server.addHandler(&wsCarInput);
      server.begin();
#ifdef DEBUG
      Serial.println("HTTP server started");
#endif
      return;
    }
  }
}

void initTasks()
{
  xTaskCreatePinnedToCore(
      keepWiFiAlive,                /* Function to implement the task */
      "keepWiFiAlive",              /* Name of the task */
      STACK_SIZE,                   /* Stack size in words */
      NULL,                         /* Task input parameter */
      1,                            /* Priority of the task */
      &keepWiFiAliveTask,           /* Task handle. */
      CONFIG_ARDUINO_RUNNING_CORE); /* Core where the task should run */

  xTaskCreatePinnedToCore(
      arduinoOTA,                   /* Function to implement the task */
      "arduinoOTA",                 /* Name of the task */
      STACK_SIZE,                   /* Stack size in words */
      NULL,                         /* Task input parameter */
      1,                            /* Priority of the task */
      &arduinoOTATask,              /* Task handle. */
      CONFIG_ARDUINO_RUNNING_CORE); /* Core where the task should run */
}

void setupCamera()
{
#ifdef DEBUG
  Serial.printf("setupCamera() running on core: %d\n", xPortGetCoreID());
#endif
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_4;
  config.ledc_timer = LEDC_TIMER_2;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG; // for streaming

  // if (config.pixel_format == PIXFORMAT_JPEG)
  // {
  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality for larger pre-allocated frame buffer.
  if (psramFound())
  {
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.frame_size = FRAMESIZE_CIF; // FRAMESIZE_ + QVGA|CIF|HVGA|VGA|SVGA|XGA|HD|SXGA|UXGA
    config.jpeg_quality = 10;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
    heap_caps_malloc_extmem_enable(psramLimit);

#ifdef DEBUG
    Serial.printf("PSRAM initialized. malloc to take memory from psram above %d\n", psramLimit);
#endif
  }
  else
  {
    // Limit the frame size when PSRAM is not available
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  }
  //}
  //   else
  //   {
  //     // Best option for face detection/recognition
  //     config.frame_size = FRAMESIZE_240X240;
  // #if CONFIG_IDF_TARGET_ESP32S3
  //     config.fb_count = 2;
  // #endif
  //   }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
#ifdef DEBUG
    Serial.printf("Camera init failed with error 0x%x", err);
#endif
    return;
  }
}

void setupPinModes()
{
  pinMode(builtinPin, OUTPUT);
  digitalWrite(builtinPin, LOW); // LED OFF

  // turn off buzzer, just in case ;P
  ledcDetachPin(buzzerPin);

  panServo.attach(panPin);
  tiltServo.attach(tiltPin);
  panServo.write(panCenter);
  tiltServo.write(tiltCenter);

  pcf8574.pinMode(P5, INPUT);  // distance detector
  pcf8574.pinMode(P6, OUTPUT); // left backlight
  pcf8574.pinMode(P7, OUTPUT); // right backlight
  pinMode(lightPin, OUTPUT);   // front lights

#ifdef DEBUG
  Serial.print("Init pcf8574...");
#endif
  if (pcf8574.begin())
  {
#ifdef DEBUG
    Serial.println("OK");
#endif
  }
  else
  {
#ifdef DEBUG
    Serial.println("FAILED");
#endif
  }

  // indicate car started
  ledIndicator(3, 250);
}

void setup()
{
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // disable detector

#ifdef DEBUG
  Serial.begin(115200);
#endif

  if (!SPIFFS.begin(true))
  {
#ifdef DEBUG
    Serial.println("An error has ocurred while mounting SPIFFS");
#endif
    return;
  }
  setupPinModes();
  setupCamera();
  initTasks();
  initServer();
}

void loop()
{
#ifdef DEBUG
  Serial.printf("Total heap: %d, Free heap: %d, Total PSRAM: %d,Free PSRAM: %d\n", ESP.getHeapSize(), ESP.getFreeHeap(), ESP.getPsramSize(), ESP.getFreePsram());
  Serial.println(uxTaskGetStackHighWaterMark(nullptr));
#else
  vTaskDelay(portMAX_DELAY);
#endif
}
