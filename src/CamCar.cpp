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

// ESP32-CAM
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

const static int psramLimit = 10000;

const static int builtinLedPin = 33;

const static int lightPin = 4;
static bool enableLight = false;

const static int buzzerPin = 13;
const static int buzzerChannel = 3;
static bool melodyOn = false;

Servo panServo;
Servo tiltServo;

const static int panPin = 12;
const static int tiltPin = 2;
const static int panCenter = 80;

const static int tiltCenter = 90;

static bool enableObstacleAvoidance = false;
static bool obstacleFound = false;

PCF8574 pcf8574(0x20, 14, 15);      // PCF8574 library included in CamCar.h
Motor leftMotor(P3, P4, 1, 1, P2);  // BIN1,BIN2,PWMB,OFFSET,STBY.
Motor rightMotor(P1, P0, 3, 1, P2); // AIN1,AIN2,PWMA,OFFSET,STBY.

static int motorSpeed = 255;
static int currentDirection = 0;

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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
  for (int i = 0; i < blinkTimes; i++)
  {
    digitalWrite(builtinLedPin, LOW); // LED ON
    toneToPlay(buzzerPin, buzzerChannel, NOTE_G5, delayTimeMS);
    // vTaskDelay(pdMS_TO_TICKS(delayTimeMS));    No delay needed because ToneToPlay has it
    digitalWrite(builtinLedPin, HIGH); // LED OFF
    vTaskDelay(pdMS_TO_TICKS(delayTimeMS));
  }
}

void ledIndicator(int state)
{
  if (state == HIGH)
  {
    digitalWrite(builtinLedPin, LOW); // LED ON
  }
  if (state == LOW)
  {
    digitalWrite(builtinLedPin, HIGH); // LED OFF
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
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void keepWiFiAlive(void *parameters)
{
#ifdef DEBUG
  Serial.printf("keepWiFiAlive() running on core: %d\n", xPortGetCoreID());
#endif

  // ===========================
  // Enter your WiFi credentials
  // ===========================
  // Wifi Credentials //
  const char *sta_ssid = "My Pills";       // set Wifi network you want to connect to
  const char *sta_password = "Ivonne2011"; // set password for Wifi network
  const int WIFI_TIMEOUT_MS = 10000;

  ////////////// Set ESP32 Wifi hostname based on chip mac address//////////////////
  char chip_id[15];
  snprintf(chip_id, 15, "%04X", (uint16_t)(ESP.getEfuseMac() >> 32));
  String hostname = "esp32cam-" + String(chip_id);
  /////////////////////////////////////////////////////////////////////////////////

  for (;;)
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      // Serial.println("WiFi still connected");
      vTaskDelay(pdMS_TO_TICKS(WIFI_TIMEOUT_MS));
      continue;
    }
    else
    {
      // first, set ESP32 as STA mode to connect with a Wifi network

      Serial.println("");
      Serial.printf("Connecting to: %s\n", String(sta_ssid));
      Serial.printf("Password: %s\n", String(sta_password));
      WiFi.mode(WIFI_STA);
      WiFi.begin(sta_ssid, sta_password);
      WiFi.setTxPower(WIFI_POWER_19_5dBm);

      // try to connect with Wifi network about 10 seconds

      for (int i = 0; i < WIFI_TIMEOUT_MS / 1000; i++)
      {
        Serial.print(".");
        ledIndicator(1, 500);
        if (WiFi.status() == WL_CONNECTED)
        {
          IPAddress myIP = WiFi.localIP();
          Serial.println("");
          Serial.println("*WiFi-STA-Mode*");
          Serial.printf("IP: %s\n", myIP.toString());
          ledIndicator(10, 50);
          ledIndicator(HIGH);
          break;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
      }

      // Check if connected with Wifi network
      if (WiFi.status() != WL_CONNECTED)
      {
        Serial.println("[WIFI] FAILED");
        continue;
      }
    }
  }
}

void playMelody(void *parameters)
{
#ifdef DEBUG
  Serial.printf("playMelody() running on core: %d\n", xPortGetCoreID());
#endif
  gameOfThrones(buzzerPin, buzzerChannel);
  melodyOn = false;
  ledcDetachPin(buzzerPin);
  vTaskDelete(playMelodyTask);
}

void cleanupWSClients(void *parameters)
{
#ifdef DEBUG
  Serial.printf("cleanupWSClients() running on core: %d\n", xPortGetCoreID());
#endif
  for (;;)
  {
#ifdef DEBUG
// Serial.println( uxTaskGetStackHighWaterMark(nullptr));
#endif
    wsCamera.cleanupClients();
    wsCarInput.cleanupClients();
#ifdef DEBUG
    Serial.println("ws cleaned");
#endif
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void sendCameraPicture(void *parameters)
{
  camera_fb_t *fb = nullptr;
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
    else
    {
      // capture a frame
      fb = esp_camera_fb_get();
      if (!fb)
      {
#ifdef DEBUG
        Serial.println("Frame buffer could not be acquired");
#endif
        esp_camera_fb_return(fb);
        continue;
      }
      else
      {
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
          vTaskDelay(pdMS_TO_TICKS(1));
        }
      }
    }
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
        tskIDLE_PRIORITY,             /* Priority of the task */
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
    currentDirection = FORWARD;
    forward(leftMotor, rightMotor, motorSpeed);
    leftBackLed(HIGH);
    rightBackLed(HIGH);
    break;
  case BACKWARD:
    currentDirection = BACKWARD;
    back(leftMotor, rightMotor, motorSpeed);
    leftBackLed(HIGH);
    rightBackLed(HIGH);
    break;
  case LEFT:
    currentDirection = LEFT;
    left(leftMotor, rightMotor, motorSpeed);
    leftBackLed(HIGH);
    rightBackLed(HIGH);
    break;
  case RIGHT:
    currentDirection = RIGHT;
    right(leftMotor, rightMotor, motorSpeed);
    leftBackLed(HIGH);
    rightBackLed(HIGH);
    break;
  case FORWARDLEFT:
    currentDirection = FORWARDLEFT;
    forwardleft(leftMotor, rightMotor, motorSpeed);
    leftBackLed(HIGH);
    rightBackLed(HIGH);
    break;
  case FORWARDRIGHT:
    currentDirection = FORWARDRIGHT;
    forwardright(leftMotor, rightMotor, motorSpeed);
    leftBackLed(HIGH);
    rightBackLed(HIGH);
    break;
  case BACKLEFT:
    currentDirection = BACKLEFT;
    backleft(leftMotor, rightMotor, motorSpeed);
    leftBackLed(HIGH);
    rightBackLed(HIGH);
    break;
  case BACKRIGHT:
    currentDirection = BACKRIGHT;
    backright(leftMotor, rightMotor, motorSpeed);
    leftBackLed(HIGH);
    rightBackLed(HIGH);
    break;
  case STOP:
    currentDirection = STOP;
    brake(leftMotor, rightMotor);
    leftBackLed(LOW);
    rightBackLed(LOW);
    break;
  }
}

void obstacleAvoidanceMode(void *parameters)
{
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

    if (detect == LOW && (currentDirection == FORWARD || currentDirection == FORWARDLEFT || currentDirection == FORWARDRIGHT))
    {
#ifdef DEBUG
      Serial.println("Obstacle detected");
#endif
      obstacleFound = true;
      moveCar(STOP);
    }
    else
    {
#ifdef DEBUG
      Serial.println("Clear - no obstacle detected");
#endif
      obstacleFound = false;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
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
      obstacleFound = false;
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
        int temp = map(valueInt, 0, 100, 200, 255);
        motorSpeed = temp;
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
        int tempTilt = map(valueInt, 0, 180, 180, 0);
        tiltServo.write(tempTilt);
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
              tskIDLE_PRIORITY,             /* Priority of the task */
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
          obstacleFound = false;
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
      Serial.println(" HTTP server started");
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
      tskIDLE_PRIORITY,             /* Priority of the task */
      &keepWiFiAliveTask,           /* Task handle. */
      CONFIG_ARDUINO_RUNNING_CORE); /* Core where the task should run */

  xTaskCreatePinnedToCore(
      arduinoOTA,                   /* Function to implement the task */
      "arduinoOTA",                 /* Name of the task */
      STACK_SIZE,                   /* Stack size in words */
      NULL,                         /* Task input parameter */
      tskIDLE_PRIORITY,             /* Priority of the task */
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
  pinMode(builtinLedPin, OUTPUT);
  digitalWrite(builtinLedPin, HIGH); // LED OFF

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
  // Serial.printf("Total heap: %d, Free heap: %d, Total PSRAM: %d,Free PSRAM: %d\n", ESP.getHeapSize(), ESP.getFreeHeap(), ESP.getPsramSize(), ESP.getFreePsram());
  // Serial.println(uxTaskGetStackHighWaterMark(nullptr));
#else
  vTaskDelay(portMAX_DELAY);
#endif
}
