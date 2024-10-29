#include <Arduino.h>
#include <WiFi.h>
#include <SensirionI2cSht3x.h>
#include <Wire.h>

#include "Motor.hpp"
#include "wifi_setup.h"

// Define USE_SERIAL to enable Serial debugging
// #define USE_SERIAL

String hostname = "tumbller";

Motor Motor;

/*You can set the speed: 0~255 */
#define SPEED 60
#define FORWARD_BACK_TIME 2000  // 2 seconds in milliseconds
#define TURN_TIME 1000         // 1 second in milliseconds

char strbuf[][50]={
  "FORWARD",
  "BACK",
  "LEFT",
  "RIGHT",
  "STOP"
};

WiFiServer server(80);

String header;

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

// Motor control timing
unsigned long motorStartTime = 0;
bool motorRunning = false;
unsigned long currentMotorTimeout = FORWARD_BACK_TIME;  // Default to forward/back time

String motorState;

SensirionI2cSht3x sensor;

static char errorMessage[64];
static int16_t error;

void setup() {
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_GREEN, HIGH);
  Motor.Pin_init();
  Motor.Stop(0);
  
#ifdef USE_SERIAL
  Serial.begin(115200);
  while (!Serial) {
    delay(100);
  }
  Serial.println("Connecting to wifi");
  Serial.println(WIFI_SSID);
#endif

  //Setup wifi
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname(hostname.c_str()); //define hostname
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  
  int count = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    digitalWrite(LED_RED, !digitalRead(LED_RED));
#ifdef USE_SERIAL
    Serial.print(".");
    count++;
    if(count > 80) {
      Serial.print("\n");
    }
#endif
  }
  digitalWrite(LED_RED, HIGH);

#ifdef USE_SERIAL
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
#endif

  server.begin();

  //Setup humidity and temp sensor
  // Wire.begin();
  // sensor.begin(Wire, SHT30_I2C_ADDR_44);

  // sensor.stopMeasurement();
  // delay(1);
  // sensor.softReset();
  // delay(100);
  // uint16_t aStatusRegister = 0u;
  // error = sensor.readStatusRegister(aStatusRegister);
  // if (error != NO_ERROR) {
  //     Serial.print("Error trying to execute readStatusRegister(): ");
  //     errorToString(error, errorMessage, sizeof errorMessage);
  //     Serial.println(errorMessage);
  //     return;
  // }
  // Serial.print("aStatusRegister: ");
  // Serial.print(aStatusRegister);
  // Serial.println(); 
}

void startMotorTimer(unsigned long timeout) {
  motorStartTime = millis();
  motorRunning = true;
  currentMotorTimeout = timeout;
}

void checkAndStopMotor() {
  if (motorRunning && (millis() - motorStartTime >= currentMotorTimeout)) {
    Motor.Stop(0);
    motorRunning = false;
#ifdef USE_SERIAL
    Serial.println("Motor stopped after time limit");
#endif
    motorState = strbuf[4]; // STOP
  }
}

void loop() {
  // Check if we need to stop the motor
  checkAndStopMotor();

  WiFiClient client = server.available();   // Listen for incoming clients

  if (millis() % 1000 == 0) {
    digitalWrite(LED_BLUE, !digitalRead(LED_BLUE));
  }
  
  if (client) {
    currentTime = millis();
    previousTime = currentTime;
#ifdef USE_SERIAL
    Serial.println("New Client.");
#endif
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected() && currentTime - previousTime <= timeoutTime) {
      currentTime = millis();
      if (client.available()) {
        char c = client.read();
#ifdef USE_SERIAL
        Serial.write(c);
#endif
        header += c;
        if (c == '\n') {
          if (currentLine.length() == 0) {
            if (header.indexOf("GET /sensor/ht") >= 0) {
                float aTemperature = 0.0;
                float aHumidity = 0.0;
                error = sensor.measureSingleShot(REPEATABILITY_MEDIUM, false, aTemperature, aHumidity);
                if (error != NO_ERROR) {
#ifdef USE_SERIAL
                    Serial.print("Error trying to execute blockingReadMeasurement(): ");
                    errorToString(error, errorMessage, sizeof errorMessage);
                    Serial.println(errorMessage);
#endif
                    return;
                }
#ifdef USE_SERIAL
                Serial.print("aTemperature: ");
                Serial.print(aTemperature);
                Serial.print("\t");
                Serial.print("aHumidity: ");
                Serial.print(aHumidity);
                Serial.println();
#endif

              String jsonResponse = "{\"temperature\": " + String(aTemperature) + ", \"humidity\": " + String(aHumidity) + "}";

              client.println("HTTP/1.1 200 OK");
              client.println("Content-type: application/json");
              client.println("Connection: close");
              client.println();
              client.println(jsonResponse);
            } else {
              client.println("HTTP/1.1 200 OK");
              client.println("Content-type:text/html");
              client.println("Connection: close");
              client.println();

              // Move the motors
              if (header.indexOf("GET /motor/forward") >= 0) {
#ifdef USE_SERIAL
                Serial.println(strbuf[0]);
#endif
                motorState = strbuf[0];
                Motor.Forward(SPEED);
                startMotorTimer(FORWARD_BACK_TIME);
              } else if (header.indexOf("GET /motor/back") >= 0) {
#ifdef USE_SERIAL
                Serial.println(strbuf[1]);
#endif
                motorState = strbuf[1];
                Motor.Back(SPEED);
                startMotorTimer(FORWARD_BACK_TIME);
              } else if (header.indexOf("GET /motor/left") >= 0) {
#ifdef USE_SERIAL
                Serial.println(strbuf[2]);
#endif
                motorState = strbuf[2];
                Motor.Left(SPEED);
                startMotorTimer(TURN_TIME);
              } else if (header.indexOf("GET /motor/right") >= 0) {
#ifdef USE_SERIAL
                Serial.println(strbuf[3]);
#endif
                motorState = strbuf[3];
                Motor.Right(SPEED);
                startMotorTimer(TURN_TIME);
              } else if (header.indexOf("GET /motor/stop") >= 0) {
#ifdef USE_SERIAL
                Serial.println(strbuf[4]);
#endif
                motorState = strbuf[4];
                Motor.Stop(SPEED);
                motorRunning = false;
              }

              client.println();
              break;
            }
          } else {
            currentLine = "";
          }
        } else if (c != '\r') {
          currentLine += c;
        }
      }
    }
    header = "";
    client.stop();
#ifdef USE_SERIAL
    Serial.println("Client disconnected.");
    Serial.println("");
#endif
  }
}