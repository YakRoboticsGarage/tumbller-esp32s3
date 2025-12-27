// HTTP server task: listens on port 80 and dispatches /info, /sensor/ht, and /motor routes.
#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <SensirionI2cSht3x.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "task_common.hpp"
#include "server_task.hpp"

static WiFiServer server(80);
static void serverTask(void *pvParameters);
static void sendJson(WiFiClient &client, const char *status, const String &body);
static bool handleInfoRequest(WiFiClient &client, const String &header);
static bool handleSensorRequest(WiFiClient &client, const String &header);
static bool handleMotorRequest(WiFiClient &client, const String &header);

void server_task_start() {
  // Launch a dedicated server task pinned to core 0
  server.begin();
  xTaskCreatePinnedToCore(serverTask, "serverTask", 8192, nullptr, 1, nullptr, 0);
}

static void serverTask(void *pvParameters) {
  (void)pvParameters;
  const long timeoutTime = 2000; // client timeout window (ms)
  for (;;) {
    // Blink heartbeat LED once per second to show the task is alive
    {
      static unsigned long lastBlinkMs = 0;
      unsigned long now = millis();
      if (now - lastBlinkMs >= 1000) {
        lastBlinkMs = now;
        digitalWrite(LED_BLUE, !digitalRead(LED_BLUE));
      }
    }

    WiFiClient client = server.available();   // Listen for incoming clients
    if (!client) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    unsigned long currentTime = millis();
    unsigned long previousTime = currentTime;
#ifdef USE_SERIAL
    Serial.println("New Client.");
#endif
        String currentLine = ""; // collect the current header line
        String header;            // full HTTP header buffer
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
            // End of headers: dispatch by path (info → JSON, sensor → JSON, motor → HTML)
            if (handleInfoRequest(client, header)) break;
            if (handleSensorRequest(client, header)) break;
            if (handleMotorRequest(client, header)) break;

            // Unknown route
            client.println("HTTP/1.1 404 Not Found");
            client.println("Connection: close");
            client.println();
            break;
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

static void sendJson(WiFiClient &client, const char *status, const String &body) {
  // Minimal helper to emit JSON responses with a given HTTP status line
  client.println(status);
  client.println("Content-type: application/json");
  client.println("Connection: close");
  client.println();
  client.println(body);
}

static bool handleInfoRequest(WiFiClient &client, const String &header) {
  // /info → report hostname and current IP as JSON
  if (header.indexOf("GET /info") < 0) return false;
  String ipStr = WiFi.localIP().toString();
  String jsonResponse = "{\"hostname\":\"" + String(WIFI_HOSTNAME) + "\",\"ip\":\"" + ipStr + "\"}";
  sendJson(client, "HTTP/1.1 200 OK", jsonResponse);
  return true;
}

static bool handleSensorRequest(WiFiClient &client, const String &header) {
  // /sensor/ht → read SHT3x once and return temperature/humidity JSON
  if (header.indexOf("GET /sensor/ht") < 0) return false;
  if (!sht3xReady) {
    sendJson(client, "HTTP/1.1 503 Service Unavailable", "{\"error\":\"SHT3x not initialized\"}");
    return true;
  }

  float aTemperature = 0.0;
  float aHumidity = 0.0;
  int16_t error = sensor.measureSingleShot(REPEATABILITY_MEDIUM, false, aTemperature, aHumidity);
  if (error != NO_ERROR) {
#ifdef USE_SERIAL
    char errorMessage[64];
    Serial.print("Error trying to execute blockingReadMeasurement(): ");
    errorToString(error, errorMessage, sizeof errorMessage);
    Serial.println(errorMessage);
#endif
    sendJson(client, "HTTP/1.1 500 Internal Server Error", "{\"error\":\"Sensor read failed\"}");
    return true;
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
  sendJson(client, "HTTP/1.1 200 OK", jsonResponse);
  return true;
}

static bool handleMotorRequest(WiFiClient &client, const String &header) {
  // /motor/* → enqueue a motor command and acknowledge
  if (header.indexOf("GET /motor/") < 0) return false;

  client.println("HTTP/1.1 200 OK");
  client.println("Content-type:text/html");
  client.println("Connection: close");
  client.println();

  if (!g_motorQueue) {
    client.println();
    return true;
  }

  MotorCommandMsg msg{};
  bool recognized = false;
  // Dispatch motor commands by matching the request path to a direction/stop action.
  // header.indexOf(...) returns the substring position or -1; >= 0 means the route is present.
  if (header.indexOf("GET /motor/forward") >= 0) { // forward command
#ifdef USE_SERIAL
    Serial.println(MOTOR_STATE_STRINGS[0]);
#endif
    msg.cmd = MotorCommand::Forward;
    msg.timeoutMs = MOTOR_FORWARD_BACK_TIME;
    recognized = true;
  } else if (header.indexOf("GET /motor/back") >= 0) {
#ifdef USE_SERIAL
    Serial.println(MOTOR_STATE_STRINGS[1]);
#endif
    msg.cmd = MotorCommand::Back;
    msg.timeoutMs = MOTOR_FORWARD_BACK_TIME;
    recognized = true;
  } else if (header.indexOf("GET /motor/left") >= 0) {
#ifdef USE_SERIAL
    Serial.println(MOTOR_STATE_STRINGS[2]);
#endif
    msg.cmd = MotorCommand::Left;
    msg.timeoutMs = MOTOR_TURN_TIME;
    recognized = true;
  } else if (header.indexOf("GET /motor/right") >= 0) {
#ifdef USE_SERIAL
    Serial.println(MOTOR_STATE_STRINGS[3]);
#endif
    msg.cmd = MotorCommand::Right;
    msg.timeoutMs = MOTOR_TURN_TIME;
    recognized = true;
  } else if (header.indexOf("GET /motor/stop") >= 0) {
#ifdef USE_SERIAL
    Serial.println(MOTOR_STATE_STRINGS[4]);
#endif
    msg.cmd = MotorCommand::Stop;
    msg.timeoutMs = 0;
    recognized = true;
  }

  if (recognized) {
    xQueueSend(g_motorQueue, &msg, 0);
  }

  client.println();
  return true;
}
