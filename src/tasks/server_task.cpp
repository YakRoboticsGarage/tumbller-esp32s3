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
static bool handleBalanceRequest(WiFiClient &client, const String &header);

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
            if (handleBalanceRequest(client, header)) break;

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
  
  // Take I2C mutex to avoid bus contention with balancer's MPU6050 reads
  if (xSemaphoreTake(g_i2cMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    sendJson(client, "HTTP/1.1 503 Service Unavailable", "{\"error\":\"I2C bus busy\"}");
    return true;
  }
  int16_t error = sensor.measureSingleShot(REPEATABILITY_MEDIUM, false, aTemperature, aHumidity);
  xSemaphoreGive(g_i2cMutex);
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

// Helper to get state name string
static const char* balanceStateName(BalanceState state) {
  switch (state) {
    case BalanceState::INIT:      return "INIT";
    case BalanceState::LEAN_BACK: return "LEAN_BACK";
    case BalanceState::START:     return "START";
    case BalanceState::BALANCING: return "BALANCING";
    case BalanceState::FALLEN:    return "FALLEN";
    default:                      return "UNKNOWN";
  }
}

// Helper to extract float value from path like "/balance/kp/55.5"
static float parsePathFloat(const String &header, const char* prefix) {
  int idx = header.indexOf(prefix);
  if (idx < 0) return -1.0f;
  idx += strlen(prefix);
  int end = header.indexOf(' ', idx);
  if (end < 0) end = header.length();
  return header.substring(idx, end).toFloat();
}

static bool handleBalanceRequest(WiFiClient &client, const String &header) {
  // /balance/* → balance control endpoints
  if (header.indexOf("GET /balance/") < 0) return false;

  // /balance/status → return current state, angle, speed as JSON
  if (header.indexOf("GET /balance/status") >= 0) {
    Balancer::Gains gains = g_balancer.getGains();
    String json = "{";
    json += "\"state\":\"" + String(balanceStateName(g_balancer.getState())) + "\",";
    json += "\"angle\":" + String(g_balancer.getAngle(), 2) + ",";
    json += "\"speed\":" + String(g_balancer.getSpeedEstimate(), 2) + ",";
    json += "\"upright\":" + String(g_balancer.isUpright() ? "true" : "false") + ",";
    json += "\"kp\":" + String(gains.kp, 3) + ",";
    json += "\"kd\":" + String(gains.kd, 3) + ",";
    json += "\"kp_speed\":" + String(gains.kp_speed, 3) + ",";
    json += "\"ki_speed\":" + String(gains.ki_speed, 3) + ",";
    json += "\"kp_turn\":" + String(gains.kp_turn, 3) + ",";
    json += "\"kd_turn\":" + String(gains.kd_turn, 3) + ",";
    json += "\"i2c_fails\":" + String(g_balancer.getI2CFailures());
    json += "}";
    sendJson(client, "HTTP/1.1 200 OK", json);
    return true;
  }

  // /balance/start → reset to INIT state
  if (header.indexOf("GET /balance/start") >= 0) {
    g_balancer.restart();
    sendJson(client, "HTTP/1.1 200 OK", "{\"status\":\"restarting\"}");
    return true;
  }

  // /balance/stop → set to FALLEN state (motors off)
  if (header.indexOf("GET /balance/stop") >= 0) {
    g_balancer.fall();
    sendJson(client, "HTTP/1.1 200 OK", "{\"status\":\"stopped\"}");
    return true;
  }

  // Individual gain endpoints: /balance/<param>/<value>
  Balancer::Gains gains = g_balancer.getGains();
  bool gainUpdated = false;

  if (header.indexOf("GET /balance/kp/") >= 0) {
    float val = parsePathFloat(header, "/balance/kp/");
    if (val >= 0) { gains.kp = val; gainUpdated = true; }
  } else if (header.indexOf("GET /balance/kd/") >= 0) {
    float val = parsePathFloat(header, "/balance/kd/");
    if (val >= 0) { gains.kd = val; gainUpdated = true; }
  } else if (header.indexOf("GET /balance/kp_speed/") >= 0) {
    float val = parsePathFloat(header, "/balance/kp_speed/");
    if (val >= 0) { gains.kp_speed = val; gainUpdated = true; }
  } else if (header.indexOf("GET /balance/ki_speed/") >= 0) {
    float val = parsePathFloat(header, "/balance/ki_speed/");
    if (val >= 0) { gains.ki_speed = val; gainUpdated = true; }
  } else if (header.indexOf("GET /balance/kp_turn/") >= 0) {
    float val = parsePathFloat(header, "/balance/kp_turn/");
    if (val >= 0) { gains.kp_turn = val; gainUpdated = true; }
  } else if (header.indexOf("GET /balance/kd_turn/") >= 0) {
    float val = parsePathFloat(header, "/balance/kd_turn/");
    if (val >= 0) { gains.kd_turn = val; gainUpdated = true; }
  }

  if (gainUpdated) {
    g_balancer.setGains(gains);
    String json = "{\"status\":\"updated\",";
    json += "\"kp\":" + String(gains.kp, 3) + ",";
    json += "\"kd\":" + String(gains.kd, 3) + ",";
    json += "\"kp_speed\":" + String(gains.kp_speed, 3) + ",";
    json += "\"ki_speed\":" + String(gains.ki_speed, 3) + ",";
    json += "\"kp_turn\":" + String(gains.kp_turn, 3) + ",";
    json += "\"kd_turn\":" + String(gains.kd_turn, 3);
    json += "}";
    sendJson(client, "HTTP/1.1 200 OK", json);
    return true;
  }

  // Unknown balance route
  sendJson(client, "HTTP/1.1 404 Not Found", "{\"error\":\"Unknown balance endpoint\"}");
  return true;
}
