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

void server_task_start() {
  server.begin();
  xTaskCreatePinnedToCore(serverTask, "serverTask", 8192, nullptr, 1, nullptr, 0);
}

static void serverTask(void *pvParameters) {
  (void)pvParameters;
  const long timeoutTime = 2000;
  for (;;) {
    // Blink heartbeat
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
    String currentLine = "";                // make a String to hold incoming data from the client
    String header;
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
            if (header.indexOf("GET /info") >= 0) {
              String ipStr = WiFi.localIP().toString();
              String jsonResponse = "{\"hostname\":\"" + String(WIFI_HOSTNAME) + "\",\"ip\":\"" + ipStr + "\"}";
              client.println("HTTP/1.1 200 OK");
              client.println("Content-type: application/json");
              client.println("Connection: close");
              client.println();
              client.println(jsonResponse);
              break;
            } else if (header.indexOf("GET /sensor/ht") >= 0) {
                if (!sht3xReady) {
                  client.println("HTTP/1.1 503 Service Unavailable");
                  client.println("Content-type: application/json");
                  client.println("Connection: close");
                  client.println();
                  client.println("{\"error\":\"SHT3x not initialized\"}");
                  break;
                }
                float aTemperature = 0.0;
                float aHumidity = 0.0;
                int16_t error = 0;
                char errorMessage[64];
                error = sensor.measureSingleShot(REPEATABILITY_MEDIUM, false, aTemperature, aHumidity);
                if (error != NO_ERROR) {
#ifdef USE_SERIAL
                    Serial.print("Error trying to execute blockingReadMeasurement(): ");
                    errorToString(error, errorMessage, sizeof errorMessage);
                    Serial.println(errorMessage);
#endif
                    client.println("HTTP/1.1 500 Internal Server Error");
                    client.println("Content-type: application/json");
                    client.println("Connection: close");
                    client.println();
                    client.println("{\"error\":\"Sensor read failed\"}");
                    break;
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

              if (g_motorQueue) {
                MotorCommandMsg msg{};
                bool recognized = false;
                if (header.indexOf("GET /motor/forward") >= 0) {
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
