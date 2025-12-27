#include <Arduino.h>
#include <WiFi.h>
#include <SensirionI2cSht3x.h>
#include <Wire.h>
#include <ESPmDNS.h>

#include "config.hpp"

#include "tasks/task_common.hpp"
#include "tasks/motor_task.hpp"
#include "tasks/server_task.hpp"

#include "drivers/wifi_setup.h"

String hostname = WIFI_HOSTNAME;

void setup() {
  pinMode(LED_RED, OUTPUT);

  pinMode(LED_BLUE, OUTPUT);
#ifdef USE_SERIAL
  Serial.begin(115200);
  unsigned long serialWaitStart = millis();
  while (!Serial && (millis() - serialWaitStart) < 500) {
    delay(10);
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

  // Start mDNS service
  // Note: hostname should not contain underscores, use hyphens instead
  delay(100); // Small delay to ensure WiFi is fully ready
  if (!MDNS.begin(hostname.c_str())) {
#ifdef USE_SERIAL
    Serial.println("Error setting up MDNS responder!");
    Serial.print("Failed hostname: ");
    Serial.println(hostname);
#endif
    // Blink red LED to indicate error
    for (int i = 0; i < 5; i++) {
      digitalWrite(LED_RED, LOW);
      delay(100);
      digitalWrite(LED_RED, HIGH);
      delay(100);
    }
  } else {
#ifdef USE_SERIAL
    Serial.println("mDNS responder started");
    Serial.print("Hostname: ");
    Serial.print(hostname);
    Serial.println(".local");
#endif
    // Add service to MDNS-SD
    MDNS.addService("http", "tcp", 80);
    // Blink blue LED to indicate success
    for (int i = 0; i < 5; i++) {
      digitalWrite(LED_BLUE, LOW);
      delay(100);
      digitalWrite(LED_BLUE, HIGH);
      delay(100);
    }
  }

  // Shared queue and tasks
  task_common_init();
  motor_task_start();
  server_task_start();

}

void loop() {
  // Nothing to do here; work happens in tasks
  delay(1000);
}