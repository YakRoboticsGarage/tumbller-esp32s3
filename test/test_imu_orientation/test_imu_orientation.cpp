#include <Arduino.h>
#include <Wire.h>

#include "../../src/drivers/MPU6050.h"

static MPU6050 mpu;
static bool testPassed = false;

/**
 * IMU Orientation Test
 *
 * Reads IMU values for 3 seconds and prints averages.
 * Then continuously prints live values.
 *
 * Expected values when robot is UPRIGHT and STILL:
 * - The axis pointing UP will read ~+16384 (positive 1g)
 * - The axis pointing DOWN will read ~-16384 (negative 1g)
 * - Horizontal axes will read ~0
 * - All gyro values should be ~0 when still
 */

void printLine(const char* msg) {
  Serial.print(msg);
  Serial.print("\r\n");
  Serial.flush();
}

void setup()
{
  Serial.begin(115200);
  delay(2000);  // Wait for serial connection

  printLine("");
  printLine("========================================");
  printLine("       I2C BUS SCAN");
  printLine("========================================");

  Wire.begin();
  Wire.setClock(100000);  // 100kHz - more reliable with MPU6050 on ESP32

  bool found = false;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  Found device at 0x%02X", addr);
      if (addr == 0x68 || addr == 0x69) Serial.print(" (MPU6050)");
      if (addr == 0x44 || addr == 0x45) Serial.print(" (SHT3x)");
      Serial.print("\r\n");
      Serial.flush();
      found = true;
    }
  }
  if (!found) {
    printLine("  No I2C devices found!");
    printLine("  Check wiring: SDA=A4, SCL=A5");
    while (1) delay(1000);  // Halt
  }

  mpu.initialize();
  if (!mpu.testConnection()) {
    printLine("ERROR: MPU6050 not responding!");
    while (1) delay(1000);  // Halt
  }

  printLine("");
  printLine("========================================");
  printLine("       IMU ORIENTATION TEST");
  printLine("========================================");
  printLine("Hold robot STILL in desired position...");
  printLine("Reading for 3 seconds...");
  printLine("");

  const int numSamples = 300;  // 3 seconds at 10ms
  long sumAx = 0, sumAy = 0, sumAz = 0;
  long sumGx = 0, sumGy = 0, sumGz = 0;
  int16_t ax, ay, az, gx, gy, gz;

  for (int i = 0; i < numSamples; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sumAx += ax;
    sumAy += ay;
    sumAz += az;
    sumGx += gx;
    sumGy += gy;
    sumGz += gz;

    // Print progress every 20%
    if (i % 60 == 0) {
      Serial.printf("  %d%%\r\n", (i * 100) / numSamples);
      Serial.flush();
    }
    delay(10);
  }
  printLine("  100%");

  // Calculate averages
  float avgAx = (float)sumAx / numSamples;
  float avgAy = (float)sumAy / numSamples;
  float avgAz = (float)sumAz / numSamples;
  float avgGx = (float)sumGx / numSamples;
  float avgGy = (float)sumGy / numSamples;
  float avgGz = (float)sumGz / numSamples;

  printLine("");
  printLine("========================================");
  printLine("       AVERAGE RAW VALUES");
  printLine("========================================");
  printLine("Accelerometer (+/-16384 = +/-1g):");

  Serial.printf("  ax = %8.1f", avgAx);
  if (abs(avgAx) > 14000) Serial.printf("  <-- %s", avgAx > 0 ? "X+ UP" : "X+ DOWN");
  Serial.print("\r\n");

  Serial.printf("  ay = %8.1f", avgAy);
  if (abs(avgAy) > 14000) Serial.printf("  <-- %s", avgAy > 0 ? "Y+ UP" : "Y+ DOWN");
  Serial.print("\r\n");

  Serial.printf("  az = %8.1f", avgAz);
  if (abs(avgAz) > 14000) Serial.printf("  <-- %s", avgAz > 0 ? "Z+ UP" : "Z+ DOWN");
  Serial.print("\r\n");
  Serial.flush();

  printLine("");
  printLine("Gyroscope (should be ~0 when still):");
  Serial.printf("  gx = %8.1f\r\n", avgGx);
  Serial.printf("  gy = %8.1f\r\n", avgGy);
  Serial.printf("  gz = %8.1f\r\n", avgGz);
  Serial.flush();

  // Calculate tilt angles
  float angleXZ = atan2(avgAx, avgAz) * 57.29578f;
  float angleYZ = atan2(avgAy, avgAz) * 57.29578f;

  printLine("");
  printLine("========================================");
  printLine("       CALCULATED ANGLES");
  printLine("========================================");
  Serial.printf("Tilt angle (X-Z plane): %6.1f degrees\r\n", angleXZ);
  Serial.printf("Tilt angle (Y-Z plane): %6.1f degrees\r\n", angleYZ);
  Serial.flush();

  printLine("");
  printLine("========================================");
  printLine("       ORIENTATION SUMMARY");
  printLine("========================================");

  // Determine primary UP axis
  if (abs(avgAz) > abs(avgAx) && abs(avgAz) > abs(avgAy)) {
    Serial.printf("Primary UP axis: Z (%s)\r\n", avgAz > 0 ? "Z+ up" : "Z- up");
  } else if (abs(avgAx) > abs(avgAy)) {
    Serial.printf("Primary UP axis: X (%s)\r\n", avgAx > 0 ? "X+ up" : "X- up");
  } else {
    Serial.printf("Primary UP axis: Y (%s)\r\n", avgAy > 0 ? "Y+ up" : "Y- up");
  }

  printLine("");
  printLine("For self-balancing with X-forward, Z-up:");
  Serial.printf("  Tilt angle: atan2(ax, az) = %.1f deg\r\n", angleXZ);
  Serial.printf("  Gyro bias for gy: %.1f\r\n", avgGy);
  printLine("========================================");

  printLine("");
  printLine("Now showing LIVE values (tilt robot to test):");
  printLine("");

  testPassed = true;
}

void loop()
{
  if (!testPassed) return;

  static unsigned long lastPrint = 0;
  static int zeroCount = 0;

  if (millis() - lastPrint > 200) {
    lastPrint = millis();
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Detect I2C failure (all zeros)
    if (ax == 0 && ay == 0 && az == 0) {
      zeroCount++;
      if (zeroCount >= 3) {
        printLine("I2C failure detected, recovering...");
        Wire.end();
        delay(100);
        Wire.begin();
        Wire.setClock(100000);  // Try slower clock (100kHz)
        mpu.initialize();
        zeroCount = 0;
        return;
      }
    } else {
      zeroCount = 0;
    }

    float angle = atan2((float)ax, (float)az) * 57.29578f;
    Serial.printf("ax=%6d ay=%6d az=%6d | tilt=%.1f deg\r\n", ax, ay, az, angle);
    Serial.flush();
  }
}
