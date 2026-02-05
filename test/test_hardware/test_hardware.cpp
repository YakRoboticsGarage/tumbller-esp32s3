#include <Arduino.h>
#include <unity.h>
#include <Wire.h>

#include "../src/config.hpp"
#include "../src/drivers/Motor.hpp"
#include "../src/drivers/MPU6050.h"

static Motor motor;
static MPU6050 mpu;

void setUp() {}
void tearDown() {}


void test_motors()
{
  motor.Pin_init();
  motor.Encoder_init();
  delay(200);

  const int speed = 80;

  // Forward
  noInterrupts(); Motor::encoder_count_left_a = Motor::encoder_count_right_a = 0; interrupts();
  motor.Forward(speed); delay(500); motor.Stop(0);
  TEST_ASSERT_TRUE(Motor::encoder_count_left_a > 0 || Motor::encoder_count_right_a > 0);

  // Back
  noInterrupts(); Motor::encoder_count_left_a = Motor::encoder_count_right_a = 0; interrupts();
  motor.Back(speed); delay(500); motor.Stop(0);
  TEST_ASSERT_TRUE(Motor::encoder_count_left_a > 0 || Motor::encoder_count_right_a > 0);
}

void test_imu()
{
  Wire.begin();
  mpu.initialize();
  TEST_ASSERT_TRUE_MESSAGE(mpu.testConnection(), "MPU6050 WHO_AM_I failed");
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  // Basic sanity: at least one accel axis should be non-zero (gravity)
  TEST_ASSERT_TRUE(ax != 0 || ay != 0 || az != 0);
}

void setup()
{
  delay(2000); // allow serial to connect
  UNITY_BEGIN();
  RUN_TEST(test_motors);
  RUN_TEST(test_imu);
  UNITY_END();
}

void loop() {}
