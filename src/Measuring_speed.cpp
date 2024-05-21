#include <Arduino.h>
#include "Motor.hpp"
// #include "PinChangeInt.h"



void Motor::Encoder_init()
{
//   attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A_PIN), EncoderCountLeftA, CHANGE);
//   attachPinChangeInterrupt(ENCODER_RIGHT_A_PIN, EncoderCountRightA, CHANGE);
    pinMode(ENCODER_LEFT_A_PIN, INPUT_PULLDOWN);
    pinMode(ENCODER_RIGHT_A_PIN, INPUT_PULLDOWN);
    attachInterrupt(ENCODER_LEFT_A_PIN, EncoderCountLeftA, RISING);
    attachInterrupt(ENCODER_RIGHT_A_PIN, EncoderCountRightA, RISING);
}

unsigned long Motor::encoder_count_right_a;
//Getting right wheel speed.
void Motor::EncoderCountRightA()
{
  Motor::encoder_count_right_a++;
}


unsigned long Motor::encoder_count_left_a;
//Getting left wheel speed.
void Motor::EncoderCountLeftA()
{
  Motor::encoder_count_left_a++;
}