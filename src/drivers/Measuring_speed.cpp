#include <Arduino.h>
#include "Motor.hpp"
// #include "PinChangeInt.h"



void Motor::Encoder_init()
{
    pinMode(ENCODER_LEFT_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_A_PIN, INPUT_PULLUP);
    attachInterrupt(ENCODER_LEFT_A_PIN, EncoderCountLeftA, FALLING);
    attachInterrupt(ENCODER_RIGHT_A_PIN, EncoderCountRightA, FALLING);
}

volatile unsigned long Motor::encoder_count_right_a;
//Getting right wheel speed.
void Motor::EncoderCountRightA()
{
  Motor::encoder_count_right_a++;
}


volatile unsigned long Motor::encoder_count_left_a;
//Getting left wheel speed.
void Motor::EncoderCountLeftA()
{
  Motor::encoder_count_left_a++;
}