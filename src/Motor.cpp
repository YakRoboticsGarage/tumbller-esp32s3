#include <Arduino.h>
#include "Motor.hpp"

void Motor::Pin_init()
{
  pinMode(AIN1, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(PWMA_LEFT, OUTPUT);
  pinMode(PWMB_RIGHT, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, HIGH);
  pinMode(OE, OUTPUT);
  digitalWrite(OE, HIGH);
}

Motor::Motor()
{
  MOVE[0] = &Motor::Forward;
  MOVE[1] = &Motor::Back;
  MOVE[2] = &Motor::Left;
  MOVE[3] = &Motor::Right;
  MOVE[4] = &Motor::Stop;

}

void Motor::Stop(int speed)
{
  (void)speed;
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, LOW);
  analogWrite(PWMA_LEFT, 0);
  analogWrite(PWMB_RIGHT, 0);
}

void Motor::Forward(int speed)
{
  digitalWrite(AIN1, 0);
  digitalWrite(BIN1, 0);
  analogWrite(PWMA_LEFT, speed);
  analogWrite(PWMB_RIGHT, speed);
}

void Motor::Back(int speed)
{
  digitalWrite(AIN1, 1);
  digitalWrite(BIN1, 1);
  analogWrite(PWMA_LEFT, speed);
  analogWrite(PWMB_RIGHT, speed);
}


void Motor::Left(int speed)
{
  digitalWrite(AIN1, 1);
  digitalWrite(BIN1, 1);
  analogWrite(PWMA_LEFT, speed);
  analogWrite(PWMB_RIGHT, 0);
}

void Motor::Right(int speed)
{
  digitalWrite(AIN1, 1);
  digitalWrite(BIN1, 1);
  analogWrite(PWMA_LEFT,0);
  analogWrite(PWMB_RIGHT,speed);
}