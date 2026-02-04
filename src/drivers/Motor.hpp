#ifndef _MOTOR_H
#define _MOTOR_H

#include "../config.hpp"

class Motor
{
  public:
          Motor();
          
          void Pin_init();
          /*Measuring_speed*/
          void Encoder_init();
          static void EncoderCountRightA();
          static void EncoderCountLeftA();
          
          void (Motor::*MOVE[5])(int speed);
          void Stop(int speed);
          void Forward(int speed);
          void Back(int speed);
          void Left(int speed);
          void Right(int speed);
          // Signed per-wheel drive: negative = reverse
          void Drive(int leftPWM, int rightPWM);

  public:
          static unsigned long encoder_count_right_a;
          static unsigned long encoder_count_left_a;

  };







#endif