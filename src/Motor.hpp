#ifndef _MOTOR_H
#define _MOTOR_H


class Motor
{
  public:
          Motor();
          
          void Pin_init();
          /*Measuring_speed*/
          void Encoder_init();
          static void EncoderCountRightA();
          static void EncoderCountLeftA();
          
          void (Motor::*MOVE[4])(int speed); 
          void Stop(int speed);
          void Forward(int speed);
          void Back(int speed);
          void Left(int speed);
          void Right(int speed);

  public:
          static unsigned long encoder_count_right_a;
          static unsigned long encoder_count_left_a;
          
  private:
          /*Motor pin*/
          #define OE   A6
          #define AIN1 7
          #define PWMA_LEFT 5
          #define BIN1 12
          #define PWMB_RIGHT 6
          #define STBY_PIN 8
          
          /*Encoder measuring speed  pin*/
          #define ENCODER_LEFT_A_PIN 2
          #define ENCODER_RIGHT_A_PIN 4

  };







#endif