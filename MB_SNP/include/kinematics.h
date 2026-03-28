#pragma once

#include <Arduino.h>
#include "motor.h"

typedef struct __Velocity
{
    float Vx, Vy, Vz;
} Velocity;


class BaseDrive {
  public:
      enum DriveType {
          TRUCK_TIRE,      // Differential
          OMNI_WHEEL,      // 3 or 4 omni
          MECANUM_WHEEL
      };

      BaseDrive(DriveType type, float chassis_L = 0.2, float chassis_W = 0);

      float chassis_length , chassis_breadth;
      void attachMotors(Actuator::Motor* m1,
                Actuator::Motor* m2,
                Actuator::Motor* m3 = nullptr,
                Actuator::Motor* m4 = nullptr);
      void setVelocity(float vx, float vy, float vw);
      void computeIK();
      void applyPWM();
      void applyPWM(int pwm1,int pwm2,int pwm3 = 0,int pwm4 = 0);
      void update(float vx, float vy, float vw);

  private:

      DriveType driveType;
      Actuator::Motor* motor1 = nullptr;
      Actuator::Motor* motor2 = nullptr;
      Actuator::Motor* motor3 = nullptr;
      Actuator::Motor* motor4 = nullptr;
      float vx, vy, vw;
      float wheel1, wheel2, wheel3, wheel4;
      void differentialIK();
      void mecanumIK();
      
  };

