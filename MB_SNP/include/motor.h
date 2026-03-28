#pragma once
#include <Arduino.h>
#include "driver/ledc.h"
namespace Actuator
{
  class Motor {
  private:
    uint8_t dirPin, pwmPin, channel;
    bool direction;
  public:
    Motor(uint8_t dir, uint8_t pwm);
    void brake();
    void set_PWM(int pwm);
    void set_Speed(float speed);
  };
};