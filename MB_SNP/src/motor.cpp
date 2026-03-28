#include "motor.h"

Actuator::Motor::Motor(uint8_t dir, uint8_t pwm) {
  this->dirPin = dir;
    this->pwmPin = pwm;
    this->channel = pwm; // use pin number as channel id

    pinMode(dirPin, OUTPUT);

    ledcSetup(channel, 5000, 8);     // 5kHz, 8-bit
    ledcAttachPin(pwmPin, channel);
    }

void Actuator::Motor::brake() {
      digitalWrite(dirPin, LOW);
      analogWrite(pwmPin, 0);
    }

void Actuator::Motor::set_PWM(int pwm) {
      pwm = constrain(pwm, -250, 250);
      (pwm <= 0) ? digitalWrite(dirPin, LOW) : digitalWrite(dirPin, HIGH);
      ledcWrite(channel, abs(pwm));
    }

void Actuator::Motor::set_Speed(float speed){
    int pwm = speed * 255;
    set_PWM(pwm);
}