#pragma once

#include <Arduino.h>

#include "math.h"

namespace mtrn3100 {


// The motor class is a simple interface designed to assist in motor control
// You may choose to impliment additional functionality in the future such as dual motor or speed control 
class Motor {
public:
    Motor( uint8_t pwm, uint8_t in2) :  pwm_pin(pwm), dir_pin(in2) {
        // TODO: Set both pins as output
        pinMode(pwm_pin, OUTPUT);
        pinMode(dir_pin, OUTPUT);
    }


    // This function outputs the desired motor direction and the PWM signal. 
    // NOTE: a pwm signal > 255 could cause troubles as such ensure that pwm is clamped between 0 - 255.
    void setPWM(int16_t pwm) {
      if (pwm >= 0) {
        digitalWrite(dir_pin, HIGH);
      } else {
        digitalWrite(dir_pin, LOW);
        pwm = -pwm; // take magnitude
      }

      // Clamp PWM to 0-255
      pwm = constrain(pwm, 0, 170);

      analogWrite(pwm_pin, pwm);
      // TODO: Output digital direction pin based on if input signal is positive or negative.
      // TODO: Output PWM signal between 0 - 255.
    }

private:
    const uint8_t pwm_pin;
    const uint8_t dir_pin;
};

}  // namespace mtrn3100
