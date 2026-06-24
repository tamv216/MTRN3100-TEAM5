#include "Encoder.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"

#define MOT1PWM 9 // PIN 9 is a PWM pin
#define MOT1DIR 10
mtrn3100::Motor motor(MOT1PWM,MOT1DIR);

#define EN_A 2 // PIN 2 is an interupt
#define EN_B 4
mtrn3100::Encoder encoder(EN_A, EN_B);

mtrn3100::PIDController new_controller(5000,0,1000);


void setup() {
  Serial.begin(9600);
  new_controller.zeroAndSetTarget(encoder.getRotationRadians(), PI);
  
  encoder.resetCount();
}

void loop() {
  float angle = encoder.getRotationRadians();

  float u = new_controller.compute(angle);

  motor.setPWM((int16_t)u);

  Serial.println(angle);

  delay(10);
}
