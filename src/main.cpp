#include <Arduino.h>

#include <Encoder.hpp>
#include <Motor.hpp>
#include <PIDController.hpp>

#define WHEEL_D 32.0
#define WHEELBASE 96.0

#define MOT1PWM 11 // PIN 11 is a PWM pin
#define MOT1DIR 12

#define MOT2PWM 9 // PIN 9 is a PWM pin
#define MOT2DIR 10


#define EN_A1 2 // PIN 2 is an interupt
#define EN_B1 7

#define EN_A2 3 // PIN 3 is an interupt
#define EN_B2 8

typedef enum {
  Left,
  Right
} MotorSide;

mtrn3100::Motor motorL(MOT1PWM,MOT1DIR);
mtrn3100::Motor motorR(MOT2PWM,MOT2DIR);

mtrn3100::Encoder<EN_A1, EN_B1> encoderL;
mtrn3100::Encoder<EN_A2, EN_B2> encoderR;

mtrn3100::PIDController controllerL(5000,0,0);
mtrn3100::PIDController controllerR(5000,0,0);

uint64_t time;

/**
 * Converts robot translation to equivalent wheel rotation
 */
static inline float distToRadians(float dist) {
  return dist / (PI * WHEEL_D);
}

/**
 * Converts heading change to equivalent wheel rotation 
 */
static inline float headingToDistance(float angleRad) {
  return angleRad * WHEELBASE / 2;
}

static inline float positionPath(uint64_t time, MotorSide motor) {
  float target = 2000;

  float rot = headingToDistance(PI/4);
  rot *= (motor == Right) ? -1 : 1;

  int n = (((time - 2000) / 500) % 4) + 1;
  n *= (time >= 2000 && time < 6000) ? 1 : -1;
  return target + n * rot;
}

void setup() {
  Serial.begin(9600);
  encoderL.resetCount();
  encoderR.resetCount();
  delay(2000);
}

void loop() {
  // motorL.setPWM((int16_t)255);
  // float pL =  positionPath(time, Left);
  // float pR =  positionPath(time, Right);
  controllerL.zeroAndSetTarget(encoderL.getRotationRadians(), PI);//pL);
  // controllerR.zeroAndSetTarget(encoderR.getRotationRadians(), pR);
  float angleL = encoderL.getRotationRadians();
  // float angleR = encoderR.getRotationRadians();

  float uL = controllerL.compute(angleL);
  // float uR = controllerR.compute(angleR);
  Serial.println(uL);

  motorL.setPWM((int16_t)uL);
  // motorR.setPWM((int16_t)uR);

  Serial.println(angleL);
  // Serial.println(angleR);
  delay(1);
  // time = millis();
}
