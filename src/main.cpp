#include <Arduino.h>

#include <Encoder.hpp>
#include <Motor.hpp>
#include <PIDController.hpp>

#define WHEEL_D 32.0
#define WHEELBASE 89.0

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

// mtrn3100::PIDController controllerL(70,8,10);
mtrn3100::PIDController controllerL(120,20,10);
mtrn3100::PIDController controllerR(120,20,10);

uint64_t time;
uint64_t startTime;

/**
 * Converts robot translation to equivalent wheel rotation
 */
static inline float distToRad(float dist) {
  return 2 * dist / (WHEEL_D);
}

/**
 * Converts heading change to equivalent wheel distance
 */
static inline float headingToDistance(float angleRad) {
  return angleRad * WHEELBASE / 2;
}

int n = 1;
int dir = 1;
uint64_t lastT = 2500;
int rotations = 1;

float positionPath(uint64_t time, MotorSide motor) {
  uint64_t t = time - startTime;
  if (t < 2500) return 200;
  if (rotations == 8) return 200;
  if (t - lastT >= 500) {
    rotations++;
    n += dir;
    if (n % 4 == 0) {
      dir *= -1;
    }
    lastT = t;
  }

  float turn = n * headingToDistance(PI/2);
  Serial.print(">n:");
  Serial.println(n);

  if (motor == Left) return 200 - turn;
  else return 200 + turn;
}

void setup() {
  Serial.begin(9600);
  encoderL.resetCount();
  encoderR.resetCount();
  controllerL.zeroAndSetTarget(encoderL.getRotationRadians(), 0);//pL);
  controllerR.zeroAndSetTarget(encoderR.getRotationRadians(), 0);//pL);
  delay(500);
  startTime = millis();
  time = startTime;
}

void loop() {
  time = millis();
  float pL = distToRad(positionPath(time, Left));
  float pR = distToRad(positionPath(time, Right));
  // Serial.print(">left:");
  // Serial.println(pL);
  // Serial.print(">right:");
  // Serial.println(pR);

  controllerL.changeTarget(pL);
  controllerR.changeTarget(pR);

  float angleL = encoderL.getRotationRadians();
  float angleR = encoderR.getRotationRadians();

  Serial.print(">left:");
  Serial.println(angleL);
  Serial.print(">right:");
  Serial.println(angleR);

  float uL = controllerL.compute(angleL);
  float uR = controllerR.compute(angleR);

  motorL.setPWM((int16_t)uL);
  motorR.setPWM((int16_t)uR);

  delay(1);
}
