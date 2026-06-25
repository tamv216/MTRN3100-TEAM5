#include <Arduino.h>

#include <Encoder.hpp>
#include <Motor.hpp>
#include <PIDController.hpp>

#define WHEEL_D 32.0
#define WHEELBASE 85.0

#define MOT1PWM 11
#define MOT1DIR 12

#define MOT2PWM 9
#define MOT2DIR 10

#define EN_A1 2
#define EN_B1 7

#define EN_A2 3
#define EN_B2 8

typedef enum {
  Left,
  Right
} MotorSide;


mtrn3100::Motor motorL(MOT1PWM, MOT1DIR);
mtrn3100::Motor motorR(MOT2PWM, MOT2DIR);

mtrn3100::Encoder<EN_A1, EN_B1> encoderL;
mtrn3100::Encoder<EN_A2, EN_B2> encoderR;

// mtrn3100::PIDController controllerL(5000, 0, 0);
// mtrn3100::PIDController controllerR(5000, 0, 0);

uint64_t time_ms = 0;

// distance travelled --> radians needed by wheel
static inline float distToRadians(float dist) {
  return dist / (PI * WHEEL_D);
}

// wheel heading to distance travelled
static inline float headingToDistance(float angleRad) {
  return angleRad * WHEELBASE / 2;
}

const float TARGET_D = 200.0;

const int COUNTS_PER_REV = 700;

const float WHEEL_CIRC = PI * WHEEL_D;

const long TARGET_COUNTS =
    (TARGET_D / WHEEL_CIRC) * COUNTS_PER_REV;


void moveForward(float target_d) {
  encoderL.resetCount();
  encoderR.resetCount();

  long targetCounts =  distToRadians(TARGET_D)* COUNTS_PER_REV;

  motorL.setPWM(150);
  motorR.setPWM(150);

  while (true) {
    long avg = (encoderL.getCount() + encoderR.getCount()) / 2;

    if (avg >= targetCounts) break;
  }

  motorL.setPWM(0);
  motorR.setPWM(0);
}

const int R_TURN = 1;
const int L_TURN = 0;
void turnDegrees(int dir, float deg) {

  encoderL.resetCount();
  encoderR.resetCount();

  float angleRad = deg * PI / 180.0;

  float arc = angleRad * (WHEELBASE / 2.0);

  long targetCounts = (arc / WHEEL_CIRC) * COUNTS_PER_REV;

  if (dir == R_TURN) {
    motorL.setPWM(150);
    motorR.setPWM(-150);
  } else if (dir == L_TURN) {
    motorL.setPWM(-150);
    motorR.setPWM(150);
  }
  
  while (true) {

    long left = abs(encoderL.getCount());
    long right = abs(encoderR.getCount());

    long avg = (left + right) / 2;

    if (avg >= targetCounts) break;
  }

  motorL.setPWM(0);
  motorR.setPWM(0);
}


void setup() {
  encoderL.resetCount();
  encoderR.resetCount();

  delay(1000);
}

void loop() {
  moveForward(200);
  delay(500);

  for (int i = 0; i < 4; i++) {
    turnDegrees(L_TURN, 90);
    delay(500);
  }

  for (int i = 0; i < 4; i++) {
    turnDegrees(R_TURN, 90);
    delay(500);
  }

  while (1); // stop forever
}