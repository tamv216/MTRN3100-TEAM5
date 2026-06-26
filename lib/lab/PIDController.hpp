#pragma once

#include <math.h>

namespace mtrn3100 {

class PIDController {
public:
    PIDController(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd) {}

    // Compute the output signal required from the current/actual value.
    float compute(float input) {
      
        curr_time = micros();
        dt = (curr_time - prev_time <= 0) ? 1e-6 : static_cast<float>(curr_time - prev_time) / 1e6;
        prev_time = curr_time;

        error = setpoint - (input - zero_ref);

        // TODO: IMPLIMENT PID CONTROLLER
        integral += error * dt;
        integral = constrain(integral, -170/ki, 170/ki);
        derivative = (error - prev_error) / dt;
        output = (kp * error) + (ki * integral) + (kd * derivative);
        prev_error = error;

        Serial.print(">e:");
        Serial.println(kp * error);
        Serial.print(">i:");
        Serial.println(ki * integral);
        Serial.print(">d:");
        Serial.println(kd * derivative);
        Serial.print(">u:");
        Serial.println(output);

        return output;
    }

    // Function used to return the last calculated error. 
    // The error is the difference between the desired position and current position. 
    void tune(float p, float i, float d) {
        kp = p;
        ki = i;
        kd = d;
    }

    float getError() {
      return error;
    }

    // This must be called before trying to achieve a setpoint.
    // The first argument becomes the new zero reference point.
    // Target is the setpoint value.
    void zeroAndSetTarget(float zero, float target) {
        prev_time = micros();
        zero_ref = zero;
        setpoint = target;
    }

    void changeTarget(float target) {
        setpoint = target;
    }

public:
    uint32_t prev_time, curr_time = micros();
    float dt;

private:
    float kp, ki, kd;
    float error, derivative, integral, output;
    float prev_error = 0;
    float setpoint = 0;
    float zero_ref = 0;

    
};

}  // namespace mtrn3100


