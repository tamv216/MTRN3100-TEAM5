#pragma once

#include <Arduino.h>

namespace mtrn3100 {


// The encoder class is a simple interface which counts and stores an encoders count.
// Encoder pin 1 is attached to the interupt on the arduino and used to trigger the count.
// Encoder pin 2 is attached to any digital pin and used to derrive rotation direction.
// The count is stored as a volatile variable due to the high frequency updates. 
template<uint8_t EN_PIN1, uint8_t EN_PIN2> 
class Encoder {
public:
    Encoder(): encoder1_pin{EN_PIN1}, encoder2_pin{EN_PIN2} {
        instance = this;  // Store the instance pointer
        pinMode(encoder1_pin, INPUT_PULLUP);
        pinMode(encoder2_pin, INPUT_PULLUP);

        // TODO: attach the interrupt on pin one such that it calls the readEncoderISR function on a rising edge
        attachInterrupt(digitalPinToInterrupt(encoder1_pin), readEncoderISR, RISING);
    }


    // Encoder function used to update the encoder
    void readEncoder() {
        noInterrupts();

        // NOTE: DO NOT PLACE SERIAL PRINT STATEMENTS IN THIS FUNCTION
        // NOTE: DO NOT CALL THIS FUNCTION MANUALLY IT WILL ONLY WORK IF CALLED BY THE INTERRUPT
        // TODO: Increase or Decrease the count by one based on the reading on encoder pin 2
        if (digitalRead(encoder2_pin) == HIGH) {
          count++;
        } else {
          count--;
        }
        
        interrupts();
    }

    long getCount() const {
      return count;
    }

    void resetCount() {
      count = 0;
    }

    // Helper function which to convert encouder count to radians
    float getRotationRadians() {
      if (counts_per_revolution == 0) return 0.0f;

      return (2.0f * PI * ((float)count / counts_per_revolution));
    }

    float getRotationDegrees() {
      if (counts_per_revolution == 0) return 0.0f;

      return (2.0f * PI * ((float)count / counts_per_revolution)) * 180 / PI;
    }
    
    const uint8_t encoder1_pin;
    const uint8_t encoder2_pin;
    volatile int8_t direction;
    float position = 0;
    uint16_t counts_per_revolution = 700; //TODO: Identify how many encoder counts are in one rotation
    volatile long count = 0;
    uint32_t prev_time;
    bool read = false;

private:
    static void readEncoderISR() {
        if (instance != nullptr) {
            instance->readEncoder();
        }
    }

    static Encoder<EN_PIN1, EN_PIN2>* instance;
};

template<uint8_t EN_PIN1, uint8_t EN_PIN2>
Encoder<EN_PIN1, EN_PIN2>* Encoder<EN_PIN1, EN_PIN2>::instance = nullptr;

}  // namespace mtrn3100