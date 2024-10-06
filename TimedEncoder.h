#ifndef TIMEDENCODER_H
#define TIMEDENCODER_H

#include <Arduino.h>
#include "Encoder.h"

class TimedEncoder : public Encoder {
private:
    volatile int32_t position;  // Current position of the encoder
    int32_t lastPosition;       // Last recorded position for speed calculation
    unsigned long lastTime;     // Last time speed was calculated
    float speed;                // Speed of the encoder (pulses per second)
    uint8_t pinA, pinB;         // Encoder pins
    uint32_t timerInterval;     // Timer interval in milliseconds for speed calculation

public:
    TimedEncoder(uint8_t encoderPinA, uint8_t encoderPinB, uint32_t interval = 100) 
        : pinA(encoderPinA), pinB(encoderPinB), position(0), lastPosition(0), lastTime(0), speed(0.0f), timerInterval(interval) {}

    // Initialize encoder (set up pins)
    void init() override {
        pinMode(pinA, INPUT);
        pinMode(pinB, INPUT);
        lastTime = millis();  // Initialize the timer
    }

    void updatePosition() {
        if (digitalRead(pinA) == digitalRead(pinB)) {
            position++;
        } else {
            position--;
        }
    }

    // Get current encoder position
    int32_t getPosition() override {
        return position;
    }

    // Reset encoder position
    void resetPosition() override {
        position = 0;
        lastPosition = 0;
    }

    // Calculate and return the speed of the encoder (pulses per second)
    float getSpeed() {
        unsigned long currentTime = millis();
        unsigned long timeInterval = currentTime - lastTime;
        if (timeInterval >= timerInterval) {
            int32_t positionChange = position - lastPosition;
            speed = (positionChange * 1000.0) / timeInterval; // Speed in pulses per second
            lastPosition = position;
            lastTime = currentTime;
        }

        return speed; // Return the calculated speed
    }
};

#endif 
