#ifndef INTERRUPTENCODER_H
#define INTERRUPTENCODER_H

#include "Encoder.h"

class InterruptEncoder : public Encoder {
private:
    volatile int32_t position;
    uint8_t pinA, pinB; // GPIO pins for the encoder channels

    // Static function for the interrupt handler
    static void updatePosition(InterruptEncoder *instance) {
        if (digitalRead(instance->pinA) == digitalRead(instance->pinB)) {
            instance->position++;
        } else {
            instance->position--;
        }
    }

public:
    // Constructor
    InterruptEncoder(uint8_t encoderPinA, uint8_t encoderPinB) 
        : pinA(encoderPinA), pinB(encoderPinB), position(0) {}

    // Initialize encoder (attach interrupts)
    void init() override {
        pinMode(pinA, INPUT);
        pinMode(pinB, INPUT);

        // Attach interrupts and pass the instance pointer to the handler
        attachInterrupt(digitalPinToInterrupt(pinA), [this] { updatePosition(this); }, CHANGE);
        attachInterrupt(digitalPinToInterrupt(pinB), [this] { updatePosition(this); }, CHANGE);
    }

    // Get current encoder position
    int32_t getPosition() override {
        return position;  // Return the current position
    }

    // Reset encoder position
    void resetPosition() override {
        position = 0;
    }
    float getSpeed() {
        unsigned long currentTime = millis();
        unsigned long timeInterval = currentTime - lastTime; // Time elapsed in milliseconds

        if (timeInterval > 0) {
            int32_t positionChange = position - lastPosition;
            speed = (positionChange * 1000.0) / timeInterval; // Speed in pulses per second

            // Update last known position and time
            lastPosition = position;
            lastTime = currentTime;
        }

        return speed; // Return the calculated speed
    }
};

#endif // INTERRUPTENCODER_H
