#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

class Encoder {
public:
    virtual void init() = 0;         
    virtual int32_t getPosition() = 0; 
    virtual void resetPosition() = 0;  
};

#endif // ENCODER_H
