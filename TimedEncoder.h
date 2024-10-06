#ifndef TIMEDENCODER_H
#define TIMEDENCODER_H

#include "Encoder.h"

class TimedEncoder : public Encoder {
private:
    TIM_TypeDef *timer;
    int32_t position;

public:
    // Constructor
    TimedEncoder(TIM_TypeDef *timerInstance) : timer(timerInstance), position(0) {}

    // Initialize the encoder with timer setup
    void init() override {
        // Enable timer clock (assuming TIM2)
        if (timer == TIM2) {
            RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
        } else if (timer == TIM3) {
            RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
        }
        
        // Configure timer in encoder mode
        timer->CR1 = 0;
        timer->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1; // Encoder mode 3
        timer->CCMR1 = 0x01 | (0x01 << 8);             // Input capture on both channels
        timer->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;   // Enable capture for both channels
        timer->ARR = 0xFFFF;                           // Auto-reload max value
        timer->CNT = 0;                                // Reset the counter
        timer->CR1 |= TIM_CR1_CEN;                     // Enable timer
    }

    // Get current encoder position
    int32_t getPosition() override {
        int32_t currentCount = (int16_t)(timer->CNT);
        position = currentCount;
        return position;
    }

    // Reset encoder position
    void resetPosition() override {
        timer->CNT = 0;
        position = 0;
    }
};

#endif // TIMEDENCODER_H
