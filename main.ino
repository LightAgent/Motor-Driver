#include "TimedEncoder.h"
#include "InterruptEncoder.h"

// Example using TIM2 for TimedEncoder
TimedEncoder timedEncoder(TIM2);

// Example using pins PA0 and PA1 for InterruptEncoder
InterruptEncoder interruptEncoder(PA0, PA1);

void setup() {
  Serial.begin(115200);

  // Initialize both encoders
  timedEncoder.init();
  interruptEncoder.init();
}

void loop() {
  // Get the position from the timer-based encoder
  int32_t timedPosition = timedEncoder.getPosition();
  Serial.print("Timed Encoder Position: ");
  Serial.println(timedPosition);

  // Get the position from the interrupt-based encoder
  int32_t interruptPosition = interruptEncoder.getPosition();
  Serial.print("Interrupt Encoder Position: ");
  Serial.println(interruptPosition);

  delay(100);
}
