#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

class Encoder {
public:
    Encoder(int pinA, int pinB, const char* id);
    // Setup now associates the instance with static ISR functions
    void setup(void (*isrA_func)(void), void (*isrB_func)(void));
    long getCount();
    void reset();

    // These are public to be callable by the static ISRs
    void IRAM_ATTR handlePinA_ISR();
    void IRAM_ATTR handlePinB_ISR();

private:
    const int _pinA;
    const int _pinB;
    const char* _id; // For debugging
    
    volatile long _count;
    volatile int _lastPinAState;
    volatile int _lastPinBState;
};

#endif // ENCODER_H
