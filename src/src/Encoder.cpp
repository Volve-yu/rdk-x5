#include "Encoder.h"
#include "Config.h" // For DEBUG_PRINTLN

Encoder::Encoder(int pinA, int pinB, const char* id)
    : _pinA(pinA), _pinB(pinB), _id(id), _count(0), 
      _lastPinAState(LOW), _lastPinBState(LOW) {}

void Encoder::setup(void (*isrA_func)(void), void (*isrB_func)(void)) {
    pinMode(_pinA, INPUT_PULLUP);
    pinMode(_pinB, INPUT_PULLUP);

    // Initialize last states after setting pinMode
    _lastPinAState = digitalRead(_pinA);
    _lastPinBState = digitalRead(_pinB);
    
    _count = 0; // Ensure count is reset at setup

    // Attach interrupts to the provided static ISR functions
    attachInterrupt(digitalPinToInterrupt(_pinA), isrA_func, CHANGE);
    attachInterrupt(digitalPinToInterrupt(_pinB), isrB_func, CHANGE);
    
    DEBUG_PRINT("Encoder '"); DEBUG_PRINT(_id);
    DEBUG_PRINTLN("' setup on pins A/B: " + String(_pinA) + "/" + String(_pinB));
}

long Encoder::getCount() {
    // Reading volatile variable, ensure atomic or disable interrupts if needed for multi-byte,
    // but for 'long' on ESP32 it's often atomic.
    return _count;
}

void Encoder::reset() {
    // Disabling interrupts briefly for atomicity if concerned
    // noInterrupts();
    _count = 0;
    // interrupts();
    DEBUG_PRINT("Encoder '"); DEBUG_PRINT(_id); DEBUG_PRINTLN("' reset.");
}

// ISR handler for Pin A state change (4x Quadrature)
void IRAM_ATTR Encoder::handlePinA_ISR() {
    int currentPinAState = digitalRead(_pinA);
    int currentPinBState = digitalRead(_pinB); // Read B state close to A's change

    if (currentPinAState != _lastPinAState) { // Confirm change on A
        if (currentPinAState == HIGH) { // Rising edge on A
            if (currentPinBState == LOW) {
                _count++; // Forward
            } else {
                _count--; // Backward
            }
        } else { // Falling edge on A
            if (currentPinBState == HIGH) {
                _count++; // Forward
            } else {
                _count--; // Backward
            }
        }
        _lastPinAState = currentPinAState;
    }
}

// ISR handler for Pin B state change (4x Quadrature)
void IRAM_ATTR Encoder::handlePinB_ISR() {
    int currentPinBState = digitalRead(_pinB);
    int currentPinAState = digitalRead(_pinA); // Read A state close to B's change

    if (currentPinBState != _lastPinBState) { // Confirm change on B
        if (currentPinBState == HIGH) { // Rising edge on B
            if (currentPinAState == HIGH) {
                _count++; // Forward
            } else {
                _count--; // Backward
            }
        } else { // Falling edge on B
            if (currentPinAState == LOW) {
                _count++; // Forward
            } else {
                _count--; // Backward
            }
        }
        _lastPinBState = currentPinBState;
    }
}
