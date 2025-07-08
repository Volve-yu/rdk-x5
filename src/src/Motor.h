#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
public:
    Motor(int pinA, int pinB, int pwmChannelA, int pwmChannelB);
    void setup();
    void setSpeed(int speed); // speed: -MAX_PWM_VALUE to +MAX_PWM_VALUE

private:
    int _pinA, _pinB;
    int _pwmChannelA, _pwmChannelB;
};

#endif // MOTOR_H
