#include "Motor.h"
#include "Config.h" // For MAX_PWM_VALUE, MOTOR_PWM_FREQ, DEBUG_PRINTLN

Motor::Motor(int pinA, int pinB, int pwmChannelA, int pwmChannelB)
    : _pinA(pinA), _pinB(pinB), 
      _pwmChannelA(pwmChannelA), _pwmChannelB(pwmChannelB) {}

void Motor::setup() {
    ledcSetup(_pwmChannelA, MOTOR_PWM_FREQ, 8); // 8-bit resolution (0-255)
    ledcAttachPin(_pinA, _pwmChannelA);

    ledcSetup(_pwmChannelB, MOTOR_PWM_FREQ, 8);
    ledcAttachPin(_pinB, _pwmChannelB);

    setSpeed(0); // Initialize motor stopped
    DEBUG_PRINT("Motor init: Pins A/B="); DEBUG_PRINT(_pinA); DEBUG_PRINT("/"); DEBUG_PRINT(_pinB);
    DEBUG_PRINT(", Channels A/B="); DEBUG_PRINT(_pwmChannelA); DEBUG_PRINT("/"); DEBUG_PRINT(_pwmChannelB);
    DEBUG_PRINTLN("");
}

void Motor::setSpeed(int speed) {
    speed = constrain(speed, -MAX_PWM_VALUE, MAX_PWM_VALUE);

    if (speed == 0) { // Slow Decay Brake
        ledcWrite(_pwmChannelA, 0);
        ledcWrite(_pwmChannelB, 0);
    } else { // Locked Anti-Phase PWM
        int pwmValA = (speed + MAX_PWM_VALUE) / 2;
        pwmValA = constrain(pwmValA, 0, MAX_PWM_VALUE);
        int pwmValB = MAX_PWM_VALUE - pwmValA;

        ledcWrite(_pwmChannelA, pwmValA);
        ledcWrite(_pwmChannelB, pwmValB);
    }
}