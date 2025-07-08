#include "PIDController.h"
#include <Arduino.h> // For millis() and constrain()

PIDController::PIDController(float kp, float ki, float kd, float outputMin, float outputMax)
    : _kp(kp), _ki(ki), _kd(kd), _outputMin(outputMin), _outputMax(outputMax),
      _integral(0), _previousError(0), _lastTime(millis()) {}

float PIDController::compute(float setpoint, float measuredValue) {
    unsigned long currentTime = millis();
    float dt = (float)(currentTime - _lastTime) / 1000.0f; // Time difference in seconds
    _lastTime = currentTime;

    if (dt == 0) return _previousError * _kd; // Avoid division by zero or if called too quickly

    float error = setpoint - measuredValue;
    _integral += error * dt;
    _integral = constrain(_integral, _outputMin / (_ki + 1e-9), _outputMax / (_ki + 1e-9)); // Anti-windup

    float derivative = (error - _previousError) / dt;
    _previousError = error;

    float output = _kp * error + _ki * _integral + _kd * derivative;
    return constrain(output, _outputMin, _outputMax);
}

void PIDController::reset() {
    _integral = 0;
    _previousError = 0;
    _lastTime = millis();
}

void PIDController::setTunings(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void PIDController::setOutputLimits(float min, float max) {
    _outputMin = min;
    _outputMax = max;
}
