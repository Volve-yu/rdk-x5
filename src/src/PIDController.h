#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float outputMin, float outputMax);
    float compute(float setpoint, float measuredValue);
    void reset();
    void setTunings(float kp, float ki, float kd);
    void setOutputLimits(float min, float max);

private:
    float _kp, _ki, _kd;
    float _outputMin, _outputMax;
    float _integral;
    float _previousError;
    unsigned long _lastTime;
};

#endif // PIDCONTROLLER_H
