#ifndef CHASSIS_H
#define CHASSIS_H

#include "Config.h"
#include "Motor.h"
#include "Encoder.h"
#include "PIDController.h" // Included for structure

struct RobotPose {
    float x;        // meters
    float y;        // meters
    float thetaRad; // radians, CCW positive, 0 along X-axis
};

enum class MovementState {
    IDLE,
    MOVING_FORWARD,
    MOVING_BACKWARD,
    TURNING_LEFT,  // CCW
    TURNING_RIGHT, // CW
    STALLED
};

class Chassis {
public:
    Chassis(Motor& flMotor, Motor& frMotor, Motor& rlMotor, Motor& rrMotor,
            Encoder& flEncoder, Encoder& frEncoder, Encoder& rlEncoder, Encoder& rrEncoder);

    void setup();
    void update(); // Main periodic update for movement and odometry

    void moveDistance(float distanceMeters); // Positive forward, negative backward
    void turnAngle(float angleDegrees);    // Positive left/CCW, negative right/CW
    void stop();                           // Stops all motors and sets state to IDLE

    RobotPose getPose() const;
    MovementState getState() const;
    bool isBusy() const; // True if not IDLE

private:
    Motor& _flMotor; Motor& _frMotor; Motor& _rlMotor; Motor& _rrMotor;
    Encoder& _flEncoder; Encoder& _frEncoder; Encoder& _rlEncoder; Encoder& _rrEncoder;

    PIDController _linearPid; // Kept for potential future use
    PIDController _angularPid;

    RobotPose _currentPose;
    MovementState _currentState;

    // Target values for current movement command
    float _targetDistanceSegment;    // meters (always positive magnitude for the segment)
    float _targetAngleSegmentRad;  // radians (always positive magnitude for the segment)

    // Accumulated values during current movement command
    float _distanceMovedThisSegment; // meters
    float _angleTurnedThisSegmentRad;// radians

    long _prevFlCounts, _prevFrCounts, _prevRlCounts, _prevRrCounts;

    // Stall detection
    unsigned long _lastStallCheckTimeMs;
    float _distAtLastStallCheck;    // odometry distance (_distanceMovedThisSegment) at last check
    float _angleAtLastStallCheckRad;// odometry angle (_angleTurnedThisSegmentRad) at last check
    int _consecutiveStallReadings;

    void updateOdometry();
    void executeCurrentMovement();
    void checkForStall();
    float normalizeAngleRad(float angleRad);
};

#endif // CHASSIS_H
