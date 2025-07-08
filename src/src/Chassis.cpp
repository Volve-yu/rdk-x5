#include "Chassis.h"
#include <cmath> // For fabs, sin, cos, PI

Chassis::Chassis(Motor& flMotor, Motor& frMotor, Motor& rlMotor, Motor& rrMotor,
                 Encoder& flEncoder, Encoder& frEncoder, Encoder& rlEncoder, Encoder& rrEncoder)
    : _flMotor(flMotor), _frMotor(frMotor), _rlMotor(rlMotor), _rrMotor(rrMotor),
      _flEncoder(flEncoder), _frEncoder(frEncoder), _rlEncoder(rlEncoder), _rrEncoder(rrEncoder),
      _linearPid(PID_LINEAR_KP, PID_LINEAR_KI, PID_LINEAR_KD, PID_OUTPUT_MIN, PID_OUTPUT_MAX),
      _angularPid(PID_ANGULAR_KP, PID_ANGULAR_KI, PID_ANGULAR_KD, PID_OUTPUT_MIN, PID_OUTPUT_MAX),
      _currentState(MovementState::IDLE),
      _targetDistanceSegment(0.0f), _targetAngleSegmentRad(0.0f),
      _distanceMovedThisSegment(0.0f), _angleTurnedThisSegmentRad(0.0f),
      _prevFlCounts(0), _prevFrCounts(0), _prevRlCounts(0), _prevRrCounts(0),
      _lastStallCheckTimeMs(0), _distAtLastStallCheck(0.0f), _angleAtLastStallCheckRad(0.0f),
      _consecutiveStallReadings(0) {
    _currentPose = {0.0f, 0.0f, 0.0f};
}

void Chassis::setup() {
    _flMotor.setup(); _frMotor.setup(); _rlMotor.setup(); _rrMotor.setup();
    // Encoders are setup in main.cpp by passing ISRs

    // Reset encoders and get initial counts
    _flEncoder.reset(); _frEncoder.reset(); _rlEncoder.reset(); _rrEncoder.reset();
    _prevFlCounts = _flEncoder.getCount();
    _prevFrCounts = _frEncoder.getCount();
    _prevRlCounts = _rlEncoder.getCount();
    _prevRrCounts = _rrEncoder.getCount();

    _linearPid.reset(); _angularPid.reset();
    _currentPose = {0.0f, 0.0f, 0.0f};
    _currentState = MovementState::IDLE;
    _lastStallCheckTimeMs = millis();
    DEBUG_PRINTLN("Chassis initialized (Simplified Fixed Speed Mode).");
}

void Chassis::update() {
    updateOdometry();

    if (_currentState != MovementState::IDLE && _currentState != MovementState::STALLED) {
        executeCurrentMovement();
        if (millis() - _lastStallCheckTimeMs >= STALL_CHECK_INTERVAL_MS) {
            checkForStall();
            _lastStallCheckTimeMs = millis();
            // Update baseline for next stall check
            _distAtLastStallCheck = _distanceMovedThisSegment;
            _angleAtLastStallCheckRad = _angleTurnedThisSegmentRad;
        }
    }
}

float Chassis::normalizeAngleRad(float angleRad) {
    while (angleRad > PI) angleRad -= 2.0f * PI;
    while (angleRad < -PI) angleRad += 2.0f * PI;
    return angleRad;
}

void Chassis::updateOdometry() {
    long currentFlCounts = _flEncoder.getCount();
    long currentFrCounts = _frEncoder.getCount();
    long currentRlCounts = _rlEncoder.getCount();
    long currentRrCounts = _rrEncoder.getCount();

    float distFl = (float)(currentFlCounts - _prevFlCounts) * DISTANCE_PER_COUNT;
    float distFr = (float)(currentFrCounts - _prevFrCounts) * DISTANCE_PER_COUNT;
    float distRl = (float)(currentRlCounts - _prevRlCounts) * DISTANCE_PER_COUNT;
    float distRr = (float)(currentRrCounts - _prevRrCounts) * DISTANCE_PER_COUNT;

    _prevFlCounts = currentFlCounts;
    _prevFrCounts = currentFrCounts;
    _prevRlCounts = currentRlCounts;
    _prevRrCounts = currentRrCounts;

    float deltaDistance = (distFl + distFr + distRl + distRr) / 4.0f;
    float deltaTheta = ((distFr + distRr) - (distFl + distRl)) / (2.0f * WHEEL_BASE_WIDTH); // Simpler: (right_avg - left_avg) / base_width

    _currentPose.x += deltaDistance * cos(_currentPose.thetaRad + deltaTheta / 2.0f);
    _currentPose.y += deltaDistance * sin(_currentPose.thetaRad + deltaTheta / 2.0f);
    _currentPose.thetaRad = normalizeAngleRad(_currentPose.thetaRad + deltaTheta);

    if (_currentState == MovementState::MOVING_FORWARD || _currentState == MovementState::MOVING_BACKWARD) {
        _distanceMovedThisSegment += fabs(deltaDistance); // Accumulate absolute distance for segment
    } else if (_currentState == MovementState::TURNING_LEFT || _currentState == MovementState::TURNING_RIGHT) {
        _angleTurnedThisSegmentRad += fabs(deltaTheta); // Accumulate absolute angle for segment
    }
}

void Chassis::executeCurrentMovement() {
    bool targetReached = false;

    if (_currentState == MovementState::MOVING_FORWARD || _currentState == MovementState::MOVING_BACKWARD) {
        if (_distanceMovedThisSegment >= _targetDistanceSegment - LINEAR_TARGET_TOLERANCE_METERS) {
            targetReached = true;
        } else {
            int speed = (_currentState == MovementState::MOVING_FORWARD) ? DEFAULT_LINEAR_SPEED_PWM : -DEFAULT_LINEAR_SPEED_PWM;
            _flMotor.setSpeed(speed); _frMotor.setSpeed(speed);
            _rlMotor.setSpeed(speed); _rrMotor.setSpeed(speed);
        }
    } else if (_currentState == MovementState::TURNING_LEFT || _currentState == MovementState::TURNING_RIGHT) {
        if (_angleTurnedThisSegmentRad >= _targetAngleSegmentRad - (ANGULAR_TARGET_TOLERANCE_DEGREES * PI / 180.0f)) {
            targetReached = true;
        } else {
            int turnPwm = DEFAULT_TURN_SPEED_PWM;
            if (_currentState == MovementState::TURNING_LEFT) { // CCW
                _flMotor.setSpeed(-turnPwm); _frMotor.setSpeed(turnPwm);
                _rlMotor.setSpeed(-turnPwm); _rrMotor.setSpeed(turnPwm);
            } else { // TURNING_RIGHT (CW)
                _flMotor.setSpeed(turnPwm);  _frMotor.setSpeed(-turnPwm);
                _rlMotor.setSpeed(turnPwm);  _rrMotor.setSpeed(-turnPwm);
            }
        }
    }

    if (targetReached) {
        DEBUG_PRINTLN("Target reached.");
        stop();
    }
}

void Chassis::checkForStall() {
    if (_currentState == MovementState::IDLE || _currentState == MovementState::STALLED) {
        _consecutiveStallReadings = 0;
        return;
    }

    bool noMovement = false;
    if (_currentState == MovementState::MOVING_FORWARD || _currentState == MovementState::MOVING_BACKWARD) {
        if (fabs(_distanceMovedThisSegment - _distAtLastStallCheck) < MIN_DIST_CHANGE_NO_STALL) {
            noMovement = true;
        }
    } else if (_currentState == MovementState::TURNING_LEFT || _currentState == MovementState::TURNING_RIGHT) {
        if (fabs(_angleTurnedThisSegmentRad - _angleAtLastStallCheckRad) < (MIN_ANGLE_CHANGE_NO_STALL_DEG * PI / 180.0f)) {
            noMovement = true;
        }
    }

    if (noMovement) {
        _consecutiveStallReadings++;
        DEBUG_PRINT("Stall check: No significant movement. Count: "); DEBUG_PRINTLN(_consecutiveStallReadings);
        if (_consecutiveStallReadings >= STALL_TRIGGER_COUNT_THRESHOLD) {
            DEBUG_PRINTLN("STALL DETECTED!");
            stop(); // Stop motors
            _currentState = MovementState::STALLED; // Set STALLED state
        }
    } else {
        _consecutiveStallReadings = 0; // Reset if movement detected
    }
}

void Chassis::moveDistance(float distanceMeters) {
    if (isBusy() && _currentState != MovementState::STALLED) { // Allow new command if previously stalled
        DEBUG_PRINTLN("Chassis busy. Command ignored."); return;
    }
    DEBUG_PRINT("Command: Move distance "); DEBUG_PRINT(distanceMeters); DEBUG_PRINTLN("m");

    _targetDistanceSegment = fabs(distanceMeters);
    _distanceMovedThisSegment = 0.0f;
    _angleTurnedThisSegmentRad = 0.0f; // Reset angular progress too
    _linearPid.reset();
    _consecutiveStallReadings = 0;
    _distAtLastStallCheck = 0.0f;
    _angleAtLastStallCheckRad = 0.0f;
    _lastStallCheckTimeMs = millis();

    if (distanceMeters == 0.0f) { stop(); return; }
    _currentState = (distanceMeters > 0) ? MovementState::MOVING_FORWARD : MovementState::MOVING_BACKWARD;
}

void Chassis::turnAngle(float angleDegrees) {
    if (isBusy() && _currentState != MovementState::STALLED) {
        DEBUG_PRINTLN("Chassis busy. Command ignored."); return;
    }
    DEBUG_PRINT("Command: Turn angle "); DEBUG_PRINT(angleDegrees); DEBUG_PRINTLN("deg");

    _targetAngleSegmentRad = fabs(angleDegrees * PI / 180.0f);
    _angleTurnedThisSegmentRad = 0.0f;
    _distanceMovedThisSegment = 0.0f; // Reset linear progress too
    _angularPid.reset();
    _consecutiveStallReadings = 0;
    _distAtLastStallCheck = 0.0f;
    _angleAtLastStallCheckRad = 0.0f;
    _lastStallCheckTimeMs = millis();

    if (angleDegrees == 0.0f) { stop(); return; }
    _currentState = (angleDegrees > 0) ? MovementState::TURNING_LEFT : MovementState::TURNING_RIGHT;
}

void Chassis::stop() {
    _flMotor.setSpeed(0); _frMotor.setSpeed(0);
    _rlMotor.setSpeed(0); _rrMotor.setSpeed(0);
    // Don't change state from STALLED to IDLE on an automatic stop from stall detection.
    // Only an explicit stop command or new move command should clear STALLED.
    if (_currentState != MovementState::STALLED) {
         _currentState = MovementState::IDLE;
    }
    _consecutiveStallReadings = 0; // Reset on any stop
    DEBUG_PRINTLN("Chassis stopped.");
}

RobotPose Chassis::getPose() const { return _currentPose; }
MovementState Chassis::getState() const { return _currentState; }
bool Chassis::isBusy() const { return _currentState != MovementState::IDLE; }
