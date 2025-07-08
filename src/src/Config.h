#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// --- Pin Definitions ---
// Motor Pins (AT8236 or similar H-Bridge controlled by two PWM signals)
// Each motor uses two LEDC PWM channels.

// Motor Front Left (FL)
#define MOTOR_FL_PIN_A 12 // Connect to IN1 or equivalent of H-Bridge
#define MOTOR_FL_PIN_B 13 // Connect to IN2 or equivalent of H-Bridge
#define MOTOR_FL_PWM_CHANNEL_A 0 // LEDC PWM channel for PIN_A
#define MOTOR_FL_PWM_CHANNEL_B 1 // LEDC PWM channel for PIN_B

// Motor Front Right (FR)
#define MOTOR_FR_PIN_A 16 // Example pin, ensure it's usable
#define MOTOR_FR_PIN_B 17 // Example pin
#define MOTOR_FR_PWM_CHANNEL_A 2
#define MOTOR_FR_PWM_CHANNEL_B 3

// Motor Rear Left (RL)
#define MOTOR_RL_PIN_A 21
#define MOTOR_RL_PIN_B 22
#define MOTOR_RL_PWM_CHANNEL_A 4
#define MOTOR_RL_PWM_CHANNEL_B 5

// Motor Rear Right (RR)
#define MOTOR_RR_PIN_A 26
#define MOTOR_RR_PIN_B 27
#define MOTOR_RR_PWM_CHANNEL_A 6
#define MOTOR_RR_PWM_CHANNEL_B 7

// Encoder Pins (A/B phase for quadrature)
// Ensure these pins are interrupt-capable.
// GPIOs 34, 35, 36, 39 are input-only.
#define ENCODER_FL_PIN_A 14
#define ENCODER_FL_PIN_B 15
#define ENCODER_FR_PIN_A 18
#define ENCODER_FR_PIN_B 19
#define ENCODER_RL_PIN_A 23
#define ENCODER_RL_PIN_B 25
#define ENCODER_RR_PIN_A 33
#define ENCODER_RR_PIN_B 32 // Note: Default I2C0 SDA. Re-assign if I2C0 needed.

// Serial Communication
#define SERIAL_COMMAND_PORT Serial // Use the main USB serial for commands
#define SERIAL_COMMAND_BAUD_RATE 115200

// --- Robot Physical Parameters (CRITICAL - TUNE THESE CAREFULLY) ---
#define WHEEL_DIAMETER 0.065       // Wheel diameter in meters (e.g., 65mm)
#define WHEEL_RADIUS (WHEEL_DIAMETER / 2.0f)
#define ENCODER_PPR 13             // Pulses Per Revolution for *one phase* of your encoder (e.g., 13 for a 13-line encoder on motor shaft)
#define GEAR_RATIO 48.0f           // Gearbox reduction ratio (e.g., 1:48)

// For quadrature encoders (A/B phase), we typically count 4 edges per PPR cycle.
#define COUNTS_PER_MOTOR_REVOLUTION (ENCODER_PPR * 4) // Counts from encoder per one MOTOR shaft revolution (4x counting)
#define COUNTS_PER_WHEEL_REVOLUTION (COUNTS_PER_MOTOR_REVOLUTION * GEAR_RATIO)
#define DISTANCE_PER_COUNT ((PI * WHEEL_DIAMETER) / COUNTS_PER_WHEEL_REVOLUTION) // Meters per encoder count

#define WHEEL_BASE_WIDTH 0.15f     // Distance between the center of left and right wheels in meters (TUNABLE)

// --- Simplified Movement Control Parameters (TUNABLE) ---
#define DEFAULT_LINEAR_SPEED_PWM 150   // Fixed PWM value for forward/backward (0-255). START LOW AND INCREASE.
#define DEFAULT_TURN_SPEED_PWM 120     // Fixed PWM value for turning (0-255). START LOW AND INCREASE.

#define LINEAR_TARGET_TOLERANCE_METERS 0.02f // Stop if within this distance (e.g., 2cm). TUNABLE
#define ANGULAR_TARGET_TOLERANCE_DEGREES 3.0f // Stop if within this angle (e.g., 3 degrees). TUNABLE

// --- Stall Detection Parameters (TUNABLE) ---
#define STALL_CHECK_INTERVAL_MS 500       // How often to check for stall condition (milliseconds)
#define MIN_DIST_CHANGE_NO_STALL 0.005f   // Min distance change (m) in interval to be NOT stalled (linear).
#define MIN_ANGLE_CHANGE_NO_STALL_DEG 1.0f// Min angle change (deg) in interval to be NOT stalled (turn).
#define STALL_TRIGGER_COUNT_THRESHOLD 4   // How many consecutive stall checks must fail to trigger STALLED state.

// --- PID Parameters (Kept for structure, not primary for speed control in this simplified version) ---
#define PID_LINEAR_KP 0.5f
#define PID_LINEAR_KI 0.0f
#define PID_LINEAR_KD 0.0f
#define PID_ANGULAR_KP 0.5f
#define PID_ANGULAR_KI 0.0f
#define PID_ANGULAR_KD 0.0f
#define PID_OUTPUT_MIN (-MAX_PWM_VALUE)
#define PID_OUTPUT_MAX (MAX_PWM_VALUE)

// --- General Control Settings ---
#define MAX_PWM_VALUE 255          // Max PWM value for motor control
#define MOTOR_PWM_FREQ 5000        // PWM frequency in Hz for LEDC

// FreeRTOS Task settings
#define CHASSIS_TASK_PRIORITY 3
#define CHASSIS_TASK_STACK_SIZE 4096
#define CHASSIS_TASK_CORE 1

#define COMMANDER_TASK_PRIORITY 2
#define COMMANDER_TASK_STACK_SIZE 3072
#define COMMANDER_TASK_CORE 0

// Debug Printing Macros
// #define DEBUG_MODE // Uncomment to enable debug prints
#ifdef DEBUG_MODE
    #define DEBUG_PRINTLN(x) SERIAL_COMMAND_PORT.println(x)
    #define DEBUG_PRINT(x) SERIAL_COMMAND_PORT.print(x)
#else
    #define DEBUG_PRINTLN(x)
    #define DEBUG_PRINT(x)
#endif

#endif // CONFIG_H
