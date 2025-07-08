#include <Arduino.h>
#include "Config.h"
#include "Motor.h"
#include "Encoder.h"
#include "Chassis.h"
#include "SerialCommander.h"

// Motor Objects
Motor motorFL(MOTOR_FL_PIN_A, MOTOR_FL_PIN_B, MOTOR_FL_PWM_CHANNEL_A, MOTOR_FL_PWM_CHANNEL_B);
Motor motorFR(MOTOR_FR_PIN_A, MOTOR_FR_PIN_B, MOTOR_FR_PWM_CHANNEL_A, MOTOR_FR_PWM_CHANNEL_B);
Motor motorRL(MOTOR_RL_PIN_A, MOTOR_RL_PIN_B, MOTOR_RL_PWM_CHANNEL_A, MOTOR_RL_PWM_CHANNEL_B);
Motor motorRR(MOTOR_RR_PIN_A, MOTOR_RR_PIN_B, MOTOR_RR_PWM_CHANNEL_A, MOTOR_RR_PWM_CHANNEL_B);

// Encoder Objects
Encoder encoderFL(ENCODER_FL_PIN_A, ENCODER_FL_PIN_B, "FL");
Encoder encoderFR(ENCODER_FR_PIN_A, ENCODER_FR_PIN_B, "FR");
Encoder encoderRL(ENCODER_RL_PIN_A, ENCODER_RL_PIN_B, "RL");
Encoder encoderRR(ENCODER_RR_PIN_A, ENCODER_RR_PIN_B, "RR");

// Static ISR wrapper functions
void IRAM_ATTR isr_fl_a_wrapper() { encoderFL.handlePinA_ISR(); }
void IRAM_ATTR isr_fl_b_wrapper() { encoderFL.handlePinB_ISR(); }
void IRAM_ATTR isr_fr_a_wrapper() { encoderFR.handlePinA_ISR(); }
void IRAM_ATTR isr_fr_b_wrapper() { encoderFR.handlePinB_ISR(); }
void IRAM_ATTR isr_rl_a_wrapper() { encoderRL.handlePinA_ISR(); }
void IRAM_ATTR isr_rl_b_wrapper() { encoderRL.handlePinB_ISR(); }
void IRAM_ATTR isr_rr_a_wrapper() { encoderRR.handlePinA_ISR(); }
void IRAM_ATTR isr_rr_b_wrapper() { encoderRR.handlePinB_ISR(); }

// Chassis Object
Chassis chassis(motorFL, motorFR, motorRL, motorRR,
                encoderFL, encoderFR, encoderRL, encoderRR);

// Serial Commander Object
SerialCommander serialCommander(chassis);

// FreeRTOS Task Handles
TaskHandle_t chassisTaskHandle = NULL;
TaskHandle_t commanderTaskHandle = NULL;

// Task for Chassis Control Loop (runs at defined frequency)
void chassisControlTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    // Chassis update frequency. 50Hz (20ms) is common.
    // Odometry and control loop benefits from consistent timing.
    const TickType_t xFrequency = pdMS_TO_TICKS(20); 

    for (;;) {
        chassis.update();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Task for Serial Command Handling
void serialCommanderTask(void *pvParameters) {
    for (;;) {
        serialCommander.handleCommands();
        // Check for commands fairly often, but don't hog CPU if idle
        vTaskDelay(pdMS_TO_TICKS(20)); 
    }
}

void setup() {
    // Serial for debugging and commands - ensure it's the one specified in Config.h
    SERIAL_COMMAND_PORT.begin(SERIAL_COMMAND_BAUD_RATE);
    unsigned long setupStartTime = millis();
    while (!SERIAL_COMMAND_PORT && (millis() - setupStartTime < 2000)) {
        ; // Wait up to 2 seconds for serial port to connect.
    }
    DEBUG_PRINTLN(""); // Start with a newline for cleaner output
    DEBUG_PRINTLN("--- System Booting ---");

    DEBUG_PRINTLN("Setting up Encoders with ISRs...");
    encoderFL.setup(isr_fl_a_wrapper, isr_fl_b_wrapper);
    encoderFR.setup(isr_fr_a_wrapper, isr_fr_b_wrapper);
    encoderRL.setup(isr_rl_a_wrapper, isr_rl_b_wrapper);
    encoderRR.setup(isr_rr_a_wrapper, isr_rr_b_wrapper);
    DEBUG_PRINTLN("Encoders setup complete.");

    // Motors are setup within chassis.setup()
    chassis.setup();
    serialCommander.setup(); // Initializes its serial port and prints ready message

    DEBUG_PRINTLN("Creating FreeRTOS tasks...");
    xTaskCreatePinnedToCore(
        chassisControlTask,     "ChassisCtrl",          CHASSIS_TASK_STACK_SIZE,
        NULL,                   CHASSIS_TASK_PRIORITY,  &chassisTaskHandle,     CHASSIS_TASK_CORE);

    xTaskCreatePinnedToCore(
        serialCommanderTask,    "SerialCmd",            COMMANDER_TASK_STACK_SIZE,
        NULL,                   COMMANDER_TASK_PRIORITY,&commanderTaskHandle,   COMMANDER_TASK_CORE);
    
    if(chassisTaskHandle == NULL || commanderTaskHandle == NULL){
        SERIAL_COMMAND_PORT.println("FATAL: Failed to create one or more tasks!");
        while(1); // Halt
    } else {
        DEBUG_PRINTLN("Chassis Control Task & Serial Commander Task started.");
    }

    DEBUG_PRINTLN("--- Setup complete. Robot ready. ---");
}

void loop() {
    // Main loop is unused, all work is in FreeRTOS tasks.
    // Yield to other tasks.
    vTaskDelay(portMAX_DELAY);
}
