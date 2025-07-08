#include "SerialCommander.h"
#include "Config.h" // For SERIAL_COMMAND_PORT and DEBUG prints

SerialCommander::SerialCommander(Chassis& chassis) : _chassis(chassis) {}

void SerialCommander::setup() {
    SERIAL_COMMAND_PORT.begin(SERIAL_COMMAND_BAUD_RATE);
    _commandBuffer.reserve(64); // Pre-allocate buffer
    DEBUG_PRINTLN("Serial Commander initialized.");
    SERIAL_COMMAND_PORT.println("Robot Command Interface Ready. Commands: F <m>, B <m>, L <deg>, R <deg>, S (stop)");
}

void SerialCommander::handleCommands() {
    while (SERIAL_COMMAND_PORT.available() > 0) {
        char c = SERIAL_COMMAND_PORT.read();
        #ifdef DEBUG_MODE // More verbose char-by-char debug if enabled
        // SERIAL_COMMAND_PORT.print("Char: '"); SERIAL_COMMAND_PORT.print(c); SERIAL_COMMAND_PORT.println("'");
        #endif

        if (c == '\n' || c == '\r') { // Command delimiter
            if (_commandBuffer.length() > 0) {
                DEBUG_PRINT("Received command string for parsing: ["); DEBUG_PRINT(_commandBuffer); DEBUG_PRINTLN("]");
                parseCommand(_commandBuffer);
                _commandBuffer = ""; // Clear buffer for next command
            }
        } else if (isprint(c)) { // Add printable characters to buffer
            _commandBuffer += c;
        }
        // Ignore non-printable characters that are not delimiters
    }
}

void SerialCommander::parseCommand(String cmd) {
    cmd.trim(); // Remove leading/trailing whitespace
    if (cmd.length() == 0) return;

    char commandType = toupper(cmd.charAt(0));
    String valueStr = "";
    if (cmd.length() > 1) {
        valueStr = cmd.substring(1);
        valueStr.trim();
    }
    float value = valueStr.toFloat(); // Returns 0.0 if conversion fails or empty

    DEBUG_PRINT("Cmd Type: "); DEBUG_PRINT(commandType);
    DEBUG_PRINT(", ValStr: ["); DEBUG_PRINT(valueStr);
    DEBUG_PRINT("], ValFloat: "); DEBUG_PRINTLN(value);

    // If chassis is stalled, only allow 'S' to clear it or a new move command to override
    if (_chassis.getState() == MovementState::STALLED && commandType != 'S') {
        DEBUG_PRINTLN("Chassis STALLED. Send 'S' to try and clear or new move cmd to override.");
        // Allow new move commands to override STALLED state
    }


    switch (commandType) {
        case 'F': // Forward
            _chassis.moveDistance(value);
            SERIAL_COMMAND_PORT.print("Executing: Forward "); SERIAL_COMMAND_PORT.print(value); SERIAL_COMMAND_PORT.println("m");
            break;
        case 'B': // Backward
            _chassis.moveDistance(-value); // Negative for backward
            SERIAL_COMMAND_PORT.print("Executing: Backward "); SERIAL_COMMAND_PORT.print(value); SERIAL_COMMAND_PORT.println("m");
            break;
        case 'L': // Turn Left
            _chassis.turnAngle(value); // Positive for left
            SERIAL_COMMAND_PORT.print("Executing: Turn Left "); SERIAL_COMMAND_PORT.print(value); SERIAL_COMMAND_PORT.println("deg");
            break;
        case 'R': // Turn Right
            _chassis.turnAngle(-value); // Negative for right
            SERIAL_COMMAND_PORT.print("Executing: Turn Right "); SERIAL_COMMAND_PORT.print(value); SERIAL_COMMAND_PORT.println("deg");
            break;
        case 'S': // Stop
            _chassis.stop(); // Explicit stop command
            if (_chassis.getState() == MovementState::STALLED) { // If it was stalled, 'S' should reset it to IDLE
                 // The stop() call itself might not reset STALLED to IDLE to prevent auto-reset after stall.
                 // But an explicit 'S' command should.
                 // Forcing a new moveDistance(0) will call stop and reset state.
                 _chassis.moveDistance(0.0); // This calls stop() and sets state to IDLE
            }
            SERIAL_COMMAND_PORT.println("Executing: Stop");
            break;
        default:
            SERIAL_COMMAND_PORT.print("Unknown command: "); SERIAL_COMMAND_PORT.println(cmd);
            break;
    }
}
