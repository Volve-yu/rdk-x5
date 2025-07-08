#ifndef SERIALCOMMANDER_H
#define SERIALCOMMANDER_H

#include "Chassis.h" // Needs to know about Chassis to send commands

class SerialCommander {
public:
    SerialCommander(Chassis& chassis);
    void setup();
    void handleCommands();

private:
    Chassis& _chassis;
    String _commandBuffer;
    void parseCommand(String cmd);
};

#endif // SERIALCOMMANDER_H
