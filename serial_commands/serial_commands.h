#ifndef SERIAL_COMMANDS_H
#define SERIAL_COMMANDS_H


#include "mbed.h"

void start_serial_commands();
void serial_commands_func();
void InterpretCommand(char* cmd);
void PrintGaitCommands();

#endif