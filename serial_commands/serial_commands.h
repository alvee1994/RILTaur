#ifndef SERIAL_COMMANDS_H
#define SERIAL_COMMANDS_H


#define USE_PC_SERIAL

#include "mbed.h"
#include "position_control.h"

void start_serial_commands(struct LegIdentifier* legs);
void serial_commands_func();
void InterpretCommand(char* cmd);
void PrintGaitCommands();

extern struct LegIdentifier* legs_;

#endif