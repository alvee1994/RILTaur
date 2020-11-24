#ifndef CAN_COMM_H
#define CAN_COMM_H

#include <cstdint>
#define PI 3.14159265359f
#define SQRT3 1.73205080757f

#include "mbed.h"
#include "math.h"
#include "CAN.h"


// can communication
void send(CAN& can_interface, uint8_t can_id, uint8_t cmsg[], int dlc);
void receive(CAN& can_interface);
void transmit(CAN& can_interface, uint8_t can_id, int position, int velocity, int kp, int kd, int ff);

// conversion
int float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);

// CAN message definitions
struct LegModes{
    uint8_t motor_mode[8] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfc}; // enters motor mode/closed loop control
    uint8_t exit_mode[8] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfd}; // exits the closed loop control
    uint8_t zero_mode[8] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfe}; // set current position to zero
};

// identifying each leg and their corresponding parameters
struct LegIdentifier{
    uint8_t motorA = 0;
    uint8_t motorB = 0;
    float tff = 0.0;
    float theta = 0.0;
    float gamma = 0.0;
};

#endif
