#include "all_libs.h"

// #include <iostream>
 
Ticker ticker;

// can interface
CAN can_one(D10, D2);
float tff = 0;

struct LegModes leg_modes; // different modes of the motor
using namespace std::chrono;
Timer t;

// main() runs in its own thread in the OS
int main()
{
    t.start();
    can_one.frequency(1000000);

    struct LegIdentifier legs[4] = {
        // Left leg

        // leg direction, ma, mb, feed forward, theta, gamma
        {1, 4, 3, tff, 0.0, 0.0}, // leg0
        {1, 2, 1, tff, 0.0, 0.0}, // leg1

        // Right leg
        {-1, 8, 7, tff, 0.0, 0.0}, // leg2
        {-1, 6, 5, tff, 0.0, 0.0} // leg3
    };

    // these are 13 different gait parameters for the 13 different things that the dogg should be able to do
    // we are starting with TROT only, the rest are unchanged from the original code and untested for RILtaur
    struct GaitParams state_gait_params[3] = {
      //{s.h,  d.a., u.a., f.p., s.l., fr., s.d.}
        {0.18, 0.03, 0.03, 0.5, 0.15, 1.5, 0.0}, // TROT
    };

    printf("started...\n");

    // motor mode and zero position
    for (int i = 0; i<4; i++){
        send(can_one, legs[i].motorA, leg_modes.zero_mode, 8);
        send(can_one, legs[i].motorB, leg_modes.zero_mode, 8);
        wait_us(500000);
        send(can_one, legs[i].motorA, leg_modes.motor_mode, 8);
        send(can_one, legs[i].motorB, leg_modes.motor_mode, 8);

        printf("IDs are %i, %i\n", legs[i].motorA, legs[i].motorB);
    }

    // send(can_one, 1, leg_modes.motor_mode, 8);
    // send(can_one, 1, leg_modes.zero_mode, 8);
    PrintGaitCommands();
    start_serial_commands();

    while(true){
        unsigned long long time_ms = duration_cast<milliseconds>(t.elapsed_time()).count(); // get the system time in milliseconds
        
        // gait(legs, state_gait_params[0], time_ms, 0.0, 0.5, 0.0, 0.5);
        gait(legs, state_gait_params[0], time_ms, 0.0, 0.5, 0.0, 0.5);
        postion_16bit(can_one, legs, 0.35);

    }



}


