#include "all_libs.h"

// #include <iostream>
 
Ticker ticker;

// can interface
CAN can_one(D10, D2);
// float tff = 0;

struct LegModes leg_modes; // different modes of the motor
// using namespace std::chrono;
// Timer t;

// main() runs in its own thread in the OS
int main()
{
    // t.start();
    // can_one.frequency(1000000);

    // struct LegIdentifier legs[4] = {
    //     // Left leg

    //     // leg direction, ma, mb, feed forward, theta, gamma
    //     {1, 4, 3, tff, 0.0, 0.0}, // leg0
    //     {1, 2, 1, tff, 0.0, 0.0}, // leg1

    //     // Right leg
    //     {-1, 8, 7, tff, 0.0, 0.0}, // leg2
    //     {-1, 6, 5, tff, 0.0, 0.0} // leg3
    // };

    // these are 13 different gait parameters for the 13 different things that the dogg should be able to do
    // we are starting with TROT only, the rest are unchanged from the original code and untested for RILtaur
    // struct GaitParams state_gait_params[3] = {
    //   //{s.h,  d.a., u.a., f.p., s.l., fr., s.d.}
    //     {0.18, 0.03, 0.03, 0.5, 0.15, 1.5, 0.0}, // TROT
    // };

    printf("started...\n");

    // motor mode and zero position
    // for (int i = 0; i<4; i++){
    //     send(can_one, legs[i].motorA, leg_modes.zero_mode, 8);
    //     send(can_one, legs[i].motorB, leg_modes.zero_mode, 8);
    //     wait_us(500000);
    //     send(can_one, legs[i].motorA, leg_modes.motor_mode, 8);
    //     send(can_one, legs[i].motorB, leg_modes.motor_mode, 8);

    //     printf("IDs are %i, %i\n", legs[i].motorA, legs[i].motorB);
    // }

    // send(can_one, 1, leg_modes.motor_mode, 8);
    // send(can_one, 1, leg_modes.zero_mode, 8);

    struct LegIdentifier legs[4] = {
        // Left leg

        // leg direction, ma, mb, feed forward, tff, theta, gamma
        {1, 3, 4, 0, 0.0, 0.0}, // leg0
        {1, 10, 2, 0, 0.0, 0.0}, // leg1

        // Right leg
        {-1, 7, 8, 0, 0.0, 0.0}, // leg2
        {-1, 5, 6, 0, 0.0, 0.0} // leg3
    };

    can_one.frequency(1000000);


    for (int i = 0; i<4; i++){
        send(can_one, legs[i].motorA, leg_modes.exit_mode, 8);
        send(can_one, legs[i].motorB, leg_modes.exit_mode, 8);
        wait_us(500000);
        send(can_one, legs[i].motorA, leg_modes.zero_mode, 8);
        send(can_one, legs[i].motorB, leg_modes.zero_mode, 8);
        wait_us(500000);
        send(can_one, legs[i].motorA, leg_modes.motor_mode, 8);
        send(can_one, legs[i].motorB, leg_modes.motor_mode, 8);
        wait_us(500000);

        printf("IDs are %i, %i\n", legs[i].motorA, legs[i].motorB);
    }

    // for (int i=0; i < 4; i++) {
    //     transmit(can_one, legs[i].motorA, 32767, 2047, 60, 400, 2047);
    //     transmit(can_one, legs[i].motorB, 32767, 2047, 60, 400, 2047);
    //     wait_us(500000);
    // }

    wait_us(500000);

    start_position_control(can_one, legs);

    PrintGaitCommands();
    start_serial_commands(legs);

    while(true){
        // unsigned long long time_ms = duration_cast<milliseconds>(t.elapsed_time()).count(); // get the system time in milliseconds
        
        // gait(legs, state_gait_params[0], time_ms, 0.0, 0.5, 0.0, 0.5);
        // gait(legs, state_gait_params[0], time_ms, 0.0, 0.5, 0.0, 0.5);
        // postion_16bit(can_one, legs, 0.35);

    }



}


