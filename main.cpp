#include "PinNames.h"
#include "all_libs.h"



// #include <iostream>
 
Ticker ticker;

// can interface
CAN can_one(D10, D2);
// float tff = 0;

struct LegModes leg_modes; // different modes of the motor


// main() runs in its own thread in the OS
int main()
{
    printf("=============== Started RILtaur ===============\n");

    struct LegIdentifier legs[4] = {
        // Left leg

        // leg direction, ma, mb, feed forward, tff, theta, gamma
        {1, 4, 3, 0, 0.0, 0.0}, // leg0
        {1, 2, 1, 0, 0.0, 0.0}, // leg1

        // Right leg
        {-1, 8, 7, 0, 0.0, 0.0}, // leg2
        {-1, 6, 5, 0, 0.0, 0.0} // leg3
    };

    PrintGaitCommands();
    start_serial_commands();
    start_imu_thread();
    can_one.frequency(1000000);
    wait_us(5000000);
    


    for (int i = 0; i<4; i++){
        send(can_one, legs[i].motorA, leg_modes.exit_mode, 8);
        send(can_one, legs[i].motorB, leg_modes.exit_mode, 8);
        wait_us(200000);
        send(can_one, legs[i].motorA, leg_modes.zero_mode, 8);
        send(can_one, legs[i].motorB, leg_modes.zero_mode, 8);
        wait_us(500000);
        send(can_one, legs[i].motorA, leg_modes.motor_mode, 8);
        send(can_one, legs[i].motorB, leg_modes.motor_mode, 8);

        printf("IDs are %i, %i\n", legs[i].motorA, legs[i].motorB);
    }

    wait_us(5000000);
    start_position_control(can_one, legs);
}


