#include "PinNames.h"
#include "all_libs.h"
#include "imuHandler.h"
#include <ios>



// #include <iostream>
 
Ticker ticker;
// string thestring = "a string";
// hash<string> somestring;
// can interface

CAN can_one(D10, D2);
// float tff = 0;

struct LegModes leg_modes; // different modes of the motor


// main() runs in its own thread in the OS
int main()
{

    printf("started...\n");
    struct LegIdentifier legs[4] = {
        // Left leg

        // leg direction, ma, mb, feed forward, tff, theta, gamma
        {1, 4, 3, 0, 0.0, 0.0}, // leg0
        {1, 2, 1, 0, 0.0, 0.0}, // leg1

        // Right leg
        {-1, 8, 7, 0, 0.0, 0.0}, // leg2
        {-1, 6, 5, 0, 0.0, 0.0} // leg3
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

    start_imu_thread(); // IMU thread
    start_position_control(can_one, legs); // GAIT control thread
    PrintGaitCommands();
    start_serial_commands(legs); // Serial comm/xbee thread

    while(true){

    }



}
