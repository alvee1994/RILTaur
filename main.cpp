#include "CAN.h"
#include "ThisThread.h"
#include "can_helper.h"
#include "coap_security_handler.h"
#include "mbed.h"
#include "mbed_wait_api.h"
#include <cmath>
#include <cstdint>
#include <limits>
#include "can_comm.h"
#include "position_control.h"

// #include <iostream>
 
Ticker ticker;

// can interface
CAN can_one(D10, D2);

struct LegModes leg_modes; // different modes of the motor
using namespace std::chrono;
Timer t;


void postion_16bit(struct LegIdentifier legs[]){
    for (int i = 0; i < 4; i++){
        float alpha = legs[0].theta + legs[0].gamma; 
        float beta = legs[0].theta - legs[0].gamma;

        int motorA_pos = float_to_uint(alpha, -95.5, 95.5, 16);
        int motorB_pos = float_to_uint(beta, -95.5, 95.5, 16);

        transmit(can_one, legs[0].motorA, motorA_pos, 2047, 30, 250, 2047);
        transmit(can_one, legs[0].motorB, motorB_pos, 2047, 30, 250, 2047);
    }
};

// main() runs in its own thread in the OS
int main()
{
    t.start();
    can_one.frequency(1000000);

    struct LegIdentifier legs[4] = {
    //   mA, mB,             theta, gamma
        {4, 3, -650, 0.0, 0.0}, // leg0
        {1, 2, -650, 0.0, 0.0}, // leg1
        {8, 7, -615, 0.0, 0.0}, // leg2
        {5, 6, -500, 0.0, 0.0} // leg3
    };

    // these are 13 different gait parameters for the 13 different things that the dogg should be able to do
    // we are starting with TROT only, the rest are unchanged from the original code and untested for RILtaur
    struct GaitParams state_gait_params[3] = {
      //{s.h,  d.a., u.a., f.p., s.l., fr., s.d.}
        {0.22, 0.02, 0.02, 0.35, 0.15, 1.0, 0.0}, // TROT
    };

    printf("started...\n");

    // motor mode and zero position
    for (int i = 0; i<4; i++){
        send(can_one, legs[i].motorA, leg_modes.motor_mode, 8);
        send(can_one, legs[i].motorA, leg_modes.zero_mode, 8);
        send(can_one, legs[i].motorB, leg_modes.motor_mode, 8);
        send(can_one, legs[i].motorB, leg_modes.zero_mode, 8);
    }


    // send(can_one, 1, leg_modes.motor_mode, 8);
    // send(can_one, 1, leg_modes.zero_mode, 8);

    while(true){
        unsigned long long time_ms = duration_cast<milliseconds>(t.elapsed_time()).count(); // get the system time in milliseconds
        
        gait(legs, state_gait_params[0], time_ms, 0.0, 0.5, 0.0, 0.5);
        // wait_us(350000);
        postion_16bit(legs);

    }



}


