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
        int received[2];

        float alpha = legs[i].theta + legs[i].gamma; 
        float beta = legs[i].theta - legs[i].gamma;

        int motorA_pos = float_to_uint(int(alpha*1000)/1000, -95.5, 95.5, 16);
        int motorB_pos = float_to_uint(int(beta*1000)/1000, -95.5, 95.5, 16);

        int kp = 30;
        int kd = 400;

        transmit(can_one, legs[i].motorA, motorA_pos, 2047, kp, kd, 2047 - legs[i].tff);
        // printf("motorA: %i, ", motorA_pos);
        // int motorA_encoder = receiveCAN(can_one);
        transmit(can_one, legs[i].motorB, motorB_pos, 2047, kp, kd, 2047 + legs[i].tff);
        // printf("motorB: %i, ", motorB_pos);
        // int motorB_encoder = receiveCAN(can_one);

        // printf("%i, %i, %i, %i\n", motorA_pos, motorA_encoder, motorB_pos, motorB_encoder);
        wait_us(8000);

        
    }
};

// main() runs in its own thread in the OS
int main()
{
    t.start();
    can_one.frequency(1000000);

    struct LegIdentifier legs[4] = {
    //   mA, mB,             theta, gamma

        // left leg
        {1, 4, 3, 200, 0.0, 0.0}, // leg0
        {-1, 2, 1, 200, 0.0, 0.0}, // leg1

        // right leg
        {1, 7, 8, 200, 0.0, 0.0}, // leg2
        {-1, 5, 6, 200, 0.0, 0.0} // leg3
    };

    // these are 13 different gait parameters for the 13 different things that the dogg should be able to do
    // we are starting with TROT only, the rest are unchanged from the original code and untested for RILtaur
    struct GaitParams state_gait_params[3] = {
      //{s.h,  d.a., u.a., f.p., s.l., fr., s.d.}
        {0.20, 0.05, 0.03, 0.6, 0.20, 0.8, 0.0}, // TROT
    };

    printf("started...\n");

    // motor mode and zero position
    for (int i = 0; i<4; i++){
        // send(can_one, legs[i].motorA, leg_modes.zero_mode, 8);
        // send(can_one, legs[i].motorB, leg_modes.zero_mode, 8);
        wait_us(500000);
        send(can_one, legs[i].motorA, leg_modes.motor_mode, 8);
        send(can_one, legs[i].motorB, leg_modes.motor_mode, 8);

        printf("IDs are %i, %i\n", legs[i].motorA, legs[i].motorB);
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


