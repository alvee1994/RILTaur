#ifndef POSITION_CONTROL_H
#define POSITION_CONTROL_H

#define PI 3.14159265359f
#define SQRT3 1.73205080757f

#include "mbed.h"
#include "math.h"
#include "can_comm/can_comm.h"

void postion_16bit(CAN& can_interface, struct LegIdentifier legs[], float delay);
void GetGamma(float L, float theta, float& gamma);
void LegParamsToCartesian(float L, float theta, float& x, float& y);
void CartesianToLegParams(float x, float y, float leg_direction, float& L, float& theta);
void CartesianToThetaGamma(float x, float y, float leg_direction, float& theta, float& gamma);
void SinTrajectory (float t, struct GaitParams params, float gaitOffset, float& x, float& y);
void CoupledMoveLeg(float t, struct GaitParams params, float gait_offset, float leg_direction, float& theta, float& gamma);
bool IsValidGaitParams(struct GaitParams params);
bool IsValidLegGain(struct LegGain gain);
void SinTrajectoryPosControl();
void gait(struct LegIdentifier legs[], struct GaitParams params, unsigned long long& time_ms, float leg0_offset, float leg1_offset, float leg2_offset, float leg3_offset);



enum States {
    STOP = 0,
    TROT = 1,
    BOUND = 2,
    WALK = 3,
    PRONK = 4,
    JUMP = 5,
    DANCE = 6,
    HOP = 7,
    TEST = 8,
    ROTATE = 9,
    FLIP = 10,
    TURN_TROT = 11
};

extern States state;

struct GaitParams {
    float stance_height = 0.0; // Desired height of body from ground during walking (m)
    float down_amp = 0.00; // Peak amplitude below stanceHeight in sinusoidal trajectory (m)
    float up_amp = 0.00; // Height the foot peaks at above the stanceHeight in sinusoidal trajectory (m)
    float flight_percent = 0.0; // Portion of the gait time should be doing the down portion of trajectory
    float step_length = 0.0; // Length of entire step (m)
    float freq = 0.0; // Frequency of one gait cycle (Hz)
    float step_diff = 0.0; //difference between left and right leg step length
};

struct LegGain {
    float kp_theta = 0;
    float kd_theta = 0;
    float kp_gamma = 0;
    float kd_gamma = 0;
};

struct MotorParams {
    int start = 32768; // 16-bit
    int ff = 2047; // 12-bit
    int kp = 15; // 0-4095
    int kd = 200; // 0-4095 
};

// identifying each leg and their corresponding parameters
struct LegIdentifier{
    int leg_direction = 0;
    uint8_t motorA = 0;
    uint8_t motorB = 0;
    float tff = 0.0;
    float theta = 0.0;
    float gamma = 0.0;
};


extern struct MotorParams motor_params;
// extern struct GaitParams state_gait_params[13];
extern struct LegGain gait_gains;

#endif