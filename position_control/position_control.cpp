#include "position_control.h"
#include <cstdio>
#include <iterator>

// untested and probably wont be used
// struct LegGain gait_gains = {80, 0.5, 50, 0.5};

Thread position_control_thread;

/*----------------------------------------------------------------*/
using namespace std::chrono;
Timer t;
unsigned long long time_ms;
CAN* can_comm;
float tff = 0;
extern struct LegModes leg_modes; // different modes of the motor
struct LegIdentifier* legs_;

struct GaitParams state_gait_params[] = {
    //{s.h,  d.a., u.a., f.p., s.l., fr., s.d.}
    {NAN, NAN, NAN, NAN, NAN, NAN, NAN}, // STOP
    {0.18, 0.03, 0.03, 0.5, 0.15, 1.5, 0.0}, // TROT
    {0.17, 0.04, 0.06, 0.35, 0.0, 2.0, 0.0}, // BOUND
    {0.15, 0.00, 0.06, 0.25, 0.0, 1.5, 0.0}, // WALK
    {0.12, 0.05, 0.0, 0.75, 0.0, 1.0, 0.0}, // PRONK
    {NAN, NAN, NAN, NAN, NAN, NAN, NAN}, // JUMP
    {0.15, 0.05, 0.05, 0.35, 0.0, 1.5, 0.0}, // DANCE
    {0.15, 0.05, 0.05, 0.2, 0.0, 1.0, 0.0}, // HOP
    {NAN, NAN, NAN, NAN, NAN, 1.0, NAN}, // TEST
    {NAN, NAN, NAN, NAN, NAN, NAN, NAN}, // ROTATE
    {0.15, 0.07, 0.06, 0.2, 0.0, 1.0, 0.0}, // FLIP
    {0.18, 0.03, 0.03, 0.5, 0.15, 1.5, 0.06}, // TURN_TROT
    {NAN, NAN, NAN, NAN, NAN, NAN, NAN} // RESET
};
long rotate_start = 0; // milliseconds when rotate was commanded
States state = STOP;
int straight_dir = 1;
int turn_dir = 1;
/*----------------------------------------------------------------*/

void start_position_control(CAN& can_interface, struct LegIdentifier* legs) {
    can_comm = &can_interface;
    legs_ = legs;
    position_control_thread.start(position_control_func);
}

void position_control_func() {
    t.start();

    while(true) {
        time_ms = duration_cast<milliseconds>(t.elapsed_time()).count(); // get the system time in milliseconds
        struct GaitParams gait_params = state_gait_params[state];
        // printf("%d\n",state);
        switch(state) {
            case STOP:
                {
                    LegGain stop_gain = {50, 0.5, 50, 0.5};
                    float y1 = 0.15;
                    float y2 = 0.15;

                    int kp = 60;
                    int kd = 400;

                    for (int i=0; i < 4; i++) {
                        CartesianToThetaGamma(0.0, y1, legs_[i].leg_direction, legs_[i].theta, legs_[i].gamma);
                        transmit(*can_comm, legs_[i].motorA, 32767, 2047, kp, kd, 2047);
                        transmit(*can_comm, legs_[i].motorB, 32767, 2047, kp, kd, 2047);
                    }
                    // CartesianToThetaGamma(0.0, y2, legs_[0].leg_direction, legs_[0].theta, legs_[0].gamma);
                    // CartesianToThetaGamma(0.0, y1, legs_[1].leg_direction, legs_[1].theta, legs_[1].gamma);
                    // CartesianToThetaGamma(0.0, y1, legs_[2].leg_direction, legs_[2].theta, legs_[2].gamma);
                    // CartesianToThetaGamma(0.0, y2, legs_[3].leg_direction, legs_[3].theta, legs_[3].gamma);
                    
                    // postion_16bit(can_comm, legs_, 0.35);
                    //transmit(can_comm, legs_[0].motorA, 32767, 2047, kp, kd, 2047);
                    // transmit(can_comm, legs_[0].motorB, 32767, 2047, kp, kd, 2047);
                    // transmit(can_comm, legs_[1].motorA, 32767, 2047, kp, kd, 2047);
                    // transmit(can_comm, legs_[1].motorB, 32767, 2047, kp, kd, 2047);
                    // transmit(can_comm, legs_[2].motorA, 32767, 2047, kp, kd, 2047);
                    // transmit(can_comm, legs_[2].motorB, 32767, 2047, kp, kd, 2047);
                    // transmit(can_comm, legs_[3].motorA, 32767, 2047, kp, kd, 2047);
                    // transmit(can_comm, legs_[3].motorB, 32767, 2047, kp, kd, 2047);
                }
                break;
            case DANCE:
                //gait(legs_, gait_params, 0.0, 0.5, 0.0, 0.5, gait_gains);
                break;
            case BOUND:
                //gait(gait_params, 0.0, 0.5, 0.5, 0.0, gait_gains);
                break;
            case TROT:
                gait(legs_, gait_params, time_ms, 0.0, 0.5, 0.0, 0.5);
                postion_16bit(*can_comm, legs_, 0.35);
                break;
            case TURN_TROT:
                gait(legs_, gait_params, time_ms, 0.0, 0.5, 0.0, 0.5);
                postion_16bit(*can_comm, legs_, 0.35);
                break;
            case WALK:
                gait(legs_, gait_params, time_ms, 0.0, 0.25, 0.75, 0.5);
                postion_16bit(*can_comm, legs_, 0.35);
                break;
            case PRONK:
                //gait(gait_params, 0.0, 0.0, 0.0, 0.0, gait_gains);
                break;
            case JUMP:
                //ExecuteJump();
                break;
            case ROTATE:
                {
                float theta,gamma;
                CartesianToThetaGamma(0, 0.24, 1.0, theta, gamma);
                float freq = 0.1;
                float phase = freq * (time_ms - rotate_start)/1000.0f;
                theta = (-cos(2*PI * phase) + 1.0f) * 0.5 * 2 * PI;
                //CommandAllLegs(theta, gamma, gait_gains);
                }
            case HOP:
                //hop(gait_params);
                break;
            case FLIP:
                //ExecuteFlip(gait_params);
                break;
            // case RESET_:
            //     //reset();
            //     break;
            // case TEST:
            //     //test();
            //     break;
        }
    }
}

void postion_16bit(CAN& can_interface, struct LegIdentifier legs_[], float delay){
    for (int i = 0; i < 4; i++){
        float alpha;
        float beta;

        if (i == 0 || i == 2) {
            alpha = (legs_[i].gamma - legs_[i].theta) - PI/2.0; 
            beta = (legs_[i].gamma + legs_[i].theta) - PI/2.0;
        } else {
            alpha = ((-1*legs_[i].gamma) - legs_[i].theta) + PI/2.0; 
            beta = ((-1*legs_[i].gamma) + legs_[i].theta) + PI/2.0;
        }


        // alpha = (legs_[i].gamma - legs_[i].theta) - PI/2.0; 
        // beta = (legs_[i].gamma + legs_[i].theta) - PI/2.0;

        // alpha = (legs_[i].gamma - legs_[i].theta) - PI/2.0; 
        // beta = (legs_[i].gamma + legs_[i].theta) - PI/2.0;
        int received[2];




        int motorA_pos = float_to_uint(alpha, -95.5, 95.5, 16);
        int motorB_pos = float_to_uint(beta, -95.5, 95.5, 16);

        // int motorA_pos = float_to_uint(int(alpha*10)/10, -95.5, 95.5, 16);
        // int motorB_pos = float_to_uint(int(beta*10)/10, -95.5, 95.5, 16);

        int kp = 60;
        int kd = 400;

        transmit(can_interface, legs_[i].motorA, motorA_pos, 2047, kp, kd, 2047 - legs_[i].tff);
        transmit(can_interface, legs_[i].motorB, motorB_pos, 2047, kp, kd, 2047 + legs_[i].tff);
        
        ThisThread::sleep_for(delay);
        
    }
};

bool IsValidGaitParams(struct GaitParams params) {
    const float maxL = 0.35;
    const float minL = 0.08;

    float stanceHeight = params.stance_height;
    float downAMP = params.down_amp;
    float upAMP = params.up_amp;
    float flightPercent = params.flight_percent;
    float stepLength = params.step_length;
    float FREQ = params.freq;

    if (stanceHeight + downAMP > maxL || sqrt(pow(stanceHeight, 2) + pow(stepLength / 2.0, 2)) > maxL) {
        printf("Gait overextends leg");
        return false;
    }
    if (stanceHeight - upAMP < minL) {
        printf("Gait underextends leg");
        return false;
    }

    if (flightPercent <= 0 || flightPercent > 1.0) {
        printf("Flight percent is invalid");
        return false;
    }

    if (FREQ < 0) {
        printf("Frequency cannot be negative");
        return false;
    }

    if (FREQ > 10.0) {
        printf("Frequency is too high (>10)");
        return false;
    }

    return true;
}


void gait(struct LegIdentifier legs_[],
                struct GaitParams params,
                unsigned long long& time_ms,
                float leg0_offset, float leg1_offset,
                float leg2_offset, float leg3_offset) {

    struct GaitParams paramsR = params;
    struct GaitParams paramsL = params;
    paramsR.step_length -= params.step_diff;
    paramsL.step_length += params.step_diff;

    float t = time_ms/1000.0;

    const float leg0_direction = legs_[0].leg_direction;
    CoupledMoveLeg(t, paramsL, leg0_offset, leg0_direction,
        legs_[0].theta, legs_[0].gamma);

    const float leg1_direction = legs_[1].leg_direction;
    CoupledMoveLeg(t, paramsL, leg1_offset, leg1_direction,
        legs_[1].theta, legs_[1].gamma);

    const float leg2_direction = legs_[2].leg_direction;
    CoupledMoveLeg(t, paramsR, leg2_offset, leg2_direction,
        legs_[2].theta, legs_[2].gamma);

    const float leg3_direction = legs_[3].leg_direction;
    CoupledMoveLeg(t, paramsR, leg3_offset, leg3_direction,
        legs_[3].theta, legs_[3].gamma);
    // wait_us(100000);
}

void CoupledMoveLeg(float t, struct GaitParams params,
                    float gait_offset, float leg_direction,
                    float& theta, float& gamma) {
    float x; // float x for leg 0 to be set by the sin trajectory
    float y;
    SinTrajectory(t, params, gait_offset, x, y);
    CartesianToThetaGamma(x, y, leg_direction, theta, gamma);
    // SetCoupledPosition(theta, gamma, gains);
}

/**
* Sinusoidal trajectory generator function with flexibility from parameters described below. Can do 4-beat, 2-beat, trotting, etc with this.
*/
void SinTrajectory (float t, struct GaitParams params, float gaitOffset, float& x, float& y) {
    static float p = 0;
    static float prev_t = 0;

    float stanceHeight = params.stance_height;
    float downAMP = params.down_amp;
    float upAMP = params.up_amp;
    float flightPercent = params.flight_percent;
    float stepLength = params.step_length;
    float FREQ = params.freq;



    p += FREQ * (t - prev_t < 0.5 ? t - prev_t : 0); // should reduce the lurching when starting a new gait
    // p += FREQ * (t - prev_t < 0.5 ? t - prev_t : 0); // should reduce the lurching when starting a new gait
    prev_t = t;

    float gp = fmod((p+gaitOffset),1.0); // mod(a,m) returns remainder division of a by m
    if (gp <= flightPercent) {
        x = ((gp/flightPercent)*stepLength) - stepLength/2.0;
        y = -upAMP*sin(PI*gp/flightPercent) + stanceHeight;
    }
    else {
        float percentBack = (gp-flightPercent)/(1.0-flightPercent);
        x = -percentBack*stepLength + stepLength/2.0;
        y = downAMP*sin(PI*percentBack) + stanceHeight;
    }

    // printf("%i,%i\n", int(x*100), int(y*100));
    

    
}

void CartesianToThetaGamma(float x, float y, float leg_direction, float& theta, float& gamma) {
    float L = 0.0;
    CartesianToLegParams(x, y, leg_direction, L, theta);
    GetGamma(L, theta, gamma);
    //Serial << "Th, Gam: " << theta << " " << gamma << '\n';
}

/**
* Converts the cartesian coords x, y (m) to leg params L (m), theta (rad)
*/
void CartesianToLegParams(float x, float y, float leg_direction, float& L, float& theta) {
    L = pow((pow(x,2.0) + pow(y,2.0)), 0.5);
    theta = atan2(leg_direction * x, y);
}

/**
* Takes the leg parameters and returns the gamma angle (rad) of the legs_
*/
void GetGamma(float L, float theta, float& gamma) {
    float L1 = 0.09; // upper leg length (m)
    float L2 = 0.161; // lower leg length (m)
    float cos_param = (pow(L1,2.0) + pow(L,2.0) - pow(L2,2.0)) / (2.0*L1*L);
    if (cos_param < -1.0) {
        gamma = PI;
        printf("\nERROR: L is too small to find valid alpha and beta!");
      } else if (cos_param > 1.0) {
        gamma = 0;
        printf("\nERROR: L is too large to find valid alpha and beta!");
      } else {
        gamma = acos(cos_param);
      }
}

void UpdateStateGaitParams(States curr_state) {
    if (!isnan(state_gait_params[STOP].stance_height)) {
        state_gait_params[curr_state].stance_height = state_gait_params[STOP].stance_height;
        state_gait_params[STOP].stance_height = NAN;
    }
    if (!isnan(state_gait_params[STOP].down_amp)) {
        state_gait_params[curr_state].down_amp = state_gait_params[STOP].down_amp;
        state_gait_params[STOP].down_amp = NAN;
    }
    if (!isnan(state_gait_params[STOP].up_amp)) {
        state_gait_params[curr_state].up_amp = state_gait_params[STOP].up_amp;
        state_gait_params[STOP].up_amp = NAN;
    }
    if (!isnan(state_gait_params[STOP].flight_percent)) {
        state_gait_params[curr_state].flight_percent = state_gait_params[STOP].flight_percent;
        state_gait_params[STOP].flight_percent = NAN;
    }
    if (!isnan(state_gait_params[STOP].step_length)) {
        state_gait_params[curr_state].step_length = state_gait_params[STOP].step_length;
        state_gait_params[STOP].step_length = NAN;
    }
    if (!isnan(state_gait_params[STOP].freq)) {
        state_gait_params[curr_state].freq = state_gait_params[STOP].freq;
        state_gait_params[STOP].freq = NAN;
    }
    if (!isnan(state_gait_params[STOP].step_diff)) {
        state_gait_params[curr_state].step_diff = state_gait_params[STOP].step_diff;
        state_gait_params[STOP].step_diff = NAN;
    }
}

/**
 * Dance gait parameters
 */
void TransitionToDance() {
    state = DANCE;
    printf("DANCE\n");
    //            {s.h, d.a., u.a., f.p., s.l., fr.}
    //gait_params = {0.15, 0.05, 0.05, 0.35, 0.0, 1.5};
    UpdateStateGaitParams(DANCE);
    //gait_gains = {50, 0.5, 30, 0.5};
    //PrintGaitParams();
}
/**
* Pronk gait parameters
*/
void TransitionToPronk() {
    state = PRONK;
    printf("PRONK\n");
    //            {s.h, d.a., u.a., f.p., s.l., fr.}
    //gait_params = {0.12, 0.05, 0.0, 0.75, 0.0, 1.0};
    UpdateStateGaitParams(PRONK);
    //gait_gains = {80, 0.50, 50, 0.50};
    //PrintGaitParams();
}

/**
* Bound gait parameters
*/
void TransitionToBound() {
    state = BOUND;
    printf("BOUND\n");
    //            {s.h, d.a., u.a., f.p., s.l., fr.}
    //gait_params = {0.17, 0.04, 0.06, 0.35, 0.0, 2.0};
    UpdateStateGaitParams(BOUND);
    // gait_gains = {80, 0.5, 50, 0.5};
    // PrintGaitParams();
}

/**
 * Walk gait parameters
 */
void TransitionToWalk() {
    state = WALK;
    printf("WALK\n");
    //            {s.h, d.a., u.a., f.p., s.l., fr.}
    //gait_params = {0.15, 0.00, 0.06, 0.25, 0.0, 1.5};
    UpdateStateGaitParams(WALK);
    // gait_gains = {80, 0.5, 50, 0.5};
    // PrintGaitParams();
}

/**
* Trot gait parameters
*/
void TransitionToTrot() {
    state = TROT;
    printf("TROT\n");
    //            {s.h, d.a., u.a., f.p., s.l., fr.}
    //gait_params = {0.17, 0.04, 0.06, 0.35, 0.15, 2.0};
    UpdateStateGaitParams(TROT);
    state_gait_params[TROT].step_diff = 0.0; // TROT should always go straight
    // gait_gains = {80, 0.5, 50, 0.5};
    // PrintGaitParams();
}

/**
* Turn Trot gait parameters
*/
void TransitionToTurnTrot() {
    state = TURN_TROT;
    printf("TURN_TROT\n");
    //            {s.h, d.a., u.a., f.p., s.l., fr., sd.}
    //gait_params = {0.17, 0.04, 0.06, 0.35, 0.1, 2.0, 0.06};
    UpdateStateGaitParams(TURN_TROT);
    // gait_gains = {80, 0.5, 80, 0.5};
    // PrintGaitParams();
}

void TransitionToRotate() {
    state = ROTATE;
    rotate_start = time_ms;
    printf("ROTATE\n");
    //gait_gains = {30,0.5,30,0.5};
}
void TransitionToHop() {
    state = HOP;
    printf("HOP\n");
    //            {s.h, d.a., u.a., f.p., s.l., fr.}
    //gait_params = {0.15, 0.05, 0.05, 0.2, 0, 1.0};
    // UpdateStateGaitParams(HOP);
    // PrintGaitParams();
}