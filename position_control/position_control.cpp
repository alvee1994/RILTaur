#include "position_control.h"

// untested and probably wont be used
// struct LegGain gait_gains = {80, 0.5, 50, 0.5};

bool IsValidGaitParams(struct GaitParams params) {
    const float maxL = 0.5;
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


void gait(struct LegIdentifier legs[],
                struct GaitParams params,
                unsigned long long& time_ms,
                float leg0_offset, float leg1_offset,
                float leg2_offset, float leg3_offset) {

    struct GaitParams paramsR = params;
    struct GaitParams paramsL = params;
    paramsR.step_length -= params.step_diff;
    paramsL.step_length += params.step_diff;

    float t = time_ms/1000.0;

    const float leg0_direction = -1.0;
    CoupledMoveLeg(t, paramsL, leg0_offset, leg0_direction,
        legs[0].theta, legs[0].gamma);

    const float leg1_direction = -1.0;
    CoupledMoveLeg(t, paramsL, leg1_offset, leg1_direction,
        legs[1].theta, legs[1].gamma);

    const float leg2_direction = 1.0;
    CoupledMoveLeg(t, paramsR, leg2_offset, leg2_direction,
        legs[2].theta, legs[2].gamma);

    const float leg3_direction = 1.0;
    CoupledMoveLeg(t, paramsR, leg3_offset, leg3_direction,
        legs[3].theta, legs[3].gamma);
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
    theta = atan2(y, leg_direction * x);
}

/**
* Takes the leg parameters and returns the gamma angle (rad) of the legs
*/
void GetGamma(float L, float theta, float& gamma) {
    float L1 = 0.095; // upper leg length (m)
    float L2 = 0.16; // lower leg length (m)
    float cos_param = (pow(L1,2.0) + pow(L,2.0) - pow(L2,2.0)) / (2.0*L1*L);
    if (cos_param < -1.0) {
        gamma = PI;
        #ifdef DEBUG_HIGH
        Serial.println("ERROR: L is too small to find valid alpha and beta!");
        #endif
      } else if (cos_param > 1.0) {
        gamma = 0;
        #ifdef DEBUG_HIGH
        Serial.println("ERROR: L is too large to find valid alpha and beta!");
        #endif
      } else {
        gamma = acos(cos_param);
      }
}

