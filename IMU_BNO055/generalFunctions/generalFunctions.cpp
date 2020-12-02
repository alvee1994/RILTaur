#include "generalFunctions.h"

float generalFunctions::sign_f(float in)
{
    if(in >= 0) return 1;
    else        return -1;
}

float generalFunctions::abs_f(float in)
{
    if(in >= 0) return in;
    else        return (-1.0*in);
}

float generalFunctions::map_f(float in, float in_min, float in_max, float out_min, float out_max)
{
    return (in - in_min)*(out_max - out_min)/(in_max - in_min) + out_min;
}

float generalFunctions::constrain_f(float in, float out_min, float out_max)
{
    if(in <= out_min)
        return (out_min);
    else if(in >= out_max)
        return (out_max);
    else
        return (in);
}

float generalFunctions::moving_window(float array[], unsigned int window_size)
{
    float movAvg_out=0;

    for(volatile unsigned int i=0; i<window_size; i++) {
        movAvg_out += array[i];
    }

    movAvg_out = movAvg_out/((float)window_size);

    return (movAvg_out);
}
