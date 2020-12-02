#ifndef generalFunctions_H
#define generalFunctions_H

//** Math ***************************************
#define M_PI            ( 3.14159265358979 )
#define DEG_TO_RAD(x)   ( x * 0.0174532925 )
#define RAD_TO_DEG(x)   ( x * 57.295779513 )

//** IMU MPU9150 ********************************
#define DRIFT_CORRECTION(x) (kp1*x*x + kp2*x + kp3)*eq_correction

//** Encoder ************************************
#define encoder_cpr (4 * encoder_resolution)


//** General functions class ********************
class generalFunctions
{
public:
    static float sign_f(float in);
    static float abs_f(float in);
    static float map_f(float in, float in_min, float in_max, float out_min, float out_max);
    static float constrain_f(float in, float out_min, float out_max);
    static float moving_window(float array[], unsigned int window_size);

private:
    
    
};


#endif