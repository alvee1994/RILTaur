 /*
 * Copyright (C) 2016 Akash Vibhute <akash.roboticist@gmail.com>
 * 
 * Wrapper to handle all quirks of MPU9250 and BNO055 IMU on Virgo 3 robot. 
 * Based off various user libraries
 *
 *
 * Initial Release: Apr/26/2016
 *
 */
 
/**
 * @file imuHandler.h
 *
 * Class declaration for IMU handling helper functions
 */

#ifndef imuHandler_H
#define imuHandler_H

//* IMU *
#define Pose_MWindowSize 4
#define GyroAcc_MWindowSize 4
//* BNO055 *
#define BNO055_StabilizationReadings 200 //number of non deviating data points to determine if imu has stabilized

#include "mbed.h"
#include "config.h"
#include "imuCalib.h"
#include "generalFunctions.h"
#include "BNO055.h"

class IMU_BNO055
{
public:
    IMU_BNO055();
    bool imuInit();
    void imuUpdate();
    bool imuInit_function();
    void imuReset();
    
    void getPose(float *pose[3]);
    void getQuat(float *quaternion[4]);
    void getAngVel(float *angVelocity[3]);
    void getLinAcc(float *linAcc[3]);
    float getTime();
    
    void setPose(float pose_in[3]); //to reset position on the fly

    float Pose[3]; //x,y,z
    float Quat[4]; //w,x,y,z
    float AngVel[3]; //x,y,z
    float LinAcc[3]; //x,y,z
    char calib_stat[5]; //Mag, Acc, Gyro, System, Overall
    
    float time_s; //imu timestamp
    uint8_t imu_stabilized[2];
    
    void readCalibrationData();
    int16_t offset_acc[3];
    int16_t offset_mag[3];
    int16_t offset_gyr[3];
    int16_t radius_acc;
    int16_t radius_mag;
    
    void writeCalibrationData();
    
    BNO055 imu_BNO055;
    
private:
    Timer bno_timer;
    float imuTime_s[2]; //index 0 in array holds values to correct wrap around condition

    
    
    float imu_initialAngles[3]; //array holds initial imu angles to offset
    uint16_t unstable_readings;
    float initialAcc[3]; //array hold the accelerometer values used during initial calib
    
    uint8_t calibration_status;
    char calibration_regs[22];
    
    static const unsigned int movWindow_lenMax = 32; //to prevent excessive RAM usage at program runtime
    unsigned int movWindow_len_Pose, movWindow_len_GyroAcc;
    unsigned int movWindow_index_Pose, movWindow_index_GyroAcc;

    float movWindow_Pose[3][movWindow_lenMax];
    float movWindow_AngVel[3][movWindow_lenMax];
    float movWindow_LinAcc[3][movWindow_lenMax];
    
    float PoseCorrection[3]; //x,y,z
};

void start_imu_thread();
void imu_thread();


#endif