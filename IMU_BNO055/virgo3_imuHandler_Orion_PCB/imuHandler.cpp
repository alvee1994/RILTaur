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
#include "imuHandler.h"
/*** BNO055 ***/

#define imu_UpdateRateHz 400000 // imu update rate in Hz
#define imu_UpdatePeriodMS (1/imu_UpdateRateHz) // imu update rate in ms
#define bno_timer_read duration<float>{bno_timer.elapsed_time()}.count()

float imuTime;
using namespace std::chrono;
IMU_BNO055 imu; //Bosch BNO055 IMU wrapper class
// DigitalOut imuRst(D9); // didnt need to reset or use the 3.3v to ADR pin for the other address

// //** start IMU funtion as Thread **
Thread imu_function_thread;
void start_imu_thread(){
    imu_function_thread.start(imu_thread);
}
    

void imu_thread()
{
    bool init_status = imu.imuInit_function();
    if(init_status){
        printf("\tIMU successfully initialized\n");
    } else {
        printf("\tIMU failed to initialize (non fatal)\n");
    }
    ThisThread::sleep_for(200ms);
    while(init_status) {
        imu.imuUpdate();
        imuTime = imu.time_s;
        ThisThread::sleep_for(1ms);
        // printf("%.2f\n", imu.imu_BNO055.euler.yaw);
        // printf("Orientation (roll-pitch-yaw): (%.2f , %.2f , %.2f)\n", imu.Pose[0], imu.Pose[1], imu.Pose[2]);
    }
}

IMU_BNO055::IMU_BNO055(): imu_BNO055(I2C_SDA, I2C_SCL)
{
    unsigned int movWindowSize_Pose = Pose_MWindowSize;
    unsigned int movWindowSize_GyroAcc = GyroAcc_MWindowSize;

    unstable_readings = BNO055_StabilizationReadings;
    //imu_stabilized[0] =0;
    imu_stabilized[0] =1; //for imu mode no need to calibrate
    imu_stabilized[1] =0;

    if(movWindowSize_Pose <= movWindow_lenMax) movWindow_len_Pose = movWindowSize_Pose;
    else movWindow_len_Pose = movWindow_lenMax;

    if(movWindow_len_GyroAcc <= movWindow_lenMax) movWindow_len_GyroAcc = movWindowSize_GyroAcc;
    else movWindow_len_GyroAcc = movWindow_lenMax;

    movWindow_index_Pose=0;
    movWindow_index_GyroAcc=0;

    for(int i=0; i<3; i++)
        PoseCorrection[i]=0;
    
    bno_timer.start();    
}

bool IMU_BNO055::imuInit_function()
{
    imu_BNO055.reset();
    return (imuInit());
}

// void IMU_BNO055::imuReset() //Physical reset
// {   
//     imuRst = 0;
//     printf("reseting IMU\n");
//     wait_us(50000);
//     imuRst = 1;
//     return;
// }

bool IMU_BNO055::imuInit()
{
//    Debug.printf("print imu check %d", imu_BNO055.check());
    wait_us(300000);
    if (imu_BNO055.check()) {
        //load default calib parameters
        offset_acc[0] =10;
        offset_acc[1] =509;
        offset_acc[2] =500;

        offset_mag[0] =409;
        offset_mag[1] =487;
        offset_mag[2] =290;

        offset_gyr[0] =4;
        offset_gyr[0] =0;
        offset_gyr[2] =1;

        radius_acc = 235;
        radius_mag = 165;

        writeCalibrationData();

        imu_BNO055.set_mapping(2);
        imu_BNO055.setmode(OPERATION_MODE_IMUPLUS);

        imu_BNO055.get_calib();
        calibration_status = (imu_BNO055.calib & 0xc0) >> 6; //upper 2 MSB's denoting calibration status

        return(true);
    } else return(false);

    //debug_printf("Init finish!\n");
}

void IMU_BNO055::imuUpdate()
{
    imu_BNO055.get_calib();
    imu_BNO055.get_angles();

    imu_BNO055.get_lia(); //imu_BNO055.get_accel();
    imu_BNO055.get_gyro();
    //imu_BNO055.get_mag();
    imu_BNO055.get_quat();
    //imu_BNO055.get_grv();

    float posePRY[3];

    //debug_printf("DEBUG>> got YPR data\n");
    if(bno_timer_read > 2100) { //reset as timer is close to overflow
        bno_timer.reset();
        imuTime_s[0] = imuTime_s[1];
    } else
        imuTime_s[1] = imuTime_s[0] + bno_timer_read;

    time_s = imuTime_s[1]; //imu timestamp

    calib_stat[4] = imu_BNO055.calib;
    calib_stat[3] = (calib_stat[4] & 0xc0) >> 6;
    calib_stat[2] = (calib_stat[4] & 0x30) >> 4;
    calib_stat[1] = (calib_stat[4] & 0x0c) >> 2;
    calib_stat[0] = (calib_stat[4] & 0x03);

    calibration_status = (imu_BNO055.calib & 0xc0) >> 6; //upper 2 MSB's denoting calibration status

    //if((calib_stat[4] >= 193) && (imu_stabilized[1] == 0)) {
    if(imu_stabilized[1] == 0) { //for imu only mode
        if(imu_stabilized[0] == 0)
            imu_stabilized[0] = 1;
            
        if( ((generalFunctions::abs_f(imu_BNO055.lia.x) - initialAcc[0])* 9.81*1000 <= 5.0) && ((generalFunctions::abs_f(imu_BNO055.lia.y) - initialAcc[1])* 9.81*1000 <= 5.0) && ((generalFunctions::abs_f(imu_BNO055.lia.z) - initialAcc[2])* 9.81*1000 <= 5.0)) {

            unstable_readings--;
        }

        initialAcc[0] = generalFunctions::abs_f(imu_BNO055.lia.x);
        initialAcc[1] = generalFunctions::abs_f(imu_BNO055.lia.y);
        initialAcc[2] = generalFunctions::abs_f(imu_BNO055.lia.z);

        if(unstable_readings <= 1) {
            //imu_initialAngles[0] = imu_BNO055.euler.pitch;
            //imu_initialAngles[1] = imu_BNO055.euler.roll;
            imu_initialAngles[2] = 2*RAD_TO_DEG(M_PI) - imu_BNO055.euler.yaw;

            imu_initialAngles[0] = 0;
            imu_initialAngles[1] = 0;

            imu_stabilized[1] =1;

            bno_timer.reset();
        }
    }

    posePRY[0] = imu_BNO055.euler.pitch - imu_initialAngles[0] - PoseCorrection[0];
    posePRY[1] = imu_BNO055.euler.roll - imu_initialAngles[1] - PoseCorrection[1];
    //posePRY[2] = -1.0*(imu_BNO055.euler.yaw - imu_initialAngles[0]- DRIFT_CORRECTION(imuTime_s[1])) - PoseCorrection[2];
    //posePRY[2] = -1.0*(imu_BNO055.euler.yaw - imu_initialAngles[0]) - PoseCorrection[2];
    posePRY[2] = (2*RAD_TO_DEG(M_PI) - imu_BNO055.euler.yaw) - imu_initialAngles[2] - PoseCorrection[2];

    //convert angles to 0 to 2pi range

    for(int i=0; i<3; i++) {
        if(posePRY[i] > 2*RAD_TO_DEG(M_PI))
            posePRY[i] -= 2*RAD_TO_DEG(M_PI);
        if(posePRY[i] < 0.0)
            posePRY[i] += 2*RAD_TO_DEG(M_PI);
    }


    //moving window average of pose angle
    for(int i=0; i<3; i++) {
        movWindow_Pose[i][movWindow_index_Pose] = posePRY[i];
        Pose[i] = generalFunctions::moving_window(movWindow_Pose[i], movWindow_len_Pose);
    }

    /** Todo: offset->filter quaternion, use this quaternion to generate gravity vector, and consequently aaReal and aaWorld  **/

    Quat[0]= imu_BNO055.quat.w;
    Quat[1]= imu_BNO055.quat.x;
    Quat[2]= imu_BNO055.quat.y;
    Quat[3]= imu_BNO055.quat.z;

    //moving window average of gyro velocities
    movWindow_AngVel[0][movWindow_index_GyroAcc] = imu_BNO055.gyro.x;
    movWindow_AngVel[1][movWindow_index_GyroAcc] = imu_BNO055.gyro.y;
    movWindow_AngVel[2][movWindow_index_GyroAcc] = imu_BNO055.gyro.z;

    movWindow_LinAcc[0][movWindow_index_GyroAcc] = imu_BNO055.lia.x * 9.81 * 1000; //convert to mm/s2
    movWindow_LinAcc[1][movWindow_index_GyroAcc] = imu_BNO055.lia.y * 9.81 * 1000; //convert to mm/s2
    movWindow_LinAcc[2][movWindow_index_GyroAcc] = imu_BNO055.lia.z * 9.81 * 1000; //convert to mm/s2

    for(int i=0; i<3; i++) {
        AngVel[i] = generalFunctions::moving_window(movWindow_AngVel[i]/**/, movWindow_index_GyroAcc);    /****** PROBABLY WILL BREAK HERE!!! ******/
        LinAcc[i] = generalFunctions::moving_window(movWindow_LinAcc[i]/**/, movWindow_index_GyroAcc);    /****** PROBABLY WILL BREAK HERE!!! ******/
    }

    movWindow_index_Pose++;
    if(movWindow_index_Pose >= movWindow_len_Pose) movWindow_index_Pose=0;

    movWindow_index_GyroAcc++;
    if(movWindow_index_GyroAcc >= movWindow_len_GyroAcc) movWindow_index_GyroAcc=0;

    //debug_printf("Roll:%6.2fdeg, Pitch:%6.2fdeg, Yaw:%6.2fdeg\n", RAD_TO_DEG(imu_Data.roll), RAD_TO_DEG(imu_Data.pitch), RAD_TO_DEG(imu_Data.yaw));
}

void IMU_BNO055::getPose(float *pose[3])
{
    //pitch about X, roll about Y, yaw about Z 
    for(int i=0; i<3; i++)      
        *pose[i]=Pose[i];
}

void IMU_BNO055::getQuat(float *quaternion[4])
{
    for(int i=0; i<4; i++)
        *quaternion[i]=Quat[i];
}

void IMU_BNO055::getAngVel(float *angVelocity[3])
{
    for(int i=0; i<3; i++)
        *angVelocity[i]=AngVel[i];
}

void IMU_BNO055::getLinAcc(float *linAcc[3])
{
    for(int i=0; i<3; i++)
        *linAcc[i]=LinAcc[i];
}

float IMU_BNO055::getTime()
{
    return(imuTime_s[1]);
}

void IMU_BNO055::setPose(float pose_in[3])
{
    IMU_BNO055::imuUpdate(); //refresh imu before updating Pose
    
    //pitch about X, roll about Y, yaw about Z
    for(int i=0; i<3; i++)   
        PoseCorrection[i]=(Pose[i] - pose_in[i]);
}

// void IMU_BNO055::readCalibrationData()
// {
//     //imu_BNO055.get_calib();
//     imu_BNO055.read_calibration_data();
    
//     for(int i=0; i<22; i++)
//         calibration_regs[i]=imu_BNO055.calibration[i];
        
//     offset_acc[0] = (calibration_regs[1] + calibration_regs[0]);
//     offset_acc[1] = (calibration_regs[3] + calibration_regs[2]);
//     offset_acc[2] = (calibration_regs[5] + calibration_regs[4]);
    
//     offset_mag[0] = (calibration_regs[7] + calibration_regs[6]);
//     offset_mag[1] = (calibration_regs[9] + calibration_regs[8]);
//     offset_mag[2] = (calibration_regs[11] + calibration_regs[10]);
    
//     offset_gyr[0] = (calibration_regs[13] + calibration_regs[12]);
//     offset_gyr[1] = (calibration_regs[15] + calibration_regs[14]);
//     offset_gyr[2] = (calibration_regs[17] + calibration_regs[16]);
    
//     radius_acc = (calibration_regs[19] + calibration_regs[18]);
//     radius_mag = (calibration_regs[21] + calibration_regs[20]);    
// }

void IMU_BNO055::writeCalibrationData()
{
    //acc
    calibration_regs[0] = offset_acc[0] & 0b00001111;
    calibration_regs[1] = offset_acc[0] & 0b11110000;
    
    calibration_regs[2] = offset_acc[1] & 0b00001111;
    calibration_regs[3] = offset_acc[1] & 0b11110000;
    
    calibration_regs[4] = offset_acc[2] & 0b00001111;
    calibration_regs[5] = offset_acc[2] & 0b11110000;
    
    //mag
    calibration_regs[6] = offset_mag[0] & 0b00001111;
    calibration_regs[7] = offset_mag[0] & 0b11110000;
    
    calibration_regs[8] = offset_mag[1] & 0b00001111;
    calibration_regs[9] = offset_mag[1] & 0b11110000;
    
    calibration_regs[10] = offset_mag[2] & 0b00001111;
    calibration_regs[11] = offset_mag[2] & 0b11110000;
    
    //gyro
    calibration_regs[12] = offset_gyr[0] & 0b00001111;
    calibration_regs[13] = offset_gyr[0] & 0b11110000;
    
    calibration_regs[14] = offset_gyr[1] & 0b00001111;
    calibration_regs[15] = offset_gyr[1] & 0b11110000;
    
    calibration_regs[16] = offset_gyr[2] & 0b00001111;
    calibration_regs[17] = offset_gyr[2] & 0b11110000;
    
    //acc_radius
    calibration_regs[18] = radius_acc & 0b00001111;
    calibration_regs[19] = radius_acc & 0b11110000;
    
    //mag_radius
    calibration_regs[20] = radius_mag & 0b00001111;
    calibration_regs[21] = radius_mag & 0b11110000;
    
    for(int i=0; i<22; i++)
        imu_BNO055.calibration[i] = calibration_regs[i];
    
    imu_BNO055.write_calibration_data();
}

/***  ***/