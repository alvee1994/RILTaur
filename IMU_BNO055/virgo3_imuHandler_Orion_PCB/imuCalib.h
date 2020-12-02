#ifndef imuCalib_H
#define imuCalib_H

//** IMU offsets ********************************
#define MPU6050_Offset_Ax 150
#define MPU6050_Offset_Ay -350
#define MPU6050_Offset_Az 1000
#define MPU6050_Offset_Gx -110
#define MPU6050_Offset_Gy 5
#define MPU6050_Offset_Gz 0

//** IMU yaw drift correction polynomial ********
#define kp1  2.207/(1000*1000*10)
#define kp2 -7.921/1000
#define kp3 -0.1917
#define eq_correction  0.01

#endif