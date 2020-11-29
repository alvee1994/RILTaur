#include "BufferedSerial.h"
#include "DigitalOut.h"
#include "PeripheralPins.h"
#include "PinNames.h"
#include "tmatrix.h"
#include <mbed.h>
#include <BNO080.h>
#include <SerialStream.h>
#include <USBSerial.h>

BufferedSerial debugPort(USBTX, USBRX);

// BNO080SPI::BNO080SPI(BufferedSerial *debugPort, PinName rstPin, PinName intPin, PinName wakePin, PinName misoPin,
// 					 PinName mosiPin, PinName sclkPin, PinName csPin, int spiSpeed):

BNO080SPI imu(&debugPort, D6, D3, PA_0, PB_4, PB_5, PB_3, D9, 3000000);
 
int main()
{
    
    printf("=======================started==========================\n");

    imu.begin();
    imu.enableReport(BNO080::TOTAL_ACCELERATION, 200);
 
    while (true)
    {
        wait_us(.001f * 1000);
        // poll the IMU for new data -- this returns true if any packets were received
        if(imu.updateData())
        {
            // now check for the specific type of data that was received (can be multiple at once)
            if (imu.hasNewData(BNO080::ROTATION))
            {
                // convert quaternion to Euler degrees and print
                printf("IMU Rotation Euler: ");
                TVector3 eulerRadians = imu.rotationVector.euler();
                TVector3 eulerDegrees = eulerRadians * (180.0 / M_PI);
                printf("%f, %f, %f", eulerDegrees[0], eulerDegrees[1], eulerDegrees[2]);
                printf("\n");
            }
            if (imu.hasNewData(BNO080::TOTAL_ACCELERATION))
            {
                // print the acceleration vector using its builtin print() method
                printf("IMU Total Acceleration: ");
                TVector3 totalAccel = imu.totalAcceleration;
                printf("%f, %f, %f", totalAccel[0], totalAccel[1], totalAccel[2]);
                printf("\n");
            }
        }
    }
 
}
 