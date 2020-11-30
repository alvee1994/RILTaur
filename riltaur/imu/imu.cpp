#include "imu.h"


// These options are typically called CPOL and CPHA.
// The BNO08X uses CPOL = 1 and CPHA = 1. In this configuration the clock idles high and data is captured on the
// rising edge of the clock:
// SPI imu(SPI_MOSI, SPI_MISO, SPI_SCK);

