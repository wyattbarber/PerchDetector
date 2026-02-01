#include <iostream>

#include "STSW_IMG040/Platform/platform.h"
#include "STSW_IMG040/VL53L8CX_ULD_API/inc/vl53l8cx_api.h"

VL53L8CX_Configuration dev;

int main(int argc, char** argv)
{
    open_VL53L8CX("/dev/spidev0.0", &dev.platform);

    auto res = vl53l8cx_init(&dev);
    std::cout << "Initialization result: " << (int)res << std::endl;
    
    uint8_t alive = 0;
    res = vl53l8cx_is_alive(&dev, &alive);
    std::cout << "Is alive result: " << (int)res << ", " << (int)alive << std::endl;

    close_VL53L8CX(&dev.platform);

    return 0;
}