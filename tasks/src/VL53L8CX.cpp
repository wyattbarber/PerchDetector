#include "VL53L8CX.hpp"
#include <chrono>
#include <thread>


using namespace std::chrono_literals;


bool VL53L8CX::start_impl()
{
    // Open the SPI interface
    open_VL53L8CX(path, &device.platform);

    // Test and configure the sensor
    uint8_t status, alive;

    status = vl53l8cx_is_alive(&device, &alive);
    if(!alive || status)
    {
        error("Failed to communicate with sensor, result ", (int)status, ", alive ", (int)alive);
        return false;
    }

    status = vl53l8cx_init(&device);
    if(status)
    {
        error("Failed to initialize sensor, result ", (int)status);
        return false;
    }

    info("Sensor initialization completed.");

    status = vl53l8cx_start_ranging(&device);
    if(status)
    {
        error("Failed to begin ranging, result ", (int)status);
        return false;
    }

    return true;
}


void VL53L8CX::stop_impl()
{
    // Shutdown measurements
    int status;

    status = vl53l8cx_stop_ranging(&device);
    if(status)
    {
        error("Failed to stop ranging, result ", (int)status, ". Will retry after 100ms.");
        std::this_thread::sleep_for(100ms);
        status = vl53l8cx_stop_ranging(&device);        
        if(status)
        {
            error("Failed to stop ranging, result ", (int)status, ". Not retrying.");
        }
    }
    
    // Close the SPI interface
    close_VL53L8CX(&device.platform);
}


void VL53L8CX::step()
{
    uint8_t status, ready;

    if(is_alive())
    {
		status = vl53l8cx_check_data_ready(&device, &ready);
        if(status)
        {
            warning("Failed to check for new data, result ", (int)status);
            return;
        }
   
        if(ready)
        {
			vl53l8cx_get_ranging_data(&device, &data);
        }
    }
}

