#pragma once

#include "Task.hpp"
#include "vl53l8cx_api.h"


/** Manage communication with a VL53L8CX lidar sensor.


*/
class VL53L8CX : public Task
{
public:
    /** Create a new sensor manager task 
    
    Opens the file for the device, but does not begin SPI communication.
    The file will be closed when this object is deleted.

    @param name Task name
    @param dev Path to the SPI device for the sensor
    */
    VL53L8CX(const char* name, const char* dev) : 
        Task(name)
    {
        open_VL53L8CX(dev, &device.platform);
    }
    ~VL53L8CX()
    {
        close_VL53L8CX(&device.platform);
    }

    /** Checks communication and configures the sensor to start measurements.
    
    @return True if configuration and communication was succesfull.
    */
    bool start_impl();

    /** Stops communication.
    
    Doesn't close the device file, 
    that is only done when this object is deleted.
    */
    void stop_impl();

    /** Collects and stores a new measurement.
    
    */
    void step();

protected:
    VL53L8CX_Configuration device;
    VL53L8CX_ResultsData data;
};