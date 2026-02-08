#pragma once

#include "Task.hpp"
#include "vl53l8cx_api.h"


/** Manage communication with a VL53L8CX lidar sensor.


*/
class VL53L8CX : public Task
{
public:
    /** Create a new sensor manager task 
    
    @param name Task name
    @param path Path to the SPI device for the sensor
    */
    VL53L8CX(const char* name, const char* path) : 
        Task(name, {}),
        path(path)
    {
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
    const char* path;
    VL53L8CX_Configuration device;
    VL53L8CX_ResultsData data;
};