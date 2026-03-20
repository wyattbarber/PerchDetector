#pragma once

#include "DataSource.hpp"
#include "vl53l8cx_api.h"


FWD_DECL_DATA_SOURCE(VL53L8CX, VL53L8CX_ResultsData)

/** Manage communication with a VL53L8CX lidar sensor.


*/
class VL53L8CX : public DataSource<VL53L8CX>
{
public:
    /** Create a new sensor manager task 
    
    @param name Task name
    @param path Path to the SPI device for the sensor
    */
    VL53L8CX(const char* name, const char* path) : 
        DataSource<VL53L8CX>(name, {}),
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

    std::vector<size_t> dims(){ return {1}; }

protected:
    const char* path;
    VL53L8CX_Configuration device;
    VL53L8CX_ResultsData data;
};


/** Helper to format lidar data as single plane image display.

*/
typedef struct lidar_display_conv
{ 
    using conversion_type = uint16_t[64];
    static std::vector<size_t> dims() { return {8, 8}; }
    static void eval(void* dst, const VL53L8CX::value_type& src) { memcpy(dst, (const void*)&src.distance_mm, sizeof(conversion_type)); } 
} lidar_display_conv;
