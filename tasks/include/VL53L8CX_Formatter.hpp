#pragma once

#include "DataSource.hpp"
#include "VL53L8CX.hpp"


FWD_DECL_DATA_SOURCE(VL53L8CX_Formatter, uint16_t[64])

/** Formats VL53L8CX data as 1 channel arrays.


*/
class VL53L8CX_Formatter : public DataSource<VL53L8CX_Formatter>
{
public:
    /** Create a new sensor manager task 
    
    @param name Task name
    @param path Path to the SPI device for the sensor
    */
    VL53L8CX_Formatter(const char* name, std::shared_ptr<VL53L8CX> task) : 
        DataSource<VL53L8CX_Formatter>(name, {task}),
        task(task)
    {
    }

    bool start_impl()
    {
        if(!task->is_alive()) return false;
        latest = task->acquire();
        return true;
    }

    void stop_impl(){}

    void step()
    {        
        if(is_alive())
        {
            if(!latest->stale) return;
            latest = task->acquire();
            swap_data(*reinterpret_cast<value_type*>(latest->data.distance_mm)); // Take only the first plane of closest detections as output
            tick();
        }
    }

    std::vector<size_t> dims(){ return {8, 8}; }

protected:
    std::shared_ptr<VL53L8CX> task;
    typename VL53L8CX::update_ptr_type latest;
};