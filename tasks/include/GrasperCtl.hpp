#pragma once

#include "Task.hpp"




/** Implements I2C an GPIO controlls for the grasper.

*/
class GrasperController : public TaskBase<GrasperController>
{
public:
    GrasperController(const char* name, const char* dev) : 
        TaskBase(name, {}),
        i2c_dev(dev)
    {}

    /** Setup i2c bus and configure IO 
    */
    bool start_impl();
    
    /** Close the i2c busStops the camera 
    */
    void stop_impl();

    /** Update i2c comms and set IO values.
    */
    void step();


private:

    // ADC i2c params
    int i2c_file;
    const char* i2c_dev;
    static const char adc_addr = 0x48;
    static const int16_t adc_max = 0x07FF;
    static constexpr float adc_v_max = 4.096;


    int16_t read_adc_channel(uint8_t chn);
};