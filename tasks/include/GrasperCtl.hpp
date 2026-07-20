#pragma once

#include "Task.hpp"




/** Implements I2C an GPIO controlls for the grasper.

*/
class GrasperController : public TaskBase<GrasperController>
{
public:
    GrasperController(const char* name, const char* dev) : 
        TaskBase(name, {}),
        i2c_dev(dev),
        servo_0_pin(18),
        servo_1_pin(19),
        servo_2_pin(34),
        servo_0(servo_0_pin),
        servo_1(servo_1_pin),
        servo_2(servo_2_pin)
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
    static constexpr char adc_addr = 0x48;
    static constexpr int16_t adc_max = 0x07FF;
    static constexpr float adc_v_max = 4.096;
    const uint8_t servo_0_pin;
    const uint8_t servo_1_pin;
    const uint8_t servo_2_pin;

    int16_t read_adc_channel(uint8_t chn);

    void pwm_write(uint8_t chn, uint16_t pulse_width_ms);

    class ServoWinder
    {
    public:
        ServoWinder(uint8_t pin) :
            pin(pin),
            state(0),
            goal_wind(false),
            wind_count(0)
        {}
        void step();
        void set_goal(bool wind);
    private:
        const uint8_t pin;
        uint8_t state;
        bool goal_wind;
        uint16_t wind_count;
    };

    ServoWinder servo_0;
    ServoWinder servo_1;
    ServoWinder servo_2;
};