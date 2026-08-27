#pragma once

#include "Task.hpp"
#include <iostream>
#include <vector>
#include <atomic>
#include <Adafruit_ADS1X15.hpp>


/** Implements I2C an GPIO controlls for the grasper.

*/
class GrasperController : public TaskBase<GrasperController>
{
public:
    GrasperController(const char* name, const char* dev, const std::string& settings) : 
        TaskBase(name, {}),
        adc(dev),
        settings(settings + "/grasp_ctrl.json"),
        servo_ena_pin(27),
        servo_0_pin(18),
        servo_1_pin(19),
        servo_2_pin(26),
        servo_0(servo_0_pin, 0),
        servo_1(servo_1_pin, 1),
        servo_2(servo_2_pin, 2)
    {
        state = 0;
        cmd_grasp = false;
        cmd_release = false;
        servo_enable_count = 0;
        manual_override = false;

        declare_cli_command("close", [](Task* task, std::istream& in, std::ostream& out, const std::vector<std::string>& args){
            out << "Closing grasper..." << std::endl;
            static_cast<GrasperController*>(task)->grasp();
            out << "Closed grasper." << std::endl;
        });
        declare_cli_command("open", [](Task* task, std::istream& in, std::ostream& out, const std::vector<std::string>& args){
            out << "Opening grasper..." << std::endl;
            static_cast<GrasperController*>(task)->release();
            out << "Opened grasper." << std::endl;
        });
        
        declare_cli_command("wind", [](Task* task, std::istream& in, std::ostream& out, const std::vector<std::string>& args){
            if(args.size() < 1)
            {
                out << "Servo number is required" << std::endl;
                return;
            }
            ServoWinder* servo;
            if(args[0] == "0")
            {
                servo = &((GrasperController*)task)->servo_0;
            }
            else if(args[0] == "1")
            {
                servo = &((GrasperController*)task)->servo_1;
            }
            else if(args[0] == "2")
            {
                servo = &((GrasperController*)task)->servo_2;
            }
            else
            {
                out << "Unknown servo number." << std::endl;
                return;
            }
            servo->set_goal(true);
        });
        declare_cli_command("unwind", [](Task* task, std::istream& in, std::ostream& out, const std::vector<std::string>& args){
            if(args.size() < 1)
            {
                out << "Servo number is required" << std::endl;
                return;
            }
            ServoWinder* servo;
            if(args[0] == "0")
            {
                servo = &((GrasperController*)task)->servo_0;
            }
            else if(args[0] == "1")
            {
                servo = &((GrasperController*)task)->servo_1;
            }
            else if(args[0] == "2")
            {
                servo = &((GrasperController*)task)->servo_2;
            }
            else
            {
                out << "Unknown servo number." << std::endl;
                return;
            }
            servo->set_goal(false);
        });
        declare_cli_command("power", [](Task* task, std::istream& in, std::ostream& out, const std::vector<std::string>& args){
            if(args.size() < 1)
            {
                out << "State is required" << std::endl;
                return;
            }
            bool on = (args[0] == "1") || (args[0] == "on");
            if(on)
            {
                ((GrasperController*)task)->acquire_servo_enable();
                out << "Enabled servo power." << std::endl;
            }
            else
            {
                ((GrasperController*)task)->release_servo_enable();
                out << "Disabled servo power." << std::endl;
            }
        });
        declare_cli_command("override", [](Task* task, std::istream& in, std::ostream& out, const std::vector<std::string>& args){
            bool on = true;
            if(args.size() >= 1)
            {
                if((args[0] == "off") || (args[0] == "0"))
                {
                    on = false;
                }
            }
            out << "Manual override " << (on ? "activated" : "deactivated") << std::endl;
            ((GrasperController*)task)->manual_override = on;
        });
        declare_cli_command("servo", [](Task* task, std::istream& in, std::ostream& out, const std::vector<std::string>& args){
            if(args.size() < 2)
            {
                out << "Servo number and PWM level are required" << std::endl;
                return;
            }

            ServoWinder* servo;
            if(args[0] == "0")
            {
                servo = &((GrasperController*)task)->servo_0;
            }
            else if(args[0] == "1")
            {
                servo = &((GrasperController*)task)->servo_1;
            }
            else if(args[0] == "2")
            {
                servo = &((GrasperController*)task)->servo_2;
            }
            else
            {
                out << "Unknown servo number." << std::endl;
                return;
            }
            unsigned level = std::stoi(args[1]);
            ((GrasperController*)task)->pwm_write(servo->pin, level);
        });
    }

    /** Setup i2c bus and configure IO 
    */
    bool start_impl();
    
    /** Close the i2c busStops the camera 
    */
    void stop_impl();

    /** Update i2c comms and set IO values.
    */
    void step();

    /** Command grasper to close.

    Blocks until motion is complete.
    Should be called from a separate thread than this task is running in.
    */
    void grasp();

    /** Command grasper to open.

    Blocks until motion is complete.
    Should be called from a separate thread than this task is running in.
    */
    void release();

    void acquire_servo_enable();
    void release_servo_enable();

private:

    // ADC i2c params
    Adafruit_ADS1015 adc;
    static constexpr char adc_addr = 0x48;
    static constexpr int16_t adc_max = 0x07FF;
    static constexpr float adc_v_max = 4.096;
    static constexpr float curr_meas_res = 1.0;

    const std::string settings;

    bool manual_override;
    
    const uint8_t servo_ena_pin;
    const uint8_t servo_0_pin;
    const uint8_t servo_1_pin;
    const uint8_t servo_2_pin;

    std::atomic<unsigned> servo_enable_count;

    uint8_t state;
    bool cmd_grasp, cmd_release;

    float current_convert(uint8_t chn);

    void pwm_write(uint8_t chn, uint16_t pulse_width_ms);

    class ServoWinder
    {
    public:
        ServoWinder(uint8_t servo_pin, uint8_t adc_chn) :
            pin(servo_pin),
            adc_chn(adc_chn),
            state(0),
            goal_wind(false),
            wind_count(0)
        {
            current_lim = 0.3;
            direction = false;
            speed = 500;
        }
        void step(GrasperController* parent);
        void set_goal(bool wind){ goal_wind = wind; }
        bool wound(){ return state == 2; }
        bool unwound(){ return state == 0; }
        
        float current_lim;
        bool direction;
        unsigned speed;
        const uint8_t pin;
        const uint8_t adc_chn;
        uint8_t state;
        bool goal_wind;
        uint16_t wind_count;
    };

    ServoWinder servo_0;
    ServoWinder servo_1;
    ServoWinder servo_2;
};