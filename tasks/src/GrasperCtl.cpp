#include "GrasperCtl.hpp"
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cstring>
#include <cerrno>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <thread>
#include <json_loader.hpp>


using namespace std::chrono_literals;


void _pwm_mode(unsigned pin)
{
    if((pin == 18) || (pin == 19))
    {
        pinMode(pin, PWM_OUTPUT);
    }
    else
    {
        softPwmCreate(pin, 0, 200);
    }
}


void _pwm_stop(unsigned pin)
{
    if((pin != 18) && (pin != 19))
    {
        softPwmStop(pin);
    }
}


bool GrasperController::start_impl()
{
    // Initialize i2c
    if (!adc.begin())
    {
        error("Failed to initialize ADC.");
        return false;
    }
    info("ADC initialized.");

    // Initialize GPIO
    if(wiringPiSetupGpio() == -1)
    {
        error("GPIO setup failed.");
        return false;
    }
    pinMode(servo_ena_pin, OUTPUT);
    _pwm_mode(servo_0_pin);
    _pwm_mode(servo_1_pin);
    _pwm_mode(servo_2_pin);
    pwmSetMode(PWM_MODE_MS);
    pwmSetClock(192);   
    pwmSetRange(2000); 
    pwm_write(servo_0_pin, 1500);
    pwm_write(servo_1_pin, 1500);
    pwm_write(servo_2_pin, 1500);
    info("GPIO initialized.");

    // Configure servo controllers
    if (!load_json_value_pairs(
            settings,
            std::make_tuple("servo_0"),
            "current_lim_ma", servo_0.current_lim,
            "reverse", servo_0.direction,
            "speed", servo_0.speed
        ))
    {
        error("Failed to load servo 0 settings");
        return false;
    }
    if (!load_json_value_pairs(
            settings,
            std::make_tuple("servo_1"),
            "current_lim_ma", servo_1.current_lim,
            "reverse", servo_1.direction,
            "speed", servo_1.speed
        ))
    {
        error("Failed to load servo 1 settings");
        return false;
    }
    if (!load_json_value_pairs(
            settings,
            std::make_tuple("servo_2"),
            "current_lim_ma", servo_2.current_lim,
            "reverse", servo_2.direction,
            "speed", servo_2.speed
        ))
    {
        error("Failed to load servo 2 settings");
        return false;
    }
    info("Parameters loaded.");

    return true;
}
    

void GrasperController::stop_impl()
{
    adc.end();
    servo_0.state = ServoWinder::REST;
    servo_0.request_close = false;
    servo_0.request_open = false;
    servo_1.state = ServoWinder::REST;
    servo_1.request_close = false;
    servo_1.request_open = false;
    servo_2.state = ServoWinder::REST;
    servo_2.request_close = false;
    servo_2.request_open = false;
    servo_enable_count = ServoWinder::REST;
    _pwm_stop(servo_0_pin);
    _pwm_stop(servo_1_pin);
    _pwm_stop(servo_2_pin);
    digitalWrite(servo_ena_pin, LOW);
}


void GrasperController::grasp()
{
    cmd_grasp = true;
    while(!grasp_done)
    {
        std::this_thread::sleep_for(50ms);
    }
}


void GrasperController::release()
{
    cmd_release = true;
    while(!release_done)
    {
        std::this_thread::sleep_for(50ms);
    }
}


void GrasperController::step()
{
    if(!manual_override)
    {
        servo_0.step(this);
        servo_1.step(this);
        servo_2.step(this);

        switch(state)
        {
            case REST:
            {
                if(cmd_grasp)
                { 
                    servo_0.set_goal(true);
                    servo_1.set_goal(true);
                    cmd_grasp = false;
                    grasp_done = false;
                    state = CLOSING; 
                }
                else if(cmd_release)
                {
                    servo_0.set_goal(false);
                    servo_1.set_goal(false);
                    cmd_release = false;
                    release_done = false;
                    state = OPENING;
                }
                break;
            }
            case CLOSING:
            {
                if (servo_0.closed() && servo_1.closed())
                {
                    state = CLOSED;
                }
                break;
            }
            case CLOSED:
            {
                grasp_done = true;
                state = REST;
                break;
            }
            case OPENING:
            {
                if (servo_0.opened() && servo_1.opened())
                {
                    state = OPENED;
                }
                break;
            }
            case OPENED:
            {
                release_done = true;
                state = REST;
                break;
            }
            default: state = REST; break;
        }
    }
    tick();
}


float GrasperController::current_convert(uint8_t chn)
{
    auto v = adc.computeVolts(adc.readADC_SingleEnded(chn));
    auto i = v / curr_meas_res;
    return i;
}


void GrasperController::pwm_write(uint8_t pin, uint16_t pulse_width_ms)
{
    if((pin == 18) || (pin == 19))
    {
        pwmWrite(pin, pulse_width_ms / 10u);
    }
    else
    {        
        softPwmWrite(pin, pulse_width_ms / 100u);
    }
}


void GrasperController::acquire_servo_enable()
{
    if(servo_enable_count == 0)
    {
        info("Initial servo enable request received, turning on power.");
        digitalWrite(servo_ena_pin, HIGH);
    }
    ++servo_enable_count;
}


void GrasperController::release_servo_enable()
{
    if(servo_enable_count >= 1)
    {
        --servo_enable_count;
    }
    if(servo_enable_count == 0)
    {
        servo_enable_count = 0;
        info("Last servo enable request release, turning off power.");
        digitalWrite(servo_ena_pin, LOW);
    }
}


void GrasperController::ServoWinder::step(GrasperController* parent)
{
    switch(state)
    {
        case REST:
        {
            if(request_close)
            { 
                parent->pwm_write(pin, 1500);
                parent->acquire_servo_enable();
                request_close = false;
                finished_close = false;
                state = CLOSING; 
            }
            else if(request_open)
            {
                parent->pwm_write(pin, 1500);
                parent->acquire_servo_enable();
                request_open = false;
                finished_open = false;
                state = OPENING;
            }
            break;
        }
        case CLOSING:
        {
            parent->pwm_write(pin, 1500 + ((direction ? 1 : -1) * speed));
            i = parent->current_convert(adc_chn);
            if (i > current_lim)
            {
                state = CLOSED;
            }
            break;
        }
        case CLOSED:
        {
            parent->release_servo_enable();
            finished_close = true;
            state = REST;
            break;
        }
        case OPENING:
        {
            parent->pwm_write(pin, 1500 - ((direction ? 1 : -1) * speed));
            i = parent->current_convert(adc_chn);
            if (i > current_lim)
            {
                state = OPENED;
            }
            break;
        }
        case OPENED:
        {
            parent->release_servo_enable();
            finished_open = true;
            state = REST;
            break;
        }
        default: state = REST; break;
    }
}