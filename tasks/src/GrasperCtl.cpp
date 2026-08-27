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
#include <thread>
#include <json_loader.hpp>


using namespace std::chrono_literals;


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
    pinMode(servo_0_pin, PWM_OUTPUT);
    pinMode(servo_1_pin, PWM_OUTPUT);
    pinMode(servo_2_pin, PWM_OUTPUT);
    pwmSetMode(PWM_MODE_MS);
    pwmSetClock(192);   
    pwmSetRange(2000); 
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
    servo_0.state = 0;
    servo_1.state = 0;
    servo_2.state = 0;
    servo_enable_count = 0;
    digitalWrite(servo_ena_pin, LOW);
}


void GrasperController::grasp()
{
    cmd_grasp = true;
    while(cmd_grasp)
    {
        std::this_thread::sleep_for(50ms);
    }
}


void GrasperController::release()
{
    cmd_release = true;
    while(cmd_release)
    {
        std::this_thread::sleep_for(50ms);
    }
}

void GrasperController::step()
{
    servo_0.step(this);
    servo_1.step(this);
    servo_2.step(this);

    switch(state)
    {
        case 0: // Released
        {
            if(cmd_grasp)
            {
                servo_0.set_goal(true);
                state = 1;
            }
            break;
        }
        case 1: // Raise grasper
        {
            if(servo_0.wound())
            {
                servo_1.set_goal(true);
                state = 2;
            }
            break;
        }
        case 2: // Close grasper
        {
            if(servo_1.wound())
            {
                servo_2.set_goal(true);
                state = 3;
            }
            break;
        }
        case 3: // Latch grasper
        {
            if(servo_2.wound())
            {
                state = 4;
                cmd_grasp = false;
            }
            break;
        }
        case 4: // Grasped
        {
            if(cmd_release)
            {
                servo_2.set_goal(false);
                state = 5;
            }
            break;
        }
        case 5: // Unlatch grasper
        {
            if(servo_2.unwound())
            {                
                servo_1.set_goal(false);
                state = 6;
            }
            break;
        }
        case 6: // Open grasper
        {
            if(servo_1.unwound())
            {
                servo_0.set_goal(false);
                state = 7;
            }
            break;
        }
        case 7: // Lower grasper
        {   
            if(servo_0.unwound())
            {
                cmd_release = false;
                state = 0;
            }
            break;
        }
        default: state = 0; break;
    }
    tick();
}


float GrasperController::current_convert(uint8_t chn)
{
    auto v = adc.computeVolts(adc.readADC_SingleEnded(chn));
    auto i = v / curr_meas_res;
    return i;
}


void GrasperController::pwm_write(uint8_t chn, uint16_t pulse_width_ms)
{
    pwmWrite(chn, pulse_width_ms / 10u); 
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
        case 0: // Unwound
        {
            parent->pwm_write(pin, 1500);
            if (goal_wind)
            {
                state = 1;
                parent->info("Winding servo on pin ", (int)pin);
                parent->acquire_servo_enable();
            }
            break;
        }
        case 1: // Winding
        {
            parent->pwm_write(pin, 1500 + ((direction ? 1 : -1) * speed));
            ++wind_count;
            auto i = parent->current_convert(adc_chn);
            parent->info("Servo on pin ", (int)pin, " measured current of ", i, 'A');
            if (i > current_lim)
            {
                state = 2;
                parent->release_servo_enable();
                parent->info("Servo on pin ", (int)pin, " wound completely");
            }
            break;
        }
        case 2: // Wound
        {
            parent->pwm_write(pin, 1500);
            if (!goal_wind)
            {
                parent->info("Unwinding servo on pin ", (int)pin);
                parent->acquire_servo_enable();
                state = 3;
            }
            break;
        }
        case 3: // Unwinding
        {
            parent->pwm_write(pin, 1500 - ((direction ? 1 : -1) * speed));
            --wind_count;
            if (wind_count <= 0)
            {
                parent->release_servo_enable();
                parent->info("Servo on pin ", (int)pin, " unwound completely");
                wind_count = 0;
                state = 0;
            }
            break;
        }
        default: state = 0; break;
    }
}