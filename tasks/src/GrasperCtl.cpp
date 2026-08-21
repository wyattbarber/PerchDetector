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


using namespace std::chrono_literals;


/*=== ADC config register definitions (copied from Adafruit code) ===*/
#define ADS1X15_REG_POINTER_MASK (0x03)      ///< Point mask
#define ADS1X15_REG_POINTER_CONVERT (0x00)   ///< Conversion
#define ADS1X15_REG_POINTER_CONFIG (0x01)    ///< Configuration
#define ADS1X15_REG_POINTER_LOWTHRESH (0x02) ///< Low threshold
#define ADS1X15_REG_POINTER_HITHRESH (0x03)  ///< High threshold

#define ADS1X15_REG_CONFIG_OS_MASK (0x8000) ///< OS Mask
#define ADS1X15_REG_CONFIG_OS_SINGLE  (0x8000) ///< Write: Set to start a single-conversion
#define ADS1X15_REG_CONFIG_OS_BUSY    (0x0000) ///< Read: Bit = 0 when conversion is in progress
#define ADS1X15_REG_CONFIG_OS_NOTBUSY (0x8000) ///< Read: Bit = 1 when device is not performing a conversion

#define ADS1X15_REG_CONFIG_MUX_MASK (0x7000) ///< Mux Mask
#define ADS1X15_REG_CONFIG_MUX_DIFF_0_1 (0x0000) ///< Differential P = AIN0, N = AIN1 (default)
#define ADS1X15_REG_CONFIG_MUX_DIFF_0_3 (0x1000) ///< Differential P = AIN0, N = AIN3
#define ADS1X15_REG_CONFIG_MUX_DIFF_1_3 (0x2000) ///< Differential P = AIN1, N = AIN3
#define ADS1X15_REG_CONFIG_MUX_DIFF_2_3 (0x3000) ///< Differential P = AIN2, N = AIN3
#define ADS1X15_REG_CONFIG_MUX_SINGLE_0 (0x4000) ///< Single-ended AIN0
#define ADS1X15_REG_CONFIG_MUX_SINGLE_1 (0x5000) ///< Single-ended AIN1
#define ADS1X15_REG_CONFIG_MUX_SINGLE_2 (0x6000) ///< Single-ended AIN2
#define ADS1X15_REG_CONFIG_MUX_SINGLE_3 (0x7000) ///< Single-ended AIN3

constexpr uint16_t MUX_BY_CHANNEL[] = {
    ADS1X15_REG_CONFIG_MUX_SINGLE_0, ///< Single-ended AIN0
    ADS1X15_REG_CONFIG_MUX_SINGLE_1, ///< Single-ended AIN1
    ADS1X15_REG_CONFIG_MUX_SINGLE_2, ///< Single-ended AIN2
    ADS1X15_REG_CONFIG_MUX_SINGLE_3  ///< Single-ended AIN3
}; ///< MUX config by channel

#define ADS1X15_REG_CONFIG_PGA_MASK (0x0E00)   ///< PGA Mask
#define ADS1X15_REG_CONFIG_PGA_6_144V (0x0000) ///< +/-6.144V range = Gain 2/3
#define ADS1X15_REG_CONFIG_PGA_4_096V (0x0200) ///< +/-4.096V range = Gain 1
#define ADS1X15_REG_CONFIG_PGA_2_048V (0x0400) ///< +/-2.048V range = Gain 2 (default)
#define ADS1X15_REG_CONFIG_PGA_1_024V (0x0600) ///< +/-1.024V range = Gain 4
#define ADS1X15_REG_CONFIG_PGA_0_512V (0x0800) ///< +/-0.512V range = Gain 8
#define ADS1X15_REG_CONFIG_PGA_0_256V (0x0A00) ///< +/-0.256V range = Gain 16

#define ADS1X15_REG_CONFIG_MODE_MASK (0x0100)   ///< Mode Mask
#define ADS1X15_REG_CONFIG_MODE_CONTIN (0x0000) ///< Continuous conversion mode
#define ADS1X15_REG_CONFIG_MODE_SINGLE (0x0100) ///< Power-down single-shot mode (default)

#define ADS1X15_REG_CONFIG_RATE_MASK (0x00E0) ///< Data Rate Mask

#define ADS1X15_REG_CONFIG_CMODE_MASK (0x0010) ///< CMode Mask
#define ADS1X15_REG_CONFIG_CMODE_TRAD   (0x0000) ///< Traditional comparator with hysteresis (default)
#define ADS1X15_REG_CONFIG_CMODE_WINDOW (0x0010) ///< Window comparator

#define ADS1X15_REG_CONFIG_CPOL_MASK (0x0008) ///< CPol Mask
#define ADS1X15_REG_CONFIG_CPOL_ACTVLOW (0x0000) ///< ALERT/RDY pin is low when active (default)
#define ADS1X15_REG_CONFIG_CPOL_ACTVHI (0x0008) ///< ALERT/RDY pin is high when active

#define ADS1X15_REG_CONFIG_CLAT_MASK (0x0004) ///< Determines if ALERT/RDY pin latches once asserted
#define ADS1X15_REG_CONFIG_CLAT_NONLAT  (0x0000) ///< Non-latching comparator (default)
#define ADS1X15_REG_CONFIG_CLAT_LATCH (0x0004) ///< Latching comparator

#define ADS1X15_REG_CONFIG_CQUE_MASK (0x0003) ///< CQue Mask
#define ADS1X15_REG_CONFIG_CQUE_1CONV (0x0000) ///< Assert ALERT/RDY after one conversions
#define ADS1X15_REG_CONFIG_CQUE_2CONV (0x0001) ///< Assert ALERT/RDY after two conversions
#define ADS1X15_REG_CONFIG_CQUE_4CONV (0x0002) ///< Assert ALERT/RDY after four conversions
#define ADS1X15_REG_CONFIG_CQUE_NONE  (0x0003) ///< Disable the comparator and put ALERT/RDY in high state (default)

#define RATE_ADS1015_128SPS (0x0000)  ///< 128 samples per second
#define RATE_ADS1015_250SPS (0x0020)  ///< 250 samples per second
#define RATE_ADS1015_490SPS (0x0040)  ///< 490 samples per second
#define RATE_ADS1015_920SPS (0x0060)  ///< 920 samples per second
#define RATE_ADS1015_1600SPS (0x0080) ///< 1600 samples per second (default)
#define RATE_ADS1015_2400SPS (0x00A0) ///< 2400 samples per second
#define RATE_ADS1015_3300SPS (0x00C0) ///< 3300 samples per second
/*=========================================================================*/


bool GrasperController::start_impl()
{
    // Initialize i2c
    if ((i2c_file = open(i2c_dev, O_RDWR)) < 0) {
        error("Failed to open i2c bus ", i2c_dev, ": ", std::strerror(errno));
        return false;
    }
    if (ioctl(i2c_file, I2C_SLAVE, adc_addr) < 0) {
        error("Failed to communicate with device ", adc_addr, ": ", std::strerror(errno));
        return false;
    }
    // Initialize GPIO
    if(wiringPiSetup() == -1)
    {
        error("GPIO setup failed.");
        return false;
    }
    pinMode(servo_0_pin, PWM_OUTPUT);
    pinMode(servo_1_pin, PWM_OUTPUT);
    pinMode(servo_2_pin, PWM_OUTPUT);
    pwmSetMode(PWM_MODE_MS);
    pwmSetClock(384);   // Sets a 50 kHz clock tick rate (20 microseconds per tick)
    pwmSetRange(1000);  // 1000 ticks * 20 microseconds = 20,000 microseconds (20ms/50Hz)


    return true;
}
    

void GrasperController::stop_impl()
{
    close(i2c_file);
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
                state = 3;
            }
            break;
        }
        case 3: // Latch grasper
        {
            pwm_write(servo_2_pin, 1250);
            state = 4;
            cmd_grasp = false;
            break;
        }
        case 4: // Grasped
        {
            if(cmd_release)
            {
                state = 5;
            }
            break;
        }
        case 5: // Unlatch grasper
        {
            pwm_write(servo_2_pin, 1750);
            state = 6;
            break;
        }
        case 6: // Open grasper
        {
            servo_1.set_goal(false);
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
}


int16_t GrasperController::read_adc_channel(uint8_t chn) 
// Mostly copied from Adafruits arduino interface for this sensor, 
// https://github.com/adafruit/Adafruit_ADS1X15/blob/master/Adafruit_ADS1X15.cpp
{
    // Set config register to get one reading
    uint16_t config = 
      ADS1X15_REG_CONFIG_CQUE_1CONV |
      ADS1X15_REG_CONFIG_CLAT_NONLAT | 
      ADS1X15_REG_CONFIG_CPOL_ACTVLOW |
      ADS1X15_REG_CONFIG_CMODE_TRAD |
      ADS1X15_REG_CONFIG_MODE_SINGLE |
      ADS1X15_REG_CONFIG_PGA_4_096V |
      ADS1X15_REG_CONFIG_OS_SINGLE |
      MUX_BY_CHANNEL[chn] |
      RATE_ADS1015_1600SPS;

    const char config_buf[] = {ADS1X15_REG_POINTER_CONFIG, (config & 0xFF00) >> 8, config & 0x00FF};
    if (write(i2c_file, config_buf, 3) != 3) {
        error("Failed to write to config register: ", std::strerror(errno));
        return -1;
    }

    // Wait for conversion to complete
    bool complete = false;
    do {
        struct i2c_rdwr_ioctl_data packets;
        struct i2c_msg messages[2];
        const char check_buf = {ADS1X15_REG_POINTER_CONFIG};
        uint16_t res;
        messages[0].addr  = adc_addr;
        messages[0].flags = 0;              
        messages[0].len   = 1;              
        messages[0].buf   = (unsigned char*)&check_buf;
        messages[1].addr  = adc_addr;
        messages[1].flags = I2C_M_RD;       
        messages[1].len   = 2;              
        messages[1].buf   = (unsigned char*)&res;
        packets.msgs  = messages;
        packets.nmsgs = 2;
        if (ioctl(i2c_file, I2C_RDWR, &packets) < 0) {
            error("Failed to check conversion completion: ", std::strerror(errno));
            return -1;
        }
        complete = (res & 0x8000) != 0;
    } while(!complete);

    // Read result
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];
    const char check_buf = {ADS1X15_REG_POINTER_CONVERT};
    int16_t res;
    messages[0].addr  = adc_addr;
    messages[0].flags = 0;              
    messages[0].len   = 1;              
    messages[0].buf   = (unsigned char*)&check_buf;
    messages[1].addr  = adc_addr;
    messages[1].flags = I2C_M_RD;       
    messages[1].len   = 2;              
    messages[1].buf   = (unsigned char*)&res;
    packets.msgs  = messages;
    packets.nmsgs = 2;
    if (ioctl(i2c_file, I2C_RDWR, &packets) < 0) {
        error("Failed to check conversion result: ", std::strerror(errno));
        return -1;
    }

    info("Read ADC value ", res >> 4, " from channel ", static_vast<unsigned>(chn));
    
    return res >> 4;
}


void GrasperController::pwm_write(uint8_t chn, uint16_t pulse_width_ms)
{
    pwmWrite(1, pulse_width_ms * 1000u / 20u); 
}


void GrasperController::ServoWinder::step(GrasperController* parent)
{
    switch(state)
    {
        case 0: // Unwound
        {
            if (goal_wind)
            {
                state = 1;
            }
            break;
        }
        case 1: // Winding
        {
            ++wind_count;
            if ((static_cast<float>(parent->read_adc_channel(adc_chn)) * 0.003) > current_lim)
            {
                state = 2;
            }
            break;
        }
        case 2: // Wound
        {
            if (!goal_wind)
            {
                state = 3;
            }
            break;
        }
        case 3: // Unwinding
        {
            --wind_count;
            if (wind_count <= 0)
            {
                wind_count = 0;
                state = 0;
            }
            break;
        }
        default: state = 0; break;
    }
}