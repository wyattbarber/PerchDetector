#include "VL53L8CX.hpp"
#include <chrono>
#include <thread>
#include <opencv2/core.hpp>

using namespace std::chrono_literals;


bool VL53L8CX::start_impl()
{
    // Open the SPI interface
    open_VL53L8CX(path, &device.platform);

    // Test and configure the sensor
    uint8_t status, alive;

    status = vl53l8cx_is_alive(&device, &alive);
    if(!alive || status)
    {
        error("Failed to communicate with sensor, result ", (int)status, ", alive ", (int)alive);
        return false;
    }

    status = vl53l8cx_init(&device);
    if(status)
    {
        error("Failed to initialize sensor, result ", (int)status);
        return false;
    }

    info("Sensor initialization completed.");

    status = vl53l8cx_set_resolution(&device, VL53L8CX_RESOLUTION_8X8);
    if(status)
    {
        error("Failed to set resolution, result ", (int)status);
        return false;
    }

    status = vl53l8cx_set_target_order(&device, VL53L8CX_TARGET_ORDER_CLOSEST);    
    if(status)
    {
        error("Failed to set target order, result ", (int)status);
        return false;
    }

    status = vl53l8cx_start_ranging(&device);
    if(status)
    {
        error("Failed to begin ranging, result ", (int)status);
        return false;
    }

    info("Sensor configuration completed.");

    return true;
}


void VL53L8CX::stop_impl()
{
    // Shutdown measurements
    int status;

    status = vl53l8cx_stop_ranging(&device);
    if(status)
    {
        error("Failed to stop ranging, result ", (int)status, ". Will retry after 100ms.");
        std::this_thread::sleep_for(100ms);
        status = vl53l8cx_stop_ranging(&device);        
        if(status)
        {
            error("Failed to stop ranging, result ", (int)status, ". Not retrying.");
        }
    }
    
    // Close the SPI interface
    close_VL53L8CX(&device.platform);
}


void VL53L8CX::step()
{
    uint8_t status, ready;

    if(is_alive())
    {
		status = vl53l8cx_check_data_ready(&device, &ready);
        if(status)
        {
            warning("Failed to check for new data, result ", (int)status);
            return;
        }
   
        if(ready)
        {
			status = vl53l8cx_get_ranging_data(&device, &data);
            if(status)
            {
                warning("Failed to read new data, result ", (int)status);
                return;
            }

            update_ptr_type next = allocate_next();
            // Rotate data to match camera frame
            for(size_t i = 0; i < VL53L8CX_NB_TARGET_PER_ZONE; ++i)
            {
                // Source matrices
                cv::Mat src_dist(8, 8, CV_16UC1, (void*)(data.distance_mm + (64*i)));
                cv::Mat src_sigma(8, 8, CV_16UC1, (void*)(data.range_sigma_mm + (64*i)));
                cv::Mat src_status(8, 8, CV_8UC1, (void*)(data.target_status + (64*i)));
                // Destination matrices
                cv::Mat dst_dist(8, 8, CV_16UC1, (void*)(next->data.distance_mm + (64*i)));
                cv::Mat dst_sigma(8, 8, CV_16UC1, (void*)(next->data.range_sigma_mm + (64*i)));
                cv::Mat dst_status(8, 8, CV_8UC1, (void*)(next->data.target_status + (64*i)));
                // Rotate
                cv::rotate(src_dist, dst_dist, cv::ROTATE_90_CLOCKWISE);
                cv::rotate(src_sigma, dst_sigma, cv::ROTATE_90_CLOCKWISE);
                cv::rotate(src_status, dst_status, cv::ROTATE_90_CLOCKWISE);
            }
            // Number of detections is always one channel
            cv::Mat src_n_detect(8, 8, CV_8UC1, (void*)data.nb_target_detected);
            cv::Mat dst_n_detect(8, 8, CV_8UC1, (void*)next->data.nb_target_detected);
            cv::rotate(src_n_detect, dst_n_detect, cv::ROTATE_90_CLOCKWISE);

            swap_data();
            tick();
        }        
    }
}

