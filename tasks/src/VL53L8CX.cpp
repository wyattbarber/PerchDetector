#include "VL53L8CX.hpp"
#include <chrono>
#include <thread>
#include <opencv2/core.hpp>
#include "json/json.h"
#include <json_loader.hpp>
#include <string>

using namespace std::chrono_literals;


bool VL53L8CX::start_impl()
{
    uint8_t sharpener, rate;
    if(!load_json_value_pairs(
        settings,
        std::make_tuple(),
        "sharpener", sharpener,
        "max_rate_hz", rate
    ))
    {
        error("Failed to load lidar settings.");
        return false;
    }
    
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

    status = vl53l8cx_set_sharpener_percent(&device, sharpener);
    if(status)
    {
        error("Failed to set sharpener to ", sharpener, " percent, result ", (int)status);
        return false;
    }

    status = vl53l8cx_set_ranging_frequency_hz(&device, rate);
    if(status)
    {
        error("Failed to set maximum update rate to ", rate, " Hz, result ", (int)status);
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
        tick();

        update_ptr_type next = allocate_next();
        // Rotate data to match camera frame
        // Source matrices
        cv::Mat src_dist(8, 8, CV_16UC4, (void*)data.distance_mm);
        cv::Mat src_sigma(8, 8, CV_16UC4, (void*)data.range_sigma_mm);
        cv::Mat src_status(8, 8, CV_8UC4, (void*)data.target_status);
        cv::Mat src_n_detect(8, 8, CV_8UC1, (void*)data.nb_target_detected);
        // Destination matrices
        cv::Mat dst_dist(8, 8, CV_16UC4, (void*)next->data.distance_mm);
        cv::Mat dst_sigma(8, 8, CV_16UC4, (void*)next->data.range_sigma_mm);
        cv::Mat dst_status(8, 8, CV_8UC4, (void*)next->data.target_status);
        cv::Mat dst_n_detect(8, 8, CV_8UC1, (void*)next->data.nb_target_detected);
        // Rotate
        cv::rotate(src_dist, dst_dist, cv::ROTATE_90_CLOCKWISE);
        cv::rotate(src_sigma, dst_sigma, cv::ROTATE_90_CLOCKWISE);
        cv::rotate(src_status, dst_status, cv::ROTATE_90_CLOCKWISE);
        cv::rotate(src_n_detect, dst_n_detect, cv::ROTATE_90_CLOCKWISE);

        swap_data();
    }        
}



void VL53L8CX::data_to_json(update_ptr_const_type data, std::ofstream& file)
{
    Json::Value root;
    root["distance"] = Json::Value(Json::arrayValue);
    root["sigma"] = Json::Value(Json::arrayValue);
    root["status"] = Json::Value(Json::arrayValue);
    root["num_detections"] = Json::Value(Json::arrayValue);

    for(size_t i = 0; i < VL53L8CX_NB_TARGET_PER_ZONE; ++i)
    {
        root["distance"][(int)i] = Json::Value(Json::arrayValue);
        root["sigma"][(int)i] = Json::Value(Json::arrayValue);
        root["status"][(int)i] = Json::Value(Json::arrayValue);
        for(size_t j = 0; j < 64; ++j)
        {   
            root["distance"][(int)i][(int)j] = Json::Value(data->data.distance_mm[(j*4)+i]);
            root["sigma"][(int)i][(int)j] = Json::Value(data->data.range_sigma_mm[(j*4)+i]);
            root["status"][(int)i][(int)j] = Json::Value(data->data.target_status[(j*4)+i]);
        }
    }    
    
    for(size_t i = 0; i < 64; ++i)
    {   
        root["num_detections"][(int)i] = Json::Value(data->data.nb_target_detected[i]);
    }

    Json::StreamWriterBuilder wbuilder;
    file << Json::writeString(wbuilder, root).c_str();
}


void lidar_display_conv::eval(void* dst, const VL53L8CX::value_type& src) 
{ 
    for(size_t i = 0; i < 64; ++i)
    {
        reinterpret_cast<uint16_t*>(dst)[i] = src.distance_mm[i*4];
    }
} 