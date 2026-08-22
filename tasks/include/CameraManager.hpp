#pragma once

#include "Task.hpp"
#include <libcamera/libcamera.h>
#include <memory>
#include <unordered_map>
#include <mutex>


/** Task for a camera manager.

Just wraps starting the manager and logging
libcamera events.
*/
class CameraManagerTask : public TaskBase<CameraManagerTask>
{
public:
    CameraManagerTask() : 
        TaskBase("_camera_manager", {})
    {        
        cm = std::make_unique<libcamera::CameraManager>();
        cm->cameraAdded.connect<void>([](std::shared_ptr<libcamera::Camera> cam){ Logger::instance() << "[INFO][_camera_manager]" << cam->id() << " connected." << std::endl; });
        cm->cameraRemoved.connect<void>([](std::shared_ptr<libcamera::Camera> cam){ Logger::instance() << "[WARNING][_camera_manager]" << cam->id() << " removed." << std::endl; });
    }

    bool start_impl(){ cm->start(); return true; }

    void stop_impl(){ cm->stop(); }

    void step(){}

    const std::unique_ptr<libcamera::CameraManager>& manager(){ return cm; }

    void submit_settings(const std::string& name, int32_t et, float ag, float rg, float bg)
    {
        std::lock_guard<std::mutex> gd(settings_mutex);
        exposure_time[name] = et;
        analog_gain[name] = ag;
        red_gain[name] = rg;
        blue_gain[name] = bg;
    }

    size_t n_settings_submitted()
    {
        std::lock_guard<std::mutex> gd(settings_mutex);
        return exposure_time.size();
    }

    std::tuple<int32_t, float, float, float> get_global_settings()
    {
        int32_t et = 0;
        float ag = 0.0, rg = 0.0, bg = 0.0;
        std::lock_guard<std::mutex> gd(settings_mutex);
        for(const auto& item : exposure_time)
        {
            et += exposure_time[item.first];
            ag += analog_gain[item.first];
            rg += red_gain[item.first];
            bg += blue_gain[item.first];
        }
        return {
            et/exposure_time.size(),
            ag/exposure_time.size(),
            rg/exposure_time.size(),
            bg/exposure_time.size()
        };
    }


protected:
    std::unique_ptr<libcamera::CameraManager> cm;
    
    std::mutex settings_mutex;
    std::unordered_map<std::string, int32_t> exposure_time;
    std::unordered_map<std::string, float> analog_gain, red_gain, blue_gain;
};

