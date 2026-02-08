#pragma once

#include "Task.hpp"
#include <libcamera/libcamera.h>
#include <memory>


/** Task for a camera manager.

Just wraps starting the manager and logging
libcamera events.
*/
class CameraManagerTask : public Task
{
public:
    CameraManagerTask() : 
        Task("_camera_manager", {})
    {        
        cm = std::make_unique<libcamera::CameraManager>();
        cm->cameraAdded.connect<void>([](std::shared_ptr<libcamera::Camera> cam){ Logger::instance() << "[INFO][_camera_manager]" << cam->id() << " connected." << std::endl; });
        cm->cameraRemoved.connect<void>([](std::shared_ptr<libcamera::Camera> cam){ Logger::instance() << "[WARNING][_camera_manager]" << cam->id() << " removed." << std::endl; });
    }

    bool start_impl(){ cm->start(); return true; }

    void stop_impl(){ cm->stop(); }

    void step(){}

    const std::unique_ptr<libcamera::CameraManager>& manager(){ return cm; }

protected:
    std::unique_ptr<libcamera::CameraManager> cm;
};

