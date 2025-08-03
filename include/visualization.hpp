#pragma once

#include <ImageSender.hpp>
#include <CameraWrapper.hpp>
#include <ArgParser.hpp>

/** Helpers for communication to a GUI

*/
namespace visualization
{
    extern std::unique_ptr<ImageSender> image, depth;

    int setup(ArgParser& args, std::shared_ptr<CameraWrapper> camera_left, std::shared_ptr<CameraWrapper> camera_right);

    void teardown();
}