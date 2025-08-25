#pragma once
#include <CameraWrapper.hpp>
#include <Depth.hpp>
#include <Logging.hpp>
/** Helper functions for initializing and destroying global camera wrappers.

*/
namespace stereo
{
    extern std::unique_ptr<libcamera::CameraManager> cm;
    extern std::shared_ptr<CameraWrapper> left, right;
    extern std::shared_ptr<DepthCamera> depth;
    void setup();
    void start();
    void stop();
    void teardown();
}