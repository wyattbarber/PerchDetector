#pragma once

#include <libcamera/libcamera.h>
#include <memory>
#include <opencv2/core/mat.hpp>
#include "CameraWrapper.hpp"
#include <Eigen/Dense>


FWD_DECL_DATA_SOURCE(CameraSimulator, CameraWrapper::value_type)

/** Wrapper to simplify reading grayscale images from a libcamera::Camera.
 * 
 * Handles detection of a suitable camera device, configuring the desired image
 * format, mapping DMA buffers to memory, and keeping data in scope as the application needs.
 * 
 */
class CameraSimulator : public DataSource<CameraSimulator>
{
    public:
    static const size_t Width = CameraWrapper::Width;
    static const size_t Height = CameraWrapper::Height;

    /** Create a new wrapper for one camera on the system. 

    The given check function should take a constant string reference, which
    is the id of a camera detected on the system, and return a boolean indicating
    if that id matches the camera that this wrapper should attach itself to. 

    @param name Camera name, for logging
    */
    CameraSimulator(const char* name, bool right) : 
        DataSource(name, {}),
        right(right)
    {
    }

    /** Starts the camera 
    */
    bool start_impl(){ return true; }

    /** Stops the camera 
    */
    void stop_impl(){}

    void step();

    std::vector<size_t> dims(){ return {Height, Width}; }

protected:
    static const unsigned block_width = 100;
    static const unsigned block_height = 50;
    const bool right;

    // Preallocated intermediates
    Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> background, block;
    cv::Mat cv_out;
};


