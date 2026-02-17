#pragma once

#include <libcamera/libcamera.h>
#include <memory>
#include <opencv2/core/mat.hpp>
#include "CameraWrapper.hpp"


/** Wrapper to simplify reading grayscale images from a libcamera::Camera.
 * 
 * Handles detection of a suitable camera device, configuring the desired image
 * format, mapping DMA buffers to memory, and keeping data in scope as the application needs.
 * 
 */
class CameraSimulator : public CameraWrapper
{
    public:
    /** Create a new wrapper for one camera on the system. 

    The given check function should take a constant string reference, which
    is the id of a camera detected on the system, and return a boolean indicating
    if that id matches the camera that this wrapper should attach itself to. 

    @param name Camera name, for logging
    */
    CameraSimulator(const char* name, const std::shared_ptr<CameraManagerTask> cm) : 
        CameraWrapper(name, cm, [](const std::string& s){ return false; })
    {}

    /** Starts the camera 
    */
    bool start_impl();

    /** Stops the camera 
    */
    void stop_impl();

    void step();

    /** Image width
     * 
     * @return Width, in pixels
    */
    size_t get_width(){ return CameraWrapper::Width; }

    /** Image height
     * 
     * @return Height, in pixels
    */
    size_t get_height(){ return CameraWrapper::Height; }
    
    /** Buffer stride
     * 
     * @return Bytes between the start of each row in the image, or 0 if there is no padding.
    */
    size_t get_stride(){ return 0; }


protected:
    static const unsigned block_width = 100;
    static const unsigned block_height = 50;

};


