#pragma once

#include <libcamera/libcamera.h>
#include <memory>

class CameraWrapper
{
    public:

    /** Create a new wrapper for one camera on the system. 

    The given check function should take a constant string reference, which
    is the id of a camera detected on the system, and return a boolean indicating
    if that id matches the camera that this wrapper should attach itself to.

    @param cm Global camera manager instance
    @param check Function to check the string id of a detected camera    
    */
    CameraWrapper(const std::string name, const std::unique_ptr<libcamera::CameraManager>& cm, bool(*check)(const std::string&)) :
        name(name)
    {
        // Check all cameras
        for (auto const &camera : cm->cameras())
        {
            if(check(camera->id()))
            {
                // This cameras id matches what this wrapper should attach to
                std::cout << name << ": Attaching to detected camera " << camera->id() << std::endl;
                this->camera = cm->get(camera->id());
                return;
            }
        }
        std::cerr << name << ": No camera matching the given criteria was detected." << std::endl;
    }

    /** Acquires control of the wrapped camera 
    */
    void acquire();
    
    /** Releases control of the wrapped camera 
    */
    void release();

    /** Sets the camera configuration
    
    Configures the camera to acquire the desired image size, and also 
    performs frame buffer allocation.
    
    @param width Image width to acquire
    @param height Image height to acquire
    */
    void configure(unsigned width, unsigned height);

    /** Starts the camera 
    */
    void start();

    /** Stops the camera 
    */
    void stop();

    /** Trigger the capture of a single frame 
    */
    void capture_start();

    /** Get the result of the capture.

    Blocks until the frame is available. The returned 
    pointer will be valid until the next call to 
    capture_start, after which it may be overwritten.
    */
    libcamera::FrameBuffer* capture_wait();

    protected:
    const std::string name;
    std::shared_ptr<libcamera::Camera> camera;
    std::unique_ptr<libcamera::CameraConfiguration> config;
    libcamera::Stream *stream;
    const std::vector<std::unique_ptr<libcamera::FrameBuffer>>* buffers;
    std::vector<std::unique_ptr<libcamera::Request>> requests;
};