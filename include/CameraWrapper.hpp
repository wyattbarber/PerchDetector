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
    */
    void configure();

    /** Starts the camera 
    */
    void start();

    /** Stops the camera 
    */
    void stop();

    /** Gets the size of the image
    
    @return Image size as a (height, width) pair.
    */
    std::pair<int, int> shape();


    /** Get a pointer to the image data as a memory-mapped array.

    @return start of image data.
    */
    void* data();

    /** Get the size of the image data in bytes.
    
    @return data size.
    */
    size_t size();

    protected:
    const std::string name;
    std::shared_ptr<libcamera::Camera> camera;
    std::unique_ptr<libcamera::CameraConfiguration> config;
    libcamera::FrameBufferAllocator *allocator;
    libcamera::Stream *stream;
    const std::vector<std::unique_ptr<libcamera::FrameBuffer>>* buffers;
    std::vector<std::unique_ptr<libcamera::Request>> requests;
};


