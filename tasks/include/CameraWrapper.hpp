#pragma once

#include <Logging.hpp>
#include <libcamera/libcamera.h>
#include <memory>
#include <opencv2/core/mat.hpp>
#include "Task.hpp"
#include "DataSource.hpp"
#include "CameraManager.hpp"



const unsigned width = 600;
const unsigned height = 800;

/** Wrapper to simplify reading grayscale images from a libcamera::Camera.
 * 
 * Handles detection of a suitable camera device, configuring the desired image
 * format, mapping DMA buffers to memory, and keeping data in scope as the application needs.
 * 
 */
class CameraWrapper : public Task, public DataSource<uint8_t[width * height]>
{
    public:
    typedef uint8_t dtype; /// Datatype of pixel intensity values
    static constexpr auto cvtype = cv::DataType<dtype>::type; // OpenCV type ID of pixel intensity values

    /** Create a new wrapper for one camera on the system. 

    The given check function should take a constant string reference, which
    is the id of a camera detected on the system, and return a boolean indicating
    if that id matches the camera that this wrapper should attach itself to. 

    @param name Camera name, for logging
    @param cm Global camera manager instance
    @param check Function to check the string id of a detected camera    
    @param color Use RGB color format instead of grayscale.
    */
    CameraWrapper(const char* name, const std::shared_ptr<CameraManagerTask> cm, bool(*check)(const std::string&), bool color = false) :
        Task(name, {cm}),
        DataSource<uint8_t[width * height]>(),
        cm(cm),
        check(check),
        color(color)
    {}

    /** Starts the camera 
    */
    bool start_impl();

    /** Stops the camera 
    */
    void stop_impl();

    void step(){}

    /** Image width
     * 
     * @return Width, in pixels
    */
    size_t get_width(){ return width; }

    /** Image height
     * 
     * @return Height, in pixels
    */
    size_t get_height(){ return height; }
    
    /** Buffer stride
     * 
     * @return Bytes between the start of each row in the image, or 0 if there is no padding.
    */
    size_t get_stride(){ return stride; }


    /** Gets a pointer to the camera instance.
     * 
     * @return Camera pointer wrapped by this class.
     */
    std::shared_ptr<libcamera::Camera> get_camera(){ return camera; }

    /* Indicates which buffer is the most recent, and queues the next in line.
    Should be used only by the capture callback, not by application code.*/
    void set_freshest(uint8_t);


protected:

    /** Identifies the correct camera to read from.
    
    @return true if a camera was found and connected to.
    */
    bool connect();
    
    /** Sets the camera configuration
    
    Configures the camera to to aqcuire the desired image format,
    and allocates frame buffers.
    */
    void configure();



    uint64_t cookie;
    const std::shared_ptr<CameraManagerTask> cm;
    bool(*check)(const std::string&);
    std::shared_ptr<libcamera::Camera> camera;
    std::unique_ptr<libcamera::CameraConfiguration> config;
    libcamera::FrameBufferAllocator *allocator;
    libcamera::Stream *stream;
    const std::vector<std::unique_ptr<libcamera::FrameBuffer>>* buffers;
    std::vector<std::unique_ptr<libcamera::Request>> requests;
    std::vector<void*> map;
    size_t bytes, stride;
    const bool color;
};


