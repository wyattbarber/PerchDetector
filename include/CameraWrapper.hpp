#pragma once

#include <libcamera/libcamera.h>
#include <memory>
#include <opencv2/core/mat.hpp>

/** Wrapper to simplify reading grayscale images from a libcamera::Camera.
 * 
 * Handles detection of a suitable camera device, configuring the desired image
 * format, mapping DMA buffers to memory, and keeping data in scope as the application needs.
 * 
 */
class CameraWrapper
{
    public:

    typedef uint8_t dtype; /// Datatype of pixel intensity values
    static const auto cvtype = cv::DataType<dtype>::type; // OpenCV type ID of pixel intensity values

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
    
    /** Releases control of the wrapped camera and frees resources
    */
    void release();

    /** Sets the camera configuration
    
    Configures the camera to to aqcuire the desired image format,
    and allocates frame buffers.
    */
    void configure();

    /** Starts the camera 
    */
    void start();

    /** Stops the camera 
    */
    void stop();

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

    /** Latest image buffer.
     * 
     * Gets a pointer to the image buffer most recenltly updated and locked.
     * 
     * The image provides is a single channel grayscale image.
     * 
     * @return start of image data.
    */
    void* data(){ return map[locked_idx]; }

    /** Latest image matrix.
     * 
     * Provides the same data as data(), however it returns
     * a pointer to a cv::Mat that is mapped to the same buffer,
     * for easier copying to other matrices or passing to OpenCV 
     * functions.
     * 
     * The image provides is a single channel grayscale image.
     * 
     * @return image
    */
    cv::Mat* image(){ return &matrices[locked_idx]; }

    /** Get the size of the image data in bytes.
    
    @return data size.
    */
    size_t size(){ return bytes; }


    /** Gets a pointer to the camera instance.
     * 
     * @return Camera pointer wrapped by this class.
     */
    std::shared_ptr<libcamera::Camera> get_camera(){ return camera; }

    /* Indicates which buffer is the most recent, and queues the next in line.
    Should be used only by the capture callback, not by application code.*/
    void set_freshest(uint8_t);

    /** Locks the most recently updated buffer.
     * 
     * Allows the application to acquire the most recently updated buffer
     * and ensure that the data remains valid while it is used. Calling this
     * method will cause the value pointed to by data() or image() to remain 
     * unchanged until unlock() is called. For that duration, if the associated
     * buffer and request is reached in the frame reading cycle, the locked buffer 
     * will be skipped and the next request queued instead.
     */
    void lock();

    /** Unlock the currently in scope buffer.
     * 
     * Should be called after lock() once all processing of the 
     * latest frame has completed. Calling this releases the currently
     * locked buffer to again be queued once it is due in the update loop, 
     * and the data pointed to by data() or image() may be overwritten.
     */
    void unlock();


    protected:
    const std::string name;
    uint64_t cookie;
    std::shared_ptr<libcamera::Camera> camera;
    std::unique_ptr<libcamera::CameraConfiguration> config;
    libcamera::FrameBufferAllocator *allocator;
    libcamera::Stream *stream;
    const std::vector<std::unique_ptr<libcamera::FrameBuffer>>* buffers;
    std::vector<std::unique_ptr<libcamera::Request>> requests;
    std::vector<void*> map;
    std::vector<cv::Mat> matrices;
    size_t bytes, width, height, stride;
    size_t freshest_buffer;
    uint16_t locked_idx;
};


