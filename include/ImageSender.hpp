#pragma once

#include <opencv2/core/mat.hpp>

/** Transmits images for other applications to view
*/
class ImageSender
{
    public:
    /** Create a new image sender.
    
    @param fd File descriptor for mapped memory region
    */
    ImageSender(const char * file, const cv::Mat& image) : file(file), image(image)
    {
        valid = false;
    }
    /** Starts the server 
    */
    void start();

    /** Closes the server
    */
    void stop();

    /** Sends a new image with timestamp
    
    @param image Image data to send
    */
    void transmit();

    bool valid;

    protected:
    const char* file;
    const cv::Mat& image;
    int fd;
    void* map;
    int size;
};