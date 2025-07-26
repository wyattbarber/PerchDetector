#pragma once

#include <opencv2/core/mat.hpp>
#include <memory>

/** Transmits images for other applications to view via a memory mapped matrix.
 * 
 * Maps an image to a file that is meant to be readable by other applications.
 * The format of the memory region is as follows, from the start of the region:
 * 
 * * 1 byte used as a lock, non-zero if a program is writing to the region.
 * 
 * * 1 byte which contains the OpenCV datatype ID of the pixels in the image.
 * 
 * * 2 bytes containing the image height, in pixels
 * 
 * * 2 bytes containing the image width, in pixels
 * 
 * * The remaining bytes contain the image data
 * 
 * This object will create the file if needed and set it to the specified size.
*/
class ImageSender
{
    public:
    /** Create a new image sender.
    
    @tparam T datatype of the pixels of the image

    @param file File name for mapped memory region
    @param width Pixel width of the image
    @param height Pixel height of the image
    @param elemsize Size in bytes of each pixel
    @param cvtype OpenCV type id of the pixel elements
    */
    ImageSender(const char * file, int width, int height, size_t elemsize, int cvtype) : 
        file(file),
        width(width),
        height(height),
        size((width*height*elemsize)+6),
        cvtype(cvtype)
    {
        valid = false;
    }
    /** Starts the server 
    */
    void start();

    /** Closes the server
    */
    void stop();

    /** Lock the matrix for write access.
     * 
     */
    void acquire();

    /** Release write access to the matrix.
     * 
     */
    void release();

    /** Access the memory mapped matrix managed by this object.
     * 
     * @return Pointer to matrix.
     */
    cv::Mat& get_image();

    /** Check if the mapped memory has successully been initialized.
     * 
     * @return true if the mapped object is valid
     */
    bool opened();

    
    protected:
    const char* file;
    const int width, height, size;
    const int cvtype;
    cv::Mat image;
    int fd;
    void* map;
    bool valid;
};


/** Creates an ImageSender instance, with OpenCV type ID determined based on C++ datatype.

@tparam T C++ datatype of the image matrix

@param file File name for the mapped memory region
@param width Pixel width of the mapped matrix
@param heigth Pixel height of the mapped matrix
*/
template<typename T>
std::unique_ptr<ImageSender> make_sender(const char * file, int width, int height)
{ 
    return std::make_unique<ImageSender>(file, width, height, sizeof(T), cv::DataType<T>::type);
}