#pragma once

#include <libcamera/libcamera.h>
#include <memory>
#include <opencv2/core/mat.hpp>

/** Maps a DMA buffer descriptor to an OpenCV matrix 

*/
class DMAMat
{
public:
DMAMat(int height, int width, int cvtype): 
    fd(0), 
    map(nullptr)
    height(height),
    width(width),
    cvtype(cvtype),
    matrix(height, width, cvtype)
{}

/** Set the mapped buffer.

@param fb Pointer to buffer to map to
@param cvtype OpenCV datatype specifier

@return true if mapping succeeded.
*/
bool set(const libcamera::FrameBuffer* fb, int height, int width, int cvtype);

/** Get the matrix mapped to the last given DMA buffer. 

@return Pointer to matrix
*/
cv::Mat get();

protected:

bool unmap();

int fd;
void* map;
int size;
cv::Mat matrix;

};