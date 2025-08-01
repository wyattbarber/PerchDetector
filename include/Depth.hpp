#pragma once

#include <CameraWrapper.hpp>
#include <opencv2/calib3d.hpp>

/** Manages getting grayscale frames from two cameras and maintaining a buffer of depth maps.
*/
class DepthCamera
{
public:
    /** Construct a new depth camera
    
    @param left Left camera wrapper
    @param right Right camera wrapper
    */  
    DepthCamera(CameraWrapper& left, CameraWrapper& right) : 
        left(left),
        right(right),
        stereo(cv::StereoBM::create(n_disparity))
    {
        locked_idx = 0;
    }

    /** Locks the most recent depth map,
    preventing it from being overwritten,
    */
    void lock(){ locked_idx = latest_idx; }

    /** Release the previously locked depth map to be overwritten.
    */
    void unlock(){ locked_idx = -1; }

    /** Collects one new disparity map.
    */
    void update();

    /** Gets a pointer to the most recently 
    accuired and locked disparity map. 
    
    @return Pointer to disparity data.
    */
    const cv::Mat* disparity(){ return &_disparity[locked_idx]; }

    /** Computes depth for the latest data.
    
    @return Depth image
    */
    const cv::Mat depth();

protected:
    CameraWrapper& left, right;
    cv::Ptr<cv::StereoBM> stereo;
    // Data buffer
    static constexpr size_t N = 4;
    int locked_idx;
    size_t latest_idx;
    cv::Mat _disparity[N];
    // Camera parameters (TODO)
    static constexpr float f = 1;
    static constexpr float B = 10;
    static constexpr int n_disparity = 128; 
    static constexpr float M = f * B * static_cast<float>(n_disparity) * float(16.0);
};