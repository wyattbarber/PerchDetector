#pragma once

#include <CameraWrapper.hpp>
#include <opencv2/calib3d.hpp>
#include <memory>


/** Manages getting grayscale frames from two cameras and maintaining a buffer of depth maps.
*/
class DepthCamera
{
public:
    typedef float dtype; /// Datatype used for depth of each pixel
    static constexpr auto cvtype = cv::DataType<dtype>::type; // OpenCV type ID of pixel depth values

    /** Construct a new depth camera
    
    @param left Left camera wrapper
    @param right Right camera wrapper
    */  
    DepthCamera(std::shared_ptr<CameraWrapper> left, std::shared_ptr<CameraWrapper> right) : 
        left(left),
        right(right)
    {
        stereo = cv::StereoBM::create(n_disparity);
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

    /** Initalizes data arrays.
    
    Should be called after cameras are initialized and their shape data
    is available.
    */
    void initialize();

protected:
    std::shared_ptr<CameraWrapper> left, right;
    cv::Ptr<cv::StereoBM> stereo;
    // Data buffer
    static constexpr size_t N = 4;
    int locked_idx;
    size_t latest_idx;
    cv::Mat _disparity[N];
    // Camera parameters (distance units ar cm)
    static constexpr float f = 0.304; 
    static constexpr float B = 7.3;
    static constexpr int n_disparity = 128; 
    static constexpr float M = f * B * static_cast<float>(n_disparity) * float(16.0);
};