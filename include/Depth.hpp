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
        right(right),
        left_intr(3,3,CV_64F,(void*)_left_intrinsic), 
        right_intr(3,3,CV_64F,(void*)_right_intrinsic), 
        left_dist(1,5,CV_64F,(void*)_left_distortion), 
        right_dist(1,5,CV_64F,(void*)_right_distortion), 
        R(3,3,CV_64F,(void*)_R), 
        T(3,1,CV_64F,(void*)_T)
    {
        stereo = cv::StereoSGBM::create(
            	0, // minDisparity
                n_disparity, // numDisparities
                7, // blockSize
                0, // P1
                0, // P2
                0, // disp12MaxDiff
                0, // preFilterCap
                0, // uniquenessRatio
                150, // speckleWindowSize
                2 // speckleRange
            );
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
    cv::Mat depth();

    /** Initalizes data arrays.
    
    Should be called after cameras are initialized and their shape data
    is available.
    */
    void initialize();

protected:
    std::shared_ptr<CameraWrapper> left, right;
    cv::Ptr<cv::StereoSGBM> stereo;
    // Data buffer
    static constexpr size_t N = 4;
    int locked_idx;
    size_t latest_idx;
    cv::Mat _disparity[N];
    cv::Mat rect_l, rect_r;

    struct disp_conv
    {
        float M;

        void operator()(float& p, const int* idx) const
        {
            if(std::abs(p) < 1e-3)
                p = 0.0;
            else
                p = M / (p + std::numeric_limits<float>::epsilon());
        }
    };
    struct disp_conv converter;

    // Camera parameters (distance units are cm)
    cv::Mat mapl1, mapl2, mapr1, mapr2, Q;
    static constexpr int n_disparity = 128; 
    const cv::Mat left_intr, right_intr, left_dist, right_dist, R, T;
    static constexpr double _left_intrinsic[] = {629.9333903041515, 0, 405.2045199935685, 0, 629.9333903041515, 318.0172477395962, 0, 0, 1};
    static constexpr double _left_distortion[] = {0.141697251352411, -0.1700086026438087, 0, 0, 0, 0, 0, 0.3306948327230533, 0, 0, 0, 0, 0, 0};
    static constexpr double _right_intrinsic[] = {629.9333903041515, 0, 404.147042716075, 0, 629.9333903041515, 308.7248574850515, 0, 0, 1};
    static constexpr double _right_distortion[] = {0.1603950071594413, -0.148940226425748, 0, 0, 0, 0, 0, 0.5399694529707213, 0, 0, 0, 0, 0, 0};
    static constexpr double _R[] = {0.9989588969333708, -0.04343256770475815, -0.01395472322314447, 0.04347647946309361, 0.9990503606365717, 0.002858783907747735, 0.0138173069430395, -0.003462509856678753, 0.9998985413802414}; 
    static constexpr double _T[] = {-5.47029237377605, -0.01683788389047397, 0.08469902480152951}; 
};