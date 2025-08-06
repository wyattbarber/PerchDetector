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
        left_dist(14,1,CV_64F,(void*)_left_distortion), 
        right_dist(14,1,CV_64F,(void*)_right_distortion), 
        R(3,3,CV_64F,(void*)_R), 
        T(1,3,CV_64F,(void*)_T), 
        E(3,3,CV_64F,(void*)_E), 
        F(3,3,CV_64F,(void*)_F)
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
    cv::Mat depth();

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
    cv::Mat rect_l, rect_r;
    // Camera parameters (distance units are cm)
    cv::Mat mapl1, mapl2, mapr1, mapr2, Q;
    static constexpr int n_disparity = 128; 
    const cv::Mat left_intr, right_intr, left_dist, right_dist, R, T, E, F;
    static constexpr double _left_intrinsic[] = {1032.877814553041, 0, 457.3715211897926, 0, 1032.877814553041, 303.9427669495349, 0, 0, 1};
    static constexpr double _left_distortion[] = {0.5162895403076873, -5.185855062428619, 0, 0, 0, 0, 0, -67.91970353042133, 0, 0, 0, 0, 0, 0};
    static constexpr double _right_intrinsic[] = {1032.877814553041, 0, 457.2347516537842, 0, 1032.877814553041, 303.8946878162808, 0, 0, 1};
    static constexpr double _right_distortion[] = {0.332360544778726, 16.42783295428983, 0, 0, 0, 0, 0, 188.1307257633954, 0, 0, 0, 0, 0, 0};
    static constexpr double _R[] = {0.9959579217120413, -0.08243779967637739, -0.03566268867526335, 0.08044445632882448, 0.9952886822381827, -0.05412142324940952, 0.03995632146470866, 0.051033794617874, 0.9978973114413675}; 
    static constexpr double _T[] = {-3.786648392104754, 3.169911070779685, 1.500731136769793}; 
    static constexpr double _E[] = {0.005932485365427187, -1.331888124966322, 3.24446744007596, 1.645965604454552, 0.06953006351164649, 3.725166142539217, -3.461712913277011, -3.507487794432408, 0.3179863519712841}; 
    static constexpr double _F[] = {1.394179814319769e-08, -3.130039813636423e-06, 0.008820410163351524, 3.868146113209892e-06, 1.634010116590128e-07, 0.007223409757011645, -0.00958464668839288, -0.007132367996385186, 1.0}; 

};