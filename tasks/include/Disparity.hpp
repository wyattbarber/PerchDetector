#pragma once

#include <CameraWrapper.hpp>
#include <CameraSim.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/stereo.hpp>
#include <memory>
#include "Task.hpp"
#include "DataSource.hpp"

#ifdef WSL_SIM
typedef CameraSimulator CameraType;
#else
typedef CameraWrapper CameraType;
#endif

typedef struct {
    int16_t disparity[CameraType::Width * CameraType::Height];
    uint8_t confidence[CameraType::Width * CameraType::Height];
    CameraType::update_ptr_const_type left_img, right_img;
} DisparityUpdate_t;

FWD_DECL_DATA_SOURCE(DepthCamera, DisparityUpdate_t)


using StereoMatcherType = cv::stereo::StereoBinarySGBM; // Select block matching algorithm


/** Generates disparity maps from stereo image pairs.

Gets grayscale images from the left and right cameras and processes
them with OpenCVs StereoSGBM class. Also uses the DisparityWLSFilter
from the ximgproc OpenCV extension to refine the disparity.

Produces 16 bit filtered disparity maps and an associated confidence map 
as output
*/
class DepthCamera : public DataSource<DepthCamera>
{
public:
    static constexpr auto Height = CameraType::Height;
    static constexpr auto Width = CameraType::Width;

    /** Construct a new disparity generator.

    The given calibration data path must be a folder containing the following files:
    - left.json: File containing left camera calibration
    - right.json: File containing right camera calibration
    - stereo_calibration.json: File containing stereo calibration
    - stereo_settings.json: File containing block matching and filtering parameters
    
    @param name Task name
    @param cal_path Folder containing calibration files
    @param left Left camera task
    @param right Right camera task
    */  
    DepthCamera(const char* name, const std::string& cal_path, std::shared_ptr<CameraType> left, std::shared_ptr<CameraType> right) : 
        DataSource<DepthCamera>(name, {left, right}),
        left(left),
        right(right),
        left_cal(cal_path + "/left.json"),
        right_cal(cal_path + "/right.json"),
        stereo_cal(cal_path + "/stereo_calibration.json"),
        stereo_param(cal_path + "/stereo_settings.json"),
        left_intr(3,3,CV_64F), 
        right_intr(3,3,CV_64F), 
        left_dist(1,5,CV_64F), 
        right_dist(1,5,CV_64F), 
        R(3,3,CV_64F), 
        T(3,1,CV_64F)
    {
    }


    /** Collects one new disparity map.
    */
    void step();

    /** Initalizes data arrays.
    
    Should be called after cameras are initialized and their shape data
    is available.
    */
    bool start_impl();

    void stop_impl(){}

    std::vector<size_t> dims(){ return {1}; }

    /** Provides the maximum dimensions of the point cloud.
    
    Returns the dimensions, x (width), y (height), and z (depth),
    of the bounding box the point cloud may occupy.

    @return Bounding box dimensions.
    */
    std::array<float, 3> volume();
  
protected:
    
    /** Applies rectification from the calibration data to a pair of images.
    
    */
    void rectify(const cv::Mat& left_in, const cv::Mat& right_in, cv::Mat& left_out, cv::Mat& right_out);

    std::shared_ptr<CameraType> left, right;
    typename CameraType::update_ptr_const_type latest_left, latest_right;

    bool downsample;
    cv::Ptr<StereoMatcherType> stereo_left, stereo_right;
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> filter;    
    
    // Intermediate matrices
    cv::Mat rect_l, rect_r, disp_left, disp_right, depth;
    cv::Mat rect_l_small, rect_r_small, disp_left_small, disp_right_small;

    // Camera parameters (distance units are cm)
    const std::string left_cal, right_cal, stereo_cal, stereo_param;
    cv::Mat mapl1, mapl2, mapr1, mapr2, Q;
    cv::Mat left_intr, right_intr, left_dist, right_dist, R, T; 

    // Detection bounding box sizes
    double bb_width, bb_height, bb_depth;

    bool configure_matchers();
    bool load_calibration();
    void dist_to_disp(double min_dist, double max_dist, int& min_disp, int& max_disp);
    std::tuple<double, double> bb_xy(double max_dist);
    
};


/** Converts disparity updates to single channel
    images that the gui can display.
*/
typedef struct stereo_display_conv
{ 
    using conversion_type = int16_t[DepthCamera::Height * DepthCamera::Width];
    static std::vector<size_t> dims() { return {DepthCamera::Height, DepthCamera::Width}; }
    static void eval(void* dst, const DepthCamera::value_type& src);
} stereo_display_conv;