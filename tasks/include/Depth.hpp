#pragma once

#include <CameraWrapper.hpp>
#include <opencv2/calib3d.hpp>
#include <memory>
#include "Task.hpp"
#include "DataSource.hpp"


FWD_DECL_DATA_SOURCE(DepthCamera, float[CameraWrapper::Width * CameraWrapper::Height])


/** Manages getting grayscale frames from two cameras and maintaining a buffer of depth maps.
*/
class DepthCamera : public DataSource<DepthCamera>
{
public:
    /** Construct a new depth camera
    
    @param name Task name
    @param cal_path Folder containing calibration files
    @param left Left camera task
    @param right Right camera task
    */  
    DepthCamera(const char* name, const std::string& cal_path, std::shared_ptr<CameraWrapper> left, std::shared_ptr<CameraWrapper> right) : 
        DataSource<DepthCamera>(name, {left, right}),
        left(left),
        right(right),
        left_cal(cal_path + "/left.json"),
        right_cal(cal_path + "/right.json"),
        stereo_cal(cal_path + "/stereo.json"),
        left_intr(3,3,CV_64F), 
        right_intr(3,3,CV_64F), 
        left_dist(1,5,CV_64F), 
        right_dist(1,5,CV_64F), 
        R(3,3,CV_64F), 
        T(3,1,CV_64F)
    {
        // stereo = cv::StereoSGBM::create(
        //     	0, // minDisparity
        //         16*5, // numDisparities
        //         7, // blockSize
        //         1, // P1
        //         8, // P2
        //         -1, // disp12MaxDiff
        //         0, // preFilterCap
        //         10, // uniquenessRatio
        //         50, // speckleWindowSize
        //         1 // speckleRange
        //     );
        stereo = cv::StereoBM::create(0, 16*6);
        stereo->setBlockSize(15);
        // stereo->setUniquenessRatio(1);
        // stereo->setSpeckleWindowSize(50);
        // stereo->setSpeckleRange(1);
        // stereo->setTextureThreshold(0);
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

    /** Changes the parameters for StereoSGBM.
     
    */
    void set_params(int	minDisparity,
                    int	numDisparities,
                    int	blockSize,
                    int	P1,
                    int	P2,
                    int	disp12MaxDiff,
                    int	preFilterCap,
                    int	uniquenessRatio,
                    int	speckleWindowSize,
                    int	speckleRange);

    
protected:

    
    /** Applies rectification from the calibration data to a pair of images.
    
    */
    void rectify(cv::Mat& left_in, cv::Mat& right_in, cv::Mat& left_out, cv::Mat& right_out);

    std::shared_ptr<CameraWrapper> left, right;
    std::shared_ptr<CameraWrapper::update_type> latest_left, latest_right;
    cv::Ptr<cv::StereoBM> stereo;

    // Intermediate matrices
    cv::Mat rect_l, rect_r, disp, depth;

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
    const std::string left_cal, right_cal, stereo_cal;
    cv::Mat mapl1, mapl2, mapr1, mapr2, Q;
    cv::Mat left_intr, right_intr, left_dist, right_dist, R, T;
};