#pragma once

#include <CameraWrapper.hpp>
#include <opencv2/calib3d.hpp>
#include <memory>
#include "Task.hpp"
#include "DataSource.hpp"


/** Manages getting grayscale frames from two cameras and maintaining a buffer of depth maps.
*/
class DepthCamera : public Task, public DataSource<float>
{
public:
    typedef float dtype; /// Datatype used for depth of each pixel
    static constexpr auto cvtype = cv::DataType<dtype>::type; // OpenCV type ID of pixel depth values

    /** Construct a new depth camera
    
    @param name Task name
    @param left Left camera task
    @param right Right camera task
    */  
    DepthCamera(const char* name, std::shared_ptr<CameraWrapper> left, std::shared_ptr<CameraWrapper> right) : 
        Task(name, {left, right}),
        DataSource<float>(),
        left(left),
        right(right),
        left_intr(3,3,CV_64F,(void*)_left_intrinsic), 
        right_intr(3,3,CV_64F,(void*)_right_intrinsic), 
        left_dist(1,5,CV_64F,(void*)_left_distortion), 
        right_dist(1,5,CV_64F,(void*)_right_distortion), 
        R(3,3,CV_64F,(void*)_R), 
        T(3,1,CV_64F,(void*)_T)
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

        _stereo_locked = false;
        locked_idx = 0;
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

    /** Get depth image size.
    
    @return number of pixels in the image.
    */
    size_t size() override;
    
protected:
    /** Locks the most recent depth map,
    preventing it from being overwritten,
    */
    float* lock() override 
    { 
        locked_idx = latest_idx; 
        _disparity[locked_idx].convertTo(depth, CV_32F);
        depth.forEach<float>(converter);
        return (float*)depth.data;
    }

    /** Release the previously locked depth map to be overwritten.
    */
    void unlock() override 
    { 
        locked_idx = -1; 
    }

    
    /** Applies rectification from the calibration data to a pair of images.
    
    */
    void rectify(cv::Mat& left_in, cv::Mat& right_in, cv::Mat& left_out, cv::Mat& right_out);

    std::shared_ptr<CameraWrapper> left, right;
    cv::Ptr<cv::StereoBM> stereo;
    bool _stereo_locked;

    // Data buffer
    static constexpr size_t N = 4;
    int locked_idx;
    size_t latest_idx;
    cv::Mat _disparity[N];
    cv::Mat rect_l, rect_r;
    cv::Mat depth;

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
    const cv::Mat left_intr, right_intr, left_dist, right_dist, R, T;
    static constexpr double _left_intrinsic[] = {635.1785761582178, 0, 403.9176435893709, 0, 635.1785761582178, 315.6895331764429, 0, 0, 1};
    static constexpr double _left_distortion[] = {0.06425766346938531, 0.2803647129199536, 0, 0, 0, 0, 0, 0.8826899528336022, 0, 0, 0, 0, 0, 0};
    static constexpr double _right_intrinsic[] = {635.1785761582178, 0, 402.351935054699, 0, 635.1785761582178, 315.7204369891957, 0, 0, 1};
    static constexpr double _right_distortion[] = {0.07421160862648429, 0.5709942396909135, 0, 0, 0, 0, 0, 2.225296151452876, 0, 0, 0, 0, 0, 0};
    static constexpr double _R[] = {0.9996814737070656, -0.01851791283139589, -0.01714753717705048, 0.01817539854252873, 0.9996363685898212, -0.01991947490209907, 0.01751016889380995, 0.01960146670338558, 0.9996545285689383}; 
    static constexpr double _T[] = {-7.302706912589366, 0.2571508864407846, -0.137874049067649}; 
};