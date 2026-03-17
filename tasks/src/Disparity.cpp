#include <Disparity.hpp>
#include <Logging.hpp>
#include <limits>
#include <thread>
#include <json_loader.hpp>
#include <opencv2/imgproc.hpp>
#include <Eigen/Dense>


using namespace std::chrono_literals;


void set_stereo_params(cv::Ptr<StereoMatcherType> stereo,
                int	minDisparity,
                int	numDisparities,
                int	blockSize,
                int	P1,
                int	P2,
                int	disp12MaxDiff,
                int	preFilterCap,
                int	uniquenessRatio,
                int	speckleWindowSize,
                int	speckleRange)                
{
    stereo->setMinDisparity(minDisparity);
    stereo->setNumDisparities(numDisparities);
    stereo->setDisp12MaxDiff(disp12MaxDiff);
    stereo->setPreFilterCap(preFilterCap);
    stereo->setUniquenessRatio(uniquenessRatio);
    stereo->setSpeckleWindowSize(speckleWindowSize);
    stereo->setSpeckleRange(speckleRange);
    if constexpr (std::is_same_v<StereoMatcherType, cv::StereoSGBM>)
    {
        stereo->setP1(P1);
        stereo->setP2(P2);
    }
}


bool DepthCamera::start_impl()
{
    if(!left->is_alive() || !right->is_alive())
    {
        warning("Input cameras are not started, cannot start depth computation");
        return false;
    }

    // Load calibration data
    info("Loading left camera calibration from ", left_cal);
    if(!load_camera_matrices(left_cal, left_dist, left_intr))
    {
        error("Failed to load left camera calibration");
        return false;
    }
    info("Loading right camera calibration from ", right_cal);
    if(!load_camera_matrices(right_cal, right_dist, right_intr))
    {
        error("Failed to load right camera calibration");
        return false;
    }
    info("Loading stereo calibration from ", stereo_cal);
    if(!load_stereo_matrices(stereo_cal, R, T))
    {
        error("Failed to load stereo calibration");
        return false;
    }


    // Initialize stereo calibration data
    cv::Mat _Rl, _Rr, _Pl, _Pr;
    info("Computing stereo rectification");
    cv::stereoRectify(
        left_intr, left_dist,
        right_intr, right_dist,
        cv::Size(left->get_width(), left->get_height()),
        R, T, 
        _Rl, _Rr, _Pl, _Pr,
        Q,
        cv::CALIB_ZERO_DISPARITY,
        0 
    );
    info("Computing left remap");
    cv::initUndistortRectifyMap(
        left_intr, left_dist, _Rl, _Pl,
        cv::Size(left->get_width(), left->get_height()),
        CV_16SC2, mapl1, mapl2
    );    
    info("Computing right remap");
    cv::initUndistortRectifyMap(
        right_intr, right_dist, _Rr, _Pr,
        cv::Size(left->get_width(), left->get_height()),
        CV_16SC2, mapr1, mapr2
    );

    // Pre-allocate intermediates
    rect_l = cv::Mat(left->get_height(), left->get_width(), CV_8U);
    rect_r = cv::Mat(left->get_height(), left->get_width(), CV_8U);
    disp_left = cv::Mat(left->get_height(), left->get_width(), CV_16S);
    disp_right = cv::Mat(left->get_height(), left->get_width(), CV_16S);
    depth = cv::Mat(left->get_height(), left->get_width(), CV_32F);

    // Initialize stereo matchers and filter
    stereo_right = std::dynamic_pointer_cast<StereoMatcherType>(cv::ximgproc::createRightMatcher(stereo_left));
    filter = cv::ximgproc::createDisparityWLSFilter(stereo_left);

    // Get first data so pointers are valid
    latest_left = left->acquire();
    latest_right = right->acquire();

    return true;
}


void DepthCamera::step()
{
    // Don't update if stopped or if images are not fresh
    if(!is_alive()) return;
    if(!latest_left->stale || !latest_right->stale) return;
    tick();
    
    latest_left = left->acquire();
    latest_right = right->acquire();

    const cv::Mat left_im(left->get_height(), left->get_width(), CV_8UC1, const_cast<uint8_t*>(latest_left->data));
    const cv::Mat right_im(right->get_height(), right->get_width(), CV_8UC1, const_cast<uint8_t*>(latest_right->data));
    rectify(left_im, right_im, rect_l, rect_r);

    stereo_left->compute(rect_l, rect_r, disp_left);
    stereo_right->compute(rect_r, rect_l, disp_right);
    update_ptr_type next = allocate_next();
    filter->filter(
        disp_left, 
        rect_l, 
        cv::Mat(left->get_height(), left->get_width(), CV_16S, next->data.disparity), 
        disp_right
    );
    cv::Mat conf = filter->getConfidenceMap();
    conf.convertTo(
        cv::Mat(left->get_height(), left->get_width(), CV_8UC1, next->data.confidence),
        CV_8U
    );

    swap_data();
}


void DepthCamera::rectify(const cv::Mat& left_in, const cv::Mat& right_in, cv::Mat& left_out, cv::Mat& right_out)
{
    cv::remap(left_in, left_out, mapl1, mapl2, cv::INTER_LINEAR);
    cv::remap(right_in, right_out, mapr1, mapr2, cv::INTER_LINEAR);
}


void disparity_display_conv(void* dst, const DepthCamera::value_type& value)
{
    using tmp_t = Eigen::Map<Eigen::Matrix<int16_t, CameraWrapper::Height, CameraWrapper::Width>>;
    const tmp_t tmp_src(const_cast<int16_t*>(value.disparity), CameraWrapper::Height, CameraWrapper::Width);
    tmp_t tmp_dst((int16_t*)dst, CameraWrapper::Height, CameraWrapper::Width); 
    tmp_dst = tmp_src.unaryExpr(
            [](int16_t x){ return x > 0 ? x : 0; }
        ).cast<int16_t>();
}