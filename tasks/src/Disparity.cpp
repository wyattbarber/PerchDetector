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

    if(!load_calibration()) return false;

    // Pre-allocate intermediates
    rect_l = cv::Mat(Height, Width, CV_8U);
    rect_r = cv::Mat(Height, Width, CV_8U);
    disp_left = cv::Mat(Height, Width, CV_16S);
    disp_right = cv::Mat(Height, Width, CV_16S);
    depth = cv::Mat(Height, Width, CV_32F);
    if(downsample)
    {
        rect_l_small = cv::Mat(Height/2, Width/2, CV_8U);
        rect_r_small = cv::Mat(Height/2, Width/2, CV_8U);
        disp_left_small = cv::Mat(Height/2, Width/2, CV_16S);
        disp_right_small = cv::Mat(Height/2, Width/2, CV_16S);
    }

    if(!configure_matchers()) return false;

    // Get first data so pointers are valid
    latest_left = left->acquire();
    latest_right = right->acquire();

    return true;
}


void DepthCamera::step()
{
    // Don't update if images are not fresh
    if(!latest_left->stale || !latest_right->stale) return;
    tick();
    
    latest_left = left->acquire();
    latest_right = right->acquire();

    const cv::Mat left_im(Height, Width, CV_8UC1, const_cast<uint8_t*>(latest_left->data));
    const cv::Mat right_im(Height, Width, CV_8UC1, const_cast<uint8_t*>(latest_right->data));
    rectify(left_im, right_im, rect_l, rect_r);

    if(downsample)
    {
        cv::resize(rect_l, rect_l_small, cv::Size(Width/2, Height/2), 0, 0, cv::INTER_AREA);
        cv::resize(rect_r, rect_r_small, cv::Size(Width/2, Height/2), 0, 0, cv::INTER_AREA);
        stereo_left->compute(rect_l_small, rect_r_small, disp_left_small);
        stereo_right->compute(rect_r_small, rect_l_small, disp_right_small);
        disp_left_small *= 2;
        disp_right_small *= 2;
        cv::resize(disp_left_small, disp_left, cv::Size(Width, Height), 0, 0, cv::INTER_NEAREST);
        cv::resize(disp_right_small, disp_right, cv::Size(Width, Height), 0, 0, cv::INTER_NEAREST);
    }
    else
    {
        stereo_left->compute(rect_l, rect_r, disp_left);
        stereo_right->compute(rect_r, rect_l, disp_right);
    }    
    update_ptr_type next = allocate_next();
    filter->filter(
        disp_left, 
        rect_l, 
        cv::Mat(Height, Width, CV_16S, next->data.disparity), 
        disp_right
    );
    cv::Mat conf = filter->getConfidenceMap();
    conf.convertTo(
        cv::Mat(Height, Width, CV_8UC1, next->data.confidence),
        CV_8U
    );
    next->data.left_img = latest_left;
    next->data.right_img = latest_right;
    swap_data();
}


void DepthCamera::rectify(const cv::Mat& left_in, const cv::Mat& right_in, cv::Mat& left_out, cv::Mat& right_out)
{
    cv::remap(left_in, left_out, mapl1, mapl2, cv::INTER_LINEAR);
    cv::remap(right_in, right_out, mapr1, mapr2, cv::INTER_LINEAR);
}


bool DepthCamera::configure_matchers()
{
    // Load settings
    double min_dist=0.0, max_dist=0.0;
    int minDisparity=0, maxDisparity=0, blockSize=0, P1=0, P2=0, disp12MaxDiff=0, preFilterCap=0, uniquenessRatio=0, speckleWindowSize=0, speckleRange=0;
    if(!load_json_value_pairs(
        stereo_param,
        std::make_tuple("stereo"),        
        "minDistance", min_dist,
        "maxDistance", max_dist,
        "blockSize", blockSize,
        "P1", P1,
        "P2", P2,
        "disp12MaxDiff", disp12MaxDiff,
        "preFilterCap", preFilterCap,
        "uniquenessRatio", uniquenessRatio,
        "speckleWindowSize", speckleWindowSize,
        "speckleRange", speckleRange,
        "downsample", downsample
    ))
    {
        error("Failed to load block matcher settings.");
        return false;
    }
    dist_to_disp(min_dist, max_dist, minDisparity, maxDisparity);
    if((minDisparity < 1) || (maxDisparity < minDisparity) || (maxDisparity > static_cast<int>(Width/2)))
    {
        error("Computed disparity range ", minDisparity, ":", maxDisparity, " is invalid.");
        return false;
    }
    auto numDisparities = ((maxDisparity-minDisparity)/16)*16; // Num disparities should be a multiple of 16
    info("Configured stereo matching for distance range ", min_dist, ":", max_dist, ", disparity range ", minDisparity, ":", minDisparity+numDisparities);

    int lambda=0, dr=0, lrc=0;
    float sigma=0.0;
    if(!load_json_value_pairs(
        stereo_param,
        std::make_tuple("filter"),   
        "lambda", lambda,
        "sigma", sigma,
        "DR", dr,
        "LRC", lrc
    ))
    {
        error("Failed to load WLS filter settings.");
        return false;
    }

    // Initialize stereo matchers and filter
    stereo_left = StereoMatcherType::create();
    set_stereo_params(stereo_left, 
        minDisparity,
        numDisparities,
        blockSize,
        P1,
        P2,
        disp12MaxDiff,
        preFilterCap,
        uniquenessRatio,
        speckleWindowSize,
        speckleRange
    );
    stereo_right = std::dynamic_pointer_cast<StereoMatcherType>(cv::ximgproc::createRightMatcher(stereo_left));
    filter = cv::ximgproc::createDisparityWLSFilter(stereo_left);
    filter->setLambda(lambda);
    filter->setSigmaColor(sigma);
    filter->setDepthDiscontinuityRadius(dr);
    filter->setLRCthresh(lrc);

    // Calculate bounding box size
    bb_depth = max_dist - min_dist;
    std::tie(bb_width, bb_height) = bb_xy(max_dist);

    return true;
}


bool DepthCamera::load_calibration()
{
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
        cv::Size(Width, Height),
        R, T, 
        _Rl, _Rr, _Pl, _Pr,
        Q,
        cv::CALIB_ZERO_DISPARITY,
        0 
    );
    info("Computing left remap");
    cv::initUndistortRectifyMap(
        left_intr, left_dist, _Rl, _Pl,
        cv::Size(Width, Height),
        CV_16SC2, mapl1, mapl2
    );    
    info("Computing right remap");
    cv::initUndistortRectifyMap(
        right_intr, right_dist, _Rr, _Pr,
        cv::Size(Width, Height),
        CV_16SC2, mapr1, mapr2
    );

    return true;
}


void DepthCamera::dist_to_disp(double min_dist, double max_dist, int& min_disp, int& max_disp)
{
    auto b = 1.0 / Q.at<double>(3,2);
    auto f = Q.at<double>(2,3);
    info("Computing disparity range with baseline ", b, "m and focal length ", f, "px.");
    min_disp = static_cast<int>((f*b) / max_dist);
    max_disp = static_cast<int>((f*b) / min_dist);
    if(downsample)
    {
        min_disp /= 2;
        max_disp /= 2;
    }
}


std::tuple<double, double> DepthCamera::bb_xy(double max_dist)
{
    auto f = Q.at<double>(2,3);
    double w = static_cast<double>(CameraWrapper::Width) * max_dist / f;
    double h = static_cast<double>(CameraWrapper::Height) * max_dist / f;
    return {w, h};
}


std::array<double, 3> DepthCamera::volume()
{   
    return {bb_width, bb_height, bb_depth};
}



void stereo_display_conv::eval(void* dst, const DepthCamera::value_type& src)
{
    using tmp_t = Eigen::Map<Eigen::Matrix<int16_t, DepthCamera::Height, DepthCamera::Width>>;
    const tmp_t tmp_src(const_cast<int16_t*>(src.disparity), DepthCamera::Height, DepthCamera::Width);
    tmp_t tmp_dst((int16_t*)dst, DepthCamera::Height, DepthCamera::Width); 
    tmp_dst = tmp_src.unaryExpr(
            [](int16_t x){ return x > 0 ? x : 0; }
        ).cast<int16_t>();
}