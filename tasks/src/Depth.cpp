#include <Depth.hpp>
#include <Logging.hpp>
#include <opencv2/imgproc.hpp>
#include <limits>
#include <thread>
#include <loader.hpp>


using namespace std::chrono_literals;


bool DepthCamera::start_impl()
{
    if(!left->is_alive() || !right->is_alive())
    {
        warning("Input cameras are not started, cannot start depth computation");
        return false;
    }

    // Load calibration data
    std::string cal_err;
    info("Loading left camera calibration from ", left_cal);
    if(!load_camera_matrices(left_cal, cal_err, left_dist, left_intr))
    {
        error("Failed to load calibration: ", cal_err);
        return false;
    }
    info("Loading right camera calibration from ", right_cal);
    if(!load_camera_matrices(right_cal, cal_err, right_dist, right_intr))
    {
        error("Failed to load calibration: ", cal_err);
        return false;
    }
    info("Loading stereo calibration from ", stereo_cal);
    if(!load_stereo_matrices(stereo_cal, cal_err, R, T))
    {
        error("Failed to load calibration: ", cal_err);
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
        Q
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

    /* Initialize disparity arrays and depth conversion
        Conversion factor is Z = B*f / disparity, and opencv disparity
        is scaled up by 15. The matrix Q is expected, for horizontal 
        stereo cameras, to have f at element (2,3), and -1/B at element
        (3,2)
    */
    for(size_t i = 0; i < N; ++i)
    {
        _disparity[i] = cv::Mat(left->get_height(), left->get_width(), CV_16S);
    }
    depth = cv::Mat(_disparity[0].rows, _disparity[0].cols, CV_32F);
    converter.M =  static_cast<float>(Q.at<double>(2,3) / (16.0 * std::abs(Q.at<double>(3,2))));
    info("Q: ", Q);
    info("Set disparity to depth conversion to ", converter.M);

    return true;
}


void DepthCamera::step()
{
    if(!is_alive()) return;
    tick();

    cv::Mat left_rect, right_rect;
    cv::Mat left_im(left->get_height(), left->get_width(), CV_8UC1, left->acquire()->data, left->get_stride());
    cv::Mat right_im(right->get_height(), right->get_width(), CV_8UC1, right->acquire()->data, right->get_stride());
    rectify(left_im, right_im, left_rect, right_rect);

    stereo->compute(left_rect, right_rect, disp);
    if(disp.empty())
        warning("Empty disparity map produced");

}


void DepthCamera::rectify(cv::Mat& left_in, cv::Mat& right_in, cv::Mat& left_out, cv::Mat& right_out)
{
    cv::remap(left_in, left_out, mapl1, mapl2, cv::INTER_LINEAR);
    cv::remap(right_in, right_out, mapr1, mapr2, cv::INTER_LINEAR);
}


size_t DepthCamera::size()
{
    return _disparity[0].total();
}


void DepthCamera::set_params(int minDisparity,
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
    // stereo->setP1(P1);
    // stereo->setP2(P2);
    stereo->setDisp12MaxDiff(disp12MaxDiff);
    stereo->setPreFilterCap(preFilterCap);
    stereo->setUniquenessRatio(uniquenessRatio);
    stereo->setSpeckleWindowSize(speckleWindowSize);
    stereo->setSpeckleRange(speckleRange);
}