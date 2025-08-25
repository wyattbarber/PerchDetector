#include <Depth.hpp>
#include <opencv2/imgproc.hpp>
#include <limits>
#include <thread>


using namespace std::chrono_literals;


void DepthCamera::initialize()
{
    for(int i = 0; i < N; ++i)
    {
        _disparity[i] = cv::Mat(left->get_height(), left->get_width(), CV_16S);
    }

    // Initialize stereo calibration data
    cv::Mat _Rl, _Rr, _Pl, _Pr;
    std::cout << "Computing stereo rectification" << std::endl;
    cv::stereoRectify(
        left_intr, left_dist,
        right_intr, right_dist,
        cv::Size(left->get_width(), left->get_height()),
        R, T, 
        _Rl, _Rr, _Pl, _Pr,
        Q
    );
    std::cout << "Computing left remap" << std::endl;
    cv::initUndistortRectifyMap(
        left_intr, left_dist, _Rl, _Pl,
        cv::Size(left->get_width(), left->get_height()),
        CV_16SC2, mapl1, mapl2
    );    
    std::cout << "Computing right remap" << std::endl;
    cv::initUndistortRectifyMap(
        right_intr, right_dist, _Rr, _Pr,
        cv::Size(left->get_width(), left->get_height()),
        CV_16SC2, mapr1, mapr2
    );

    /* Initialize disparity to depth conversion
        Conversion factor is Z = B*f / disparity, and opencv disparity
        is scaled up by 15. The matrix Q is expected, for horizontal 
        stereo cameras, to have f at element (2,3), and -1/B at element
        (3,2)
    */
    converter.M =  static_cast<float>(Q.at<double>(2,3) / (16.0 * std::abs(Q.at<double>(3,2))));
    std::cout << "Q: " << Q << std::endl;
    std::cout << "Set disparity to depth conversion to " << converter.M << std::endl;
}


void DepthCamera::update()
{
    // Determine the next buffer to update
    std::cout << "Depth index updating" << std::endl;
    size_t target_idx = latest_idx + 1;
    if(target_idx == locked_idx)
    {
        target_idx += 1;
    }
    if(target_idx >= N)
    {
        target_idx = (locked_idx == 0) ? 1 : 0;
    }

    cv::Mat right_im, left_im, left_rect, right_rect;
    left->lock(); 
    right->lock();
    rectify(*left->image(), *right->image(), left_rect, right_rect);
    left->unlock();
    right->unlock();

    _stereo_locked = true;
    stereo->compute(left_rect, right_rect, _disparity[target_idx]);
    if(_disparity[target_idx].empty())
        std::cout << "Empty disparity map produced" << std::endl;
    _stereo_locked = false;

    // Update indices
    latest_idx = target_idx;
}


cv::Mat DepthCamera::depth()
{
    cv::Mat out(disparity()->rows, disparity()->cols, CV_32F);
    disparity()->convertTo(out, CV_32F);
    out.forEach<float>(converter);
    return out;
}


void DepthCamera::rectify(cv::Mat& left_in, cv::Mat& right_in, cv::Mat& left_out, cv::Mat& right_out)
{
    cv::remap(left_in, left_out, mapl1, mapl2, cv::INTER_LINEAR);
    cv::remap(right_in, right_out, mapr1, mapr2, cv::INTER_LINEAR);
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
    while(_stereo_locked)
    {
        // Ensure any parameter changes are done, should be a very short wait
        std::this_thread::sleep_for(100ms);
    }
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