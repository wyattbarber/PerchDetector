#include <Depth.hpp>
#include <opencv2/imgproc.hpp>


void DepthCamera::initialize()
{
    for(int i = 0; i < N; ++i)
    {
        _disparity[i] = cv::Mat(left->get_height(), left->get_width(), CV_16S);
    }

    // Initialize stereo calibration data
    // cv::Mat _Rl, _Rr, _Pl, _Pr;
    // std::cout << "Computing stereo rectification" << std::endl;
    // cv::stereoRectify(
    //     left_intr, left_dist,
    //     right_intr, right_dist,
    //     cv::Size(left->get_height(), left->get_width()),
    //     R, T, 
    //     _Rl, _Rr, _Pl, _Pr,
    //     Q
    // );
    // cv::Mat intr_l2, intr_r2;
    // std::cout << "Computing left remap" << std::endl;
    // cv::initUndistortRectifyMap(
    //     left_intr, left_dist, _Rl, intr_l2,
    //     cv::Size(left->get_height(), left->get_width()),
    //     CV_16SC2, mapl1, mapl2
    // );    
    // std::cout << "Computing right remap" << std::endl;
    // cv::initUndistortRectifyMap(
    //     right_intr, right_dist, _Rr, intr_r2,
    //     cv::Size(left->get_height(), left->get_width()),
    //     CV_16SC2, mapr1, mapr2
    // );
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

    
    left->lock(); 
    right->lock();
    auto l = left->image();
    auto r = right->image();
    auto d = &_disparity[target_idx];
    cv::undistort(*l, rect_l, left_intr, left_dist);
    cv::undistort(*r, rect_r, right_intr, right_dist);
    left->unlock();
    right->unlock();
    stereo->compute(rect_l, rect_r, *d);

    // Update indices
    latest_idx = target_idx;
}


struct disp_conv
{
    double M = 5.0;

    void operator()(float& p, const int* idx) const
    {
        p = M / p;
    }
};


cv::Mat DepthCamera::depth()
{
    cv::Mat out(disparity()->rows, disparity()->cols, CV_32F);
    disparity()->convertTo(out, CV_32F);
    out.forEach<float>(disp_conv());
    return out;
}
