#include <Depth.hpp>


void DepthCamera::initialize()
{
    for(int i = 0; i < N; ++i)
    {
        _disparity[i] = cv::Mat(left->get_height(), left->get_width(), CV_16S);
    }
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

    // Calculate disparity map
    std::cout << "Calculating disparity buffer " << target_idx << std::endl;
    left->lock(); 
    right->lock();
    std::cout << "Getting left image " << std::endl;
    auto l = left->image();
    std::cout << "Getting right image " << std::endl;
    auto r = right->image();
    stereo->compute(*l, *r, _disparity[target_idx]);
    left->unlock();
    right->unlock();

    // Update indices
    latest_idx = target_idx;
}


struct disp_conv
{
    float M;

    void operator()(float &p, const int * position) const
    {
        p *= M;
    }
};


const cv::Mat DepthCamera::depth()
{
    std::cout << "Calculating depth" << std::endl;
    cv::Mat disp(disparity()->rows, disparity()->cols, CV_32F);
    disparity()->convertTo(disp, CV_32F, 1.0);
    disp_conv converter;
    converter.M = M;
    disp.forEach<float>(converter);
    return disp;
}
