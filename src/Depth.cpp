#include <Depth.hpp>

 
void DepthCamera::update()
{
    // Determine the next buffer to update
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
    left.lock(); 
    right.lock();
    stereo->compute(*left.image(), *right.image(), _disparity[target_idx]);
    left.unlock();
    right.unlock();

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
    cv::Mat disp(disparity()->rows, disparity()->cols, CV_32F);
    disparity()->convertTo(disp, CV_32F, 1.0);
    disp_conv converter;
    converter.M = M;
    disp.forEach<float>(converter);
    return disp;
}
