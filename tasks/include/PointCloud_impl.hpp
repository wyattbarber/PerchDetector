#pragma once

#include <algorithm>


template<uint8_t X> 
bool PointCloud<X>::start_impl()
{
    if(!stereo->is_alive()) return false;

    // Load calibration data
    cv::Mat left_intr, right_intr, left_dist, right_dist, R, T;
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

    info("Computing stereo projection");
    cv::Mat _Rl, _Rr, _Pl, _Pr;
    cv::stereoRectify(
        left_intr, left_dist,
        right_intr, right_dist,
        cv::Size(stereo::Width, Stereo::Height),
        R, T, 
        _Rl, _Rr, _Pl, _Pr,
        Q,
        cv::CALIB_ZERO_DISPARITY,
        0 
    );

    latest_disparity = stereo->acquire();
    return true;
}


template<uint8_t X> 
void PointCloud<X>::step()
{
    if(is_alive())
    {
        // Compute point cloud
        cv::Mat disparity(stereo::Height, stereo::Width, CV_16S, latest_disparity->data.disparity);
        cv::reprojectImageTo3D(disparity, point_cloud, Q, handleMissingValues=false);
        // Partial sort confidence and indices to get low and high confidence partitions
        for(size_t i = 0; i < stereo::Height*stereo::Width; ++i)
        {
            confidence_w_indices[i].first = latest_disparity->data.confidence[i];
            confidence_w_indices[i].second = i;
        }
        std::partial_sort(confidence_w_indices, confidence_w_indices+num_points<X>(), confidence_w_indices+(stereo::Height*stereo::Width));
        // Copy best points to new update
        auto next = allocate_next();        
        auto point_clout_ptr = reinterpret_cast<float*>(point_cloud.data);
        for(size_t i = 0; i < num_points<X>(); ++i)
        {
            auto idx_orig = confidence_w_indices[i].second;
            next->data[3*i] = point_clout_ptr[3*idx_orig]; // x
            next->data[3*i + 1] = point_clout_ptr[3*idx_orig + 1]; // y
            next->data[3*i + 2] = point_clout_ptr[3*idx_orig + 2]; // z
        }
        swap_data();
    }
}
