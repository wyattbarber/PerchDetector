#pragma once

#include <algorithm>
#include <json_loader.hpp>


template<uint8_t X> 
bool _PointCloud<X>::start_impl()
{
    if(!stereo->is_alive()) return false;

    // Load calibration data
    cv::Mat left_intr(3,3,CV_64F);
    cv::Mat right_intr(3,3,CV_64F); 
    cv::Mat left_dist(1,5,CV_64F);
    cv::Mat right_dist(1,5,CV_64F);
    cv::Mat R(3,3,CV_64F);
    cv::Mat T(3,1,CV_64F);

    this->info("Loading left camera calibration from ", left_cal);
    if(!load_camera_matrices(left_cal, left_dist, left_intr))
    {
        this->error("Failed to load left camera calibration");
        return false;
    }
    this->info("Loading right camera calibration from ", right_cal);
    if(!load_camera_matrices(right_cal, right_dist, right_intr))
    {
        this->error("Failed to load right camera calibration");
        return false;
    }
    this->info("Loading stereo calibration from ", stereo_cal);
    if(!load_stereo_matrices(stereo_cal, R, T))
    {
        this->error("Failed to load stereo calibration");
        return false;
    }

    this->info("Computing stereo projection");
    cv::Mat _Rl, _Rr, _Pl, _Pr;
    cv::stereoRectify(
        left_intr, left_dist,
        right_intr, right_dist,
        cv::Size(DepthCamera::Width, DepthCamera::Height),
        R, T, 
        _Rl, _Rr, _Pl, _Pr,
        Q,
        cv::CALIB_ZERO_DISPARITY,
        0 
    );

    if(!load_json_value_pairs(
        stereo_param,
        std::make_tuple("stereo"),
        "minDistance", min_dist,
        "maxDistance", max_dist
    ))
    {
        this->error("Failed to load point cloud distance settings.");
        return false;
    }

    // Load filtering settings
    if(!load_json_value_pairs(
        stereo_param,
        std::make_tuple("filter"),
        "outlier_n", filter_n,
        "outlier_r", filter_r
    ))
    {
        this->error("Failed to load point cloud filter settings.");
        return false;
    }

    // Allocate intermediate
    point_cloud = cv::Mat(DepthCamera::Height, DepthCamera::Width, CV_32FC3);
    
    latest_disparity = stereo->acquire();
    return true;
}


template<uint8_t X> 
void _PointCloud<X>::step()
{
    this->tick();
    // Compute point cloud
    const cv::Mat disparity(DepthCamera::Height, DepthCamera::Width, CV_16S, const_cast<int16_t*>(latest_disparity->data.disparity));
    cv::reprojectImageTo3D(disparity, point_cloud, Q, false);
    // Partial sort confidence and indices to get low and high confidence partitions
    for(size_t i = 0; i < DepthCamera::Height*DepthCamera::Width; ++i)
    {
        confidence_w_indices[i].first = latest_disparity->data.confidence[i];
        confidence_w_indices[i].second = i;
    }
    const size_t split_idx = (DepthCamera::Height*DepthCamera::Width) - num_points<X>();
    std::partial_sort(
        confidence_w_indices, 
        confidence_w_indices+split_idx, 
        confidence_w_indices+(DepthCamera::Height*DepthCamera::Width),
        [](const std::pair<uint8_t, size_t>& a, const std::pair<uint8_t, size_t>& b)
        {
            return a.first < b.first;
        }
    );
    // Copy best points to new update
    auto next = this->allocate_next();        
    next->data.n_valid = 0;
    next->data.disparity = latest_disparity;
    auto point_clout_ptr = reinterpret_cast<float*>(point_cloud.data);
    for(size_t i = split_idx; i < DepthCamera::Height*DepthCamera::Width; ++i)
    {
        auto idx_orig = confidence_w_indices[i].second;
        if((point_clout_ptr[3*idx_orig + 2] >= min_dist) && (point_clout_ptr[3*idx_orig + 2] <= max_dist))
        {
            // Copy best points and convert to mm
            next->data.cloud[3*next->data.n_valid] = point_clout_ptr[3*idx_orig] * 1000.0; // x
            next->data.cloud[3*next->data.n_valid + 1] = point_clout_ptr[3*idx_orig + 1] * 1000.0; // y
            next->data.cloud[3*next->data.n_valid + 2] = point_clout_ptr[3*idx_orig + 2] * 1000.0; // z
            ++next->data.n_valid;
        }
    }
    this->swap_data();

    // Wait for next update
    while(!latest_disparity->stale){}
    latest_disparity = stereo->acquire();
}
