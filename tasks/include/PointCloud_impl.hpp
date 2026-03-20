#pragma once

#include <algorithm>
#include <json_loader.hpp>


template<uint8_t X> 
bool PointCloud<X>::start_impl()
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
    
    latest_disparity = stereo->acquire();
    return true;
}


template<uint8_t X> 
void PointCloud<X>::step()
{
    if(this->is_alive())
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
        std::partial_sort(confidence_w_indices, confidence_w_indices+num_points<X>(), confidence_w_indices+(DepthCamera::Height*DepthCamera::Width));
        // Copy best points to new update
        auto next = this->allocate_next();        
        auto point_clout_ptr = reinterpret_cast<float*>(point_cloud.data);
        for(size_t i = 0; i < num_points<X>(); ++i)
        {
            auto idx_orig = confidence_w_indices[i].second;
            next->data[3*i] = point_clout_ptr[3*idx_orig]; // x
            next->data[3*i + 1] = point_clout_ptr[3*idx_orig + 1]; // y
            next->data[3*i + 2] = point_clout_ptr[3*idx_orig + 2]; // z
        }
        this->swap_data();

        // Wait for next update
        while(!latest_disparity->stale){}
        latest_disparity = stereo->acquire();
    }
}
