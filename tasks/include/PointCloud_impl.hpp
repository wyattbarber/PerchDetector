#pragma once

#include <algorithm>
#include <json_loader.hpp>


template<uint8_t N, uint8_t M>
bool _PointCloud<N,M>::start_impl()
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

    baseline = cv::norm(T);

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
        "outlier_r", filter_r,
        "voxel_size", filter_v
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


template<typename T>
auto max_confidence_pool(const Eigen::MatrixBase<T>& confidence, const cv::Mat& points)
{
    Eigen::Index maxRow, maxCol;
    confidence.maxCoeff(&maxRow, &maxCol);
    return points.at<cv::Vec3f>(maxRow, maxCol);
}


template<uint8_t N, uint8_t M>
void _PointCloud<N,M>::step()
{
    this->tick();
    // Compute point cloud and convert confidence to Eigen Matrix
    const cv::Mat disparity(
        DepthCamera::Height, DepthCamera::Width, CV_16S, const_cast<int16_t*>(latest_disparity->data.disparity)
    );
    cv::reprojectImageTo3D(disparity / 16.0, point_cloud, Q, false);
    Eigen::Map<const Eigen::Matrix<uint8_t, DepthCamera::Height, DepthCamera::Width>> confidence(
        latest_disparity->data.confidence
    );

    // Create new update    
    auto next = this->allocate_next();  
    next->data.n_valid = 0;
    next->data.disparity = latest_disparity;

    // Copy points after downsampling
    for(unsigned i = 0; i <= DepthCamera::Height - N; i += N)
    {
        for(unsigned j = 0; j <= DepthCamera::Width - M; j += M)
        {   
            // Select highest confidence point in this block
            auto conf_region = confidence.block<N,M>(i,j);
            auto point = max_confidence_pool(conf_region, point_cloud(cv::Rect(j, i, M, N)));
            // Copy best points and convert to mm, with origin between cameras
            next->data.cloud[3*next->data.n_valid] = point[0] * 1000.0; // x
            next->data.cloud[3*next->data.n_valid + 1] = point[1] * 1000.0; // y
            next->data.cloud[3*next->data.n_valid + 2] = point[2] * 1000.0; // z
            next->data.n_valid += 1;
        }
    }
    this->swap_data();

    // Wait for next update
    while(!latest_disparity->stale){}
    latest_disparity = stereo->acquire();
}
