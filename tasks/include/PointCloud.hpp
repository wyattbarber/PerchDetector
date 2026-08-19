#pragma once

#include "DataSource.hpp"
#include "Disparity.hpp"
#include <opencv2/calib3d.hpp>

/// Point cloud size
template<uint8_t N, uint8_t M> 
constexpr size_t num_points(){ return (DepthCamera::Height / N) * (DepthCamera::Width / M); }

/// Point cloud type, array of [x0,y0,z0,x1,y1,z1,x2...] coordinates in mm
template<uint8_t N, uint8_t M> 
class stereo_point_cloud_t
{
public:
    float cloud[3*num_points<N,M>()];
    size_t n_valid;
    DepthCamera::update_ptr_const_type disparity;
};


// Forward declarations and traits for datasource interface
template<uint8_t N, uint8_t M> class _PointCloud; 
template<uint8_t N, uint8_t M>
struct DataSourceTraits<_PointCloud<N,M>>
{ 
    using value_type = stereo_point_cloud_t<N,M>; 
}; 


typedef _PointCloud<2,2> PointCloud;

/** Converts disparity data into point clouds.

Uses disparity task disparity and confidence data to 
produce a point cloud containing the top X percent of
points, ranked by confidence

@tparam X Percentage of pixels to keep in the point cloud, value 1-100
*/
template<uint8_t N, uint8_t M>
class _PointCloud : public DataSource<_PointCloud<N,M>>
{
public:
    /// Number of points in the output point cloud after filtering
    static constexpr size_t NumPoints = num_points<N,M>();

    /** Create new point cloud generator.
    
    @param name Name of the new task.
    @param stereo Disparity task to get data from.
    @param cal_path Path to folder containing stereo calibration and settings.
    */
    _PointCloud(const char* name, std::shared_ptr<DepthCamera> stereo, const std::string& cal_path):
        DataSource<_PointCloud<N,M>>(name, {stereo}),
        stereo(stereo),        
        left_cal(cal_path + "/left.json"),
        right_cal(cal_path + "/right.json"),
        stereo_cal(cal_path + "/stereo_calibration.json"),
        stereo_param(cal_path + "/stereo_settings.json")
    {
        static_assert((N >= 1)  && (N <= DepthCamera::Height/2), "N must be an integer between 1 and half the image height.");
        static_assert((M >= 1)  && (M <= DepthCamera::Width/2), "M must be an integer between 1 and half the image width.");
    }
    void step();

    bool start_impl();

    void stop_impl(){}

    std::vector<size_t> dims(){ return {NumPoints, 3}; }

    /** Provides the maximum dimensions of the point cloud.
    
    Returns the dimensions, x (width), y (height), and z (depth),
    of the bounding box the point cloud may occupy.

    @return Bounding box dimensions.
    */
    auto volume() { return stereo->volume(); }
  
protected:
    std::shared_ptr<DepthCamera> stereo;
    typename DepthCamera::update_ptr_const_type latest_disparity;
    
    const std::string left_cal, right_cal, stereo_cal, stereo_param;
    cv::Mat Q, point_cloud;
    std::pair<uint8_t, size_t> unconfidence_w_indices[DepthCamera::Height*DepthCamera::Width];

    double max_dist, min_dist;
    unsigned filter_n;
    double filter_r, filter_v;
    float baseline;
};

/** Converts point clouds to flat stream for sharing
*/
template<uint8_t N, uint8_t M>
class _point_cloud_conv
{ 
public:
    using conversion_type = float[3*_PointCloud<N,M>::NumPoints];
    static std::vector<size_t> dims() { return {3, _PointCloud<N,M>::NumPoints}; }
    static void eval(void* dst, const typename _PointCloud<N,M>::value_type& src)
    {
        memcpy(dst, (void*)src.cloud, 3*_PointCloud<N,M>::NumPoints*sizeof(float));
    }
};
using point_cloud_conv = _point_cloud_conv<2,2>;

#include "PointCloud_impl.hpp"