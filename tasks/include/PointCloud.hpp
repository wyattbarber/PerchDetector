#pragma once

#include "DataSource.hpp"
#include "Disparity.hpp"
#include <opencv2/calib3d.hpp>

/// Point cloud size
template<uint8_t X> 
constexpr size_t num_points(){ return (DepthCamera::Height * DepthCamera::Width * X) / 100; }

/// Point cloud type, array of [x0,y0,z0,x1,y1,z1,x2...] coordinates in mm
template<uint8_t X> using stereo_point_cloud_t = float[3*num_points<X>()];


// Forward declarations and traits for datasource interface
template<uint8_t X> class PointCloud; 
template<uint8_t X> 
struct DataSourceTraits<PointCloud<X>>
{ 
    using value_type = stereo_point_cloud_t<X>; 
}; 


/** Converts disparity data into point clouds.

Uses disparity task disparity and confidence data to 
produce a point cloud containing the top X percent of
points, ranked by confidence

@tparam X Percentage of pixels to keep in the point cloud, value 1-100
*/
template<uint8_t X>
class PointCloud : public DataSource<PointCloud<X>>
{
public:
    /** Create new point cloud generator.
    
    @param name Name of the new task.
    @param stereo Disparity task to get data from.
    @param cal_path Path to folder containing stereo calibration and settings.
    */
    PointCloud(const char* name, std::shared_ptr<DepthCamera> stereo, const std::string& cal_path):
        DataSource<PointCloud<X>>(name, {stereo}),
        stereo(stereo),        
        left_cal(cal_path + "/left.json"),
        right_cal(cal_path + "/right.json"),
        stereo_cal(cal_path + "/stereo_calibration.json"),
        stereo_param(cal_path + "/stereo_settings.json")
    {
        static_assert((X >= 1)  && (X <= 100), "Percent of points kept must be an integer between 1 and 100");
    }
    void step();

    bool start_impl();

    void stop_impl(){}

    std::vector<size_t> dims(){ return {3, num_points<X>()}; }
  
protected:
    std::shared_ptr<DepthCamera> stereo;
    typename DepthCamera::update_ptr_const_type latest_disparity;
    
    const std::string left_cal, right_cal, stereo_cal, stereo_param;
    cv::Mat Q, point_cloud;
    std::pair<uint8_t, size_t> confidence_w_indices[DepthCamera::Height*DepthCamera::Width];
};


#include "PointCloud_impl.hpp"