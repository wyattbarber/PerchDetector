#pragma once


#include "DataSource.hpp"
#include "PointCloud.hpp"
#include <hough3d_tform.hpp>


typedef struct {
    double anchor[3];
    double dir[3];
} LineFinderUpdate_t;

FWD_DECL_DATA_SOURCE(LineFinder, LineFinderUpdate_t)


class LineFinder : public DataSource<LineFinder>
{
public:
    /** Create new point cloud generator.
    
    @param name Name of the new task.
    @param cloud PointCloud task to get data from.
    @param cal_path Path to folder containing calibration and settings.
    */
    LineFinder(const char* name, std::shared_ptr<PointCloud> cloud, const std::string& cal_path):
        DataSource<LineFinder>(name, {cloud}),
        cloud(cloud),        
        settings(cal_path + "/detector.json")
    {}

    void step();

    bool start_impl();

    void stop_impl();

    std::vector<size_t> dims(){ return {1}; }
  
protected:
    std::shared_ptr<PointCloud> cloud;
    PointCloud::update_ptr_const_type latest;
    const std::string settings;

    unsigned min_vote;
    size_t max_lines;
    size_t granularity; 
    double min_width, max_width;
    double min_ratio;
    double max_angle;
};