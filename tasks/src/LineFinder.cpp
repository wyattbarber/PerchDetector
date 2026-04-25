#include "LineFinder.hpp"
#include <hough3d_tform.hpp>
#include "json_loader.hpp"
#include <Eigen/Dense>


void LineFinder::step()
{
    if(latest){
        if(!latest->stale) return; // No update
    }
    latest = cloud->acquire();

    // Process new point cloud for lines
    const Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> points(const_cast<float*>(latest->data), 3, cloud->dims()[0]);
    auto lines = hough3d(
        points.cast<double>(), 
        min_vote, 
        max_lines, 
        granularity, 
        min_width, 
        max_width, 
        min_ratio,
        max_angle
    );
}


bool LineFinder::start_impl()
{
    return load_json_value_pairs(settings,
        std::make_tuple(),
        "min_vote", min_vote,
        "max_lines", max_lines,
        "granularity", granularity,
        "min_ratio", min_ratio,
        "max_angle", max_angle,
        "min_width", min_width,
        "max_width", max_width
    );
}


void LineFinder::stop_impl()
{}

