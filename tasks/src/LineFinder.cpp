#include "LineFinder.hpp"
#include <hough3d_tform.hpp>
#include "json_loader.hpp"
#include <Eigen/Dense>


/** Selects the best candidate line.

This function identifies the line that is most likely to be 
a suitable perch. The best method for this found so far is to simply
take the closest (smallest Z coordinate) line.

@param lines Vector of parameters for lines detected in a point cloud.

@return Index of the best candidate in the input vector.
*/
size_t best_line_idx(const std::vector<Line>& lines)
{
    return 0;
}


void LineFinder::step()
{
    if(latest){
        if(!latest->stale) return; // No update
    }
    latest = cloud->acquire();
    tick();

    // Process new point cloud for lines
    const Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> points(const_cast<float*>(latest->data), 3, cloud->dims()[0]);
    const auto lines = hough3d(
        points.cast<double>(), 
        *hough,
        min_vote, 
        max_lines,
        min_width, 
        max_width, 
        min_ratio,
        max_angle
    );

    if(lines.size() > 0)
    {
        // Select the canditate line and form data update
        const auto idx = best_line_idx(lines);
        const auto anchor = std::get<0>(lines[idx]);
        const auto dir = std::get<1>(lines[idx]);
        auto next = allocate_next();
        memcpy((void*)next->data.anchor, (void*)anchor.data(), 3*sizeof(double));
        memcpy((void*)next->data.dir, (void*)dir.data(), 3*sizeof(double));
        swap_data();
    }
    else
    {
        info("No candidates detected.");
    }
}


bool LineFinder::start_impl()
{
    if(!load_json_value_pairs(settings,
        std::make_tuple(),
        "min_vote", min_vote,
        "max_lines", max_lines,
        "granularity", granularity,
        "min_ratio", min_ratio,
        "max_angle", max_angle,
        "min_width", min_width,
        "max_width", max_width
    )) return false;
    
    // estimate size of Hough space. Mostly copied from hough 3d library, with size estimated as worst case point cloud bounding box
    auto volume = cloud->volume();
    Eigen::Vector<double, 3> min_p = {-volume[0]/2.0, -volume[1]/2.0, 0};
    Eigen::Vector<double, 3> max_p = {volume[0]/2.0, volume[1]/2.0, volume[2]};
    double d = (max_p - min_p).norm();
    double opt_dx = d / 64.0;
    hough = new Hough(min_p, max_p, opt_dx, granularity);

    auto next = allocate_next();
    memset((void*)next->data.anchor, 0, 3*sizeof(double));
    memset((void*)next->data.dir, 0, 3*sizeof(double));
    swap_data();

    return true;
}


void LineFinder::stop_impl()
{}

