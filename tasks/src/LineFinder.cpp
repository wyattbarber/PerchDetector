#include "LineFinder.hpp"
#include <hough3d_tform.hpp>
#include "json_loader.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <cmath>
#include <algorithm>


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
    const Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> points(
        const_cast<float*>(latest->data.cloud),
        3, latest->data.n_valid
    );
    const auto lines = hough3d(
        points.cast<double>(), 
        *hough,
        min_vote, 
        max_lines,
        min_width, 
        max_width, 
        min_ratio,
        max_angle,
        center
    );

    auto next = allocate_next();
    if(lines.size() > 0)
    {
        // Select the canditate line and form data update
        next->data.valid = true;
        const auto idx = best_line_idx(lines);
        next->data.n_lines = lines.size();
        next->data.selected_line = idx;

        for(size_t i = 0; i < std::min(static_cast<unsigned>(lines.size()), MAX_LINES); ++i)
        {
            const auto anchor = std::get<0>(lines[i]);
            const auto dir = std::get<1>(lines[i]);
            memcpy((void*)next->data.lines[i].anchor, (void*)anchor.data(), 3*sizeof(double));
            memcpy((void*)next->data.lines[i].dir, (void*)dir.data(), 3*sizeof(double));
        }
        next->data.pointcloud = latest;
    }
    else
    {        
        // Flag update as invalid
        next->data.valid = false;
        next->data.pointcloud = latest;
        warning("No candidates detected.");
    }
    swap_data();
}


bool LineFinder::start_impl()
{
    double max_angle_deg;
    if(!load_json_value_pairs(settings,
        std::make_tuple(),
        "min_vote", min_vote,
        "max_lines", max_lines,
        "granularity", granularity,
        "min_ratio", min_ratio,
        "max_angle", max_angle_deg,
        "min_width", min_width,
        "max_width", max_width
    )) return false;
    max_angle = max_angle_deg * M_PI / 180.0;
    
    // estimate size of Hough space. Mostly copied from hough 3d library, with size estimated as worst case point cloud bounding box
    auto volume = cloud->volume();
    auto min_p = Eigen::Vector<double, 3>{-volume[0]/2.0, -volume[1]/2.0, 0} * 1000.0;
    auto max_p = Eigen::Vector<double, 3>{volume[0]/2.0, volume[1]/2.0, volume[2]} * 1000.0;
    center = Eigen::Vector<double, 3>{0.0, 0.0, volume[2]/2.0} * 1000.0;
    hough = new Hough(min_p - center, max_p - center, 0, granularity);
    info("Configured Hough space with volume of ", volume[0], "m x ", volume[1], "m x ", volume[2], "m, and resolution of ", hough->dx, "mm");

    return true;
}


void LineFinder::stop_impl()
{}


bool _wait_flag_given(const std::vector<std::string>& args)
{
    if(args.size() > 1)
    {
        return (args[1] == "--next-valid");
    }
    return false;
}


void save_line_detect(Task* task, std::istream& in, std::ostream& out, const std::vector<std::string>& args)
{
    // Initialize
    auto taskptr = (LineFinder*)(task);
    const auto filename = args[0];
    std::ofstream file(filename, std::ios::out | std::ios::binary);
    if(!file.is_open())
    {
        out << "Failed to open " << filename << std::endl;
        return;
    }

    bool ready = !_wait_flag_given(args);
    LineFinder::update_ptr_const_type update = taskptr->acquire();
    // Wait for valid data if not ready
    while(!ready)
    {
        update = taskptr->acquire();
        ready = update->data.valid;
    }

    // Pack data
    const uint32_t n = static_cast<uint32_t>(update->data.pointcloud->data.n_valid);
    const uint32_t w = static_cast<uint32_t>(CameraWrapper::Width);
    const uint32_t h = static_cast<uint32_t>(CameraWrapper::Height);
    const uint8_t idx = update->data.selected_line;

    file.write((char*)&update->data.lines[idx].anchor, 3*sizeof(double));
    file.write((char*)&update->data.lines[idx].dir, 3*sizeof(double));
    file.write((char*)&n, sizeof(uint32_t));
    file.write((char*)&update->data.pointcloud->data.cloud, n * 3 * sizeof(float));
    file.write((char*)&w, sizeof(uint32_t));
    file.write((char*)&h, sizeof(uint32_t));
    file.write((char*)&update->data.pointcloud->data.disparity->data.left_img->data, w*h*sizeof(uint8_t));
    file.write((char*)&update->data.pointcloud->data.disparity->data.right_img->data, w*h*sizeof(uint8_t));
    file.write((char*)&update->data.pointcloud->data.disparity->data.confidence, w*h*sizeof(uint8_t));
    file.write((char*)&update->data.pointcloud->data.disparity->data.disparity, w*h*sizeof(int16_t));
    file.write((char*)&update->data.n_lines-1, sizeof(uint8_t));
    for(unsigned i = 0; i < std::min(static_cast<unsigned>(update->data.n_lines), MAX_LINES); ++i)
    {
        if(i != idx)
        {
            file.write((char*)&update->data.lines[i].anchor, 3*sizeof(double));
            file.write((char*)&update->data.lines[i].dir, 3*sizeof(double));
        }
    }
}


void report_line_detect(Task* task, std::istream& in, std::ostream& out, const std::vector<std::string>& args)
{
    bool ready = !_wait_flag_given(args);
    auto taskptr = (LineFinder*)(task);
    LineFinder::update_ptr_const_type update = taskptr->acquire();
    // Wait for valid data if not ready
    while(!ready)
    {
        update = taskptr->acquire();
        ready = update->data.valid;
    }

    if(!update->data.valid)
    {
        out << "No perch detected." << std::endl;
        return;
    }

    auto idx = update->data.selected_line;
    auto a = Eigen::Map<const Eigen::Vector3d>(update->data.lines[idx].anchor);
    auto d = Eigen::Map<const Eigen::Vector3d>(update->data.lines[idx].dir);
    double angle_z = std::acos(d(2) / d.norm()); // Angle in radians to z axis 
    const Eigen::Vector3d d_xy = {d(0), d(1), 0};
    double angle_y = std::acos(d_xy(1) / d_xy.norm()); // Angle in radians to y axis 


    out << "Perch found." << std::endl;
    out << "\tDistance: " << a[2] << " mm" << std::endl;
    out << "\tAnchor Point: " << a << std::endl;
    out << "\tAngle From Camera Plane: " << 90.0 - (angle_z * 180.0 / M_PI) << " degrees" << std::endl;
    out << "\tAngle From Camera Vertical: " << angle_y * 180.0 / M_PI << " degrees" << std::endl;
}