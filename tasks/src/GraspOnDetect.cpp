#include "GraspOnDetect.hpp"
#include <json_loader.hpp>
#include <cmath>

bool GraspOnDetect::start_impl()
{
    float max_z_tilt_deg, max_y_tilt_deg;
    if(!load_json_value_pairs(settings,
        std::make_tuple(),
        "max_offset", max_offset,
        "max_dist", max_dist,
        "max_z_tilt", max_z_tilt_deg,
        "max_y_tilt", max_y_tilt_deg
    )) return false;
    max_z_tilt_rad = max_z_tilt_deg * M_PI / 180.0;
    max_y_tilt_rad = max_y_tilt_deg * M_PI / 180.0;
    return true;
}


void GraspOnDetect::stop_impl()
{}


void GraspOnDetect::step()
{
    latest_detect = line_detect->acquire();
    if(latest_detect->data.valid)
    {
        const Line* line = &latest_detect->data.lines[latest_detect->data.selected_line];
        auto a = Eigen::Map<const Eigen::Vector3f>(line->anchor);
        auto d = Eigen::Map<const Eigen::Vector3f>(line->dir);    
        float angle_z = std::acos(d(2) / d.norm()); // Angle in radians to z axis
        const Eigen::Vector3f d_xy = {d(0), d(1), 0};
        float angle_y = std::acos(d_xy(1) / d_xy.norm()); // Angle in radians to y axis 
        
        bool angle_z_ok = max_z_tilt_rad >= ((M_PI / 2.0) - angle_z);
        bool angle_y_ok = max_y_tilt_rad >= angle_y;
        bool dist_h_ok = max_offset >=  std::abs(a(0));
        bool dist_v_ok = max_dist >= a(2);

        if(
            angle_y_ok && angle_z_ok && dist_h_ok && dist_v_ok
        ) {
            grasp_ctl->grasp();
        }
    }
}
