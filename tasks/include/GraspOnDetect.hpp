#pragma once

#include "Task.hpp"
#include "GrasperCtl.hpp"
#include "LineFinder.hpp"


class GraspOnDetect : public TaskBase<GraspOnDetect>
{
public:
    GraspOnDetect(const char* name, const std::string& settings, std::shared_ptr<LineFinder> line_detect, std::shared_ptr<GrasperController> grasp_ctl) : 
        TaskBase(name, {line_detect, grasp_ctl}),
        settings(settings + "/grasp_ctrl.json"),
        line_detect(line_detect),
        grasp_ctl(grasp_ctl)
        {}

    bool start_impl();

    void stop_impl();

    void step();

protected:
    const std::string settings;
    std::shared_ptr<LineFinder> line_detect;
    std::shared_ptr<GrasperController> grasp_ctl;
    LineFinder::update_ptr_const_type latest_detect;

    float max_offset; /// Max horizontal offset from center to allow grasping
    float max_dist; /// Maximum vertical distance to allow grasping
    float max_z_tilt_rad; /// Maximum angle of the perch from horizontal to allow grasping
    float max_y_tilt_rad; /// Maximum angle of the perch from camera vertical to allow grasping
};