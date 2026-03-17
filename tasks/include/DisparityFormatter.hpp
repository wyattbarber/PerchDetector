#pragma once

#include "Disparity.hpp"
#include <Eigen/Dense>


FWD_DECL_DATA_SOURCE(DisparityFormatter, decltype(DisparityUpdate_t::disparity))


/** Formats disparity data for display

*/
class DisparityFormatter : public DataSource<DisparityFormatter>
{
public:
    /** Construct a new disparity formatter.
    
    @param name Task name
    @param task Task producing the disparity data
    */  
    DisparityFormatter(const char* name, std::shared_ptr<DepthCamera> task) : 
        DataSource<DisparityFormatter>(name, {task}),
        task(task)
    {
    }

    void step()
    { 
        if(is_alive())
        {
            if(!latest->stale) return;
            tick();
            latest = task->acquire();
            update_ptr_type next = allocate_next();
            using tmp_t = Eigen::Map<Eigen::Matrix<int16_t, CameraWrapper::Height, CameraWrapper::Width>>;
            const tmp_t tmp_src(const_cast<int16_t*>(latest->data.disparity), CameraWrapper::Height, CameraWrapper::Width);
            tmp_t tmp_dst(next->data, CameraWrapper::Height, CameraWrapper::Width); 
            tmp_dst = tmp_src.unaryExpr(
                    [](int16_t x){ return x > 0 ? x : 0; }
                ).cast<int16_t>();
            swap_data();
        }
    }

    bool start_impl()
    { 
        if(!task->is_alive()) return false;
        latest = task->acquire();
        return true;
    }

    void stop_impl()
    {}

    std::vector<size_t> dims(){ return {CameraWrapper::Height, CameraWrapper::Width}; }
  
protected:
    std::shared_ptr<DepthCamera> task;
    typename DepthCamera::update_ptr_const_type latest;
};