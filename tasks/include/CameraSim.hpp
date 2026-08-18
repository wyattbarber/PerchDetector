#pragma once

#include <libcamera/libcamera.h>
#include <memory>
#include <opencv2/core/mat.hpp>
#include "CameraWrapper.hpp"
#include <Eigen/Dense>


FWD_DECL_DATA_SOURCE(CameraSimulator, CameraWrapper::value_type)


typedef struct {
    CameraWrapper::value_type left, right;
} CameraSimData;

FWD_DECL_DATA_SOURCE(CameraSimManager, CameraSimData)


class CameraSimManager : public DataSource<CameraSimManager>
{
    public:
    static const size_t Width = CameraWrapper::Width;
    static const size_t Height = CameraWrapper::Height;

    CameraSimManager(const char* name) : 
        DataSource(name, {})
    {
        declare_cli_command("set-source", set_source_file);
    }

    bool start_impl()
    { 
        auto next = allocate_next();
        memset((void*)next->data.left, 0, Width*Height);
        memset((void*)next->data.right, 0, Width*Height);
        swap_data();
        return true; 
    }

    void stop_impl(){}

    void step(){}

    protected:
    static void set_source_file(Task* task, std::istream& in, std::ostream& out, const std::vector<std::string>& args);
};


class CameraSimulator : public DataSource<CameraSimulator>
{
    public:
    static const size_t Width = CameraWrapper::Width;
    static const size_t Height = CameraWrapper::Height;

    CameraSimulator(const char* name, std::shared_ptr<CameraSimManager> manager, bool right) : 
        DataSource(name, {manager}),
        manager(manager),
        right(right)
    {
    }

    /** Starts the camera 
    */
    bool start_impl(){ return true; }

    /** Stops the camera 
    */
    void stop_impl(){}

    void step();

    std::vector<size_t> dims(){ return {Height, Width}; }

protected:
    std::shared_ptr<CameraSimManager> manager;
    const bool right;
};

