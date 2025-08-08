#include <visualization.hpp>
#include <Depth.hpp>

std::unique_ptr<ImageSender> visualization::image, visualization::depth;


int visualization::setup(
    ArgParser& args, 
    std::shared_ptr<CameraWrapper> camera_left, 
    std::shared_ptr<CameraWrapper> camera_right)
{
    image = make_sender<CameraWrapper::dtype>(
        args.image_map_file(), camera_left->get_width(), camera_left->get_height()
    );
    image->start();
    if(!image->opened())
    {
        std::cerr << "Failed to setup image communication channel" << std::endl;
        return -1;
    }

    depth = make_sender<DepthCamera::dtype>(
        args.depth_map_file(), camera_left->get_width(), camera_left->get_height()
    );
    depth->start();
    if(!depth->opened())
    {
        std::cerr << "Failed to setup depth communication channel" << std::endl;
        return -1;
    }

    return 0;
}

void visualization::teardown()
{
    image->stop();
    depth->stop();
}