#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>

#include <libcamera/libcamera.h>

using namespace libcamera;

static std::shared_ptr<Camera> camera;

int main(int argc, char** argv)
{
    std::unique_ptr<CameraManager> cm = std::make_unique<CameraManager>();
    cm->start();

    for (auto const &camera : cm->cameras())
        std::cout << camera->id() << std::endl;
    if(cm->cameras().empty())
        std::cout << "No cameras detected in this system." << std::endl;

    return 0;
}