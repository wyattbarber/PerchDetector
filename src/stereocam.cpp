#include <stereocam.hpp>

std::unique_ptr<libcamera::CameraManager> stereo::cm;
std::shared_ptr<CameraWrapper> stereo::left, stereo::right;
std::shared_ptr<DepthCamera> stereo::depth;

// CAM1: /base/soc/i2c0mux/i2c@1/imx219@10 
// CAM0: /base/soc/i2c0mux/i2c@0/imx219@10
static bool id_cam_left(const std::string& id){return id.find("i2c@0/imx219@10") != std::string::npos;}
static bool id_cam_right(const std::string& id){return id.find("i2c@1/imx219@10") != std::string::npos;}

void stereo::setup()
{
    cm = std::make_unique<libcamera::CameraManager>();
    cm->start();
    left = std::make_shared<CameraWrapper>("left-camera", cm, id_cam_left);
    right = std::make_shared<CameraWrapper>("right-camera", cm, id_cam_right);
    depth = std::make_shared<DepthCamera>(left, right);
    
    left->acquire();
    left->configure();
    right->acquire();
    right->configure();

    depth->initialize();
}

void stereo::start()
{
    left->start();
    right->start();
}


void stereo::stop()
{
    left->stop();
    right->stop();
}


void stereo::teardown()
{
    left->release();
    right->release();
    cm->stop();
}