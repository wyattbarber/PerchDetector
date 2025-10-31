#include <stereocam.hpp>
#include <Logging.hpp>
#include <thread>


using namespace std::chrono_literals;


std::unique_ptr<libcamera::CameraManager> stereo::cm;
std::shared_ptr<CameraWrapper> stereo::left, stereo::right;
std::shared_ptr<DepthCamera> stereo::depth;

// CAM1: /base/soc/i2c0mux/i2c@1/imx219@10 
// CAM0: /base/soc/i2c0mux/i2c@0/imx219@10
static bool id_cam_left(const std::string& id){return id.find("i2c@0/imx219@10") != std::string::npos;}
static bool id_cam_right(const std::string& id){return id.find("i2c@1/imx219@10") != std::string::npos;}

static bool running, shutdown;
static bool _depth_on;
static std::thread depth_thread;


static void depth_thread_f()
{
    while(!shutdown)
    {        
        if(running)
        {
            stereo::depth->update();
        }
    }
}


void stereo::setup(bool depth_on, bool color)
{
    _depth_on = depth_on;
    shutdown = false;
    running = false;

    cm = std::make_unique<libcamera::CameraManager>();
    cm->cameraAdded.connect<void>([](std::shared_ptr<libcamera::Camera> cam){ Logger::instance() << cam->id() << " connected." << std::endl; });
    cm->cameraRemoved.connect<void>([](std::shared_ptr<libcamera::Camera> cam){ Logger::instance() << cam->id() << " removed." << std::endl; });
    cm->start();
    left = std::make_shared<CameraWrapper>("left-camera", cm, id_cam_left, color);
    right = std::make_shared<CameraWrapper>("right-camera", cm, id_cam_right, color);
    
    left->acquire();
    left->configure();
    right->acquire();
    right->configure();

    if(_depth_on)
    {
        depth = std::make_shared<DepthCamera>(left, right);
        depth->initialize();
        depth_thread = std::thread(depth_thread_f);
    }
}

void stereo::start()
{
    left->start();
    right->start();
    std::this_thread::sleep_for(100ms);
    running = true;
}


void stereo::stop()
{
    running = false;
    left->stop();
    right->stop();
}


void stereo::teardown()
{
    shutdown = true;
    if(_depth_on)
    {
        depth_thread.join();
    }
    left->release();
    right->release();
    cm->stop();
}