#include <CameraWrapper.hpp>
#include <ArgParser.hpp>
#include <functional>
#include <chrono>
#include <thread>
#include <opencv2/core/mat.hpp>

using namespace libcamera;
using namespace std::chrono_literals;

static std::unique_ptr<CameraManager> cm;
static std::unique_ptr<CameraWrapper> camera_left, camera_right;

static bool id_cam_left(const std::string& id){return false;}
static bool id_cam_right(const std::string& id){return false;}


static void sig_handle(int signum)
{
    camera_left->stop();
    camera_left->release();
    camera_right->stop();
    camera_right->release();
    cm->stop();
    exit(signum);
}


int main_headless(ArgParser& args)
{
    cm = std::make_unique<CameraManager>();
    cm->start();
    camera_left = std::make_unique<CameraWrapper>("testcamera", cm, id_cam_left);
    camera_right = std::make_unique<CameraWrapper>("testcamera", cm, id_cam_right);
    
    camera_left->acquire();
    camera_left->configure();
    camera_right->acquire();
    camera_right->configure();

    camera_left->start();
    camera_right->start();

    while(true)
    {        
        std::cout << "Stereo cameras running..." << std::endl;
        std::this_thread::sleep_for(1s);
    }

    return 0;
}