#include <CameraWrapper.hpp>
#include <ImageSender.hpp>
#include <ArgParser.hpp>
#include <functional>
#include <chrono>
#include <thread>
#include <opencv2/core/mat.hpp>

using namespace libcamera;
using namespace std::chrono_literals;

std::unique_ptr<CameraManager> cm;
std::unique_ptr<CameraWrapper> camera_left, camera_right;

cv::Mat im_placeholder;
std::unique_ptr<ImageSender> image_send, depth_send;

bool id_cam_left(const std::string& id){return false;}
bool id_cam_right(const std::string& id){return false;}

int main_gui(ArgParser& args)
{
    cm = std::make_unique<CameraManager>();
    cm->start();
    camera_left = std::make_unique<CameraWrapper>("testcamera", cm, id_cam_left);
    camera_right = std::make_unique<CameraWrapper>("testcamera", cm, id_cam_right);
    
    camera_left->acquire();
    camera_left->configure();
    camera_right->acquire();
    camera_right->configure();

    im_placeholder  cv::Mat(camera_left->get_height(), camera_left->get_width(), camera_left->cvtype)

    image_send = make_sender<camera_left->type>(
        args.image_map_file(), camera_left->get_width(), camera_left->get_height()
    );
    image_send->start();
    if(!image_send->opened())
    {
        std::cerr << "Failed to setup image communication channel" << std::endl;
        return -1;
    }

    depth_send = make_sender<camera_left->type>(
        args.image_depth_file(), camera_left->get_width(), camera_left->get_height()
    );
    depth_send->start();
    if(!depth_send->opened())
    {
        std::cerr << "Failed to setup depth communication channel" << std::endl;
        return -1;
    }

    camera_left->start();
    camera_right->start();

    while(true)
    {        
        std::cout << "Writing one frame to the display" << std::endl;
        camera_left->lock();
        camera_left->image()->copyTo(im_placeholder);
        camera_left->unlock();
        depth_send->acquire();
        memcpy(depth_send->get_image()->data, im_placeholder.data, im_placeholder.total() * im_placeholder.elemSize());
        depth_send->release();
        std::this_thread::sleep_for(100ms);
    }

    return 0;
}