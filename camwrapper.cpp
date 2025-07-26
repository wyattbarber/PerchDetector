#include <CameraWrapper.hpp>
#include <ImageSender.hpp>
#include <csignal>
#include <functional>
#include <chrono>
#include <thread>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>

using namespace libcamera;
using namespace std::chrono_literals;

std::unique_ptr<CameraManager> cm;
std::unique_ptr<CameraWrapper> wrapper;
std::unique_ptr<ImageSender> sender;

void sig_handle(int signum)
{
    sender->stop();
    wrapper->stop();
    wrapper->release();
    cm->stop();
    exit(signum);
}

int main(int argc, char** argv)
{
    if(argc < 2)
    {
        std::cerr << "filename must be passed from command line" << std::endl;
        return -1;
    }
    
    signal(SIGINT, sig_handle);

    cm = std::make_unique<CameraManager>();
    cm->start();
    wrapper = std::make_unique<CameraWrapper>("testcamera", cm, [](const std::string& id){ return true; });
    
    wrapper->acquire();
    wrapper->configure();

    char * file = argv[1];
    sender = make_sender<uint8_t>(file, 
        std::get<1>(wrapper->shape()), std::get<0>(wrapper->shape()));
    sender->start();
    if(!sender->opened())
    {
        std::cerr << "Failed to setup communication channel" << std::endl;
        return -1;
    }

    wrapper->start();

    std::cout << "Started image feed" << std::endl;
    cv::Mat placeholder(std::get<0>(wrapper->shape()), std::get<1>(wrapper->shape()), cv::DataType<uint8_t>::type);
    
    while(true)
    {        
        std::cout << "Writing one frame to the display" << std::endl;
        wrapper->lock();
        wrapper->image()->copyTo(placeholder);
        wrapper->unlock();
        sender->acquire();
        memcpy(sender->get_image().data, placeholder.data, placeholder.total() * placeholder.elemSize());
        sender->release();
        cv::imwrite("testim.png", placeholder);
        std::this_thread::sleep_for(1s);
    }

    return 0;
}