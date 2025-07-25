#include <CameraWrapper.hpp>
#include <csignal>
#include <functional>
#include <chrono>
#include <thread>

using namespace libcamera;
using namespace std::chrono_literals;

std::unique_ptr<CameraManager> cm;
std::unique_ptr<CameraWrapper> wrapper;

void sig_handle(int signum)
{
    wrapper->stop();
    wrapper->release();
    cm->stop();
    exit(signum);
}

int main(int argc, char** argv)
{
    signal(SIGINT, sig_handle);

    cm = std::make_unique<CameraManager>();
    cm->start();
    wrapper = std::make_unique<CameraWrapper>("testcamera", cm, [](const std::string& id){ return true; });

    wrapper->acquire();
    wrapper->configure();
    wrapper->start();

    std::chrono::steady_clock::time_point ts, tf;
    cv::Mat m;
    
    while(true)
    
        std::this_thread::sleep_for(10ms);
    
    return 0;
}