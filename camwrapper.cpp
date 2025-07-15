#include <CameraWrapper.hpp>
#include <csignal>
#include <functional>
#include <chrono>

using namespace libcamera;

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
    wrapper->configure(640, 480);
    wrapper->start();

    std::chrono::steady_clock::time_point ts, tf;
    unsigned n_planes;
    while(true)
    {
        ts = std::chrono::steady_clock::now();
        wrapper->capture_start();
        n_planes = wrapper->capture_wait()->metadata().planes().size();
        tf = std::chrono::steady_clock::now();
        std::cout << "Main: Captured buffer with " << n_planes << " planes in " 
            << std::chrono::duration_cast<std::chrono::milliseconds>(tf - ts).count() << " ms." << std::endl;
    }
    
    return 0;
}