#include <CameraWrapper.hpp>
#include <ArgParser.hpp>
#include <Depth.hpp>
#include <Logging.hpp>
#include <stereocam.hpp>
#include <functional>
#include <chrono>
#include <thread>
#include <csignal>
#include <opencv2/core/mat.hpp>

using namespace libcamera;
using namespace std::chrono_literals;

static void sig_handle(int signum)
{
    stereo::stop();
    stereo::teardown();
    exit(signum);
}


int main_headless(ArgParser& args)
{
    signal(SIGINT, sig_handle);

    stereo::setup();
    stereo::start();
    std::this_thread::sleep_for(100ms); // Makes sure image data is populated before calculating depth
    while(true)
    {        
        Logger::instance() << "Stereo cameras running..." << std::endl;
        std::this_thread::sleep_for(100ms);
    }

    return 0;
}