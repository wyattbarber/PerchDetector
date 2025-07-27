#include <CameraWrapper.hpp>
#include <ImageSender.hpp>
#include <ArgParser.hpp>
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

// Extern main functions for the two operation modes
int main_headless(ArgParser&);
int main_gui(ArgParser&);

int main(int argc, char** argv)
{
    signal(SIGINT, sig_handle);
    
    ArgParser args(argc, argv);
    if(!args.valid())
    {
        std::cerr << "Provided arguments invalid." << std::endl;
        return -1;
    }
    
    if(args.headless())
    {
        return main_headless(args);
    }
    else
    {
        return main_gui(args);
    }
}