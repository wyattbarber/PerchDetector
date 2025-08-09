#include <CameraWrapper.hpp>
#include <ImageSender.hpp>
#include <ArgParser.hpp>
#include <stereocam.hpp>
#include <visualization.hpp>
#include <functional>
#include <csignal>
#include <chrono>
#include <thread>
#include <string>
#include <opencv2/core/mat.hpp>

using namespace libcamera;
using namespace std::chrono_literals;



static cv::Mat im_placeholder;  /// Needed to convert image to zero-stride before sending to python


static void sig_handle(int signum)
{
    visualization::teardown();
    stereo::stop();
    stereo::teardown();
    exit(signum);
}


int main_gui(ArgParser& args)
{
    signal(SIGINT, sig_handle);

    stereo::setup();

    im_placeholder = cv::Mat(stereo::left->get_height(), stereo::left->get_width(), stereo::left->cvtype);

    auto res = visualization::setup(args, stereo::left, stereo::right);
    if(res != 0)
    {
        return res;
    }

    stereo::start();
    std::cout << "Started image feed" << std::endl;
    std::this_thread::sleep_for(100ms); // Makes sure image data is populated before calculating depth

    while(true)
    {        
        std::cout << "Writing one frame to the display" << std::endl;
        // Write grayscale image
        stereo::left->lock();
        stereo::left->image()->copyTo(im_placeholder);
        stereo::left->unlock();
        visualization::image->acquire();
        memcpy(visualization::image->get_image()->data, im_placeholder.data, im_placeholder.total() * im_placeholder.elemSize());
        visualization::image->release();
        // Write depth image
        stereo::depth->lock();
        auto d = *stereo::depth->disparity();
        memcpy(visualization::depth->get_image()->data, d.data, d.total() * d.elemSize());
        stereo::depth->unlock();
        std::this_thread::sleep_for(100ms);
        if(visualization::params->check())
        {
            std::cerr << "Changing stereo camera parameters" << std::endl;
            visualization::params->lock();
            stereo::depth->set_params(
                visualization::params->minDisparity(),
                visualization::params->numDisparities(),
                visualization::params->blockSize(),
                visualization::params->P1(),
                visualization::params->P2(),
                visualization::params->disp12MaxDiff(),
                visualization::params->preFilterCap(),
                visualization::params->uniquenessRatio(),
                visualization::params->speckleWindowSize(),
                visualization::params->speckleRange()
            );
            visualization::params->unlock();
        }
    }

    return 0;
}