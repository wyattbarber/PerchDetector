#include <CameraWrapper.hpp>
#include <stereocam.hpp>
#include <chrono>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <thread>
#include <vector>
#include <cstdio>
#include <cmath>
#include <iostream>
#include <fstream>


using namespace std::chrono_literals;
using namespace std;
using namespace cv;


void stop()
{   
    stereo::stop();
    stereo::teardown();
}


static void sig_handle(int signum)
{
    stop();
    exit(signum);
}


Mat left_im, right_im, left_im_rect, right_im_rect;


int main(int argc, char** argv)
{
    stereo::setup();
    stereo::start();
    this_thread::sleep_for(1s); // Makes sure image data is populated before calibrating
    cout << "Starting collection" << endl;
        
    while(true)
    {        
        std::cout << "Collecting frame" << std::endl;
        
        stereo::left->lock();
        stereo::right->lock();
        left_im = stereo::left->image()->clone();
        right_im = stereo::right->image()->clone();
        stereo::left->unlock();
        stereo::right->unlock();

        stereo::depth->rectify(left_im, right_im, left_im_rect, right_im_rect);

        imwrite("left.png", left_im_rect);
        imwrite("right.png", right_im_rect);
     
        this_thread::sleep_for(1s);
    }
    return 0;
}