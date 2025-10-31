#include <CameraWrapper.hpp>
#include <stereocam.hpp>
#include <chrono>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_board.hpp>
#include <opencv2/objdetect/charuco_detector.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>
#include <thread>
#include <vector>
#include <cstdio>
#include <cmath>
#include <iostream>
#include <fstream>
#include <Logging.hpp>


using namespace std::chrono_literals;
using namespace std;
using namespace cv;

int N;
char left_filename[100];
char right_filename[100];

void stop()
{   
    std::cout << "Ending calibration program" << std::endl;
    stereo::stop();
    stereo::teardown();
}


static void sig_handle(int signum)
{
    stop();
    exit(signum);
}


int main(int argc, char** argv)
{
    Logger::instance().set_file("capture-log.txt");

    N = atoi(argv[1]);

    stereo::setup(false, true);
    stereo::start();
    std::this_thread::sleep_for(1s); // Makes sure image data is populated before calibrating
        

    for(int i = 0; i < N; ++i)
    {        
        std::cout << "Collecting calibration sample " << i+1 << " of " << N << std::endl;
        sprintf(left_filename, "left-%d.png", i);
        sprintf(right_filename, "right-%d.png", i);

        stereo::left->lock();
        stereo::right->lock();
        imwrite(left_filename, *stereo::left->image());
        imwrite(right_filename, *stereo::right->image());
        stereo::left->unlock();
        stereo::right->unlock();

        std::this_thread::sleep_for(1s); 
    }

    stop();
    return 0;
}