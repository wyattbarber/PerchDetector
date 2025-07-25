#include <ImageSender.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <cstdlib>
#include <iostream>
#include <csignal>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

std::unique_ptr<ImageSender> sender;

struct pixel_op
{
    void operator()(uchar &p, const int * position) const
    {
        p = static_cast<uchar>(rand() % 255);
    }
};

void rand_fill(cv::Mat& m)
{
    m.forEach<uchar>(pixel_op());
}


void sig_handle(int signum)
{
    sender->stop();
    exit(signum);
}


int main(int argc, char** argv)
{
    if(argc < 2)
    {
        std::cerr << "filename must be passed from command line" << std::endl;
        return -1;
    }

    int width = 500;
    int height = 500;
    char * file = argv[1];

    sender = make_sender<uint8_t>(file, height, width);

    std::cout << "Starting random image feed" << std::endl;

    signal(SIGINT, sig_handle);

    sender->start();

    if(!sender->opened())
    {
        std::cerr << "Failed to setup communication channel" << std::endl;
        return -1;
    }

    std::cout << "Started image feed" << std::endl;
    
    while(true)
    {
        sender->acquire();
        rand_fill(sender->get_image());
        sender->release();
        std::this_thread::sleep_for(20ms);
    }
}