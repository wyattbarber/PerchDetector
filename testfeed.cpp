#include <ImageSender.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <cstdlib>
#include <iostream>
#include <csignal>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

ImageSender* sender;

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
    delete sender;
    exit(signum);
}


int main(int argc, char** argv)
{
    if(argc < 4)
    {
        std::cerr << "width, height, and filename must be passed from command line" << std::endl;
        return -1;
    }

    int width = atoi(argv[1]);
    int height = atoi(argv[2]);
    char * file = argv[3];

    cv::Mat image = cv::Mat::zeros(height, width, CV_8U);
    sender = new ImageSender(file, image);

    std::cout << "Starting random image feed" << std::endl;

    signal(SIGINT, sig_handle);

    sender->start();

    if(!sender->valid)
    {
        std::cerr << "Failed to setup communication channel" << std::endl;
        return -1;
    }

    std::cout << "Started random image feed" << std::endl;
    
    while(true)
    {
        rand_fill(image);
        sender->transmit();
        std::this_thread::sleep_for(20ms);
    }
}