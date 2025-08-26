#include <CameraWrapper.hpp>
#include <ImageSender.hpp>
#include <ArgParser.hpp>
#include <Logging.hpp>
#include <functional>
#include <chrono>
#include <thread>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>


// Externally linked main functions for the two operation modes
int main_headless(ArgParser&);
int main_gui(ArgParser&);


int main(int argc, char** argv)
{    
    ArgParser args(argc, argv);
    if(!args.valid())
    {
        std::cerr << "Provided arguments invalid." << std::endl;
        return -1;
    }
    
    if(args.log_file() != nullptr)
    {
        // Enable file output for logging
        Logger::instance().set_file(args.log_file());
    }
    if(args.stats_file() != nullptr)
    {
        // Enable file output for data logging
        DataLogger::instance().set_file(args.stats_file());
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