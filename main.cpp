#include <CameraManager.hpp>
#include <CameraWrapper.hpp>
#include <Depth.hpp>
#include <VL53L8CX.hpp>
#include <DataEncoder.hpp>
#include <functional>
#include <cstring>
#include <iostream>
#include <string>
#include <csignal>
#include <vector>
#include <map>
#include <algorithm>
#include "main.hpp"


// Parameters and Defaults //
program_context context; // Global program state and configuration
const char* default_log = "logout.txt";


// Task Definitions //
std::shared_ptr<CameraManagerTask> cam_manager;
// CAM1: /base/soc/i2c0mux/i2c@1/imx219@10 
// CAM0: /base/soc/i2c0mux/i2c@0/imx219@10
static bool id_cam_left(const std::string& id){return id.find("i2c@0/imx219@10") != std::string::npos;}
static bool id_cam_right(const std::string& id){return id.find("i2c@1/imx219@10") != std::string::npos;}
std::shared_ptr<CameraWrapper> cam_left, cam_right;
std::shared_ptr<DepthCamera> depth;
std::shared_ptr<VL53L8CX> lidar;
std::shared_ptr<DataEncoder<DepthCamera, float[CameraWrapper::Width * CameraWrapper::Height]>> depth_stream;


// Command Line Helper Functions //
void list_tasks(std::istream&, std::ostream&, const std::vector<std::string>&, program_context&);
void start(std::istream&, std::ostream&, const std::vector<std::string>&, program_context&);
void autostart(std::istream&, std::ostream&, const std::vector<std::string>&, program_context&);
void stop(std::istream&, std::ostream&, const std::vector<std::string>&, program_context&);
void autostop(std::istream&, std::ostream&, const std::vector<std::string>&, program_context&);
void exit(std::istream&, std::ostream&, const std::vector<std::string>&, program_context&);
void capture(std::istream&, std::ostream&, const std::vector<std::string>&, program_context&);
void cmd_list(std::istream&, std::ostream&, const std::vector<std::string>&, program_context&);

void call_task_command(const std::string&, std::istream&, std::ostream&, const std::vector<std::string>&, program_context&);


// Program Setup Helper Functions //
void sig_handle(int signum)
{
    kill_tasks(context.tasks);
    exit(signum);
}

void make_tasks()
{
    cam_manager = std::make_shared<CameraManagerTask>();
    
    cam_left = std::make_shared<CameraWrapper>("camera-left", cam_manager, id_cam_left, context.color);
    cam_right = std::make_shared<CameraWrapper>("camera-right", cam_manager, id_cam_right, context.color);
    
    depth = std::make_shared<DepthCamera>("stereo", context.cal_folder, cam_left, cam_right);
    depth_stream = std::make_shared<DataEncoder<DepthCamera, float[CameraWrapper::Width * CameraWrapper::Height]>>("depth_streamer", depth);

    lidar = std::make_shared<VL53L8CX>("lidar", "/dev/spidev0.0");

    cam_manager->init();
    cam_left->init();
    cam_right->init();
    depth->init();
    depth_stream->init();
    lidar->init();
}

void make_commands()
{
    context.commands["status"] = &list_tasks;
    context.commands["commands"] = &cmd_list;
    context.commands["start"] = &start;
    context.commands["autostart"] = &autostart;
    context.commands["stop"] = &stop;
    context.commands["autostop"] = &autostop;
    context.commands["capture"] = &capture;
    context.commands["exit"] = &exit;
}


int main(int argc, char** argv)
{    
    std::cout << "Starting perch detector CLI..." << std::endl;

    signal(SIGINT, sig_handle);

    // Set defaults and process arguments
    std::cout << "-- Configuring program" << std::endl;
    context.logfile = std::string(default_log);
    context.cal_folder = std::string("~/camera_calibrations");
    context.color = false;
    
    int i = 1;
    while(i < argc)
    {
        if(strcmp(argv[i], "--logfile") == 0) // Set logfile name
        {
            context.logfile = std::string(argv[i+1]);
            i += 2;
        }
        else if(strcmp(argv[i], "--color") == 0) // Use color images
        {
            context.color = true;
            i++;
        }
        else
        {
            std::cout << "    Unrecognized argument " << argv[i] << std::endl;
            i++;
        }
    }

    Logger::instance().set_file(context.logfile.c_str());
    make_commands();

    // Construct tasts
    std::cout << "-- Constructing tasks" << std::endl;
    make_tasks();    

    // Launch all tasks
    std::cout << "-- Launching tasks" << std::endl;
    context.tasks = launch_tasks({
        cam_manager,
        cam_left,
        cam_right,
        depth,
        lidar,
        depth_stream
    });
    
    // Start base tasks that should be run without user input
    std::cout << "-- Starting core tasks" << std::endl;
    cam_manager->start();

    context.running = true;
    std::cout << "Perch detector CLI started" << std::endl;

    // Main program started, continue on user input
    std::string line, word;
    std::stringstream input;
    std::vector<std::string> cmd;
    while(context.running)
    {   
        // Reset inputs
        cmd.clear();

        // Get user command and split on spaces
        std::cout << ">>> ";
        std::getline(std::cin, line);
        input = std::stringstream(line);
        while(input >> word){ cmd.push_back(word); }
        if(cmd.size() == 0)
        {
            // Empty command
            continue;
        }

        // Match command to operation and execute
        if(context.commands.find(cmd[0]) != context.commands.end())
        {
            // Operation on the global program
            context.commands[cmd[0]](std::cin, std::cout, {cmd.begin()+1, cmd.end()}, context);
        }
        else if(context.tasks.find(cmd[0]) != context.tasks.end())
        {
            // Operation on specific task
            call_task_command(cmd[0], std::cin, std::cout, {cmd.begin()+1, cmd.end()}, context);
        }
        else
        {
            std::cout << "Command " << cmd[0] << " is not recognized." << std::endl;
        }
    }

    std::cout << "Shutting down perch detector CLI..." << std::endl;

    // Shutdown and exit
    std::cout << "-- Killing tasks" << std::endl;    
    kill_tasks(context.tasks);

    std::cout << "Perch detector CLI shutdown" << std::endl;
    return 0;
}