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

// Functions for identifying left and right cameras.
// CAM1: /base/soc/i2c0mux/i2c@1/imx219@10 
// CAM0: /base/soc/i2c0mux/i2c@0/imx219@10
static bool id_cam_left(const std::string& id){return id.find("i2c@0/imx219@10") != std::string::npos;}
static bool id_cam_right(const std::string& id){return id.find("i2c@1/imx219@10") != std::string::npos;}

// Command Line Helper Functions //
void list_tasks(std::istream&, std::ostream&, const std::vector<std::string>&, program_context&);
void start(std::istream&, std::ostream&, const std::vector<std::string>&, program_context&);
void autostart(std::istream&, std::ostream&, const std::vector<std::string>&, program_context&);
void stop(std::istream&, std::ostream&, const std::vector<std::string>&, program_context&);
void autostop(std::istream&, std::ostream&, const std::vector<std::string>&, program_context&);
void exit(std::istream&, std::ostream&, const std::vector<std::string>&, program_context&);
void capture(std::istream&, std::ostream&, const std::vector<std::string>&, program_context&);
void cmd_list(std::istream&, std::ostream&, const std::vector<std::string>&, program_context&);
void log_dump(std::istream&, std::ostream&, const std::vector<std::string>&, program_context&);

void call_task_command(const std::string&, std::istream&, std::ostream&, const std::vector<std::string>&, program_context&);


// Program Setup Helper Functions //
void sig_handle(int signum)
{
    context.tasks.kill();
    exit(signum);
}


void make_tasks(task_executor& tasks)
{
    auto cam_manager = std::make_shared<CameraManagerTask>();
    tasks.add(cam_manager);
    
    auto cam_left = (context.simulation) ? 
        std::make_shared<CameraSimulator>("camera-left", cam_manager) :
        std::make_shared<CameraWrapper>("camera-left", cam_manager, id_cam_left, context.color);
    tasks.create<DataMapper<CameraWrapper>>("left_feed", cam_left);
    tasks.add(cam_left);

    auto cam_right = (context.simulation) ? 
        std::make_shared<CameraSimulator>("camera-right", cam_manager, true) :
        std::make_shared<CameraWrapper>("camera-right", cam_manager, id_cam_right, context.color);
    tasks.create<DataMapper<CameraWrapper>>("right_feed", cam_right);
    tasks.add(cam_right);

    auto stereo = std::make_shared<DepthCamera>("stereo", context.cal_folder, cam_left, cam_right);
    tasks.add(make_data_mapper<decltype(DepthCamera::value_type::disparity)>("stereo_feed", stereo, disparity_display_conv));
    tasks.add(stereo);

    auto lidar = std::make_shared<VL53L8CX>("lidar", "/dev/spidev0.0");
    tasks.add(make_data_mapper<uint16_t[64]>("lidar_feed", lidar, lidar_display_conv));
    tasks.add(lidar);

    auto pointcloud = std::make_shared<PointCloud<50>>("point_cloud", stereo, context.cal_folder);
    tasks.add(pointcloud);
}


void make_commands(std::map<std::string, cli_cmd_executor>& commands)
{
    commands["status"] = &list_tasks;
    commands["commands"] = &cmd_list;
    commands["start"] = &start;
    commands["autostart"] = &autostart;
    commands["stop"] = &stop;
    commands["autostop"] = &autostop;
    commands["capture"] = &capture;
    commands["exit"] = &exit;
    commands["log"] = &log_dump;
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
    context.simulation = false;
    
    int i = 1;
    while(i < argc)
    {
        if(strcmp(argv[i], "--logfile") == 0) // Set logfile name
        {
            context.logfile = std::string(argv[i+1]);
            i += 2;
        }
        else if(strcmp(argv[i], "--calibrations") == 0) // Set calibration folder
        {
            context.cal_folder = std::string(argv[i+1]);
            i += 2;
        }
        else if(strcmp(argv[i], "--color") == 0) // Use color images
        {
            context.color = true;
            i++;
        }
        else if(strcmp(argv[i], "--simulate") == 0) // Run with simulated inputs
        {
            context.simulation = true;
            i++;
        }
        else
        {
            std::cout << "    Unrecognized argument " << argv[i] << std::endl;
            i++;
        }
    }

    Logger::instance().set_file(context.logfile.c_str());
    make_commands(context.commands);

    // Construct tasts
    std::cout << "-- Constructing tasks" << std::endl;
    make_tasks(context.tasks);    

    // Launch all tasks
    std::cout << "-- Launching tasks" << std::endl;
    context.tasks.launch();
    
    // Start base tasks that should be run without user input
    std::cout << "-- Starting core tasks" << std::endl;
    context.tasks["_camera_manager"]->start();

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
    context.tasks.kill();

    std::cout << "Perch detector CLI shutdown" << std::endl;
    return 0;
}