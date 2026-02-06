#include <CameraManager.hpp>
#include <CameraWrapper.hpp>
#include <Depth.hpp>
#include <chrono>
#include <functional>
#include <cstring>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <string>
#include <csignal>
#include <vector>
#include <algorithm>


using namespace std::chrono_literals;


// Parameters and Defaults //
char logfile[] = "logout.txt"; // Default log file


// Task Definitions //
task_executor tasks;
std::shared_ptr<CameraManagerTask> cam_manager;
// CAM1: /base/soc/i2c0mux/i2c@1/imx219@10 
// CAM0: /base/soc/i2c0mux/i2c@0/imx219@10
static bool id_cam_left(const std::string& id){return id.find("i2c@0/imx219@10") != std::string::npos;}
static bool id_cam_right(const std::string& id){return id.find("i2c@1/imx219@10") != std::string::npos;}
std::shared_ptr<CameraWrapper> cam_left, cam_right;
std::shared_ptr<DepthCamera> depth;


// Helper Functions //

void sig_handle(int signum)
{
    kill_tasks(tasks);
    exit(signum);
}

void make_tasks()
{
    cam_manager = std::make_shared<CameraManagerTask>();
    
    cam_left = std::make_shared<CameraWrapper>("camera-left", cam_manager, id_cam_left, false);
    cam_right = std::make_shared<CameraWrapper>("camera-right", cam_manager, id_cam_right, false);
    
    depth = std::make_shared<DepthCamera>("stereo", cam_left, cam_right);
}

void list_tasks(const char* line_start, task_executor& tasks)
{
    for(const auto& pair : tasks)
    {
        std::cout << line_start << pair.first << ": " << (std::get<0>(pair.second)->is_alive() ? "started" : "stopped") << std::endl;
    }
}


int main(int argc, char** argv)
{    
    std::cout << "Starting perch detector CLI..." << std::endl;

    signal(SIGINT, sig_handle);

    // Process arguments
    std::cout << "-- Configuring program" << std::endl;
    int i = 1;
    while(i < argc)
    {
        if(strcmp(argv[i], "--logfile") == 0) // Set logfile name
        {
            strcpy(logfile, argv[i+1]);
            i += 2;
        }
        else
        {
            i++;
        }
    }

    // Configure execution based on arguments
    Logger::instance().set_file(logfile);

    // Construct tasts
    std::cout << "-- Constructing tasks" << std::endl;
    make_tasks();    

    // Launch all tasks
    std::cout << "-- Launching tasks" << std::endl;
    tasks = launch_tasks({
        cam_manager,
        cam_left,
        cam_right,
        depth
    });
    
    // Start base tasks that should be run without user input
    std::cout << "-- Starting core tasks" << std::endl;
    cam_manager->start();

    std::cout << "Perch detector CLI started" << std::endl;
    // Main program started, continue on user input
    bool running = true;
    std::string line, word;
    std::stringstream input;
    std::vector<std::string> cmd;
    while(running)
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
        if(cmd[0] == "list")
        {
            // List all tasks and status
            list_tasks("\t", tasks);
        }
        else if(cmd[0] == "start")
        {
            // Start a task
            if(tasks.find(cmd[1]) == tasks.end())
            {
                std::cout << "Task " << cmd[1] << " is not defined." << std::endl;
                continue;
            }
            if(std::get<0>(tasks[cmd[1]])->is_alive())
            {
                std::cout << "Task " << cmd[1] << " is already started." << std::endl;
                continue;
            }

            if(std::get<0>(tasks[cmd[1]])->start())
            {
                std::cout << "Task " << cmd[1] << " has been started." << std::endl;
            }
            else
            {
                std::cout << "Task " << cmd[1] << " failed to start." << std::endl;
            }
        }
        else if(cmd[0] == "stop")
        {
            // Stop a task
            if(tasks.find(cmd[1]) == tasks.end())
            {
                std::cout << "Task " << cmd[1] << " is not defined." << std::endl;
                continue;
            }
            if(!std::get<0>(tasks[cmd[1]])->is_alive())
            {
                std::cout << "Task " << cmd[1] << " is already stopped." << std::endl;
                continue;
            }

            std::get<0>(tasks[cmd[1]])->stop();
            std::cout << "Stopped task " << cmd[1] << std::endl;
        }
        else if(cmd[0] == "exit")
        {
            running = false;
        }
        else if(cmd[0] == "capture")
        {
            if(!cam_left->is_alive() || !cam_right->is_alive())
            {
                std::cout << "Cannot run capture without left and right cameras running." << std::endl;
                continue;
            }
            if(cmd.size() < 2)
            {
                std::cout << "Capture requires at least one argument (count)." << std::endl;
                continue;
            }

            int n = std::stoi(cmd[1]);
            std::string path = (cmd.size() > 2) ? cmd[2] : "./";
            std::string left_file, right_file;
            
            std::cout << "Saving " << n << " image pairs to " << path << ", at 1 second interval." << std::endl;
            for(int i = 0; i < n; ++i)
            {
                left_file = path + "left-" + std::to_string(i) + ".png";
                right_file = path + "right-" + std::to_string(i) + ".png";
                cam_left->lock();
                cam_right->lock();
                imwrite(left_file.c_str(), *cam_left->image());
                imwrite(right_file.c_str(), *cam_right->image());
                cam_left->unlock();
                cam_right->unlock();
                std::this_thread::sleep_for(1s); 
            }
            std::cout << "Captures completed." << std::endl;
        }
        else
        {
            std::cout << "Command " << cmd[0] << " is not recognized." << std::endl;
        }
    }

    std::cout << "Shutting down perch detector CLI..." << std::endl;

    // Shutdown and exit
    std::cout << "-- Killing tasks" << std::endl;    
    kill_tasks(tasks);

    std::cout << "Perch detector CLI shutdown" << std::endl;
    return 0;
}