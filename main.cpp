#include <CameraManager.hpp>
#include <CameraWrapper.hpp>
#include <Depth.hpp>
#include <VL53L8CX.hpp>
#include <DataEncoder.hpp>
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
std::shared_ptr<VL53L8CX> lidar;
std::shared_ptr<DataEncoder<DepthCamera, float>> depth_stream;


// Helper Functions //

void sig_handle(int signum)
{
    kill_tasks(tasks);
    exit(signum);
}

void make_tasks()
{
    cam_manager = std::make_shared<CameraManagerTask>();
    cam_manager->init();
    
    cam_left = std::make_shared<CameraWrapper>("camera-left", cam_manager, id_cam_left, false);
    cam_left->init();
    cam_right = std::make_shared<CameraWrapper>("camera-right", cam_manager, id_cam_right, false);
    cam_right->init();
    
    depth = std::make_shared<DepthCamera>("stereo", cam_left, cam_right);
    depth->init();
    depth_stream = std::make_shared<DataEncoder<DepthCamera, float>>("depth_streamer", depth);
    depth_stream->init();

    lidar = std::make_shared<VL53L8CX>("lidar", "/dev/spidev0.0");
    lidar->init();
}

void list_tasks(const char* line_start, task_executor& tasks)
{
    // Compute padding to make list into even columns
    auto max_len = tasks.begin()->first.size();
    for(const auto& pair : tasks)
    {
        auto len = pair.first.size();
        max_len = (len > max_len) ? len : max_len;
    }

    // Print two columns of task names and states
    for(const auto& pair : tasks)
    {
        std::cout << line_start << pair.first;
        for(auto i = pair.first.size(); i < max_len; ++i){ std::cout << ' '; } // Space pad the name to make columns even
        std::cout << "\t\t";
        std::cout << (std::get<0>(pair.second)->is_alive() ? "running" : "stopped") << std::endl;
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
        depth,
        lidar,
        depth_stream
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
        else if((cmd[0] == "start") || (cmd[0] == "autostart"))
        {
            // Start a task
            if(cmd.size() < 2)
            {
                std::cout << "Need a task name to start." << std::endl;
                continue;
            }
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

            if((cmd[0] == "start") ? std::get<0>(tasks[cmd[1]])->start() : std::get<0>(tasks[cmd[1]])->autostart())
            {
                std::cout << "Task " << cmd[1] << " has been started." << std::endl;
            }
            else
            {
                std::cout << "Task " << cmd[1] << " failed to start." << std::endl;
            }
        }
        else if((cmd[0] == "stop") || (cmd[0] == "autostop"))
        {
            // Stop a task
            if(cmd.size() < 2)
            {
                std::cout << "Need a task name to stop." << std::endl;
                continue;
            }
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

            (cmd[0] == "stop") ? std::get<0>(tasks[cmd[1]])->stop() : std::get<0>(tasks[cmd[1]])->autostop();
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
                cv::Mat left_im(cam_left->get_height(), cam_left->get_width(), CV_8UC1, cam_left->acquire(), cam_left->get_stride());
                cv::Mat right_im(cam_right->get_height(), cam_right->get_width(), CV_8UC1, cam_right->acquire(), cam_right->get_stride());
                imwrite(left_file.c_str(), left_im);
                imwrite(right_file.c_str(), right_im);
                cam_left->release();
                cam_right->release();
                std::this_thread::sleep_for(1s); 
            }
            std::cout << "Captures completed." << std::endl;
        }
        else if(tasks.find(cmd[0]) != tasks.end())
        {
            if(cmd.size() < 2)
            {
                std::cout << "Need a command name to run a task specific action." << std::endl;
                continue;
            }

            auto task = std::get<0>(tasks[cmd[0]]);
            if(!task->is_alive())
            {
                std::cout << "Cannot call commands of a task that is not running." << std::endl;
                continue;
            }
            
            if(!task->implements(cmd[1]))
            {
                std::cout << "Task " << cmd[0] << " has no command named " << cmd[1] << std::endl;
                continue; 
            }

            std::vector<std::string> args;
            if(cmd.size() > 2) args = std::vector<std::string>(cmd.begin()+2, cmd.end());
            task->call(cmd[1], std::cin, std::cout, args);
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