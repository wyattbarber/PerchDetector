#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iomanip>
#include <chrono>
#include <string>
#include <fstream>
#include "main.hpp"

using namespace std::chrono_literals;


void list_tasks(std::istream& in, std::ostream& out, const std::vector<std::string>& args, program_context& context)
{
    // Compute padding to make list into even columns
    auto max_name_len = context.tasks.begin()->first.size();
    for(const auto& pair : context.tasks)
    {
        auto len = pair.first.size();
        max_name_len = (len > max_name_len) ? len : max_name_len;
    }

    // Print two columns of task names and states
    for(const auto& pair : context.tasks)
    {
        // Task name
        out << "\t" << pair.first;
        for(auto i = pair.first.size(); i < max_name_len; ++i){ out << ' '; } // Space pad the name to make columns even
        out << "\t\t";
        // State
        out << (pair.second->is_alive() ? "running" : "stopped");
        out << "\t\t";
        // Update rate
        auto r = pair.second->rate();
        if(r.second) out << std::fixed << std::setprecision(2) << r.first << std::defaultfloat << " Hz";
        else out << "---";
        out << std::endl;
    }
}


void _start(std::istream& in, std::ostream& out, const std::vector<std::string>& args, program_context& context, bool autostart)
{
    if(args.size() < 1)
    {
        out << "Need a task name to start." << std::endl;
        return;
    }

    for(const auto& arg : args)
    {
        if(context.tasks.find(arg) == context.tasks.end())
        {
            out << "Task " << arg << " is not defined." << std::endl;
            continue;
        }

        auto task = context.tasks.get(arg);
        if(task->is_alive())
        {
            out << "Task " << arg << " is already started." << std::endl;
            continue;
        }

        if(autostart ? task->autostart() : task->start())
        {
            out << "Task " << arg << " has been started." << std::endl;
        }
        else
        {
            out << "Task " << arg << " failed to start." << std::endl;
        }
    }
}


void start(std::istream& in, std::ostream& out, const std::vector<std::string>& args, program_context& context)
{ 
    _start(in, out, args, context, false); 
}


void autostart(std::istream& in, std::ostream& out, const std::vector<std::string>& args, program_context& context)
{ 
    _start(in, out, args, context, true); 
}


void _stop(std::istream& in, std::ostream& out, const std::vector<std::string>& args, program_context& context, bool autostop)
{
    // Stop a task
    if(args.size() < 1)
    {
        out << "Need a task name to stop." << std::endl;
        return;
    }
    
    for(const auto& arg : args)
    {
        if(context.tasks.find(arg) == context.tasks.end())
        {
            out << "Task " << arg << " is not defined." << std::endl;
            continue;
        }

        auto task = context.tasks.get(arg);
        if(!task->is_alive())
        {
            out << "Task " << arg << " is already stopped." << std::endl;
            continue;
        }

        autostop ? task->autostop() : task->stop();
        out << "Stopped task " << arg << std::endl;
    }
}


void stop(std::istream& in, std::ostream& out, const std::vector<std::string>& args, program_context& context)
{
    _stop(in, out, args, context, false);
}


void autostop(std::istream& in, std::ostream& out, const std::vector<std::string>& args, program_context& context)
{
    _stop(in, out, args, context, true);
}


void exit(std::istream& in, std::ostream& out, const std::vector<std::string>& args, program_context& context)
{
    context.running = false;
}


void capture(std::istream& in, std::ostream& out, const std::vector<std::string>& args, program_context& context)
{
    auto cam_left = context.tasks.get<CameraWrapper>("camera-left");
    auto cam_right = context.tasks.get<CameraWrapper>("camera-right");
    auto lidar = context.tasks.get<VL53L8CX>("lidar");


    if(!cam_left->is_alive() || !cam_right->is_alive())
    {
        out << "Cannot run capture without left and right cameras running." << std::endl;
        return;
    }
    if(args.size() < 1)
    {
        out << "Capture requires at least one argument (count)." << std::endl;
        return;
    }

    bool with_lidar = false;
    bool n_given = false;
    int n;
    if(args[0] != "inf")
    {
        n_given = true;
        n = std::stoi(args[0]);
    }
    if(args.size()>2)
    {
        if(args[2] == "--lidar")
        {
            with_lidar = true;
            if(!lidar->is_alive())
            {
                out << "Option to save lidar data given but lidar task is not started." << std::endl;
                return;
            }
        }
    }
    
    std::string path = (args.size() > 1) ? args[1] : "./";
    std::string left_file, right_file, lidar_file;
    
    if(n_given) out << "Saving " << n << " image pairs to " << path << ", at 1 second interval." << std::endl;
    else out << "Saving images on user trigger, enter exit to stop." << std::endl;
    
    bool capturing = true;
    int i = 0;
    while(capturing)
    {
        if(!n_given) // Wait for user trigger
        {
            out << "Press enter to capture an image." << std::endl;
            std::string line;
            std::getline(in, line);
            if(line == "exit")
            {
                break;
            }
        }
        left_file = path + "left-" + std::to_string(i) + ".png";
        right_file = path + "right-" + std::to_string(i) + ".png";
        lidar_file = path + "lidar-" + std::to_string(i) + ".json";

        // Get data
        auto im_left = cam_left->acquire();
        auto im_right = cam_right->acquire();        
        VL53L8CX::update_ptr_const_type lidar_data;
        if(with_lidar)
        {   
            lidar_data = lidar->acquire();
        }
        const cv::Mat left_im(cam_left->Height, cam_left->Width, CV_8UC1, const_cast<uint8_t*>(im_left->data));
        const cv::Mat right_im(cam_right->Height, cam_right->Width, CV_8UC1, const_cast<uint8_t*>(im_right->data));
        // Save data
        imwrite(left_file.c_str(), left_im);
        imwrite(right_file.c_str(), right_im);
        if(with_lidar)
        {
            std::ofstream lidar_out(lidar_file);
            VL53L8CX::data_to_json(lidar_data, lidar_out);
        }

        ++i;
        if(n_given) // Await next capture period
        {
            out << '\r' << "Completed capture " << i << " of " << n;
            std::this_thread::sleep_for(1s); 
            capturing = i < n;
        }
        else
        {
            out << "Captured image " << i << std::endl;
        }
    }
    out << std::endl;
    out << "Captures completed." << std::endl;
}


void call_task_command(const std::string& taskname, std::istream& in, std::ostream& out, const std::vector<std::string>& args, program_context& context)
{
    if(args.size() < 1)
    {
        out << "Need a command name to run a task specific action." << std::endl;
        return;
    }

    auto task = context.tasks.get(taskname);
    if(!task->is_alive())
    {
        out << "Cannot call commands of a task that is not running." << std::endl;
        return;
    }
    
    if(!task->implements(args[0]))
    {
        out << "Task " << taskname << " has no command named " << args[0] << std::endl;
        return; 
    }

    task->call(args[0], in, out, {args.begin()+1, args.end()});
}


void cmd_list(std::istream& in, std::ostream& out, const std::vector<std::string>& args, program_context& context)
{
    if(args.size() != 1)
    {
        out << "Need one task name to list commands for." << std::endl;
        return;
    }
    if(context.tasks.find(args[0]) == context.tasks.end())
    {
        out << "Task " << args[0] << " is not defined." << std::endl;
        return;
    }

    for(const auto & cmd : context.tasks.get(args[0])->list_commands())
    {
        out << cmd << " ";
    }
    out << std::endl;
}


void log_dump(std::istream& in, std::ostream& out, const std::vector<std::string>& args, program_context& context)
{
    std::ifstream log(context.logfile);
    if(!log.is_open())
    {
        return;
    }

    std::stringstream content;
    content << log.rdbuf();
    out << content.str() << std::endl;
}


void set_tick(std::istream& in, std::ostream& out, const std::vector<std::string>& args, program_context& context)
{
    if(args.size() < 1)
    {
        out << "Tick frequency is required." << std::endl;
        return;
    }

    float f = std::stof(args[0]);
    if(f <= 0.0)
    {
        out << "TIck frequency must be a positive value." << std::endl;
        return;
    }

    context.tasks.set_tick(f);
    out << "Program tick set to " << f << " Hz." << std::endl;
}


void dot_graph(std::istream& in, std::ostream& out, const std::vector<std::string>& args, program_context& context)
{
    out << "digraph {" << std::endl;
    for(const auto& task : context.tasks.names())
    {
        for(const auto& dep : context.tasks.get(task)->dependencies())
        {
            out << "\t" << dep << " -> " << task << std::endl;
        }
    }
    out << "}" << std::endl;
}