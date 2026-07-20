#include "main.hpp"



// Functions for identifying left and right cameras.
// CAM1: /base/soc/i2c0mux/i2c@1/imx219@10 
// CAM0: /base/soc/i2c0mux/i2c@0/imx219@10
static bool id_cam_left(const std::string& id){return id.find("i2c@0/imx219@10") != std::string::npos;}
static bool id_cam_right(const std::string& id){return id.find("i2c@1/imx219@10") != std::string::npos;}


void make_tasks(program_context& context)
{
    auto cam_manager = std::make_shared<CameraManagerTask>();
    context.tasks.add(cam_manager);
    
    auto cam_left = std::make_shared<CameraWrapper>("camera-left", cam_manager, id_cam_left);
    context.tasks.add(make_data_mapper("left_feed", cam_left));
    context.tasks.add(cam_left);

    auto cam_right = std::make_shared<CameraWrapper>("camera-right", cam_manager, id_cam_right);
    context.tasks.add(make_data_mapper("right_feed", cam_right));
    context.tasks.add(cam_right);

    auto stereo = std::make_shared<DepthCamera>("stereo", context.cal_folder, cam_left, cam_right);
    context.tasks.add(make_data_mapper("stereo_feed", stereo, stereo_display_conv()));
    context.tasks.add(stereo);

    auto lidar = std::make_shared<VL53L8CX>("lidar", "/dev/spidev0.0", context.cal_folder);
    context.tasks.add(make_data_mapper("lidar_feed", lidar, lidar_display_conv()));
    context.tasks.add(lidar);

    auto pointcloud = std::make_shared<PointCloud>("point_cloud", stereo, context.cal_folder);
    context.tasks.add(pointcloud);
    context.tasks.add(make_data_mapper("point_cloud_feed", pointcloud, point_cloud_conv()));

    auto detector = std::make_shared<LineFinder>("detector", pointcloud, context.cal_folder);
    context.tasks.add(detector);

    auto ioctl = std::make_shared<GrasperController>("ioctl", "/dev/i2c-89");
    context.tasks.add(ioctl);
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
    commands["tick"] = &set_tick;
    commands["graph"] = &dot_graph;
}


void run_main_loop(program_context& context)
{
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
}