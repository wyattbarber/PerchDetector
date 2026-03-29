#include <iostream>
#include <string>
#include <csignal>
#include "main.hpp"


// Global program configuration and state
program_context context;


// Ctrl-C handler
void sig_handle(int signum)
{
    context.tasks.kill();
    exit(signum);
}


int main(int argc, char** argv)
{    
    std::cout << "Starting perch detector CLI..." << std::endl;

    signal(SIGINT, sig_handle);

    // Set defaults and process arguments
    std::cout << "-- Configuring program" << std::endl;
    context.logfile = std::string("logout.txt");
    context.cal_folder = std::string("~/camera_calibrations");
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
        else if(strcmp(argv[i], "--simulate") == 0) // Run with simulated inputs
        {
            std::cout << "    Warning: Simulation is not supported, ignoring." << std::endl;
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
    make_tasks(context);    

    // Launch all tasks
    std::cout << "-- Launching tasks" << std::endl;
    context.tasks.launch();
    
    // Start base tasks that should be run without user input
    std::cout << "-- Starting core tasks" << std::endl;
    context.tasks.get("_camera_manager")->start();

    context.running = true;
    std::cout << "Perch detector CLI started" << std::endl;

    // Main program started, continue on user input
    run_main_loop(context);

    std::cout << "Shutting down perch detector CLI..." << std::endl;

    // Shutdown and exit
    std::cout << "-- Killing tasks" << std::endl;    
    context.tasks.kill();

    std::cout << "Perch detector CLI shutdown" << std::endl;
    return 0;
}