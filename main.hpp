#pragma once

#include <Task.hpp>
#include <string>

struct program_context;

/** Signature of functions that handle terminal commands.

Functions that operate on the global program state (rather than
operations defined within a task class) need to implement this 
interface. 
*/
typedef void(*cli_cmd_executor)(std::istream&, std::ostream&, const std::vector<std::string>&, program_context&);

/** Struct containing all general program configuration and state info.


*/
typedef struct program_context{
    bool running; // Program is started
    bool color; /// Use color image input instead of grayscale
    bool simulation; /// Program running in simulation mode
    std::string logfile; /// Path to log output
    std::string cal_folder; /// Folder containing calibration files
    task_executor tasks; /// Map of task names to tasks and their threads
    std::map<std::string, cli_cmd_executor> commands; /// Map of CLI user commands and their functions
} program_context;