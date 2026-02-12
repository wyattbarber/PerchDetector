#pragma once

#include <Task.hpp>


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
    bool running;
    bool color;
    char logfile[100];
    task_executor tasks;
    std::map<std::string, cli_cmd_executor> commands;
} program_context;