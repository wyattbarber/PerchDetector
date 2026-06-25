#pragma once

#include <Task.hpp>
#include <TaskManager.hpp>
#include <TaskTypes.hpp>
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
    bool simulation; /// Program running in simulation mode
    std::string logfile; /// Path to log output
    std::string cal_folder; /// Folder containing calibration files
    task_executor tasks; /// Map of task names to tasks and their threads
    std::map<std::string, cli_cmd_executor> commands; /// Map of CLI user commands and their functions
} program_context;



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
void set_tick(std::istream&, std::ostream&, const std::vector<std::string>&, program_context&);
void dot_graph(std::istream&, std::ostream&, const std::vector<std::string>&, program_context&);

void call_task_command(const std::string&, std::istream&, std::ostream&, const std::vector<std::string>&, program_context&);

// Helpers for program initialization and execution //
void make_tasks(program_context& context);
void make_commands(std::map<std::string, cli_cmd_executor>& commands);
void run_main_loop(program_context& context);