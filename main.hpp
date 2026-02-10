#pragma once

#include <Task.hpp>


struct program_context;
typedef void(*cli_cmd_executor)(std::istream&, std::ostream&, const std::vector<std::string>&, program_context&);

typedef struct program_context{
    bool running;
    task_executor tasks;
    std::map<std::string, cli_cmd_executor> commands;
} program_context;