#include "Task.hpp"


void Task::init()
{    
    for(auto task : depends_on)
    {
        task->declare_dependency_of(shared_from_this());
    }
}

bool Task::start()
{
    if(!alive)
    {
        auto res = this->start_impl();
        if(res)
        {
            alive = true;
        }
        return res;
    }
    return true;
}


void Task::stop()
{
    alive = false;
    this->stop_impl();
}


void Task::declare_cli_command(const char* cmd, command_executor_t executor)
{
    commands[cmd] = executor;
}


std::vector<std::string> Task::list_commands()
{
    std::vector<std::string> out;
    for(const auto& pair : commands){ out.push_back(pair.first); }
    return out;
}


bool Task::implements(const std::string& cmd)
{
    return commands.find(cmd) != commands.end();
}


void Task::call(const std::string& cmd, std::istream& in, std::ostream& out, const std::vector<std::string>& args)
{
    auto method = commands[cmd];
    (*method)(this, in, out, args);
}

bool Task::autostart()
{
    if(!is_alive())
    {
        for(auto task : depends_on)
        {
            info("Starting dependency ", task->name);
            if(!task->autostart())
            {
                error("Failed to start dependency ", task->name);
                return false;
            }
        }
        info("Started dependencies.");
        return start();
    }
    return true;
}

  
void Task::autostop()
{
    if(is_alive())
    {
        for(auto task : depended_by)
        {
            info("Stopping dependent task ", task->name);
            task->autostop();
        }
        info("Stopped dependent tasks");
        stop();
    }
}


void Task::declare_dependency_of(std::shared_ptr<Task> task)
{
    depended_by.push_back(task);
}


void Task::tick()
{
    auto now = std::chrono::steady_clock::now();
    if(tick_called_once)
    {
        tick_called_twice = true;
        std::chrono::duration<float> t = now - last_tick_call;
        rate_est = 1.0 / t.count();
    }
    else
    {
        tick_called_once = true;
    }
    last_tick_call = now;
}
