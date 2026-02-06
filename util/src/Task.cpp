#include "Task.hpp"

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


void _task_runner(std::shared_ptr<Task> task, bool* alive)
{
    while(*alive)
    {
        task->step();
    }
}


task_executor launch_tasks(std::initializer_list<std::shared_ptr<Task>> tasks)
{   
    task_executor out;

    for (auto task : tasks)
    {
        auto b = new bool(true);
        auto t = std::make_unique<std::thread>(_task_runner, task, b);
        out[task->name] = {task, std::move(t), b};
    }

    return out;
}



void kill_tasks(task_executor& exe)
{
    // Stop tasks that haven't been stopped
    for (auto& pair : exe)
    {
        auto task = std::get<0>(pair.second);
        if(task->is_alive())
        {
            task->stop();
        }
    }

    // Flag all threads to stop
    for (auto& pair : exe)
    {
        *std::get<2>(pair.second) = false;
    }

    // End and delete all threads
    for (auto& pair : exe)
    {
        std::get<1>(pair.second)->join();
        delete std::get<2>(pair.second);
    }
}