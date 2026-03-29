#include <TaskManager.hpp>


void _task_runner(std::shared_ptr<Task> task, bool* alive)
{
    while(*alive)
    {
        if(task->is_alive())
        {
            task->step();
        }
    }
}


std::shared_ptr<Task>& TaskManager::operator[](const std::string& name)
{
    return tasks[name];
}

   
void TaskManager::launch()
{    
    // Perform post-construction initialization
    for (auto& item : tasks)
    {
        item.second->init();
    }
    // Begin execution
    running = true;
    for (auto& item : tasks)
    {
        auto name = item.first;
        auto task = item.second;

        threads[name] = std::make_unique<std::thread>(_task_runner, task, &running);
    }
}


void TaskManager::kill()
{
    // Stop tasks that haven't been stopped
    for (auto& item : tasks)
    {
        auto task = item.second;
        if(task->is_alive())
        {
            task->autostop();
        }
    }

    // Flag all threads to stop
    running = false;

    // End all tasks
    for (auto& item : tasks)
    {
        threads[item.first]->join();
    }
}
