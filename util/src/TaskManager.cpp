#include <TaskManager.hpp>

   
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

        threads[name] = std::make_unique<std::thread>(executors[name], task, this);
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


void TaskManager::set_tick(float f){ tick_period = 1.0 / f; }


size_t TaskManager::get_ticks()
{
    auto t = std::chrono::steady_clock::now();
    auto dt = t - last_tick;
    if(std::chrono::duration<float>(dt).count() >= tick_period)
    {
        ++tick_counter;
        last_tick = t;
    }
    return tick_counter;
}